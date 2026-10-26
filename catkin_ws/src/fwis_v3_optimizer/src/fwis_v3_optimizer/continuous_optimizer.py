"""Direct, fixed-topology optimization of continuous motion controls.

The first implementation keeps the mode/direction word, block lengths, and
switch stations fixed. It changes only low-frequency C1 control deformations
inside existing A/O blocks. A differentiable four-disc distance-field proxy
and terminal pose constraints guide the solve. The proxy does not certify
rectangular-body safety; the caller must run the independent swept-OBB audit.
"""
import math
import time

import numpy as np
from scipy.ndimage import distance_transform_edt

from .continuous_geometry import (ContinuousPlan, ControlKnot, MotionBlock,
                                  fit_continuous_endpoint, replay_continuous,
                                  sample_block)
from .core import features
from .numerics import trapezoid


class _DeadlineReached(RuntimeError):
    pass


def _constrain_terminal_pose(opt, values, targets, invariant_tolerance=1e-7):
    """Constrain symbolic coordinates; validate fixed coordinates directly."""
    import casadi as ca
    components = []
    for name, value, target in zip(("x", "y", "yaw"), values, targets):
        expression = ca.MX(value)
        if expression.is_constant():
            if abs(float(ca.evalf(expression))-target) > invariant_tolerance:
                raise ValueError("continuous_optimizer_invariant_terminal_"+name)
        else:
            opt.subject_to(expression == target)
            components.append(name)
    return components


def _basis(t, degree):
    z = 2.*t-1.
    envelope = t*t*(1.-t)*(1.-t)
    return envelope*z**degree


def _basis_derivative(t, degree):
    z = 2.*t-1.
    envelope = t*t*(1.-t)*(1.-t)
    envelope_d = 2.*t*(1.-t)*(1.-2.*t)
    if degree == 0:
        return envelope_d
    return envelope_d*z**degree+2.*degree*envelope*z**(degree-1)


def _window_basis(t, start, end, degree):
    """Compact C1 deformation basis with zero value/slope at window ends."""
    if t <= start or t >= end:
        return 0.
    u = (t-start)/(end-start)
    z = 2.*u-1.
    envelope = u*u*(1.-u)*(1.-u)
    return envelope*z**degree


def _window_basis_derivative(t, start, end, degree):
    if t <= start or t >= end:
        return 0.
    span = end-start
    u = (t-start)/span
    z = 2.*u-1.
    envelope = u*u*(1.-u)*(1.-u)
    envelope_d = 2.*u*(1.-u)*(1.-2.*u)
    if degree == 0:
        return envelope_d/span
    return (envelope_d*z**degree+
            2.*degree*envelope*z**(degree-1))/span


def _optimize_continuous_geometry_scipy(plan, evaluator, time_limit=1.0,
                                        sample_spacing=.08, max_iterations=24,
                                        max_variables=25):
    """Optimize continuous A curvature and O angle while preserving topology.

    Endpoint equality is enforced on the same Hermite arc-length model used by
    ``replay_continuous``. Every candidate is checked with exact OBB distance
    constraints on a fixed sample grid, then must pass the caller's stricter
    swept-body and timing audits before it can be selected.
    """
    from scipy.optimize import minimize

    if not isinstance(plan, ContinuousPlan):
        raise ValueError("continuous_optimizer_plan_required")
    if not math.isfinite(time_limit) or time_limit <= 0:
        raise ValueError("continuous_optimizer_invalid_budget")
    if not math.isfinite(sample_spacing) or sample_spacing <= 0:
        raise ValueError("continuous_optimizer_invalid_spacing")

    begin = time.perf_counter()
    deadline = begin+time_limit
    layout, x0, bounds, block_columns = [], [], [], {}
    translational_blocks = [i for i, b in enumerate(plan.blocks)
                            if b.mode in (0, 1) and len(b.knots) >= 3]
    if not translational_blocks:
        return plan, {"attempted": False, "status": "no_continuous_control_dofs",
                      "iterations": 0, "elapsed_s": time.perf_counter()-begin}
    if len(translational_blocks)*5 > max_variables:
        return plan, {"attempted": False, "status": "topology_exceeds_variable_budget",
                      "iterations": 0, "variable_count": len(translational_blocks)*5,
                      "max_variables": max_variables,
                      "elapsed_s": time.perf_counter()-begin}

    for bi in translational_blocks:
        block = plan.blocks[bi]
        columns = []
        max_delta = (.16 if block.mode == 0 else .16)
        for degree in range(5):
            columns.append(len(layout))
            layout.append((bi, degree))
            x0.append(0.)
            bounds.append((-max_delta, max_delta))
        block_columns[bi] = columns
    x0 = np.asarray(x0, dtype=float)

    def build(values):
        if time.perf_counter() >= deadline:
            raise _DeadlineReached("continuous_optimizer_budget")
        blocks = list(plan.blocks)
        for bi, columns in block_columns.items():
            original = plan.blocks[bi]
            deltas = [float(values[j]) for j in columns]
            knots = []
            for knot in original.knots:
                t = knot.s/original.length
                value = knot.value+sum(deltas[j]*_basis(t, j)
                                       for j in range(5))
                normalized_slope = knot.derivative*original.length + sum(
                    deltas[j]*_basis_derivative(t, j) for j in range(5))
                knots.append(ControlKnot(knot.s, value,
                                         normalized_slope/original.length))
            # Basis value and derivative vanish at block boundaries, so a
            # same-mode direction cusp retains its original wheel setting.
            blocks[bi] = MotionBlock(original.mode, original.direction,
                                     original.length, tuple(knots),
                                     slope_policy="fixed")
        return ContinuousPlan(plan.start, tuple(blocks), plan.goal)

    cache_key, cache_value = None, None

    def analyze(values):
        nonlocal cache_key, cache_value
        key = tuple(float(v) for v in values)
        if key == cache_key:
            return cache_value
        candidate = build(values)
        rows_by_block, clearances, control_margins = [], [], []
        pose = candidate.start
        length = reverse_length = spin_angle = 0.
        risk_integral = peak_risk = 0.
        curvature_energy = curvature_tv = oblique_tv = 0.
        kmax = 1./(evaluator.limits.r_min_ack if evaluator.limits else .810)
        preferred = evaluator.profile.preferred_clearance_factor*evaluator.width
        for block in candidate.blocks:
            rows = sample_block(block, pose, sample_spacing)
            rows_by_block.append(rows)
            pose = rows[-1][1]
            if block.mode != 2:
                length += block.length
                if block.direction < 0:
                    reverse_length += block.length
            else:
                spin_angle += abs(block.signed_angle)
            block_clearances = []
            for row_index, (_, sample_pose, control, mode, _) in enumerate(rows):
                if row_index % 24 == 0 and time.perf_counter() >= deadline:
                    raise _DeadlineReached("continuous_optimizer_budget")
                distance = evaluator.distance.distance(sample_pose)
                block_clearances.append(distance)
                clearances.append(distance)
                if mode == 0:
                    control_margins.append(kmax-abs(control))
                elif mode == 1:
                    control_margins.append(math.pi/2-abs(control))
            if block.mode == 0:
                curvature_energy += sum(.5*(a[2]**2+b[2]**2)*(b[0]-a[0])
                                        for a, b in zip(rows, rows[1:]))/(kmax*kmax)
                curvature_tv += sum(abs(b[2]-a[2])
                                    for a, b in zip(rows, rows[1:]))
            elif block.mode == 1:
                oblique_tv += sum(abs(b[2]-a[2])
                                  for a, b in zip(rows, rows[1:]))
            measure = (block.length if block.mode != 2 else
                       evaluator.distance.corner*abs(block.signed_angle))
            if len(rows) > 1:
                if block.mode == 2:
                    stations = np.linspace(0., measure, len(rows))
                else:
                    stations = np.asarray([row[0] for row in rows])
                risks = [max(0., (preferred-distance)/preferred)**2
                         for distance in block_clearances]
                risk_integral += float(trapezoid(risks, x=stations))
                peak_risk = max(peak_risk, max(risks))
        endpoint = pose
        goal_yaw = candidate.goal.th+2.*math.pi*round(
            (endpoint.th-candidate.goal.th)/(2.*math.pi))
        terminal = np.asarray([endpoint.x-candidate.goal.x,
                               endpoint.y-candidate.goal.y,
                               endpoint.th-goal_yaw], dtype=float)
        clearance = np.asarray(clearances, dtype=float)
        controls = np.asarray(control_margins, dtype=float)
        L0 = max(evaluator.L0, 1e-12)
        T0 = max(evaluator.T0, 1e-12)
        terms = {
            "clearance": (risk_integral/L0+.5*peak_risk),
            "shape": (curvature_energy/L0+.1*curvature_tv/(kmax*L0)),
            "adjustment": (curvature_tv/kmax+oblique_tv/(math.pi/4)),
            "time_proxy": ((length/evaluator.profile.reference_speed_mps+
                            spin_angle/evaluator.profile.reference_spin_omega_radps)/T0),
            "length": length/L0,
        }
        objective = (evaluator.profile.clearance*terms["clearance"]+
                     evaluator.profile.shape*terms["shape"]+
                     evaluator.profile.adjustment*terms["adjustment"]+
                     evaluator.profile.time*terms["time_proxy"]+
                     evaluator.profile.length*terms["length"])
        objective += 1e-4*float(np.dot(values, values))
        cache_value = {"plan": candidate, "rows_by_block": rows_by_block,
                       "clearance": clearance,
                       "control_margins": controls, "terminal": terminal,
                       "objective": float(objective), "terms": terms,
                       "min_clearance": float(np.min(clearance)) if len(clearance) else 0.,
                       "estimated_safe_clearance": (
                           float(np.min(clearance))-.04 if len(clearance) else 0.)}
        cache_key = key
        return cache_value

    try:
        seed_data = analyze(x0)
    except _DeadlineReached:
        return plan, {"attempted": True, "status": "budget_exhausted_before_seed_eval",
                      "converged": False, "iterations": 0,
                      "variable_count": len(layout), "elapsed_s": time.perf_counter()-begin,
                      "final_swept_audit_required": True}
    best_values = x0.copy()
    best_objective = seed_data["objective"]
    best_data = seed_data
    sample_margin = 0.0

    def objective(values):
        nonlocal best_values, best_objective, best_data
        data = analyze(values)
        terminal_ok = (np.linalg.norm(data["terminal"][:2]) <= 1e-3 and
                       abs(data["terminal"][2]) <= math.radians(.05))
        safe_ok = (len(data["clearance"]) and
                   float(np.min(data["clearance"])) >= -1e-7 and
                   (not len(data["control_margins"]) or
                    float(np.min(data["control_margins"])) >= -1e-7))
        if terminal_ok and safe_ok and data["objective"] < best_objective-1e-7:
            best_values = np.asarray(values, dtype=float).copy()
            best_objective = data["objective"]
            best_data = data
        return data["objective"]

    def terminal_constraint(values):
        # Spin angles and O body yaw are fixed in this deformation model.
        # A constant yaw equality gives SLSQP a zero Jacobian row.
        terminal = analyze(values)["terminal"]
        return terminal if any(plan.blocks[bi].mode == 0 for bi in block_columns) else terminal[:2]

    def clearance_constraint(values):
        return analyze(values)["clearance"]-sample_margin

    def control_constraint(values):
        return analyze(values)["control_margins"]

    constraints = [
        {"type": "eq", "fun": terminal_constraint},
        {"type": "ineq", "fun": clearance_constraint},
        {"type": "ineq", "fun": control_constraint},
    ]
    result = None
    stopped_by_deadline = False
    try:
        result = minimize(objective, x0, method="SLSQP", bounds=bounds,
                          constraints=constraints,
                          options={"maxiter": int(max_iterations), "ftol": 1e-4,
                                   "disp": False})
        final_data = analyze(result.x)
        final_ok = (np.linalg.norm(final_data["terminal"][:2]) <= 1e-3 and
                    abs(final_data["terminal"][2]) <= math.radians(.05) and
                    float(np.min(final_data["clearance"])) >= sample_margin-1e-6 and
                    (not len(final_data["control_margins"]) or
                     float(np.min(final_data["control_margins"])) >= -1e-6))
        if final_ok and final_data["objective"] < best_objective-1e-7:
            best_values = np.asarray(result.x, dtype=float).copy()
            best_objective = final_data["objective"]
            best_data = final_data
    except _DeadlineReached:
        stopped_by_deadline = True

    elapsed = time.perf_counter()-begin
    if best_objective >= seed_data["objective"]-1e-7:
        return plan, {"attempted": True,
                      "status": "budget_exhausted" if stopped_by_deadline else "no_improvement",
                      "converged": bool(result.success) if result is not None else False,
                      "solver_message": str(result.message) if result is not None else
                                       "wall_clock_budget_reached",
                      "iterations": int(getattr(result, "nit", 0)) if result is not None else 0,
                      "variable_count": len(layout), "elapsed_s": elapsed,
                      "objective_before": seed_data["objective"],
                      "objective_after": seed_data["objective"],
                      "min_body_clearance_before_m": seed_data["min_clearance"],
                      "min_body_clearance_after_m": seed_data["min_clearance"],
                      "terminal_error_from_solver_iterate": best_data["terminal"].tolist(),
                      "exact_terminal_constraint": True,
                      "obstacle_constraint": "exact_OBB_distance_samples",
                      "final_swept_audit_required": True}

    optimized = build(best_values)
    data = best_data
    return optimized, {"attempted": True,
                       "status": "budget_feasible_iterate" if stopped_by_deadline else
                                ("converged_feasible" if result is not None and result.success
                                 else "iteration_limit_feasible"),
                       "converged": bool(result.success) if result is not None else False,
                       "iterations": int(getattr(result, "nit", 0)) if result is not None else 0,
                       "solver_message": str(result.message) if result is not None else
                                        "wall_clock_budget_reached",
                       "variable_count": len(layout), "elapsed_s": elapsed,
                       "objective_before": seed_data["objective"],
                       "objective_after": data["objective"],
                       "objective_terms_after": data["terms"],
                       "min_body_clearance_before_m": seed_data["min_clearance"],
                       "min_body_clearance_after_m": data["min_clearance"],
                       "estimated_safe_clearance_after_m": data["estimated_safe_clearance"],
                       "terminal_error_after": data["terminal"].tolist(),
                       "fixed_mode_direction_word": [[b.mode, b.direction]
                                                     for b in plan.blocks],
                       "lengths_and_switch_stations_fixed": True,
                       "control_slope_policy":
                           "seed_normalized_slopes_plus_basis_derivative",
                       "obstacle_constraint": "exact_OBB_distance_samples",
                       "sample_spacing_m": sample_spacing,
                       "sample_margin_m": sample_margin,
                       "final_swept_audit_required": True}


def _optimize_continuous_geometry_ipopt(plan, evaluator, time_limit=1.0,
                                        sample_spacing=.50, max_variables=25):
    """CasADi/IPOPT solve on the existing continuous Hermite control profile."""
    import casadi as ca

    if not isinstance(plan, ContinuousPlan):
        raise ValueError("continuous_optimizer_plan_required")
    if not math.isfinite(time_limit) or time_limit <= 0:
        raise ValueError("continuous_optimizer_invalid_budget")
    begin = time.perf_counter()
    deadline = begin+time_limit
    endpoint_fit_report = None
    if time_limit > .10:
        # Candidate evaluation normally already projected its ContinuousPlan
        # to a strict endpoint tolerance. Repeating that expensive fit before
        # the NLP used to consume nearly half of the one-second solve budget.
        # Check the actual continuous rollout first and only project plans
        # that still need it.
        # Match fit_continuous_endpoint's 25 mm integration spacing. A coarse
        # check here can report a spurious terminal error for curved routes.
        endpoint_rows = replay_continuous(plan, .025)
        endpoint = endpoint_rows[-1][1]
        target_yaw = plan.goal.th+2.*math.pi*round(
            (endpoint.th-plan.goal.th)/(2.*math.pi))
        endpoint_xy_error = math.hypot(endpoint.x-plan.goal.x,
                                       endpoint.y-plan.goal.y)
        endpoint_yaw_error = abs(endpoint.th-target_yaw)
        # Candidate evaluation has already fitted this plan to the strict
        # release contract (5 mm / 0.2 deg). Do not spend a second fit budget
        # chasing 1e-7 before a bounded local solve; the NLP carries the exact
        # terminal equality and the final independent audit rechecks it.
        if (endpoint_xy_error <= .005 and
                endpoint_yaw_error <= math.radians(.2)):
            endpoint_fit_report = {
                "success": True,
                "status": "already_within_release_tolerance",
                "terminal_error_after": [endpoint_xy_error,
                                          endpoint_yaw_error],
            }
        else:
            fit_deadline = begin+min(.45*time_limit, time_limit-.10)
            try:
                plan, endpoint_fit_report = fit_continuous_endpoint(
                    plan, evaluator.limits.r_min_ack if evaluator.limits else .810,
                    max_iterations=80, terminal_xy_tol=1e-7,
                    terminal_yaw_tol=1e-7, deadline=fit_deadline)
            except (ValueError, RuntimeError) as exc:
                return plan, {"attempted": False,
                              "status": "endpoint_projection_budget_or_failure",
                              "solver_message": str(exc).splitlines()[-1],
                              "elapsed_s": time.perf_counter()-begin,
                              "endpoint_fit": endpoint_fit_report,
                              "final_swept_audit_required": True}
            if not endpoint_fit_report.get("success"):
                return plan, {"attempted": False, "status": "endpoint_projection_failed",
                              "endpoint_fit": endpoint_fit_report,
                              "elapsed_s": time.perf_counter()-begin,
                              "final_swept_audit_required": True}
    movable = [i for i, block in enumerate(plan.blocks)
               if block.mode in (0, 1) and len(block.knots) >= 3]
    if not movable:
        return plan, {"attempted": False, "status": "no_continuous_control_dofs",
                      "iterations": 0, "elapsed_s": time.perf_counter()-begin}
    base_control_window_m = 3.0
    control_window_m = base_control_window_m

    def allocate_windows(window_length):
        bases_by_block = {}
        count = 0
        for bi in movable:
            block = plan.blocks[bi]
            window_count = max(1, int(math.ceil(block.length/window_length)))
            bases = tuple((wi/window_count, (wi+1.)/window_count, degree)
                          for wi in range(window_count) for degree in range(3))
            bases_by_block[bi] = bases
            count += len(bases)
        return bases_by_block, count

    block_bases, variable_count = allocate_windows(control_window_m)
    # Preserve the 25-variable solver contract. When a route barely exceeds
    # it, coalesce adjacent local windows on longer blocks until the budget
    # fits; this keeps local deformation instead of skipping the whole path.
    # The actual spacing and any adaptation are stored in the solver report.
    while variable_count > max_variables and control_window_m < 12.0:
        control_window_m = min(12.0, control_window_m+.25)
        block_bases, variable_count = allocate_windows(control_window_m)
    if variable_count > max_variables:
        return plan, {"attempted": False, "status": "topology_exceeds_variable_budget",
                      "variable_count": variable_count,
                      "max_variables": max_variables,
                      "base_control_window_length_m": base_control_window_m,
                      "control_window_length_m": control_window_m,
                      "variable_budget_adapted": control_window_m != base_control_window_m,
                      "elapsed_s": time.perf_counter()-begin}

    opt = ca.Opti()
    beta_by_block = {}
    beta_offsets = {}
    beta_values = []
    for bi in movable:
        block = plan.blocks[bi]
        bases = block_bases[bi]
        beta = opt.variable(len(bases))
        max_delta = .16
        opt.subject_to(opt.bounded(-max_delta, beta, max_delta))
        opt.set_initial(beta, np.zeros(len(bases)))
        beta_by_block[bi] = beta
        beta_offsets[bi] = len(beta_values)
        beta_values.extend([beta[i] for i in range(len(bases))])

    occupied = np.asarray(evaluator.grid.occ, dtype=bool)
    free = np.pad(~occupied, 1, constant_values=False)
    field = np.maximum(0.,
        distance_transform_edt(free)[1:-1, 1:-1]*
        evaluator.grid.res-evaluator.grid.res/math.sqrt(2.))
    x_axis = evaluator.grid.ox+(np.arange(evaluator.grid.width)+.5)*evaluator.grid.res
    y_axis = evaluator.grid.oy+(np.arange(evaluator.grid.height)+.5)*evaluator.grid.res
    distance_field = ca.interpolant(
        "continuous_clearance_%d" % int(begin*1e6), "linear",
        [x_axis.tolist(), y_axis.tolist()],
        field.T.ravel(order="F").tolist())
    body_features = features(evaluator.grid)
    # A circumscribed disc for each quarter-body strip overstates the actual
    # rectangle substantially near the front/rear corners. Account for the
    # maximum geometric overcoverage in the NLP proxy; exact OBB sweep remains
    # the sole safety gate for accepting a candidate.
    half_length, half_width = evaluator.length/2., evaluator.width/2.
    proxy_overcoverage = max((radius-min(half_length-abs(ox),
                                           half_width-abs(oy)))
                             for ox, oy, radius in body_features)
    proxy_slack = max(.025, proxy_overcoverage+.5*evaluator.grid.res)
    kmax = 1./(evaluator.limits.r_min_ack if evaluator.limits else .810)
    preferred = max(1e-6, evaluator.profile.preferred_clearance_factor*evaluator.width)
    objective = 0
    clearance_constraints = []
    field_bounds_constraints = []
    control_constraints = []
    clearance_pose_count = 0
    control_bound_sample_count = 0
    pose_x, pose_y, pose_yaw = plan.start.x, plan.start.y, plan.start.th
    total_length = max(sum(b.length for b in plan.blocks if b.mode != 2), 1e-9)
    # Two-point Gauss is sufficient for this local search model; the solved
    # candidate is replayed at 25 mm and independently swept at 5 mm before
    # it can enter the feasible pool.
    gauss_nodes = (-.5773502691896257, .5773502691896257)
    gauss_weights = (1., 1.)

    def polynomial(q, c0, c1, m0, m1):
        h00 = 2*q**3-3*q**2+1
        h10 = q**3-2*q**2+q
        h01 = -2*q**3+3*q**2
        h11 = q**3-q**2
        return h00*c0+h10*m0+h01*c1+h11*m1

    def primitive(q, c0, c1, m0, m1):
        p00 = .5*q**4-q**3+q
        p10 = .25*q**4-(2./3.)*q**3+.5*q**2
        p01 = -.5*q**4+q**3
        p11 = .25*q**4-(1./3.)*q**3
        return p00*c0+p10*m0+p01*c1+p11*m1

    def add_clearance(px, py, yaw, sample_weight, pose_has_variables):
        nonlocal objective, clearance_pose_count
        clearance_pose_count += 1
        local_clearances = []
        for ox, oy, radius in body_features:
            fx = px+ox*ca.cos(yaw)-oy*ca.sin(yaw)
            fy = py+ox*ca.sin(yaw)+oy*ca.cos(yaw)
            if pose_has_variables:
                field_bounds_constraints.extend((fx-float(x_axis[0]),
                    float(x_axis[-1])-fx, fy-float(y_axis[0]),
                    float(y_axis[-1])-fy))
            clearance = distance_field(ca.vertcat(fx, fy))-radius
            # This grid/circle constraint guides the solve. It never certifies
            # the rectangular body; final acceptance uses the exact sweep.
            if pose_has_variables:
                local_clearances.append(clearance+proxy_slack)
            deficit = ca.fmax(0., (preferred-clearance)/preferred)
            objective += (evaluator.profile.clearance*sample_weight*
                          deficit*deficit/len(body_features))
        clearance_constraints.extend(local_clearances)

    def control_terms(block, bi):
        knots = block.knots
        fractions = [k.s/block.length for k in knots]
        beta = beta_by_block.get(bi)
        bases = block_bases.get(bi, ())
        controls, slopes = [], []
        for knot, t in zip(knots, fractions):
            control = knot.value
            normalized_slope = knot.derivative*block.length
            if beta is not None:
                for index, (start, end, degree) in enumerate(bases):
                    control += beta[index]*_window_basis(t, start, end, degree)
                    normalized_slope += beta[index]*_window_basis_derivative(
                        t, start, end, degree)
            controls.append(control)
            slopes.append(normalized_slope)
        return fractions, controls, slopes

    try:
        pose_has_variables = False
        for bi, block in enumerate(plan.blocks):
            if time.perf_counter() >= deadline:
                raise _DeadlineReached("continuous_optimizer_model_budget")
            add_clearance(pose_x, pose_y, pose_yaw,
                          .5*max(block.length, evaluator.length)/total_length,
                          pose_has_variables)
            if block.mode == 2:
                count = max(1, int(math.ceil(abs(block.signed_angle)/math.radians(2.))))
                for i in range(1, count+1):
                    yaw_i = pose_yaw+block.signed_angle*i/count
                    add_clearance(pose_x, pose_y, yaw_i,
                                  evaluator.length*abs(block.signed_angle)/
                                  (total_length*count), pose_has_variables)
                pose_yaw += block.signed_angle
                continue

            block_start_yaw = pose_yaw
            fractions, controls, slopes = control_terms(block, bi)
            ds_total = block.length/total_length
            curvature_tv = 0
            oblique_tv = 0
            block_integral_prefix = [0]
            for j in range(len(fractions)-1):
                dfj = fractions[j+1]-fractions[j]
                block_integral_prefix.append(
                    block_integral_prefix[-1]+dfj*primitive(
                        1., controls[j], controls[j+1],
                        slopes[j]*dfj, slopes[j+1]*dfj))
            for left in range(len(fractions)-1):
                f0, f1 = fractions[left], fractions[left+1]
                df = f1-f0
                c0, c1 = controls[left], controls[left+1]
                m0, m1 = slopes[left]*df, slopes[left+1]*df
                completed = block_integral_prefix[left]
                if block.mode == 0:
                    curvature_tv += ca.sqrt((c1-c0)**2+1e-10)
                else:
                    oblique_tv += ca.sqrt((c1-c0)**2+1e-10)
                # Local window bases have higher polynomial degree than the
                # seed Hermite profile. Dense control checks reduce the chance
                # of an off-grid curvature/oblique-angle extremum; the final
                # continuous dynamics audit remains authoritative.
                for q in np.linspace(0., 1., 11):
                    c = polynomial(q, c0, c1, m0, m1)
                    if beta_by_block.get(bi) is not None:
                        margin = (0. if q <= 1e-12 or q >= 1.-1e-12 else
                                  (.005 if block.mode == 0 else .01))
                        if block.mode == 0:
                            control_constraints.extend((c+kmax-margin,
                                                        kmax-margin-c))
                        else:
                            control_constraints.extend((c+math.pi/2-margin,
                                                        math.pi/2-margin-c))
                        control_bound_sample_count += 1

                subdivisions = max(1, int(math.ceil(block.length*df/sample_spacing)))
                for sub in range(subdivisions):
                    if sub % 24 == 0 and time.perf_counter() >= deadline:
                        raise _DeadlineReached("continuous_optimizer_model_budget")
                    q0, q1 = sub/subdivisions, (sub+1)/subdivisions
                    du = q1-q0
                    ds = block.length*df*du
                    mid_yaw = pose_yaw
                    dx = 0
                    dy = 0
                    control_energy = 0
                    for node, weight in zip(gauss_nodes, gauss_weights):
                        q = q0+.5*du*(node+1.)
                        control = polynomial(q, c0, c1, m0, m1)
                        if block.mode == 0:
                            int_to_q = completed+df*primitive(q, c0, c1, m0, m1)
                            angle = block_start_yaw+block.direction*block.length*int_to_q
                            control_energy += weight*(control/kmax)**2
                        else:
                            angle = pose_yaw+control
                        dx += weight*ca.cos(angle)
                        dy += weight*ca.sin(angle)
                    pose_x += block.direction*.5*ds*dx
                    pose_y += block.direction*.5*ds*dy
                    pose_has_variables = (pose_has_variables or
                                          beta_by_block.get(bi) is not None)
                    if block.mode == 0:
                        # Recompute the block-relative yaw from its exact
                        # integrated curvature, without accumulating quadrature error.
                        pose_yaw = (block_start_yaw + block.direction*block.length*
                                    (completed+df*primitive(q1, c0, c1, m0, m1)))
                    objective += (evaluator.profile.shape*ds_total*du*
                                  .5*control_energy)
                    add_clearance(pose_x, pose_y, pose_yaw,
                                  ds_total*du, pose_has_variables)
                if block.mode == 0:
                    objective += (evaluator.profile.adjustment*curvature_tv/kmax/
                                  max(1, len(fractions)-1))
                else:
                    objective += (evaluator.profile.adjustment*oblique_tv/
                                  (math.pi/4*max(1, len(fractions)-1)))
            # The next block starts at the exact integrated endpoint of this one.
        if clearance_constraints:
            opt.subject_to(ca.vertcat(*clearance_constraints) >= 0.)
        if field_bounds_constraints:
            opt.subject_to(ca.vertcat(*field_bounds_constraints) >= 0.)
        if control_constraints:
            opt.subject_to(ca.vertcat(*control_constraints) >= 0.)
        seed_end = replay_continuous(plan, sample_spacing)[-1][1]
        target_yaw = plan.goal.th+2.*math.pi*round(
            (seed_end.th-plan.goal.th)/(2.*math.pi))
        terminal_components = _constrain_terminal_pose(
            opt, (pose_x, pose_y, pose_yaw),
            (plan.goal.x, plan.goal.y, target_yaw))
        objective += 1e-4*sum(beta*beta for beta in beta_values)
        opt.minimize(objective)
        setup_elapsed = time.perf_counter()-begin
        remaining = time_limit-setup_elapsed
        if remaining < .05:
            raise _DeadlineReached("continuous_optimizer_setup_budget")
        # Expand the scalar control graph once. Limited-memory Hessians avoid
        # repeatedly differentiating the full obstacle/interpolation graph;
        # every returned iterate still needs the independent swept audit.
        opt.solver("ipopt", {"print_time": False, "expand": True},
                   {"print_level": 0, "max_iter": 120,
                    "hessian_approximation": "limited-memory",
                    "max_wall_time": remaining, "tol": 1e-5})
        solution = opt.solve_limited()
        values = np.asarray(solution.value(ca.vertcat(*beta_values)), dtype=float).ravel()
        objective_after = float(solution.value(objective))

        blocks = list(plan.blocks)
        for bi in movable:
            original = plan.blocks[bi]
            offset = beta_offsets[bi]
            bases = block_bases[bi]
            beta = values[offset:offset+len(bases)]
            knots = []
            for knot in original.knots:
                t = knot.s/original.length
                value = knot.value+sum(beta[j]*_window_basis(t, start, end, degree)
                    for j, (start, end, degree) in enumerate(bases))
                normalized_slope = knot.derivative*original.length+sum(
                    beta[j]*_window_basis_derivative(t, start, end, degree)
                    for j, (start, end, degree) in enumerate(bases))
                knots.append(ControlKnot(knot.s, float(value),
                                         float(normalized_slope/original.length)))
            blocks[bi] = MotionBlock(original.mode, original.direction,
                                     original.length, tuple(knots),
                                     slope_policy="fixed")
        optimized = ContinuousPlan(plan.start, tuple(blocks), plan.goal)
        optimized_end = replay_continuous(optimized, .025)[-1][1]
        xy_error = math.hypot(optimized_end.x-plan.goal.x,
                              optimized_end.y-plan.goal.y)
        yaw_error = abs(optimized_end.th-target_yaw)
        if xy_error > .005 or yaw_error > math.radians(.2):
            return plan, {"attempted": True, "status": "rollout_mismatch",
                          "converged": False,
                          "solver_status": opt.stats().get("return_status", "unknown"),
                          "iterations": int(opt.stats().get("iter_count", 0)),
                          "variable_count": variable_count,
                          "base_control_window_length_m": base_control_window_m,
                          "control_window_length_m": control_window_m,
                          "variable_budget_adapted": control_window_m != base_control_window_m,
                          "elapsed_s": time.perf_counter()-begin,
                          "terminal_error_after": [xy_error, yaw_error],
                          "objective_after": objective_after,
                          "final_swept_audit_required": True}
        return optimized, {"attempted": True,
                           "status": "solver_feasible_candidate",
                           "converged": bool(opt.stats().get("success", False)),
                           "solver_status": opt.stats().get("return_status", "unknown"),
                           "iterations": int(opt.stats().get("iter_count", 0)),
                           "variable_count": variable_count,
                           "base_control_window_length_m": base_control_window_m,
                           "control_basis": "local_c1_arc_windows",
                           "control_window_length_m": control_window_m,
                           "variable_budget_adapted": control_window_m != base_control_window_m,
                           "interior_control_margin": {"A_curvature_per_m": .005,
                                                        "O_angle_rad": .01},
                           "variables_by_block": {
                               str(bi): len(block_bases[bi]) for bi in movable},
                           "elapsed_s": time.perf_counter()-begin,
                           "model_setup_s": setup_elapsed,
                           "endpoint_fit": endpoint_fit_report,
                           "endpoint_constraint_components": terminal_components,
                           "objective_before": None,
                           "objective_after": objective_after,
                           "terminal_error_after": [xy_error, yaw_error],
                           "fixed_mode_direction_word": [[b.mode, b.direction]
                                                         for b in plan.blocks],
                           "lengths_and_switch_stations_fixed": True,
                           "control_slope_policy":
                               "seed_normalized_slopes_plus_basis_derivative",
                           "obstacle_constraint": "four_disc_distance_field_proxy",
                           "clearance_sample_spacing_m": sample_spacing,
                           "clearance_proxy_slack_m": proxy_slack,
                           "clearance_proxy_geometric_overcoverage_m":
                               proxy_overcoverage,
                           "integration_quadrature": "two_point_gauss",
                           "final_swept_audit_required": True}
    except _DeadlineReached as exc:
        return plan, {"attempted": True, "status": "budget_exhausted",
                      "converged": False, "iterations": 0,
                      "variable_count": variable_count,
                      "base_control_window_length_m": base_control_window_m,
                      "control_window_length_m": control_window_m,
                      "variable_budget_adapted": control_window_m != base_control_window_m,
                      "elapsed_s": time.perf_counter()-begin,
                      "solver_message": str(exc),
                      "endpoint_fit": endpoint_fit_report,
                      "model_clearance_poses": clearance_pose_count,
                      "model_clearance_constraints": len(clearance_constraints),
                      "model_field_bounds_constraints": len(field_bounds_constraints),
                      "model_control_bound_samples": control_bound_sample_count,
                      "final_swept_audit_required": True}
    except (RuntimeError, ValueError) as exc:
        debug_terminal = None
        try:
            debug_terminal = [float(opt.debug.value(pose_x)),
                              float(opt.debug.value(pose_y)),
                              float(opt.debug.value(pose_yaw))]
        except Exception:
            pass
        return plan, {"attempted": True, "status": "no_solution",
                      "converged": False, "iterations": 0,
                      "variable_count": variable_count,
                      "base_control_window_length_m": base_control_window_m,
                      "control_window_length_m": control_window_m,
                      "variable_budget_adapted": control_window_m != base_control_window_m,
                      "elapsed_s": time.perf_counter()-begin,
                      "solver_message": str(exc).splitlines()[-1],
                      "debug_terminal": debug_terminal,
                      "final_swept_audit_required": True}


def optimize_continuous_geometry(plan, evaluator, time_limit=1.0,
                                 sample_spacing=.50, max_iterations=120,
                                 max_variables=25):
    """Run the differentiable fixed-topology solver, with a small SciPy fallback."""
    try:
        candidate, report = _optimize_continuous_geometry_ipopt(
            plan, evaluator, time_limit=time_limit, sample_spacing=sample_spacing,
            max_variables=max_variables)
        report["solver_configuration"] = {"expand": True,
            "hessian_approximation": "limited-memory"}
        return candidate, report
    except ImportError:
        return _optimize_continuous_geometry_scipy(
            plan, evaluator, time_limit=time_limit,
            sample_spacing=min(sample_spacing, .08),
            max_iterations=max_iterations, max_variables=max_variables)
