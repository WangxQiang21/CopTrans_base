"""Fit and audit a straight-to-straight C2 A replacement for a turning connector."""
from dataclasses import asdict, replace
import math
import time

import numpy as np
from scipy.integrate import quad
from scipy.optimize import least_squares

from .continuous_geometry import ControlKnot
from .formation import FormationPose, FormationValidationError
from .formation_collision import FormationCollisionChecker
from .formation_continuous_a.audit import audit_dynamics, audit_geometry
from .formation_continuous_a.collision import audit_collision
from .formation_continuous_a.contracts import (
    CollisionConfig, GoalPolicy, InitialAssumption, PlannerConfig, TimingConfig,
    VirtualAContractError, VirtualAMapRequest, VirtualARequest, request_fingerprint,
)
from .formation_continuous_a.geometry import C2CurvatureProfile, VirtualAPath
from .formation_continuous_a.timing import retime_path
from .formation_quality import FEASIBLE, FormationEvaluation
from .uniform_mixed_trajectory import UniformMixedTimedPlan
from .uniform_motion import (
    MODE_S, PRIMITIVE_A_ARC, PRIMITIVE_A_STRAIGHT, a_straight_edge, o_translation_edge, s_spin_edge,
)
from .uniform_search import _o_branch

_NODES, _WEIGHTS = np.polynomial.legendre.leggauss(12)
_NODES, _WEIGHTS = (_NODES + 1.0) / 2.0, _WEIGHTS / 2.0
_CLAMPED_CUBIC = np.array(((4, 1, 0), (1, 4, 1), (0, 1, 4)), dtype=float)


def _advance(qf, length):
    return FormationPose(qf.x + length * math.cos(qf.psi),
                         qf.y + length * math.sin(qf.psi), qf.psi)


def _fit_path(request, arc_length_m):
    """Fit C2 endpoint parameters, then let the exact path model rebuild."""
    start = request.start_qf
    target = request.goal_qf
    xy = np.array((target.x - start.x, target.y - start.y))
    distance = float(np.linalg.norm(xy))
    if distance < 1.0:
        return None
    angle = target.psi - start.psi
    integral_delta = _NODES**3 - 0.5 * _NODES**4
    integral_left_slope = _NODES**4/4 - 2*_NODES**3/3 + _NODES**2/2
    integral_right_slope = _NODES**4/4 - _NODES**3/3

    def values(asymmetry, length):
        steady = 4 * angle / (3 * length)
        knots = np.array((0.0, steady + asymmetry, steady, steady - asymmetry, 0.0))
        # Clamped endpoint slopes and continuous second derivatives at the
        # three internal knots uniquely determine the Hermite slopes.
        slopes = np.zeros(5)
        slopes[1:-1] = np.linalg.solve(
            _CLAMPED_CUBIC, 12 * (knots[2:] - knots[:-2]) / length)
        return knots, slopes

    def residual(params):
        asymmetry, length = params
        knots, slopes = values(asymmetry, length)
        step = length / 4
        heading = start.psi
        integral = np.zeros(2)
        for left, right, d_left, d_right in zip(knots, knots[1:], slopes, slopes[1:]):
            yaw = heading + step * (
                left * _NODES + (right - left) * integral_delta
                + step * (d_left * integral_left_slope + d_right * integral_right_slope))
            integral += step * np.array((np.dot(_WEIGHTS, np.cos(yaw)),
                                         np.dot(_WEIGHTS, np.sin(yaw))))
            heading += step * ((left + right) / 2 + step * (d_left - d_right) / 12)
        return integral - xy

    upper = max(distance * 1.8, arc_length_m * 1.3)
    fit = least_squares(residual, (0.0, min(upper, max(distance, arc_length_m))),
                        bounds=((-3.0, distance), (3.0, upper)),
                        xtol=1e-13, ftol=1e-13, gtol=1e-13, max_nfev=45)
    if np.linalg.norm(fit.fun) > 1e-8:
        return None
    _, length = fit.x
    knots, slopes = values(*fit.x)
    profile = C2CurvatureProfile.from_knots([
        ControlKnot(index * length / 4, k, derivative)
        for index, (k, derivative) in enumerate(zip(knots, slopes))])
    path = VirtualAPath(start, profile, request.model)
    path.request_sha256 = request_fingerprint(request)
    return path


def _pose_of_edge(edge, start):
    return (edge.end_qf, dict(edge.end_headings)) if edge is not None else start


def _translate(request, qf, headings, target, motion_limits):
    dx, dy = target.x - qf.x, target.y - qf.y
    if math.hypot(dx, dy) <= 1e-9:
        return None
    bearing = math.atan2(dy, dx)
    reverse = request.successor_config.allow_reverse
    branches = {i: _o_branch(theta, bearing, reverse) for i, theta in headings.items()}
    if any(branch is None for branch in branches.values()):
        raise FormationValidationError("C2_SPAN_TRANSLATION_UNAVAILABLE")
    directions = {i: branch[0] for i, branch in branches.items()}
    angles = {i: branch[1] for i, branch in branches.items()}
    if all(abs(angle) <= 1e-9 for angle in angles.values()):
        return a_straight_edge(request.model, qf, headings, (dx, dy), directions,
                               limits=motion_limits, allow_reverse=reverse)
    return o_translation_edge(request.model, qf, headings, (dx, dy), directions, angles,
                              limits=motion_limits, allow_reverse=reverse)


def _align(request, qf, headings, target, motion_limits):
    deltas = {i: math.remainder(target[i] - theta, math.tau) for i, theta in headings.items()}
    if all(abs(delta) <= 1e-9 for delta in deltas.values()):
        return None
    return s_spin_edge(request.model, qf, headings, deltas, limits=motion_limits)


def _cargo_collision_witness(request, path, margin):
    """Reject only if an inner cargo OBB hits the map at an evaluated station.

    The inner half-widths account for translation and rotation error of the
    integrated path. A pass here has no collision-certification meaning.
    """
    cargo = request.cargo_footprint
    if cargo is None:
        return False
    corner = math.hypot(cargo.length_m + margin, cargo.width_m + margin) / 2
    lever = math.hypot(cargo.offset_x_m, cargo.offset_y_m) + corner
    slack = path.position_error_bound_m + lever * path.angle_error_bound_rad + 1e-8
    if 2 * slack >= min(cargo.length_m, cargo.width_m):
        return False
    inner = replace(cargo, length_m=cargo.length_m - 2 * slack,
                    width_m=cargo.width_m - 2 * slack)
    checker = FormationCollisionChecker(
        request.grid, request.model, request.vehicle_shapes, margin, cargo_footprint=inner)
    count = min(64, max(1, math.ceil(path.length_m / .25)))
    return any(not checker.cargo_map_clear(path.state_at_s(float(s)).qf)
               for s in np.linspace(0, path.length_m, count + 1))


def _audit_curve(request, prefix, target, path, cutoff, *, screen_cargo):
    opts = request.successor_config
    curve_request = VirtualARequest(
        request.plan_id, request.formation_id, request.model, request.vehicle_shapes,
        prefix.goal_qf, dict(prefix.goal_headings), target,
        InitialAssumption(True, "A", True, "verified_terminal", prefix.model_sha256()),
        GoalPolicy.STRAIGHT, opts.timing_limits)
    path.request_sha256 = request_fingerprint(curve_request)
    limits = CollisionConfig(max(0.05, opts.collision_margin_m),
                             min(0.01, opts.sweep_spacing_m),
                             min(200000, opts.max_samples_per_segment))
    map_request = VirtualAMapRequest(curve_request, request.grid, PlannerConfig(), TimingConfig(), limits)
    context = map_request.context()
    geometry = audit_geometry(curve_request, path, planning_context=context)
    if not geometry.passed or time.perf_counter() >= cutoff:
        return None
    if screen_cargo and _cargo_collision_witness(request, path, limits.baseline_margin_m):
        return None
    if time.perf_counter() >= cutoff:
        return None
    result = retime_path(curve_request, path, map_request.timing_config, planning_context=context)
    if result.plan is None or time.perf_counter() >= cutoff:
        return None
    # Sweep the map first so an obstructed route never spends the budget on
    # the more expensive interval and inverse-time dynamics checks.
    collision = audit_collision(curve_request, result.plan, request.grid, limits,
                                cargo_footprint=request.cargo_footprint)
    if not collision.passed or time.perf_counter() >= cutoff:
        return None
    dynamics = audit_dynamics(curve_request, result.plan, expected_planning_context=context)
    if not dynamics.passed or time.perf_counter() >= cutoff:
        return None
    return result.plan, (geometry, dynamics, collision)


def _mixed_result(request, prefix, curve, suffix, audits, cutoff):
    clock = None
    if request.reachability_retime and time.perf_counter() < cutoff:
        from .uniform_curve_clock import choose_curve_clock

        try:
            clock = choose_curve_clock(curve, prefix.limits)
        except ValueError:
            pass
    mixed = UniformMixedTimedPlan(prefix, curve, suffix, clock=clock)
    projection = mixed.sample()
    if time.perf_counter() >= cutoff:
        return None
    joins = int(bool(prefix.edges)) + int(bool(suffix.edges))
    metrics = {
        "model_version": mixed.model_version,
        "model_sha256": mixed.model_sha256(),
        "map_version_checked": True,
        "duration_s": mixed.duration_s,
        "continuous_a_length_m": curve.total_S_m,
        "continuous_a_duration_s": mixed.curve_phase.duration_s,
        "phase_totals": mixed.phase_totals(),
        "stopped_join_count": joins,
    }
    if request.cargo_footprint is not None:
        metrics["cargo_footprint"] = asdict(request.cargo_footprint)
    evaluation = FormationEvaluation(
        plan_id=request.plan_id, formation_id=request.formation_id, status=FEASIBLE,
        feasible=True, reason="OK", failure=None,
        detail="uniform pieces, continuous A geometry, dynamics, collision and stopped joins audited",
        checks=sum(a.checks if isinstance(a, FormationEvaluation) else a.check_count
                   for a in audits) + joins,
        metrics=metrics)
    return mixed, projection, evaluation


def _candidate(request, base, arc_index, padding, audit_uniform, cutoff, *, screen_cargo=False):
    # The short real A leg fixes the prefix's actual stopped wheel state.
    # O/S profiles before it are timed by the existing stationary setup law.
    reference = base.edges[arc_index]
    before, after = padding
    pre = _advance(reference.start_qf, -before)
    post = _advance(reference.end_qf, after)
    opts = request.successor_config
    motion = base.motion_limits
    prefix_edges = list(base.edges[:arc_index])
    if prefix_edges and prefix_edges[-1].mode == "S":
        prefix_edges.pop()
    qf, headings = (_pose_of_edge(prefix_edges[-1], None) if prefix_edges
                    else (request.start_qf, request.start_headings))
    starts_on_curve = (not prefix_edges and math.hypot(pre.x - qf.x, pre.y - qf.y) <= 1e-9
                       and request.initial_edge is None
                       and all(abs(math.remainder(theta - qf.psi, math.tau)) <= 1e-9
                               for theta in headings.values()))
    staging = pre if starts_on_curve else _advance(pre, -0.35)
    incoming = _translate(request, qf, headings, staging, motion)
    if incoming is not None:
        prefix_edges.append(incoming)
    qf, headings = _pose_of_edge(incoming, (qf, headings))
    smooth_start_headings = dict.fromkeys(base.vehicle_ids, qf.psi)
    align = _align(request, qf, headings, smooth_start_headings, motion)
    if align is not None:
        prefix_edges.append(align)
        qf, headings = _pose_of_edge(align, None)
    if not starts_on_curve:
        prefix_edges.append(a_straight_edge(
            request.model, qf, headings, (pre.x - qf.x, pre.y - qf.y),
            dict.fromkeys(base.vehicle_ids, 1), limits=motion, allow_reverse=opts.allow_reverse))
    prefix_request = replace(request, goal_qf=pre, goal_headings=smooth_start_headings)
    prefix, _, prefix_audit = audit_uniform(
        prefix_edges, prefix_request, zero_motion=starts_on_curve)
    if not prefix_audit.feasible or time.perf_counter() >= cutoff:
        return None

    curve_request = VirtualARequest(
        request.plan_id, request.formation_id, request.model, request.vehicle_shapes,
        prefix.goal_qf, dict(prefix.goal_headings), post,
        InitialAssumption(True, "A", True, "verified_terminal", prefix.model_sha256()),
        GoalPolicy.STRAIGHT, opts.timing_limits)
    arc_length = abs(reference.control.delta_psi) * math.hypot(
        reference.control.pivot_x, reference.control.pivot_y)
    path = _fit_path(curve_request, arc_length + before + after)
    if path is None or time.perf_counter() >= cutoff:
        return None
    checked = _audit_curve(request, prefix, post, path, cutoff, screen_cargo=screen_cargo)
    if checked is None:
        return None
    curve, curve_audits = checked
    endpoint = curve.state_at(curve.duration_s)
    headings = {v.vehicle_id: v.theta for v in endpoint.vehicles}
    suffix_request = replace(request, start_qf=endpoint.qf, start_headings=headings, initial_edge=None)
    waypoints = tuple(edge.end_qf for edge in base.edges[arc_index + 1:]
                      if edge.mode in ("A", "O")
                      and abs(math.remainder(edge.end_qf.psi - endpoint.qf.psi, math.tau)) <= 1e-9)
    suffix = None
    for route in dict.fromkeys(((request.goal_qf,), (*waypoints, request.goal_qf))):
        if time.perf_counter() >= cutoff:
            return None
        suffix_edges = []
        qf, current = endpoint.qf, headings
        for target in route:
            tail = _translate(request, qf, current, target, motion)
            if tail is not None:
                suffix_edges.append(tail)
                qf, current = _pose_of_edge(tail, None)
        if request.goal_headings is not None:
            align = _align(request, qf, current, request.goal_headings, motion)
            if align is not None:
                suffix_edges.append(align)
        plan, _, suffix_audit = audit_uniform(suffix_edges, suffix_request,
                                              zero_motion=not suffix_edges)
        if suffix_audit.feasible:
            suffix = plan
            break
    if suffix is None or time.perf_counter() >= cutoff:
        return None

    return _mixed_result(request, prefix, curve, suffix,
                         (prefix_audit, *curve_audits, suffix_audit), cutoff)


def _cruise_profile(start, target):
    """Fit an exactly anchored bend and tangent cruise with one C² control law."""
    turn = math.remainder(target.psi - start.psi, math.tau)
    if abs(turn) < math.radians(4.):
        return None
    heights = (0., 1./6., 2./3., 1./6., 0.)
    slopes = (0., .5, 0., -.5, 0.)

    def knots(length):
        return [ControlKnot(i * length / 4., 4. * turn * value / length,
                            16. * turn * slope / length**2)
                for i, (value, slope) in enumerate(zip(heights, slopes))]

    unit = C2CurvatureProfile.from_knots(knots(1.))
    displacement = [quad(lambda s: trig(start.psi + unit.psi_at(s)), 0., 1.,
                         points=(.25, .5, .75), epsabs=1e-12)[0]
                    for trig in (math.cos, math.sin)]
    tangent = math.cos(target.psi), math.sin(target.psi)
    try:
        bend, cruise = np.linalg.solve(np.array((displacement, tangent)).T,
                                      (target.x - start.x, target.y - start.y))
    except np.linalg.LinAlgError:
        return None
    if bend < 1. or cruise < 1.:
        return None
    return C2CurvatureProfile.from_knots(knots(float(bend)) + [
        ControlKnot(float(bend + cruise), 0., 0.)])


def _from_start(request, base, profile, target, tail, audit_uniform, cutoff):
    if request.initial_edge is not None or any(abs(math.remainder(theta - request.start_qf.psi, math.tau)) > 1e-9
                                               for theta in request.start_headings.values()):
        return None
    prefix_request = replace(request, goal_qf=request.start_qf, goal_headings=request.start_headings)
    prefix, _, prefix_audit = audit_uniform((), prefix_request, zero_motion=True)
    if not prefix_audit.feasible:
        return None
    path = VirtualAPath(prefix.goal_qf, profile, request.model)
    checked = _audit_curve(request, prefix, target, path, cutoff, screen_cargo=True)
    if checked is None:
        return None
    curve, curve_audits = checked
    endpoint = curve.state_at(curve.duration_s)
    headings = {v.vehicle_id: v.theta for v in endpoint.vehicles}
    suffix_request = replace(request, start_qf=endpoint.qf, start_headings=headings, initial_edge=None)
    tail = tuple(tail)
    if tail:
        first = tail[0]
        target_headings = first.end_headings if first.mode == MODE_S else first.start_headings
        align = _align(request, endpoint.qf, headings, dict(target_headings), base.motion_limits)
        tail = (() if align is None else (align,)) + (tail[1:] if first.mode == MODE_S else tail)
    elif request.goal_headings is not None:
        align = _align(request, endpoint.qf, headings, dict(request.goal_headings), base.motion_limits)
        tail = () if align is None else (align,)
    suffix, projection, suffix_audit = audit_uniform(tail, suffix_request, zero_motion=not tail)
    if not suffix_audit.feasible:
        return None
    if tail and request.reachability_retime and time.perf_counter() < cutoff:
        from .uniform_reachability import retime_certified_uniform

        try:
            retimed = retime_certified_uniform(suffix, suffix_audit)
        except ValueError:
            retimed = None
        if retimed is not None:
            suffix, projection, suffix_audit = retimed
    return _mixed_result(request, prefix, curve, suffix,
                         (prefix_audit, *curve_audits, suffix_audit), cutoff)


def refine_cruise(request, base, audit_uniform, cutoff):
    """Replace the approach and the following A cruise with one smooth span."""
    straights = sorted(((edge.control.parameter_length, index, edge.end_qf)
                        for index, edge in enumerate(base.edges)
                        if edge.primitive == PRIMITIVE_A_STRAIGHT),
                       reverse=True, key=lambda row: row[0])
    for length, index, target in straights[:3]:
        if length < 4. or time.perf_counter() >= cutoff or (request.cancelled and request.cancelled()):
            return None
        try:
            profile = _cruise_profile(request.start_qf, target)
            if profile is not None:
                candidate = _from_start(request, base, profile, target, base.edges[index + 1:],
                                        audit_uniform, cutoff)
                if candidate is not None:
                    return candidate
        except (FormationValidationError, VirtualAContractError):
            continue
    return None


def refine_corridor(request, base, checker, audit_uniform, cutoff):
    """Try a full A route with turns that remain continuous across long lanes."""
    from .uniform_continuous_corridor import corridor_candidates

    for profile, target in corridor_candidates(request, checker, cutoff):
        if time.perf_counter() >= cutoff or (request.cancelled and request.cancelled()):
            break
        try:
            tail = ()
            if target != request.goal_qf:
                headings = dict.fromkeys(request.model.vehicle_ids, target.psi)
                displacement = request.goal_qf.x - target.x, request.goal_qf.y - target.y
                reverse = a_straight_edge(
                    request.model, target, headings, displacement, dict.fromkeys(headings, -1),
                    limits=base.motion_limits, allow_reverse=request.successor_config.allow_reverse)
                tail = (reverse,)
                if request.goal_headings is not None:
                    align = _align(request, reverse.end_qf, headings, request.goal_headings,
                                   base.motion_limits)
                    if align is not None:
                        tail += (align,)
            candidate = _from_start(request, base, profile, target, tail, audit_uniform, cutoff)
            if candidate is not None:
                return candidate
        except (FormationValidationError, VirtualAContractError):
            continue
    return None


def refine_turn(request, base, audit_uniform, cutoff, *, screen_cargo=False,
                allow_slower_for_fewer_spins=True):
    """Return one certified refinement, preserving the base on any rejection."""
    spins = sum(edge.mode == "S" for edge in base.edges)
    for index, edge in enumerate(base.edges):
        if edge.primitive != PRIMITIVE_A_ARC:
            continue
        if abs(math.remainder(request.goal_qf.psi - edge.end_qf.psi, math.tau)) > 1e-4:
            continue
        if abs(math.remainder(request.start_qf.psi - edge.start_qf.psi, math.tau)) > 1e-9:
            continue
        for padding in ((0.0, 0.0), (1.25, 1.25), (0.0, 1.75), (0.75, 0.75), (2.0, 2.0)):
            if time.perf_counter() >= cutoff or (request.cancelled and request.cancelled()):
                return None
            try:
                candidate = _candidate(request, base, index, padding, audit_uniform, cutoff,
                                       screen_cargo=screen_cargo)
            except (FormationValidationError, VirtualAContractError):
                continue
            if candidate is not None:
                plan = candidate[0]
                fewer_spins = sum(edge.mode == "S" for edge in plan.edges) < spins
                if (plan.duration_s < base.duration_s
                        or allow_slower_for_fewer_spins and fewer_spins
                        and plan.duration_s <= 1.10 * base.duration_s):
                    return candidate
    return None
