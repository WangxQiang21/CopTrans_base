"""Seed-preserving per-vehicle rectangular regions for the continuous A NLP."""
from dataclasses import dataclass, replace
import math
import time

import numpy as np

from . import bounds
from .audit import audit_dynamics, audit_geometry
from .collision import (
    RELATIVE_HEADING_BUDGET_RAD, _independent_segment_bounds, _offset_h_interval,
    audit_collision,
)
from .contracts import FOOTPRINT_GUIDANCE, InitialGuess, VirtualAContractError
from .corridor import _polygon_separated_from_box
from .formation_corridor import (
    FormationCorridorConstraint, FormationCorridorHalfspace,
    FormationCorridorSample, FormationCorridorVertexRows,
)
from .optimizer import (
    _CorridorConstraint, _attempt_record, _candidate_path, _constraint_residual,
    _gauss, _numeric_objective, _result, _search_packet, _solve_with_ipopt,
    _solve_with_scipy, solve_open_path_guided,
)
from .timing import retime_path


_INSET_M = 1e-7
_MAX_REGIONS = 192
_MIN_INTERVAL_M = 0.01
_REFINEMENT_INSET_M = 0.005


def straight_a_length(request):
    start, goal = request.start_qf, request.goal_qf
    c, s = math.cos(start.psi), math.sin(start.psi)
    dx, dy = goal.x - start.x, goal.y - start.y
    forward = c * dx + s * dy
    if (forward > 0 and abs(-s * dx + c * dy) <= 1e-12
            and abs(math.remainder(goal.psi - start.psi, math.tau)) <= 1e-12):
        return forward
    return None


@dataclass(frozen=True)
class FootprintRegion:
    vehicle_id: int
    sigma_start: float
    sigma_end: float
    box: tuple
    yaw: float = 0.0

    def as_dict(self):
        return {
            "vehicle_id": self.vehicle_id,
            "sigma_start": self.sigma_start,
            "sigma_end": self.sigma_end,
            "vertices": _box_vertices(self.box, self.yaw),
        }


def _box_vertices(box, yaw):
    x0, y0, x1, y1 = box
    c, s = math.cos(yaw), math.sin(yaw)
    return tuple((c * x - s * y, s * x + c * y)
                 for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)))


def _inside(inner, outer):
    return (outer[0] <= inner[0] and outer[1] <= inner[1]
            and inner[2] <= outer[2] and inner[3] <= outer[3])


def _corners(shape, margin):
    hx, hy = (shape.length_m + margin) / 2, (shape.width_m + margin) / 2
    return ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))


def _world_corners(path, station, vehicle_id, corners):
    body = next(v for v in path.state_at_s(station).vehicles
                if v.vehicle_id == vehicle_id)
    c, s = math.cos(body.theta), math.sin(body.theta)
    return tuple((body.x + c * x - s * y, body.y + s * x + c * y)
                 for x, y in corners)


def _footprint_position_error(path, radius, offset_radius):
    return (path.position_error_bound_m
            + (offset_radius + radius) * path.angle_error_bound_rad
            + radius * RELATIVE_HEADING_BUDGET_RAD + _INSET_M)


def _projection_box(points, yaw, error):
    c, s = math.cos(yaw), math.sin(yaw)
    xs, ys = zip(*((c * x + s * y, -s * x + c * y) for x, y in points))
    return (bounds._next_down(min(xs) - error), bounds._next_down(min(ys) - error),
            bounds._next_up(max(xs) + error), bounds._next_up(max(ys) + error))


def _swept_box(path, offset, corners, a, b, yaw=0.0):
    motion = _independent_segment_bounds(path.profile, offset, a, b)
    rho = math.hypot(offset.x, offset.y)
    radius = math.hypot(*corners[0])
    # Include F integration and both orientation error contributions.
    error = _footprint_position_error(path, radius, rho)
    if all(knot.value == knot.derivative == 0.0 for knot in path.profile.knots):
        points = (_world_corners(path, a, offset.vehicle_id, corners)
                  + _world_corners(path, b, offset.vehicle_id, corners))
    else:
        points = _world_corners(path, (a + b) / 2, offset.vehicle_id, corners)
        # F moves at unit arc speed. The rigid offset and the body corner
        # additionally rotate at bounded rates with respect to F arc length.
        error += 0.5 * (motion["center_speed_bound_m_per_lambda"]
                        + rho * motion["formation_rate_bound_rad_per_lambda"]
                        + radius * motion["body_rate_bound_rad_per_lambda"])
    return _projection_box(points, yaw, error)


def _chord_swept_box(path, offset, corners, a, b, yaw):
    """Enclose corner sweeps by endpoint chords and a second derivative bound."""
    jets = path.profile.interval_bounds(a, b)
    k, eta, u = jets["k"], jets["eta"], jets["u"]
    ax, by = float(offset.x), float(offset.y)
    heading_rate = _offset_h_interval(k, eta, offset)
    heading_acceleration = eta
    if ax:
        A = bounds.isub((1.0, 1.0), bounds.iscale(k, by))
        B = bounds.iscale(k, ax)
        D = bounds.iadd(bounds.isquare(A), bounds.isquare(B))
        D_s = bounds.iscale(bounds.imul(
            bounds.isub(bounds.iscale(B, ax), bounds.iscale(A, by)), eta), 2.0)
        correction = bounds.isub(bounds.idiv_positive_interval(u, D),
                                  bounds.idiv_positive_interval(bounds.imul(eta, D_s),
                                                                bounds.isquare(D)))
        heading_acceleration = bounds.iadd(eta, bounds.iscale(correction, ax))
    # p = F + R(psi) r_i + R(theta_i) v. The norm of the second
    # derivative bounds any projection of the corner's chord remainder.
    rho = bounds._next_up(math.hypot(ax, by))
    radius = bounds._next_up(math.hypot(*corners[0]))
    formation_accel = bounds.iadd((0.0, bounds.imax_abs(eta)), bounds.isquare(k))
    corner_accel = bounds.iadd((0.0, bounds.imax_abs(heading_acceleration)),
                               bounds.isquare(heading_rate))
    acceleration = bounds.iadd((0.0, bounds.imax_abs(k)), bounds.iadd(
        bounds.iscale(formation_accel, rho), bounds.iscale(corner_accel, radius)))
    span = bounds._next_up(b - a)
    deviation = bounds.iscale(bounds.imul(acceleration, bounds.isquare((0.0, span))), 0.125)[1]
    error = bounds._next_up(_footprint_position_error(path, radius, rho) + deviation)
    points = (_world_corners(path, a, offset.vehicle_id, corners)
              + _world_corners(path, b, offset.vehicle_id, corners))
    return _projection_box(points, yaw, error)


def _grow_rectangle(grid, seed, padding_m):
    """Extend an empty rectangle to the nearest occupied raster strips."""
    x0, y0, x1, y1 = seed
    res = float(grid.res)
    ix0, ix1 = (math.floor((x - grid.ox) / res) for x in (x0, x1))
    iy0, iy1 = (math.floor((y - grid.oy) / res) for y in (y0, y1))
    if ix0 < 0 or iy0 < 0 or ix1 >= grid.width or iy1 >= grid.height:
        return None
    if grid.occ[iy0:iy1 + 1, ix0:ix1 + 1].any():
        return None
    pad = math.ceil(padding_m / res)
    left, right = max(0, ix0 - pad), min(grid.width - 1, ix1 + pad)
    columns = grid.occ[iy0:iy1 + 1, left:right + 1].any(axis=0)
    blocked_left = np.flatnonzero(columns[:ix0 - left])
    blocked_right = np.flatnonzero(columns[ix1 - left + 1:])
    ix0 = left + int(blocked_left[-1]) + 1 if len(blocked_left) else left
    ix1 = ix1 + int(blocked_right[0]) if len(blocked_right) else right
    bottom, top = max(0, iy0 - pad), min(grid.height - 1, iy1 + pad)
    rows = grid.occ[bottom:top + 1, ix0:ix1 + 1].any(axis=1)
    blocked_bottom = np.flatnonzero(rows[:iy0 - bottom])
    blocked_top = np.flatnonzero(rows[iy1 - bottom + 1:])
    iy0 = bottom + int(blocked_bottom[-1]) + 1 if len(blocked_bottom) else bottom
    iy1 = iy1 + int(blocked_top[0]) if len(blocked_top) else top
    region = (grid.ox + ix0 * res + _INSET_M, grid.oy + iy0 * res + _INSET_M,
              grid.ox + (ix1 + 1) * res - _INSET_M,
              grid.oy + (iy1 + 1) * res - _INSET_M)
    return region if _inside(seed, region) else None


def _oriented_rectangle_is_free(grid, box, yaw):
    vertices = _box_vertices(box, yaw)
    xs, ys = zip(*vertices)
    res = float(grid.res)
    ix0 = math.floor((min(xs) - _INSET_M - grid.ox) / res)
    ix1 = math.floor((max(xs) + _INSET_M - grid.ox) / res)
    iy0 = math.floor((min(ys) - _INSET_M - grid.oy) / res)
    iy1 = math.floor((max(ys) + _INSET_M - grid.oy) / res)
    if not (0 <= ix0 <= ix1 < grid.width and 0 <= iy0 <= iy1 < grid.height):
        return False
    rows, columns = np.nonzero(grid.occ[iy0:iy1 + 1, ix0:ix1 + 1])
    for row, column in zip(rows, columns):
        iy, ix = iy0 + int(row), ix0 + int(column)
        cell = (grid.ox + ix * res, grid.oy + iy * res,
                grid.ox + (ix + 1) * res, grid.oy + (iy + 1) * res)
        if not _polygon_separated_from_box(vertices, cell):
            return False
    return True


def _grow_oriented_rectangle(grid, seed, yaw, padding_m):
    if not _oriented_rectangle_is_free(grid, seed, yaw):
        return None
    box = seed
    lower, upper = 0.0, padding_m
    for _ in range(6):
        pad = (lower + upper) / 2
        candidate = (seed[0] - pad, seed[1] - pad, seed[2] + pad, seed[3] + pad)
        if _oriented_rectangle_is_free(grid, candidate, yaw):
            box, lower = candidate, pad
        else:
            upper = pad
    return box


def build_footprint_regions(path, request, grid, margin, padding_m, deadline):
    """Cover each vehicle independently. Adjacent boxes share its seed seam."""
    count = path.profile.segment_count
    length = path.length_m
    regions = []
    for offset in request.model.offsets:
        corners = _corners(request.vehicle_shapes[offset.vehicle_id], margin)
        pending = [(j / count, (j + 1) / count)
                   for j in reversed(range(count))]
        while pending:
            if time.monotonic() >= deadline:
                raise VirtualAContractError("FOOTPRINT_BUDGET", "region construction timed out")
            if len(regions) + len(pending) > _MAX_REGIONS:
                raise VirtualAContractError("FOOTPRINT_REGION_LIMIT", "adaptive cover exhausted")
            a, b = pending.pop()
            envelope = _swept_box(path, offset, corners, a * length, b * length)
            box = _grow_rectangle(grid, envelope, padding_m)
            yaw = 0.0
            if box is None:
                center = path.state_at_s((a + b) * length / 2)
                body = next(v for v in center.vehicles if v.vehicle_id == offset.vehicle_id)
                yaw = math.remainder(body.theta, math.pi)
                if yaw:
                    envelope = _swept_box(path, offset, corners, a * length, b * length, yaw)
                    box = _grow_oriented_rectangle(grid, envelope, yaw, padding_m)
            if box is not None:
                regions.append(FootprintRegion(offset.vehicle_id, a, b, box, yaw))
                continue
            if (b - a) * length <= _MIN_INTERVAL_M:
                raise VirtualAContractError(
                    "FOOTPRINT_BOX_UNCONFIRMED", "no empty rectangle contains the envelope",
                    vehicle_id=offset.vehicle_id, sigma=(a, b))
            mid = (a + b) / 2
            pending.extend(((mid, b), (a, mid)))
    return tuple(regions)


def _refinement_limits(path, regions, request, margin):
    """Give each solver face clearance without excluding the seed sweep."""
    offsets = {o.vehicle_id: o for o in request.model.offsets}
    limits = []
    for region in regions:
        offset = offsets[region.vehicle_id]
        corners = _corners(request.vehicle_shapes[region.vehicle_id], margin)
        a = region.sigma_start * path.length_m
        b = region.sigma_end * path.length_m
        middle = (a + b) / 2
        original = _swept_box(path, offset, corners, a, b, region.yaw)
        left = _chord_swept_box(path, offset, corners, a, middle, region.yaw)
        right = _chord_swept_box(path, offset, corners, middle, b, region.yaw)
        # The intersection of the two enclosures still contains the seed.
        envelope = (max(original[0], min(left[0], right[0])),
                    max(original[1], min(left[1], right[1])),
                    min(original[2], max(left[2], right[2])),
                    min(original[3], max(left[3], right[3])))
        limits.append(tuple(face + direction * min(_REFINEMENT_INSET_M,
                                                    max(0.0, direction * (edge - face) / 2))
                            for face, edge, direction in zip(region.box, envelope, (1, 1, -1, -1))))
    return tuple(limits)


def _region_rows(regions, request, config, margin, refinement_limits=None):
    """Reuse the solver's complete-OBB row evaluator with whole rectangles."""
    n, qcount = int(config.intervals), int(config.corridor_quadrature_points)
    groups = [[] for _ in range(n)]
    counts = {o.vehicle_id: 0 for o in request.model.offsets}
    offsets = {o.vehicle_id: (i, o) for i, o in enumerate(request.model.offsets)}
    region_counts = []
    start = request.start_qf
    c, s = math.cos(start.psi), math.sin(start.psi)
    if refinement_limits is None:
        refinement_limits = tuple(region.box for region in regions)
    for index, (region, box) in enumerate(zip(regions, refinement_limits)):
        x0, y0, x1, y1 = box
        rc, rs = math.cos(region.yaw), math.sin(region.yaw)
        normals = ((-rc, -rs), (rs, -rc), (rc, rs), (-rs, rc))
        halfspaces = tuple(
            FormationCorridorHalfspace(j, index, (nx*c + ny*s, -nx*s + ny*c),
                                       limit - nx*start.x - ny*start.y)
            for j, ((nx, ny), limit) in enumerate(zip(normals, (-x0, -y0, x1, y1))))
        vehicle_index, offset = offsets[region.vehicle_id]
        vertices = tuple(FormationCorridorVertexRows(j, corner, halfspaces)
                         for j, corner in enumerate(_corners(
                             request.vehicle_shapes[region.vehicle_id], margin)))
        left, right = region.sigma_start, region.sigma_end
        # Each region is built inside one knot interval, with both endpoints
        # constrained so adjacent regions share the same vehicle state. Keep
        # node density proportional to span when the cover splits a piece.
        count = max(1, math.ceil(qcount * min(1.0, (right - left) * n)))
        nodes = (0.0, *map(float, _gauss(count)[0]), 1.0)
        piece = int((left + right) * n / 2)
        for node in nodes:
            sigma = left + (right - left) * node
            q = min(1.0, max(0.0, sigma * n - piece))
            groups[piece].append(FormationCorridorSample(
                piece, q, sigma, FOOTPRINT_GUIDANCE, index, region.vehicle_id,
                vehicle_index, offset.x, offset.y, "whole", 0, index, vertices))
        rows = len(nodes) * len(vertices) * len(halfspaces)
        counts[region.vehicle_id] += rows
        region_counts.append(rows)
    empty = ((),) * n
    formation = FormationCorridorConstraint(
        template_id="", margin_m=margin, margin_source="seed_sweep_margin",
        quadrature_points=qcount, groups=tuple(map(tuple, groups)),
        sample_count=sum(map(len, groups)), scalar_row_count=sum(region_counts),
        rows_by_vehicle=tuple(sorted(counts.items())), rows_by_vehicle_region=tuple(
            (region.vehicle_id, j, rows)
            for j, (region, rows) in enumerate(zip(regions, region_counts))),
        segment_row_counts=tuple(region_counts), segment_count=len(regions),
        scope=FOOTPRINT_GUIDANCE, f_groups=empty)
    return _CorridorConstraint(
        0.0, 0.0, (), empty, 0, len(regions), None, qcount, "hard", formation)


def _cover_passes(path, regions, request, margin, deadline):
    offsets = {o.vehicle_id: o for o in request.model.offsets}
    checked = 0
    length = path.length_m
    for region in regions:
        offset = offsets[region.vehicle_id]
        corners = _corners(request.vehicle_shapes[region.vehicle_id], margin)
        pending = [(region.sigma_start * length, region.sigma_end * length)]
        while pending:
            if time.monotonic() >= deadline or checked >= 4096:
                return False
            a, b = pending.pop()
            checked += 1
            if _inside(_chord_swept_box(path, offset, corners, a, b, region.yaw), region.box):
                continue
            if b - a <= _MIN_INTERVAL_M:
                return False
            mid = (a + b) / 2
            pending.extend(((mid, b), (a, mid)))
    return True


def _audit_refined_motion(candidate, map_request, dynamic_config, regions, margin, deadline):
    request = map_request.request
    budget = "FOOTPRINT_REFINEMENT_BUDGET"
    if not _cover_passes(candidate, regions, request, margin, deadline):
        return budget if time.monotonic() >= deadline else "FOOTPRINT_REFINEMENT_COVER_UNCONFIRMED"
    context = map_request.context()
    timed = retime_path(request, candidate, map_request.timing_config, planning_context=context)
    if timed.plan is None:
        return timed.reason or "FOOTPRINT_REFINEMENT_TIMING"
    if time.monotonic() >= deadline:
        return budget
    dynamic = audit_dynamics(request, timed.plan, dynamic_config,
                             expected_planning_context=context)
    if not dynamic.passed:
        return dynamic.reason or "FOOTPRINT_REFINEMENT_DYNAMICS"
    if time.monotonic() >= deadline:
        return budget
    collision = audit_collision(request, timed.plan, map_request.grid, map_request.collision_config)
    if not collision.passed:
        return collision.reason or "FOOTPRINT_REFINEMENT_COLLISION"
    return budget if time.monotonic() >= deadline else ""


def solve_footprint_seed(map_request, config, *, routes, obstacle_context,
                         audit_config, dynamic_config, heading_branch,
                         guide_length_m, corridor_chain, corridor_allocation,
                         allow_straight, excluded_initial_keys,
                         initial_limit, evidence, on_regions):
    """Find and check a full-model seed, then refine it inside its cover."""
    started = time.monotonic()
    deadline = started + config.max_seconds
    request, grid = map_request.request, map_request.grid
    n = int(config.intervals)
    try:
        packet = _search_packet(request, config, heading_branch, guide_length_m)
    except VirtualAContractError as exc:
        evidence.update(seed_status=exc.reason, detail=exc.detail)
        return _result("NO_ACCEPTED_CANDIDATE", request=request, config=config,
                       reason=exc.reason, detail=exc.detail,
                       wall_time_s=time.monotonic() - started)
    attempts = ()
    straight = straight_a_length(request)
    analytic_straight = allow_straight and straight is not None
    if analytic_straight:
        zeros = (0.0,) * (n + 1)
        guess = InitialGuess("straight_a", "straight_a", straight, zeros, zeros, 1.0, 0)
        seed = _candidate_path(request, guess, {"z": np.asarray(guess.vector)})
        seed_audit = audit_geometry(request, seed, audit_config)
        evidence["seed_source"] = "straight_a"
    else:
        routed = corridor_chain is not None
        source = "reference_corridor_connector" if routed else "continuous_a_connector"
        evidence["seed_source"] = source
        seed_result = solve_open_path_guided(
            request, replace(
                config, guidance_mode="corridor" if routed else "legacy_route",
                max_seconds=config.max_seconds if routed else config.max_seconds * 0.55),
            route_guides=routes, obstacle_context=obstacle_context,
            geometry_audit_config=audit_config, heading_branch=heading_branch,
            guide_length_m=guide_length_m,
            corridor_chain=corridor_chain, corridor_allocation=corridor_allocation,
            excluded_initial_keys=excluded_initial_keys,
            max_new_initials=min(3, initial_limit),
            stop_after_route_audit=bool(routes))
        attempts = seed_result.attempts
        if seed_result.path is None or not seed_result.audit.passed:
            evidence["seed_status"] = seed_result.reason
            return seed_result
        seed, seed_audit = seed_result.path, seed_result.audit
        knots = seed.profile.knots
        guess = InitialGuess(
            "footprint_seed", source, seed.length_m,
            tuple(k.value for k in knots), tuple(k.derivative for k in knots), 1.0, 0)

    def reject(reason, detail):
        evidence.update(seed_status=reason, detail=detail)
        return _result(
            "BUDGET_EXHAUSTED" if reason == "FOOTPRINT_BUDGET" else
            "NO_ACCEPTED_CANDIDATE", request=request, config=config,
            reason=reason, detail=detail, attempts=attempts, audit=seed_audit,
            wall_time_s=time.monotonic() - started)

    if not seed_audit.passed:
        return reject(seed_audit.reason, seed_audit.detail)
    if time.monotonic() >= deadline:
        return reject("FOOTPRINT_BUDGET", "seed budget ended before sweep check")
    seed_timing = retime_path(request, seed, map_request.timing_config,
                             planning_context=map_request.context())
    if seed_timing.plan is None:
        return reject(seed_timing.reason, seed_timing.detail)
    seed_collision = audit_collision(request, seed_timing.plan, grid,
                                     map_request.collision_config)
    if not seed_collision.passed:
        return reject("FOOTPRINT_SEED_SWEEP_REJECTED",
                      seed_collision.reason + ": " + seed_collision.detail)
    margin = float(map_request.collision_config.baseline_margin_m)
    refine_config = replace(config, guidance_mode="corridor")
    try:
        regions = build_footprint_regions(
            seed, request, grid, margin, float(config.corridor_box_padding_m), deadline)
        refinement_limits = (_refinement_limits(seed, regions, request, margin)
                             if corridor_chain is not None else None)
        constraint = _region_rows(regions, request, config, margin, refinement_limits)
        seed_residual = _constraint_residual(
            guess.vector, request, refine_config, packet, corridor_constraint=constraint)
    except VirtualAContractError as exc:
        return reject(exc.reason, exc.detail)
    if not math.isfinite(seed_residual) or seed_residual > 1e-6:
        return reject("FOOTPRINT_SEED_NOT_REPRESENTABLE",
                      "seed residual exceeds tolerance: {!r}".format(seed_residual))
    evidence.update(
        seed_status="SWEEP_PASSED", seed_length_m=seed.length_m, margin_m=margin,
        region_count=len(regions), obb_rows=constraint.formation_obb_constraint_count,
        oriented_region_count=sum(region.yaw != 0.0 for region in regions),
        seed_nlp_residual=seed_residual,
        regions=[region.as_dict() for region in regions])
    if refinement_limits is not None:
        evidence["refinement_inset_cap_m"] = _REFINEMENT_INSET_M
    on_regions(seed)

    # The already checked geometry is a fallback if refinement fails. The
    # caller subjects either chosen model to the normal final timing/audits.
    result_path, result_audit, solver_used = seed, seed_audit, "SEED_RETAINED"
    seed_objective = _numeric_objective(
        guess.vector, request, refine_config, packet, config.collocation_points)
    evidence.update(seed_objective=seed_objective, refinement="BUDGET", selected="seed")
    remaining = deadline - time.monotonic()
    if analytic_straight:
        # The endpoint distance is a lower bound on S and every other
        # objective term is nonnegative. The straight seed meets that bound.
        evidence["refinement"] = "ANALYTIC_STRAIGHT_OPTIMUM"
        solver_used = "ANALYTIC"
    elif remaining > 0.1:
        solve = (_solve_with_scipy if str(config.solver).lower() in ("scipy", "slsqp")
                 else _solve_with_ipopt)
        try:
            solved = solve(request, refine_config, packet, guess,
                           wall_limit_s=remaining, corridor_constraint=constraint)
            evidence["refinement"] = str(solved["solver_status"])
            # A bounded refinement may reach its iteration limit with a
            # usable iterate. It must earn the same independent checks.
            has_candidate = solved["success"] or (
                solved["solver_status"] == "Maximum_Iterations_Exceeded")
            if has_candidate and time.monotonic() < deadline:
                residual = _constraint_residual(
                    solved["z"], request, refine_config, packet,
                    corridor_constraint=constraint)
                candidate = _candidate_path(request, guess, solved)
                checked = audit_geometry(request, candidate, audit_config)
                evidence["refined_objective"] = solved["objective"]
                evidence["refinement_residual"] = residual
                evidence["refined_length_m"] = candidate.length_m
                if not checked.passed:
                    rejection = checked.reason or "FOOTPRINT_REFINEMENT_GEOMETRY"
                elif not math.isfinite(residual) or residual > 1e-6:
                    rejection = "FOOTPRINT_REFINEMENT_RESIDUAL"
                elif not math.isfinite(solved["objective"]) or solved["objective"] >= seed_objective - 1e-9:
                    rejection = "FOOTPRINT_REFINEMENT_NO_IMPROVEMENT"
                else:
                    rejection = _audit_refined_motion(
                        candidate, map_request, dynamic_config, regions, margin, deadline)
                if rejection:
                    evidence["refinement_rejection"] = rejection
                else:
                    result_path, result_audit = candidate, checked
                    solver_used = solved["solver"]
                    evidence["selected"] = "refined"
                attempts += (_attempt_record(
                    len(attempts), guess, solved, checked, rejection,
                    "REJECTED" if rejection else "ACCEPTED"),)
            else:
                evidence["refinement_rejection"] = (
                    "FOOTPRINT_REFINEMENT_BUDGET" if time.monotonic() >= deadline
                    else "FOOTPRINT_REFINEMENT_NO_ITERATE")
        except (VirtualAContractError, RuntimeError, ImportError, ValueError) as exc:
            evidence["refinement_rejection"] = getattr(exc, "reason", type(exc).__name__)
            evidence["refinement_error"] = str(exc)
    return _result(
        "GEOMETRY_FEASIBLE", request=request, path=result_path, audit=result_audit,
        attempts=attempts, wall_time_s=time.monotonic() - started,
        request_sha256=seed.request_sha256, config=config, solver_used=solver_used,
        formation_obb_constraint_count=constraint.formation_obb_constraint_count)
