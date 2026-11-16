"""G4 finite-map planning pipeline for A-only rigid formations.

The pipeline is deliberately one-way: a physical request plus an authoritative
map config enters; only a complete geometry/dynamics/collision certificate may
leave as ``SUCCESS`` or ``NO_MOTION``.  A guide route is a topology hint, never
an executable polyline.  The final authority is always the old analytic
three-vehicle swept checker bound to the same planning context as the timed
model.
"""
from dataclasses import replace
import heapq
import math
import time

import numpy as np

from .audit import audit_dynamics
from .collision import audit_collision
from .contracts import (
    COLLISION_SCOPE,
    DYNAMICS_SCOPE,
    CachedClearanceField,
    DynamicAuditConfig,
    GuideRoute,
    ObstacleContext,
    RouteGuideConfig,
    VirtualAContractError,
    VirtualAMapRequest,
    VirtualAResult,
    grid_signature,
    grid_summary,
    planning_context_sha256,
    validate_collision_config,
    validate_dynamic_audit_config,
    validate_planner_config,
    validate_route_guide_config,
    validate_timing_config,
    validate_virtual_a_request,
)
from .optimizer import solve_open_path_guided
from .timing import retime_path


G4_TOTAL_SOFT_BUDGET_S = 60.0
MAX_COLLISION_RETRIES = 2
G4_ROUTE_CLEARANCE_WEIGHT = 2.0
ROUTE_NEIGHBOURS = (
    (-1, -1, math.sqrt(2.0)), (-1, 0, 1.0), (-1, 1, math.sqrt(2.0)),
    (0, -1, 1.0), (0, 1, 1.0),
    (1, -1, math.sqrt(2.0)), (1, 0, 1.0), (1, 1, math.sqrt(2.0)),
)


# ---------------------------------------------------------------------------
# Route guidance: finite 8-neighbour A* on the authoritative grid
# ---------------------------------------------------------------------------

def _clearance_field(grid):
    from scipy.ndimage import distance_transform_edt
    occupied = np.asarray(grid.occ, dtype=bool)
    free = ~occupied
    obstacle_distance = (
        distance_transform_edt(free) * float(grid.res)
        if free.any() else np.zeros_like(free, dtype=float))
    height, width = occupied.shape
    xs = grid.ox + (np.arange(width) + 0.5) * grid.res
    ys = grid.oy + (np.arange(height) + 0.5) * grid.res
    right = grid.ox + width * grid.res
    top = grid.oy + height * grid.res
    boundary = np.minimum(
        np.minimum(xs[None, :] - grid.ox, right - xs[None, :]),
        np.minimum(ys[:, None] - grid.oy, top - ys[:, None]))
    clearance = np.minimum(obstacle_distance, boundary)
    clearance[~free] = 0.0
    return clearance


CLEARANCE_FIELD_DOWNSAMPLE = 4


def _cached_clearance_field(grid, downsample=CLEARANCE_FIELD_DOWNSAMPLE):
    """Freeze a coarse clearance field for the NLP body-footprint proxy.

    The exact EDT is computed once from the authoritative grid (including the
    map boundary), then sampled on a coarser cell-centre lattice.  The NLP may
    query this cache with an OBB support radius at actual per-vehicle headings;
    the cache never replaces or weakens the real sweep.
    """
    clearance = _clearance_field(grid)
    height, width = clearance.shape
    factor = max(1, int(downsample))
    while factor > 1 and (height // factor < 2 or width // factor < 2):
        factor //= 2
    sampled = np.asarray(clearance[::factor, ::factor], dtype=float)
    values = tuple(tuple(float(value) for value in row) for row in sampled)
    return CachedClearanceField(
        origin_x=float(grid.ox + 0.5 * grid.res),
        origin_y=float(grid.oy + 0.5 * grid.res),
        resolution_m=float(grid.res * factor),
        grid_sha256=grid_signature(grid),
        values=values,
        downsample=int(factor))


def _cell_of(grid, x, y):
    ix = int(math.floor((float(x) - grid.ox) / grid.res))
    iy = int(math.floor((float(y) - grid.oy) / grid.res))
    if iy < 0 or ix < 0 or iy >= grid.occ.shape[0] \
            or ix >= grid.occ.shape[1]:
        return None
    return iy, ix


def _allowed_mask(grid, clearance, blocked_radius, blocked_points):
    allowed = np.asarray(grid.occ, dtype=bool) == False  # noqa: E712
    allowed &= np.asarray(clearance, dtype=float) >= float(blocked_radius)
    for x, y in blocked_points:
        iy, ix = _cell_of(grid, x, y)
        if iy is None:
            continue
        radius_cells = int(math.ceil(float(blocked_radius) / grid.res))
        y0, y1 = max(0, iy - radius_cells), min(
            grid.occ.shape[0], iy + radius_cells + 1)
        x0, x1 = max(0, ix - radius_cells), min(
            grid.occ.shape[1], ix + radius_cells + 1)
        gy, gx = np.ogrid[y0:y1, x0:x1]
        cx = grid.ox + (gx + 0.5) * grid.res
        cy = grid.oy + (gy + 0.5) * grid.res
        disk = (cx - float(x)) ** 2 + (cy - float(y)) ** 2 <= \
            float(blocked_radius) ** 2
        allowed[y0:y1, x0:x1] &= ~disk
    return allowed


def _astar_route(grid, allowed, cost_field, start, goal, config,
                 deadline):
    start_cell = _cell_of(grid, start[0], start[1])
    goal_cell = _cell_of(grid, goal[0], goal[1])
    if start_cell is None or goal_cell is None:
        return None
    if not allowed[start_cell] or not allowed[goal_cell]:
        return None
    height, width = allowed.shape
    start_ix = start_cell[1], start_cell[0]
    goal_ix = goal_cell[1], goal_cell[0]
    open_heap = []
    tie = 0
    heapq.heappush(open_heap, (
        math.hypot(goal[0] - start[0], goal[1] - start[1]),
        0.0, tie, start_cell, None))
    g_score = {start_cell: 0.0}
    closed = set()
    parent = {}
    expansions = 0
    maximum = int(config.max_expansions)
    while open_heap:
        if expansions >= maximum:
            return None
        if time.monotonic() >= deadline:
            return None
        _, g, _, cell, predecessor = heapq.heappop(open_heap)
        if cell in closed:
            continue
        closed.add(cell)
        parent[cell] = predecessor
        if cell == goal_cell:
            path = []
            cursor = cell
            while cursor is not None:
                path.append(cursor)
                cursor = parent.get(cursor)
            path.reverse()
            return path
        expansions += 1
        iy, ix = cell
        for dy, dx, step in ROUTE_NEIGHBOURS:
            jy, jx = iy + dy, ix + dx
            if jy < 0 or jx < 0 or jy >= height or jx >= width:
                continue
            if not allowed[jy, jx]:
                continue
            if dx and dy and (
                    not allowed[iy, jx] or not allowed[jy, ix]):
                continue
            neighbour = (jy, jx)
            if neighbour in closed:
                continue
            step_cost = step * (1.0 + float(config.clearance_weight) * (
                max(0.0, float(config.clearance_cap_m) - float(
                    cost_field[jy, jx])) / max(
                        float(config.clearance_cap_m), 1e-12)))
            candidate = g + step_cost
            if candidate >= g_score.get(neighbour, math.inf):
                continue
            g_score[neighbour] = candidate
            h = math.hypot(
                goal[0] - (grid.ox + (jx + 0.5) * grid.res),
                goal[1] - (grid.oy + (jy + 0.5) * grid.res))
            tie += 1
            heapq.heappush(open_heap, (
                candidate + h, candidate, tie, neighbour, cell))
    return None


def _route_points_from_cells(grid, cells, start, goal):
    points = []
    for iy, ix in cells:
        point = (grid.ox + (ix + 0.5) * grid.res,
                 grid.oy + (iy + 0.5) * grid.res)
        if not points or math.hypot(
                point[0] - points[-1][0], point[1] - points[-1][1]) > 1e-12:
            points.append(point)
    exact_start = (float(start[0]), float(start[1]))
    exact_goal = (float(goal[0]), float(goal[1]))
    if not points or math.hypot(
            points[0][0] - exact_start[0],
            points[0][1] - exact_start[1]) > 1e-12:
        points.insert(0, exact_start)
    if math.hypot(points[-1][0] - exact_goal[0],
                  points[-1][1] - exact_goal[1]) > 1e-12:
        points.append(exact_goal)
    return tuple(points)


def make_route_guides(grid, start_qf, goal_qf, config=None, *,
                      blocked_points=()):
    """Return up to ``max_routes`` finite A* F-point guides.

    The search uses the authoritative occupancy grid, a non-negative EDT
    clearance cost and deterministic eight-neighbour moves.  It is a topology
    hint only: it contains no vehicle path, curvature reference or known-good
    solution.
    """
    if config is None:
        config = RouteGuideConfig()
    validation = validate_route_guide_config(config)
    if not validation.passed:
        raise VirtualAContractError(
            validation.reason, validation.detail, **validation.location)
    start = (float(start_qf.x), float(start_qf.y))
    goal = (float(goal_qf.x), float(goal_qf.y))
    digest = grid_signature(grid)
    clearance = _clearance_field(grid)
    blocked = tuple((float(x), float(y)) for x, y in blocked_points)
    allowed = _allowed_mask(
        grid, clearance, config.blocked_radius_m, blocked)
    base_cost = clearance
    penalty = np.zeros_like(clearance, dtype=float)
    deadline = time.monotonic() + max(0.0, float(config.max_seconds))
    routes = []
    seen = set()
    for index in range(int(config.max_routes)):
        if time.monotonic() >= deadline:
            break
        route_penalty = 1.0 + penalty
        cost_field = base_cost / np.maximum(route_penalty, 1.0)
        cells = _astar_route(
            grid, allowed, cost_field, start, goal, config, deadline)
        if cells is None:
            break
        signature = tuple(cells)
        if signature in seen:
            break
        seen.add(signature)
        points = _route_points_from_cells(grid, cells, start, goal)
        routes.append(GuideRoute(
            points=points,
            source="astar_primary" if index == 0 else
            "astar_alternative_{}".format(index),
            grid_sha256=digest))
        # Penalise a small band around the previous guide so the next search
        # explores a genuinely different topology.  This is not an executable
        # path and never enters the final certificate.
        for iy, ix in cells:
            y0, y1 = max(0, iy - 2), min(
                grid.occ.shape[0], iy + 3)
            x0, x1 = max(0, ix - 2), min(
                grid.occ.shape[1], ix + 3)
            penalty[y0:y1, x0:x1] += float(config.alternative_penalty)
    return tuple(routes)


# ---------------------------------------------------------------------------
# Result assembly
# ---------------------------------------------------------------------------

def _empty_result(map_request, status, reason, detail, first_layer,
                  timings, wall, search, checks=0, metrics=None,
                  attempts=(), routes=None, geometry_audit=None,
                  dynamic_audit=None, collision_audit=None):
    """Build a failed result with all reached evidence, but no plan/path.

    The contract forbids publishing a candidate from a failed result.  Audit
    reports and rejected attempts are retained because they are evidence about
    why the pipeline stopped, not executable outputs.
    """
    return VirtualAResult(
        status=status,
        reason=reason,
        detail=detail,
        first_failure_layer=first_layer,
        request_sha256=getattr(map_request, "request_sha256", ""),
        planning_context_sha256=_safe_context_sha(map_request),
        grid_sha256=getattr(map_request, "grid_sha256", ""),
        grid_summary=_safe_grid_summary(map_request),
        geometry_audit=geometry_audit,
        dynamic_audit=dynamic_audit,
        collision_audit=collision_audit,
        attempts=tuple(attempts),
        routes=tuple(search.get("routes", ()))
        if routes is None else tuple(routes),
        search=search,
        wall_time_s=float(wall),
        timings=timings,
        metrics=dict(metrics or {}),
        checks=int(checks))


def _safe_context_sha(map_request):
    try:
        return planning_context_sha256(map_request.context())
    except Exception:
        return ""


def _safe_grid_summary(map_request):
    try:
        return grid_summary(map_request.grid)
    except Exception:
        return {}


def _failure_layer_for_timing(status):
    if status in ("GEOMETRY_REJECTED", "INVALID_PATH"):
        return "GEOMETRY"
    if status == "INVALID_REQUEST":
        return "REQUEST"
    return "DYNAMICS"


def _timing_status_for_geometry(status):
    return {
        "INVALID_REQUEST": "INVALID_REQUEST",
        "BUDGET_EXHAUSTED": "BUDGET_EXHAUSTED",
        "NUMERICAL_FAILURE": "NUMERICAL_FAILURE",
    }.get(status, "NO_ACCEPTED_CANDIDATE")


def _collision_retry_context(report, plan, grid_sha256, route, intervals,
                            retry_index=1):
    location = dict(getattr(report, "location", {}) or {})
    qf = location.get("qf")
    blocked = []
    station = location.get("station")
    if isinstance(qf, (list, tuple)) and len(qf) >= 2:
        blocked.append((float(qf[0]), float(qf[1])))
    elif station is not None:
        try:
            blocked.append((float(plan.path.state_at_s(float(station)).qf.x),
                            float(plan.path.state_at_s(float(station)).qf.y)))
        except Exception:
            pass
    targets = []
    if route is not None and blocked and station is not None:
        try:
            length = float(plan.path.length_m)
            fraction = min(1.0, max(0.0, float(station) / length))
            index = int(round(fraction * int(intervals)))
            route_length = sum(math.hypot(
                b[0] - a[0], b[1] - a[1])
                for a, b in zip(route.points, route.points[1:]))
            target_distance = fraction * route_length
            cursor = 0.0
            anchor = route.points[0]
            for a, b in zip(route.points, route.points[1:]):
                segment = math.hypot(b[0] - a[0], b[1] - a[1])
                if cursor + segment >= target_distance or segment <= 0.0:
                    ratio = (target_distance - cursor) / segment \
                        if segment > 0.0 else 0.0
                    anchor = (a[0] + ratio * (b[0] - a[0]),
                              a[1] + ratio * (b[1] - a[1]))
                    break
                cursor += segment
                anchor = b
            bx, by = blocked[0]
            vx, vy = anchor[0] - bx, anchor[1] - by
            norm = math.hypot(vx, vy)
            if norm <= 1e-12:
                vx, vy, norm = 1.0, 0.0, 1.0
            push = 0.20
            targets.append({
                "index": max(0, min(int(intervals), index)),
                "x": float(anchor[0] + push * vx / norm),
                "y": float(anchor[1] + push * vy / norm),
                "radius_m": float(push),
            })
        except Exception:
            targets = []
    geometric_target = bool(blocked) and station is not None
    relaxed = (not geometric_target) or int(retry_index) >= 2
    return ObstacleContext(
        grid_sha256=grid_sha256,
        node_targets=tuple(targets),
        source="collision_audit_retry_relaxed_guidance" if relaxed
        else "collision_audit_retry",
        collision_station=float(station) if station is not None else None,
        collision_reason=str(getattr(report, "reason", "") or ""),
        blocked_points=tuple(blocked),
        route_weight_scale=0.02 if relaxed else 1.0)


def _report_checks(report):
    return int(getattr(report, "check_count", 0) or 0)


# ---------------------------------------------------------------------------
# G4 end-to-end pipeline
# ---------------------------------------------------------------------------

def plan_virtual_a(map_request):
    """Autonomously solve, retime, audit and certify one map request.

    No reference profile, known-good trajectory, scene manifest or plan ID is
    read.  Only ``VirtualAMapRequest`` fields are used.
    """
    started = time.monotonic()
    timings = {
        "route_generation_s": 0.0,
        "geometry_solve_s": 0.0,
        "retime_s": 0.0,
        "dynamics_audit_s": 0.0,
        "collision_audit_s": 0.0,
        "result_assembly_s": 0.0,
    }
    search = {"routes": (), "rounds": [], "budget": {}}

    if not isinstance(map_request, VirtualAMapRequest):
        return _empty_result(
            map_request, "INVALID_REQUEST", "INVALID_MAP_REQUEST",
            "expected VirtualAMapRequest",
            "REQUEST", timings, time.monotonic() - started, search)
    request = map_request.request
    grid = map_request.grid
    planner_config = map_request.planner_config
    timing_config = map_request.timing_config
    collision_config = map_request.collision_config
    for validator, config in (
            (validate_virtual_a_request, request),
            (validate_planner_config, planner_config),
            (validate_timing_config, timing_config),
            (validate_collision_config, collision_config)):
        validation = validator(config)
        if not validation.passed:
            return _empty_result(
                map_request, "INVALID_REQUEST", validation.reason,
                validation.detail, "REQUEST", timings,
                time.monotonic() - started, search)
    try:
        context = map_request.context()
        context_sha = planning_context_sha256(context)
        digest = grid_signature(grid)
    except Exception as exc:
        return _empty_result(
            map_request, "INVALID_REQUEST", "INVALID_PLANNING_CONTEXT",
            "{}: {}".format(type(exc).__name__, exc), "REQUEST", timings,
            time.monotonic() - started, search)
    try:
        dynamic_config = DynamicAuditConfig(
            geometry_audit_config=timing_config.geometry_audit_config)
        dynamic_validation = validate_dynamic_audit_config(dynamic_config)
        if not dynamic_validation.passed:
            raise VirtualAContractError(
                dynamic_validation.reason, dynamic_validation.detail,
                **dynamic_validation.location)
    except Exception as exc:
        return _empty_result(
            map_request, "INVALID_REQUEST", "INVALID_AUDIT_CONFIG",
            "{}: {}".format(type(exc).__name__, exc), "REQUEST", timings,
            time.monotonic() - started, search)
    audit_config = timing_config.geometry_audit_config
    # The solver certifies an exact identical start/goal with a hold path and
    # never consults route guides.  A route search must not pre-empt the
    # mandatory static map/vehicle check for such a request: the hold audit has
    # to run even when the start cell is blocked or has no allowed guide.
    hold_request = request.start_qf == request.goal_qf
    search["budget"] = {
        "total_soft_budget_s": G4_TOTAL_SOFT_BUDGET_S,
        "max_collision_retries": MAX_COLLISION_RETRIES,
        "planner_config_max_seconds": float(planner_config.max_seconds),
        "route_config": replace(
            RouteGuideConfig(),
            clearance_weight=G4_ROUTE_CLEARANCE_WEIGHT).as_dict(),
        "hard_real_time_guarantee": False,
    }

    def remaining():
        return max(0.0, G4_TOTAL_SOFT_BUDGET_S - (
            time.monotonic() - started))

    # A zero-length request still needs the complete static/hold certificate.
    try:
        geometry_config = replace(
            planner_config,
            max_seconds=min(float(planner_config.max_seconds), remaining()))
    except Exception:
        geometry_config = planner_config

    routes = ()
    obstacle_context = None
    clearance_field = None
    collision_retries = 0
    all_attempts = []
    last_geometry_audit = None
    last_dynamic_audit = None
    last_collision_audit = None
    while True:
        round_started = time.monotonic()
        if remaining() <= 0.0:
            return _empty_result(
                map_request, "BUDGET_EXHAUSTED", "SOFT_BUDGET_EXHAUSTED",
                "no route/solve budget remained", "PLANNING", timings,
                time.monotonic() - started, search,
                attempts=all_attempts, routes=tuple(routes),
                geometry_audit=last_geometry_audit,
                dynamic_audit=last_dynamic_audit,
                collision_audit=last_collision_audit)
        if not routes and not hold_request:
            route_started = time.monotonic()
            if clearance_field is None:
                try:
                    clearance_field = _cached_clearance_field(grid)
                except Exception as exc:
                    return _empty_result(
                        map_request, "INVALID_REQUEST",
                        "CLEARANCE_FIELD_FAILED",
                        "{}: {}".format(type(exc).__name__, exc),
                        "PLANNING", timings, time.monotonic() - started,
                        search)
            route_config = replace(
                RouteGuideConfig(),
                clearance_weight=G4_ROUTE_CLEARANCE_WEIGHT,
                max_seconds=min(8.0, max(0.001, remaining())))
            try:
                routes = make_route_guides(
                    grid, request.start_qf, request.goal_qf, route_config,
                    blocked_points=tuple(
                        getattr(obstacle_context, "blocked_points", ())))
            except Exception as exc:
                return _empty_result(
                    map_request, "INVALID_REQUEST", "ROUTE_GENERATION_FAILED",
                    "{}: {}".format(type(exc).__name__, exc), "PLANNING",
                    timings, time.monotonic() - started, search,
                    attempts=all_attempts,
                    geometry_audit=last_geometry_audit,
                    dynamic_audit=last_dynamic_audit,
                    collision_audit=last_collision_audit)
            timings["route_generation_s"] += time.monotonic() - route_started
            if obstacle_context is None:
                obstacle_context = ObstacleContext(
                    grid_sha256=digest,
                    source="planning_clearance_field",
                    clearance_field=clearance_field)
            else:
                obstacle_context = replace(
                    obstacle_context, clearance_field=clearance_field)
            search["routes"] = tuple({
                "source": route.source,
                "point_count": len(route.points),
                "length_m": route.length_m,
                "start": route.start,
                "goal": route.goal,
                "grid_sha256": route.grid_sha256,
            } for route in routes)
            if not routes:
                return _empty_result(
                    map_request, "NO_ACCEPTED_CANDIDATE",
                    "NO_ROUTE_GUIDE",
                    "A* found no finite guide on the authoritative grid",
                    "PLANNING", timings, time.monotonic() - started, search,
                    attempts=all_attempts,
                    geometry_audit=last_geometry_audit,
                    dynamic_audit=last_dynamic_audit,
                    collision_audit=last_collision_audit)
        if not isinstance(geometry_config, type(planner_config)):
            geometry_config = planner_config
        geometry_config = replace(
            planner_config,
            max_seconds=min(float(planner_config.max_seconds), remaining()))
        solve_started = time.monotonic()
        geometry = solve_open_path_guided(
            request, geometry_config, route_guides=routes,
            obstacle_context=obstacle_context,
            geometry_audit_config=audit_config)
        timings["geometry_solve_s"] += time.monotonic() - solve_started
        all_attempts.extend(geometry.attempts)
        last_geometry_audit = geometry.audit
        round_record = {
            "round_index": len(search["rounds"]),
            "geometry_status": geometry.status,
            "geometry_reason": geometry.reason,
            "geometry_attempts": len(geometry.attempts),
            "route_count": len(routes),
            "obstacle_context": obstacle_context.as_dict()
            if obstacle_context is not None else None,
        }
        if geometry.status not in ("GEOMETRY_FEASIBLE", "NO_MOTION"):
            search["rounds"].append(round_record)
            status = _timing_status_for_geometry(geometry.status)
            return _empty_result(
                map_request, status,
                geometry.reason or "NO_ACCEPTED_CANDIDATE",
                geometry.detail or "geometry solver produced no audited plan",
                "GEOMETRY", timings, time.monotonic() - started, search,
                checks=_report_checks(geometry.audit),
                attempts=all_attempts, routes=tuple(routes),
                geometry_audit=geometry.audit)
        path = geometry.path
        retime_started = time.monotonic()
        timing = retime_path(
            request, path, timing_config, planning_context=context)
        timings["retime_s"] += time.monotonic() - retime_started
        round_record["timing_status"] = timing.status
        round_record["timing_reason"] = timing.reason
        round_record["duration_s"] = timing.T_s
        last_geometry_audit = timing.geometry_audit or last_geometry_audit
        if timing.status not in ("DYNAMIC_FEASIBLE", "NO_MOTION"):
            search["rounds"].append(round_record)
            layer = _failure_layer_for_timing(timing.status)
            status = "GEOMETRY_FAILED" if layer == "GEOMETRY" else \
                ("DYNAMICS_FAILED" if layer == "DYNAMICS" else
                 "NO_ACCEPTED_CANDIDATE")
            return _empty_result(
                map_request, status, timing.reason or "TIMING_REJECTED",
                timing.detail or "retimer produced no dynamic plan", layer,
                timings, time.monotonic() - started, search,
                checks=_report_checks(last_geometry_audit),
                attempts=all_attempts, routes=tuple(routes),
                geometry_audit=last_geometry_audit)
        plan = timing.plan
        dynamics_started = time.monotonic()
        dynamics = audit_dynamics(
            request, plan, dynamic_config,
            expected_planning_context=context)
        timings["dynamics_audit_s"] += time.monotonic() - dynamics_started
        last_dynamic_audit = dynamics
        round_record["dynamics_passed"] = bool(dynamics.passed)
        round_record["dynamics_reason"] = dynamics.reason
        if not dynamics.passed:
            search["rounds"].append(round_record)
            return _empty_result(
                map_request, "DYNAMICS_FAILED",
                dynamics.reason or "DYNAMIC_AUDIT_REJECTED",
                dynamics.detail or "independent dynamic audit rejected plan",
                "DYNAMICS", timings, time.monotonic() - started, search,
                checks=_report_checks(last_geometry_audit) +
                _report_checks(dynamics),
                attempts=all_attempts, routes=tuple(routes),
                geometry_audit=last_geometry_audit,
                dynamic_audit=dynamics)
        collision_started = time.monotonic()
        collision = audit_collision(
            request, plan, grid, collision_config)
        timings["collision_audit_s"] += time.monotonic() - collision_started
        last_collision_audit = collision
        round_record["collision_passed"] = bool(collision.passed)
        round_record["collision_reason"] = collision.reason
        round_record["collision_first_failure"] = collision.first_failure
        if collision.passed:
            search["rounds"].append(round_record)
            break
        if hold_request or collision_retries >= MAX_COLLISION_RETRIES \
                or remaining() <= 1.0:
            search["rounds"].append(round_record)
            return _empty_result(
                map_request, "COLLISION_FAILED",
                collision.reason or "COLLISION_AUDIT_REJECTED",
                collision.detail or "real three-vehicle sweep rejected plan",
                "COLLISION", timings, time.monotonic() - started, search,
                checks=_report_checks(last_geometry_audit) +
                _report_checks(dynamics) + _report_checks(collision),
                attempts=all_attempts, routes=tuple(routes),
                geometry_audit=last_geometry_audit,
                dynamic_audit=dynamics, collision_audit=collision,
                metrics={
                    "collision_checks": _report_checks(collision),
                    "collision_margin_m": (
                        collision.metrics.get("margin")
                        if isinstance(collision.metrics, dict) else None),
                })
        collision_retries += 1
        try:
            obstacle_context = _collision_retry_context(
                collision, plan, digest, routes[0],
                int(geometry_config.intervals),
                retry_index=collision_retries)
        except Exception:
            obstacle_context = ObstacleContext(
                grid_sha256=digest, source="collision_audit_retry")
        # A new topological guide is regenerated with the failed formation
        # centre as a blocked disk on the next loop iteration.
        routes = ()
        search["rounds"].append(round_record)

    # Every layer passed: publish the exact plan that was audited.  No field
    # is added to the timed model after the dynamic/collision audits.
    try:
        model_sha = plan.model_sha256()
        plan_context_sha = plan.planning_context_sha256()
        request_sha = plan.request_sha256
    except Exception as exc:
        return _empty_result(
            map_request, "NUMERICAL_FAILURE", "MODEL_BINDING_FAILURE",
            "{}: {}".format(type(exc).__name__, exc), "MODEL", timings,
            time.monotonic() - started, search, attempts=all_attempts,
            routes=tuple(routes), geometry_audit=last_geometry_audit,
            dynamic_audit=last_dynamic_audit,
            collision_audit=last_collision_audit)
    if plan_context_sha != context_sha:
        return _empty_result(
            map_request, "NUMERICAL_FAILURE", "CONTEXT_BINDING_FAILURE",
            "plan and request planning-context digests differ", "MODEL",
            timings, time.monotonic() - started, search,
            attempts=all_attempts, routes=tuple(routes),
            geometry_audit=last_geometry_audit,
            dynamic_audit=last_dynamic_audit,
            collision_audit=last_collision_audit)
    status = "NO_MOTION" if getattr(plan, "kind", None) == "hold" else "SUCCESS"
    checks = _report_checks(last_geometry_audit) + _report_checks(
        last_dynamic_audit) + _report_checks(last_collision_audit)
    try:
        result = VirtualAResult(
            status=status,
            reason="",
            detail="geometry, dynamics and collision audits passed",
            first_failure_layer="",
            request_sha256=request_sha,
            planning_context_sha256=plan_context_sha,
            grid_sha256=digest,
            grid_summary=grid_summary(grid),
            model_sha256=model_sha,
            path=path,
            plan=plan,
            projection=None,
            geometry_audit=last_geometry_audit,
            dynamic_audit=last_dynamic_audit,
            collision_audit=last_collision_audit,
            attempts=tuple(all_attempts),
            routes=tuple(routes),
            search=search,
            wall_time_s=time.monotonic() - started,
            timings=timings,
            metrics={
                "path_length_m": float(path.length_m),
                "duration_s": float(getattr(plan, "duration_s", 0.0)),
                "collision_checks": _report_checks(last_collision_audit),
                "collision_margin_m": (
                    last_collision_audit.metrics.get("margin")
                    if isinstance(last_collision_audit.metrics, dict) else None),
            },
            checks=checks)
    except VirtualAContractError as exc:
        return _empty_result(
            map_request, "NUMERICAL_FAILURE", "RESULT_ASSEMBLY_FAILURE",
            str(exc), "MODEL", timings, time.monotonic() - started, search,
            attempts=all_attempts, routes=tuple(routes),
            geometry_audit=last_geometry_audit,
            dynamic_audit=last_dynamic_audit,
            collision_audit=last_collision_audit)
    timings["result_assembly_s"] = max(
        0.0, time.monotonic() - started - sum(
            value for key, value in timings.items()
            if key != "result_assembly_s"))
    timings["total_s"] = time.monotonic() - started
    return result


__all__ = [
    "G4_ROUTE_CLEARANCE_WEIGHT",
    "G4_TOTAL_SOFT_BUDGET_S",
    "MAX_COLLISION_RETRIES",
    "make_route_guides",
    "plan_virtual_a",
]