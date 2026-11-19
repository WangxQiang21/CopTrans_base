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
from types import MappingProxyType, SimpleNamespace

import numpy as np

from .audit import audit_dynamics
from .collision import audit_collision
from .contracts import (
    COLLISION_SCOPE,
    DYNAMICS_SCOPE,
    PROGRESS_FAILED_STAGE,
    PROGRESS_SCHEMA,
    PROGRESS_STAGE_INDEX,
    PROGRESS_STAGES,
    PROGRESS_STATES,
    CachedClearanceField,
    DynamicAuditConfig,
    GuideRoute,
    HeadingBranch,
    MAX_INITIALS,
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
    _wrap,
)
from .corridor import (
    allocate_corridors,
    audit_corridor_chain,
    audit_path_in_corridors,
    build_corridor_chain,
    corridor_effective_margin,
    make_corridor_guide,
    reallocate_adjacent_portal_once,
)
from .optimizer import initial_guess_key, solve_open_path_guided
from .route_processing import segment_is_free, simplify_route
from .timing import retime_path


G4_TOTAL_SOFT_BUDGET_S = 60.0
# C3: one type-directed recovery replaces the historical two collision
# retries.  No stacked loop outside the single planning loop.
MAX_RECOVERIES = 1
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


def _routes_shortcut_equivalent(grid, first, second):
    """First-version route-diversity screen, not a homotopy classification.

    Two simplified guides are treated as the same candidate when every pair
    of their vertices is exactly mutually visible.  Each pair then admits a
    two-point shortcut inside the same free region, so the difference is only
    a slight dogleg.  A blocked cross pair means the guides are separated by
    obstacles (for example the two sides of one box) and both are retained.
    This screen can merge non-equivalent routes in principle; it makes no
    completeness claim and only schedules initial values.
    """
    return all(
        segment_is_free(grid, first_point, second_point)
        for first_point in first for second_point in second)


def make_route_guides(grid, start_qf, goal_qf, config=None, *,
                      blocked_points=()):
    """Return up to ``max_routes`` finite A* F-point guides.

    The search uses the authoritative occupancy grid, a non-negative EDT
    clearance cost and deterministic eight-neighbour moves.  It is a topology
    hint only: it contains no vehicle path, curvature reference or known-good
    solution.

    C3 candidate scheduling: each A* route is screened by its greedy
    simplification, and an exact start-goal line-of-sight route joins the
    same bounded capacity when it is distinct.  The screen merges slight
    doglegs, keeps obstacle-separated sides, and adds no extra A* search.
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
    kept = []
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
        candidate = GuideRoute(
            points=points,
            source="astar_primary" if index == 0 else
            "astar_alternative_{}".format(index),
            grid_sha256=digest)
        shape = simplify_route(grid, candidate).points
        if not any(_routes_shortcut_equivalent(grid, shape, other)
                   for _, other in kept):
            kept.append((candidate, shape))
        # Penalise a small band around the previous guide so the next search
        # explores a genuinely different topology.  This is not an executable
        # path and never enters the final certificate.
        for iy, ix in cells:
            y0, y1 = max(0, iy - 2), min(
                grid.occ.shape[0], iy + 3)
            x0, x1 = max(0, ix - 2), min(
                grid.occ.shape[1], ix + 3)
            penalty[y0:y1, x0:x1] += float(config.alternative_penalty)
    if not blocked and math.hypot(goal[0] - start[0], goal[1] - start[1]) > 0.0 \
            and segment_is_free(grid, start, goal):
        # An exact C1 line-of-sight is only a topology candidate, never a
        # three-vehicle certificate.  A collision local-block regeneration
        # deliberately omits it so the real blocked position stays in charge.
        direct = GuideRoute(
            points=(start, goal), source="open_direct_los",
            grid_sha256=digest)
        if not any(_routes_shortcut_equivalent(
                grid, direct.points, other) for _, other in kept):
            kept.append((direct, direct.points))
    return tuple(guide for guide, _ in kept[:int(config.max_routes)])


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


def _collision_retry_context(report, plan, grid_sha256, route, intervals):
    """Build one geometric local block from the real collision position.

    This is an explicitly heuristic recovery input: only the audited
    collision location enters, and the regenerated guide plus every existing
    audit remain authoritative.  An empty ``blocked_points`` means the audit
    reported no usable position and the caller must not recover.
    """
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
    return ObstacleContext(
        grid_sha256=grid_sha256,
        node_targets=tuple(targets),
        source="collision_recovery_local_block_heuristic",
        collision_station=float(station) if station is not None else None,
        collision_reason=str(getattr(report, "reason", "") or ""),
        blocked_points=tuple(blocked),
        route_weight_scale=1.0)


def _report_checks(report):
    return int(getattr(report, "check_count", 0) or 0)


# ---------------------------------------------------------------------------
# Display-only progress events (TASK 01 Step 1)
# ---------------------------------------------------------------------------

def _progress_scalar(value):
    """Return a JSON-safe copy of one metric value, or None.

    Values that are not finite or not plain data are dropped: the ROS node
    serialises events with ``allow_nan=False``, and a display-only payload must
    never be able to break planning or a later serialisation.
    """
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (list, tuple)):
        return [_progress_scalar(item) for item in value]
    if hasattr(value, "items"):
        return {str(key): _progress_scalar(item)
                for key, item in value.items()}
    return None


def _progress_audit_metrics(report):
    """Real summary of one independent audit report; no placeholder fields."""
    if report is None:
        return {"report_available": False}
    return {
        "report_available": True,
        "scope": str(getattr(report, "scope", "") or ""),
        "passed": bool(getattr(report, "passed", False)),
        "reason": str(getattr(report, "reason", "") or ""),
        "detail": str(getattr(report, "detail", "") or ""),
        "check_count": int(getattr(report, "check_count", 0) or 0),
        "first_failure": _progress_scalar(
            getattr(report, "first_failure", None)),
        "metrics": _progress_scalar(getattr(report, "metrics", None) or {}),
    }


def _corridor_evidence(geometry):
    """Actual corridor halfspace count and maximum residual of one solve.

    ``None`` means the solve constructed no corridor constraint; it is never
    replaced by a count estimated from the display polygon.
    """
    count = getattr(geometry, "corridor_halfspace_constraint_count", None)
    residual = getattr(geometry, "corridor_max_residual_m", None)
    return {
        "corridor_halfspace_constraint_count": (
            None if count is None else int(count)),
        "corridor_max_residual_m": (
            None if residual is None else float(residual)),
    }


def _s_domain_evidence(geometry):
    """Finite S domain, solved S and upper-bound activity of one solve.

    Every field is copied from an ``AttemptRecord``, which received the values
    from the single solver packet.  For a successful solve the audited attempt
    whose solved S equals the returned path length is reported; otherwise the
    last attempt is reported.  ``upper_active`` is the attempt's numeric
    tolerance decision, never a solver status string.
    """
    attempts = tuple(getattr(geometry, "attempts", ()) or ())
    audited = [
        attempt for attempt in attempts
        if str(getattr(attempt, "status", "") or "")
        == "SOLVER_SUCCESS_AUDITED"]
    decisive = None
    path = getattr(geometry, "path", None)
    if path is not None and audited:
        length = float(getattr(path, "length_m", float("nan")))
        for attempt in audited:
            solution = getattr(attempt, "s_solution_m", None)
            if solution is not None and math.isfinite(length) \
                    and abs(float(solution) - length) <= 1e-9:
                decisive = attempt
                break
        if decisive is None:
            decisive = audited[-1]
    elif attempts:
        decisive = attempts[-1]
    if decisive is None:
        return {
            "distance_m": None,
            "guide_length_m": None,
            "S_lower_m": None,
            "S_upper_m": None,
            "S_solution_m": None,
            "upper_active": None,
        }
    return {
        "distance_m": getattr(decisive, "distance_m", None),
        "guide_length_m": getattr(decisive, "guide_length_m", None),
        "S_lower_m": getattr(decisive, "s_lower_m", None),
        "S_upper_m": getattr(decisive, "s_upper_m", None),
        "S_solution_m": getattr(decisive, "s_solution_m", None),
        "upper_active": getattr(decisive, "s_upper_active", None),
    }


def _progress_polyline_length(points):
    points = tuple(points or ())
    total = 0.0
    for first, second in zip(points, points[1:]):
        total += math.hypot(
            float(second[0]) - float(first[0]),
            float(second[1]) - float(first[1]))
    return float(total)


# Step 3: a guide segment shorter than this is treated as numerical noise and
# removed before the tangent turn is accumulated.  Real A*/corridor guides
# have metre-scale segments; the threshold only removes duplicate vertices.
WINDING_GUIDE_SEGMENT_EPS_M = 1e-6
# The finite branch set is {k0-1, k0, k0+1}: the preferred branch plus its two
# neighbours, never an unbounded integer search.
WINDING_NEIGHBOUR_BRANCHES = 1
# Accumulating wrapped atan2 turns leaves a few float ulps in otherwise equal
# branch distances.  Ranking compares distances at 1e-12 rad so the specified
# tie-break (smaller |k|, then smaller k) decides mathematical ties instead of
# floating-point noise; 1e-12 rad is far below any physical heading scale.
WINDING_RANK_TIE_DIGITS = 12


def _guide_segment_tangents(points):
    """Effective guide-segment tangents after duplicate/near-zero removal."""
    cleaned = []
    for point in tuple(points or ()):
        try:
            x, y = float(point[0]), float(point[1])
        except (TypeError, ValueError, IndexError) as exc:
            raise VirtualAContractError(
                "INVALID_GUIDE_ROUTE",
                "guide points must be (x, y) pairs") from exc
        if not (math.isfinite(x) and math.isfinite(y)):
            raise VirtualAContractError(
                "INVALID_GUIDE_ROUTE", "guide point is not finite")
        if cleaned:
            last_x, last_y = cleaned[-1]
            if math.hypot(x - last_x, y - last_y) <= \
                    WINDING_GUIDE_SEGMENT_EPS_M:
                continue
        cleaned.append((x, y))
    return tuple(
        math.atan2(cleaned[index + 1][1] - cleaned[index][1],
                   cleaned[index + 1][0] - cleaned[index][0])
        for index in range(len(cleaned) - 1))


def _heading_branch_candidates(request, points, source):
    """Finite deterministic endpoint-heading branches for one guide.

    ``Theta_G`` accumulates the wrapped turn of the effective guide tangents
    between the start tangent, every segment tangent and the goal tangent.
    ``base = wrap(psi_goal - psi_start)`` and the preferred integer is
    ``round((Theta_G - base)/2*pi)``.  The finite set ``{k0-1, k0, k0+1}`` is
    ordered by unwrapped-turn error, then smaller ``|k|``, then smaller ``k``.
    A degenerate or absent guide has no reliable turn: only the primary
    shortest-wrap branch ``k=0`` is offered.  No guide is ever executable and
    the physical goal pose is never modified.
    """
    base = _wrap(request.goal_qf.psi - request.start_qf.psi)
    tangents = _guide_segment_tangents(points)
    if not tangents:
        return (HeadingBranch(
            base_heading_rad=base,
            winding_k=0,
            unwrapped_heading_rad=base,
            guide_turn_rad=None,
            source="no_reliable_guide",
            candidate_rank=0),)
    guide_turn = 0.0
    previous = request.start_qf.psi
    for tangent in tangents:
        guide_turn += _wrap(tangent - previous)
        previous = tangent
    guide_turn += _wrap(request.goal_qf.psi - previous)
    preferred = int(round((guide_turn - base) / (2.0 * math.pi)))
    ks = {
        preferred + offset
        for offset in range(-WINDING_NEIGHBOUR_BRANCHES,
                            WINDING_NEIGHBOUR_BRANCHES + 1)}
    ordered = sorted(
        ks,
        key=lambda k: (
            round(abs(base + 2.0 * math.pi * k - guide_turn),
                  WINDING_RANK_TIE_DIGITS),
            abs(k), k))
    return tuple(HeadingBranch(
        base_heading_rad=base,
        winding_k=int(k),
        unwrapped_heading_rad=float(base + 2.0 * math.pi * k),
        guide_turn_rad=guide_turn,
        source=str(source),
        candidate_rank=rank) for rank, k in enumerate(ordered))


def _winding_evidence(branch, guide_length_m):
    """Round/search evidence for the packet's unwrapped heading branch."""
    return {
        "heading_branch": None if branch is None else branch.as_dict(),
        "heading_base_rad": (
            None if branch is None else float(branch.base_heading_rad)),
        "winding_k": None if branch is None else int(branch.winding_k),
        "unwrapped_heading_rad": (
            None if branch is None else float(branch.unwrapped_heading_rad)),
        "guide_turn_rad": (
            None if branch is None else branch.guide_turn_rad),
        "winding_candidate_rank": (
            None if branch is None else int(branch.candidate_rank)),
        "guide_length_m": (
            None if guide_length_m is None else float(guide_length_m)),
    }


def _winding_retry_allowed(geometry):
    """Only solver non-convergence or an endpoint-constraint rejection may
    spend the single recovery on the next finite heading branch.  Collision,
    dynamics, numerical-certification and corridor-containment failures never
    guess another winding."""
    reason = str(getattr(geometry, "reason", "") or "")
    if reason in ("SOLVER_NOT_CONVERGED", "FIXED_ALLOCATION_LIMIT"):
        return True
    if reason != "AUDIT_REJECTED":
        return False
    # The optimizer derives AUDIT_REJECTED from the last audit-rejected
    # attempt; an earlier endpoint rejection does not make a final numerical
    # certification failure an endpoint constraint.
    rejections = [
        attempt for attempt in tuple(getattr(geometry, "attempts", ()))
        if str(getattr(attempt, "status", "") or "")
        == "SOLVER_SUCCESS_AUDIT_REJECTED"]
    if not rejections:
        return False
    return str(getattr(rejections[-1], "rejection_reason", "") or "") in (
        "ENDPOINT_POSITION_ERROR", "ENDPOINT_HEADING_ERROR")


def _progress_astar_display(routes):
    """Copy each actual raw A* polyline for the progressive display layer.

    Only the real generated route points are exported; nothing is simplified,
    resampled or refitted.  The node renders them as dashed display geometry.
    """
    display = []
    for index, route in enumerate(routes or ()):
        points = tuple(getattr(route, "points", ()) or ())
        if len(points) < 2:
            continue
        display.append({
            "route_id": int(index),
            "points": [[float(point[0]), float(point[1])]
                       for point in points],
        })
    return tuple(display)


def _progress_result_view(progress, search, routes):
    """Read-only result-shaped view of the still-running pipeline.

    The node's existing ``_display_from_corridor_round()`` consumes exactly
    ``search``/``routes``/``grid_sha256``; providing them here lets it reuse
    the same corridor display builder instead of copying chain geometry.
    ``model_sha256`` stays ``None`` until the node publishes the envelope.
    """
    return SimpleNamespace(
        search=search,
        routes=tuple(routes or ()),
        grid_sha256=progress.grid_sha256,
        planning_context_sha256=progress.planning_context_sha256,
        model_sha256=None)


class _ProgressEmitter:
    """Cover the optional ROS-free callback with identity and stage checks.

    The callback is display-only: it cannot alter the request, the solver, the
    budget or ``search``.  One exception disables the remaining callbacks for
    this request and is retained on ``error`` for the node's explicit
    ``PROGRESS_DISPLAY_ERROR`` record; planning and every safety audit continue.
    """

    def __init__(self, map_request, callback):
        self._callback = callback if callable(callback) else None
        self._started = time.monotonic()
        self._request_sha256 = ""
        self._grid_sha256 = ""
        self._context_sha256 = ""
        try:
            self._request_sha256 = str(map_request.request_sha256)
            self._grid_sha256 = str(map_request.grid_sha256)
            self._context_sha256 = str(
                planning_context_sha256(map_request.context()))
        except Exception:
            # An invalid request still gets a FAILED event; identity fields
            # stay empty rather than inventing a digest.
            pass
        self.route_index = None
        self.route_count = 0
        self.round_index = None
        self.last_stage_index = 0
        self.last_path = None
        # Process-local display geometry, never serialized by the pipeline.
        # ``last_astar`` lets a FAILED document redraw the already completed
        # A* layer; ``last_result_view`` carries the actual ``search``/
        # ``routes`` so the node can reuse ``_display_from_corridor_round()``
        # instead of duplicating corridor geometry.
        self.last_astar = None
        self.last_result_view = None
        self.error = None
        self.error_count = 0

    @property
    def request_sha256(self):
        return self._request_sha256

    @property
    def grid_sha256(self):
        return self._grid_sha256

    @property
    def planning_context_sha256(self):
        return self._context_sha256

    @property
    def disabled(self):
        return self.error is not None

    def emit(self, stage, state="PLANNING", metrics=None,
             diagnostic_path=None, route_index=None, route_count=None,
             round_index=None, diagnostic_display=None):
        if stage not in PROGRESS_STAGE_INDEX \
                and stage != PROGRESS_FAILED_STAGE:
            raise ValueError("unknown progress stage: {!r}".format(stage))
        if state not in PROGRESS_STATES:
            raise ValueError("unknown progress state: {!r}".format(state))
        if stage == "PUBLISHED" or state == "PUBLISHED":
            raise ValueError(
                "only the ROS node emits PUBLISHED after the atomic envelope")
        if (state == "FAILED") != (stage == PROGRESS_FAILED_STAGE):
            raise ValueError(
                "FAILED state and FAILED stage must be used together")
        if route_index is not None:
            self.route_index = int(route_index)
        if route_count is not None:
            self.route_count = int(route_count)
        if round_index is not None:
            self.round_index = int(round_index)
        if diagnostic_path is not None:
            self.last_path = diagnostic_path
        if diagnostic_display is not None:
            astar = diagnostic_display.get("astar")
            result_view = diagnostic_display.get("result")
            if astar is not None:
                self.last_astar = astar
            if result_view is not None:
                self.last_result_view = result_view
        if stage == PROGRESS_FAILED_STAGE:
            stage_index = int(self.last_stage_index)
        else:
            stage_index = int(PROGRESS_STAGE_INDEX[stage])
            self.last_stage_index = stage_index
        event = MappingProxyType({
            "schema": PROGRESS_SCHEMA,
            "stage": stage,
            "stage_index": stage_index,
            "state": state,
            "elapsed_s": max(0.0, time.monotonic() - self._started),
            "route_index": self.route_index,
            "route_count": int(self.route_count),
            "round_index": self.round_index,
            "request_sha256": self._request_sha256,
            "grid_sha256": self._grid_sha256,
            "planning_context_sha256": self._context_sha256,
            "metrics": MappingProxyType(dict(metrics or {})),
            "diagnostic_path": (
                diagnostic_path if diagnostic_path is not None
                else self.last_path),
            # Display-only in-process geometry; the ROS node reads it and the
            # pipeline itself never serializes or feeds it back to the solver.
            "diagnostic_display": MappingProxyType({
                "astar": self.last_astar,
                "result": self.last_result_view,
            }),
        })
        callback = self._callback
        if callback is None or self.disabled:
            return
        try:
            callback(event)
        except Exception as exc:
            # Display failure only: record it and disable later callbacks.
            self.error_count += 1
            self.error = "{}: {}".format(type(exc).__name__, exc)


def _progress_diagnostic_path(geometry):
    """The real path a solver actually constructed, never an interpolation.

    ``geometry.path`` is the accepted path when the geometry audit passed.
    Otherwise the last attempt that retained a ``candidate_path`` is the
    diagnostic object the solver really built.
    """
    path = getattr(geometry, "path", None)
    if path is not None:
        return path
    for attempt in reversed(tuple(getattr(geometry, "attempts", ()) or ())):
        candidate = getattr(attempt, "candidate_path", None)
        if candidate is not None:
            return candidate
    return None


# ---------------------------------------------------------------------------
# G4 end-to-end pipeline
# ---------------------------------------------------------------------------

def plan_virtual_a(map_request, progress_callback=None):
    """Autonomously solve, retime, audit and certify one map request.

    ``progress_callback`` is optional and ROS-free.  It receives read-only
    stage dictionaries and is never allowed to influence planning.  The
    pipeline never emits ``PUBLISHED``: only the ROS node knows whether the
    atomic playback envelope was actually published.
    """
    progress = _ProgressEmitter(map_request, progress_callback)
    try:
        result = _plan_virtual_a_impl(map_request, progress)
    except BaseException as exc:
        if not progress.disabled:
            progress.emit(
                PROGRESS_FAILED_STAGE, state="FAILED",
                metrics={
                    "phase": "exception",
                    "reason": "PIPELINE_EXCEPTION",
                    "detail": "{}: {}".format(type(exc).__name__, exc),
                },
                diagnostic_path=progress.last_path)
        raise
    if not bool(getattr(result, "executable", False)):
        progress.emit(
            PROGRESS_FAILED_STAGE, state="FAILED",
            metrics={
                "phase": "failed",
                "status": str(getattr(result, "status", "") or ""),
                "reason": str(getattr(result, "reason", "") or ""),
                "detail": str(getattr(result, "detail", "") or ""),
                "first_failure_layer": str(
                    getattr(result, "first_failure_layer", "") or ""),
                "wall_time_s": float(
                    getattr(result, "wall_time_s", 0.0) or 0.0),
                "checks": int(getattr(result, "checks", 0) or 0),
            },
            diagnostic_path=progress.last_path)
    return result


def _plan_virtual_a_impl(map_request, progress):
    started = time.monotonic()
    timings = {
        "route_generation_s": 0.0,
        "geometry_solve_s": 0.0,
        "retime_s": 0.0,
        "dynamics_audit_s": 0.0,
        "collision_audit_s": 0.0,
        "result_assembly_s": 0.0,
    }
    search = {
        "routes": (), "rounds": [], "budget": {},
        "route_count": 0, "initial_count": 0, "recovery_count": 0,
        "initial_count_total": 0, "duplicate_initial_launch_count": 0,
        "initial_keys": [], "reallocation": None,
        "first_failure_layer": "", "last_reason": "",
        "recoveries": [],
        "winding": {
            "candidates": [], "selected_rank": None,
            "selected": None, "guide_length_m": None},
    }

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
    corridor_mode = str(planner_config.guidance_mode).strip().lower() == \
        "corridor"
    corridor_enforcement = str(
        planner_config.corridor_enforcement).strip().lower()
    # C3 rework 01: one request-wide initial pool.  The configured cap may be
    # stricter than six but can never exceed the contract's six.
    initial_limit = min(MAX_INITIALS, int(planner_config.max_initials))
    search["initial_limit"] = initial_limit
    if corridor_mode:
        timings.update({
            "corridor_build_s": 0.0,
            "corridor_guide_s": 0.0,
            "corridor_allocation_s": 0.0,
            "corridor_containment_s": 0.0,
        })
        search["corridor"] = {
            "guidance_mode": "corridor",
            "enforcement": corridor_enforcement,
            "rounds": [],
        }
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
        "max_recoveries": MAX_RECOVERIES,
        "planner_config_max_seconds": float(planner_config.max_seconds),
        "route_config": replace(
            RouteGuideConfig(),
            clearance_weight=G4_ROUTE_CLEARANCE_WEIGHT).as_dict(),
        "hard_real_time_guarantee": False,
    }
    if corridor_mode:
        search["budget"]["planner_config"] = planner_config.as_dict()

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
    route_index = 0
    tried_routes = set()
    recovery_count = 0
    endpoint_only = False
    winding_guide_key = None
    winding_branches = ()
    winding_branch_rank = 0
    used_initial_keys = set()
    reallocation_done = False
    first_failure_layer = ""
    obstacle_context = None
    clearance_field = None
    chain = None
    corridor_guide = None
    corridor_allocation = None
    corridor_round = None
    corridor_route_index = None
    containment = None
    all_attempts = []
    last_geometry_audit = None
    last_dynamic_audit = None
    last_collision_audit = None

    def budget_stop(stage):
        record_failure("PLANNING", "SOFT_BUDGET_EXHAUSTED")
        return _empty_result(
            map_request, "BUDGET_EXHAUSTED", "SOFT_BUDGET_EXHAUSTED",
            "soft request budget exhausted before " + str(stage),
            "PLANNING", timings, time.monotonic() - started, search,
            attempts=all_attempts, routes=tuple(routes),
            geometry_audit=last_geometry_audit,
            dynamic_audit=last_dynamic_audit,
            collision_audit=last_collision_audit)

    def untried_route_index():
        for index in range(len(routes)):
            if index not in tried_routes:
                return index
        return None

    def remaining_initial_slots():
        return max(0, initial_limit - len(used_initial_keys))

    def absorb_attempt_initials(geometry):
        """Count only distinct initials actually launched by this round.

        ``AttemptRecord.initial`` is the InitialGuess that produced the
        attempt.  An IPOPT-to-SLSQP fallback yields two attempts with the same
        identity, so the set counts it once.  Initials constructed but never
        launched before a timeout are not counted, because they have no
        attempt record.
        """
        round_keys = set()
        for attempt in geometry.attempts:
            initial = getattr(attempt, "initial", None)
            if initial is not None:
                round_keys.add(initial_guess_key(initial))
        duplicate = len(round_keys & used_initial_keys)
        new_count = len(round_keys - used_initial_keys)
        used_initial_keys.update(round_keys)
        search["initial_count_total"] = len(used_initial_keys)
        search["duplicate_initial_launch_count"] = int(
            search.get("duplicate_initial_launch_count", 0)) + duplicate
        search["initial_keys"] = sorted(
            [list(key) for key in used_initial_keys], key=repr)
        return new_count

    def can_recover():
        # The single C3 recovery consumes one recovery slot and at least one
        # still-unused initial slot.  Once the adjacent-portal reallocation
        # has been used, every later failure is final.
        return (not reallocation_done
                and recovery_count < MAX_RECOVERIES
                and remaining() > 1.0
                and remaining_initial_slots() > 0)

    def record_failure(layer, reason):
        nonlocal first_failure_layer
        if not first_failure_layer:
            first_failure_layer = str(layer)
            search["first_failure_layer"] = first_failure_layer
        search["last_reason"] = str(reason or "")

    def route_switch(reason, next_index):
        # C3: the single recovery action.  It consumes the only recovery slot
        # and only selects an already generated route; it never starts a new
        # A* search or a stacked planning loop.
        nonlocal route_index, corridor_route_index, recovery_count
        recovery_count += 1
        search["recovery_count"] = recovery_count
        search["recoveries"].append({
            "recovery_index": recovery_count,
            "action": "route_switch",
            "first_failure_layer": first_failure_layer,
            "reason": str(reason or ""),
            "from_route_index": int(route_index),
            "to_route_index": int(next_index),
            "wall_time_s": time.monotonic() - started,
        })
        tried_routes.add(route_index)
        route_index = int(next_index)
        corridor_route_index = None

    def endpoint_only_recovery(reason):
        # C3: numerical failure after every initial of this round has been
        # tried may spend the same single recovery on the true endpoint-only
        # family when no untried route remains.  In corridor mode only the
        # high-weight guide is dropped; the hard halfspaces remain in force.
        nonlocal recovery_count, endpoint_only
        recovery_count += 1
        search["recovery_count"] = recovery_count
        search["recoveries"].append({
            "recovery_index": recovery_count,
            "action": "endpoint_only_initial",
            "first_failure_layer": first_failure_layer,
            "reason": str(reason or ""),
            "from_route_index": int(route_index),
            "to_route_index": None,
            "wall_time_s": time.monotonic() - started,
        })
        endpoint_only = True

    def winding_branch_recovery(reason):
        # Step 3: the one recovery may advance this guide's finite branch
        # ordering by exactly one candidate.  The branch changes only the
        # unwrapped endpoint heading used by the packet, never the physical
        # goal; the retry still consumes the request-wide recovery and initial
        # budgets through ``can_recover()``.
        nonlocal recovery_count, winding_branch_rank
        previous = winding_branches[winding_branch_rank]
        winding_branch_rank += 1
        branch = winding_branches[winding_branch_rank]
        recovery_count += 1
        search["recovery_count"] = recovery_count
        search["recoveries"].append({
            "recovery_index": recovery_count,
            "action": "winding_branch",
            "first_failure_layer": first_failure_layer,
            "reason": str(reason or ""),
            "from_route_index": int(route_index),
            "to_route_index": int(route_index),
            "from_winding_k": int(previous.winding_k),
            "to_winding_k": int(branch.winding_k),
            "from_candidate_rank": int(previous.candidate_rank),
            "to_candidate_rank": int(branch.candidate_rank),
            "wall_time_s": time.monotonic() - started,
        })

    def next_winding_branch(solve_routes):
        return (not hold_request and not endpoint_only and bool(solve_routes)
                and winding_branch_rank + 1 < len(winding_branches)
                and can_recover())

    while True:
        containment = None
        if remaining() <= 0.0:
            return budget_stop("route generation/solve")
        if not routes and not hold_request:
            progress.emit("ASTAR_SEARCH", metrics={"phase": "running"})
            route_started = time.monotonic()
            if clearance_field is None:
                try:
                    clearance_field = _cached_clearance_field(grid)
                except Exception as exc:
                    record_failure("PLANNING", "CLEARANCE_FIELD_FAILED")
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
                record_failure("PLANNING", "ROUTE_GENERATION_FAILED")
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
                record_failure("PLANNING", "NO_ROUTE_GUIDE")
                return _empty_result(
                    map_request, "NO_ACCEPTED_CANDIDATE",
                    "NO_ROUTE_GUIDE",
                    "A* found no finite guide on the authoritative grid",
                    "PLANNING", timings, time.monotonic() - started, search,
                    attempts=all_attempts,
                    geometry_audit=last_geometry_audit,
                    dynamic_audit=last_dynamic_audit,
                    collision_audit=last_collision_audit)
            route_index = 0
            tried_routes = set()
            endpoint_only = False
            corridor_route_index = None
            search["route_count"] = len(routes)
            progress.emit(
                "ASTAR_SEARCH",
                metrics={
                    "phase": "complete",
                    "route_count": len(routes),
                    "route_sources": [route.source for route in routes],
                    "route_lengths_m": [
                        float(route.length_m) for route in routes],
                    "route_point_counts": [
                        len(route.points) for route in routes],
                },
                route_index=0, route_count=len(routes),
                diagnostic_display={
                    "astar": _progress_astar_display(routes)})
        if remaining() <= 0.0:
            return budget_stop("corridor build/solve")
        if corridor_mode and not hold_request \
                and corridor_route_index != route_index:
            route = routes[route_index]
            route_failure = None
            failed_stage = "CORRIDOR_BUILD"
            progress.emit(
                "CORRIDOR_BUILD", metrics={"phase": "running"},
                route_index=route_index, route_count=len(routes),
                round_index=len(search["rounds"]))
            build_started = time.monotonic()
            try:
                simplified = simplify_route(grid, route)
                chain = build_corridor_chain(
                    grid, simplified,
                    box_padding_m=float(
                        planner_config.corridor_box_padding_m))
            except VirtualAContractError as exc:
                route_failure = ("CORRIDOR_BUILD_FAILED", exc.detail)
            timings["corridor_build_s"] += time.monotonic() - build_started
            if route_failure is None:
                chain_audit = audit_corridor_chain(grid, chain)
                if not chain_audit["passed"]:
                    route_failure = (
                        chain_audit["reason"], chain_audit["detail"])
            if route_failure is None:
                progress.emit(
                    "CORRIDOR_BUILD",
                    metrics={
                        "phase": "complete",
                        "simplified_point_count": len(simplified.points),
                        "region_count": int(chain_audit["region_count"]),
                        "chain_digest": str(chain.digest),
                        "chain_audit_passed": bool(
                            chain_audit["passed"]),
                        "chain_audit_region_count": int(
                            chain_audit["region_count"]),
                        "chain_audit_min_region_area_m2": _progress_scalar(
                            chain_audit.get("min_region_area_m2")),
                        "chain_audit_min_portal_radius_m": _progress_scalar(
                            chain_audit.get("min_portal_radius_m")),
                    },
                    route_index=route_index, route_count=len(routes))
                failed_stage = "PORTAL_GUIDE"
                progress.emit(
                    "PORTAL_GUIDE", metrics={"phase": "running"},
                    route_index=route_index, route_count=len(routes))
                guide_started = time.monotonic()
                try:
                    corridor_guide = make_corridor_guide(
                        chain,
                        (float(request.start_qf.x),
                         float(request.start_qf.y)),
                        (float(request.goal_qf.x),
                         float(request.goal_qf.y)))
                except VirtualAContractError as exc:
                    route_failure = ("CORRIDOR_GUIDE_FAILED", exc.detail)
                timings["corridor_guide_s"] += \
                    time.monotonic() - guide_started
            if route_failure is None:
                allocation_started = time.monotonic()
                try:
                    corridor_allocation = allocate_corridors(
                        chain, corridor_guide,
                        intervals=int(planner_config.intervals))
                except VirtualAContractError as exc:
                    route_failure = (
                        "CORRIDOR_ALLOCATION_FAILED", exc.detail)
                timings["corridor_allocation_s"] += \
                    time.monotonic() - allocation_started
            if route_failure is None:
                # Record the actual round before announcing completion so the
                # progressive display builder reads the same search/rounds the
                # solver will consume, with no duplicated corridor geometry.
                corridor_round = {
                    "round_index": len(search["rounds"]),
                    "route_index": int(route_index),
                    "route_source": route.source,
                    "simplified_route": simplified.as_dict(),
                    "chain": chain.as_dict(),
                    "chain_digest": chain.digest,
                    "chain_audit": chain_audit,
                    "guide": corridor_guide.as_dict(),
                    "allocation": [
                        list(entry) for entry in corridor_allocation],
                    "effective_margin_m": corridor_effective_margin(
                        chain, planner_config),
                }
                search["corridor"]["rounds"].append(corridor_round)
                corridor_route_index = route_index
                progress.emit(
                    "PORTAL_GUIDE",
                    metrics={
                        "phase": "complete",
                        "guide_source": str(corridor_guide.source),
                        "guide_point_count": len(corridor_guide.points),
                        "guide_length_m": _progress_polyline_length(
                            corridor_guide.points),
                        "guide_quality": _progress_scalar(
                            corridor_guide.quality),
                        "allocation_count": len(corridor_allocation),
                        "effective_margin_m": float(
                            corridor_effective_margin(
                                chain, planner_config)),
                    },
                    route_index=route_index, route_count=len(routes),
                    diagnostic_display={
                        "result": _progress_result_view(
                            progress, search, routes)})
            if route_failure is not None:
                progress.emit(
                    failed_stage,
                    metrics={
                        "phase": "failed",
                        "reason": str(route_failure[0] or ""),
                        "detail": str(route_failure[1] or ""),
                    },
                    route_index=route_index, route_count=len(routes))
                # A degenerate C-side corridor is not a solver failure: the
                # single recovery may select another already generated route,
                # never relax the allocation, intervals or audit.
                reason, detail = route_failure
                record_failure("PLANNING", reason)
                tried_routes.add(route_index)
                search["rounds"].append({
                    "round_index": len(search["rounds"]),
                    "geometry_status": reason,
                    "geometry_reason": reason,
                    "geometry_attempts": 0,
                    "route_index": int(route_index),
                    "route_count": len(routes),
                    "initial_count": 0,
                    "corridor_round_index": None,
                })
                next_index = untried_route_index()
                if can_recover() and next_index is not None:
                    route_switch(reason, next_index)
                    continue
                return _empty_result(
                    map_request, "NO_ACCEPTED_CANDIDATE", reason, detail,
                    "PLANNING", timings, time.monotonic() - started, search,
                    attempts=all_attempts, routes=tuple(routes),
                    geometry_audit=last_geometry_audit,
                    dynamic_audit=last_dynamic_audit,
                    collision_audit=last_collision_audit)
        if not isinstance(geometry_config, type(planner_config)):
            geometry_config = planner_config
        geometry_config = replace(
            planner_config,
            max_seconds=min(float(planner_config.max_seconds), remaining()))
        if corridor_mode and not hold_request:
            # The endpoint-only recovery keeps the corridor halfspaces but
            # drops the corridor guide as a high-weight route target.
            solve_routes = () if endpoint_only else (corridor_guide,)
            solve_chain = chain
            solve_allocation = corridor_allocation
        else:
            # C3 schedules one route per round so the single recovery can
            # select a genuinely untried route instead of reseeding a route
            # that the solver already attempted.  The endpoint-only recovery
            # uses the historical no-route G2 initial family instead.
            solve_routes = (routes[route_index],) \
                if not hold_request and routes and not endpoint_only else ()
            solve_chain = None
            solve_allocation = None
        # Step 3: one finite branch ordering per route/guide.  The preferred
        # branch is the default; only the winding recovery may advance by one.
        heading_branch = None
        guide_length_m = None
        if solve_routes:
            guide = solve_routes[0]
            guide_key = (int(route_index), bool(corridor_mode))
            if guide_key != winding_guide_key:
                winding_guide_key = guide_key
                winding_branches = _heading_branch_candidates(
                    request, guide.points,
                    getattr(guide, "source", "") or "route_guide")
                winding_branch_rank = 0
            winding_branch_rank = min(
                winding_branch_rank, len(winding_branches) - 1)
            heading_branch = winding_branches[winding_branch_rank]
            guide_length_m = _progress_polyline_length(guide.points)
        else:
            winding_guide_key = None
            winding_branches = ()
            winding_branch_rank = 0
        search["winding"] = {
            "candidates": [
                branch.as_dict() for branch in winding_branches],
            "selected_rank": (
                None if heading_branch is None
                else int(heading_branch.candidate_rank)),
            "selected": (
                None if heading_branch is None else heading_branch.as_dict()),
            "guide_length_m": guide_length_m,
        }
        if remaining() <= 0.0:
            return budget_stop("geometry solve")
        progress.emit(
            "NLP_SOLVE",
            metrics={
                "phase": "running",
                "route_guided": bool(solve_routes),
                "endpoint_only": bool(endpoint_only),
                "corridor_guided": bool(solve_chain is not None),
                **_winding_evidence(heading_branch, guide_length_m),
            },
            route_index=route_index if routes else None,
            route_count=len(routes),
            round_index=len(search["rounds"]))
        solve_started = time.monotonic()
        geometry = solve_open_path_guided(
            request, geometry_config, route_guides=solve_routes,
            obstacle_context=obstacle_context,
            geometry_audit_config=audit_config,
            corridor_chain=solve_chain,
            corridor_allocation=solve_allocation,
            heading_branch=heading_branch,
            guide_length_m=guide_length_m,
            excluded_initial_keys=tuple(used_initial_keys),
            max_new_initials=remaining_initial_slots(),
            stop_after_route_audit=bool(solve_routes))
        timings["geometry_solve_s"] += time.monotonic() - solve_started
        all_attempts.extend(geometry.attempts)
        last_geometry_audit = geometry.audit
        diagnostic_path = _progress_diagnostic_path(geometry)
        last_attempt = (geometry.attempts[-1]
                        if geometry.attempts else None)
        progress.emit(
            "NLP_SOLVE",
            metrics={
                "phase": "complete",
                "status": str(geometry.status),
                "reason": str(geometry.reason or ""),
                "detail": str(geometry.detail or ""),
                "solver_used": str(geometry.solver_used or ""),
                "initial_count": int(geometry.initial_count),
                "attempt_count": len(geometry.attempts),
                "iterations": int(getattr(last_attempt, "iterations", 0)
                                  or 0),
                "constraint_residual": _progress_scalar(getattr(
                    last_attempt, "constraint_residual", None)),
                **_corridor_evidence(geometry),
                **_winding_evidence(heading_branch, guide_length_m),
                **_s_domain_evidence(geometry),
                "S_m": (None if diagnostic_path is None else float(
                    getattr(diagnostic_path, "length_m", 0.0) or 0.0)),
                "wall_time_s": float(geometry.wall_time_s),
            },
            diagnostic_path=diagnostic_path,
            route_index=route_index if routes else None,
            route_count=len(routes),
            round_index=len(search["rounds"]))
        progress.emit(
            "GEOMETRY_AUDIT",
            metrics={
                "phase": "complete",
                "source": "geometry_solver",
                "geometry_status": str(geometry.status),
                **_progress_audit_metrics(geometry.audit),
            },
            route_index=route_index if routes else None,
            route_count=len(routes))
        tried_routes.add(route_index)
        new_initial_count = absorb_attempt_initials(geometry)
        search["initial_count"] = int(geometry.initial_count)
        round_record = {
            "round_index": len(search["rounds"]),
            "geometry_status": geometry.status,
            "geometry_reason": geometry.reason,
            "geometry_attempts": len(geometry.attempts),
            "route_index": int(route_index) if routes else None,
            "route_count": len(routes),
            "initial_count": int(geometry.initial_count),
            "new_initial_count": int(new_initial_count),
            "initial_mode": "endpoint_only" if endpoint_only
            else "route_guided",
            **_winding_evidence(heading_branch, guide_length_m),
            **_s_domain_evidence(geometry),
            **_corridor_evidence(geometry),
            "corridor_round_index": corridor_round["round_index"]
            if corridor_mode and not hold_request
            and corridor_round is not None else None,
            "obstacle_context": obstacle_context.as_dict()
            if obstacle_context is not None else None,
        }
        if geometry.status not in ("GEOMETRY_FEASIBLE", "NO_MOTION"):
            search["rounds"].append(round_record)
            status = _timing_status_for_geometry(geometry.status)
            record_failure(
                "GEOMETRY", geometry.reason or "NO_ACCEPTED_CANDIDATE")
            if _winding_retry_allowed(geometry) \
                    and next_winding_branch(solve_routes):
                # The preferred branch produced no audited candidate and the
                # failure is solver non-convergence or an endpoint-constraint
                # rejection: spend the one recovery on the next finite branch.
                winding_branch_recovery(
                    geometry.reason or "NO_ACCEPTED_CANDIDATE")
                continue
            if geometry.reason in (
                    "SOLVER_NOT_CONVERGED", "ALL_ATTEMPTS_NUMERICAL_FAILURE"):
                # Numerical non-convergence: the solver already scheduled and
                # exhausted this round's initial values; the one recovery may
                # spend the next route's initial instead.  A timeout is not a
                # numerical failure and never falls back.
                next_index = untried_route_index()
                if not hold_request and can_recover() \
                        and next_index is not None:
                    route_switch(geometry.reason, next_index)
                    continue
                if not hold_request and can_recover() and not endpoint_only:
                    endpoint_only_recovery(geometry.reason)
                    continue
            return _empty_result(
                map_request, status,
                geometry.reason or "NO_ACCEPTED_CANDIDATE",
                geometry.detail or "geometry solver produced no audited plan",
                "GEOMETRY", timings, time.monotonic() - started, search,
                checks=_report_checks(geometry.audit),
                attempts=all_attempts, routes=tuple(routes),
                geometry_audit=geometry.audit,
                metrics=(
                    _corridor_evidence(geometry) if corridor_mode else None))
        if corridor_mode and not hold_request:
            if remaining() <= 0.0:
                search["rounds"].append(round_record)
                return budget_stop("corridor containment audit")
            containment_started = time.monotonic()
            try:
                containment = audit_path_in_corridors(
                    geometry.path, chain, corridor_allocation,
                    position_budget_m=float(
                        planner_config.corridor_audit_position_margin_m),
                    max_depth=int(planner_config.corridor_audit_max_depth),
                    max_intervals=int(
                        planner_config.corridor_audit_max_intervals),
                    min_width_m=float(
                        planner_config.corridor_audit_min_width_m))
            except VirtualAContractError as exc:
                timings["corridor_containment_s"] += \
                    time.monotonic() - containment_started
                record_failure("GEOMETRY", "CORRIDOR_AUDIT_INPUT_FAILED")
                return _empty_result(
                    map_request, "GEOMETRY_FAILED",
                    "CORRIDOR_AUDIT_INPUT_FAILED", exc.detail, "GEOMETRY",
                    timings, time.monotonic() - started, search,
                    attempts=all_attempts, routes=tuple(routes),
                    geometry_audit=geometry.audit)
            timings["corridor_containment_s"] += \
                time.monotonic() - containment_started
            corridor_round["containment"] = containment
            round_record["corridor_containment_passed"] = bool(
                containment["passed"])
            round_record["corridor_containment_reason"] = containment["reason"]
            if not containment["passed"] and corridor_enforcement == "hard":
                # C3 rework 01: a hard fixed-sigma containment failure may
                # spend the one recovery on a single adjacent-portal
                # reallocation.  The candidate allocation is only a generator
                # input; the next round must rerun the NLP, continuous
                # containment and every downstream audit.
                search["rounds"].append(round_record)
                record_failure("GEOMETRY", containment["reason"])
                first_failure = containment.get("first_failure")
                failure_s = None
                failure_sigma = None
                failure_index = None
                failure_residual = None
                if isinstance(first_failure, dict) or hasattr(
                        first_failure, "get"):
                    try:
                        failure_s = float(first_failure.get("s_m"))
                        failure_sigma = failure_s / float(
                            geometry.path.length_m)
                        failure_index = int(
                            first_failure.get("corridor_index"))
                        failure_residual = float(
                            first_failure.get("point_residual_m"))
                    except (TypeError, ValueError, ZeroDivisionError):
                        failure_s = failure_sigma = None
                        failure_index = failure_residual = None
                reallocation_record = {
                    "attempted": False,
                    "available": False,
                    "status": "fixed_sigma_reallocation_unavailable",
                    "reason": "fixed_sigma_reallocation_unavailable",
                    "detail": "",
                    "trigger_reason": containment["reason"],
                    "failure_s_m": failure_s,
                    "failure_sigma": failure_sigma,
                    "failure_corridor_index": failure_index,
                    "failure_point_residual_m": failure_residual,
                    "old_allocation": [
                        list(entry) for entry in corridor_allocation],
                    "new_allocation": None,
                    "old_boundary_sigma": None,
                    "new_boundary_sigma": None,
                    "adjacent_corridors": None,
                    "consumed_recovery": False,
                }
                if not can_recover():
                    if reallocation_done:
                        reallocation_record["detail"] = (
                            "a second fixed-sigma containment failure is "
                            "final; no second recovery action is allowed")
                        # Keep the applied reallocation as the authorising
                        # evidence; this second failure is recorded in the
                        # already-spent recovery event instead.
                        search["recoveries"][0]["second_failure"] = {
                            "reason": containment["reason"],
                            "failure_s_m": failure_s,
                            "failure_sigma": failure_sigma,
                            "failure_corridor_index": failure_index,
                            "failure_point_residual_m": failure_residual,
                            "detail": reallocation_record["detail"],
                        }
                    elif remaining_initial_slots() <= 0:
                        reallocation_record["detail"] = (
                            "no unused initial slot remains for a reallocated "
                            "solve")
                    elif remaining() <= 1.0:
                        reallocation_record["detail"] = (
                            "soft request budget exhausted before a "
                            "reallocated solve")
                    else:
                        reallocation_record["detail"] = (
                            "the single recovery slot has already been spent")
                    if not reallocation_done:
                        search["reallocation"] = reallocation_record
                else:
                    reallocation_record["attempted"] = True
                    search["reallocation"] = reallocation_record
                    new_allocation = reallocate_adjacent_portal_once(
                        geometry.path, chain, corridor_allocation, containment)
                    if new_allocation is None:
                        reallocation_record["detail"] = (
                            "no eligible adjacent allocation segment with a "
                            "non-degenerate local overlap; the fixed-sigma "
                            "failure is not at a switch")
                    else:
                        old_allocation = tuple(corridor_allocation)
                        new_allocation = tuple(new_allocation)
                        old_switches = {
                            position: float(old_allocation[position][0])
                            for position in range(1, len(old_allocation))
                            if old_allocation[position - 1][2]
                            != old_allocation[position][2]}
                        new_switches = {
                            position: float(new_allocation[position][0])
                            for position in range(1, len(new_allocation))
                            if new_allocation[position - 1][2]
                            != new_allocation[position][2]}
                        changed = [
                            position for position in old_switches
                            if position in new_switches
                            and abs(old_switches[position]
                                    - new_switches[position]) > 1e-15]
                        structural_change_ok = (
                            len(changed) == 1
                            and all(
                                tuple(old_allocation[index])
                                == tuple(new_allocation[index])
                                for index in range(len(old_allocation))
                                if index not in (changed[0] - 1, changed[0])))
                        if not structural_change_ok:
                            reallocation_record["detail"] = (
                                "candidate allocation changed more than one "
                                "adjacent switch boundary")
                        else:
                            position = changed[0]
                            left_index = int(old_allocation[position - 1][2])
                            right_index = int(old_allocation[position][2])
                            old_boundary = old_switches[position]
                            new_boundary = new_switches[position]
                            recovery_count += 1
                            search["recovery_count"] = recovery_count
                            search["recoveries"].append({
                                "recovery_index": recovery_count,
                                "action": "adjacent_portal_reallocation",
                                "first_failure_layer": first_failure_layer,
                                "reason": str(containment["reason"] or ""),
                                "trigger_reason": str(
                                    containment["reason"] or ""),
                                "from_route_index": int(route_index),
                                "to_route_index": None,
                                "adjacent_corridors": [
                                    left_index, right_index],
                                "old_boundary_sigma": old_boundary,
                                "new_boundary_sigma": new_boundary,
                                "old_boundary_s_m": (
                                    old_boundary * geometry.path.length_m),
                                "new_boundary_s_m": (
                                    new_boundary * geometry.path.length_m),
                                "failure_sigma": failure_sigma,
                                "failure_s_m": failure_s,
                                "wall_time_s": time.monotonic() - started,
                            })
                            reallocation_done = True
                            reallocation_record.update({
                                "available": True,
                                "status": "adjacent_portal_reallocation_"
                                          "applied",
                                "reason": "",
                                "detail": (
                                    "one adjacent switch boundary moved "
                                    "inside the direct two-corridor "
                                    "halfspace overlap"),
                                "new_allocation": [
                                    list(entry)
                                    for entry in new_allocation],
                                "old_boundary_sigma": old_boundary,
                                "new_boundary_sigma": new_boundary,
                                "adjacent_corridors": [
                                    left_index, right_index],
                                "consumed_recovery": True,
                            })
                            corridor_allocation = new_allocation
                            previous_corridor_round = corridor_round
                            corridor_round = dict(previous_corridor_round)
                            corridor_round["round_index"] = len(
                                search["rounds"])
                            corridor_round["allocation"] = [
                                list(entry) for entry in new_allocation]
                            corridor_round["allocation_source"] = (
                                "adjacent_portal_reallocation")
                            corridor_round["previous_allocation"] = [
                                list(entry) for entry in old_allocation]
                            corridor_round["reallocation"] = {
                                "old_boundary_sigma": old_boundary,
                                "new_boundary_sigma": new_boundary,
                                "adjacent_corridors": [
                                    left_index, right_index],
                                "trigger_reason": containment["reason"],
                                "failure_sigma": failure_sigma,
                            }
                            search["corridor"]["rounds"].append(
                                corridor_round)
                            continue
                return _empty_result(
                    map_request, "GEOMETRY_FAILED", containment["reason"],
                    containment["detail"], "GEOMETRY", timings,
                    time.monotonic() - started, search,
                    checks=_report_checks(geometry.audit),
                    attempts=all_attempts, routes=tuple(routes),
                    geometry_audit=geometry.audit)
        path = geometry.path
        if remaining() <= 0.0:
            search["rounds"].append(round_record)
            return budget_stop("retime")
        retime_started = time.monotonic()
        timing = retime_path(
            request, path, timing_config, planning_context=context)
        timings["retime_s"] += time.monotonic() - retime_started
        round_record["timing_status"] = timing.status
        round_record["timing_reason"] = timing.reason
        round_record["duration_s"] = timing.T_s
        last_geometry_audit = timing.geometry_audit or last_geometry_audit
        if timing.geometry_audit is not None:
            # The retimer reruns the independent geometry audit on the timed
            # model; report that actual report instead of the solver's copy.
            progress.emit(
                "GEOMETRY_AUDIT",
                metrics={
                    "phase": "complete",
                    "source": "retime",
                    "timing_status": str(timing.status),
                    **_progress_audit_metrics(timing.geometry_audit),
                },
                route_index=route_index if routes else None,
                route_count=len(routes))
        if timing.status not in ("DYNAMIC_FEASIBLE", "NO_MOTION"):
            search["rounds"].append(round_record)
            layer = _failure_layer_for_timing(timing.status)
            record_failure(layer, timing.reason or "TIMING_REJECTED")
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
        if remaining() <= 0.0:
            search["rounds"].append(round_record)
            return budget_stop("dynamics audit")
        progress.emit(
            "DYNAMICS_AUDIT",
            metrics={
                "phase": "running",
                "timing_status": str(timing.status),
                "duration_s": float(getattr(timing, "T_s", 0.0) or 0.0),
            },
            route_index=route_index if routes else None,
            route_count=len(routes))
        dynamics_started = time.monotonic()
        dynamics = audit_dynamics(
            request, plan, dynamic_config,
            expected_planning_context=context)
        timings["dynamics_audit_s"] += time.monotonic() - dynamics_started
        last_dynamic_audit = dynamics
        progress.emit(
            "DYNAMICS_AUDIT",
            metrics={
                "phase": "complete",
                "timing_status": str(timing.status),
                **_progress_audit_metrics(dynamics),
            },
            route_index=route_index if routes else None,
            route_count=len(routes),
            diagnostic_path=path)
        round_record["dynamics_passed"] = bool(dynamics.passed)
        round_record["dynamics_reason"] = dynamics.reason
        if not dynamics.passed:
            search["rounds"].append(round_record)
            record_failure(
                "DYNAMICS", dynamics.reason or "DYNAMIC_AUDIT_REJECTED")
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
        if remaining() <= 0.0:
            search["rounds"].append(round_record)
            return budget_stop("collision audit")
        progress.emit(
            "OBB_COLLISION_AUDIT", metrics={"phase": "running"},
            route_index=route_index if routes else None,
            route_count=len(routes),
            diagnostic_path=path)
        collision_started = time.monotonic()
        collision = audit_collision(
            request, plan, grid, collision_config)
        timings["collision_audit_s"] += time.monotonic() - collision_started
        last_collision_audit = collision
        progress.emit(
            "OBB_COLLISION_AUDIT",
            metrics={
                "phase": "complete",
                **_progress_audit_metrics(collision),
            },
            route_index=route_index if routes else None,
            route_count=len(routes),
            diagnostic_path=path)
        round_record["collision_passed"] = bool(collision.passed)
        round_record["collision_reason"] = collision.reason
        round_record["collision_first_failure"] = collision.first_failure
        if collision.passed:
            search["rounds"].append(round_record)
            break
        record_failure(
            "COLLISION", collision.reason or "COLLISION_AUDIT_REJECTED")
        collision_failure = _empty_result(
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
        if hold_request or not can_recover():
            search["rounds"].append(round_record)
            return collision_failure
        next_index = untried_route_index()
        if next_index is not None:
            # OBB failure: prefer one already generated, still untried route.
            route_switch(collision.reason, next_index)
            search["rounds"].append(round_record)
            continue
        target_context = _collision_retry_context(
            collision, plan, digest, routes[route_index],
            int(geometry_config.intervals))
        if not target_context.blocked_points:
            if collision.reason == "COLLISION_NUMERICAL_FAILURE" \
                    and not endpoint_only and can_recover():
                # Numerical non-certification, not a located contact: spend
                # the single recovery on the remaining initial family once.
                # The G2 no-route sequence is a true endpoint-only set; in
                # corridor mode the hard halfspaces still apply.
                endpoint_only_recovery(collision.reason)
                search["rounds"].append(round_record)
                continue
            search["rounds"].append(round_record)
            return collision_failure
        # No untried route remains: one explicitly heuristic local block,
        # derived only from the real collision position.  The regenerated
        # guide and every audit remain authoritative.
        recovery_count += 1
        search["recovery_count"] = recovery_count
        search["recoveries"].append({
            "recovery_index": recovery_count,
            "action": "collision_local_block",
            "first_failure_layer": first_failure_layer,
            "reason": str(collision.reason or ""),
            "from_route_index": int(route_index),
            "to_route_index": None,
            "wall_time_s": time.monotonic() - started,
        })
        obstacle_context = target_context
        # A new topological guide is regenerated with the failed formation
        # centre as a blocked disk on the next loop iteration.
        routes = ()
        tried_routes = set()
        route_index = 0
        corridor_route_index = None
        search["rounds"].append(round_record)

    # Every layer passed: publish the exact plan that was audited.  No field
    # is added to the timed model after the dynamic/collision audits.
    try:
        model_sha = plan.model_sha256()
        plan_context_sha = plan.planning_context_sha256()
        request_sha = plan.request_sha256
    except Exception as exc:
        record_failure("MODEL", "MODEL_BINDING_FAILURE")
        return _empty_result(
            map_request, "NUMERICAL_FAILURE", "MODEL_BINDING_FAILURE",
            "{}: {}".format(type(exc).__name__, exc), "MODEL", timings,
            time.monotonic() - started, search, attempts=all_attempts,
            routes=tuple(routes), geometry_audit=last_geometry_audit,
            dynamic_audit=last_dynamic_audit,
            collision_audit=last_collision_audit)
    if plan_context_sha != context_sha:
        record_failure("MODEL", "CONTEXT_BINDING_FAILURE")
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
    corridor_metrics = {}
    corridor_detail = ""
    if corridor_mode and containment is not None:
        corridor_metrics = {
            "corridor_effective_margin_m": corridor_effective_margin(
                chain, planner_config),
            "corridor_containment_passed": bool(containment["passed"]),
            "corridor_containment_certified": bool(
                containment["passed"] and corridor_enforcement == "hard"),
            "corridor_containment_samples": int(containment["sample_count"]),
            "corridor_containment_max_bound_residual_m": containment[
                "max_bound_residual_m"],
            **_corridor_evidence(geometry),
        }
        corridor_detail = (
            "; continuous corridor containment certified"
            if corridor_metrics["corridor_containment_certified"] else
            "; corridor containment diagnostic only")
    try:
        result = VirtualAResult(
            status=status,
            reason="",
            detail="geometry, dynamics and collision audits passed"
            + corridor_detail,
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
                **corridor_metrics,
            },
            checks=checks)
    except VirtualAContractError as exc:
        record_failure("MODEL", "RESULT_ASSEMBLY_FAILURE")
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
    "MAX_RECOVERIES",
    "PROGRESS_SCHEMA",
    "PROGRESS_STAGES",
    "PROGRESS_STAGE_INDEX",
    "PROGRESS_STATES",
    "make_route_guides",
    "plan_virtual_a",
]