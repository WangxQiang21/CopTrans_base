"""Exact line-of-sight tests and greedy simplification of guide routes.

A ``GuideRoute`` is a finite topology hint, never an executable path.  This
module decides visibility only from the authoritative closed occupancy cells:
unknown and out-of-map cells are obstacles, and touching a cell edge or corner
is rejected.  The distance field used by A* is never accepted as a certificate.

The only geometric primitive is an exact segment/AABB nearest pair (endpoint
to box edge plus box vertex to segment projection, with a prior closed-slab
intersection test), shared with :mod:`corridor` for corridor construction.
"""
import math

import numpy as np

from .contracts import GuideRoute, VirtualAContractError, grid_signature


# Exact-touch tolerance.  A segment closer than this to an obstacle boundary is
# treated as touching; one picometre is far below any map resolution and guards
# the floating result of an exactly tangent projection.
_TOUCH_TOL_M = 1e-12


def _nearest_on_segment(point, a, b):
    """Closest point on the closed segment ``a-b`` to ``point``."""
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    length2 = dx * dx + dy * dy
    if length2 <= 0.0:
        return ax, ay
    t = ((point[0] - ax) * dx + (point[1] - ay) * dy) / length2
    if t <= 0.0:
        return ax, ay
    if t >= 1.0:
        return bx, by
    return ax + t * dx, ay + t * dy


def _segment_intersects_box(a, b, box):
    """Closed-slab test: touching the box boundary counts as intersection."""
    x0, y0, x1, y1 = box
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, ax - x0), (dx, x1 - ax),
                 (-dy, ay - y0), (dy, y1 - ay)):
        if p == 0.0:
            if q < 0.0:
                return False
            continue
        ratio = q / p
        if p < 0.0:
            if ratio > t1:
                return False
            if ratio > t0:
                t0 = ratio
        else:
            if ratio < t0:
                return False
            if ratio < t1:
                t1 = ratio
    return True


def _box_edges(box):
    x0, y0, x1, y1 = box
    return (
        ((x0, y0), (x1, y0)),
        ((x1, y0), (x1, y1)),
        ((x1, y1), (x0, y1)),
        ((x0, y1), (x0, y0)),
    )


def _box_corners(box):
    x0, y0, x1, y1 = box
    return ((x0, y0), (x1, y0), (x1, y1), (x0, y1))


def _point_box_nearest(point, box):
    """Exact nearest point on a closed AABB to a point.

    For a point outside the box this is the standard clamp of each coordinate;
    for a point inside it returns the point itself.  Callers only use the
    inside case after their closed-slab test already reported an intersection.
    """
    px, py = point
    x0, y0, x1, y1 = box
    qx = x0 if px < x0 else (x1 if px > x1 else px)
    qy = y0 if py < y0 else (y1 if py > y1 else py)
    return math.hypot(px - qx, py - qy), (qx, qy)


def _axis_aligned_box_nearest_pair(a, b, box):
    """Exact nearest pair for a horizontal or vertical closed segment.

    The generic endpoint-clamp plus four-corner test reduces, for an
    axis-aligned segment, to the two endpoint clamps and at most two box
    boundary projections along the segment's free coordinate.  The closed-slab
    intersection is handled inline with the same touch semantics.
    """
    x0, y0, x1, y1 = box
    if a[1] == b[1]:
        y = a[1]
        lo, hi = (a[0], b[0]) if a[0] <= b[0] else (b[0], a[0])
        if y0 <= y <= y1 and not (hi < x0 or lo > x1):
            return 0.0, None, None
    else:
        x = a[0]
        lo, hi = (a[1], b[1]) if a[1] <= b[1] else (b[1], a[1])
        if x0 <= x <= x1 and not (hi < y0 or lo > y1):
            return 0.0, None, None
    best = math.inf
    best_segment = None
    best_box = None
    for endpoint in (a, b):
        distance, projected = _point_box_nearest(endpoint, box)
        if distance < best:
            best = distance
            best_segment = endpoint
            best_box = projected
    if a[1] == b[1]:
        y = a[1]
        edge_y = y0 if y < y0 else (y1 if y > y1 else y)
        if edge_y != y:
            for edge_x in (x0, x1):
                if lo <= edge_x <= hi:
                    distance = abs(y - edge_y)
                    if distance < best:
                        best = distance
                        best_segment = (edge_x, y)
                        best_box = (edge_x, edge_y)
    else:
        x = a[0]
        edge_x = x0 if x < x0 else (x1 if x > x1 else x)
        if edge_x != x:
            for edge_y in (y0, y1):
                if lo <= edge_y <= hi:
                    distance = abs(x - edge_x)
                    if distance < best:
                        best = distance
                        best_segment = (x, edge_y)
                        best_box = (edge_x, edge_y)
    return best, best_segment, best_box


def _segment_box_nearest_pair(a, b, box):
    """Exact nearest pair between a closed segment and a closed AABB.

    Returns ``(distance, segment_point, box_point)``.  If the segment intersects
    or touches the box the distance is zero and both points are ``None``.
    Otherwise the minimum is attained either at a segment endpoint projected
    onto the box (coordinate clamp) or at a box vertex projected onto the
    segment.  Both cases are evaluated exactly; the clamp is algebraically the
    same nearest point as the previous per-edge endpoint projections but avoids
    four redundant edge projections per endpoint.  Axis-aligned segments use
    the exact closed-form specialization above.
    """
    if a[0] == b[0] or a[1] == b[1]:
        return _axis_aligned_box_nearest_pair(a, b, box)
    if _segment_intersects_box(a, b, box):
        return 0.0, None, None
    best = math.inf
    best_segment = None
    best_box = None
    for endpoint in (a, b):
        distance, projected = _point_box_nearest(endpoint, box)
        if distance < best:
            best = distance
            best_segment = endpoint
            best_box = projected
    for corner in _box_corners(box):
        projected = _nearest_on_segment(corner, a, b)
        distance = math.hypot(
            corner[0] - projected[0], corner[1] - projected[1])
        if distance < best:
            best = distance
            best_segment = projected
            best_box = corner
    return best, best_segment, best_box


def _map_boundary_clearance(p0, p1, grid):
    """Minimum distance from a segment inside the map to the map boundary.

    ``p0`` and ``p1`` must lie inside the closed map rectangle (both callers
    check this first).  Distance to the rectangle boundary is the minimum of
    four affine functions, hence concave on that convex rectangle; the minimum
    of a concave function on a segment is attained at an endpoint.  Evaluating
    the endpoint boundary distances is therefore exact and avoids the previous
    edge/corner projections.
    """
    ox, oy = grid.ox, grid.oy
    right = ox + grid.width * grid.res
    top = oy + grid.height * grid.res
    x0, y0 = p0
    x1, y1 = p1
    return min(x0 - ox, right - x0, y0 - oy, top - y0,
               x1 - ox, right - x1, y1 - oy, top - y1)


def _validate_los_query(grid, p0, p1, margin_m):
    if not hasattr(grid, "occ") or not hasattr(grid, "res"):
        raise VirtualAContractError(
            "INVALID_LOS_GRID", "expected an authoritative Grid")
    try:
        start = (float(p0[0]), float(p0[1]))
        end = (float(p1[0]), float(p1[1]))
        margin = float(margin_m)
    except (TypeError, ValueError, IndexError) as exc:
        raise VirtualAContractError(
            "INVALID_LOS_QUERY", "points and margin must be real") from exc
    for value in start + end + (margin,):
        if not math.isfinite(value):
            raise VirtualAContractError(
                "INVALID_LOS_QUERY", "LOS values must be finite")
    if margin < 0.0:
        raise VirtualAContractError(
            "INVALID_LOS_QUERY", "margin_m must be nonnegative")
    return start, end, margin


def build_occupied_index(grid):
    """Return ``(centres, boxes)`` for every occupied cell.

    The centres are only a conservative candidate filter for
    :func:`segment_min_clearance`; every reported distance is still produced
    by the shared exact nearest-pair primitive.
    """
    rows, columns = np.nonzero(np.asarray(grid.occ, dtype=bool))
    res = float(grid.res)
    x0 = grid.ox + columns * res
    y0 = grid.oy + rows * res
    boxes = np.column_stack((x0, y0, x0 + res, y0 + res))
    centres = np.column_stack((x0 + 0.5 * res, y0 + 0.5 * res))
    return centres, boxes


def segment_min_clearance(grid, p0, p1, occupied_index=None):
    """Exact minimum distance from a segment to occupied cells and the map.

    Returns metres, or ``None`` when an endpoint lies outside the map.  A
    segment that intersects or touches an occupied cell or the map boundary
    reports ``0.0``.  Cell centres are a conservative filter: a box whose
    exact distance is ``d`` has its centre at most ``d + sqrt(2) r / 2`` from
    the segment.  Candidates are visited in increasing centre distance and the
    scan stops when no remaining box can beat the current exact minimum.
    """
    start, end, _margin = _validate_los_query(grid, p0, p1, 0.0)
    right = grid.ox + grid.width * grid.res
    top = grid.oy + grid.height * grid.res
    for x, y in (start, end):
        if x < grid.ox or x > right or y < grid.oy or y > top:
            return None
    best = float(_map_boundary_clearance(start, end, grid))
    if best == 0.0:
        return 0.0
    if occupied_index is None:
        occupied_index = build_occupied_index(grid)
    centres, boxes = occupied_index
    if not len(boxes):
        return best
    res = float(grid.res)
    half_diagonal = 0.5 * math.sqrt(2.0) * res
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length2 = dx * dx + dy * dy
    cx = centres[:, 0]
    cy = centres[:, 1]
    if length2 > 0.0:
        t = np.clip(
            ((cx - start[0]) * dx + (cy - start[1]) * dy) / length2, 0.0, 1.0)
        px = start[0] + t * dx
        py = start[1] + t * dy
    else:
        px = start[0]
        py = start[1]
    centre_distance = np.hypot(cx - px, cy - py)
    candidates = np.nonzero(centre_distance <= best + half_diagonal)[0]
    order = candidates[np.argsort(centre_distance[candidates])]
    for index in order.tolist():
        if centre_distance[index] > best + half_diagonal:
            break
        distance, _segment_point, _box_point = _segment_box_nearest_pair(
            start, end, tuple(boxes[index]))
        if distance < best:
            best = float(distance)
            if best == 0.0:
                break
    return best


def segment_is_free(grid, p0, p1, margin_m=0.0):
    """Exact closed-cell segment test with a strict ``margin_m`` clearance.

    The segment must stay strictly farther than ``margin_m`` from every
    occupied cell and from the map boundary.  A zero-length segment is treated
    as a point.
    """
    start, end, margin = _validate_los_query(grid, p0, p1, margin_m)
    right = grid.ox + grid.width * grid.res
    top = grid.oy + grid.height * grid.res
    for x, y in (start, end):
        if x < grid.ox or x > right or y < grid.oy or y > top:
            return False
    if _map_boundary_clearance(start, end, grid) <= margin + _TOUCH_TOL_M:
        return False
    width = grid.width
    height = grid.height
    res = float(grid.res)
    x_min = min(start[0], end[0]) - margin
    x_max = max(start[0], end[0]) + margin
    y_min = min(start[1], end[1]) - margin
    y_max = max(start[1], end[1]) + margin
    ix0 = max(0, int(math.floor((x_min - grid.ox) / res)) - 1)
    ix1 = min(width - 1, int(math.floor((x_max - grid.ox) / res)) + 1)
    iy0 = max(0, int(math.floor((y_min - grid.oy) / res)) - 1)
    iy1 = min(height - 1, int(math.floor((y_max - grid.oy) / res)) + 1)
    if ix1 < ix0 or iy1 < iy0:
        return True
    block = np.asarray(grid.occ[iy0:iy1 + 1, ix0:ix1 + 1], dtype=bool)
    if not block.any():
        return True
    rows, columns = np.nonzero(block)
    # Axis-aligned bounding box of the actual segment.  A box whose distance
    # from this bbox already exceeds the margin cannot violate it; that lower
    # bound is exact for the bbox and conservative for the segment inside it.
    seg_x0 = min(start[0], end[0])
    seg_x1 = max(start[0], end[0])
    seg_y0 = min(start[1], end[1])
    seg_y1 = max(start[1], end[1])
    limit = margin + _TOUCH_TOL_M
    for row, column in zip(rows.tolist(), columns.tolist()):
        iy = iy0 + row
        ix = ix0 + column
        box = (
            grid.ox + ix * res, grid.oy + iy * res,
            grid.ox + (ix + 1) * res, grid.oy + (iy + 1) * res)
        gap_x = max(box[0] - seg_x1, seg_x0 - box[2], 0.0)
        gap_y = max(box[1] - seg_y1, seg_y0 - box[3], 0.0)
        if math.hypot(gap_x, gap_y) > limit:
            continue
        distance, _segment_point, _box_point = _segment_box_nearest_pair(
            start, end, box)
        if distance <= limit:
            return False
    return True


def simplify_route(grid, route, margin_m=0.0):
    """Greedy farthest-visible simplification of one ``GuideRoute``.

    The original A* adjacency is verified first; a diagonal or straight segment
    that touches an obstacle cell rejects the whole route instead of assuming
    the A* cell path was geometrically legal.  Every adjacency and shortcut
    uses the same strict ``margin_m`` as the route search when the caller
    supplies it; the default keeps the historical zero-margin behaviour for
    non-route callers.  Endpoints, source and grid hash are preserved.
    """
    if not isinstance(route, GuideRoute):
        raise VirtualAContractError(
            "INVALID_GUIDE_ROUTE", "expected GuideRoute")
    try:
        margin = float(margin_m)
    except (TypeError, ValueError) as exc:
        raise VirtualAContractError(
            "INVALID_GUIDE_ROUTE", "margin_m must be real") from exc
    if not math.isfinite(margin) or margin < 0.0:
        raise VirtualAContractError(
            "INVALID_GUIDE_ROUTE",
            "margin_m must be finite and nonnegative")
    digest = grid_signature(grid)
    if route.grid_sha256 != digest:
        raise VirtualAContractError(
            "ROUTE_GRID_MISMATCH",
            "guide grid_sha256 does not match the authoritative grid")
    points = route.points
    for index in range(len(points) - 1):
        if not segment_is_free(
                grid, points[index], points[index + 1], margin):
            raise VirtualAContractError(
                "ROUTE_SEGMENT_BLOCKED",
                "adjacent A* segment {}->{} is not exactly free above "
                "margin_m={}".format(index, index + 1, margin),
                segment=index)
    kept = [points[0]]
    index = 0
    last = len(points) - 1
    while index < last:
        target = last
        while target > index + 1 and not segment_is_free(
                grid, points[index], points[target], margin):
            target -= 1
        kept.append(points[target])
        index = target
    return GuideRoute(
        points=tuple(kept),
        source=route.source,
        grid_sha256=route.grid_sha256)


__all__ = [
    "build_occupied_index",
    "segment_is_free",
    "segment_min_clearance",
    "simplify_route",
]