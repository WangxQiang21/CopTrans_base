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


def _segment_box_nearest_pair(a, b, box):
    """Exact nearest pair between a closed segment and a closed AABB.

    Returns ``(distance, segment_point, box_point)``.  If the segment intersects
    or touches the box the distance is zero and both points are ``None``.
    Otherwise the minimum is attained either at a segment endpoint projected
    onto a box edge or at a box vertex projected onto the segment.
    """
    if _segment_intersects_box(a, b, box):
        return 0.0, None, None
    best = math.inf
    best_segment = None
    best_box = None
    for endpoint in (a, b):
        for edge_start, edge_end in _box_edges(box):
            projected = _nearest_on_segment(endpoint, edge_start, edge_end)
            distance = math.hypot(
                endpoint[0] - projected[0], endpoint[1] - projected[1])
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
    """Minimum distance from a segment inside the map to the map boundary."""
    boundary_edges = _box_edges((
        grid.ox, grid.oy,
        grid.ox + grid.width * grid.res,
        grid.oy + grid.height * grid.res))
    clearance = math.inf
    for endpoint in (p0, p1):
        for edge_start, edge_end in boundary_edges:
            projected = _nearest_on_segment(endpoint, edge_start, edge_end)
            clearance = min(clearance, math.hypot(
                endpoint[0] - projected[0],
                endpoint[1] - projected[1]))
    for corner in _box_corners((
            grid.ox, grid.oy,
            grid.ox + grid.width * grid.res,
            grid.oy + grid.height * grid.res)):
        projected = _nearest_on_segment(corner, p0, p1)
        clearance = min(clearance, math.hypot(
            corner[0] - projected[0], corner[1] - projected[1]))
    return clearance


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
    for row, column in zip(rows.tolist(), columns.tolist()):
        iy = iy0 + row
        ix = ix0 + column
        box = (
            grid.ox + ix * res, grid.oy + iy * res,
            grid.ox + (ix + 1) * res, grid.oy + (iy + 1) * res)
        distance, _segment_point, _box_point = _segment_box_nearest_pair(
            start, end, box)
        if distance <= margin + _TOUCH_TOL_M:
            return False
    return True


def simplify_route(grid, route):
    """Greedy farthest-visible simplification of one ``GuideRoute``.

    The original A* adjacency is verified first; a diagonal or straight segment
    that touches an obstacle cell rejects the whole route instead of assuming
    the A* cell path was geometrically legal.  Endpoints, source and grid hash
    are preserved.
    """
    if not isinstance(route, GuideRoute):
        raise VirtualAContractError(
            "INVALID_GUIDE_ROUTE", "expected GuideRoute")
    digest = grid_signature(grid)
    if route.grid_sha256 != digest:
        raise VirtualAContractError(
            "ROUTE_GRID_MISMATCH",
            "guide grid_sha256 does not match the authoritative grid")
    points = route.points
    for index in range(len(points) - 1):
        if not segment_is_free(grid, points[index], points[index + 1]):
            raise VirtualAContractError(
                "ROUTE_SEGMENT_BLOCKED",
                "adjacent A* segment {}->{} is not exactly free".format(
                    index, index + 1),
                segment=index)
    kept = [points[0]]
    index = 0
    last = len(points) - 1
    while index < last:
        target = last
        while target > index + 1 and not segment_is_free(
                grid, points[index], points[target]):
            target -= 1
        kept.append(points[target])
        index = target
    return GuideRoute(
        points=tuple(kept),
        source=route.source,
        grid_sha256=route.grid_sha256)


__all__ = ["segment_is_free", "simplify_route"]