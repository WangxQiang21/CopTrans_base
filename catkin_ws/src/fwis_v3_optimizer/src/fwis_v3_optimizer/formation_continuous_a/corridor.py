"""Separating-hyperplane convex corridors and their independent audit.

For every seed segment of a guide route this module builds a small local box
``B`` (the seed bounding box expanded by ``box_padding_m``, clipped to the map
with a 1e-8 m inset).  Every occupied closed cell intersecting ``B`` contributes
one halfspace whose unit normal is the exact seed/cell nearest-pair direction.
The corridor is ``B`` intersected with all halfspaces, stored as the canonical
counter-clockwise polygon together with that polygon's minimal edge
halfspaces (an exact, equivalent replacement for the generating set).

This is reference-point geometry only.  ``d_safe`` is zero: a corridor says
nothing about the three-vehicle swept OBB safety, which remains with the
unchanged final audit.  The independent auditor deliberately does not trust the
generating halfspaces for obstacle containment: it traverses all original
occupied cells intersecting each polygon's bounding box and applies a
polygon-versus-cell SAT, treating a touching edge as a collision.
"""
import bisect
import math
import numbers
from collections.abc import Mapping as MappingABC

import numpy as np
from scipy.optimize import linprog, minimize

from .contracts import (
    ConvexCorridor,
    CorridorChain,
    GuideRoute,
    VirtualAContractError,
    grid_signature,
)
from .geometry import VirtualAPath
from .route_processing import _box_corners, _segment_box_nearest_pair


_MAP_INSET_M = 1e-8
_PLANE_INSET_M = 1e-8
_RHO_MAX_M = 0.05
_MIN_REGION_AREA_M2 = 1e-10
_SEED_VIOLATION_TOL_M = 1e-9
_TOUCH_TOL_M = 1e-9
_VERTEX_DEDUP_M = 1e-12
_HALFSPACE_MATCH_TOL_M = 1e-9
_MIN_PORTAL_RADIUS_FLOOR_M = 0.005
_MIN_PORTAL_RADIUS_CELL_FRACTION = 0.1
_GUIDE_POINT_DEDUP_M = 1e-8
_GUIDE_MIN_SEED_LENGTH_M = 0.1
_GUIDE_PRIMAL_TOL_M = 1e-8
_GUIDE_QP_MAXITER = 300
_GUIDE_QP_FTOL = 1e-9
_ALLOCATION_ZERO_M = 1e-15


class _DegenerateSeed(Exception):
    """Internal one-attempt rejection of a seed; converted to a contract error."""


def _portal_threshold_m(grid):
    return min(_MIN_PORTAL_RADIUS_FLOOR_M,
               _MIN_PORTAL_RADIUS_CELL_FRACTION * float(grid.res))


def _clip_polygon(vertices, axis, limit):
    """Clip a convex polygon by ``axis . p <= limit`` (Sutherland-Hodgman)."""
    if len(vertices) < 3:
        return []
    kept = []
    count = len(vertices)
    for index in range(count):
        point = vertices[index]
        following = vertices[(index + 1) % count]
        point_value = axis[0] * point[0] + axis[1] * point[1] - limit
        following_value = (
            axis[0] * following[0] + axis[1] * following[1] - limit)
        point_inside = point_value <= 0.0
        following_inside = following_value <= 0.0
        if point_inside:
            kept.append(point)
        if point_inside != following_inside:
            denominator = point_value - following_value
            if denominator != 0.0:
                ratio = point_value / denominator
                kept.append((
                    point[0] + ratio * (following[0] - point[0]),
                    point[1] + ratio * (following[1] - point[1])))
    result = []
    for point in kept:
        if not result or math.hypot(
                point[0] - result[-1][0],
                point[1] - result[-1][1]) > _VERTEX_DEDUP_M:
            result.append(point)
    if len(result) > 1 and math.hypot(
            result[0][0] - result[-1][0],
            result[0][1] - result[-1][1]) <= _VERTEX_DEDUP_M:
        result.pop()
    return result


def _canonical_vertices(vertices):
    """Return ``(counter-clockwise canonical vertices, area)`` or ``None``."""
    points = list(vertices)
    cleaned = []
    for point in points:
        if not cleaned or math.hypot(
                point[0] - cleaned[-1][0],
                point[1] - cleaned[-1][1]) > _VERTEX_DEDUP_M:
            cleaned.append(point)
    if len(cleaned) > 1 and math.hypot(
            cleaned[0][0] - cleaned[-1][0],
            cleaned[0][1] - cleaned[-1][1]) <= _VERTEX_DEDUP_M:
        cleaned.pop()
    if len(cleaned) < 3:
        return None
    twice_area = sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(
            cleaned, cleaned[1:] + cleaned[:1]))
    if twice_area == 0.0:
        return None
    if twice_area < 0.0:
        cleaned.reverse()
        twice_area = -twice_area
    start = min(range(len(cleaned)), key=lambda index: cleaned[index])
    cleaned = cleaned[start:] + cleaned[:start]
    return tuple(cleaned), 0.5 * twice_area


def _edge_constraints(vertices):
    """Outward halfspaces of a counter-clockwise convex polygon."""
    constraints = []
    for point, following in zip(vertices, vertices[1:] + vertices[:1]):
        dx = following[0] - point[0]
        dy = following[1] - point[1]
        length = math.hypot(dx, dy)
        if length <= 0.0:
            continue
        normal = (dy / length, -dx / length)
        constraints.append(
            (normal, normal[0] * point[0] + normal[1] * point[1]))
    return constraints


def _signed_twice_area(vertices):
    return sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(
            vertices, vertices[1:] + vertices[:1]))


def _seed_box(grid, start, end, padding):
    left = grid.ox + _MAP_INSET_M
    bottom = grid.oy + _MAP_INSET_M
    right = grid.ox + grid.width * grid.res - _MAP_INSET_M
    top = grid.oy + grid.height * grid.res - _MAP_INSET_M
    if right <= left or top <= bottom:
        raise _DegenerateSeed("map is too small for a corridor box")
    x0 = max(left, min(start[0], end[0]) - padding)
    x1 = min(right, max(start[0], end[0]) + padding)
    y0 = max(bottom, min(start[1], end[1]) - padding)
    y1 = min(top, max(start[1], end[1]) + padding)
    if x1 <= x0 or y1 <= y0:
        raise _DegenerateSeed("empty corridor box after map clipping")
    return x0, y0, x1, y1


def _occupied_cells(grid, x0, y0, x1, y1):
    """All occupied cells whose closed square touches the search box.

    One extra cell on every index boundary is included on purpose: a cell that
    merely touches ``B`` is still inside the closed expanded range and must not
    be skipped by an unproved distance argument.
    """
    res = float(grid.res)
    ix0 = max(0, int(math.floor((x0 - grid.ox) / res)) - 1)
    ix1 = min(grid.width - 1, int(math.floor((x1 - grid.ox) / res)) + 1)
    iy0 = max(0, int(math.floor((y0 - grid.oy) / res)) - 1)
    iy1 = min(grid.height - 1, int(math.floor((y1 - grid.oy) / res)) + 1)
    if ix1 < ix0 or iy1 < iy0:
        return
    block = np.asarray(grid.occ[iy0:iy1 + 1, ix0:ix1 + 1], dtype=bool)
    rows, columns = np.nonzero(block)
    for row, column in zip(rows.tolist(), columns.tolist()):
        yield iy0 + row, ix0 + column


def _cell_box(grid, iy, ix):
    res = float(grid.res)
    return (
        grid.ox + ix * res, grid.oy + iy * res,
        grid.ox + (ix + 1) * res, grid.oy + (iy + 1) * res)


def _build_region(grid, start, end, padding):
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    if length <= _VERTEX_DEDUP_M:
        raise _DegenerateSeed("zero-length seed")
    x0, y0, x1, y1 = _seed_box(grid, start, end, padding)
    constraints = [
        ((-1.0, 0.0), -x0),
        ((1.0, 0.0), x1),
        ((0.0, -1.0), -y0),
        ((0.0, 1.0), y1),
    ]
    pairs = []
    for iy, ix in _occupied_cells(grid, x0, y0, x1, y1):
        distance, segment_point, box_point = _segment_box_nearest_pair(
            start, end, _cell_box(grid, iy, ix))
        if segment_point is None or distance <= 0.0:
            raise _DegenerateSeed(
                "seed touches occupied cell ({}, {})".format(iy, ix))
        normal = (
            (box_point[0] - segment_point[0]) / distance,
            (box_point[1] - segment_point[1]) / distance)
        high = max(
            normal[0] * start[0] + normal[1] * start[1],
            normal[0] * end[0] + normal[1] * end[1])
        low = min(
            normal[0] * corner[0] + normal[1] * corner[1]
            for corner in _box_corners(_cell_box(grid, iy, ix)))
        gap = low - high
        if gap <= 0.0:
            raise _DegenerateSeed(
                "occupied cell ({}, {}) is not separated from the seed".format(
                    iy, ix))
        pairs.append((iy, ix, normal, high, low, gap))
    if pairs:
        rho = min(_RHO_MAX_M, min(pair[5] for pair in pairs) / 4.0)
    else:
        rho = 0.0
    for iy, ix, normal, high, low, _gap in sorted(
            pairs, key=lambda pair: (pair[0], pair[1])):
        constraints.append(
            (normal, (high + rho + low) / 2.0 - _PLANE_INSET_M))
    vertices = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    for axis, limit in constraints:
        vertices = _clip_polygon(vertices, axis, limit)
        if len(vertices) < 3:
            raise _DegenerateSeed(
                "empty polygon after clipping by a separating halfspace")
    canonical = _canonical_vertices(vertices)
    if canonical is None:
        raise _DegenerateSeed("empty corridor polygon")
    vertices, area = canonical
    if area <= _MIN_REGION_AREA_M2:
        raise _DegenerateSeed("corridor area {} is too small".format(area))
    violation = 0.0
    for point in (start, end):
        for axis, limit in constraints:
            value = axis[0] * point[0] + axis[1] * point[1] - limit
            if value > violation:
                violation = value
    if violation > _SEED_VIOLATION_TOL_M:
        raise _DegenerateSeed(
            "seed violates a corridor halfspace by {} m".format(violation))
    # The clipped polygon is exactly the intersection of every generated
    # halfspace.  Store its minimal edge halfspaces rather than the whole
    # O(occupied cells) generating set: both define the same region, but the
    # NLP then replicates a handful of rows per sample instead of thousands.
    edge_constraints = _edge_constraints(vertices)
    return ConvexCorridor(
        seed_start=start,
        seed_end=end,
        A=tuple(axis for axis, _limit in edge_constraints),
        b=tuple(limit for _axis, limit in edge_constraints),
        vertices=vertices)


def _chebyshev_center(rows, limits):
    """Maximise the certified inscribed radius over ``A p + ||A_row|| r <= b``.

    Returns ``(certified_radius, (x, y))``.  The radius is recomputed from the
    LP center's actual slacks instead of trusting the solver objective, which
    can be optimistic within the HiGHS primal feasibility tolerance.
    """
    if not rows:
        return None
    normals = [
        (float(row[0]), float(row[1]), math.hypot(row[0], row[1]))
        for row in rows]
    if any(normal[2] <= 0.0 for normal in normals):
        return None
    matrix = np.asarray(normals, dtype=float)
    values = np.asarray([float(value) for value in limits], dtype=float)
    result = linprog(
        np.asarray([0.0, 0.0, -1.0]),
        A_ub=matrix, b_ub=values,
        bounds=((None, None), (None, None), (0.0, None)),
        method="highs")
    if not result.success:
        return None
    x = float(result.x[0])
    y = float(result.x[1])
    radius = min(
        (value - normal[0] * x - normal[1] * y) / normal[2]
        for normal, value in zip(normals, values))
    if not math.isfinite(radius):
        return None
    return radius, (x, y)


def _intersection_vertices(vertices, constraints):
    clipped = list(vertices)
    for axis, limit in constraints:
        clipped = _clip_polygon(clipped, axis, limit)
        if len(clipped) < 3:
            return (), 0.0
    canonical = _canonical_vertices(clipped)
    if canonical is None:
        return (), 0.0
    return canonical


def _portal_measure(first, second):
    """Generation-side portal measure from the stored halfspaces."""
    radius = _chebyshev_center(first.A + second.A, first.b + second.b)
    vertices, area = _intersection_vertices(
        first.vertices, tuple(zip(second.A, second.b)))
    return radius, area, vertices


def _audit_portal_measure(first, second):
    """Audit-side portal measure from polygon vertices, not stored halfspaces."""
    vertices, area = _intersection_vertices(
        first.vertices, _edge_constraints(second.vertices))
    if not vertices:
        return None, area, ()
    portal_constraints = _edge_constraints(vertices)
    radius = _chebyshev_center(
        [axis for axis, _limit in portal_constraints],
        [limit for _axis, limit in portal_constraints])
    return radius, area, vertices


def _build_regions(grid, seeds, split_flags, padding):
    regions = []
    owners = []
    for index, (start, end) in enumerate(seeds):
        if not split_flags[index]:
            try:
                regions.append(_build_region(grid, start, end, padding))
                owners.append(index)
                continue
            except _DegenerateSeed:
                split_flags[index] = True
        middle = (
            0.5 * (start[0] + end[0]), 0.5 * (start[1] + end[1]))
        if math.hypot(middle[0] - start[0], middle[1] - start[1]) \
                <= _VERTEX_DEDUP_M:
            raise VirtualAContractError(
                "CORRIDOR_DEGENERATE",
                "seed {} cannot be split at its midpoint".format(index),
                seed=index)
        for half_start, half_end in ((start, middle), (middle, end)):
            try:
                regions.append(_build_region(
                    grid, half_start, half_end, padding))
            except _DegenerateSeed as exc:
                raise VirtualAContractError(
                    "CORRIDOR_DEGENERATE",
                    "seed {} half after one split: {}".format(index, exc),
                    seed=index) from exc
            owners.append(index)
    return regions, owners


def build_corridor_chain(grid, guide, *, box_padding_m=2.0):
    """Build one convex corridor per guide seed, with a single split retry."""
    if not isinstance(guide, GuideRoute):
        raise VirtualAContractError(
            "INVALID_GUIDE_ROUTE", "expected GuideRoute")
    try:
        padding = float(box_padding_m)
    except (TypeError, ValueError) as exc:
        raise VirtualAContractError(
            "INVALID_CORRIDOR_REQUEST",
            "box_padding_m must be real") from exc
    if not math.isfinite(padding) or padding < 0.0:
        raise VirtualAContractError(
            "INVALID_CORRIDOR_REQUEST",
            "box_padding_m must be finite and nonnegative")
    digest = grid_signature(grid)
    if guide.grid_sha256 != digest:
        raise VirtualAContractError(
            "CORRIDOR_GRID_MISMATCH",
            "guide grid_sha256 does not match the authoritative grid")
    points = guide.points
    seeds = list(zip(points, points[1:]))
    split_flags = [False] * len(seeds)
    regions, owners = _build_regions(grid, seeds, split_flags, padding)
    threshold = _portal_threshold_m(grid)
    failures = _portal_failures(regions, grid, threshold)
    if failures:
        changed = False
        for index, _reason in failures:
            for owner in (owners[index], owners[index + 1]):
                if not split_flags[owner]:
                    split_flags[owner] = True
                    changed = True
        if changed:
            regions, owners = _build_regions(
                grid, seeds, split_flags, padding)
            failures = _portal_failures(regions, grid, threshold)
        if failures:
            index, reason = failures[0]
            raise VirtualAContractError(
                "CORRIDOR_DEGENERATE",
                "portal {} between regions failed: {}".format(index, reason),
                portal=index)
    return CorridorChain(
        regions=tuple(regions),
        route_source=guide.source,
        grid_sha256=guide.grid_sha256)


def _portal_failures(regions, grid, threshold):
    failures = []
    for index in range(len(regions) - 1):
        radius, area, _vertices = _portal_measure(
            regions[index], regions[index + 1])
        if radius is None:
            failures.append((index, "portal is empty or infeasible"))
            continue
        if area <= _MIN_REGION_AREA_M2:
            failures.append((
                index, "portal area {} is too small".format(area)))
            continue
        if radius[0] < threshold:
            failures.append((
                index,
                "portal radius {} < {} m".format(radius[0], threshold)))
    return failures


def _polygon_separated_from_box(vertices, box):
    """Conservative convex-polygon/closed-cell SAT.

    A projection gap must exceed ``_TOUCH_TOL_M`` on at least one axis; a
    smaller positive gap is treated as touching and therefore as a collision.
    """
    axes = []
    for point, following in zip(vertices, vertices[1:] + vertices[:1]):
        dx = following[0] - point[0]
        dy = following[1] - point[1]
        length = math.hypot(dx, dy)
        if length <= 0.0:
            continue
        axes.append((-dy / length, dx / length))
    axes.append((1.0, 0.0))
    axes.append((0.0, 1.0))
    corners = _box_corners(box)
    for axis in axes:
        polygon = [axis[0] * x + axis[1] * y for x, y in vertices]
        cell = [axis[0] * x + axis[1] * y for x, y in corners]
        if min(polygon) > max(cell) + _TOUCH_TOL_M:
            return True
        if min(cell) > max(polygon) + _TOUCH_TOL_M:
            return True
    return False


def _point_polygon_violation(point, vertices):
    """Outward distance by which a point lies outside a CCW convex polygon."""
    violation = 0.0
    for first, second in zip(vertices, vertices[1:] + vertices[:1]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length = math.hypot(dx, dy)
        if length <= 0.0:
            continue
        cross = dx * (point[1] - first[1]) - dy * (point[0] - first[0])
        if cross < 0.0:
            violation = max(violation, -cross / length)
    return violation


def _audit_region(grid, region, index):
    """Validate one corridor independently of the generator's obstacle proof.

    Stored halfspaces are used only to require that the polygon and the
    halfspace set define the same region; obstacle containment itself is the
    polygon-cell SAT below.
    """
    if len(region.A) != len(region.b):
        return None, "CORRIDOR_INVALID", "A/b length mismatch", None
    for axis in region.A:
        if abs(math.hypot(axis[0], axis[1]) - 1.0) > 1e-9:
            return None, "CORRIDOR_INVALID", "non-unit normal", None
    twice_area = _signed_twice_area(region.vertices)
    if twice_area <= 0.0:
        return None, "CORRIDOR_INVALID", \
            "vertices are not counter-clockwise", None
    area = 0.5 * twice_area
    if area <= _MIN_REGION_AREA_M2:
        return None, "CORRIDOR_DEGENERATE", \
            "region area {} is too small".format(area), None
    scale = 1e-30
    for point, following in zip(region.vertices, region.vertices[1:] + region.vertices[:1]):
        scale = max(scale, (following[0] - point[0]) ** 2
                    + (following[1] - point[1]) ** 2)
    for first, second, third in zip(
            region.vertices,
            region.vertices[1:] + region.vertices[:1],
            region.vertices[2:] + region.vertices[:2]):
        cross = ((second[0] - first[0]) * (third[1] - second[1])
                 - (second[1] - first[1]) * (third[0] - second[0]))
        if cross < -1e-9 * scale:
            return None, "CORRIDOR_INVALID", "polygon is not convex", None
    # Every polygon edge must lie on a stored halfspace boundary.  Together
    # with the vertex check below this makes {p : A p <= b} equal the audited
    # polygon; otherwise a looser stored A,b could expose an obstacle that the
    # polygon SAT never sees, while C2 would consume the looser halfspaces.
    for first, second in zip(
            region.vertices, region.vertices[1:] + region.vertices[:1]):
        supported = False
        for axis, limit in zip(region.A, region.b):
            if (abs(axis[0] * first[0] + axis[1] * first[1] - limit)
                    <= _HALFSPACE_MATCH_TOL_M
                    and abs(axis[0] * second[0] + axis[1] * second[1] - limit)
                    <= _HALFSPACE_MATCH_TOL_M):
                supported = True
                break
        if not supported:
            return None, "CORRIDOR_INVALID", \
                "polygon edge is not on a stored halfspace boundary", None
    right = grid.ox + grid.width * grid.res
    top = grid.oy + grid.height * grid.res
    for x, y in region.vertices:
        if (x < grid.ox or x > right or y < grid.oy or y > top):
            return None, "CORRIDOR_MAP_BOUNDARY", \
                "vertex ({}, {}) is outside the map".format(x, y), None
    for x, y in region.vertices:
        for axis, limit in zip(region.A, region.b):
            if axis[0] * x + axis[1] * y - limit > _SEED_VIOLATION_TOL_M:
                return None, "CORRIDOR_INVALID", \
                    "vertex violates a stored halfspace", None
    violation = 0.0
    for point in (region.seed_start, region.seed_end):
        for axis, limit in zip(region.A, region.b):
            violation = max(
                violation,
                axis[0] * point[0] + axis[1] * point[1] - limit)
        violation = max(
            violation, _point_polygon_violation(point, region.vertices))
    if violation > _SEED_VIOLATION_TOL_M:
        return None, "CORRIDOR_SEED_VIOLATION", \
            "seed violates a corridor by {} m".format(violation), None
    minimum_x = min(x for x, _y in region.vertices)
    maximum_x = max(x for x, _y in region.vertices)
    minimum_y = min(y for _x, y in region.vertices)
    maximum_y = max(y for _x, y in region.vertices)
    res = float(grid.res)
    ix0 = max(0, int(math.floor(
        (minimum_x - grid.ox - _TOUCH_TOL_M) / res)))
    ix1 = min(grid.width - 1, int(math.floor(
        (maximum_x - grid.ox + _TOUCH_TOL_M) / res)))
    iy0 = max(0, int(math.floor(
        (minimum_y - grid.oy - _TOUCH_TOL_M) / res)))
    iy1 = min(grid.height - 1, int(math.floor(
        (maximum_y - grid.oy + _TOUCH_TOL_M) / res)))
    checked = 0
    first_collision = None
    if ix1 >= ix0 and iy1 >= iy0:
        block = np.asarray(
            grid.occ[iy0:iy1 + 1, ix0:ix1 + 1], dtype=bool)
        rows, columns = np.nonzero(block)
        for row, column in zip(rows.tolist(), columns.tolist()):
            iy = iy0 + row
            ix = ix0 + column
            checked += 1
            if not _polygon_separated_from_box(
                    region.vertices, _cell_box(grid, iy, ix)):
                first_collision = {"region": index, "cell": [iy, ix]}
                break
    if first_collision is not None:
        return None, "CORRIDOR_CELL_OVERLAP", \
            "corridor polygon overlaps occupied cell {}".format(
                first_collision["cell"]), first_collision
    return {
        "index": index,
        "seed_start": [region.seed_start[0], region.seed_start[1]],
        "seed_end": [region.seed_end[0], region.seed_end[1]],
        "area_m2": area,
        "seed_violation_m": violation,
        "vertex_count": len(region.vertices),
        "sat_checked_cells": checked,
    }, None, None, None


def audit_corridor_chain(grid, chain):
    """Independently audit every region and portal of a corridor chain.

    Obstacle containment uses polygon-versus-original-cell SAT over every
    occupied cell in the polygon bounding box, never the generator's halfspace
    list.  The stored halfspaces are separately required to describe exactly
    the same polygon, because C2 consumes ``A, b``.  Map inclusion, seed
    containment, region area, convexity and portal area/radius are checked
    separately.  There is deliberately no three-vehicle ``FEASIBLE`` field
    here.
    """
    result = {
        "passed": False,
        "reason": "",
        "detail": "",
        "grid_sha256": getattr(chain, "grid_sha256", ""),
        "route_source": getattr(chain, "route_source", ""),
        "region_count": 0,
        "regions": [],
        "portals": [],
        "min_region_area_m2": None,
        "max_seed_violation_m": None,
        "min_portal_radius_m": None,
        "min_portal_area_m2": None,
        "first_collision": None,
        "thresholds": {
            "min_region_area_m2": _MIN_REGION_AREA_M2,
            "min_portal_area_m2": _MIN_REGION_AREA_M2,
            "min_portal_radius_m": _portal_threshold_m(grid),
            "seed_violation_m": _SEED_VIOLATION_TOL_M,
            "halfspace_match_m": _HALFSPACE_MATCH_TOL_M,
            "touch_tolerance_m": _TOUCH_TOL_M,
            "map_inset_m": _MAP_INSET_M,
        },
    }
    if not isinstance(chain, CorridorChain):
        result["reason"] = "INVALID_CORRIDOR_CHAIN"
        result["detail"] = "expected CorridorChain"
        return result
    if chain.grid_sha256 != grid_signature(grid):
        result["reason"] = "CORRIDOR_GRID_MISMATCH"
        result["detail"] = "chain grid_sha256 does not match the grid"
        return result
    result["region_count"] = len(chain.regions)
    minimum_area = math.inf
    maximum_violation = 0.0
    for index, region in enumerate(chain.regions):
        if index:
            previous = chain.regions[index - 1].seed_end
            current = region.seed_start
            if math.hypot(previous[0] - current[0],
                          previous[1] - current[1]) > _SEED_VIOLATION_TOL_M:
                result["reason"] = "CORRIDOR_SEED_DISCONTINUITY"
                result["detail"] = "region {} seed does not continue".format(index)
                return result
        metrics, reason, detail, collision = _audit_region(
            grid, region, index)
        if reason is not None:
            result["reason"] = reason
            result["detail"] = detail
            result["first_collision"] = collision
            result["min_region_area_m2"] = (
                None if minimum_area is math.inf else minimum_area)
            result["max_seed_violation_m"] = maximum_violation
            return result
        result["regions"].append(metrics)
        minimum_area = min(minimum_area, metrics["area_m2"])
        maximum_violation = max(
            maximum_violation, metrics["seed_violation_m"])
    result["min_region_area_m2"] = minimum_area
    result["max_seed_violation_m"] = maximum_violation
    threshold = _portal_threshold_m(grid)
    minimum_radius = math.inf
    minimum_portal_area = math.inf
    for index in range(len(chain.regions) - 1):
        radius, area, _vertices = _audit_portal_measure(
            chain.regions[index], chain.regions[index + 1])
        if radius is None:
            result["reason"] = "CORRIDOR_DEGENERATE"
            result["detail"] = "portal {} is empty or infeasible".format(index)
            result["min_portal_radius_m"] = (
                None if minimum_radius is math.inf else minimum_radius)
            result["min_portal_area_m2"] = (
                None if minimum_portal_area is math.inf
                else minimum_portal_area)
            return result
        radius_m, center = radius
        if area <= _MIN_REGION_AREA_M2:
            result["reason"] = "CORRIDOR_DEGENERATE"
            result["detail"] = "portal {} area {} is too small".format(
                index, area)
            result["min_portal_radius_m"] = (
                None if minimum_radius is math.inf else minimum_radius)
            result["min_portal_area_m2"] = (
                None if minimum_portal_area is math.inf
                else minimum_portal_area)
            return result
        if radius_m < threshold:
            result["reason"] = "CORRIDOR_DEGENERATE"
            result["detail"] = "portal {} radius {} < {} m".format(
                index, radius_m, threshold)
            result["min_portal_radius_m"] = (
                None if minimum_radius is math.inf else minimum_radius)
            result["min_portal_area_m2"] = (
                None if minimum_portal_area is math.inf
                else minimum_portal_area)
            return result
        result["portals"].append({
            "index": index,
            "radius_m": radius_m,
            "center": [center[0], center[1]],
            "area_m2": area,
        })
        minimum_radius = min(minimum_radius, radius_m)
        minimum_portal_area = min(minimum_portal_area, area)
    result["min_portal_radius_m"] = (
        None if minimum_radius is math.inf else minimum_radius)
    result["min_portal_area_m2"] = (
        None if minimum_portal_area is math.inf else minimum_portal_area)
    result["passed"] = True
    result["reason"] = "CORRIDOR_CHAIN_VERIFIED"
    return result


def _row_violation(point, rows, limits):
    if not rows:
        return 0.0
    return max(
        0.0,
        max(row[0] * point[0] + row[1] * point[1] - limit
            for row, limit in zip(rows, limits)))


def _portal_bundle(chain, index):
    """Return the Chebyshev centre and both halfspace sets of one portal."""
    first = chain.regions[index - 1]
    second = chain.regions[index]
    rows = first.A + second.A
    limits = first.b + second.b
    measured = _chebyshev_center(rows, limits)
    if measured is None:
        raise VirtualAContractError(
            "CORRIDOR_GUIDE_DEGENERATE",
            "portal {} has no Chebyshev centre".format(index - 1),
            portal=index - 1)
    radius, centre = measured
    return centre, rows, limits, radius


def corridor_min_portal_radius(chain):
    """Minimum Chebyshev portal radius, or ``None`` for a one-region chain."""
    if not isinstance(chain, CorridorChain):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_CHAIN", "expected CorridorChain")
    if not chain.regions:
        raise VirtualAContractError(
            "INVALID_CORRIDOR_CHAIN", "chain has no regions")
    radii = []
    for index in range(1, len(chain.regions)):
        _centre, _rows, _limits, radius = _portal_bundle(chain, index)
        radii.append(float(radius))
    return None if not radii else min(radii)


def corridor_effective_margin(chain, config):
    """The NLP halfspace inset after the portal-radius safety cap."""
    margin = float(config.corridor_margin_m)
    fraction = float(config.corridor_margin_portal_fraction)
    radius = corridor_min_portal_radius(chain)
    if radius is not None:
        margin = min(margin, fraction * radius)
    if not math.isfinite(margin) or margin < 0.0:
        raise VirtualAContractError(
            "INVALID_CORRIDOR_MARGIN",
            "effective margin must be finite and nonnegative")
    return float(margin)


def validate_corridor_allocation(chain, allocation):
    """Validate one ``(sigma_a, sigma_b, corridor_index)`` allocation."""
    if not isinstance(chain, CorridorChain):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_CHAIN", "expected CorridorChain")
    if not allocation:
        raise VirtualAContractError(
            "CORRIDOR_ALLOCATION_DEGENERATE", "allocation is empty")
    result = []
    previous = None
    seen = set()
    for entry in allocation:
        if not isinstance(entry, (tuple, list)) or len(entry) != 3:
            raise VirtualAContractError(
                "CORRIDOR_ALLOCATION_DEGENERATE",
                "allocation entries must be (a, b, corridor_index)")
        try:
            left = float(entry[0])
            right = float(entry[1])
        except (TypeError, ValueError) as exc:
            raise VirtualAContractError(
                "CORRIDOR_ALLOCATION_DEGENERATE",
                "allocation bounds must be real") from exc
        index = entry[2]
        if isinstance(index, bool) or not isinstance(index, numbers.Integral):
            raise VirtualAContractError(
                "CORRIDOR_ALLOCATION_DEGENERATE",
                "corridor_index must be an integer")
        index = int(index)
        if not (math.isfinite(left) and math.isfinite(right)):
            raise VirtualAContractError(
                "CORRIDOR_ALLOCATION_DEGENERATE",
                "allocation bounds must be finite")
        if left < -_ALLOCATION_ZERO_M or right > 1.0 + _ALLOCATION_ZERO_M \
                or right - left <= _ALLOCATION_ZERO_M:
            raise VirtualAContractError(
                "CORRIDOR_ALLOCATION_DEGENERATE",
                "allocation interval is not a positive subset of [0, 1]")
        if index < 0 or index >= len(chain.regions):
            raise VirtualAContractError(
                "CORRIDOR_ALLOCATION_DEGENERATE",
                "corridor_index {} leaves the chain".format(index))
        if previous is not None:
            if abs(previous[1] - left) > _ALLOCATION_ZERO_M:
                raise VirtualAContractError(
                    "CORRIDOR_ALLOCATION_DEGENERATE",
                    "allocation has a gap or reversal")
            if index < previous[2]:
                raise VirtualAContractError(
                    "CORRIDOR_ALLOCATION_DEGENERATE",
                    "allocation skips backward")
        result.append((left, right, index))
        seen.add(index)
        previous = (left, right, index)
    if abs(result[0][0]) > _ALLOCATION_ZERO_M or \
            abs(result[-1][1] - 1.0) > _ALLOCATION_ZERO_M:
        raise VirtualAContractError(
            "CORRIDOR_ALLOCATION_DEGENERATE",
            "allocation does not cover [0, 1]")
    if seen != set(range(len(chain.regions))):
        raise VirtualAContractError(
            "CORRIDOR_ALLOCATION_DEGENERATE",
            "allocation skipped a corridor")
    return tuple(result)


def _guide_region_violation(chain, points):
    """Max halfspace violation of every guide segment in its assigned C."""
    regions = chain.regions
    if len(points) != len(regions) + 1:
        raise VirtualAContractError(
            "CORRIDOR_GUIDE_DEGENERATE",
            "a corridor guide needs one vertex per region endpoint")
    violation = 0.0
    for index, region in enumerate(regions):
        violation = max(
            violation,
            _row_violation(points[index], region.A, region.b),
            _row_violation(points[index + 1], region.A, region.b))
    return violation


def _guide_segment_lengths(points):
    return [math.hypot(
        second[0] - first[0], second[1] - first[1])
        for first, second in zip(points, points[1:])]


def _guide_metrics(chain, points, reason):
    """Validate count, positive lengths and assigned-region containment."""
    regions = chain.regions
    if len(points) != len(regions) + 1:
        raise VirtualAContractError(
            reason,
            "guide has {} points for {} regions".format(
                len(points), len(regions)))
    lengths = _guide_segment_lengths(points)
    if any(length <= _GUIDE_POINT_DEDUP_M for length in lengths):
        raise VirtualAContractError(
            reason, "guide contains a zero-length segment")
    violation = _guide_region_violation(chain, points)
    return lengths, violation


def _guide_qp_terms(vector, start, goal, centres, seed_lengths, l0):
    """C2-A convex QP value and analytic gradient.

    The nine-point Hessian is not assembled: SLSQP receives the exact smooth
    objective and linear portal constraints only.  ``seed_lengths`` are the
    frozen per-region $\ell_j$, never the optimized segment lengths.
    """
    count = len(seed_lengths)
    points = np.empty((count + 1, 2), dtype=float)
    points[0] = start
    points[-1] = goal
    if count > 1:
        points[1:-1] = np.asarray(vector, dtype=float).reshape(count - 1, 2)
    segments = np.diff(points, axis=0)
    length_term = float(np.sum(segments * segments)) / (l0 * l0)
    normalised = segments / np.asarray(seed_lengths, dtype=float)[:, None]
    smooth_residual = normalised[1:] - normalised[:-1]
    smooth_term = 0.1 * float(np.sum(smooth_residual * smooth_residual))
    centre_residual = points[1:-1] - np.asarray(centres, dtype=float)
    centre_term = 0.01 * float(
        np.sum(centre_residual * centre_residual)) / (l0 * l0)
    gradient = np.zeros(2 * (count - 1), dtype=float)
    for index in range(1, count):
        offset = 2 * (index - 1)
        gradient[offset:offset + 2] += (
            2.0 / (l0 * l0)
            * (2.0 * points[index] - points[index - 1] - points[index + 1]))
        gradient[offset:offset + 2] += (
            0.2 * (-1.0 / seed_lengths[index]
                   - 1.0 / seed_lengths[index - 1])
            * smooth_residual[index - 1])
        if index <= count - 2:
            gradient[offset:offset + 2] += (
                0.2 / seed_lengths[index] * smooth_residual[index])
        if index >= 2:
            gradient[offset:offset + 2] += (
                0.2 / seed_lengths[index - 1] * smooth_residual[index - 2])
        gradient[offset:offset + 2] += (
            0.02 / (l0 * l0) * centre_residual[index - 1])
    return {
        "value": length_term + smooth_term + centre_term,
        "gradient": gradient,
        "length_term": length_term,
        "smoothness_term": smooth_term,
        "centre_term": centre_term,
    }


def _negative_turn_indices(points):
    indices = []
    for index in range(1, len(points) - 1):
        left = (
            points[index][0] - points[index - 1][0],
            points[index][1] - points[index - 1][1])
        right = (
            points[index + 1][0] - points[index][0],
            points[index + 1][1] - points[index][1])
        if left[0] * right[0] + left[1] * right[1] < 0.0:
            indices.append(index)
    return indices


def _guide_quality(chain, points, source, portal_radii, qp):
    lengths = _guide_segment_lengths(points)
    seed_lengths = [
        max(math.hypot(region.seed_end[0] - region.seed_start[0],
                       region.seed_end[1] - region.seed_start[1]),
            _GUIDE_MIN_SEED_LENGTH_M)
        for region in chain.regions]
    negative_turns = _negative_turn_indices(points)
    quality = {
        "stage": "C2_A",
        "source": source,
        "chain_digest": chain.digest,
        "region_count": len(chain.regions),
        "segment_corridor_indices": list(range(len(chain.regions))),
        "segment_lengths_m": [float(value) for value in lengths],
        "guide_length_m": float(sum(lengths)),
        "seed_lengths_m": [float(value) for value in seed_lengths],
        "portal_radius_m": [float(value) for value in portal_radii],
        "negative_turn_indices": negative_turns,
        "warnings": (
            ["NEGATIVE_DOT_PRODUCT_PRESERVED"] if negative_turns else []),
        "rounding": "PL_ONLY_NO_BEZIER",
        "smoothness_claim": "NONE",
        "is_executable_path": False,
    }
    quality.update(qp)
    return quality


def make_corridor_guide(chain, start, goal):
    """Build the C2-A safe portal centreline for one validated chain.

    The first version is deliberately a polyline: the QP only moves the
    internal portal vertices inside ``P_j = C_{j-1} intersect C_j``.  Because
    both endpoints of segment ``j`` lie in convex ``C_j``, the whole segment
    lies in ``C_j`` without a new collision proof.  No quadratic fillet is
    applied; ``quality["rounding"]`` therefore says ``PL_ONLY_NO_BEZIER`` and
    no smoothness is claimed.  Negative turns are recorded but never deleted,
    so a genuine U/hairpin is preserved.
    """
    if not isinstance(chain, CorridorChain):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_CHAIN", "expected CorridorChain")
    try:
        start = (float(start[0]), float(start[1]))
        goal = (float(goal[0]), float(goal[1]))
    except (TypeError, ValueError, IndexError) as exc:
        raise VirtualAContractError(
            "INVALID_CORRIDOR_GUIDE",
            "start and goal must be (x, y) pairs") from exc
    for name, point in (("start", start), ("goal", goal)):
        if not (math.isfinite(point[0]) and math.isfinite(point[1])):
            raise VirtualAContractError(
                "INVALID_CORRIDOR_GUIDE",
                "{} must be finite".format(name))
    regions = chain.regions
    count = len(regions)
    for name, region, point in (
            ("start", regions[0], start), ("goal", regions[-1], goal)):
        violation = _row_violation(point, region.A, region.b)
        if violation > _GUIDE_PRIMAL_TOL_M:
            raise VirtualAContractError(
                "CORRIDOR_GUIDE_OUTSIDE",
                "{} violates its endpoint corridor by {} m".format(
                    name, violation))
    centres = []
    portal_radii = []
    portal_rows = []
    portal_limits = []
    for index in range(1, count):
        centre, rows, limits, radius = _portal_bundle(chain, index)
        centres.append(centre)
        portal_radii.append(radius)
        portal_rows.append(rows)
        portal_limits.append(limits)
    centres_array = np.asarray(centres, dtype=float) if centres else \
        np.zeros((0, 2), dtype=float)
    seed_lengths = [
        max(math.hypot(region.seed_end[0] - region.seed_start[0],
                       region.seed_end[1] - region.seed_start[1]),
            _GUIDE_MIN_SEED_LENGTH_M)
        for region in regions]
    l0 = max(math.hypot(goal[0] - start[0], goal[1] - start[1]), 1.0)
    fallback_points = [start] + centres + [goal]
    fallback_objective = None
    if count > 1:
        fallback_objective = _guide_qp_terms(
            np.asarray(centres, dtype=float).reshape(2 * (count - 1)),
            start, goal, centres_array, seed_lengths, l0)["value"]

    qp = {
        "qp_attempted": False,
        "qp_used": False,
        "qp_status": "NO_INTERNAL_PORTAL" if count == 1 else "NOT_ATTEMPTED",
        "qp_iterations": 0,
        "qp_objective": None,
        "fallback_objective": fallback_objective,
        "fallback_reason": "NO_INTERNAL_PORTAL" if count == 1 else "",
        "primal_violation_m": 0.0,
    }
    points = fallback_points
    source = "corridor_endpoint_polyline" if count == 1 else \
        "corridor_chebyshev_polyline"
    accepted = False
    if count > 1:
        initial = np.asarray(centres, dtype=float).reshape(2 * (count - 1))
        matrix = np.zeros(
            (sum(len(rows) for rows in portal_rows), 2 * (count - 1)),
            dtype=float)
        upper = np.zeros(matrix.shape[0], dtype=float)
        offset = 0
        for index, (rows, limits) in enumerate(
                zip(portal_rows, portal_limits)):
            width = len(rows)
            matrix[offset:offset + width, 2 * index:2 * index + 2] = rows
            upper[offset:offset + width] = limits
            offset += width
        initial_violation = float(np.max(
            matrix @ initial - upper, initial=0.0))
        qp["qp_attempted"] = True
        failure = ""
        result = None
        if initial_violation > _GUIDE_PRIMAL_TOL_M:
            failure = "INITIAL_PORTAL_INFEASIBLE"
            qp["qp_status"] = failure
        else:
            qp["qp_status"] = "SLSQP_RUNNING"
            try:
                result = minimize(
                    lambda vector: _guide_qp_terms(
                        vector, start, goal, centres_array,
                        seed_lengths, l0)["value"],
                    initial,
                    method="SLSQP",
                    jac=lambda vector: _guide_qp_terms(
                        vector, start, goal, centres_array,
                        seed_lengths, l0)["gradient"],
                    constraints=[{
                        "type": "ineq",
                        "fun": lambda vector: upper - matrix @ vector,
                        "jac": lambda vector: -matrix,
                    }],
                    options={
                        "maxiter": _GUIDE_QP_MAXITER,
                        "ftol": _GUIDE_QP_FTOL,
                        "disp": False,
                    })
            except Exception as exc:
                failure = "{}: {}".format(type(exc).__name__, exc)
                qp["qp_status"] = "SLSQP_EXCEPTION"
        if result is not None:
            qp["qp_status"] = str(result.message)
            qp["qp_iterations"] = int(getattr(result, "nit", 0))
            candidate = np.asarray(result.x, dtype=float)
            if not result.success:
                failure = str(result.message)
            elif not np.all(np.isfinite(candidate)):
                failure = "NONFINITE_QP_RESULT"
                qp["qp_status"] = failure
            else:
                primal = float(np.max(
                    matrix @ candidate - upper, initial=0.0))
                if primal > _GUIDE_PRIMAL_TOL_M:
                    failure = "PRIMAL_VIOLATION_{:.3e}_M".format(primal)
                    qp["qp_status"] = "PRIMAL_VIOLATION"
                else:
                    candidate_points = [start] + [
                        (float(candidate[2 * index]),
                         float(candidate[2 * index + 1]))
                        for index in range(count - 1)] + [goal]
                    lengths = _guide_segment_lengths(candidate_points)
                    if any(length <= _GUIDE_POINT_DEDUP_M
                           for length in lengths):
                        failure = "ZERO_LENGTH_QP_SEGMENT"
                        qp["qp_status"] = failure
                    elif _guide_region_violation(
                            chain, candidate_points) > _GUIDE_PRIMAL_TOL_M:
                        failure = "QP_GUIDE_LEAVES_ASSIGNED_REGION"
                        qp["qp_status"] = failure
                    else:
                        points = candidate_points
                        source = "corridor_qp_polyline"
                        accepted = True
                        qp["qp_used"] = True
                        qp["qp_status"] = "SLSQP_SUCCESS"
                        qp["qp_objective"] = float(result.fun)
                        qp["primal_violation_m"] = primal
                        qp["qp_terms"] = {
                            "length": float(_guide_qp_terms(
                                candidate, start, goal, centres_array,
                                seed_lengths, l0)["length_term"]),
                            "smoothness": float(_guide_qp_terms(
                                candidate, start, goal, centres_array,
                                seed_lengths, l0)["smoothness_term"]),
                            "centre": float(_guide_qp_terms(
                                candidate, start, goal, centres_array,
                                seed_lengths, l0)["centre_term"]),
                        }
        if not accepted:
            qp["fallback_reason"] = failure or "QP_NOT_ACCEPTED"
        else:
            qp["fallback_reason"] = None
    lengths, violation = _guide_metrics(
        chain, points, "CORRIDOR_GUIDE_DEGENERATE")
    if violation > _GUIDE_PRIMAL_TOL_M:
        if accepted:
            points = fallback_points
            source = "corridor_chebyshev_polyline"
            accepted = False
            qp["qp_used"] = False
            qp["qp_status"] = "GUIDE_PRIMAL_VIOLATION"
            qp["fallback_reason"] = "GUIDE_PRIMAL_VIOLATION_{:.3e}_M".format(
                violation)
            lengths, violation = _guide_metrics(
                chain, points, "CORRIDOR_GUIDE_DEGENERATE")
        if violation > _GUIDE_PRIMAL_TOL_M:
            raise VirtualAContractError(
                "CORRIDOR_GUIDE_DEGENERATE",
                "guide violates assigned corridors by {} m".format(violation))
    qp["primal_violation_m"] = float(violation)
    quality = _guide_quality(
        chain, points, source, portal_radii, qp)
    return GuideRoute(
        points=tuple(points),
        source=source,
        grid_sha256=chain.grid_sha256,
        quality=quality)


def allocate_corridors(chain, guide, intervals=8):
    """Map one safe corridor centreline to fixed Hermite fractions.

    Each guide segment ``[x_j, x_j+1]`` is assigned to corridor ``j``.  Its
    normalized length gives the frozen corridor boundary ``tau_j``.  The
    output interval ends are the sorted union of all ``tau_j`` cuts and all
    ``k / intervals`` cuts; a Hermite interval may therefore contain several
    corridors.  Two adjacent intervals with different indices meet at one
    switch fraction which C2-B must constrain by both corridor halfspace
    sets.  Zero-length cuts are removed together with their ownership, and
    every corridor must still contribute a positive interval.
    """
    if not isinstance(chain, CorridorChain):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_CHAIN", "expected CorridorChain")
    if not isinstance(guide, GuideRoute):
        raise VirtualAContractError(
            "INVALID_GUIDE_ROUTE", "expected GuideRoute")
    if chain.grid_sha256 != guide.grid_sha256:
        raise VirtualAContractError(
            "CORRIDOR_GRID_MISMATCH",
            "guide grid_sha256 does not match the chain")
    if (not isinstance(intervals, int) or isinstance(intervals, bool)
            or intervals < 1):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_ALLOCATION",
            "intervals must be a positive integer")
    points = guide.points
    count = len(chain.regions)
    if len(points) != count + 1:
        raise VirtualAContractError(
            "CORRIDOR_ALLOCATION_MISMATCH",
            "guide has {} points for {} corridors; C2-A needs one segment "
            "per corridor".format(len(points), count))
    lengths, violation = _guide_metrics(
        chain, points, "CORRIDOR_ALLOCATION_DEGENERATE")
    if violation > _GUIDE_PRIMAL_TOL_M:
        raise VirtualAContractError(
            "CORRIDOR_ALLOCATION_MISMATCH",
            "guide leaves an assigned corridor by {} m".format(violation))
    total = math.fsum(lengths)
    if total <= _GUIDE_POINT_DEDUP_M:
        raise VirtualAContractError(
            "CORRIDOR_ALLOCATION_DEGENERATE", "guide length is zero")
    tau = [0.0]
    running = 0.0
    for length in lengths:
        running += length
        tau.append(running / total)
    tau[-1] = 1.0
    raw_cuts = [0.0, 1.0]
    raw_cuts.extend(tau[1:-1])
    raw_cuts.extend(
        float(index) / float(intervals)
        for index in range(1, intervals))
    raw_cuts.sort()
    cuts = []
    for value in raw_cuts:
        if not cuts or value - cuts[-1] > _ALLOCATION_ZERO_M:
            cuts.append(value)
    result = []
    for left, right in zip(cuts, cuts[1:]):
        if right - left <= _ALLOCATION_ZERO_M:
            continue
        middle = 0.5 * (left + right)
        corridor_index = bisect.bisect_right(tau, middle) - 1
        corridor_index = min(max(corridor_index, 0), count - 1)
        result.append((float(left), float(right), int(corridor_index)))
    if not result:
        raise VirtualAContractError(
            "CORRIDOR_ALLOCATION_DEGENERATE", "allocation is empty")
    if abs(result[0][0]) > _ALLOCATION_ZERO_M or \
            abs(result[-1][1] - 1.0) > _ALLOCATION_ZERO_M:
        raise VirtualAContractError(
            "CORRIDOR_ALLOCATION_DEGENERATE",
            "allocation does not cover [0, 1]")
    for previous, current in zip(result, result[1:]):
        if abs(previous[1] - current[0]) > _ALLOCATION_ZERO_M:
            raise VirtualAContractError(
                "CORRIDOR_ALLOCATION_DEGENERATE",
                "allocation has a gap or reversal")
        if current[2] < previous[2]:
            raise VirtualAContractError(
                "CORRIDOR_ALLOCATION_DEGENERATE",
                "allocation skips backward")
    if {item[2] for item in result} != set(range(count)):
        raise VirtualAContractError(
            "CORRIDOR_ALLOCATION_DEGENERATE",
            "allocation skipped a corridor")
    return tuple(result)


def audit_path_in_corridors(
        path, chain, allocation, *, position_budget_m=5e-8,
        max_depth=20, max_intervals=20000, min_width_m=1e-5):
    """Certify a continuous C2 path inside its assigned corridor chain.

    Every allocation boundary and Hermite/profile boundary splits the path
    parameter.  On each interval the midpoint is evaluated with the true path
    model.  Because the reference point has unit speed, every point of the
    interval is within half the arc-length width of the midpoint, so

        n.p(s) <= n.p(mid) + width/2

    for every unit outward normal.  An interval is certified only when this
    worst-case upper bound, plus the independent position budget, stays below
    the halfspace limit.  A failed bound is bisected; exceeding the depth,
    width or total-interval limit returns ``passed=False``.  No collocation
    point is treated as a certificate.
    """
    if not isinstance(path, VirtualAPath):
        raise VirtualAContractError(
            "INVALID_PATH", "expected VirtualAPath")
    if not isinstance(chain, CorridorChain):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_CHAIN", "expected CorridorChain")
    allocation = validate_corridor_allocation(chain, allocation)
    try:
        budget = float(position_budget_m)
        width_limit = float(min_width_m)
    except (TypeError, ValueError) as exc:
        raise VirtualAContractError(
            "INVALID_CORRIDOR_AUDIT_CONFIG",
            "position budget and width limit must be real") from exc
    if not math.isfinite(budget) or budget < 0.0:
        raise VirtualAContractError(
            "INVALID_CORRIDOR_AUDIT_CONFIG",
            "position_budget_m must be finite and nonnegative")
    if not math.isfinite(width_limit) or width_limit <= 0.0:
        raise VirtualAContractError(
            "INVALID_CORRIDOR_AUDIT_CONFIG",
            "min_width_m must be finite and positive")
    for name, value, minimum in (
            ("max_depth", max_depth, 0),
            ("max_intervals", max_intervals, 1)):
        if isinstance(value, bool) or not isinstance(value, numbers.Integral):
            raise VirtualAContractError(
                "INVALID_CORRIDOR_AUDIT_CONFIG",
                "{} must be an integer".format(name))
        if int(value) < minimum:
            raise VirtualAContractError(
                "INVALID_CORRIDOR_AUDIT_CONFIG",
                "{} must be >= {}".format(name, minimum))
    max_depth = int(max_depth)
    max_intervals = int(max_intervals)
    length = float(path.length_m)
    if not math.isfinite(length) or length <= 0.0:
        raise VirtualAContractError(
            "INVALID_PATH", "corridor containment needs a positive path length")
    path_budget = float(path.position_error_bound_m)
    if not math.isfinite(path_budget) or path_budget < 0.0:
        raise VirtualAContractError(
            "INVALID_PATH", "path position error bound is not finite")
    epsilon = budget + path_budget
    guard = 1e-12
    cuts = {0.0, length}
    for left, right, _index in allocation:
        cuts.add(float(left) * length)
        cuts.add(float(right) * length)
    intervals = int(getattr(path.profile, "segment_count", 0) or 0)
    if intervals > 0:
        for index in range(1, intervals):
            cuts.add(length * float(index) / float(intervals))
    try:
        for s0, s1 in path.profile.domain_segments:
            cuts.add(min(length, max(0.0, float(s0))))
            cuts.add(min(length, max(0.0, float(s1))))
    except (TypeError, ValueError) as exc:
        raise VirtualAContractError(
            "INVALID_PATH", "profile domain segments are malformed") from exc
    cuts = sorted(cuts)
    deduplicated = []
    dedup_tolerance = max(1e-15, 1e-15 * length)
    for value in cuts:
        if not deduplicated or value - deduplicated[-1] > dedup_tolerance:
            deduplicated.append(value)
    if abs(deduplicated[-1] - length) > dedup_tolerance:
        raise VirtualAContractError(
            "INVALID_CORRIDOR_AUDIT",
            "path parameter cuts do not cover the full length")
    starts = [item[0] for item in allocation]
    state = {
        "samples": 0,
        "max_depth_reached": 0,
        "max_point_residual_m": -math.inf,
        "max_bound_residual_m": -math.inf,
        "first_failure": None,
    }

    def fail(reason, s, index, width, point_residual, bound_residual, depth,
             detail):
        if state["first_failure"] is None:
            state["first_failure"] = {
                "reason": reason,
                "s_m": float(s),
                "corridor_index": int(index),
                "width_m": float(width),
                "depth": int(depth),
                "point_residual_m": float(point_residual),
                "bound_residual_m": float(bound_residual),
                "detail": str(detail),
            }
        return False

    def check(s0, s1, region, index, depth):
        width = float(s1) - float(s0)
        if width <= dedup_tolerance:
            return True
        if state["samples"] >= max_intervals:
            midpoint = 0.5 * (s0 + s1)
            return fail(
                "CORRIDOR_AUDIT_INTERVAL_LIMIT", midpoint, index, width,
                math.inf, math.inf, depth,
                "total interval budget {} exhausted".format(max_intervals))
        midpoint = 0.5 * (float(s0) + float(s1))
        qf = path.state_at_s(midpoint).qf
        point_residual = max(
            row[0] * qf.x + row[1] * qf.y - limit
            for row, limit in zip(region.A, region.b))
        bound_residual = point_residual + 0.5 * width + epsilon + guard
        state["samples"] += 1
        state["max_depth_reached"] = max(state["max_depth_reached"], depth)
        state["max_point_residual_m"] = max(
            state["max_point_residual_m"], point_residual)
        state["max_bound_residual_m"] = max(
            state["max_bound_residual_m"], bound_residual)
        if bound_residual <= 0.0:
            return True
        if width <= width_limit:
            if point_residual > 0.0:
                return fail(
                    "CORRIDOR_CONTAINMENT_VIOLATION", midpoint, index,
                    width, point_residual, bound_residual, depth,
                    "path point exceeds a corridor halfspace")
            return fail(
                "CORRIDOR_AUDIT_WIDTH_LIMIT", midpoint, index, width,
                point_residual, bound_residual, depth,
                "interval width {!r} m cannot close the bound".format(width))
        if depth >= max_depth:
            return fail(
                "CORRIDOR_AUDIT_DEPTH_LIMIT", midpoint, index, width,
                point_residual, bound_residual, depth,
                "subdivision reached max_depth={}".format(max_depth))
        split = midpoint
        if not check(s0, split, region, index, depth + 1):
            return False
        return check(split, s1, region, index, depth + 1)

    passed = True
    for s0, s1 in zip(deduplicated, deduplicated[1:]):
        if s1 - s0 <= dedup_tolerance:
            continue
        midpoint_sigma = 0.5 * (float(s0) + float(s1)) / length
        index = min(
            len(allocation) - 1,
            max(0, bisect.bisect_right(starts, midpoint_sigma) - 1))
        left, right, corridor_index = allocation[index]
        if not (left - _ALLOCATION_ZERO_M <= midpoint_sigma
                <= right + _ALLOCATION_ZERO_M):
            fail(
                "CORRIDOR_AUDIT_ALLOCATION_MISMATCH", 0.5 * (s0 + s1),
                corridor_index, s1 - s0, math.inf, math.inf, 0,
                "base interval is not covered by one allocation entry")
            passed = False
            break
        if not check(s0, s1, chain.regions[corridor_index],
                     corridor_index, 0):
            passed = False
            break
    result = {
        "audit": "continuous_path_in_corridors",
        "method": "arc_length_midpoint_interval_bound",
        "passed": bool(passed),
        "reason": (
            "CORRIDOR_CONTAINMENT_VERIFIED" if passed
            else state["first_failure"]["reason"]),
        "detail": (
            "continuous path containment certified with epsilon={!r} m"
            .format(epsilon) if passed else
            state["first_failure"]["detail"]),
        "position_budget_m": budget,
        "path_position_error_bound_m": path_budget,
        "epsilon_total_m": epsilon,
        "min_width_m": width_limit,
        "max_depth": max_depth,
        "max_intervals": max_intervals,
        "sample_count": state["samples"],
        "max_depth_reached": state["max_depth_reached"],
        "base_interval_count": len(deduplicated) - 1,
        "allocation_count": len(allocation),
        "corridor_count": len(chain.regions),
        "max_point_residual_m": (
            None if state["max_point_residual_m"] == -math.inf
            else state["max_point_residual_m"]),
        "max_bound_residual_m": (
            None if state["max_bound_residual_m"] == -math.inf
            else state["max_bound_residual_m"]),
        "first_failure": state["first_failure"],
        "is_three_vehicle_certificate": False,
    }
    return result


_REALLOCATION_HALFSPACE_TOL_M = 1e-8
_REALLOCATION_MIN_OVERLAP_M = 1e-6
_REALLOCATION_COARSE_SAMPLES = 513
_REALLOCATION_BISECTION_STEPS = 64


def reallocate_adjacent_portal_once(path, chain, allocation, containment):
    """Return one deterministic adjacent-portal reallocation or ``None``.

    C3 rework 02 allows exactly one fixed-sigma recovery only when the hard
    continuous-containment failure is genuinely adjacent to an allocation
    switch: the failure sigma must lie in one of the two allocation segments
    directly on either side of that switch.  A point that merely hugs a
    halfspace boundary in an earlier allocation segment is an interior
    width failure, not a fixed-sigma switch failure, and is rejected.

    When eligible, move that single switch boundary to the midpoint of the
    connected path-parameter interval that lies inside *both* corridors'
    original halfspaces.  The failure point itself must be in that common
    feasible region, or within a local arc-length neighbourhood derived only
    from the recorded audit ``width_m`` and ``min_width_m``.  The old switch
    location alone is never sufficient.  No other boundary, corridor order or
    allocation entry is changed.

    This function is a candidate generator, not a certificate.  It checks the
    two original halfspace sets directly on the real path model (never a
    sparse collision approximation), then calls the existing
    ``validate_corridor_allocation`` before returning.  The new allocation
    must still pass the NLP, continuous containment and every downstream
    geometry/dynamics/collision audit.  ``None`` means no legal candidate was
    available and the caller must keep the original fail-closed result.
    """
    if not isinstance(path, VirtualAPath):
        raise VirtualAContractError("INVALID_PATH", "expected VirtualAPath")
    if not isinstance(chain, CorridorChain):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_CHAIN", "expected CorridorChain")
    allocation = validate_corridor_allocation(chain, allocation)
    if not isinstance(containment, MappingABC) or containment.get("passed"):
        return None
    first_failure = containment.get("first_failure")
    if not isinstance(first_failure, MappingABC):
        return None
    failure_reason = str(
        first_failure.get("reason") or containment.get("reason") or "")
    if failure_reason != "CORRIDOR_AUDIT_WIDTH_LIMIT":
        return None
    if isinstance(first_failure.get("corridor_index"), bool):
        return None
    try:
        failure_s = float(first_failure.get("s_m"))
        failure_index = int(first_failure.get("corridor_index"))
        failure_residual = float(first_failure.get("point_residual_m"))
        failure_width = float(first_failure.get("width_m"))
        min_width = float(containment.get("min_width_m"))
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(failure_s)
            and math.isfinite(failure_residual)
            and math.isfinite(failure_width)
            and math.isfinite(min_width)):
        return None
    if failure_width < 0.0 or min_width < 0.0:
        return None
    length = float(path.length_m)
    if not math.isfinite(length) or length <= 0.0:
        return None
    if failure_s < 0.0 or failure_s > length:
        return None
    if failure_index < 0 or failure_index >= len(chain.regions):
        return None
    count = len(chain.regions)
    if count < 2:
        return None

    # C3 rework 02: only a failure sigma in one of the two allocation
    # segments directly adjacent to the switch is a fixed-sigma switch
    # failure.  A width-limit point elsewhere in the same corridor is an
    # interior failure and must not spend the single recovery.
    failure_sigma = failure_s / length

    def inside_pair(station, left_index, right_index):
        qf = path.state_at_s(float(station)).qf
        point = (float(qf.x), float(qf.y))
        first_violation = _row_violation(
            point, chain.regions[left_index].A,
            chain.regions[left_index].b)
        second_violation = _row_violation(
            point, chain.regions[right_index].A,
            chain.regions[right_index].b)
        return (first_violation <= _REALLOCATION_HALFSPACE_TOL_M
                and second_violation <= _REALLOCATION_HALFSPACE_TOL_M)

    # A switch is a shared allocation boundary whose left/right corridor
    # indices are adjacent.  Only switches adjacent to the failing corridor
    # are eligible; the failing corridor itself selects the pair.  The
    # failure sigma must be in one of the two segments that meet at that
    # switch.  Point residuals alone cannot qualify a switch.
    candidates = []
    for position in range(1, len(allocation)):
        left_entry = allocation[position - 1]
        right_entry = allocation[position]
        left_index = int(left_entry[2])
        right_index = int(right_entry[2])
        if right_index != left_index + 1:
            continue
        if failure_index not in (left_index, right_index):
            continue
        in_left = (float(left_entry[0]) - _ALLOCATION_ZERO_M
                   <= failure_sigma
                   <= float(left_entry[1]) + _ALLOCATION_ZERO_M)
        in_right = (float(right_entry[0]) - _ALLOCATION_ZERO_M
                    <= failure_sigma
                    <= float(right_entry[1]) + _ALLOCATION_ZERO_M)
        if not (in_left or in_right):
            continue
        candidates.append((position, left_index, right_index,
                           float(right_entry[0])))
    if not candidates:
        return None

    # The failure point must be in the two corridors' common feasible
    # region, or within a local arc-length neighbourhood whose only size is
    # the recorded audit resolution: max(width_m, min_width_m).  This is
    # deliberately not a free distance constant.
    local_radius = max(failure_width, min_width)

    # Pick the nearest eligible switch.  The failure station itself is
    # preferred as the probe when it already lies in both original
    # halfspaces.  Otherwise probe only within the local audit-resolution
    # neighbourhood; an old switch that happens to lie in the overlap is not
    # sufficient for a failure elsewhere in the corridor.
    chosen = None
    for position, left_index, right_index, boundary_sigma \
            in sorted(candidates, key=lambda item: (
                abs(item[3] * length - failure_s), item[0])):
        if inside_pair(failure_s, left_index, right_index):
            chosen = (position, left_index, right_index,
                      boundary_sigma, float(failure_s))
            break
        probes = sorted(set(
            float(failure_s) + float(fraction) * local_radius
            for fraction in (-1.0, -0.5, 0.0, 0.5, 1.0)))
        inside_probes = [
            station for station in probes
            if 0.0 <= station <= length
            and inside_pair(station, left_index, right_index)]
        if not inside_probes:
            continue
        probe = min(
            inside_probes,
            key=lambda station: (abs(station - failure_s), station))
        chosen = (position, left_index, right_index,
                  boundary_sigma, float(probe))
        break
    if chosen is None:
        return None
    position, left_index, right_index, boundary_sigma, probe = chosen

    def inside_chosen(station):
        return inside_pair(station, left_index, right_index)

    # Find the connected inside-both component containing the probe.  The
    # coarse stations are only a bracket for the two bisections; every
    # membership decision is a direct halfspace check on the real path.
    coarse = [
        float(index) * length / float(_REALLOCATION_COARSE_SAMPLES)
        for index in range(_REALLOCATION_COARSE_SAMPLES + 1)]
    stations = sorted(set(coarse + [float(probe)]))
    probe_position = stations.index(float(probe))
    inside_cache = {}

    def cached_inside(station):
        if station not in inside_cache:
            inside_cache[station] = inside_chosen(station)
        return inside_cache[station]

    if not cached_inside(stations[probe_position]):
        return None
    left = probe_position
    while left > 0 and cached_inside(stations[left - 1]):
        left -= 1
    right = probe_position
    while right + 1 < len(stations) and cached_inside(stations[right + 1]):
        right += 1

    def refine(inside_station, outside_station):
        for _ in range(_REALLOCATION_BISECTION_STEPS):
            middle = 0.5 * (inside_station + outside_station)
            if cached_inside(middle):
                inside_station = middle
            else:
                outside_station = middle
        return 0.5 * (inside_station + outside_station)

    lower = 0.0 if left == 0 else refine(
        stations[left], stations[left - 1])
    upper = length if right == len(stations) - 1 else refine(
        stations[right], stations[right + 1])
    if upper - lower < _REALLOCATION_MIN_OVERLAP_M:
        return None
    new_station = 0.5 * (lower + upper)
    if not (lower < new_station < upper):
        return None
    if not cached_inside(new_station):
        # A coarse bracket can hide a narrow exit/re-entry gap.  The chosen
        # boundary itself must lie on the path inside both original
        # halfspaces, not merely between two inside samples.
        return None
    new_sigma = new_station / length
    left_entry = allocation[position - 1]
    right_entry = allocation[position]
    if new_sigma - float(left_entry[0]) <= _ALLOCATION_ZERO_M \
            or float(right_entry[1]) - new_sigma <= _ALLOCATION_ZERO_M:
        return None
    if abs(new_sigma - float(right_entry[0])) <= _ALLOCATION_ZERO_M:
        return None
    candidate = list(allocation)
    candidate[position - 1] = (
        float(left_entry[0]), float(new_sigma), int(left_index))
    candidate[position] = (
        float(new_sigma), float(right_entry[1]), int(right_index))
    try:
        return validate_corridor_allocation(chain, tuple(candidate))
    except VirtualAContractError:
        return None


CORRIDOR_DISPLAY_STATE = "CANDIDATE_REFERENCE_POINT_ONLY"
CORRIDOR_DISPLAYS_KEY = "corridor_displays"


def _display_points(points, name):
    """Copy one actual algorithm polyline for the planning-debug display."""
    try:
        copied = tuple((float(point[0]), float(point[1]))
                       for point in points)
    except (TypeError, ValueError, IndexError) as exc:
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DISPLAY",
            "{} must be (x, y) pairs".format(name)) from exc
    if len(copied) < 2:
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DISPLAY",
            "{} needs at least two points".format(name))
    for x, y in copied:
        if not (math.isfinite(x) and math.isfinite(y)):
            raise VirtualAContractError(
                "INVALID_CORRIDOR_DISPLAY",
                "{} contains a non-finite point".format(name))
    return copied


def corridor_display_fields(chain, identity, *, raw_route_points=None,
                            guide_points=None, guide_quality=None,
                            guide_source=None, corridor_used=False):
    """Export one built chain verbatim for the planning-debug display.

    This is a display-only view: it copies the original ``A``, ``b`` and
    counter-clockwise ``vertices`` without refitting or enlarging any region.
    The identity is the admission identity of one planning generation.  The
    resulting fields are *candidate reference-point geometry*; they carry no
    executable-path or three-vehicle safety claim and cannot be used to
    certify playback.

    V2 adds the actual raw A* route, the centreline points and the centreline
    quality that the same pipeline round consumed.  Nothing here is refitted,
    sampled from another source or drawn from an illustrative line.
    ``corridor_used`` states whether this chain was the solver's hard-halfspace
    input for the current round.  A missing guide or guide quality is exported
    under the explicit ``UNVALIDATED_GUIDE`` label.
    """
    if not isinstance(chain, CorridorChain):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DISPLAY", "chain must be a CorridorChain")
    if not isinstance(identity, MappingABC):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DISPLAY", "identity must be a mapping")
    plan_id = identity.get("plan_id")
    if not isinstance(plan_id, int) or isinstance(plan_id, bool):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DISPLAY", "identity.plan_id must be an integer")
    formation_id = identity.get("formation_id")
    backend = identity.get("backend")
    for name, value in (("formation_id", formation_id),
                        ("backend", backend)):
        if not isinstance(value, str) or not value:
            raise VirtualAContractError(
                "INVALID_CORRIDOR_DISPLAY",
                "identity.{} must be a non-empty string".format(name))
    generation = identity.get("generation")
    route_id = identity.get("route_id")
    for name, value in (("generation", generation), ("route_id", route_id)):
        if (not isinstance(value, int) or isinstance(value, bool) or
                value < 0):
            raise VirtualAContractError(
                "INVALID_CORRIDOR_DISPLAY",
                "identity.{} must be a non-negative integer".format(name))
    grid_sha256 = identity.get("grid_sha256")
    if grid_sha256 != chain.grid_sha256:
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DISPLAY",
            "identity.grid_sha256 differs from the chain grid")
    planning_context_sha256 = identity.get("planning_context_sha256")
    if (not isinstance(planning_context_sha256, str) or
            len(planning_context_sha256) != 64):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DISPLAY",
            "identity.planning_context_sha256 must be 64 characters")
    model_sha256 = identity.get("model_sha256")
    if model_sha256 is not None and (
            not isinstance(model_sha256, str) or len(model_sha256) != 64):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DISPLAY",
            "identity.model_sha256 must be null or 64 characters")
    if not isinstance(corridor_used, bool):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DISPLAY", "corridor_used must be a bool")
    copied_raw_route = ([] if raw_route_points is None
                        else [list(point) for point in _display_points(
                            raw_route_points, "raw_route_points")])
    copied_guide = ([] if guide_points is None
                    else [list(point) for point in _display_points(
                        guide_points, "guide_points")])
    if guide_quality is not None and not isinstance(
            guide_quality, MappingABC):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DISPLAY", "guide_quality must be a mapping")
    if guide_source is not None and (
            not isinstance(guide_source, str) or not guide_source):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DISPLAY",
            "guide_source must be a non-empty string or None")
    regions = []
    for index, region in enumerate(chain.regions):
        payload = region.as_dict()
        payload["region_id"] = index
        regions.append(payload)
    display = {
        "state": CORRIDOR_DISPLAY_STATE,
        "plan_id": plan_id,
        "formation_id": formation_id,
        "backend": backend,
        "generation": generation,
        "route_id": route_id,
        "grid_sha256": grid_sha256,
        "planning_context_sha256": planning_context_sha256,
        "model_sha256": model_sha256,
        "route_source": chain.route_source,
        "chain_digest": chain.digest,
        "regions": regions,
    }
    has_v2_data = (
        raw_route_points is not None or guide_points is not None
        or guide_quality is not None or guide_source is not None
        or corridor_used)
    if not has_v2_data:
        # A V1 caller supplies only the chain.  Omitting the V2 keys keeps
        # that payload's corridor display unchanged; playback clears the
        # guide layer because no actual route/guide was exported.
        return display
    quality = None if guide_quality is None else dict(guide_quality)
    display.update({
        "chain_sha256": chain.digest,
        "corridor_used": corridor_used,
        "raw_route_points": copied_raw_route,
        "guide_points": copied_guide,
        "guide_quality": quality,
        "guide_source": guide_source,
        "guide_label": (
            "PL_GUIDE" if _display_guide_valid(
                chain, copied_guide, quality) else "UNVALIDATED_GUIDE"),
        "guide_is_executable_path": False,
    })
    return display


def _display_guide_valid(chain, points, quality):
    """Same structural test the guide layer uses before showing PL_GUIDE."""
    if not points or not isinstance(quality, MappingABC):
        return False
    if quality.get("stage") != "C2_A":
        return False
    if quality.get("chain_digest") != chain.digest:
        return False
    if quality.get("is_executable_path") is not False:
        return False
    if quality.get("rounding") != "PL_ONLY_NO_BEZIER":
        return False
    if quality.get("smoothness_claim") != "NONE":
        return False
    if quality.get("region_count") != len(chain.regions):
        return False
    if quality.get("segment_corridor_indices") != list(
            range(len(chain.regions))):
        return False
    warnings = quality.get("warnings")
    return isinstance(warnings, list) and not warnings


def corridor_chain_from_payload(payload):
    """Rebuild the chain object from ``result.search[...]['chain']``."""
    if not isinstance(payload, MappingABC):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DISPLAY", "chain payload must be a mapping")
    regions_payload = payload.get("regions")
    if not isinstance(regions_payload, (list, tuple)) or not regions_payload:
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DISPLAY",
            "chain payload needs at least one region")
    regions = []
    for index, region in enumerate(regions_payload):
        if not isinstance(region, MappingABC):
            raise VirtualAContractError(
                "INVALID_CORRIDOR_DISPLAY",
                "chain region must be a mapping")
        if region.get("region_id") not in (None, index):
            raise VirtualAContractError(
                "INVALID_CORRIDOR_DISPLAY",
                "chain region_id does not match its list position")
        regions.append(ConvexCorridor(
            seed_start=tuple(region["seed_start"]),
            seed_end=tuple(region["seed_end"]),
            A=tuple(tuple(row) for row in region["A"]),
            b=tuple(region["b"]),
            vertices=tuple(tuple(vertex) for vertex in region["vertices"])))
    chain = CorridorChain(
        regions=tuple(regions),
        route_source=payload.get("route_source"),
        grid_sha256=payload.get("grid_sha256"))
    if payload.get("digest") not in (None, chain.digest):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DISPLAY",
            "chain payload digest differs from the rebuilt chain")
    return chain


__all__ = [
    "CORRIDOR_DISPLAYS_KEY",
    "CORRIDOR_DISPLAY_STATE",
    "allocate_corridors",
    "audit_corridor_chain",
    "audit_path_in_corridors",
    "build_corridor_chain",
    "corridor_chain_from_payload",
    "corridor_display_fields",
    "corridor_effective_margin",
    "corridor_min_portal_radius",
    "make_corridor_guide",
    "validate_corridor_allocation",
]