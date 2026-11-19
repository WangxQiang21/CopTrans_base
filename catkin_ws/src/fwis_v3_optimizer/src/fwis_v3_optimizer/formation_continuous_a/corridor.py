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
import hashlib
import json
import math
import numbers
import sys
import time
from collections.abc import Mapping as MappingABC

import numpy as np
from scipy.optimize import linprog, minimize

from ..formation import FormationPose
from .contracts import (
    ConvexCorridor,
    CorridorChain,
    CurvatureJet,
    GuideRoute,
    VirtualAContractError,
    grid_signature,
)
from .geometry import VirtualAPath, map_spatial_state
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


# ---------------------------------------------------------------------------
# TASK 02: complete OBB coverage in one convex corridor or an adjacent union
# ---------------------------------------------------------------------------
# These are fixed-sample diagnostic predicates.  ``SINGLE_REGION`` and
# ``COVERED_BY_ADJACENT_UNION`` describe only the supplied polygon (one static
# vehicle rectangle) against the artificial convex regions.  They never
# certify the map free space, never enlarge a halfspace by a tolerance and
# never replace the continuous audits in this module.
COVERAGE_SINGLE_REGION = "SINGLE_REGION"
COVERAGE_ADJACENT_UNION = "COVERED_BY_ADJACENT_UNION"
COVERAGE_GAP = "GAP"
COVERAGE_UNCONFIRMED = "UNCONFIRMED_NUMERICS"
_COVERAGE_TOL_FLOOR_M = 1e-9


def _coverage_scale(polygons, corridors):
    """Coordinate/limit scale used by the suggested coverage tolerance."""
    scale = 1.0
    for polygon in polygons:
        for point in polygon:
            try:
                scale = max(scale, abs(float(point[0])),
                            abs(float(point[1])))
            except (TypeError, ValueError, IndexError):
                continue
    for corridor in corridors:
        if corridor is None:
            continue
        for value in getattr(corridor, "b", ()) or ():
            try:
                scale = max(scale, abs(float(value)))
            except (TypeError, ValueError):
                continue
        for point in getattr(corridor, "vertices", ()) or ():
            try:
                scale = max(scale, abs(float(point[0])),
                            abs(float(point[1])))
            except (TypeError, ValueError, IndexError):
                continue
    return scale


def _coverage_tolerance(tol_m, polygons, corridors):
    """Explicit tolerance or ``max(1e-9, 64 eps scale)`` when omitted."""
    suggested = max(
        _COVERAGE_TOL_FLOOR_M,
        64.0 * sys.float_info.epsilon * _coverage_scale(polygons, corridors))
    if tol_m is None:
        return suggested
    try:
        value = float(tol_m)
    except (TypeError, ValueError) as exc:
        raise VirtualAContractError(
            "INVALID_COVERAGE_TOLERANCE",
            "tol_m must be a real number") from exc
    if not math.isfinite(value) or value < 0.0:
        raise VirtualAContractError(
            "INVALID_COVERAGE_TOLERANCE",
            "tol_m must be finite and nonnegative")
    return value


def obb_polygon(center_xy, theta_rad, shape):
    """Four world-frame corners of one vehicle rectangle, CCW and unclosed.

    ``shape`` is the request's ``VehicleShape``-like object; only its
    ``length_m`` and ``width_m`` are read.  The body heading is the actual
    mapped vehicle heading supplied by the caller, never ``psi_F``.
    """
    try:
        center_x = float(center_xy[0])
        center_y = float(center_xy[1])
        theta = float(theta_rad)
        length = float(shape.length_m)
        width = float(shape.width_m)
    except (TypeError, ValueError, IndexError, AttributeError) as exc:
        raise VirtualAContractError(
            "INVALID_OBB_GEOMETRY",
            "OBB centre, heading and vehicle shape must be numeric") from exc
    if not (math.isfinite(center_x) and math.isfinite(center_y)
            and math.isfinite(theta)):
        raise VirtualAContractError(
            "INVALID_OBB_GEOMETRY", "OBB pose must be finite")
    if not (math.isfinite(length) and math.isfinite(width)
            and length > 0.0 and width > 0.0):
        raise VirtualAContractError(
            "INVALID_OBB_GEOMETRY",
            "vehicle length_m and width_m must be finite and positive")
    cosine = math.cos(theta)
    sine = math.sin(theta)
    half_length = 0.5 * length
    half_width = 0.5 * width
    corners = []
    for local_x, local_y in (
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width)):
        corners.append((
            center_x + cosine * local_x - sine * local_y,
            center_y + sine * local_x + cosine * local_y))
    return tuple(corners)


def obb_support_halfwidth(normal, theta_rad, shape):
    """Support half-width ``h(n, theta)`` of the full rotated rectangle.

    ``normal`` must be a unit vector.  For the rectangle centre ``p`` this is
    exactly ``max_corner n . corner - n . p``; the coverage checks below use
    the equivalent explicit-vertex form.
    """
    try:
        normal_x = float(normal[0])
        normal_y = float(normal[1])
        theta = float(theta_rad)
        length = float(shape.length_m)
        width = float(shape.width_m)
    except (TypeError, ValueError, IndexError, AttributeError) as exc:
        raise VirtualAContractError(
            "INVALID_OBB_GEOMETRY",
            "support normal, heading and vehicle shape must be numeric") from exc
    if not (math.isfinite(normal_x) and math.isfinite(normal_y)
            and math.isfinite(theta)):
        raise VirtualAContractError(
            "INVALID_OBB_GEOMETRY", "support inputs must be finite")
    if abs(math.hypot(normal_x, normal_y) - 1.0) > 1e-9:
        raise VirtualAContractError(
            "INVALID_OBB_GEOMETRY", "support normal must be unit length")
    if not (math.isfinite(length) and math.isfinite(width)
            and length > 0.0 and width > 0.0):
        raise VirtualAContractError(
            "INVALID_OBB_GEOMETRY",
            "vehicle length_m and width_m must be finite and positive")
    cosine = math.cos(theta)
    sine = math.sin(theta)
    tangent_x, tangent_y = cosine, sine
    lateral_x, lateral_y = -sine, cosine
    return (0.5 * length * abs(
                normal_x * tangent_x + normal_y * tangent_y)
            + 0.5 * width * abs(
                normal_x * lateral_x + normal_y * lateral_y))


def _coverage_polygon(vertices, tol_m):
    """Validate one convex CCW polygon for the coverage predicates.

    Returns ``{"polygon", "perimeter_m", "reason"}``; ``polygon`` is ``None``
    for any non-finite, clockwise, non-convex or degenerate input so the
    public predicates can fail closed without raising on numeric input.
    """
    try:
        points = [(float(point[0]), float(point[1])) for point in vertices]
    except (TypeError, ValueError, IndexError) as exc:
        return {"polygon": None, "perimeter_m": 0.0,
                "reason": "INVALID_POLYGON"}
    if len(points) < 3:
        return {"polygon": None, "perimeter_m": 0.0,
                "reason": "DEGENERATE_POLYGON"}
    for x, y in points:
        if not (math.isfinite(x) and math.isfinite(y)):
            return {"polygon": None, "perimeter_m": 0.0,
                    "reason": "NON_FINITE_POLYGON"}
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
    if len(cleaned) < 3 or _signed_twice_area(cleaned) <= 0.0:
        return {"polygon": None, "perimeter_m": 0.0,
                "reason": "CLOCKWISE_OR_DEGENERATE_POLYGON"}
    perimeter = sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(cleaned, cleaned[1:] + cleaned[:1]))
    cross_tol = tol_m * max(1.0, perimeter)
    count = len(cleaned)
    for index in range(count):
        first = cleaned[index]
        middle = cleaned[(index + 1) % count]
        last = cleaned[(index + 2) % count]
        cross = ((middle[0] - first[0]) * (last[1] - middle[1])
                 - (middle[1] - first[1]) * (last[0] - middle[0]))
        if cross < -cross_tol:
            return {"polygon": None, "perimeter_m": 0.0,
                    "reason": "NON_CONVEX_POLYGON"}
    canonical = _canonical_vertices(cleaned)
    if canonical is None:
        return {"polygon": None, "perimeter_m": 0.0,
                "reason": "DEGENERATE_POLYGON"}
    return {"polygon": canonical[0], "perimeter_m": float(perimeter),
            "reason": None}


def _corridor_halfspaces(corridor):
    if not isinstance(corridor, ConvexCorridor):
        raise VirtualAContractError(
            "INVALID_CONVEX_CORRIDOR",
            "coverage geometry needs a ConvexCorridor")
    return tuple(corridor.A), tuple(corridor.b)


def _polygon_violation(points, rows, limits):
    """Signed maximum violation and its first witness over all halfspaces."""
    max_violation = -math.inf
    max_witness = None
    for vertex_index, (x, y) in enumerate(points):
        for halfspace_index, (row, limit) in enumerate(zip(rows, limits)):
            violation = row[0] * x + row[1] * y - limit
            if violation > max_violation:
                max_violation = violation
                max_witness = (halfspace_index, vertex_index, (x, y),
                               violation)
    return max_violation, max_witness


def _violation_witness(witness):
    halfspace_index, vertex_index, vertex, violation = witness
    return {
        "halfspace_index": int(halfspace_index),
        "vertex_index": int(vertex_index),
        "vertex": [float(vertex[0]), float(vertex[1])],
        "violation_m": float(violation),
    }


def _coverage_band_state(violation, tol_m):
    """Robust inside / strictly outside / numeric tolerance band."""
    if violation <= -tol_m:
        return "INSIDE"
    if violation > tol_m:
        return "OUTSIDE"
    return "BAND"


def obb_coverage_in_corridor(obb_vertices, corridor, tol_m=None,
                             region_index=None):
    """Complete-polygon coverage of one convex corridor by one OBB.

    The whole polygon is checked against every halfspace of ``corridor`` via
    its explicit vertices (equivalent to the support half-width for a
    rectangle).  ``GAP`` requires a strictly positive violation; any boundary
    or degenerate evidence returns ``UNCONFIRMED_NUMERICS`` instead of
    silently accepting it.
    """
    rows, limits = _corridor_halfspaces(corridor)
    tol = _coverage_tolerance(tol_m, (obb_vertices,), (corridor,))
    checked = _coverage_polygon(obb_vertices, tol)
    if checked["polygon"] is None:
        return {
            "status": COVERAGE_UNCONFIRMED,
            "region_index": region_index,
            "max_violation_m": None,
            "witness": {"reason": checked["reason"]},
            "tol_m": tol,
            "reason": checked["reason"],
        }
    violation, witness = _polygon_violation(
        checked["polygon"], rows, limits)
    state = _coverage_band_state(violation, tol)
    if state == "INSIDE":
        status, reason = COVERAGE_SINGLE_REGION, "ROBUSTLY_INSIDE"
    elif state == "OUTSIDE":
        status, reason = COVERAGE_GAP, "ROBUST_VIOLATION"
    else:
        status, reason = COVERAGE_UNCONFIRMED, "TOLERANCE_BAND"
    return {
        "status": status,
        "region_index": region_index,
        "max_violation_m": float(violation),
        "witness": _violation_witness(witness),
        "tol_m": tol,
        "reason": reason,
    }


def _union_order(polygon, first_rows, first_limits, second_rows,
                 second_limits, tol_m, area_tol):
    """One-order constructive union classification for a fixed polygon.

    The remaining part of ``polygon`` is clipped by the first region's
    halfspaces.  Every positive-area outside piece must be completely inside
    the second region (checked at all of its vertices, valid by convexity).
    A positive-area piece with a vertex strictly violating the second region
    proves a gap; tiny or boundary-band pieces only make the order ambiguous.
    """
    violation, witness = _polygon_violation(
        polygon, first_rows, first_limits)
    if _coverage_band_state(violation, tol_m) == "INSIDE":
        return {
            "kind": "SINGLE_FIRST",
            "max_violation_m": float(violation),
            "witness": _violation_witness(witness),
            "covered_pieces": 0,
            "outside_pieces": 0,
            "ambiguous": False,
            "gap": None,
        }
    remaining = list(polygon)
    outside_pieces = 0
    covered_pieces = 0
    ambiguous = False
    gap = None
    worst_covered = -math.inf
    for halfspace_index, (row, limit) in enumerate(
            zip(first_rows, first_limits)):
        outside = _clip_polygon(remaining, (-row[0], -row[1]), -limit)
        canonical = (_canonical_vertices(outside)
                     if len(outside) >= 3 else None)
        if canonical is None:
            if outside:
                # A segment/point outside piece is only a boundary sliver.
                ambiguous = True
        else:
            piece, area = canonical
            if area <= area_tol:
                ambiguous = True
            else:
                outside_pieces += 1
                piece_violation, piece_witness = _polygon_violation(
                    piece, second_rows, second_limits)
                state = _coverage_band_state(piece_violation, tol_m)
                if state == "INSIDE":
                    covered_pieces += 1
                    worst_covered = max(worst_covered, piece_violation)
                elif state == "OUTSIDE":
                    if gap is None:
                        gap = _violation_witness(piece_witness)
                        gap["outside_area_m2"] = float(area)
                        gap["clip_halfspace_index"] = int(halfspace_index)
                else:
                    ambiguous = True
        inside = _clip_polygon(remaining, row, limit)
        canonical_inside = (_canonical_vertices(inside)
                            if len(inside) >= 3 else None)
        if canonical_inside is None:
            remaining = []
            break
        remaining = list(canonical_inside[0])
    if gap is not None:
        return {
            "kind": "GAP",
            "max_violation_m": float(gap["violation_m"]),
            "witness": gap,
            "covered_pieces": covered_pieces,
            "outside_pieces": outside_pieces,
            "ambiguous": ambiguous,
            "gap": gap,
        }
    if outside_pieces == 0:
        return {
            "kind": "UNCONFIRMED",
            "max_violation_m": float(violation),
            "witness": {"reason": "NO_SUBSTANTIVE_OUTSIDE_PIECE"},
            "covered_pieces": 0,
            "outside_pieces": 0,
            "ambiguous": ambiguous,
            "gap": None,
        }
    if ambiguous or covered_pieces != outside_pieces:
        return {
            "kind": "UNCONFIRMED",
            "max_violation_m": (float(worst_covered)
                                if covered_pieces else float(violation)),
            "witness": {"reason": "TOLERANCE_BAND"},
            "covered_pieces": covered_pieces,
            "outside_pieces": outside_pieces,
            "ambiguous": ambiguous,
            "gap": None,
        }
    return {
        "kind": "COVERED",
        "max_violation_m": float(worst_covered),
        "witness": {
            "covered_pieces": covered_pieces,
            "outside_pieces": outside_pieces,
            "worst_piece_violation_m": float(worst_covered),
        },
        "covered_pieces": covered_pieces,
        "outside_pieces": outside_pieces,
        "ambiguous": False,
        "gap": None,
    }


def obb_coverage_in_adjacent_union(obb_vertices, left, right, tol_m=None):
    """Complete-polygon coverage of ``C_left union C_right`` by one OBB.

    A polygon entirely inside one region returns ``SINGLE_REGION`` with its
    region index.  Otherwise the constructive clipping test is run in both
    orders; only two covered orders return ``COVERED_BY_ADJACENT_UNION`` and
    only two strict-gap orders return ``GAP``.  Any disagreement, boundary
    contact or degenerate clip returns ``UNCONFIRMED_NUMERICS``.  The convex
    hull of the two regions is never built: that would fill a real gap.
    """
    left_rows, left_limits = _corridor_halfspaces(left)
    right_rows, right_limits = _corridor_halfspaces(right)
    tol = _coverage_tolerance(tol_m, (obb_vertices,), (left, right))
    checked = _coverage_polygon(obb_vertices, tol)
    if checked["polygon"] is None:
        return {
            "status": COVERAGE_UNCONFIRMED,
            "region_index": None,
            "max_violation_m": None,
            "witness": {"reason": checked["reason"]},
            "tol_m": tol,
            "reason": checked["reason"],
            "order_results": {"forward": checked["reason"],
                              "reverse": checked["reason"]},
        }
    polygon = checked["polygon"]
    area_tol = tol * max(1.0, checked["perimeter_m"])
    left_violation, left_witness = _polygon_violation(
        polygon, left_rows, left_limits)
    if _coverage_band_state(left_violation, tol) == "INSIDE":
        return {
            "status": COVERAGE_SINGLE_REGION,
            "region_index": 0,
            "max_violation_m": float(left_violation),
            "witness": _violation_witness(left_witness),
            "tol_m": tol,
            "reason": "ROBUSTLY_INSIDE_LEFT_REGION",
            "order_results": {"forward": "SINGLE_FIRST",
                              "reverse": "SINGLE_SECOND"},
        }
    right_violation, right_witness = _polygon_violation(
        polygon, right_rows, right_limits)
    if _coverage_band_state(right_violation, tol) == "INSIDE":
        return {
            "status": COVERAGE_SINGLE_REGION,
            "region_index": 1,
            "max_violation_m": float(right_violation),
            "witness": _violation_witness(right_witness),
            "tol_m": tol,
            "reason": "ROBUSTLY_INSIDE_RIGHT_REGION",
            "order_results": {"forward": "SINGLE_SECOND",
                              "reverse": "SINGLE_FIRST"},
        }
    forward = _union_order(
        polygon, left_rows, left_limits, right_rows, right_limits,
        tol, area_tol)
    reverse = _union_order(
        polygon, right_rows, right_limits, left_rows, left_limits,
        tol, area_tol)
    order_results = {"forward": forward["kind"], "reverse": reverse["kind"]}
    if forward["kind"] == "COVERED" and reverse["kind"] == "COVERED":
        return {
            "status": COVERAGE_ADJACENT_UNION,
            "region_index": None,
            "max_violation_m": max(forward["max_violation_m"],
                                   reverse["max_violation_m"]),
            "witness": {
                "covered_pieces_forward": forward["covered_pieces"],
                "covered_pieces_reverse": reverse["covered_pieces"],
                "worst_piece_violation_m": max(
                    forward["max_violation_m"],
                    reverse["max_violation_m"]),
            },
            "tol_m": tol,
            "reason": "BOTH_ORDERS_COVERED",
            "order_results": order_results,
        }
    if forward["kind"] == "GAP" and reverse["kind"] == "GAP":
        return {
            "status": COVERAGE_GAP,
            "region_index": None,
            "max_violation_m": float(forward["max_violation_m"]),
            "witness": forward["witness"],
            "tol_m": tol,
            "reason": "BOTH_ORDERS_GAP",
            "order_results": order_results,
        }
    return {
        "status": COVERAGE_UNCONFIRMED,
        "region_index": None,
        "max_violation_m": None,
        "witness": {
            "reason": "ORDER_RESULTS_DISAGREE",
            "forward": forward["witness"],
            "reverse": reverse["witness"],
        },
        "tol_m": tol,
        "reason": "ORDER_RESULTS_DISAGREE",
        "order_results": order_results,
    }


# ---------------------------------------------------------------------------
# TASK 02 Step C: finite candidate sampling and formation assessment
# ---------------------------------------------------------------------------

ASSESSMENT_SCHEMA = "virtual_a_formation_assessment_v1"
ROUTE_ASSESSMENT_SCHEMA = "virtual_a_route_assessment_v1"

MAP_CLEAR_AT_SAMPLES = "CLEAR_AT_SAMPLES"
MAP_COLLISION_OBSERVED = "COLLISION_OBSERVED"
MAP_SAMPLE_UNCONFIRMED = "UNCONFIRMED"

KINEMATIC_NORMAL_BRANCH_OK = "NORMAL_BRANCH_OK"
KINEMATIC_NORMAL_BRANCH_INVALID = "NORMAL_BRANCH_INVALID"
KINEMATIC_CURVATURE_UNCONFIRMED = "CURVATURE_HINT_UNCONFIRMED"

GUIDE_HINT_VALID = "VALID_HINT"
GUIDE_HINT_LOW_CONFIDENCE = "LOW_CONFIDENCE"
GUIDE_HINT_DEGENERATE = "DEGENERATE"

CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY = "DIAGNOSTIC_ONLY"

FORMATION_ASSESSMENT_MAX_STATIONS = 64
ROUTE_ASSESSMENT_MAX_STATIONS = 32
TRANSITION_WINDOW_SPACING_M = 0.10
GUIDE_UNIFORM_SPACING_M = 0.50
ROUTE_UNIFORM_SPACING_M = 1.00
TRANSITION_BISECTION_LEVELS = 2
CORRIDOR_GUIDE_CANDIDATE_LIMIT = 5
CORRIDOR_GUIDE_PORTAL_VARIANT_LIMIT = 3
PORTAL_LOCAL_CANDIDATE_POINT_LIMIT = 4
PORTAL_LOCAL_ASSESSMENT_MAX_STATIONS = 16
GUIDE_CANDIDATE_EVIDENCE_SCHEMA = "virtual_a_guide_candidate_generation_v1"
_CANDIDATE_POINT_DEDUP_M = 1e-9
MAX_ASSESSMENT_COLLISIONS = 8
MAX_WINDOW_COLLISIONS = 3
MAX_VEHICLE_WITNESSES = 3
PORTAL_DIAGNOSTIC_COLLISION_LIMIT = 5
PORTAL_DIAGNOSTIC_GAP_LIMIT = 6
PORTAL_DIAGNOSTIC_WINDOW_LIMIT = 3
PORTAL_DIAGNOSTICS_SCHEMA = "virtual_a_portal_diagnostics_v1"

_SHORT_GUIDE_SEGMENT_M = 1e-8
_BISECTOR_SUM_MIN_M = 1e-6
_NEAR_REVERSE_COS = -0.999999
_ENDPOINT_TANGENT_TOL_RAD = math.radians(20.0)
_CURVATURE_JUMP_HINT_PER_M = 1.0
_STATION_DEDUP_M = 1e-9
_ASSESSMENT_TOL_M = None  # use the coverage module's scale-derived default
_ASSESSMENT_DETAIL_CHARS = 240


def _guide_arc_data(points):
    """Finite polyline arc lengths and per-segment tangents."""
    try:
        values = tuple((float(point[0]), float(point[1]))
                       for point in points)
    except (TypeError, ValueError, IndexError) as exc:
        raise VirtualAContractError(
            "INVALID_GUIDE_POLYLINE",
            "points must be (x, y) pairs") from exc
    if len(values) < 2:
        raise VirtualAContractError(
            "INVALID_GUIDE_POLYLINE",
            "a guide polyline needs at least two points")
    for x, y in values:
        if not (math.isfinite(x) and math.isfinite(y)):
            raise VirtualAContractError(
                "INVALID_GUIDE_POLYLINE", "guide point is not finite")
    cumulative = [0.0]
    lengths = []
    tangents = []
    for first, second in zip(values, values[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length = math.hypot(dx, dy)
        lengths.append(length)
        tangents.append(
            math.atan2(dy, dx) if length > _SHORT_GUIDE_SEGMENT_M else None)
        cumulative.append(cumulative[-1] + length)
    return values, tuple(cumulative), tuple(lengths), tuple(tangents)


def _angle_delta(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


def _assessment_digest(payload):
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _assessment_identity(schema, request_sha256, grid_sha256,
                         planning_context_sha256, route_index, round_index,
                         chain_digest, guide_source, points):
    return _assessment_digest({
        "schema": str(schema),
        "request_sha256": str(request_sha256 or ""),
        "grid_sha256": str(grid_sha256 or ""),
        "planning_context_sha256": str(planning_context_sha256 or ""),
        "route_index": None if route_index is None else int(route_index),
        "round_index": None if round_index is None else int(round_index),
        "chain_digest": None if chain_digest is None else str(chain_digest),
        "guide_source": str(guide_source or ""),
        "guide_points": [[float(x), float(y)] for x, y in points],
    })


def _guide_points_digest(points):
    return _assessment_digest(
        [[float(x), float(y)] for x, y in points])


def _validate_assessment_checker(checker, model, shapes, grid_sha256):
    """Bind the re-used static checker to the request's model and map."""
    checker_model = getattr(checker, "model", None)
    if checker_model is None or not hasattr(checker_model, "offsets"):
        raise VirtualAContractError(
            "INVALID_ASSESSMENT_CHECKER",
            "checker must expose the FormationModel it was built with")
    expected = tuple((offset.vehicle_id, offset.x, offset.y)
                     for offset in model.offsets)
    actual = tuple((offset.vehicle_id, offset.x, offset.y)
                   for offset in checker_model.offsets)
    if expected != actual:
        raise VirtualAContractError(
            "INVALID_ASSESSMENT_CHECKER",
            "checker model offsets do not match the request model")
    checker_grid = getattr(checker, "grid", None)
    if checker_grid is not None and grid_sha256:
        try:
            actual_grid = grid_signature(checker_grid)
        except Exception:
            actual_grid = ""
        if actual_grid and actual_grid != str(grid_sha256):
            raise VirtualAContractError(
                "INVALID_ASSESSMENT_CHECKER",
                "checker grid does not match grid_sha256")
    checker_shapes = getattr(checker, "_shapes", None)
    if isinstance(checker_shapes, MappingABC) and isinstance(shapes, MappingABC):
        for vehicle_id, shape in shapes.items():
            other = checker_shapes.get(vehicle_id)
            if other is None:
                raise VirtualAContractError(
                    "INVALID_ASSESSMENT_CHECKER",
                    "checker has no shape for vehicle_id={}".format(
                        vehicle_id))
            if (abs(float(other.length_m) - float(shape.length_m)) > 1e-12
                    or abs(float(other.width_m) - float(shape.width_m))
                    > 1e-12):
                raise VirtualAContractError(
                    "INVALID_ASSESSMENT_CHECKER",
                    "checker shape differs for vehicle_id={}".format(
                        vehicle_id))


def _shape_view(shape):
    """Length/width of one request shape, or None when unusable."""
    try:
        length = float(getattr(shape, "length_m"))
        width = float(getattr(shape, "width_m"))
    except (TypeError, ValueError, AttributeError):
        return None
    if not (math.isfinite(length) and math.isfinite(width)):
        return None
    if length <= 0.0 or width <= 0.0:
        return None
    return length, width


def _shape_set_complete(model, shapes):
    """True only when every modelled vehicle has a usable request shape."""
    if not isinstance(shapes, MappingABC):
        return False
    for offset in model.offsets:
        if _shape_view(shapes.get(offset.vehicle_id)) is None:
            return False
    return True


def _assessment_rho(model, shapes):
    """Finite reach radius for Portal windows, or (None, reason)."""
    if not isinstance(shapes, MappingABC):
        return None, "MISSING_VEHICLE_SHAPE"
    radii = []
    for offset in model.offsets:
        shape = shapes.get(offset.vehicle_id)
        view = _shape_view(shape)
        if view is None:
            return None, "MISSING_VEHICLE_SHAPE"
        half_diagonal = 0.5 * math.hypot(view[0], view[1])
        radii.append(math.hypot(offset.x, offset.y) + half_diagonal)
    if not radii:
        return None, "EMPTY_VEHICLE_SET"
    return float(max(radii)), ""


def formation_transition_windows(chain, guide_points, model, vehicle_shapes):
    """Bounded Portal reach windows around every adjacent region pair.

    The window is only a finite candidate-sampling hint; it is not a portal
    inscribed radius and it never certifies a continuous transition.
    """
    if not isinstance(chain, CorridorChain):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_CHAIN", "expected CorridorChain")
    points, cumulative, _lengths, _tangents = _guide_arc_data(guide_points)
    regions = chain.regions
    if len(points) != len(regions) + 1:
        raise VirtualAContractError(
            "CORRIDOR_GUIDE_CHAIN_MISMATCH",
            "guide has {} points for {} regions".format(
                len(points), len(regions)))
    violation = _guide_region_violation(chain, points)
    if violation > _GUIDE_PRIMAL_TOL_M:
        raise VirtualAContractError(
            "CORRIDOR_GUIDE_OUTSIDE",
            "guide leaves an assigned corridor by {} m".format(violation))
    total = cumulative[-1]
    rho, rho_reason = _assessment_rho(model, vehicle_shapes)
    windows = []
    for index in range(len(regions) - 1):
        station = cumulative[index + 1]
        sigma = station / total if total > _SHORT_GUIDE_SEGMENT_M else None
        if total <= _SHORT_GUIDE_SEGMENT_M:
            lower = upper = None
            reason = "GUIDE_LENGTH_DEGENERATE"
        elif rho is None:
            lower = upper = None
            reason = rho_reason
        else:
            lower = max(0.0, station - rho)
            upper = min(total, station + rho)
            reason = ""
        windows.append({
            "portal_index": int(index),
            "source_region_index": int(index),
            "target_region_index": int(index + 1),
            "portal_seed_point": [float(points[index + 1][0]),
                                  float(points[index + 1][1])],
            "s_hint_m": float(station),
            "sigma_hint": None if sigma is None else float(sigma),
            "rho_m": None if rho is None else float(rho),
            "s_window_m": (None if lower is None
                           else [float(lower), float(upper)]),
            "sigma_window": (
                None if lower is None or total <= _SHORT_GUIDE_SEGMENT_M
                else [float(lower / total), float(upper / total)]),
            "reason": reason,
        })
    return windows


def _station_meta(points, cumulative, lengths, tangents, station,
                  node_curvature, node_curvature_valid,
                  start_heading, goal_heading):
    """One guide sample and its finite heading/curvature hints."""
    total = cumulative[-1]
    count = len(points)
    eps = _STATION_DEDUP_M
    low = []
    if station <= eps:
        x, y = points[0]
        psi = float(start_heading)
        source = "START_REQUEST"
        first = tangents[0]
        if first is None:
            low.append("ZERO_LENGTH_SEGMENT")
        elif abs(_angle_delta(first, psi)) > _ENDPOINT_TANGENT_TOL_RAD:
            low.append("START_TANGENT_MISMATCH")
        kappa, kappa_source = 0.0, "ENDPOINT_ZERO"
        segment_index = 0
    elif station >= total - eps:
        x, y = points[-1]
        psi = float(goal_heading)
        source = "GOAL_REQUEST"
        last = tangents[-1]
        if last is None:
            low.append("ZERO_LENGTH_SEGMENT")
        elif abs(_angle_delta(last, psi)) > _ENDPOINT_TANGENT_TOL_RAD:
            low.append("GOAL_TANGENT_MISMATCH")
        kappa, kappa_source = 0.0, "ENDPOINT_ZERO"
        segment_index = count - 2
    else:
        index = bisect.bisect_right(cumulative, station) - 1
        index = min(max(index, 0), count - 2)
        vertex = None
        for candidate in (index, index + 1):
            if 0 < candidate < count - 1 and \
                    abs(cumulative[candidate] - station) <= eps:
                vertex = candidate
                break
        if vertex is None:
            x, y = _point_on_segment(
                points[index], points[index + 1], cumulative[index],
                lengths[index], station)
            segment_index = index
            tangent = tangents[index]
            if tangent is None:
                psi = None
                source = "SEGMENT_DEGENERATE"
                low.append("ZERO_LENGTH_SEGMENT")
            else:
                psi = float(tangent)
                source = "SEGMENT_TANGENT"
            kappa, kappa_source = 0.0, "SEGMENT_ZERO"
        else:
            x, y = points[vertex]
            segment_index = min(vertex, count - 2)
            incoming = tangents[vertex - 1]
            outgoing = tangents[vertex]
            if lengths[vertex - 1] <= _SHORT_GUIDE_SEGMENT_M or \
                    lengths[vertex] <= _SHORT_GUIDE_SEGMENT_M:
                low.append("SHORT_SEGMENT")
            if incoming is None or outgoing is None:
                psi = outgoing if outgoing is not None else incoming
                source = ("VERTEX_OUTGOING_FALLBACK"
                          if outgoing is not None else "VERTEX_DEGENERATE")
                low.append("BISECTOR_DEGENERATE")
            else:
                cos_sum = math.cos(incoming) + math.cos(outgoing)
                sin_sum = math.sin(incoming) + math.sin(outgoing)
                norm = math.hypot(cos_sum, sin_sum)
                dot = math.cos(incoming - outgoing)
                if norm <= _BISECTOR_SUM_MIN_M or dot <= _NEAR_REVERSE_COS:
                    psi = float(outgoing)
                    source = "VERTEX_OUTGOING_FALLBACK"
                    low.append("BISECTOR_DEGENERATE")
                else:
                    psi = math.atan2(sin_sum, cos_sum)
                    source = "VERTEX_BISECTOR"
            kappa = None
            kappa_source = "UNAVAILABLE"
            if node_curvature is not None and vertex < len(node_curvature):
                value = node_curvature[vertex]
                valid = True
                if node_curvature_valid is not None:
                    valid = bool(node_curvature_valid[vertex])
                else:
                    valid = _three_point_hint_valid(points, vertex)
                if valid and value is not None and math.isfinite(
                        float(value)):
                    kappa = float(value)
                    kappa_source = "VERTEX_THREE_POINT"
                else:
                    low.append("CURVATURE_HINT_DEGENERATE")
            else:
                low.append("CURVATURE_HINT_UNAVAILABLE")
    reliable = (psi is not None and kappa is not None and source in (
        "START_REQUEST", "GOAL_REQUEST", "SEGMENT_TANGENT",
        "VERTEX_BISECTOR") and "SHORT_SEGMENT" not in low)
    if not reliable and source == "VERTEX_BISECTOR" and \
            "CURVATURE_HINT_DEGENERATE" in low:
        pass
    status = GUIDE_HINT_VALID if not low else GUIDE_HINT_LOW_CONFIDENCE
    return {
        "s_m": float(station),
        "x": float(x),
        "y": float(y),
        "segment_index": int(segment_index),
        "heading_hint_rad": None if psi is None else float(psi),
        "heading_source": source,
        "kappa_hint": None if kappa is None else float(kappa),
        "kappa_source": kappa_source,
        "guide_hint_status": status,
        "hint_reasons": sorted(set(low)),
        "pose_reliable": bool(reliable),
        "window_indices": [],
    }


def _point_on_segment(first, second, first_s, length, station):
    if length <= _SHORT_GUIDE_SEGMENT_M:
        return float(first[0]), float(first[1])
    ratio = (station - first_s) / length
    return (float(first[0] + ratio * (second[0] - first[0])),
            float(first[1] + ratio * (second[1] - first[1])))


def _three_point_hint_valid(points, index):
    if index <= 0 or index >= len(points) - 1:
        return False
    first = math.hypot(points[index][0] - points[index - 1][0],
                       points[index][1] - points[index - 1][1])
    second = math.hypot(points[index + 1][0] - points[index][0],
                        points[index + 1][1] - points[index][1])
    chord = math.hypot(points[index + 1][0] - points[index - 1][0],
                       points[index + 1][1] - points[index - 1][1])
    return min(first, second, chord) > _SHORT_GUIDE_SEGMENT_M


def _merge_station_entries(entries):
    entries.sort(key=lambda entry: float(entry["s"]))
    merged = []
    for entry in entries:
        if merged and abs(entry["s"] - merged[-1]["s"]) <= _STATION_DEDUP_M:
            merged[-1]["tier"] = min(merged[-1]["tier"], entry["tier"])
            merged[-1]["kinds"].update(entry["kinds"])
        else:
            merged.append({
                "s": float(entry["s"]),
                "tier": int(entry["tier"]),
                "kinds": set(entry["kinds"]),
            })
    return merged


def _plan_assessment_stations(points, cumulative, lengths, tangents,
                              windows, cap, uniform_spacing_m,
                              window_spacing_m, node_curvature,
                              node_curvature_valid, start_heading,
                              goal_heading):
    """Deterministic station set with required-first caps."""
    total = cumulative[-1]
    count = len(points)
    entries = [{"s": 0.0, "tier": 0, "kinds": {"start"}},
               {"s": total, "tier": 0, "kinds": {"goal"}}]
    for index in range(1, count - 1):
        entries.append({
            "s": cumulative[index], "tier": 2, "kinds": {"corner"}})
    for window in windows:
        bounds = window.get("s_window_m")
        if bounds is None:
            continue
        lower, upper = float(bounds[0]), float(bounds[1])
        width = upper - lower
        entries.append({"s": lower, "tier": 1, "kinds": {"window_start"}})
        entries.append({"s": upper, "tier": 1, "kinds": {"window_end"}})
        if width <= 0.0:
            samples = 1
        else:
            samples = max(1, int(math.ceil(
                width / max(float(window_spacing_m), 1e-12))))
        for index in range(samples + 1):
            entries.append({
                "s": lower + width * index / float(samples),
                "tier": 1, "kinds": {"window_uniform"}})
        for index in range(1, count - 1):
            if lower - _STATION_DEDUP_M <= cumulative[index] \
                    <= upper + _STATION_DEDUP_M:
                entries.append({
                    "s": cumulative[index], "tier": 1,
                    "kinds": {"portal_corner"}})
        if width > 0.0:
            entries.append({
                "s": window["s_hint_m"], "tier": 1,
                "kinds": {"portal_sigma"}})
    if total > 0.0:
        samples = max(1, int(math.ceil(
            total / max(float(uniform_spacing_m), 1e-12))))
        for index in range(1, samples):
            entries.append({
                "s": total * index / float(samples), "tier": 4,
                "kinds": {"uniform"}})
    merged = _merge_station_entries(entries)
    ordered = sorted(merged, key=lambda entry: (entry["tier"], entry["s"]))
    selected = ordered[:max(1, int(cap))]
    selected_keys = {round(entry["s"], 12) for entry in selected}
    dropped_required = sum(
        1 for entry in ordered[len(selected):] if entry["tier"] < 4)
    sample_cap = dropped_required > 0
    selected.sort(key=lambda entry: entry["s"])
    window_complete = []
    for window in windows:
        bounds = window.get("s_window_m")
        if bounds is None:
            window_complete.append(False)
            continue
        lower, upper = float(bounds[0]), float(bounds[1])
        required = [
            entry for entry in merged
            if entry["tier"] < 4 and lower - _STATION_DEDUP_M
            <= entry["s"] <= upper + _STATION_DEDUP_M]
        window_complete.append(
            all(round(entry["s"], 12) in selected_keys
                for entry in required))
    stations = []
    for entry in selected:
        meta = _station_meta(
            points, cumulative, lengths, tangents, entry["s"],
            node_curvature, node_curvature_valid, start_heading,
            goal_heading)
        for index, window in enumerate(windows):
            bounds = window.get("s_window_m")
            if bounds is not None and float(bounds[0]) - _STATION_DEDUP_M \
                    <= entry["s"] <= float(bounds[1]) + _STATION_DEDUP_M:
                meta["window_indices"].append(index)
        stations.append(meta)
    curvature_jumps = 0
    if node_curvature is not None:
        previous = None
        for index in range(1, count - 1):
            value = node_curvature[index]
            valid = (_three_point_hint_valid(points, index)
                     if node_curvature_valid is None
                     else bool(node_curvature_valid[index]))
            if not valid or value is None or not math.isfinite(float(value)):
                previous = None
                continue
            if previous is not None and abs(
                    float(value) - previous) > _CURVATURE_JUMP_HINT_PER_M:
                curvature_jumps += 1
            previous = float(value)
    return {
        "stations": stations,
        "complete": not sample_cap,
        "sample_cap": bool(sample_cap),
        "dropped_required": int(dropped_required),
        "required_count": int(sum(1 for entry in merged
                                  if entry["tier"] < 4)),
        "window_complete": window_complete,
        "curvature_jumps": int(curvature_jumps),
        "total_length_m": float(total),
    }


def _coverage_region_ids(status, region_index):
    if status == COVERAGE_SINGLE_REGION and region_index is not None:
        return [int(region_index)]
    return []


def _station_coverage(meta, mapped, model, shapes, chain, tol_m):
    """Independent per-vehicle single-region and adjacent-union coverage."""
    regions = None if chain is None else chain.regions
    results = {}
    for offset in model.offsets:
        vehicle_id = offset.vehicle_id
        unknown = {
            "status": COVERAGE_UNCONFIRMED,
            "region_ids": [],
            "max_violation_m": None,
            "reason": "",
            "witness": None,
        }
        if regions is None or not regions:
            unknown["reason"] = "NO_CORRIDOR"
            results[vehicle_id] = unknown
            continue
        view = _shape_view(shapes.get(vehicle_id))
        if view is None:
            unknown["reason"] = "MISSING_VEHICLE_SHAPE"
            results[vehicle_id] = unknown
            continue
        try:
            vehicle = mapped.vehicle(vehicle_id)
        except Exception:
            unknown["reason"] = "VEHICLE_NOT_IN_STATE"
            results[vehicle_id] = unknown
            continue
        try:
            polygon = obb_polygon((vehicle.x, vehicle.y), vehicle.theta,
                                  shapes.get(vehicle_id))
        except VirtualAContractError as exc:
            unknown["reason"] = exc.reason
            results[vehicle_id] = unknown
            continue
        segment = meta["segment_index"]
        segment = min(max(segment, 0), len(regions) - 1)
        singles = []
        single_notes = []
        checked = obb_coverage_in_corridor(
            polygon, regions[segment], tol_m=tol_m,
            region_index=segment)
        if checked["status"] == COVERAGE_SINGLE_REGION:
            singles.append((segment, checked))
            single_notes.append(checked)
        window_results = []
        for window_index in meta["window_indices"]:
            left = window_index
            if left < 0 or left + 1 >= len(regions):
                continue
            left_result = obb_coverage_in_corridor(
                polygon, regions[left], tol_m=tol_m, region_index=left)
            right_result = obb_coverage_in_corridor(
                polygon, regions[left + 1], tol_m=tol_m,
                region_index=left + 1)
            for result, region_id in ((left_result, left),
                                      (right_result, left + 1)):
                if result["status"] == COVERAGE_SINGLE_REGION and \
                        region_id not in [item[0] for item in singles]:
                    singles.append((region_id, result))
                    single_notes.append(result)
            union = obb_coverage_in_adjacent_union(
                polygon, regions[left], regions[left + 1], tol_m=tol_m)
            window_results.append((left, union))
        if singles:
            worst = max(item[1]["max_violation_m"] for item in singles)
            results[vehicle_id] = {
                "status": COVERAGE_SINGLE_REGION,
                "region_ids": sorted(item[0] for item in singles),
                "max_violation_m": float(worst),
                "reason": "ROBUSTLY_INSIDE_SINGLE_REGION",
                "witness": _coverage_witness_view(single_notes[0]),
                "window_index": (
                    None if not meta["window_indices"]
                    else int(meta["window_indices"][0])),
            }
            continue
        covered = [(index, union) for index, union in window_results
                   if union["status"] == COVERAGE_ADJACENT_UNION]
        if covered:
            index, union = covered[0]
            results[vehicle_id] = {
                "status": COVERAGE_ADJACENT_UNION,
                "region_ids": [int(index), int(index + 1)],
                "max_violation_m": float(union["max_violation_m"]),
                "reason": union["reason"],
                "witness": _coverage_witness_view(union),
                "window_index": int(index),
            }
            continue
        gaps = [(index, union) for index, union in window_results
                if union["status"] == COVERAGE_GAP]
        unknowns = [(index, union) for index, union in window_results
                    if union["status"] == COVERAGE_UNCONFIRMED]
        if gaps and not unknowns:
            index, union = gaps[0]
            witness = _coverage_witness_view(union)
            witness["window_index"] = int(index)
            results[vehicle_id] = {
                "status": COVERAGE_GAP,
                "region_ids": [int(index), int(index + 1)],
                "max_violation_m": float(union["max_violation_m"]),
                "reason": union["reason"],
                "witness": witness,
                "window_index": int(index),
            }
            continue
        if not window_results:
            unknown["reason"] = "OUTSIDE_TRANSITION_WINDOW"
        elif unknowns:
            index, union = unknowns[0]
            unknown["reason"] = union.get("reason") or "UNCONFIRMED_UNION"
            unknown["witness"] = _coverage_witness_view(union)
            unknown["witness"]["window_index"] = int(index)
            unknown["max_violation_m"] = union.get("max_violation_m")
        else:
            unknown["reason"] = "UNCONFIRMED_UNION"
        results[vehicle_id] = unknown
    return results


def _coverage_witness_view(result):
    witness = result.get("witness")
    view = {}
    if isinstance(witness, MappingABC):
        for key in ("reason", "halfspace_index", "vertex_index", "vertex",
                    "violation_m", "outside_area_m2", "clip_halfspace_index",
                    "covered_pieces_forward", "covered_pieces_reverse",
                    "worst_piece_violation_m", "covered_pieces",
                    "outside_pieces"):
            if key in witness:
                value = witness[key]
                if isinstance(value, (list, tuple)):
                    view[key] = [float(item) if isinstance(item, (int, float))
                                 else item for item in value]
                elif isinstance(value, (int, float)) and not isinstance(
                        value, bool):
                    view[key] = float(value)
                else:
                    view[key] = value
    if result.get("region_index") is not None:
        view["region_index"] = int(result["region_index"])
    view["status"] = result.get("status")
    return view


def _evaluate_station(meta, chain, checker, model, vehicle_shapes, tol_m):
    """Map/vehicle SAT and coverage for one station, with separate statuses."""
    vehicle_ids = tuple(offset.vehicle_id for offset in model.offsets)
    coverage = {
        vehicle_id: {
            "status": COVERAGE_UNCONFIRMED,
            "region_ids": [],
            "max_violation_m": None,
            "reason": "",
            "witness": None,
        } for vehicle_id in vehicle_ids}
    normal_branch = {vehicle_id: KINEMATIC_CURVATURE_UNCONFIRMED
                     for vehicle_id in vehicle_ids}
    result = {
        "meta": meta,
        "map": {
            "status": MAP_SAMPLE_UNCONFIRMED,
            "reason": "",
            "vehicle_id": None,
            "other_vehicle_id": None,
            "detail": "",
        },
        "coverage": coverage,
        "normal_branch": normal_branch,
        "normal_branch_reason": {vehicle_id: "" for vehicle_id in vehicle_ids},
    }
    if not meta["pose_reliable"]:
        reason = "HINT_UNAVAILABLE"
        result["map"]["reason"] = reason
        for vehicle_id in vehicle_ids:
            coverage[vehicle_id]["reason"] = reason
            result["normal_branch_reason"][vehicle_id] = reason
        return result
    qf = FormationPose(meta["x"], meta["y"], meta["heading_hint_rad"])
    jet = CurvatureJet(float(meta["kappa_hint"]), 0.0, 0.0, 0.0)
    try:
        mapped = map_spatial_state(qf, jet, model, s=meta["s_m"])
    except VirtualAContractError as exc:
        reason = str(exc.reason)
        result["map"]["reason"] = reason
        result["map"]["detail"] = str(exc.detail)[:_ASSESSMENT_DETAIL_CHARS]
        for vehicle_id in vehicle_ids:
            coverage[vehicle_id]["reason"] = reason
            result["normal_branch_reason"][vehicle_id] = reason
            if reason == "NORMAL_BRANCH_VIOLATION" and \
                    exc.location.get("vehicle_id") == vehicle_id:
                normal_branch[vehicle_id] = KINEMATIC_NORMAL_BRANCH_INVALID
        return result
    headings = {vehicle.vehicle_id: vehicle.theta
                for vehicle in mapped.vehicles}
    try:
        report = checker.static_check(qf, headings)
    except Exception as exc:
        result["map"]["reason"] = "CHECKER_EXCEPTION"
        result["map"]["detail"] = "{}: {}".format(
            type(exc).__name__, exc)[:_ASSESSMENT_DETAIL_CHARS]
        for vehicle_id in vehicle_ids:
            coverage[vehicle_id]["reason"] = "CHECKER_EXCEPTION"
        return result
    reason = str(getattr(report, "reason", "") or "")
    if bool(getattr(report, "success", False)):
        result["map"]["status"] = MAP_CLEAR_AT_SAMPLES
        result["map"]["reason"] = "MAP_AND_VEHICLE_SAT_CLEAR"
        for vehicle_id in vehicle_ids:
            normal_branch[vehicle_id] = KINEMATIC_NORMAL_BRANCH_OK
    elif reason in ("MAP_OBB_COLLISION", "VEHICLE_OBB_COLLISION"):
        result["map"]["status"] = MAP_COLLISION_OBSERVED
        result["map"]["reason"] = reason
        result["map"]["vehicle_id"] = getattr(report, "vehicle_id", None)
        result["map"]["other_vehicle_id"] = getattr(
            report, "other_vehicle_id", None)
        result["map"]["detail"] = str(
            getattr(report, "detail", "") or "")[:_ASSESSMENT_DETAIL_CHARS]
        for vehicle_id in vehicle_ids:
            normal_branch[vehicle_id] = KINEMATIC_NORMAL_BRANCH_OK
    else:
        result["map"]["status"] = MAP_SAMPLE_UNCONFIRMED
        result["map"]["reason"] = reason or "UNKNOWN_CHECKER_RESULT"
        result["map"]["vehicle_id"] = getattr(report, "vehicle_id", None)
        result["map"]["detail"] = str(
            getattr(report, "detail", "") or "")[:_ASSESSMENT_DETAIL_CHARS]
    result["coverage"] = _station_coverage(
        meta, mapped, model, vehicle_shapes, chain, tol_m)
    return result


def _station_signature(result, vehicle_ids):
    return (
        result["map"]["status"],
        tuple(result["coverage"][vehicle_id]["status"]
              for vehicle_id in vehicle_ids))


def _window_map_summary(results):
    counts = {"sampled": 0, "clear": 0, "map_collision": 0,
              "vehicle_collision": 0, "unconfirmed": 0}
    collisions = []
    for result in results:
        counts["sampled"] += 1
        status = result["map"]["status"]
        if status == MAP_CLEAR_AT_SAMPLES:
            counts["clear"] += 1
        elif status == MAP_COLLISION_OBSERVED:
            if result["map"]["reason"] == "VEHICLE_OBB_COLLISION":
                counts["vehicle_collision"] += 1
            else:
                counts["map_collision"] += 1
            if len(collisions) < MAX_WINDOW_COLLISIONS:
                collisions.append({
                    "s_m": result["meta"]["s_m"],
                    "station_index": result["meta"].get("station_index"),
                    "reason": result["map"]["reason"],
                    "vehicle_id": result["map"]["vehicle_id"],
                    "other_vehicle_id": result["map"]["other_vehicle_id"],
                })
        else:
            counts["unconfirmed"] += 1
    if collisions:
        status = MAP_COLLISION_OBSERVED
    elif not results or counts["unconfirmed"]:
        status = MAP_SAMPLE_UNCONFIRMED
    else:
        status = MAP_CLEAR_AT_SAMPLES
    return status, counts, collisions


def _vehicle_coverage_summary(results, vehicle_id, incomplete):
    counts = {"single": 0, "union": 0, "gap": 0, "unconfirmed": 0}
    region_ids = set()
    margins = []
    gap_violations = []
    witnesses = []
    first_gap = None
    worst_covered = None
    first_unknown = None
    for result in results:
        coverage = result["coverage"][vehicle_id]
        status = coverage["status"]
        if status == COVERAGE_SINGLE_REGION:
            counts["single"] += 1
        elif status == COVERAGE_ADJACENT_UNION:
            counts["union"] += 1
        elif status == COVERAGE_GAP:
            counts["gap"] += 1
        else:
            counts["unconfirmed"] += 1
        for region_id in coverage.get("region_ids") or ():
            region_ids.add(int(region_id))
        value = coverage.get("max_violation_m")
        if status in (COVERAGE_SINGLE_REGION, COVERAGE_ADJACENT_UNION) \
                and value is not None:
            margin = -float(value)
            margins.append(margin)
            if worst_covered is None or margin < worst_covered[0]:
                worst_covered = (margin, result)
        if status == COVERAGE_GAP and value is not None:
            gap_violations.append(float(value))
            if first_gap is None:
                first_gap = result
        if status == COVERAGE_UNCONFIRMED and first_unknown is None:
            first_unknown = result
    if incomplete:
        # A capped or degenerate sample set cannot claim a definite
        # corridor-coverage state; observed gap counts stay visible above.
        status = COVERAGE_UNCONFIRMED
    elif counts["gap"]:
        status = COVERAGE_GAP
    elif counts["unconfirmed"]:
        status = COVERAGE_UNCONFIRMED
    elif counts["union"]:
        status = COVERAGE_ADJACENT_UNION
    elif counts["single"]:
        status = COVERAGE_SINGLE_REGION
    else:
        status = COVERAGE_UNCONFIRMED
    for pick in (first_gap, None if worst_covered is None else worst_covered[1],
                 first_unknown):
        if pick is None or len(witnesses) >= MAX_VEHICLE_WITNESSES:
            continue
        cover = pick["coverage"][vehicle_id]
        witnesses.append({
            "s_m": pick["meta"]["s_m"],
            "station_index": pick["meta"].get("station_index"),
            "status": cover["status"],
            "reason": cover.get("reason"),
            "region_ids": list(cover.get("region_ids") or ()),
            "max_violation_m": cover.get("max_violation_m"),
            "window_index": cover.get("window_index"),
            "witness": cover.get("witness"),
        })
    return {
        "status": status,
        "counts": counts,
        "single_region_ids": sorted(region_ids),
        "min_covered_margin_m": (None if not margins else float(min(margins))),
        "worst_gap_violation_m": (
            None if not gap_violations else float(max(gap_violations))),
        "witnesses": witnesses[:MAX_VEHICLE_WITNESSES],
        "vehicle_curvature_status": KINEMATIC_CURVATURE_UNCONFIRMED,
        "region_pair_sampled": bool(counts["union"]),
    }


def _vehicle_normal_branch_summary(results, vehicle_id, incomplete):
    counts = {KINEMATIC_NORMAL_BRANCH_OK: 0,
              KINEMATIC_NORMAL_BRANCH_INVALID: 0,
              KINEMATIC_CURVATURE_UNCONFIRMED: 0}
    witnesses = []
    for result in results:
        status = result["normal_branch"][vehicle_id]
        counts[status] = counts.get(status, 0) + 1
        if status == KINEMATIC_NORMAL_BRANCH_INVALID and \
                len(witnesses) < MAX_VEHICLE_WITNESSES:
            witnesses.append({
                "s_m": result["meta"]["s_m"],
                "station_index": result["meta"].get("station_index"),
                "reason": result["normal_branch_reason"][vehicle_id],
            })
    if counts[KINEMATIC_NORMAL_BRANCH_INVALID]:
        status = KINEMATIC_NORMAL_BRANCH_INVALID
    elif incomplete or not results or \
            counts[KINEMATIC_NORMAL_BRANCH_OK] != len(results):
        status = KINEMATIC_CURVATURE_UNCONFIRMED
    else:
        status = KINEMATIC_NORMAL_BRANCH_OK
    return {
        "normal_branch_status": status,
        "counts": counts,
        "witnesses": witnesses[:MAX_VEHICLE_WITNESSES],
        "vehicle_curvature_status": KINEMATIC_CURVATURE_UNCONFIRMED,
    }


def _assessment_map_summary(results, incomplete):
    counts = {"sampled": len(results), "clear": 0, "map_collision": 0,
              "vehicle_collision": 0, "unconfirmed": 0}
    for result in results:
        status = result["map"]["status"]
        if status == MAP_CLEAR_AT_SAMPLES:
            counts["clear"] += 1
        elif status == MAP_COLLISION_OBSERVED:
            if result["map"]["reason"] == "VEHICLE_OBB_COLLISION":
                counts["vehicle_collision"] += 1
            else:
                counts["map_collision"] += 1
        else:
            counts["unconfirmed"] += 1
    if counts["map_collision"] or counts["vehicle_collision"]:
        status = MAP_COLLISION_OBSERVED
    elif incomplete or counts["unconfirmed"] or not results:
        status = MAP_SAMPLE_UNCONFIRMED
    else:
        status = MAP_CLEAR_AT_SAMPLES
    return status, counts, _collect_collisions(results)


def _assessment_coverage_summary(results, vehicle_ids, incomplete):
    vehicles = {}
    for vehicle_id in vehicle_ids:
        vehicles[vehicle_id] = _vehicle_coverage_summary(
            results, vehicle_id, incomplete)
    statuses = [vehicles[vehicle_id]["status"] for vehicle_id in vehicle_ids]
    if COVERAGE_GAP in statuses:
        status = COVERAGE_GAP
    elif incomplete or not results or COVERAGE_UNCONFIRMED in statuses:
        status = COVERAGE_UNCONFIRMED
    elif all(item == COVERAGE_SINGLE_REGION for item in statuses):
        status = COVERAGE_SINGLE_REGION
    else:
        status = COVERAGE_ADJACENT_UNION
    return status, vehicles


def _assessment_kinematic_summary(results, vehicle_ids, incomplete):
    vehicles = {}
    for vehicle_id in vehicle_ids:
        vehicles[vehicle_id] = _vehicle_normal_branch_summary(
            results, vehicle_id, incomplete)
    statuses = [
        vehicles[vehicle_id]["normal_branch_status"]
        for vehicle_id in vehicle_ids]
    if KINEMATIC_NORMAL_BRANCH_INVALID in statuses:
        status = KINEMATIC_NORMAL_BRANCH_INVALID
    else:
        # eta is never continuously estimated in TASK 02, so the aggregate
        # curvature hint stays unknown even when every normal branch is OK.
        status = KINEMATIC_CURVATURE_UNCONFIRMED
    return status, vehicles


def _assessment_guide_hint(stations, plan, total_length):
    if total_length <= _SHORT_GUIDE_SEGMENT_M or not stations:
        return GUIDE_HINT_DEGENERATE, ["GUIDE_LENGTH_DEGENERATE"]
    reasons = set()
    for station in stations:
        reasons.update(station.get("hint_reasons") or ())
    if plan.get("curvature_jumps"):
        reasons.add("CURVATURE_JUMP")
    if reasons:
        return GUIDE_HINT_LOW_CONFIDENCE, sorted(reasons)
    return GUIDE_HINT_VALID, []


def _assessment_windows(windows, results, vehicle_ids, plan, incomplete):
    summary = []
    for index, window in enumerate(windows):
        members = [result for result in results
                   if index in result["meta"]["window_indices"]]
        window_map_status, window_map_counts, window_collisions = \
            _window_map_summary(members)
        complete = bool(plan["window_complete"][index]) and not incomplete
        vehicles = {}
        for vehicle_id in vehicle_ids:
            vehicle = _vehicle_coverage_summary(
                members, vehicle_id, not complete)
            vehicles[vehicle_id] = {
                "single_region_ids": vehicle["single_region_ids"],
                "union_cover_status": vehicle["status"],
                "counts": vehicle["counts"],
                "min_covered_margin_m": vehicle["min_covered_margin_m"],
                "worst_gap_violation_m": vehicle["worst_gap_violation_m"],
                "witnesses": vehicle["witnesses"],
            }
        statuses = [vehicles[vehicle_id]["union_cover_status"]
                    for vehicle_id in vehicle_ids]
        if not complete or not members:
            # A window whose required stations were dropped cannot claim a
            # transition state; gap counts remain recorded per vehicle.
            status = "PORTAL_TRANSITION_UNCONFIRMED"
        elif COVERAGE_GAP in statuses:
            status = "PORTAL_TRANSITION_GAP"
        elif COVERAGE_UNCONFIRMED in statuses:
            status = "PORTAL_TRANSITION_UNCONFIRMED"
        else:
            status = "PORTAL_TRANSITION_SAMPLED"
        reason = window.get("reason") or ""
        if not plan["window_complete"][index]:
            reason = "SAMPLE_CAP" if not reason else reason + "+SAMPLE_CAP"
        meta = {
            "portal_index": int(window["portal_index"]),
            "source_region_index": int(window["source_region_index"]),
            "target_region_index": int(window["target_region_index"]),
            "portal_seed_point": list(window["portal_seed_point"]),
            "sigma_hint": window["sigma_hint"],
            "sigma_window": window["sigma_window"],
            "s_window_m": (None if window["s_window_m"] is None
                           else list(window["s_window_m"])),
            "rho_m": window["rho_m"],
            "station_count": len(members),
            "status": status,
            "reason": reason,
            "map_sample_status": window_map_status,
            "map_sample_counts": window_map_counts,
            "collisions": window_collisions,
            "vehicles": [dict({"vehicle_id": int(vehicle_id)},
                              **vehicles[vehicle_id])
                         for vehicle_id in vehicle_ids],
            "continuously_certified": False,
        }
        channel = [result for result in members
                   if any(coverage.get("status") == COVERAGE_GAP
                          for coverage in result["coverage"].values())]
        meta["gap_station_count"] = len(channel)
        summary.append(meta)
    return summary


def _allocation_view(allocation):
    if allocation is None:
        return None
    view = []
    for entry in allocation:
        try:
            view.append([float(entry[0]), float(entry[1]), int(entry[2])])
        except (TypeError, ValueError, IndexError):
            raise VirtualAContractError(
                "INVALID_ASSESSMENT_ALLOCATION",
                "allocation entries must be (start, end, corridor_index)")
    return view


def _station_bucket(results, window_index=None):
    if window_index is None:
        return list(results)
    return [result for result in results
            if window_index in result["meta"]["window_indices"]]


def _assign_station_indices(results):
    results.sort(key=lambda result: result["meta"]["s_m"])
    for index, result in enumerate(results):
        result["meta"]["station_index"] = index
    return results


def _collect_collisions(results):
    collisions = []
    for result in results:
        if result["map"]["status"] != MAP_COLLISION_OBSERVED:
            continue
        if len(collisions) >= MAX_ASSESSMENT_COLLISIONS:
            break
        collisions.append({
            "station_index": result["meta"].get("station_index"),
            "s_m": result["meta"]["s_m"],
            "reason": result["map"]["reason"],
            "vehicle_id": result["map"]["vehicle_id"],
            "other_vehicle_id": result["map"]["other_vehicle_id"],
            "detail": result["map"]["detail"],
        })
    return collisions


def _assessment_result_common(results, vehicle_ids, plan, windows, deadline_hit,
                              request_incomplete=False):
    incomplete = (not plan["complete"] or bool(deadline_hit)
                  or bool(request_incomplete))
    degenerate = plan["total_length_m"] <= _SHORT_GUIDE_SEGMENT_M
    map_status, map_counts, _unused = _assessment_map_summary(
        results, incomplete)
    coverage_status, coverage_vehicles = _assessment_coverage_summary(
        results, vehicle_ids, incomplete or degenerate)
    kinematic_status, kinematic_vehicles = _assessment_kinematic_summary(
        results, vehicle_ids, incomplete)
    normal_statuses = [kinematic_vehicles[vehicle_id][
        "normal_branch_status"] for vehicle_id in vehicle_ids]
    guide_status, guide_reasons = _assessment_guide_hint(
        plan["stations"], plan, plan["total_length_m"])
    reasons = []
    if deadline_hit:
        reasons.append("SOFT_BUDGET")
    if plan["sample_cap"]:
        reasons.append("SAMPLE_CAP")
    if degenerate:
        reasons.append("GUIDE_LENGTH_DEGENERATE")
    if request_incomplete:
        reasons.append("MISSING_VEHICLE_SHAPE")
    return {
        "station_count": len(results),
        "sampled_station_count": len(results),
        "required_station_count": int(plan["required_count"]),
        "sample_cap": bool(plan["sample_cap"]),
        "dropped_required_stations": int(plan["dropped_required"]),
        "incomplete_reasons": reasons,
        "map_sample_status": map_status,
        "map_sample_counts": map_counts,
        "vehicle_map_collisions": _collect_collisions(results),
        "coverage_sample_status": coverage_status,
        "coverage_vehicles": coverage_vehicles,
        "kinematic_hint_status": kinematic_status,
        "kinematic_vehicles": kinematic_vehicles,
        "normal_branch_status": (
            KINEMATIC_NORMAL_BRANCH_INVALID
            if KINEMATIC_NORMAL_BRANCH_INVALID in normal_statuses
            else (KINEMATIC_NORMAL_BRANCH_OK
                  if not incomplete and all(
                      status == KINEMATIC_NORMAL_BRANCH_OK
                      for status in normal_statuses)
                  else KINEMATIC_CURVATURE_UNCONFIRMED)),
        "guide_hint_status": guide_status,
        "guide_hint_reasons": guide_reasons,
        "transition_windows": _assessment_windows(
            windows, results, vehicle_ids, plan, incomplete),
        "candidate_certification": CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY,
        "formation_obb_hard_constraints": False,
        "continuously_certified": False,
    }


def _station_results_for_plan(plan, chain, checker, model, shapes, windows,
                              deadline):
    results = []
    deadline_hit = False
    for station in plan["stations"]:
        if deadline is not None and time.monotonic() >= deadline:
            deadline_hit = True
            break
        results.append(_evaluate_station(
            station, chain, checker, model, shapes, _ASSESSMENT_TOL_M))
    capacity = int(plan.get("capacity_stations", len(results)))
    existing = {round(result["meta"]["s_m"], 12) for result in results}
    vehicle_ids = tuple(offset.vehicle_id for offset in model.offsets)
    for _level in range(TRANSITION_BISECTION_LEVELS):
        if deadline_hit or not windows or len(results) >= capacity:
            break
        candidates = []
        seen = set()
        for window_index in range(len(windows)):
            members = [result for result in results
                       if window_index in result["meta"]["window_indices"]]
            members.sort(key=lambda result: result["meta"]["s_m"])
            for first, second in zip(members, members[1:]):
                if _station_signature(first, vehicle_ids) == \
                        _station_signature(second, vehicle_ids):
                    continue
                midpoint = 0.5 * (first["meta"]["s_m"]
                                  + second["meta"]["s_m"])
                key = round(midpoint, 12)
                if key in existing or key in seen:
                    continue
                seen.add(key)
                candidates.append(float(midpoint))
        allowed = max(0, capacity - len(results))
        candidates = candidates[:allowed]
        if not candidates:
            break
        for station_s in candidates:
            if deadline is not None and time.monotonic() >= deadline:
                deadline_hit = True
                break
            frame = _station_meta(
                plan["points"], plan["cumulative"], plan["lengths"],
                plan["tangents"], float(station_s),
                plan["node_curvature"], plan["node_curvature_valid"],
                plan["start_heading"], plan["goal_heading"])
            for index, window in enumerate(windows):
                bounds = window.get("s_window_m")
                if bounds is not None and \
                        float(bounds[0]) - _STATION_DEDUP_M <= \
                        station_s <= float(bounds[1]) + _STATION_DEDUP_M:
                    frame["window_indices"].append(index)
            results.append(_evaluate_station(
                frame, chain, checker, model, shapes, _ASSESSMENT_TOL_M))
            existing.add(round(station_s, 12))
        if deadline_hit:
            break
    return results, deadline_hit


def assess_route_guide(route_index, points, checker, model, vehicle_shapes,
                       start_heading_rad, goal_heading_rad, *,
                       node_curvature=None, node_curvature_valid=None,
                       grid_sha256="", request_sha256="",
                       planning_context_sha256="",
                       max_stations=ROUTE_ASSESSMENT_MAX_STATIONS,
                       uniform_spacing_m=ROUTE_UNIFORM_SPACING_M,
                       deadline=None):
    """Bounded coarse map/hint assessment of one simplified A* route.

    No corridor coverage is claimed here: a route's chain is built only when
    the real pipeline attempts that original index.
    """
    _validate_assessment_checker(checker, model, vehicle_shapes, grid_sha256)
    values, cumulative, lengths, tangents = _guide_arc_data(points)
    if max_stations < 1:
        raise VirtualAContractError(
            "INVALID_ASSESSMENT_LIMIT", "max_stations must be positive")
    plan = _plan_assessment_stations(
        values, cumulative, lengths, tangents, [], int(max_stations),
        float(uniform_spacing_m), float(TRANSITION_WINDOW_SPACING_M),
        node_curvature, node_curvature_valid, float(start_heading_rad),
        float(goal_heading_rad))
    plan["points"] = values
    plan["cumulative"] = cumulative
    plan["lengths"] = lengths
    plan["tangents"] = tangents
    plan["node_curvature"] = node_curvature
    plan["node_curvature_valid"] = node_curvature_valid
    plan["start_heading"] = float(start_heading_rad)
    plan["goal_heading"] = float(goal_heading_rad)
    plan["capacity_stations"] = int(max_stations)
    results, deadline_hit = _station_results_for_plan(
        plan, None, checker, model, vehicle_shapes, [], deadline)
    results = _assign_station_indices(results)
    vehicle_ids = tuple(offset.vehicle_id for offset in model.offsets)
    common = _assessment_result_common(
        results, vehicle_ids, plan, [], deadline_hit,
        request_incomplete=not _shape_set_complete(model, vehicle_shapes))
    payload = {
        "schema": ROUTE_ASSESSMENT_SCHEMA,
        "route_index": int(route_index),
        "request_sha256": str(request_sha256 or ""),
        "grid_sha256": str(grid_sha256 or ""),
        "planning_context_sha256": str(planning_context_sha256 or ""),
        "point_count": len(values),
        "route_length_m": float(plan["total_length_m"]),
        "candidate_certification": CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY,
        "continuously_certified": False,
        "chain_digest": None,
    }
    payload.update(common)
    return payload


def assess_formation_guide(chain, points, checker, model, vehicle_shapes,
                           start_heading_rad, goal_heading_rad, *,
                           route_index=0, round_index=0, guide_source="",
                           allocation=None, node_curvature=None,
                           node_curvature_valid=None, grid_sha256="",
                           request_sha256="", planning_context_sha256="",
                           max_stations=FORMATION_ASSESSMENT_MAX_STATIONS,
                           uniform_spacing_m=GUIDE_UNIFORM_SPACING_M,
                           window_spacing_m=TRANSITION_WINDOW_SPACING_M,
                           deadline=None, tol_m=None):
    """Detailed finite formation assessment for one selected corridor guide.

    Every returned status is diagnostic.  The OBB certifies no continuous
    transition, never replaces the F-only allocation, and never feeds a
    ``formation_obb_hard_constraints`` flag.
    """
    if not isinstance(chain, CorridorChain):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_CHAIN", "expected CorridorChain")
    _validate_assessment_checker(checker, model, vehicle_shapes, grid_sha256)
    values, cumulative, lengths, tangents = _guide_arc_data(points)
    if len(values) != len(chain.regions) + 1:
        raise VirtualAContractError(
            "CORRIDOR_GUIDE_CHAIN_MISMATCH",
            "guide has {} points for {} regions".format(
                len(values), len(chain.regions)))
    violation = _guide_region_violation(chain, values)
    if violation > _GUIDE_PRIMAL_TOL_M:
        raise VirtualAContractError(
            "CORRIDOR_GUIDE_OUTSIDE",
            "guide leaves an assigned corridor by {} m".format(violation))
    if max_stations < 1:
        raise VirtualAContractError(
            "INVALID_ASSESSMENT_LIMIT", "max_stations must be positive")
    windows = formation_transition_windows(
        chain, values, model, vehicle_shapes)
    plan = _plan_assessment_stations(
        values, cumulative, lengths, tangents, windows, int(max_stations),
        float(uniform_spacing_m), float(window_spacing_m),
        node_curvature, node_curvature_valid, float(start_heading_rad),
        float(goal_heading_rad))
    plan["points"] = values
    plan["cumulative"] = cumulative
    plan["lengths"] = lengths
    plan["tangents"] = tangents
    plan["node_curvature"] = node_curvature
    plan["node_curvature_valid"] = node_curvature_valid
    plan["start_heading"] = float(start_heading_rad)
    plan["goal_heading"] = float(goal_heading_rad)
    plan["capacity_stations"] = int(max_stations)
    results, deadline_hit = _station_results_for_plan(
        plan, chain, checker, model, vehicle_shapes, windows, deadline)
    results = _assign_station_indices(results)
    vehicle_ids = tuple(offset.vehicle_id for offset in model.offsets)
    common = _assessment_result_common(
        results, vehicle_ids, plan, windows, deadline_hit,
        request_incomplete=not _shape_set_complete(model, vehicle_shapes))
    candidate_id = _assessment_identity(
        ASSESSMENT_SCHEMA, request_sha256, grid_sha256,
        planning_context_sha256, route_index, round_index, chain.digest,
        guide_source, values)
    payload = {
        "schema": ASSESSMENT_SCHEMA,
        "candidate_id": candidate_id,
        "route_index": int(route_index),
        "round_index": int(round_index),
        "request_sha256": str(request_sha256 or ""),
        "grid_sha256": str(grid_sha256 or ""),
        "planning_context_sha256": str(planning_context_sha256 or ""),
        "chain_digest": str(chain.digest),
        "selected_guide_source": str(guide_source or ""),
        "selected_guide_sha256": _guide_points_digest(values),
        "legacy_f_allocation": _allocation_view(allocation),
        "route_length_m": float(plan["total_length_m"]),
    }
    payload.update(common)
    return payload


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


def _candidate_points_differ(first, second):
    """Whether two guide point lists differ beyond the dedup tolerance."""
    if len(first) != len(second):
        return True
    for left, right in zip(first, second):
        if math.hypot(left[0] - right[0], left[1] - right[1]) > \
                _CANDIDATE_POINT_DEDUP_M:
            return True
    return False


def _candidate_guide(chain, points, source, portal_radii, origin):
    """Build one validated diagnostic guide candidate with C2-A quality."""
    lengths = _guide_segment_lengths(points)
    if any(length <= _GUIDE_POINT_DEDUP_M for length in lengths):
        raise VirtualAContractError(
            "CORRIDOR_GUIDE_DEGENERATE",
            "candidate guide contains a zero-length segment")
    violation = _guide_region_violation(chain, points)
    if violation > _GUIDE_PRIMAL_TOL_M:
        raise VirtualAContractError(
            "CORRIDOR_GUIDE_OUTSIDE",
            "candidate guide leaves an assigned corridor by {} m".format(
                violation))
    qp = {
        "qp_attempted": False,
        "qp_used": False,
        "qp_status": "CANDIDATE_POLYLINE",
        "qp_iterations": 0,
        "qp_objective": None,
        "fallback_objective": None,
        "fallback_reason": str(origin),
        "primal_violation_m": float(violation),
        "candidate_origin": str(origin),
    }
    quality = _guide_quality(chain, points, source, portal_radii, qp)
    quality["candidate_origin"] = str(origin)
    quality["candidate_certification"] = \
        CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY
    return GuideRoute(
        points=tuple(points), source=str(source),
        grid_sha256=chain.grid_sha256, quality=quality)


_PORTAL_RISK_PRIORITY = {
    "MAP_COLLISION": 0,
    "UNION_GAP": 1,
    "TRANSITION_UNKNOWN": 2,
}


def _portal_window_index(window):
    try:
        value = int(window.get("portal_index"))
    except (TypeError, ValueError, AttributeError):
        return None
    if value < 0:
        return None
    return value


def _collision_portal_index(cumulative, windows, s_m):
    """Portal point index for one collision arc length, or the nearest one."""
    for window in windows:
        bounds = window.get("s_window_m")
        if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
            continue
        try:
            lower = float(bounds[0])
            upper = float(bounds[1])
        except (TypeError, ValueError):
            continue
        if lower - _STATION_DEDUP_M <= s_m <= upper + _STATION_DEDUP_M:
            index = _portal_window_index(window)
            if index is not None and index + 1 < len(cumulative):
                return index + 1
    count = len(cumulative) - 1
    if count <= 1:
        return None
    return min(
        range(1, count),
        key=lambda index: (abs(cumulative[index] - s_m), index))


def select_guide_target_portal(chain, base_guide, baseline_assessment):
    """Choose one repair Portal from the real baseline assessment.

    A baseline map collision, adjacent-union gap or unconfirmed transition
    wins by its earliest (smallest) Portal point index.  When the baseline
    reports none of those risks, the Portal with the smallest Chebyshev
    radius wins, ties broken by ascending index.  The returned evidence is
    diagnostic only; it never relaxes a corridor constraint.
    """
    if not isinstance(chain, CorridorChain):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_CHAIN", "expected CorridorChain")
    if not isinstance(base_guide, GuideRoute):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_GUIDE", "base_guide must be a GuideRoute")
    count = len(chain.regions)
    evidence = {
        "schema": GUIDE_CANDIDATE_EVIDENCE_SCHEMA,
        "selection": "NO_TARGET",
        "target_portal_index": None,
        "target_window_index": None,
        "target_reason": None,
        "risk_portals": [],
        "portal_radii_m": [],
        "certification_claim": "NONE",
        "candidate_certification": CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY,
        "continuously_certified": False,
        "formation_obb_hard_constraints": False,
    }
    if count <= 1 or len(base_guide.points) != count + 1:
        return None, evidence
    values, cumulative, _lengths, _tangents = _guide_arc_data(base_guide.points)
    if len(values) != count + 1:
        return None, evidence
    baseline = baseline_assessment if isinstance(
        baseline_assessment, MappingABC) else {}
    windows = [window for window in baseline.get("transition_windows") or ()
               if isinstance(window, MappingABC)]
    risks = {}

    def record(portal_index, kind, s_m=None):
        if not isinstance(portal_index, int) or isinstance(
                portal_index, bool) or not 1 <= portal_index < count:
            return
        entry = risks.setdefault(portal_index, {
            "portal_index": int(portal_index),
            "kinds": set(),
            "first_s_m": None,
        })
        entry["kinds"].add(str(kind))
        if s_m is not None:
            try:
                value = float(s_m)
            except (TypeError, ValueError):
                value = None
            if value is not None and math.isfinite(value):
                if entry["first_s_m"] is None or value < entry["first_s_m"]:
                    entry["first_s_m"] = float(value)

    for collision in baseline.get("vehicle_map_collisions") or ():
        if not isinstance(collision, MappingABC):
            continue
        try:
            s_m = float(collision.get("s_m"))
        except (TypeError, ValueError):
            continue
        if not math.isfinite(s_m):
            continue
        record(_collision_portal_index(cumulative, windows, s_m),
               "MAP_COLLISION", s_m)
    for window in windows:
        window_index = _portal_window_index(window)
        if window_index is None:
            continue
        portal_index = window_index + 1
        portal_s_m = float(cumulative[portal_index])
        for collision in window.get("collisions") or ():
            if not isinstance(collision, MappingABC):
                continue
            record(portal_index, "MAP_COLLISION", collision.get("s_m"))
        status = str(window.get("status") or "")
        if status == "PORTAL_TRANSITION_GAP":
            record(portal_index, "UNION_GAP", portal_s_m)
        elif status == "PORTAL_TRANSITION_UNCONFIRMED":
            record(portal_index, "TRANSITION_UNKNOWN", portal_s_m)
        for vehicle in window.get("vehicles") or ():
            if not isinstance(vehicle, MappingABC):
                continue
            counts = vehicle.get("counts") or {}
            try:
                gap = int(counts.get("gap", 0) or 0)
            except (TypeError, ValueError):
                gap = 0
            if gap > 0:
                record(portal_index, "UNION_GAP", portal_s_m)
    coverage = baseline.get("coverage_vehicles") or {}
    if isinstance(coverage, MappingABC):
        for vehicle in coverage.values():
            if not isinstance(vehicle, MappingABC):
                continue
            for witness in vehicle.get("witnesses") or ():
                if not isinstance(witness, MappingABC) or \
                        witness.get("status") != COVERAGE_GAP:
                    continue
                window_index = witness.get("window_index")
                if isinstance(window_index, int) and not isinstance(
                        window_index, bool) and window_index >= 0:
                    record(window_index + 1, "UNION_GAP",
                           witness.get("s_m"))
    if risks:
        ordered = sorted(risks)
        bounded = []
        for portal_index in ordered[:8]:
            entry = risks[portal_index]
            kinds = sorted(
                entry["kinds"],
                key=lambda kind: (_PORTAL_RISK_PRIORITY.get(kind, 9), kind))
            bounded.append({
                "portal_index": int(portal_index),
                "window_index": int(portal_index - 1),
                "kinds": list(kinds),
                "first_s_m": entry["first_s_m"],
            })
        evidence["risk_portals"] = bounded
        target = int(ordered[0])
        kinds = risks[target]["kinds"]
        reason = min(
            kinds, key=lambda kind: (_PORTAL_RISK_PRIORITY.get(kind, 9),
                                     kind))
        evidence["selection"] = "BASELINE_RISK"
        evidence["target_portal_index"] = target
        evidence["target_window_index"] = target - 1
        evidence["target_reason"] = str(reason)
        return target, evidence
    radii = []
    for portal_index in range(1, count):
        try:
            _centre, _rows, _limits, radius = _portal_bundle(
                chain, portal_index)
        except VirtualAContractError:
            continue
        radii.append((int(portal_index), float(radius)))
    evidence["portal_radii_m"] = [
        [int(portal_index), float(radius)]
        for portal_index, radius in radii][:16]
    if not radii:
        evidence["selection"] = "NO_TARGET"
        return None, evidence
    target, radius = min(radii, key=lambda item: (item[1], item[0]))
    evidence["selection"] = "SMALLEST_CHEBYSHEV_RADIUS"
    evidence["target_portal_index"] = int(target)
    evidence["target_window_index"] = int(target - 1)
    evidence["target_reason"] = "SMALLEST_CHEBYSHEV_RADIUS"
    evidence["target_radius_m"] = float(radius)
    return int(target), evidence


def _portal_local_stations(cumulative, portal_index, rho):
    """Deterministic finite stations across one local Portal window."""
    total = float(cumulative[-1])
    station = float(cumulative[portal_index])
    if total <= _SHORT_GUIDE_SEGMENT_M:
        return [], False
    if rho is None or not math.isfinite(float(rho)):
        return [station], False
    lower = max(0.0, station - float(rho))
    upper = min(total, station + float(rho))
    if upper - lower <= _STATION_DEDUP_M:
        return [station], False
    intervals = max(
        1, int(math.ceil(
            (upper - lower) / TRANSITION_WINDOW_SPACING_M)))
    values = [lower + (upper - lower) * index / intervals
              for index in range(intervals + 1)]
    values.append(station)
    unique = []
    for value in sorted(values):
        if not unique or value - unique[-1] > _STATION_DEDUP_M:
            unique.append(float(value))
    if len(unique) <= PORTAL_LOCAL_ASSESSMENT_MAX_STATIONS:
        return unique, False
    # Keep both window ends and the Portal station, then fill the interior
    # deterministically.  The cap is reported and forces an UNKNOWN local
    # status, never a sampled-clear claim.
    picks = {0, len(unique) - 1}
    picks.add(min(range(len(unique)),
                  key=lambda index: abs(unique[index] - station)))
    remaining = PORTAL_LOCAL_ASSESSMENT_MAX_STATIONS - len(picks)
    if remaining > 0:
        for slot in range(remaining):
            picks.add(int(round(
                (slot + 1) * (len(unique) - 1) / (remaining + 1))))
    return sorted(unique[index] for index in picks), True


def _portal_local_assessment(chain, points, checker, model, vehicle_shapes,
                             portal_index, start_heading_rad,
                             goal_heading_rad, deadline=None):
    """Evaluate only the selected Portal's finite transition stations.

    This is a bounded pruning signal for candidate generation.  It is not a
    complete Guide assessment and never a coverage or collision certificate;
    every retained candidate still receives the normal full assessment.
    """
    values, cumulative, lengths, tangents = _guide_arc_data(points)
    rho, rho_reason = _assessment_rho(model, vehicle_shapes)
    stations, sample_cap = _portal_local_stations(
        cumulative, int(portal_index), rho)
    counts = {
        "sampled": 0,
        "map_collision": 0,
        "map_unconfirmed": 0,
        "gap": 0,
        "coverage_unconfirmed": 0,
    }
    deadline_hit = False
    if checker is None or model is None or vehicle_shapes is None:
        return {
            "status": "UNASSESSED",
            "reason": "LOCAL_CHECKER_UNAVAILABLE",
            "portal_index": int(portal_index - 1),
            "sample_cap": bool(sample_cap),
            "counts": counts,
        }
    for station in stations:
        if deadline is not None and time.monotonic() >= deadline:
            deadline_hit = True
            break
        frame = _station_meta(
            values, cumulative, lengths, tangents, float(station),
            None, None, float(start_heading_rad), float(goal_heading_rad))
        frame["window_indices"] = [int(portal_index - 1)]
        result = _evaluate_station(
            frame, chain, checker, model, vehicle_shapes, _ASSESSMENT_TOL_M)
        counts["sampled"] += 1
        map_status = result["map"]["status"]
        if map_status == MAP_COLLISION_OBSERVED:
            counts["map_collision"] += 1
        elif map_status != MAP_CLEAR_AT_SAMPLES:
            counts["map_unconfirmed"] += 1
        for coverage in result["coverage"].values():
            status = coverage.get("status")
            if status == COVERAGE_GAP:
                counts["gap"] += 1
            elif status != COVERAGE_SINGLE_REGION and \
                    status != COVERAGE_ADJACENT_UNION:
                counts["coverage_unconfirmed"] += 1
    complete = (
        counts["sampled"] == len(stations) and not sample_cap
        and not deadline_hit)
    if complete:
        status, reason = "ASSESSED", ""
    elif deadline_hit:
        status, reason = "UNKNOWN", "SOFT_BUDGET"
    elif sample_cap:
        status, reason = "UNKNOWN", "LOCAL_SAMPLE_CAP"
    else:
        status, reason = "UNKNOWN", (
            rho_reason or "LOCAL_ASSESSMENT_INCOMPLETE")
    return {
        "status": status,
        "reason": str(reason),
        "portal_index": int(portal_index - 1),
        "station_count": len(stations),
        "sample_cap": bool(sample_cap),
        "counts": counts,
    }


def _portal_local_rank(local, point_index):
    """Deterministic local pruning order before the full assessment.

    A fully assessed local sample outranks a capped/pre-empted one.  Partial
    observed counts still order the remaining candidates; an UNKNOWN status
    is never treated as clear, and the full assessment remains authoritative.
    """
    index = int(point_index)
    counts = local.get("counts") if isinstance(local, MappingABC) else None
    if not isinstance(counts, MappingABC):
        return (2, 0, 0, 0, 0, index)
    try:
        collisions = int(counts.get("map_collision", 0) or 0)
        gaps = int(counts.get("gap", 0) or 0)
        unknown = int(counts.get("map_unconfirmed", 0) or 0) + \
            int(counts.get("coverage_unconfirmed", 0) or 0)
        sampled = int(counts.get("sampled", 0) or 0)
    except (TypeError, ValueError):
        return (2, 0, 0, 0, 0, index)
    tier = 0 if (isinstance(local, MappingABC)
                 and local.get("status") == "ASSESSED") else (
        1 if sampled > 0 else 2)
    return (tier, int(collisions > 0), collisions, gaps, unknown, index)


def _portal_local_candidate_points(chain, base, portal_index):
    """At most four deterministic interior points of one Portal intersection."""
    centre, _rows, _limits, _radius = _portal_bundle(chain, portal_index)
    _measured_radius, _area, vertices = _portal_measure(
        chain.regions[portal_index - 1], chain.regions[portal_index])
    base_point = base.points[portal_index]
    points = []
    for vertex in vertices:
        point = (0.5 * (centre[0] + float(vertex[0])),
                 0.5 * (centre[1] + float(vertex[1])))
        if math.hypot(point[0] - centre[0],
                      point[1] - centre[1]) <= _CANDIDATE_POINT_DEDUP_M:
            continue
        if math.hypot(point[0] - base_point[0],
                      point[1] - base_point[1]) <= _CANDIDATE_POINT_DEDUP_M:
            continue
        if any(math.hypot(point[0] - other[0],
                          point[1] - other[1]) <= _CANDIDATE_POINT_DEDUP_M
               for other in points):
            continue
        points.append((float(point[0]), float(point[1])))
        if len(points) >= PORTAL_LOCAL_CANDIDATE_POINT_LIMIT:
            break
    return points


def plan_guide_candidates(chain, start, goal, base_guide=None,
                          baseline_assessment=None, checker=None, model=None,
                          vehicle_shapes=None, start_heading_rad=0.0,
                          goal_heading_rad=0.0, deadline=None):
    """Bounded deterministic Guide candidates with baseline-driven repair.

    The returned tuple always starts with the caller's exact QP guide.  The
    all-Chebyshev-centre polyline follows when it differs.  Local variants
    are generated only from a real baseline assessment: one target Portal is
    chosen (earliest baseline collision/gap/unknown transition, otherwise the
    smallest Chebyshev radius), up to four interior points
    ``0.5 * centre + 0.5 * vertex`` are formed, evaluated at that Portal's
    local transition stations, and at most three are kept.  Every candidate
    is checked against the same assigned-corridor halfspaces and must be
    re-allocated by the caller.  Total candidates remain capped at five.
    """
    if not isinstance(chain, CorridorChain):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_CHAIN", "expected CorridorChain")
    base = (make_corridor_guide(chain, start, goal)
            if base_guide is None else base_guide)
    if not isinstance(base, GuideRoute):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_GUIDE", "base_guide must be a GuideRoute")
    candidates = [base]
    count = len(chain.regions)
    evidence = {
        "schema": GUIDE_CANDIDATE_EVIDENCE_SCHEMA,
        "selection": "NO_CANONICAL_BASE",
        "target_portal_index": None,
        "target_window_index": None,
        "target_reason": None,
        "risk_portals": [],
        "portal_radii_m": [],
        "local_candidate_point_count": 0,
        "local_candidate_count": 0,
        "local_candidates_kept": 0,
        "local_candidates": [],
        "dropped_local_candidates": [],
        "certification_claim": "NONE",
        "candidate_certification": CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY,
        "continuously_certified": False,
        "formation_obb_hard_constraints": False,
    }
    if count <= 1 or len(base.points) != count + 1:
        return tuple(candidates[:CORRIDOR_GUIDE_CANDIDATE_LIMIT]), evidence
    centres = []
    radii = []
    for portal_index in range(1, count):
        centre, _rows, _limits, radius = _portal_bundle(chain, portal_index)
        centres.append(centre)
        radii.append(float(radius))
    centre_points = [base.points[0]] + list(centres) + [base.points[-1]]
    if _candidate_points_differ(centre_points, base.points):
        try:
            candidates.append(_candidate_guide(
                chain, centre_points, "corridor_candidate_chebyshev",
                radii, "CHEBYSHEV_CENTRE_GUIDE"))
        except VirtualAContractError:
            pass
    if not isinstance(baseline_assessment, MappingABC):
        evidence["selection"] = "NO_BASELINE_ASSESSMENT"
        return tuple(candidates[:CORRIDOR_GUIDE_CANDIDATE_LIMIT]), evidence
    target, target_evidence = select_guide_target_portal(
        chain, base, baseline_assessment)
    evidence.update(target_evidence)
    if target is None:
        return tuple(candidates[:CORRIDOR_GUIDE_CANDIDATE_LIMIT]), evidence
    raw_points = _portal_local_candidate_points(chain, base, target)
    evidence["local_candidate_point_count"] = len(raw_points)
    local_items = []
    for point_index, point in enumerate(raw_points):
        points = list(base.points)
        points[target] = (float(point[0]), float(point[1]))
        if not _candidate_points_differ(points, base.points):
            evidence["dropped_local_candidates"].append({
                "point_index": int(point_index),
                "point": [float(point[0]), float(point[1])],
                "reason": "DUPLICATE_BASE_POINT",
            })
            continue
        if any(not _candidate_points_differ(points, existing.points)
               for existing in candidates):
            evidence["dropped_local_candidates"].append({
                "point_index": int(point_index),
                "point": [float(point[0]), float(point[1])],
                "reason": "DUPLICATE_EXISTING_CANDIDATE",
            })
            continue
        try:
            guide = _candidate_guide(
                chain, points,
                "corridor_candidate_portal_{}".format(target - 1),
                radii, "ONE_PORTAL_VARIANT_{}".format(target - 1))
        except VirtualAContractError as exc:
            evidence["dropped_local_candidates"].append({
                "point_index": int(point_index),
                "point": [float(point[0]), float(point[1])],
                "reason": str(exc.reason),
                "detail": str(exc.detail)[:160],
            })
            continue
        local = _portal_local_assessment(
            chain, guide.points, checker, model, vehicle_shapes, target,
            start_heading_rad, goal_heading_rad, deadline=deadline)
        guide.quality["target_portal_index"] = int(target)
        guide.quality["target_window_index"] = int(target - 1)
        guide.quality["local_portal_assessment"] = dict(local)
        local_items.append({
            "guide": guide,
            "local": local,
            "point_index": int(point_index),
            "point": (float(point[0]), float(point[1])),
        })
    local_items.sort(
        key=lambda item: _portal_local_rank(
            item["local"], item["point_index"]))
    kept = local_items[:CORRIDOR_GUIDE_PORTAL_VARIANT_LIMIT]
    evidence["local_candidate_count"] = len(local_items)
    evidence["local_candidates_kept"] = len(kept)
    evidence["local_candidates"] = [
        {
            "point_index": int(item["point_index"]),
            "point": [float(item["point"][0]),
                      float(item["point"][1])],
            "status": str(item["local"].get("status") or ""),
            "reason": str(item["local"].get("reason") or ""),
            "counts": dict(item["local"].get("counts") or {}),
            "kept": True,
        }
        for item in kept]
    for item in local_items[CORRIDOR_GUIDE_PORTAL_VARIANT_LIMIT:]:
        evidence["dropped_local_candidates"].append({
            "point_index": int(item["point_index"]),
            "point": [float(item["point"][0]),
                      float(item["point"][1])],
            "reason": "LOCAL_CANDIDATE_LIMIT",
            "status": str(item["local"].get("status") or ""),
            "counts": dict(item["local"].get("counts") or {}),
        })
    for item in kept:
        candidates.append(item["guide"])
    return tuple(candidates[:CORRIDOR_GUIDE_CANDIDATE_LIMIT]), evidence


def corridor_guide_candidates(chain, start, goal, base_guide=None,
                              baseline_assessment=None, checker=None,
                              model=None, vehicle_shapes=None,
                              start_heading_rad=0.0,
                              goal_heading_rad=0.0, deadline=None):
    """Tuple-only wrapper around :func:`plan_guide_candidates`."""
    candidates, _evidence = plan_guide_candidates(
        chain, start, goal, base_guide=base_guide,
        baseline_assessment=baseline_assessment, checker=checker,
        model=model, vehicle_shapes=vehicle_shapes,
        start_heading_rad=start_heading_rad,
        goal_heading_rad=goal_heading_rad, deadline=deadline)
    return candidates


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


def _display_point_at_s(values, cumulative, s_m):
    """Interpolate the guide reference point at one real arc length."""
    s_m = min(max(float(s_m), 0.0), float(cumulative[-1]))
    for index, (first, second) in enumerate(zip(values, values[1:])):
        segment_end = float(cumulative[index + 1])
        if s_m <= segment_end or index == len(values) - 2:
            length = segment_end - float(cumulative[index])
            if length <= _SHORT_GUIDE_SEGMENT_M:
                return (first[0], first[1])
            ratio = (s_m - float(cumulative[index])) / length
            return (first[0] + ratio * (second[0] - first[0]),
                    first[1] + ratio * (second[1] - first[1]))
    return (values[-1][0], values[-1][1])


def _display_optional_point(value, name):
    """Copy one real display coordinate; return ``None`` when malformed."""
    try:
        x, y = float(value[0]), float(value[1])
    except (TypeError, ValueError, IndexError, KeyError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return (x, y)


def _display_count(value):
    try:
        return max(0, int(value))
    except (TypeError, ValueError):
        return 0


def corridor_portal_diagnostics(round_state, *, guide_points,
                                guide_source=None):
    """Bounded display-only samples of one round's real assessments.

    The returned markers are derived exclusively from the selected guide and
    its already-computed finite assessment: sampled map collisions, coverage
    gap witnesses and the finite Portal transition windows.  Collision sample
    positions are the guide reference point at the assessed arc length, not a
    contact point or a reconstructed vehicle pose; every claim remains
    ``DIAGNOSTIC_ONLY`` and nothing here can certify playback.

    The lists are capped for RViz.  Totals in ``counts`` stay the full
    assessment counts so a capped marker layer never understates the sampled
    risk.
    """
    if not isinstance(round_state, MappingABC):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_DIAGNOSTICS",
            "round state must be a mapping")
    assessment = round_state.get("formation_assessment")
    if not isinstance(assessment, MappingABC):
        return None
    values, cumulative, _lengths, _tangents = _guide_arc_data(guide_points)
    collisions = []
    for entry in assessment.get("vehicle_map_collisions") or ():
        if not isinstance(entry, MappingABC):
            continue
        s_m = _display_optional_point(
            (entry.get("s_m"), 0.0), "collision s_m")
        if s_m is None:
            continue
        point = _display_point_at_s(values, cumulative, s_m[0])
        vehicle_id = entry.get("vehicle_id")
        station_index = entry.get("station_index")
        collisions.append({
            "s_m": float(s_m[0]),
            "point": [point[0], point[1]],
            "vehicle_id": (
                int(vehicle_id) if isinstance(vehicle_id, int)
                and not isinstance(vehicle_id, bool) else None),
            "station_index": (
                int(station_index) if isinstance(station_index, int)
                and not isinstance(station_index, bool) else None),
            "reason": str(entry.get("reason") or ""),
            "other_vehicle_id": (
                int(entry["other_vehicle_id"])
                if isinstance(entry.get("other_vehicle_id"), int)
                and not isinstance(entry.get("other_vehicle_id"), bool)
                else None),
        })
    collisions = collisions[:PORTAL_DIAGNOSTIC_COLLISION_LIMIT]
    coverage = assessment.get("coverage_vehicles") or {}
    gaps = []
    if isinstance(coverage, MappingABC):
        for vehicle_key in sorted(coverage, key=lambda item: str(item)):
            vehicle = coverage.get(vehicle_key)
            if not isinstance(vehicle, MappingABC):
                continue
            vehicle_id = vehicle.get("vehicle_id")
            if not isinstance(vehicle_id, int) or isinstance(
                    vehicle_id, bool):
                try:
                    vehicle_id = int(vehicle_key)
                except (TypeError, ValueError):
                    continue
            for witness in vehicle.get("witnesses") or ():
                if not isinstance(witness, MappingABC) or \
                        witness.get("status") != COVERAGE_GAP:
                    continue
                s_value = witness.get("s_m")
                try:
                    s_m = float(s_value)
                except (TypeError, ValueError):
                    continue
                if not math.isfinite(s_m):
                    continue
                point = None
                detail = witness.get("witness")
                if isinstance(detail, MappingABC):
                    point = _display_optional_point(
                        detail.get("vertex"), "gap witness vertex")
                if point is None:
                    point = _display_point_at_s(values, cumulative, s_m)
                gaps.append({
                    "s_m": float(s_m),
                    "point": [point[0], point[1]],
                    "vehicle_id": int(vehicle_id),
                    "station_index": witness.get("station_index"),
                    "status": COVERAGE_GAP,
                    "reason": str(witness.get("reason") or ""),
                    "region_ids": [
                        int(region_id)
                        for region_id in witness.get("region_ids") or ()
                        if isinstance(region_id, int)
                        and not isinstance(region_id, bool)],
                    "window_index": witness.get("window_index"),
                    "max_violation_m": (
                        None if witness.get("max_violation_m") is None
                        else float(witness["max_violation_m"])),
                })
    gaps.sort(key=lambda entry: (
        entry["s_m"], entry["vehicle_id"]))
    gaps = gaps[:PORTAL_DIAGNOSTIC_GAP_LIMIT]
    windows = []
    for window in assessment.get("transition_windows") or ():
        if not isinstance(window, MappingABC):
            continue
        bounds = window.get("s_window_m")
        start = end = None
        if isinstance(bounds, (list, tuple)) and len(bounds) == 2:
            try:
                start = _display_point_at_s(
                    values, cumulative, float(bounds[0]))
                end = _display_point_at_s(
                    values, cumulative, float(bounds[1]))
            except (TypeError, ValueError):
                start = end = None
        if start is None:
            continue
        portal_index = window.get("portal_index")
        vehicles = []
        for vehicle in window.get("vehicles") or ():
            if not isinstance(vehicle, MappingABC):
                continue
            vehicle_id = vehicle.get("vehicle_id")
            if not isinstance(vehicle_id, int) or isinstance(
                    vehicle_id, bool):
                continue
            counts = vehicle.get("counts") or {}
            vehicles.append({
                "vehicle_id": int(vehicle_id),
                "union_cover_status": str(
                    vehicle.get("union_cover_status") or ""),
                "counts": {
                    "single": _display_count(counts.get("single")),
                    "union": _display_count(counts.get("union")),
                    "gap": _display_count(counts.get("gap")),
                    "unconfirmed": _display_count(
                        counts.get("unconfirmed")),
                },
                "min_covered_margin_m": (
                    None if vehicle.get("min_covered_margin_m") is None
                    else float(vehicle["min_covered_margin_m"])),
                "worst_gap_violation_m": (
                    None if vehicle.get("worst_gap_violation_m") is None
                    else float(vehicle["worst_gap_violation_m"])),
            })
        seed = _display_optional_point(
            window.get("portal_seed_point"), "portal seed point")
        windows.append({
            "portal_index": (
                int(portal_index) if isinstance(portal_index, int)
                and not isinstance(portal_index, bool) else None),
            "source_region_index": window.get("source_region_index"),
            "target_region_index": window.get("target_region_index"),
            "s_window_m": [float(bounds[0]), float(bounds[1])],
            "start": [start[0], start[1]],
            "end": [end[0], end[1]],
            "portal_seed_point": (
                None if seed is None else [seed[0], seed[1]]),
            "status": str(window.get("status") or ""),
            "map_sample_status": str(
                window.get("map_sample_status") or ""),
            "vehicles": vehicles,
        })
    windows = windows[:PORTAL_DIAGNOSTIC_WINDOW_LIMIT]
    map_counts = assessment.get("map_sample_counts") or {}
    single_region = 0
    union_covered = 0
    union_gap = 0
    union_unknown = 0
    if isinstance(coverage, MappingABC):
        for vehicle in coverage.values():
            if not isinstance(vehicle, MappingABC):
                continue
            counts = vehicle.get("counts") or {}
            # SINGLE_REGION and COVERED_BY_ADJACENT_UNION are distinct
            # states; neither may be folded into the other's total.
            single_region += _display_count(counts.get("single"))
            union_covered += _display_count(counts.get("union"))
            union_gap += _display_count(counts.get("gap"))
            union_unknown += _display_count(counts.get("unconfirmed"))
    window_total = len(assessment.get("transition_windows") or ())
    source_name = guide_source if guide_source is not None else \
        assessment.get("selected_guide_source")
    return {
        "schema": PORTAL_DIAGNOSTICS_SCHEMA,
        "guide_source": str(source_name or ""),
        "chain_digest": (
            None if assessment.get("chain_digest") is None
            else str(assessment["chain_digest"])),
        "assessment_candidate_id": str(
            assessment.get("candidate_id") or ""),
        "map_sample_status": str(
            assessment.get("map_sample_status") or ""),
        "map_sample_counts": {
            "sampled": _display_count(map_counts.get("sampled")),
            "clear": _display_count(map_counts.get("clear")),
            "map_collision": _display_count(
                map_counts.get("map_collision")),
            "vehicle_collision": _display_count(
                map_counts.get("vehicle_collision")),
            "unconfirmed": _display_count(map_counts.get("unconfirmed")),
        },
        "station_count": _display_count(assessment.get("station_count")),
        "counts": {
            "map_collision_total": (
                _display_count(map_counts.get("map_collision"))
                + _display_count(map_counts.get("vehicle_collision"))),
            "single_region_total": int(single_region),
            "union_covered_total": int(union_covered),
            "union_gap_total": int(union_gap),
            "union_unknown_total": int(union_unknown),
            "portal_window_total": int(window_total),
            "map_collision_marker_count": len(collisions),
            "coverage_gap_marker_count": len(gaps),
            "portal_window_marker_count": len(windows),
        },
        "limits": {
            "map_collision_markers": PORTAL_DIAGNOSTIC_COLLISION_LIMIT,
            "coverage_gap_markers": PORTAL_DIAGNOSTIC_GAP_LIMIT,
            "portal_window_markers": PORTAL_DIAGNOSTIC_WINDOW_LIMIT,
        },
        "sample_point_semantics": "GUIDE_REFERENCE_POINT_AT_ARC_LENGTH",
        "map_collision_samples": collisions,
        "coverage_gaps": gaps,
        "portal_windows": windows,
        "certification_claim": "NONE",
        "candidate_certification": CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY,
        "formation_obb_hard_constraints": False,
        "continuously_certified": False,
    }


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
    "ASSESSMENT_SCHEMA",
    "CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY",
    "CORRIDOR_DISPLAYS_KEY",
    "CORRIDOR_DISPLAY_STATE",
    "CORRIDOR_GUIDE_CANDIDATE_LIMIT",
    "CORRIDOR_GUIDE_PORTAL_VARIANT_LIMIT",
    "COVERAGE_ADJACENT_UNION",
    "COVERAGE_GAP",
    "COVERAGE_SINGLE_REGION",
    "COVERAGE_UNCONFIRMED",
    "FORMATION_ASSESSMENT_MAX_STATIONS",
    "GUIDE_CANDIDATE_EVIDENCE_SCHEMA",
    "GUIDE_HINT_DEGENERATE",
    "GUIDE_HINT_LOW_CONFIDENCE",
    "GUIDE_HINT_VALID",
    "KINEMATIC_CURVATURE_UNCONFIRMED",
    "KINEMATIC_NORMAL_BRANCH_INVALID",
    "KINEMATIC_NORMAL_BRANCH_OK",
    "MAP_CLEAR_AT_SAMPLES",
    "MAP_COLLISION_OBSERVED",
    "MAP_SAMPLE_UNCONFIRMED",
    "PORTAL_DIAGNOSTIC_COLLISION_LIMIT",
    "PORTAL_DIAGNOSTIC_GAP_LIMIT",
    "PORTAL_LOCAL_ASSESSMENT_MAX_STATIONS",
    "PORTAL_LOCAL_CANDIDATE_POINT_LIMIT",
    "PORTAL_DIAGNOSTIC_WINDOW_LIMIT",
    "PORTAL_DIAGNOSTICS_SCHEMA",
    "ROUTE_ASSESSMENT_MAX_STATIONS",
    "ROUTE_ASSESSMENT_SCHEMA",
    "TRANSITION_WINDOW_SPACING_M",
    "allocate_corridors",
    "assess_formation_guide",
    "assess_route_guide",
    "audit_corridor_chain",
    "audit_path_in_corridors",
    "build_corridor_chain",
    "corridor_chain_from_payload",
    "corridor_display_fields",
    "corridor_effective_margin",
    "corridor_guide_candidates",
    "corridor_min_portal_radius",
    "corridor_portal_diagnostics",
    "formation_transition_windows",
    "make_corridor_guide",
    "obb_coverage_in_adjacent_union",
    "plan_guide_candidates",
    "select_guide_target_portal",
    "obb_coverage_in_corridor",
    "obb_polygon",
    "obb_support_halfwidth",
    "validate_corridor_allocation",
]