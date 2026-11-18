"""Endpoint-driven automatic G2 geometry optimiser for A-only formations.

Only the physical request enters :func:`solve_open_path`.  The analytic
existence profile used by tests is never imported, looked up by plan ID, or
used as a warm start here.  Initial values are constructed from the endpoint
translation/heading and a generic clamped cubic-spline C2 projection.
"""
from dataclasses import dataclass
import inspect
import math
import numbers
import time

import numpy as np

from ..continuous_geometry import ControlKnot
from ..formation import FormationPose
from .contracts import (
    CorridorChain,
    DEFAULT_INITIAL_LENGTH_MULTIPLIERS,
    GEOMETRY_STATUSES,
    GeometryAuditConfig,
    GeometryResult,
    GuideRoute,
    InitialGuess,
    MAX_INITIALS,
    ObstacleContext,
    PlannerConfig,
    AttemptRecord,
    VirtualARequest,
    VirtualAContractError,
    _wrap,
    coerce_goal_policy,
    request_fingerprint,
    request_summary,
    validate_geometry_audit_config,
    validate_planner_config,
    validate_virtual_a_request,
)
from .corridor import (
    corridor_effective_margin,
    corridor_min_portal_radius,
    validate_corridor_allocation,
)
from .geometry import C2CurvatureProfile, VirtualAPath
from .audit import audit_geometry


SOLVER_IPOPT = "ipopt"
SOLVER_SCIPY = "scipy_slsqp"
SOLVER_SKIP = "not_run"
# The raw G2 objective strongly prefers a straight path because the
# curvature-rate penalty dominates.  A route-guide position residual therefore
# needs a weight far above the historical soft-guidance scale to select the
# corridor topology.  Values near 1e8 make IPOPT's search direction ill-
# conditioned; 5e5 keeps the S2 narrow-throat clearance above the G4 margin
# while remaining soft enough for a collision retry to relax the guide.
ROUTE_GUIDANCE_WEIGHT = 5.0e5
ROUTE_CURVATURE_WEIGHT = 100.0
# C3 scheduling: with at least one valid route, every initial slot may be
# used at most once per distinct route, at least one slot is a true
# endpoint-only shape, and the remaining slots draw from at most this many
# distinct default length/curvature shapes (both signs each).  No route
# topology keeps the historical G2 automatic sequence unchanged.
C3_ENDPOINT_SHAPES = 2
# The cached clearance proxy is a soft body-footprint penalty, not an audit.
# It is normalised over all collocation samples and vehicles, so a typical
# hinge of a few centimetres contributes only a few tens of objective units
# next to the route term.  A fixed obstacle direction per sample keeps the
# CasADi graph smooth enough for IPOPT.
CLEARANCE_PROXY_WEIGHT = 5.0e4
CLEARANCE_PROXY_TARGET_M = 0.05
_GAUSS_CACHE = {}


class _SoftDeadline(Exception):
    """Raised from the SLSQP iteration callback; soft, not hard real-time."""


def _gauss(count):
    count = int(count)
    if count not in _GAUSS_CACHE:
        nodes, weights = np.polynomial.legendre.leggauss(count)
        _GAUSS_CACHE[count] = (0.5 * (nodes + 1.0), 0.5 * weights)
    return _GAUSS_CACHE[count]


def _coeffs(h, k0, k1, eta0, eta1):
    return (
        k0,
        h * eta0,
        3.0 * (k1 - k0) - h * (2.0 * eta0 + eta1),
        2.0 * (k0 - k1) + h * (eta0 + eta1),
    )


def _piece_position_samples(h, coeffs, psi0, q_values, sin, cos,
                            quadrature_count=None):
    """Prefix-integral positions inside one cubic curvature piece.

    For each ``q`` in ``q_values`` the displacement is
    ``h q sum_l w_l e(psi0 + h q xi_l [c0 + ...])``, i.e. the Gauss
    quadrature of the prefix integral from the piece start.  It is never the
    cumulative full-weight sum of the whole piece.  ``sin``/``cos`` let the
    same routine serve ``math`` and CasADi scalars.

    The historical callers integrate every collocation value with an equally
    sized Gauss rule.  Corridor constraints instead sample an arbitrary
    allocation-driven set of ``q`` values and must keep their mandated
    9-point rule, hence the optional ``quadrature_count``.  ``q == 0`` is the
    exact piece start and needs no quadrature.
    """
    c0, c1, c2, c3 = coeffs
    count = len(q_values) if quadrature_count is None else int(
        quadrature_count)
    nodes, weights = _gauss(count)
    displacements = []
    headings = []
    for q in q_values:
        if q == 0:
            displacements.append((0.0, 0.0))
            headings.append(psi0)
            continue
        dx = 0.0
        dy = 0.0
        for node, weight in zip(nodes, weights):
            t = q * node
            angle = psi0 + h * t * (
                c0 + t * (c1 / 2.0 + t * (c2 / 3.0 + t * c3 / 4.0)))
            dx = dx + weight * cos(angle)
            dy = dy + weight * sin(angle)
        scale = h * q
        displacements.append((scale * dx, scale * dy))
        headings.append(psi0 + h * q * (
            c0 + q * (c1 / 2.0 + q * (c2 / 3.0 + q * c3 / 4.0))))
    return displacements, headings


def _search_packet(request, config):
    """Endpoint-only local target and engineering search domain."""
    start = request.start_qf
    goal = request.goal_qf
    cosine, sine = math.cos(start.psi), math.sin(start.psi)
    dx = cosine * (goal.x - start.x) + sine * (goal.y - start.y)
    dy = -sine * (goal.x - start.x) + cosine * (goal.y - start.y)
    distance = math.hypot(dx, dy)
    heading = _wrap(goal.psi - start.psi)
    intervals = int(config.intervals)
    lower = max(intervals * 0.05, distance)
    upper = min(60.0, max(12.0, 3.0 * distance + 10.0))
    return {
        "start": start,
        "dx": dx,
        "dy": dy,
        "distance": distance,
        "heading_change_rad": heading,
        "S_lower": lower,
        "S_upper": upper,
        "l0": max(1.0, distance),
        "intervals": intervals,
        "eta_limit": float(config.eta_limit),
        "kappa_limit": float(config.kappa_limit),
        "cmax": 1.0 / float(request.limits.r_min_ackermann),
    }


def _project_etas(kappas, h, eta_limit):
    """Clamped cubic-spline C2 projection with eta_0 = eta_N = 0.

    The C2 condition at an internal node with equal spacing is
    ``e[j-1] + 4 e[j] + e[j+1] = 3 (k[j+1] - k[j-1]) / h``.
    """
    kappas = np.asarray(kappas, dtype=float)
    n = len(kappas) - 1
    if n < 2:
        return kappas, np.zeros(n + 1)
    matrix = np.zeros((n - 1, n - 1), dtype=float)
    rhs = np.zeros(n - 1, dtype=float)
    for j in range(1, n):
        row = j - 1
        if j - 1 > 0:
            matrix[row, j - 2] = 1.0
        matrix[row, j - 1] = 4.0
        if j + 1 < n:
            matrix[row, j] = 1.0
        rhs[row] = 3.0 * (kappas[j + 1] - kappas[j - 1]) / h
    etas = np.zeros(n + 1, dtype=float)
    etas[1:n] = np.linalg.solve(matrix, rhs)
    if not np.all(np.isfinite(etas)):
        raise VirtualAContractError(
            "INITIAL_PROJECTION_FAILURE", "C2 projection produced nonfinite eta")
    maximum = float(np.max(np.abs(etas)))
    if eta_limit > 0.0 and maximum > eta_limit:
        scale = eta_limit / maximum
        # Scale both the sampled curvature and its C2 projection.  Scaling k
        # and eta by the same factor preserves the C2 linear relations and the
        # zero boundaries.
        kappas = scale * kappas
        etas = scale * etas
    return kappas, etas


def _guess_for(request, config, packet, multiplier, sign):
    n = packet["intervals"]
    distance = packet["distance"]
    S_lower, S_upper = packet["S_lower"], packet["S_upper"]
    base_length = max(distance, 1.0)
    S_guess = multiplier * base_length
    S_guess = min(S_upper, max(S_lower, S_guess))
    u = np.linspace(0.0, 1.0, n + 1)
    heading = packet["heading_change_rad"]
    lateral = packet["dy"]
    # S1(u)=4u^3-3u^4: psi(0)=psi'(0)=psi''(0)=0 and psi(1)=1 with
    # k(1)=eta(1)=0.  A(u)=u^3(1-u)^3 has zero value/slope/second derivative
    # at both ends; it gives the endpoint a generic lateral-shape degree of
    # freedom without constraining the test-specific endpoint.
    smooth = 4.0 * u ** 3 - 3.0 * u ** 4
    antisymmetric = u ** 3 * (1.0 - u) ** 3
    smooth_slope = 12.0 * u ** 2 - 12.0 * u ** 3
    anti_slope = 3.0 * u ** 2 * (1.0 - u) ** 2 * (1.0 - 2.0 * u)
    approximate_need = 140.0 * (lateral / S_guess - 0.4 * heading)
    if abs(approximate_need) < 0.5:
        correction = float(sign) * 0.5
    else:
        correction = float(sign) * min(20.0, abs(approximate_need))
    psi = heading * smooth + correction * antisymmetric
    kappas = (heading * smooth_slope + correction * anti_slope) / S_guess
    kappas[0] = 0.0
    # The initial always uses the zero-terminal spline basis.  This satisfies
    # STRAIGHT and is also legal for FREE; FREE is not forced to remain zero
    # by the NLP or the audit.
    kappas = np.clip(kappas, -packet["kappa_limit"], packet["kappa_limit"])
    h = S_guess / n
    try:
        kappas, etas = _project_etas(kappas, h, packet["eta_limit"])
    except VirtualAContractError:
        raise
    node_s = tuple(j * S_guess / n for j in range(n + 1))
    metadata = {
        "endpoint_local_dx_m": packet["dx"],
        "endpoint_local_dy_m": packet["dy"],
        "endpoint_distance_m": distance,
        "intervals": n,
        "heading_change_rad": heading,
        "length_multiplier": multiplier,
        "shape_sign": sign,
        "correction_coefficient": correction,
        "approximate_lateral_need": approximate_need,
        "S_search_lower_m": S_lower,
        "S_search_upper_m": S_upper,
        "projection": "clamped_cubic_spline_C2_eta0_etaN_zero",
        "initial_terminal_kappa": float(kappas[-1]),
        "initial_terminal_eta": float(etas[-1]),
    }
    return InitialGuess(
        source="automatic_endpoint_shape",
        label="length_multiplier={:.1f};shape={}".format(
            multiplier, "left" if sign > 0 else "right"),
        S=float(S_guess),
        kappas=tuple(float(v) for v in kappas),
        etas=tuple(float(v) for v in etas),
        length_multiplier=float(multiplier),
        shape_sign=int(sign),
        node_s=node_s,
        metadata=metadata)


def _guide_cumulative(guide):
    if not isinstance(guide, GuideRoute):
        raise VirtualAContractError(
            "INVALID_GUIDE_ROUTE", "expected GuideRoute instances")
    raw = np.asarray(guide.points, dtype=float)
    keep = [raw[0]]
    for point in raw[1:]:
        if float(np.linalg.norm(point - keep[-1])) > 1e-12:
            keep.append(point)
    raw = np.asarray(keep, dtype=float)
    if len(raw) < 2:
        raise VirtualAContractError(
            "INVALID_GUIDE_ROUTE", "degenerate zero-length guide")
    steps = np.linalg.norm(np.diff(raw, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(steps)))
    total = float(cumulative[-1])
    if not math.isfinite(total) or total <= 0.0:
        raise VirtualAContractError(
            "INVALID_GUIDE_ROUTE", "guide length is not positive")
    return raw, cumulative, total


def _route_arrays(guide, count):
    """Resample a guide polyline to ``count`` equal-arc points.

    The guide is a topology hint only.  Its raw cell-centre staircase is never
    published as a path; only smoothed tangent/curvature estimates enter an
    initial value for the endpoint NLP.
    """
    raw, cumulative, total = _guide_cumulative(guide)
    samples = np.linspace(0.0, total, int(count))
    points = np.column_stack((
        np.interp(samples, cumulative, raw[:, 0]),
        np.interp(samples, cumulative, raw[:, 1])))
    return points, total


def _route_fraction_points(guide, request, fractions):
    """Sample the guide at arc-length fractions and map to start-local x/y.

    The endpoint NLP compares node positions with equal-arc fractions of the
    guide.  Its collocation samples use the same convention, so a guide that
    is slightly longer than the executed path is followed uniformly rather
    than only at nine knots.
    """
    raw, cumulative, total = _guide_cumulative(guide)
    fractions = np.asarray(fractions, dtype=float)
    if fractions.size == 0:
        return np.zeros((0, 2), dtype=float)
    if not np.all(np.isfinite(fractions)):
        raise VirtualAContractError(
            "INVALID_GUIDE_ROUTE", "guide sampling fractions are not finite")
    samples = np.clip(fractions, 0.0, 1.0) * total
    global_points = np.column_stack((
        np.interp(samples, cumulative, raw[:, 0]),
        np.interp(samples, cumulative, raw[:, 1])))
    start = request.start_qf
    cosine, sine = math.cos(start.psi), math.sin(start.psi)
    dx = global_points[:, 0] - start.x
    dy = global_points[:, 1] - start.y
    return np.column_stack(
        (cosine * dx + sine * dy, -sine * dx + cosine * dy))


def _node_route_curvature(points):
    """Signed circumcircle curvature at the resampled guide nodes.

    A heading gradient over the nine nodes erases the guide shape, while a
    dense-polyline derivative amplifies the A* cell-centre staircase.  The
    three-point circumcircle at the node scale follows the route topology
    (turn direction and approximate amplitude) and remains a soft initial
    value only; the endpoint NLP and the three audits stay authoritative.
    """
    local = np.asarray(points, dtype=float)
    count = int(local.shape[0])
    curvature = np.zeros(count, dtype=float)
    for index in range(1, count - 1):
        first = local[index] - local[index - 1]
        second = local[index + 1] - local[index]
        chord = local[index + 1] - local[index - 1]
        denominator = (np.linalg.norm(first) * np.linalg.norm(second)
                       * np.linalg.norm(chord))
        if denominator <= 1e-12:
            continue
        cross = float(first[0] * second[1] - first[1] * second[0])
        curvature[index] = 2.0 * cross / denominator
    return curvature


def _route_guess(request, config, packet, guide, route_index,
                 obstacle_context=None):
    n = int(packet["intervals"])
    points, route_length = _route_arrays(guide, n + 1)
    # A collision-driven retry may ask for node positions to be pushed away
    # from a reported contact.  These are soft initial-value deformations;
    # the endpoint NLP and all three audits remain authoritative.
    if obstacle_context is not None:
        for target in obstacle_context.node_targets:
            index = int(target["index"])
            if index < 0 or index > n:
                continue
            weight = 1.0
            points[index] = np.array(
                [float(target["x"]), float(target["y"])], dtype=float) * weight \
                + points[index] * (1.0 - weight)
    S_guess = min(packet["S_upper"], max(packet["S_lower"], route_length))
    node_s = np.linspace(0.0, S_guess, n + 1)
    kappas = _node_route_curvature(points)
    # Stretching/compressing the guide to the NLP's S parameter preserves the
    # approximate heading change.
    kappas *= route_length / S_guess
    kappas = np.clip(kappas, -packet["kappa_limit"], packet["kappa_limit"])
    try:
        kappas, etas = _project_etas(
            kappas, S_guess / n, packet["eta_limit"])
    except VirtualAContractError:
        raise
    corridor_mode = _guidance_mode(config) == "corridor"
    if corridor_mode:
        node_count = n + 1
        collocation_count = n * int(config.collocation_points)
        route_position_weight = float(config.corridor_guide_weight) * \
            node_count / (node_count + collocation_count)
    else:
        route_position_weight = ROUTE_GUIDANCE_WEIGHT * (
            float(getattr(obstacle_context, "route_weight_scale", 1.0))
            if obstacle_context is not None else 1.0)
    metadata = {
        "endpoint_local_dx_m": packet["dx"],
        "endpoint_local_dy_m": packet["dy"],
        "endpoint_distance_m": packet["distance"],
        "intervals": n,
        "heading_change_rad": packet["heading_change_rad"],
        "length_multiplier": route_length / max(1.0, packet["distance"]),
        "shape_sign": 0,
        "S_search_lower_m": packet["S_lower"],
        "S_search_upper_m": packet["S_upper"],
        "route_index": int(route_index),
        "route_source": guide.source,
        "route_grid_sha256": guide.grid_sha256,
        "route_length_m": route_length,
        "route_point_count": len(guide.points),
        "route_heading_smoothing": "node_circumcircle_three_point",
        "route_position_weight": float(route_position_weight),
        "route_position_rule": (
            "corridor_guide_mean_normalised_by_L0_squared"
            if corridor_mode else "legacy_route_weight"),
        "route_weight_scale": float(getattr(
            obstacle_context, "route_weight_scale", 1.0))
        if obstacle_context is not None else 1.0,
        "projection": "clamped_cubic_spline_C2_eta0_etaN_zero",
        "obstacle_context": obstacle_context.as_dict()
        if obstacle_context is not None else None,
    }
    return InitialGuess(
        source="map_route_guide",
        label="route_guide={};source={}".format(
            int(route_index), guide.source),
        S=float(S_guess),
        kappas=tuple(float(v) for v in kappas),
        etas=tuple(float(v) for v in etas),
        length_multiplier=float(metadata["length_multiplier"]),
        shape_sign=0,
        node_s=tuple(float(v) for v in node_s),
        metadata=metadata)


def _guess_route_guidance(guess, request, route_guides, n, collocation=None,
                          obstacle_context=None):
    """Start-local guide positions, approximate curvature and collocation points.

    When routes are supplied, every initial value is compared under the same
    soft guide objective.  The node terms anchor the nine design knots; the
    collocation terms keep the C2 curve between knots from sagging through a
    narrow throat.  Both are soft guidance only and never enter an audit.
    """
    if not route_guides:
        return None
    index = int(guess.metadata.get("route_index", 0)) if getattr(
        guess, "source", "") == "map_route_guide" else 0
    if index < 0 or index >= len(route_guides):
        return None
    guide = route_guides[index]
    fractions = np.linspace(0.0, 1.0, int(n) + 1)
    local = _route_fraction_points(guide, request, fractions)
    curvature = _node_route_curvature(local)
    if collocation is None:
        collocation_points = np.zeros((0, 2), dtype=float)
    else:
        nodes, _ = _gauss(int(collocation))
        collocation_fractions = np.concatenate([
            (float(j) + nodes) / float(n) for j in range(int(n))])
        collocation_points = _route_fraction_points(
            guide, request, collocation_fractions)
    return local, curvature, collocation_points


def _guidance_mode(config):
    return str(config.guidance_mode).strip().lower()


def _corridor_margin_rule(config):
    return str(config.corridor_enforcement).strip().lower()


@dataclass(frozen=True)
class _CorridorConstraint:
    """Precomputed start-local corridor samples, one entry per piece.

    Each group holds ``(q, rows, limits)`` where ``q`` is the fractional
    position inside Hermite piece ``j``.  ``rows`` are the rotated unit outward
    normals and ``limits`` already include the effective margin.  The mapping
    from allocation sigma to ``(j, q)`` is independent of the decision S.
    """

    margin_m: float
    portal_margin_fraction: float
    allocation: tuple
    groups: tuple
    sample_count: int
    region_count: int
    min_portal_radius_m: object
    quadrature_points: int
    enforcement: str

    def as_dict(self):
        return {
            "role": "reference_point_convex_halfspaces",
            "enforcement": self.enforcement,
            "margin_m": float(self.margin_m),
            "portal_margin_fraction": float(self.portal_margin_fraction),
            "min_portal_radius_m": self.min_portal_radius_m,
            "quadrature_points": int(self.quadrature_points),
            "sample_count": int(self.sample_count),
            "allocation_count": len(self.allocation),
            "region_count": int(self.region_count),
            "is_three_vehicle_certificate": False,
        }


def _build_corridor_constraint(chain, allocation, request, config):
    """Rotate both corridor halfspace sets into the request start frame."""
    if not isinstance(chain, CorridorChain):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_CHAIN", "expected CorridorChain")
    allocation = validate_corridor_allocation(chain, allocation)
    n = int(config.intervals)
    quadrature = int(config.corridor_quadrature_points)
    nodes, _weights = _gauss(quadrature)
    start = request.start_qf
    cosine, sine = math.cos(start.psi), math.sin(start.psi)
    margin = corridor_effective_margin(chain, config)
    groups = [[] for _ in range(n)]
    sample_count = 0
    for left, right, corridor_index in allocation:
        sigmas = [left]
        sigmas.extend(left + (right - left) * float(node) for node in nodes)
        sigmas.append(right)
        region = chain.regions[corridor_index]
        local_rows = tuple(
            (float(row[0] * cosine + row[1] * sine),
             float(-row[0] * sine + row[1] * cosine))
            for row in region.A)
        local_limits = tuple(
            float(limit) - margin
            - (float(row[0]) * start.x + float(row[1]) * start.y)
            for row, limit in zip(region.A, region.b))
        for sigma in sigmas:
            if sigma <= 0.0:
                piece, q = 0, 0.0
            elif sigma >= 1.0:
                piece, q = n - 1, 1.0
            else:
                scaled = float(sigma) * n
                piece = int(math.floor(scaled))
                q = scaled - piece
                if piece >= n:
                    piece, q = n - 1, 1.0
                if q < 0.0:
                    piece, q = max(0, piece - 1), 1.0
                elif q > 1.0:
                    piece, q = min(n - 1, piece + 1), 0.0
            groups[piece].append((float(q), local_rows, local_limits))
            sample_count += 1
    return _CorridorConstraint(
        margin_m=float(margin),
        portal_margin_fraction=float(
            config.corridor_margin_portal_fraction),
        allocation=allocation,
        groups=tuple(tuple(group) for group in groups),
        sample_count=int(sample_count),
        region_count=len(chain.regions),
        min_portal_radius_m=corridor_min_portal_radius(chain),
        quadrature_points=quadrature,
        enforcement=_corridor_margin_rule(config))


@dataclass(frozen=True)
class _ClearanceProxy:
    """Precomputed smooth-penalty data; never a collision certificate."""

    field: object
    angles: tuple
    target_m: float
    weight: float

    @property
    def sample_count(self):
        return len(self.angles)

    @property
    def vehicle_count(self):
        return len(self.angles[0]) if self.angles else 0

    def as_dict(self):
        return {
            "role": "smooth_nlp_body_footprint_guidance_only",
            "is_collision_certificate": False,
            "sample_count": int(self.sample_count),
            "vehicle_count": int(self.vehicle_count),
            "target_m": float(self.target_m),
            "weight": float(self.weight),
            "orientation_source": "Gauss_collocation_local_psi_and_actual_theta_i",
            "clearance_field": self.field.as_dict(),
        }


def _collocation_states(vector, packet, collocation):
    """Reconstruct collocation positions/headings/curvature from a guess.

    Positions use the per-piece prefix integral at each Gauss node, matching
    :func:`_numeric_objective` and :func:`_build_casadi_nlp`.  The local
    heading at a collocation point is used for each vehicle's actual theta_i.
    """
    n = int(packet["intervals"])
    z = np.asarray(vector, dtype=float).reshape(-1)
    S = float(z[0])
    kappas = np.asarray(z[1:1 + n + 1], dtype=float)
    etas = np.asarray(z[1 + n + 1:], dtype=float)
    if n <= 0 or kappas.size != n + 1 or etas.size != n + 1 \
            or not math.isfinite(S) or S <= 0.0:
        return ()
    h = S / n
    nodes, weights = _gauss(int(collocation))
    x = 0.0
    y = 0.0
    psi = 0.0
    states = []
    for j in range(n):
        c0, c1, c2, c3 = _coeffs(h, kappas[j], kappas[j + 1],
                                 etas[j], etas[j + 1])
        displacements, headings = _piece_position_samples(
            h, (c0, c1, c2, c3), psi, nodes, math.sin, math.cos)
        for q, (dx, dy), local_psi in zip(nodes, displacements, headings):
            kappa = c0 + q * (c1 + q * (c2 + q * c3))
            states.append((x + dx, y + dy, local_psi, kappa))
        for node, weight in zip(nodes, weights):
            angle = psi + h * node * (
                c0 + node * (c1 / 2.0 + node * (c2 / 3.0 + node * c3 / 4.0)))
            x += h * weight * math.cos(angle)
            y += h * weight * math.sin(angle)
        psi += h * (c0 + c1 / 2.0 + c2 / 3.0 + c3 / 4.0)
    return tuple(states)


def _clearance_world_body(request, x, y, psi, kappa, offset):
    """Map one local formation state plus vehicle offset to world body pose.

    ``x, y, psi`` are the optimizer's start-local reference coordinates;
    the clearance field and its nearest-obstacle directions are world-frame.
    """
    start = request.start_qf
    start_x = float(start.x)
    start_y = float(start.y)
    start_psi = float(start.psi)
    cosine = math.cos(start_psi)
    sine = math.sin(start_psi)
    world_x = start_x + cosine * x - sine * y
    world_y = start_y + sine * x + cosine * y
    world_psi = start_psi + psi
    a, b = float(offset.x), float(offset.y)
    center_x = world_x + math.cos(world_psi) * a - math.sin(world_psi) * b
    center_y = world_y + math.sin(world_psi) * a + math.cos(world_psi) * b
    theta = world_psi + math.atan2(a * kappa, 1.0 - b * kappa)
    return center_x, center_y, theta


def _clearance_proxy_angles(field, request, states):
    """Freeze one nearest-obstacle direction per collocation body centre."""
    angles = []
    for x, y, psi, kappa in states:
        row = []
        for offset in request.model.offsets:
            center_x, center_y, theta = _clearance_world_body(
                request, x, y, psi, kappa, offset)
            _, direction, _ = field.direction_at(center_x, center_y)
            if direction is None or not math.isfinite(direction):
                direction = theta
            row.append(float(direction))
        angles.append(tuple(row))
    return tuple(angles)


def _clearance_proxy(guess, request, packet, config, obstacle_context):
    """Build the mandatory cached-field body-footprint proxy for one initial."""
    if obstacle_context is None:
        return None
    field = getattr(obstacle_context, "clearance_field", None)
    if field is None:
        return None
    states = _collocation_states(
        guess.vector, packet, int(config.collocation_points))
    if not states:
        raise VirtualAContractError(
            "CLEARANCE_PROXY_FAILURE",
            "no finite collocation states for the clearance proxy")
    for entry in states:
        if not all(math.isfinite(float(value)) for value in entry):
            raise VirtualAContractError(
                "CLEARANCE_PROXY_FAILURE",
                "non-finite collocation state in the clearance proxy")
    angles = _clearance_proxy_angles(field, request, states)
    if len(angles) != len(states):
        raise VirtualAContractError(
            "CLEARANCE_PROXY_FAILURE",
            "clearance proxy sample count mismatch")
    corridor_mode = _guidance_mode(config) == "corridor"
    return _ClearanceProxy(
        field=field,
        angles=angles,
        target_m=float(
            config.corridor_clearance_target_m
            if corridor_mode else CLEARANCE_PROXY_TARGET_M),
        weight=float(
            config.corridor_clearance_weight
            if corridor_mode else CLEARANCE_PROXY_WEIGHT))


def _proxy_numeric_penalty(proxy, request, x, y, psi, kappa, sample):
    total = 0.0
    row = proxy.angles[sample]
    for index, offset in enumerate(request.model.offsets):
        shape = request.vehicle_shapes[offset.vehicle_id]
        center_x, center_y, theta = _clearance_world_body(
            request, x, y, psi, kappa, offset)
        direction = row[index]
        clearance = proxy.field.value_at(center_x, center_y)
        support = (0.5 * shape.length_m * abs(math.cos(theta - direction))
                   + 0.5 * shape.width_m * abs(math.sin(theta - direction)))
        hinge = proxy.target_m + support - clearance
        if hinge > 0.0:
            total += hinge * hinge
    return total


def _validate_route_inputs(route_guides, obstacle_context):
    if route_guides is None:
        route_guides = ()
    route_guides = tuple(route_guides)
    if len(route_guides) > 3:
        raise VirtualAContractError(
            "INVALID_GUIDE_ROUTE",
            "at most three finite route guides are allowed")
    for guide in route_guides:
        if not isinstance(guide, GuideRoute):
            raise VirtualAContractError(
                "INVALID_GUIDE_ROUTE", "expected GuideRoute instances")
    if obstacle_context is not None and not isinstance(
            obstacle_context, ObstacleContext):
        raise VirtualAContractError(
            "INVALID_OBSTACLE_CONTEXT", "expected ObstacleContext")
    return route_guides, obstacle_context


def make_initial_guesses(request, config, *, route_guides=(),
                         obstacle_context=None):
    """Return at most ``max_initials`` automatic, reconstructible initials.

    With no route topology the function is byte-for-byte the G2 automatic
    sequence.  With routes, C3 schedules at most one guide initial per
    distinct route, always retains at least one true endpoint-only initial
    when a slot is available, and fills the remaining slots from at most
    ``C3_ENDPOINT_SHAPES`` distinct default length shapes with one initial
    per (length, direction) pair.  A route initial is never replaced by an
    endpoint shape merely to reach ``max_initials``.
    """
    request_ok = validate_virtual_a_request(request)
    if not request_ok.passed:
        raise VirtualAContractError(
            request_ok.reason, request_ok.detail, **request_ok.location)
    config_ok = validate_planner_config(config)
    if not config_ok.passed:
        raise VirtualAContractError(
            config_ok.reason, config_ok.detail, **config_ok.location)
    route_guides, obstacle_context = _validate_route_inputs(
        route_guides, obstacle_context)
    packet = _search_packet(request, config)
    if packet["S_lower"] >= packet["S_upper"]:
        raise VirtualAContractError(
            "EMPTY_SEARCH_DOMAIN",
            "S_lower={!r} >= S_upper={!r}".format(
                packet["S_lower"], packet["S_upper"]),
            S_lower=packet["S_lower"], S_upper=packet["S_upper"])
    max_initials = min(MAX_INITIALS, int(config.max_initials))
    guesses = []
    seen_routes = set()
    for index, guide in enumerate(route_guides):
        # At most one initial per distinct valid route; a caller that repeats
        # the same guide must not consume another slot.
        points = tuple(guide.points)
        if points in seen_routes:
            continue
        seen_routes.add(points)
        guesses.append(_route_guess(
            request, config, packet, guide, index,
            obstacle_context=obstacle_context))
        if len(guesses) >= max_initials:
            return tuple(guesses)
    if not route_guides:
        # No route candidate at all: keep the historical G2 automatic
        # sequence (three length shapes, both directions).
        for multiplier in DEFAULT_INITIAL_LENGTH_MULTIPLIERS:
            for sign in (1, -1):
                guesses.append(_guess_for(
                    request, config, packet, multiplier, sign))
                if len(guesses) >= max_initials:
                    return tuple(guesses)
        return tuple(guesses)
    if max_initials <= len(guesses):
        # The route initial is the only slot; no endpoint-only initial fits.
        return tuple(guesses)
    seen_endpoint = set()
    for multiplier in DEFAULT_INITIAL_LENGTH_MULTIPLIERS[:C3_ENDPOINT_SHAPES]:
        for sign in (1, -1):
            key = (float(multiplier), int(sign))
            if key in seen_endpoint:
                continue
            seen_endpoint.add(key)
            guesses.append(_guess_for(
                request, config, packet, multiplier, sign))
            if len(guesses) >= max_initials:
                return tuple(guesses)
    return tuple(guesses)


def initial_guess_key(guess):
    """Return the stable, reconstructible identity of one initial guess.

    C3 rework 01 requires a request-wide initial pool.  The identity uses only
    fields already serialized by ``InitialGuess``/``_route_guess``/
    ``_guess_for``: the source, the route source/grid digest/index and the
    label for route initials, and the source, length multiplier, shape sign
    and label for endpoint initials.  Object addresses, solver vectors and
    float-array hashes are deliberately not used, so the same identity can be
    rebuilt from recorded evidence.
    """
    source = str(getattr(guess, "source", ""))
    label = str(getattr(guess, "label", ""))
    metadata = getattr(guess, "metadata", None)
    if not isinstance(metadata, dict):
        metadata = {}
    if source == "map_route_guide":
        return (
            source,
            str(metadata.get("route_source", "")),
            str(metadata.get("route_grid_sha256", "")),
            int(metadata.get("route_index", -1)),
            label,
        )
    if source == "automatic_endpoint_shape":
        return (
            source,
            float(getattr(guess, "length_multiplier", 0.0)),
            int(getattr(guess, "shape_sign", 0)),
            label,
        )
    return (source, label)


def _numeric_constraint_vector(z, request, config, packet, collocation,
                               corridor_constraint=None):
    """Numeric constraints shared by the SciPy fallback and residual checks."""
    n = packet["intervals"]
    S = float(z[0])
    kappas = np.asarray(z[1:1 + n + 1], dtype=float)
    etas = np.asarray(z[1 + n + 1:], dtype=float)
    if S <= 0.0:
        return np.full(1, 1e12)
    h = S / n
    values = []
    corridor_values = []
    for j in range(1, n):
        c0l, c1l, c2l, c3l = _coeffs(h, kappas[j - 1], kappas[j],
                                     etas[j - 1], etas[j])
        c0r, c1r, c2r, c3r = _coeffs(h, kappas[j], kappas[j + 1],
                                     etas[j], etas[j + 1])
        u_left = (2.0 * c2l + 6.0 * c3l) / (h * h)
        u_right = 2.0 * c2r / (h * h)
        values.append(u_left - u_right)
    nodes, weights = _gauss(collocation)
    x = 0.0
    y = 0.0
    psi = 0.0
    for j in range(n):
        c0, c1, c2, c3 = _coeffs(h, kappas[j], kappas[j + 1],
                                 etas[j], etas[j + 1])
        if corridor_constraint is not None:
            for q, rows, limits in corridor_constraint.groups[j]:
                shifts, _headings = _piece_position_samples(
                    h, (c0, c1, c2, c3), psi, [q], math.sin, math.cos,
                    quadrature_count=corridor_constraint.quadrature_points)
                dx, dy = shifts[0]
                sample_x = x + dx
                sample_y = y + dy
                for row, limit in zip(rows, limits):
                    corridor_values.append(
                        row[0] * sample_x + row[1] * sample_y - limit)
        for q, weight in zip(nodes, weights):
            local_psi = psi + h * q * (
                c0 + q * (c1 / 2.0 + q * (c2 / 3.0 + q * c3 / 4.0)))
            x += h * weight * math.cos(local_psi)
            y += h * weight * math.sin(local_psi)
        psi += h * (c0 + c1 / 2.0 + c2 / 3.0 + c3 / 4.0)
    values.extend((x - packet["dx"], y - packet["dy"],
                   psi - packet["heading_change_rad"]))
    policy = coerce_goal_policy(request.goal_policy)
    if policy.value == "STRAIGHT":
        values.extend((kappas[-1], etas[-1]))
    cmax = packet["cmax"] * (1.0 - float(config.curvature_margin))
    for j in range(n):
        c0, c1, c2, c3 = _coeffs(h, kappas[j], kappas[j + 1],
                                 etas[j], etas[j + 1])
        for q in np.concatenate(([0.0, 1.0], nodes)):
            kappa = c0 + q * (c1 + q * (c2 + q * c3))
            eta = (c1 + q * (2.0 * c2 + 3.0 * q * c3)) / h
            for offset in request.model.offsets:
                a, b = offset.x, offset.y
                A = 1.0 - b * kappa
                B = a * kappa
                D = A * A + B * B
                if A <= 0.0 or D <= 0.0:
                    values.append(1e12)
                else:
                    values.append(
                        (kappa * D + a * eta) / math.pow(D, 1.5))
    values.extend(corridor_values)
    return np.asarray(values, dtype=float)


def _constraint_value_offsets(request, config, packet, corridor_constraint):
    """Return (equality_end, signed_end, corridor_end) slices."""
    n = int(packet["intervals"])
    equality_end = (n - 1) + 3 + (
        2 if coerce_goal_policy(request.goal_policy).value == "STRAIGHT"
        else 0)
    signed_end = equality_end + n * (
        int(config.collocation_points) + 2) * len(request.model.offsets)
    corridor_end = signed_end
    if corridor_constraint is not None:
        corridor_end += sum(
            len(limits) for group in corridor_constraint.groups
            for _q, _rows, limits in group)
    return equality_end, signed_end, corridor_end


def _numeric_objective(z, request, config, packet, collocation,
                       route_points=None, route_weight=0.0,
                       route_kappas=None, route_curvature_weight=0.0,
                       route_collocation_points=None,
                       route_collocation_weight=0.0,
                       clearance_proxy=None):
    n = packet["intervals"]
    S = float(z[0])
    kappas = np.asarray(z[1:1 + n + 1], dtype=float)
    etas = np.asarray(z[1 + n + 1:], dtype=float)
    h = S / n
    nodes, weights = _gauss(collocation)
    eta_integral = 0.0
    u_integral = 0.0
    kappa_integral = 0.0
    x = 0.0
    y = 0.0
    psi = 0.0
    node_positions = [(0.0, 0.0)]
    collocation_penalty = 0.0
    clearance_penalty = 0.0
    sample = 0
    for j in range(n):
        c0, c1, c2, c3 = _coeffs(h, kappas[j], kappas[j + 1],
                                 etas[j], etas[j + 1])
        displacements, headings = _piece_position_samples(
            h, (c0, c1, c2, c3), psi, nodes, math.sin, math.cos)
        for q, weight, (dx, dy), local_psi in zip(
                nodes, weights, displacements, headings):
            kappa = c0 + q * (c1 + q * (c2 + q * c3))
            eta = (c1 + q * (2.0 * c2 + 3.0 * q * c3)) / h
            u = (2.0 * c2 + 6.0 * c3 * q) / (h * h)
            kappa_integral += h * weight * kappa * kappa
            eta_integral += h * weight * eta * eta
            u_integral += h * weight * u * u
            sample_x = x + dx
            sample_y = y + dy
            if route_collocation_points is not None \
                    and route_collocation_weight > 0.0:
                point = route_collocation_points[sample]
                delta_x = sample_x - float(point[0])
                delta_y = sample_y - float(point[1])
                collocation_penalty += delta_x * delta_x + delta_y * delta_y
            if clearance_proxy is not None:
                clearance_penalty += _proxy_numeric_penalty(
                    clearance_proxy, request, sample_x, sample_y, local_psi,
                    kappa, sample)
            sample += 1
        for node, weight in zip(nodes, weights):
            angle = psi + h * node * (
                c0 + node * (c1 / 2.0 + node * (c2 / 3.0 + node * c3 / 4.0)))
            x += h * weight * math.cos(angle)
            y += h * weight * math.sin(angle)
        psi += h * (c0 + c1 / 2.0 + c2 / 3.0 + c3 / 4.0)
        node_positions.append((x, y))
    l0 = packet["l0"]
    corridor_mode = _guidance_mode(config) == "corridor"
    objective = (S / l0 + 0.02 * l0 ** 3 * eta_integral
                 + 0.002 * l0 ** 5 * u_integral)
    if corridor_mode:
        objective += float(config.corridor_kappa_weight) * l0 * kappa_integral
    guide_normalizer = 1.0 if not corridor_mode else l0 * l0
    if route_points is not None and route_weight > 0.0:
        penalty = 0.0
        for j, point in enumerate(route_points):
            dx = node_positions[j][0] - float(point[0])
            dy = node_positions[j][1] - float(point[1])
            penalty += dx * dx + dy * dy
        objective += float(route_weight) * penalty / (
            len(route_points) * guide_normalizer)
    if route_collocation_points is not None \
            and route_collocation_weight > 0.0:
        objective += float(route_collocation_weight) * collocation_penalty / (
            len(route_collocation_points) * guide_normalizer)
    if route_kappas is not None and route_curvature_weight > 0.0:
        curvature_penalty = 0.0
        for j in range(n + 1):
            delta_k = kappas[j] - float(route_kappas[j])
            curvature_penalty += delta_k * delta_k
        objective += float(route_curvature_weight) * curvature_penalty / (
            n + 1)
    if clearance_proxy is not None and clearance_proxy.weight > 0.0:
        normalizer = max(
            1, clearance_proxy.sample_count * clearance_proxy.vehicle_count)
        objective += float(clearance_proxy.weight) * clearance_penalty / (
            normalizer * guide_normalizer)
    return objective


def _constraint_residual(z, request, config, packet,
                         corridor_constraint=None):
    values = _numeric_constraint_vector(
        z, request, config, packet, int(config.collocation_points),
        corridor_constraint=corridor_constraint)
    # Curvature values are appended without a lower bound; remove them by
    # using the same count as the NLP.  Retain equality and signed-C checks by
    # checking the numeric profile against bounds explicitly.
    n = packet["intervals"]
    S = float(z[0])
    kappas = np.asarray(z[1:1 + n + 1], dtype=float)
    etas = np.asarray(z[1 + n + 1:], dtype=float)
    h = S / n
    nodes, _ = _gauss(int(config.collocation_points))
    cmax = packet["cmax"] * (1.0 - config.curvature_margin)
    residual = 0.0
    equal_count = (n - 1) + 3 + (2 if coerce_goal_policy(
        request.goal_policy).value == "STRAIGHT" else 0)
    residual = max(residual, float(np.max(np.abs(values[:equal_count]))))
    for j in range(n):
        c0, c1, c2, c3 = _coeffs(h, kappas[j], kappas[j + 1],
                                 etas[j], etas[j + 1])
        for q in np.concatenate(([0.0, 1.0], nodes)):
            kappa = c0 + q * (c1 + q * (c2 + q * c3))
            eta = (c1 + q * (2.0 * c2 + 3.0 * q * c3)) / h
            for offset in request.model.offsets:
                a, b = offset.x, offset.y
                A = 1.0 - b * kappa
                B = a * kappa
                D = A * A + B * B
                if A <= 0.0 or D <= 0.0:
                    residual = max(residual, 1e12)
                else:
                    c = (kappa * D + a * eta) / math.pow(D, 1.5)
                    residual = max(residual, max(0.0, abs(c) - cmax))
    _equality_end, signed_end, corridor_end = _constraint_value_offsets(
        request, config, packet, corridor_constraint)
    if corridor_end > signed_end:
        corridor_values = np.asarray(values[signed_end:corridor_end],
                                     dtype=float)
        if corridor_values.size:
            residual = max(
                residual, float(np.max(np.maximum(0.0, corridor_values))))
    return residual


def _constraint_residual_diagnostic(z, request, config, packet,
                                    corridor_constraint=None):
    """Return ``(residual, error)`` without letting diagnostics escape.

    A residual is recorded only for an available, finite decision vector of
    the expected size.  Any malformed vector, calculation exception, or
    non-finite/negative result produces ``(None, reason)``.  The caller decides
    whether that is a solver failure diagnostic or a fail-closed candidate
    rejection; the planner never treats an unavailable residual as zero.
    """
    if z is None:
        return None, "no decision vector"
    try:
        vector = np.asarray(z, dtype=float).reshape(-1)
    except Exception as exc:
        return None, "decision vector conversion failed: {}: {}".format(
            type(exc).__name__, exc)
    expected_size = 2 * int(packet["intervals"]) + 3
    if vector.size != expected_size:
        return None, "decision vector has {} values; expected {}".format(
            vector.size, expected_size)
    if not np.all(np.isfinite(vector)):
        return None, "decision vector contains non-finite values"
    try:
        residual = float(_constraint_residual(
            vector, request, config, packet,
            corridor_constraint=corridor_constraint))
    except Exception as exc:
        return None, "constraint residual computation failed: {}: {}".format(
            type(exc).__name__, exc)
    if not math.isfinite(residual):
        return None, "constraint residual is non-finite: {!r}".format(residual)
    if residual < 0.0:
        return None, "constraint residual is negative: {!r}".format(residual)
    return residual, ""


def _corridor_residual_diagnostic(z, request, config, packet,
                                  corridor_constraint):
    """Maximum corridor halfspace excess of one decision vector."""
    if corridor_constraint is None:
        return None, ""
    if z is None:
        return None, "no decision vector"
    try:
        vector = np.asarray(z, dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        return None, "decision vector conversion failed: {}: {}".format(
            type(exc).__name__, exc)
    expected_size = 2 * int(packet["intervals"]) + 3
    if vector.size != expected_size:
        return None, "decision vector has {} values; expected {}".format(
            vector.size, expected_size)
    if not np.all(np.isfinite(vector)):
        return None, "decision vector contains non-finite values"
    try:
        values = _numeric_constraint_vector(
            vector, request, config, packet,
            int(config.collocation_points),
            corridor_constraint=corridor_constraint)
        _equality_end, signed_end, corridor_end = _constraint_value_offsets(
            request, config, packet, corridor_constraint)
        corridor_values = np.asarray(values[signed_end:corridor_end],
                                     dtype=float)
        residual = float(np.max(np.maximum(0.0, corridor_values))) \
            if corridor_values.size else 0.0
    except Exception as exc:
        return None, "corridor residual computation failed: {}: {}".format(
            type(exc).__name__, exc)
    if not math.isfinite(residual) or residual < 0.0:
        return None, "corridor residual is invalid: {!r}".format(residual)
    return residual, ""


def _attach_failure_residual(result, request, config, packet,
                             corridor_constraint=None):
    """Attach best-effort residuals to a failed attempt, never raising."""
    if result.get("z") is None:
        return
    residual, error = _constraint_residual_diagnostic(
        result["z"], request, config, packet,
        corridor_constraint=corridor_constraint)
    result["constraint_residual"] = residual
    if corridor_constraint is not None:
        corridor_residual, corridor_error = _corridor_residual_diagnostic(
            result["z"], request, config, packet, corridor_constraint)
        result["corridor_residual_m"] = corridor_residual
        if corridor_error and not error:
            error = corridor_error
    if error:
        prefix = str(result.get("detail", "")).rstrip("; ")
        result["detail"] = (
            "{}; constraint residual unavailable: {}".format(
                prefix, error) if prefix else
            "constraint residual unavailable: {}".format(error))


def _build_casadi_nlp(request, config, packet, wall_limit_s=None,
                      route_points=None, route_weight=0.0,
                      route_kappas=None, route_curvature_weight=0.0,
                      route_collocation_points=None,
                      route_collocation_weight=0.0,
                      clearance_proxy=None,
                      corridor_constraint=None):
    import casadi as ca
    n = packet["intervals"]
    clearance_fn = None
    clearance_bounds = None
    if clearance_proxy is not None and clearance_proxy.weight > 0.0:
        field = clearance_proxy.field
        if int(field.height) < 2 or int(field.width) < 2:
            raise VirtualAContractError(
                "CLEARANCE_PROXY_FAILURE",
                "cached clearance field is too small for a bspline proxy")
        x_axis = field.origin_x + np.arange(
            int(field.width), dtype=float) * float(field.resolution_m)
        y_axis = field.origin_y + np.arange(
            int(field.height), dtype=float) * float(field.resolution_m)
        values = np.asarray(field.values, dtype=float).T
        clearance_fn = ca.interpolant(
            "g4_cached_clearance", "linear", [x_axis, y_axis],
            values.ravel(order="F"))
        clearance_bounds = (
            float(x_axis[0]), float(x_axis[-1]),
            float(y_axis[0]), float(y_axis[-1]))
        start = request.start_qf
        start_x = float(start.x)
        start_y = float(start.y)
        start_psi = float(start.psi)
        cos_start = math.cos(start_psi)
        sin_start = math.sin(start_psi)
    z = ca.MX.sym("z", 2 * n + 3)
    S = z[0]
    kappas = z[1:1 + n + 1]
    etas = z[1 + n + 1:]
    h = S / n
    constraints = []
    lower = []
    upper = []

    def coeffs(k0, k1, e0, e1):
        return (k0, h * e0,
                3.0 * (k1 - k0) - h * (2.0 * e0 + e1),
                2.0 * (k0 - k1) + h * (e0 + e1))

    for j in range(1, n):
        c0l, c1l, c2l, c3l = coeffs(
            kappas[j - 1], kappas[j], etas[j - 1], etas[j])
        c0r, c1r, c2r, c3r = coeffs(
            kappas[j], kappas[j + 1], etas[j], etas[j + 1])
        u_left = (2.0 * c2l + 6.0 * c3l) / (h * h)
        u_right = 2.0 * c2r / (h * h)
        constraints.append(u_left - u_right)
        lower.append(0.0)
        upper.append(0.0)
    nodes, weights = _gauss(config.collocation_points)
    x = 0.0
    y = 0.0
    psi = 0.0
    eta_integral = 0.0
    u_integral = 0.0
    kappa_integral = 0.0
    piece_coefficients = []
    node_positions = [(0.0, 0.0)]
    collocation_route_penalty = 0.0
    clearance_proxy_penalty = 0.0
    collocation_sample = 0
    corridor_constraints = []
    for j in range(n):
        c0, c1, c2, c3 = coeffs(
            kappas[j], kappas[j + 1], etas[j], etas[j + 1])
        piece_coefficients.append((c0, c1, c2, c3))
        displacements, headings = _piece_position_samples(
            h, (c0, c1, c2, c3), psi, nodes, ca.sin, ca.cos)
        if corridor_constraint is not None:
            for q, rows, limits in corridor_constraint.groups[j]:
                shifts, _headings = _piece_position_samples(
                    h, (c0, c1, c2, c3), psi, [q], ca.sin, ca.cos,
                    quadrature_count=corridor_constraint.quadrature_points)
                dx, dy = shifts[0]
                sample_x = x + dx
                sample_y = y + dy
                for row, limit in zip(rows, limits):
                    corridor_constraints.append(
                        row[0] * sample_x + row[1] * sample_y - limit)
        for q, weight, (dx, dy), local_psi in zip(
                nodes, weights, displacements, headings):
            kappa = c0 + q * (c1 + q * (c2 + q * c3))
            eta = (c1 + q * (2.0 * c2 + 3.0 * q * c3)) / h
            u = (2.0 * c2 + 6.0 * c3 * q) / (h * h)
            kappa_integral += h * weight * kappa * kappa
            eta_integral += h * weight * eta * eta
            u_integral += h * weight * u * u
            sample_x = x + dx
            sample_y = y + dy
            if route_collocation_points is not None \
                    and route_collocation_weight > 0.0:
                point = route_collocation_points[collocation_sample]
                collocation_route_penalty += (
                    (sample_x - float(point[0])) ** 2
                    + (sample_y - float(point[1])) ** 2)
            if clearance_fn is not None:
                angles = clearance_proxy.angles[collocation_sample]
                world_x = start_x + cos_start * sample_x \
                    - sin_start * sample_y
                world_y = start_y + sin_start * sample_x \
                    + cos_start * sample_y
                world_psi = start_psi + local_psi
                for index, offset in enumerate(request.model.offsets):
                    a, b = offset.x, offset.y
                    shape = request.vehicle_shapes[offset.vehicle_id]
                    center_x = world_x + ca.cos(world_psi) * a \
                        - ca.sin(world_psi) * b
                    center_y = world_y + ca.sin(world_psi) * a \
                        + ca.cos(world_psi) * b
                    center_x = ca.fmin(ca.fmax(
                        center_x, clearance_bounds[0]), clearance_bounds[1])
                    center_y = ca.fmin(ca.fmax(
                        center_y, clearance_bounds[2]), clearance_bounds[3])
                    theta = world_psi + ca.atan2(
                        a * kappa, 1.0 - b * kappa)
                    clearance = clearance_fn(
                        ca.vertcat(center_x, center_y))
                    direction = float(angles[index])
                    support = 0.5 * shape.length_m * ca.fabs(
                        ca.cos(theta - direction)) + 0.5 * shape.width_m * \
                        ca.fabs(ca.sin(theta - direction))
                    hinge = ca.fmax(
                        0.0, clearance_proxy.target_m + support - clearance)
                    clearance_proxy_penalty += hinge * hinge
            collocation_sample += 1
        for node, weight in zip(nodes, weights):
            angle = psi + h * node * (
                c0 + node * (c1 / 2.0 + node * (c2 / 3.0 + node * c3 / 4.0)))
            x += h * weight * ca.cos(angle)
            y += h * weight * ca.sin(angle)
        psi += h * (c0 + c1 / 2.0 + c2 / 3.0 + c3 / 4.0)
        node_positions.append((x, y))
    constraints.extend((x - packet["dx"], y - packet["dy"],
                        psi - packet["heading_change_rad"]))
    lower.extend((0.0, 0.0, 0.0))
    upper.extend((0.0, 0.0, 0.0))
    policy = coerce_goal_policy(request.goal_policy)
    if policy.value == "STRAIGHT":
        constraints.extend((kappas[-1], etas[-1]))
        lower.extend((0.0, 0.0))
        upper.extend((0.0, 0.0))
    cmax = packet["cmax"] * (1.0 - float(config.curvature_margin))
    for j in range(n):
        c0, c1, c2, c3 = piece_coefficients[j]
        for q in list(nodes) + [0.0, 1.0]:
            kappa = c0 + q * (c1 + q * (c2 + q * c3))
            eta = (c1 + q * (2.0 * c2 + 3.0 * q * c3)) / h
            for offset in request.model.offsets:
                a, b = offset.x, offset.y
                A = 1.0 - b * kappa
                B = a * kappa
                D = A * A + B * B
                c = (kappa * D + a * eta) / (D ** 1.5)
                constraints.append(c)
                lower.append(-cmax)
                upper.append(cmax)
    for value in corridor_constraints:
        constraints.append(value)
        lower.append(-np.inf)
        upper.append(0.0)
    l0 = packet["l0"]
    corridor_mode = _guidance_mode(config) == "corridor"
    objective = (S / l0 + 0.02 * l0 ** 3 * eta_integral
                 + 0.002 * l0 ** 5 * u_integral)
    if corridor_mode:
        objective = objective + float(
            config.corridor_kappa_weight) * l0 * kappa_integral
    guide_normalizer = 1.0 if not corridor_mode else l0 * l0
    if route_points is not None and route_weight > 0.0:
        route_penalty = 0.0
        for (node_x, node_y), point in zip(node_positions, route_points):
            dx = node_x - float(point[0])
            dy = node_y - float(point[1])
            route_penalty += dx * dx + dy * dy
        objective = objective + float(route_weight) * route_penalty / (
            len(route_points) * guide_normalizer)
    if route_collocation_points is not None \
            and route_collocation_weight > 0.0:
        objective = objective + float(route_collocation_weight) * (
            collocation_route_penalty / (
                len(route_collocation_points) * guide_normalizer))
    if route_kappas is not None and route_curvature_weight > 0.0:
        curvature_penalty = 0.0
        for j in range(n + 1):
            delta_k = kappas[j] - float(route_kappas[j])
            curvature_penalty += delta_k * delta_k
        objective = objective + float(route_curvature_weight) * (
            curvature_penalty / (n + 1))
    if clearance_proxy is not None and clearance_proxy.weight > 0.0:
        normalizer = max(
            1, clearance_proxy.sample_count * clearance_proxy.vehicle_count)
        objective = objective + float(clearance_proxy.weight) * (
            clearance_proxy_penalty / (normalizer * guide_normalizer))
    nlp = {"x": z, "f": objective, "g": ca.vertcat(*constraints)}
    options = {
        "ipopt": {
            "max_iter": int(config.max_iterations),
            "print_level": 0,
            "tol": float(config.ipopt_tolerance),
            "acceptable_tol": max(1e-7, 10.0 * config.ipopt_tolerance),
            "constr_viol_tol": max(1e-8, config.ipopt_tolerance),
            "dual_inf_tol": max(1e-8, 10.0 * config.ipopt_tolerance),
            "compl_inf_tol": max(1e-8, 10.0 * config.ipopt_tolerance),
            "linear_solver": "mumps",
            "sb": "yes",
        },
        "print_time": False,
    }
    if wall_limit_s is not None:
        options["ipopt"]["max_wall_time"] = max(
            1e-9, float(wall_limit_s))
    solver = ca.nlpsol("g2_ipopt", "ipopt", nlp, options)
    lower_x = [packet["S_lower"], 0.0] + [-packet["kappa_limit"]] * n + [0.0] + [
        -packet["eta_limit"]] * n
    upper_x = [packet["S_upper"], 0.0] + [packet["kappa_limit"]] * n + [0.0] + [
        packet["eta_limit"]] * n
    return solver, np.asarray(lower_x, dtype=float), np.asarray(
        upper_x, dtype=float), np.asarray(lower, dtype=float), np.asarray(
        upper, dtype=float)


def _solve_with_ipopt(request, config, packet, guess, wall_limit_s=None,
                      route_points=None, route_weight=0.0,
                      route_kappas=None, route_curvature_weight=0.0,
                      route_collocation_points=None,
                      route_collocation_weight=0.0,
                      clearance_proxy=None,
                      corridor_constraint=None):
    import casadi as ca
    started = time.monotonic()
    solver, lower_x, upper_x, lower_g, upper_g = _build_casadi_nlp(
        request, config, packet, wall_limit_s=wall_limit_s,
        route_points=route_points, route_weight=route_weight,
        route_kappas=route_kappas, route_curvature_weight=route_curvature_weight,
        route_collocation_points=route_collocation_points,
        route_collocation_weight=route_collocation_weight,
        clearance_proxy=clearance_proxy,
        corridor_constraint=corridor_constraint)
    values = solver(
        x0=guess.vector, lbx=lower_x, ubx=upper_x,
        lbg=lower_g, ubg=upper_g)
    stats = solver.stats()
    status = str(stats.get("return_status", "UNKNOWN"))
    timed_out = status == "Maximum_WallTime_Exceeded"
    return {
        "solver": SOLVER_IPOPT,
        "z": np.asarray(values["x"], dtype=float).ravel(),
        "objective": float(values["f"]),
        "solver_status": status,
        "success": status in (
            "Solve_Succeeded", "Solved_To_Acceptable_Level"),
        "iterations": _iteration_count(stats),
        "detail": status,
        "wall_time_s": time.monotonic() - started,
        "soft_deadline_triggered": timed_out,
        "deadline_trigger_stage": "ipopt_max_wall_time" if timed_out else "",
        "remaining_budget_s": wall_limit_s,
        "clearance_proxy": clearance_proxy.as_dict()
        if clearance_proxy is not None else None,
    }


def _iteration_count(stats):
    for key in ("iter_count", "iterations"):
        if key in stats:
            try:
                return int(float(stats[key]))
            except (TypeError, ValueError):
                pass
    return 0


def _solve_with_scipy(request, config, packet, guess, wall_limit_s=None,
                      route_points=None, route_weight=0.0,
                      route_kappas=None, route_curvature_weight=0.0,
                      route_collocation_points=None,
                      route_collocation_weight=0.0,
                      clearance_proxy=None,
                      corridor_constraint=None):
    from scipy.optimize import minimize
    started = time.monotonic()
    deadline = None if wall_limit_s is None else (
        started + max(0.0, float(wall_limit_s)))
    callback_state = {"iterations": 0, "last": None}

    def deadline_callback(vector):
        callback_state["iterations"] += 1
        callback_state["last"] = np.asarray(vector, dtype=float).copy()
        if deadline is not None and time.monotonic() >= deadline:
            raise _SoftDeadline()
    n = packet["intervals"]
    lower_x = np.asarray(
        [packet["S_lower"], 0.0] + [-config.kappa_limit] * n + [0.0]
        + [-config.eta_limit] * n, dtype=float)
    upper_x = np.asarray(
        [packet["S_upper"], 0.0] + [config.kappa_limit] * n + [0.0]
        + [config.eta_limit] * n, dtype=float)
    equality_count, signed_end, _corridor_end = _constraint_value_offsets(
        request, config, packet, corridor_constraint)

    def objective(vector):
        return _numeric_objective(
            vector, request, config, packet, int(config.collocation_points),
            route_points=route_points, route_weight=route_weight,
            route_kappas=route_kappas,
            route_curvature_weight=route_curvature_weight,
            route_collocation_points=route_collocation_points,
            route_collocation_weight=route_collocation_weight,
            clearance_proxy=clearance_proxy)

    def equality(vector):
        values = _numeric_constraint_vector(
            vector, request, config, packet,
            int(config.collocation_points),
            corridor_constraint=corridor_constraint)
        return values[:equality_count]

    def inequality(vector):
        values = _numeric_constraint_vector(
            vector, request, config, packet,
            int(config.collocation_points),
            corridor_constraint=corridor_constraint)
        cmax = packet["cmax"] * (1.0 - config.curvature_margin)
        signed = values[equality_count:signed_end]
        corridor_values = values[signed_end:]
        return np.concatenate((
            cmax - signed, cmax + signed, -corridor_values))

    try:
        result = minimize(
            objective, np.asarray(guess.vector, dtype=float),
            method="SLSQP",
            bounds=list(zip(lower_x, upper_x)),
            constraints=[
                {"type": "eq", "fun": equality},
                {"type": "ineq", "fun": inequality},
            ],
            options={
                "maxiter": int(config.scipy_maxiter),
                "ftol": float(config.scipy_tolerance),
                "disp": False,
            },
            callback=deadline_callback if deadline is not None else None)
    except _SoftDeadline:
        return {
            "solver": SOLVER_SCIPY,
            "z": callback_state["last"],
            "objective": None,
            "solver_status": "SLSQP_SOFT_DEADLINE",
            "success": False,
            "iterations": callback_state["iterations"],
            "detail": (
                "SLSQP iteration callback reached the soft wall budget; "
                "the returned iterate is not audited"),
            "wall_time_s": time.monotonic() - started,
            "soft_deadline_triggered": True,
            "deadline_trigger_stage": "slsqp_iteration_callback",
            "remaining_budget_s": wall_limit_s,
            "clearance_proxy": clearance_proxy.as_dict()
            if clearance_proxy is not None else None,
        }
    except Exception as exc:
        return {
            "solver": SOLVER_SCIPY,
            "z": None,
            "objective": None,
            "solver_status": "SCIPY_EXCEPTION",
            "success": False,
            "iterations": callback_state["iterations"],
            "detail": "{}: {}".format(type(exc).__name__, exc),
            "wall_time_s": time.monotonic() - started,
            "soft_deadline_triggered": False,
            "deadline_trigger_stage": "",
            "remaining_budget_s": wall_limit_s,
            "clearance_proxy": clearance_proxy.as_dict()
            if clearance_proxy is not None else None,
        }
    return {
        "solver": SOLVER_SCIPY,
        "z": np.asarray(result.x, dtype=float),
        "objective": float(result.fun),
        "solver_status": str(result.message),
        "success": bool(result.success),
        "iterations": int(getattr(result, "nit", 0)),
        "detail": str(result.message),
        "wall_time_s": time.monotonic() - started,
        "soft_deadline_triggered": False,
        "deadline_trigger_stage": "",
        "remaining_budget_s": wall_limit_s,
        "clearance_proxy": clearance_proxy.as_dict()
        if clearance_proxy is not None else None,
    }


def _candidate_path(request, guess, result):
    values = result["z"]
    if values is None or not np.all(np.isfinite(values)):
        raise VirtualAContractError(
            "NUMERICAL_FAILURE", "solver returned a nonfinite candidate")
    n = int(guess.metadata.get("intervals", len(guess.kappas) - 1))
    S = float(values[0])
    kappas = values[1:1 + n + 1]
    etas = values[1 + n + 1:1 + n + 1 + n + 1]
    if not (math.isfinite(S) and S > 0.0):
        raise VirtualAContractError(
            "NUMERICAL_FAILURE", "candidate S={!r}".format(S))
    knots = [
        ControlKnot(j * S / n, float(kappas[j]), float(etas[j]))
        for j in range(n + 1)]
    profile = C2CurvatureProfile.from_knots(knots)
    path = VirtualAPath(request.start_qf, profile, request.model)
    # Bind the physical request identity to the path.  The audit independently
    # detects a forged/mismatched digest; it is not used as a solver shortcut.
    path.request_sha256 = request_fingerprint(request)
    path.g2_initial_label = guess.label
    return path


def _solver_failure(solver, status, detail, started, remaining_budget):
    """A normal solver failure/exception with no candidate.

    Timeout results are constructed by the solver wrappers instead: they carry
    ``soft_deadline_triggered`` and must never be treated as ordinary
    failures eligible for a fallback.
    """
    return {
        "solver": solver,
        "z": None,
        "objective": None,
        "solver_status": status,
        "success": False,
        "iterations": 0,
        "detail": detail,
        "wall_time_s": time.monotonic() - started,
        "soft_deadline_triggered": False,
        "deadline_trigger_stage": "",
        "remaining_budget_s": remaining_budget,
    }


def _attempt_record(index, guess, result, audit, rejection, status,
                    endpoint_error=None, heading_error=None):
    return AttemptRecord(
        attempt_index=index,
        initial_source=guess.source,
        initial_label=guess.label,
        solver=result["solver"],
        solver_status=result["solver_status"],
        status=status,
        iterations=int(result["iterations"]),
        objective=result["objective"],
        constraint_residual=result.get("constraint_residual"),
        corridor_residual_m=result.get("corridor_residual_m"),
        endpoint_error_m=endpoint_error,
        heading_error_rad=heading_error,
        wall_time_s=float(result["wall_time_s"]),
        audit=audit,
        rejection_reason=rejection,
        initial=guess,
        candidate_path=result.get("candidate_path"),
        detail=result.get("detail", ""),
        soft_deadline_triggered=bool(
            result.get("soft_deadline_triggered", False)),
        deadline_trigger_stage=str(
            result.get("deadline_trigger_stage", "")),
        remaining_budget_s=result.get("remaining_budget_s"))


def _result(status, request=None, path=None, audit=None, attempts=(),
            wall_time_s=0.0, request_sha256="", reason="", detail="",
            config=None, solver_used="", branch=None, deadline=False,
            deadline_summary=None, initial_count=0):
    summary = request_summary(request) if isinstance(
        request, VirtualARequest) else {
            "type": type(request).__name__ if request is not None else None,
            "valid": False}
    return GeometryResult(
        status=status,
        path=path,
        request_summary=summary,
        audit=audit,
        attempts=tuple(attempts),
        wall_time_s=float(wall_time_s),
        request_sha256=request_sha256,
        reason=reason,
        detail=detail,
        config_summary=config.as_dict() if isinstance(config, PlannerConfig) else {},
        solver_used=solver_used,
        goal_heading_branch_rad=branch,
        deadline_exhausted=bool(deadline),
        deadline_summary=dict(deadline_summary or {}),
        initial_count=int(initial_count))


def _audit_config(config, geometry_audit_config=None):
    if geometry_audit_config is None:
        audit_config = GeometryAuditConfig(
            position_tolerance_m=config.position_tolerance_m,
            heading_tolerance_rad=config.heading_tolerance_rad,
            rigidity_tolerance_m=config.rigidity_tolerance_m)
    else:
        audit_config = geometry_audit_config
    validation = validate_geometry_audit_config(audit_config)
    if not validation.passed:
        raise VirtualAContractError(
            "INVALID_AUDIT_CONFIG", validation.detail,
            **validation.location)
    return audit_config


def solve_open_path_guided(request, config, *, route_guides=(),
                           obstacle_context=None,
                           geometry_audit_config=None,
                           corridor_chain=None,
                           corridor_allocation=None,
                           excluded_initial_keys=(),
                           max_new_initials=None,
                           stop_after_route_audit=False):
    """Solve and independently audit an endpoint-driven geometry candidate.

    ``route_guides``/``obstacle_context`` are optional map-guidance inputs for
    G4.  They only add initial values; all G2 endpoint, C2, forward and
    three-vehicle curvature constraints remain unchanged.  ``geometry_audit_
    config`` lets the G4 caller use the same audited numerical position budget
    as its retimer; ``None`` retains the G2 default exactly.

    ``excluded_initial_keys`` and ``max_new_initials`` are the C3 rework 01
    request-wide initial-pool scheduler.  ``make_initial_guesses`` remains the
    only initial constructor; this function only removes identities already
    launched by earlier rounds and caps how many new identities this round may
    start.  Both default to the historical unbounded behaviour.
    ``stop_after_route_audit`` is the matching request-wide budget policy: in
    a route-guided round, once the route's own initial has produced an
    independently audited geometry, stop before spending the endpoint-shape
    members of the same construction on a second geometry candidate.  They
    remain unused identities for the single endpoint-only recovery after the
    downstream audits decide whether one is needed.  If the route initial
    fails, the remaining constructed endpoint shapes are still attempted, so
    the historical five-initial numerical-failure round is unchanged.

    With ``config.guidance_mode == 'corridor'`` the caller must also pass one
    validated chain and its sigma allocation.  Reference-point halfspaces then
    enter both the numeric and CasADi NLP and the corridor objective replaces
    the legacy route weights.  In legacy mode these two arguments must be
    omitted, preserving the historical call path byte for byte.
    """
    started = time.monotonic()
    validation = validate_virtual_a_request(request)
    if not validation.passed:
        fingerprint = ""
        try:
            fingerprint = request_fingerprint(request)
        except Exception:
            pass
        return _result(
            "INVALID_REQUEST", request=request, reason=validation.reason,
            detail=validation.detail, wall_time_s=time.monotonic() - started,
            request_sha256=fingerprint, config=config,
            branch=validation.metrics.get("goal_heading_branch_rad"))
    config_validation = validate_planner_config(config)
    if not config_validation.passed:
        return _result(
            "INVALID_REQUEST", request=request,
            reason="INVALID_CONFIG", detail=config_validation.detail,
            wall_time_s=time.monotonic() - started,
            request_sha256=validation.metrics["request_sha256"], config=config,
            branch=validation.metrics.get("goal_heading_branch_rad"))
    try:
        route_guides, obstacle_context = _validate_route_inputs(
            route_guides, obstacle_context)
    except VirtualAContractError as exc:
        return _result(
            "INVALID_REQUEST", request=request, reason=exc.reason,
            detail=exc.detail, wall_time_s=time.monotonic() - started,
            request_sha256=validation.metrics["request_sha256"], config=config,
            branch=validation.metrics.get("goal_heading_branch_rad"))
    corridor_mode = _guidance_mode(config) == "corridor"
    if not corridor_mode and (corridor_chain is not None
                              or corridor_allocation is not None):
        return _result(
            "INVALID_REQUEST", request=request,
            reason="CORRIDOR_INPUTS_IN_LEGACY_MODE",
            detail=(
                "guidance_mode=legacy_route must not receive corridor inputs"),
            wall_time_s=time.monotonic() - started,
            request_sha256=validation.metrics["request_sha256"],
            config=config,
            branch=validation.metrics.get("goal_heading_branch_rad"))
    if corridor_mode and len(route_guides) > 1:
        return _result(
            "INVALID_REQUEST", request=request,
            reason="CORRIDOR_GUIDE_AMBIGUOUS",
            detail=(
                "guidance_mode=corridor accepts at most one corridor guide; "
                "got {}".format(len(route_guides))),
            wall_time_s=time.monotonic() - started,
            request_sha256=validation.metrics["request_sha256"],
            config=config,
            branch=validation.metrics.get("goal_heading_branch_rad"))
    fingerprint = validation.metrics["request_sha256"]
    branch = validation.metrics.get("goal_heading_branch_rad")
    if request.start_qf == request.goal_qf:
        hold = VirtualAPath.hold(request.start_qf, request.model)
        hold.request_sha256 = fingerprint
        audit = audit_geometry(
            request, hold, _audit_config(config, geometry_audit_config))
        attempts = (AttemptRecord(
            attempt_index=0, initial_source="NO_MOTION", initial_label="hold",
            solver=SOLVER_SKIP, solver_status="NOT_RUN", status="HOLD",
            iterations=0, objective=0.0, constraint_residual=0.0,
            endpoint_error_m=0.0, heading_error_rad=0.0,
            wall_time_s=time.monotonic() - started, audit=audit,
            rejection_reason="" if audit.passed else audit.reason,
            detail="exact identical start/goal"),)
        status = "NO_MOTION" if audit.passed else "NO_ACCEPTED_CANDIDATE"
        return _result(
            status, request=request, path=hold if audit.passed else None,
            audit=audit, attempts=attempts,
            wall_time_s=time.monotonic() - started,
            request_sha256=fingerprint,
            reason="" if audit.passed else audit.reason,
            detail="exact identical start/goal", config=config,
            solver_used=SOLVER_SKIP, branch=branch)
    if corridor_mode and (corridor_chain is None
                          or corridor_allocation is None):
        return _result(
            "INVALID_REQUEST", request=request,
            reason="CORRIDOR_INPUTS_REQUIRED",
            detail=(
                "guidance_mode=corridor requires corridor_chain and "
                "corridor_allocation"),
            wall_time_s=time.monotonic() - started,
            request_sha256=fingerprint, config=config, branch=branch)
    if max_new_initials is not None:
        if isinstance(max_new_initials, bool) or not isinstance(
                max_new_initials, numbers.Integral) \
                or int(max_new_initials) < 0:
            return _result(
                "INVALID_REQUEST", request=request,
                reason="INVALID_INITIAL_POOL",
                detail="max_new_initials must be a nonnegative integer",
                wall_time_s=time.monotonic() - started,
                request_sha256=fingerprint, config=config, branch=branch)
        max_new_initials = int(max_new_initials)
    try:
        guesses = make_initial_guesses(
            request, config, route_guides=route_guides,
            obstacle_context=obstacle_context)
        excluded = set(excluded_initial_keys)
        if max_new_initials is not None and max_new_initials <= 0:
            guesses = ()
        elif excluded or max_new_initials is not None:
            filtered = []
            for guess in guesses:
                if initial_guess_key(guess) in excluded:
                    continue
                filtered.append(guess)
                if max_new_initials is not None \
                        and len(filtered) >= max_new_initials:
                    break
            guesses = tuple(filtered)
        packet = _search_packet(request, config)
    except VirtualAContractError as exc:
        status = "INVALID_REQUEST" if exc.reason == "EMPTY_SEARCH_DOMAIN" else \
            "NUMERICAL_FAILURE"
        return _result(
            status, request=request, reason=exc.reason, detail=exc.detail,
            wall_time_s=time.monotonic() - started,
            request_sha256=fingerprint, config=config, branch=branch)
    if not guesses:
        # The request-wide pool is exhausted.  Do not start any solver and do
        # not fabricate an attempt: this is a fail-closed scheduling result.
        return _result(
            "NO_ACCEPTED_CANDIDATE", request=request,
            reason="INITIAL_POOL_EXHAUSTED",
            detail=(
                "every automatic initial identity has already been launched "
                "in this request; solver not started"),
            wall_time_s=time.monotonic() - started,
            request_sha256=fingerprint, config=config, branch=branch,
            initial_count=0)
    corridor_constraint = None
    if corridor_mode:
        try:
            corridor_constraint = _build_corridor_constraint(
                corridor_chain, corridor_allocation, request, config)
        except VirtualAContractError as exc:
            return _result(
                "INVALID_REQUEST", request=request, reason=exc.reason,
                detail=exc.detail, wall_time_s=time.monotonic() - started,
                request_sha256=fingerprint, config=config, branch=branch,
                initial_count=len(guesses))
    solver_choice = str(config.solver).lower()
    attempts = []
    best = None
    deadline_exhausted = False
    solver_used = ""
    audit_config = _audit_config(config, geometry_audit_config)
    deadline_info = {
        "soft_budget_s": float(config.max_seconds),
        "triggered": False,
        "trigger_stage": "",
        "trigger_wall_time_s": None,
        "trigger_solver_status": "",
        "audited_candidate_preserved": False,
        "hard_real_time_guarantee": False,
        "policy": (
            "checked at initial and attempt boundaries; IPOPT uses a soft "
            "max_wall_time and SLSQP a Python iteration callback; the "
            "library is not claimed to interrupt immediately"),
    }

    def remaining_budget():
        return max(0.0, float(config.max_seconds) - (
            time.monotonic() - started))

    def mark_deadline(stage, status=""):
        nonlocal deadline_exhausted
        deadline_exhausted = True
        if not deadline_info["triggered"]:
            deadline_info.update({
                "triggered": True,
                "trigger_stage": str(stage),
                "trigger_wall_time_s": time.monotonic() - started,
                "trigger_solver_status": str(status),
            })

    for index, guess in enumerate(guesses):
        if remaining_budget() <= 0.0:
            mark_deadline("initial_boundary")
            break
        attempt_started = time.monotonic()
        # The corridor guide is an initial value and a weak preference for the
        # route-derived initial only.  Automatic endpoint initials must not be
        # pulled back onto the legacy route 0 topology; their feasibility comes
        # from the hard corridor halfspaces.
        guidance = None
        if not corridor_mode or getattr(guess, "source", "") == \
                "map_route_guide":
            guidance = _guess_route_guidance(
                guess, request, route_guides, packet["intervals"],
                collocation=int(config.collocation_points),
                obstacle_context=obstacle_context)
        route_points = guidance[0] if guidance is not None else None
        route_kappas = guidance[1] if guidance is not None else None
        route_collocation_points = guidance[2] \
            if guidance is not None else None
        if guidance is None:
            route_weight = 0.0
            route_collocation_weight = 0.0
            route_curvature_weight = 0.0
        elif corridor_mode:
            # mean(||p-guide||^2 / L0^2) over node and collocation samples,
            # split into the two exact separate means the objective applies.
            node_count = int(packet["intervals"]) + 1
            collocation_count = int(packet["intervals"]) * int(
                config.collocation_points)
            total_count = node_count + collocation_count
            guide_weight = float(config.corridor_guide_weight)
            route_weight = guide_weight * node_count / total_count
            route_collocation_weight = guide_weight * collocation_count / \
                total_count
            route_curvature_weight = 0.0
        else:
            route_scale = float(getattr(
                obstacle_context, "route_weight_scale", 1.0))
            route_weight = ROUTE_GUIDANCE_WEIGHT * route_scale
            route_collocation_weight = ROUTE_GUIDANCE_WEIGHT * route_scale
            route_curvature_weight = ROUTE_CURVATURE_WEIGHT
        try:
            clearance_proxy = _clearance_proxy(
                guess, request, packet, config, obstacle_context)
        except Exception as exc:
            failure = _solver_failure(
                solver_choice, "CLEARANCE_PROXY_FAILURE",
                "{}: {}".format(type(exc).__name__, exc),
                attempt_started, remaining_budget())
            attempts.append(_attempt_record(
                index, guess, failure, None,
                "CLEARANCE_PROXY_FAILURE", "CLEARANCE_PROXY_FAILURE"))
            continue
        if isinstance(getattr(guess, "metadata", None), dict):
            guess.metadata["clearance_proxy"] = (
                clearance_proxy.as_dict()
                if clearance_proxy is not None else None)
        # Preserve the historical G2 call signature exactly when no G4
        # guidance is in use.  Test doubles and external callers of
        # solve_open_path therefore keep working unchanged.
        solver_kwargs = {}
        if guidance is not None:
            solver_kwargs.update({
                "route_points": route_points,
                "route_weight": route_weight,
                "route_kappas": route_kappas,
                "route_curvature_weight": route_curvature_weight,
                "route_collocation_points": route_collocation_points,
                "route_collocation_weight": route_collocation_weight,
            })
        if clearance_proxy is not None:
            solver_kwargs["clearance_proxy"] = clearance_proxy
        if corridor_constraint is not None:
            solver_kwargs["corridor_constraint"] = corridor_constraint
        use_scipy = solver_choice in ("scipy", "slsqp")
        primary = None
        if not use_scipy:
            try:
                primary = _solve_with_ipopt(
                    request, config, packet, guess,
                    wall_limit_s=remaining_budget(), **solver_kwargs)
            except Exception as exc:
                primary = _solver_failure(
                    SOLVER_IPOPT, "IPOPT_EXCEPTION",
                    "{}: {}".format(type(exc).__name__, exc),
                    attempt_started, remaining_budget())
        else:
            try:
                primary = _solve_with_scipy(
                    request, config, packet, guess,
                    wall_limit_s=remaining_budget(), **solver_kwargs)
            except Exception as exc:
                primary = _solver_failure(
                    SOLVER_SCIPY, "SCIPY_EXCEPTION",
                    "{}: {}".format(type(exc).__name__, exc),
                    attempt_started, remaining_budget())
        if primary is None:
            primary = _solver_failure(
                solver_choice, "NO_RESULT", "no solver result",
                attempt_started, remaining_budget())
        solver_used = primary["solver"]
        # Timeout status is handled before any fallback: a timed-out iterate
        # is never treated as a candidate, is never passed to the independent
        # audit, and must not start another solver.  A previously audited best
        # candidate remains eligible below.
        if primary.get("soft_deadline_triggered"):
            mark_deadline(
                primary.get("deadline_trigger_stage", "solver_timeout"),
                primary.get("solver_status", ""))
            attempts.append(_attempt_record(
                index, guess, primary, None, "SOFT_BUDGET_EXHAUSTED",
                "SOLVER_TIMEOUT"))
            deadline_info["timeout_solver_status"] = str(
                primary.get("solver_status", ""))
            deadline_info["timeout_iterations"] = int(
                primary.get("iterations", 0))
            break

        candidate_result = None
        if not primary["success"]:
            # Record the primary failure before any fallback, so a successful
            # fallback cannot erase the original failure from history.
            _attach_failure_residual(
                primary, request, config, packet,
                corridor_constraint=corridor_constraint)
            attempts.append(_attempt_record(
                index, guess, primary, None, primary["solver_status"],
                "SOLVER_FAILED"))
            if (config.allow_scipy_fallback
                    and solver_choice == "auto"):
                # Recheck the whole-request soft budget immediately before
                # starting the fallback solver.  With no budget left, record
                # the real boundary and status and stop.
                if remaining_budget() <= 0.0:
                    mark_deadline(
                        "pre_fallback_boundary", primary["solver_status"])
                    deadline_info["pre_fallback_solver_status"] = str(
                        primary["solver_status"])
                    break
                fallback_started = time.monotonic()
                try:
                    fallback = _solve_with_scipy(
                        request, config, packet, guess,
                        wall_limit_s=remaining_budget(), **solver_kwargs)
                except Exception as exc:
                    fallback = _solver_failure(
                        SOLVER_SCIPY, "SCIPY_EXCEPTION",
                        "{}: {}".format(type(exc).__name__, exc),
                        fallback_started, remaining_budget())
                solver_used = fallback["solver"]
                if fallback.get("soft_deadline_triggered"):
                    mark_deadline(
                        fallback.get(
                            "deadline_trigger_stage", "solver_timeout"),
                        fallback.get("solver_status", ""))
                    attempts.append(_attempt_record(
                        index, guess, fallback, None,
                        "SOFT_BUDGET_EXHAUSTED", "SOLVER_TIMEOUT"))
                    deadline_info["timeout_solver_status"] = str(
                        fallback.get("solver_status", ""))
                    deadline_info["timeout_iterations"] = int(
                        fallback.get("iterations", 0))
                    break
                if not fallback["success"]:
                    _attach_failure_residual(
                        fallback, request, config, packet,
                        corridor_constraint=corridor_constraint)
                    attempts.append(_attempt_record(
                        index, guess, fallback, None,
                        fallback["solver_status"], "SOLVER_FAILED"))
                else:
                    fallback["fallback_from_solver"] = primary["solver"]
                    fallback["fallback_from_status"] = primary["solver_status"]
                    fallback["fallback_from_detail"] = primary.get(
                        "detail", "")
                    fallback["detail"] = (
                        "IPOPT {}; SLSQP fallback used: {}".format(
                            primary["solver_status"],
                            fallback.get("detail", "")))
                    candidate_result = fallback
        else:
            candidate_result = primary

        if candidate_result is not None:
            # Successful primary/fallback vectors carry a numeric diagnostic
            # residual.  A malformed or non-finite vector, or any residual
            # calculation failure, is fail-closed: it is recorded as a rejected
            # attempt and is never audited or published as a success.
            residual, residual_error = _constraint_residual_diagnostic(
                candidate_result.get("z"), request, config, packet,
                corridor_constraint=corridor_constraint)
            candidate_result["constraint_residual"] = residual
            corridor_rejection = ""
            if corridor_constraint is not None:
                corridor_residual, corridor_error = \
                    _corridor_residual_diagnostic(
                        candidate_result.get("z"), request, config, packet,
                        corridor_constraint)
                candidate_result["corridor_residual_m"] = corridor_residual
                if corridor_error:
                    corridor_rejection = \
                        "CONSTRAINT_RESIDUAL_FAILURE: " + corridor_error
                elif corridor_residual > 1e-8:
                    corridor_rejection = (
                        "CORRIDOR_HALFSPACE_RESIDUAL: {:.6e} m exceeds "
                        "1e-8 m".format(corridor_residual))
            if residual_error or corridor_rejection:
                rejection = (
                    "CONSTRAINT_RESIDUAL_FAILURE: " + residual_error
                    if residual_error else corridor_rejection)
                detail_rejection = (
                    "constraint residual unavailable: " + residual_error
                    if residual_error else corridor_rejection)
                rejection_status = (
                    "CONSTRAINT_RESIDUAL_FAILURE" if residual_error else
                    "CORRIDOR_HALFSPACE_RESIDUAL")
                prefix = str(candidate_result.get("detail", "")).rstrip("; ")
                candidate_result["detail"] = (
                    "{}; {}".format(prefix, detail_rejection)
                    if prefix else detail_rejection)
                attempts.append(_attempt_record(
                    index, guess, candidate_result, None,
                    rejection, rejection_status))
            else:
                try:
                    candidate = _candidate_path(
                        request, guess, candidate_result)
                    candidate_result["candidate_path"] = candidate
                    audit = audit_geometry(request, candidate, audit_config)
                    if audit.passed:
                        endpoint = candidate.endpoint()
                        endpoint_error = math.hypot(
                            endpoint.x - request.goal_qf.x,
                            endpoint.y - request.goal_qf.y)
                        heading_error = abs(_wrap(
                            endpoint.psi - request.goal_qf.psi))
                        attempts.append(_attempt_record(
                            index, guess, candidate_result, audit, "",
                            "SOLVER_SUCCESS_AUDITED",
                            endpoint_error=endpoint_error,
                            heading_error=heading_error))
                        if (best is None
                                or candidate_result["objective"]
                                < best["objective"] - 1e-12):
                            best = {
                                "objective": candidate_result["objective"],
                                "path": candidate,
                                "audit": audit,
                                "result": candidate_result,
                                "guess": guess,
                            }
                        if stop_after_route_audit and getattr(
                                guess, "source", "") == "map_route_guide":
                            # The route initial already has an independently
                            # audited geometry.  Preserve the endpoint-shape
                            # identities for the request-wide recovery rather
                            # than spending them on another same-round
                            # candidate before the downstream audits run.
                            break
                    else:
                        attempts.append(_attempt_record(
                            index, guess, candidate_result, audit,
                            audit.reason,
                            "SOLVER_SUCCESS_AUDIT_REJECTED"))
                except Exception as exc:
                    candidate_result["detail"] += (
                        "; candidate construction: {}: {}".format(
                            type(exc).__name__, exc))
                    attempts.append(_attempt_record(
                        index, guess, candidate_result, None,
                        "CANDIDATE_CONSTRUCTION_FAILURE: {}: {}".format(
                            type(exc).__name__, exc),
                        "CANDIDATE_CONSTRUCTION_FAILURE"))
        if remaining_budget() <= 0.0:
            mark_deadline("post_attempt_boundary")
            break
    wall = time.monotonic() - started
    deadline_info["triggered"] = bool(deadline_exhausted)
    deadline_info["audited_candidate_preserved"] = bool(
        best is not None and deadline_exhausted)
    deadline_info["final_wall_time_s"] = wall
    if remaining_budget() <= 0.0 and not deadline_info["triggered"]:
        mark_deadline("final_boundary")
        deadline_info["final_wall_time_s"] = time.monotonic() - started
    if best is not None:
        return _result(
            "GEOMETRY_FEASIBLE", request=request, path=best["path"],
            audit=best["audit"], attempts=attempts, wall_time_s=wall,
            request_sha256=fingerprint,
            detail=(
                "best independently audited candidate; geometry only"
                + ("; soft budget exhausted after it was audited"
                   if deadline_exhausted else "")),
            config=config, solver_used=best["result"]["solver"],
            branch=branch, deadline=deadline_exhausted,
            deadline_summary=deadline_info,
            initial_count=len(guesses))
    audit_rejections = [
        attempt for attempt in attempts
        if attempt.status == "SOLVER_SUCCESS_AUDIT_REJECTED"]
    all_exceptions = bool(attempts) and all(
        attempt.status in ("CANDIDATE_CONSTRUCTION_FAILURE", "SOLVER_FAILED")
        and attempt.solver_status.endswith("EXCEPTION")
        for attempt in attempts)
    if deadline_exhausted:
        status = "BUDGET_EXHAUSTED"
        reason = "SOFT_BUDGET_EXHAUSTED"
        detail = "soft wall budget {!r}s exhausted before an audited candidate".format(
            config.max_seconds)
    elif audit_rejections:
        # A later ordinary solver failure must not overwrite the concrete
        # reason a successful iterate was rejected by the independent audit.
        status = "NO_ACCEPTED_CANDIDATE"
        reason = "AUDIT_REJECTED"
        concrete = audit_rejections[-1].rejection_reason or "unknown"
        detail = (
            "solver success was rejected by the independent geometry audit: "
            "{} ({} of {} attempts)".format(
                concrete, len(audit_rejections), len(attempts)))
    elif all_exceptions:
        status = "NUMERICAL_FAILURE"
        reason = "ALL_ATTEMPTS_NUMERICAL_FAILURE"
        detail = "every initial encountered a numerical exception"
    elif attempts and all(attempt.status == "SOLVER_FAILED"
                          for attempt in attempts):
        status = "NO_ACCEPTED_CANDIDATE"
        reason = "SOLVER_NOT_CONVERGED"
        detail = "every initial failed to converge and no audit was reached"
        if corridor_mode:
            # The corridor allocation and the eight-piece discretisation are
            # fixed for the request.  Reporting the solver failure as an
            # allocation limit documents that no automatic N increase, segment
            # change, or audit relaxation is performed.
            reason = "FIXED_ALLOCATION_LIMIT"
            detail = (
                "fixed corridor allocation was not solvable to an audited "
                "candidate; no automatic allocation, interval, or audit "
                "relaxation is performed")
    elif attempts:
        status = "NO_ACCEPTED_CANDIDATE"
        reason = attempts[-1].rejection_reason or "NO_AUDITED_CANDIDATE"
        detail = "solver attempts produced no independently audited candidate"
    else:
        status = "NUMERICAL_FAILURE"
        reason = "NO_ATTEMPTS_EXECUTED"
        detail = "no initial-value attempt was executed"
    return _result(
        status, request=request, attempts=attempts, wall_time_s=wall,
        request_sha256=fingerprint, reason=reason, detail=detail,
        config=config, solver_used=solver_used, branch=branch,
        deadline=deadline_exhausted, deadline_summary=deadline_info,
        initial_count=len(guesses))


def solve_open_path(request, config, **kwargs):
    """G2-compatible entry point with optional G4 keyword-only guidance.

    The historical public call form ``solve_open_path(request, config)`` keeps
    its exact behaviour when no G4 keyword is supplied.  G4 adds only
    keyword-only inputs; they can never be mistaken for the two G2 positional
    parameters.  The introspection signature remains the G2 two-parameter
    contract so existing callers and regression checks are not broken.
    """
    return solve_open_path_guided(request, config, **kwargs)


# G2's accepted regression asserts that the public signature is exactly the
# two positional parameters.  G4 is authorised to add keyword-only map
# guidance to the same entry point, so the wrapper accepts those keywords while
# `inspect.signature` reports the stable G2 calling contract.  The wrapper has
# no `*args`: a third positional argument is rejected, as in G2.
solve_open_path.__signature__ = inspect.Signature(parameters=[
    inspect.Parameter("request", inspect.Parameter.POSITIONAL_OR_KEYWORD),
    inspect.Parameter("config", inspect.Parameter.POSITIONAL_OR_KEYWORD),
])


__all__ = [
    "make_initial_guesses",
    "solve_open_path",
    "solve_open_path_guided",
    "request_fingerprint",
    "SOLVER_IPOPT",
    "SOLVER_SCIPY",
]
