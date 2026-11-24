"""Whole-path common-time quintic retiming for the A-only formation (G3).

The formation progress is one rest-to-rest quintic over the complete C2
Hermite path; there is no per-knot stop, dwell, reconfiguration or hidden
setup.  Dynamic limits are the original per-vehicle ``FormationTimingLimits``.
The total duration is selected from outward whole-domain interval envelopes
at ``T0 = 1 s`` and a time-law power scaling, then multiplied by the explicit
safety factor.

This module does not claim executable, collision-checked or ROS-valid
behaviour.  The independent dynamic audit lives in :mod:`audit`.
"""
import json
import math
import time

from . import bounds
from .contracts import (
    BACKEND,
    TIMING_LAW_VERSION,
    TIMING_STATUSES,
    PATH_MODEL_VERSION,
    TIMED_MODEL_VERSION,
    TimingConfig,
    TimingResult,
    VirtualAContractError,
    VirtualARequest,
    _finite,
    geometry_path_summary,
    request_fingerprint,
    request_summary,
    validate_timing_config,
    validate_virtual_a_request,
)
from .geometry import VirtualAPath

TIME_INVERSE_ABS_TOL_S = 1e-9
_SIDES = ("auto", "left", "right")

# Quantity name, limit field on FormationTimingLimits, time-law order.
_SCALED_METRICS = (
    ("speed", "v_max", 1),
    ("acceleration", "a_max", 2),
    ("jerk", "jerk_max", 3),
    ("yaw_rate", "yaw_rate_max", 1),
    ("yaw_acceleration", "yaw_accel_max", 2),
    ("yaw_jerk", "yaw_jerk_max", 3),
    ("lateral_acceleration", "lateral_accel_max", 2),
    ("wheel_speed", "wheel_speed_max", 1),
    ("steer_rate", "steer_rate_max", 1),
    ("steer_acceleration", "steer_accel_max", 2),
)

_WHEEL_COORDINATES = (
    (0.5, 0.5), (0.5, -0.5), (-0.5, 0.5), (-0.5, -0.5))


class _BoundFailure(Exception):
    def __init__(self, reason, detail="", **location):
        self.reason = str(reason)
        self.detail = str(detail)
        self.location = dict(location)
        super().__init__(self.reason)


def _require_side(side):
    if side not in _SIDES:
        raise VirtualAContractError(
            "INVALID_SIDE", "side={!r} allowed={}".format(side, list(_SIDES)))


def quintic_progress(tau):
    """Unit rest-to-rest displacement ``10 tau^3 - 15 tau^4 + 6 tau^5``."""
    tau = _finite(tau, "tau")
    return tau ** 3 * (10.0 - 15.0 * tau + 6.0 * tau * tau)


def quintic_derivatives(tau):
    """Return ``(h, h1, h2, h3)`` for the common time law."""
    tau = _finite(tau, "tau")
    h = tau ** 3 * (10.0 - 15.0 * tau + 6.0 * tau * tau)
    h1 = 30.0 * tau * tau * (1.0 - tau) ** 2
    h2 = 60.0 * tau - 180.0 * tau * tau + 120.0 * tau ** 3
    h3 = 60.0 - 360.0 * tau + 360.0 * tau * tau
    return h, h1, h2, h3


def progress_is_strictly_increasing(tau):
    """Analytic interior check: ``h1(tau) > 0`` for ``0 < tau < 1``."""
    tau = _finite(tau, "tau")
    return 0.0 < tau < 1.0


def tau_at_fraction(fraction, iterations=100):
    """Invert ``h(tau) = fraction`` by monotone bisection.

    ``fraction`` is in ``[0, 1]``.  The returned tau interval width is below
    2e-15 after at most 100 halvings, independent of the final duration.
    """
    fraction = _finite(fraction, "progress fraction")
    if fraction < 0.0 or fraction > 1.0:
        raise VirtualAContractError(
            "S_DOMAIN",
            "progress fraction={!r} outside [0, 1]".format(fraction),
            fraction=fraction)
    if fraction == 0.0:
        return 0.0
    if fraction == 1.0:
        return 1.0
    low, high = 0.0, 1.0
    for _ in range(int(iterations)):
        middle = 0.5 * (low + high)
        if quintic_progress(middle) < fraction:
            low = middle
        else:
            high = middle
        if high - low <= 2e-15:
            break
    return 0.5 * (low + high)


def tau_at_s(s, total_s):
    """Normalized time corresponding to formation station ``s``."""
    s = _finite(s, "station")
    total = _finite(total_s, "total_S")
    if total < 0.0:
        raise VirtualAContractError(
            "INVALID_PATH_LENGTH", "total_S={!r}".format(total))
    if total == 0.0:
        if s != 0.0:
            raise VirtualAContractError(
                "S_DOMAIN", "zero-motion plan accepts s=0 only; got {!r}".format(
                    s), s=s)
        return 0.0
    if s < 0.0 or s > total:
        raise VirtualAContractError(
            "S_DOMAIN",
            "s={!r} outside [0, {!r}]; no clipping".format(s, total), s=s)
    return tau_at_fraction(s / total)


def time_at_s_seconds(s, total_s, duration_s, iterations=100):
    """Inverse time mapping with a certified bisection width."""
    duration = _finite(duration_s, "duration")
    if duration < 0.0:
        raise VirtualAContractError(
            "INVALID_DURATION", "duration={!r}".format(duration))
    total = _finite(total_s, "total_S")
    s = _finite(s, "station")
    if total == 0.0:
        if s != 0.0:
            raise VirtualAContractError(
                "S_DOMAIN", "zero-motion plan accepts s=0 only; got {!r}".format(
                    s), s=s)
        return 0.0
    if duration == 0.0:
        raise VirtualAContractError(
            "INVALID_DURATION", "a motion plan needs duration > 0")
    if s < 0.0 or s > total:
        raise VirtualAContractError(
            "S_DOMAIN",
            "s={!r} outside [0, {!r}]; no clipping".format(s, total), s=s)
    if s == 0.0:
        return 0.0
    if s == total:
        return duration
    fraction = s / total
    low, high = 0.0, 1.0
    target = 0.5 * TIME_INVERSE_ABS_TOL_S
    for _ in range(int(iterations)):
        middle = 0.5 * (low + high)
        if quintic_progress(middle) < fraction:
            low = middle
        else:
            high = middle
        # The root is inside [low, high]; half of the time width bounds the
        # error of the returned midpoint by 0.5*width*duration.
        if 0.5 * (high - low) * duration <= target:
            break
    if 0.5 * (high - low) * duration > TIME_INVERSE_ABS_TOL_S:
        raise VirtualAContractError(
            "TIME_INVERSE_UNCONFIRMED",
            "bisection width exceeds the 1e-9 s contract",
            width_s=0.5 * (high - low) * duration)
    return 0.5 * (low + high) * duration


def _isqrt_interval(interval):
    return bounds.isqrt_positive(interval)


def _divide_positive(numerator, denominator):
    return bounds.idiv_positive_interval(numerator, denominator)


def _vehicle_bounds(jets, offset, limits, w, a, j, xw, yw):
    """Rigorous dynamic bounds for one vehicle and one wheel position.

    ``jets``/``w``/``a``/``j`` are closed intervals for one curvature-profile
    cell and one closed tau cell.  Every Hermite segment is partitioned into
    closed cells, so this enclosure applies to the complete segment including
    both node limits: a node belongs to the last cell of the segment on its
    left and to the first cell of the segment on its right.  The third spatial
    derivative and jerk expressions below are the same rigid-offset formulas
    used by the independent pointwise audit:

    ``R(-psi) p_sss = (-k²+b*k³-3*a*k*eta-b*u,
                        eta-3*b*k*eta+a*u-a*k³)``
    ``jerk = p_sss*w³ + 3*p_ss*w*a_time + p_s*j_time``.

    All expressions are measured in metres and radians, with
    first/second/third time orders.  The scalar ``jerk`` return value is the
    outward Euclidean norm upper bound used by duration selection; the
    component intervals are additional independent-coverage diagnostics.
    """
    k = jets["k"]
    eta = jets["eta"]
    u = jets["u"]
    g = jets["g"]
    aoff, boff = offset.x, offset.y
    radius2 = aoff * aoff + boff * boff
    A = bounds.iadd((1.0, 1.0), bounds.iscale(k, -boff))
    B = bounds.iscale(k, aoff)
    D = bounds.iadd(bounds.imul(A, A), bounds.imul(B, B))
    if A[0] <= 0.0 or D[0] <= 0.0:
        raise _BoundFailure(
            "NORMAL_BRANCH_VIOLATION",
            "vehicle_id={} A_low={!r} D_low={!r}".format(
                offset.vehicle_id, A[0], D[0]),
            vehicle_id=offset.vehicle_id, A_low=A[0], D_low=D[0])
    Q = _isqrt_interval(D)
    Q2 = bounds.isquare(Q)
    Q3 = bounds.imul(Q2, Q)
    D_s = bounds.iscale(
        bounds.imul(bounds.iadd(
            bounds.iscale(k, radius2), (-boff, -boff)), eta), 2.0)
    D_ss = bounds.iscale(
        bounds.iadd(
            bounds.iscale(bounds.isquare(eta), radius2),
            bounds.imul(
                bounds.iadd(bounds.iscale(k, radius2), (-boff, -boff)), u)),
        2.0)
    Q_s = _divide_positive(D_s, bounds.iscale(Q, 2.0))
    Q_ss = bounds.isub(
        _divide_positive(D_ss, bounds.iscale(Q, 2.0)),
        _divide_positive(bounds.isquare(D_s), bounds.iscale(Q3, 4.0)))
    H = bounds.iadd(k, _divide_positive(bounds.iscale(eta, aoff), D))
    H_s = bounds.iadd(
        eta,
        bounds.iscale(
            bounds.isub(
                _divide_positive(u, D),
                _divide_positive(
                    bounds.imul(eta, D_s), bounds.isquare(D))),
            aoff))
    H_ss = bounds.iadd(
        u,
        bounds.iscale(
            bounds.iadd(
                bounds.isub(
                    _divide_positive(g, D),
                    _divide_positive(
                        bounds.iscale(bounds.imul(u, D_s), 2.0),
                        bounds.isquare(D))),
                bounds.isub(
                    bounds.iscale(
                        _divide_positive(
                            bounds.imul(eta, D_ss), bounds.isquare(D)), -1.0),
                    bounds.iscale(
                        _divide_positive(
                            bounds.imul(eta, bounds.isquare(D_s)),
                            bounds.imul(bounds.isquare(D), D)), -2.0))),
            aoff))
    C = _divide_positive(H, Q)
    C_s = bounds.isub(
        _divide_positive(H_s, Q),
        _divide_positive(bounds.imul(H, Q_s), Q2))
    C_ss = bounds.iadd(
        bounds.isub(
            bounds.isub(
                _divide_positive(H_ss, Q),
                _divide_positive(
                    bounds.iscale(bounds.imul(H_s, Q_s), 2.0), Q2)),
            _divide_positive(bounds.imul(H, Q_ss), Q2)),
        _divide_positive(
            bounds.iscale(bounds.imul(H, bounds.isquare(Q_s)), 2.0),
            bounds.imul(Q2, Q)))
    # Norm bounds of the explicit world spatial derivatives.  Rotation by psi
    # preserves the Euclidean norm; it is not needed for a scalar upper bound.
    z2 = (
        bounds.iadd(bounds.iscale(bounds.isquare(k), -aoff),
                    bounds.iscale(eta, -boff)),
        bounds.iadd(bounds.isub(k, bounds.iscale(bounds.isquare(k), boff)),
                    bounds.iscale(eta, aoff)))
    k2 = bounds.isquare(k)
    k3 = bounds.imul(k2, k)
    z3 = (
        bounds.isub(
            bounds.iadd(
                bounds.isub(bounds.iscale(k3, boff), k2),
                bounds.iscale(bounds.imul(k, eta), -3.0 * aoff)),
            bounds.iscale(u, boff)),
        bounds.isub(
            bounds.iadd(
                bounds.isub(eta,
                            bounds.iscale(bounds.imul(k, eta), 3.0 * boff)),
                bounds.iscale(u, aoff)),
            bounds.iscale(k3, aoff)))
    p_s_norm = bounds.inorm2_upper((A, B))
    p_ss_norm = bounds.inorm2_upper(z2)
    p_sss_norm = bounds.inorm2_upper(z3)
    w_abs = bounds.imax_abs(w)
    a_abs = bounds.imax_abs(a)
    j_abs = bounds.imax_abs(j)
    w2 = bounds._next_up(w_abs * w_abs)
    w3 = bounds._next_up(bounds._next_up(w_abs * w_abs) * w_abs)
    Q_abs = bounds.imax_abs(Q)
    v_abs = bounds._next_up(Q_abs * w_abs)
    omega_abs = bounds._next_up(bounds.imax_abs(H) * w_abs)
    acceleration = bounds._next_up(
        bounds._next_up(p_ss_norm * w2) + bounds._next_up(p_s_norm * a_abs))
    jerk = bounds._next_up(
        bounds._next_up(p_sss_norm * w3)
        + bounds._next_up(
            bounds._next_up(3.0 * p_ss_norm * w_abs) * a_abs)
        + bounds._next_up(p_s_norm * j_abs))
    # Per-component enclosures in the same local (tangent/normal) frame as
    # ``z2``/``z3``.  These are diagnostics and independent coverage checks;
    # ``jerk`` above remains the certified Euclidean upper bound used by the
    # selection and by the dynamic audit.
    w_cubed = bounds.imul(bounds.imul(w, w), w)
    jerk_components = (
        bounds.iadd(
            bounds.iadd(
                bounds.imul(z3[0], w_cubed),
                bounds.iscale(bounds.imul(bounds.imul(z2[0], w), a), 3.0)),
            bounds.imul(A, j)),
        bounds.iadd(
            bounds.iadd(
                bounds.imul(z3[1], w_cubed),
                bounds.iscale(bounds.imul(bounds.imul(z2[1], w), a), 3.0)),
            bounds.imul(B, j)))
    yaw_accel = bounds._next_up(
        bounds._next_up(bounds.imax_abs(H) * a_abs)
        + bounds._next_up(bounds.imax_abs(H_s) * w2))
    yaw_jerk = bounds._next_up(
        bounds._next_up(bounds.imax_abs(H) * j_abs)
        + bounds._next_up(
            bounds._next_up(3.0 * bounds.imax_abs(H_s) * w_abs) * a_abs)
        + bounds._next_up(bounds.imax_abs(H_ss) * w3))
    lateral = bounds._next_up(bounds.imax_abs(C) * bounds._next_up(
        v_abs * v_abs))
    m = bounds.iadd((1.0, 1.0), bounds.iscale(C, -yw))
    n = bounds.iscale(C, xw)
    Ew = bounds.iadd(bounds.isquare(m), bounds.isquare(n))
    if Ew[0] <= 0.0:
        raise _BoundFailure(
            "WHEEL_GEOMETRY_VIOLATION",
            "vehicle_id={} Ew_low={!r}".format(offset.vehicle_id, Ew[0]),
            vehicle_id=offset.vehicle_id, Ew_low=Ew[0])
    factor = _isqrt_interval(Ew)
    Ew2 = bounds.isquare(Ew)
    Ew_C = bounds.iadd(
        (-2.0 * yw, -2.0 * yw),
        bounds.iscale(C, 2.0 * (xw * xw + yw * yw)))
    delta_s = _divide_positive(bounds.iscale(C_s, xw), Ew)
    delta_ss = bounds.isub(
        _divide_positive(bounds.iscale(C_ss, xw), Ew),
        _divide_positive(
            bounds.iscale(bounds.imul(Ew_C, bounds.isquare(C_s)), xw), Ew2))
    wheel_speed = bounds._next_up(bounds.imax_abs(factor) * v_abs)
    steer_rate = bounds._next_up(bounds.imax_abs(delta_s) * w_abs)
    steer_accel = bounds._next_up(
        bounds._next_up(bounds.imax_abs(delta_ss) * w2)
        + bounds._next_up(bounds.imax_abs(delta_s) * a_abs))
    # Static ideal-wheel angle bound is time-independent; endpoints of the C
    # interval suffice because delta(C) is monotone for each fixed wheel.
    wheel_angle = _static_wheel_angle_interval(C, limits.wheelbase, limits.track)
    return {
        "speed": v_abs,
        "acceleration": acceleration,
        "jerk": jerk,
        "yaw_rate": omega_abs,
        "yaw_acceleration": yaw_accel,
        "yaw_jerk": yaw_jerk,
        "lateral_acceleration": lateral,
        "wheel_speed": wheel_speed,
        "steer_rate": steer_rate,
        "steer_acceleration": steer_accel,
        "steer_angle": wheel_angle,
        "Q_upper": Q_abs,
        "C_abs_upper": bounds.imax_abs(C),
        "C_s_abs_upper": bounds.imax_abs(C_s),
        "C_ss_abs_upper": bounds.imax_abs(C_ss),
        "H_abs_upper": bounds.imax_abs(H),
        "H_s_abs_upper": bounds.imax_abs(H_s),
        "H_ss_abs_upper": bounds.imax_abs(H_ss),
        "p_s_norm_upper": p_s_norm,
        "p_ss_norm_upper": p_ss_norm,
        "p_sss_norm_upper": p_sss_norm,
        "p_sss_local_interval": z3,
        "p_sss_component_upper": (
            bounds.imax_abs(z3[0]), bounds.imax_abs(z3[1])),
        "jerk_local_interval": jerk_components,
        "jerk_component_upper": (
            bounds.imax_abs(jerk_components[0]),
            bounds.imax_abs(jerk_components[1])),
        "jerk_component_norm_upper": bounds.inorm2_upper(jerk_components),
        "wheel_angle_bounds": wheel_angle,
        "wheel_factor_upper": bounds.imax_abs(factor),
        "delta_s_abs_upper": bounds.imax_abs(delta_s),
        "delta_ss_abs_upper": bounds.imax_abs(delta_ss),
    }


def _static_wheel_angle(curvature, wheelbase, track):
    maximum = 0.0
    for xw, yw in ((0.5 * wheelbase, 0.5 * track),
                   (0.5 * wheelbase, -0.5 * track),
                   (-0.5 * wheelbase, 0.5 * track),
                   (-0.5 * wheelbase, -0.5 * track)):
        m = 1.0 - yw * curvature
        n = xw * curvature
        angle = abs(math.atan2(n, m))
        maximum = max(maximum, angle)
    return maximum


def _static_wheel_angle_interval(curvature_interval, wheelbase, track):
    lower, upper = curvature_interval
    value = max(_static_wheel_angle(lower, wheelbase, track),
                _static_wheel_angle(upper, wheelbase, track))
    return bounds._next_up(value)


def _poly_value(coefficients, tau):
    result = 0.0
    for coefficient in reversed(coefficients):
        result = result * tau + coefficient
    return result


_H1_COEFFICIENTS = (0.0, 0.0, 30.0, -60.0, 30.0)
_H2_COEFFICIENTS = (0.0, 60.0, -180.0, 120.0)
_H3_COEFFICIENTS = (60.0, -360.0, 360.0)


def _range_from_values(values):
    lowered = bounds.widen(min(values))
    raised = bounds.widen(max(values))
    return (lowered[0], raised[1])


def _h1_range(tau0, tau1):
    """Exact range of ``h1 = 30 tau^2 (1-tau)^2`` on a tau interval."""
    values = [_poly_value(_H1_COEFFICIENTS, tau0),
              _poly_value(_H1_COEFFICIENTS, tau1)]
    if tau0 <= 0.5 <= tau1:
        values.append(1.875)
    upper = bounds.widen(max(values))[1]
    return (0.0, upper)


def _h2_range(tau0, tau1):
    """Exact range of the cubic ``h2`` including its two critical points."""
    root = math.sqrt(3.0)
    critical = (
        (3.0 - root) / 6.0,
        (3.0 + root) / 6.0,
    )
    values = [_poly_value(_H2_COEFFICIENTS, tau0),
              _poly_value(_H2_COEFFICIENTS, tau1)]
    for tau in critical:
        if tau0 < tau < tau1:
            values.append(_poly_value(_H2_COEFFICIENTS, tau))
    return _range_from_values(values)


def _h3_range(tau0, tau1):
    """Exact range of the convex quadratic ``h3``."""
    values = [_poly_value(_H3_COEFFICIENTS, tau0),
              _poly_value(_H3_COEFFICIENTS, tau1)]
    if tau0 < 0.5 < tau1:
        values.append(-30.0)
    return _range_from_values(values)


def _time_intervals(segment_tau, total_s, duration):
    """Rigorous tight ranges of ``S/T^order * h_order`` on a tau interval.

    Direct interval Horner evaluation is dependency-loose near the endpoints
    (for the straight path it overestimated ``h1`` by a factor of more than
    twenty).  ``h1``/``h2``/``h3`` are fixed low-degree polynomials whose exact
    extrema are available, so their ranges are computed from the endpoints and
    the interior critical points instead.  Scaling by ``S/T^order`` is rounded
    outward by the interval operator.
    """
    tau0, tau1 = segment_tau
    if not (0.0 <= tau0 <= tau1 <= 1.0):
        raise _BoundFailure(
            "INVALID_TAU_INTERVAL",
            "tau interval [{!r}, {!r}]".format(tau0, tau1))
    h1 = _h1_range(tau0, tau1)
    h2 = _h2_range(tau0, tau1)
    h3 = _h3_range(tau0, tau1)
    scale1 = total_s / duration
    scale2 = total_s / (duration * duration)
    scale3 = total_s / (duration * duration * duration)
    return (bounds.iscale(h1, scale1),
            bounds.iscale(h2, scale2),
            bounds.iscale(h3, scale3))


def _iterate_segment_bounds(profile, segment_index, q0, q1, segment_tau,
                            total_s, duration, request):
    piece = profile._pieces[segment_index]
    jets = bounds.segment_jet_bounds(
        piece.c0, piece.c1, piece.c2, piece.c3, piece.h, q0, q1)
    w, a, j = _time_intervals(segment_tau, total_s, duration)
    records = {}
    for offset in request.model.offsets:
        for wheel_index, (xw_factor, yw_factor) in enumerate(
                _WHEEL_COORDINATES):
            xw = xw_factor * request.limits.wheelbase
            yw = yw_factor * request.limits.track
            values = _vehicle_bounds(
                jets, offset, request.limits, w, a, j, xw, yw)
            records[(offset.vehicle_id, wheel_index)] = values
    return records


def _selection_bounds(profile, request, config):
    """Conservative whole-domain maxima at ``T0 = 1 s`` and their locations."""
    total_s = float(profile.length_m)
    if not (math.isfinite(total_s) and total_s > 0.0):
        raise _BoundFailure(
            "INVALID_PATH_LENGTH", "length_m={!r}".format(total_s))
    maxima = {
        name: {
            "value": 0.0, "vehicle_id": None, "wheel_index": None,
            "segment_index": None, "s0": None, "s1": None, "order": order,
            "limit": float(getattr(request.limits, limit_name)),
        }
        for name, limit_name, order in _SCALED_METRICS}
    wheel_angle_max = {
        "value": 0.0, "vehicle_id": None, "wheel_index": None,
        "segment_index": None, "limit": request.limits.steer_angle_max}
    subdivisions = int(config.interval_subdivisions_per_segment)
    for segment_index, (s0, s1) in enumerate(profile.domain_segments):
        tau0 = tau_at_fraction(s0 / total_s)
        tau1 = tau_at_fraction(s1 / total_s)
        for index in range(subdivisions):
            q0 = index / float(subdivisions)
            q1 = (index + 1) / float(subdivisions)
            # Tau is monotone in s, but not linear in q inside a Hermite
            # piece.  Invert both subinterval endpoints exactly.
            local_tau = (
                tau_at_fraction((s0 + (s1 - s0) * q0) / total_s),
                tau_at_fraction((s0 + (s1 - s0) * q1) / total_s))
            records = _iterate_segment_bounds(
                profile, segment_index, q0, q1, local_tau,
                total_s, 1.0, request)
            for (vehicle_id, wheel_index), values in records.items():
                for name, _, _ in _SCALED_METRICS:
                    value = values[name]
                    _finite_bound(value, name, segment_index, vehicle_id)
                    if value > maxima[name]["value"]:
                        maxima[name].update({
                            "value": value, "vehicle_id": vehicle_id,
                            "wheel_index": wheel_index,
                            "segment_index": segment_index,
                            "s0": s0 + (s1 - s0) * q0,
                            "s1": s0 + (s1 - s0) * q1,
                        })
                angle = values["steer_angle"]
                _finite_bound(angle, "steer_angle", segment_index, vehicle_id)
                if angle > wheel_angle_max["value"]:
                    wheel_angle_max.update({
                        "value": angle, "vehicle_id": vehicle_id,
                        "wheel_index": wheel_index,
                        "segment_index": segment_index,
                        "s0": s0 + (s1 - s0) * q0,
                        "s1": s0 + (s1 - s0) * q1,
                    })
    if wheel_angle_max["value"] > wheel_angle_max["limit"]:
        raise _BoundFailure(
            "WHEEL_ANGLE_LIMIT",
            "static ideal-wheel angle bound {!r} exceeds {!r}".format(
                wheel_angle_max["value"], wheel_angle_max["limit"]),
            vehicle_id=wheel_angle_max["vehicle_id"],
            wheel_index=wheel_angle_max["wheel_index"],
            constraint="WHEEL_ANGLE")
    required = 0.0
    worst = None
    per_metric = {}
    for name, _, order in _SCALED_METRICS:
        record = maxima[name]
        ratio = record["value"] / record["limit"]
        candidate = ratio ** (1.0 / order)
        per_metric[name] = {
            "upper_at_T0_s": record["value"],
            "limit": record["limit"],
            "order": order,
            "required_duration_s": candidate,
            "upper_bound_interval": [record["s0"], record["s1"]],
            "vehicle_id": record["vehicle_id"],
            "wheel_index": record["wheel_index"],
            "segment_index": record["segment_index"],
        }
        if candidate > required:
            required = candidate
            worst = (name, record["vehicle_id"], record["wheel_index"],
                     order, record["segment_index"], record["s0"],
                     record["s1"])
    if not (math.isfinite(required) and required > 0.0):
        raise _BoundFailure(
            "NUMERICAL_FAILURE",
            "selected required duration={!r}".format(required))
    duration = required * float(config.safety_factor)
    return {
        "required_duration_s": required,
        "selected_duration_s": duration,
        "safety_factor": float(config.safety_factor),
        "worst": {
            "quantity": worst[0] if worst else None,
            "vehicle_id": worst[1] if worst else None,
            "wheel_index": worst[2] if worst else None,
            "order": worst[3] if worst else None,
            "segment_index": worst[4] if worst else None,
            "s0": worst[5] if worst else None,
            "s1": worst[6] if worst else None,
        },
        "metrics_at_T0_s": per_metric,
        "wheel_angle_bound_rad": wheel_angle_max["value"],
        "wheel_angle_limit_rad": wheel_angle_max["limit"],
        "interval_subdivisions_per_segment": subdivisions,
        "T0_s": 1.0,
    }


def _finite_bound(value, name, segment_index, vehicle_id):
    if not math.isfinite(value) or value < 0.0:
        raise _BoundFailure(
            "NONFINITE_BOUND",
            "{} bound={!r} segment={} vehicle_id={}".format(
                name, value, segment_index, vehicle_id),
            segment_index=segment_index, vehicle_id=vehicle_id,
            constraint="NONFINITE_BOUND")
    return value


def _certify_bounds(profile, request, duration, config):
    """Recheck the whole domain at the selected duration (retimer-side only)."""
    total_s = float(profile.length_m)
    maxima = {name: 0.0 for name, _, _ in _SCALED_METRICS}
    locations = {}
    subdivisions = int(config.interval_subdivisions_per_segment)
    for segment_index, (s0, s1) in enumerate(profile.domain_segments):
        tau0 = tau_at_fraction(s0 / total_s)
        tau1 = tau_at_fraction(s1 / total_s)
        for index in range(subdivisions):
            q0 = index / float(subdivisions)
            q1 = (index + 1) / float(subdivisions)
            local_tau = (
                tau_at_fraction((s0 + (s1 - s0) * q0) / total_s),
                tau_at_fraction((s0 + (s1 - s0) * q1) / total_s))
            records = _iterate_segment_bounds(
                profile, segment_index, q0, q1, local_tau,
                total_s, duration, request)
            for (vehicle_id, wheel_index), values in records.items():
                for name, limit_name, _ in _SCALED_METRICS:
                    value = _finite_bound(
                        values[name], name, segment_index, vehicle_id)
                    if value > maxima[name]:
                        maxima[name] = value
                        locations[name] = {
                            "vehicle_id": vehicle_id,
                            "wheel_index": wheel_index,
                            "segment_index": segment_index,
                            "s0": s0 + (s1 - s0) * q0,
                            "s1": s0 + (s1 - s0) * q1,
                        }
                    limit = float(getattr(request.limits, limit_name))
                    if value > limit:
                        raise _BoundFailure(
                            "DYNAMIC_LIMIT",
                            "{} bound={!r} exceeds {!r}".format(
                                name, value, limit),
                            constraint=name, vehicle_id=vehicle_id,
                            wheel_index=wheel_index,
                            segment_index=segment_index,
                            value=value, limit=limit)
    utilization = {}
    for name, limit_name, _ in _SCALED_METRICS:
        limit = float(getattr(request.limits, limit_name))
        utilization[name] = maxima[name] / limit
    return {
        "duration_s": duration,
        "upper_bounds": maxima,
        "locations": locations,
        "max_utilization": max(utilization.values()),
        "utilization": utilization,
        "interval_subdivisions_per_segment": subdivisions,
    }


def _tau_intervals(profile):
    total_s = float(profile.length_m)
    return [
        (tau_at_fraction(s0 / total_s), tau_at_fraction(s1 / total_s))
        for s0, s1 in profile.domain_segments]


def _validate_path_binding(request, path):
    if not isinstance(path, VirtualAPath):
        return "INVALID_PATH"
    attached = getattr(path, "request_sha256", None)
    if attached is not None and attached != request_fingerprint(request):
        return "FORGED_REQUEST_DIGEST"
    try:
        path_offsets = tuple(
            (offset.vehicle_id, float(offset.x), float(offset.y))
            for offset in path.model.offsets)
        request_offsets = tuple(
            (offset.vehicle_id, float(offset.x), float(offset.y))
            for offset in request.model.offsets)
        if path_offsets != request_offsets:
            return "MODEL_CHANGED"
        if path.start_qf != request.start_qf:
            return "START_MISMATCH"
    except (AttributeError, TypeError, ValueError):
        return "MODEL_CHANGED"
    return ""


def retime_path(request, path, config=None, *, planning_context=None):
    """Retime a G2 path with one common-time quintic; no per-node stop.

    The returned plan is a G3 timed model, not an executable or collision
    certificate.  ``audit_dynamics`` must be run separately.
    """
    started = time.monotonic()
    if config is None:
        config = TimingConfig()
    request_summary_value = request_summary(request) if isinstance(
        request, VirtualARequest) else {
            "type": type(request).__name__, "valid": False}
    request_sha = ""
    path_summary_value = None
    if path is not None:
        try:
            path_summary_value = geometry_path_summary(path)
        except Exception as exc:
            path_summary_value = {
                "summary_error": "{}: {}".format(type(exc).__name__, exc)}
    request_validation = validate_virtual_a_request(request)
    if not request_validation.passed:
        return TimingResult(
            status="INVALID_REQUEST", reason=request_validation.reason,
            detail=request_validation.detail,
            request_summary=request_summary_value,
            config_summary=config.as_dict() if isinstance(
                config, TimingConfig) else {},
            wall_time_s=time.monotonic() - started,
            metrics={"request_validation": request_validation.metrics})
    request_sha = request_fingerprint(request)
    config_validation = validate_timing_config(config)
    if not config_validation.passed:
        return TimingResult(
            status="INVALID_TIMING_CONFIG", reason=config_validation.reason,
            detail=config_validation.detail,
            request_summary=request_summary_value,
            request_sha256=request_sha,
            config_summary=config.as_dict() if isinstance(
                config, TimingConfig) else {},
            wall_time_s=time.monotonic() - started,
            metrics={"config_validation": config_validation.metrics})
    binding = _validate_path_binding(request, path)
    if binding:
        return TimingResult(
            status="INVALID_PATH", reason=binding,
            detail="path is not bound to the original request",
            request_summary=request_summary_value,
            request_sha256=request_sha,
            config_summary=config.as_dict(),
            wall_time_s=time.monotonic() - started)
    # The geometry audit is the G2 authority; timing never overrides a
    # rejected geometry candidate.
    from .audit import audit_geometry
    geometry_audit = audit_geometry(
        request, path, config.geometry_audit_config,
        planning_context=planning_context)
    if not geometry_audit.passed:
        return TimingResult(
            status="GEOMETRY_REJECTED", reason=geometry_audit.reason,
            detail=geometry_audit.detail,
            request_summary=request_summary_value,
            request_sha256=request_sha,
            config_summary=config.as_dict(),
            geometry_audit=geometry_audit,
            wall_time_s=time.monotonic() - started,
            metrics={"geometry_audit": geometry_audit.metrics})
    from .trajectory import VirtualATimedPlan
    try:
        if path.is_hold:
            if request.start_qf != request.goal_qf:
                return TimingResult(
                    status="INVALID_PATH", reason="HOLD_GOAL_MISMATCH",
                    detail="NO_MOTION requires an exactly identical goal",
                    request_summary=request_summary_value,
                    request_sha256=request_sha,
                    config_summary=config.as_dict(),
                    geometry_audit=geometry_audit,
                    wall_time_s=time.monotonic() - started)
            plan = VirtualATimedPlan._build_hold(
                request, path, planning_context=planning_context)
            return TimingResult(
                status="NO_MOTION",
                plan=plan,
                request_summary=request_summary_value,
                request_sha256=request_sha,
                config_summary=config.as_dict(),
                geometry_audit=geometry_audit,
                duration_s=0.0,
                total_S_m=0.0,
                T_s=0.0,
                bounds_summary={"kind": "hold", "S": 0.0, "T": 0.0},
                wall_time_s=time.monotonic() - started,
                metrics={"hold": True})
        profile = path.profile
        if profile is None:
            return TimingResult(
                status="INVALID_PATH", reason="MOTION_PROFILE_MISSING",
                detail="motion path has no curvature profile",
                request_summary=request_summary_value,
                request_sha256=request_sha,
                config_summary=config.as_dict(),
                geometry_audit=geometry_audit,
                wall_time_s=time.monotonic() - started)
        total_s = float(path.length_m)
        selection = _selection_bounds(profile, request, config)
        duration = selection["selected_duration_s"]
        if duration > float(config.max_total_duration_s):
            return TimingResult(
                status="TIMING_BUDGET_EXCEEDED",
                reason="TIMING_BUDGET_EXCEEDED",
                detail="selected duration {!r} s exceeds {!r} s".format(
                    duration, config.max_total_duration_s),
                request_summary=request_summary_value,
                request_sha256=request_sha,
                config_summary=config.as_dict(),
                geometry_audit=geometry_audit,
                duration_s=duration, total_S_m=total_s, T_s=duration,
                bounds_summary=selection,
                wall_time_s=time.monotonic() - started)
        rescalings = 0
        certificate = None
        while True:
            try:
                certificate = _certify_bounds(
                    profile, request, duration, config)
                break
            except _BoundFailure as exc:
                if exc.reason == "DYNAMIC_LIMIT" and \
                        rescalings < int(config.max_rescalings):
                    duration *= float(config.rescale_factor)
                    rescalings += 1
                    if duration > float(config.max_total_duration_s):
                        return TimingResult(
                            status="TIMING_BUDGET_EXCEEDED",
                            reason="TIMING_BUDGET_EXCEEDED",
                            detail=exc.detail,
                            request_summary=request_summary_value,
                            request_sha256=request_sha,
                            config_summary=config.as_dict(),
                            geometry_audit=geometry_audit,
                            duration_s=duration, total_S_m=total_s,
                            T_s=duration, rescalings=rescalings,
                            bounds_summary=selection,
                            wall_time_s=time.monotonic() - started)
                    continue
                return TimingResult(
                    status="BOUND_UNCONFIRMED", reason=exc.reason,
                    detail=exc.detail,
                    request_summary=request_summary_value,
                    request_sha256=request_sha,
                    config_summary=config.as_dict(),
                    geometry_audit=geometry_audit,
                    duration_s=duration, total_S_m=total_s, T_s=duration,
                    rescalings=rescalings, bounds_summary=selection,
                    wall_time_s=time.monotonic() - started,
                    metrics={"bound_failure": exc.location})
        selection["selected_duration_s"] = duration
        selection["rescalings"] = rescalings
        plan = VirtualATimedPlan._build_motion(
            request, path, duration, selection,
            planning_context=planning_context)
        return TimingResult(
            status="DYNAMIC_FEASIBLE",
            plan=plan,
            request_summary=request_summary_value,
            request_sha256=request_sha,
            config_summary=config.as_dict(),
            geometry_audit=geometry_audit,
            duration_s=duration,
            total_S_m=total_s,
            T_s=duration,
            rescalings=rescalings,
            bounds_summary=selection,
            wall_time_s=time.monotonic() - started,
            metrics={
                "retimer_certificate": certificate,
                "worst_quantity": selection["worst"]["quantity"],
                "worst_vehicle_id": selection["worst"]["vehicle_id"],
                "worst_order": selection["worst"]["order"],
            })
    except VirtualAContractError as exc:
        return TimingResult(
            status="NUMERICAL_FAILURE", reason=exc.reason,
            detail=exc.detail,
            request_summary=request_summary_value,
            request_sha256=request_sha,
            config_summary=config.as_dict(),
            geometry_audit=geometry_audit,
            wall_time_s=time.monotonic() - started,
            metrics={"location": exc.location})
    except Exception as exc:
        return TimingResult(
            status="NUMERICAL_FAILURE", reason="NUMERICAL_FAILURE",
            detail="{}: {}".format(type(exc).__name__, exc),
            request_summary=request_summary_value,
            request_sha256=request_sha,
            config_summary=config.as_dict(),
            geometry_audit=geometry_audit,
            wall_time_s=time.monotonic() - started)