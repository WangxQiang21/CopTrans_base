"""Rigorous interval and integration-remainder operators for G1.

Only the operators this curvature model needs are implemented: outward-rounded
interval arithmetic for normalized cubics and their derivatives, the fourth
derivative envelope of ``exp(i psi)`` and the composite-Simpson remainder.

Outward rounding uses the IEEE-754 successor/predecessor of each result.
Coefficient construction from knot values is float arithmetic; its rounding is
covered by an explicit per-coefficient widening (``COEFFICIENT_ULPS``).  This
is a documented numerical assumption, not a formal floating-point proof.
"""
import math
import struct

EPSILON = 2.220446049250313e-16
COEFFICIENT_ULPS = 16
POSITION_BUDGET_M = 1e-7
ANGLE_BUDGET_RAD = 1e-9
POSITION_ROUNDOFF_GUARD_M = 1e-10
ANGLE_ROUNDOFF_GUARD_RAD = 1e-11
MAX_PANELS = 200000
NUMERICS_VERSION = "virtual_rigid_a.numerics.g1.v2"


def _next_up(value):
    if math.isnan(value) or math.isinf(value):
        return value
    if value == 0.0:
        return 5e-324
    bits = struct.unpack("<Q", struct.pack("<d", value))[0]
    bits = bits + 1 if value > 0.0 else bits - 1
    return struct.unpack("<d", struct.pack("<Q", bits))[0]


def _next_down(value):
    if math.isnan(value) or math.isinf(value):
        return value
    if value == 0.0:
        return -5e-324
    bits = struct.unpack("<Q", struct.pack("<d", value))[0]
    bits = bits - 1 if value > 0.0 else bits + 1
    return struct.unpack("<d", struct.pack("<Q", bits))[0]


def widen(value, ulps=COEFFICIENT_ULPS):
    """Outward interval covering the float rounding of ``value``."""
    magnitude = ulps * EPSILON * (abs(value) + 1.0)
    return (_next_down(value - magnitude), _next_up(value + magnitude))


def _check_interval(interval):
    lower, upper = interval
    if not math.isfinite(lower) or not math.isfinite(upper):
        raise ValueError("nonfinite_interval")
    if lower > upper:
        raise ValueError("inverted_interval")
    return lower, upper


def iadd(first, second):
    first, second = _check_interval(first), _check_interval(second)
    return (_next_down(first[0] + second[0]),
            _next_up(first[1] + second[1]))


def isub(first, second):
    first, second = _check_interval(first), _check_interval(second)
    return (_next_down(first[0] - second[1]),
            _next_up(first[1] - second[0]))


def imul(first, second):
    first, second = _check_interval(first), _check_interval(second)
    products = (
        first[0] * second[0], first[0] * second[1],
        first[1] * second[0], first[1] * second[1])
    return (_next_down(min(products)), _next_up(max(products)))


def iscale(interval, scalar):
    """Multiply by a finite scalar with outward rounding both directions."""
    lower, upper = _check_interval(interval)
    if not math.isfinite(scalar):
        raise ValueError("nonfinite_scalar")
    if scalar >= 0.0:
        return (_next_down(lower * scalar), _next_up(upper * scalar))
    return (_next_down(upper * scalar), _next_up(lower * scalar))


def idiv_positive(interval, divisor):
    """Divide by a positive finite scalar, rounding outward."""
    lower, upper = _check_interval(interval)
    if not math.isfinite(divisor) or divisor <= 0.0:
        raise ValueError("nonpositive_divisor")
    return (_next_down(lower / divisor), _next_up(upper / divisor))


def interval_from_coefficients(coefficients, q0=0.0, q1=1.0):
    """Interval Horner bound of a scalar polynomial on ``[q0, q1]``."""
    coefficients = tuple(float(value) for value in coefficients)
    if not coefficients:
        raise ValueError("empty_polynomial")
    if not math.isfinite(q0) or not math.isfinite(q1) or q0 > q1:
        raise ValueError("invalid_parameter_interval")
    result = (coefficients[-1], coefficients[-1])
    q_interval = (q0, q1)
    for coefficient in reversed(coefficients[:-1]):
        result = iadd(imul(result, q_interval), (coefficient, coefficient))
    return result


def interval_from_widened_coefficients(coefficients, q0=0.0, q1=1.0):
    """Interval Horner bound with each coefficient widened for rounding."""
    widened = tuple(widen(float(value)) for value in coefficients)
    if not widened:
        raise ValueError("empty_polynomial")
    if not math.isfinite(q0) or not math.isfinite(q1) or q0 > q1:
        raise ValueError("invalid_parameter_interval")
    result = widened[-1]
    q_interval = (q0, q1)
    for coefficient in reversed(widened[:-1]):
        result = iadd(imul(result, q_interval), coefficient)
    return result


def segment_jet_bounds(c0, c1, c2, c3, h, q0=0.0, q1=1.0):
    """Rigorous k/eta/u/g intervals on one normalized cubic piece.

    ``q0``/``q1`` are normalized positions inside the piece, ``0 <= q0 <= q1
    <= 1``.  ``h`` is the positive piece length in metres.
    """
    if not math.isfinite(h) or h <= 0.0:
        raise ValueError("nonpositive_segment_length")
    if not (0.0 <= q0 <= q1 <= 1.0):
        raise ValueError("segment_parameter_out_of_range")
    k = interval_from_widened_coefficients((c0, c1, c2, c3), q0, q1)
    eta = idiv_positive(interval_from_widened_coefficients(
        (c1, 2.0 * c2, 3.0 * c3), q0, q1), h)
    u = idiv_positive(interval_from_widened_coefficients(
        (2.0 * c2, 6.0 * c3), q0, q1), h * h)
    g = idiv_positive(widen(6.0 * c3), h * h * h)
    return {"k": k, "eta": eta, "u": u, "g": g}


def m4_upper(k_upper, eta_upper, u_upper, g_upper):
    """Upper bound of ``G + 4KU + 3E^2 + 6K^2E + K^4``.

    Inputs are non-negative upper bounds of ``|k|/|eta|/|u|/|g|`` on a panel
    that does not cross a Hermite node.  Every product and sum rounds upward.
    """
    for name, value in (("k", k_upper), ("eta", eta_upper),
                        ("u", u_upper), ("g", g_upper)):
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("invalid_{}_upper".format(name))
    total = g_upper
    total = _next_up(total + _next_up(_next_up(4.0 * k_upper) * u_upper))
    squared_eta = _next_up(_next_up(eta_upper * eta_upper) * 3.0)
    total = _next_up(total + _next_up(squared_eta))
    squared_k = _next_up(k_upper * k_upper)
    total = _next_up(total + _next_up(_next_up(6.0 * squared_k) * eta_upper))
    total = _next_up(total + _next_up(squared_k * squared_k))
    return total


def simpson_panel_remainder(panel_width, m4):
    """Per-coordinate remainder of one Simpson panel: ``H^5 M4 / 2880``."""
    if not math.isfinite(panel_width) or panel_width < 0.0:
        raise ValueError("invalid_panel_width")
    if not math.isfinite(m4) or m4 < 0.0:
        raise ValueError("invalid_m4_bound")
    fifth = _next_up(_next_up(_next_up(_next_up(
        panel_width * panel_width) * panel_width) * panel_width) * panel_width)
    return _next_up(_next_up(fifth * m4) / 2880.0)


def simpson_segment_budget(h, panels, m4, local_panels=1):
    """One segment's Simpson remainder with the widths actually integrated.

    The production grid has ``panels`` equal subintervals of width
    ``d = h / panels``.  A composite-Simpson *panel* therefore spans ``2*d``,
    and a complete segment contains exactly ``panels / 2`` such panels.  A
    query inside the segment uses an even number of complete panels plus at
    most one local Simpson panel: if ``floor(local/d)`` is odd, the prefix is
    shortened to the preceding even node, so the remaining width is strictly
    less than ``2*d``.  The local term is therefore bounded by one panel of
    width ``2*d``.  Adding it once per segment is conservative for the
    per-path budget even though a single query lies in only one segment.

    No sampled maximum and no ``d``-wide panel is used here.
    """
    if not math.isfinite(h) or h <= 0.0:
        raise ValueError("nonpositive_segment_length")
    if not isinstance(panels, int) or panels < 2 or panels % 2:
        raise ValueError("simpson_panels_must_be_even")
    if not isinstance(local_panels, int) or local_panels < 0 or local_panels > 1:
        raise ValueError("at_most_one_local_panel")
    d = h / panels
    # The width of one full Simpson panel is 2*d.  Widen it outward because
    # the grid station and the final local station are formed by float
    # additions; this is the numerical-width assumption, not a sampled value.
    width = _next_up(2.0 * d)
    full_count = panels // 2
    full = _next_up(full_count * simpson_panel_remainder(width, m4))
    local = _next_up(local_panels * simpson_panel_remainder(width, m4))
    return {
        "subintervals": panels,
        "subinterval_width_m": d,
        "full_panel_count": full_count,
        "full_panel_width_m": width,
        "full_remainder_m": full,
        "local_panel_count": local_panels,
        "local_panel_width_m": width,
        "local_remainder_m": local,
        "segment_remainder_m": _next_up(full + local),
    }


def position_budget(records):
    """Conservative per-coordinate and Euclidean Simpson budgets.

    ``records`` is an iterable of ``(h, panels, m4)``.  ``panels`` is the
    number of equal *subintervals* in the production Simpson grid, not the
    number of Simpson panels.  Each segment contributes ``panels/2`` complete
    panels of width ``2*h/panels`` and one local-query panel of the same width.
    The terms are added across every segment, then the two coordinate budgets
    are combined in quadrature.
    """
    per_coordinate = 0.0
    total_subintervals = 0
    total_full_panels = 0
    total_local_panels = 0
    segment_budgets = []
    for h, panels, m4 in records:
        record = simpson_segment_budget(h, panels, m4)
        per_coordinate = _next_up(
            per_coordinate + record["segment_remainder_m"])
        total_subintervals += record["subintervals"]
        total_full_panels += record["full_panel_count"]
        total_local_panels += record["local_panel_count"]
        segment_budgets.append(record)
    euclidean = _next_up(_next_up(math.sqrt(2.0)) * per_coordinate)
    return {
        "per_coordinate_m": per_coordinate,
        "euclidean_m": euclidean,
        "total_panels": total_subintervals,
        "full_panels": total_full_panels,
        "local_panels": total_local_panels,
        "segment_budgets": segment_budgets,
    }


def position_guard(length_m, simpson_panels, angle_bound_rad):
    """Floating-point and angle-error protection added to the remainder.

    ``simpson_panels`` counts the actual Simpson accumulation panels
    (``panels/2`` per complete segment plus the one local-query panel per
    segment), not the number of equal subintervals.  The per-panel guard is a
    documented numerical assumption covering libm/trigonometric evaluation
    and float accumulation; it is not a formal floating-point proof.
    """
    if not isinstance(simpson_panels, int) or simpson_panels < 0:
        raise ValueError("invalid_guard_panel_count")
    if length_m < 0.0:
        raise ValueError("negative_guard_input")
    roundoff = POSITION_ROUNDOFF_GUARD_M * (simpson_panels + 8)
    angle = _next_up(length_m * angle_bound_rad)
    return _next_up(roundoff + angle)


def angle_budget(segment_count, psi_span_abs):
    """Conservative angle error budget for exact polynomial integration."""
    if segment_count < 1 or psi_span_abs < 0.0:
        raise ValueError("invalid_angle_budget_input")
    guard = ANGLE_ROUNDOFF_GUARD_RAD * (segment_count + 8)
    rounding = _next_up(8.0 * EPSILON * (1.0 + psi_span_abs))
    return _next_up(guard + rounding)


def iabs_interval(interval):
    """Outward interval enclosure of ``|x|`` for ``x`` in ``interval``."""
    lower, upper = _check_interval(interval)
    if lower >= 0.0:
        return (lower, upper)
    if upper <= 0.0:
        return (_next_down(-upper), _next_up(-lower))
    magnitude = max(-lower, upper)
    return (0.0, _next_up(magnitude))


def imax_abs(interval):
    """Upper bound of ``max(|x|)`` on a finite interval."""
    lower, upper = _check_interval(interval)
    return _next_up(max(abs(lower), abs(upper)))


def isquare(interval):
    """Outward interval enclosure of ``x*x`` (signs handled exactly)."""
    lower, upper = _check_interval(interval)
    if lower >= 0.0:
        return (_next_down(lower * lower), _next_up(upper * upper))
    if upper <= 0.0:
        return (_next_down(upper * upper), _next_up(lower * lower))
    magnitude = max(-lower, upper)
    return (0.0, _next_up(magnitude * magnitude))


def ipow_nonnegative(interval, exponent):
    """Outward enclosure of ``x**exponent`` for an interval ``x >= 0``."""
    lower, upper = _check_interval(interval)
    if isinstance(exponent, bool) or not isinstance(exponent, int):
        raise ValueError("invalid_integer_exponent")
    if exponent < 0:
        raise ValueError("negative_exponent")
    if lower < 0.0:
        raise ValueError("negative_power_base")
    if exponent == 0:
        return (1.0, 1.0)
    return (_next_down(lower ** exponent), _next_up(upper ** exponent))


def isqrt_positive(interval):
    """Outward square root of a non-negative finite interval."""
    lower, upper = _check_interval(interval)
    if lower < 0.0:
        raise ValueError("negative_sqrt_interval")
    low = 0.0 if lower == 0.0 else _next_down(math.sqrt(lower))
    high = 0.0 if upper == 0.0 else _next_up(math.sqrt(upper))
    if low > high:
        low, high = high, low
    return (low, high)


def idiv_positive_interval(numerator, denominator):
    """Divide two intervals; the denominator must be strictly positive."""
    num = _check_interval(numerator)
    den = _check_interval(denominator)
    if den[0] <= 0.0:
        raise ValueError("nonpositive_interval_divisor")
    quotients = (
        num[0] / den[0], num[0] / den[1],
        num[1] / den[0], num[1] / den[1])
    return (_next_down(min(quotients)), _next_up(max(quotients)))


def inorm2_upper(components):
    """Upper bound of ``sqrt(sum(x_i^2))`` from component intervals."""
    total = 0.0
    for component in components:
        magnitude = imax_abs(component)
        total = _next_up(total + _next_up(magnitude * magnitude))
    return _next_up(math.sqrt(total))