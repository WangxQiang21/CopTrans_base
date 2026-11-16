"""Exact per-panel fourth-derivative bounds for the independent Simpson audit.

The G1 production path may keep its global triangle bound.  The independent
geometry audit, however, must confirm its 5e-8 m budget.  A global bound such
as ``G + 4KU + 3E^2 + 6K^2E + K^4`` loses too much on the long S2 profile:
the panel grid can no longer certify the stricter audit threshold even though
the actual fourth derivative is substantially smaller.

This module implements the ``exp(i psi(s))`` fourth derivative analytically.
With ``f = exp(i psi)`` and

    k = psi'      e = psi''      u = psi'''      g = psi''''

the logarithmic derivative is

    f''''/f = (k^4 - 4*k*u - 3*e^2) + i*(g - 6*k^2*e).

Because ``|f| = 1``, ``|f''''| = sqrt(Re^2 + Im^2) <= sqrt(Re_max^2 +
Im_max^2)``.  The right-hand side is evaluated per composite-Simpson panel
with outward-rounded interval polynomial arithmetic, not by sampling.  A
panel bound is therefore valid for every real station in that panel, and the
maximum over panels is used in the same Simpson remainder
``H^5*M4/2880`` as G1.  The floating/angle guard and the production G1
``1e-7 m`` contract are untouched; the audit remains fail-closed.

The cubic coefficients are floats.  As in :mod:`bounds`, each coefficient is
widened by ``COEFFICIENT_ULPS`` before interval arithmetic, and the panel
endpoints are widened to cover the rounded float interpolation nodes used by
the audit integrator.  These are documented floating-point assumptions, not a
formal proof of the libm or IEEE-754 implementation.
"""
import math

from . import bounds


class PanelBoundError(ValueError):
    """Structured failure for a non-finite or invalid panel bound."""


def _finite(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise PanelBoundError(
            "{} is not a finite real: {!r}".format(label, value)) from exc
    if not math.isfinite(result):
        raise PanelBoundError(
            "{} is non-finite: {!r}".format(label, result))
    return result


def _coefficient_interval(value):
    """A widened interval containing the real coefficient ``value``."""
    return bounds.widen(_finite(value, "coefficient"))


def _poly_scale(poly, scalar):
    return [bounds.iscale(coefficient, scalar) for coefficient in poly]


def _poly_add(first, second):
    size = max(len(first), len(second))
    zero = (0.0, 0.0)
    result = []
    for index in range(size):
        left = first[index] if index < len(first) else zero
        right = second[index] if index < len(second) else zero
        result.append(bounds.iadd(left, right))
    while len(result) > 1 and result[-1] == zero:
        result.pop()
    return result


def _poly_mul(first, second):
    result = [(0.0, 0.0)] * (len(first) + len(second) - 1)
    for left_index, left in enumerate(first):
        for right_index, right in enumerate(second):
            result[left_index + right_index] = bounds.iadd(
                result[left_index + right_index],
                bounds.imul(left, right))
    return result


def _poly_interval(poly, q0, q1):
    """Interval Horner enclosure of an interval-coefficient polynomial."""
    _finite(q0, "panel_lower_q")
    _finite(q1, "panel_upper_q")
    if q0 > q1:
        raise PanelBoundError("inverted panel interval")
    q_interval = (q0, q1)
    result = poly[-1]
    for coefficient in reversed(poly[:-1]):
        result = bounds.iadd(
            bounds.imul(result, q_interval), coefficient)
    return result


def _coefficient_polynomials(c0, c1, c2, c3, h):
    """Interval coefficient polynomials for k, e, u and g.

    The normalized piece parameter is ``q in [0, 1]``, and ``s = s0 + h*q``:

        k(q) = c0 + c1*q + c2*q^2 + c3*q^3
        e(q) = (c1 + 2*c2*q + 3*c3*q^2) / h
        u(q) = (2*c2 + 6*c3*q) / h^2
        g(q) = 6*c3 / h^3
    """
    length = _finite(h, "piece_length")
    if length <= 0.0:
        raise PanelBoundError("piece length must be positive")
    k = [_coefficient_interval(c0), _coefficient_interval(c1),
         _coefficient_interval(c2), _coefficient_interval(c3)]
    e_raw = [
        _coefficient_interval(c1),
        _coefficient_interval(2.0 * c2),
        _coefficient_interval(3.0 * c3),
    ]
    u_raw = [
        _coefficient_interval(2.0 * c2),
        _coefficient_interval(6.0 * c3),
    ]
    e = [bounds.idiv_positive(value, length) for value in e_raw]
    u = [bounds.idiv_positive(value, length * length)
         for value in u_raw]
    g = [bounds.idiv_positive(_coefficient_interval(6.0 * c3),
                              length * length * length)]
    return {"k": k, "e": e, "u": u, "g": g}


def _m4_polynomials(c0, c1, c2, c3, h):
    """Interval coefficient polynomials for Re and Im of ``f''''/f``.

    These depend only on the piece, not on the Simpson panel, so a segment
    evaluates them once and reuses them for every panel and every refinement.
    """
    jets = _coefficient_polynomials(c0, c1, c2, c3, h)
    k = jets["k"]
    e = jets["e"]
    u = jets["u"]
    g = jets["g"]
    k2 = _poly_mul(k, k)
    k4 = _poly_mul(k2, k2)
    e2 = _poly_mul(e, e)
    ku = _poly_mul(k, u)
    k2e = _poly_mul(k2, e)
    real = _poly_add(
        _poly_add(_poly_scale(ku, -4.0), _poly_scale(e2, -3.0)), k4)
    imaginary = _poly_add(g, _poly_scale(k2e, -6.0))
    return {"real": real, "imaginary": imaginary}


def _panel_m4_from_polynomials(polynomials, q0, q1):
    real_interval = _poly_interval(polynomials["real"], q0, q1)
    imaginary_interval = _poly_interval(polynomials["imaginary"], q0, q1)
    real_max = bounds.imax_abs(real_interval)
    imaginary_max = bounds.imax_abs(imaginary_interval)
    real_square = bounds.imul((real_max, real_max),
                              (real_max, real_max))[1]
    imaginary_square = bounds.imul(
        (imaginary_max, imaginary_max),
        (imaginary_max, imaginary_max))[1]
    magnitude_square = bounds.iadd((real_square, real_square),
                                   (imaginary_square, imaginary_square))
    magnitude = bounds.isqrt_positive(magnitude_square)[1]
    return _finite(magnitude, "panel_m4")


def _panel_m4(c0, c1, c2, c3, h, q0, q1):
    """Outward upper bound of ``|f''''|`` on ``q in [q0, q1]``."""
    return _panel_m4_from_polynomials(
        _m4_polynomials(c0, c1, c2, c3, h), q0, q1)


def _widened_panel(q0, q1):
    """Panel interval widened to cover rounded float interpolation nodes."""
    lower = max(0.0, bounds.widen(q0)[0])
    upper = min(1.0, bounds.widen(q1)[1])
    if lower > upper:
        raise PanelBoundError("widened panel is inverted")
    return lower, upper


def panel_m4_bounds(c0, c1, c2, c3, h, subintervals):
    """One exact M4 bound for each composite-Simpson panel.

    ``subintervals`` is the even number of equal production Simpson
    subintervals; a composite-Simpson panel spans two subintervals.  The
    returned list therefore has ``subintervals / 2`` entries.  Panel endpoints
    are widened outward, so the bounds also cover the rounded nodes formed by
    the audit's float interpolation.
    """
    if isinstance(subintervals, bool) or not isinstance(subintervals, int):
        raise PanelBoundError("subintervals must be an even integer")
    if subintervals < 2 or subintervals % 2:
        raise PanelBoundError("subintervals must be even and >= 2")
    count = subintervals // 2
    polynomials = _m4_polynomials(c0, c1, c2, c3, h)
    result = []
    for index in range(count):
        q0 = 0.0 if index == 0 else 2.0 * index / subintervals
        q1 = 1.0 if index + 1 == count else 2.0 * (index + 1) / subintervals
        widened = _widened_panel(q0, q1)
        result.append(_panel_m4_from_polynomials(
            polynomials, widened[0], widened[1]))
    return result


def _sum_panel_remainders(width, panel_m4s):
    total = 0.0
    for m4 in panel_m4s:
        total = bounds._next_up(
            total + bounds.simpson_panel_remainder(width, m4))
    return total


def segment_budget(h, subintervals, panel_m4s, local_panels=1):
    """Simpson remainder of one piece using exact per-panel M4 bounds.

    The full piece contains ``subintervals / 2`` complete Simpson panels, each
    with its own M4.  A prefix query inside the piece is integrated with
    complete panels plus at most one local panel; the local panel may lie in
    any complete panel, so the maximum panel M4 is used for that one local
    term.  This is the same conservative local-panel accounting as G1 and
    does not reduce any protection coefficient.
    """
    if isinstance(subintervals, bool) or not isinstance(subintervals, int):
        raise PanelBoundError("subintervals must be an even integer")
    if subintervals < 2 or subintervals % 2:
        raise PanelBoundError("subintervals must be even and >= 2")
    if not isinstance(local_panels, int) or local_panels < 0 or \
            local_panels > 1:
        raise PanelBoundError("at most one local panel is allowed")
    length = _finite(h, "piece_length")
    if length <= 0.0:
        raise PanelBoundError("piece length must be positive")
    if len(panel_m4s) != subintervals // 2:
        raise PanelBoundError(
            "expected {} panel bounds, got {}".format(
                subintervals // 2, len(panel_m4s)))
    values = [_finite(value, "panel_m4") for value in panel_m4s]
    if any(value < 0.0 for value in values):
        raise PanelBoundError("negative panel M4")
    width = bounds._next_up(2.0 * length / subintervals)
    full = _sum_panel_remainders(width, values)
    maximum = max(values)
    local = bounds._next_up(
        local_panels * bounds.simpson_panel_remainder(width, maximum))
    return {
        "subintervals": subintervals,
        "subinterval_width_m": length / subintervals,
        "full_panel_count": subintervals // 2,
        "full_panel_width_m": width,
        "full_remainder_m": full,
        "local_panel_count": local_panels,
        "local_panel_width_m": width,
        "local_remainder_m": local,
        "maximum_panel_m4_upper_m": maximum,
        "segment_remainder_m": bounds._next_up(full + local),
    }


def position_budget(records):
    """Euclidean budget from exact per-panel segment categories.

    ``records`` is an iterable of ``(h, subintervals, panel_m4s)``.  The
    per-coordinate remainders are added with outward rounding; the two
    coordinates are then combined by ``sqrt(2)``, exactly as G1.
    """
    per_coordinate = 0.0
    total_subintervals = 0
    total_full_panels = 0
    total_local_panels = 0
    segment_budgets = []
    for h, subintervals, panel_m4s in records:
        record = segment_budget(h, subintervals, panel_m4s)
        per_coordinate = bounds._next_up(
            per_coordinate + record["segment_remainder_m"])
        total_subintervals += record["subintervals"]
        total_full_panels += record["full_panel_count"]
        total_local_panels += record["local_panel_count"]
        segment_budgets.append(record)
    euclidean = bounds._next_up(bounds._next_up(math.sqrt(2.0)) *
                                per_coordinate)
    return {
        "per_coordinate_m": per_coordinate,
        "euclidean_m": euclidean,
        "total_subintervals": total_subintervals,
        "full_panels": total_full_panels,
        "local_panels": total_local_panels,
        "segment_budgets": segment_budgets,
    }
