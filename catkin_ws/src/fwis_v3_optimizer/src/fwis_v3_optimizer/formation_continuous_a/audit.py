"""Independent G2 geometry audit for the A-only virtual formation.

The audit deliberately does not call the optimiser's residual evaluator.  It
rebuilds the Hermite coefficients from the raw knots, integrates the endpoint
with an independent Gauss-Legendre rule, and certifies the whole curvature
domain with outward-rounded interval arithmetic.  A solver's success message is
never treated as a geometry certificate.
"""
from collections.abc import Mapping as MappingABC
from dataclasses import dataclass
import hashlib
import json
import math

from ..continuous_geometry import ControlKnot
from ..formation import FormationPose
from . import bounds, integration_bounds
from .contracts import (
    BACKEND,
    COLLISION_SCOPE,
    DEFAULT_GEOMETRY_HEADING_TOL_RAD,
    DEFAULT_GEOMETRY_POSITION_TOL_M,
    DEFAULT_GEOMETRY_RIGIDITY_TOL_M,
    DYNAMICS_SCOPE,
    G4_PLANNING_SCHEMA,
    INITIAL_CURVATURE_TOLERANCE,
    INITIAL_HEADING_TOLERANCE_RAD,
    PATH_MODEL_VERSION,
    TIMED_MODEL_VERSION,
    TIMING_LAW_VERSION,
    AuditReport,
    DynamicAuditConfig,
    GeometryAuditConfig,
    ValidationResult,
    VirtualARequest,
    VirtualAContractError,
    _finite,
    _pose_summary,
    coerce_goal_policy,
    geometry_path_summary,
    grid_summary_sha256,
    planning_context_sha256,
    request_fingerprint,
    request_payload,
    request_summary,
    validate_dynamic_audit_config,
    validate_geometry_audit_config,
    validate_virtual_a_request,
)
from .geometry import VirtualAPath, _wrap, validate_initial_state


_INTEGRATION_METHOD = "independent_composite_simpson_v2_exact_panel"
_AUDIT_REFINEMENT_LIMIT = 64


class _AuditFailure(Exception):
    def __init__(self, reason, detail="", **location):
        self.reason = str(reason)
        self.detail = str(detail)
        self.location = dict(location)
        super().__init__(self.reason)


@dataclass(frozen=True)
class _Piece:
    s0: float
    s1: float
    h: float
    c0: float
    c1: float
    c2: float
    c3: float

    def value(self, q):
        return self.c0 + q * (self.c1 + q * (self.c2 + q * self.c3))

    def slope(self, q):
        return (self.c1 + q * (2.0 * self.c2 + 3.0 * q * self.c3)) / self.h

    def second(self, q):
        return (2.0 * self.c2 + 6.0 * q * self.c3) / (self.h * self.h)

    def third(self):
        return 6.0 * self.c3 / (self.h * self.h * self.h)

    def psi_increment(self, q):
        return self.h * q * (
            self.c0 + q * (
                self.c1 / 2.0 + q * (
                    self.c2 / 3.0 + q * self.c3 / 4.0)))


def _audit_to_local(vector, psi):
    """Rotate a world vector into the formation tangent/normal frame."""
    cosine, sine = math.cos(psi), math.sin(psi)
    return (cosine * vector[0] + sine * vector[1],
            -sine * vector[0] + cosine * vector[1])


def _wrap_abs(value):
    return abs(_wrap(value))


def _knot_value(knot, name):
    try:
        value = getattr(knot, name)
    except AttributeError:
        try:
            # Some serialised records use the descriptive aliases.
            if name == "value":
                value = getattr(knot, "kappa")
            elif name == "derivative":
                value = getattr(knot, "eta")
            else:
                raise
        except AttributeError as exc:
            raise _AuditFailure(
                "INVALID_PROFILE",
                "knot lacks {!r} field".format(name)) from exc
    return _finite(value, "knot." + name)


def _independent_pieces(knots):
    supplied = tuple(knots)
    if len(supplied) < 2:
        raise _AuditFailure(
            "INVALID_PROFILE", "at least two knots are required")
    stations = []
    values = []
    derivatives = []
    for index, knot in enumerate(supplied):
        if not isinstance(knot, ControlKnot):
            # Accept a structurally identical immutable record only for the
            # existing serialised evidence path.  A synthetic profile still
            # has to pass every numerical check below.
            if not all(hasattr(knot, name) for name in ("s", "value",
                                                        "derivative")):
                raise _AuditFailure(
                    "INVALID_PROFILE",
                    "knot index={} lacks s/value/derivative".format(index),
                    interval=index)
        s = _finite(getattr(knot, "s"), "knot.s")
        stations.append(s)
        values.append(_knot_value(knot, "value"))
        derivatives.append(_knot_value(knot, "derivative"))
    if stations[0] != 0.0:
        raise _AuditFailure(
            "INVALID_PROFILE_DOMAIN",
            "first knot must start at s=0; got {!r}".format(stations[0]),
            s=stations[0])
    pieces = []
    for index in range(len(supplied) - 1):
        s0, s1 = stations[index], stations[index + 1]
        h = s1 - s0
        if not math.isfinite(h) or h <= 0.0:
            raise _AuditFailure(
                "NON_POSITIVE_SEGMENT",
                "segment {} h={!r}".format(index, h),
                interval=index, h=h, s=s0)
        k0, k1 = values[index], values[index + 1]
        eta0, eta1 = derivatives[index], derivatives[index + 1]
        # Independent transcription of the G1 Hermite definition.  The audit
        # intentionally does not call C2CurvatureProfile._hermite_coefficients.
        c0 = k0
        c1 = h * eta0
        c2 = 3.0 * (k1 - k0) - h * (2.0 * eta0 + eta1)
        c3 = 2.0 * (k0 - k1) + h * (eta0 + eta1)
        for value in (c0, c1, c2, c3):
            if not math.isfinite(value):
                raise _AuditFailure(
                    "NONFINITE_COEFFICIENT",
                    "segment {} coefficient is non-finite".format(index),
                    interval=index)
        pieces.append(_Piece(s0, s1, h, c0, c1, c2, c3))
    return tuple(pieces), tuple(stations), tuple(values), tuple(derivatives)


def _node_c2_audit(pieces, config):
    """Recompute C0/C1/C2 from raw coefficients and record the residuals."""
    records = []
    max_value = 0.0
    max_slope = 0.0
    max_second = 0.0
    max_g = 0.0
    for index, piece in enumerate(pieces):
        g = piece.third()
        max_g = max(max_g, abs(g))
        if not math.isfinite(g) or abs(g) > config.c2_third_abs_limit:
            raise _AuditFailure(
                "UNBOUNDED_CURVATURE_THIRD",
                "segment {} |g|={!r} exceeds {!r}".format(
                    index, abs(g), config.c2_third_abs_limit),
                interval=index, constraint="G_BOUND", g=g,
                limit=config.c2_third_abs_limit)
        if index == 0:
            continue
        previous = pieces[index - 1]
        value_error = _finite_audit(
            abs(previous.value(1.0) - piece.value(0.0)),
            "c2_value_error", interval=index)
        slope_error = _finite_audit(
            abs(previous.slope(1.0) - piece.slope(0.0)),
            "c2_slope_error", interval=index)
        second_error = _finite_audit(
            abs(previous.second(1.0) - piece.second(0.0)),
            "c2_second_error", interval=index)
        max_value = max(max_value, value_error)
        max_slope = max(max_slope, slope_error)
        max_second = max(max_second, second_error)
        record = {
            "node": index,
            "s": piece.s0,
            "value_error": value_error,
            "slope_error": slope_error,
            "second_error": second_error,
            "u_left": previous.second(1.0),
            "u_right": piece.second(0.0),
        }
        records.append(record)
        if value_error > config.c2_value_tol:
            raise _AuditFailure(
                "NON_C2_PROFILE",
                "C0 discontinuity node={} error={!r}".format(
                    index, value_error),
                interval=index, s=piece.s0, node=index,
                constraint="C2_VALUE", error=value_error,
                tolerance=config.c2_value_tol)
        if slope_error > config.c2_slope_tol:
            raise _AuditFailure(
                "NON_C2_PROFILE",
                "C1 discontinuity node={} error={!r}".format(
                    index, slope_error),
                interval=index, s=piece.s0, node=index,
                constraint="C2_SLOPE", error=slope_error,
                tolerance=config.c2_slope_tol)
        if second_error > config.c2_second_tol:
            raise _AuditFailure(
                "NON_C2_PROFILE",
                "C2 discontinuity node={} error={!r}".format(
                    index, second_error),
                interval=index, s=piece.s0, node=index,
                constraint="C2_SECOND", error=second_error,
                tolerance=config.c2_second_tol)
    return {
        "node_count": max(0, len(pieces) - 1),
        "nodes": records,
        "max_value_error": max_value,
        "max_slope_error": max_slope,
        "max_second_error": max_second,
        "max_abs_g": max_g,
        "value_tolerance": config.c2_value_tol,
        "slope_tolerance": config.c2_slope_tol,
        "second_tolerance": config.c2_second_tol,
        "third_limit": config.c2_third_abs_limit,
    }


def _profile_pieces_crosscheck(path, pieces):
    """Check that the path's reported coefficients equal the rebuild."""
    profile = getattr(path, "profile", None)
    if profile is None:
        return
    try:
        reported = profile.as_dict().get("pieces")
    except Exception:
        return
    if not isinstance(reported, list) or len(reported) != len(pieces):
        return
    worst = 0.0
    for index, (record, expected) in enumerate(zip(reported, pieces)):
        coefficients = record.get("coefficients")
        if not isinstance(coefficients, list) or len(coefficients) != 4:
            continue
        for actual, wanted in zip(coefficients, (
                expected.c0, expected.c1, expected.c2, expected.c3)):
            error = _finite_audit(
                abs(float(actual) - wanted), "coefficient_error",
                interval=index)
            worst = max(worst, error)
            if error > 1e-12:
                raise _AuditFailure(
                    "PROFILE_COEFFICIENT_MISMATCH",
                    "segment {} coefficients disagree by {!r}".format(
                        index, error),
                    interval=index, constraint="PROFILE_COEFFICIENTS",
                    error=error)
    profile._audit_coefficient_crosscheck_error = worst


def _finite_audit(value, label, **location):
    """Fail closed if any computed audit quantity is not a finite real."""
    location.setdefault("constraint", "NONFINITE_AUDIT_VALUE")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise _AuditFailure(
            "NONFINITE_AUDIT_VALUE",
            "{}={!r} is not a finite real".format(label, value),
            **location) from exc
    if not math.isfinite(result):
        raise _AuditFailure(
            "NONFINITE_AUDIT_VALUE",
            "{}={!r} is non-finite".format(label, result),
            **location)
    return result


def _even_at_least(value):
    if not math.isfinite(value) or value < 0.0:
        raise _AuditFailure(
            "NONFINITE_AUDIT_VALUE",
            "invalid panel target {!r}".format(value),
            constraint="AUDIT_INTEGRATION_PLAN")
    count = max(2, int(math.ceil(value)))
    return count if count % 2 == 0 else count + 1


def _refresh_panel_bounds(record):
    """Recompute the exact per-panel fourth-derivative bounds."""
    try:
        panel_m4s = integration_bounds.panel_m4_bounds(
            record["c0"], record["c1"], record["c2"], record["c3"],
            record["h"], record["subintervals"])
    except (TypeError, ValueError, OverflowError) as exc:
        raise _AuditFailure(
            "BOUND_UNCONFIRMED",
            "unable to form exact per-panel M4 bounds: {}: {}".format(
                type(exc).__name__, exc),
            constraint="AUDIT_INTEGRATION_REMAINDER")
    for value in panel_m4s:
        _finite_audit(value, "panel_m4", constraint="AUDIT_INTEGRATION_REMAINDER")
    maximum = max(panel_m4s)
    record["panel_m4s"] = list(panel_m4s)
    record["m4"] = maximum
    record["maximum_panel_m4"] = maximum
    return record


def _integration_plan(pieces, start_psi, config):
    """Certified Simpson plan built only from the independently rebuilt knots.

    The budget is a composite-Simpson truncation remainder whose fourth
    derivative envelope is evaluated analytically per Simpson panel with
    outward interval arithmetic in :mod:`integration_bounds`, plus the
    documented G1 floating-point/angle protection terms.  It never reads the
    path's own ``position_error_bound_m``.  The coarse G1 triangle bound is
    retained in the record only as a diagnostic; it is not used in the
    budget.
    """
    base = []
    max_kappa = 0.0
    for piece in pieces:
        panels = _even_at_least(piece.h / config.audit_panel_width_m)
        try:
            jets = bounds.segment_jet_bounds(
                piece.c0, piece.c1, piece.c2, piece.c3, piece.h, 0.0, 1.0)
            k_upper = max(abs(jets["k"][0]), abs(jets["k"][1]))
            eta_upper = max(abs(jets["eta"][0]), abs(jets["eta"][1]))
            u_upper = max(abs(jets["u"][0]), abs(jets["u"][1]))
            g_upper = max(abs(jets["g"][0]), abs(jets["g"][1]))
            coarse_m4 = bounds.m4_upper(
                k_upper, eta_upper, u_upper, g_upper)
        except (TypeError, ValueError, OverflowError) as exc:
            raise _AuditFailure(
                "BOUND_UNCONFIRMED",
                "unable to form a coarse fourth-derivative bound: {}: "
                "{}".format(type(exc).__name__, exc),
                constraint="AUDIT_INTEGRATION_REMAINDER")
        for label, value in (
                ("k_upper", k_upper), ("eta_upper", eta_upper),
                ("u_upper", u_upper), ("g_upper", g_upper),
                ("coarse_m4", coarse_m4)):
            _finite_audit(value, label, constraint="AUDIT_INTEGRATION_REMAINDER")
        max_kappa = max(max_kappa, k_upper)
        record = {
            "segment_index": len(base),
            "s0": piece.s0, "s1": piece.s1, "h": piece.h,
            "c0": piece.c0, "c1": piece.c1, "c2": piece.c2,
            "c3": piece.c3,
            "subintervals": panels,
            "k_upper": k_upper, "eta_upper": eta_upper,
            "u_upper": u_upper, "g_upper": g_upper,
            "coarse_m4": coarse_m4,
        }
        _refresh_panel_bounds(record)
        base.append(record)
    psi_span = abs(_finite_audit(start_psi, "start_psi")) + sum(
        record["h"] * record["k_upper"] for record in base)
    angle_bound = bounds.angle_budget(len(pieces), psi_span)
    _finite_audit(angle_bound, "audit_angle_budget")
    if angle_bound > config.angle_budget_rad:
        raise _AuditFailure(
            "BOUND_UNCONFIRMED",
            "audit angle budget {!r} exceeds G1 angle contract {!r}".format(
                angle_bound, config.angle_budget_rad),
            constraint="AUDIT_ANGLE_BUDGET", value=angle_bound,
            limit=config.angle_budget_rad)
    threshold = config.audit_position_budget_m
    refinement_steps = 0
    while True:
        for record in base:
            if record["subintervals"] > config.audit_max_subintervals_per_segment:
                raise _AuditFailure(
                    "BOUND_UNCONFIRMED",
                    "segment {} needs {} subintervals but the audit limit is "
                    "{}".format(record["segment_index"],
                                record["subintervals"],
                                config.audit_max_subintervals_per_segment),
                    interval=record["segment_index"],
                    constraint="AUDIT_INTEGRATION_PANEL_LIMIT",
                    value=record["subintervals"],
                    limit=config.audit_max_subintervals_per_segment)
        total_subintervals = sum(
            record["subintervals"] for record in base)
        if total_subintervals > config.audit_max_total_subintervals:
            raise _AuditFailure(
                "BOUND_UNCONFIRMED",
                "audit needs {} subintervals but the total limit is {}".format(
                    total_subintervals,
                    config.audit_max_total_subintervals),
                constraint="AUDIT_INTEGRATION_TOTAL_LIMIT",
                value=total_subintervals,
                limit=config.audit_max_total_subintervals)
        remainder = integration_bounds.position_budget([
            (record["h"], record["subintervals"], record["panel_m4s"])
            for record in base])
        guard = bounds.position_guard(
            pieces[-1].s1,
            remainder["full_panels"] + remainder["local_panels"],
            angle_bound)
        _finite_audit(remainder["per_coordinate_m"], "audit_per_coordinate")
        _finite_audit(remainder["euclidean_m"], "audit_euclidean")
        _finite_audit(guard, "audit_guard")
        total_budget = bounds._next_up(remainder["euclidean_m"] + guard)
        _finite_audit(total_budget, "audit_total_budget")
        if total_budget <= threshold:
            break
        if refinement_steps >= _AUDIT_REFINEMENT_LIMIT:
            raise _AuditFailure(
                "BOUND_UNCONFIRMED",
                "audit Simpson refinement did not reach {!r}".format(
                    threshold),
                constraint="AUDIT_INTEGRATION_REMAINDER",
                value=total_budget, limit=threshold,
                refinement_steps=refinement_steps)
        worst = max(
            range(len(base)),
            key=lambda index: remainder["segment_budgets"][index][
                "segment_remainder_m"])
        # Refine the worst segment in even increments rather than doubling
        # it.  The guard grows with the total panel count, so doubling a
        # whole segment overshoots the useful allocation: the high-curvature
        # S3 planner pieces need a few extra panels, not twice as many.  The
        # Simpson remainder formula, exact per-panel M4 bound, panel limits
        # and floating/angle guard are unchanged.
        increment = max(8, 2 * ((base[worst]["subintervals"] // 8) // 2))
        candidate = base[worst]["subintervals"] + increment
        if candidate > config.audit_max_subintervals_per_segment:
            raise _AuditFailure(
                "BOUND_UNCONFIRMED",
                "refining segment {} would exceed {} subintervals".format(
                    worst, config.audit_max_subintervals_per_segment),
                interval=worst, constraint="AUDIT_INTEGRATION_PANEL_LIMIT",
                value=candidate,
                limit=config.audit_max_subintervals_per_segment)
        base[worst]["subintervals"] = candidate
        _refresh_panel_bounds(base[worst])
        refinement_steps += 1
    segments = []
    for index, record in enumerate(base):
        segment_remainder = integration_bounds.segment_budget(
            record["h"], record["subintervals"], record["panel_m4s"])
        segments.append({
            "segment_index": index,
            "s0": record["s0"], "s1": record["s1"], "h": record["h"],
            "subintervals": record["subintervals"],
            "panel_width_m": 2.0 * record["h"] / record["subintervals"],
            "panel_m4_upper_m": list(record["panel_m4s"]),
            "maximum_panel_m4_upper_m":
                record["maximum_panel_m4"],
            "coarse_triangle_m4_upper_m": record["coarse_m4"],
            "m4_method": "exact_interval_fourth_derivative_per_panel",
            "k_upper": record["k_upper"],
            "eta_upper": record["eta_upper"],
            "u_upper": record["u_upper"],
            "g_upper": record["g_upper"],
            "m4_upper": record["m4"],
            "remainder_m": segment_remainder["segment_remainder_m"],
            "full_remainder_m": segment_remainder["full_remainder_m"],
            "local_remainder_m": segment_remainder["local_remainder_m"],
        })
    budget_record = {
        "method": _INTEGRATION_METHOD,
        "formula": (
            "sum over complete Simpson panels of H^5*M4_panel/2880 plus one "
            "local panel of H^5*max_panel_M4/2880; "
            "M4_panel=sqrt(max(Re)^2+max(Im)^2), "
            "Re=k^4-4*k*u-3*e^2, Im=g-6*k^2*e"),
        "m4_method": "exact_interval_fourth_derivative_per_panel",
        "segments": segments,
        "per_coordinate_remainder_m": remainder["per_coordinate_m"],
        "euclidean_remainder_m": remainder["euclidean_m"],
        "total_subintervals": remainder["total_subintervals"],
        "full_simpson_panels": remainder["full_panels"],
        "local_simpson_panels": remainder["local_panels"],
        "angle_bound_rad": angle_bound,
        "roundoff_and_angle_guard_m": guard,
        "total_budget_m": total_budget,
        "fixed_threshold_m": threshold,
        "confirmed": True,
        "refinement_steps": refinement_steps,
        "floating_point_assumption": (
            "G1 POSITION_ROUNDOFF_GUARD_M per accumulation panel and "
            "ANGLE_ROUNDOFF_GUARD_RAD plus 8*EPSILON*(1+|psi|); libm/trig "
            "values are assumed correctly rounded within these guards; "
            "cubic coefficients and panel endpoints are widened by "
            "bounds.COEFFICIENT_ULPS before the exact per-panel interval M4"),
    }
    return {
        "subintervals": tuple(record["subintervals"] for record in base),
        "prefixes": _psi_prefixes(pieces),
        "budget": budget_record,
    }


def _simpson_piece_integral(piece, psi0, qa, qb, subintervals):
    """Composite Simpson of exp(i*psi) over ``qa..qb`` in one piece."""
    if not (math.isfinite(qa) and math.isfinite(qb)
            and 0.0 <= qa <= qb <= 1.0):
        raise _AuditFailure(
            "NONFINITE_AUDIT_VALUE",
            "invalid Simpson interval qa={!r} qb={!r}".format(qa, qb),
            constraint="AUDIT_INTEGRATION")
    if subintervals < 1 or subintervals % 2:
        raise _AuditFailure(
            "NONFINITE_AUDIT_VALUE",
            "Simpson requires an even positive subinterval count",
            constraint="AUDIT_INTEGRATION")
    width = (qb - qa) / subintervals
    factor = piece.h * (qb - qa) / (3.0 * subintervals)
    cosine = 0.0
    sine = 0.0
    for index in range(subintervals + 1):
        q = qa + width * index
        psi = psi0 + piece.psi_increment(q)
        _finite_audit(psi, "simpson_psi", constraint="AUDIT_INTEGRATION")
        weight = 1.0 if index in (0, subintervals) else (
            4.0 if index % 2 else 2.0)
        cosine += weight * math.cos(psi)
        sine += weight * math.sin(psi)
    x = factor * cosine
    y = factor * sine
    return (
        _finite_audit(x, "simpson_x", constraint="AUDIT_INTEGRATION"),
        _finite_audit(y, "simpson_y", constraint="AUDIT_INTEGRATION"))


def _locate(pieces, s):
    if s <= 0.0:
        return 0, 0.0
    for index, piece in enumerate(pieces):
        if s <= piece.s1:
            if s == piece.s0 and index:
                return index, 0.0
            return index, (s - piece.s0) / piece.h
    return len(pieces) - 1, 1.0


def _jet(pieces, s):
    index, q = _locate(pieces, s)
    piece = pieces[index]
    return piece.value(q), piece.slope(q), piece.second(q), piece.third()


def _psi_prefixes(pieces):
    prefixes = [0.0]
    for piece in pieces:
        increment = _finite_audit(piece.psi_increment(1.0), "psi_increment")
        prefixes.append(_finite_audit(
            prefixes[-1] + increment, "psi_prefix"))
    return tuple(prefixes)


def _psi_local(pieces, prefixes, s):
    if s <= 0.0:
        return 0.0
    index, q = _locate(pieces, s)
    piece = pieces[index]
    local = min(max(s - piece.s0, 0.0), piece.h)
    return prefixes[index] + piece.psi_increment(local / piece.h)


def _integrate_xy(pieces, prefixes, start_qf, s, subintervals):
    """Independent composite-Simpson integration from 0 to ``s``.

    ``subintervals`` contains the even grid counts certified by
    :func:`_integration_plan`.  Each earlier piece is integrated in full;
    the containing piece uses complete even Simpson panels plus one local
    three-point panel, whose remainder is included once per segment in the
    audit budget.
    """
    s = _finite_audit(s, "integration_station")
    if s <= 0.0:
        return start_qf.x, start_qf.y
    index, q_end = _locate(pieces, s)
    x, y = start_qf.x, start_qf.y
    for piece_index in range(index + 1):
        piece = pieces[piece_index]
        psi0 = start_qf.psi + prefixes[piece_index]
        if piece_index < index:
            dx, dy = _simpson_piece_integral(
                piece, psi0, 0.0, 1.0, subintervals[piece_index])
        else:
            count = subintervals[piece_index]
            u = min(max(q_end, 0.0), 1.0)
            scaled = u * count
            complete = min(count, max(0, int(math.floor(scaled))))
            if complete % 2:
                complete -= 1
            q_node = complete / count
            if q_node > u:
                complete = max(0, complete - 2)
                q_node = complete / count
            if complete >= 2:
                dx, dy = _simpson_piece_integral(
                    piece, psi0, 0.0, q_node, complete)
            else:
                dx, dy = 0.0, 0.0
            if u > q_node:
                local_x, local_y = _simpson_piece_integral(
                    piece, psi0, q_node, u, 2)
                dx += local_x
                dy += local_y
        x += dx
        y += dy
    return (
        _finite_audit(x, "independent_x", s=s),
        _finite_audit(y, "independent_y", s=s))


def _direct_curvature(piece, q, a, b):
    k = piece.value(q)
    eta = piece.slope(q)
    A = 1.0 - b * k
    B = a * k
    D = A * A + B * B
    if not (A > 0.0 and D > 0.0 and math.isfinite(D)):
        return None, None, A, D
    return (k * D + a * eta) / math.pow(D, 1.5), eta, A, D


def _curvature_interval(piece, q0, q1, a, b):
    """Outward interval enclosure of C_i on one normalized sub-interval."""
    jets = bounds.segment_jet_bounds(
        piece.c0, piece.c1, piece.c2, piece.c3, piece.h, q0, q1)
    k_interval = jets["k"]
    eta_interval = jets["eta"]
    A_interval = bounds.iadd((1.0, 1.0), bounds.iscale(k_interval, -b))
    B_interval = bounds.iscale(k_interval, a)
    D_interval = bounds.iadd(
        bounds.imul(A_interval, A_interval),
        bounds.imul(B_interval, B_interval))
    A_low = A_interval[0]
    D_low = D_interval[0]
    D_high = D_interval[1]
    if not math.isfinite(D_high) or D_high < 0.0:
        return None, A_low, D_low, D_high, jets
    if A_low <= 0.0 or D_low <= 0.0:
        return None, A_low, D_low, D_high, jets
    numerator = bounds.iadd(
        bounds.imul(k_interval, D_interval),
        bounds.iscale(eta_interval, a))
    denominator_low = math.pow(D_low, 1.5)
    denominator_high = math.pow(D_high, 1.5)
    if denominator_low <= 0.0 or not math.isfinite(denominator_high):
        return None, A_low, D_low, D_high, jets
    quotients = (
        numerator[0] / denominator_low,
        numerator[0] / denominator_high,
        numerator[1] / denominator_low,
        numerator[1] / denominator_high,
    )
    c_low = bounds._next_down(min(quotients))
    c_high = bounds._next_up(max(quotients))
    if not (math.isfinite(c_low) and math.isfinite(c_high)):
        return None, A_low, D_low, D_high, jets
    return (c_low, c_high), A_low, D_low, D_high, jets


def _static_wheel_angle(curvature, wheelbase, track):
    """Maximum ideal-wheel angle for one actual vehicle curvature ``C``.

    This is the project's ideal-wheel definition (``retiming.py`` and the
    G3 geometry contract): wheel centres ``xw = +/- wheelbase/2``,
    ``yw = +/- track/2``; ``m = 1 - yw*C``, ``n = xw*C``; the wheel angle is
    ``delta = atan2(n, m)``.  The constrained quantity is the largest
    ``|delta|`` over the four wheels.
    """
    curvature = float(curvature)
    maximum = 0.0
    for xw in (0.5 * wheelbase, -0.5 * wheelbase):
        for yw in (0.5 * track, -0.5 * track):
            m = 1.0 - yw * curvature
            n = xw * curvature
            if m == 0.0 and n == 0.0:
                angle = 0.0
            else:
                angle = abs(math.atan2(n, m))
            if angle > maximum:
                maximum = angle
    return maximum


def _static_wheel_angle_interval(curvature_interval, wheelbase, track):
    """Upper bound of the maximum wheel angle over a C interval.

    For each fixed wheel, ``delta(C) = atan2(xw*C, 1 - yw*C)`` is monotone in
    ``C`` (its derivative has the sign of ``xw``), so ``|delta|`` attains its
    maximum on a closed C interval at one of the two interval endpoints.  The
    result is widened once for the float evaluation of those endpoints; the
    C endpoints already come from the outward-rounded interval enclosure.
    """
    lower, upper = curvature_interval
    maximum = max(
        _static_wheel_angle(lower, wheelbase, track),
        _static_wheel_angle(upper, wheelbase, track))
    return bounds._next_up(maximum)


def _check_segment_envelope(piece, index, offsets, cmax, config, limits,
                            state):
    """Certify one complete segment with bounded recursive subdivision."""
    segment = {
        "segment_index": index,
        "s0": piece.s0,
        "s1": piece.s1,
        "h": piece.h,
        "leaves": 0,
        "max_depth_reached": 0,
        "passed": False,
        "k": [math.inf, -math.inf],
        "eta": [math.inf, -math.inf],
        "A_lower_by_vehicle": {},
        "D_lower_by_vehicle": {},
        "max_abs_C_by_vehicle": {},
        "max_static_wheel_angle_rad": 0.0,
    }
    per_vehicle_max_c = {offset.vehicle_id: 0.0 for offset in offsets}
    per_vehicle_A = {offset.vehicle_id: math.inf for offset in offsets}
    per_vehicle_D = {offset.vehicle_id: math.inf for offset in offsets}
    max_wheel = 0.0
    stack = [(0.0, 1.0, 0)]
    # Depth-first so the first actual violation is deterministic.  If a leaf
    # reaches max depth with a conservative enclosure but no sampled actual
    # violation, remember it and keep examining the remaining leaves; a real
    # violation in a sibling should be reported as such rather than hidden
    # behind a premature BOUND_UNCONFIRMED.
    unconfirmed = None
    while stack:
        q0, q1, depth = stack.pop()
        state["leaves"] += 1
        if state["leaves"] > config.max_leaves:
            raise _AuditFailure(
                "AUDIT_BUDGET_EXHAUSTED",
                "leaf budget {} exceeded".format(config.max_leaves),
                interval=index, constraint="AUDIT_LEAF_BUDGET",
                s=piece.s0 + 0.5 * piece.h * (q0 + q1),
                leaves=state["leaves"], limit=config.max_leaves)
        state["checks"] += 1
        all_ok = True
        failures = []
        for offset in offsets:
            c_interval, A_low, D_low, D_high, jets = _curvature_interval(
                piece, q0, q1, offset.x, offset.y)
            if jets["k"][0] < segment["k"][0]:
                segment["k"][0] = jets["k"][0]
            if jets["k"][1] > segment["k"][1]:
                segment["k"][1] = jets["k"][1]
            if jets["eta"][0] < segment["eta"][0]:
                segment["eta"][0] = jets["eta"][0]
            if jets["eta"][1] > segment["eta"][1]:
                segment["eta"][1] = jets["eta"][1]
            if c_interval is None:
                all_ok = False
                failures.append((offset.vehicle_id, "NORMAL_BRANCH_VIOLATION",
                                 A_low, D_low, D_high))
                continue
            c_abs = _finite_audit(
                max(abs(c_interval[0]), abs(c_interval[1])),
                "curvature_interval_abs", interval=index,
                vehicle_id=offset.vehicle_id)
            per_vehicle_max_c[offset.vehicle_id] = max(
                per_vehicle_max_c[offset.vehicle_id], c_abs)
            if A_low < per_vehicle_A[offset.vehicle_id]:
                per_vehicle_A[offset.vehicle_id] = A_low
            if D_low < per_vehicle_D[offset.vehicle_id]:
                per_vehicle_D[offset.vehicle_id] = D_low
            wheel = _finite_audit(
                _static_wheel_angle_interval(
                    c_interval, limits.wheelbase, limits.track),
                "static_wheel_angle_interval", interval=index,
                vehicle_id=offset.vehicle_id)
            max_wheel = max(max_wheel, wheel)
            if c_abs > cmax:
                all_ok = False
                failures.append((offset.vehicle_id, "CURVATURE_LIMIT",
                                 c_interval, D_low, D_high))
            elif wheel > limits.steer_angle_max:
                all_ok = False
                failures.append((offset.vehicle_id, "STATIC_WHEEL_LIMIT",
                                 wheel, D_low, D_high))
        if all_ok:
            segment["leaves"] += 1
            segment["max_depth_reached"] = max(
                segment["max_depth_reached"], depth)
            continue
        if depth < config.max_depth:
            midpoint = 0.5 * (q0 + q1)
            # Right first because stack is LIFO; left is processed first.
            stack.append((midpoint, q1, depth + 1))
            stack.append((q0, midpoint, depth + 1))
            continue
        # At maximum depth, distinguish a true point violation from an
        # interval-enclosure that could not be resolved.  Examine several
        # actual points, not only the midpoint: a conservative enclosure near
        # the edge of a violation interval can contain a legal midpoint.
        sample_qs = (q0, 0.5 * (q0 + q1), q1)
        sampled = {}
        for offset in offsets:
            records = []
            for q in sample_qs:
                c_value, _, A, D = _direct_curvature(
                    piece, q, offset.x, offset.y)
                records.append((q, c_value, A, D))
            sampled[offset.vehicle_id] = records
        true_violation = None
        for vehicle_id, reason, *_ in failures:
            for q, c_value, A, D in sampled[vehicle_id]:
                if reason == "NORMAL_BRANCH_VIOLATION":
                    if A is None or A <= 0.0 or not math.isfinite(
                            D) or D <= 0.0:
                        true_violation = (
                            vehicle_id, "NORMAL_BRANCH_VIOLATION", A, D, q)
                        break
                elif reason == "CURVATURE_LIMIT":
                    if c_value is not None and abs(c_value) > cmax:
                        true_violation = (
                            vehicle_id, "CURVATURE_LIMIT", c_value, cmax, q)
                        break
                elif reason == "STATIC_WHEEL_LIMIT":
                    if c_value is not None:
                        angle = _static_wheel_angle(
                            c_value, limits.wheelbase, limits.track)
                        if angle > limits.steer_angle_max:
                            true_violation = (
                                vehicle_id, "STATIC_WHEEL_LIMIT", angle,
                                limits.steer_angle_max, q)
                            break
            if true_violation is not None:
                break
        if true_violation is not None:
            vehicle_id, reason, value, limit, q = true_violation
            raise _AuditFailure(
                reason,
                "{} vehicle_id={} s={!r}".format(
                    reason, vehicle_id, piece.s0 + piece.h * q),
                interval=index, vehicle_id=vehicle_id, constraint=reason,
                s=piece.s0 + piece.h * q, value=value, limit=limit,
                depth=depth)
        if unconfirmed is None:
            unconfirmed = {
                "interval": index,
                "s": piece.s0 + piece.h * 0.5 * (q0 + q1),
                "depth": depth,
                "leaves": state["leaves"],
            }
        continue
    if unconfirmed is not None:
        raise _AuditFailure(
            "BOUND_UNCONFIRMED",
            "interval could not be certified at depth {!r}".format(
                unconfirmed["depth"]),
            interval=unconfirmed["interval"],
            constraint="CURVATURE_INTERVAL", s=unconfirmed["s"],
            depth=unconfirmed["depth"], leaves=unconfirmed["leaves"])
    segment["passed"] = True
    segment["A_lower_by_vehicle"] = {
        str(key): value for key, value in sorted(per_vehicle_A.items())}
    segment["D_lower_by_vehicle"] = {
        str(key): value for key, value in sorted(per_vehicle_D.items())}
    segment["max_abs_C_by_vehicle"] = {
        str(key): value for key, value in sorted(per_vehicle_max_c.items())}
    segment["max_static_wheel_angle_rad"] = max_wheel
    return segment, max_wheel


def _expected_vehicle_state(qf, curvature, offset):
    kappa, eta = curvature[0], curvature[1]
    a, b = offset.x, offset.y
    A = 1.0 - b * kappa
    B = a * kappa
    D = A * A + B * B
    if not (A > 0.0 and D > 0.0):
        raise _AuditFailure(
            "NORMAL_BRANCH_VIOLATION",
            "vehicle_id={} A={!r} D={!r}".format(
                offset.vehicle_id, A, D),
            vehicle_id=offset.vehicle_id, constraint="NORMAL_BRANCH",
            A=A, D=D)
    root = math.sqrt(D)
    theta = qf.psi + math.atan2(B, A)
    C = (kappa * D + a * eta) / math.pow(D, 1.5)
    cosine, sine = math.cos(qf.psi), math.sin(qf.psi)
    center_x = qf.x + cosine * a - sine * b
    center_y = qf.y + sine * a + cosine * b
    return {
        "vehicle_id": offset.vehicle_id,
        "x": center_x, "y": center_y, "theta": theta,
        "curvature": C, "theta_s": C * root,
        "A": A, "D": D, "Q": root,
    }


def _sample_audit(path, request, pieces, start_qf, config, state, plan):
    """Check rigidity, tangent/body headings and theta_s=C_i*Q_i at stations."""
    count = max(2, int(config.sample_points_per_segment))
    prefixes = plan["prefixes"]
    audit_budget = plan["budget"]["total_budget_m"]
    audit_angle_budget = plan["budget"]["angle_bound_rad"]
    stations = []
    for piece in pieces:
        for index in range(count + 1):
            station = piece.s0 + piece.h * index / count
            if not stations or station != stations[-1]:
                stations.append(_finite_audit(
                    station, "sample_station", constraint="SAMPLE_AUDIT"))
    max_position = 0.0
    max_heading = 0.0
    max_rigidity = 0.0
    max_theta_slope = 0.0
    max_curvature = 0.0
    first = None
    budget = max(0.0, float(getattr(path, "position_error_bound_m", 0.0)))
    position_tol = max(1e-8, 10.0 * budget + audit_budget + 1e-9)
    angle_budget = max(0.0, float(getattr(path, "angle_error_bound_rad", 0.0)))
    heading_tol = max(1e-9, 10.0 * angle_budget + audit_angle_budget + 1e-10)
    for station in stations:
        state["checks"] += 1
        try:
            actual = path.state_at_s(station)
        except Exception as exc:
            raise _AuditFailure(
                "PATH_EVALUATION_FAILURE",
                "state_at_s({!r}) raised {}: {}".format(
                    station, type(exc).__name__, exc),
                s=station, constraint="PATH_EVALUATION")
        x, y = _integrate_xy(
            pieces, prefixes, start_qf, station, plan["subintervals"])
        psi_local = _psi_local(pieces, prefixes, station)
        qf = FormationPose(
            _finite_audit(x, "sample_x", s=station),
            _finite_audit(y, "sample_y", s=station),
            _finite_audit(start_qf.psi + psi_local, "sample_psi", s=station))
        curvature = tuple(_finite_audit(
            value, "curvature_jet", s=station) for value in _jet(
                pieces, station))
        if actual.vehicle_ids != tuple(
                offset.vehicle_id for offset in request.model.offsets):
            raise _AuditFailure(
                "VEHICLE_ID_SET_MISMATCH",
                "state vehicle ids={} expected={}".format(
                    actual.vehicle_ids,
                    tuple(offset.vehicle_id
                          for offset in request.model.offsets)),
                s=station, constraint="VEHICLE_ID_SET")
        center_errors = {}
        headings = {}
        for offset in request.model.offsets:
            expected = _expected_vehicle_state(qf, curvature, offset)
            vehicle = actual.vehicle(offset.vehicle_id)
            center_error = _finite_audit(
                math.hypot(vehicle.x - expected["x"],
                           vehicle.y - expected["y"]),
                "center_error", s=station,
                vehicle_id=offset.vehicle_id)
            heading_error = _finite_audit(
                _wrap_abs(vehicle.theta - expected["theta"]),
                "heading_error", s=station,
                vehicle_id=offset.vehicle_id)
            curvature_error = _finite_audit(
                abs(vehicle.curvature - expected["curvature"]),
                "curvature_error", s=station,
                vehicle_id=offset.vehicle_id)
            theta_s_error = _finite_audit(
                abs(vehicle.theta_s - expected["theta_s"]),
                "theta_s_error", s=station,
                vehicle_id=offset.vehicle_id)
            max_position = max(max_position, center_error)
            max_heading = max(max_heading, heading_error)
            max_curvature = max(max_curvature, curvature_error)
            max_theta_slope = max(max_theta_slope, theta_s_error)
            center_errors[str(offset.vehicle_id)] = center_error
            headings[str(offset.vehicle_id)] = heading_error
            if center_error > position_tol:
                raise _AuditFailure(
                    "RIGIDITY_MISMATCH",
                    "vehicle_id={} center error={!r} tolerance={!r}".format(
                        offset.vehicle_id, center_error, position_tol),
                    s=station, vehicle_id=offset.vehicle_id,
                    constraint="RIGIDITY", error=center_error,
                    tolerance=position_tol)
            if heading_error > heading_tol:
                raise _AuditFailure(
                    "HEADING_MISMATCH",
                    "vehicle_id={} heading error={!r} tolerance={!r}".format(
                        offset.vehicle_id, heading_error, heading_tol),
                    s=station, vehicle_id=offset.vehicle_id,
                    constraint="BODY_HEADING", error=heading_error,
                    tolerance=heading_tol)
            if curvature_error > 1e-7:
                raise _AuditFailure(
                    "CURVATURE_FORMULA_MISMATCH",
                    "vehicle_id={} curvature error={!r}".format(
                        offset.vehicle_id, curvature_error),
                    s=station, vehicle_id=offset.vehicle_id,
                    constraint="CURVATURE_FORMULA", error=curvature_error)
            if theta_s_error > 1e-7:
                raise _AuditFailure(
                    "THETA_SLOPE_MISMATCH",
                    "vehicle_id={} theta_s error={!r}".format(
                        offset.vehicle_id, theta_s_error),
                    s=station, vehicle_id=offset.vehicle_id,
                    constraint="THETA_S", error=theta_s_error)
        # Pairwise rigidity is measured independently from the centre map.
        for first_index in range(len(request.model.offsets)):
            for second_index in range(first_index + 1,
                                      len(request.model.offsets)):
                first_offset = request.model.offsets[first_index]
                second_offset = request.model.offsets[second_index]
                actual_first = actual.vehicle(first_offset.vehicle_id)
                actual_second = actual.vehicle(second_offset.vehicle_id)
                actual_distance = _finite_audit(
                    math.hypot(
                        actual_first.x - actual_second.x,
                        actual_first.y - actual_second.y),
                    "pairwise_actual_distance", s=station,
                    vehicle_id=first_offset.vehicle_id)
                expected_distance = _finite_audit(
                    math.hypot(
                        first_offset.x - second_offset.x,
                        first_offset.y - second_offset.y),
                    "pairwise_expected_distance", s=station,
                    vehicle_id=first_offset.vehicle_id)
                error = _finite_audit(
                    abs(actual_distance - expected_distance),
                    "pairwise_rigidity_error", s=station,
                    vehicle_id=first_offset.vehicle_id)
                max_rigidity = max(max_rigidity, error)
                if error > config.rigidity_tolerance_m:
                    raise _AuditFailure(
                        "RIGIDITY_MISMATCH",
                        "pair ({}, {}) distance error={!r} tolerance={!r}".format(
                            first_offset.vehicle_id,
                            second_offset.vehicle_id, error,
                            config.rigidity_tolerance_m),
                        s=station,
                        vehicle_id=first_offset.vehicle_id,
                        other_vehicle_id=second_offset.vehicle_id,
                        constraint="PAIRWISE_RIGIDITY", error=error,
                        tolerance=config.rigidity_tolerance_m)
        # The real declared headings are checked at the initial state.
        if station == 0.0:
            for vehicle_id, heading in request.start_headings.items():
                vehicle = actual.vehicle(vehicle_id)
                error = _finite_audit(
                    _wrap_abs(vehicle.theta - heading),
                    "initial_heading_error", s=station,
                    vehicle_id=vehicle_id)
                if error > INITIAL_HEADING_TOLERANCE_RAD:
                    raise _AuditFailure(
                        "INITIAL_HEADING_MISMATCH",
                        "vehicle_id={} actual={!r} declared={!r}".format(
                            vehicle_id, vehicle.theta, heading),
                        s=station, vehicle_id=vehicle_id,
                        constraint="INITIAL_HEADING", error=error,
                        tolerance=INITIAL_HEADING_TOLERANCE_RAD)
        state["checks"] += 1
    return {
        "stations": stations,
        "sample_count": len(stations),
        "position_tolerance_m": position_tol,
        "heading_tolerance_rad": heading_tol,
        "max_position_error_m": max_position,
        "max_heading_error_rad": max_heading,
        "max_curvature_error": max_curvature,
        "max_theta_s_error": max_theta_slope,
        "max_pairwise_rigidity_error_m": max_rigidity,
    }


def _endpoint_audit(path, request, pieces, start_qf, config, plan):
    endpoint = path.endpoint()
    independent_x, independent_y = _integrate_xy(
        pieces, plan["prefixes"], start_qf, pieces[-1].s1,
        plan["subintervals"])
    independent_psi = start_qf.psi + _psi_local(
        pieces, plan["prefixes"], pieces[-1].s1)
    goal = request.goal_qf
    independent_x = _finite_audit(independent_x, "independent_x")
    independent_y = _finite_audit(independent_y, "independent_y")
    independent_psi = _finite_audit(independent_psi, "independent_psi")
    endpoint_x = _finite_audit(endpoint.x, "reported_endpoint.x")
    endpoint_y = _finite_audit(endpoint.y, "reported_endpoint.y")
    endpoint_psi = _finite_audit(endpoint.psi, "reported_endpoint.psi")
    indep_error = _finite_audit(
        math.hypot(independent_x - goal.x, independent_y - goal.y),
        "independent_position_error")
    indep_heading_error = _finite_audit(
        _wrap_abs(independent_psi - goal.psi),
        "independent_heading_error")
    path_error = _finite_audit(
        math.hypot(endpoint_x - goal.x, endpoint_y - goal.y),
        "reported_position_error")
    path_heading_error = _finite_audit(
        _wrap_abs(endpoint_psi - goal.psi), "reported_heading_error")
    difference = _finite_audit(
        math.hypot(endpoint_x - independent_x, endpoint_y - independent_y),
        "reported_independent_difference")
    difference_heading = _finite_audit(
        _wrap_abs(endpoint_psi - independent_psi),
        "reported_independent_heading_difference")
    position_budget = float(getattr(path, "position_error_bound_m", math.inf))
    angle_budget = float(getattr(path, "angle_error_bound_rad", math.inf))
    audit_budget = plan["budget"]["total_budget_m"]
    audit_angle_budget = plan["budget"]["angle_bound_rad"]
    combined_position = bounds._next_up(position_budget + audit_budget)
    combined_angle = bounds._next_up(angle_budget + audit_angle_budget)
    checks = {
        "method": _INTEGRATION_METHOD,
        "audit_integration": plan["budget"],
        "independent_endpoint": {
            "x": independent_x, "y": independent_y, "psi": independent_psi},
        "reported_endpoint": {
            "x": endpoint_x, "y": endpoint_y, "psi": endpoint_psi},
        "goal": _pose_summary(goal),
        "start": _pose_summary(start_qf),
        "goal_heading_branch_rad": _wrap(goal.psi - start_qf.psi),
        "independent_position_error_m": indep_error,
        "independent_heading_error_rad": indep_heading_error,
        "reported_position_error_m": path_error,
        "reported_heading_error_rad": path_heading_error,
        "reported_independent_difference_m": difference,
        "reported_independent_difference_rad": difference_heading,
        "position_budget_m": position_budget,
        "angle_budget_rad": angle_budget,
        "audit_position_budget_m": audit_budget,
        "audit_angle_budget_rad": audit_angle_budget,
        "combined_position_budget_m": combined_position,
        "combined_angle_budget_rad": combined_angle,
        "position_tolerance_m": config.position_tolerance_m,
        "heading_tolerance_rad": config.heading_tolerance_rad,
    }
    if not math.isfinite(position_budget) or position_budget < 0.0:
        raise _AuditFailure(
            "INTEGRATION_BUDGET_INVALID",
            "position_error_bound_m={!r}".format(position_budget),
            constraint="INTEGRATION_BUDGET")
    if position_budget > config.integration_budget_m:
        raise _AuditFailure(
            "INTEGRATION_BUDGET_EXCEEDED",
            "position budget {!r} exceeds G1 contract {!r}".format(
                position_budget, config.integration_budget_m),
            constraint="INTEGRATION_BUDGET", value=position_budget,
            limit=config.integration_budget_m)
    if not math.isfinite(angle_budget) or angle_budget < 0.0:
        raise _AuditFailure(
            "INTEGRATION_BUDGET_INVALID",
            "angle_error_bound_rad={!r}".format(angle_budget),
            constraint="ANGLE_BUDGET")
    if angle_budget > config.angle_budget_rad:
        raise _AuditFailure(
            "INTEGRATION_BUDGET_EXCEEDED",
            "angle budget {!r} exceeds G1 contract {!r}".format(
                angle_budget, config.angle_budget_rad),
            constraint="ANGLE_BUDGET", value=angle_budget,
            limit=config.angle_budget_rad)
    # The true endpoint differs from each reported integral by at most its
    # own certified remainder+guard.  A pass must remain within tolerance
    # after adding that error budget; a definite failure is still reported
    # when the computed error alone exceeds the tolerance.
    if indep_error > config.position_tolerance_m or (
            indep_error + audit_budget > config.position_tolerance_m):
        raise _AuditFailure(
            "ENDPOINT_POSITION_ERROR",
            "independent endpoint error={!r} + audit budget={!r} "
            "tolerance={!r}".format(
                indep_error, audit_budget, config.position_tolerance_m),
            constraint="ENDPOINT_POSITION", error=indep_error,
            tolerance=config.position_tolerance_m,
            audit_budget_m=audit_budget, s=pieces[-1].s1)
    if indep_heading_error > config.heading_tolerance_rad or (
            indep_heading_error + audit_angle_budget >
            config.heading_tolerance_rad):
        raise _AuditFailure(
            "ENDPOINT_HEADING_ERROR",
            "independent heading error={!r} + audit budget={!r} "
            "tolerance={!r}".format(
                indep_heading_error, audit_angle_budget,
                config.heading_tolerance_rad),
            constraint="ENDPOINT_HEADING", error=indep_heading_error,
            tolerance=config.heading_tolerance_rad,
            audit_budget_rad=audit_angle_budget, s=pieces[-1].s1)
    # The path's own reported endpoint is also a claim that must satisfy the
    # G2 endpoint contract.  A candidate cannot pass merely because an
    # independently evaluated endpoint happens to be acceptable while the
    # returned object reports a different endpoint.
    if not math.isfinite(path_error) or (
            path_error + position_budget > config.position_tolerance_m):
        raise _AuditFailure(
            "ENDPOINT_POSITION_ERROR",
            "reported endpoint error={!r} + G1 budget={!r} tolerance={!r}"
            .format(path_error, position_budget,
                    config.position_tolerance_m),
            constraint="ENDPOINT_POSITION_REPORTED", error=path_error,
            tolerance=config.position_tolerance_m,
            position_budget_m=position_budget, s=pieces[-1].s1)
    if not math.isfinite(path_heading_error) or (
            path_heading_error + angle_budget >
            config.heading_tolerance_rad):
        raise _AuditFailure(
            "ENDPOINT_HEADING_ERROR",
            "reported heading error={!r} + G1 budget={!r} tolerance={!r}"
            .format(path_heading_error, angle_budget,
                    config.heading_tolerance_rad),
            constraint="ENDPOINT_HEADING_REPORTED",
            error=path_heading_error,
            tolerance=config.heading_tolerance_rad,
            angle_budget_rad=angle_budget, s=pieces[-1].s1)
    if difference > combined_position:
        raise _AuditFailure(
            "INTEGRATION_DISAGREEMENT",
            "reported/independent endpoint difference={!r} exceeds the "
            "sum of G1 and audit budgets {!r}".format(
                difference, combined_position),
            constraint="INTEGRATION_DISAGREEMENT", error=difference,
            limit=combined_position)
    if difference_heading > combined_angle:
        raise _AuditFailure(
            "INTEGRATION_DISAGREEMENT",
            "reported/independent heading difference={!r} exceeds the "
            "sum of G1 and audit angle budgets {!r}".format(
                difference_heading, combined_angle),
            constraint="ANGLE_DISAGREEMENT", error=difference_heading,
            limit=combined_angle)
    return checks


def _hold_audit(path, request, config):
    state = {"checks": 0, "leaves": 0}
    request_summary_value = request_summary(request)
    limits = request.limits
    limits_summary_value = limits.as_dict()
    path_summary_value = geometry_path_summary(path)
    request_sha = request_fingerprint(request)
    whole = []
    for offset in request.model.offsets:
        whole.append({
            "segment_index": 0, "s0": 0.0, "s1": 0.0, "h": 0.0,
            "leaves": 1, "max_depth_reached": 0, "passed": True,
            "k": [0.0, 0.0], "eta": [0.0, 0.0],
            "A_lower_by_vehicle": {str(offset.vehicle_id): 1.0},
            "D_lower_by_vehicle": {str(offset.vehicle_id): 1.0},
            "max_abs_C_by_vehicle": {str(offset.vehicle_id): 0.0},
            "max_static_wheel_angle_rad": 0.0,
        })
        break
    if not path.is_hold:
        raise _AuditFailure(
            "HOLD_NOT_VALID", "expected a hold path", constraint="HOLD")
    if path.length_m != 0.0:
        raise _AuditFailure(
            "HOLD_NOT_VALID", "hold path length={!r}".format(path.length_m),
            constraint="HOLD", length_m=path.length_m)
    if path.start_qf != request.start_qf:
        raise _AuditFailure(
            "ENDPOINT_MISMATCH",
            "hold start does not equal original request start",
            constraint="HOLD_START")
    if request.start_qf != request.goal_qf:
        raise _AuditFailure(
            "ENDPOINT_MISMATCH",
            "NO_MOTION hold is allowed only for an exactly identical goal",
            constraint="HOLD_GOAL")
    actual = path.state_at_s(0.0)
    expected = []
    for offset in request.model.offsets:
        expected.append(_expected_vehicle_state(
            request.start_qf, (0.0, 0.0, 0.0, 0.0), offset))
    for record in expected:
        vehicle = actual.vehicle(record["vehicle_id"])
        error = math.hypot(vehicle.x - record["x"],
                           vehicle.y - record["y"])
        heading_error = _wrap_abs(vehicle.theta - record["theta"])
        if error > config.rigidity_tolerance_m:
            raise _AuditFailure(
                "RIGIDITY_MISMATCH", "hold center error={!r}".format(error),
                vehicle_id=record["vehicle_id"], constraint="RIGIDITY",
                error=error, tolerance=config.rigidity_tolerance_m)
        if heading_error > config.heading_tolerance_rad:
            raise _AuditFailure(
                "HEADING_MISMATCH",
                "hold body heading error={!r}".format(heading_error),
                vehicle_id=record["vehicle_id"], constraint="BODY_HEADING",
                error=heading_error,
                tolerance=config.heading_tolerance_rad)
        if abs(vehicle.curvature) > 1e-12:
            raise _AuditFailure(
                "CURVATURE_LIMIT", "hold curvature is non-zero",
                vehicle_id=record["vehicle_id"], constraint="CURVATURE_LIMIT")
    state["checks"] += len(expected)
    return AuditReport.ok(
        detail="exact-start hold; geometry only",
        request_summary=request_summary_value,
        path_summary=path_summary_value,
        limits_summary=limits_summary_value,
        whole_segment_bounds=tuple(whole),
        integration={
            "kind": "hold", "position_error_bound_m": 0.0,
            "angle_error_bound_rad": 0.0,
            "independent_position_error_m": 0.0,
            "independent_heading_error_rad": 0.0,
        },
        c2={"node_count": 0, "nodes": [], "max_value_error": 0.0,
            "max_slope_error": 0.0, "max_second_error": 0.0,
            "max_abs_g": 0.0},
        check_count=state["checks"],
        metrics={"hold": True, "leaves": 0,
                 "goal_heading_branch_rad": 0.0},
        request_sha256=request_sha)


def audit_geometry(request, path, config=None, *, planning_context=None):
    """Independently audit one candidate path against the original request."""
    if config is None:
        config = GeometryAuditConfig()
    request_ok = validate_virtual_a_request(request)
    base_summary = request_summary(request) if isinstance(
        request, VirtualARequest) else {
            "type": type(request).__name__, "valid": False}
    path_summary_value = None
    if path is not None:
        try:
            path_summary_value = geometry_path_summary(path)
        except Exception as exc:
            path_summary_value = {
                "summary_error": "{}: {}".format(type(exc).__name__, exc)}
    if not isinstance(config, GeometryAuditConfig):
        return AuditReport.fail(
            "INVALID_AUDIT_CONFIG",
            "expected GeometryAuditConfig",
            request_summary=base_summary, path_summary=path_summary_value or {},
            limits_summary={}, check_count=0)
    config_validation = validate_geometry_audit_config(config)
    if not config_validation.passed:
        return AuditReport.fail(
            "INVALID_AUDIT_CONFIG", config_validation.detail,
            location=config_validation.location,
            request_summary=base_summary,
            path_summary=path_summary_value or {},
            limits_summary={}, check_count=0,
            metrics={"config_validation": config_validation.metrics})
    if not request_ok.passed:
        return AuditReport.fail(
            request_ok.reason, request_ok.detail,
            location=request_ok.location,
            request_summary=base_summary,
            path_summary=path_summary_value or {},
            limits_summary={},
            check_count=1,
            metrics={"request_validation": request_ok.metrics})
    limits = request.limits
    limits_summary_value = limits.as_dict()
    request_sha = request_fingerprint(request)
    context_sha = ""
    context_grid_sha = ""
    if planning_context is not None:
        if not isinstance(planning_context, MappingABC):
            return AuditReport.fail(
                "INVALID_PLANNING_CONTEXT",
                "planning_context must be a mapping",
                location={"constraint": "PLANNING_CONTEXT"},
                request_summary=base_summary,
                path_summary=path_summary_value or {},
                limits_summary=limits_summary_value,
                request_sha256=request_sha, check_count=1)
        try:
            context_sha = planning_context_sha256(planning_context)
        except (TypeError, ValueError) as exc:
            return AuditReport.fail(
                "INVALID_PLANNING_CONTEXT",
                "planning context is not finite JSON: {}: {}".format(
                    type(exc).__name__, exc),
                location={"constraint": "PLANNING_CONTEXT"},
                request_summary=base_summary,
                path_summary=path_summary_value or {},
                limits_summary=limits_summary_value,
                request_sha256=request_sha, check_count=1)
        if planning_context.get("schema") == G4_PLANNING_SCHEMA:
            expected_keys = {
                "schema", "stage", "request_sha256", "grid",
                "planner_config", "timing_config", "collision_config",
            }
            if set(planning_context) != expected_keys:
                return AuditReport.fail(
                    "INVALID_PLANNING_CONTEXT",
                    "G4 context keys missing={} extra={}".format(
                        sorted(expected_keys - set(planning_context)),
                        sorted(set(planning_context) - expected_keys)),
                    location={"constraint": "PLANNING_CONTEXT"},
                    request_summary=base_summary,
                    path_summary=path_summary_value or {},
                    limits_summary=limits_summary_value,
                    request_sha256=request_sha, check_count=1,
                    planning_context_sha256=context_sha)
            if planning_context.get("request_sha256") != request_sha:
                return AuditReport.fail(
                    "PLANNING_CONTEXT_MISMATCH",
                    "context request_sha256 differs from the request",
                    location={"constraint": "PLANNING_CONTEXT"},
                    request_summary=base_summary,
                    path_summary=path_summary_value or {},
                    limits_summary=limits_summary_value,
                    request_sha256=request_sha, check_count=1,
                    planning_context_sha256=context_sha)
            try:
                context_grid_sha = grid_summary_sha256(
                    planning_context["grid"])
            except (TypeError, ValueError) as exc:
                return AuditReport.fail(
                    "INVALID_PLANNING_CONTEXT",
                    "context grid is not finite JSON: {}: {}".format(
                        type(exc).__name__, exc),
                    location={"constraint": "PLANNING_CONTEXT"},
                    request_summary=base_summary,
                    path_summary=path_summary_value or {},
                    limits_summary=limits_summary_value,
                    request_sha256=request_sha, check_count=1,
                    planning_context_sha256=context_sha)
    if not isinstance(path, VirtualAPath):
        return AuditReport.fail(
            "INVALID_PATH", "expected VirtualAPath",
            request_summary=base_summary,
            path_summary=path_summary_value or {},
            limits_summary=limits_summary_value,
            request_sha256=request_sha, check_count=1)
    attached = getattr(path, "request_sha256", None)
    if attached is not None and attached != request_sha:
        return AuditReport.fail(
            "FORGED_REQUEST_DIGEST",
            "path request_sha256 does not match the original request",
            location={"constraint": "REQUEST_DIGEST"},
            request_summary=base_summary,
            path_summary=path_summary_value or {},
            limits_summary=limits_summary_value,
            request_sha256=request_sha, check_count=1)
    # FormationModel deliberately has object identity equality.  Bind the
    # path to the request by the exact rigid offset structure instead, so an
    # independently reconstructed, structurally identical model is accepted
    # while any changed vehicle ID or offset is rejected.
    try:
        path_offsets = tuple(
            (o.vehicle_id, float(o.x), float(o.y)) for o in path.model.offsets)
        request_offsets = tuple(
            (o.vehicle_id, float(o.x), float(o.y))
            for o in request.model.offsets)
        if path_offsets != request_offsets:
            raise ValueError("offset structure differs")
    except Exception:
        return AuditReport.fail(
            "MODEL_CHANGED", "unable to bind path model to request",
            location={"constraint": "MODEL_IDENTITY"},
            request_summary=base_summary,
            path_summary=path_summary_value or {},
            limits_summary=limits_summary_value,
            request_sha256=request_sha, check_count=1)
    if path.start_qf != request.start_qf:
        return AuditReport.fail(
            "START_MISMATCH",
            "path start_qf={} request start_qf={}".format(
                _pose_summary(path.start_qf), _pose_summary(request.start_qf)),
            location={"constraint": "START_QF"},
            request_summary=base_summary,
            path_summary=path_summary_value or {},
            limits_summary=limits_summary_value,
            request_sha256=request_sha, check_count=1)
    if path.is_hold:
        try:
            report = _hold_audit(path, request, config)
        except _AuditFailure as exc:
            return AuditReport.fail(
                exc.reason, exc.detail, location=exc.location,
                request_summary=base_summary,
                path_summary=path_summary_value or {},
                limits_summary=limits_summary_value,
                request_sha256=request_sha, check_count=1,
                metrics={"planning_context_provided": planning_context is not None})
        metrics = dict(report.metrics)
        metrics["planning_context_provided"] = planning_context is not None
        return AuditReport.ok(
            detail=report.detail, request_summary=report.request_summary,
            path_summary=report.path_summary,
            limits_summary=report.limits_summary,
            whole_segment_bounds=report.whole_segment_bounds,
            integration=report.integration, c2=report.c2,
            check_count=report.check_count, metrics=metrics,
            request_sha256=report.request_sha256)
    profile = path.profile
    if profile is None:
        return AuditReport.fail(
            "INVALID_PATH", "motion path has no curvature profile",
            location={"constraint": "PROFILE"},
            request_summary=base_summary,
            path_summary=path_summary_value or {},
            limits_summary=limits_summary_value,
            request_sha256=request_sha, check_count=1)
    profile_state = {"checks": 1, "leaves": 0}
    try:
        pieces, stations, knot_values, knot_derivatives = \
            _independent_pieces(profile.knots)
        if not (math.isfinite(path.length_m)
                and abs(path.length_m - stations[-1]) <= 1e-12):
            raise _AuditFailure(
                "LENGTH_MISMATCH",
                "path length={!r} knots end={!r}".format(
                    path.length_m, stations[-1]),
                constraint="PROFILE_LENGTH", s=stations[-1])
        if len(pieces) < 1:
            raise _AuditFailure(
                "INVALID_PROFILE", "no nonzero segments",
                constraint="PROFILE")
        if abs(knot_values[0]) > INITIAL_CURVATURE_TOLERANCE:
            raise _AuditFailure(
                "UNSUPPORTED_INITIAL_STATE",
                "k0={!r} must be zero".format(knot_values[0]),
                constraint="INITIAL_CURVATURE", s=0.0)
        if abs(knot_derivatives[0]) > INITIAL_CURVATURE_TOLERANCE:
            raise _AuditFailure(
                "UNSUPPORTED_INITIAL_STATE",
                "eta0={!r} must be zero".format(knot_derivatives[0]),
                constraint="INITIAL_CURVATURE", s=0.0)
        policy = coerce_goal_policy(request.goal_policy)
        if policy.value == "STRAIGHT":
            if abs(knot_values[-1]) > config.c2_value_tol:
                raise _AuditFailure(
                    "GOAL_POLICY_VIOLATION",
                    "STRAIGHT terminal k={!r}".format(knot_values[-1]),
                    constraint="STRAIGHT_K", s=stations[-1],
                    value=knot_values[-1], limit=config.c2_value_tol)
            if abs(knot_derivatives[-1]) > config.c2_slope_tol:
                raise _AuditFailure(
                    "GOAL_POLICY_VIOLATION",
                    "STRAIGHT terminal eta={!r}".format(
                        knot_derivatives[-1]),
                    constraint="STRAIGHT_ETA", s=stations[-1],
                    value=knot_derivatives[-1], limit=config.c2_slope_tol)
        c2 = _node_c2_audit(pieces, config)
        _profile_pieces_crosscheck(path, pieces)
        plan = _integration_plan(pieces, path.start_qf.psi, config)
        endpoint_checks = _endpoint_audit(
            path, request, pieces, path.start_qf, config, plan)
        cmax = 1.0 / float(limits.r_min_ackermann)
        state = profile_state
        whole = []
        max_wheel = 0.0
        for index, piece in enumerate(pieces):
            segment, wheel = _check_segment_envelope(
                piece, index, request.model.offsets, cmax, config, limits,
                state)
            whole.append(segment)
            max_wheel = max(max_wheel, wheel)
        sample_checks = _sample_audit(
            path, request, pieces, path.start_qf, config, state, plan)
        # The envelope has to cover every segment, not merely the sample
        # stations.  Its per-segment records carry the outward bounds.
        if len(whole) != len(pieces):
            raise _AuditFailure(
                "AUDIT_INCOMPLETE", "not every segment was certified",
                constraint="WHOLE_SEGMENT_COVERAGE")
        for segment in whole:
            if not segment["passed"]:
                raise _AuditFailure(
                    "AUDIT_INCOMPLETE", "segment {} is unconfirmed".format(
                        segment["segment_index"]),
                    interval=segment["segment_index"],
                    constraint="WHOLE_SEGMENT_COVERAGE")
        integration = dict(endpoint_checks)
        integration["total_checks"] = state["checks"]
        integration["c2"] = c2
        metrics = {
            "planning_context_provided": planning_context is not None,
            "leaves": state["leaves"],
            "segment_count": len(pieces),
            "knot_count": len(stations),
            "max_static_wheel_angle_rad": max_wheel,
            "r_min_ackermann": limits.r_min_ackermann,
            "curvature_limit": cmax,
            "goal_heading_branch_rad": endpoint_checks[
                "goal_heading_branch_rad"],
            "sample_audit": sample_checks,
        }
        return AuditReport.ok(
            detail=(
                "independent C2/interval/integration audit; geometry only, "
                "no timing/map/ROS/collision claim"),
            request_summary=base_summary,
            path_summary=path_summary_value or {},
            limits_summary=limits_summary_value,
            whole_segment_bounds=tuple(whole),
            integration=integration, c2=c2,
            check_count=state["checks"], metrics=metrics,
            request_sha256=request_sha,
            planning_context_sha256=context_sha,
            grid_sha256=context_grid_sha)
    except _AuditFailure as exc:
        return AuditReport.fail(
            exc.reason, exc.detail, location=exc.location,
            request_summary=base_summary,
            path_summary=path_summary_value or {},
            limits_summary=limits_summary_value,
            request_sha256=request_sha,
            check_count=profile_state["checks"],
            planning_context_sha256=context_sha,
            grid_sha256=context_grid_sha,
            metrics={"planning_context_provided": planning_context is not None})
    except Exception as exc:
        return AuditReport.fail(
            "AUDIT_NUMERICAL_FAILURE",
            "{}: {}".format(type(exc).__name__, exc),
            location={"constraint": "AUDIT_NUMERICAL_FAILURE"},
            request_summary=base_summary,
            path_summary=path_summary_value or {},
            limits_summary=limits_summary_value,
            request_sha256=request_sha,
            check_count=profile_state["checks"],
            planning_context_sha256=context_sha,
            grid_sha256=context_grid_sha,
            metrics={"planning_context_provided": planning_context is not None})


# ---------------------------------------------------------------------------
# G3 independent common-time dynamic audit
# ---------------------------------------------------------------------------

_DYNAMIC_METRICS = (
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
_DYNAMIC_WHEELS = ((0.5, 0.5), (0.5, -0.5), (-0.5, 0.5), (-0.5, -0.5))
_MODEL_TOP_KEYS = frozenset((
    "backend", "model_version", "path_model_version", "request_inputs",
    "request_sha256", "path", "timing", "numerics", "planning_context"))
_MODEL_PATH_KEYS = frozenset((
    "kind", "model_version", "start_qf", "length_m",
    "position_error_bound_m", "angle_error_bound_rad", "integration_budget",
    "profile"))
_MODEL_PROFILE_KEYS = frozenset(("kind", "numerics_version", "length_m",
                                "knots"))
_MODEL_TIMING_KEYS = frozenset(("law", "kind", "S", "T_s", "selection"))
_MODEL_NUMERICS_KEYS = frozenset((
    "version", "position_error_bound_m", "angle_error_bound_rad",
    "integration_method", "floating_point_assumption"))


def _audit_canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False)


def _audit_core_float(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _AuditFailure(
            "INVALID_MODEL_JSON",
            "{} must be a JSON number, got {!r}".format(label, value),
            constraint="MODEL_JSON")
    return _finite_audit(value, label, constraint="MODEL_JSON")


def _audit_progress(tau):
    return tau ** 3 * (10.0 - 15.0 * tau + 6.0 * tau * tau)


def _audit_tau_for_fraction(fraction):
    """Independent monotone inversion of the declared quintic progress law."""
    fraction = _finite_audit(fraction, "progress_fraction")
    if fraction <= 0.0:
        return 0.0
    if fraction >= 1.0:
        return 1.0
    low, high = 0.0, 1.0
    for _ in range(160):
        middle = 0.5 * (low + high)
        if _audit_progress(middle) < fraction:
            low = middle
        else:
            high = middle
        if high - low <= 1e-17:
            break
    return 0.5 * (low + high)


def _audit_poly(coefficients, tau):
    result = 0.0
    for coefficient in reversed(coefficients):
        result = result * tau + coefficient
    return result


def _audit_range(values):
    low = bounds.widen(min(values))
    high = bounds.widen(max(values))
    return (low[0], high[1])


def _audit_h1_range(tau0, tau1):
    values = [_audit_poly((0.0, 0.0, 30.0, -60.0, 30.0), tau0),
              _audit_poly((0.0, 0.0, 30.0, -60.0, 30.0), tau1)]
    if tau0 <= 0.5 <= tau1:
        values.append(1.875)
    return (0.0, bounds.widen(max(values))[1])


def _audit_h2_range(tau0, tau1):
    root = math.sqrt(3.0)
    values = [_audit_poly((0.0, 60.0, -180.0, 120.0), tau0),
              _audit_poly((0.0, 60.0, -180.0, 120.0), tau1)]
    for tau in ((3.0 - root) / 6.0, (3.0 + root) / 6.0):
        if tau0 < tau < tau1:
            values.append(_audit_poly((0.0, 60.0, -180.0, 120.0), tau))
    return _audit_range(values)


def _audit_h3_range(tau0, tau1):
    values = [_audit_poly((60.0, -360.0, 360.0), tau0),
              _audit_poly((60.0, -360.0, 360.0), tau1)]
    if tau0 < 0.5 < tau1:
        values.append(-30.0)
    return _audit_range(values)


def _audit_time_intervals(tau0, tau1, total_s, duration):
    if not (0.0 <= tau0 <= tau1 <= 1.0):
        raise _AuditFailure(
            "INVALID_TAU_INTERVAL",
            "tau interval [{!r}, {!r}]".format(tau0, tau1),
            constraint="TIME_LAW")
    scale1 = total_s / duration
    scale2 = total_s / (duration * duration)
    scale3 = total_s / (duration * duration * duration)
    return (bounds.iscale(_audit_h1_range(tau0, tau1), scale1),
            bounds.iscale(_audit_h2_range(tau0, tau1), scale2),
            bounds.iscale(_audit_h3_range(tau0, tau1), scale3))


def _audit_locate_side(pieces, s, side):
    if s <= 0.0:
        return 0, 0.0
    last = len(pieces) - 1
    for index, piece in enumerate(pieces):
        if s < piece.s1:
            return index, (s - piece.s0) / piece.h
        if s == piece.s1:
            if index == last or side == "left":
                return index, 1.0
            return index + 1, 0.0
    return last, 1.0


def _audit_spatial(pieces, prefixes, start_qf, request, s, side,
                   subintervals):
    """Independent spatial state from raw Hermite coefficients and knots."""
    index, q = _audit_locate_side(pieces, s, side)
    piece = pieces[index]
    k = _finite_audit(piece.value(q), "audit.k", s=s)
    eta = _finite_audit(piece.slope(q), "audit.eta", s=s)
    u = _finite_audit(piece.second(q), "audit.u", s=s)
    g = _finite_audit(piece.third(), "audit.g", s=s)
    psi = _finite_audit(
        start_qf.psi + prefixes[index] + piece.psi_increment(q),
        "audit.psi", s=s)
    x, y = _integrate_xy(
        pieces, prefixes, start_qf, s, subintervals)
    x = _finite_audit(x, "audit.x", s=s)
    y = _finite_audit(y, "audit.y", s=s)
    cosine, sine = math.cos(psi), math.sin(psi)
    vehicles = {}
    for offset in request.model.offsets:
        aoff, boff = offset.x, offset.y
        radius2 = aoff * aoff + boff * boff
        A = 1.0 - boff * k
        B = aoff * k
        D = A * A + B * B
        if not (A > 0.0 and D > 0.0 and math.isfinite(D)):
            raise _AuditFailure(
                "NORMAL_BRANCH_VIOLATION",
                "vehicle_id={} A={!r} D={!r}".format(
                    offset.vehicle_id, A, D),
                vehicle_id=offset.vehicle_id, s=s,
                constraint="NORMAL_BRANCH")
        Q = math.sqrt(D)
        H = k + aoff * eta / D
        C = H / Q
        D_s = 2.0 * (radius2 * k - boff) * eta
        D_ss = 2.0 * (radius2 * eta * eta + (radius2 * k - boff) * u)
        Q_s = D_s / (2.0 * Q)
        Q_ss = D_ss / (2.0 * Q) - D_s * D_s / (4.0 * Q ** 3)
        H_s = eta + aoff * (u / D - eta * D_s / (D * D))
        H_ss = u + aoff * (
            g / D - 2.0 * u * D_s / (D * D)
            - eta * D_ss / (D * D)
            + 2.0 * eta * D_s * D_s / (D ** 3))
        C_s = H_s / Q - H * Q_s / (Q * Q)
        C_ss = (H_ss / Q - 2.0 * H_s * Q_s / (Q * Q)
                - H * Q_ss / (Q * Q)
                + 2.0 * H * Q_s * Q_s / (Q ** 3))
        z2 = (-aoff * k * k - boff * eta,
              k - boff * k * k + aoff * eta)
        z3 = (-k * k + boff * k ** 3 - 3.0 * aoff * k * eta - boff * u,
              eta - 3.0 * boff * k * eta + aoff * u - aoff * k ** 3)

        def rotate(local_x, local_y):
            return (cosine * local_x - sine * local_y,
                    sine * local_x + cosine * local_y)
        p_s = rotate(A, B)
        p_ss = rotate(*z2)
        p_sss = rotate(*z3)
        center_x = x + cosine * aoff - sine * boff
        center_y = y + sine * aoff + cosine * boff
        theta = psi + math.atan2(B, A)
        vehicles[offset.vehicle_id] = {
            "vehicle_id": offset.vehicle_id,
            "x": _finite_audit(center_x, "audit.vehicle.x", s=s),
            "y": _finite_audit(center_y, "audit.vehicle.y", s=s),
            "theta": _finite_audit(theta, "audit.vehicle.theta", s=s),
            "C": _finite_audit(C, "audit.vehicle.C", s=s),
            "C_s": _finite_audit(C_s, "audit.vehicle.C_s", s=s),
            "C_ss": _finite_audit(C_ss, "audit.vehicle.C_ss", s=s),
            "Q": _finite_audit(Q, "audit.vehicle.Q", s=s),
            "H": _finite_audit(H, "audit.vehicle.H", s=s),
            "H_s": _finite_audit(H_s, "audit.vehicle.H_s", s=s),
            "H_ss": _finite_audit(H_ss, "audit.vehicle.H_ss", s=s),
            "p_s": p_s, "p_ss": p_ss, "p_sss": p_sss,
        }
    return {"s": s, "x": x, "y": y, "psi": psi,
            "k": k, "eta": eta, "u": u, "g": g,
            "vehicles": vehicles, "side": side}


def _audit_wheel_values(vehicle, limits, v, w, a):
    angles, speeds, rates, accels = [], [], [], []
    C = vehicle["C"]
    C_s = vehicle["C_s"]
    C_ss = vehicle["C_ss"]
    for xw_factor, yw_factor in _DYNAMIC_WHEELS:
        xw = xw_factor * limits.wheelbase
        yw = yw_factor * limits.track
        m = 1.0 - yw * C
        n = xw * C
        Ew = m * m + n * n
        if not (Ew > 0.0 and math.isfinite(Ew)):
            raise _AuditFailure(
                "WHEEL_GEOMETRY_VIOLATION",
                "vehicle_id={} Ew={!r}".format(vehicle["vehicle_id"], Ew),
                vehicle_id=vehicle["vehicle_id"],
                constraint="WHEEL_GEOMETRY")
        Ew_C = -2.0 * yw + 2.0 * (xw * xw + yw * yw) * C
        delta_s = xw / Ew * C_s
        delta_ss = xw / Ew * C_ss - xw * Ew_C / (Ew * Ew) * C_s * C_s
        angles.append(math.atan2(n, m))
        speeds.append(math.sqrt(Ew) * v)
        rates.append(delta_s * w)
        accels.append(delta_ss * w * w + delta_s * a)
    return angles, speeds, rates, accels


def _audit_dynamic_from_spatial(spatial, limits, w, a, j):
    vehicles = {}
    for vehicle_id, vehicle in spatial["vehicles"].items():
        Q, H = vehicle["Q"], vehicle["H"]
        v = Q * w
        omega = H * w
        p_s, p_ss, p_sss = vehicle["p_s"], vehicle["p_ss"], vehicle["p_sss"]
        w2, w3 = w * w, w * w * w
        velocity = (p_s[0] * w, p_s[1] * w)
        acceleration = (p_ss[0] * w2 + p_s[0] * a,
                        p_ss[1] * w2 + p_s[1] * a)
        jerk = (p_sss[0] * w3 + 3.0 * p_ss[0] * w * a + p_s[0] * j,
                p_sss[1] * w3 + 3.0 * p_ss[1] * w * a + p_s[1] * j)
        yaw_accel = H * a + vehicle["H_s"] * w2
        yaw_jerk = (H * j + 3.0 * vehicle["H_s"] * w * a
                    + vehicle["H_ss"] * w3)
        lateral = abs(vehicle["C"]) * v * v
        angles, speeds, rates, accels = _audit_wheel_values(
            vehicle, limits, v, w, a)
        vehicles[vehicle_id] = {
            "vehicle_id": vehicle_id,
            "x": vehicle["x"], "y": vehicle["y"],
            "theta": vehicle["theta"], "C_i": vehicle["C"],
            "v_i": _finite_audit(v, "audit.v", vehicle_id=vehicle_id),
            "omega_i": _finite_audit(
                omega, "audit.omega", vehicle_id=vehicle_id),
            "velocity": velocity, "acceleration": acceleration, "jerk": jerk,
            "yaw_accel": _finite_audit(
                yaw_accel, "audit.yaw_accel", vehicle_id=vehicle_id),
            "yaw_jerk": _finite_audit(
                yaw_jerk, "audit.yaw_jerk", vehicle_id=vehicle_id),
            "lateral_accel": _finite_audit(
                lateral, "audit.lateral_accel", vehicle_id=vehicle_id),
            "wheel_angles": angles, "wheel_speeds": speeds,
            "steer_rates": rates, "steer_accels": accels,
        }
    return vehicles


def _audit_dynamic_at_station(pieces, prefixes, start_qf, request, s,
                              side, w, a, j, subintervals):
    spatial = _audit_spatial(
        pieces, prefixes, start_qf, request, s, side, subintervals)
    return spatial, _audit_dynamic_from_spatial(
        spatial, request.limits, w, a, j)


def _audit_dynamic_at_time(pieces, prefixes, start_qf, request, t, side,
                           total_s, duration, subintervals):
    tau = t / duration
    if not (0.0 < tau < 1.0):
        raise _AuditFailure(
            "TIME_DOMAIN", "interior time required, t={!r}".format(t),
            constraint="TIME_LAW")
    h = _audit_progress(tau)
    h1 = 30.0 * tau * tau * (1.0 - tau) ** 2
    h2 = 60.0 * tau - 180.0 * tau * tau + 120.0 * tau ** 3
    h3 = 60.0 - 360.0 * tau + 360.0 * tau * tau
    s = total_s * h
    w = total_s * h1 / duration
    a = total_s * h2 / (duration * duration)
    j = total_s * h3 / (duration ** 3)
    return _audit_dynamic_at_station(
        pieces, prefixes, start_qf, request, s, side, w, a, j, subintervals)


def _audit_compare_scalar(actual, expected, label, tolerance, **location):
    error = _finite_audit(
        abs(float(actual) - float(expected)),
        label + "_error", **location)
    if error > tolerance:
        raise _AuditFailure(
            "DYNAMIC_STATE_MISMATCH",
            "{} actual={!r} independent={!r} error={!r} tolerance={!r}".format(
                label, actual, expected, error, tolerance),
            error=error, tolerance=tolerance, quantity=label, **location)
    return error


def _audit_compare_vector(actual, expected, label, tolerance, **location):
    actual = tuple(actual)
    expected = tuple(expected)
    if len(actual) != len(expected):
        raise _AuditFailure(
            "DYNAMIC_STATE_MISMATCH",
            "{} length {} != {}".format(label, len(actual), len(expected)),
            quantity=label, **location)
    maximum = 0.0
    for index, (left, right) in enumerate(zip(actual, expected)):
        maximum = max(maximum, _audit_compare_scalar(
            left, right, "{}_{}".format(label, index), tolerance,
            component=index, **location))
    return maximum


def _audit_compare_state(actual, expected_vehicles, expected_qf, expected_s,
                         config, label, *, position_tolerance=None, **location):
    """Compare one production state with the independent raw-profile state."""
    if not hasattr(actual, "vehicles") or not hasattr(actual, "qf"):
        raise _AuditFailure(
            "INVALID_PLAN_STATE", "{} is not a time state".format(label),
            constraint="STATE_API")
    if position_tolerance is None:
        position_tolerance = config.state_tolerance
    maximum = 0.0
    maximum = max(maximum, _audit_compare_scalar(
        actual.s, expected_s, "s", config.state_tolerance, **location))
    maximum = max(maximum, _audit_compare_scalar(
        actual.qf.x, expected_qf["x"], "qf.x", position_tolerance,
        **location))
    maximum = max(maximum, _audit_compare_scalar(
        actual.qf.y, expected_qf["y"], "qf.y", position_tolerance,
        **location))
    maximum = max(maximum, _audit_compare_scalar(
        _wrap_abs(actual.qf.psi - expected_qf["psi"]), 0.0, "qf.psi",
        position_tolerance, **location))
    actual_vehicles = {vehicle.vehicle_id: vehicle for vehicle in actual.vehicles}
    if set(actual_vehicles) != set(expected_vehicles):
        raise _AuditFailure(
            "VEHICLE_ID_SET_MISMATCH",
            "state ids={} independent ids={}".format(
                sorted(actual_vehicles), sorted(expected_vehicles)),
            constraint="VEHICLE_ID_SET", **location)
    for vehicle_id in sorted(expected_vehicles):
        vehicle = actual_vehicles[vehicle_id]
        expected = expected_vehicles[vehicle_id]
        local = dict(location)
        local["vehicle_id"] = vehicle_id
        maximum = max(maximum, _audit_compare_scalar(
            vehicle.x, expected["x"], "x", position_tolerance,
            **local))
        maximum = max(maximum, _audit_compare_scalar(
            vehicle.y, expected["y"], "y", position_tolerance,
            **local))
        maximum = max(maximum, _audit_compare_scalar(
            _wrap_abs(vehicle.theta - expected["theta"]), 0.0, "theta",
            position_tolerance, **local))
        maximum = max(maximum, _audit_compare_scalar(
            vehicle.C_i, expected["C_i"], "C_i", config.state_tolerance,
            **local))
        maximum = max(maximum, _audit_compare_scalar(
            vehicle.v_i, expected["v_i"], "v_i", config.state_tolerance,
            **local))
        maximum = max(maximum, _audit_compare_scalar(
            vehicle.omega_i, expected["omega_i"], "omega_i",
            config.state_tolerance, **local))
        for name in ("velocity", "acceleration", "jerk"):
            maximum = max(maximum, _audit_compare_vector(
                getattr(vehicle, name), expected[name], name,
                config.state_tolerance, **local))
        for name in ("yaw_accel", "yaw_jerk", "lateral_accel"):
            maximum = max(maximum, _audit_compare_scalar(
                getattr(vehicle, name), expected[name], name,
                config.state_tolerance, **local))
        for name in ("wheel_angles", "wheel_speeds", "steer_rates",
                     "steer_accels"):
            maximum = max(maximum, _audit_compare_vector(
                getattr(vehicle, name), expected[name], name,
                config.state_tolerance, **local))
    return maximum


def _audit_hold_expected(pieces, prefixes, start_qf, request, s, side,
                         subintervals):
    spatial = _audit_spatial(
        pieces, prefixes, start_qf, request, s, side, subintervals)
    vehicles = {}
    for vehicle_id, vehicle in spatial["vehicles"].items():
        angles = []
        for xw_factor, yw_factor in _DYNAMIC_WHEELS:
            xw = xw_factor * request.limits.wheelbase
            yw = yw_factor * request.limits.track
            m = 1.0 - yw * vehicle["C"]
            n = xw * vehicle["C"]
            angles.append(math.atan2(n, m))
        vehicles[vehicle_id] = {
            "vehicle_id": vehicle_id,
            "x": vehicle["x"], "y": vehicle["y"],
            "theta": vehicle["theta"], "C_i": vehicle["C"],
            "v_i": 0.0, "omega_i": 0.0,
            "velocity": (0.0, 0.0), "acceleration": (0.0, 0.0),
            "jerk": (0.0, 0.0),
            "yaw_accel": 0.0, "yaw_jerk": 0.0, "lateral_accel": 0.0,
            "wheel_angles": angles,
            "wheel_speeds": (0.0, 0.0, 0.0, 0.0),
            "steer_rates": (0.0, 0.0, 0.0, 0.0),
            "steer_accels": (0.0, 0.0, 0.0, 0.0),
        }
    return spatial, vehicles


def _audit_interval_vehicle(jets, offset, limits, w, a, j, xw, yw):
    """Independent outward dynamic bounds for one vehicle/wheel cell.

    The inputs are closed intervals for one curvature-profile cell and one
    closed tau cell.  The whole-domain certificate partitions every Hermite
    segment into closed cells, so a node's left and right limits are enclosed
    by the adjacent cells (the ``_audit_node_checks`` routine additionally
    compares both one-sided pointwise limits with those cells).  The dynamic
    jerk enclosure is exactly ``p_sss*w³ + 3*p_ss*w*a_time + p_s*j_time`` with
    the rigid-offset formula

    ``R(-psi) p_sss = (-k²+b*k³-3*a*k*eta-b*u,
                        eta-3*b*k*eta+a*u-a*k³)``.

    The returned scalar ``jerk`` is the certified norm upper used against
    ``jerk_max``; the component intervals and their norm are independent
    diagnostics used to check that the enclosure covers pointwise values.
    """
    k, eta, u, g = jets["k"], jets["eta"], jets["u"], jets["g"]
    aoff, boff = offset.x, offset.y
    radius2 = aoff * aoff + boff * boff
    A = bounds.iadd((1.0, 1.0), bounds.iscale(k, -boff))
    B = bounds.iscale(k, aoff)
    D = bounds.iadd(bounds.imul(A, A), bounds.imul(B, B))
    if A[0] <= 0.0 or D[0] <= 0.0:
        raise _AuditFailure(
            "NORMAL_BRANCH_VIOLATION",
            "vehicle_id={} A_low={!r} D_low={!r}".format(
                offset.vehicle_id, A[0], D[0]),
            vehicle_id=offset.vehicle_id, constraint="NORMAL_BRANCH",
            A_low=A[0], D_low=D[0])
    Q = bounds.isqrt_positive(D)
    Q2 = bounds.isquare(Q)
    Q3 = bounds.imul(Q2, Q)
    D_s = bounds.iscale(
        bounds.imul(bounds.iadd(bounds.iscale(k, radius2),
                              (-boff, -boff)), eta), 2.0)
    D_ss = bounds.iscale(
        bounds.iadd(
            bounds.iscale(bounds.isquare(eta), radius2),
            bounds.imul(
                bounds.iadd(bounds.iscale(k, radius2), (-boff, -boff)), u)),
        2.0)
    Q_s = bounds.idiv_positive_interval(D_s, bounds.iscale(Q, 2.0))
    Q_ss = bounds.isub(
        bounds.idiv_positive_interval(D_ss, bounds.iscale(Q, 2.0)),
        bounds.idiv_positive_interval(
            bounds.isquare(D_s), bounds.iscale(Q3, 4.0)))
    H = bounds.iadd(k, bounds.idiv_positive_interval(
        bounds.iscale(eta, aoff), D))
    H_s = bounds.iadd(
        eta,
        bounds.iscale(
            bounds.isub(
                bounds.idiv_positive_interval(u, D),
                bounds.idiv_positive_interval(
                    bounds.imul(eta, D_s), bounds.isquare(D))),
            aoff))
    H_ss = bounds.iadd(
        u,
        bounds.iscale(
            bounds.iadd(
                bounds.isub(
                    bounds.idiv_positive_interval(g, D),
                    bounds.idiv_positive_interval(
                        bounds.iscale(bounds.imul(u, D_s), 2.0),
                        bounds.isquare(D))),
                bounds.isub(
                    bounds.iscale(bounds.idiv_positive_interval(
                        bounds.imul(eta, D_ss), bounds.isquare(D)), -1.0),
                    bounds.iscale(bounds.idiv_positive_interval(
                        bounds.imul(eta, bounds.isquare(D_s)),
                        bounds.imul(bounds.isquare(D), D)), -2.0))),
            aoff))
    C = bounds.idiv_positive_interval(H, Q)
    C_s = bounds.isub(
        bounds.idiv_positive_interval(H_s, Q),
        bounds.idiv_positive_interval(bounds.imul(H, Q_s), Q2))
    C_ss = bounds.iadd(
        bounds.isub(
            bounds.isub(
                bounds.idiv_positive_interval(H_ss, Q),
                bounds.idiv_positive_interval(
                    bounds.iscale(bounds.imul(H_s, Q_s), 2.0), Q2)),
            bounds.idiv_positive_interval(bounds.imul(H, Q_ss), Q2)),
        bounds.idiv_positive_interval(
            bounds.iscale(bounds.imul(H, bounds.isquare(Q_s)), 2.0),
            bounds.imul(Q2, Q)))
    k2 = bounds.isquare(k)
    k3 = bounds.imul(k2, k)
    z2 = (
        bounds.iadd(bounds.iscale(k2, -aoff), bounds.iscale(eta, -boff)),
        bounds.iadd(bounds.isub(k, bounds.iscale(k2, boff)),
                    bounds.iscale(eta, aoff)))
    z3 = (
        bounds.isub(
            bounds.iadd(
                bounds.isub(bounds.iscale(k3, boff), k2),
                bounds.iscale(bounds.imul(k, eta), -3.0 * aoff)),
            bounds.iscale(u, boff)),
        bounds.isub(
            bounds.iadd(
                bounds.isub(eta, bounds.iscale(bounds.imul(k, eta),
                                               3.0 * boff)),
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
        bounds._next_up(p_ss_norm * w2)
        + bounds._next_up(p_s_norm * a_abs))
    jerk = bounds._next_up(
        bounds._next_up(p_sss_norm * w3)
        + bounds._next_up(3.0 * p_ss_norm * w_abs * a_abs)
        + bounds._next_up(p_s_norm * j_abs))
    # Local per-component enclosures, used by the independent coverage
    # checks.  The scalar ``jerk`` remains the certified norm bound.
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
        + bounds._next_up(3.0 * bounds.imax_abs(H_s) * w_abs * a_abs)
        + bounds._next_up(bounds.imax_abs(H_ss) * w3))
    lateral = bounds._next_up(
        bounds.imax_abs(C) * bounds._next_up(v_abs * v_abs))
    m = bounds.iadd((1.0, 1.0), bounds.iscale(C, -yw))
    n = bounds.iscale(C, xw)
    Ew = bounds.iadd(bounds.isquare(m), bounds.isquare(n))
    if Ew[0] <= 0.0:
        raise _AuditFailure(
            "WHEEL_GEOMETRY_VIOLATION",
            "vehicle_id={} Ew_low={!r}".format(offset.vehicle_id, Ew[0]),
            vehicle_id=offset.vehicle_id, constraint="WHEEL_GEOMETRY",
            Ew_low=Ew[0])
    factor = bounds.isqrt_positive(Ew)
    Ew_C = bounds.iadd(
        (-2.0 * yw, -2.0 * yw),
        bounds.iscale(C, 2.0 * (xw * xw + yw * yw)))
    delta_s = bounds.iscale(
        bounds.idiv_positive_interval(C_s, Ew), xw)
    delta_ss = bounds.isub(
        bounds.iscale(bounds.idiv_positive_interval(C_ss, Ew), xw),
        bounds.iscale(
            bounds.idiv_positive_interval(
                bounds.iscale(bounds.imul(Ew_C, bounds.isquare(C_s)), xw),
                bounds.isquare(Ew)), 1.0))
    wheel_speed = bounds._next_up(bounds.imax_abs(factor) * v_abs)
    steer_rate = bounds._next_up(bounds.imax_abs(delta_s) * w_abs)
    steer_accel = bounds._next_up(
        bounds._next_up(bounds.imax_abs(delta_ss) * w2)
        + bounds._next_up(bounds.imax_abs(delta_s) * a_abs))
    return {
        "C_interval": C,
        "A_low": A[0], "D_low": D[0],
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
        "p_sss_local_interval": z3,
        "p_sss_norm_upper": p_sss_norm,
        "p_sss_component_upper": (
            bounds.imax_abs(z3[0]), bounds.imax_abs(z3[1])),
        "jerk_local_interval": jerk_components,
        "jerk_component_upper": (
            bounds.imax_abs(jerk_components[0]),
            bounds.imax_abs(jerk_components[1])),
        "jerk_component_norm_upper": bounds.inorm2_upper(jerk_components),
    }


def _audit_interval_leaf(piece, q0, q1, tau0, tau1, request, total_s,
                         duration):
    jets = bounds.segment_jet_bounds(
        piece.c0, piece.c1, piece.c2, piece.c3, piece.h, q0, q1)
    w, a, j = _audit_time_intervals(tau0, tau1, total_s, duration)
    records = {}
    for offset in request.model.offsets:
        wheels = {}
        C_interval = None
        for wheel_index, (xw_factor, yw_factor) in enumerate(
                _DYNAMIC_WHEELS):
            xw = xw_factor * request.limits.wheelbase
            yw = yw_factor * request.limits.track
            record = _audit_interval_vehicle(
                jets, offset, request.limits, w, a, j, xw, yw)
            if C_interval is None:
                C_interval = record["C_interval"]
            wheels[wheel_index] = record
        angle = _static_wheel_angle_interval(
            C_interval, request.limits.wheelbase, request.limits.track)
        records[offset.vehicle_id] = {
            "wheels": wheels, "wheel_angle_bound_rad": angle,
            "C_interval": C_interval}
    return {"vehicles": records, "w": w, "a": a, "j": j,
            "q0": q0, "q1": q1, "tau0": tau0, "tau1": tau1}


def _audit_actual_finite(value, name, vehicle_id, wheel_index):
    """Fail closed on a non-finite actual metric, with a JSON-safe location.

    ``NaN`` and both infinities are never legal witnesses: they cannot be
    compared with a limit and must not be skipped, replaced by zero, or
    published as a JSON number.  The detail describes the value as text so a
    failure report remains serialisable with ``allow_nan=False``.
    """
    try:
        finite = math.isfinite(value)
    except (TypeError, ValueError, OverflowError):
        finite = False
    if finite:
        return
    try:
        number = float(value)
        if math.isnan(number):
            description = "NaN"
        elif number == math.inf:
            description = "+Infinity"
        elif number == -math.inf:
            description = "-Infinity"
        else:
            description = "non-finite"
    except (TypeError, ValueError, OverflowError):
        description = "not a finite real ({})".format(type(value).__name__)
    raise _AuditFailure(
        "NONFINITE_AUDIT_VALUE",
        "vehicle_id={} wheel={} {} is non-finite ({})".format(
            vehicle_id, wheel_index, name, description),
        vehicle_id=vehicle_id, wheel_index=wheel_index,
        quantity=name, constraint="NONFINITE_AUDIT_VALUE")


def _audit_actual_metrics(dynamic, request):
    records = {}
    for vehicle_id, vehicle in dynamic.items():
        wheels = {}
        for wheel_index in range(4):
            wheel = {
                "speed": abs(vehicle["v_i"]),
                "acceleration": math.hypot(*vehicle["acceleration"]),
                "jerk": math.hypot(*vehicle["jerk"]),
                "yaw_rate": abs(vehicle["omega_i"]),
                "yaw_acceleration": abs(vehicle["yaw_accel"]),
                "yaw_jerk": abs(vehicle["yaw_jerk"]),
                "lateral_acceleration": abs(vehicle["lateral_accel"]),
                "wheel_speed": abs(vehicle["wheel_speeds"][wheel_index]),
                "steer_rate": abs(vehicle["steer_rates"][wheel_index]),
                "steer_acceleration": abs(vehicle["steer_accels"][wheel_index]),
            }
            wheels[wheel_index] = wheel
        # ``atan2`` maps a non-finite curvature to a finite wheel angle, so
        # validate the source quantity before forming the scalar.
        _audit_actual_finite(
            vehicle["C_i"], "C_i", vehicle_id, None)
        angle = _static_wheel_angle(
            float(vehicle["C_i"]), request.limits.wheelbase,
            request.limits.track)
        records[vehicle_id] = {
            "wheels": wheels, "wheel_angle_rad": angle}
    return records


def _audit_check_actual_metrics(actual, request):
    """Reject an actual query, reporting the worst-utilized vehicle/wheel.

    A single leaf can contain many vehicles and four wheels.  Reporting the
    first vehicle in dictionary order would make the diagnostic depend on
    ID ordering; the witness is therefore the largest value/limit ratio.
    Non-finite actual values are rejected immediately and never enter that
    comparison or a JSON-serialisable failure record.
    """
    worst = None
    for vehicle_id, vehicle in actual.items():
        wheels = vehicle["wheels"]
        for name, limit_name, order in _DYNAMIC_METRICS:
            limit = float(getattr(request.limits, limit_name))
            for wheel_index, record in wheels.items():
                value = record[name]
                _audit_actual_finite(
                    value, name, vehicle_id, wheel_index)
                if value <= limit:
                    continue
                utilization = math.inf if limit == 0.0 else value / limit
                if worst is None or utilization > worst[0]:
                    worst = (utilization, "DYNAMIC_LIMIT", vehicle_id,
                             wheel_index, name, value, limit)
        angle = vehicle["wheel_angle_rad"]
        _audit_actual_finite(angle, "wheel_angle", vehicle_id, None)
        angle_limit = float(request.limits.steer_angle_max)
        if angle > angle_limit:
            utilization = math.inf if angle_limit == 0.0 \
                else angle / angle_limit
            if worst is None or utilization > worst[0]:
                worst = (utilization, "WHEEL_ANGLE_LIMIT", vehicle_id,
                         None, "wheel_angle", angle, angle_limit)
    if worst is not None:
        _, reason, vehicle_id, wheel_index, name, value, limit = worst
        raise _AuditFailure(
            reason,
            "vehicle_id={} wheel={} {}={!r} exceeds {!r}".format(
                vehicle_id, wheel_index, name, value, limit),
            vehicle_id=vehicle_id, wheel_index=wheel_index,
            quantity=name, value=value, limit=limit,
            constraint=reason)
    return True


def _audit_pointwise_interval_check(dynamic, spatial, bounds_record, label):
    """Compare a pointwise evaluation with the enclosing interval certificate.

    This is deliberately a one-way containment check: the pointwise formulas
    are evaluated independently of the interval enclosure, and every actual
    jerk component/norm must lie below the corresponding local bound.  A
    failure means the interval certificate is under-estimating, which must
    fail closed rather than be published as ``DYNAMIC_FEASIBLE``.
    """
    maximum = {"jerk_ratio": 0.0, "component_ratio": 0.0,
               "p_sss_ratio": 0.0, "worst": None}
    for vehicle_id, vehicle in dynamic.items():
        wheel_records = bounds_record["vehicles"][vehicle_id]["wheels"]
        jerk_bound = min(record["jerk"] for record in wheel_records.values())
        component_x_bound = min(
            record["jerk_component_upper"][0]
            for record in wheel_records.values())
        component_y_bound = min(
            record["jerk_component_upper"][1]
            for record in wheel_records.values())
        p_sss_x_bound = min(
            record["p_sss_component_upper"][0]
            for record in wheel_records.values())
        p_sss_y_bound = min(
            record["p_sss_component_upper"][1]
            for record in wheel_records.values())
        actual_jerk_norm = math.hypot(*vehicle["jerk"])
        local_jerk = _audit_to_local(vehicle["jerk"], spatial["psi"])
        local_ps = _audit_to_local(
            spatial["vehicles"][vehicle_id]["p_sss"], spatial["psi"])
        values = (
            ("jerk", actual_jerk_norm, jerk_bound),
            ("jerk_x", abs(local_jerk[0]), component_x_bound),
            ("jerk_y", abs(local_jerk[1]), component_y_bound),
            ("p_sss_x", abs(local_ps[0]), p_sss_x_bound),
            ("p_sss_y", abs(local_ps[1]), p_sss_y_bound),
        )
        for quantity, actual, bound in values:
            if not (math.isfinite(actual) and math.isfinite(bound)):
                raise _AuditFailure(
                    "NONFINITE_BOUND",
                    "{} {} nonfinite actual/bound".format(label, quantity),
                    vehicle_id=vehicle_id, quantity=quantity,
                    constraint="DYNAMIC_INTERVAL")
            ratio = 0.0 if bound == 0.0 and actual == 0.0 else (
                math.inf if bound == 0.0 else actual / bound)
            if quantity == "jerk":
                maximum["jerk_ratio"] = max(maximum["jerk_ratio"], ratio)
            elif quantity.startswith("jerk_"):
                maximum["component_ratio"] = max(
                    maximum["component_ratio"], ratio)
            else:
                maximum["p_sss_ratio"] = max(
                    maximum["p_sss_ratio"], ratio)
            if actual > bound:
                raise _AuditFailure(
                    "BOUND_UNCONFIRMED",
                    "{} vehicle={} {} actual={!r} exceeds local bound "
                    "{!r} (ratio={!r})".format(
                        label, vehicle_id, quantity, actual, bound, ratio),
                    vehicle_id=vehicle_id, quantity=quantity,
                    value=actual, limit=bound,
                    constraint="DYNAMIC_INTERVAL")
            if maximum["worst"] is None or ratio > maximum["worst"][0]:
                maximum["worst"] = (
                    ratio, vehicle_id, quantity, actual, bound, label)
    return maximum


def _audit_pointwise_public(coverage):
    """JSON-safe summary of one pointwise-vs-interval containment check."""
    worst = coverage["worst"]
    return {
        "jerk_ratio": coverage["jerk_ratio"],
        "component_ratio": coverage["component_ratio"],
        "p_sss_ratio": coverage["p_sss_ratio"],
        "worst": None if worst is None else {
            "ratio": worst[0], "vehicle_id": worst[1],
            "quantity": worst[2], "actual": worst[3],
            "bound": worst[4], "label": worst[5]},
    }


def _audit_certificate(pieces, prefixes, start_qf, request, total_s,
                       duration, config, subintervals):
    """Certify every dynamic limit on the complete common-time domain."""
    subdivisions = int(config.interval_subdivisions_per_segment)
    leaves = 0
    whole = []
    total_utilization = {
        name: 0.0 for name, _limit_name, _order in _DYNAMIC_METRICS}
    max_wheel_angle = 0.0
    pointwise = {
        "leaf_count": 0, "vehicle_count": 0,
        "max_jerk_ratio": 0.0, "max_component_ratio": 0.0,
        "max_p_sss_ratio": 0.0, "worst": None,
    }
    for segment_index, piece in enumerate(pieces):
        metric_best = {
            name: {
                "upper_bound": -1.0, "limit": float(
                    getattr(request.limits, limit_name)),
                "order": order, "segment_index": segment_index,
                "vehicle_id": None, "wheel_index": None,
                "q0": None, "q1": None, "tau0": None, "tau1": None,
                "s0": None, "s1": None,
            }
            for name, limit_name, order in _DYNAMIC_METRICS}
        segment_best_angle = {
            "upper_bound": -1.0,
            "limit": float(request.limits.steer_angle_max),
            "segment_index": segment_index,
            "vehicle_id": None, "q0": None, "q1": None,
            "tau0": None, "tau1": None, "s0": None, "s1": None,
        }
        segment_leaves = 0
        segment_depth = 0
        stack = []
        for cell in range(subdivisions - 1, -1, -1):
            stack.append((cell / float(subdivisions),
                          (cell + 1) / float(subdivisions), 0))
        while stack:
            q0, q1, depth = stack.pop()
            leaves += 1
            segment_leaves += 1
            segment_depth = max(segment_depth, depth)
            if leaves > int(config.max_leaves):
                raise _AuditFailure(
                    "AUDIT_BUDGET_EXCEEDED",
                    "whole-domain leaves {} exceed {}".format(
                        leaves, config.max_leaves),
                    constraint="BOUND_LEAF_BUDGET", value=leaves,
                    limit=config.max_leaves, segment_index=segment_index)
            station0 = piece.s0 + piece.h * q0
            station1 = piece.s0 + piece.h * q1
            tau0 = _audit_tau_for_fraction(station0 / total_s)
            tau1 = _audit_tau_for_fraction(station1 / total_s)
            try:
                bounds_record = _audit_interval_leaf(
                    piece, q0, q1, tau0, tau1, request, total_s, duration)
            except _AuditFailure as interval_error:
                # A normal-branch/wheel-geometry failure in the interval
                # enclosure is checked against actual dynamics at the cell
                # midpoint.  If actual dynamics are legal, refine the cell;
                # only a real dynamic violation is a DYNAMIC_LIMIT.
                midpoint = 0.5 * (tau0 + tau1)
                spatial, dynamic = _audit_dynamic_at_time(
                    pieces, prefixes, start_qf, request,
                    midpoint * duration, "right", total_s, duration,
                    subintervals)
                actual_record = _audit_actual_metrics(dynamic, request)
                _audit_check_actual_metrics(actual_record, request)
                if depth >= int(config.max_depth):
                    raise _AuditFailure(
                        "BOUND_UNCONFIRMED",
                        "interval enclosure failed at depth {}: {}: {}".format(
                            depth, interval_error.reason,
                            interval_error.detail),
                        segment_index=segment_index,
                        vehicle_id=interval_error.location.get("vehicle_id"),
                        constraint="BOUND_UNCONFIRMED")
                middle = 0.5 * (q0 + q1)
                if middle <= q0 or middle >= q1:
                    raise _AuditFailure(
                        "BOUND_UNCONFIRMED",
                        "cell cannot be subdivided further",
                        segment_index=segment_index,
                        constraint="BOUND_UNCONFIRMED")
                stack.append((middle, q1, depth + 1))
                stack.append((q0, middle, depth + 1))
                continue
            failing_metric = None
            failing_vehicle = None
            failing_wheel = None
            failing_upper = None
            failing_limit = None
            for vehicle_id, vehicle in bounds_record["vehicles"].items():
                for wheel_index, wheel in vehicle["wheels"].items():
                    for name, limit_name, order in _DYNAMIC_METRICS:
                        upper = wheel[name]
                        limit = float(getattr(request.limits, limit_name))
                        if not math.isfinite(upper):
                            raise _AuditFailure(
                                "NONFINITE_BOUND",
                                "{} nonfinite bound".format(name),
                                vehicle_id=vehicle_id,
                                wheel_index=wheel_index,
                                segment_index=segment_index,
                                quantity=name,
                                constraint="DYNAMIC_INTERVAL")
                        if upper > limit and failing_metric is None:
                            failing_metric = name
                            failing_vehicle = vehicle_id
                            failing_wheel = wheel_index
                            failing_upper = upper
                            failing_limit = limit
                angle_upper = vehicle["wheel_angle_bound_rad"]
                angle_limit = float(request.limits.steer_angle_max)
                if angle_upper > angle_limit and failing_metric is None:
                    failing_metric = "wheel_angle"
                    failing_vehicle = vehicle_id
                    failing_wheel = None
                    failing_upper = angle_upper
                    failing_limit = angle_limit
            if failing_metric is not None:
                # Distinguish a true over-limit (witness by independent
                # evaluation) from interval dependency; only true violations
                # are reported as DYNAMIC_LIMIT.
                midpoint = 0.5 * (tau0 + tau1)
                spatial, dynamic = _audit_dynamic_at_time(
                    pieces, prefixes, start_qf, request,
                    midpoint * duration, "right", total_s, duration,
                    subintervals)
                actual_record = _audit_actual_metrics(dynamic, request)
                _audit_check_actual_metrics(actual_record, request)
                if depth >= int(config.max_depth):
                    raise _AuditFailure(
                        "BOUND_UNCONFIRMED",
                        "{} bound {!r} > limit {!r} remains after depth {} "
                        "and no actual over-limit was found".format(
                            failing_metric, failing_upper, failing_limit,
                            depth),
                        vehicle_id=failing_vehicle,
                        wheel_index=failing_wheel,
                        segment_index=segment_index,
                        quantity=failing_metric,
                        value=failing_upper, limit=failing_limit,
                        constraint="BOUND_UNCONFIRMED")
                middle = 0.5 * (q0 + q1)
                if middle <= q0 or middle >= q1:
                    raise _AuditFailure(
                        "BOUND_UNCONFIRMED",
                        "cell cannot be subdivided further",
                        segment_index=segment_index,
                        constraint="BOUND_UNCONFIRMED")
                stack.append((middle, q1, depth + 1))
                stack.append((q0, middle, depth + 1))
                continue
            # Every upper bound in this leaf is within its limit.  Evaluate
            # the actual pointwise dynamics at the time-midpoint of the leaf
            # and require it to lie below the same local interval bound.
            # This is a coverage check of the certificate, not a replacement
            # for the whole-domain interval proof.
            midpoint = 0.5 * (tau0 + tau1)
            actual_spatial, actual_dynamic = _audit_dynamic_at_time(
                pieces, prefixes, start_qf, request,
                midpoint * duration, "right", total_s, duration,
                subintervals)
            actual_s = actual_spatial["s"]
            if not (station0 - 1e-12 <= actual_s <= station1 + 1e-12):
                raise _AuditFailure(
                    "BOUND_UNCONFIRMED",
                    "pointwise sample s={!r} escaped certificate cell "
                    "[{!r}, {!r}]".format(actual_s, station0, station1),
                    segment_index=segment_index,
                    constraint="DYNAMIC_INTERVAL")
            coverage = _audit_pointwise_interval_check(
                actual_dynamic, actual_spatial, bounds_record,
                "segment.{}[{:.6g},{:.6g}].point".format(
                    segment_index, q0, q1))
            pointwise["leaf_count"] += 1
            pointwise["vehicle_count"] += len(actual_dynamic)
            pointwise["max_jerk_ratio"] = max(
                pointwise["max_jerk_ratio"], coverage["jerk_ratio"])
            pointwise["max_component_ratio"] = max(
                pointwise["max_component_ratio"],
                coverage["component_ratio"])
            pointwise["max_p_sss_ratio"] = max(
                pointwise["max_p_sss_ratio"], coverage["p_sss_ratio"])
            if coverage["worst"] is not None and (
                    pointwise["worst"] is None or
                    coverage["worst"][0] > pointwise["worst"][0]):
                pointwise["worst"] = coverage["worst"]
            # Retain the maximum certificate value for the segment report.
            for vehicle_id, vehicle in bounds_record["vehicles"].items():
                for wheel_index, wheel in vehicle["wheels"].items():
                    for name, limit_name, order in _DYNAMIC_METRICS:
                        upper = wheel[name]
                        limit = float(getattr(request.limits, limit_name))
                        utilization = upper / limit
                        if utilization > total_utilization[name]:
                            total_utilization[name] = utilization
                        record = metric_best[name]
                        if upper > record["upper_bound"]:
                            record.update({
                                "upper_bound": upper,
                                "vehicle_id": vehicle_id,
                                "wheel_index": wheel_index,
                                "q0": q0, "q1": q1,
                                "tau0": tau0, "tau1": tau1,
                                "s0": station0, "s1": station1,
                            })
                angle_upper = vehicle["wheel_angle_bound_rad"]
                max_wheel_angle = max(max_wheel_angle, angle_upper)
                angle_limit = float(request.limits.steer_angle_max)
                total_utilization["wheel_angle"] = max(
                    total_utilization.get("wheel_angle", 0.0),
                    angle_upper / angle_limit)
                if angle_upper > segment_best_angle["upper_bound"]:
                    segment_best_angle.update({
                        "upper_bound": angle_upper,
                        "vehicle_id": vehicle_id,
                        "q0": q0, "q1": q1,
                        "tau0": tau0, "tau1": tau1,
                        "s0": station0, "s1": station1,
                    })
        if segment_leaves < 1:
            raise _AuditFailure(
                "AUDIT_INCOMPLETE",
                "segment {} was not covered".format(segment_index),
                segment_index=segment_index,
                constraint="WHOLE_SEGMENT_COVERAGE")
        whole.append({
            "segment_index": segment_index,
            "s0": piece.s0, "s1": piece.s1, "h": piece.h,
            "leaves": segment_leaves, "max_depth": segment_depth,
            "certified": True,
            "metrics": {name: dict(record)
                        for name, record in metric_best.items()},
            "wheel_angle": dict(segment_best_angle),
        })
    worst = pointwise["worst"]
    if worst is not None:
        pointwise["worst"] = {
            "ratio": worst[0], "vehicle_id": worst[1],
            "quantity": worst[2], "actual": worst[3],
            "bound": worst[4], "label": worst[5]}
    return {
        "whole_segment_bounds": tuple(whole),
        "leaves": leaves,
        "max_utilization": total_utilization,
        "max_wheel_angle_rad": max_wheel_angle,
        "pointwise_interval_checks": pointwise,
    }


def _audit_no_self_digest(value, path="model"):
    if isinstance(value, dict):
        for key, child in value.items():
            if key == "model_sha256" and path != "model.planning_context":
                raise _AuditFailure(
                    "SELF_REFERENTIAL_MODEL",
                    "model embeds model_sha256 at {}.{}".format(path, key),
                    constraint="MODEL_DIGEST")
            _audit_no_self_digest(child, "{}.{}".format(path, key))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _audit_no_self_digest(child, "{}[{}]".format(path, index))


def _audit_bind_plan(request, plan, expected_planning_context):
    """Bind a timed plan to the original request and to its own model core."""
    from .trajectory import VirtualATimedPlan
    if not isinstance(plan, VirtualATimedPlan):
        raise _AuditFailure(
            "INVALID_PLAN", "expected VirtualATimedPlan",
            constraint="PLAN_TYPE")
    try:
        core = plan.to_model_dict()
        core_json = plan.to_model_json()
        reported_digest = plan.model_sha256()
    except Exception as exc:
        raise _AuditFailure(
            "INVALID_PLAN",
            "unable to read model core: {}: {}".format(
                type(exc).__name__, exc), constraint="PLAN_MODEL") from exc
    if not isinstance(core, dict) or set(core) != _MODEL_TOP_KEYS:
        raise _AuditFailure(
            "UNSUPPORTED_MODEL_FIELD",
            "model top-level keys={} expected={}".format(
                sorted(core) if isinstance(core, dict) else core,
                sorted(_MODEL_TOP_KEYS)),
            constraint="MODEL_JSON")
    _audit_no_self_digest(core)
    if core["backend"] != BACKEND or core["model_version"] != \
            TIMED_MODEL_VERSION or core["path_model_version"] != \
            PATH_MODEL_VERSION:
        raise _AuditFailure(
            "UNKNOWN_MODEL_VERSION",
            "backend={!r} model={!r} path={!r}".format(
                core["backend"], core["model_version"],
                core["path_model_version"]),
            constraint="MODEL_VERSION")
    expected_payload = request_payload(request)
    if core["request_inputs"] != expected_payload:
        raise _AuditFailure(
            "REQUEST_MISMATCH",
            "model request_inputs differ from the original request",
            constraint="REQUEST_BINDING")
    request_sha = request_fingerprint(request)
    if core["request_sha256"] != request_sha:
        raise _AuditFailure(
            "REQUEST_DIGEST_MISMATCH",
            "model request_sha256 differs from the original request",
            constraint="REQUEST_DIGEST")
    try:
        canonical = _audit_canonical_json(core)
        recomputed_digest = hashlib.sha256(
            canonical.encode("utf-8")).hexdigest()
    except (TypeError, ValueError) as exc:
        raise _AuditFailure(
            "INVALID_MODEL_JSON",
            "model core is not finite canonical JSON: {}".format(exc),
            constraint="MODEL_JSON") from exc
    if canonical != core_json:
        raise _AuditFailure(
            "INVALID_MODEL_JSON",
            "model core JSON is not canonical sort-key JSON",
            constraint="MODEL_JSON")
    if recomputed_digest != reported_digest:
        raise _AuditFailure(
            "MODEL_DIGEST_MISMATCH",
            "recomputed model digest differs from plan.model_sha256()",
            constraint="MODEL_DIGEST")
    context = core["planning_context"]
    if expected_planning_context is None:
        if context is not None:
            raise _AuditFailure(
                "PLANNING_CONTEXT_UNVERIFIED",
                "plan carries a planning context but audit got no expected "
                "context",
                constraint="PLANNING_CONTEXT")
    else:
        try:
            expected_context = json.loads(_audit_canonical_json(
                expected_planning_context))
        except (TypeError, ValueError) as exc:
            raise _AuditFailure(
                "INVALID_PLANNING_CONTEXT",
                "expected planning context is not finite JSON: {}".format(exc),
                constraint="PLANNING_CONTEXT") from exc
        if context != expected_context:
            raise _AuditFailure(
                "PLANNING_CONTEXT_MISMATCH",
                "plan planning_context differs from expected context",
                constraint="PLANNING_CONTEXT")
        if isinstance(expected_context, dict):
            for key in ("request_sha256", "request_sha"):
                if key in expected_context and \
                        expected_context[key] != request_sha:
                    raise _AuditFailure(
                        "PLANNING_CONTEXT_MISMATCH",
                        "planning context {} differs from request digest".format(
                            key),
                        constraint="PLANNING_CONTEXT")
    path_block = core["path"]
    if not isinstance(path_block, dict) or \
            set(path_block) != _MODEL_PATH_KEYS:
        raise _AuditFailure(
            "UNSUPPORTED_MODEL_FIELD", "path model keys are invalid",
            constraint="MODEL_JSON")
    if path_block["model_version"] != PATH_MODEL_VERSION:
        raise _AuditFailure(
            "UNKNOWN_PATH_VERSION",
            "path model_version={!r}".format(path_block["model_version"]),
            constraint="MODEL_VERSION")
    if not isinstance(path_block["start_qf"], list) or \
            len(path_block["start_qf"]) != 3:
        raise _AuditFailure(
            "INVALID_MODEL_JSON", "path start_qf must have three values",
            constraint="MODEL_JSON")
    start_qf = FormationPose(*[
        _audit_core_float(value, "path.start_qf[{}]".format(index))
        for index, value in enumerate(path_block["start_qf"])])
    if start_qf != request.start_qf:
        raise _AuditFailure(
            "START_MISMATCH", "path start_qf differs from request start_qf",
            constraint="START_QF")
    timing = core["timing"]
    if not isinstance(timing, dict) or set(timing) != _MODEL_TIMING_KEYS:
        raise _AuditFailure(
            "UNSUPPORTED_MODEL_FIELD", "timing model keys are invalid",
            constraint="MODEL_JSON")
    if timing["law"] != TIMING_LAW_VERSION:
        raise _AuditFailure(
            "UNKNOWN_TIMING_LAW", "timing law={!r}".format(timing["law"]),
            constraint="TIMING_LAW")
    if timing["kind"] not in ("motion", "hold"):
        raise _AuditFailure(
            "INVALID_MODEL_JSON", "timing kind={!r}".format(timing["kind"]),
            constraint="MODEL_JSON")
    if timing["kind"] != path_block["kind"]:
        raise _AuditFailure(
            "MODEL_TIMING_INCONSISTENT",
            "path kind={!r} timing kind={!r}".format(
                path_block["kind"], timing["kind"]),
            constraint="TIMING_KIND")
    total_s = _audit_core_float(timing["S"], "timing.S")
    duration = _audit_core_float(timing["T_s"], "timing.T_s")
    if plan.kind != timing["kind"] or \
            _finite_audit(plan.total_S_m, "plan.total_S_m") != total_s or \
            _finite_audit(plan.duration_s, "plan.duration_s") != duration:
        raise _AuditFailure(
            "MODEL_TIMING_INCONSISTENT",
            "public plan S/T/kind differ from model core",
            constraint="MODEL_TIMING")
    if not isinstance(plan.path, VirtualAPath):
        raise _AuditFailure(
            "INVALID_PATH", "plan.path is not a VirtualAPath",
            constraint="PLAN_PATH")
    path = plan.path
    if path.start_qf != start_qf:
        raise _AuditFailure(
            "MODEL_PATH_MISMATCH", "plan path start differs from model",
            constraint="PLAN_PATH")
    if _finite_audit(path.length_m, "plan.path.length_m") != total_s or \
            _finite_audit(
                path_block["length_m"], "model.path.length_m") != total_s:
        raise _AuditFailure(
            "MODEL_PATH_MISMATCH", "plan path length differs from model",
            constraint="PLAN_PATH")
    if _finite_audit(path.position_error_bound_m,
                     "path.position_error_bound_m") != _audit_core_float(
            path_block["position_error_bound_m"],
            "model.position_error_bound_m") or \
            _finite_audit(path.angle_error_bound_rad,
                          "path.angle_error_bound_rad") != _audit_core_float(
            path_block["angle_error_bound_rad"], "model.angle_error_bound_rad"):
        raise _AuditFailure(
            "MODEL_NUMERICS_MISMATCH",
            "plan path budgets differ from the model core",
            constraint="MODEL_NUMERICS")
    try:
        path_budget = json.loads(_audit_canonical_json(
            path.integration_budget))
    except (TypeError, ValueError) as exc:
        raise _AuditFailure(
            "MODEL_NUMERICS_MISMATCH",
            "plan path integration budget is not finite JSON: {}".format(exc),
            constraint="MODEL_NUMERICS") from exc
    if path_budget != path_block["integration_budget"]:
        raise _AuditFailure(
            "MODEL_NUMERICS_MISMATCH",
            "plan path integration budget differs from the model core",
            constraint="MODEL_NUMERICS")
    numerics = core["numerics"]
    if not isinstance(numerics, dict) or set(numerics) != \
            _MODEL_NUMERICS_KEYS:
        raise _AuditFailure(
            "UNSUPPORTED_MODEL_FIELD", "numerics model keys are invalid",
            constraint="MODEL_JSON")
    if numerics["version"] != bounds.NUMERICS_VERSION:
        raise _AuditFailure(
            "UNKNOWN_NUMERICS_VERSION",
            "numerics version={!r}".format(numerics["version"]),
            constraint="MODEL_NUMERICS")
    expected_method = "motion" if timing["kind"] == "motion" else "hold"
    if numerics["integration_method"] != expected_method:
        raise _AuditFailure(
            "MODEL_NUMERICS_MISMATCH",
            "integration method={!r}".format(numerics["integration_method"]),
            constraint="MODEL_NUMERICS")
    if _audit_core_float(numerics["position_error_bound_m"],
                         "numerics.position_error_bound_m") != \
            _finite_audit(path.position_error_bound_m,
                          "path.position_error_bound_m") or \
            _audit_core_float(numerics["angle_error_bound_rad"],
                              "numerics.angle_error_bound_rad") != \
            _finite_audit(path.angle_error_bound_rad,
                          "path.angle_error_bound_rad"):
        raise _AuditFailure(
            "MODEL_NUMERICS_MISMATCH",
            "numerics budgets differ from the production path",
            constraint="MODEL_NUMERICS")
    pieces = None
    prefixes = None
    if timing["kind"] == "motion":
        profile_block = path_block["profile"]
        if not isinstance(profile_block, dict) or \
                set(profile_block) != _MODEL_PROFILE_KEYS:
            raise _AuditFailure(
                "UNSUPPORTED_MODEL_FIELD", "profile model keys are invalid",
                constraint="MODEL_JSON")
        if profile_block["kind"] != "hermite_c2" or \
                profile_block["numerics_version"] != bounds.NUMERICS_VERSION:
            raise _AuditFailure(
                "UNSUPPORTED_PROFILE",
                "profile kind/version={!r}/{!r}".format(
                    profile_block["kind"],
                    profile_block["numerics_version"]),
                constraint="PROFILE")
        if path.is_hold or path.profile is None:
            raise _AuditFailure(
                "MODEL_PATH_MISMATCH",
                "motion model requires a production curvature path",
                constraint="PLAN_PATH")
        profile = path.profile
        if profile.kind != "hermite_c2" or \
                profile.numerics_version != bounds.NUMERICS_VERSION:
            raise _AuditFailure(
                "UNSUPPORTED_PROFILE", "plan profile version differs",
                constraint="PROFILE")
        if _finite_audit(profile.length_m, "profile.length_m") != total_s or \
                _audit_core_float(
                    profile_block["length_m"], "profile.length_m") != total_s:
            raise _AuditFailure(
                "MODEL_PROFILE_LENGTH_MISMATCH",
                "profile length differs from timing S",
                constraint="PROFILE_LENGTH")
        knots = profile_block["knots"]
        if not isinstance(knots, list) or len(knots) < 2:
            raise _AuditFailure(
                "INVALID_MODEL_JSON", "profile knots are missing",
                constraint="MODEL_JSON")
        control_knots = []
        for index, knot in enumerate(knots):
            if not isinstance(knot, dict) or set(knot) != {
                    "s", "value", "derivative"}:
                raise _AuditFailure(
                    "UNSUPPORTED_MODEL_FIELD",
                    "profile knot {} keys are invalid".format(index),
                    constraint="MODEL_JSON")
            control_knots.append(ControlKnot(
                _audit_core_float(knot["s"], "knot.s"),
                _audit_core_float(knot["value"], "knot.value"),
                _audit_core_float(knot["derivative"], "knot.derivative")))
        actual_knots = tuple(profile.knots)
        if len(actual_knots) != len(control_knots):
            raise _AuditFailure(
                "MODEL_PATH_MISMATCH", "profile knot count differs",
                constraint="PROFILE")
        for index, (actual, expected) in enumerate(
                zip(actual_knots, control_knots)):
            if actual.s != expected.s or actual.value != expected.value or \
                    actual.derivative != expected.derivative:
                raise _AuditFailure(
                    "MODEL_PATH_MISMATCH",
                    "profile knot {} differs from model core".format(index),
                    constraint="PROFILE")
        pieces, stations, knot_values, knot_derivatives = \
            _independent_pieces(control_knots)
        if abs(stations[-1] - total_s) > 1e-12:
            raise _AuditFailure(
                "MODEL_PROFILE_LENGTH_MISMATCH",
                "raw knot end={!r} timing S={!r}".format(
                    stations[-1], total_s),
                constraint="PROFILE_LENGTH")
        prefixes = _psi_prefixes(pieces)
        if abs(knot_values[0]) > INITIAL_CURVATURE_TOLERANCE or \
                abs(knot_derivatives[0]) > INITIAL_CURVATURE_TOLERANCE:
            raise _AuditFailure(
                "UNSUPPORTED_INITIAL_STATE",
                "k0={!r} eta0={!r} must be zero".format(
                    knot_values[0], knot_derivatives[0]),
                constraint="INITIAL_CURVATURE")
        if not (total_s > 0.0 and duration > 0.0):
            raise _AuditFailure(
                "INVALID_DURATION", "motion needs S>0 and T>0",
                constraint="MODEL_TIMING")
    else:
        if path_block["profile"] is not None:
            raise _AuditFailure(
                "MODEL_PATH_MISMATCH", "hold path has a profile",
                constraint="PLAN_PATH")
        if not path.is_hold or path.profile is not None:
            raise _AuditFailure(
                "MODEL_PATH_MISMATCH",
                "hold model requires VirtualAPath.hold with profile=None",
                constraint="PLAN_PATH")
        if total_s != 0.0 or duration != 0.0:
            raise _AuditFailure(
                "MODEL_TIMING_INCONSISTENT", "hold needs S=T=0",
                constraint="MODEL_TIMING")
        if request.start_qf != request.goal_qf:
            raise _AuditFailure(
                "HOLD_GOAL_MISMATCH",
                "NO_MOTION requires exactly identical start and goal",
                constraint="HOLD_GOAL")
        stations = ()
    return {
        "core": core, "pieces": pieces, "prefixes": prefixes,
        "stations": stations, "start_qf": start_qf, "total_s": total_s,
        "duration": duration, "kind": timing["kind"],
        "request_sha": request_sha, "digest": reported_digest,
        "profile": path.profile if path.profile is not None else None,
        "path": path,
    }


def bind_timed_plan(request, plan, expected_planning_context=None):
    """Public, non-raising plan binding used by the G4 collision layer.

    A collision audit must not accept a timed model whose request, model core,
    or planning context was swapped after the dynamic audit.  This wrapper
    reuses the same strict binding code as :func:`audit_dynamics` but does not
    rerun the dynamic certificate.
    """
    try:
        bind = _audit_bind_plan(
            request, plan, expected_planning_context)
    except _AuditFailure as exc:
        return ValidationResult.fail(
            exc.reason, exc.detail, **exc.location)
    except Exception as exc:
        return ValidationResult.fail(
            "AUDIT_NUMERICAL_FAILURE",
            "binding raised {}: {}".format(type(exc).__name__, exc))
    context_sha = planning_context_sha256(expected_planning_context) \
        if expected_planning_context is not None else ""
    grid_sha = ""
    if isinstance(expected_planning_context, MappingABC):
        grid = expected_planning_context.get("grid")
        if grid is not None:
            try:
                grid_sha = grid_summary_sha256(grid)
            except (TypeError, ValueError):
                return ValidationResult.fail(
                    "INVALID_PLANNING_CONTEXT",
                    "context grid is not finite JSON")
    return ValidationResult.ok(
        kind=bind["kind"], model_sha256=bind["digest"],
        request_sha256=bind["request_sha"],
        planning_context_sha256=context_sha, grid_sha256=grid_sha,
        total_S_m=bind["total_s"], duration_s=bind["duration"])


def _audit_zero_dynamics(state, label):
    for vehicle in state.vehicles:
        for name in ("v_i", "omega_i", "yaw_accel", "lateral_accel"):
            value = abs(getattr(vehicle, name))
            if value > 1e-12:
                raise _AuditFailure(
                    "ENDPOINT_DYNAMICS_NONZERO",
                    "{}.vehicle={} {}={!r} should be zero".format(
                        label, vehicle.vehicle_id, name, getattr(vehicle, name)),
                    vehicle_id=vehicle.vehicle_id,
                    quantity=name, value=value,
                    constraint="ENDPOINT_ZERO")
        for name in ("velocity", "acceleration"):
            for component in getattr(vehicle, name):
                if abs(component) > 1e-12:
                    raise _AuditFailure(
                        "ENDPOINT_DYNAMICS_NONZERO",
                        "{}.vehicle={} {} is nonzero".format(
                            label, vehicle.vehicle_id, name),
                        vehicle_id=vehicle.vehicle_id,
                        quantity=name,
                        constraint="ENDPOINT_ZERO")
        for name in ("wheel_speeds", "steer_rates", "steer_accels"):
            maximum = max(abs(value) for value in getattr(vehicle, name))
            if maximum > 1e-12:
                raise _AuditFailure(
                    "ENDPOINT_DYNAMICS_NONZERO",
                    "{}.vehicle={} {}={!r} should be zero".format(
                        label, vehicle.vehicle_id, name, maximum),
                    vehicle_id=vehicle.vehicle_id,
                    quantity=name, value=maximum,
                    constraint="ENDPOINT_ZERO")


def _audit_compare_states(first, second, config, label):
    maximum = 0.0
    maximum = max(maximum, _audit_compare_scalar(
        first.s, second.s, label + ".s", config.continuity_tolerance))
    maximum = max(maximum, _audit_compare_scalar(
        first.qf.x, second.qf.x, label + ".qf.x",
        config.continuity_tolerance))
    maximum = max(maximum, _audit_compare_scalar(
        first.qf.y, second.qf.y, label + ".qf.y",
        config.continuity_tolerance))
    maximum = max(maximum, _audit_compare_scalar(
        _wrap_abs(first.qf.psi - second.qf.psi), 0.0,
        label + ".qf.psi", config.continuity_tolerance))
    first_vehicles = {v.vehicle_id: v for v in first.vehicles}
    second_vehicles = {v.vehicle_id: v for v in second.vehicles}
    if set(first_vehicles) != set(second_vehicles):
        raise _AuditFailure(
            "VEHICLE_ID_SET_MISMATCH", "{} vehicle sets differ".format(label),
            constraint="VEHICLE_ID_SET")
    for vehicle_id in sorted(first_vehicles):
        left = first_vehicles[vehicle_id]
        right = second_vehicles[vehicle_id]
        for name in ("x", "y", "C_i", "v_i", "omega_i", "yaw_accel",
                     "lateral_accel"):
            maximum = max(maximum, _audit_compare_scalar(
                getattr(left, name), getattr(right, name),
                "{}.{}.{}".format(label, vehicle_id, name),
                config.continuity_tolerance, vehicle_id=vehicle_id))
        maximum = max(maximum, _audit_compare_scalar(
            _wrap_abs(left.theta - right.theta), 0.0,
            "{}.{}.theta".format(label, vehicle_id),
            config.continuity_tolerance, vehicle_id=vehicle_id))
        for name in ("velocity", "acceleration", "wheel_angles",
                     "wheel_speeds", "steer_rates"):
            maximum = max(maximum, _audit_compare_vector(
                getattr(left, name), getattr(right, name),
                "{}.{}.{}".format(label, vehicle_id, name),
                config.continuity_tolerance, vehicle_id=vehicle_id))
    return maximum


def _audit_jump_bounds(vehicle, request, label):
    limits = request.limits
    checks = (
        ("jerk", math.hypot(*vehicle.jerk), limits.jerk_max),
        ("yaw_jerk", abs(vehicle.yaw_jerk), limits.yaw_jerk_max),
    )
    for name, value, limit in checks:
        if not math.isfinite(value) or value > float(limit):
            raise _AuditFailure(
                "DYNAMIC_LIMIT",
                "{}.vehicle={} one-sided {}={!r} exceeds {!r}".format(
                    label, vehicle.vehicle_id, name, value, limit),
                vehicle_id=vehicle.vehicle_id, quantity=name,
                value=value, limit=limit,
                constraint="DYNAMIC_LIMIT")
    maximum = max(abs(value) for value in vehicle.steer_accels)
    if not math.isfinite(maximum) or maximum > float(limits.steer_accel_max):
        raise _AuditFailure(
            "DYNAMIC_LIMIT",
            "{}.vehicle={} one-sided steer_acceleration={!r} exceeds {!r}".format(
                label, vehicle.vehicle_id, maximum, limits.steer_accel_max),
            vehicle_id=vehicle.vehicle_id, quantity="steer_acceleration",
            value=maximum, limit=limits.steer_accel_max,
            constraint="DYNAMIC_LIMIT")


def _audit_node_checks(bind, request, plan, config, subintervals):
    pieces = bind["pieces"]
    prefixes = bind["prefixes"]
    start_qf = bind["start_qf"]
    total_s = bind["total_s"]
    duration = bind["duration"]
    position_tolerance = max(
        config.state_tolerance,
        10.0 * bind["path"].position_error_bound_m + 1e-9)
    records = []
    max_state_error = 0.0
    max_continuity_error = 0.0
    max_jerk_jump = 0.0
    max_inverse_error = 0.0
    subdivisions = int(config.interval_subdivisions_per_segment)
    q_step = 1.0 / float(subdivisions)
    internal = pieces[:-1]
    for index, piece in enumerate(internal):
        station = piece.s1
        tau = _audit_tau_for_fraction(station / total_s)
        if not (0.0 < tau < 1.0):
            raise _AuditFailure(
                "INTERNAL_STOP_OR_ENDPOINT",
                "internal knot {} maps to tau={!r}".format(index, tau),
                segment_index=index, constraint="INTERNAL_MOTION")
        time_value = duration * tau
        # Node dynamics require the actual one-sided w/a/j, not the hold zeros.
        h1 = 30.0 * tau * tau * (1.0 - tau) ** 2
        h2 = 60.0 * tau - 180.0 * tau * tau + 120.0 * tau ** 3
        h3 = 60.0 - 360.0 * tau + 360.0 * tau * tau
        w = total_s * h1 / duration
        a = total_s * h2 / (duration * duration)
        j = total_s * h3 / (duration ** 3)
        if w <= 0.0:
            raise _AuditFailure(
                "INTERNAL_STOP_OR_ENDPOINT",
                "internal knot {} has w={!r}".format(index, w),
                segment_index=index, constraint="INTERNAL_MOTION")
        left_spatial, left_expected = _audit_dynamic_at_station(
            pieces, prefixes, start_qf, request, station, "left",
            w, a, j, subintervals)
        right_spatial, right_expected = _audit_dynamic_at_station(
            pieces, prefixes, start_qf, request, station, "right",
            w, a, j, subintervals)
        # Verify both one-sided node limits against the interval certificates
        # of the adjacent first/last cells.  The cell bounds are recomputed
        # here from the same production interval operator used by the
        # whole-domain certificate, while the node values above come from the
        # independent pointwise evaluator.
        left_q0 = 1.0 - q_step
        left_q1 = 1.0
        left_tau0 = _audit_tau_for_fraction(
            (piece.s0 + piece.h * left_q0) / total_s)
        left_bounds = _audit_interval_leaf(
            piece, left_q0, left_q1, left_tau0, tau,
            request, total_s, duration)
        right_piece = pieces[index + 1]
        right_q0 = 0.0
        right_q1 = q_step
        right_tau1 = _audit_tau_for_fraction(
            (right_piece.s0 + right_piece.h * right_q1) / total_s)
        right_bounds = _audit_interval_leaf(
            right_piece, right_q0, right_q1, tau, right_tau1,
            request, total_s, duration)
        left_pointwise = _audit_pointwise_interval_check(
            left_expected, left_spatial, left_bounds,
            "node.{}[left].point".format(index))
        right_pointwise = _audit_pointwise_interval_check(
            right_expected, right_spatial, right_bounds,
            "node.{}[right].point".format(index))
        actual_left = plan.state_at(time_value, "left")
        actual_right = plan.state_at(time_value, "right")
        actual_auto = plan.state_at(time_value)
        if actual_left.region != "motion" or \
                actual_right.region != "motion" or \
                actual_auto.region != "motion":
            raise _AuditFailure(
                "TIME_REGION_MISMATCH",
                "internal node {} is not reported as motion".format(index),
                segment_index=index, constraint="TIME_REGION")
        max_state_error = max(max_state_error, _audit_compare_state(
            actual_left, left_expected, left_spatial,
            station, config, "node.{}".format(index),
            position_tolerance=position_tolerance,
            segment_index=index, side="left"))
        max_state_error = max(max_state_error, _audit_compare_state(
            actual_right, right_expected, right_spatial,
            station, config, "node.{}".format(index),
            position_tolerance=position_tolerance,
            segment_index=index, side="right"))
        max_state_error = max(max_state_error, _audit_compare_state(
            actual_auto, right_expected, right_spatial,
            station, config, "node.{}".format(index) + ".auto",
            position_tolerance=position_tolerance,
            segment_index=index, side="auto"))
        max_continuity_error = max(max_continuity_error, _audit_compare_states(
            actual_left, actual_right, config,
            "node.{}".format(index)))
        max_state_error = max(max_state_error, _audit_compare_scalar(
            actual_left.w, w, "node.{}".format(index) + ".left.w",
            config.state_tolerance, segment_index=index))
        max_state_error = max(max_state_error, _audit_compare_scalar(
            actual_right.w, w, "node.{}".format(index) + ".right.w",
            config.state_tolerance, segment_index=index))
        for side_name, state in (("left", actual_left),
                                 ("right", actual_right)):
            for vehicle in state.vehicles:
                _audit_jump_bounds(
                    vehicle, request, "node.{}.{}".format(index, side_name))
        internal_jump = 0.0
        left_vehicles = {v.vehicle_id: v for v in actual_left.vehicles}
        right_vehicles = {v.vehicle_id: v for v in actual_right.vehicles}
        for vehicle_id in left_vehicles:
            left_vehicle = left_vehicles[vehicle_id]
            right_vehicle = right_vehicles[vehicle_id]
            internal_jump = max(
                internal_jump,
                math.hypot(left_vehicle.jerk[0] - right_vehicle.jerk[0],
                           left_vehicle.jerk[1] - right_vehicle.jerk[1]),
                abs(left_vehicle.yaw_jerk - right_vehicle.yaw_jerk),
                max(abs(a_value - b_value) for a_value, b_value in zip(
                    left_vehicle.steer_accels, right_vehicle.steer_accels)))
        max_jerk_jump = max(max_jerk_jump, internal_jump)
        inverse_value = plan.time_at_s(station)
        inverse_error = _finite_audit(
            abs(inverse_value - time_value), "node_inverse_error",
            segment_index=index)
        max_inverse_error = max(max_inverse_error, inverse_error)
        if inverse_error > config.time_inverse_tolerance_s:
            raise _AuditFailure(
                "TIME_INVERSE_MISMATCH",
                "node {} inverse error={!r} exceeds {!r}".format(
                    index, inverse_error, config.time_inverse_tolerance_s),
                segment_index=index, value=inverse_error,
                limit=config.time_inverse_tolerance_s,
                constraint="TIME_INVERSE")
        records.append({
            "segment_index": index, "s": station, "t": time_value,
            "tau": tau, "w": w, "left_jerk": [v.jerk for v in
                                               actual_left.vehicles],
            "right_jerk": [v.jerk for v in actual_right.vehicles],
            "pointwise_interval": {
                "left": _audit_pointwise_public(left_pointwise),
                "right": _audit_pointwise_public(right_pointwise)},
            "max_continuity_error": max_continuity_error,
            "jump_max": internal_jump,
            "inverse_error_s": inverse_error,
            "left_region": actual_left.region,
            "right_region": actual_right.region,
            "auto_region": actual_auto.region,
        })
    return {
        "records": tuple(records),
        "max_state_error": max_state_error,
        "max_continuity_error": max_continuity_error,
        "max_jump_magnitude": max_jerk_jump,
        "max_inverse_error_s": max_inverse_error,
        "internal_node_count": len(internal),
    }


def _audit_endpoint_checks(bind, request, plan, config, subintervals):
    pieces = bind["pieces"]
    prefixes = bind["prefixes"]
    start_qf = bind["start_qf"]
    total_s = bind["total_s"]
    duration = bind["duration"]
    position_tolerance = max(
        config.state_tolerance,
        10.0 * bind["path"].position_error_bound_m + 1e-9)
    initial_spatial, initial_hold = _audit_hold_expected(
        pieces, prefixes, start_qf, request, 0.0, "right", subintervals)
    terminal_spatial, terminal_hold = _audit_hold_expected(
        pieces, prefixes, start_qf, request, total_s, "left", subintervals)
    h3 = 60.0
    j_start = total_s * h3 / (duration ** 3)
    start_spatial, start_motion = _audit_dynamic_at_station(
        pieces, prefixes, start_qf, request, 0.0, "right",
        0.0, 0.0, j_start, subintervals)
    end_spatial, end_motion = _audit_dynamic_at_station(
        pieces, prefixes, start_qf, request, total_s, "left",
        0.0, 0.0, j_start, subintervals)
    maximum = 0.0
    checks = []

    def compare(actual, expected, spatial, station, label, tolerance):
        nonlocal maximum
        maximum = max(maximum, _audit_compare_state(
            actual, expected, spatial, station, config, label,
            position_tolerance=tolerance))

    for label, side, expected, spatial, station, instant in (
            ("t=0.left", "left", initial_hold, initial_spatial, 0.0, 0.0),
            ("t<0.left", "left", initial_hold, initial_spatial, 0.0, -1.0),
            ("t=0.auto", "auto", start_motion, start_spatial, 0.0, 0.0),
            ("t=T.left", "left", end_motion, end_spatial, total_s, duration),
            ("t=T.right", "right", terminal_hold, terminal_spatial, total_s,
             duration),
            ("t>T.right", "right", terminal_hold, terminal_spatial, total_s,
             duration + 1.0)):
        actual = plan.state_at(instant, side)
        compare(actual, expected, spatial, station, label, position_tolerance)
        checks.append({"label": label, "t": actual.t, "s": actual.s,
                       "region": actual.region})
    for label, station, side, expected, spatial, instant in (
            ("t=0.right", 0.0, "right", start_motion, start_spatial, 0.0),
            ("t=T.auto", total_s, "auto", end_motion, end_spatial, duration)):
        actual = plan.state_at(instant, side)
        compare(actual, expected, spatial, station, label, position_tolerance)
        checks.append({"label": label, "t": actual.t, "s": actual.s,
                       "region": actual.region})
    if abs(plan.time_at_s(0.0)) > config.time_inverse_tolerance_s or \
            abs(plan.time_at_s(total_s) - duration) > \
            config.time_inverse_tolerance_s:
        raise _AuditFailure(
            "TIME_INVERSE_MISMATCH", "endpoint inverse times differ",
            constraint="TIME_INVERSE")
    start_motion_state = plan.state_at(0.0, "right")
    end_motion_state = plan.state_at(duration, "left")
    _audit_zero_dynamics(start_motion_state, "motion start")
    _audit_zero_dynamics(end_motion_state, "motion terminal")
    # Start is the declared straight stationary state; terminal FREE must
    # preserve curvature/heading/wheel angle rather than zero them.
    for vehicle in start_motion_state.vehicles:
        for angle in vehicle.wheel_angles:
            if abs(angle) > 1e-12:
                raise _AuditFailure(
                    "INITIAL_WHEEL_NOT_STRAIGHT",
                    "vehicle={} start wheel angle={!r}".format(
                        vehicle.vehicle_id, angle),
                    vehicle_id=vehicle.vehicle_id,
                    quantity="wheel_angle", value=abs(angle),
                    constraint="INITIAL_STRAIGHT")
    for label, state, expected_spatial, expected_hold in (
            ("initial", plan.state_at(0.0, "left"), initial_spatial,
             initial_hold),
            ("terminal_hold", plan.state_at(duration, "right"),
             terminal_spatial, terminal_hold)):
        for vehicle, expected_vehicle in zip(state.vehicles, [
                expected_spatial["vehicles"][offset.vehicle_id]
                for offset in request.model.offsets]):
            error = _finite_audit(
                abs(vehicle.C_i - expected_vehicle["C"]),
                "hold_curvature_error", vehicle_id=vehicle.vehicle_id)
            if error > config.state_tolerance:
                raise _AuditFailure(
                    "HOLD_CURVATURE_MISMATCH",
                    "{} curvature changed in hold: {!r}".format(
                        label, error),
                    vehicle_id=vehicle.vehicle_id, value=error,
                    constraint="HOLD_CURVATURE")
            for actual_angle, expected_angle in zip(
                    vehicle.wheel_angles,
                    expected_hold[vehicle.vehicle_id]["wheel_angles"]):
                error = _finite_audit(
                    abs(actual_angle - expected_angle),
                    "hold_wheel_angle_error",
                    vehicle_id=vehicle.vehicle_id)
                if error > config.state_tolerance:
                    raise _AuditFailure(
                        "HOLD_WHEEL_ANGLE_MISMATCH",
                        "{} wheel angle changed in hold: {!r}".format(
                            label, error),
                        vehicle_id=vehicle.vehicle_id, value=error,
                        constraint="HOLD_WHEEL_ANGLE")
    # One-sided endpoint jerk is deliberately not required to be zero.
    for vehicle in start_motion_state.vehicles:
        for component in vehicle.jerk:
            if not math.isfinite(component):
                raise _AuditFailure(
                    "ENDPOINT_JERK_NONFINITE", "start jerk is non-finite",
                    constraint="ENDPOINT_JERK")
    return {
        "checks": tuple(checks),
        "max_state_error": maximum,
        "initial_region": plan.state_at(-1.0, "left").region,
        "terminal_region": plan.state_at(duration + 1.0, "right").region,
        "jerk_semantics": "one-sided endpoint jerk retained, not required zero",
    }


def _audit_time_inverse(bind, plan, request, config):
    total_s = bind["total_s"]
    duration = bind["duration"]
    taus = [0.5 - 0.5 * math.cos(math.pi * (index + 0.5) / 100.0)
            for index in range(100)]
    maximum = 0.0
    independent_maximum = 0.0
    samples = []
    for tau in taus:
        if not (0.0 < tau < 1.0):
            continue
        progress = _audit_progress(tau)
        station = total_s * progress
        # The contract is time_at_s(s), so the expected time is the inverse
        # of the rounded station actually passed to the plan.  Near an
        # endpoint a 1-ulp change in s can move the inverse time by much more
        # than 1e-9 s; the independent inverse of the same s is the correct
        # comparison, and the source-tau consistency is reported separately.
        independent_tau = _audit_tau_for_fraction(station / total_s)
        expected_time = duration * independent_tau
        actual_time = plan.time_at_s(station)
        error = _finite_audit(abs(actual_time - expected_time),
                              "time_inverse_error", s=station)
        source_time = duration * tau
        source_error = _finite_audit(
            abs(actual_time - source_time),
            "time_inverse_source_error", s=station)
        maximum = max(maximum, error)
        independent_maximum = max(independent_maximum, source_error)
        samples.append({
            "tau": tau, "s": station, "t": expected_time,
            "source_t": source_time,
            "time_at_s": actual_time, "error_s": error,
            "source_error_s": source_error,
        })
        if error > config.time_inverse_tolerance_s:
            raise _AuditFailure(
                "TIME_INVERSE_MISMATCH",
                "time_at_s({!r})={!r} expected={!r} error={!r}".format(
                    station, actual_time, expected_time, error),
                s=station, value=error,
                limit=config.time_inverse_tolerance_s,
                constraint="TIME_INVERSE")
    for station, expected in ((0.0, 0.0), (total_s, duration)):
        value = plan.time_at_s(station)
        error = _finite_audit(abs(value - expected), "inverse_endpoint_error")
        maximum = max(maximum, error)
        if error > config.time_inverse_tolerance_s:
            raise _AuditFailure(
                "TIME_INVERSE_MISMATCH",
                "endpoint time_at_s({!r})={!r} expected={!r}".format(
                    station, value, expected),
                s=station, constraint="TIME_INVERSE")
    for invalid in (-1e-6, total_s + 1e-6):
        try:
            plan.time_at_s(invalid)
        except Exception:
            pass
        else:
            raise _AuditFailure(
                "INVALID_INVERSE_ACCEPTED",
                "time_at_s accepted s={!r} outside the domain".format(invalid),
                s=invalid, constraint="TIME_INVERSE_DOMAIN")
    return {
        "sample_count": len(samples), "samples": samples,
        "max_error_s": maximum,
        "max_independent_error_s": independent_maximum,
        "tolerance_s": config.time_inverse_tolerance_s,
    }


def _audit_oracle_grid(pieces, oracle_step_m):
    grid = []
    for piece in pieces:
        count = max(2, int(math.ceil(piece.h / oracle_step_m)))
        if count % 2:
            count += 1
        grid.append(count)
    return tuple(grid)


def _audit_derivative_stencils(values, dt):
    f_m2, f_m1, f_0, f_p1, f_p2 = values
    first = (f_m2 - 8.0 * f_m1 + 8.0 * f_p1 - f_p2) / (12.0 * dt)
    second = (-f_m2 + 16.0 * f_m1 - 30.0 * f_0
              + 16.0 * f_p1 - f_p2) / (12.0 * dt * dt)
    third = (-f_m2 + 2.0 * f_m1 - 2.0 * f_p1 + f_p2) / (2.0 * dt ** 3)
    return first, second, third


def _audit_differential(bind, request, plan, config, oracle_grid):
    pieces = bind["pieces"]
    prefixes = bind["prefixes"]
    start_qf = bind["start_qf"]
    total_s = bind["total_s"]
    duration = bind["duration"]
    station_list = []
    for piece in pieces:
        for fraction in (0.2, 0.35, 0.5, 0.65, 0.8):
            station = piece.s0 + piece.h * fraction
            if piece.s0 < station < piece.s1:
                station_list.append((piece, station))
    results = []
    maxima = {}
    skipped = 0
    checked = 0
    for dt in tuple(float(value) for value in config.differential_dt_s):
        records = []
        for piece, station in station_list:
            tau = _audit_tau_for_fraction(station / total_s)
            center_time = duration * tau
            times = [center_time + offset * dt
                     for offset in (-2, -1, 0, 1, 2)]
            if times[0] <= 0.0 or times[-1] >= duration:
                skipped += 1
                continue
            progress_values = [
                _audit_progress(value / duration) for value in times]
            stations = [total_s * value for value in progress_values]
            if not (piece.s0 < min(stations) and max(stations) < piece.s1):
                skipped += 1
                continue
            try:
                samples = [
                    _audit_dynamic_at_time(
                        pieces, prefixes, start_qf, request, value,
                        "right", total_s, duration, oracle_grid)
                    for value in times]
            except Exception as exc:
                raise _AuditFailure(
                    "DIFFERENTIAL_ORACLE_FAILURE",
                    "{}: {}".format(type(exc).__name__, exc),
                    s=station, constraint="DIFFERENTIAL") from exc
            actual = plan.state_at(center_time)
            actual_vehicles = {v.vehicle_id: v for v in actual.vehicles}
            headings = {}
            for offset in request.model.offsets:
                vehicle_id = offset.vehicle_id
                center_heading = samples[2][0]["vehicles"][vehicle_id]["theta"]
                headings[vehicle_id] = [
                    center_heading + _wrap(
                        sample[0]["vehicles"][vehicle_id]["theta"]
                        - center_heading)
                    for sample in samples]
            record = {
                "dt_s": dt, "s": station, "t": center_time,
            }
            for vehicle_id in sorted(actual_vehicles):
                actual_vehicle = actual_vehicles[vehicle_id]
                position_x = [sample[0]["vehicles"][vehicle_id]["x"]
                              for sample in samples]
                position_y = [sample[0]["vehicles"][vehicle_id]["y"]
                              for sample in samples]
                derivative_x = _audit_derivative_stencils(position_x, dt)
                derivative_y = _audit_derivative_stencils(position_y, dt)
                heading_derivatives = _audit_derivative_stencils(
                    headings[vehicle_id], dt)
                velocity = (derivative_x[0], derivative_y[0])
                acceleration = (derivative_x[1], derivative_y[1])
                jerk = (derivative_x[2], derivative_y[2])
                velocity_error = max(
                    abs(velocity[0] - actual_vehicle.velocity[0]),
                    abs(velocity[1] - actual_vehicle.velocity[1]))
                acceleration_error = max(
                    abs(acceleration[0] - actual_vehicle.acceleration[0]),
                    abs(acceleration[1] - actual_vehicle.acceleration[1]))
                jerk_error = max(
                    abs(jerk[0] - actual_vehicle.jerk[0]),
                    abs(jerk[1] - actual_vehicle.jerk[1]))
                angular_rate_error = abs(
                    heading_derivatives[0] - actual_vehicle.omega_i)
                angular_accel_error = abs(
                    heading_derivatives[1] - actual_vehicle.yaw_accel)
                angular_jerk_error = abs(
                    heading_derivatives[2] - actual_vehicle.yaw_jerk)
                for name, value, tolerance in (
                        ("velocity", velocity_error,
                         config.velocity_tolerance),
                        ("acceleration", acceleration_error,
                         config.acceleration_tolerance),
                        ("jerk", jerk_error, config.jerk_tolerance),
                        ("angular_rate", angular_rate_error,
                         config.angular_rate_tolerance),
                        ("angular_accel", angular_accel_error,
                         config.angular_accel_tolerance),
                        ("angular_jerk", angular_jerk_error,
                         config.angular_jerk_tolerance)):
                    maxima[name] = max(maxima.get(name, 0.0), value)
                    if not math.isfinite(value) or value > tolerance:
                        raise _AuditFailure(
                            "DIFFERENTIAL_MISMATCH",
                            "dt={} vehicle={} {} error={!r} tolerance={!r} "
                            "at s={!r}".format(
                                dt, vehicle_id, name, value, tolerance,
                                station),
                            vehicle_id=vehicle_id, quantity=name,
                            value=value, limit=tolerance, s=station,
                            dt_s=dt, constraint="DIFFERENTIAL")
                record.setdefault("vehicles", {})[vehicle_id] = {
                    "velocity_error": velocity_error,
                    "acceleration_error": acceleration_error,
                    "jerk_error": jerk_error,
                    "angular_rate_error": angular_rate_error,
                    "angular_accel_error": angular_accel_error,
                    "angular_jerk_error": angular_jerk_error,
                }
            record["third_difference_amplification"] = 1.0 / (2.0 * dt ** 3)
            record["position_budget_amplified_m"] = (
                bind["path"].position_error_bound_m
                * record["third_difference_amplification"])
            record["truncation_orders"] = {
                "velocity": 4, "acceleration": 4, "jerk": 2}
            records.append(record)
            checked += 1
        results.append({
            "dt_s": dt, "samples": tuple(records),
            "sample_count": len(records),
            "third_difference_amplification": 1.0 / (2.0 * dt ** 3),
        })
    return {
        "steps": tuple(results),
        "checked_sample_count": checked,
        "skipped_sample_count": skipped,
        "max_errors": dict(sorted(maxima.items())),
        "tolerances": {
            "velocity": config.velocity_tolerance,
            "acceleration": config.acceleration_tolerance,
            "jerk": config.jerk_tolerance,
            "angular_rate": config.angular_rate_tolerance,
            "angular_accel": config.angular_accel_tolerance,
            "angular_jerk": config.angular_jerk_tolerance,
        },
        "oracle": {
            "method": "independent_composite_simpson_positions",
            "oracle_step_m": config.oracle_step_m,
            "grid_subintervals": list(oracle_grid),
            "boundary_rule": (
                "stencils that cross a Hermite g-jump or the time-domain hold "
                "boundary are skipped; analytic one-sided limits are used "
                "instead"),
        },
    }


def _audit_hold_dynamics(bind, request, plan, config):
    start_qf = request.start_qf
    cosine, sine = math.cos(start_qf.psi), math.sin(start_qf.psi)
    spatial = {"x": start_qf.x, "y": start_qf.y, "psi": start_qf.psi,
               "s": 0.0, "vehicles": {}}
    for offset in request.model.offsets:
        spatial["vehicles"][offset.vehicle_id] = {
            "vehicle_id": offset.vehicle_id,
            "x": start_qf.x + cosine * offset.x - sine * offset.y,
            "y": start_qf.y + sine * offset.x + cosine * offset.y,
            "theta": start_qf.psi, "C": 0.0,
        }
    expected = {}
    for offset in request.model.offsets:
        vehicle = spatial["vehicles"][offset.vehicle_id]
        expected[offset.vehicle_id] = {
            "vehicle_id": offset.vehicle_id,
            "x": vehicle["x"], "y": vehicle["y"],
            "theta": vehicle["theta"], "C_i": 0.0,
            "v_i": 0.0, "omega_i": 0.0,
            "velocity": (0.0, 0.0), "acceleration": (0.0, 0.0),
            "jerk": (0.0, 0.0),
            "yaw_accel": 0.0, "yaw_jerk": 0.0, "lateral_accel": 0.0,
            "wheel_angles": (0.0, 0.0, 0.0, 0.0),
            "wheel_speeds": (0.0, 0.0, 0.0, 0.0),
            "steer_rates": (0.0, 0.0, 0.0, 0.0),
            "steer_accels": (0.0, 0.0, 0.0, 0.0),
        }
    maximum = 0.0
    checks = []
    for instant, side in ((-1.0, "left"), (0.0, "left"), (0.0, "right"),
                          (0.0, "auto"), (1.0, "right")):
        state = plan.state_at(instant, side)
        if state.mode != "A" or state.s != 0.0 or state.w != 0.0 or \
                state.a != 0.0 or state.j != 0.0:
            raise _AuditFailure(
                "HOLD_STATE_MISMATCH",
                "NO_MOTION state t={} side={} has s/w/a/j={!r}/{!r}/{!r}/{!r}".format(
                    instant, side, state.s, state.w, state.a, state.j),
                constraint="HOLD_STATE", t=instant, s=side)
        maximum = max(maximum, _audit_compare_state(
            state, expected, spatial, 0.0, config,
            "hold.t={}.{}".format(instant, side),
            position_tolerance=config.state_tolerance))
        _audit_zero_dynamics(state, "hold")
        checks.append({"t": instant, "side": side, "region": state.region})
    projection = plan.sample((0.0,))
    if len(projection.times) != 1 or projection.times != (0.0,) or \
            len(projection.states) != 1:
        raise _AuditFailure(
            "HOLD_SAMPLE_MISMATCH",
            "NO_MOTION sample must contain exactly t=0",
            constraint="HOLD_SAMPLE")
    for invalid in ((), (0.0, 1.0), (1.0,)):
        try:
            plan.sample(invalid)
        except Exception:
            pass
        else:
            raise _AuditFailure(
                "HOLD_SAMPLE_MISMATCH",
                "NO_MOTION accepted sample times={!r}".format(invalid),
                constraint="HOLD_SAMPLE")
    if plan.time_at_s(0.0) != 0.0:
        raise _AuditFailure(
            "HOLD_INVERSE_MISMATCH", "NO_MOTION time_at_s(0) != 0",
            constraint="HOLD_INVERSE")
    for invalid in (-1.0, 1e-12):
        try:
            plan.time_at_s(invalid)
        except Exception:
            pass
        else:
            raise _AuditFailure(
                "HOLD_INVERSE_MISMATCH",
                "NO_MOTION accepted s={!r}".format(invalid),
                constraint="HOLD_INVERSE")
    return {
        "checks": tuple(checks), "max_state_error": maximum,
        "sample_times": [0.0], "hold_model": "explicit_zero_duration",
        "inverse_division_free": True,
    }


def _audit_scoped_fail(reason, detail, request, plan, config, **kwargs):
    return AuditReport.fail(
        reason, detail, scope=DYNAMICS_SCOPE,
        request_summary=kwargs.pop("request_summary", {}),
        path_summary=kwargs.pop("path_summary", {}),
        limits_summary=kwargs.pop("limits_summary", {}),
        request_sha256=kwargs.pop("request_sha256", ""),
        check_count=kwargs.pop("check_count", 0),
        timing=kwargs.pop("timing", {}),
        **kwargs)


def audit_dynamics(request, plan, config=None, *,
                   expected_planning_context=None):
    """Independently audit the whole-path timed model.

    The audit derives every dynamic quantity from the raw profile knots and
    the declared T; it never uses the retimer's selection as a certificate and
    never trusts ``plan.state_at`` for the interval proof.  The production
    evaluator is only differentially compared afterwards.
    """
    if config is None:
        config = DynamicAuditConfig()
    request_value = request_summary(
        request) if isinstance(request, VirtualARequest) else {
            "type": type(request).__name__, "valid": False}
    path_value = {}
    if plan is not None and hasattr(plan, "path"):
        try:
            path_value = geometry_path_summary(plan.path)
        except Exception as exc:
            path_value = {"summary_error": "{}: {}".format(
                type(exc).__name__, exc)}
    limits_value = {}
    if isinstance(request, VirtualARequest):
        try:
            limits_value = request.limits.as_dict()
        except Exception:
            limits_value = {}
    request_sha = ""
    if isinstance(request, VirtualARequest):
        try:
            request_sha = request_fingerprint(request)
        except Exception:
            request_sha = ""
    context_sha = ""
    context_grid_sha = ""
    if isinstance(expected_planning_context, MappingABC):
        try:
            context_sha = planning_context_sha256(expected_planning_context)
            grid = expected_planning_context.get("grid")
            if grid is not None:
                context_grid_sha = grid_summary_sha256(grid)
        except (TypeError, ValueError):
            context_sha = ""
            context_grid_sha = ""
    config_validation = validate_dynamic_audit_config(config)
    if not config_validation.passed:
        return _audit_scoped_fail(
            "INVALID_AUDIT_CONFIG", config_validation.detail,
            request, plan, config, request_summary=request_value,
            path_summary=path_value, limits_summary=limits_value,
            request_sha256=request_sha,
            location=config_validation.location)
    if not isinstance(request, VirtualARequest):
        return _audit_scoped_fail(
            "INVALID_REQUEST", "expected VirtualARequest",
            request, plan, config, request_summary=request_value,
            path_summary=path_value, limits_summary=limits_value)
    request_validation = validate_virtual_a_request(request)
    if not request_validation.passed:
        return _audit_scoped_fail(
            request_validation.reason, request_validation.detail,
            request, plan, config, request_summary=request_value,
            path_summary=path_value, limits_summary=limits_value,
            location=request_validation.location, check_count=1,
            metrics={"request_validation": request_validation.metrics})
    try:
        bind = _audit_bind_plan(request, plan, expected_planning_context)
    except _AuditFailure as exc:
        return _audit_scoped_fail(
            exc.reason, exc.detail, request, plan, config,
            request_summary=request_value, path_summary=path_value,
            limits_summary=limits_value, request_sha256=request_sha,
            location=exc.location, check_count=1, metrics={"binding": False})
    except Exception as exc:
        return _audit_scoped_fail(
            "AUDIT_NUMERICAL_FAILURE",
            "binding raised {}: {}".format(type(exc).__name__, exc),
            request, plan, config, request_summary=request_value,
            path_summary=path_value, limits_summary=limits_value,
            request_sha256=request_sha,
            location={"constraint": "BINDING"},
            check_count=1, metrics={"binding": False})
    geometry = audit_geometry(
        request, plan.path, config.geometry_audit_config,
        planning_context=expected_planning_context)
    if not geometry.passed:
        return _audit_scoped_fail(
            "GEOMETRY_REJECTED",
            "independent geometry audit failed: {}: {}".format(
                geometry.reason, geometry.detail),
            request, plan, config,
            request_summary=request_value, path_summary=path_value,
            limits_summary=limits_value, request_sha256=request_sha,
            location=geometry.location, check_count=geometry.check_count,
            metrics={"geometry_audit": geometry.as_dict(),
                     "binding": True})
    try:
        if bind["kind"] == "hold":
            hold = _audit_hold_dynamics(bind, request, plan, config)
            return AuditReport.ok(
                scope=DYNAMICS_SCOPE,
                detail=(
                    "independent NO_MOTION hold audit; S=T=0; zero dynamics; "
                    "collision_checked=false; executable=false; no map/ROS"),
                request_summary=request_value,
                path_summary=path_value,
                limits_summary=limits_value,
                whole_segment_bounds=(),
                integration={"hold": hold},
                c2=geometry.c2,
                check_count=geometry.check_count + len(hold["checks"]) + 8,
                request_sha256=request_sha,
                planning_context_sha256=context_sha,
                grid_sha256=context_grid_sha,
                timing={
                    "law": TIMING_LAW_VERSION,
                    "kind": "hold", "S_m": 0.0, "T_s": 0.0,
                    "model_sha256": bind["digest"],
                    "retimer_selection_is_certificate": False,
                    "collision_checked": False, "executable": False,
                },
                metrics={
                    "kind": "hold", "motion": False,
                    "hold": hold,
                    "model_sha256": bind["digest"],
                    "collision_checked": False, "executable": False,
                    "map_used": False, "ros_used": False,
                })
        integration_plan = _integration_plan(
            bind["pieces"], bind["start_qf"].psi,
            config.geometry_audit_config)
        subintervals = integration_plan["subintervals"]
        certificate = _audit_certificate(
            bind["pieces"], bind["prefixes"], bind["start_qf"], request,
            bind["total_s"], bind["duration"], config, subintervals)
        nodes = _audit_node_checks(
            bind, request, plan, config, subintervals)
        endpoints = _audit_endpoint_checks(
            bind, request, plan, config, subintervals)
        inverse = _audit_time_inverse(bind, plan, request, config)
        oracle_grid = _audit_oracle_grid(
            bind["pieces"], config.oracle_step_m)
        differential = _audit_differential(
            bind, request, plan, config, oracle_grid)
        selection = bind["core"]["timing"]["selection"]
        check_count = (geometry.check_count + certificate["leaves"]
                       + certificate["pointwise_interval_checks"]["leaf_count"]
                       + nodes["internal_node_count"] * 4
                       + len(endpoints["checks"]) * 2
                       + inverse["sample_count"] + 2
                       + differential["checked_sample_count"] * 6)
        return AuditReport.ok(
            scope=DYNAMICS_SCOPE,
            detail=(
                "independent raw-profile common-time dynamic audit; "
                "whole-domain interval certificate plus node/endpoint/"
                "inverse/differential checks; DYNAMIC_FEASIBLE; "
                "collision_checked=false; executable=false; no map/ROS"),
            request_summary=request_value,
            path_summary=path_value,
            limits_summary=limits_value,
            whole_segment_bounds=certificate["whole_segment_bounds"],
            integration=differential,
            c2=geometry.c2,
            check_count=check_count,
            request_sha256=request_sha,
            planning_context_sha256=context_sha,
            grid_sha256=context_grid_sha,
            timing={
                "law": TIMING_LAW_VERSION,
                "kind": "motion",
                "S_m": bind["total_s"], "T_s": bind["duration"],
                "model_sha256": bind["digest"],
                "retimer_selection_is_certificate": False,
                "selection_hint": selection,
                "duration_origin": "model core selected duration",
                "collision_checked": False, "executable": False,
            },
            metrics={
                "kind": "motion", "motion": True,
                "leaves": certificate["leaves"],
                "max_utilization": certificate["max_utilization"],
                "max_wheel_angle_rad": certificate["max_wheel_angle_rad"],
                "pointwise_interval_checks":
                    certificate["pointwise_interval_checks"],
                "nodes": nodes,
                "endpoints": endpoints,
                "time_inverse": inverse,
                "differential": differential,
                "integration_plan": {
                    "subintervals": list(subintervals),
                    "total_budget_m":
                        integration_plan["budget"]["total_budget_m"],
                    "angle_bound_rad":
                        integration_plan["budget"]["angle_bound_rad"],
                    "method": _INTEGRATION_METHOD,
                },
                "model_sha256": bind["digest"],
                "whole_domain_certificate": True,
                "collision_checked": False, "executable": False,
                "map_used": False, "ros_used": False,
            })
    except _AuditFailure as exc:
        return _audit_scoped_fail(
            exc.reason, exc.detail, request, plan, config,
            request_summary=request_value, path_summary=path_value,
            limits_summary=limits_value, request_sha256=request_sha,
            location=exc.location, check_count=geometry.check_count,
            metrics={"binding": True, "dynamic_audit_passed": False})
    except Exception as exc:
        return _audit_scoped_fail(
            "AUDIT_NUMERICAL_FAILURE",
            "{}: {}".format(type(exc).__name__, exc),
            request, plan, config, request_summary=request_value,
            path_summary=path_value, limits_summary=limits_value,
            request_sha256=request_sha,
            location={"constraint": "DYNAMIC_AUDIT"},
            check_count=geometry.check_count,
            metrics={"binding": True, "dynamic_audit_passed": False})
