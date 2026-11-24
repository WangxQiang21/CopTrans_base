"""TASK03 Step 4 independent continuous F+OBB containment certificate.

The certificate works on the actual :class:`VirtualAPath` arc-length domain and
the immutable :class:`FormationCorridorTemplate`.  It never evaluates the
optimiser's finite row vector and never treats a collocation/cut station as a
certificate.  On every closed sub-interval it forms outward-rounded intervals
for the formation reference point and for every complete-OBB piece vertex,
then checks every assigned convex-region halfspace on that interval's upper
bound.  Required cuts are two-sided:

* every profile/Hermite piece boundary,
* every template segment boundary (the finite Portal cuts), and
* every allocation boundary.

A positive outward upper bound splits the interval; exceeding the depth,
width, interval count or wall clock limit returns ``UNCONFIRMED``.  A concrete
sample with a positive *lower* bound beyond the evaluation error is a
``VIOLATED`` witness.  Only ``VERIFIED`` means every outward upper residual is
``<= 0`` on the whole domain.

This module is the only continuous formation certificate.  The Step 3 finite
sample audit and the NLP rows are evidence, not certificates.
"""
from __future__ import annotations

import hashlib
import json
import math
import time

from . import bounds
from .contracts import (
    FORMATION_TOLERANCE_M,
    VirtualAContractError,
    request_fingerprint,
)
from .corridor import (
    audit_path_in_corridors,
    corridor_effective_margin,
    validate_corridor_allocation,
)
from .formation_corridor import (
    formation_model_sha256,
    validate_formation_template,
)
from .geometry import VirtualAPath


FORMATION_CERTIFICATE_SCHEMA = \
    "virtual_a_formation_continuous_certificate_v1"
FORMATION_CERTIFICATE_VERSION = \
    "virtual_a_formation_certificate.v1"
FORMATION_CERTIFICATE_VERIFIED = "VERIFIED"
FORMATION_CERTIFICATE_VIOLATED = "VIOLATED"
FORMATION_CERTIFICATE_UNCONFIRMED = "UNCONFIRMED"
FORMATION_CERTIFICATE_STATUSES = (
    FORMATION_CERTIFICATE_VERIFIED,
    FORMATION_CERTIFICATE_VIOLATED,
    FORMATION_CERTIFICATE_UNCONFIRMED,
)
FORMATION_CERTIFICATE_DEFAULT_MAX_DEPTH = 24
FORMATION_CERTIFICATE_DEFAULT_MAX_INTERVALS = 20000
FORMATION_CERTIFICATE_DEFAULT_MIN_WIDTH_M = 1e-5
FORMATION_CERTIFICATE_DEFAULT_MAX_SECONDS = 30.0
FORMATION_CERTIFICATE_DEFINITE_M = 1e-9
FORMATION_CERTIFICATE_EVALUATION_GUARD_M = 1e-12


def _canonical_sha256(value):
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _finite_float(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise VirtualAContractError(
            "INVALID_FORMATION_CERTIFICATE_INPUT",
            "{} is not real: {!r}".format(label, value)) from exc
    if not math.isfinite(result):
        raise VirtualAContractError(
            "INVALID_FORMATION_CERTIFICATE_INPUT",
            "{} is not finite: {!r}".format(label, value))
    return result


def _path_sha256(path, model_sha256):
    payload = {
        "kind": str(getattr(path, "kind", "")),
        "numerics_version": str(getattr(
            path, "numerics_version", "")),
        "model_sha256": str(model_sha256),
        "length_m": float(path.length_m),
        "start_qf": {
            "x": float(path.start_qf.x),
            "y": float(path.start_qf.y),
            "psi": float(path.start_qf.psi),
        },
        "profile": path.profile.as_dict(),
    }
    return _canonical_sha256(payload)


def _identity(path, request, chain, template):
    model_sha = formation_model_sha256(request)
    return {
        "request_sha256": request_fingerprint(request),
        "grid_sha256": chain.grid_sha256,
        "chain_digest": chain.digest,
        "model_sha256": model_sha,
        "planning_context_sha256": template.planning_context_sha256,
        "template_id": template.template_id,
        "allocation_digest": template.allocation_digest,
        "guide_sha256": template.guide_sha256,
        "path_sha256": _path_sha256(path, model_sha),
    }


def _identity_or_empty(request, chain, template):
    """Identity fields that do not require a valid path payload."""
    try:
        request_sha = request_fingerprint(request)
    except Exception:
        request_sha = ""
    return {
        "request_sha256": request_sha,
        "grid_sha256": str(getattr(chain, "grid_sha256", "")),
        "chain_digest": str(getattr(chain, "digest", "")),
        "model_sha256": str(getattr(template, "model_sha256", "")),
        "planning_context_sha256": str(getattr(
            template, "planning_context_sha256", "")),
        "template_id": str(getattr(template, "template_id", "")),
        "allocation_digest": str(getattr(template, "allocation_digest", "")),
        "guide_sha256": str(getattr(template, "guide_sha256", "")),
        "path_sha256": "",
    }


def _inflate(interval, delta):
    lower, upper = interval
    if not (math.isfinite(lower) and math.isfinite(upper)):
        raise ValueError("nonfinite_interval")
    delta = float(delta)
    if not math.isfinite(delta) or delta < 0.0:
        raise ValueError("invalid_inflation")
    return (bounds._next_down(lower - delta),
            bounds._next_up(upper + delta))


def _cos_sin_interval(angle_interval):
    """Outward cos/sin enclosure from the angle's Lipschitz constant 1."""
    lower, upper = angle_interval
    if not (math.isfinite(lower) and math.isfinite(upper)):
        raise ValueError("nonfinite_angle_interval")
    half = 0.5 * (upper - lower)
    if not math.isfinite(half) or half < 0.0:
        raise ValueError("invalid_angle_interval")
    middle = 0.5 * (lower + upper)
    if half >= 1.0:
        return (-1.0, 1.0), (-1.0, 1.0)
    cosine = math.cos(middle)
    sine = math.sin(middle)
    cos_interval = (
        bounds._next_down(max(-1.0, cosine - half)),
        bounds._next_up(min(1.0, cosine + half)))
    sin_interval = (
        bounds._next_down(max(-1.0, sine - half)),
        bounds._next_up(min(1.0, sine + half)))
    return cos_interval, sin_interval


def _sum_intervals(*intervals):
    total = intervals[0]
    for interval in intervals[1:]:
        total = bounds.iadd(total, interval)
    return total


def _scale(interval, scalar):
    return bounds.iscale(interval, float(scalar))


def _sub_limit(interval, limit):
    return bounds.isub(interval, (float(limit), float(limit)))


def _finite_or_none(value):
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _residual_interval(row, point_x, point_y, limit):
    return _sub_limit(
        _sum_intervals(_scale(point_x, row[0]), _scale(point_y, row[1])),
        limit)


def _heading_interval(psi, kappa, width):
    """Outward enclosure of the heading over one whole interval.

    For any ``s`` in ``[a, b]`` the increment ``psi(s) - psi(a)`` is the
    integral of ``kappa`` over ``[a, s]``, i.e. an element of the interval
    product ``[k_lo, k_hi] * [0, width]``.  The zero factor keeps the
    interval start inside the enclosure whenever the curvature has one
    sign; multiplying by the scalar ``width`` alone would drop it.
    """
    lower, upper = bounds.imul(
        (float(kappa[0]), float(kappa[1])), (0.0, float(width)))
    return bounds.iadd((float(psi), float(psi)), (lower, upper))


def _base_payload(status, *, reason, detail, identity, margin_m,
                  path_length_m, limits, **extra):
    payload = {
        "schema": FORMATION_CERTIFICATE_SCHEMA,
        "certificate_version": FORMATION_CERTIFICATE_VERSION,
        "role": "independent_continuous_f_plus_three_obb_certificate",
        "status": status,
        "reason": str(reason),
        "detail": str(detail),
        "is_continuous_certificate": True,
        "finite_sample_audit_is_certificate": False,
        "identity": identity,
        "request_sha256": identity["request_sha256"],
        "grid_sha256": identity["grid_sha256"],
        "chain_digest": identity["chain_digest"],
        "model_sha256": identity["model_sha256"],
        "planning_context_sha256": identity["planning_context_sha256"],
        "template_id": identity["template_id"],
        "allocation_digest": identity["allocation_digest"],
        "guide_sha256": identity["guide_sha256"],
        "path_sha256": identity["path_sha256"],
        "path_length_m": float(path_length_m),
        "margin_m": float(margin_m),
        "limits": dict(limits),
    }
    payload.update(extra)
    return payload


_SEAL_EXCLUDED_KEYS = ("elapsed_s",)


def _sealed_body(payload):
    return {key: value for key, value in payload.items()
            if key not in _SEAL_EXCLUDED_KEYS}


def _seal(payload):
    payload["certificate_sha256"] = _canonical_sha256(_sealed_body(payload))
    return payload


def _segment_sigma_bounds(segment, length):
    return (min(length, max(0.0, float(segment.sigma_start) * length)),
            min(length, max(0.0, float(segment.sigma_end) * length)))


def _dedupe_cuts(values, length):
    values = sorted(values)
    # A contractual template gap is at most FORMATION_TOLERANCE_M in sigma.
    # Merge it into one cut so no interval is classified inside an ambiguous
    # gap; the two-sided concrete cut check still visits the shared station.
    tolerance = max(
        1e-12, 4.0 * FORMATION_TOLERANCE_M * max(1.0, length))
    result = []
    for value in values:
        if not result or value - result[-1] > tolerance:
            result.append(value)
    result[0] = 0.0
    result[-1] = length
    return result


def _segments_at(template, sigma, tolerance):
    return [
        segment for segment in template.segments
        if segment.sigma_start - tolerance <= sigma
        <= segment.sigma_end + tolerance]


def _frames_by_id(template):
    return {frame.vehicle_id: frame for frame in template.vehicle_frames}


def _rotated_region_rows(region, margin):
    """World halfspaces with the template margin subtracted.

    The finite row plan rotates the same world halfspaces into the start-local
    frame and subtracts ``n.start`` from the limit.  Here the certificate
    stays in the world frame, which is an independent derivation of the same
    sufficient condition.
    """
    rows = tuple((float(row[0]), float(row[1])) for row in region.A)
    limits = tuple(
        float(limit) - float(margin) for limit in region.b)
    return rows, limits


def _point_corner(frame, local_vertex, center_x, center_y, theta):
    cosine = math.cos(theta)
    sine = math.sin(theta)
    local_x, local_y = local_vertex
    return (center_x + cosine * local_x - sine * local_y,
            center_y + sine * local_x + cosine * local_y)


def _point_error(frame, position_error_m, angle_error_rad):
    """Conservative concrete corner error.

    The formation position error moves the whole rigid body; the heading
    error rotates the vehicle centre about the formation centre by
    ``|r_i| * angle_error`` and every corner by another half-diagonal
    ``angle_error``.
    """
    half_diagonal = 0.5 * math.hypot(
        float(frame.length_m), float(frame.width_m))
    radius = math.hypot(float(frame.offset_x), float(frame.offset_y))
    return bounds._next_up(
        float(position_error_m)
        + (radius + half_diagonal) * float(angle_error_rad)
        + FORMATION_CERTIFICATE_EVALUATION_GUARD_M)


def _witness(evidence, **location):
    payload = dict(location)
    payload["evidence_m"] = float(evidence)
    return payload


_CLOSED = "closed"
_OPEN = "open"
_DEGENERATE = "degenerate"
_NONFINITE = "nonfinite"


def _check_interval(path, template, chain, allocation, a, b, depth,
                    margin_m, position_error_m, angle_error_rad,
                    frames, rotated, tolerance):
    """Classify one closed interval by interval arithmetic.

    Returns a tuple ``(outcome, record)``.  ``record`` contains the maximum
    outward upper residual, a possible definite witness, or the reason the
    interval could not be closed.
    """
    if not (math.isfinite(a) and math.isfinite(b) and a < b):
        return _NONFINITE, {"reason": "invalid_interval"}
    width = b - a
    middle_sigma = (0.5 * (a + b)) / float(path.length_m)
    segments = _segments_at(template, middle_sigma, tolerance)
    if not segments:
        return _DEGENERATE, {
            "reason": "no template segment at sigma={!r}".format(
                middle_sigma)}
    try:
        state = path.state_at_s(a, "right")
    except VirtualAContractError as exc:
        if exc.reason == "NORMAL_BRANCH_VIOLATION":
            return _DEGENERATE, {
                "reason": "normal branch fails at s={!r}: {}".format(a, exc)}
        return _NONFINITE, {"reason": str(exc)}
    try:
        jet_bounds = path.profile.interval_bounds(a, b)
    except Exception as exc:
        return _NONFINITE, {
            "reason": "profile interval bounds failed: {}: {}".format(
                type(exc).__name__, exc)}
    kappa = tuple(jet_bounds.get("k") or ())
    if len(kappa) != 2 or not all(math.isfinite(float(v)) for v in kappa):
        return _NONFINITE, {"reason": "nonfinite kappa interval"}
    kappa = (float(kappa[0]), float(kappa[1]))
    qf = state.qf
    # Arc-length speed is exactly one, so the reference point over [a, b]
    # lies within ``width`` of qf(a).  Each of qf(a) and qf(s) carries the
    # path's global integration error, hence the factor two.
    position_x = _inflate(
        (float(qf.x), float(qf.x)),
        float(position_error_m) * 2.0 + width)
    position_y = _inflate(
        (float(qf.y), float(qf.y)),
        float(position_error_m) * 2.0 + width)
    # For any s in [a, b], the heading increment is the integral of kappa
    # over [a, s], i.e. kappa evaluated on the interval product
    # [k_lo, k_hi] * [0, width].  Multiplying by the scalar width alone
    # would omit the zero increment whenever the curvature has one sign.
    psi_interval = _inflate(
        _heading_interval(qf.psi, kappa, width),
        float(angle_error_rad) * 2.0)
    max_upper = -math.inf
    open_records = []
    # The template segment owns the F reference-point region on this open
    # interval, exactly as the union NLP compiled its rows.  At a segment
    # boundary the midpoint selects both neighbouring assignments; checking
    # both regions is the two-sided F-region cut requirement.  The historical
    # allocation is retained only as the f_only comparison diagnostic.
    checked_f_regions = set()
    for segment in segments:
        region_index = int(segment.f_region_index)
        if region_index in checked_f_regions:
            continue
        checked_f_regions.add(region_index)
        if not (0 <= region_index < len(chain.regions)):
            return _NONFINITE, {"reason": "F region out of range"}
        rows, limits = rotated[region_index]
        for halfspace_id, (row, limit) in enumerate(zip(rows, limits)):
            residual = _residual_interval(
                row, position_x, position_y, limit)
            if residual[1] > max_upper:
                max_upper = residual[1]
            if residual[1] > 0.0:
                open_records.append({
                    "kind": "f", "region_index": int(region_index),
                    "template_segment_index": int(
                        template.segments.index(segment)),
                    "halfspace_id": int(halfspace_id),
                    "upper_m": float(residual[1]),
                })
    # Every complete-OBB piece vertex on every selected template segment.
    # Multiple selected segments can occur only within the contractual
    # tolerance of a shared boundary; checking the union is conservative.
    for segment in segments:
        for cover in segment.covers:
            frame = frames.get(cover.vehicle_id)
            if frame is None:
                return _NONFINITE, {"reason": "cover vehicle frame missing"}
            a_offset = float(frame.offset_x)
            b_offset = float(frame.offset_y)
            cos_psi, sin_psi = _cos_sin_interval(psi_interval)
            center_x = _sum_intervals(
                position_x,
                bounds.isub(_scale(cos_psi, a_offset),
                            _scale(sin_psi, b_offset)))
            center_y = _sum_intervals(
                position_y,
                bounds.iadd(_scale(sin_psi, a_offset),
                            _scale(cos_psi, b_offset)))
            forward = _sum_intervals((1.0, 1.0),
                                     _scale(kappa, -b_offset))
            lateral = _scale(kappa, a_offset)
            if forward[0] <= 0.0:
                return _DEGENERATE, {
                    "reason": "A interval is not strictly positive",
                    "vehicle_id": int(cover.vehicle_id),
                    "A_interval": [float(forward[0]), float(forward[1])]}
            denominator = _sum_intervals(
                bounds.isquare(forward), bounds.isquare(lateral))
            if denominator[0] <= 0.0:
                return _DEGENERATE, {
                    "reason": "D interval is not strictly positive",
                    "vehicle_id": int(cover.vehicle_id),
                    "D_interval": [float(denominator[0]),
                                   float(denominator[1])]}
            ratio = bounds.idiv_positive_interval(lateral, forward)
            phi = (bounds._next_down(math.atan(ratio[0])),
                   bounds._next_up(math.atan(ratio[1])))
            theta = bounds.iadd(psi_interval, phi)
            cos_theta, sin_theta = _cos_sin_interval(theta)
            for body_piece_index, piece in enumerate(cover.pieces):
                region_index = int(piece.region_index)
                if not (0 <= region_index < len(chain.regions)):
                    return _NONFINITE, {
                        "reason": "piece region out of range"}
                rows, limits = rotated[region_index]
                for vertex_id, local_vertex in enumerate(
                        piece.local_vertices):
                    vertex_x, vertex_y = local_vertex
                    corner_x = bounds.iadd(
                        center_x,
                        bounds.isub(_scale(cos_theta, vertex_x),
                                    _scale(sin_theta, vertex_y)))
                    corner_y = bounds.iadd(
                        center_y,
                        bounds.iadd(_scale(sin_theta, vertex_x),
                                    _scale(cos_theta, vertex_y)))
                    for halfspace_id, (row, limit) in enumerate(
                            zip(rows, limits)):
                        residual = _residual_interval(
                            row, corner_x, corner_y, limit)
                        if residual[1] > max_upper:
                            max_upper = residual[1]
                        if residual[1] > 0.0:
                            open_records.append({
                                "kind": "obb",
                                "vehicle_id": int(cover.vehicle_id),
                                "decomposition": str(cover.decomposition),
                                "body_piece_index": int(
                                    body_piece_index),
                                "region_index": int(region_index),
                                "halfspace_id": int(halfspace_id),
                                "vertex_id": int(vertex_id),
                                "upper_m": float(residual[1]),
                            })
    if not open_records:
        return _CLOSED, {"max_upper_m": float(max_upper)}
    return _OPEN, {
        "max_upper_m": float(max_upper),
        "open": open_records[:8],
        "open_count": len(open_records),
    }


def _sample_max_evidence(path, template, chain, allocation, s, side,
                         margin_m, frames, rotated):
    """Concrete evaluation of every assigned row at one station."""
    try:
        state = path.state_at_s(s, side)
    except VirtualAContractError as exc:
        return None, str(exc)
    qf = state.qf
    position_error_m = float(state.position_error_bound_m)
    angle_error_rad = float(state.angle_error_bound_rad)
    length = float(path.length_m)
    sigma = 0.0 if length <= 0.0 else min(1.0, max(0.0, s / length))
    tolerance = FORMATION_TOLERANCE_M
    segments = _segments_at(template, sigma, tolerance)
    best = None
    checked_f_regions = set()
    for segment in segments:
        region_index = int(segment.f_region_index)
        if region_index in checked_f_regions:
            continue
        checked_f_regions.add(region_index)
        if not (0 <= region_index < len(chain.regions)):
            continue
        rows, limits = rotated[region_index]
        for halfspace_id, (row, limit) in enumerate(zip(rows, limits)):
            residual = float(row[0]) * float(qf.x) + float(row[1]) * \
                float(qf.y) - float(limit)
            evidence = residual - position_error_m - \
                FORMATION_CERTIFICATE_EVALUATION_GUARD_M
            if best is None or evidence > best["evidence_m"]:
                best = _witness(
                    evidence, kind="f", s_m=float(s),
                    sigma=float(sigma), region_index=int(region_index),
                    template_segment_index=int(
                        template.segments.index(segment)),
                    halfspace_id=int(halfspace_id),
                    point_x=float(qf.x), point_y=float(qf.y),
                    residual_m=float(residual),
                    error_budget_m=float(position_error_m))
    vehicles = {int(vehicle.vehicle_id): vehicle
                for vehicle in state.vehicles}
    for segment in segments:
        for cover in segment.covers:
            frame = frames.get(cover.vehicle_id)
            vehicle = vehicles.get(int(cover.vehicle_id))
            if frame is None or vehicle is None:
                continue
            error = _point_error(
                frame, position_error_m, angle_error_rad)
            for body_piece_index, piece in enumerate(cover.pieces):
                region_index = int(piece.region_index)
                if not (0 <= region_index < len(chain.regions)):
                    continue
                rows, limits = rotated[region_index]
                for vertex_id, local_vertex in enumerate(
                        piece.local_vertices):
                    point = _point_corner(
                        frame, local_vertex,
                        float(vehicle.x), float(vehicle.y),
                        float(vehicle.theta))
                    for halfspace_id, (row, limit) in enumerate(
                            zip(rows, limits)):
                        residual = float(row[0]) * point[0] + \
                            float(row[1]) * point[1] - float(limit)
                        norm = math.hypot(float(row[0]), float(row[1]))
                        evidence = residual - norm * error
                        if best is None or evidence > best["evidence_m"]:
                            best = _witness(
                                evidence, kind="obb", s_m=float(s),
                                sigma=float(sigma),
                                vehicle_id=int(cover.vehicle_id),
                                decomposition=str(cover.decomposition),
                                body_piece_index=int(body_piece_index),
                                region_index=int(region_index),
                                halfspace_id=int(halfspace_id),
                                vertex_id=int(vertex_id),
                                point_x=float(point[0]),
                                point_y=float(point[1]),
                                residual_m=float(residual),
                                error_budget_m=float(norm * error))
    return best, ""


def _point_station_check(path, template, chain, allocation, cuts, margin_m,
                          frames, rotated, definite_margin_m, deadline=None):
    """Two-sided concrete checks at every required cut.  Evidence only."""
    results = []
    timed_out = False
    length = float(path.length_m)
    for s in cuts:
        if deadline is not None and time.monotonic() > deadline:
            timed_out = True
            break
        sigma = min(1.0, max(0.0, float(s) / length))
        if s <= 0.0:
            sides = ("right",)
        elif s >= length:
            sides = ("left",)
        else:
            sides = ("left", "right")
        for side in sides:
            try:
                best, error = _sample_max_evidence(
                    path, template, chain, allocation, s, side, margin_m,
                    frames, rotated)
            except Exception as exc:
                best, error = None, "{}: {}".format(
                    type(exc).__name__, exc)
            if best is None and error:
                results.append({
                    "s_m": float(s), "side": side,
                    "status": FORMATION_CERTIFICATE_UNCONFIRMED,
                    "reason": str(error)})
                continue
            if best is None:
                results.append({
                    "s_m": float(s), "side": side,
                    "status": FORMATION_CERTIFICATE_UNCONFIRMED,
                    "reason": "no applicable assignment"})
                continue
            status = (FORMATION_CERTIFICATE_VIOLATED
                      if best["evidence_m"] > definite_margin_m else
                      ("CLOSED" if best["evidence_m"] <= 0.0
                       else FORMATION_CERTIFICATE_UNCONFIRMED))
            results.append({
                "s_m": float(s), "side": side,
                "status": status,
                "witness": best,
            })
    return results, timed_out


def certify_formation_path(
        path, request, chain, template, *, allocation=None, guide=None,
        planning_context_sha256=None, config=None,
        max_depth=FORMATION_CERTIFICATE_DEFAULT_MAX_DEPTH,
        max_intervals=FORMATION_CERTIFICATE_DEFAULT_MAX_INTERVALS,
        min_width_m=FORMATION_CERTIFICATE_DEFAULT_MIN_WIDTH_M,
        max_seconds=FORMATION_CERTIFICATE_DEFAULT_MAX_SECONDS,
        definite_margin_m=FORMATION_CERTIFICATE_DEFINITE_M):
    """Certify the full F and three complete OBBs on the actual path domain.

    ``VERIFIED`` means every outward upper residual on every closed
    sub-interval and every two-sided cut was ``<= 0`` within the declared
    limits.  ``VIOLATED`` carries one concrete positive-evidence witness.
    Everything else (limits, non-finite data, A/D branch, allocation/template
    ambiguity) is ``UNCONFIRMED`` and must not be executed or published.
    """
    started = time.monotonic()
    if not isinstance(path, VirtualAPath):
        raise VirtualAContractError(
            "INVALID_PATH", "formation certificate requires a VirtualAPath")
    empty_identity = _identity_or_empty(request, chain, template)
    try:
        validate_formation_template(
            template, request, chain, allocation=allocation, guide=guide,
            planning_context_sha256=planning_context_sha256)
    except VirtualAContractError as exc:
        return _seal(_base_payload(
            FORMATION_CERTIFICATE_UNCONFIRMED,
            reason="TEMPLATE_IDENTITY_MISMATCH",
            detail="{}: {}".format(exc.reason, exc),
            identity=empty_identity,
            margin_m=float(getattr(template, "margin_m", 0.0) or 0.0),
            path_length_m=float(getattr(path, "length_m", 0.0) or 0.0),
            limits={}, elapsed_s=time.monotonic() - started,
            intervals_checked=0, intervals_verified=0,
            max_depth_reached=0, max_outward_residual_m=None,
            unclosed_intervals=[], cut_checks=[], vehicle_coverage=[],
            witness=None, f_only_audit=None, timed_out=False))
    if allocation is None:
        return _seal(_base_payload(
            FORMATION_CERTIFICATE_UNCONFIRMED,
            reason="MISSING_ALLOCATION",
            detail=(
                "a union certificate needs the F allocation used for the "
                "fixed-sigma row plan"),
            identity=_identity_or_empty(request, chain, template),
            margin_m=float(template.margin_m),
            path_length_m=float(getattr(path, "length_m", 0.0) or 0.0),
            limits={}, elapsed_s=time.monotonic() - started,
            intervals_checked=0, intervals_verified=0,
            max_depth_reached=0, max_outward_residual_m=None,
            unclosed_intervals=[], cut_checks=[], vehicle_coverage=[],
            witness=None, f_only_audit=None, timed_out=False))
    try:
        allocation = validate_corridor_allocation(chain, allocation)
    except VirtualAContractError as exc:
        return _seal(_base_payload(
            FORMATION_CERTIFICATE_UNCONFIRMED,
            reason="INVALID_ALLOCATION",
            detail="{}: {}".format(exc.reason, exc),
            identity=_identity_or_empty(request, chain, template),
            margin_m=float(template.margin_m),
            path_length_m=float(getattr(path, "length_m", 0.0) or 0.0),
            limits={}, elapsed_s=time.monotonic() - started,
            intervals_checked=0, intervals_verified=0,
            max_depth_reached=0, max_outward_residual_m=None,
            unclosed_intervals=[], cut_checks=[], vehicle_coverage=[],
            witness=None, f_only_audit=None, timed_out=False))
    config_margin = None
    if config is not None:
        try:
            config_margin = float(corridor_effective_margin(chain, config))
        except VirtualAContractError as exc:
            return _seal(_base_payload(
                FORMATION_CERTIFICATE_UNCONFIRMED,
                reason="INVALID_CONFIG_MARGIN",
                detail="{}: {}".format(exc.reason, exc),
                identity=_identity_or_empty(request, chain, template),
                margin_m=float(template.margin_m),
                path_length_m=float(getattr(path, "length_m", 0.0) or 0.0),
                limits={}, elapsed_s=time.monotonic() - started,
                intervals_checked=0, intervals_verified=0,
                max_depth_reached=0, max_outward_residual_m=None,
                unclosed_intervals=[], cut_checks=[], vehicle_coverage=[],
                witness=None, f_only_audit=None, timed_out=False))
        if float(template.margin_m) + FORMATION_TOLERANCE_M < config_margin:
            return _seal(_base_payload(
                FORMATION_CERTIFICATE_UNCONFIRMED,
                reason="MARGIN_BELOW_ACTIVE",
                detail=(
                    "certificate margin_m={!r} is below the active corridor "
                    "margin {!r}".format(
                        float(template.margin_m), config_margin)),
                identity=_identity_or_empty(request, chain, template),
                margin_m=float(template.margin_m),
                path_length_m=float(getattr(path, "length_m", 0.0) or 0.0),
                limits={}, elapsed_s=time.monotonic() - started,
                intervals_checked=0, intervals_verified=0,
                max_depth_reached=0, max_outward_residual_m=None,
                unclosed_intervals=[], cut_checks=[], vehicle_coverage=[],
                witness=None, f_only_audit=None, timed_out=False))
    try:
        max_depth = int(max_depth)
        max_intervals = int(max_intervals)
    except (TypeError, ValueError) as exc:
        raise VirtualAContractError(
            "INVALID_FORMATION_CERTIFICATE_LIMITS",
            "depth/interval limits must be integers") from exc
    if max_depth < 0 or max_intervals < 1:
        raise VirtualAContractError(
            "INVALID_FORMATION_CERTIFICATE_LIMITS",
            "max_depth must be >= 0 and max_intervals >= 1")
    min_width = _finite_float(min_width_m, "min_width_m")
    definite = _finite_float(definite_margin_m, "definite_margin_m")
    if min_width <= 0.0 or definite < 0.0:
        raise VirtualAContractError(
            "INVALID_FORMATION_CERTIFICATE_LIMITS",
            "min_width_m must be > 0 and definite_margin_m >= 0")
    if max_seconds is None:
        wall_limit = FORMATION_CERTIFICATE_DEFAULT_MAX_SECONDS
    else:
        wall_limit = _finite_float(max_seconds, "max_seconds")
        if wall_limit < 0.0:
            raise VirtualAContractError(
                "INVALID_FORMATION_CERTIFICATE_LIMITS",
                "max_seconds must be nonnegative")
    try:
        identity = _identity(path, request, chain, template)
    except Exception as exc:
        return _seal(_base_payload(
            FORMATION_CERTIFICATE_UNCONFIRMED,
            reason="PATH_PAYLOAD_INVALID",
            detail="{}: {}".format(type(exc).__name__, exc),
            identity=_identity_or_empty(request, chain, template),
            margin_m=float(template.margin_m),
            path_length_m=float(getattr(path, "length_m", 0.0) or 0.0),
            limits={}, elapsed_s=time.monotonic() - started,
            intervals_checked=0, intervals_verified=0,
            max_depth_reached=0, max_outward_residual_m=None,
            unclosed_intervals=[], cut_checks=[], vehicle_coverage=[],
            witness=None, f_only_audit=None, timed_out=False))
    limits = {
        "max_depth": int(max_depth),
        "max_intervals": int(max_intervals),
        "min_width_m": float(min_width),
        "max_seconds": float(wall_limit),
        "definite_margin_m": float(definite),
        "evaluation_guard_m": float(
            FORMATION_CERTIFICATE_EVALUATION_GUARD_M),
    }
    length = float(path.length_m)
    if not math.isfinite(length) or length <= 0.0:
        return _seal(_base_payload(
            FORMATION_CERTIFICATE_UNCONFIRMED,
            reason="ZERO_LENGTH_DOMAIN",
            detail=(
                "a continuous union certificate needs a positive arc-length "
                "domain; a hold path has no F/OBB interval"),
            identity=identity, margin_m=float(template.margin_m),
            path_length_m=max(0.0, length) if math.isfinite(length) else 0.0,
            limits=limits, elapsed_s=time.monotonic() - started,
            intervals_checked=0, intervals_verified=0,
            max_depth_reached=0, max_outward_residual_m=None,
            unclosed_intervals=[], cut_checks=[], vehicle_coverage=[],
            witness=None, f_only_audit=None, timed_out=False))
    position_error_m = _finite_float(
        path.position_error_bound_m, "path.position_error_bound_m")
    angle_error_rad = _finite_float(
        path.angle_error_bound_rad, "path.angle_error_bound_rad")
    if position_error_m < 0.0 or angle_error_rad < 0.0:
        raise VirtualAContractError(
            "INVALID_PATH", "path error bounds must be nonnegative")
    margin_m = float(template.margin_m)
    frames = _frames_by_id(template)
    rotated = {}
    used_region_indices = set()
    for segment in template.segments:
        used_region_indices.add(int(segment.f_region_index))
        for cover in segment.covers:
            for piece in cover.pieces:
                used_region_indices.add(int(piece.region_index))
    for region_index in sorted(used_region_indices):
        if not (0 <= region_index < len(chain.regions)):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "piece or F region {} is outside the chain".format(
                    region_index))
        rotated[region_index] = _rotated_region_rows(
            chain.regions[region_index], margin_m)
    f_only_audit = None
    try:
        legacy = audit_path_in_corridors(path, chain, allocation)
        f_only_audit = {
            "schema": "virtual_a_f_only_corridor_audit_comparison",
            "passed": bool(legacy.get("passed")),
            "reason": str(legacy.get("reason") or ""),
            "sample_count": int(legacy.get("sample_count", 0) or 0),
            "max_bound_residual_m": _finite_or_none(
                legacy.get("max_bound_residual_m")),
            "is_continuous_certificate": True,
            "certificate_scope": "f_only_comparison_not_the_three_obb_gate",
        }
    except Exception as exc:
        f_only_audit = {
            "schema": "virtual_a_f_only_corridor_audit_comparison",
            "passed": None,
            "reason": "{}: {}".format(type(exc).__name__, exc),
            "sample_count": 0,
            "max_bound_residual_m": None,
            "is_continuous_certificate": True,
            "certificate_scope": "f_only_comparison_not_the_three_obb_gate",
        }
    try:
        cuts = [0.0, length]
        for segment in template.segments:
            left, right = _segment_sigma_bounds(segment, length)
            cuts.extend((left, right))
        for left, right, _index in allocation:
            cuts.append(min(length, max(0.0, float(left) * length)))
            cuts.append(min(length, max(0.0, float(right) * length)))
        for s0, s1 in path.profile.domain_segments:
            cuts.append(min(length, max(0.0, float(s0))))
            cuts.append(min(length, max(0.0, float(s1))))
        for knot in getattr(path.profile, "knots", ()):
            # Hermite control-knot boundaries are explicit cuts even if a
            # profile implementation ever exposes more pieces than knots.
            cuts.append(min(length, max(0.0, float(knot.s))))
        cuts = _dedupe_cuts(cuts, length)
    except Exception as exc:
        return _seal(_base_payload(
            FORMATION_CERTIFICATE_UNCONFIRMED,
            reason="CUT_CONSTRUCTION_FAILED",
            detail="{}: {}".format(type(exc).__name__, exc),
            identity=identity, margin_m=margin_m,
            path_length_m=length, limits=limits,
            elapsed_s=time.monotonic() - started,
            intervals_checked=0, intervals_verified=0,
            max_depth_reached=0, max_outward_residual_m=None,
            unclosed_intervals=[], cut_checks=[], vehicle_coverage=[],
            witness=None, f_only_audit=None, timed_out=False))
    if cuts[0] != 0.0 or cuts[-1] != length:
        return _seal(_base_payload(
            FORMATION_CERTIFICATE_UNCONFIRMED,
            reason="CUT_COVERAGE_FAILED",
            detail="required cuts do not cover [0, length]",
            identity=identity, margin_m=margin_m,
            path_length_m=length, limits=limits,
            elapsed_s=time.monotonic() - started,
            intervals_checked=0, intervals_verified=0,
            max_depth_reached=0, max_outward_residual_m=None,
            unclosed_intervals=[], cut_checks=[], vehicle_coverage=[],
            witness=None, f_only_audit=None, timed_out=False))
    deadline = started + wall_limit
    cut_checks, cut_timed_out = _point_station_check(
        path, template, chain, allocation, cuts, margin_m,
        frames, rotated, definite, deadline=deadline)
    if cut_timed_out or time.monotonic() > deadline:
        return _seal(_base_payload(
            FORMATION_CERTIFICATE_UNCONFIRMED,
            reason="TIME_LIMIT",
            detail="certificate wall clock limit reached at the cut checks",
            identity=identity, margin_m=margin_m,
            path_length_m=length, limits=limits,
            elapsed_s=time.monotonic() - started,
            intervals_checked=0, intervals_verified=0,
            max_depth_reached=0, max_outward_residual_m=None,
            unclosed_intervals=[], cut_checks=cut_checks,
            vehicle_coverage=[], witness=None,
            f_only_audit=f_only_audit, timed_out=True))
    for check in cut_checks:
        if check["status"] == FORMATION_CERTIFICATE_VIOLATED:
            return _seal(_base_payload(
                FORMATION_CERTIFICATE_VIOLATED,
                reason="DEFINITE_CUT_VIOLATION",
                detail="a two-sided required cut has positive evidence",
                identity=identity, margin_m=margin_m,
                path_length_m=length, limits=limits,
                elapsed_s=time.monotonic() - started,
                intervals_checked=0, intervals_verified=0,
                max_depth_reached=0, max_outward_residual_m=None,
                unclosed_intervals=[], cut_checks=cut_checks,
                vehicle_coverage=[], witness=check["witness"],
                f_only_audit=f_only_audit, timed_out=False))
    tolerance = FORMATION_TOLERANCE_M
    intervals_checked = 0
    intervals_verified = 0
    max_depth_reached = 0
    max_outward = None
    unclosed = []
    witness = None
    timed_out = False
    interval_limit = False
    stack = [(cuts[index], cuts[index + 1], 0)
             for index in range(len(cuts) - 1)]
    # Left-to-right deterministic order: push the reversed list so the first
    # interval is processed first.
    stack = list(reversed(stack))
    while stack:
        if wall_limit >= 0.0 and time.monotonic() > deadline:
            timed_out = True
            break
        if intervals_checked >= max_intervals:
            interval_limit = True
            break
        a, b, depth = stack.pop()
        intervals_checked += 1
        max_depth_reached = max(max_depth_reached, int(depth))
        try:
            outcome, record = _check_interval(
                path, template, chain, allocation, a, b, depth, margin_m,
                position_error_m, angle_error_rad, frames, rotated,
                tolerance)
        except Exception as exc:
            outcome = _NONFINITE
            record = {"reason": "interval evaluation failed: {}: {}".format(
                type(exc).__name__, exc)}
        if outcome == _CLOSED:
            intervals_verified += 1
            if max_outward is None or record["max_upper_m"] > max_outward:
                max_outward = float(record["max_upper_m"])
            continue
        if outcome == _NONFINITE:
            unclosed.append({
                "s0_m": float(a), "s1_m": float(b),
                "depth": int(depth), "outcome": outcome,
                "reason": str(record.get("reason", ""))})
            # A non-finite interval can still hide a concrete violation; try
            # the three interior samples once before failing closed.
        can_split = depth < max_depth and (b - a) > min_width
        if can_split and outcome != _NONFINITE:
            middle = 0.5 * (a + b)
            stack.append((middle, b, depth + 1))
            stack.append((a, middle, depth + 1))
            continue
        # Leaf or non-finite interval: a concrete interior sample can turn it
        # into a definite witness immediately.
        for fraction in (0.5, 0.25, 0.75):
            sample = a + fraction * (b - a)
            try:
                best, _error = _sample_max_evidence(
                    path, template, chain, allocation, sample, "right",
                    margin_m, frames, rotated)
            except Exception:
                best = None
            if best is not None and best["evidence_m"] > definite:
                witness = best
                break
        if witness is not None:
            break
        if outcome == _NONFINITE:
            continue
        unclosed.append({
            "s0_m": float(a), "s1_m": float(b),
            "depth": int(depth), "outcome": outcome,
            "reason": str(record.get("reason", "")),
            "max_upper_m": _finite_or_none(record.get("max_upper_m")),
            "open_count": int(record.get("open_count", 0)),
            "open": list(record.get("open") or ()),
        })
    elapsed = time.monotonic() - started
    if witness is not None:
        status = FORMATION_CERTIFICATE_VIOLATED
        reason = "DEFINITE_INTERVAL_VIOLATION"
        detail = "a concrete point has positive evidence inside the domain"
    elif timed_out or interval_limit or unclosed or any(
            check["status"] != "CLOSED" for check in cut_checks):
        status = FORMATION_CERTIFICATE_UNCONFIRMED
        if timed_out:
            reason = "TIME_LIMIT"
            detail = "certificate wall clock limit reached"
        elif interval_limit:
            reason = "INTERVAL_LIMIT"
            detail = "certificate interval limit reached"
        elif unclosed:
            reason = "INTERVAL_NOT_CLOSED"
            detail = "one or more interval upper bounds did not close"
        else:
            reason = "CUT_NOT_CLOSED"
            detail = "one or more two-sided cut checks did not close"
    else:
        status = FORMATION_CERTIFICATE_VERIFIED
        reason = ""
        detail = (
            "all outward upper residuals <= 0 on every closed sub-interval "
            "and every two-sided required cut")
    vehicle_coverage = [
        {
            "vehicle_id": int(frame.vehicle_id),
            "offset_x": float(frame.offset_x),
            "offset_y": float(frame.offset_y),
            "length_m": float(frame.length_m),
            "width_m": float(frame.width_m),
        }
        for frame in template.vehicle_frames]
    return _seal(_base_payload(
        status, reason=reason, detail=detail, identity=identity,
        margin_m=margin_m, path_length_m=length, limits=limits,
        elapsed_s=float(elapsed), intervals_checked=int(intervals_checked),
        intervals_verified=int(intervals_verified),
        max_depth_reached=int(max_depth_reached),
        max_outward_residual_m=(
            None if max_outward is None else float(max_outward)),
        unclosed_intervals=unclosed[:16],
        unclosed_interval_count=len(unclosed),
        cut_checks=cut_checks,
        vehicle_coverage=vehicle_coverage,
        witness=witness,
        f_only_audit=f_only_audit,
        timed_out=bool(timed_out),
        interval_limit_reached=bool(interval_limit)))


def verify_formation_certificate(
        certificate, path, request, chain, template, *, allocation=None,
        guide=None, planning_context_sha256=None):
    """Re-check certificate identity and its canonical self-hash.

    This does not re-run the interval arithmetic; it is the fail-closed
    identity gate used by the pipeline, result and playback verifier.  A
    missing certificate, a non-``VERIFIED`` status, a stale template/request/
    grid/context/path/model digest, or a modified payload body all fail.
    """
    if not isinstance(certificate, dict):
        return {"passed": False, "reason": "MISSING_CERTIFICATE",
                "detail": "certificate must be a mapping"}
    if certificate.get("schema") != FORMATION_CERTIFICATE_SCHEMA:
        return {"passed": False, "reason": "CERTIFICATE_SCHEMA_MISMATCH",
                "detail": str(certificate.get("schema"))}
    stored_hash = certificate.get("certificate_sha256")
    body = {key: value for key, value in certificate.items()
            if key != "certificate_sha256"}
    try:
        actual_hash = _canonical_sha256(_sealed_body(body))
    except Exception as exc:
        return {"passed": False, "reason": "CERTIFICATE_PAYLOAD_INVALID",
                "detail": "{}: {}".format(type(exc).__name__, exc)}
    if stored_hash != actual_hash:
        return {"passed": False, "reason": "CERTIFICATE_HASH_MISMATCH",
                "detail": "stored certificate body hash differs"}
    try:
        identity = _identity(path, request, chain, template)
        validate_formation_template(
            template, request, chain, allocation=allocation, guide=guide,
            planning_context_sha256=planning_context_sha256)
    except Exception as exc:
        return {"passed": False, "reason": "CERTIFICATE_IDENTITY_UNAVAILABLE",
                "detail": "{}: {}".format(type(exc).__name__, exc)}
    for key, value in identity.items():
        if certificate.get(key) != value:
            return {"passed": False,
                    "reason": "CERTIFICATE_IDENTITY_MISMATCH",
                    "detail": "{} differs from the current input".format(key)}
    if certificate.get("identity") != identity:
        return {"passed": False,
                "reason": "CERTIFICATE_IDENTITY_MISMATCH",
                "detail": "the embedded identity record is inconsistent"}
    if certificate.get("template_id") != template.template_id:
        return {"passed": False,
                "reason": "CERTIFICATE_IDENTITY_MISMATCH",
                "detail": "template_id differs from the current template"}
    status = certificate.get("status")
    if status != FORMATION_CERTIFICATE_VERIFIED:
        return {"passed": False, "reason": "CERTIFICATE_NOT_VERIFIED",
                "detail": "status={!r}".format(status), "status": status}
    if certificate.get("is_continuous_certificate") is not True:
        return {"passed": False,
                "reason": "CERTIFICATE_FLAG_MISMATCH",
                "detail": "is_continuous_certificate is not true"}
    maximum = certificate.get("max_outward_residual_m")
    try:
        maximum = float(maximum)
    except (TypeError, ValueError):
        return {"passed": False,
                "reason": "CERTIFICATE_RESIDUAL_UNAVAILABLE",
                "detail": "max_outward_residual_m={!r}".format(maximum)}
    if not math.isfinite(maximum) or maximum > 0.0:
        return {"passed": False,
                "reason": "CERTIFICATE_RESIDUAL_POSITIVE",
                "detail": "max_outward_residual_m={!r}".format(maximum)}
    try:
        unclosed_count = int(
            certificate.get("unclosed_interval_count", 0))
    except (TypeError, ValueError):
        return {"passed": False,
                "reason": "CERTIFICATE_COUNTS_UNAVAILABLE",
                "detail": "unclosed_interval_count is not an integer"}
    if unclosed_count != 0:
        return {"passed": False,
                "reason": "CERTIFICATE_UNCLOSED_INTERVALS",
                "detail": "unclosed_interval_count is not zero"}
    if certificate.get("witness") is not None:
        return {"passed": False,
                "reason": "CERTIFICATE_HAS_WITNESS",
                "detail": "a verified certificate must not carry a witness"}
    try:
        verified_intervals = int(certificate.get("intervals_verified"))
        checked_intervals = int(certificate.get("intervals_checked"))
    except (TypeError, ValueError):
        return {"passed": False,
                "reason": "CERTIFICATE_COUNTS_UNAVAILABLE",
                "detail": "certificate interval counts are not integers"}
    if verified_intervals <= 0 or checked_intervals < verified_intervals:
        return {"passed": False,
                "reason": "CERTIFICATE_COUNTS_INVALID",
                "detail": "verified/checked interval counts are inconsistent"}
    cut_checks = certificate.get("cut_checks")
    if not isinstance(cut_checks, (list, tuple)) or not cut_checks:
        return {"passed": False,
                "reason": "CERTIFICATE_CUTS_MISSING",
                "detail": "a verified certificate must carry two-sided cuts"}
    if any(not isinstance(check, dict)
           or check.get("status") != "CLOSED"
           for check in cut_checks):
        return {"passed": False,
                "reason": "CERTIFICATE_CUT_NOT_CLOSED",
                "detail": "a two-sided cut check is not closed"}
    return {"passed": True, "reason": "", "detail": "",
            "status": status,
            "certificate_sha256": actual_hash}


__all__ = [
    "FORMATION_CERTIFICATE_SCHEMA",
    "FORMATION_CERTIFICATE_VERSION",
    "FORMATION_CERTIFICATE_VERIFIED",
    "FORMATION_CERTIFICATE_VIOLATED",
    "FORMATION_CERTIFICATE_UNCONFIRMED",
    "FORMATION_CERTIFICATE_STATUSES",
    "FORMATION_CERTIFICATE_DEFAULT_MAX_DEPTH",
    "FORMATION_CERTIFICATE_DEFAULT_MAX_INTERVALS",
    "FORMATION_CERTIFICATE_DEFAULT_MIN_WIDTH_M",
    "FORMATION_CERTIFICATE_DEFAULT_MAX_SECONDS",
    "FORMATION_CERTIFICATE_DEFINITE_M",
    "FORMATION_CERTIFICATE_EVALUATION_GUARD_M",
    "certify_formation_path",
    "verify_formation_certificate",
]