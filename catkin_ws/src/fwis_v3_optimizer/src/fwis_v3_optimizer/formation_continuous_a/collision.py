"""G4 conservative map/collision adapter for the A-only formation.

This module is the only bridge between the production C2 Hermite path and the
unchanged legacy :mod:`fwis_v3_optimizer.formation_collision` engine.  It never
re-implements OBB/SAT or map-boundary logic.  Its job is to:

* load and hash the authoritative ``Grid``;
* turn every C2 piece into an ``AnalyticSweepSegment`` whose per-lambda bounds
  are derived from accepted outward interval arithmetic, not from sampled
  maxima;
* independently verify the declared bounds before the legacy checker runs;
* reserve the path's numerical error envelope on each side by increasing the
  legacy full-width margin by twice the maximum envelope, because the legacy
  checker halves the margin into per-side half extents; and
* report the actual static/swept checks with map/model/context digests.
"""
from collections.abc import Mapping as MappingABC
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import yaml

from ..core import Grid
from ..formation_collision import (
    AnalyticSweepSegment,
    FormationCollisionChecker,
    FormationState,
    SweepPolicy,
    VehicleShape,
)
from . import bounds
from .contracts import (
    COLLISION_SCOPE,
    G4_PLANNING_SCHEMA,
    RESEARCH_VEHICLE_SHAPE_M,
    AuditReport,
    CollisionConfig,
    VirtualARequest,
    VirtualAContractError,
    ValidationResult,
    geometry_path_summary,
    grid_signature,
    grid_summary,
    planning_context_sha256,
    request_fingerprint,
    request_summary,
    validate_collision_config,
    validate_virtual_a_request,
)
from .audit import bind_timed_plan
from .trajectory import VirtualATimedPlan


MAP_SCHEMA = "virtual_rigid_a.g4.map.v1"
MAP_FORMAT = "ROS occupancy grid PGM/YAML"
RELATIVE_HEADING_BUDGET_RAD = 1e-10
ENVELOPE_NOTE = (
    "e_i = eps_position + (|offset_i| + r_upper) * eps_psi "
    "+ r_upper * eps_relative_heading, maximised over segments and vehicles; "
    "the legacy checker expands each body side by (margin - baseline)/2, so "
    "the required closure is (margin_used - baseline)/2 >= max_i e_i, "
    "equivalently margin_used >= baseline + 2*max_i e_i")
ENVELOPE_UNIT_NOTE = (
    "The legacy FormationCollisionChecker uses half extents "
    "(shape_dimension + margin_m)/2.  A full-width margin increase delta "
    "therefore grows each side by only delta/2.  error_envelope reserves "
    "2*max(e_i) of full-width margin so the actual per-side growth covers "
    "the numerical envelope.  Sweep motion padding is a separate "
    "sampling-interval protection and is never counted as numerical margin.")


class MapFormatError(ValueError):
    """Structured map parsing failure; never silently treats data as free."""

    def __init__(self, reason, detail=""):
        self.reason = str(reason)
        self.detail = str(detail)
        super().__init__(self.reason + (": " + self.detail if self.detail else ""))


# ---------------------------------------------------------------------------
# PGM/YAML map I/O
# ---------------------------------------------------------------------------

def _read_pgm(path):
    """Read a binary P5 PGM with conservative header handling."""
    data = Path(path).read_bytes()
    if not data.startswith(b"P5"):
        raise MapFormatError("UNSUPPORTED_PGM", "only binary P5 is accepted")
    index = 2
    fields = []
    while len(fields) < 3:
        while index < len(data) and data[index:index + 1].isspace():
            index += 1
        if index < len(data) and data[index:index + 1] == b"#":
            while index < len(data) and data[index:index + 1] not in (b"\n", b"\r"):
                index += 1
            continue
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        if start == index:
            raise MapFormatError("INVALID_PGM", "truncated header")
        fields.append(data[start:index])
    # Exactly one whitespace byte separates maxval from binary raster data.
    if index >= len(data) or not data[index:index + 1].isspace():
        raise MapFormatError("INVALID_PGM", "missing raster separator")
    index += 1
    try:
        width, height, maxval = (int(field) for field in fields)
    except ValueError as exc:
        raise MapFormatError("INVALID_PGM", str(exc)) from exc
    if width <= 0 or height <= 0 or not (0 < maxval <= 255):
        raise MapFormatError(
            "INVALID_PGM", "width/height/maxval={}/{}/{}".format(
                width, height, maxval))
    expected = width * height
    raster = np.frombuffer(data, dtype=np.uint8, count=expected, offset=index)
    if raster.size != expected:
        raise MapFormatError(
            "INVALID_PGM",
            "raster has {} bytes, expected {}x{}".format(
                raster.size, width, height))
    return width, height, maxval, raster.reshape(height, width)


def occupancy_from_pixels(pixels, negate=False, occupied_thresh=0.65,
                          free_thresh=0.196, maxval=255):
    """Conservative occupancy: unknown/gray cells become occupied.

    ``occ = (maxval - pixel)/maxval`` for ``negate=0`` (ROS convention).  A cell
    is free only when ``occ < free_thresh``.  It is occupied when
    ``occ >= occupied_thresh`` or when it lies in the unknown band; the
    unknown band is never treated as free.
    """
    for name, value in (("occupied_thresh", occupied_thresh),
                        ("free_thresh", free_thresh)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) \
                or not math.isfinite(float(value)) or not 0.0 <= float(value) <= 1.0:
            raise MapFormatError("INVALID_YAML", "{} is invalid".format(name))
    if float(occupied_thresh) <= float(free_thresh):
        raise MapFormatError(
            "INVALID_YAML", "occupied_thresh must exceed free_thresh")
    values = np.asarray(pixels, dtype=float)
    if bool(negate):
        probability = values / float(maxval)
    else:
        probability = (float(maxval) - values) / float(maxval)
    occupied = probability >= float(occupied_thresh)
    free = probability < float(free_thresh)
    # Unknown is occupied.  Expressed without a third state on purpose.
    return ~free


def load_map(yaml_path):
    """Load a ROS PGM/YAML map into the authoritative bottom-left ``Grid``."""
    yaml_path = Path(yaml_path)
    raw = yaml_path.read_bytes()
    try:
        metadata = yaml.safe_load(raw.decode("utf-8"))
    except Exception as exc:
        raise MapFormatError(
            "INVALID_YAML", "{}: {}".format(type(exc).__name__, exc)) from exc
    if not isinstance(metadata, dict):
        raise MapFormatError("INVALID_YAML", "top level must be a mapping")
    image_name = metadata.get("image")
    if not isinstance(image_name, str) or not image_name:
        raise MapFormatError("INVALID_YAML", "image is required")
    try:
        resolution = float(metadata["resolution"])
        origin = list(metadata["origin"])
        negate = bool(metadata.get("negate", 0))
        occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
        free_thresh = float(metadata.get("free_thresh", 0.196))
    except (KeyError, TypeError, ValueError) as exc:
        raise MapFormatError("INVALID_YAML", str(exc)) from exc
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise MapFormatError("INVALID_YAML", "resolution must be positive")
    if len(origin) != 3:
        raise MapFormatError("INVALID_YAML", "origin must have three values")
    origin_x, origin_y, yaw = (float(value) for value in origin)
    for value in (origin_x, origin_y, yaw):
        if not math.isfinite(value):
            raise MapFormatError("INVALID_YAML", "origin is not finite")
    if abs(yaw) > 1e-12:
        raise MapFormatError(
            "UNSUPPORTED_MAP_YAW",
            "Grid supports an axis-aligned origin; yaw={!r}".format(yaw))
    image_path = Path(image_name)
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_bytes = image_path.read_bytes()
    width, height, maxval, pixels = _read_pgm(image_path)
    # ROS image row 0 is the top; Grid [iy, ix] with origin at the bottom-left.
    occupancy = occupancy_from_pixels(
        np.flipud(pixels), negate=negate,
        occupied_thresh=occupied_thresh, free_thresh=free_thresh,
        maxval=maxval)
    grid = Grid(
        occupancy, resolution, origin_x, origin_y,
        length=RESEARCH_VEHICLE_SHAPE_M[0],
        width=RESEARCH_VEHICLE_SHAPE_M[1])
    return grid, {
        "schema": MAP_SCHEMA,
        "format": MAP_FORMAT,
        "yaml_path": str(yaml_path),
        "image_path": str(image_path),
        "yaml_sha256": hashlib.sha256(raw).hexdigest(),
        "image_sha256": hashlib.sha256(image_bytes).hexdigest(),
        "image_size": [height, width],
        "maxval": maxval,
        "resolution_m": resolution,
        "origin": [origin_x, origin_y, yaw],
        "negate": bool(negate),
        "occupied_thresh": occupied_thresh,
        "free_thresh": free_thresh,
        "unknown_cells_are": "occupied",
        "grid": grid_summary(grid),
        "grid_sha256": grid_signature(grid),
    }


def write_map(directory, stem, occupied, resolution, origin_x, origin_y,
              occupied_thresh=0.65, free_thresh=0.196, negate=0):
    """Write a deterministic P5/YAML pair; returns paths, hashes and summary."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    occupancy = np.asarray(occupied, dtype=bool)
    if occupancy.ndim != 2:
        raise MapFormatError("INVALID_OCCUPANCY", "expected a 2-D array")
    height, width = occupancy.shape
    # Grid [iy, ix] -> image rows top-down.
    raster = np.where(np.flipud(occupancy), 0, 255).astype(np.uint8)
    image_path = directory / (stem + ".pgm")
    header = "P5\n{} {}\n255\n".format(width, height).encode("ascii")
    image_path.write_bytes(header + raster.tobytes(order="C"))
    yaml_path = directory / (stem + ".yaml")
    yaml_text = (
        "# virtual_rigid_a G4 scene map (conservative occupancy)\n"
        "image: {image}\n"
        "resolution: {resolution!r}\n"
        "origin: [{ox!r}, {oy!r}, 0.0]\n"
        "negate: {negate}\n"
        "occupied_thresh: {occ!r}\n"
        "free_thresh: {free!r}\n"
    ).format(image=image_path.name, resolution=float(resolution),
             ox=float(origin_x), oy=float(origin_y), negate=int(negate),
             occ=float(occupied_thresh), free=float(free_thresh))
    yaml_path.write_text(yaml_text, encoding="utf-8")
    grid = Grid(occupancy, resolution, origin_x, origin_y,
                length=RESEARCH_VEHICLE_SHAPE_M[0],
                width=RESEARCH_VEHICLE_SHAPE_M[1])
    return {
        "image_path": str(image_path),
        "yaml_path": str(yaml_path),
        "image_sha256": hashlib.sha256(
            image_path.read_bytes()).hexdigest(),
        "yaml_sha256": hashlib.sha256(
            yaml_path.read_bytes()).hexdigest(),
        "grid": grid_summary(grid),
        "grid_sha256": grid_signature(grid),
    }


# ---------------------------------------------------------------------------
# Analytic sweep construction and independent bound verification
# ---------------------------------------------------------------------------

def _offset_h_interval(k_interval, eta_interval, offset):
    """Enclosure of H_i = k + a*eta/D, D=(1-b*k)^2+(a*k)^2."""
    a = float(offset.x)
    b = float(offset.y)
    k_int = tuple(k_interval)
    eta_int = tuple(eta_interval)
    one = (1.0, 1.0)
    A = bounds.isub(one, bounds.imul((b, b), k_int))
    B = bounds.imul((a, a), k_int)
    D = bounds.iadd(bounds.isquare(A), bounds.isquare(B))
    if not (D[0] > 0.0 and D[1] > 0.0):
        raise VirtualAContractError(
            "NORMAL_BRANCH_VIOLATION",
            "D interval={} is not strictly positive".format(D),
            vehicle_id=offset.vehicle_id)
    if a == 0.0:
        return k_int
    quotient = bounds.idiv_positive_interval(
        bounds.imul((a, a), eta_int), D)
    return bounds.iadd(k_int, quotient)


def _independent_segment_bounds(profile, offset, sa, sb):
    """Recompute the closed-segment k/eta/H bounds from raw piece data."""
    record = profile.interval_bounds(sa, sb)
    sup_k = 0.0
    sup_h = 0.0
    piece_records = record.get("segments") or [record]
    for piece in piece_records:
        sup_k = max(sup_k, bounds.imax_abs(piece["k"]))
        sup_h = max(sup_h, bounds.imax_abs(
            _offset_h_interval(piece["k"], piece["eta"], offset)))
    if not (math.isfinite(sup_k) and math.isfinite(sup_h)):
        raise VirtualAContractError(
            "NONFINITE_SWEEP_BOUND", "segment bounds are not finite")
    if sup_k < 0.0 or sup_h < 0.0:
        raise VirtualAContractError(
            "INVALID_SWEEP_BOUND", "negative supremum")
    delta = float(sb) - float(sa)
    if not (math.isfinite(delta) and delta > 0.0):
        raise VirtualAContractError(
            "INVALID_SWEEP_SEGMENT", "delta={!r}".format(delta))
    formation = delta * sup_k
    body = delta * sup_h
    if formation > 0.0:
        formation = bounds._next_up(formation)
    if body > 0.0:
        body = bounds._next_up(body)
    return {
        "delta_m": delta,
        "sup_abs_k": sup_k,
        "sup_abs_H": sup_h,
        "center_speed_bound_m_per_lambda": delta,
        "formation_rate_bound_rad_per_lambda": formation,
        "body_rate_bound_rad_per_lambda": body,
        "piece_records": [
            {"index": piece.get("index"), "s0": piece["s0"],
             "s1": piece["s1"], "k": list(piece["k"]),
             "eta": list(piece["eta"])}
            for piece in piece_records],
    }


def _plan_domain_segments(plan):
    if plan.kind == "hold":
        return ()
    profile = plan.path.profile
    if profile is None:
        raise VirtualAContractError(
            "INVALID_PATH", "motion plan has no curvature profile")
    segments = tuple(profile.domain_segments)
    if not segments:
        raise VirtualAContractError(
            "INVALID_PATH", "motion plan has no domain segments")
    return segments


def make_sweep_segments(plan, config=None):
    """Adapt every C2 piece to the unchanged legacy analytic sweep API.

    The returned bounds are per unit ``lambda``: every per-metre derivative is
    multiplied by ``Delta = sb - sa``.  ``time_at(lambda)`` uses the G3
    common-time inverse at the same station; station numbers are never used as
    seconds.
    """
    if config is None:
        config = CollisionConfig()
    validation = validate_collision_config(config)
    if not validation.passed:
        raise VirtualAContractError(
            validation.reason, validation.detail, **validation.location)
    if not isinstance(plan, VirtualATimedPlan):
        raise VirtualAContractError(
            "INVALID_PLAN", "expected VirtualATimedPlan")
    path = plan.path
    if plan.kind == "hold":
        return ()
    result = []
    for index, (sa, sb) in enumerate(_plan_domain_segments(plan)):
        independent = {
            offset.vehicle_id: _independent_segment_bounds(
                path.profile, offset, sa, sb)
            for offset in plan.path.model.offsets}
        delta = independent[next(iter(independent))]["delta_m"]
        for vehicle_id, record in independent.items():
            if abs(record["delta_m"] - delta) > 0.0:
                raise VirtualAContractError(
                    "INVALID_SWEEP_SEGMENT", "inconsistent delta")
        formation = max(record["formation_rate_bound_rad_per_lambda"]
                        for record in independent.values())
        body = {
            vehicle_id: record["body_rate_bound_rad_per_lambda"]
            for vehicle_id, record in independent.items()}

        def state_at(fraction, sa=sa, sb=sb):
            station = sa + float(fraction) * (sb - sa)
            spatial = path.state_at_s(station)
            return FormationState(
                station=station,
                qf=spatial.qf,
                body_headings={
                    vehicle.vehicle_id: vehicle.theta
                    for vehicle in spatial.vehicles})

        def time_at(fraction, sa=sa, sb=sb):
            station = sa + float(fraction) * (sb - sa)
            return float(plan.time_at_s(station))

        result.append(AnalyticSweepSegment(
            station_start=sa,
            station_end=sb,
            state_at=state_at,
            time_at=time_at,
            center_speed_bound_m_per_lambda=delta,
            formation_rate_bound_rad_per_lambda=formation,
            body_rate_bounds_rad_per_lambda=body,
            label="piece_{}[{!r},{!r}]".format(index, sa, sb)))
    return tuple(result)


def verify_sweep_segments(plan, segments, config=None, *,
                          state_samples=5):
    """Independently reject underestimated/uncovered analytic bounds.

    This function never trusts ``segment.center_speed_bound_*``,
    ``segment.formation_rate_bound_*`` or ``segment.body_rate_bounds_*``.  It
    rebuilds the raw-profile interval enclosure, checks every declared bound
    is at least that large, checks exact [0, S] coverage/contiguity, and checks
    ``state_at``/``time_at`` against the authoritative plan.
    """
    if config is None:
        config = CollisionConfig()
    config_validation = validate_collision_config(config)
    if not config_validation.passed:
        return ValidationResult.fail(
            config_validation.reason, config_validation.detail,
            **config_validation.location)
    if not isinstance(plan, VirtualATimedPlan):
        return ValidationResult.fail(
            "INVALID_PLAN", "expected VirtualATimedPlan")
    segments = tuple(segments)
    expected_domains = _plan_domain_segments(plan)
    if plan.kind == "hold":
        if segments:
            return ValidationResult.fail(
                "SWEEP_COVERAGE_INVALID",
                "a hold plan has no swept segments")
        return ValidationResult.ok(
            segment_count=0, coverage="hold", declared_bounds_verified=True)
    if len(segments) != len(expected_domains):
        return ValidationResult.fail(
            "SWEEP_COVERAGE_INVALID",
            "segment_count={} expected={}".format(
                len(segments), len(expected_domains)))
    tolerance = 1e-12
    records = []
    previous_end = None
    total = 0.0
    for index, (segment, (sa, sb)) in enumerate(
            zip(segments, expected_domains)):
        if not isinstance(segment, AnalyticSweepSegment):
            return ValidationResult.fail(
                "INVALID_SWEEP_SEGMENT",
                "segment {} is not AnalyticSweepSegment".format(index))
        if abs(segment.station_start - sa) > tolerance or \
                abs(segment.station_end - sb) > tolerance:
            return ValidationResult.fail(
                "SWEEP_COVERAGE_INVALID",
                "segment {} stations [{}, {}] expected [{}, {}]".format(
                    index, segment.station_start, segment.station_end, sa, sb),
                segment_index=index)
        if previous_end is not None and abs(sa - previous_end) > tolerance:
            return ValidationResult.fail(
                "SWEEP_COVERAGE_INVALID",
                "gap between segments at {!r} and {!r}".format(
                    previous_end, sa), segment_index=index)
        previous_end = sb
        independent = {
            offset.vehicle_id: _independent_segment_bounds(
                plan.path.profile, offset, sa, sb)
            for offset in plan.path.model.offsets}
        delta = sb - sa
        if segment.center_speed_bound_m_per_lambda + tolerance < delta:
            return ValidationResult.fail(
                "SWEEP_BOUND_UNDERESTIMATE",
                "segment {} center bound {!r} is below Delta {!r}".format(
                    index, segment.center_speed_bound_m_per_lambda, delta),
                segment_index=index, bound="center_speed")
        formation = max(
            record["formation_rate_bound_rad_per_lambda"]
            for record in independent.values())
        if segment.formation_rate_bound_rad_per_lambda + tolerance < formation:
            return ValidationResult.fail(
                "SWEEP_BOUND_UNDERESTIMATE",
                "segment {} formation bound {!r} is below Delta*sup|k| "
                "{!r}".format(index,
                              segment.formation_rate_bound_rad_per_lambda,
                              formation),
                segment_index=index, bound="formation_rate")
        for vehicle_id, record in independent.items():
            declared = segment.body_rate_bounds_rad_per_lambda.get(vehicle_id)
            if declared is None:
                return ValidationResult.fail(
                    "SWEEP_BOUND_UNDERESTIMATE",
                    "segment {} vehicle {} has no body bound".format(
                        index, vehicle_id),
                    segment_index=index, vehicle_id=vehicle_id)
            if declared + tolerance < record["body_rate_bound_rad_per_lambda"]:
                return ValidationResult.fail(
                    "SWEEP_BOUND_UNDERESTIMATE",
                    "segment {} vehicle {} body bound {!r} is below "
                    "Delta*sup|H_i| {!r}".format(
                        index, vehicle_id, declared,
                        record["body_rate_bound_rad_per_lambda"]),
                    segment_index=index, vehicle_id=vehicle_id,
                    bound="body_rate")
        sample_count = max(1, int(state_samples))
        for sample in range(sample_count + 1):
            fraction = sample / float(sample_count)
            station = sa + fraction * (sb - sa)
            try:
                state = segment.state_at(fraction)
                state_time = float(segment.time_at(fraction))
            except Exception as exc:
                return ValidationResult.fail(
                    "INVALID_SWEEP_SEGMENT",
                    "segment {} fraction {} callback failed: {}: {}".format(
                        index, fraction, type(exc).__name__, exc),
                    segment_index=index, fraction=fraction)
            spatial = plan.path.state_at_s(station)
            expected_headings = {
                vehicle.vehicle_id: vehicle.theta
                for vehicle in spatial.vehicles}
            if not isinstance(state, FormationState) or \
                    abs(state.station - station) > tolerance or \
                    abs(state.qf.x - spatial.qf.x) > tolerance or \
                    abs(state.qf.y - spatial.qf.y) > tolerance or \
                    abs(state.qf.psi - spatial.qf.psi) > tolerance:
                return ValidationResult.fail(
                    "SWEEP_STATE_MISMATCH",
                    "segment {} fraction {} state differs from plan".format(
                        index, fraction),
                    segment_index=index, fraction=fraction)
            for vehicle_id, theta in expected_headings.items():
                actual = state.body_headings.get(vehicle_id)
                if actual is None or abs(actual - theta) > 1e-9:
                    return ValidationResult.fail(
                        "SWEEP_STATE_MISMATCH",
                        "segment {} fraction {} vehicle {} heading differs"
                        .format(index, fraction, vehicle_id),
                        segment_index=index, fraction=fraction,
                        vehicle_id=vehicle_id)
            expected_time = float(plan.time_at_s(station))
            if not (math.isfinite(state_time) and
                    abs(state_time - expected_time) <= 1e-9 *
                    max(1.0, abs(expected_time))):
                return ValidationResult.fail(
                    "SWEEP_TIME_MISMATCH",
                    "segment {} fraction {} time differs from G3 inverse"
                    .format(index, fraction),
                    segment_index=index, fraction=fraction)
        total += delta
        records.append({
            "index": index, "s0": sa, "s1": sb, "delta_m": delta,
            "declared_center_speed_bound":
                segment.center_speed_bound_m_per_lambda,
            "independent_center_speed_lower_bound": delta,
            "declared_formation_rate_bound":
                segment.formation_rate_bound_rad_per_lambda,
            "independent_formation_rate_lower_bound": formation,
            "declared_body_rate_bounds": dict(
                segment.body_rate_bounds_rad_per_lambda),
            "independent_body_rate_lower_bounds": {
                vehicle_id: record["body_rate_bound_rad_per_lambda"]
                for vehicle_id, record in independent.items()},
            "verified": True,
            "state_samples": sample_count + 1,
        })
    if abs(total - float(plan.path.length_m)) > 1e-9:
        return ValidationResult.fail(
            "SWEEP_COVERAGE_INVALID",
            "covered length {!r} differs from path length {!r}".format(
                total, plan.path.length_m))
    return ValidationResult.ok(
        segment_count=len(records), covered_length_m=total,
        segments=records, declared_bounds_verified=True,
        coverage="contiguous_closed_pieces")


# ---------------------------------------------------------------------------
# Numerical error envelope and collision audit
# ---------------------------------------------------------------------------

def _finite_nonnegative_bound(value, label, ceiling):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise VirtualAContractError(
            "INTEGRATION_BUDGET_INVALID",
            "{} is not a finite real".format(label)) from exc
    if not math.isfinite(result) or result < 0.0 or result > ceiling:
        raise VirtualAContractError(
            "INTEGRATION_BUDGET_INVALID",
            "{}={!r} outside [0, {!r}]".format(label, result, ceiling),
            field=label, value=result, ceiling=ceiling)
    return result


def _checker_radius_upper(margin_value):
    """Outward upper bound of the legacy checker's inflated circumradius.

    The checker uses exactly the binary64 half extents
    ``(length + margin)/2`` and ``(width + margin)/2``.  Squaring those
    point values and taking an outward square root covers their real
    Euclidean radius; no old fixed-point radius is reused.
    """
    half_length = (RESEARCH_VEHICLE_SHAPE_M[0] + margin_value) / 2.0
    half_width = (RESEARCH_VEHICLE_SHAPE_M[1] + margin_value) / 2.0
    squared = bounds.iadd(
        bounds.isquare((half_length, half_length)),
        bounds.isquare((half_width, half_width)))
    return bounds.isqrt_positive(squared)[1]


def _offset_magnitude_upper(offset):
    squared = bounds.iadd(
        bounds.isquare((offset.x, offset.x)),
        bounds.isquare((offset.y, offset.y)))
    return bounds.isqrt_positive(squared)[1]


def _vehicle_envelope(rho, radius, eps_position, eps_psi, eps_relative):
    """Outward upper bound of the radial displacement envelope."""
    rho_plus_radius = bounds._next_up(rho + radius)
    angle_term = bounds._next_up(rho_plus_radius * eps_psi)
    relative_term = bounds._next_up(radius * eps_relative)
    radial_terms = bounds._next_up(angle_term + relative_term)
    return bounds._next_up(eps_position + radial_terms)


def _checker_side_growth(margin_value, base):
    """Outward lower bounds of the actual legacy per-side growth.

    Both the length and width half extents grow by the same real amount,
    but their float additions can round differently.  ``isub`` returns
    interval lower bounds, so the recorded value is never an overestimate.
    """
    final_length = (RESEARCH_VEHICLE_SHAPE_M[0] + margin_value) / 2.0
    final_width = (RESEARCH_VEHICLE_SHAPE_M[1] + margin_value) / 2.0
    base_length = (RESEARCH_VEHICLE_SHAPE_M[0] + base) / 2.0
    base_width = (RESEARCH_VEHICLE_SHAPE_M[1] + base) / 2.0
    length_growth = bounds.isub(
        (final_length, final_length), (base_length, base_length))[0]
    width_growth = bounds.isub(
        (final_width, final_width), (base_width, base_width))[0]
    return {
        "length_growth_lower_m": length_growth,
        "width_growth_lower_m": width_growth,
        "per_side_growth_lower_m": min(length_growth, width_growth),
        "checker_half_length_m": final_length,
        "checker_half_width_m": final_width,
        "baseline_half_length_m": base_length,
        "baseline_half_width_m": base_width,
    }


def error_envelope(plan, config=None):
    """Per-vehicle numerical envelope with legacy half-extent semantics.

    The legacy checker expands a body by ``margin_m/2`` on each side.  A
    numerical envelope value ``e`` is a physical distance that must be
    covered by the actual per-side expansion, not by the full-width margin.
    The returned margin therefore satisfies ``(margin - baseline)/2 >=
    max(e_i)``, equivalently ``margin >= baseline + 2*max(e_i)``.  The
    vehicle-vehicle context uses two expanded bodies, so its gap reduction
    is ``margin - baseline`` and the required pair envelope is ``e_i + e_j``.
    """
    if config is None:
        config = CollisionConfig()
    config_validation = validate_collision_config(config)
    if not config_validation.passed:
        raise VirtualAContractError(
            config_validation.reason, config_validation.detail,
            **config_validation.location)
    if not isinstance(plan, VirtualATimedPlan):
        raise VirtualAContractError(
            "INVALID_PLAN", "expected VirtualATimedPlan")
    base = float(config.baseline_margin_m)
    eps_position = _finite_nonnegative_bound(
        plan.path.position_error_bound_m, "position_error_bound_m",
        bounds.POSITION_BUDGET_M)
    eps_psi = _finite_nonnegative_bound(
        plan.path.angle_error_bound_rad, "angle_error_bound_rad",
        bounds.ANGLE_BUDGET_RAD)
    eps_relative = float(RELATIVE_HEADING_BUDGET_RAD)
    if not math.isfinite(eps_relative) or eps_relative < 0.0:
        raise VirtualAContractError(
            "COLLISION_MARGIN_RELAXED",
            "relative heading budget is not a finite non-negative real")
    offsets = tuple(plan.path.model.offsets)
    if not offsets:
        raise VirtualAContractError(
            "INVALID_PLAN", "formation model has no vehicle offsets")
    rho_values = []
    for offset in offsets:
        for name in ("x", "y"):
            value = getattr(offset, name)
            if not math.isfinite(float(value)):
                raise VirtualAContractError(
                    "INVALID_PLAN",
                    "vehicle {} offset.{} is non-finite".format(
                        offset.vehicle_id, name))
        rho_values.append(_offset_magnitude_upper(offset))
    rho_max = max(rho_values)

    # Solve the monotone fixed point
    #   m = base + 2 * max_i [eps_position
    #         + (|offset_i| + r(m)) * eps_psi + r(m) * eps_relative]
    # with r(m) the actual checker-expanded circumradius.  Each iterate is
    # rounded outward.  The loop also checks the actual float half-extent
    # growth, so a one-ulp addition rounding cannot silently violate the
    # per-side closure.
    margin = float(base)
    maximum = 0.0
    radius_upper = _checker_radius_upper(margin)
    growth = _checker_side_growth(margin, base)
    for _ in range(64):
        radius_upper = _checker_radius_upper(margin)
        envelopes = [
            _vehicle_envelope(
                rho, radius_upper, eps_position, eps_psi, eps_relative)
            for rho in rho_values]
        maximum = max(envelopes)
        candidate = bounds._next_up(
            base + bounds._next_up(2.0 * maximum))
        growth = _checker_side_growth(margin, base)
        if candidate <= margin and \
                growth["per_side_growth_lower_m"] >= maximum:
            break
        if candidate > margin:
            margin = candidate
        else:
            # The guarantee failed only because the two float additions
            # rounded; advance one representable step and retry.
            margin = bounds._next_up(margin)
    else:
        raise VirtualAContractError(
            "NUMERICAL_FAILURE",
            "collision envelope fixed point did not converge")
    # Recompute all displayed values at the accepted margin.
    radius_upper = _checker_radius_upper(margin)
    records = []
    maximum = 0.0
    max_record = None
    for offset, rho in zip(offsets, rho_values):
        envelope = _vehicle_envelope(
            rho, radius_upper, eps_position, eps_psi, eps_relative)
        record = {
            "vehicle_id": offset.vehicle_id,
            "offset_m": rho,
            "offset_hypot_float_m": math.hypot(offset.x, offset.y),
            "inflated_body_radius_upper_m": radius_upper,
            "eps_position_m": eps_position,
            "eps_formation_heading_rad": eps_psi,
            "eps_relative_heading_rad": eps_relative,
            "envelope_m": envelope,
            "per_side_required_m": envelope,
        }
        records.append(record)
        if envelope > maximum:
            maximum = envelope
            max_record = record
    growth = _checker_side_growth(margin, base)
    per_side_margin = growth["per_side_growth_lower_m"]
    minimum_required = bounds._next_up(
        base + bounds._next_up(2.0 * maximum))
    margin_growth = bounds.isub((margin, margin), (base, base))[0]
    pair_required = 0.0
    pair_record = None
    for left in range(len(records)):
        for right in range(left + 1, len(records)):
            pair = bounds.iadd(
                (records[left]["envelope_m"], records[left]["envelope_m"]),
                (records[right]["envelope_m"],
                 records[right]["envelope_m"]))[1]
            if pair > pair_required:
                pair_required = pair
                pair_record = {
                    "first_vehicle_id": records[left]["vehicle_id"],
                    "second_vehicle_id": records[right]["vehicle_id"],
                    "pair_envelope_m": pair,
                }
    if margin < base or margin < minimum_required:
        raise VirtualAContractError(
            "COLLISION_MARGIN_RELAXED",
            "margin {!r} does not dominate baseline {!r} + twice the "
            "maximum envelope {!r}".format(margin, base, maximum))
    if per_side_margin < maximum:
        raise VirtualAContractError(
            "COLLISION_MARGIN_RELAXED",
            "actual checker per-side growth {!r} does not cover envelope "
            "{!r}".format(per_side_margin, maximum))
    if margin_growth < pair_required:
        raise VirtualAContractError(
            "COLLISION_MARGIN_RELAXED",
            "two-body gap reduction {!r} does not cover pair envelope "
            "{!r}".format(margin_growth, pair_required))
    if radius_upper < _checker_radius_upper(margin):
        raise VirtualAContractError(
            "COLLISION_MARGIN_RELAXED",
            "radius upper bound does not cover the actual checker radius")
    if margin < base or margin < 0.05:
        raise VirtualAContractError(
            "COLLISION_MARGIN_RELAXED", "margin below 0.05")
    padding = float(config.max_corner_step_m)
    covered = per_side_margin >= maximum
    pair_covered = margin_growth >= pair_required
    contexts = {
        "static_start_goal": {
            "checker_padding_m": 0.0,
            "required_per_side_m": maximum,
            "actual_per_side_growth_lower_m": per_side_margin,
            "covered": covered,
        },
        "no_motion_hold": {
            "checker_padding_m": 0.0,
            "required_per_side_m": maximum,
            "actual_per_side_growth_lower_m": per_side_margin,
            "covered": covered,
        },
        "motion_sweep": {
            "checker_padding_m": padding,
            "required_per_side_m": maximum,
            "actual_numerical_per_side_growth_lower_m": per_side_margin,
            "covered": covered,
            "padding_role": (
                "sampling-interval motion protection only; not counted in "
                "the numerical envelope and not subtracted from the margin"),
        },
        "map_boundary": {
            "checker_padding_m": 0.0,
            "required_per_side_m": maximum,
            "actual_per_side_growth_lower_m": per_side_margin,
            "covered": covered,
        },
        "vehicle_vehicle": {
            "checker_padding_m": 0.0,
            "required_pair_envelope_m": pair_required,
            "actual_two_body_gap_reduction_lower_m": margin_growth,
            "covered": pair_covered,
            "binding_pair": pair_record,
        },
    }
    return {
        "baseline_margin_m": base,
        "maximum_envelope_m": maximum,
        "margin_used_m": margin,
        "margin_minimum_required_m": minimum_required,
        "margin_growth_lower_m": margin_growth,
        "per_side_margin_m": per_side_margin,
        "per_side_required_m": maximum,
        "closure_gap_m": per_side_margin - maximum,
        "pair_envelope_required_m": pair_required,
        "pair_closure_gap_m": margin_growth - pair_required,
        "checker_half_length_m": growth["checker_half_length_m"],
        "checker_half_width_m": growth["checker_half_width_m"],
        "baseline_half_length_m": growth["baseline_half_length_m"],
        "baseline_half_width_m": growth["baseline_half_width_m"],
        "checker_half_length_growth_lower_m":
            growth["length_growth_lower_m"],
        "checker_half_width_growth_lower_m":
            growth["width_growth_lower_m"],
        "radius_upper_m": radius_upper,
        "checker_radius_upper_m": _checker_radius_upper(margin),
        "sweep_motion_padding_m": padding,
        "maximum_record": max_record,
        "vehicles": records,
        "contexts": contexts,
        "formula": ENVELOPE_NOTE,
        "unit_note": ENVELOPE_UNIT_NOTE,
        "radius_upper_note": (
            "radius_upper is an outward interval square root of the actual "
            "checker half extents (length+margin)/2 and (width+margin)/2 at "
            "the recorded margin; it is not the old fixed-point radius.  "
            "The margin is solved at this same expanded radius."),
    }


def _checker_headings(state):
    return {
        vehicle.vehicle_id: float(vehicle.theta)
        for vehicle in state.vehicles}


def _checker_stats(checker):
    return dict(checker.statistics())


def _audit_context(plan, request, grid, config, base_summary,
                   path_summary_value, limits_summary_value, request_sha):
    """Validate the plan's G4 context against this grid and collision config."""
    try:
        context = plan.planning_context
    except AttributeError as exc:
        return None, AuditReport.fail(
            "PLANNING_CONTEXT_REQUIRED",
            "the timed plan carries no planning context",
            location={"constraint": "PLANNING_CONTEXT"},
            request_summary=base_summary,
            path_summary=path_summary_value,
            limits_summary=limits_summary_value,
            request_sha256=request_sha, check_count=1)
    if not isinstance(context, MappingABC):
        return None, AuditReport.fail(
            "PLANNING_CONTEXT_REQUIRED",
            "the timed plan carries no G4 planning context",
            location={"constraint": "PLANNING_CONTEXT"},
            request_summary=base_summary,
            path_summary=path_summary_value,
            limits_summary=limits_summary_value,
            request_sha256=request_sha, check_count=1)
    expected_keys = {
        "schema", "stage", "request_sha256", "grid",
        "planner_config", "timing_config", "collision_config",
    }
    if set(context) != expected_keys or \
            context.get("schema") != G4_PLANNING_SCHEMA or \
            context.get("stage") != "G4":
        return None, AuditReport.fail(
            "INVALID_PLANNING_CONTEXT",
            "context is not the G4 schema",
            location={"constraint": "PLANNING_CONTEXT"},
            request_summary=base_summary,
            path_summary=path_summary_value,
            limits_summary=limits_summary_value,
            request_sha256=request_sha, check_count=1)
    if context.get("request_sha256") != request_sha:
        return None, AuditReport.fail(
            "PLANNING_CONTEXT_MISMATCH",
            "context request digest differs from the physical request",
            location={"constraint": "REQUEST_DIGEST"},
            request_summary=base_summary,
            path_summary=path_summary_value,
            limits_summary=limits_summary_value,
            request_sha256=request_sha, check_count=1)
    actual_grid = grid_summary(grid)
    if context.get("grid") != actual_grid:
        return None, AuditReport.fail(
            "GRID_DIGEST_MISMATCH",
            "context grid differs from the supplied authoritative Grid",
            location={"constraint": "GRID"},
            request_summary=base_summary,
            path_summary=path_summary_value,
            limits_summary=limits_summary_value,
            request_sha256=request_sha, check_count=1,
            planning_context_sha256=planning_context_sha256(context),
            grid_sha256=grid_signature(grid))
    if context.get("collision_config") != config.as_dict():
        return None, AuditReport.fail(
            "COLLISION_CONFIG_MISMATCH",
            "context collision config differs from the audit config",
            location={"constraint": "COLLISION_CONFIG"},
            request_summary=base_summary,
            path_summary=path_summary_value,
            limits_summary=limits_summary_value,
            request_sha256=request_sha, check_count=1,
            planning_context_sha256=planning_context_sha256(context),
            grid_sha256=grid_signature(grid))
    try:
        context_sha = planning_context_sha256(context)
    except (TypeError, ValueError) as exc:
        return None, AuditReport.fail(
            "INVALID_PLANNING_CONTEXT",
            "context is not finite JSON: {}: {}".format(
                type(exc).__name__, exc),
            location={"constraint": "PLANNING_CONTEXT"},
            request_summary=base_summary,
            path_summary=path_summary_value,
            limits_summary=limits_summary_value,
            request_sha256=request_sha, check_count=1)
    plan_context_sha = plan.planning_context_sha256()
    if plan_context_sha != context_sha:
        return None, AuditReport.fail(
            "PLANNING_CONTEXT_MISMATCH",
            "plan context digest differs from its canonical context",
            location={"constraint": "PLANNING_CONTEXT"},
            request_summary=base_summary,
            path_summary=path_summary_value,
            limits_summary=limits_summary_value,
            request_sha256=request_sha, check_count=1,
            planning_context_sha256=context_sha)
    return {
        "context": context,
        "context_sha": context_sha,
        "grid_sha": grid_signature(grid),
        "grid": actual_grid,
    }, None


def _enrich_trace(trace, plan):
    for entry in trace:
        if entry.get("kind") in ("sample", "segment_start"):
            station = float(entry["station"])
            try:
                state = plan.path.state_at_s(station)
                entry["body_headings"] = _checker_headings(state)
            except Exception:
                entry["body_headings"] = {}
    return trace


def audit_collision(request, plan, grid, config=None):
    """Audit a G4 timed plan against the authoritative grid and 0.05 m margin.

    Order: exact context/binding, static start, static goal, then the unchanged
    legacy analytic sweep.  A sample-budget failure or malformed callback is a
    collision-audit failure, never a pass.  Whole-formation circumcircles are
    not used.
    """
    started = time.monotonic()
    if config is None:
        config = CollisionConfig()
    base_summary = request_summary(request) if isinstance(
        request, VirtualARequest) else {
            "type": type(request).__name__, "valid": False}
    path_summary_value = {}
    if plan is not None and hasattr(plan, "path"):
        try:
            path_summary_value = geometry_path_summary(plan.path)
        except Exception as exc:
            path_summary_value = {"summary_error": "{}: {}".format(
                type(exc).__name__, exc)}
    limits_summary_value = {}
    if isinstance(request, VirtualARequest):
        try:
            limits_summary_value = request.limits.as_dict()
        except Exception:
            limits_summary_value = {}
    request_sha = ""
    if isinstance(request, VirtualARequest):
        try:
            request_sha = request_fingerprint(request)
        except Exception:
            request_sha = ""
    config_validation = validate_collision_config(config)
    if not config_validation.passed:
        return AuditReport.fail(
            config_validation.reason, config_validation.detail,
            scope=COLLISION_SCOPE, location=config_validation.location,
            request_summary=base_summary, path_summary=path_summary_value,
            limits_summary=limits_summary_value, request_sha256=request_sha,
            check_count=1)
    if not isinstance(request, VirtualARequest):
        return AuditReport.fail(
            "INVALID_REQUEST", "expected VirtualARequest",
            scope=COLLISION_SCOPE,
            request_summary=base_summary, path_summary=path_summary_value,
            limits_summary=limits_summary_value, check_count=1)
    request_validation = validate_virtual_a_request(request)
    if not request_validation.passed:
        return AuditReport.fail(
            request_validation.reason, request_validation.detail,
            scope=COLLISION_SCOPE, location=request_validation.location,
            request_summary=base_summary, path_summary=path_summary_value,
            limits_summary=limits_summary_value, check_count=1)
    if not isinstance(grid, Grid):
        return AuditReport.fail(
            "INVALID_GRID", "expected core.Grid", scope=COLLISION_SCOPE,
            request_summary=base_summary, path_summary=path_summary_value,
            limits_summary=limits_summary_value, request_sha256=request_sha,
            check_count=1)
    context_result, context_failure = _audit_context(
        plan, request, grid, config, base_summary, path_summary_value,
        limits_summary_value, request_sha)
    if context_failure is not None:
        return context_failure
    context = context_result["context"]
    context_sha = context_result["context_sha"]
    grid_sha = context_result["grid_sha"]
    binding = bind_timed_plan(request, plan, context)
    if not binding.passed:
        return AuditReport.fail(
            binding.reason, binding.detail, scope=COLLISION_SCOPE,
            location=binding.location,
            request_summary=base_summary, path_summary=path_summary_value,
            limits_summary=limits_summary_value, request_sha256=request_sha,
            check_count=1, planning_context_sha256=context_sha,
            grid_sha256=grid_sha)
    if binding.metrics.get("model_sha256") != plan.model_sha256():
        return AuditReport.fail(
            "MODEL_BINDING_MISMATCH",
            "public model digest differs from bound core",
            scope=COLLISION_SCOPE,
            location={"constraint": "MODEL_DIGEST"},
            request_summary=base_summary, path_summary=path_summary_value,
            limits_summary=limits_summary_value, request_sha256=request_sha,
            check_count=1, planning_context_sha256=context_sha,
            grid_sha256=grid_sha)
    try:
        envelope = error_envelope(plan, config)
        checker = FormationCollisionChecker(
            grid, request.model,
            {vehicle_id: VehicleShape(*RESEARCH_VEHICLE_SHAPE_M)
             for vehicle_id in request.vehicle_ids},
            envelope["margin_used_m"])
        policy = SweepPolicy(
            max_corner_step_m=float(config.max_corner_step_m),
            max_samples_per_segment=int(config.max_samples_per_segment))
        stats_before = _checker_stats(checker)
        trace = []
        check_count = 0
        static_records = []
        if plan.kind == "hold":
            exact_state = plan.path.state_at_s(0.0)
            start_report = checker.static_check(
                exact_state.qf, _checker_headings(exact_state))
            static_records.append({
                "stage": "hold",
                "passed": bool(start_report.success),
                "reason": start_report.reason,
                "qf": [exact_state.qf.x, exact_state.qf.y, exact_state.qf.psi],
            })
            if not start_report.success:
                stats_after = _checker_stats(checker)
                return AuditReport.fail(
                    start_report.reason or "MAP_OBB_COLLISION",
                    "NO_MOTION hold failed the static map/vehicle check",
                    scope=COLLISION_SCOPE,
                    location={
                        "constraint": "COLLISION", "stage": "hold",
                        "vehicle_id": start_report.vehicle_id,
                        "other_vehicle_id": start_report.other_vehicle_id,
                    },
                    request_summary=base_summary,
                    path_summary=path_summary_value,
                    limits_summary=limits_summary_value,
                    request_sha256=request_sha,
                    check_count=(stats_after["map_queries"] - stats_before[
                        "map_queries"]) + (stats_after["obb_pair_queries"] -
                                           stats_before["obb_pair_queries"]),
                    planning_context_sha256=context_sha,
                    grid_sha256=grid_sha,
                    timing={"kind": "hold", "S_m": 0.0, "T_s": 0.0})
            stats_after = _checker_stats(checker)
            counts = {key: int(stats_after[key] - stats_before[key])
                      for key in stats_after}
            check_count = counts["map_queries"] + counts["obb_pair_queries"]
            return AuditReport.ok(
                scope=COLLISION_SCOPE,
                detail=(
                    "NO_MOTION hold passed the static map and vehicle-OBB "
                    "check; no sweep exists; executable only when the other "
                    "two layers pass"),
                request_summary=base_summary,
                path_summary=path_summary_value,
                limits_summary=limits_summary_value,
                whole_segment_bounds=(),
                integration={"static": static_records, "sweep": None},
                check_count=check_count,
                request_sha256=request_sha,
                planning_context_sha256=context_sha,
                grid_sha256=grid_sha,
                timing={
                    "kind": "hold", "S_m": 0.0, "T_s": 0.0,
                    "model_sha256": plan.model_sha256(),
                    "collision_checked": True,
                },
                metrics={
                    "kind": "hold",
                    "collision_checked": True,
                    "map_used": True,
                    "grid": context_result["grid"],
                    "grid_sha256": grid_sha,
                    "planning_context_sha256": context_sha,
                    "model_sha256": plan.model_sha256(),
                    "margin": envelope,
                    "checker_statistics": counts,
                    "static": static_records,
                    "sweep_segment_count": 0,
                    "byte_counts": {},
                })
        if plan.kind != "motion":
            return AuditReport.fail(
                "INVALID_PLAN", "plan kind={!r}".format(plan.kind),
                scope=COLLISION_SCOPE,
                request_summary=base_summary, path_summary=path_summary_value,
                limits_summary=limits_summary_value,
                request_sha256=request_sha,
                planning_context_sha256=context_sha, grid_sha256=grid_sha,
                check_count=1)
        segments = make_sweep_segments(plan, config)
        verification = verify_sweep_segments(plan, segments, config)
        if not verification.passed:
            return AuditReport.fail(
                verification.reason, verification.detail,
                scope=COLLISION_SCOPE, location=verification.location,
                request_summary=base_summary, path_summary=path_summary_value,
                limits_summary=limits_summary_value,
                request_sha256=request_sha, check_count=1,
                planning_context_sha256=context_sha, grid_sha256=grid_sha,
                timing={"kind": "motion", "S_m": plan.total_S_m,
                        "T_s": plan.duration_s})
        start_state = plan.path.state_at_s(0.0)
        start_report = checker.static_check(
            start_state.qf, _checker_headings(start_state))
        start_qf = [start_state.qf.x, start_state.qf.y, start_state.qf.psi]
        static_records.append({
            "stage": "start", "passed": bool(start_report.success),
            "reason": start_report.reason, "qf": start_qf,
        })
        if not start_report.success:
            stats_after = _checker_stats(checker)
            counts = {key: int(stats_after[key] - stats_before[key])
                      for key in stats_after}
            return AuditReport.fail(
                start_report.reason or "MAP_OBB_COLLISION",
                "static start pose failed the map/vehicle check",
                scope=COLLISION_SCOPE,
                location={
                    "constraint": "COLLISION", "stage": "static_start",
                    "vehicle_id": start_report.vehicle_id,
                    "other_vehicle_id": start_report.other_vehicle_id,
                    "qf": start_qf,
                },
                request_summary=base_summary,
                path_summary=path_summary_value,
                limits_summary=limits_summary_value,
                request_sha256=request_sha,
                check_count=counts["map_queries"] + counts[
                    "obb_pair_queries"],
                planning_context_sha256=context_sha,
                grid_sha256=grid_sha,
                timing={"kind": "motion", "S_m": plan.total_S_m,
                        "T_s": plan.duration_s},
                integration={"static": static_records},
                metrics={"margin": envelope,
                         "checker_statistics": counts})
        goal_state = plan.path.state_at_s(
            float(plan.path.length_m), side="left")
        goal_report = checker.static_check(
            goal_state.qf, _checker_headings(goal_state))
        goal_qf = [goal_state.qf.x, goal_state.qf.y, goal_state.qf.psi]
        static_records.append({
            "stage": "goal", "passed": bool(goal_report.success),
            "reason": goal_report.reason, "qf": goal_qf,
        })
        if not goal_report.success:
            stats_after = _checker_stats(checker)
            counts = {key: int(stats_after[key] - stats_before[key])
                      for key in stats_after}
            return AuditReport.fail(
                goal_report.reason or "MAP_OBB_COLLISION",
                "static goal pose failed the map/vehicle check",
                scope=COLLISION_SCOPE,
                location={
                    "constraint": "COLLISION", "stage": "static_goal",
                    "vehicle_id": goal_report.vehicle_id,
                    "other_vehicle_id": goal_report.other_vehicle_id,
                    "qf": goal_qf,
                },
                request_summary=base_summary,
                path_summary=path_summary_value,
                limits_summary=limits_summary_value,
                request_sha256=request_sha,
                check_count=counts["map_queries"] + counts[
                    "obb_pair_queries"],
                planning_context_sha256=context_sha,
                grid_sha256=grid_sha,
                timing={"kind": "motion", "S_m": plan.total_S_m,
                        "T_s": plan.duration_s},
                integration={"static": static_records},
                metrics={"margin": envelope,
                         "checker_statistics": counts})
        sweep_report = checker.swept_check_analytic(segments, policy, trace)
        stats_after = _checker_stats(checker)
        counts = {key: int(stats_after[key] - stats_before[key])
                  for key in stats_after}
        _enrich_trace(trace, plan)
        budgets = [entry for entry in trace if entry.get("kind") == "budget"]
        if not sweep_report.success or \
                sweep_report.reason == "SWEEP_SAMPLE_BUDGET_EXCEEDED":
            failure = sweep_report
            return AuditReport.fail(
                failure.reason or "COLLISION",
                "analytic sweep failed: {}".format(failure.detail or ""),
                scope=COLLISION_SCOPE,
                location={
                    "constraint": "COLLISION",
                    "stage": "analytic_sweep",
                    "reason": failure.reason,
                    "vehicle_id": failure.vehicle_id,
                    "other_vehicle_id": failure.other_vehicle_id,
                    "segment_index": failure.segment_index,
                    "sample_index": failure.sample_index,
                    "station": failure.station,
                    "time_s": failure.time_s,
                    "qf": None if failure.qf is None else [
                        failure.qf.x, failure.qf.y, failure.qf.psi],
                },
                request_summary=base_summary,
                path_summary=path_summary_value,
                limits_summary=limits_summary_value,
                request_sha256=request_sha,
                check_count=counts["map_queries"] + counts[
                    "obb_pair_queries"],
                planning_context_sha256=context_sha,
                grid_sha256=grid_sha,
                timing={"kind": "motion", "S_m": plan.total_S_m,
                        "T_s": plan.duration_s,
                        "model_sha256": plan.model_sha256()},
                integration={"static": static_records},
                metrics={
                    "margin": envelope,
                    "checker_statistics": counts,
                    "sweep_trace": trace,
                    "sweep_bounds": verification.metrics.get("segments", []),
                    "sweep_budget": budgets,
                })
        check_count = (counts["map_queries"] + counts["obb_pair_queries"]
                       + int(verification.metrics.get("segment_count", 0)) * 4)
        return AuditReport.ok(
            scope=COLLISION_SCOPE,
            detail=(
                "static start/goal plus unchanged analytic three-vehicle OBB "
                "sweep over the whole closed path; declared per-lambda bounds "
                "independently verified; conservative margin includes the "
                "numerical envelope; map and G4 context bound"),
            request_summary=base_summary,
            path_summary=path_summary_value,
            limits_summary=limits_summary_value,
            whole_segment_bounds=tuple(verification.metrics.get(
                "segments", [])),
            integration={
                "static": static_records,
                "sweep": {
                    "segment_count": len(segments),
                    "sample_count": counts["swept_samples_checked"],
                    "bounds_verified": True,
                    "policy": policy.__dict__,
                },
            },
            check_count=check_count,
            request_sha256=request_sha,
            planning_context_sha256=context_sha,
            grid_sha256=grid_sha,
            timing={
                "kind": "motion", "S_m": plan.total_S_m,
                "T_s": plan.duration_s,
                "model_sha256": plan.model_sha256(),
                "wall_time_s": time.monotonic() - started,
                "collision_checked": True,
            },
            metrics={
                "kind": "motion",
                "collision_checked": True,
                "map_used": True,
                "grid": context_result["grid"],
                "grid_sha256": grid_sha,
                "planning_context_sha256": context_sha,
                "model_sha256": plan.model_sha256(),
                "margin": envelope,
                "checker_statistics": counts,
                "static": static_records,
                "sweep_segment_count": len(segments),
                "sweep_sample_counts": {
                    str(entry.get("segment_index")): int(
                        entry.get("sample_count", 0)) for entry in budgets},
                "sweep_budget": budgets,
                "sweep_bounds": verification.metrics.get("segments", []),
                "sweep_trace": trace,
                "byte_counts": {
                    "trace_entries": len(trace),
                },
            })
    except Exception as exc:
        return AuditReport.fail(
            "COLLISION_NUMERICAL_FAILURE",
            "{}: {}".format(type(exc).__name__, exc),
            scope=COLLISION_SCOPE,
            location={"constraint": "COLLISION_NUMERICAL_FAILURE"},
            request_summary=base_summary, path_summary=path_summary_value,
            limits_summary=limits_summary_value, request_sha256=request_sha,
            check_count=1, planning_context_sha256=context_sha,
            grid_sha256=grid_sha)


__all__ = [
    "MAP_SCHEMA", "MAP_FORMAT", "RELATIVE_HEADING_BUDGET_RAD",
    "MapFormatError", "load_map", "write_map", "occupancy_from_pixels",
    "make_sweep_segments", "verify_sweep_segments", "error_envelope",
    "audit_collision",
]
