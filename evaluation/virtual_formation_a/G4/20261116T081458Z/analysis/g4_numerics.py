"""Independent numerical recheck of the G4 scene and benchmark evidence.

This script uses only the Python standard library.  It reparses every frozen
PGM/YAML map from raw bytes, rebuilds the conservative occupancy grid, and
recomputes the canonical G4 grid signature.  It then rechecks the endpoint,
rigidity, dynamic-utilization, collision-margin, envelope and per-lambda bound
records from the raw benchmark JSON.  It never imports the project and never
claims Codex acceptance.
"""
import datetime
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import struct

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
SCENES = ("S1", "S1_mirror", "S2", "S2_mirror", "S3", "S3_mirror")
TASK_BASE_MARGIN_M = 0.05
TASK_CORNER_STEP_M = 0.01
TASK_SWEEP_MOTION_PADDING_M = 0.01
AUDIT_POSITION_BUDGET_M = 5e-8
SAMPLES_PER_PANEL = 65
TASK_SAMPLE_BUDGET = 200000
TASK_ENDPOINT_M = 0.005
TASK_HEADING_RAD = 0.2 * math.pi / 180.0
TASK_RIGIDITY_M = 1e-6
VEHICLE_LENGTH_M = 0.738
VEHICLE_WIDTH_M = 0.500
DYNAMIC_LIMITS_RAD = tuple(
    name for name in (
        "acceleration", "jerk", "lateral_acceleration", "speed",
        "steer_acceleration", "steer_rate", "wheel_angle", "wheel_speed",
        "yaw_acceleration", "yaw_jerk", "yaw_rate"))
EXPECTED_MODEL = [[1, 1.0, 0.0], [2, 0.0, 1.0], [3, 0.0, -1.0]]
EXPECTED_SHAPES = [[1, 0.738, 0.500], [2, 0.738, 0.500],
                   [3, 0.738, 0.500]]
EXPECTED_LIMITS = {
    "a_max": 0.6,
    "jerk_max": 2.0,
    "lateral_accel_max": 0.6,
    "max_block_duration_s": 120.0,
    "min_mode_dwell_s": 0.3,
    "mode_setup_time": 0.15,
    "mode_switch_duration": 0.3,
    "r_min_ackermann": 0.81,
    "steer_accel_max": 1.5,
    "steer_angle_max": 1.5707963267948966,
    "steer_rate_max": 0.8,
    "track": 0.364,
    "v_max": 0.8,
    "wheel_speed_max": 1.0,
    "wheelbase": 0.494,
    "yaw_accel_max": 0.8,
    "yaw_jerk_max": 3.0,
    "yaw_rate_max": 0.8,
}
EXPECTED_TIMING_KEYS = (
    "route_generation_s", "geometry_solve_s", "retime_s",
    "dynamics_audit_s", "collision_audit_s", "result_assembly_s")


def sha256_bytes(payload):
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_json(value, ensure_ascii=False):
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"),
        ensure_ascii=ensure_ascii, allow_nan=False)


def next_up(value):
    value = float(value)
    if math.isnan(value) or math.isinf(value):
        return value
    if value == 0.0:
        return 5e-324
    bits = struct.unpack("<Q", struct.pack("<d", value))[0]
    bits = bits + 1 if value > 0.0 else bits - 1
    return struct.unpack("<d", struct.pack("<Q", bits))[0]


def read_pgm(path):
    data = Path(path).read_bytes()
    if not data.startswith(b"P5"):
        raise ValueError("not a P5 PGM")
    index = 2
    fields = []
    while len(fields) < 3:
        while index < len(data) and data[index:index + 1].isspace():
            index += 1
        if index < len(data) and data[index:index + 1] == b"#":
            while index < len(data) and data[index:index + 1] not in (
                    b"\n", b"\r"):
                index += 1
            continue
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        if start == index:
            raise ValueError("truncated PGM header")
        fields.append(data[start:index])
    if index >= len(data) or not data[index:index + 1].isspace():
        raise ValueError("missing raster separator")
    index += 1
    width, height, maxval = (int(field) for field in fields)
    expected = width * height
    pixels = data[index:index + expected]
    if len(pixels) != expected:
        raise ValueError("truncated raster")
    return width, height, maxval, pixels


def parse_simple_yaml(path):
    metadata = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        key, _, value = line.partition(":")
        metadata[key.strip()] = value.strip()
    image = metadata["image"].strip('"\'')
    origin = tuple(float(part.strip()) for part in
                   metadata["origin"].strip("[]").split(","))
    return {
        "image": image,
        "resolution": float(metadata["resolution"]),
        "origin": origin,
        "negate": int(metadata.get("negate", "0")),
        "occupied_thresh": float(metadata.get("occupied_thresh", "0.65")),
        "free_thresh": float(metadata.get("free_thresh", "0.196")),
    }


def grid_from_map(yaml_path):
    metadata = parse_simple_yaml(yaml_path)
    width, height, maxval, pixels = read_pgm(
        Path(yaml_path).parent / metadata["image"])
    occupied_rows = []
    for image_row in range(height - 1, -1, -1):
        row = []
        start = image_row * width
        for pixel in pixels[start:start + width]:
            probability = (pixel / float(maxval)) if metadata["negate"] else \
                ((float(maxval) - pixel) / float(maxval))
            row.append(0 if probability < metadata["free_thresh"] else 1)
        occupied_rows.append(row)
    flat = bytes(value for row in occupied_rows for value in row)
    summary = {
        "height": height,
        "width": width,
        "resolution_m": metadata["resolution"],
        "origin_x": metadata["origin"][0],
        "origin_y": metadata["origin"][1],
        "occupancy_sha256": sha256_bytes(flat),
        "cell_count": height * width,
        "occupied_cells": sum(flat),
    }
    signature = sha256_bytes(canonical_json(summary).encode("utf-8"))
    return summary, signature, flat


def check(condition, label, detail=None):
    if not condition:
        message = "FAILED: {}".format(label)
        if detail is not None:
            message += " ({})".format(detail)
        raise SystemExit(message)


def check_scene_manifest(manifest):
    document = dict(manifest)
    declared = document.pop("manifest_sha256", None)
    recomputed = sha256_bytes(
        canonical_json(document, ensure_ascii=True).encode("utf-8"))
    check(declared == recomputed,
          "scene manifest self-digest", (declared, recomputed))
    check(manifest.get("schema") ==
          "virtual_rigid_a.g4.scene_manifest.v1", "scene schema")
    check(manifest.get("stage") == "G4-A", "scene stage")
    check(manifest.get("frozen") is True, "scene manifest frozen")
    check(manifest.get("reference_verified") is True,
          "scene references verified")
    check(manifest.get("inputs_have_no_reference") is True,
          "inputs claim no reference")
    check(manifest.get("failures") == [], "scene failures empty")
    check(sorted(manifest["scenes"]) == sorted(SCENES), "scene set")
    check(manifest.get("base_commit") ==
          "567bdecba9676313da3bc9a97b82f57b5b569a2b", "base commit")
    collision = manifest["collision_config"]
    check(collision["baseline_margin_m"] == TASK_BASE_MARGIN_M,
          "scene baseline margin")
    check(collision["max_corner_step_m"] == TASK_CORNER_STEP_M,
          "scene corner step")
    check(collision["max_samples_per_segment"] == TASK_SAMPLE_BUDGET,
          "scene sample budget")
    checks = manifest["reference_verification"]
    check(len(checks) == 6, "reference verification count")
    for record in checks:
        check(record["passed"] is True,
              "reference verification {}".format(record["scene_id"]))
        check(all(record["checks"].values()),
              "reference verification subchecks {}".format(
                  record["scene_id"]))
    return recomputed


def check_input_bundle(run, name, scene, manifest):
    input_dir = run / "scenes" / scene["input_dir"]
    files = sorted(path.name for path in input_dir.iterdir()
                   if path.is_file())
    check(files == ["map.pgm", "map.yaml", "request.json"],
          "{} input bundle".format(name), files)
    check(not any("reference" in value.lower() for value in files),
          "{} input has no reference".format(name))
    payload = json.loads(
        (input_dir / "request.json").read_text(encoding="utf-8"))
    check(payload["schema"] == "virtual_rigid_a.g4.planning_context.v1",
          "{} payload schema".format(name))
    check(payload["stage"] == "G4", "{} payload stage".format(name))
    check(payload["map"] == {"image": "map.pgm", "yaml": "map.yaml"},
          "{} payload map names".format(name))
    check(payload["planner_config"] == manifest["planner_config"],
          "{} planner config frozen".format(name))
    check(payload["timing_config"] == manifest["timing_config"],
          "{} timing config frozen".format(name))
    check(payload["collision_config"] == manifest["collision_config"],
          "{} collision config frozen".format(name))
    request = payload["request"]
    check(request["schema"] == "virtual_rigid_a.g2.request.v1",
          "{} request schema".format(name))
    check(request["model"] == EXPECTED_MODEL, "{} model offsets".format(name))
    check(request["vehicle_shapes"] == EXPECTED_SHAPES,
          "{} vehicle shapes".format(name))
    check(request["limits"] == EXPECTED_LIMITS,
          "{} vehicle limits".format(name))
    check(request["initial"]["stationary"] is True,
          "{} initial stationary".format(name))
    check(request["initial"]["mode"] == "A", "{} initial A".format(name))
    check(request["initial"]["wheels_straight"] is True,
          "{} initial straight".format(name))
    check(request["goal_policy"] == "FREE", "{} FREE terminal".format(name))
    check(request["start_qf"] == [0.0, 0.0, 0.0],
          "{} start qf".format(name))
    check(request["start_headings"] == [[1, 0.0], [2, 0.0], [3, 0.0]],
          "{} start headings".format(name))
    map_summary, signature, flat = grid_from_map(input_dir / "map.yaml")
    check(signature == scene["map_sha256"],
          "{} raw map signature".format(name), (signature,
                                                scene["map_sha256"]))
    return {
        "files": files,
        "map_file_sha256": sha256_file(input_dir / "map.pgm"),
        "yaml_file_sha256": sha256_file(input_dir / "map.yaml"),
        "request_file_sha256": sha256_file(input_dir / "request.json"),
        "map_signature": signature,
        "map_summary": map_summary,
        "occupancy_bytes_sha256": sha256_bytes(flat),
        "request_sha256_declared": scene["request_sha256"],
        "planning_context_sha256_declared": scene["planning_context_sha256"],
    }


def check_reference(run, name, scene, manifest):
    reference = json.loads(
        (run / "scenes" / scene["reference_file"]).read_text(encoding="utf-8"))
    check(reference["schema"] == "virtual_rigid_a.g4.reference.v1",
          "{} reference schema".format(name))
    check(reference["status"] == "REFERENCE_PASSED",
          "{} reference status".format(name))
    check(reference["scene_id"] == name, "{} reference scene".format(name))
    for layer in ("geometry_audit", "dynamic_audit", "collision_audit"):
        check(reference[layer]["passed"] is True,
              "{} reference {} passed".format(name, layer))
        check(reference[layer]["check_count"] > 0,
              "{} reference {} checks".format(name, layer))
    check(reference["timing"]["status"] == "DYNAMIC_FEASIBLE",
          "{} reference timing".format(name))
    check(reference["independent"]["available"] is True,
          "{} independent reference available".format(name))
    check(reference["independent"]["accepted"] is True,
          "{} independent reference accepted".format(name))
    check(reference["grid_sha256"] == scene["map_sha256"],
          "{} reference grid digest".format(name))
    check(reference["request_sha256"] == scene["request_sha256"],
          "{} reference request digest".format(name))
    check(reference["planning_context_sha256"] ==
          scene["planning_context_sha256"],
          "{} reference context digest".format(name))
    check(reference["timed_model_sha256"] == scene["timed_model_sha256"],
          "{} reference model digest".format(name))
    integration = check_integration_audit(
        reference["geometry_integration"], reference["knots"],
        "{} reference".format(name))
    envelope = check_envelope(
        reference["collision_margin"],
        "{} reference".format(name))
    return {
        "status": reference["status"],
        "reference_file": scene["reference_file"],
        "reference_sha256": sha256_file(run / "scenes" / scene["reference_file"]),
        "timed_model_sha256": reference["timed_model_sha256"],
        "geometry_checks": reference["geometry_audit"]["check_count"],
        "dynamic_checks": reference["dynamic_audit"]["check_count"],
        "collision_checks": reference["collision_audit"]["check_count"],
        "endpoint_difference_m": reference["independent"][
            "endpoint_difference"]["norm_m"],
        "independent_accepted": reference["independent"]["accepted"],
        "integration": integration,
        "envelope": envelope,
    }


def check_envelope(margin, label):
    """Recheck the factor-2 per-side envelope with exact rational closure.

    The legacy checker half extent is ``(dimension + margin) / 2``.  A margin
    increase therefore grows each side by half of the increase.  The exact
    rational comparison below uses ``fractions.Fraction`` on the recorded
    binary64 values, so it cannot hide a one-ulp deficit in a float
    comparison, and it is independent of the production code path.
    """
    baseline = margin["baseline_margin_m"]
    used = margin["margin_used_m"]
    maximum = margin["maximum_envelope_m"]
    check(baseline == TASK_BASE_MARGIN_M,
          "{} baseline margin".format(label), baseline)
    check(used >= TASK_BASE_MARGIN_M,
          "{} margin floor".format(label), used)
    check(len(margin["vehicles"]) == 3, "{} envelope vehicles".format(label))
    recomputed_maximum = max(
        record["envelope_m"] for record in margin["vehicles"])
    check(recomputed_maximum == maximum,
          "{} maximum envelope".format(label),
          (recomputed_maximum, maximum))
    per_side = margin["per_side_margin_m"]
    half_length = (VEHICLE_LENGTH_M + used) / 2.0
    half_width = (VEHICLE_WIDTH_M + used) / 2.0
    baseline_half_length = (VEHICLE_LENGTH_M + baseline) / 2.0
    baseline_half_width = (VEHICLE_WIDTH_M + baseline) / 2.0
    check(margin["checker_half_length_m"] == half_length,
          "{} checker half length".format(label),
          (margin["checker_half_length_m"], half_length))
    check(margin["checker_half_width_m"] == half_width,
          "{} checker half width".format(label),
          (margin["checker_half_width_m"], half_width))
    check(margin["baseline_half_length_m"] == baseline_half_length,
          "{} baseline half length".format(label))
    check(margin["baseline_half_width_m"] == baseline_half_width,
          "{} baseline half width".format(label))
    exact_length_growth = (
        Fraction(half_length) - Fraction(baseline_half_length))
    exact_width_growth = (
        Fraction(half_width) - Fraction(baseline_half_width))
    exact_per_side_growth = min(exact_length_growth, exact_width_growth)
    check(Fraction(per_side) <= exact_per_side_growth,
          "{} per-side record is a conservative lower bound".format(label),
          (per_side, float(exact_per_side_growth)))
    check(exact_per_side_growth >= Fraction(maximum),
          "{} exact checker per-side growth covers maximum envelope".format(
              label),
          (float(exact_per_side_growth), maximum))
    exact_margin_growth = Fraction(used) - Fraction(baseline)
    check(Fraction(margin["margin_growth_lower_m"]) <= exact_margin_growth,
          "{} margin growth record is conservative".format(label))
    exact_required_margin = Fraction(baseline) + 2 * Fraction(maximum)
    check(Fraction(used) >= exact_required_margin,
          "{} exact margin >= baseline + 2*max(envelope)".format(label),
          (used, float(exact_required_margin)))
    check(Fraction(margin["margin_minimum_required_m"]) >=
          exact_required_margin,
          "{} recorded minimum required margin is conservative".format(label))
    check(used >= margin["margin_minimum_required_m"],
          "{} used margin dominates recorded minimum".format(label))
    radius_upper = margin["radius_upper_m"]
    radius_square = Fraction(radius_upper) ** 2
    exact_radius_square = Fraction(half_length) ** 2 + Fraction(
        half_width) ** 2
    check(radius_square >= exact_radius_square,
          "{} radius upper covers actual checker half extents".format(label),
          (radius_upper, float(math.sqrt(exact_radius_square))))
    check(margin["checker_radius_upper_m"] == radius_upper,
          "{} checker radius upper matches expanded radius".format(label))
    check(margin["maximum_record"]["inflated_body_radius_upper_m"] ==
          radius_upper, "{} maximum envelope uses expanded radius".format(
              label))
    for record in margin["vehicles"]:
        recomputed = next_up(
            record["eps_position_m"] +
            next_up((record["offset_m"] +
                     record["inflated_body_radius_upper_m"]) *
                    record["eps_formation_heading_rad"]) +
            next_up(record["inflated_body_radius_upper_m"] *
                    record["eps_relative_heading_rad"]))
        check(recomputed == record["envelope_m"],
              "{} vehicle {} envelope".format(
                  label, record["vehicle_id"]),
              (recomputed, record["envelope_m"]))
        check(record["eps_relative_heading_rad"] == 1e-10,
              "{} relative heading budget".format(label))
        check(record["offset_m"] >=
              record["offset_hypot_float_m"],
              "{} vehicle {} offset upper bound".format(
                  label, record["vehicle_id"]))
        check(record["inflated_body_radius_upper_m"] == radius_upper,
              "{} vehicle {} expanded radius".format(
                  label, record["vehicle_id"]))
    envelopes = sorted(
        (record["vehicle_id"], record["envelope_m"])
        for record in margin["vehicles"])
    pair_envelope = Fraction(0)
    pair_ids = None
    for index in range(len(envelopes)):
        for other in range(index + 1, len(envelopes)):
            candidate = (Fraction(envelopes[index][1]) +
                         Fraction(envelopes[other][1]))
            if candidate > pair_envelope:
                pair_envelope = candidate
                pair_ids = (envelopes[index][0], envelopes[other][0])
    recorded_pair = margin["pair_envelope_required_m"]
    check(Fraction(recorded_pair) >= pair_envelope,
          "{} recorded pair envelope is conservative".format(label),
          (recorded_pair, float(pair_envelope)))
    check(Fraction(margin["margin_growth_lower_m"]) >=
          Fraction(recorded_pair),
          "{} two-body gap reduction covers pair envelope".format(label))
    contexts = margin["contexts"]
    for name in ("static_start_goal", "no_motion_hold", "motion_sweep",
                 "map_boundary", "vehicle_vehicle"):
        check(name in contexts, "{} context {}".format(label, name))
        check(contexts[name]["covered"] is True,
              "{} context {} covered".format(label, name))
    check(contexts["static_start_goal"]["checker_padding_m"] == 0.0,
          "{} static padding".format(label))
    check(contexts["no_motion_hold"]["checker_padding_m"] == 0.0,
          "{} no-motion padding".format(label))
    check(contexts["map_boundary"]["checker_padding_m"] == 0.0,
          "{} map padding".format(label))
    check(contexts["motion_sweep"]["checker_padding_m"] ==
          TASK_SWEEP_MOTION_PADDING_M,
          "{} sweep padding".format(label))
    check(margin["sweep_motion_padding_m"] ==
          TASK_SWEEP_MOTION_PADDING_M,
          "{} sweep padding field".format(label))
    check(contexts["motion_sweep"]["required_per_side_m"] == maximum,
          "{} sweep uses numerical envelope only".format(label))
    check(contexts["vehicle_vehicle"]["required_pair_envelope_m"] ==
          recorded_pair, "{} pair context envelope".format(label))
    return {
        "baseline_margin_m": baseline,
        "maximum_envelope_m": maximum,
        "margin_used_m": used,
        "margin_minus_floor_m": used - baseline,
        "per_side_growth_record_m": per_side,
        "exact_per_side_growth_m": float(exact_per_side_growth),
        "exact_required_margin_m": float(exact_required_margin),
        "closure_gap_m": float(exact_per_side_growth - Fraction(maximum)),
        "pair_envelope_m": float(pair_envelope),
        "binding_pair": pair_ids,
        "vehicles": len(margin["vehicles"]),
        "radius_upper_m": radius_upper,
        "exact_half_diagonal_m": float(math.sqrt(exact_radius_square)),
        "contexts": sorted(contexts),
        "sweep_motion_padding_m": margin["sweep_motion_padding_m"],
        "radius_upper_note": margin.get("radius_upper_note"),
    }


def check_sweep(audit, label):
    integration = audit["integration"]
    sweep = integration["sweep"]
    check(sweep["bounds_verified"] is True,
          "{} sweep bounds verified".format(label))
    check(sweep["policy"]["max_corner_step_m"] == TASK_CORNER_STEP_M,
          "{} sweep corner step".format(label))
    check(sweep["policy"]["max_samples_per_segment"] == TASK_SAMPLE_BUDGET,
          "{} sweep sample budget".format(label))
    bounds = audit["metrics"]["sweep_bounds"]
    check(len(bounds) == sweep["segment_count"],
          "{} sweep segment count".format(label))
    check(sweep["sample_count"] > 0, "{} sweep samples".format(label))
    check(audit["check_count"] > 0, "{} collision checks".format(label))
    check(audit["metrics"]["kind"] == "motion",
          "{} collision kind".format(label))
    check(audit["metrics"]["map_used"] is True, "{} map used".format(label))
    check(audit["metrics"]["collision_checked"] is True,
          "{} collision checked".format(label))
    stages = {record["stage"]: record
              for record in audit["metrics"]["static"]}
    check(sorted(stages) == ["goal", "start"],
          "{} static stages".format(label), sorted(stages))
    check(all(record["passed"] is True for record in stages.values()),
          "{} static pass".format(label))
    previous_end = 0.0
    for index, record in enumerate(bounds):
        check(record["verified"] is True,
              "{} bound {} verified".format(label, index))
        check(record["index"] == index,
              "{} bound {} index".format(label, index))
        check(record["s0"] == previous_end,
              "{} bound {} contiguity".format(label, index),
              (record["s0"], previous_end))
        check(record["s1"] > record["s0"],
              "{} bound {} positive interval".format(label, index))
        check(record["delta_m"] == record["s1"] - record["s0"],
              "{} bound {} delta".format(label, index))
        check(record["declared_center_speed_bound"] >=
              record["independent_center_speed_lower_bound"],
              "{} bound {} center".format(label, index))
        check(record["declared_formation_rate_bound"] >=
              record["independent_formation_rate_lower_bound"],
              "{} bound {} formation".format(label, index))
        for vehicle_id, value in record[
                "declared_body_rate_bounds"].items():
            check(value >= record[
                "independent_body_rate_lower_bounds"][vehicle_id],
                "{} bound {} vehicle {}".format(
                    label, index, vehicle_id))
        previous_end = record["s1"]
    return {
        "segment_count": sweep["segment_count"],
        "sample_count": sweep["sample_count"],
        "bounds_verified": sweep["bounds_verified"],
        "max_corner_step_m": sweep["policy"]["max_corner_step_m"],
        "max_samples_per_segment":
            sweep["policy"]["max_samples_per_segment"],
        "static_stages": sorted(stages),
        "check_count": audit["check_count"],
    }


def check_dynamics(audit, label):
    metrics = audit["metrics"]
    check(metrics["motion"] is True, "{} dynamic motion".format(label))
    check(metrics["whole_domain_certificate"] is True,
          "{} whole-domain certificate".format(label))
    check(metrics["leaves"] == 64, "{} dynamic leaves".format(label))
    utilization = metrics["max_utilization"]
    check(sorted(utilization) == sorted(DYNAMIC_LIMITS_RAD),
          "{} utilization keys".format(label), sorted(utilization))
    check(max(utilization.values()) <= 1.0,
          "{} dynamic utilization".format(label),
          max(utilization.values()))
    pointwise = metrics["pointwise_interval_checks"]
    check(pointwise["max_component_ratio"] <= 1.0,
          "{} pointwise component".format(label))
    check(pointwise["max_jerk_ratio"] <= 1.0,
          "{} pointwise jerk".format(label))
    check(pointwise["max_p_sss_ratio"] <= 1.0,
          "{} pointwise p_sss".format(label))
    check(pointwise["leaf_count"] == 64,
          "{} pointwise leaves".format(label))
    check(pointwise["vehicle_count"] == 192,
          "{} pointwise vehicles".format(label))
    return {
        "whole_domain_certificate": True,
        "leaves": metrics["leaves"],
        "max_utilization": max(utilization.values()),
        "pointwise_component_ratio":
            pointwise["max_component_ratio"],
        "pointwise_jerk_ratio": pointwise["max_jerk_ratio"],
        "pointwise_p_sss_ratio": pointwise["max_p_sss_ratio"],
        "check_count": audit["check_count"],
    }


def _hermite_pieces(knots):
    """Rebuild the C2 Hermite curvature pieces from serialised knots."""
    pieces = []
    for index in range(len(knots) - 1):
        left = knots[index]
        right = knots[index + 1]
        s0 = float(left["s"])
        s1 = float(right["s"])
        h = s1 - s0
        check(h > 0.0, "integration knot spacing", (index, h))
        k0 = float(left["value"])
        k1 = float(right["value"])
        eta0 = float(left["derivative"])
        eta1 = float(right["derivative"])
        pieces.append({
            "s0": s0,
            "s1": s1,
            "h": h,
            "c0": k0,
            "c1": h * eta0,
            "c2": 3.0 * (k1 - k0) - h * (2.0 * eta0 + eta1),
            "c3": 2.0 * (k0 - k1) + h * (eta0 + eta1),
        })
    return pieces


def _actual_fourth_derivative(piece, q):
    h = piece["h"]
    k = (piece["c0"] + q * (piece["c1"] + q * (
        piece["c2"] + q * piece["c3"])))
    e = (piece["c1"] + q * (2.0 * piece["c2"] +
                            3.0 * q * piece["c3"])) / h
    u = (2.0 * piece["c2"] + 6.0 * q * piece["c3"]) / (h * h)
    g = 6.0 * piece["c3"] / (h * h * h)
    real = k ** 4 - 4.0 * k * u - 3.0 * e * e
    imaginary = g - 6.0 * k * k * e
    return math.hypot(real, imaginary)


def _panel_remainder_independent(width, m4):
    fifth = next_up(next_up(next_up(next_up(
        width * width) * width) * width) * width)
    return next_up(next_up(fifth * m4) / 2880.0)


def check_integration_audit(payload, knots, label):
    """Independent recheck of the 5e-8 m exact-panel Simpson budget.

    The segment coefficients are rebuilt from the serialised knots.  The
    panel M4 bounds recorded by the audit are checked against dense samples
    of the actual fourth derivative; the budget arithmetic is recomputed
    from the recorded panel bounds with the documented outward rounding.
    The interval proof remains the authority; sampling is corroboration, not
    a whole-domain proof.
    """
    audit = payload["audit_integration"]
    pieces = _hermite_pieces(knots)
    check(audit["method"] == "independent_composite_simpson_v2_exact_panel",
          "{} integration method".format(label), audit["method"])
    check(audit["m4_method"] ==
          "exact_interval_fourth_derivative_per_panel",
          "{} integration M4 method".format(label))
    check(audit["fixed_threshold_m"] == AUDIT_POSITION_BUDGET_M,
          "{} integration threshold".format(label),
          audit["fixed_threshold_m"])
    check(audit["confirmed"] is True, "{} integration confirmed".format(label))
    check(audit["total_budget_m"] <= AUDIT_POSITION_BUDGET_M,
          "{} integration total budget <= 5e-8".format(label),
          audit["total_budget_m"])
    check(len(audit["segments"]) == len(pieces),
          "{} integration segment count".format(label),
          (len(audit["segments"]), len(pieces)))
    total_subintervals = 0
    full_panels = 0
    local_panels = 0
    accumulated = 0.0
    sampled_max_actual = 0.0
    sampled_bound_deficit = 0.0
    for index, (segment, piece) in enumerate(zip(audit["segments"],
                                                 pieces)):
        check(segment["segment_index"] == index,
              "{} integration segment index {}".format(label, index))
        check(segment["s0"] == piece["s0"] and
              segment["s1"] == piece["s1"],
              "{} integration segment domain {}".format(label, index))
        check(segment["h"] == piece["h"],
              "{} integration segment height {}".format(label, index))
        subintervals = segment["subintervals"]
        check(isinstance(subintervals, int) and subintervals >= 2 and
              subintervals % 2 == 0,
              "{} integration even subintervals {}".format(label, index))
        panels = segment["panel_m4_upper_m"]
        check(len(panels) == subintervals // 2,
              "{} integration panel count {}".format(label, index))
        check(all(math.isfinite(value) and value >= 0.0 for value in panels),
              "{} integration panel bounds".format(label, index))
        maximum = max(panels)
        check(segment["maximum_panel_m4_upper_m"] == maximum,
              "{} integration maximum panel M4 {}".format(label, index))
        check(segment["coarse_triangle_m4_upper_m"] >= maximum,
              "{} integration coarse bound encloses panels".format(
                  label, index))
        width = next_up(2.0 * segment["h"] / subintervals)
        full = 0.0
        for m4 in panels:
            full = next_up(full + _panel_remainder_independent(width, m4))
        local = next_up(_panel_remainder_independent(width, maximum))
        check(segment["full_remainder_m"] == full,
              "{} integration full remainder {}".format(label, index),
              (segment["full_remainder_m"], full))
        check(segment["local_remainder_m"] == local,
              "{} integration local remainder {}".format(label, index),
              (segment["local_remainder_m"], local))
        check(segment["remainder_m"] == next_up(full + local),
              "{} integration segment remainder {}".format(label, index))
        for panel_index, bound in enumerate(panels):
            q0 = 2.0 * panel_index / subintervals
            q1 = 2.0 * (panel_index + 1) / subintervals
            actual = 0.0
            for sample in range(SAMPLES_PER_PANEL):
                q = q0 + (q1 - q0) * sample / float(
                    SAMPLES_PER_PANEL - 1)
                actual = max(actual, _actual_fourth_derivative(piece, q))
            sampled_max_actual = max(sampled_max_actual, actual)
            sampled_bound_deficit = max(
                sampled_bound_deficit, actual - bound)
            check(bound >= actual,
                  "{} integration panel bound encloses sample {}".format(
                      label, index),
                  (index, panel_index, bound, actual))
        accumulated = next_up(accumulated + segment["remainder_m"])
        total_subintervals += subintervals
        full_panels += subintervals // 2
        local_panels += 1
    check(total_subintervals == audit["total_subintervals"],
          "{} integration total subintervals".format(label))
    check(full_panels == audit["full_simpson_panels"],
          "{} integration full panels".format(label))
    check(local_panels == audit["local_simpson_panels"],
          "{} integration local panels".format(label))
    check(accumulated == audit["per_coordinate_remainder_m"],
          "{} integration per-coordinate remainder".format(label),
          (accumulated, audit["per_coordinate_remainder_m"]))
    euclidean = next_up(next_up(math.sqrt(2.0)) * accumulated)
    check(euclidean == audit["euclidean_remainder_m"],
          "{} integration euclidean remainder".format(label),
          (euclidean, audit["euclidean_remainder_m"]))
    length = float(knots[-1]["s"])
    angle_bound = audit["angle_bound_rad"]
    guard = next_up(
        1e-10 * (full_panels + local_panels + 8) +
        next_up(length * angle_bound))
    check(guard == audit["roundoff_and_angle_guard_m"],
          "{} integration floating/angle guard".format(label),
          (guard, audit["roundoff_and_angle_guard_m"]))
    check(next_up(euclidean + guard) == audit["total_budget_m"],
          "{} integration total composition".format(label),
          (next_up(euclidean + guard), audit["total_budget_m"]))
    check(audit["total_budget_m"] <= AUDIT_POSITION_BUDGET_M,
          "{} integration budget threshold".format(label))
    return {
        "method": audit["method"],
        "m4_method": audit["m4_method"],
        "fixed_threshold_m": audit["fixed_threshold_m"],
        "total_budget_m": audit["total_budget_m"],
        "per_coordinate_remainder_m":
            audit["per_coordinate_remainder_m"],
        "euclidean_remainder_m": audit["euclidean_remainder_m"],
        "roundoff_and_angle_guard_m":
            audit["roundoff_and_angle_guard_m"],
        "total_subintervals": audit["total_subintervals"],
        "full_simpson_panels": audit["full_simpson_panels"],
        "local_simpson_panels": audit["local_simpson_panels"],
        "subintervals": [segment["subintervals"]
                         for segment in audit["segments"]],
        "refinement_steps": audit["refinement_steps"],
        "sampled_max_actual_m4": sampled_max_actual,
        "sampled_bound_deficit_m4": sampled_bound_deficit,
        "samples_per_panel": SAMPLES_PER_PANEL,
    }


def check_geometry(audit, label):
    sample = audit["metrics"]["sample_audit"]
    check(sample["sample_count"] > 0, "{} geometry samples".format(label))
    check(sample["max_position_error_m"] <= TASK_ENDPOINT_M,
          "{} geometry position".format(label))
    check(sample["max_heading_error_rad"] <= TASK_HEADING_RAD,
          "{} geometry heading".format(label))
    check(sample["max_pairwise_rigidity_error_m"] <= TASK_RIGIDITY_M,
          "{} geometry rigidity".format(label))
    check(sample["position_tolerance_m"] <= TASK_ENDPOINT_M,
          "{} geometry tolerance".format(label))
    return {
        "sample_count": sample["sample_count"],
        "max_position_error_m": sample["max_position_error_m"],
        "max_heading_error_rad": sample["max_heading_error_rad"],
        "max_pairwise_rigidity_error_m":
            sample["max_pairwise_rigidity_error_m"],
        "position_tolerance_m": sample["position_tolerance_m"],
        "check_count": audit["check_count"],
    }


def check_result(name, result, map_record):
    check(result["status"] == "SUCCESS",
          "{} status".format(name), result["status"])
    check(result["executable"] is True, "{} executable".format(name))
    check(result["grid_sha256"] == map_record["map_signature"],
          "{} result grid".format(name))
    check(result["manifest_map_match"] is True,
          "{} manifest map match".format(name))
    check(result["manifest_request_match"] is True,
          "{} manifest request match".format(name))
    check(result["reference_free_bundle"] is True,
          "{} reference-free bundle".format(name))
    check(result["endpoint_error_m"] <= TASK_ENDPOINT_M,
          "{} endpoint".format(name), result["endpoint_error_m"])
    check(result["heading_error_rad"] <= TASK_HEADING_RAD,
          "{} heading".format(name), result["heading_error_rad"])
    check(result["rigidity_error_m"] <= TASK_RIGIDITY_M,
          "{} rigidity".format(name), result["rigidity_error_m"])
    audits = result["audits"]
    # The three audit payloads bind the map, request and planning context.
    # Geometry audit is produced before the timed model exists, so its
    # model_sha256 field is null by design; dynamics and collision are only
    # produced after retiming and do carry the model binding.
    for layer in ("geometry", "dynamics", "collision"):
        check(audits[layer]["passed"] is True,
              "{} {} audit".format(name, layer))
        check(audits[layer]["check_count"] > 0,
              "{} {} checks".format(name, layer))
        check(audits[layer]["request_sha256"] == result["request_sha256"],
              "{} {} request binding".format(name, layer))
        check(audits[layer]["grid_sha256"] == result["grid_sha256"],
              "{} {} grid binding".format(name, layer))
        check(audits[layer]["planning_context_sha256"] ==
              result["planning_context_sha256"],
              "{} {} context binding".format(name, layer))
    checks = sum(audits[layer]["check_count"]
                 for layer in ("geometry", "dynamics", "collision"))
    check(result["checks"] == checks, "{} check sum".format(name),
          (result["checks"], checks))
    check(result["checks"] > 0, "{} checks positive".format(name))
    check(result["metrics"]["collision_checks"] ==
          audits["collision"]["check_count"],
          "{} collision check metric".format(name))
    model_text = result["model_json"]
    check(hashlib.sha256(model_text.encode("utf-8")).hexdigest() ==
          result["model_sha256"], "{} model digest".format(name))
    model = json.loads(model_text)
    check(model["request_sha256"] == result["request_sha256"],
          "{} model request".format(name))
    check(model["timing"]["kind"] == "motion",
          "{} model timing kind".format(name))
    check(model["timing"]["T_s"] == result["metrics"]["duration_s"],
          "{} model duration".format(name))
    check(model["path"]["length_m"] == result["metrics"]["path_length_m"],
          "{} model path length".format(name))
    check(model["planning_context"]["schema"] ==
          "virtual_rigid_a.g4.planning_context.v1",
          "{} model context schema".format(name))
    check(model["planning_context"]["request_sha256"] ==
          result["request_sha256"], "{} model context request".format(name))
    check(model["planning_context"]["grid"]["occupancy_sha256"] ==
          map_record["map_summary"]["occupancy_sha256"],
          "{} model context grid".format(name))
    check(model["numerics"]["position_error_bound_m"] ==
          audits["collision"]["metrics"]["margin"]["vehicles"][0][
              "eps_position_m"], "{} model position bound".format(name))
    check(model["path"]["position_error_bound_m"] ==
          audits["collision"]["metrics"]["margin"]["vehicles"][0][
              "eps_position_m"], "{} path position bound".format(name))
    timing_values = result["timings"]
    for key in EXPECTED_TIMING_KEYS:
        check(key in timing_values, "{} timing {}".format(name, key))
        check(timing_values[key] >= 0.0,
              "{} timing nonnegative {}".format(name, key))
    check(sum(timing_values[key] for key in EXPECTED_TIMING_KEYS) <=
          result["wall_time_s"] + 1e-9, "{} timing sum".format(name))
    budget = result["search"]["budget"]
    check(budget["total_soft_budget_s"] == 60.0,
          "{} total budget".format(name))
    check(budget["max_collision_retries"] == 2,
          "{} retry budget".format(name))
    check(budget["hard_real_time_guarantee"] is False,
          "{} real-time nonclaim".format(name))
    check(0 < len(result["routes"]) <= 3, "{} route count".format(name))
    for route in result["routes"]:
        check(route["grid_sha256"] == map_record["map_signature"],
              "{} route grid".format(name))
        check(len(route["points"]) >= 2, "{} route points".format(name))
        check(route["length_m"] >= 0.0, "{} route length".format(name))
    geometry = check_geometry(audits["geometry"], name)
    integration = check_integration_audit(
        audits["geometry"]["integration"],
        model["path"]["profile"]["knots"], name)
    dynamics = check_dynamics(audits["dynamics"], name)
    collision = check_sweep(audits["collision"], name)
    envelope = check_envelope(
        audits["collision"]["metrics"]["margin"], name)
    check(collision["check_count"] == result["metrics"][
        "collision_checks"], "{} collision count metric".format(name))
    return {
        "status": result["status"],
        "executable": result["executable"],
        "checks": result["checks"],
        "endpoint_error_m": result["endpoint_error_m"],
        "heading_error_rad": result["heading_error_rad"],
        "rigidity_error_m": result["rigidity_error_m"],
        "attempt_count": len(result["attempts"]),
        "route_count": len(result["routes"]),
        "search_rounds": len(result["search"]["rounds"]),
        "wall_time_s": result["wall_time_s"],
        "timings": timing_values,
        "model_sha256": result["model_sha256"],
        "geometry": geometry,
        "integration": integration,
        "dynamics": dynamics,
        "collision": collision,
        "envelope": envelope,
    }


def main():
    manifest_path = RUN / "scenes/scene_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest_digest = check_scene_manifest(manifest)
    summary = json.loads(
        (RUN / "benchmark/summary.json").read_text(encoding="utf-8"))
    results = json.loads(
        (RUN / "benchmark/results.json").read_text(encoding="utf-8"))
    check(summary["schema"] == "virtual_rigid_a.g4.benchmark.v1",
          "benchmark schema")
    check(summary["stage"] == "G4", "benchmark stage")
    check(summary["status"] == "PASS", "benchmark status")
    check(summary["scene_count"] == 6, "benchmark scene count")
    check(summary["manifest_sha256"] == manifest_digest,
          "benchmark manifest digest")
    check(summary["codex_acceptance"] == "NOT_RUN_NOT_CLAIMED",
          "benchmark acceptance nonclaim")
    check(len(summary["checks"]) == 84, "benchmark check count",
          len(summary["checks"]))
    check(all(record["pass"] is True for record in summary["checks"]),
          "benchmark all checks passed")
    check(sorted(results) == sorted(SCENES), "benchmark result scenes")

    scene_records = {}
    for name in SCENES:
        scene = manifest["scenes"][name]
        check(scene["status"] == "REFERENCE_PASSED",
              "{} manifest status".format(name))
        map_record = check_input_bundle(RUN, name, scene, manifest)
        reference = check_reference(RUN, name, scene, manifest)
        autonomous = check_result(name, results[name], map_record)
        check(results[name]["request_sha256"] == scene["request_sha256"],
              "{} benchmark request digest".format(name))
        check(results[name]["planning_context_sha256"] ==
              scene["planning_context_sha256"],
              "{} benchmark context digest".format(name))
        scene_records[name] = {
            "map": map_record,
            "reference": reference,
            "autonomous": autonomous,
        }

    document = {
        "schema": "virtual_rigid_a.g4.numerics.v1",
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN.name,
        "source": {
            "scene_manifest": "scenes/scene_manifest.json",
            "scene_manifest_sha256": sha256_file(manifest_path),
            "scene_manifest_self_digest": manifest_digest,
            "benchmark_summary": "benchmark/summary.json",
            "benchmark_summary_sha256":
                sha256_file(RUN / "benchmark/summary.json"),
            "benchmark_results": "benchmark/results.json",
            "benchmark_results_sha256":
                sha256_file(RUN / "benchmark/results.json"),
        },
        "independent_method": [
            "PGM P5 and simple YAML reparsed from raw bytes with the "
            "standard library; occupancy follows the conservative ROS "
            "convention free only when probability < free_thresh",
            "canonical G4 grid summary and SHA-256 recomputed without the "
            "project module",
            "collision error envelope recomputed from the recorded binary64 fields; "
            "the per-side closure (margin-baseline)/2 >= max(e_i) and the "
            "two-body gap reduction are verified with exact Fraction "
            "arithmetic on the recorded floats, and radius_upper is checked "
            "by squared rational enclosure of the actual checker half extents",
            "independent Simpson audit budget rebuilt from the serialised "
            "C2 knots and recorded per-panel M4 bounds with the documented "
            "outward rounding; dense samples of the actual fourth derivative "
            "are required to lie below every recorded panel bound, and the "
            "fixed threshold is 5e-8 m",
            "per-lambda declared bounds checked against the independent "
            "lower bounds recorded by the adapter; no segment self-report "
            "is trusted alone",
            "dynamic max-utilization, whole-domain certificate and all three "
            "pointwise interval ratios checked below one",
        ],
        "thresholds": {
            "endpoint_m": TASK_ENDPOINT_M,
            "heading_rad": TASK_HEADING_RAD,
            "rigidity_m": TASK_RIGIDITY_M,
            "baseline_margin_m": TASK_BASE_MARGIN_M,
            "max_corner_step_m": TASK_CORNER_STEP_M,
            "sweep_motion_padding_m": TASK_SWEEP_MOTION_PADDING_M,
            "max_samples_per_segment": TASK_SAMPLE_BUDGET,
            "audit_position_budget_m": AUDIT_POSITION_BUDGET_M,
        },
        "scenes": scene_records,
        "status": "PASS",
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        "nonclaims": [
            "six frozen known-feasible A-only scenes only",
            "no general completeness, optimality or real-time claim",
            "no ROS, Gazebo or hardware validation",
            "numerical checks are not Codex acceptance",
        ],
    }
    target = RUN / "analysis/g4_numerics.json"
    target.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "status": "PASS",
        "path": str(target.relative_to(REPO_ROOT)),
        "scenes": len(scene_records),
        "max_margin_m": max(
            record["autonomous"]["envelope"]["margin_used_m"]
            for record in scene_records.values()),
        "max_endpoint_m": max(
            record["autonomous"]["endpoint_error_m"]
            for record in scene_records.values()),
        "max_rigidity_m": max(
            record["autonomous"]["rigidity_error_m"]
            for record in scene_records.values()),
        "max_dynamic_utilization": max(
            record["autonomous"]["dynamics"]["max_utilization"]
            for record in scene_records.values()),
        "max_audit_budget_m": max(
            record["autonomous"]["integration"]["total_budget_m"]
            for record in scene_records.values()),
        "max_envelope_m": max(
            record["autonomous"]["envelope"]["maximum_envelope_m"]
            for record in scene_records.values()),
        "min_per_side_closure_gap_m": min(
            min(record["reference"]["envelope"]["closure_gap_m"],
                record["autonomous"]["envelope"]["closure_gap_m"])
            for record in scene_records.values()),
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()