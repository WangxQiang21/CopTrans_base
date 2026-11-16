"""Independent numerical recheck of the G4 scene and benchmark evidence.

This script uses only the Python standard library.  It reparses every frozen
PGM/YAML map from raw bytes, rebuilds the conservative occupancy grid, and
recomputes the canonical G4 grid signature.  It then rechecks the endpoint,
rigidity, dynamic-utilization, collision-margin, envelope and per-lambda bound
records from the raw benchmark JSON.  It never imports the project and never
claims Codex acceptance.
"""
import datetime
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
    }


def check_envelope(margin, label):
    check(margin["baseline_margin_m"] == TASK_BASE_MARGIN_M,
          "{} baseline margin".format(label), margin["baseline_margin_m"])
    check(margin["margin_used_m"] >= TASK_BASE_MARGIN_M,
          "{} margin floor".format(label), margin["margin_used_m"])
    check(len(margin["vehicles"]) == 3, "{} envelope vehicles".format(label))
    maximum = max(record["envelope_m"] for record in margin["vehicles"])
    check(maximum == margin["maximum_envelope_m"],
          "{} maximum envelope".format(label), (maximum,
                                               margin["maximum_envelope_m"]))
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
        radius = math.hypot(
            (VEHICLE_LENGTH_M + margin["margin_used_m"]) / 2.0,
            (VEHICLE_WIDTH_M + margin["margin_used_m"]) / 2.0)
        check(abs(record["inflated_body_radius_upper_m"] - radius) <=
              1e-15, "{} vehicle {} inflated radius".format(
                  label, record["vehicle_id"]),
              (record["inflated_body_radius_upper_m"], radius))
    check(margin["margin_used_m"] >=
          TASK_BASE_MARGIN_M + margin["maximum_envelope_m"],
          "{} margin dominates envelope".format(label),
          (margin["margin_used_m"],
           TASK_BASE_MARGIN_M + margin["maximum_envelope_m"]))
    return {
        "baseline_margin_m": margin["baseline_margin_m"],
        "maximum_envelope_m": margin["maximum_envelope_m"],
        "margin_used_m": margin["margin_used_m"],
        "margin_minus_floor_m": margin["margin_used_m"] -
        TASK_BASE_MARGIN_M,
        "vehicles": len(margin["vehicles"]),
        "radius_upper_m": margin["maximum_record"][
            "inflated_body_radius_upper_m"],
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
    check(len(summary["checks"]) == 72, "benchmark check count",
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
            "collision error envelope recomputed with the same binary64 "
            "outward-rounding operation order as the audit record",
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
            "max_samples_per_segment": TASK_SAMPLE_BUDGET,
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
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()