#!/usr/bin/env python3
"""S10 fixed-case regression for multi-vehicle 4WIS rigid formations.

The runner is deliberately ROS-free.  It drives the production Python layers
directly (request validation, S6 continuous geometry, S8 common-time retiming,
S9 independent evaluator and the transactional node core) so that a failure
can be attributed to an exact contract check without a ROS launch in between.
``tests/ros_formation_smoke.py`` exercises the ROS boundary separately.

Every case in ``tools/formation_eval_cases.yaml`` is repeated at least three
times by the command line interface.  Failed scenarios are retained verbatim;
nothing is silently dropped.  The Python backend has no graph search, so
``expanded_nodes`` is recorded as null rather than invented.
"""
import argparse
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import resource
import subprocess
import sys
import time
import traceback
import tracemalloc

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
OPTIMIZER_SRC = ROOT / "catkin_ws/src/fwis_v3_optimizer/src"
OPTIMIZER_SCRIPTS = ROOT / "catkin_ws/src/fwis_v3_optimizer/scripts"
for _path in (str(OPTIMIZER_SRC), str(OPTIMIZER_SCRIPTS)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_geometry import build_continuous_plans
from fwis_v3_optimizer.formation_kinematics import KinematicLimits
from fwis_v3_optimizer.formation_quality import (
    FEASIBLE,
    GEOMETRY_ONLY_NOT_EXECUTABLE,
    INVALID,
    INVALID_REQUEST,
    INVALID_TIMED_TRAJECTORY,
    KINEMATIC_INFEASIBLE,
    MODE_SCHEDULE_INFEASIBLE,
    REJECTED,
    START_FORMATION_COLLISION,
    GOAL_FORMATION_COLLISION,
    FORMATION_NO_GEOMETRIC_PATH,
    FORMATION_SWEEP_COLLISION,
    FORMATION_NLP_INFEASIBLE,
    FORMATION_RETIMING_INFEASIBLE,
    FORMATION_RIGIDITY_VIOLATION,
    PUBLICATION_SWEEP_INVALID,
    FormationQualityConfig,
    evaluate_formation_trajectory,
)
from fwis_v3_optimizer.formation_request import validate_formation_request
from fwis_v3_optimizer.formation_retiming import (
    FormationTimedTrajectory,
    FormationTimingLimits,
    retime_formation_plan,
)
from formation_optimizer_node import (
    FormationNodeCore,
    FormationPipelineConfig,
    FormationPlanningPipeline,
)

SCHEMA = "formation_regression/v1"
EXPECTED_IDS = tuple("E{:02d}".format(index) for index in range(1, 21))
ALLOWED_KINDS = ("pipeline", "evaluator", "request_invalid", "node", "group")
ALLOWED_EXPECTATIONS = (
    "success", "must_fail", "scenario_pass", "statistical")
KINEMATIC_CODES = frozenset((
    KINEMATIC_INFEASIBLE,
    MODE_SCHEDULE_INFEASIBLE,
    FORMATION_RETIMING_INFEASIBLE,
    FORMATION_RIGIDITY_VIOLATION,
))
PUBLICATION_CODES = frozenset((PUBLICATION_SWEEP_INVALID,))
COLLISION_CODES = frozenset((
    START_FORMATION_COLLISION,
    GOAL_FORMATION_COLLISION,
    FORMATION_NO_GEOMETRIC_PATH,
    FORMATION_SWEEP_COLLISION,
    "COLLISION_PRECHECK_FAILED",
))

SCALAR_QUALITY_FIELDS = (
    "collision_margin_m",
    "sweep_spacing_m",
    "publication_dt_s",
    "max_samples_per_segment",
    "position_tolerance_m",
    "heading_tolerance_rad",
    "dynamics_tolerance_ratio",
    "wheel_angle_tolerance_rad",
    "expected_map_signature",
)

ABLATIONS = {
    "none": None,
    "no_spin": (0, 1),
    "no_oblique": (0, 2),
    "no_spin_no_oblique": (0,),
}

METHOD = {
    "backend": "direct_python_s6_s7_s8_s9",
    "enable_nlp": "from case manifest unless --enable-nlp is set",
    "expanded_nodes": "null: the Python S6/S7 backend consumes a direct seed "
                      "and performs no graph search",
    "n_kinematic_reject": "pipeline/retiming rejections and S9 evaluations "
                          "with KINEMATIC/MODE/RIGIDITY/RETIMING codes",
    "n_audit_reject": "S9 evaluator instances that did not return FEASIBLE",
    "success_rate": "FEASIBLE runs divided by attempted runs; group cases "
                    "count every sub-run",
    "expected_failure_rate": "runs meeting the declared must-fail "
                             "expectation divided by attempted runs",
    "time_percentile": "linear interpolation on sorted per-repeat values",
    "memory": "tracemalloc peak for the isolated run; process ru_maxrss also "
              "reported once",
}


# ---------------------------------------------------------------------------
# generic helpers


def _deep_merge(base, override):
    """Recursively merge mappings; lists and scalars replace."""
    if not isinstance(base, dict) or not isinstance(override, dict):
        return override
    merged = {key: value for key, value in base.items()}
    for key, value in override.items():
        if key in merged:
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _finite_float(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a number".format(name))
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("{} must be finite".format(name))
    return result


def _sequence(value, name):
    if not isinstance(value, list) or not value:
        raise ValueError("{} must be a non-empty list".format(name))
    return value


def json_safe(value):
    """Recursively convert to JSON-safe values; non-finite floats become null."""
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, (int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    return number if math.isfinite(number) else None


def percentile(values, quantile):
    ordered = sorted(float(value) for value in values
                     if value is not None)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * quantile / 100.0
    lower = int(math.floor(position))
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def stats(values):
    present = [float(value) for value in values if value is not None]
    if not present:
        return None
    return {
        "count": len(present),
        "min": min(present),
        "median": percentile(present, 50),
        "p95": percentile(present, 95),
        "max": max(present),
    }


def flatten_numeric(value, prefix="", output=None):
    output = {} if output is None else output
    if isinstance(value, dict):
        for key, item in value.items():
            name = "{}.{}".format(prefix, key) if prefix else str(key)
            flatten_numeric(item, name, output)
    elif isinstance(value, (list, tuple)):
        return output
    elif isinstance(value, bool) or value is None:
        return output
    elif isinstance(value, (int, float)):
        number = float(value)
        if math.isfinite(number):
            output[prefix] = number
    return output


def _percentile_stats(mapping_of_values):
    return {name: stats(values) for name, values in sorted(
        mapping_of_values.items()) if values}


def _git_value(*args):
    try:
        process = subprocess.run(
            ["git"] + list(args), cwd=str(ROOT), text=True,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False)
    except OSError:
        return ""
    return process.stdout.strip() if process.returncode == 0 else ""


def environment_report():
    versions = {
        "python": platform.python_version(),
        "numpy": np.__version__,
    }
    try:
        import scipy
        versions["scipy"] = scipy.__version__
    except ImportError:
        versions["scipy"] = None
    try:
        import casadi
        versions["casadi"] = casadi.__version__
    except ImportError:
        versions["casadi"] = None
    cpu_model = ""
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.lower().startswith("model name"):
                cpu_model = line.split(":", 1)[1].strip()
                break
    except OSError:
        pass
    return {
        "platform": platform.platform(),
        "system": platform.system(),
        "release": platform.release(),
        "machine": platform.machine(),
        "processor": platform.processor(),
        "cpu_model": cpu_model,
        "cpu_count": os.cpu_count(),
        "versions": versions,
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "openblas_num_threads": os.environ.get("OPENBLAS_NUM_THREADS"),
        "process_peak_rss_mb": resource.getrusage(
            resource.RUSAGE_SELF).ru_maxrss / 1024.0,
        "git_commit": _git_value("rev-parse", "HEAD"),
        "git_dirty": bool(_git_value("status", "--porcelain")),
    }


# ---------------------------------------------------------------------------
# manifest


def load_cases(path):
    path = Path(path).resolve()
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema") != (
            "formation_eval_cases/v1"):
        raise ValueError("expected schema formation_eval_cases/v1")
    defaults = raw.get("defaults")
    if not isinstance(defaults, dict):
        raise ValueError("defaults must be a mapping")
    cases = _sequence(raw.get("cases"), "cases")
    if len(cases) != len(EXPECTED_IDS):
        raise ValueError(
            "manifest must contain exactly {} cases, got {}".format(
                len(EXPECTED_IDS), len(cases)))
    seen = set()
    resolved = []
    for index, item in enumerate(cases):
        if not isinstance(item, dict):
            raise ValueError("cases[{}] must be a mapping".format(index))
        case_id = item.get("id")
        if case_id not in EXPECTED_IDS:
            raise ValueError("unexpected case id {!r}".format(case_id))
        if case_id in seen:
            raise ValueError("duplicate case id {!r}".format(case_id))
        seen.add(case_id)
        case = _deep_merge(defaults, item)
        _validate_case(case)
        resolved.append(case)
    missing = set(EXPECTED_IDS) - seen
    if missing:
        raise ValueError("missing case ids: {}".format(sorted(missing)))
    resolved.sort(key=lambda item: item["id"])
    return {
        "path": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "schema": raw["schema"],
        "defaults": defaults,
        "cases": resolved,
    }


def _validate_case(case):
    case_id = case["id"]
    kind = case.get("kind")
    if kind not in ALLOWED_KINDS:
        raise ValueError("{} kind must be one of {}".format(case_id, ALLOWED_KINDS))
    expectation = case.get("expectation")
    if expectation not in ALLOWED_EXPECTATIONS:
        raise ValueError("{} expectation must be one of {}".format(
            case_id, ALLOWED_EXPECTATIONS))
    _sequence(case.get("tags", []), case_id + ".tags")
    map_spec = case.get("map")
    if not isinstance(map_spec, dict):
        raise ValueError("{} map must be a mapping".format(case_id))
    for name in ("width", "height", "resolution"):
        if name not in map_spec:
            raise ValueError("{} map.{} is required".format(case_id, name))
    _finite_float(map_spec["resolution"], case_id + ".map.resolution")
    if not isinstance(map_spec.get("origin"), list) or len(
            map_spec.get("origin", [])) != 2:
        raise ValueError("{} map.origin must be [x, y]".format(case_id))
    for rect in map_spec.get("obstacles", []):
        if not isinstance(rect, list) or len(rect) != 4:
            raise ValueError(
                "{} obstacle must be [x0, y0, x1, y1]".format(case_id))
        x0, y0, x1, y1 = (_finite_float(value, "obstacle")
                          for value in rect)
        if x1 <= x0 or y1 <= y0:
            raise ValueError("{} obstacle is empty or inverted".format(case_id))
    if kind != "group":
        formation = case.get("formation")
        if not isinstance(formation, dict):
            raise ValueError("{} formation must be a mapping".format(case_id))
        offsets = _sequence(formation.get("offsets"), case_id + ".offsets")
        headings = _sequence(formation.get("start_headings"),
                             case_id + ".start_headings")
        goals = _sequence(formation.get("goal_headings"),
                          case_id + ".goal_headings")
        if not (len(offsets) == len(headings) == len(goals)):
            raise ValueError(
                "{} offsets/headings lengths differ".format(case_id))
        for offset in offsets:
            if not isinstance(offset, list) or len(offset) != 2:
                raise ValueError("{} offset must be [x, y]".format(case_id))
            _finite_float(offset[0], case_id + ".offset.x")
            _finite_float(offset[1], case_id + ".offset.y")
        for name in ("start", "goal"):
            pose = formation.get(name)
            if not isinstance(pose, list) or len(pose) != 3:
                raise ValueError("{} formation.{} must be [x, y, psi]".format(
                    case_id, name))
            for value in pose:
                _finite_float(value, case_id + "." + name)
        for value in headings + goals:
            _finite_float(value, case_id + ".heading")
        primitives = _sequence(case.get("primitives"), case_id + ".primitives")
        for primitive in primitives:
            if not isinstance(primitive, dict) or primitive.get(
                    "type") not in ("translation", "rotation"):
                raise ValueError(
                    "{} primitive type must be translation or rotation".format(
                        case_id))
            for name in ("dx", "dy", "dpsi", "pivot_x", "pivot_y"):
                if name in primitive:
                    _finite_float(primitive[name],
                                  "{}.primitive.{}".format(case_id, name))
    if kind == "group":
        runs = _sequence(case.get("group_runs"), case_id + ".group_runs")
        labels = set()
        for run in runs:
            if not isinstance(run, dict) or not run.get("label"):
                raise ValueError("{} group run needs a label".format(case_id))
            if run["label"] in labels:
                raise ValueError("{} duplicate group label {}".format(
                    case_id, run["label"]))
            labels.add(run["label"])
            merged = _deep_merge(
                {key: value for key, value in case.items()
                 if key != "group_runs"}, run)
            merged["kind"] = "pipeline"
            merged["expectation"] = "statistical"
            _validate_case(merged)
    tamper = case.get("tamper")
    if tamper is not None:
        if not isinstance(tamper, dict) or tamper.get("kind") not in (
                "mode_control_offset", "theta_offset", "center_speed",
                "point_time_offset", "time_grid_shift"):
            raise ValueError("{} invalid tamper kind".format(case_id))
    mutation = case.get("request_mutation")
    if mutation is not None:
        if not isinstance(mutation, dict) or mutation.get("kind") not in (
                "start_center_offset", "goal_center_offset",
                "goal_mirror"):
            raise ValueError("{} invalid request_mutation kind".format(case_id))


# ---------------------------------------------------------------------------
# geometry / message construction


def build_grid(map_spec):
    width = int(map_spec["width"])
    height = int(map_spec["height"])
    resolution = float(map_spec["resolution"])
    origin_x, origin_y = (float(value) for value in map_spec["origin"])
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValueError("invalid map dimensions")
    occupied = np.zeros((height, width), dtype=bool)
    for x0, y0, x1, y1 in map_spec.get("obstacles", []):
        col0 = max(0, int(math.ceil((x0 - origin_x) / resolution - 0.5)))
        col1 = min(width, int(math.floor(
            (x1 - origin_x) / resolution - 0.5)) + 1)
        row0 = max(0, int(math.ceil((y0 - origin_y) / resolution - 0.5)))
        row1 = min(height, int(math.floor(
            (y1 - origin_y) / resolution - 0.5)) + 1)
        if col1 > col0 and row1 > row0:
            occupied[row0:row1, col0:col1] = True
    grid = Grid(occupied, resolution, origin_x, origin_y)
    return grid, {
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": [origin_x, origin_y],
        "obstacles": [list(rect) for rect in map_spec.get("obstacles", [])],
        "occupied_cells": int(occupied.sum()),
        "free_fraction": float(1.0 - occupied.mean()),
    }


def plan_id_for(name, offset=0):
    digest = hashlib.sha1(name.encode("utf-8")).hexdigest()
    base = int(digest[:8], 16) % 2000000000 + 1
    return (base + int(offset)) % 2000000000 or 1


def stamp_from_case(case):
    secs, nsecs = case.get("stamp", [123, 456])
    return int(secs), int(nsecs)


def _header(case):
    secs, nsecs = stamp_from_case(case)
    return _simple_header(secs, nsecs)


def _simple_header(secs, nsecs):
    from types import SimpleNamespace
    return SimpleNamespace(
        frame_id="map",
        stamp=SimpleNamespace(secs=int(secs), nsecs=int(nsecs)))


def _pose_message(pose):
    from types import SimpleNamespace
    return SimpleNamespace(
        x=float(pose[0]), y=float(pose[1]), theta=float(pose[2]))


def request_message(case, plan_id):
    from types import SimpleNamespace
    formation = case["formation"]
    offsets = tuple(
        VehicleOffset(index + 1, float(offset[0]), float(offset[1]))
        for index, offset in enumerate(formation["offsets"]))
    model = FormationModel(offsets)
    start = FormationPose(*[float(value) for value in formation["start"]])
    goal = FormationPose(*[float(value) for value in formation["goal"]])
    start_centers = model.centers(start)
    goal_centers = model.centers(goal)
    body = case["body"]
    vehicles = []
    for index, offset in enumerate(offsets):
        vehicles.append(SimpleNamespace(
            vehicle_id=offset.vehicle_id,
            offset_in_formation=SimpleNamespace(
                x=offset.x, y=offset.y, z=0.0),
            start_center=SimpleNamespace(
                x=start_centers[offset.vehicle_id][0],
                y=start_centers[offset.vehicle_id][1],
                theta=float(formation["start_headings"][index])),
            goal_center=SimpleNamespace(
                x=goal_centers[offset.vehicle_id][0],
                y=goal_centers[offset.vehicle_id][1],
                theta=float(formation["goal_headings"][index])),
            goal_heading_free=bool(formation.get("goal_heading_free", False)),
            body_length_m=float(body["length_m"]),
            body_width_m=float(body["width_m"])))
    return SimpleNamespace(
        header=_header(case),
        plan_id=int(plan_id),
        formation_id="formation_{}".format(
            case["id"].lower().replace("/", "_")),
        formation_start=_pose_message(formation["start"]),
        formation_goal=_pose_message(formation["goal"]),
        vehicles=vehicles,
        allow_reverse=bool(formation.get("allow_reverse", False)))


def primitive_messages(case):
    from types import SimpleNamespace
    messages = []
    for primitive in case["primitives"]:
        messages.append(SimpleNamespace(
            geometry_type=0 if primitive["type"] == "translation" else 1,
            dx=float(primitive.get("dx", 0.0)),
            dy=float(primitive.get("dy", 0.0)),
            dpsi=float(primitive.get("dpsi", 0.0)),
            pivot_x=float(primitive.get("pivot_x", 0.0)),
            pivot_y=float(primitive.get("pivot_y", 0.0))))
    return tuple(messages)


def seed_message(case, plan_id):
    from types import SimpleNamespace
    request = request_message(case, plan_id)
    secs, nsecs = stamp_from_case(case)
    return SimpleNamespace(
        header=_simple_header(secs, nsecs),
        plan_id=int(plan_id),
        formation_id=request.formation_id,
        formation_start=request.formation_start,
        formation_goal=request.formation_goal,
        primitives=primitive_messages(case))


def mutate_request(request, mutation):
    if mutation is None:
        return request
    if mutation["kind"] == "goal_mirror":
        # Keep the declared offsets unchanged but place every goal center at
        # the mirrored offset.  A valid request must derive its goal centers
        # from q_goal and those declared offsets, so the mirror must fail the
        # request contract for the vehicle whose y offset is non-zero.
        goal = request.formation_goal
        cosine, sine = math.cos(float(goal.theta)), math.sin(float(goal.theta))
        for vehicle in request.vehicles:
            offset_x = float(vehicle.offset_in_formation.x)
            offset_y = -float(vehicle.offset_in_formation.y)
            vehicle.goal_center.x = (
                float(goal.x) + cosine * offset_x - sine * offset_y)
            vehicle.goal_center.y = (
                float(goal.y) + sine * offset_x + cosine * offset_y)
        return request
    index = int(mutation.get("vehicle_index", 0))
    if index < 0 or index >= len(request.vehicles):
        raise ValueError("request mutation vehicle_index out of range")
    target = (request.vehicles[index].start_center
              if mutation["kind"] == "start_center_offset"
              else request.vehicles[index].goal_center)
    target.x = float(target.x) + float(mutation["delta"])
    return request


def make_quality_config(case, evaluation=False):
    timing_values = asdict(FormationTimingLimits())
    timing_values.update(case.get("timing", {}))
    if evaluation:
        timing_values.update(case.get("evaluation_timing_overrides", {}))
    kinematic_values = asdict(KinematicLimits())
    kinematic_values.update(case.get("kinematic", {}))
    scalar = {
        name: case["quality"][name] for name in SCALAR_QUALITY_FIELDS
        if name in case.get("quality", {})
    }
    if evaluation:
        scalar.update(case.get("evaluation_quality_overrides", {}))
    return FormationQualityConfig(
        timing_limits=FormationTimingLimits(**timing_values),
        kinematic_limits=KinematicLimits(**kinematic_values),
        **scalar)


def make_pipeline_config(case, options):
    pipeline_spec = case["pipeline"]
    return FormationPipelineConfig(
        quality_config=make_quality_config(case),
        max_candidates=int(pipeline_spec["max_candidates"]),
        max_schedule_combinations=int(
            pipeline_spec["max_schedule_combinations"]),
        max_schedules_per_block=int(
            pipeline_spec["max_schedules_per_block"]),
        planning_dt_s=float(pipeline_spec["planning_dt_s"]),
        enable_nlp=bool(pipeline_spec.get("enable_nlp", False) or
                        options.get("enable_nlp", False)),
        require_collision_precheck=bool(
            pipeline_spec.get("require_collision_precheck", True)),
        allowed_modes=options.get("allowed_modes"))


# ---------------------------------------------------------------------------
# run rows


def _timing_start():
    return time.perf_counter(), time.process_time()


def _timing_finish(started, tracemalloc_peak):
    wall = time.perf_counter() - started[0]
    cpu = time.process_time() - started[1]
    return wall, cpu, tracemalloc_peak


def _run_row(kind, case):
    return {
        "case_id": case["id"],
        "kind": kind,
        "harness_ok": True,
        "harness_error": "",
        "status": None,
        "reason": "",
        "detail": "",
        "feasible": None,
        "published": None,
        "failure_codes": [],
        "failure_field": "",
        "T_end_s": None,
        "metrics": {},
        "metrics_example": {},
        "stages": {},
        "wall_s": None,
        "cpu_s": None,
        "python_peak_mb": None,
        "n_kinematic_reject": 0,
        "n_audit_reject": 0,
        "n_candidates_evaluated": 0,
        "candidate_count": 0,
        "scenario_ok": None,
        "scenario_steps": [],
        "expectation_met": False,
        "expectation_mode": "baseline",
    }


def _finalise(row, case, options, started, tracemalloc_peak):
    wall, cpu, peak = _timing_finish(started, tracemalloc_peak)
    row["wall_s"] = wall
    row["cpu_s"] = cpu
    row["python_peak_mb"] = peak / (1024.0 * 1024.0)
    row["expectation_mode"] = ("record_only"
                               if options.get("record_only") else "baseline")
    row["expectation_met"] = expectation_met(case, row, options)
    return row


def expectation_met(case, row, options):
    if options.get("record_only"):
        return bool(row.get("harness_ok"))
    expectation = case["expectation"]
    if not row.get("harness_ok"):
        return False
    if expectation == "success":
        return row.get("feasible") is True
    if expectation == "must_fail":
        if row.get("feasible") is True:
            return False
        expected = set(case.get("expected_failure_codes", []))
        codes = set(row.get("failure_codes", []))
        if expected and not (expected & codes):
            return False
        expected_field = case.get("expected_field")
        if expected_field and expected_field not in row.get(
                "failure_field", ""):
            return False
        return True
    if expectation == "scenario_pass":
        return row.get("scenario_ok") is True
    if expectation == "statistical":
        return True
    return False


def _record_quality_metrics(row, metrics):
    metrics = dict(metrics or {})
    row["metrics"] = flatten_numeric(metrics)
    row["metrics_example"] = json_safe(metrics)


def _record_pipeline_result(row, result):
    row["status"] = result.status
    row["reason"] = result.reason
    row["detail"] = result.detail
    row["feasible"] = result.status == FEASIBLE
    codes = []
    field = ""
    for rejection in result.rejections:
        if rejection.failure_code:
            codes.append(rejection.failure_code)
        if not field and rejection.constraint:
            field = rejection.constraint
    if result.selection is not None:
        row["n_audit_reject"] = sum(
            1 for evaluation in result.selection.evaluations
            if not evaluation.feasible)
        row["n_candidates_evaluated"] = len(result.selection.evaluations)
        for evaluation in result.selection.evaluations:
            if not evaluation.feasible and evaluation.reason:
                codes.append(evaluation.reason)
            if (not field and evaluation.failure is not None and
                    evaluation.failure.field):
                field = evaluation.failure.field
    else:
        row["n_audit_reject"] = 0
        row["n_candidates_evaluated"] = 0
    row["n_kinematic_reject"] = sum(
        1 for code in codes if code in KINEMATIC_CODES)
    row["failure_codes"] = sorted(set(codes))
    row["failure_field"] = field
    row["candidate_count"] = len(result.candidates)
    row["stages"] = {name: float(value)
                     for name, value in result.timings_s}
    selected = result.selected_evaluation
    if selected is None and result.selection is not None:
        selected = next(
            (item for item in result.selection.evaluations
             if item.status == FEASIBLE), None)
    if selected is not None:
        _record_quality_metrics(row, selected.metrics)
        row["T_end_s"] = selected.metrics.get("duration_s")
    if row["T_end_s"] is None and result.selected_timed is not None:
        row["T_end_s"] = float(result.selected_timed.duration)


def _run_pipeline(case, options):
    row = _run_row("pipeline", case)
    started = _timing_start()
    try:
        request = request_message(case, plan_id_for(case["id"]))
        validation = validate_formation_request(request)
        if not validation.valid:
            row["status"] = INVALID
            row["reason"] = validation.reason
            row["detail"] = validation.detail
            row["feasible"] = False
            row["failure_codes"] = [validation.reason]
            row["failure_field"] = validation.field
            return row
        grid, grid_info = build_grid(case["map"])
        row["map"] = grid_info
        pipeline = FormationPlanningPipeline(make_pipeline_config(case, options))
        result = pipeline.run(
            validation.snapshot, seed_message(case, plan_id_for(case["id"])),
            grid)
        _record_pipeline_result(row, result)
        row["master_status"] = (
            None if result.master is None else result.master.status)
        row["audit_status"] = (
            None if result.selection is None else result.selection.status)
    except Exception as exc:
        row["harness_ok"] = False
        row["harness_error"] = repr(exc)
        row["traceback"] = traceback.format_exc()
    return row


def _select_point(points, vehicle_id, mode, min_time, point_index=None):
    if point_index is not None:
        index = int(point_index)
        if index < 0 or index >= len(points):
            raise ValueError("point_index out of range")
        return index
    for index, point in enumerate(points):
        if int(point.mode) == int(mode) and float(
                point.time_from_start) >= float(min_time):
            return index
    raise ValueError(
        "no point with mode {} at or after {} s".format(mode, min_time))


def apply_tamper(case, timed):
    tamper = case.get("tamper")
    if tamper is None:
        return timed, None
    kind = tamper["kind"]
    vehicle_id = int(tamper.get("vehicle_id", 1))
    if vehicle_id not in timed.trajectories:
        raise ValueError("tamper vehicle_id {} absent".format(vehicle_id))
    points = list(timed.trajectories[vehicle_id])
    if kind in ("mode_control_offset", "theta_offset", "center_speed"):
        index = _select_point(
            points, vehicle_id, tamper.get("mode"),
            tamper.get("min_time_s", 0.0), tamper.get("point_index"))
        change = {"mode_control_offset": "mode_control",
                  "theta_offset": "theta",
                  "center_speed": "center_speed_mps"}[kind]
        points[index] = replace(
            points[index],
            **{change: float(getattr(points[index], change)) + float(
                tamper["delta"])})
        trajectories = dict(timed.trajectories)
        trajectories[vehicle_id] = tuple(points)
        return replace(timed, trajectories=trajectories), {
            "kind": kind, "vehicle_id": vehicle_id, "point_index": index,
            "field": change, "delta": float(tamper["delta"])}
    if kind == "point_time_offset":
        index = _select_point(points, vehicle_id, None, 0.0,
                              tamper.get("point_index"))
        points[index] = replace(
            points[index],
            time_from_start=float(points[index].time_from_start) + float(
                tamper["delta"]))
        trajectories = dict(timed.trajectories)
        trajectories[vehicle_id] = tuple(points)
        # Bypass the frozen constructor exactly like an externally corrupted
        # ROS message that would reach the evaluator.
        raw = object.__new__(FormationTimedTrajectory)
        for name in ("plan_id", "formation_id", "offsets", "formation_poses",
                     "times_from_start", "trajectories", "phases",
                     "timing_report", "certification", "dynamic_profile"):
            object.__setattr__(raw, name, getattr(timed, name))
        object.__setattr__(raw, "trajectories", trajectories)
        return raw, {
            "kind": kind, "vehicle_id": vehicle_id, "point_index": index,
            "field": "time_from_start", "delta": float(tamper["delta"])}
    if kind == "time_grid_shift":
        raw = object.__new__(FormationTimedTrajectory)
        for name in ("plan_id", "formation_id", "offsets", "formation_poses",
                     "times_from_start", "trajectories", "phases",
                     "timing_report", "certification", "dynamic_profile"):
            object.__setattr__(raw, name, getattr(timed, name))
        object.__setattr__(
            raw, "times_from_start",
            tuple(float(value) + float(tamper["delta"])
                  for value in timed.times_from_start))
        return raw, {"kind": kind, "field": "times_from_start",
                     "delta": float(tamper["delta"])}
    raise ValueError("unsupported tamper kind {}".format(kind))


def _run_evaluator(case, options):
    row = _run_row("evaluator", case)
    started = _timing_start()
    try:
        request = request_message(case, plan_id_for(case["id"]))
        validation = validate_formation_request(request)
        if not validation.valid:
            raise ValueError("evaluator case request invalid: {} {}".format(
                validation.reason, validation.field))
        grid, grid_info = build_grid(case["map"])
        row["map"] = grid_info
        build_quality = make_quality_config(case)
        pipeline_spec = case["pipeline"]
        build_started = time.perf_counter()
        build = build_continuous_plans(
            validation.snapshot,
            seed_message(case, plan_id_for(case["id"])),
            limits=build_quality.kinematic_limits,
            max_candidates=int(pipeline_spec["max_candidates"]),
            max_schedule_combinations=int(
                pipeline_spec["max_schedule_combinations"]),
            max_schedules_per_block=int(
                pipeline_spec["max_schedules_per_block"]),
            allowed_modes=options.get("allowed_modes"))
        row["stages"]["build_s"] = time.perf_counter() - build_started
        if not build.success or not build.plans:
            row["status"] = REJECTED
            row["reason"] = build.reason
            row["detail"] = build.detail
            row["feasible"] = False
            row["failure_codes"] = [build.reason] if build.reason else []
            return row
        plan = build.plans[0]
        retime_started = time.perf_counter()
        timing = retime_formation_plan(
            plan, limits=build_quality.timing_limits,
            dt=float(pipeline_spec["planning_dt_s"]))
        row["stages"]["retiming_s"] = time.perf_counter() - retime_started
        if timing.timed is None:
            row["status"] = GEOMETRY_ONLY_NOT_EXECUTABLE
            row["reason"] = timing.reason
            row["detail"] = timing.detail
            row["feasible"] = False
            row["failure_codes"] = [timing.reason] if timing.reason else []
            return row
        tampered, tamper_record = apply_tamper(case, timing.timed)
        row["tamper"] = tamper_record
        evaluation_quality = make_quality_config(case, evaluation=True)
        evaluation_started = time.perf_counter()
        evaluation = evaluate_formation_trajectory(
            validation.snapshot, plan, tampered, grid,
            config=evaluation_quality)
        row["stages"]["evaluation_s"] = (
            time.perf_counter() - evaluation_started)
        row["status"] = evaluation.status
        row["reason"] = evaluation.reason
        row["detail"] = evaluation.detail
        row["feasible"] = bool(evaluation.feasible)
        row["checks"] = int(evaluation.checks)
        row["failure_codes"] = (
            [] if evaluation.feasible else [evaluation.reason])
        row["failure_field"] = (
            "" if evaluation.failure is None else evaluation.failure.field)
        row["failure"] = (None if evaluation.failure is None
                          else json_safe(evaluation.failure.as_dict()))
        row["n_audit_reject"] = 0 if evaluation.feasible else 1
        row["n_kinematic_reject"] = (
            1 if (not evaluation.feasible and
                  evaluation.reason in KINEMATIC_CODES) else 0)
        row["n_candidates_evaluated"] = 1
        row["candidate_count"] = 1
        _record_quality_metrics(row, evaluation.metrics)
        row["T_end_s"] = float(tampered.duration)
        row["stages"]["total_s"] = sum(
            row["stages"].get(name, 0.0)
            for name in ("build_s", "retiming_s", "evaluation_s"))
    except Exception as exc:
        row["harness_ok"] = False
        row["harness_error"] = repr(exc)
        row["traceback"] = traceback.format_exc()
    return row


def _run_request_invalid(case, options):
    row = _run_row("request_invalid", case)
    started = _timing_start()
    try:
        request = mutate_request(
            request_message(case, plan_id_for(case["id"])),
            case.get("request_mutation"))
        validation_started = time.perf_counter()
        validation = validate_formation_request(request)
        row["stages"]["request_validation_s"] = (
            time.perf_counter() - validation_started)
        row["status"] = INVALID if not validation.valid else "VALID"
        row["reason"] = validation.reason
        row["detail"] = validation.detail
        row["feasible"] = False
        row["failure_codes"] = [] if validation.valid else [validation.reason]
        row["failure_field"] = validation.field
        row["n_audit_reject"] = 0 if validation.valid else 1
        row["stages"]["total_s"] = row["stages"]["request_validation_s"]
    except Exception as exc:
        row["harness_ok"] = False
        row["harness_error"] = repr(exc)
        row["traceback"] = traceback.format_exc()
    return row


def _request_with_plan(case, plan_id):
    return request_message(case, plan_id)


def _run_node(case, options):
    row = _run_row("node", case)
    started = _timing_start()
    try:
        grid, grid_info = build_grid(case["map"])
        row["map"] = grid_info
        core = FormationNodeCore(
            pipeline_config=make_pipeline_config(case, options))
        core.on_map(grid, None, "map", stamp=(1000, 0))
        base_plan = plan_id_for(case["id"])
        request = _request_with_plan(case, base_plan)
        request_event = core.on_request(
            request, publication_stamp=(1001, 0))
        seed = seed_message(case, base_plan)
        seed_event = core.on_seed(seed, publication_stamp=(1002, 0))
        published = bool(
            seed_event.status.get("trajectory_published") and
            seed_event.status.get("executable") and
            seed_event.trajectory_fields is not None)
        quality = seed_event.quality
        row["published"] = published
        row["publish_status"] = seed_event.status.get("status")
        row["publish_reason"] = seed_event.status.get("reason", "")
        row["status"] = None
        row["reason"] = ""
        row["detail"] = "transactional publication then invalidation"
        row["feasible"] = None
        row["stages"] = {
            name: float(value) for name, value in (
                quality.get("pipeline", {}).get("timings_s", {}) or {}).items()}
        row["steps"] = [{
            "step": "publish",
            "status": seed_event.status.get("status"),
            "reason": seed_event.status.get("reason", ""),
            "trajectory_published": seed_event.status.get(
                "trajectory_published"),
            "executable": seed_event.status.get("executable"),
            "clear": bool(seed_event.clear),
            "plan_id": seed_event.status.get("plan_id"),
        }]
        row["publish_certificate"] = {
            "independent_audit": bool(quality.get("independent_audit")),
            "collision_certificate": bool(
                quality.get("collision_certificate")),
            "request_valid": bool(
                (quality.get("request") or {}).get("valid")),
            "seed_status": (quality.get("seed") or {}).get("status"),
        }
        steps_ok = published
        scenario = case.get("scenario", {})
        invalid_specs = scenario.get("invalid_requests", [])
        for index, invalid_spec in enumerate(invalid_specs):
            invalid_plan = base_plan + 1 + index
            invalid_request = _request_with_plan(case, invalid_plan)
            if invalid_spec == "nan_start":
                invalid_request.vehicles[0].start_center.x = float("nan")
            elif invalid_spec == "empty_vehicles":
                invalid_request.vehicles = []
            else:
                raise ValueError(
                    "unsupported invalid request spec {}".format(invalid_spec))
            event = core.on_request(
                invalid_request,
                publication_stamp=(1010 + index, 0))
            ok = bool(
                event.clear and
                not event.status.get("trajectory_published") and
                event.status.get("status") == INVALID and
                core.gate.published_plan_id is None)
            steps_ok = steps_ok and ok
            row["steps"].append({
                "step": "invalid_request:{}".format(invalid_spec),
                "status": event.status.get("status"),
                "reason": event.status.get("reason", ""),
                "trajectory_published": event.status.get(
                    "trajectory_published"),
                "clear": bool(event.clear),
                "plan_id": event.status.get("plan_id"),
                "published_plan_id_after": core.gate.published_plan_id,
            })
        if scenario.get("stale_seed_after_invalid"):
            # A fresh valid request exists, then a seed carrying the original
            # (now old) plan id must not publish anything.
            fresh_plan = base_plan + 100
            fresh_request = _request_with_plan(case, fresh_plan)
            fresh_event = core.on_request(
                fresh_request, publication_stamp=(1020, 0))
            stale_event = core.on_seed(seed, publication_stamp=(1021, 0))
            stale_ok = bool(
                not stale_event.status.get("trajectory_published") and
                core.gate.published_plan_id is None)
            steps_ok = steps_ok and stale_ok
            row["steps"].append({
                "step": "stale_seed_after_invalid",
                "fresh_request_status": fresh_event.status.get("status"),
                "status": stale_event.status.get("status"),
                "reason": stale_event.status.get("reason", ""),
                "trajectory_published": stale_event.status.get(
                    "trajectory_published"),
                "clear": bool(stale_event.clear),
                "published_plan_id_after": core.gate.published_plan_id,
            })
        row["scenario_ok"] = bool(steps_ok)
        row["status"] = ("SCENARIO_CLEARED" if steps_ok
                         else "SCENARIO_FAILED")
        row["published_before_invalidation"] = published
        row["scenario_steps"] = row["steps"]
        row["failure_codes"] = sorted({
            step.get("reason") for step in row["steps"]
            if step.get("reason") in (
                INVALID_REQUEST, "STALE_SEED", "STALE_PLAN")})
        row["stages"]["total_s"] = row["wall_s"] if row["wall_s"] else (
            time.perf_counter() - started[0])
    except Exception as exc:
        row["harness_ok"] = False
        row["harness_error"] = repr(exc)
        row["traceback"] = traceback.format_exc()
    return row


def _run_group(case, options):
    row = _run_row("group", case)
    started = _timing_start()
    try:
        subrows = []
        success_count = 0
        attempts = 0
        codes = set()
        for index, run_spec in enumerate(case["group_runs"]):
            merged = _deep_merge(
                {key: value for key, value in case.items()
                 if key != "group_runs"}, run_spec)
            merged["id"] = "{}/{}".format(case["id"], run_spec["label"])
            merged["kind"] = "pipeline"
            merged["expectation"] = "statistical"
            _validate_case(merged)
            sub_started = _timing_start()
            own_tracer = not tracemalloc.is_tracing()
            if own_tracer:
                tracemalloc.start()
            peak = 0
            try:
                sub = _run_pipeline(merged, dict(options, record_only=True))
                peak = tracemalloc.get_traced_memory()[1]
            finally:
                if own_tracer:
                    tracemalloc.stop()
            sub = _finalise(sub, merged, dict(options, record_only=True),
                            sub_started, peak)
            sub["label"] = run_spec["label"]
            attempts += 1
            if sub.get("feasible") is True:
                success_count += 1
            codes.update(sub.get("failure_codes", []))
            subrows.append(sub)
        row["subruns"] = subrows
        row["group_success_count"] = success_count
        row["group_attempt_count"] = attempts
        row["group_success_rate"] = (
            float(success_count) / attempts if attempts else None)
        row["status"] = ("GROUP_COMPLETE"
                         if all(sub["harness_ok"] for sub in subrows)
                         else "GROUP_HARNESS_ERROR")
        row["reason"] = ""
        row["feasible"] = success_count == attempts
        row["failure_codes"] = sorted(codes)
        row["n_audit_reject"] = sum(
            int(sub.get("n_audit_reject", 0)) for sub in subrows)
        row["n_kinematic_reject"] = sum(
            int(sub.get("n_kinematic_reject", 0)) for sub in subrows)
        row["n_candidates_evaluated"] = sum(
            int(sub.get("n_candidates_evaluated", 0)) for sub in subrows)
        row["candidate_count"] = sum(
            int(sub.get("candidate_count", 0)) for sub in subrows)
        marcher = [sub for sub in subrows if sub.get("feasible") is True]
        if marcher:
            row["T_end_s"] = marcher[0].get("T_end_s")
            _record_quality_metrics(row, marcher[0].get("metrics_example", {}))
        else:
            row["T_end_s"] = None
            row["metrics"] = {}
        row["stages"]["total_s"] = time.perf_counter() - started[0]
        row["scenario_ok"] = all(sub["harness_ok"] for sub in subrows)
    except Exception as exc:
        row["harness_ok"] = False
        row["harness_error"] = repr(exc)
        row["traceback"] = traceback.format_exc()
    return row


RUNNERS = {
    "pipeline": _run_pipeline,
    "evaluator": _run_evaluator,
    "request_invalid": _run_request_invalid,
    "node": _run_node,
    "group": _run_group,
}


def run_once(case, options):
    kind = case["kind"]
    runner = RUNNERS.get(kind)
    if runner is None:
        raise ValueError("unsupported case kind {}".format(kind))
    started = _timing_start()
    tracemalloc.start()
    try:
        row = runner(case, options)
    except Exception as exc:
        row = _run_row(kind, case)
        row["harness_ok"] = False
        row["harness_error"] = repr(exc)
        row["traceback"] = traceback.format_exc()
    finally:
        peak = tracemalloc.get_traced_memory()[1]
        tracemalloc.stop()
    return _finalise(row, case, options, started, peak)


# ---------------------------------------------------------------------------
# aggregation


def aggregate_case(case, runs, options):
    row = {
        "id": case["id"],
        "title": case.get("title", ""),
        "kind": case["kind"],
        "expectation": case["expectation"],
        "tags": list(case.get("tags", [])),
        "repeat_count": len(runs),
        "runs": runs,
        "harness_ok": all(run.get("harness_ok") for run in runs),
        "passed": bool(runs) and all(
            run.get("expectation_met") is True for run in runs),
        "statuses": sorted({str(run.get("status")) for run in runs}),
        "failure_codes": sorted({
            code for run in runs for code in run.get("failure_codes", [])}),
        "failure_code_counts": {},
        "outcome_rate": None,
        "expected_failure_rate": None,
        "stages": {},
        "wall_s": stats([run.get("wall_s") for run in runs]),
        "cpu_s": stats([run.get("cpu_s") for run in runs]),
        "python_peak_mb": stats(
            [run.get("python_peak_mb") for run in runs]),
        "n_kinematic_reject": sum(
            int(run.get("n_kinematic_reject", 0)) for run in runs),
        "n_audit_reject": sum(
            int(run.get("n_audit_reject", 0)) for run in runs),
        "n_candidates_evaluated": sum(
            int(run.get("n_candidates_evaluated", 0)) for run in runs),
        "expanded_nodes": None,
        "metrics": {},
    }
    counts = {}
    for run in runs:
        for code in run.get("failure_codes", []):
            counts[code] = counts.get(code, 0) + 1
    row["failure_code_counts"] = dict(sorted(counts.items()))
    attempts = len(runs)
    if case["kind"] == "group":
        successes = sum(int(run.get("group_success_count", 0))
                        for run in runs)
        sub_attempts = sum(int(run.get("group_attempt_count", 0))
                           for run in runs)
        row["outcome_rate"] = {
            "numerator": successes,
            "denominator": sub_attempts,
            "value": (float(successes) / sub_attempts
                      if sub_attempts else None),
            "definition": "FEASIBLE sub-runs / attempted sub-runs",
        }
        row["group_success_rate"] = row["outcome_rate"]["value"]
    elif case["kind"] == "node":
        successes = sum(1 for run in runs if run.get("scenario_ok") is True)
        row["outcome_rate"] = {
            "numerator": successes,
            "denominator": attempts,
            "value": float(successes) / attempts if attempts else None,
            "definition": "scenario_ok runs / attempted runs",
        }
    elif case["expectation"] == "must_fail":
        successes = sum(1 for run in runs
                        if run.get("expectation_met") is True)
        row["expected_failure_rate"] = {
            "numerator": successes,
            "denominator": attempts,
            "value": float(successes) / attempts if attempts else None,
            "definition": "declared rejection observed / attempted runs",
        }
        row["outcome_rate"] = row["expected_failure_rate"]
    else:
        successes = sum(1 for run in runs if run.get("feasible") is True)
        row["outcome_rate"] = {
            "numerator": successes,
            "denominator": attempts,
            "value": float(successes) / attempts if attempts else None,
            "definition": "FEASIBLE runs / attempted runs",
        }
    stage_values = {}
    for run in runs:
        for name, value in run.get("stages", {}).items():
            stage_values.setdefault(name, []).append(value)
    row["stages"] = _percentile_stats(stage_values)
    metric_values = {}
    for run in runs:
        for name, value in run.get("metrics", {}).items():
            metric_values.setdefault(name, []).append(value)
    row["metrics"] = _percentile_stats(metric_values)
    metric_example = next(
        (run.get("metrics_example") for run in runs
         if run.get("metrics_example")), {})
    row["metrics_example"] = metric_example
    row["T_end_s"] = stats([run.get("T_end_s") for run in runs])
    row["maps"] = [run.get("map") for run in runs if run.get("map")]
    if case["kind"] == "group":
        row["group_subruns"] = [run.get("subruns") for run in runs]
    if not row["harness_ok"]:
        row["passed"] = False
    return row


def _summarise(case_rows, manifest, options, repeats):
    failures = [row for row in case_rows if not row["passed"]]
    harness_failures = [
        {"id": row["id"], "error": run.get("harness_error"),
         "traceback": run.get("traceback")}
        for row in case_rows for run in row["runs"]
        if not run.get("harness_ok")]
    record_only = bool(options.get("record_only"))
    return {
        "schema": SCHEMA,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "manifest": {
            "path": manifest["path"],
            "schema": manifest["schema"],
            "sha256": manifest["sha256"],
        },
        "environment": environment_report(),
        "config": {
            "repeats": repeats,
            "ablation": options.get("ablation", "none"),
            "allowed_modes": (None if options.get("allowed_modes") is None
                              else list(options["allowed_modes"])),
            "enable_nlp": bool(options.get("enable_nlp")),
            "record_only": record_only,
            "selected_ids": list(options.get("selected_ids", [])),
            "backend": METHOD["backend"],
        },
        "method": dict(METHOD),
        "rows": case_rows,
        "failures": failures,
        "harness_failures": harness_failures,
        "case_count": len(case_rows),
        "passed_case_count": sum(1 for row in case_rows if row["passed"]),
        "all_pass": bool(case_rows) and not failures and not harness_failures,
        "ros_smoke": options.get("ros_smoke"),
    }


# ---------------------------------------------------------------------------
# CLI


def _load_ros_summary(path):
    if path is None:
        return None
    data = json.loads(Path(path).resolve().read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("ROS summary must be a JSON object")
    return {"path": str(Path(path).resolve()), "summary": data}


def run_regression(cases_file, output_dir, repeats=3, ids=None,
                   ablation="none", enable_nlp=False, ros_summary=None):
    manifest = load_cases(cases_file)
    selected = list(ids) if ids else list(EXPECTED_IDS)
    missing = set(selected) - set(EXPECTED_IDS)
    if missing:
        raise ValueError("unknown ids: {}".format(sorted(missing)))
    if ablation not in ABLATIONS:
        raise ValueError("unknown ablation {!r}".format(ablation))
    options = {
        "ablation": ablation,
        "allowed_modes": ABLATIONS[ablation],
        "enable_nlp": bool(enable_nlp),
        "record_only": ablation != "none",
        "selected_ids": selected,
        "ros_smoke": _load_ros_summary(ros_summary),
    }
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    rows = []
    for case in manifest["cases"]:
        if case["id"] not in selected:
            continue
        runs = []
        for index in range(int(repeats)):
            print("[{}] repeat {}/{} kind={} expectation={}".format(
                case["id"], index + 1, repeats, case["kind"],
                case["expectation"]), flush=True)
            run = run_once(case, options)
            runs.append(run)
            print("    status={} reason={} feasible={} passed={}".format(
                run.get("status"), run.get("reason"),
                run.get("feasible"), run.get("expectation_met")),
                flush=True)
            if not run.get("harness_ok"):
                print("    harness_error={}".format(run.get("harness_error")),
                      flush=True)
        rows.append(aggregate_case(case, runs, options))
    summary = _summarise(rows, manifest, options, int(repeats))
    summary_path = destination / "summary.json"
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print("summary: {}".format(summary_path), flush=True)
    print("all_pass: {} ({}/{})".format(
        summary["all_pass"], summary["passed_case_count"],
        summary["case_count"]), flush=True)
    return summary, summary_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cases-file", type=Path,
        default=ROOT / "tools/formation_eval_cases.yaml")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--ids", nargs="+")
    parser.add_argument("--ablation", choices=sorted(ABLATIONS), default="none")
    parser.add_argument("--enable-nlp", action="store_true")
    parser.add_argument("--ros-summary", type=Path)
    args = parser.parse_args(argv)
    if args.repeats < 3:
        parser.error("--repeats must be at least 3 for the S10 contract")
    try:
        summary, _ = run_regression(
            args.cases_file, args.output_dir, repeats=args.repeats,
            ids=args.ids, ablation=args.ablation,
            enable_nlp=args.enable_nlp, ros_summary=args.ros_summary)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 2
    return 0 if summary["all_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())