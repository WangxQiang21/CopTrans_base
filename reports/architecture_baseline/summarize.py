#!/usr/bin/env python3
"""Index raw run artifacts and summarize phase timing and tracking samples."""

import argparse
from collections import Counter, defaultdict
from hashlib import sha256
import json
import math
from pathlib import Path
import re
from statistics import median

import yaml


def json_rows(path):
    if path.is_file():
        with path.open(encoding="utf-8") as stream:
            for line in stream:
                yield json.loads(line)


def statistics(values):
    if not values:
        return {"count": 0}
    values = sorted(values)
    result = {"count": len(values), "p50": median(values), "max": values[-1]}
    if len(values) >= 30:
        result["p95"] = values[math.ceil(len(values) * .95) - 1]
    if len(values) >= 100:
        result["p99"] = values[math.ceil(len(values) * .99) - 1]
    return result


def controllers(run):
    traces = defaultdict(lambda: {"phases": Counter(), "compute_ms": [], "track_stamps": [],
                                  "mode_sequence": [], "requested_mode_sequence": [],
                                  "first_tracking_sim_s": None, "first_nonzero_twist_sim_s": None,
                                  "mode_constraint_violation_count": 0})
    for row in json_rows(run / "controller.log"):
        if row["kind"] != "execution":
            continue
        trace = traces[str(row["vehicle_id"])]
        trace["phases"][row["phase"]] += 1
        requested_mode = row.get("requested_mode")
        if requested_mode is not None and trace["requested_mode_sequence"][-1:] != [requested_mode]:
            trace["requested_mode_sequence"].append(requested_mode)
        command = row.get("command")
        if (trace["first_nonzero_twist_sim_s"] is None and command is not None
                and any(abs(value) > 1e-12 for value in command)):
            trace["first_nonzero_twist_sim_s"] = row["stamp"]
        if row["phase"] != "TRACKING":
            continue
        if trace["first_tracking_sim_s"] is None:
            trace["first_tracking_sim_s"] = row["stamp"]
        trace["compute_ms"].append(row["compute_s"] * 1000)
        trace["track_stamps"].append(row["stamp"])
        mode = row.get("planned_mode")
        if mode is not None and trace["mode_sequence"][-1:] != [mode]:
            trace["mode_sequence"].append(mode)
        if command is not None and mode is not None:
            x, y, yaw = command
            if (not all(math.isfinite(value) for value in command)
                    or (mode == 0 and (abs(y) > 1e-9 or abs(yaw) * .83 > abs(x) + 1e-9))
                    or (mode == 1 and abs(yaw) > 1e-9)
                    or (mode == 2 and max(abs(x), abs(y)) > 1e-9)):
                trace["mode_constraint_violation_count"] += 1
    for trace in traces.values():
        trace["tracking_compute_ms"] = statistics(trace.pop("compute_ms"))
        stamps = trace.pop("track_stamps")
        # Transition gaps between TRACKING phases are deliberate, not missed deadlines.
        trace["consecutive_tracking_status_period_ms"] = statistics(
            [(current - previous) * 1000 for previous, current in zip(stamps, stamps[1:])
             if 0 < current - previous <= .050])
    return dict(traces)


def planner(run):
    result = {}
    for row in json_rows(run / "planner.log"):
        message = row["message"]
        if row["topic"] == "/formation/optimizer_status":
            status = json.loads(message)
            if status.get("trajectory_published") or "search" in status:
                result["formation_optimizer_status"] = {
                    key: status[key] for key in ("plan_id", "status", "wall_time_s", "duration_s",
                                                "search", "trajectory_published", "quality")
                    if key in status
                }
        elif message.startswith("search_success "):
            result["single_search"] = {match[1]: float(match[2]) for match in re.finditer(
                r"(?<!\S)(id|iterations|seconds|reverse_fallback_iterations|reverse_fallback_seconds)=([0-9.]+)",
                message)}
        elif message.startswith("success "):
            result["single_optimizer"] = {match[1]: float(match[2]) for match in re.finditer(
                r"(?<!\S)(id|time|duration|min_body_clearance)=([0-9.]+)", message)}
    return result


def summary(run):
    root = Path(__file__).resolve().parents[2]
    result = yaml.safe_load((run / "result.yaml").read_text())
    artifacts = []
    for name in ("command.txt", "environment.txt", "result.yaml", "planner_parameters.yaml",
                 "resolved_config.yaml", "planner.log", "controller.log", "launch.log", "capture.json",
                 "launch_events.jsonl", "dds_trace.jsonl", "scene/map.yaml", "scene/map.pgm",
                 "scene/scene_meta.yaml", "metrics/motion.jsonl"):
        candidate = run / name
        if candidate.is_file():
            artifacts.append(candidate)
    artifacts.extend(sorted((run / "trajectories").rglob("*.cdr")))
    artifacts.extend(sorted((run / "trajectories").rglob("*.json")))
    config = yaml.safe_load((run / "resolved_config.yaml").read_text())
    metadata = (run / "environment.txt").read_text() if (run / "environment.txt").is_file() else ""
    revision = re.search(r"^git_commit=(.+)$", metadata, re.MULTILINE)
    try:
        run_path = str(run.relative_to(root))
    except ValueError:
        run_path = str(run)
    return {
        "run_id": run.name,
        "run_directory": run_path,
        "git_sha": revision[1] if revision else None,
        "scenario": config["scenario"],
        "task": config["task"],
        "experiment": config["experiment"],
        "controller": config["controller_backend"],
        "status": result["status"],
        "reason": result["reason"],
        "request_to_verified_plan_wall_s": result.get("planner_wall_s"),
        "reference_duration_sim_s": result.get("plan_duration_s"),
        "routed_plan_keys": result.get("routed_plan_keys"),
        "metrics": result.get("metrics"),
        "planner_phases": planner(run),
        "controllers": controllers(run),
        "dds_events_observed": list(json_rows(run / "dds_trace.jsonl")),
        "artifact_sha256": {str(path.relative_to(run)): sha256(path.read_bytes()).hexdigest()
                            for path in artifacts},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument("-o", "--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.write_text(json.dumps([summary(path.resolve()) for path in args.runs],
                                     indent=2, ensure_ascii=False, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
