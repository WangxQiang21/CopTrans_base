#!/usr/bin/env python3
"""Replay one archived successful Uniform V0.2 request without ROS transport."""

import argparse
from dataclasses import replace
from hashlib import sha256
import json
import os
from pathlib import Path
from resource import RUSAGE_SELF, getrusage
from statistics import median
from time import perf_counter, process_time

import numpy as np
import yaml
from rclpy.serialization import deserialize_message

from fwis_backends.multi_4wis import formation_collision, formation_retiming, uniform_pipeline, uniform_search
from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_v3_msgs.msg import FormationPlaybackPlan
from ranger_mini_gz.scene_generator import read_map


def replay_request(run, base_only, reuse):
    cdr = run / "trajectories/planner.cdr"
    envelope = deserialize_message(cdr.read_bytes(), FormationPlaybackPlan)
    model = json.loads(envelope.model_json)
    params = yaml.safe_load((run / "planner_parameters.yaml").read_text())
    config = yaml.safe_load((run / "resolved_config.yaml").read_text())
    grid_metadata, occupied = read_map(run / "scene/map.yaml")
    request = uniform_pipeline.UniformPlanningRequest(
        grid=Grid(occupied != 0, float(np.float32(grid_metadata["resolution"])), *grid_metadata["origin"][:2]),
        model=FormationModel([VehicleOffset(**entry) for entry in model["offsets"]]),
        vehicle_shapes={item["vehicle_id"]: formation_collision.VehicleShape(item["length_m"], item["width_m"])
                        for item in model["vehicle_shapes"]},
        start_qf=FormationPose(*model["initial_state"]["qf"]),
        goal_qf=FormationPose(*config["goal"]),
        start_headings={item["vehicle_id"]: item["theta"] for item in model["initial_state"]["headings"]},
        goal_headings=None,
        plan_id=envelope.plan_id,
        formation_id=envelope.formation_id,
        search_config=uniform_search.GraphSearchConfig(direct_goal_translation=True,
                                                       continuous_a_refinement=not base_only),
        successor_config=uniform_search.SuccessorConfig(
            timing_limits=formation_retiming.FormationTimingLimits(**params["timing_limits"]),
            goal_formation_heading_free=False),
        cargo_footprint=formation_collision.CargoFootprint(**params["cargo_footprint"]),
    )
    if reuse:
        request = replace(request, reuse_terminal_certificate=True)
    return envelope.model_sha256, request


def stats(values):
    ordered = sorted(values)
    n = len(ordered)
    result = {"p50": median(ordered), "max": ordered[-1]}
    if n >= 30:
        result["p95"] = ordered[(95 * n + 99) // 100 - 1]
    if n >= 100:
        result["p99"] = ordered[(99 * n + 99) // 100 - 1]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--warmups", type=int, default=5)
    parser.add_argument("--repeat", type=int, default=30)
    parser.add_argument("--base-only", action="store_true")
    parser.add_argument("--reuse", action="store_true")
    args = parser.parse_args()
    if args.warmups < 0 or args.repeat <= 0:
        parser.error("need nonnegative warmups and positive repeat")
    expected, request = replay_request(args.run, args.base_only, args.reuse)
    trials = []
    warmups = []
    for index in range(args.repeat + args.warmups):
        start, cpu_start = perf_counter(), process_time()
        planned = uniform_pipeline.plan_uniform(request)
        cpu, wall = process_time() - cpu_start, perf_counter() - start
        plan = planned.plan
        model_digest = plan.model_sha256() if planned.executable else None
        sample = {
            "index": index,
            "wall_s": wall,
            "cpu_s": cpu,
            "peak_rss_kib_so_far": getrusage(RUSAGE_SELF).ru_maxrss,
            "executable": planned.executable,
            "model_sha256": model_digest,
            "matches_ros_model": model_digest == expected,
            "duration_s": plan.duration_s if plan else None,
            "mode_sequence": [edge.mode for edge in plan.edges] if plan else [],
            "reason": planned.reason,
            "search_wall_s": planned.search.wall_time_s if planned.search else None,
            "search_expansions": planned.search.expansions if planned.search else None,
            "stages": getattr(planned, "stages", {}),
        }
        if index < args.warmups:
            warmups.append(sample)
        else:
            trials.append(sample)
        print(f"trial={index} wall_s={wall:.3f} executable={planned.executable}", flush=True)
    request_sha = sha256((args.run / "resolved_config.yaml").read_bytes()).hexdigest()
    native = formation_collision._formation_collision_native
    frozen = {
        "run_id": args.run.name,
        "source_module": uniform_pipeline.__file__,
        "native_module": native.__file__ if native else None,
        "warmups": warmups,
        "trials": trials,
        "statistics": {step: stats([sample[step] for sample in trials]) for step in ("wall_s", "cpu_s")},
        "baseline_profile_configuration_sha256": request_sha,
        "expected_ros_model_sha256": expected,
        "base_only": args.base_only,
        "reuse_terminal_certificate": args.reuse,
        "cpu_affinity": sorted(os.sched_getaffinity(0)),
    }
    args.output.write_text(json.dumps(frozen, indent=2, allow_nan=False) + "\n")
    if not all(sample["executable"] for sample in trials):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
