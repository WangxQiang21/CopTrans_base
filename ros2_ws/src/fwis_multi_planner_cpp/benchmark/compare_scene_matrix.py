#!/usr/bin/env python3
"""Offline native/Python frontier parity on the repository's generated maps."""

import argparse
from dataclasses import replace
from hashlib import sha256
import importlib
import json
import math
from pathlib import Path
from time import perf_counter

import numpy as np
import yaml

from fwis_backends.multi_4wis import formation_collision
from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_backends.multi_4wis.formation_collision import CargoFootprint, VehicleShape
from fwis_backends.multi_4wis.formation_retiming import FormationTimingLimits
from fwis_backends.multi_4wis.uniform_pipeline import UniformPlanningRequest, plan_uniform
from fwis_backends.multi_4wis.uniform_search import GraphSearchConfig, SuccessorConfig
from ranger_mini_gz.scene_generator import read_map


def make_request(folder, experiment, offsets):
    metadata = yaml.safe_load((folder / "scene_meta.yaml").read_text())
    map_info, occupied = read_map(folder / "map.yaml")
    layout = metadata["experiments"]["formation"]
    start = FormationPose(*layout["start"])
    if experiment == "full_route":
        goal = FormationPose(*layout["goal"])
    else:
        dx, dy, yaw = offsets[experiment]["goal_offset"]
        cos_yaw, sin_yaw = math.cos(start.psi), math.sin(start.psi)
        goal = FormationPose(start.x + cos_yaw * dx - sin_yaw * dy,
                             start.y + sin_yaw * dx + cos_yaw * dy, start.psi + yaw)
    cargo = metadata["cargo"]
    limits = FormationTimingLimits(v_max=0.16, a_max=0.22, yaw_rate_max=0.4,
                                   yaw_accel_max=0.45, r_min_ackermann=0.9, mode_switch_duration=0.7)
    model = FormationModel([VehicleOffset(i, *offset)
                            for i, offset in enumerate(metadata["formation_offsets_m"], 1)])
    return UniformPlanningRequest(
        Grid(occupied != 0, float(np.float32(map_info["resolution"])), *map_info["origin"][:2]),
        model, dict.fromkeys(model.vehicle_ids, VehicleShape(0.738, 0.5)),
        start, dict.fromkeys(model.vehicle_ids, start.psi), goal,
        search_config=GraphSearchConfig(direct_goal_translation=True),
        successor_config=SuccessorConfig(timing_limits=limits),
        cargo_footprint=CargoFootprint(*cargo["size_m"][:2], *cargo["offset_m"]),
        reuse_terminal_certificate=False,
    )


def sample(request, native):
    start = perf_counter()
    outcome = plan_uniform(replace(
        request, search_config=replace(request.search_config, native_frontier=native)))
    search = outcome.search
    result = {
        "executable": outcome.executable,
        "reason": outcome.reason,
        "model_sha256": outcome.plan.model_sha256() if outcome.plan is not None else None,
        "mode_sequence": [edge.mode for edge in outcome.plan.edges] if outcome.plan else [],
        "duration_s": outcome.plan.duration_s if outcome.plan else None,
        "search_status": search.status if search else None,
        "expansions": search.expansions if search else None,
        "used_connector": search.direct_goal_translation if search else None,
        "wall_s": perf_counter() - start,
        "search_s": search.wall_time_s if search else None,
    }
    comparable = {key: value for key, value in result.items() if key not in ("wall_s", "search_s")}
    return result, comparable


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--collision-module-directory", type=Path)
    cli.add_argument("--output", type=Path, required=True)
    args = cli.parse_args()

    if args.collision_module_directory is not None:
        package = importlib.import_module("fwis_backends.multi_4wis")
        package.__path__.insert(0, str(args.collision_module_directory.resolve()))
        formation_collision._formation_collision_native = importlib.import_module(
            "fwis_backends.multi_4wis._formation_collision_native")
    module = formation_collision._formation_collision_native
    if module is None or not hasattr(module.CollisionKernel, "check_spin_samples"):
        cli.error("the new native SAT/Spin extension is required")

    experiments = yaml.safe_load(Path("ros2_ws/src/fwis_bringup/config/experiments.yaml").read_text())
    root = Path("ros2_ws/generated/scenes")
    cases = [
        ("factory_indoor_01", "short_turn"),
        ("factory_indoor_01", "short_straight"),
        ("factory_indoor_01", "full_route"),
        ("port_01", "full_route"),
        ("factory_outdoor_01", "full_route"),
        ("factory_indoor_01_wide", "full_route"),
        ("factory_indoor_01_critical", "full_route"),
        ("factory_indoor_01_too_narrow", "full_route"),
    ]
    records = []
    for name, experiment in cases:
        request = make_request(root / name, experiment, experiments)
        reference, reference_signature = sample(request, False)
        cpp, cpp_signature = sample(request, True)
        records.append({
            "scenario": name,
            "experiment": experiment,
            "source_pgm_sha256": sha256((root / name / "map.pgm").read_bytes()).hexdigest(),
            "python": reference,
            "cpp": cpp,
            "equivalent": reference_signature == cpp_signature,
        })
        print(name, experiment, cpp["reason"], records[-1]["equivalent"], flush=True)
        args.output.write_text(json.dumps(records, indent=2) + "\n")

    if not all(row["equivalent"] for row in records):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
