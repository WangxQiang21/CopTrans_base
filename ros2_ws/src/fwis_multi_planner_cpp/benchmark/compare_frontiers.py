#!/usr/bin/env python3
"""Pair the two frontiers on fixed queue work and a fixed certified graph search."""

import argparse
from dataclasses import replace
import importlib
import json
import os
from pathlib import Path
import time

import numpy as np

from fwis_backends.multi_4wis import formation_collision
from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_backends.multi_4wis.formation_collision import CargoFootprint, VehicleShape
from fwis_backends.multi_4wis.uniform_pipeline import UniformPlanningRequest, plan_uniform
from fwis_backends.multi_4wis.uniform_search import (
    GraphSearchConfig,
    SuccessorConfig,
    _NativeFrontier,
    _PythonFrontier,
    initial_state,
)


def queue_problem():
    rng = np.random.default_rng(555321)
    cfg = SuccessorConfig()
    model = FormationModel([VehicleOffset(1, 1.0, 0.0), VehicleOffset(2, 0.0, 1.0),
                            VehicleOffset(3, 0.0, -1.0)])
    root = initial_state(model, FormationPose(3.0, 4.0, 0.0), dict.fromkeys(model.vehicle_ids, 0.0))
    entries = [
        replace(root, qf=FormationPose(0.4 * x, 0.4 * y, 0.0), g=float(g),
                a_labels=("A_straight_goal_gear+++",) if x % 2 else ())
        for (x, y), g in zip(rng.integers(1, 40, size=(2000, 2)), rng.uniform(0.0, 300.0, 2000))
    ]

    def run(frontier_class):
        queue = frontier_class(cfg, lambda state, _: state.g + abs(state.qf.y - 4.0))
        offers = [queue.offer(state) for state in entries]
        order = []
        while queue:
            state = queue.pop()
            order.append(None if state is None else state.g)
        return offers, order

    return {"python": lambda: run(_PythonFrontier), "cpp": lambda: run(_NativeFrontier)}


def planning_problem():
    model = FormationModel([VehicleOffset(1, 1.0, 0.0), VehicleOffset(2, 0.0, 1.0),
                            VehicleOffset(3, 0.0, -1.0)])
    request = UniformPlanningRequest(
        Grid(np.zeros((100, 100), dtype=bool), 0.1, 0.0, 0.0),
        model, dict.fromkeys(model.vehicle_ids, VehicleShape(0.738, 0.5)),
        FormationPose(3.0, 4.0, 0.0), dict.fromkeys(model.vehicle_ids, 0.0),
        FormationPose(5.0, 4.0, 0.0),
        search_config=GraphSearchConfig(max_seconds=20.0, max_expansions=24),
        cargo_footprint=CargoFootprint(1.0, 1.5, 0.32, 0.0),
    )

    def run(native_frontier):
        result = plan_uniform(replace(
            request, search_config=replace(request.search_config, native_frontier=native_frontier)))
        assert result.executable and result.search.expansions > 0
        return result

    return {"python": lambda: run(False), "cpp": lambda: run(True)}


def collect(variants, planner=False):
    samples = {name: [] for name in variants}
    model_hash = None
    for repeat in range(35):
        results = {}
        for name in sorted(variants, reverse=repeat % 2 == 1):
            start = time.perf_counter()
            result = variants[name]()
            wall_s = time.perf_counter() - start
            if planner:
                model_hash = result.plan.model_sha256()
                results[name] = model_hash, result.search.expansions, result.evaluation.as_status()
                sample = {
                    "wall_ms": 1000 * wall_s,
                    "search_ms": 1000 * result.search.wall_time_s,
                    "first_certified_ms": 1000 * result.stages["first_certified_s"],
                    "expansions": result.search.expansions,
                }
            else:
                results[name] = result
                sample = {"wall_ms": 1000 * wall_s}
            if repeat >= 5:
                samples[name].append(sample)
        assert results["python"] == results["cpp"]
    summary = {
        name: {f"{metric}_p{q}": float(np.percentile([item[metric] for item in rows], q))
               for metric in ("wall_ms", "search_ms", "first_certified_ms")
               if metric in rows[0] for q in (50, 95, 100)}
        for name, rows in samples.items()
    }
    return {"samples": samples, "summary": summary, "model_sha256": model_hash}


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("--output", type=Path, required=True)
    args.add_argument("--collision-module-directory", type=Path,
                      help="directory with the native collision binding when running from source")
    options = args.parse_args()
    if options.collision_module_directory is not None:
        package = importlib.import_module("fwis_backends.multi_4wis")
        package.__path__.insert(0, str(options.collision_module_directory.resolve()))
        formation_collision._formation_collision_native = importlib.import_module(
            "fwis_backends.multi_4wis._formation_collision_native")
    module = formation_collision._formation_collision_native
    native_frontier = importlib.import_module("fwis_multi_frontier_native")
    results = {
        "warmups": 5,
        "repeats": 30,
        "native_sat": module is not None,
        "native_spin_sweep_available": module is not None and hasattr(
            module.CollisionKernel, "check_spin_samples"),
        "native_all_sweeps_available": module is not None and hasattr(
            module.CollisionKernel, "check_edge_samples"),
        "native_candidate_library_available": hasattr(native_frontier, "generate_lattice"),
        "cpu_affinity": sorted(os.sched_getaffinity(0)),
        "queue_only": collect(queue_problem()),
        "graph_to_certificate": collect(planning_problem(), planner=True),
    }
    options.output.write_text(json.dumps(results, indent=2) + "\n")
    for name in ("queue_only", "graph_to_certificate"):
        print(name, json.dumps(results[name]["summary"]))


if __name__ == "__main__":
    main()
