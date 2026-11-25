#!/usr/bin/env python3
"""Profile the current single and formation planner on preserved local inputs."""

import argparse
from collections import Counter
from contextlib import contextmanager
from dataclasses import asdict, replace
import gzip
from hashlib import sha256
import importlib.util
import json
import math
from pathlib import Path
import platform
import runpy
import signal
from threading import Event
from time import perf_counter
from unittest.mock import patch

import numpy as np
import yaml

from fwis_backends.multi_4wis import formation_collision, uniform_pipeline
from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_backends.multi_4wis.formation_collision import CargoFootprint, VehicleShape
from fwis_backends.multi_4wis.formation_continuous_a import bounds
from fwis_backends.multi_4wis.formation_retiming import FormationTimingLimits
from fwis_backends.multi_4wis.uniform_curve_clock import _curve_clock_bounds
from fwis_backends.multi_4wis.uniform_mixed_trajectory import UniformMixedTimedPlan
from fwis_backends.multi_4wis.uniform_search import GraphSearchConfig, SuccessorConfig
from fwis_backends.single_continuous_mode_path import mode_master, quality
from fwis_backends.single_continuous_mode_path.core import Action, Grid as SingleGrid, Pose
from fwis_backends.single_continuous_mode_path.retiming import MotionLimits


ROOT = Path(__file__).resolve().parents[2]
BRINGUP = ROOT / "ros2_ws/src/fwis_bringup/config"
SINGLE_CONFIG = ROOT / "ros2_ws/src/fwis_backends/single_continuous_mode_path/config/params_restricted.yaml"
MAPS = ROOT / "ros2_ws/generated/scenes"
PROFILE_INTERVAL_S = .005


def read_yaml(path):
    return yaml.safe_load(path.read_text())


def archive_grid(archive):
    metadata = read_yaml(archive / "scene/map.yaml")
    uncompressed = archive / "scene" / metadata["image"]
    pgm = (uncompressed.read_bytes() if uncompressed.is_file() else
           gzip.decompress(uncompressed.with_suffix(".pgm.gz").read_bytes()))
    magic, size, maximum, pixels = pgm.split(b"\n", 3)
    width, height = map(int, size.split())
    if magic != b"P5" or maximum != b"255":
        raise ValueError("expected the archived 8-bit binary PGM")
    cells = np.flipud(np.frombuffer(pixels, dtype=np.uint8).reshape(height, width))
    return metadata, cells != 254, sha256(pgm).hexdigest()


def archived_formation_message(archive):
    from fwis_v3_msgs.msg import FormationPlaybackPlan
    from rclpy.serialization import deserialize_message

    artifact = archive / "trajectories/planner.cdr.gz"
    return deserialize_message(gzip.decompress(artifact.read_bytes()), FormationPlaybackPlan)


def archived_formation_request(archive):
    metadata, occupied, pgm_sha = archive_grid(archive)
    message = archived_formation_message(archive)
    document = json.loads(message.model_json)
    while "offsets" not in document:
        document = document["baseline"] if "baseline" in document else document["prefix"]
    cfg = read_yaml(archive / "planner_parameters.yaml")
    data = read_yaml(archive / "experiment.json")
    if pgm_sha != data["map_sha256"]:
        raise ValueError(f"map archive integrity check failed for {archive}")
    model = FormationModel([VehicleOffset(**row) for row in document["offsets"]])
    shapes = {row["vehicle_id"]: VehicleShape(row["length_m"], row["width_m"])
              for row in document["vehicle_shapes"]}
    heading = {row["vehicle_id"]: row["theta"] for row in document["initial_state"]["headings"]}
    limits = FormationTimingLimits(**document["timing_limits"])
    request = uniform_pipeline.UniformPlanningRequest(
        Grid(occupied, float(np.float32(metadata["resolution"])), *metadata["origin"][:2]),
        model, shapes, FormationPose(*data["start"]), heading, FormationPose(*data["goal"]),
        plan_id=int(message.plan_id), formation_id=message.formation_id, cancelled=Event().is_set,
        successor_config=SuccessorConfig(timing_limits=limits, allow_reverse=True, goal_heading_free=True),
        search_config=GraphSearchConfig(
            direct_goal_translation=True, native_frontier=cfg["native_frontier"],
            convex_corridor=cfg["convex_corridor"], prefer_ackermann=cfg["prefer_ackermann"]),
        cargo_footprint=CargoFootprint(**cfg["cargo_footprint"]),
        reuse_terminal_certificate=cfg["reuse_terminal_certificate"], reachability_retime=cfg["reachability_retime"],
    )
    return request, {"map_pgm_sha256": pgm_sha, "archived_plan_sha256": message.model_sha256,
                     "archive": str(archive), "planner_parameters": cfg}


def generated_formation_request(experiment):
    fixture = ROOT / "ros2_ws/src/fwis_multi_planner_cpp/benchmark/compare_scene_matrix.py"
    scene = MAPS / "factory_indoor_01"
    base = runpy.run_path(str(fixture))["make_request"](
        scene, experiment, read_yaml(BRINGUP / "experiments.yaml"))
    timing = replace(base.successor_config.timing_limits, v_max=.50, a_max=.36)
    base = replace(base, reuse_terminal_certificate=True, reachability_retime=True,
                   successor_config=replace(base.successor_config, timing_limits=timing),
                   search_config=replace(base.search_config, native_frontier=True, convex_corridor=True,
                                         prefer_ackermann=True))
    return base, {"experiment": experiment, "map_pgm_sha256": sha256((scene / "map.pgm").read_bytes()).hexdigest()}


def formation_solve(base, native):
    grid = Grid(base.grid.occ, base.grid.res, base.grid.ox, base.grid.oy)
    request = replace(base, grid=grid, search_config=replace(base.search_config, native_frontier=native))
    started = perf_counter()
    result = uniform_pipeline.plan_uniform(request)
    elapsed = perf_counter() - started
    if (not result.executable or not result.evaluation.independent_audit
            or not result.evaluation.collision_certificate):
        raise RuntimeError(f"formation solve not independently certified: {result.reason} {result.detail}")
    plan = result.plan
    return {"wall_s": elapsed, "plan_s": plan.duration_s, "sha256": plan.model_sha256(),
            "model_version": result.evaluation.metrics["model_version"], "stages_s": result.stages,
            "search_backend": result.search.frontier_backend, "search_expansions": result.search.expansions,
            "request_sha256": result.request_sha256}


def clock_archive_solver(archive):
    message = archived_formation_message(archive)
    plan = UniformMixedTimedPlan.from_model_json(message.model_json)
    if plan.clock is None:
        raise ValueError(f"no archived continuous-curve replacement clock in {archive}")

    def solve():
        started = perf_counter()
        values, _ = _curve_clock_bounds(plan.curve, plan.clock)
        elapsed = perf_counter() - started
        digest = sha256(json.dumps({k: v.hex() for k, v in values.items()}, sort_keys=True).encode()).hexdigest()
        return {"wall_s": elapsed, "stages_s": {}, "sha256": digest, "bounds": values}

    return solve, {"archive": str(archive), "scope": "C2 replacement-clock dynamic bound kernel",
                   "archived_plan_sha256": message.model_sha256, "clock": asdict(plan.clock)}


def single_archive_solver(archive):
    config = read_yaml(SINGLE_CONFIG)
    files = sorted((archive / "trajectories/planner_reports").glob("*.json"), reverse=True)
    for path in files:
        document = json.loads(path.read_text())
        if document.get("input_seed_candidates"):
            break
    else:
        raise ValueError(f"no archived single-car ModePathMaster seed in {archive}")
    metadata, occupied, pgm_sha = archive_grid(archive)
    qconfig = config["quality"]
    contract = document["execution_contract"]
    limits = MotionLimits(**contract["limits"])
    cost = quality.QualityProfile(**qconfig["cost"])
    solver_config = mode_master.ModeMasterConfig(
        **{**qconfig["search"], "allow_reverse": document["allow_reverse"],
           "allow_shunting": document["direction_contract"]["allow_shunting"],
           "continuous_motion_v2": True})
    seeds = [([Action(*row) for row in entry["actions"]], entry["source"])
             for entry in document["input_seed_candidates"]]
    start, goal = [Pose(*document[key]) for key in ("start", "goal")]

    def solve():
        grid = SingleGrid(occupied, metadata["resolution"], *metadata["origin"][:2])
        evaluator = quality.TrajectoryEvaluator(
            grid, start, goal, limits, contract["length_m"], contract["width_m"],
            .02, math.radians(1), cost, allow_reverse=solver_config.allow_reverse,
            allow_shunting=solver_config.allow_shunting)
        started = perf_counter()
        winner, report = mode_master.ModePathMaster(evaluator, solver_config).solve(
            seeds[0][0], auxiliary_seeds=seeds[1:])
        elapsed = perf_counter() - started
        if winner["audit"]["status"] != "FEASIBLE":
            raise RuntimeError(f"single-car seed replay not certified: {winner['audit']}")
        return {"wall_s": elapsed, "stages_s": report["stage_timings_s"],
                "source": winner["source"], "plan_s": winner["timed"].duration,
                "actions": [asdict(action) for action in winner["actions"]]}

    return solve, {"archive": str(archive), "report": str(path), "start": tuple(document["start"]),
                   "goal": tuple(document["goal"]), "input_seed_candidates": document["input_seed_candidates"],
                   "map_pgm_sha256": pgm_sha, "solver_config": asdict(solver_config),
                   "limits": asdict(limits), "cost": asdict(cost)}


@contextmanager
def cpu_samples():
    """Sample Python stacks at CPU-clock intervals, separate from wall trials."""
    stacks = Counter()

    def collect(_, frame):
        stack = []
        while frame is not None:
            stack.append(frame.f_code)
            frame = frame.f_back
        stacks[tuple(stack)] += 1

    old_handler = signal.signal(signal.SIGPROF, collect)
    old_timer = signal.setitimer(signal.ITIMER_PROF, PROFILE_INTERVAL_S, PROFILE_INTERVAL_S)
    try:
        yield stacks
    finally:
        signal.setitimer(signal.ITIMER_PROF, *old_timer)
        signal.signal(signal.SIGPROF, old_handler)


def summarize_cpu(stacks):
    def symbol(code):
        return f"{code.co_filename}:{code.co_firstlineno}:{code.co_name}"

    own, cumulative = Counter(), Counter()
    for stack, count in stacks.items():
        if stack:
            own[symbol(stack[0])] += count
            cumulative.update({symbol(code): count for code in set(stack)})
    return {"samples": sum(stacks.values()), "interval_cpu_s": PROFILE_INTERVAL_S,
            "self_top": own.most_common(40), "inclusive_top": cumulative.most_common(40),
            "raw_stacks": [[count, [symbol(code) for code in stack]] for stack, count in stacks.items()]}


def summarize_wall(rows):
    keys = sorted(set().union(*(row["stages_s"] for row in rows)))
    values = {"wall_s": [row["wall_s"] for row in rows]}
    for key in keys:
        samples = [row["stages_s"].get(key) for row in rows]
        if all(type(sample) in (int, float) for sample in samples):
            values[key] = samples
    result = {}
    for key, samples in values.items():
        stats = {"p50_s": float(np.percentile(samples, 50)), "max_s": max(samples)}
        if len(samples) >= 30:
            stats["p95_s"] = float(np.percentile(samples, 95))
        if len(samples) >= 100:
            stats["p99_s"] = float(np.percentile(samples, 99))
        result[key] = stats
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generated", action="append", choices=("short_turn", "full_route"))
    parser.add_argument("--formation-archive", action="append", type=Path)
    parser.add_argument("--clock-archive", action="append", type=Path)
    parser.add_argument("--single-archive", action="append", type=Path)
    parser.add_argument("--compare-python-frontier", action="store_true")
    parser.add_argument("--rounding-reference", type=Path,
                        help="Compare against _next_up/down loaded from a previous bounds.py.")
    parser.add_argument("--warmups", type=int, default=5)
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.warmups < 0 or args.samples < 1:
        parser.error("nonnegative warmups and positive sample count required")
    if not any((args.generated, args.formation_archive, args.clock_archive, args.single_archive)):
        parser.error("provide at least one input case")
    extension = formation_collision._formation_collision_native
    detector = importlib.util.find_spec("fwis_multi_frontier_native")
    if extension is None or detector is None:
        parser.error("source the built ROS workspace with the native SAT and frontier modules")
    reference = None
    if args.rounding_reference:
        module = runpy.run_path(str(args.rounding_reference))
        reference = {name: module[name] for name in ("_next_up", "_next_down")}
    output = {"warmups": args.warmups, "formal_trials": args.samples, "platform": platform.platform(),
              "modules": {"multi_pipeline": uniform_pipeline.__file__, "single_mode_master": mode_master.__file__,
                          "native_sat": extension.__file__, "native_frontier": detector.origin,
                          "formation_bounds": bounds.__file__},
              "cpu_sampling_scope": "Distinct run after uninstrumented wall trials. Native calls are attributed to their Python caller.",
              "cases": {}}
    cases = []
    for case in args.generated or []:
        request, provenance = generated_formation_request(case)
        cases.append((case, provenance, request))
    for archive in args.formation_archive or []:
        request, provenance = archived_formation_request(archive.resolve())
        cases.append((archive.name, provenance, request))
    for archive in args.single_archive or []:
        solver, provenance = single_archive_solver(archive.resolve())
        cases.append((archive.name, provenance, solver))
    for archive in args.clock_archive or []:
        solver, provenance = clock_archive_solver(archive.resolve())
        cases.append((f"{archive.name}:clock", provenance, solver))
    for name, provenance, source in cases:
        if callable(source):
            solvers = {"current_clock_bounds" if "scope" in provenance else "python_continuous": source}
        else:
            _, inputs = uniform_pipeline.request_fingerprint(source)
            provenance["request"] = inputs
            solvers = {"cpp_frontier_and_sat": lambda: formation_solve(source, True)}
            if args.compare_python_frontier:
                solvers["python_frontier_cpp_sat"] = lambda: formation_solve(source, False)
        first_name, first_solver = next(iter(solvers.items()))
        if reference is not None and first_name != "python_continuous":
            def reference_solve():
                with patch.multiple(bounds, **reference):
                    return first_solver()

            solvers["reference_rounding"] = reference_solve
        case = {"input": provenance, "variants": {}}
        output["cases"][name] = case
        rows = {key: [] for key in solvers}
        for trial in range(args.samples + args.warmups):
            methods = list(solvers.items())
            for key, run in (methods if trial % 2 == 0 else reversed(methods)):
                rows[key].append(run())
        for key, run in solvers.items():
            with cpu_samples() as stacks:
                profiled = run()
            warmups, trials = rows[key][:args.warmups], rows[key][args.warmups:]
            variant = {"warmups": warmups, "trials": trials, "summary": summarize_wall(trials),
                       "profiled_result": profiled, "cpu_profile": summarize_cpu(stacks)}
            case["variants"][key] = variant
            print(name, key, variant["summary"]["wall_s"], flush=True)
        if "reference_rounding" in rows:
            current = rows[first_name][args.warmups:]
            prior = rows["reference_rounding"][args.warmups:]
            case["paired_output_hash_match"] = sum(
                new["sha256"] == old["sha256"] for new, old in zip(current, prior, strict=True))
        args.output.write_text(json.dumps(output, allow_nan=False, indent=2) + "\n")
        if case.get("paired_output_hash_match", args.samples) != args.samples:
            raise RuntimeError(f"rounding comparison changed {name} outputs; see {args.output}")


if __name__ == "__main__":
    main()
