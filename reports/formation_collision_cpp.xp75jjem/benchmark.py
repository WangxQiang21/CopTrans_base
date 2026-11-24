"""Compare installed native and Python collision kernels on the same missions."""

import argparse
from hashlib import sha256
import json
from pathlib import Path
from statistics import median
from time import perf_counter

from ament_index_python.packages import get_package_share_directory
import numpy as np
import yaml

from fwis_backends.multi_4wis import formation_collision as collision
from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_backends.multi_4wis.formation_retiming import FormationTimingLimits
from fwis_backends.multi_4wis.uniform_pipeline import UniformPlanningRequest, plan_uniform
from fwis_backends.multi_4wis.uniform_search import GraphSearchConfig, SuccessorConfig
from ranger_mini_gz.scene_generator import load_scene
from ranger_mini_gz.scene_geometry import rasterize


CASES = ("factory_indoor_01", "port_01", "factory_outdoor_01")


def make_request(name, limits):
    scene = load_scene(name)
    experiment = scene.experiments["formation"]
    start, goal = (FormationPose(*experiment[key]) for key in ("start", "goal"))
    return UniformPlanningRequest(
        grid=Grid(rasterize(scene.obstacles, scene.size, scene.resolution),
                  float(np.float32(scene.resolution)), 0., 0.),
        model=FormationModel([VehicleOffset(i, *offset)
                              for i, offset in zip(scene.vehicle_ids, scene.offsets, strict=True)]),
        vehicle_shapes=dict.fromkeys(scene.vehicle_ids, collision.VehicleShape(*scene.vehicle_size[:2])),
        start_qf=start, goal_qf=goal, start_headings=dict.fromkeys(scene.vehicle_ids, start.psi),
        plan_id=600001, formation_id=scene.formation_id,
        search_config=GraphSearchConfig(direct_goal_translation=True, continuous_a_refinement=True),
        successor_config=SuccessorConfig(timing_limits=limits, goal_formation_heading_free=False),
        cargo_footprint=collision.CargoFootprint(*scene.cargo.size[:2], *scene.cargo.offset))


def run(request, native, record=True):
    module = collision._formation_collision_native
    sweep = collision.FormationCollisionChecker.swept_check_analytic
    elapsed_sweep = 0.

    def timed_sweep(checker, *args, **kwargs):
        nonlocal elapsed_sweep
        assert (checker._native_kernel is not None) == native
        started = perf_counter()
        try:
            return sweep(checker, *args, **kwargs)
        finally:
            elapsed_sweep += perf_counter() - started

    try:
        collision._formation_collision_native = module if native else None
        collision.FormationCollisionChecker.swept_check_analytic = timed_sweep
        started = perf_counter()
        result = plan_uniform(request)
        wall = perf_counter() - started
    finally:
        collision._formation_collision_native = module
        collision.FormationCollisionChecker.swept_check_analytic = sweep
    assert result.executable, (result.reason, result.detail)
    if not record:
        return None
    plan = result.plan
    times = sorted(set(plan.default_sample_times()) |
                   {i * .05 for i in range(int(plan.duration_s / .05) + 1)})
    sample = plan.sample(times)
    return {
        "planning_s": wall, "sweep_s": elapsed_sweep,
        "trajectory_duration_s": plan.duration_s, "samples": len(times),
        "model_sha256": plan.model_sha256(),
        "dense_trajectory_sha256": sha256(repr(sample.timed_trajectory).encode()).hexdigest(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("benchmark.json"))
    args = parser.parse_args()
    assert args.repeat >= 1
    module = collision._formation_collision_native
    assert module is not None, "Build and source the ROS workspace with pybind11"
    config_path = Path(get_package_share_directory("fwis_bringup")) / "config/test_defaults.yaml"
    config = yaml.safe_load(config_path.read_text())
    limits = FormationTimingLimits(**config["profiles"]["multi"]["timing_limits"])
    report = {
        "method": "Same process, map, payload, limits and configuration. Warm both kernels, "
                  "alternate execution order, then compare model and 50 ms trajectory hashes. "
                  "Planning timer ends before dense sampling or ROS transport.",
        "extension": module.__file__, "repeats_per_case_per_kernel": args.repeat,
        "limits": limits.as_dict(), "scenes": {},
    }
    for name in CASES:
        request = make_request(name, limits)
        values = {"python": [], "cpp": []}
        run(request, False, record=False)
        run(request, True, record=False)
        for trial in range(args.repeat):
            for native in ((False, True) if trial % 2 == 0 else (True, False)):
                key = "cpp" if native else "python"
                row = run(request, native)
                values[key].append(row)
                print(name, key, trial + 1, f"{row['planning_s']:.3f}s", flush=True)
        runs = values["python"] + values["cpp"]
        for key in ("trajectory_duration_s", "samples", "model_sha256", "dense_trajectory_sha256"):
            assert len({row[key] for row in runs}) == 1, (name, key, values)
        medians = {key: {stat: median(row[stat] for row in rows)
                        for stat in ("planning_s", "sweep_s")} for key, rows in values.items()}
        report["scenes"][name] = {"trials": values, "medians": medians, "trajectory_identical": True}
        args.output.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
