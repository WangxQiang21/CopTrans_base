"""Isolate the native short-turn planner with data from the captured ROS run."""

from collections import defaultdict
from dataclasses import replace
from functools import wraps
from importlib import import_module
from inspect import getattr_static
import json
from pathlib import Path
from statistics import median
from time import perf_counter

import numpy as np
import yaml

from fwis_backends.multi_4wis import (
    formation_collision, formation_retiming, uniform_curve_refinement,
    uniform_pipeline, uniform_quality, uniform_search, uniform_trajectory,
)
from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_v3_msgs.msg import FormationPlaybackPlan
from ranger_mini_gz.scene_generator import read_map
from rclpy.serialization import deserialize_message


HERE = Path(__file__).resolve().parent


class Meter:
    def __init__(self):
        self.stack = []
        self.entries = defaultdict(list)

    def instrument(self, target, method, label):
        attr = getattr_static(target, method)
        is_class = isinstance(attr, classmethod)
        func = attr.__func__ if is_class else attr

        @wraps(func)
        def measured(*args, **kwargs):
            self.stack.append(label)
            key = "/".join(self.stack)
            started = perf_counter()
            value = None
            result = {}
            try:
                value = func(*args, **kwargs)
                return value
            except Exception as error:
                result["raised"] = type(error).__name__
                result["detail"] = str(error)
                raise
            finally:
                result["wall_s"] = perf_counter() - started
                if label == "candidate":
                    result.update(padding=list(args[3]), accepted=value is not None)
                    if value is not None:
                        result["duration_s"] = value[0].duration_s
                if label == "curve_fit":
                    result["path_returned"] = value is not None
                if label in {"curve_geometry", "curve_retime", "curve_dynamics", "curve_collision"}:
                    result.update(status=getattr(value, "status", None),
                                  reason=getattr(value, "reason", None),
                                  passed=getattr(value, "passed", None))
                if label == "uniform_audit":
                    result["feasible"] = getattr(value, "feasible", None)
                self.entries[key].append(result)
                self.stack.pop()

        setattr(target, method, classmethod(measured) if is_class else measured)


def make_request():
    capture = json.loads((HERE / "capture.json").read_text())
    run = Path(capture["run"])
    envelope = deserialize_message((run / "trajectories/planner.cdr").read_bytes(), FormationPlaybackPlan)
    plan = json.loads(envelope.model_json)
    params = yaml.safe_load((run / "planner_parameters.yaml").read_text())
    config = yaml.safe_load((run / "resolved_config.yaml").read_text())
    metadata, occupied = read_map(run / "scene/map.yaml")
    request = uniform_pipeline.UniformPlanningRequest(
        grid=Grid(occupied != 0, float(np.float32(metadata["resolution"])), *metadata["origin"][:2]),
        model=FormationModel([VehicleOffset(**entry) for entry in plan["offsets"]]),
        vehicle_shapes={p["vehicle_id"]: formation_collision.VehicleShape(p["length_m"], p["width_m"])
                        for p in plan["vehicle_shapes"]},
        start_qf=FormationPose(*plan["initial_state"]["qf"]),
        goal_qf=FormationPose(*config["goal"]),
        start_headings={p["vehicle_id"]: p["theta"] for p in plan["initial_state"]["headings"]},
        goal_headings=None, plan_id=plan["plan_id"], formation_id=plan["formation_id"],
        search_config=uniform_search.GraphSearchConfig(direct_goal_translation=True,
                                                       continuous_a_refinement=True),
        successor_config=uniform_search.SuccessorConfig(
            timing_limits=formation_retiming.FormationTimingLimits(**params["timing_limits"]),
            goal_formation_heading_free=False),
        cargo_footprint=formation_collision.CargoFootprint(**params["cargo_footprint"]))
    return capture, request, envelope


def profile(capture, request, envelope, meter):
    base_builder = uniform_pipeline._build_and_audit
    no_refinement = replace(request, search_config=replace(request.search_config,
                                                         continuous_a_refinement=False))
    results = {"default": [], "without_c2": [], "without_c2_reuse": []}
    plan_args = {"default": request, "without_c2": no_refinement, "without_c2_reuse": no_refinement}

    def run_variant(variant):
        results_cache = {}
        reused = 0

        def cached_builder(edges, problem, zero_motion=False):
            nonlocal reused
            edges = tuple(edges)
            key = (id(problem), tuple(map(id, edges)), zero_motion)
            if key in results_cache:
                reused += 1
                return results_cache[key][1]
            value = base_builder(edges, problem, zero_motion)
            if value[2].feasible:
                results_cache[key] = edges, value
            return value

        uniform_pipeline._build_and_audit = cached_builder if variant == "without_c2_reuse" else base_builder
        try:
            meter.entries.clear()
            started = perf_counter()
            result = uniform_pipeline.plan_uniform(plan_args[variant])
            elapsed = perf_counter() - started
        finally:
            uniform_pipeline._build_and_audit = base_builder
        assert result.executable, (result.reason, result.detail)
        assert result.plan.model_sha256() == envelope.model_sha256, (variant, result.detail)
        return {"wall_s": elapsed, "search_s": result.search.wall_time_s,
                "reuse_count": reused, "model_sha256": result.plan.model_sha256(),
                "expansions": result.search.expansions, "candidates": result.search.candidate_count,
                "stages": dict(meter.entries)}

    for name in results:
        run_variant(name)
    for trial in range(5):
        for name in (results if trial % 2 == 0 else reversed(results)):
            result = run_variant(name)
            results[name].append(result)
            print(name, trial + 1, f"{result['wall_s']:.3f} s", "reuse", result["reuse_count"], flush=True)

    summary = {}
    for name, series in results.items():
        keys = list(series[0]["stages"])
        summary[name] = {
            "planning_wall_median_s": median(run["wall_s"] for run in series),
            "stage_medians_s": {key: median(sum(s["wall_s"] for s in run["stages"].get(key, ()))
                                             for run in series) for key in keys},
        }
    return {
        "run": capture["run"], "native_extension": formation_collision._formation_collision_native.__file__,
        "trials_per_variant": 5, "expected_model_sha256": envelope.model_sha256,
        "all_trajectory_models_identical": True,
        "analysis_only_reuse": "Use the accepted result again only for the same request and same edge objects.",
        "summary": summary, "trials": results,
    }


def main():
    assert formation_collision._formation_collision_native is not None
    capture, request, envelope = make_request()
    audit = uniform_quality._UniformAuditor
    retime = import_module("fwis_backends.multi_4wis.formation_continuous_a.timing")
    meter = Meter()
    for target, method, label in (
        (uniform_pipeline, "plan_uniform", "planner"),
        (uniform_search.UniformGraphSearch, "search", "search"),
        (uniform_pipeline, "_build_and_audit", "build_and_audit"),
        (uniform_trajectory.UniformTimedPlan, "from_edges", "time_law"),
        (uniform_trajectory.UniformTimedPlan, "sample", "sample"),
        (formation_retiming, "_peak_table", "peak_table"),
        (uniform_pipeline, "audit_uniform_timed_plan", "uniform_audit"),
        (audit, "_check_dynamic_peaks", "dynamic_peak_audit"),
        (audit, "_check_analytic_sweeps", "sweep_audit"),
        (formation_collision.FormationCollisionChecker, "swept_check_analytic", "sweep"),
        (uniform_curve_refinement, "refine_turn", "refinement"),
        (uniform_curve_refinement, "_candidate", "candidate"),
        (uniform_curve_refinement, "_fit_path", "curve_fit"),
        (uniform_curve_refinement, "audit_geometry", "curve_geometry"),
        (uniform_curve_refinement, "retime_path", "curve_retime"),
        (uniform_curve_refinement, "audit_dynamics", "curve_dynamics"),
        (uniform_curve_refinement, "audit_collision", "curve_collision"),
        (retime, "_selection_bounds", "curve_selection_bounds"),
        (retime, "_certify_bounds", "curve_certify_bounds"),
    ):
        meter.instrument(target, method, label)
    result = profile(capture, request, envelope, meter)
    (HERE / "planner_profile.json").write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
