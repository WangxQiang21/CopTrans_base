#!/usr/bin/env python3
"""Measure nested wall-clock planning stages on fixed generated maps."""

import argparse
from collections import Counter
from contextlib import ExitStack
from dataclasses import replace
from functools import wraps
import json
from pathlib import Path
import runpy
from time import perf_counter
from unittest.mock import patch

import numpy as np
import yaml

from fwis_backends.multi_4wis import formation_collision, uniform_pipeline, uniform_search, uniform_trajectory


ROOT = Path(__file__).resolve().parents[2]
SCENE = ROOT / "ros2_ws/generated/scenes/factory_indoor_01"
TARGETS = (
    (uniform_search.UniformGraphSearch, "search", "search_including_terminal_audit"),
    (formation_collision.FormationCollisionChecker, "static_check", "static_collision_queries"),
    (formation_collision.FormationCollisionChecker, "swept_check_analytic", "swept_collision_queries"),
    (formation_collision.FormationCollisionChecker, "fixed_heading_blocked_cells", "formation_raster"),
    (uniform_pipeline, "audit_uniform_timed_plan", "independent_audit_including_collision"),
    (uniform_trajectory, "schedule_uniform_edges", "base_time_parameterization"),
)


def profile(request):
    totals, counts = Counter(), Counter()

    def timed(function, label):
        @wraps(function)
        def measure(*args, **kwargs):
            started = perf_counter()
            try:
                return function(*args, **kwargs)
            finally:
                totals[label] += perf_counter() - started
                counts[label] += 1
        return measure

    with ExitStack() as patches:
        for namespace, name, label in TARGETS:
            patches.enter_context(patch.object(namespace, name, timed(getattr(namespace, name), label)))
        started = perf_counter()
        result = uniform_pipeline.plan_uniform(request)
        totals["total"] = perf_counter() - started
    assert result.executable and result.evaluation.independent_audit and result.evaluation.collision_certificate
    totals["first_certified"] = result.stages["first_certified_s"]
    totals["certified_common_jerk_clock"] = result.stages["reachability_clock_s"]
    return {"stages_wall_s": dict(totals), "call_counts": dict(counts),
            "model_sha256": result.plan.model_sha256(), "reference_s": result.plan.duration_s}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--warmups", type=int, default=5)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.samples < 1 or args.warmups < 0:
        parser.error("positive sample count and nonnegative warm-up count required")
    matrix = ROOT / "ros2_ws/src/fwis_multi_planner_cpp/benchmark/compare_scene_matrix.py"
    make_request = runpy.run_path(str(matrix))["make_request"]
    experiments = yaml.safe_load((ROOT / "ros2_ws/src/fwis_bringup/config/experiments.yaml").read_text())
    report = {"samples": args.samples, "warmups": args.warmups,
              "notes": "Inclusive nested scopes. Collision includes conservative sample geometry and SAT. "
                       "Each reported stage has a nonzero call in at least one trial. Values cannot be added.",
              "cases": {}}
    for name in ("short_turn", "full_route"):
        base = make_request(SCENE, name, experiments)
        timing = replace(base.successor_config.timing_limits, v_max=.50, a_max=.36)
        request = replace(base, reuse_terminal_certificate=True, reachability_retime=True,
                          successor_config=replace(base.successor_config, timing_limits=timing),
                          search_config=replace(base.search_config, native_frontier=True,
                                                convex_corridor=name == "full_route"))
        rows = [profile(request) for _ in range(args.samples + args.warmups)][args.warmups:]
        assert len({row["model_sha256"] for row in rows}) == 1
        stages = sorted(set().union(*(row["stages_wall_s"] for row in rows)))
        summary = {}
        for key in stages:
            values = [1000 * row["stages_wall_s"].get(key, 0.0) for row in rows]
            summary[key] = {"p50_ms": float(np.percentile(values, 50)), "max_ms": max(values)}
            if len(values) >= 30:
                summary[key]["p95_ms"] = float(np.percentile(values, 95))
            if len(values) >= 100:
                summary[key]["p99_ms"] = float(np.percentile(values, 99))
        report["cases"][name] = {"summary": summary, "trials": rows}
        print(name, summary, flush=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
