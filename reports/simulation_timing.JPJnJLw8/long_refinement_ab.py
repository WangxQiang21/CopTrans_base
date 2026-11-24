"""Check how much the same short-turn diagnosis generalizes to long cases."""
import importlib.util
import json
import sys
from dataclasses import replace
from pathlib import Path
from statistics import median
from time import perf_counter

from ament_index_python.packages import get_package_share_directory
import yaml
from fwis_backends.multi_4wis.formation_retiming import FormationTimingLimits
from fwis_backends.multi_4wis import uniform_pipeline

path = Path(sys.argv[1])
spec = importlib.util.spec_from_file_location("formation_benchmark", path / "benchmark.py")
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)
share = Path(get_package_share_directory("fwis_bringup"))
params = yaml.safe_load((share / "config/test_defaults.yaml").read_text())
limits = FormationTimingLimits(**params["profiles"]["multi"]["timing_limits"])
summary = {}
for name in bench.CASES:
    original = bench.make_request(name, limits)
    base = replace(original, search_config=replace(original.search_config,
                                                   continuous_a_refinement=False))
    trials = {"default": [], "without_c2": [], "without_c2_reuse": []}
    prev_builder = uniform_pipeline._build_and_audit
    for repeat in range(3):
        order = ("default", "without_c2", "without_c2_reuse")
        for variant in (order if repeat % 2 == 0 else reversed(order)):
            request = original if variant == "default" else base
            cache = {}
            reused = []

            def cached_builder(edges, problem, zero_motion=False):
                edges = tuple(edges)
                key = (id(problem), tuple(map(id, edges)), zero_motion)
                if key in cache:
                    reused.append(key)
                    return cache[key][1]
                value = prev_builder(edges, problem, zero_motion)
                if value[2].feasible:
                    cache[key] = edges, value
                return value

            if variant == "without_c2_reuse":
                uniform_pipeline._build_and_audit = cached_builder
            try:
                started = perf_counter()
                result = uniform_pipeline.plan_uniform(request)
                elapsed = perf_counter() - started
            finally:
                uniform_pipeline._build_and_audit = prev_builder
            assert result.executable
            trials[variant].append({"wall_s": elapsed, "search_s": result.search.wall_time_s,
                  "duration_s": result.plan.duration_s, "model_sha256": result.plan.model_sha256(),
                  "edges": len(result.plan.edges), "expansions": result.search.expansions,
                  "refinement_accepted": "refined" in result.detail, "reuse_count": len(reused)})
            print(name, repeat + 1, variant, f"{elapsed:.3f} s", flush=True)
    medians = {key: median(row["wall_s"] for row in values) for key, values in trials.items()}
    assert {r["model_sha256"] for r in trials["without_c2"]} == {
               r["model_sha256"] for r in trials["without_c2_reuse"]}
    summary[name] = {"medians_s": medians, "trials": trials}
    Path(__file__).with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n")
