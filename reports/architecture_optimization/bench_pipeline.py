#!/usr/bin/env python3
"""Paired Uniform V0.2 replay using the same archived scenario and install."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
from resource import RUSAGE_SELF, getrusage
from time import perf_counter, process_time
import sys

from fwis_backends.multi_4wis import uniform_pipeline
from fwis_backends.multi_4wis.uniform_reachability import compare_uniform_timing

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "architecture_baseline"))
from benchmark_multi import replay_request, stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--warmups", type=int, default=5)
    parser.add_argument("--repeat", type=int, default=30)
    parser.add_argument("--corridor", action="store_true")
    args = parser.parse_args()
    if args.repeat < 1 or args.warmups < 0:
        parser.error("repeat must be positive and warmups nonnegative")
    digest, original = replay_request(args.run, False, False)
    base = replace(original, search_config=replace(original.search_config, continuous_a_refinement=False))
    configs = {"legacy_sync": original, "base_no_cache": base,
               "base_cert_cached": replace(base, reuse_terminal_certificate=True)}
    if args.corridor:
        configs = {"raster": base, "convex_corridor": replace(
            base, search_config=replace(base.search_config, convex_corridor=True))}
    trials = {name: [] for name in configs}
    for index in range(args.repeat + args.warmups):
        order = list(configs)
        if index % 2:
            order.reverse()
        for name in order:
            wall, cpu = perf_counter(), process_time()
            answer = uniform_pipeline.plan_uniform(configs[name])
            seconds, cpu_seconds = perf_counter() - wall, process_time() - cpu
            model = answer.plan if answer.executable else None
            timing = compare_uniform_timing(model) if model and index == args.warmups else ()
            realized = answer.search
            trials[name].append({
                "warmup": index < args.warmups, "wall_s": seconds, "cpu_s": cpu_seconds,
                "peak_rss_kib_so_far": getrusage(RUSAGE_SELF).ru_maxrss,
                "executable": answer.executable, "reason": answer.reason,
                "model_sha256": model.model_sha256() if model else None,
                "duration_s": model.duration_s if model else None,
                "edge_modes": [edge.mode for edge in model.edges] if model else [],
                "search_s": realized.wall_time_s if realized else None,
                "search_expansions": realized.expansions if realized else None,
                "corridor_guide": realized.corridor_guide if realized else None,
                "stages": answer.stages,
                "research_retiming_s": sum(phase.duration_s for phase in timing) if timing else None,
            })
        print(f"pair {index + 1} of {args.repeat + args.warmups}", flush=True)
    report = {"reference_model": digest, "run_id": args.run.name, "planner": uniform_pipeline.__file__,
              "statistics": {name: {field: stats([row[field] for row in rows if not row["warmup"]])
                                    for field in ("wall_s", "cpu_s")} for name, rows in trials.items()},
              "samples": trials}
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
