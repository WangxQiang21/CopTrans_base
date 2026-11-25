#!/usr/bin/env python3
"""Paired warm Python / native benchmark for constant-control A/O/S state advance."""

import argparse
import json
from pathlib import Path
import platform
import subprocess
import time

import numpy as np

from fwis_backends.single_continuous_mode_path.core import Action, Pose, advance
from fwis_planning_core_native import advance_batch


def setup_workload(count):
    rng = np.random.default_rng(8417)
    inputs = np.column_stack((
        rng.uniform(-15, 15, count), rng.uniform(-15, 15, count),
        rng.uniform(-12, 12, count), np.arange(count) % 3,
        rng.uniform(-3, 3, count), rng.uniform(-3.14, 3.14, count),
    ))
    python_inputs = [
        (Pose(x, y, yaw), Action(int(mode), amount, control))
        for x, y, yaw, mode, amount, control in inputs.tolist()
    ]

    def reference():
        result = [advance(pose, motion) for pose, motion in python_inputs]
        return np.array([(pose.x, pose.y, pose.th) for pose in result])

    def native():
        return advance_batch(inputs)

    return reference, native


def measure(methods, warmup, trials):
    batches = {name: [] for name in methods}
    for repeat in range(warmup + trials):
        names = sorted(methods, reverse=repeat % 2 == 1)
        for name in names:
            start = time.perf_counter_ns()
            cpu_start = time.process_time_ns()
            methods[name]()
            cpu_ns = time.process_time_ns() - cpu_start
            wall_ns = time.perf_counter_ns() - start
            if repeat >= warmup:
                batches[name].append({"wall_ms": wall_ns / 1e6, "cpu_ms": cpu_ns / 1e6})
    return batches


def summarize(batch):
    return {
        f"p{q}_wall_ms": float(np.percentile([sample["wall_ms"] for sample in batch], q))
        for q in (50, 95, 100)
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rows", type=int, default=20_000)
    args = parser.parse_args()
    reference, native = setup_workload(args.rows)
    assert np.array_equal(reference(), native())
    methods = {"python": reference, "cpp": native}
    trials = measure(methods, warmup=5, trials=30)
    governor = Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
    result = {
        "git_sha": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "dirty": bool(subprocess.check_output(["git", "status", "--porcelain"], text=True)),
        "os": platform.platform(),
        "cpu_model": next((line.split(":", 1)[-1].strip()
                           for line in Path("/proc/cpuinfo").read_text().splitlines()
                           if line.startswith("model name")), "unknown"),
        "cpu_governor": governor.read_text().strip() if governor.is_file() else None,
        "python": platform.python_version(),
        "numpy": np.__version__,
        "rows": args.rows,
        "same_output": True,
        "warmup": 5,
        "trials": trials,
        "summary": {name: summarize(rows) for name, rows in trials.items()},
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result["summary"], indent=2))


if __name__ == "__main__":
    main()
