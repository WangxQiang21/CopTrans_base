#!/usr/bin/env python3
"""Measure CDR conversion of existing certified formation envelopes without DDS I/O."""

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import platform
from time import perf_counter_ns

from fwis_v3_msgs.msg import FormationPlaybackPlan
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.utilities import get_rmw_implementation_identifier

from fwis_backends.multi_4wis.formation_playback import (
    fields_from_timed_message, verify_uniform_playback_plan,
)


def percentile(samples, fraction):
    index = max(0, math.ceil(len(samples) * fraction) - 1)
    return sorted(samples)[index] / 1e6


def summarize(samples):
    result = {"p50": percentile(samples, .5), "max": percentile(samples, 1.)}
    if len(samples) >= 30:
        result["p95"] = percentile(samples, .95)
    if len(samples) >= 100:
        result["p99"] = percentile(samples, .99)
    return result


def measure(path, warmups, reps):
    raw = path.read_bytes()
    envelope = deserialize_message(raw, FormationPlaybackPlan)
    samples = fields_from_timed_message(envelope.trajectory)
    _, _, verified = verify_uniform_playback_plan(
        plan_id=envelope.plan_id, formation_id=envelope.formation_id, backend=envelope.backend,
        semantic_version=envelope.semantic_version, model_version=envelope.model_version,
        model_sha256=envelope.model_sha256, model_json=envelope.model_json,
        certification=envelope.certification, trajectory_fields=samples)
    work = {"serialize_message": lambda: serialize_message(envelope),
            "deserialize_message": lambda: deserialize_message(raw, FormationPlaybackPlan)}
    for _ in range(warmups):
        for run in work.values():
            run()
    timings = {name: [] for name in work}
    for trial in range(reps):
        order = tuple(work) if trial % 2 == 0 else tuple(reversed(work))
        for name in order:
            before = perf_counter_ns()
            result = work[name]()
            timings[name].append(perf_counter_ns() - before)
            del result  # Free the returned Python object outside the timed interval.
    canonical = serialize_message(envelope)
    assert deserialize_message(canonical, FormationPlaybackPlan) == envelope
    return {
        "input": str(path), "bytes": len(raw), "input_sha256": sha256(raw).hexdigest(),
        "roundtrip_bytes_match_input": raw == canonical, "semantic_roundtrip_equal": True,
        "model_sha256": verified, "model_version": envelope.model_version,
        "samples": len(envelope.trajectory.times_from_start),
        "operation_ms": {name: summarize(values) for name, values in timings.items()},
        "raw_samples_ns": timings,
    }


def main():
    runner = argparse.ArgumentParser(description=__doc__)
    runner.add_argument("cdr", type=Path, nargs="+")
    runner.add_argument("--warmups", type=int, default=5)
    runner.add_argument("--reps", type=int, default=100)
    runner.add_argument("--output", type=Path, required=True)
    args = runner.parse_args()
    if args.warmups < 0 or args.reps < 1:
        runner.error("warmups must be nonnegative and reps positive")
    output = {
        "scope": "rclpy CDR conversion only, no DDS, filesystem, or verifier in timed interval",
        "clock": "perf_counter_ns monotonic wall; decode includes Python message creation",
        "platform": platform.platform(), "python": platform.python_version(),
        "rmw": get_rmw_implementation_identifier(),
        "warmups": args.warmups, "repetitions_per_operation": args.reps,
        "cases": [measure(path, args.warmups, args.reps) for path in args.cdr],
    }
    args.output.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n")
    for row in output["cases"]:
        print(row["bytes"], row["samples"], row["model_sha256"], row["operation_ms"])


if __name__ == "__main__":
    main()
