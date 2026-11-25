#!/usr/bin/env python3
"""Compare audited common clocks with the original clock on fixed pure-A paths."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_backends.multi_4wis.formation_collision import CargoFootprint, VehicleShape
from fwis_backends.multi_4wis.formation_retiming import FormationTimingLimits
from fwis_backends.multi_4wis.uniform_motion import (
    a_arc_edge, a_arc_required_headings, a_straight_edge,
)
from fwis_backends.multi_4wis.uniform_pipeline import UniformPlanningRequest, _build_and_audit
from fwis_backends.multi_4wis.uniform_reachability import retime_certified_uniform
from fwis_backends.multi_4wis.uniform_search import SuccessorConfig
from fwis_backends.multi_4wis.uniform_timing import timing_motion_limits


def fixtures():
    grid = Grid(np.zeros((180, 180), dtype=bool), .1, 0., 0.)
    team = FormationModel([VehicleOffset(1, 1., 0.), VehicleOffset(2, 0., 1.),
                           VehicleOffset(3, 0., -1.)])
    start = FormationPose(5., 6., 0.)
    shapes = dict.fromkeys(team.vehicle_ids, VehicleShape(.738, .500))
    directions = dict.fromkeys(team.vehicle_ids, 1)
    limits = FormationTimingLimits(v_max=.50, a_max=.36, yaw_rate_max=.40,
                                   yaw_accel_max=.45, r_min_ackermann=.90,
                                   mode_switch_duration=.70)
    motion = timing_motion_limits(limits)
    for name, straight, angle, pivot in (
        ("straight_1m", 1.0, 0., None), ("straight_4m", 4.0, 0., None),
        ("straight_8m", 8.0, 0., None), ("left_shared_arc", 0., .85, (0., 2.2)),
        ("right_shared_arc", 0., -.70, (.5, -3.5)),
    ):
        if pivot is None:
            headings = dict.fromkeys(team.vehicle_ids, start.psi)
            edge = a_straight_edge(team, start, headings, (straight, 0.), directions, motion)
        else:
            headings = a_arc_required_headings(team, start, angle, pivot, directions, motion)
            edge = a_arc_edge(team, start, headings, angle, pivot, directions, motion)
        yield name, edge, UniformPlanningRequest(
            grid, team, shapes, start, headings, edge.end_qf,
            goal_headings=dict(edge.end_headings), successor_config=SuccessorConfig(timing_limits=limits),
            cargo_footprint=CargoFootprint(1.95, 2.70, .5, 0.))


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--output", required=True, type=Path)
    cli.add_argument("--samples", default=30, type=int)
    cli.add_argument("--warmups", default=5, type=int)
    args = cli.parse_args()
    if args.samples < 1 or args.warmups < 0:
        cli.error("positive sample count and nonnegative warm-up count required")

    rows = []
    for name, edge, request in fixtures():
        plan, _, evaluation = _build_and_audit((edge,), request)
        assert evaluation.feasible, (name, evaluation.detail)
        times = []
        for trial in range(args.samples + args.warmups):
            started = perf_counter()
            result = retime_certified_uniform(plan, evaluation)
            cost_ms = 1000 * (perf_counter() - started)
            assert result is not None, name
            retimed, projection, proof = result
            assert proof.collision_certificate and proof.metrics["dynamic_continuous_certificate"]
            assert projection.model_sha256 == retimed.model_sha256()
            assert retimed.baseline.edges == (edge,)
            if trial >= args.warmups:
                times.append(cost_ms)
        row = {"fixture": name, "primitive": edge.primitive,
               "original_duration_s": plan.duration_s, "common_jerk_duration_s": retimed.duration_s,
               "reduction_fraction": 1.0 - retimed.duration_s / plan.duration_s,
               "baseline_model_sha256": plan.model_sha256(), "model_sha256": retimed.model_sha256(),
               "dynamic_limit_ratio": proof.metrics["replacement_clock_max_dynamic_ratio"],
               "retiming_ms": times, "retiming_p50_ms": float(np.percentile(times, 50)),
               "retiming_max_ms": max(times)}
        if args.samples >= 30:
            row["retiming_p95_ms"] = float(np.percentile(times, 95))
        if args.samples >= 100:
            row["retiming_p99_ms"] = float(np.percentile(times, 99))
        rows.append(row)
        print(name, f"{plan.duration_s:.3f}s -> {retimed.duration_s:.3f}s", flush=True)
    report = {"limits": asdict(request.successor_config.timing_limits), "samples": args.samples,
              "warmups": args.warmups, "cases": rows,
              "median_reduction_fraction": float(np.median([row["reduction_fraction"] for row in rows]))}
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
