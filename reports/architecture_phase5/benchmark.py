#!/usr/bin/env python3
"""Measure the single lattice and fixed-path research timing against saved inputs."""

import argparse
import json
from pathlib import Path
from statistics import median
from time import perf_counter

import yaml

from fwis_backends.single_continuous_mode_path.ackermann_reachability import (
    constant_ackermann_jerk_profile, pure_ackermann_profile,
)
from fwis_backends.single_continuous_mode_path.continuous_geometry import plan_from_actions
from fwis_backends.single_continuous_mode_path.continuous_retiming import retime_continuous
from fwis_backends.single_continuous_mode_path.core import Action, Grid, Pose, advance
from fwis_backends.single_continuous_mode_path.retiming import MotionLimits
from fwis_backends.single_continuous_mode_path.state_lattice import LatticeConfig, search_lattice
from ranger_mini_gz.scene_generator import read_map


LIMITS = MotionLimits(v_max=.16, a_max=.22, jerk_max=.8, omega_max=.4, alpha_max=.45,
                      angular_jerk_max=1., r_min_ack=.9, mode_switch_duration=.7)


def lattice_replay(run):
    config = yaml.safe_load((run / "resolved_config.yaml").read_text())
    result = yaml.safe_load((run / "result.yaml").read_text())
    metadata, occupied = read_map(run / "scene/map.yaml")
    grid = Grid(occupied != 0, metadata["resolution"], *metadata["origin"][:2])
    start, goal = Pose(*config["start"]), Pose(*config["goal"])
    planned = search_lattice(start, goal, grid, LIMITS, LatticeConfig(time_budget_s=2.0))
    candidate = planned.incumbent
    candidate_quality = candidate.quality if candidate is not None else None
    clearance = (
        candidate_quality["metrics"].get("min_safe_clearance_m") if candidate_quality else None)
    return {
        "run_id": run.name,
        "experiment": config["experiment"],
        "old_live_runner_planning_wall_s": result.get("planner_wall_s"),
        "old_live_certificate": result.get("status"),
        "old_reference_duration_s": result.get("plan_duration_s"),
        "new_offline_reason": planned.reason,
        "new_offline_mode_numbers": [action.mode for action in candidate.actions] if candidate else [],
        "new_offline_first_certified_s": planned.first_certified_s,
        "new_offline_planning_wall_s": planned.wall_s,
        "new_offline_certificate": candidate_quality["status"] if candidate_quality else None,
        "new_offline_reference_duration_s": candidate.timed.duration if candidate else None,
        "new_offline_terminal": candidate_quality["terminal"] if candidate_quality else None,
        "new_offline_min_safe_clearance_m": clearance,
        "new_offline_expansions": planned.expanded,
    }


def timing_fixture(length, curvature, repeat):
    a = Action(0, length, curvature)
    start = Pose(2.0, 2.0, 0.0)
    geometry = plan_from_actions(start, advance(start, a), (a,))
    old, scalar, jerk = [], [], []
    for index in range(repeat + 5):
        candidates = (
            (old, lambda: retime_continuous(geometry, LIMITS)),
            (scalar, lambda: pure_ackermann_profile(geometry.blocks[0], LIMITS)),
            (jerk, lambda: constant_ackermann_jerk_profile(a, LIMITS)),
        )
        for destination, solve in (candidates if index % 2 == 0 else reversed(candidates)):
            begin = perf_counter()
            solve()
            elapsed = perf_counter() - begin
            if index >= 5:
                destination.append(elapsed)
    baseline = retime_continuous(geometry, LIMITS).moving_seconds
    relaxed = pure_ackermann_profile(geometry.blocks[0], LIMITS).duration_s
    bound = constant_ackermann_jerk_profile(a, LIMITS).duration_s
    return {
        "length_m": length,
        "curvature_inv_m": curvature,
        "legacy_continuous_motion_s": baseline,
        "scalar_reachability_motion_s_finite_jerk_not_enforced": relaxed,
        "constant_a_jerk_limited_motion_s": bound,
        "constant_a_jerk_limited_relative_gain": (baseline - bound) / baseline,
        "wall_samples_s": {"legacy": old, "scalar_relaxation": scalar, "constant_a_jerk_limited": jerk},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", action="append", type=Path, default=[])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeat", type=int, default=30)
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error("repeat must be positive")
    cases = [timing_fixture(length, curvature, args.repeat) for length, curvature in (
        (.02, 0.), (.08, .5), (1., 1.), (3., -.8), (-2., .3))]
    output = {
        "timing_warmups": 5,
        "timing_repeats": args.repeat,
        "fixture_limits": dict(vars(LIMITS)),
        "archived_live_vs_offline": [lattice_replay(run) for run in args.archive],
        "fixed_path_fixtures": cases,
        "median_constant_a_jerk_limited_relative_gain": median(
            case["constant_a_jerk_limited_relative_gain"] for case in cases),
    }
    args.output.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
