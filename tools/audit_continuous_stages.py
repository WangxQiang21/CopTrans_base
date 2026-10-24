#!/usr/bin/env python3
"""Offline ablation of continuous conversion, smoothing, fitting, and repair.

This consumes a frozen ROS probe run and writes only an audit JSON. It does not
start ROS or modify planner configuration.
"""
from __future__ import annotations

from dataclasses import fields
import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import Action, Grid, Pose, wrap
from fwis_v3_optimizer.quality import QualityProfile, TrajectoryEvaluator
from fwis_v3_optimizer.retiming import MotionLimits
from fwis_v3_optimizer.continuous_geometry import (
    fit_continuous_endpoint, fit_continuous_suffix, plan_from_actions,
    replay_continuous, sample_block, smooth_control_candidate,
)


def load_run(run_dir: Path):
    probe = json.loads((run_dir / "probe_result.json").read_text())
    case = json.loads((run_dir / "case.json").read_text())
    params = yaml.safe_load((run_dir / "params.yaml").read_text())
    map_dir = run_dir / "runtime_map"
    meta = yaml.safe_load((map_dir / "map.yaml").read_text())
    image_path = map_dir / meta["image"]
    image = np.flipud(np.asarray(Image.open(image_path).convert("L")))
    if meta.get("negate", 0):
        image = 255 - image
    occupied = image < 255 * (1 - meta.get("free_thresh", .196))
    vehicle = params["vehicle"]
    margin = float(params["collision"]["margin"])
    # The frozen run directory contains the already-scaled runtime map. Its
    # resolution and center-preserving origin are final; applying map_scale a
    # second time would create a false collision at the start pose.
    grid = Grid(occupied, float(np.float32(meta["resolution"])),
                float(meta["origin"][0]), float(meta["origin"][1]),
                vehicle["length"], vehicle["width"], margin)
    valid = {f.name for f in fields(MotionLimits)}
    limit_params = dict(params["quality"]["limits"])
    limit_params.update({k: v for k, v in vehicle.items() if k in valid})
    limits = MotionLimits(**limit_params)
    profile = QualityProfile(**params["quality"]["cost"])
    start, goal = Pose(*case["start"]), Pose(*case["goal"])
    evaluator = TrajectoryEvaluator(
        grid, start, goal, limits, vehicle["length"], vehicle["width"],
        float(case["xy_tol"]), float(case["yaw_tol"]), profile,
        allow_reverse=params["planning"]["direction_policy"] == "bidirectional",
        allow_shunting=bool(params["planning"].get("allow_shunting", False)))
    actions = [Action(**row) for row in probe["seed_actions"]]
    return probe, case, params, evaluator, actions


def terminal_error(plan, spacing=.005):
    end = replay_continuous(plan, spacing)[-1][1]
    return {"xy_m": math.hypot(end.x - plan.goal.x, end.y - plan.goal.y),
            "yaw_rad": abs(wrap(end.th - plan.goal.th)),
            "pose": [end.x, end.y, end.th]}


def sampled_path(plan, spacing=.02):
    points = [(0., plan.start.x, plan.start.y, plan.start.th)]
    pose = plan.start
    total = 0.
    for block in plan.blocks:
        rows = sample_block(block, pose, spacing)
        for station, p, *_ in rows[1:]:
            points.append((total + station, p.x, p.y, p.th))
        pose = rows[-1][1]
        total += block.length
    # Spins have zero translation arc length. Keep the final pose at each
    # station so the matching convention is explicit and deterministic.
    by_s = {}
    for row in points:
        by_s[round(row[0], 10)] = row
    return np.asarray([by_s[key] for key in sorted(by_s)], dtype=float)


def displacement(reference, candidate):
    a, b = sampled_path(reference), sampled_path(candidate)
    common = min(float(a[-1, 0]), float(b[-1, 0]))
    stations = np.unique(np.r_[a[a[:, 0] <= common, 0],
                               b[b[:, 0] <= common, 0],
                               np.linspace(0., common, max(2, int(common/.02)+1))])
    def interp(rows, col):
        return np.interp(stations, rows[:, 0], rows[:, col])
    ax, ay = interp(a, 1), interp(a, 2)
    bx, by = interp(b, 1), interp(b, 2)
    xy = np.hypot(ax - bx, ay - by)
    yaw = np.asarray([abs(wrap(x - y)) for x, y in zip(interp(a, 3), interp(b, 3))])
    return {"matching": "equal cumulative translation arc length over shared domain",
            "shared_domain_m": common,
            "reference_length_m": float(a[-1, 0]),
            "candidate_length_m": float(b[-1, 0]),
            "max_centerline_displacement_m": float(np.max(xy)),
            "rms_centerline_displacement_m": float(np.sqrt(np.mean(xy * xy))),
            "max_wrapped_yaw_displacement_rad": float(np.max(yaw))}


def plan_metrics(plan, evaluator, reference):
    collision = evaluator.continuous_collision(plan)
    pose = plan.start
    clearance = float("inf")
    for block in plan.blocks:
        rows = sample_block(block, pose, .01)
        for _, sample_pose, *_ in rows:
            clearance = min(clearance, evaluator.distance.distance(sample_pose))
        pose = rows[-1][1]
    max_dcontrol_ds = 0.
    max_dcontrol_by_mode = {"mode_0_curvature": 0., "mode_1_oblique_angle": 0.}
    for block in plan.blocks:
        if block.mode == 2:
            continue
        stations = np.unique(np.r_[
            np.linspace(0., block.length, max(3, int(block.length / .01) + 1)),
            [k.s for k in block.knots]])
        derivs = [abs(block.control(s)[1]) for s in stations]
        block_max = max(derivs, default=0.)
        max_dcontrol_ds = max(max_dcontrol_ds, block_max)
        key = "mode_0_curvature" if block.mode == 0 else "mode_1_oblique_angle"
        max_dcontrol_by_mode[key] = max(max_dcontrol_by_mode[key], block_max)
    return {"terminal": terminal_error(plan),
            "minimum_body_clearance_m": clearance if math.isfinite(clearance) else None,
            "first_swept_collision_interval": collision,
            "max_control_derivative_per_m": max_dcontrol_ds,
            "max_control_derivative_by_mode": max_dcontrol_by_mode,
            "geometry_displacement_from_action_replay": displacement(reference, plan),
            "block_count": len(plan.blocks),
            "mode_direction_word": [[b.mode, b.direction] for b in plan.blocks]}


def compact_fit(report):
    return {key: report.get(key) for key in (
        "success", "solver_success", "status", "solver_message", "iterations",
        "terminal_error", "fixed_prefix_length_m", "projection", "tail_length_m",
        "control_spacing_m", "objective") if key in report}


def audit_fit(name, plan, evaluator, reference, *, suffix=False, tail=2.):
    fit_start = time.perf_counter()
    try:
        if suffix:
            fitted, report = fit_continuous_suffix(
                plan, tail_length_m=tail,
                r_min_ack=evaluator.limits.r_min_ack, max_iterations=40,
                terminal_xy_tol=1e-7, terminal_yaw_tol=1e-7,
                deadline=time.perf_counter() + 8.)
        else:
            fitted, report = fit_continuous_endpoint(
                plan, r_min_ack=evaluator.limits.r_min_ack, max_iterations=40,
                terminal_xy_tol=1e-7, terminal_yaw_tol=1e-7,
                deadline=time.perf_counter() + 8.)
        return {"stage": name, "fit": compact_fit(report),
                "metrics": plan_metrics(fitted, evaluator, reference),
                "wall_s": time.perf_counter() - fit_start}
    except (ValueError, RuntimeError, ArithmeticError) as exc:
        return {"stage": name, "fit": {"success": False, "reason": str(exc)},
                "wall_s": time.perf_counter() - fit_start}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    probe, case, params, evaluator, actions = load_run(args.run)
    action_audit = evaluator.evaluate(actions, timed=None, allow_reverse=evaluator.allow_reverse)
    raw = plan_from_actions(evaluator.start, evaluator.goal, actions)
    raw_metrics = plan_metrics(raw, evaluator, raw)
    ablations = []
    for label, bandwidth, spacing in (
            ("runtime_primary", .4, .4),
            ("runtime_secondary", .3, .2)):
        smooth = smooth_control_candidate(raw, bandwidth, spacing)
        smooth_metrics = plan_metrics(smooth, evaluator, raw)
        endpoint = audit_fit(label + "_endpoint_fit", smooth, evaluator, raw)
        # Record both bounded windows for this ablation even if the prefix is
        # already unsafe. In that case they are explicitly diagnostic only;
        # runtime correctly refuses to publish a repair with an unsafe prefix.
        suffix_rows = []
        for tail in (2., 4.):
            # Match mode_master: suffix repair starts from the unprojected
            # continuous seed, preserving its prefix, not from a colliding
            # global projection or from the smoothed fitting initializer.
            suffix_rows.append(audit_fit(
                f"{label}_suffix_from_unprojected_{tail:g}m", raw,
                evaluator, raw, suffix=True, tail=tail))
        ablations.append({"initializer": label,
                          "smoothing": {"bandwidth_m": bandwidth,
                                        "station_spacing_m": spacing},
                          "metrics": smooth_metrics,
                          "endpoint_fit": endpoint,
                          "suffix_seed": "unprojected plan_from_actions",
                          "bounded_suffix_repairs": suffix_rows})
    result = {
        "schema": "continuous_stage_ablation_v1",
        "run": str(args.run.resolve()),
        "case": case.get("case"),
        "map_tree_sha256": case.get("map_tree_sha256"),
        "source_run_id": json.loads((args.run / "run.json").read_text()).get("run_id"),
        "frozen_policy": {"allow_reverse": evaluator.allow_reverse,
                          "allow_shunting": evaluator.allow_shunting,
                          "xy_tolerance_m": evaluator.xy_tol,
                          "yaw_tolerance_rad": evaluator.yaw_tol,
                          "sweep_spacing_m": evaluator.profile.sweep_spacing},
        "exact_action_replay": {
            "action_count": len(actions),
            "status": action_audit.get("status"),
            "geometry_status": action_audit.get("geometry_status"),
            "reason": action_audit.get("reason"),
            "terminal": action_audit.get("terminal"),
            "minimum_body_clearance_m": action_audit.get("metrics", {}).get("min_clearance_m"),
        },
        "plan_from_actions": raw_metrics,
        "smoothing_endpoint_and_suffix_ablation": ablations,
        "unprojected_seed_sweep_safe": raw_metrics["first_swept_collision_interval"] is None,
        "provenance_note": (
            "Offline fixed-seed diagnostic only; no ROS session started. "
            "Suffix rows are bounded diagnostic fits from each smoothed plan. "
            "If unprojected_seed_sweep_safe is false, they are not eligible "
            "runtime repairs: the production path requires the prefix to be "
            "sweep-safe before attempting local suffix repair."),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"output": str(args.output),
                      "action_status": action_audit.get("status"),
                      "raw_collision": raw_metrics["first_swept_collision_interval"] is not None,
                      "initializers": len(ablations)}, indent=2))


if __name__ == "__main__":
    main()
