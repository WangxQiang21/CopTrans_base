#!/usr/bin/env python3
"""Replay a saved probe seed offline with its exact map and simulation limits."""
from dataclasses import fields
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import Action, Grid, Pose, footprint_corners
from fwis_v3_optimizer.quality import QualityProfile, TrajectoryEvaluator
from fwis_v3_optimizer.retiming import MotionLimits
from fwis_v3_optimizer.continuous_geometry import sample_block


def load_run(run_dir):
    run_dir = Path(run_dir)
    probe = json.loads((run_dir/"probe_result.json").read_text())
    case = json.loads((run_dir/"case.json").read_text())
    params = yaml.safe_load((run_dir/"params.yaml").read_text())
    map_dir = ROOT/case["map_dir"]
    meta = yaml.safe_load((map_dir/"map.yaml").read_text())
    image = np.flipud(np.asarray(Image.open(map_dir/meta["image"]).convert("L")))
    if meta.get("negate", 0):
        image = 255-image
    occupied = image < 255*(1-meta.get("free_thresh", .196))
    scale = case["map_scale"]
    vehicle = params["vehicle"]
    # Runtime scaling preserves the map center, not the world origin.
    height,width = occupied.shape
    ox = meta["origin"][0]+width*meta["resolution"]*(1-scale)/2
    oy = meta["origin"][1]+height*meta["resolution"]*(1-scale)/2
    grid = Grid(occupied, float(np.float32(meta["resolution"]*scale)), ox, oy,
                vehicle["length"], vehicle["width"], params["collision"]["margin"])
    valid = {f.name for f in fields(MotionLimits)}
    limit_params = dict(params["quality"]["limits"])
    limit_params.update({k:v for k,v in vehicle.items() if k in valid})
    limits = MotionLimits(**limit_params)
    profile = QualityProfile(**params["quality"]["cost"])
    start, goal = Pose(*case["start"]), Pose(*case["goal"])
    evaluator = TrajectoryEvaluator(grid, start, goal, limits,
        vehicle["length"], vehicle["width"], .02, math.radians(1), profile,
        allow_reverse=params["planning"]["direction_policy"] == "bidirectional")
    actions = [Action(**a) for a in probe["seed_actions"]]
    return evaluator, actions


def collision_summary(plan, evaluator, spacing=.005):
    pose, global_s = plan.start, 0.
    collisions = []
    for bi, block in enumerate(plan.blocks):
        rows = sample_block(block, pose, spacing)
        for row in rows:
            if not evaluator.sweep_grid.pose_clear(row[1]):
                collisions.append({"block": bi, "local_s": row[0],
                    "global_s": global_s+row[0], "pose": [row[1].x,row[1].y,row[1].th],
                    "control": row[2], "mode": block.mode,
                    "safe_clearance_m": evaluator.safe_distance.distance(row[1]),
                    "footprint": footprint_corners(row[1],2*evaluator.sweep_grid.half_l,
                                                    2*evaluator.sweep_grid.half_w)})
        pose = rows[-1][1]
        global_s += block.length
    return {"collision_samples": len(collisions),
            "first_collision": collisions[0] if collisions else None,
            "last_collision": collisions[-1] if collisions else None}


if __name__ == "__main__":
    from fwis_v3_optimizer.continuous_geometry import (
        plan_from_actions,smooth_control_candidate,fit_continuous_endpoint,
        fit_continuous_suffix)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir",type=Path)
    parser.add_argument("--output",type=Path)
    args = parser.parse_args()
    evaluator,actions = load_run(args.run_dir)
    seed = smooth_control_candidate(plan_from_actions(
        evaluator.start,evaluator.goal,actions),.4,.4)
    global_fit,global_report = fit_continuous_endpoint(seed,max_iterations=40,
        terminal_xy_tol=1e-7,terminal_yaw_tol=1e-7)
    suffix,suffix_report = fit_continuous_suffix(seed,2.,max_iterations=40,
        terminal_xy_tol=1e-7,terminal_yaw_tol=1e-7)
    result = {"unprojected_collision":evaluator.continuous_collision(seed),
              "global_projection":{"fit":global_report,
                                   "collision":evaluator.continuous_collision(global_fit)},
              "suffix_projection":{"fit":suffix_report,
                                   "collision":evaluator.continuous_collision(suffix)}}
    encoded = json.dumps(result,indent=2)+"\n"
    if args.output:
        args.output.write_text(encoded)
    print(encoded,end="")
