#!/usr/bin/env python3
"""Differential checks for the planning collision model and STC body cover."""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE/"src"))
from spatial_model import Pose,RasterWorld,Vehicle
from obb_validator import OBBRasterValidator
from safe_corridor import feature_disc_cover


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--map",default=str(HERE.parents[2] / "maps/S09_underground_garage/map.yaml"))
    ap.add_argument("--pose-samples",type=int,default=5000)
    ap.add_argument("--sweep-samples",type=int,default=1000)
    ap.add_argument("--seed",type=int,default=20260930)
    ap.add_argument("--output",default=str(HERE/"results/summary/geometry_validation.json"))
    args=ap.parse_args();rng=np.random.default_rng(args.seed);vehicle=Vehicle()
    planning=RasterWorld(args.map,vehicle)
    reference=OBBRasterValidator(args.map,vehicle,safety_margin=.04)
    pose_counts={"both_clear":0,"both_collision":0,"planning_false_clear":0,"planning_conservative_reject":0}
    for _ in range(args.pose_samples):
        p=Pose(rng.uniform(reference.ox,reference.ox+reference.width*reference.res),
               rng.uniform(reference.oy,reference.oy+reference.height*reference.res),
               rng.uniform(-math.pi,math.pi))
        a,b=planning.pose_clear(p),reference.pose_clear(p)
        key="both_clear" if a and b else "both_collision" if not a and not b else "planning_false_clear" if a else "planning_conservative_reject"
        pose_counts[key]+=1
    sweep_counts={"both_clear":0,"both_collision":0,"planning_false_clear":0,"planning_conservative_reject":0}
    for _ in range(args.sweep_samples):
        a=Pose(rng.uniform(reference.ox,reference.ox+reference.width*reference.res),
               rng.uniform(reference.oy,reference.oy+reference.height*reference.res),rng.uniform(-math.pi,math.pi))
        b=Pose(a.x+rng.uniform(-.30,.30),a.y+rng.uniform(-.30,.30),a.yaw+rng.uniform(-math.radians(30),math.radians(30)))
        x,y=planning.swept_clear(a,b),reference.swept_clear(a,b)
        key="both_clear" if x and y else "both_collision" if not x and not y else "planning_false_clear" if x else "planning_conservative_reject"
        sweep_counts[key]+=1
    cover={}
    for margin in (0.0,.04):
        features=feature_disc_cover(vehicle,8,4,margin)
        length,width=vehicle.length+margin,vehicle.width+margin
        uncovered=0;max_nearest=0.0
        for _ in range(20000):
            x=rng.uniform(-length/2,length/2);y=rng.uniform(-width/2,width/2)
            nearest=min(math.hypot(x-f.longitudinal_offset,y-f.lateral_offset)-f.radius for f in features)
            max_nearest=max(max_nearest,nearest)
            uncovered += nearest>1e-12
        cover[str(margin)]={"samples":20000,"uncovered":uncovered,"max_signed_excess_m":max_nearest,
                            "feature_count":len(features),"disc_radius_m":features[0].radius}
    result={"seed":args.seed,"pose_samples":args.pose_samples,"sweep_samples":args.sweep_samples,
            "planning_checker":"inflated raster plus sampled rectangle",
            "reference_checker":"independent OBB versus occupied-cell SAT, 4 cm overall body margin",
            "pose_comparison":pose_counts,"linear_sweep_comparison":sweep_counts,"feature_disc_cover":cover}
    path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    if pose_counts["planning_false_clear"] or sweep_counts["planning_false_clear"] or any(v["uncovered"] for v in cover.values()):
        raise SystemExit(2)


if __name__=="__main__":main()
