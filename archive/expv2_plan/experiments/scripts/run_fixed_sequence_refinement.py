#!/usr/bin/env python3
"""Refine S09 coarse paths continuously while holding their mode order fixed."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import sys
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE/"src"))
from spatial_model import Pose,Vehicle,propagate,wrap
from obb_validator import OBBRasterValidator
from safe_corridor import SafeCorridorBuilder,feature_disc_cover
from fixed_mode_optimizer import expand_actions,solve_fixed_sequence

PRESETS={
"BOT_LEFT":(.740,.500,-1.5708),"BOT_MID":(5.860,.500,-1.5708),
"BOT_RIGHT":(10.980,.500,-1.5708),"MID_A":(7.250,4.000,1.5708),
"MID_B":(2.700,4.900,-1.5708),"TOP_LEFT":(.814,9.214,-.7854),
"TOP_MID":(6.014,9.214,-.7854)}
MODE_COLOR={"A":"#1976d2","P":"#ef6c00","L":"#7b1fa2","S":"#d81b60"}


def dense_replay(start,actions,vehicle,step=.02,angle_step=math.radians(2)):
    poses=[start]; current=start
    for mode,amount,control in actions:
        count=max(1,math.ceil(abs(amount)/(angle_step if mode=="S" else step)))
        base=current
        for i in range(1,count+1): poses.append(propagate(base,mode,amount*i/count,control,vehicle))
        current=poses[-1]
    return poses


def control_variation(actions,mode):
    total=0.0
    for a,b in zip(actions,actions[1:]):
        if a[0]==b[0] and a[0]==mode:
            total += abs(b[2]-a[2])
    return total


def mode_runs(actions):
    runs=[]
    for action in actions:
        if not runs or runs[-1]!=action[0]: runs.append(action[0])
    return "→".join(runs)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--coarse-run",default=str(HERE/"results/raw/s09_presets_20260930_195706"))
    ap.add_argument("--map",default=str(HERE.parents[2] / "maps/S09_underground_garage/map.yaml"))
    ap.add_argument("--output",default=None)
    ap.add_argument("--presets",nargs="*",default=None)
    args=ap.parse_args()
    coarse_run=Path(args.coarse_run)
    coarse=json.loads((coarse_run/"results.json").read_text())
    out=Path(args.output) if args.output else HERE/"results/raw"/datetime.now().strftime("s09_fixed_refine_%Y%m%d_%H%M%S")
    out.mkdir(parents=True,exist_ok=True)
    vehicle=Vehicle()
    validator=OBBRasterValidator(args.map,vehicle,safety_margin=.04)
    selected=args.presets or ["BOT_LEFT","BOT_MID","BOT_RIGHT","MID_B","TOP_LEFT","TOP_MID"]
    rows=[]; outputs={}
    for preset in selected:
        case=preset+"_multi_mode"; source=coarse[case]
        if source.get("status")!="success":
            rows.append({"preset":preset,"status":"missing_coarse_solution"});continue
        start=Pose(**source["path"][0]);goal=Pose(*PRESETS[preset])
        corridors=None; corridor_error=None; stc_margin=None; refinement_step=None
        # Use the largest body margin for which the coarse topology admits a
        # feature-disc corridor.  The independent final gate always uses the
        # fixed 4 cm OBB margin, regardless of this warm-start corridor margin.
        # Prefer 5 cm nodes for continuous safety.  A conservative disc cover
        # can reject a valid coarse pose between the original lattice nodes;
        # in that case retain the original 10 cm topology sampling and rely on
        # the fixed high-resolution OBB acceptance gate.
        for candidate_step in (.05,.10):
            seed_poses,seed_actions=expand_actions(start,source["actions"],vehicle,candidate_step,math.radians(5))
            print("[corridor]",preset,len(seed_poses),"step",candidate_step,flush=True)
            for candidate_margin in (.04,.03,.02,.01,0.0):
                candidate_features=feature_disc_cover(vehicle,8,4,safety_margin=candidate_margin)
                try:
                    candidate_corridors=SafeCorridorBuilder(validator,candidate_features,max_extent=.60).build(seed_poses)
                    features,corridors,stc_margin=candidate_features,candidate_corridors,candidate_margin
                    refinement_step=candidate_step
                    break
                except ValueError as exc:
                    corridor_error=str(exc)
            if corridors is not None: break
        if corridors is None:
            rows.append({"preset":preset,"status":"corridor_infeasible","message":corridor_error});continue
        print("[solve]",preset,flush=True)
        result=solve_fixed_sequence(vehicle,start,goal,source["actions"],features,corridors,
                                    max_translation_step=refinement_step,max_rotation_step=math.radians(5))
        if result["status"]!="success":
            rows.append({"preset":preset,**result});continue
        dense=dense_replay(start,result["actions"],vehicle)
        replay_error=math.hypot(dense[-1].x-result["path"][-1].x,dense[-1].y-result["path"][-1].y)
        clear,bad_segment=validator.path_clear(dense)
        accepted=clear and replay_error<1e-5 and result["goal_error_xy"]<=.23+1e-8 and result["goal_error_yaw"]<=math.radians(12)+1e-8
        status="success" if accepted else "validator_reject"
        seed_dense=dense_replay(start,seed_actions,vehicle)
        outputs[preset]={"status":status,"path":result["path"],"dense_path":dense,
                         "seed_path":seed_poses,"seed_dense_path":seed_dense,
                         "actions":result["actions"],"seed_actions":seed_actions}
        row={"preset":preset,"status":status,"mode_runs":mode_runs(seed_actions),
             "seed_length_m":sum(abs(a[1]) for a in seed_actions if a[0]!="S"),
             "refined_length_m":result["path_length_m"],
             "seed_ackermann_curvature_tv_1pm":control_variation(seed_actions,"A"),
             "refined_ackermann_curvature_tv_1pm":control_variation(result["actions"],"A"),
             "seed_oblique_angle_tv_rad":control_variation(seed_actions,"P"),
             "refined_oblique_angle_tv_rad":control_variation(result["actions"],"P"),
             "goal_error_xy_m":result["goal_error_xy"],"goal_error_yaw_deg":math.degrees(result["goal_error_yaw"]),
             "solver_runtime_s":result["runtime_s"],"obb_sweep_clear":clear,
             "bad_segment":bad_segment,"dynamics_replay_error_m":replay_error,
             "nodes":len(seed_poses),"feature_discs":len(features),"stc_body_margin_m":stc_margin,
             "optimizer_translation_step_m":refinement_step}
        rows.append(row); print("[result]",preset,status,flush=True)

    (out/"results.json").write_text(json.dumps(outputs,default=lambda o:o.__dict__,indent=2))
    with (out/"summary.csv").open("w",newline="") as f:
        fields=sorted(set().union(*(r.keys() for r in rows)))
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    config={"coarse_run":str(coarse_run),"map":args.map,"vehicle":vehicle.__dict__,
            "stc_feature_grid":[8,4],"stc_body_margin_policy_m":[.04,.03,.02,.01,0.0],"validator_overall_body_margin_m":.04,
            "corridor_max_extent_m":.60,"optimizer_translation_step_policy_m":[.05,.10],
            "optimizer_max_rotation_step_deg":5,"terminal_xy_tolerance_m":.23,"terminal_yaw_tolerance_deg":12,
            "python":platform.python_version(),
            "sha256":{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
                      (HERE/"src/spatial_model.py",HERE/"src/obb_validator.py",HERE/"src/safe_corridor.py",
                       HERE/"src/fixed_mode_optimizer.py",Path(__file__),Path(args.map),coarse_run/"results.json")}}
    (out/"config.json").write_text(json.dumps(config,indent=2))
    plot(out,outputs,args.map)
    print("[saved]",out)


def plot(out,outputs,map_path):
    meta=yaml.safe_load(Path(map_path).read_text()); im=np.asarray(Image.open(Path(map_path).parent/meta["image"]).convert("L"));occupied=im<=65
    res=float(meta["resolution"]);ox,oy,_=meta["origin"];h,w=occupied.shape
    fig,axs=plt.subplots(2,3,figsize=(17,10),sharex=True,sharey=True)
    for ax,preset in zip(axs.ravel(),["BOT_LEFT","BOT_MID","BOT_RIGHT","MID_B","TOP_LEFT","TOP_MID"]):
        ax.imshow(occupied[::-1,:],cmap="gray_r",origin="lower",extent=[ox,ox+w*res,oy,oy+h*res],interpolation="nearest")
        data=outputs.get(preset)
        if data:
            seed=[Pose(**p) if isinstance(p,dict) else p for p in data["seed_dense_path"]]
            path=[Pose(**p) if isinstance(p,dict) else p for p in data["dense_path"]]
            ax.plot([p.x for p in seed],[p.y for p in seed],"--",color="#757575",lw=1.5,label="coarse seed")
            # Plot refined intervals by mode so switches remain visible.
            cursor=0
            for mode,amount,control in data["actions"]:
                count=max(1,math.ceil(abs(amount)/(math.radians(2) if mode=="S" else .02)))
                section=path[cursor:cursor+count+1];cursor+=count
                ax.plot([p.x for p in section],[p.y for p in section],color=MODE_COLOR[mode],lw=2.1)
            ax.set_title(preset+" | "+data["status"])
        goal=PRESETS[preset];ax.scatter([goal[0]],[goal[1]],marker="*",s=85,color="#c62828",zorder=7)
        ax.grid(alpha=.12);ax.set_aspect("equal")
    for ax in axs[-1,:]:ax.set_xlabel("x [m]")
    for ax in axs[:,0]:ax.set_ylabel("y [m]")
    handles=[plt.Line2D([0],[0],ls="--",color="#757575",label="Coarse seed")]
    handles += [plt.Line2D([0],[0],color=MODE_COLOR[m],label=m) for m in ("A","P","L","S")]
    fig.legend(handles=handles,loc="lower center",ncol=5)
    fig.suptitle("S09 fixed-mode continuous refinement: coarse seed vs refined geometry")
    fig.tight_layout(rect=[0,.05,1,.96]);fig.savefig(out/"coarse_vs_refined.png",dpi=190)


if __name__=="__main__":main()
