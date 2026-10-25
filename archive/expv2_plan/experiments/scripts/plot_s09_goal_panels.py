#!/usr/bin/env python3
"""Readable per-goal S09 plots using exact primitive interpolation."""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
from PIL import Image
import yaml

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE/"src"))
from spatial_model import Pose,Vehicle,propagate,wrap

MODE_COLOR={"A":"#1976d2","P":"#ef6c00","L":"#7b1fa2","S":"#d81b60"}
MODE_NAME={"A":"Ackermann","P":"Oblique","L":"Lateral","S":"Spin"}
PRESETS={
"BOT_LEFT":(.740,.500,-1.5708),"BOT_MID":(5.860,.500,-1.5708),
"BOT_RIGHT":(10.980,.500,-1.5708),"MID_A":(7.250,4.000,1.5708),
"MID_B":(2.700,4.900,-1.5708),"TOP_LEFT":(.814,9.214,-.7854),
"TOP_MID":(6.014,9.214,-.7854)}


def vehicle_corners(p:Pose,v:Vehicle):
    local=np.array([[-v.length/2,-v.width/2],[v.length/2,-v.width/2],
                    [v.length/2,v.width/2],[-v.length/2,v.width/2]])
    c,s=math.cos(p.yaw),math.sin(p.yaw)
    rot=np.array([[c,-s],[s,c]])
    return local@rot.T+np.array([p.x,p.y])


def add_vehicle(ax,p,v,color,alpha=.55,lw=1.1):
    ax.add_patch(Polygon(vehicle_corners(p,v),closed=True,facecolor=color,
                         edgecolor=color,alpha=alpha,linewidth=lw,zorder=5))
    ax.arrow(p.x,p.y,.27*math.cos(p.yaw),.27*math.sin(p.yaw),
             width=.012,head_width=.10,head_length=.10,color=color,
             length_includes_head=True,zorder=6)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--run",default=str(HERE/"results/raw/s09_presets_20260930_195706"))
    ap.add_argument("--map",default=str(HERE.parents[2] / "maps/S09_underground_garage/map.yaml"))
    args=ap.parse_args()
    run=Path(args.run)
    results=json.loads((run/"results.json").read_text())
    summary=list(csv_rows(run/"summary.csv"))
    lengths={r["case"]:float(r["path_length_m"]) for r in summary if r.get("path_length_m")}
    meta=yaml.safe_load(Path(args.map).read_text())
    im=np.asarray(Image.open(Path(args.map).parent/meta["image"]).convert("L"))
    occupied=im<=65
    res=float(meta["resolution"]); ox,oy,_=meta["origin"]
    H,W=occupied.shape
    vehicle=Vehicle()
    cases=["BOT_LEFT_multi_mode","BOT_MID_multi_mode","BOT_RIGHT_multi_mode",
           "MID_B_multi_mode","TOP_LEFT_multi_mode","TOP_MID_multi_mode"]
    fig,axs=plt.subplots(2,3,figsize=(17,10),sharex=True,sharey=True)
    legend_handles={}
    for ax,case in zip(axs.ravel(),cases):
        data=results[case]
        ax.imshow(occupied[::-1,:],cmap="gray_r",origin="lower",
                  extent=[ox,ox+W*res,oy,oy+H*res],interpolation="nearest",alpha=.9)
        rawposes=[Pose(**p) for p in data["path"]]
        actions=data["actions"]
        cur=rawposes[0]
        add_vehicle(ax,cur,vehicle,"#2e7d32",.35)
        ax.scatter([cur.x],[cur.y],s=24,color="#2e7d32",zorder=7)
        for j,(mode,amount,control) in enumerate(actions):
            mode=str(mode); amount=float(amount); control=float(control)
            n=max(1,math.ceil(abs(amount)/.025),
                  math.ceil(abs(amount)/math.radians(5)) if mode=="S" else 1)
            pts=[cur]
            for i in range(1,n+1): pts.append(propagate(cur,mode,amount*i/n,control,vehicle))
            color=MODE_COLOR[mode]
            line,=ax.plot([p.x for p in pts],[p.y for p in pts],color=color,lw=2.1,zorder=4)
            legend_handles.setdefault(mode,line)
            end=pts[-1]
            if j+1<len(actions) and actions[j+1][0]!=mode:
                ax.scatter([end.x],[end.y],marker="o",s=34,facecolor="white",
                           edgecolor="#111111",linewidth=.8,zorder=7)
                add_vehicle(ax,end,vehicle,"#757575",.20,.8)
            cur=end
        key=case[:-len("_multi_mode")]
        goal=Pose(*PRESETS[key])
        add_vehicle(ax,cur,vehicle,"#c62828",.28)
        ax.scatter([goal.x],[goal.y],marker="*",s=90,color="#c62828",zorder=8)
        unique=[]
        for m in data["modes"]:
            if not unique or unique[-1]!=m: unique.append(m)
        mode_text="→".join(unique) if unique else "—"
        ax.set_title(f"{key}: {lengths.get(case,0):.1f} m | {mode_text}",fontsize=10)
        ax.grid(alpha=.12); ax.set_aspect("equal")
    for ax in axs[-1,:]: ax.set_xlabel("x [m]")
    for ax in axs[:,0]: ax.set_ylabel("y [m]")
    handles=[legend_handles[m] for m in ("A","P","L","S") if m in legend_handles]
    labels=[MODE_NAME[m] for m in ("A","P","L","S") if m in legend_handles]
    handles += [plt.Line2D([0],[0],color="#2e7d32",lw=5),plt.Line2D([0],[0],color="#c62828",lw=5),
                plt.Line2D([0],[0],marker="o",markerfacecolor="white",markeredgecolor="#111111",lw=0)]
    labels += ["Start pose","Goal pose","Mode switch"]
    fig.suptitle("S09 ideal geometric plans — one goal per panel",fontsize=15)
    fig.legend(handles,labels,loc="lower center",ncol=7,frameon=True)
    fig.tight_layout(rect=[0,.055,1,.95])
    out=run/"paths_by_goal.png"; fig.savefig(out,dpi=190)
    print(out)


def csv_rows(path):
    import csv
    with Path(path).open(newline="") as f:
        yield from csv.DictReader(f)


if __name__=="__main__": main()
