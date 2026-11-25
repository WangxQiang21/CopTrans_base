"""Independent lower bounds proving two feasible tasks require auxiliary motion."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from benchmark import invoke, identify
from cases import LOCAL, ROOT, digest, make, packed, phase, stationary


def required_tasks():
    for task in ("single","formation"):
        r,_,_=make("development",0,task,"baseline")
        limits=r["limits"]
        def boundary(x,y,yaw,h):
            return dict(reference=[x,y,yaw],heading=h,constrain_reference_yaw=True,constrain_heading=[True]*3)
        if task=="single":
            target=math.pi
            r["start"]=boundary(0.,0.,0.,[0.]*3)
            heads=[target,0.,0.]
            r["goal"]=boundary(0.,0.,target,heads)
            # For every pure A solution, |yaw change| <= total path length/r.
            # The center speed starts and ends at zero acceleration AND velocity.
            # Integrate the pointwise upper bound on |v(t)| from either end:
            # j*t^2/2, then a*(t-t0/2), then vmax. The lost area below vmax is
            # rigorously subtracted at both ends. Reverse A cannot evade it.
            v,a,j=limits["speed"],limits["acceleration"],limits["jerk"]
            t0=a/j; t1=v/a+t0/2
            assert t1>t0
            lost=v*t0-j*t0**3/6+.5*a*(t1-t0)**2
            angle=min(target,2*math.pi-target)-2*math.radians(1.)
            min_time=(limits["radius"]*angle+2*lost)/v
            r["task_duration_cap"]=18.43
            bound=dict(minimum_pure_A_duration_s=min_time,cap_s=r["task_duration_cap"],
                       endpoint_angle_uncertainty_rad=2*math.radians(1.))
            assert r["task_duration_cap"]>2*t1 and min_time>r["task_duration_cap"]
            witness=[phase("S","prepare",stationary([0.,0.,0.]),2.93,[0.]*3),
                     phase("S","motion",stationary([0.,0.,0.]),15.1,[0.]*3,heads)]
        else:
            heads=[0.,.4,-.4]
            r["start"]=boundary(0.,0.,0.,heads);r["goal"]=boundary(3.,0.,0.,heads)
            # At every instant the common reference twist lies in null(M).
            # Full rank on an open neighborhood implies a pure A state cannot
            # leave it: all three mounts are stopped and theta_dot_i=v_i*k_i=0.
            rows=[[-math.sin(h),math.cos(h),math.sin(h)*f[1]+math.cos(h)*f[0]]
                  for h,f in zip(heads,r["vehicles"])]
            sigma=float(np.linalg.svd(rows,compute_uv=False)[-1])
            # Both reference yaw and all body headings may vary by 1 degree.
            perturb=math.radians(1)*math.sqrt(3+4*sum(x*x+y*y for x,y,*_ in r["vehicles"]))
            assert sigma>perturb
            bound=dict(smallest_singular_value=sigma,endpoint_perturbation_Frobenius_max=perturb,
                       certified_rank=3)
            witness=[phase("O","prepare",stationary([0.,0.,0.]),4.,heads),
                     phase("O","motion",[[[0.,.6,1.2,1.8,2.4,3.],[0.]*6,[0.]*6]],42.,heads)]
        yield task,r,bound,witness


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--executable",type=Path,default=ROOT/"ros2_ws/install/fwis_hybrid_joint/lib/fwis_hybrid_joint/hybrid_joint_case")
    ap.add_argument("--output",type=Path,default=LOCAL/"necessary_aux.json")
    args=ap.parse_args();rows=[]
    for task,r,bound,witness in required_tasks():
        budget=8 if task=="single" else 12
        audit=invoke(args.executable,dict(operation="certify",request=r,plan=witness,budget_s=budget),budget+2)
        # The optimization input has no witness, labels, or analytical proof.
        solved=invoke(args.executable,dict(operation="solve",request=r,budget_s=budget),budget+2)
        retained=solved.get("metrics",{}).get("aux_blocks",0)>0
        rows.append(dict(task=task,request_sha=digest(r),witness_sha=digest(witness),bound=bound,
                         witness_audit=audit,solve=solved,
                         passed=bool(audit.get("executable") and solved.get("audit",{}).get("executable") and retained)))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_bytes(packed(dict(exe_sha256=identify(args.executable),cases=rows))+b"\n")
    for row in rows:print(row["task"],row["passed"],row["bound"],row["solve"].get("status"))
    return 0 if all(row["passed"] for row in rows) else 1


if __name__=="__main__":raise SystemExit(main())
