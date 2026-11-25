"""Construct independent examples where either geometry/time or full rank excludes pure A."""
import math
import numpy as np
from cases import phase, stationary, limits, CONTRACT, digest, packed
from benchmark import invoke, identify


def construct(task,profile):
    multi=task=="formation";lim=limits(task,profile);margin=.05 if multi else .04
    d=4.5; half=.45; res=.05; n=240
    initial=[.04,.17,-.13] if multi else [.02]*3
    x,y=0.,-2.;q=[x,y,0.];g=[x,y+d,0.]
    v=[[a,b,.738+2*margin,.5+2*margin] for a,b in (CONTRACT["offsets"] if multi else [(0.,0.)])]
    # No mode transition is needed at task end. Initial Ackermann -> O has a
    # stationary wheel preparation phase; ample time for steering through pi/2.
    prep=6.;motion=2.01*d/lim["speed"];cap=prep+motion+.1
    grid=np.zeros((n,n),dtype=int)
    if multi:
        # n_i dot (V + omega J r_i)=0 for all three. det nonzero means all
        # body velocities and yaw rates vanish on an open set for any A input.
        mat=[[-math.sin(h),math.cos(h),a*math.cos(h)+b*math.sin(h)]
             for h,(a,b,*_) in zip(initial,v)]
        sigma=float(np.linalg.svd(mat,compute_uv=False)[-1])
        perturb=math.radians(1)*math.sqrt(3+4*sum(a*a+b*b for a,b,*_ in v))
        assert sigma>perturb
        proof=dict(type="full_rank_start",smallest_singular_value=sigma,endpoint_perturbation_bound=perturb)
    else:
        grid[:]=1
        grid[:,n//2-9:n//2+9]=0
        # To attain |yaw|=.22 every continuous A route from .02 must first
        # fit the vehicle's projected X width through the two parallel walls.
        alpha=.22
        projected=v[0][2]*math.cos(alpha)+v[0][3]*math.sin(alpha)
        lateral_bound=lim["speed"]*cap*math.sin(alpha)
        assert projected>2*half and lateral_bound < d-.02
        proof=dict(type="corridor_lateral_reach",width_m=2*half,yaw_barrier_rad=alpha,
                   projected_width_m=projected,max_A_lateral_m=lateral_bound,required_m=d)
    # All translation vectors remain in the +/-90 degree oblique domain by
    # selecting each vehicle's signed speed. Body headings stay fixed.
    gears=[1 if math.sin(h)>=0 else -1 for h in initial]
    first=phase("O","prepare",stationary(q),prep,initial);first["gear"]=gears
    last=phase("O","motion",[[[x]*6,np.linspace(y,y+d,6).tolist(),[0.]*6]],motion,initial);last["gear"]=gears
    edge=np.diff(np.r_[0,grid.ravel(),0]);lo=np.nonzero(edge==1)[0];hi=np.nonzero(edge==-1)[0]
    def b(q):return dict(reference=q if multi else [q[0],q[1],initial[0]],heading=initial,
                          constrain_reference_yaw=True,constrain_heading=[True]*3)
    r=dict(grid=dict(width=n,height=n,resolution=res,origin=[-6.,-6.],occupied_runs=np.c_[lo,hi-lo].tolist()),
           vehicles=v,cargo=[.5,0.,2.05,2.8] if multi else None,start=b(q),goal=b(g),limits=lim,
           task_duration_cap=cap,allow_reverse=True)
    return r,[first,last],proof


def main():
    import argparse
    from pathlib import Path
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    rows=[]
    for task in ("single","formation"):
        for profile in ("standard","baseline"):
            req,witness,bound=construct(task,profile)
            budget=8 if task=="single" else 12
            audit=invoke(args.executable,dict(operation="certify",request=req,plan=witness,budget_s=budget),budget+2)
            solve=invoke(args.executable,dict(operation="solve",request=req,budget_s=budget),budget+2)
            rows.append(dict(task=task,profile=profile,bound=bound,request_sha=digest(req),witness_sha=digest(witness),
                             witness_audit=audit,solve=solve))
    with args.output.open("xb") as output:
        output.write(packed(dict(exe_sha256=identify(args.executable),cases=rows))+b"\n")
    return 0 if all(r["witness_audit"].get("executable") and r["solve"].get("audit",{}).get("executable") and
                    r["solve"].get("metrics",{}).get("aux_blocks",0)>0 for r in rows) else 1


if __name__=="__main__":
    raise SystemExit(main())
