"""Continuous geometry refinement with a fixed coarse-search mode sequence."""
from __future__ import annotations

import math
import time

import casadi as ca
import numpy as np

from spatial_model import Pose, Vehicle, propagate, wrap
from safe_corridor import FeatureDisc, CorridorBox


def expand_actions(start:Pose,actions:list,vehicle:Vehicle,max_translation=.25,
                   max_rotation=math.radians(15)):
    """Split long primitives while preserving their exact geometry and modes."""
    poses=[start]
    expanded=[]
    for mode,amount,control in actions:
        limit=max_rotation if mode=="S" else max_translation
        count=max(1,math.ceil(abs(amount)/limit))
        part=amount/count
        for _ in range(count):
            action=(mode,part,control)
            poses.append(propagate(poses[-1],*action,vehicle))
            expanded.append(action)
    return poses,expanded


def _unwrap_seed(poses:list[Pose]):
    values=[poses[0].yaw]
    for pose in poses[1:]:
        values.append(values[-1]+wrap(pose.yaw-values[-1]))
    return np.asarray(values)


def _signed_bounds(seed:float,minimum:float,maximum_factor:float=2.0):
    magnitude=max(abs(seed),minimum)
    if seed>=0:
        return minimum,maximum_factor*magnitude
    return -maximum_factor*magnitude,-minimum


def solve_fixed_sequence(vehicle:Vehicle,start:Pose,goal:Pose,seed_actions:list,
                         features:list[FeatureDisc],corridors:list[list[CorridorBox]],
                         xy_tol=.23,yaw_tol=math.radians(12),max_iterations=600,
                         max_translation_step=.10,max_rotation_step=math.radians(10)):
    seed_poses,actions=expand_actions(start,seed_actions,vehicle,max_translation_step,max_rotation_step)
    if len(corridors)!=len(seed_poses):
        raise ValueError("corridor count does not match expanded seed poses")
    n=len(actions)
    seed_x=np.asarray([p.x for p in seed_poses])
    seed_y=np.asarray([p.y for p in seed_poses])
    seed_th=_unwrap_seed(seed_poses)
    seed_d=np.asarray([a[1] for a in actions])
    seed_c=np.asarray([a[2] for a in actions])

    opti=ca.Opti()
    X=opti.variable(3,n+1)
    D=opti.variable(n)
    C=opti.variable(n)
    x,y,th=X[0,:],X[1,:],X[2,:]
    opti.subject_to(X[:,0]==ca.vertcat(start.x,start.y,start.yaw))
    objective=0
    kmax=1.0/vehicle.min_turn_radius

    for k,(mode,seed_amount,seed_control) in enumerate(actions):
        minimum=math.radians(1) if mode=="S" else .025
        lo,hi=_signed_bounds(seed_amount,minimum)
        opti.subject_to(opti.bounded(lo,D[k],hi))
        if mode=="A":
            opti.subject_to(opti.bounded(-kmax,C[k],kmax))
            z=.5*D[k]*C[k]
            # z <= 0.16 in the configured lattice; the sixth-order series is
            # numerically smooth at zero and effectively exact here.
            sinc=1-z*z/6+z**4/120-z**6/5040
            mid=th[k]+z
            opti.subject_to(x[k+1]==x[k]+D[k]*sinc*ca.cos(mid))
            opti.subject_to(y[k+1]==y[k]+D[k]*sinc*ca.sin(mid))
            opti.subject_to(th[k+1]==th[k]+D[k]*C[k])
            objective += ca.fabs(D[k])
        elif mode=="P":
            opti.subject_to(opti.bounded(-math.pi/2,C[k],math.pi/2))
            opti.subject_to(x[k+1]==x[k]+D[k]*ca.cos(th[k]+C[k]))
            opti.subject_to(y[k+1]==y[k]+D[k]*ca.sin(th[k]+C[k]))
            opti.subject_to(th[k+1]==th[k])
            objective += ca.fabs(D[k])
        elif mode=="L":
            lateral=math.copysign(math.pi/2,seed_control)
            opti.subject_to(C[k]==lateral)
            opti.subject_to(x[k+1]==x[k]+D[k]*ca.cos(th[k]+lateral))
            opti.subject_to(y[k+1]==y[k]+D[k]*ca.sin(th[k]+lateral))
            opti.subject_to(th[k+1]==th[k])
            objective += ca.fabs(D[k])
        elif mode=="S":
            opti.subject_to(C[k]==0)
            opti.subject_to(x[k+1]==x[k])
            opti.subject_to(y[k+1]==y[k])
            opti.subject_to(th[k+1]==th[k]+D[k])
            objective += .45*ca.fabs(D[k])
        else:
            raise ValueError("unknown mode %s"%mode)

    # Every feature disc centre stays in the obstacle-eroded box built around
    # its coarse-path position.  This is the STC constraint at each node.
    for k,node_corridors in enumerate(corridors):
        if len(node_corridors)!=len(features):
            raise ValueError("feature/corridor mismatch")
        ct,st=ca.cos(th[k]),ca.sin(th[k])
        for feature,box in zip(features,node_corridors):
            fx=x[k]+feature.longitudinal_offset*ct-feature.lateral_offset*st
            fy=y[k]+feature.longitudinal_offset*st+feature.lateral_offset*ct
            opti.subject_to(opti.bounded(box.xmin,fx,box.xmax))
            opti.subject_to(opti.bounded(box.ymin,fy,box.ymax))

    target_yaw=seed_th[-1]+wrap(goal.yaw-seed_th[-1])
    opti.subject_to((x[-1]-goal.x)**2+(y[-1]-goal.y)**2 <= xy_tol**2)
    opti.subject_to(opti.bounded(target_yaw-yaw_tol,th[-1],target_yaw+yaw_tol))
    objective += 8*((x[-1]-goal.x)**2+(y[-1]-goal.y)**2)+2*(th[-1]-target_yaw)**2
    # Small tracking term retains the coarse topology.  Smoothness terms are
    # applied only within a continuous run of the same mode.
    objective += .04*ca.sumsqr(x-ca.DM(seed_x).T)+.04*ca.sumsqr(y-ca.DM(seed_y).T)
    for k in range(1,n):
        if actions[k][0]==actions[k-1][0]:
            mode=actions[k][0]
            if mode=="A": objective += .22*(C[k]-C[k-1])**2
            elif mode=="P": objective += .08*(C[k]-C[k-1])**2
            if mode!="S": objective += .03*(D[k]-D[k-1])**2

    opti.set_initial(X,np.vstack((seed_x,seed_y,seed_th)))
    opti.set_initial(D,seed_d)
    opti.set_initial(C,seed_c)
    opti.minimize(objective)
    opti.solver("ipopt",{"print_time":False},{"print_level":0,"max_iter":max_iterations,
                "tol":1e-6,"acceptable_tol":1e-5,"sb":"yes"})
    t0=time.perf_counter()
    try:
        sol=opti.solve()
    except RuntimeError as exc:
        return {"status":"solver_failure","runtime_s":time.perf_counter()-t0,
                "message":str(exc).splitlines()[-1] if str(exc) else "IPOPT failure"}
    xs=np.asarray(sol.value(X)); ds=np.asarray(sol.value(D)).reshape(-1); controls=np.asarray(sol.value(C)).reshape(-1)
    poses=[Pose(float(xs[0,k]),float(xs[1,k]),wrap(float(xs[2,k]))) for k in range(n+1)]
    refined_actions=[(mode,float(ds[k]),float(controls[k])) for k,(mode,_,__) in enumerate(actions)]
    return {"status":"success","runtime_s":time.perf_counter()-t0,
            "objective":float(sol.value(objective)),"path":poses,"actions":refined_actions,
            "seed_path":seed_poses,"expanded_seed_actions":actions,
            "path_length_m":sum(abs(a[1]) for a in refined_actions if a[0]!="S"),
            "goal_error_xy":math.hypot(poses[-1].x-goal.x,poses[-1].y-goal.y),
            "goal_error_yaw":abs(wrap(poses[-1].yaw-goal.yaw))}
