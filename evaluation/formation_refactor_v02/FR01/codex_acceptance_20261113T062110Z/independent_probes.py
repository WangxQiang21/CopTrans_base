"""Codex FR01 independent acceptance probes; geometry only, no implementation edits."""
import copy
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.formation import FormationModel, FormationPose, VehicleOffset
from fwis_v3_optimizer.uniform_motion import (a_arc_edge, a_straight_edge, o_translation_edge, s_spin_edge, validate_uniform_edge, UniformMotionError)
from fwis_v3_optimizer.retiming import MotionLimits

OUT = Path(__file__).resolve().parent
MODEL = FormationModel(tuple(VehicleOffset(i,x,y) for i,x,y in [(1,1.,0.),(2,0.,1.),(3,0.,-1.)]))

def arc():
    return a_arc_edge(MODEL, FormationPose(0.,0.,0.), {1:math.pi/2,2:math.pi,3:0.}, math.pi/2, (0.,0.), {1:1,2:1,3:1})

def spin():
    return s_spin_edge(MODEL, FormationPose(.4,-.2,.3), {1:.1,2:-.4,3:1.2}, {1:math.pi/2,2:0.,3:-math.pi/4})

rng=random.Random(20261112)
valid=[]
max_lateral=0.;max_fd=0.;max_center=0.
for k in range(24):
    psi=rng.uniform(-3,3); q=FormationPose(rng.uniform(-2,2),rng.uniform(-2,2),psi)
    c=rng.choice([(0.,0.),(0.,2.),(0.,-2.),(2.,0.),(-2.,0.)])
    da=rng.choice([-1,1])*rng.uniform(.1,1.7)
    dirs={i:rng.choice([-1,1]) for i in (1,2,3)}
    heads={}
    for o in MODEL.offsets:
        rx,ry=o.x-c[0],o.y-c[1]
        ux,uy=(-ry,rx) if da>0 else (ry,-rx)
        wx=math.cos(psi)*ux-math.sin(psi)*uy
        wy=math.sin(psi)*ux+math.cos(psi)*uy
        heads[o.vehicle_id]=math.atan2(wy,wx)+(math.pi if dirs[o.vehicle_id]<0 else 0.)
    e=a_arc_edge(MODEL,q,heads,da,c,dirs,allow_reverse=True)
    for frac in (.173,.529,.863):
        lam=e.progress_interval[1]*frac; eps=1e-6
        s=e.state_at(lam);sm=e.state_at(lam-eps);sp=e.state_at(lam+eps)
        d=e.derivative_at(lam);hd=dict(s.headings);dp={i:(x,y) for i,x,y in d.dp}
        for i in (1,2,3):
            fd=tuple((a-b)/(2*eps) for a,b in zip(sp.center(i),sm.center(i)))
            max_fd=max(max_fd,max(abs(a-b) for a,b in zip(fd,dp[i])))
            lat=abs(fd[0]*(-math.sin(hd[i]))+fd[1]*math.cos(hd[i]))
            max_lateral=max(max_lateral,lat)
        pc=(q.x+math.cos(psi)*c[0]-math.sin(psi)*c[1],q.y+math.sin(psi)*c[0]+math.cos(psi)*c[1])
        ang=psi+(lam if da>0 else -lam)
        for o in MODEL.offsets:
            rx,ry=o.x-c[0],o.y-c[1]
            expected=(pc[0]+math.cos(ang)*rx-math.sin(ang)*ry,pc[1]+math.sin(ang)*rx+math.cos(ang)*ry)
            max_center=max(max_center,max(abs(a-b) for a,b in zip(expected,s.center(o.vehicle_id))))
    validate_uniform_edge(e);valid.append({'case':k,'pivot':c,'delta_psi':da,'directions':dirs})
assert max_fd<=1e-4 and max_lateral<=1e-4 and max_center<=1e-7

negative=[]
def probe(name,edge,description):
    try:
        metrics=validate_uniform_edge(edge)
    except Exception as exc:
        negative.append({'case':name,'expected':'REJECT','actual':'REJECTED','reason':getattr(exc,'reason',type(exc).__name__),'detail':str(exc),'invalidity':description})
    else:
        negative.append({'case':name,'expected':'REJECT','actual':'INCORRECTLY_ACCEPTED','metrics':metrics.as_dict(),'invalidity':description})

e=arc()
shift=.2
object.__setattr__(e,'start_headings',tuple((i,h+shift) for i,h in e.start_headings))
object.__setattr__(e,'end_headings',tuple((i,h+shift) for i,h in e.end_headings))
object.__setattr__(e,'segments',tuple(replace(s,theta_start=s.theta_start+shift,theta_end=s.theta_end+shift) for s in e.segments))
probe('N01_A_arc_body_not_tangent',e,'All body headings shifted by 0.2 rad with end headings and segments consistent. Center velocity/body lateral residual=sin(0.2)=0.19866933 m/rad; no-slip A invalid.')

e=spin()
object.__setattr__(e,'wheel_angles_end',tuple((i,(0.,0.,0.,0.) if i==2 else vals) for i,vals in e.wheel_angles_end))
object.__setattr__(e,'wheel_factors_end',tuple((i,(1.,1.,1.,1.) if i==2 else vals) for i,vals in e.wheel_factors_end))
probe('N02_S_zero_angle_end_wheels_A',e,'car2 starts canonical S but its end wheel angles/factors are A zero-curvature. FR02 would derive incorrect exit setup.')

e=arc();object.__setattr__(e,'limits',replace(e.limits,steer_angle_max=.1))
probe('N03_wheel_limit_exceeded',e,'Actual max wheel angle about 0.29325 rad exceeds edge limits.steer_angle_max=0.1 rad.')

e=arc();object.__setattr__(e,'limits',replace(e.limits,r_min_ack=2.))
probe('N04_turn_radius_below_limit',e,'Every vehicle radius is 1 m, below edge limit 2 m.')

e=arc();object.__setattr__(e,'mode','S')
probe('N05_mode_primitive_mismatch',e,'Global mode S with A_ARC primitive and all A segments.')

e=spin();object.__setattr__(e,'segments',tuple(replace(s,mode=0) if s.vehicle_id==2 else s for s in e.segments))
probe('N06_S_zero_angle_segment_A',e,'Global S edge contains zero-angle car2 segment.mode=A; violates uniform mode.')

e=arc();object.__setattr__(e,'wheel_angles_start',tuple((i,(float("nan"),)+v[1:] if i==1 else v) for i,v in e.wheel_angles_start))
probe('N07_NaN_wheel',e,'Canonical car1 wheel angle NaN must not yield a passing finite metrics report.')

payload={'module':sys.modules['fwis_v3_optimizer.uniform_motion'].__file__,'positive_arc_cases':valid,'positive_metrics':{'max_center_error_m':max_center,'max_finite_difference_error':max_fd,'max_lateral_residual':max_lateral},'negative_cases':negative,'unexpected_acceptances':sum(x['actual']=='INCORRECTLY_ACCEPTED' for x in negative),'method':'Fault injection uses object.__setattr__, matching submitted P10/P12. Does not suggest normal factories produce these bad edges.'}
(OUT/'independent_probes.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'positive_count':len(valid),'positive_metrics':payload['positive_metrics'],'negative_cases':negative},ensure_ascii=False,indent=2))
sys.exit(1 if payload['unexpected_acceptances'] else 0)
