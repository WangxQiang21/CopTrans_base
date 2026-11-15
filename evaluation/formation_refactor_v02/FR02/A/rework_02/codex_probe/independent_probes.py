"""Codex FR02-A acceptance probes. No algorithm changes or S9/collision work."""
import json
import math
from pathlib import Path
import struct
import sys

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'catkin_ws').is_dir() and (p/'tests').is_dir())
sys.path.insert(0,str(ROOT/'tests'))
import test_uniform_trajectory as fixtures
from fwis_v3_optimizer.uniform_trajectory import UniformTimedPlan
OUT=Path(__file__).resolve().parent

def below(x):
    return struct.unpack('d',struct.pack('Q',struct.unpack('Q',struct.pack('d',x))[0]-1))[0]

plans={'straight':fixtures.straight_plan(),'spin':fixtures.spin_plan(),'complex':fixtures.plan_for(fixtures.complex_edges())}
interior=[]
boundaries=[]
for name,plan in plans.items():
    restored=UniformTimedPlan.from_model_dict(plan.to_model_dict())
    for ph in plan.phases:
        for frac in (.013,.127,.371,.619,.839,.947):
            t=ph.t0_s+ph.duration_s*frac
            st=plan.state_at(t);rt=restored.state_at(t)
            for vid in plan.vehicle_ids:
                v=st.vehicle(vid);r=rt.vehicle(vid)
                assert max(abs(v.x-r.x),abs(v.y-r.y),abs(v.theta-r.theta))<1e-7
            centers=plan.model.centers(st.qf)
            assert all(max(abs(st.vehicle(vid).x-centers[vid][0]),abs(st.vehicle(vid).y-centers[vid][1]))<1e-7 for vid in plan.vehicle_ids)
            interior.append({'plan':name,'phase':ph.phase_id,'t':t,'status':'PASSED'})
        if ph.kind!='motion':continue
        times=[('predecessor_float',below(ph.end_time))]+[('relative_1e-'+str(k),ph.end_time-ph.duration_s*10.**(-k)) for k in range(5,16)]
        for label,t in times:
            if not ph.t0_s<=t<ph.end_time:
                continue  # A requested interior time can round to the endpoint.
            rec={'plan':name,'phase':ph.phase_id,'label':label,'t':t,'t0':ph.t0_s,'end':ph.end_time,'within_time_domain':True}
            try:
                plan.state_at(t);restored.state_at(t)
            except Exception as exc:
                rec.update(status='FAILED',reason=getattr(exc,'reason',type(exc).__name__),detail=str(exc))
            else:rec.update(status='PASSED')
            boundaries.append(rec)

plan=plans['spin'];ph=next(p for p in plan.phases if p.kind=='motion');state=plan.state_at(ph.t0_s+ph.duration_s*.5)
wheel=[]
positions=[(x,y) for x in (plan.limits.wheelbase/2,-plan.limits.wheelbase/2) for y in (plan.limits.track/2,-plan.limits.track/2)]
for vid in plan.vehicle_ids:
    v=state.vehicle(vid)
    for index,((x,y),angle,rolling) in enumerate(zip(positions,v.wheel_angles,v.wheel_rolling_speeds)):
        expected_vector=(-y*v.yaw_rate_radps,x*v.yaw_rate_radps)
        represented_vector=(rolling*math.cos(angle),rolling*math.sin(angle))
        expected_signed=expected_vector[0]*math.cos(angle)+expected_vector[1]*math.sin(angle)
        error=math.hypot(expected_vector[0]-represented_vector[0],expected_vector[1]-represented_vector[1])
        wheel.append({'vehicle_id':vid,'wheel_index':index,'theta_dot':v.yaw_rate_radps,'wheel_angle':angle,'actual_signed_rolling':rolling,'expected_signed_rolling':expected_signed,'wheel_velocity_error_mps':error,'status':'PASSED' if error<=1e-7 else 'FAILED'})

result={'scope':'FR02-A only','interior_roundtrip_positive_count':len(interior),'interior_roundtrip':interior,'boundary_evaluations':boundaries,'boundary_failure_count':sum(r['status']=='FAILED' for r in boundaries),'S_wheel_velocity':wheel,'S_wheel_failure_count':sum(r['status']=='FAILED' for r in wheel)}
(OUT/'independent_probes.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ['interior_roundtrip','boundary_evaluations','S_wheel_velocity']},indent=2))
print('First boundary failures:',[r for r in boundaries if r['status']=='FAILED'][:3])
print('Failed wheel rows:',[r for r in wheel if r['status']=='FAILED'])
sys.exit(1 if result['boundary_failure_count'] or result['S_wheel_failure_count'] else 0)
