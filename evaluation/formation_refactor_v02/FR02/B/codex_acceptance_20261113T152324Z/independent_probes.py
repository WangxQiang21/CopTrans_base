"""Codex FR02-B probes: certificate semantics, Spin sweep and motion-time projection."""
import json
import math
from pathlib import Path
import sys
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'tests').is_dir() and (p/'catkin_ws').is_dir())
sys.path.insert(0,str(ROOT/'tests'))
import test_uniform_quality as f
from fwis_v3_optimizer.formation_collision import FormationCollisionChecker
from fwis_v3_optimizer.formation_quality import FormationPublicationGate
from fwis_v3_optimizer.uniform_timing import MotionTiming
OUT=Path(__file__).resolve().parent

certificates=[]
for name,factory in [('straight',f.straight_plan),('nonzero_pivot_arc',f.arc_plan),('complex',f.complex_plan)]:
    plan=factory();projection=plan.sample();result=f.audit(plan,projection)
    gate=FormationPublicationGate();gate.begin_request(plan.plan_id)
    decision=gate.offer(plan.plan_id,projection.timed_trajectory,result)
    certificates.append({'name':name,'status':result.status,'feasible':result.feasible,'executable':result.executable,'dynamic_continuous_certificate':result.metrics.get('dynamic_continuous_certificate'),'limitations':result.metrics.get('dynamic_limitations'),'pure_python_publication_gate_publish':decision.publish,'unexpected_unverified_feasible':result.feasible and result.metrics.get('dynamic_continuous_certificate') is False})

plan=f.spin_plan();grid=f.Grid(f.np.zeros((400,400),dtype=bool),.05,-10.,-10.)
point=(1.075,.475);ix=int(math.floor((point[0]-grid.ox)/grid.res));iy=int(math.floor((point[1]-grid.oy)/grid.res));grid.occ[iy,ix]=True
checker=FormationCollisionChecker(grid,plan.model,plan.vehicle_shapes,.05)
ends=[]
for label,qf,heads in [('start',plan.start_qf,dict(plan.start_headings)),('goal',plan.goal_qf,dict(plan.goal_headings))]:
    c=checker.static_check(qf,heads);ends.append({'label':label,'success':c.success,'reason':c.reason})
r=f.audit(plan,plan.sample(),grid)
spin={'map':{'shape':[400,400],'resolution':.05,'origin':[-10.,-10.],'occupied_cells':[[iy,ix]]},'endpoints':ends,'audit_status':r.status,'reason':r.reason,'first_collision':r.metrics.get('first_collision')}
assert all(v['success'] for v in ends) and r.reason==f.FORMATION_SWEEP_COLLISION

plan=f.complex_plan();motion_times=[ph.t0_s+.371*ph.duration_s for ph in plan.phases if isinstance(ph,MotionTiming)]
def times(dt):return sorted(set([0.,plan.duration_s]+[ph.t0_s for ph in plan.phases]+[ph.end_time for ph in plan.phases]+[min(i*dt,plan.duration_s) for i in range(1,int(plan.duration_s/dt)+1)]+motion_times))
t02,t05=times(.02),times(.05);p02,p05=plan.sample(t02),plan.sample(t05)
comparisons=[]
for t in motion_times:
    a=p02.states[t02.index(t)];b=p05.states[t05.index(t)];ref=plan.state_at(t)
    same=a.as_dict()==b.as_dict()==ref.as_dict();assert same
    comparisons.append({'t':t,'phase':a.phase_id,'kind':a.phase_kind,'mode':a.mode,'same_exact_state':same})
original_times=[{'t':t,'phase':plan.state_at(t).phase_id,'kind':plan.state_at(t).phase_kind} for t in [.137,.419,.777]]
result={'certificate_semantics':certificates,'spin_endpoint_clear_mid_sweep_collision':spin,'moving_projection_checks':comparisons,'submitted_B07_common_times':original_times}
(OUT/'independent_probes.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False,indent=2))
sys.exit(1 if any(c['unexpected_unverified_feasible'] for c in certificates) else 0)
