"""Independent FR02-B rework confirmation and fail-closed check."""
import dataclasses
import json
import math
from pathlib import Path
import sys
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'tests').is_dir() and (p/'catkin_ws').is_dir())
sys.path.insert(0,str(ROOT/'tests'))
import test_uniform_quality as f
from fwis_v3_optimizer.formation_quality import FormationPublicationGate
OUT=Path(__file__).resolve().parent
p=f.arc_plan();r=f.audit(p,p.sample());assert r.feasible and r.metrics['dynamic_continuous_certificate']
ph=next(ph for ph in p.phases if isinstance(ph,f.MotionTiming));L=ph.edge.progress_interval[1];T=ph.duration_s
radius=math.hypot(p.model.offsets[2].x-ph.edge.control.pivot_x,p.model.offsets[2].y-ph.edge.control.pivot_y)
vals=[]
for tau in f._TAUS:
    ld=L*30*tau*tau*(1-tau)**2/T
    ldd=L*60*tau*(1-tau)*(1-2*tau)/T**2
    vals.append(radius*math.hypot(ldd,ld*ld))
peak=max(vals);ldmax=1.875*L/T;lddmax=10/math.sqrt(3)*L/T**2;ldddmax=60*L/T**3
J=radius*(ldmax**3+3*ldmax*lddmax+ldddmax)
h=max(b-a for a,b in zip(f._TAUS,f._TAUS[1:]))*T
upper=peak+J*h/2
actual=r.metrics['dynamic_evidence'][0][3]['confirmation']['acceleration']
assert f._TAUS[0]==0 and f._TAUS[-1]==1
assert abs(actual['candidate_peak']-peak)<1e-10
assert abs(actual['derivative_bound']-J)<1e-10
assert abs(actual['adopted_bound']-upper)<1e-10 and upper<p.limits.a_max
edge=p.edges[0]
negative_edge=f.a_arc_edge(edge.model,edge.start_qf,dict(edge.start_headings),edge.control.delta_psi,(edge.control.pivot_x,edge.control.pivot_y),dict(edge.directions),limits=dataclasses.replace(edge.limits,jerk_max=1.5),allow_reverse=edge.allow_reverse)
negative=f.plan_for((negative_edge,),limits=dataclasses.replace(p.limits,jerk_max=1.5))
proj=negative.sample();bad=f.audit(negative,proj)
gate=FormationPublicationGate();gate.begin_request(negative.plan_id);decision=gate.offer(negative.plan_id,proj.timed_trajectory,bad)
assert not bad.feasible and not bad.executable and bad.reason=='UNIFORM_DYNAMIC_UNCONFIRMED' and not decision.publish
assert bad.reason!=f.FORMATION_RETIMING_INFEASIBLE
payload={'positive_arc':{'feasible':r.feasible,'continuous_confirmation':r.metrics['dynamic_continuous_certificate'],'independent_radius':radius,'independent_candidate_a':peak,'independent_J':J,'independent_h':h,'independent_upper_a':upper,'a_limit':p.limits.a_max,'reported_confirmation':actual},'unconfirmed_negative':{'jerk_limit':negative.limits.jerk_max,'feasible':bad.feasible,'executable':bad.executable,'reason':bad.reason,'gate_publish':decision.publish,'failure':dataclasses.asdict(bad.failure),'confirmation':bad.metrics.get('dynamic_confirmation')}}
(OUT/'confirmation_probe.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(payload,ensure_ascii=False,indent=2))
