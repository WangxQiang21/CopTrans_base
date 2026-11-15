from pathlib import Path
import sys,json,math
R=Path.cwd();sys.path.insert(0,str(R/"catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.uniform_trajectory import UniformTimedPlan
OUT=Path(__file__).parent;data=json.loads((OUT/"fr04_results_path.json").read_text());rows=[]
for case in data["cases"]:
 if case["model"] is None:continue
 plan=UniformTimedPlan.from_model_dict(case["model"]);again=UniformTimedPlan.from_model_dict(plan.to_model_dict());times={plan.duration_s*i/200 for i in range(201)}
 for phase in plan.phases:times.update((phase.t0_s,phase.end_time))
 rigid=0.;rt=0.
 for t in sorted(times):
  a=plan.state_at(t);b=again.state_at(t);cs=math.cos(a.qf.psi);sn=math.sin(a.qf.psi)
  for off in plan.model.offsets:
   v=a.vehicle(off.vehicle_id);w=b.vehicle(off.vehicle_id)
   x=a.qf.x+cs*off.x-sn*off.y;y=a.qf.y+sn*off.x+cs*off.y
   rigid=max(rigid,math.hypot(v.x-x,v.y-y));rt=max(rt,abs(v.x-w.x),abs(v.y-w.y),abs(v.theta-w.theta))
 rows.append(dict(case=case["case_id"],evaluation_times=len(times),rigidity_max_m=rigid,roundtrip_max_error=rt));assert rigid<1e-6 and rt<1e-7
(OUT/"analytic_model_roundtrip.json").write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
