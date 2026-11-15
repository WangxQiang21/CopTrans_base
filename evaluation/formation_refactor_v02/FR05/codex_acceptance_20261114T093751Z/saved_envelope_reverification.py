from pathlib import Path
import sys,json,math
R=Path.cwd();OUT=Path(__file__).parent;sys.path.insert(0,str(R/"catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.formation_playback import verify_uniform_playback_plan,frame_at
files=["ros/envelopes/envelope_00600004.json","ros/envelopes/envelope_00600005.json","ros/envelopes/envelope_00600006.json","ros_failure/envelopes/envelope_00600010.json","ros_failure/envelopes/envelope_00600012.json"];rows=[]
for filename in files:
 d=json.loads((OUT.parent/filename).read_text());keys=("plan_id","formation_id","backend","semantic_version","model_version","model_sha256","model_json","certification","trajectory_fields");traj,plan,digest=verify_uniform_playback_plan(**{k:d[k] for k in keys});maxpos=0.;maxheading=0.;rigid=0.;n=0
 for phase in plan.phases:
  for i in range(1,12):
   t=phase.t0_s+phase.duration_s*i/12;f=frame_at(traj,t);a=plan.state_at(t);n+=1;byid={v.vehicle_id:v for v in a.vehicles};c=math.cos(a.qf.psi);s=math.sin(a.qf.psi);offset={o.vehicle_id:(o.x,o.y) for o in plan.model.offsets}
   for v in f.vehicles:
    w=byid[v.vehicle_id];maxpos=max(maxpos,math.hypot(v.center_x-w.x,v.center_y-w.y));maxheading=max(maxheading,abs(math.atan2(math.sin(v.theta-w.theta),math.cos(v.theta-w.theta))));x,y=offset[v.vehicle_id];rigid=max(rigid,math.hypot(v.center_x-(a.qf.x+c*x-s*y),v.center_y-(a.qf.y+s*x+c*y)))
 row={"file":filename,"plan_id":plan.plan_id,"model_sha256":digest,"phase_count":len(plan.phases),"internal_times_checked":n,"position_error_max_m":maxpos,"heading_error_max_rad":maxheading,"rigidity_residual_max_m":rigid,"primitives":[e.primitive for e in plan.edges]};rows.append(row);assert maxpos<=1e-7 and maxheading<=1e-7 and rigid<=1e-6
(OUT/"saved_envelope_reverification.json").write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
