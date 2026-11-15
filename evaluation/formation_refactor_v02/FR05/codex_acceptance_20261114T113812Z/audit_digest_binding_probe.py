from pathlib import Path
import sys,json
R=Path.cwd();OUT=Path(__file__).parent;sys.path[:0]=[str(R/"tests"),str(R/"catkin_ws/src/fwis_v3_optimizer/src")]
import test_uniform_playback_plan as t
from fwis_v3_optimizer.uniform_motion import a_straight_edge
from fwis_v3_optimizer.uniform_trajectory import UniformTimedPlan
from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_playback import frame_at,status_document,build_markers
old=t.straight_plan(plan_id=700124)
edge=a_straight_edge(t.model(),FormationPose(10,10,0),{i:0 for i in t.VEHICLE_IDS},(2.0,0.0),t.directions())
new=UniformTimedPlan.from_edges((edge,),t.BODY_SHAPES,plan_id=old.plan_id,formation_id=old.formation_id)
trajectory,rebuilt,digest=t.verify(new)
assert old.model_sha256()!=new.model_sha256()
status={"plan_id":old.plan_id,"formation_id":old.formation_id,"backend":"uniform_v02","model_sha256":old.model_sha256(),"status":"FEASIBLE","feasible":True,"executable":True,"trajectory_published":True}
frame=frame_at(trajectory,.5*new.duration_s);doc=status_document(trajectory,frame,status,model_sha256=digest,backend="uniform_v02")
markers,_,audit_state,executable=build_markers(trajectory,frame,status=status)
row={"case":"same_plan_id_different_model_audit_binding","status_model_sha256":old.model_sha256(),"displayed_model_sha256":new.model_sha256(),"old_goal":[old.goal_qf.x,old.goal_qf.y],"new_goal":[new.goal_qf.x,new.goal_qf.y],"status_document_executable":doc["executable"],"status_document_certification":doc["certification"],"marker_executable":executable,"marker_audit_state":audit_state,"document":doc}
assert not row["status_document_executable"] and not row["marker_executable"]
(OUT/"audit_digest_binding_probe.json").write_text(json.dumps(row,indent=2));print(json.dumps(row,indent=2))
