from pathlib import Path
import sys,json,math
from dataclasses import replace
from unittest.mock import patch
R=Path.cwd(); sys.path[:0]=[str(R/"tests"),str(R/"catkin_ws/src/fwis_v3_optimizer/src")]
import test_uniform_pipeline as t
import fwis_v3_optimizer.uniform_pipeline as p
from fwis_v3_optimizer.uniform_search import GraphSearchConfig,UniformGraphSearch,initial_state,SuccessorConfig
from fwis_v3_optimizer.uniform_motion import a_straight_edge
from fwis_v3_optimizer.uniform_trajectory import UniformTimedPlan
from fwis_v3_optimizer.uniform_quality import UniformAuditRequest,audit_uniform_timed_plan
from fwis_v3_optimizer.formation_collision import FormationCollisionChecker
OUT=Path(__file__).parent; rows=[]
# A valid but unrelated final plan is injected exactly at reconstruction output.
req=t.make_request(t.FormationPose(13,10,0),plan_id=401)
other=a_straight_edge(req.model,t.FormationPose(0,0,0),req.start_headings,(1,0),{i:1 for i in t.VEHICLE_IDS},allow_reverse=True)
wrong=UniformTimedPlan.from_edges((other,),req.vehicle_shapes,plan_id=req.plan_id,formation_id=req.formation_id,model=req.model)
real_from_edges=UniformTimedPlan.from_edges; calls=[0]
def build(*args,**kw):
 calls[0]+=1
 return wrong if calls[0]==2 else real_from_edges(*args,**kw)
with patch.object(UniformTimedPlan,"from_edges",side_effect=build):
 result=p.plan_uniform(req)
rows.append(dict(case="final_plan_request_binding_fault_injection",build_calls=calls[0],status=result.status,executable=result.executable,requested_start=[10,10,0],requested_goal=[13,10,0],plan_present=result.plan is not None,reason=result.reason,actual_start=None if result.plan is None else [result.plan.start_qf.x,result.plan.start_qf.y,result.plan.start_qf.psi],actual_goal=None if result.plan is None else [result.plan.goal_qf.x,result.plan.goal_qf.y,result.plan.goal_qf.psi],audit=result.evaluation.status))
# Same unrelated plan with independently bound request must fail the existing FR02 audit.
bound=UniformAuditRequest(plan_id=req.plan_id,formation_id=req.formation_id,model=req.model,vehicle_shapes=req.vehicle_shapes,start_qf=req.start_qf,start_headings=req.start_headings,goal_qf=req.goal_qf,goal_headings=dict(wrong.goal_headings),timing_limits=req.successor_config.timing_limits,allow_reverse=req.successor_config.allow_reverse)
a=audit_uniform_timed_plan(bound,wrong,wrong.sample(),req.grid)
rows[-1]["bound_request_audit"]=a.as_status()
assert result.status == "AUDIT_REJECTED" and not result.executable and result.plan is None
assert not a.feasible
# Real final audit refusal must suppress executable result.
real_audit=p.audit_uniform_timed_plan; n=[0]
def reject_final(*args,**kw):
 n[0]+=1
 if n[0]==2: return a
 return real_audit(*args,**kw)
with patch.object(p,"audit_uniform_timed_plan",side_effect=reject_final):
 failed=p.plan_uniform(req)
rows.append(dict(case="final_audit_refusal",status=failed.status,executable=failed.executable,audit_calls=n[0]))
# A post-failure new request produces only its own IDs/model; no cached output.
ok=p.plan_uniform(t.make_request(t.FormationPose(11,10,0),plan_id=402))
rows.append(dict(case="new_request_after_failure",status=ok.status,executable=ok.executable,plan_id=ok.plan.plan_id,goal_x=ok.plan.goal_qf.x))
assert failed.status == "AUDIT_REJECTED" and not failed.executable
assert ok.status == "SUCCESS" and ok.executable and ok.plan.plan_id==402 and ok.plan.goal_qf.x==11
(OUT/"independent_probes.json").write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
