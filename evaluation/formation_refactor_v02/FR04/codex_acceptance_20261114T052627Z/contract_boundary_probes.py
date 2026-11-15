from pathlib import Path
import sys,json,math
from dataclasses import replace
R=Path.cwd();sys.path[:0]=[str(R/"tests"),str(R/"catkin_ws/src/fwis_v3_optimizer/src")]
import test_uniform_pipeline as t
from fwis_v3_optimizer.uniform_pipeline import _build_and_audit
from fwis_v3_optimizer.uniform_motion import a_straight_edge,s_spin_edge
from fwis_v3_optimizer.uniform_quality import UniformAuditRequest,audit_uniform_timed_plan
from fwis_v3_optimizer.uniform_timing import timing_motion_limits
OUT=Path(__file__).parent;rows=[]
req=t.make_request(t.FormationPose(11,10,0),plan_id=501)
edge=a_straight_edge(req.model,req.start_qf,req.start_headings,(1,0),{i:1 for i in t.VEHICLE_IDS},allow_reverse=True)
def check(name,request,edges,expected,hold=False):
 p,projection,a=_build_and_audit(edges,request,zero_motion=hold)
 rows.append(dict(case=name,status=a.status,reason=a.reason,feasible=a.feasible,plan_present=p is not None))
 assert (a.feasible if expected=="FEASIBLE" else (not a.feasible and a.reason==expected)),rows[-1]
 return p
p=check("exact_forward",req,(edge,),"FEASIBLE")
# Independently construct the audit request; do not invoke the production contract helper.
manual=UniformAuditRequest(plan_id=req.plan_id,formation_id=req.formation_id,model=req.model,vehicle_shapes=req.vehicle_shapes,start_qf=req.start_qf,start_headings=req.start_headings,goal_qf=edge.end_qf,goal_headings=dict(edge.end_headings),timing_limits=req.successor_config.timing_limits,allow_reverse=True)
assert audit_uniform_timed_plan(manual,p,p.sample(),req.grid).feasible
check("outside_xy_tolerance",replace(req,goal_qf=t.FormationPose(11.0201,10,0)),(edge,),"REQUEST_GOAL_TOLERANCE_EXCEEDED")
check("outside_psi_tolerance",replace(req,goal_qf=t.FormationPose(11,10,math.radians(1.001))),(edge,),"REQUEST_GOAL_TOLERANCE_EXCEEDED")
check("inside_xy_psi_tolerance_free_heading",replace(req,goal_qf=t.FormationPose(11.01,10.005,math.radians(.5)),goal_headings={i:math.pi/2 for i in t.VEHICLE_IDS}),(edge,),"FEASIBLE")
check("wrong_initial_heading",replace(req,start_headings={i:.1 for i in t.VEHICLE_IDS}),(edge,),"REQUEST_START_EDGE_MISMATCH")
spin=s_spin_edge(req.model,req.start_qf,req.start_headings,{1:.1,2:0,3:0},limits=timing_motion_limits(req.successor_config.timing_limits))
spreq=replace(req,goal_qf=req.start_qf)
sp=check("uniform_S_zero_angle_vehicles_preserved",spreq,(spin,),"FEASIBLE")
for ts in (phase.t0_s+.5*phase.duration_s for phase in sp.phases if phase.kind=="motion"):
 state=sp.state_at(ts);assert state.mode=="S"
 for i in (2,3):assert state.vehicle(i).theta==0
check("explicit_terminal_heading_constraint",replace(spreq,successor_config=replace(spreq.successor_config,goal_heading_free=False),goal_headings=dict(req.start_headings)),(spin,),"REQUEST_TERMINAL_HEADINGS_MISMATCH")
check("empty_hold_near_goal",replace(req,goal_qf=t.FormationPose(10.005,10.005,math.radians(.5))),(),"FEASIBLE",hold=True)
(OUT/"contract_boundary_probes.json").write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
