from pathlib import Path
import sys,json,hashlib,math,time
from dataclasses import asdict
import numpy as np
from PIL import Image
from unittest.mock import patch
R=Path.cwd();OUT=Path(__file__).parent;FR=OUT.parent;sys.path[:0]=[str(R/"catkin_ws/src/fwis_v3_optimizer/src"),str(FR)]
import fr06_scene as scene
from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import FormationModel,FormationPose,VehicleOffset
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
from fwis_v3_optimizer.uniform_pipeline import UniformPlanningRequest,plan_uniform,request_fingerprint
from fwis_v3_optimizer.uniform_search import SuccessorConfig,GraphSearchConfig
from fwis_v3_optimizer.uniform_trajectory import UniformTimedPlan
from fwis_v3_optimizer.uniform_quality import UniformAuditRequest,audit_uniform_timed_plan
M=FormationModel((VehicleOffset(1,1,0),VehicleOffset(2,0,1),VehicleOffset(3,0,-1)))
SH={i:VehicleShape(.738,.5) for i in (1,2,3)};START=FormationPose(6,6,0);GOAL=FormationPose(10,10,math.pi/2);HEAD={i:0 for i in (1,2,3)}
def sha(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()
def grid(w):
 img=np.asarray(Image.open(FR/"maps"/("corridor_w"+str(w))/"map.pgm"));occ=np.flipud(img<127).copy();assert occ.shape==(320,320);assert np.array_equal(occ,scene.corridor_occupancy(w));return Grid(occ,.05,0,0)
def request(w,pid=7606,formation="codex_fr06"):
 return UniformPlanningRequest(grid=grid(w),model=M,vehicle_shapes=SH,start_qf=START,start_headings=HEAD,goal_qf=GOAL,goal_headings=None,plan_id=pid,formation_id=formation,successor_config=SuccessorConfig(),search_config=GraphSearchConfig())
def audit(plan,w,end_headings):
 q=UniformAuditRequest(plan_id=plan.plan_id,formation_id=plan.formation_id,model=M,vehicle_shapes=SH,start_qf=START,start_headings=HEAD,goal_qf=GOAL,goal_headings=end_headings,timing_limits=FormationTimingLimits(),allow_reverse=True)
 a=audit_uniform_timed_plan(q,plan,plan.sample(),grid(w));return a
ref=UniformTimedPlan.from_model_json((FR/"reference/reference_model.json").read_text());refhead={1:math.pi/2+math.atan2(1,2),2:math.pi/2,3:math.pi/2};records=[]
for w in (3.2,2.8,2.6):
 t=time.monotonic();a=audit(ref,w,refhead);assert a.feasible==(w!=2.6);rec=dict(kind="reference",width=w,status=a.status,feasible=a.feasible,reason=a.reason,checks=a.checks,wall_s=time.monotonic()-t);records.append(rec);print(rec,flush=True)
summary=json.loads((FR/"experiment_summary.json").read_text());assert sha(summary["config"])==summary["config_sha256"]
assert summary["config"]["successor_config"]==json.loads(json.dumps(asdict(SuccessorConfig())))
assert summary["config"]["search_config"]==asdict(GraphSearchConfig())
runs=[];models=[]
for p in sorted((FR/"runs").glob("*attempt*.json")):
 d=json.loads(p.read_text());inp=d["request_inputs"];assert sha(inp)==d["request_sha256"]
 occ=grid(d["corridor_width_m"]).occ;assert hashlib.sha256(occ.tobytes()).hexdigest()==inp["grid"]["occupied_sha256"]
 assert inp["successor_config"]==json.loads(json.dumps(asdict(SuccessorConfig()))) and inp["search_config"]==asdict(GraphSearchConfig())
 row=dict(file=p.name,status=d["status"],wall_s=d["wall_time_s"],expansions=d["search"]["expansions"],collision_s=d["search"]["collision_s"])
 if d["status"]=="SUCCESS":
  plan=UniformTimedPlan.from_model_json((R/d["plan_model_file"]).read_text());assert plan.model_sha256()==d["plan_model_sha256"]
  a=audit(plan,d["corridor_width_m"],refhead);assert a.feasible;row.update(reaudit=a.status,checks=a.checks);models.append(plan)
 else:assert d["status"]=="BUDGET_EXHAUSTED" and d["first_failure_layer"]=="budget" and not d["feasible"] and not d["executable"]
 runs.append(row);print(row,flush=True)
assert len(runs)==6 and len(models)==3
# Rebuild the saved route under the actual ROS plan identity and compare with its logged digest.
p=models[0];live=UniformTimedPlan.from_edges(p.edges,SH,plan_id=600000,formation_id="cargo_wedge",model=M,timing_limits=FormationTimingLimits())
meta=json.loads((FR/"rviz/round1/run_meta.json").read_text());rosmatch=live.model_sha256()==meta["plan"]["model_sha256"];assert rosmatch,(live.model_sha256(),meta["plan"]["model_sha256"])
# Only map, rigid model, start/headings and goal enter this independent search.
req=request(2.8);t=time.monotonic()
with patch.object(scene,"reference_plan",side_effect=AssertionError("reference must not enter search")),patch.object(scene,"reference_edges",side_effect=AssertionError("reference must not enter search")):
 fresh=plan_uniform(req)
assert fresh.executable and fresh.status=="SUCCESS",fresh.as_dict()
(OUT/"fresh_tight_plan.json").write_text(fresh.plan.to_model_json());(OUT/"fresh_tight_result.json").write_text(json.dumps(fresh.as_dict(),indent=2));freshrow=dict(status=fresh.status,wall_s=fresh.wall_time_s,expansions=fresh.search.expansions,duration_s=fresh.plan.duration_s,request_sha256=fresh.request_sha256)
# Verify the collision target is genuinely occupied independent of body heading.
neg=FormationPose(12,10,math.pi/2);g=grid(2.8);negative_centres={}
for i,(x,y) in M.centers(neg).items():negative_centres[i]=bool(g.occ[int(y/.05),int(x/.05)])
assert any(negative_centres.values())
# Independently recompute time attribution and the nominal continuous-band radial clearance.
refsetup=sum(x.duration_s for x in ref.phases if x.kind!="motion");plansetup=sum(x.duration_s for x in p.phases if x.kind!="motion")
nominal=1.4-(math.hypot(3+.5/2,.738/2)-2)
metrics=dict(reference_total_s=ref.duration_s,plan_total_s=p.duration_s,extra_duration_s=p.duration_s-ref.duration_s,reference_setup_s=refsetup,plan_setup_s=plansetup,extra_setup_s=plansetup-refsetup,nominal_band_clearance_analytic_m=nominal,nominal_band_after_margin_m=nominal-.05,clearance_is_not_a_raster_distance=True)
result=dict(reference_reaudits=records,saved_runs=runs,fresh_tight_search=freshrow,ROS_identity_rebuilt_digest=live.model_sha256(),ROS_log_digest_matches_rebuilt_route=rosmatch,negative_goal_occupied_vehicle_centres=negative_centres,independent_metrics=metrics)
(OUT/"independent_verification.json").write_text(json.dumps(result,indent=2));print(json.dumps(dict(fresh=freshrow,ROS_digest_match=rosmatch,metrics=metrics),indent=2),flush=True)
