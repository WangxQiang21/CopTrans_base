from pathlib import Path
import sys,json,threading
from types import SimpleNamespace
from unittest.mock import patch
R=Path.cwd();OUT=Path(__file__).parent;sys.path[:0]=[str(R/"catkin_ws/src/fwis_v3_optimizer/scripts"),str(R/"catkin_ws/src/fwis_v3_optimizer/src")]
import formation_uniform_planner_node as m
rows=[]
def probe(name,state,active_digest,report_digest,feedback):
 n=m.UniformPlannerNode.__new__(m.UniformPlannerNode);n._lock=threading.RLock();n._state=state;n._state_plan=700124;n._active_model_sha256=active_digest;n._active_status_word="FEASIBLE" if active_digest else None;n._active_formation_id="cargo_wedge";n.backend="uniform_v02";n._playback_seen=False;n._published_wall=123. if active_digest else None;events=[]
 n._publish_job_state=lambda *a,**k:events.append(dict(kind="job",state=n._state))
 n._base_status=lambda *a,**kw:kw
 n._publish_status=lambda d:events.append(dict(kind="status",document=d))
 d=dict(state=feedback,plan_id=700124,formation_id=n._active_formation_id,backend=n.backend,executable=False)
 if report_digest!="MISSING":d["model_sha256"]=report_digest
 with patch.object(m.rospy,"logwarn_throttle"):n.on_playback_status(SimpleNamespace(data=json.dumps(d)))
 rows.append(dict(case=name,initial_state=state,published=bool(active_digest),active_model_sha256=active_digest,feedback=d,final_state=n._state,playback_seen=n._playback_seen,events=events))
probe("original_wrong_digest",m.PLAYING,"a"*64,"b"*64,"HOLD_END")
probe("published_matching_digest",m.PLAYING,"a"*64,"a"*64,"HOLD_END")
probe("prepublication_missing_digest",m.PLANNING,None,"MISSING","HOLD_END")
probe("prepublication_null_digest",m.PLANNING,None,None,"HOLD_END")
probe("prepublication_missing_start_digest",m.PLANNING,None,"MISSING","HOLD_START")
(OUT/"feedback_phase_boundary_probe.json").write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
