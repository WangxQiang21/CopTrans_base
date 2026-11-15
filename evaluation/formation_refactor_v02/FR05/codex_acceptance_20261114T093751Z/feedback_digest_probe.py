from pathlib import Path
import sys,json,threading
from types import SimpleNamespace
R=Path.cwd();OUT=Path(__file__).parent;sys.path[:0]=[str(R/"catkin_ws/src/fwis_v3_optimizer/scripts"),str(R/"catkin_ws/src/fwis_v3_optimizer/src")]
from formation_uniform_planner_node import UniformPlannerNode,PLAYING
n=UniformPlannerNode.__new__(UniformPlannerNode);n._lock=threading.RLock();n._state=PLAYING;n._state_plan=700124;n._active_model_sha256="a"*64;n._active_status_word="FEASIBLE";n._active_formation_id="formation_three";events=[]
n._publish_job_state=lambda *a,**k:events.append(dict(kind="job",state=n._state))
n._base_status=lambda *a,**kw:kw
n._publish_status=lambda d:events.append(dict(kind="status",document=d))
feedback={"state":"HOLD_END","plan_id":700124,"formation_id":"formation_three","backend":"uniform_v02","model_sha256":"b"*64,"executable":False}
n.on_playback_status(SimpleNamespace(data=json.dumps(feedback)))
row={"case":"terminal_feedback_wrong_model_hash","expected_active_digest":n._active_model_sha256,"feedback":feedback,"resulting_job_state":n._state,"events":events};(OUT/"feedback_digest_probe.json").write_text(json.dumps(row,indent=2));print(json.dumps(row,indent=2))
