from pathlib import Path
import sys,time,json,math
R=Path.cwd();OUT=Path(__file__).parent;sys.path[:0]=[str(R/"tests"),str(R/"catkin_ws/src/fwis_v3_optimizer/scripts"),str(R/"catkin_ws/src/fwis_v3_optimizer/src")]
import rospy,rosnode
import ros_uniform_playback_smoke as smoke
from fwis_v3_optimizer.uniform_trajectory import UniformTimedPlan
rospy.init_node("codex_fr05_live_contract_probe",anonymous=True)
s=smoke.Session();t=time.monotonic()
while time.monotonic()-t<45:
 if s.job_states and rospy.has_param("/formation_uniform_planner/goal_xy_tol"):break
 rospy.sleep(.1)
else:raise RuntimeError("demo not ready")
rospy.sleep(.7)
params=rospy.get_param("/formation_uniform_planner");smoke.publish_initial_pose(10,10,0);smoke.publish_goal(10.1,10,.1)
t=time.monotonic()
while time.monotonic()-t<15:
 if s.envelopes:break
 rospy.sleep(.1)
else:raise RuntimeError("no envelope for near-goal probe")
rospy.sleep(1.2)
env=s.envelope_records[-1][1] if isinstance(s.envelope_records[-1],tuple) else None
if env is None:env=list(s.envelopes.values())[-1]
# Session stores envelope messages keyed by plan_id; tolerate a timestamp/message pair.
if isinstance(env,tuple):env=env[-1]
plan=UniformTimedPlan.from_model_json(env.model_json)
record={"request_start":[10,10,0],"request_goal":[10.1,10,.1],"actual_goal":[plan.goal_qf.x,plan.goal_qf.y,plan.goal_qf.psi],"actual_start":[plan.start_qf.x,plan.start_qf.y,plan.start_qf.psi],"duration_s":plan.duration_s,"xy_error_m":math.hypot(plan.goal_qf.x-10.1,plan.goal_qf.y-10),"psi_error_rad":abs(math.atan2(math.sin(plan.goal_qf.psi-.1),math.cos(plan.goal_qf.psi-.1))),"params":params,"statuses":[d for _,d in s.statuses],"job_states":[d for _,d in s.job_states],"playback_statuses":[d for _,d in s.playback],"nodes":rosnode.get_node_names(),"master_system_state":rospy.get_master().getSystemState()}
(OUT/"live_contract_probe.json").write_text(json.dumps(record,indent=2));print(json.dumps({k:record[k] for k in ("request_goal","actual_goal","duration_s","xy_error_m","psi_error_rad")},indent=2))
