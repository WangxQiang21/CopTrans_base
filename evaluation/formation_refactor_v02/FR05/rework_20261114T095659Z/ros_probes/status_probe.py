import json, math, sys, time
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

records = []
def ts():
    return rospy.Time.now().to_sec()
def on_status(msg):
    try: d = json.loads(msg.data)
    except Exception: return
    records.append(("optimizer", ts(), {k: d.get(k) for k in
        ("node_state","plan_id","model_sha256","formation_id","backend",
         "status","feasible","executable","trajectory_published","reason")}))
def on_playback(msg):
    try: d = json.loads(msg.data)
    except Exception: return
    records.append(("playback", ts(), {k: d.get(k) for k in
        ("state","plan_id","model_sha256","formation_id","backend",
         "executable","audit_state","reason","elapsed_s")}))

x, y, theta, duration, out_path = (float(sys.argv[1]), float(sys.argv[2]),
                                   float(sys.argv[3]), float(sys.argv[4]),
                                   sys.argv[5])
rospy.init_node("fr05_status_probe", anonymous=True)
rospy.Subscriber("/formation/optimizer_status", String, on_status, queue_size=100)
rospy.Subscriber("/formation/playback_status", String, on_playback, queue_size=300)
goal = PoseStamped(); goal.header.frame_id = "map"
goal.pose.position.x, goal.pose.position.y = x, y
goal.pose.orientation.z = math.sin(theta / 2.0)
goal.pose.orientation.w = math.cos(theta / 2.0)
pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
rospy.sleep(0.5)
goal.header.stamp = rospy.Time.now()
pub.publish(goal)
deadline = time.time() + duration
while time.time() < deadline:
    rospy.sleep(0.02)
pub.unregister()
time.sleep(0.5)
with_plan = [r for r in records if isinstance(r[2].get("plan_id"), int)]
plan_id = max(r[2]["plan_id"] for r in with_plan) if with_plan else None
selected = [r for r in records if r[2].get("plan_id") == plan_id]
transitions = []
seen = set()
for kind, t, d in selected:
    key = (kind, d.get("node_state") or d.get("state"), d.get("executable"),
           (d.get("model_sha256") or "")[:8], d.get("reason"))
    if key in seen:
        continue
    seen.add(key)
    transitions.append({"kind": kind, "t": round(t, 6), "doc": d})
hold = [r for r in selected if r[0] == "playback" and r[2].get("state") == "HOLD_END"]
report = {"plan_id": plan_id, "records_total": len(records),
          "selected_total": len(selected), "transitions": transitions,
          "hold_end_documents": len(hold),
          "hold_end_certified": [r[2].get("executable") for r in hold[:6]]}
json.dump(report, open(out_path, "w"), indent=1)
print(json.dumps(report, indent=1)[:4000])
