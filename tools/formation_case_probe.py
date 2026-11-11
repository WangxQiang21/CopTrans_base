#!/usr/bin/env python3
"""Record one RViz-equivalent cargo-goal case end to end (T0 evidence tool).

The probe reproduces exactly what the RViz tools publish -- a
``PoseWithCovarianceStamped`` on ``/initialpose`` and a ``PoseStamped`` on
``/move_base_simple/goal`` -- so a failure can be replayed from a file instead
of clicking the same pixel twice.  It never plans, retimes or evaluates; it only
publishes inputs and records every formation topic plus a stage attribution.

Usage (inside the demo's ROS environment)::

    python3 tools/formation_case_probe.py --out CASE_DIR \
        --initial 10 10 0 --goal 13 10 0 [--timeout 60] [--no-initial]

Outputs in CASE_DIR:
    inputs.json     cargo start/goal, wall times, ROS master, parameter dump
    topics.jsonl    every received message (latched ones flagged)
    verdict.json    terminal status + first stage that broke the contract
    trajectory.json final FormationTimedTrajectory, if one was published
"""
import argparse
import json
import math
import os
import time

import genpy
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from fwis_v3_msgs.msg import FormationPlanRequest, FormationTimedTrajectory

TOPICS = {
    "/formation/plan_request": FormationPlanRequest,
    "/formation/status": String,
    "/formation/optimizer_status": String,
    "/formation/quality_report": String,
    "/formation/playback_status": String,
    "/formation/planner_status": String,
}
TERMINAL = ("FEASIBLE", "INVALID", "REJECTED")


def to_plain(value):
    if isinstance(value, (genpy.Time, genpy.Duration)):
        return value.to_sec()
    if isinstance(value, (list, tuple)):
        return [to_plain(item) for item in value]
    if hasattr(value, "__slots__"):
        return {slot: to_plain(getattr(value, slot)) for slot in value.__slots__}
    if isinstance(value, (int, float, str, bool)) or value is None:
        return value
    if isinstance(value, bytes):
        return "<%d bytes>" % len(value)
    return str(value)


class CaseProbe(object):
    def __init__(self, out_dir, timeout):
        self.out_dir = out_dir
        self.timeout = timeout
        self.records = []
        self.goal_time = None
        self.plan_id = None
        self.trajectory = None
        self.marker_count = None
        os.makedirs(out_dir, exist_ok=True)
        self.initial_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1)
        for topic, msg_type in TOPICS.items():
            rospy.Subscriber(topic, msg_type, self._make_cb(topic),
                             queue_size=50)
        rospy.Subscriber("/formation/timed_trajectory",
                         FormationTimedTrajectory, self._on_trajectory,
                         queue_size=2)
        rospy.Subscriber("/formation/markers", MarkerArray, self._on_markers,
                         queue_size=1)

    # ---- recording ----------------------------------------------------
    def _make_cb(self, topic):
        def callback(message):
            self._record(topic, message)
        return callback

    def _record(self, topic, message):
        plain = to_plain(message)
        if topic in ("/formation/status", "/formation/optimizer_status",
                     "/formation/quality_report", "/formation/playback_status",
                     "/formation/planner_status"):
            try:
                plain = json.loads(message.data)
            except (ValueError, AttributeError):
                plain = {"raw": str(message.data)}
        self.records.append({
            "wall_t": time.time(),
            "topic": topic,
            "before_goal": self.goal_time is None,
            "data": plain,
        })
        if (topic == "/formation/plan_request" and self.goal_time is not None
                and self.plan_id is None):
            self.plan_id = int(message.plan_id)

    def _on_trajectory(self, message):
        if self.goal_time is None:
            return
        self._record("/formation/timed_trajectory", {
            "plan_id": int(message.plan_id),
            "vehicle_count": len(message.vehicles),
        })
        if message.vehicles:
            self.trajectory = to_plain(message)

    def _on_markers(self, message):
        self.marker_count = len(message.markers)

    # ---- inputs -------------------------------------------------------
    def wait_ready(self):
        deadline = time.time() + 15.0
        while time.time() < deadline and not rospy.is_shutdown():
            if (self.goal_pub.get_num_connections() >= 1
                    and self.initial_pub.get_num_connections() >= 1):
                return True
            rospy.sleep(0.1)
        return False

    def send_initial(self, pose):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = rospy.Time.now()
        message.pose.pose.position.x = pose[0]
        message.pose.pose.position.y = pose[1]
        message.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        message.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        self.initial_pub.publish(message)
        rospy.sleep(0.5)

    def send_goal(self, pose):
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = rospy.Time.now()
        message.pose.position.x = pose[0]
        message.pose.position.y = pose[1]
        message.pose.orientation.z = math.sin(pose[2] / 2.0)
        message.pose.orientation.w = math.cos(pose[2] / 2.0)
        self.goal_time = time.time()
        self.goal_pub.publish(message)

    # ---- attribution --------------------------------------------------
    def after_goal(self, topic):
        return [r for r in self.records
                if r["topic"] == topic and not r["before_goal"]]

    def own(self, topic):
        result = []
        for record in self.after_goal(topic):
            plan = record["data"].get("plan_id")
            if self.plan_id is None or plan is None or plan == self.plan_id:
                result.append(record)
        return result

    def verdict(self, elapsed, timed_out):
        optimizer = [r["data"] for r in self.own("/formation/optimizer_status")]
        front = [r["data"] for r in self.own("/formation/status")]
        playback = [r["data"] for r in self.own("/formation/playback_status")]
        terminal = next((d for d in reversed(optimizer)
                         if d.get("status") in TERMINAL), None)
        front_fail = next((d for d in front if d.get("success") is False
                           or d.get("certification") == "FAILED"
                           or "FAILED" in str(d.get("status", ""))), None)
        if self.plan_id is None:
            stage, reason = "NO_REQUEST", "goal bridge never issued a request"
        elif terminal is not None:
            stage = terminal.get("node_state") or terminal.get("status")
            reason = terminal.get("reason")
        elif front_fail is not None:
            stage = "FRONTEND_NO_TERMINAL"
            reason = ("front end reported %s but no terminal optimizer status "
                      "followed" % front_fail)
        elif timed_out:
            last = optimizer[-1] if optimizer else None
            stage = "TIMEOUT_AFTER_%s" % (
                (last or {}).get("node_state", "NO_OPTIMIZER_STATUS"))
            reason = "no terminal status within %.1f s" % self.timeout
        else:
            stage, reason = "UNKNOWN", ""
        return {
            "plan_id": self.plan_id,
            "elapsed_s": round(elapsed, 3),
            "timed_out": timed_out,
            "first_failing_stage": stage,
            "reason": reason,
            "optimizer_status": optimizer,
            "frontend_status": front,
            "playback_status": playback[-3:],
            "trajectory_published": self.trajectory is not None,
            "marker_count": self.marker_count,
            "terminal": terminal,
        }

    def run(self, initial, goal, params):
        if not self.wait_ready():
            raise RuntimeError("goal bridge topics have no subscriber")
        if initial is not None:
            self.send_initial(initial)
        self.send_goal(goal)
        deadline = time.time() + self.timeout
        timed_out = True
        while time.time() < deadline and not rospy.is_shutdown():
            for record in self.own("/formation/optimizer_status"):
                if record["data"].get("status") in TERMINAL:
                    timed_out = False
                    break
            if not timed_out:
                break
            rospy.sleep(0.05)
        rospy.sleep(1.0)
        verdict = self.verdict(time.time() - self.goal_time, timed_out)
        self._write("inputs.json", {
            "initial_cargo_pose": initial,
            "goal_cargo_pose": goal,
            "ros_master_uri": os.environ.get("ROS_MASTER_URI"),
            "parameters": params,
            "timeout_s": self.timeout,
        })
        with open(os.path.join(self.out_dir, "topics.jsonl"), "w") as handle:
            for record in self.records:
                handle.write(json.dumps(record) + "\n")
        self._write("verdict.json", verdict)
        if self.trajectory is not None:
            self._write("trajectory.json", self.trajectory)
        return verdict

    def _write(self, name, payload):
        with open(os.path.join(self.out_dir, name), "w") as handle:
            json.dump(payload, handle, indent=1, sort_keys=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--initial", nargs=3, type=float)
    parser.add_argument("--goal", nargs=3, type=float)
    parser.add_argument("--replay", help="inputs.json of a recorded case; "
                        "its cargo start/goal are republished verbatim")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--param", action="append", default=[],
                        help="ros parameter names to snapshot")
    args = parser.parse_args()
    rospy.init_node("formation_case_probe", anonymous=True)
    if args.replay:
        with open(args.replay) as handle:
            recorded = json.load(handle)
        initial = recorded.get("initial_cargo_pose")
        goal = recorded.get("goal_cargo_pose")
        if goal is None:
            parser.error("replay file has no goal_cargo_pose")
    else:
        if args.goal is None:
            parser.error("--goal is required unless --replay is given")
        initial, goal = args.initial, args.goal
    params = {}
    for name in args.param:
        try:
            params[name] = rospy.get_param(name)
        except Exception as exc:  # recorded, not fatal
            params[name] = "unavailable: %s" % exc
    probe = CaseProbe(args.out, args.timeout)
    verdict = probe.run(initial, goal, params)
    print(json.dumps({k: verdict[k] for k in (
        "plan_id", "elapsed_s", "first_failing_stage", "reason",
        "trajectory_published", "marker_count")}, sort_keys=True))
    return 0 if (verdict["terminal"] or {}).get("status") == "FEASIBLE" else 3


if __name__ == "__main__":
    raise SystemExit(main())
