#!/usr/bin/env python3
"""T1 cargo smoke: one authoritative contract end to end.

This test drives the real user entry point, ``run_demo.sh``, on a private
ROS master and the 30x30 m empty map.  It checks the T1 contract rather than
full motion planning:

* launch, goal-bridge request, playback markers, TF and the footprint tool
  all use the same ``transport_formation.yaml`` (ids, wedge offsets, body
  dimensions, heading policy, free goal heading);
* a manual start previews three rigid bodies at the configured policy
  headings, and a manual reset clears an earlier executable trajectory;
* a legal request whose body headings differ from each other and from the
  cargo heading (the analytic +90 deg A-orbit baseline) reaches FEASIBLE;
* a subsequent bridge goal reads one synchronized, fresh playback TF
  snapshot: its start is the displayed terminal pose and its per-vehicle
  start headings are the actual displayed headings, not re-applied policy;
* a goal that the current motion model cannot start (theta_i=0, in-place
  +90 deg) fails explicitly and is *not* rejected by the retired
  ``CARGO_ROTATION_REQUIRES_GENERAL_4WIS`` / ``CARGO_HEADING_MISMATCH``
  lock; three static bodies remain and no executable path is claimed.

Usage (from the repository root)::

    python3 tests/ros_cargo_smoke.py \
        --output evaluation/wedge_fix_2026-11-11/T1/smoke.json \
        --log-dir evaluation/wedge_fix_2026-11-11/T1/runtime

The private master default is 11447; protected demo ports 11311/11312/11320
are refused.  The script only stops the processes it started.
"""
import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from ros_formation_smoke import sourced_environment, prepare_python_path  # noqa: E402
from transport_footprint import footprint_metrics  # noqa: E402

CONFIG = (ROOT / "catkin_ws/src/fwis_v3_demo/config/"
          "transport_formation.yaml")
EMPTY_MAP = ROOT / "evaluation/wedge_fix_2026-11-11/maps/empty30"
PROTECTED_PORTS = (11311, 11312, 11320)
E05_PLAN_ID = 700000
STATIC_BODY_REASON = "STATIC"


class SmokeError(RuntimeError):
    """A smoke-test assertion or startup failure."""


def utc_now():
    return datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z +
               quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y +
                     quaternion.z * quaternion.z))


def close(first, second, tolerance=1e-6):
    return abs(first - second) <= tolerance


def close_angle(first, second, tolerance=1e-6):
    return abs(wrap(first - second)) <= tolerance


def port_open(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.2)
        return sock.connect_ex(("127.0.0.1", port)) == 0


def wait_port(port, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if port_open(port):
            return
        time.sleep(0.05)
    raise SmokeError("timeout waiting for {}".format(description))


def wait_port_closed(port, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not port_open(port):
            return True
        time.sleep(0.1)
    return False


def write_json(path, document):
    text = json.dumps(document, indent=2, sort_keys=False,
                      allow_nan=False) + "\n"
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def request_snapshot(message, wall):
    return {
        "wall": wall,
        "plan_id": int(message.plan_id),
        "formation_id": message.formation_id,
        "frame_id": message.header.frame_id,
        "formation_start": [
            float(message.formation_start.x),
            float(message.formation_start.y),
            float(message.formation_start.theta)],
        "formation_goal": [
            float(message.formation_goal.x),
            float(message.formation_goal.y),
            float(message.formation_goal.theta)],
        "allow_reverse": bool(message.allow_reverse),
        "vehicles": [{
            "vehicle_id": int(spec.vehicle_id),
            "offset": [float(spec.offset_in_formation.x),
                       float(spec.offset_in_formation.y)],
            "start_center": [float(spec.start_center.x),
                             float(spec.start_center.y),
                             float(spec.start_center.theta)],
            "goal_center": [float(spec.goal_center.x),
                            float(spec.goal_center.y),
                            float(spec.goal_center.theta)],
            "goal_heading_free": bool(spec.goal_heading_free),
            "body_length_m": float(spec.body_length_m),
            "body_width_m": float(spec.body_width_m),
        } for spec in message.vehicles],
    }


class Observations:
    def __init__(self):
        self.requests = []
        self.optimizer = []
        self.quality = []
        self.playback = []
        self.markers = []
        self.trajectories = []
        self.tf = {}
        self.tf_updates = 0

    @staticmethod
    def parse(message):
        try:
            document = json.loads(message.data)
        except (TypeError, ValueError):
            return {"parse_error": str(message.data)[:200]}
        return document if isinstance(document, dict) else {
            "parse_error": "non-object JSON"}

    def add_request(self, message):
        self.requests.append((time.time(), request_snapshot(
            message, time.time())))
        del self.requests[:-100]

    def add_optimizer(self, message):
        self.optimizer.append((time.time(), self.parse(message)))
        del self.optimizer[:-300]

    def add_quality(self, message):
        self.quality.append((time.time(), self.parse(message)))
        del self.quality[:-100]

    def add_playback(self, message):
        self.playback.append((time.time(), self.parse(message)))
        del self.playback[:-4000]

    def add_markers(self, message):
        self.markers.append((time.time(), message))
        del self.markers[:-300]

    def add_trajectory(self, message):
        self.trajectories.append((time.time(), message))
        del self.trajectories[:-20]

    def add_tf(self, message):
        self.tf_updates += 1
        received = time.time()
        for transform in message.transforms:
            child = transform.child_frame_id
            if not child.startswith("formation_"):
                continue
            self.tf[child] = {
                "parent": transform.header.frame_id,
                "stamp": transform.header.stamp.to_sec(),
                "received": received,
                "translation": [
                    float(transform.transform.translation.x),
                    float(transform.transform.translation.y)],
                "yaw": quaternion_yaw(transform.transform.rotation),
            }

    def terminal(self, plan_id, after):
        for wall, document in self.optimizer:
            if (wall >= after and document.get("plan_id") == plan_id
                    and document.get("status") in
                    ("FEASIBLE", "INVALID", "REJECTED")):
                return wall, document
        return None

    def hold_end(self, plan_id, after):
        for wall, document in self.playback:
            if (wall >= after and document.get("plan_id") == plan_id
                    and document.get("state") == "HOLD_END"
                    and document.get("terminal") is True):
                return wall, document
        return None

    def latest_hold(self, after):
        for wall, document in reversed(self.playback):
            if (wall >= after and document.get("state") == "STATIC"):
                return wall, document
        return None

    def latest_markers(self, after):
        for wall, array in reversed(self.markers):
            if wall >= after:
                return wall, array
        return None

    def brief(self):
        return {
            "last_requests": [
                {"plan_id": item.get("plan_id"),
                 "start": item.get("formation_start"),
                 "goal": item.get("formation_goal")}
                for _, item in self.requests[-3:]],
            "last_optimizer": [
                {"plan_id": item.get("plan_id"),
                 "status": item.get("status"),
                 "reason": item.get("reason")}
                for _, item in self.optimizer[-3:]],
            "last_playback": [
                {"plan_id": item.get("plan_id"),
                 "state": item.get("state"),
                 "reason": item.get("reason")}
                for _, item in self.playback[-3:]],
        }


def wait_for(predicate, label, timeout, process, observations):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value is not None and value is not False:
            return value
        if process.poll() is not None:
            raise SmokeError(
                "run_demo.sh exited while waiting for {}; see {}".format(
                    label, observations))
        time.sleep(0.05)
    raise SmokeError(
        "timeout after {:.1f} s waiting for {}; observed={}".format(
            timeout, label, json.dumps(observations.brief(), sort_keys=True)))


def check(summary, name, ok, detail):
    ok = bool(ok)
    summary["checks"][name] = {"ok": ok, "detail": str(detail)}
    if not ok:
        raise SmokeError("{}: {}".format(name, detail))


def body_geometry(array, formation_id, vehicle_ids):
    """Extract OBB centre, heading and edge lengths from one marker array."""
    found = {}
    for vehicle_id in vehicle_ids:
        namespace = "formation/{}/vehicle_{}".format(
            formation_id, vehicle_id)
        markers = [marker for marker in array.markers
                   if marker.ns == namespace and marker.action == 0]
        strips = [marker for marker in markers if marker.type == 4]
        arrows = [marker for marker in markers if marker.type == 0]
        if len(strips) != 1 or len(arrows) != 1:
            raise SmokeError(
                "vehicle {} marker set is not one OBB + one arrow: {}".format(
                    vehicle_id, [(marker.type, marker.id) for marker in markers]))
        corners = [(float(point.x), float(point.y))
                   for point in strips[0].points]
        if len(corners) != 5:
            raise SmokeError(
                "vehicle {} OBB has {} points, expected 4+closing".format(
                    vehicle_id, len(corners)))
        centre = (sum(point[0] for point in corners[:4]) / 4.0,
                  sum(point[1] for point in corners[:4]) / 4.0)
        edges = [math.hypot(second[0] - first[0], second[1] - first[1])
                 for first, second in zip(corners, corners[1:])]
        arrow = arrows[0].points
        if len(arrow) != 2:
            raise SmokeError("vehicle {} heading arrow has {} points".format(
                vehicle_id, len(arrow)))
        heading = math.atan2(arrow[1].y - arrow[0].y,
                             arrow[1].x - arrow[0].x)
        found[vehicle_id] = {
            "centre": centre, "heading": heading, "edges": edges}
    return found


def expected_centre(formation_pose, offset):
    x, y, psi = formation_pose
    c, s = math.cos(psi), math.sin(psi)
    return (x + c * offset[0] - s * offset[1],
            y + s * offset[0] + c * offset[1])


def same_pose(actual, expected, position_tolerance=1e-6,
              angle_tolerance=1e-6):
    return (math.hypot(actual[0] - expected[0],
                       actual[1] - expected[1]) <= position_tolerance
            and close_angle(actual[2], expected[2], angle_tolerance))


def send_initial(publisher, x, y, theta):
    import rospy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.pose.pose.position.x = float(x)
    message.pose.pose.position.y = float(y)
    message.pose.pose.orientation.z = math.sin(theta / 2.0)
    message.pose.pose.orientation.w = math.cos(theta / 2.0)
    publisher.publish(message)


def make_goal(x, y, theta):
    import rospy
    from geometry_msgs.msg import PoseStamped
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.pose.position.x = float(x)
    message.pose.position.y = float(y)
    message.pose.orientation.z = math.sin(theta / 2.0)
    message.pose.orientation.w = math.cos(theta / 2.0)
    return message


def make_plan_request(config, plan_id, start, goal, headings):
    import rospy
    from geometry_msgs.msg import Point, Pose2D
    from fwis_v3_msgs.msg import FormationPlanRequest, FormationVehicleSpec
    request = FormationPlanRequest()
    request.header.frame_id = "map"
    request.header.stamp = rospy.Time.now()
    request.plan_id = int(plan_id)
    request.formation_id = config["formation_id"]
    request.formation_start = Pose2D(x=start[0], y=start[1],
                                     theta=start[2])
    request.formation_goal = Pose2D(x=goal[0], y=goal[1], theta=goal[2])
    request.allow_reverse = bool(config["allow_reverse"])
    for vehicle_id, offset in zip(config["vehicle_ids"], config["offsets"]):
        spec = FormationVehicleSpec()
        spec.vehicle_id = int(vehicle_id)
        spec.offset_in_formation = Point(x=offset[0], y=offset[1], z=0.0)
        start_centre = expected_centre(start, offset)
        goal_centre = expected_centre(goal, offset)
        spec.start_center = Pose2D(x=start_centre[0], y=start_centre[1],
                                   theta=headings[vehicle_id])
        spec.goal_center = Pose2D(x=goal_centre[0], y=goal_centre[1],
                                  theta=goal[2])
        spec.goal_heading_free = bool(config["goal_heading_free"])
        spec.body_length_m = float(config["body_length_m"])
        spec.body_width_m = float(config["body_width_m"])
        request.vehicles.append(spec)
    return request


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=11447)
    parser.add_argument(
        "--output",
        default="evaluation/wedge_fix_2026-11-11/T1/smoke.json")
    parser.add_argument(
        "--log-dir", default="evaluation/wedge_fix_2026-11-11/T1/runtime")
    args = parser.parse_args(argv)
    started = time.time()
    output = Path(args.output).resolve()
    log_dir = Path(args.log_dir).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    summary = {
        "schema": "ros_cargo_smoke_t1/v1",
        "started_at": utc_now(),
        "finished_at": None,
        "duration_s": None,
        "pass": False,
        "port": args.port,
        "command": [
            "bash", "run_demo.sh", "--map", str(EMPTY_MAP),
            "--scale", "1.0", "auto_goal:=false", "rviz:=false",
            "enable_nlp:=false"],
        "config": {
            "path": str(CONFIG),
            "sha256": hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
            "formation_id": config["formation_id"],
            "vehicle_ids": config["vehicle_ids"],
            "offsets": config["offsets"],
            "body_length_m": config["body_length_m"],
            "body_width_m": config["body_width_m"],
            "allow_reverse": config["allow_reverse"],
            "goal_heading_free": config["goal_heading_free"],
            "initial_heading_policy": config["initial_heading_policy"],
        },
        "checks": {},
        "observed": {},
        "errors": [],
        "cleanup": {},
        "logs": {},
    }
    run_demo = None
    master = None
    master_log = None
    demo_log = None
    rospy = None
    observations = None
    exit_codes = {}
    try:
        if args.port in PROTECTED_PORTS:
            raise SmokeError(
                "port {} is reserved for the interactive demo".format(
                    args.port))
        if port_open(args.port):
            raise SmokeError(
                "private master port {} is already in use".format(args.port))
        if not EMPTY_MAP.is_dir():
            raise SmokeError("empty map missing: {}".format(EMPTY_MAP))
        environment = sourced_environment()
        environment.update({
            "ROS_MASTER_URI": "http://127.0.0.1:{}".format(args.port),
            "ROS_HOSTNAME": "127.0.0.1",
            "ROS_HOME": str(log_dir / "ros_home"),
            "ROS_LOG_DIR": str(log_dir / "ros_logs"),
            "PYTHONUNBUFFERED": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
        })
        os.environ.update({
            "ROS_MASTER_URI": environment["ROS_MASTER_URI"],
            "ROS_HOME": environment["ROS_HOME"],
            "ROS_LOG_DIR": environment["ROS_LOG_DIR"],
        })
        prepare_python_path(environment)

        import rospy as imported_rospy
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        from visualization_msgs.msg import MarkerArray
        from fwis_v3_msgs.msg import (
        FormationPlanRequest, FormationTimedTrajectory)
        rospy = imported_rospy

        master_log = (log_dir / "roscore.log").open("w")
        master = subprocess.Popen(
            ["roscore", "-p", str(args.port)], env=environment,
            stdout=master_log, stderr=subprocess.STDOUT,
            start_new_session=True)
        wait_port(args.port, 20.0, "private roscore")
        demo_log = (log_dir / "run_demo.log").open("w")
        demo_command = [
            "bash", str(ROOT / "run_demo.sh"), "--map", str(EMPTY_MAP),
            "--scale", "1.0", "auto_goal:=false", "rviz:=false",
            "enable_nlp:=false"]
        run_demo = subprocess.Popen(
            demo_command, cwd=str(ROOT), env=environment,
            stdout=demo_log, stderr=subprocess.STDOUT,
            start_new_session=True)
        check(summary, "real_entry_point_started",
              run_demo.poll() is None,
              "run_demo.sh pid={}".format(run_demo.pid))

        rospy.init_node("cargo_smoke_t1", anonymous=True,
                        disable_signals=True)
        observations = Observations()
        rospy.Subscriber("/formation/plan_request", FormationPlanRequest,
                         observations.add_request, queue_size=20)
        rospy.Subscriber("/formation/optimizer_status", String,
                         observations.add_optimizer, queue_size=100)
        rospy.Subscriber("/formation/quality_report", String,
                         observations.add_quality, queue_size=50)
        rospy.Subscriber("/formation/playback_status", String,
                         observations.add_playback, queue_size=200)
        rospy.Subscriber("/formation/markers", MarkerArray,
                         observations.add_markers, queue_size=100)
        rospy.Subscriber("/formation/timed_trajectory",
                         FormationTimedTrajectory,
                         observations.add_trajectory, queue_size=20)
        rospy.Subscriber("/tf", TFMessage, observations.add_tf,
                         queue_size=100)
        initial_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1)
        goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1)
        request_pub = rospy.Publisher(
            "/formation/plan_request", FormationPlanRequest, queue_size=1)
        wait_for(
            lambda: initial_pub.get_num_connections() > 0
            and goal_pub.get_num_connections() > 0,
            "goal bridge input subscribers", 120.0, run_demo, observations)
        # formation_planner, formation_optimizer, formation_playback and this
        # harness all subscribe before a one-shot request is published.
        wait_for(
            lambda: request_pub.get_num_connections() >= 4,
            "formation plan_request subscribers ready", 30.0, run_demo,
            observations)

        def bridge_case(label, goal_pose, expected_start):
            after = time.time()
            goal_pub.publish(make_goal(*goal_pose))
            item = wait_for(
                lambda: next(
                    ((wall, request) for wall, request
                     in observations.requests
                     if wall >= after
                     and close(request["formation_goal"][0], goal_pose[0],
                               1e-6)
                     and close(request["formation_goal"][1], goal_pose[1],
                               1e-6)), None),
                label + " request", 10.0, run_demo, observations)[1]
            check(summary, label + "_request_start",
                  math.hypot(
                      item["formation_start"][0] - expected_start[0],
                      item["formation_start"][1] - expected_start[1]) <= 1e-3
                  and close_angle(item["formation_start"][2],
                                  expected_start[2], 1e-3),
                  "start={} expected={}".format(
                      item["formation_start"], expected_start))
            terminal = wait_for(
                lambda: observations.terminal(item["plan_id"], after),
                label + " terminal status", 60.0, run_demo, observations)[1]
            check(summary, label + "_feasible",
                  terminal.get("status") == "FEASIBLE"
                  and terminal.get("trajectory_published") is True,
                  "status={} reason={}".format(
                      terminal.get("status"), terminal.get("reason")))
            trajectory = wait_for(
                lambda: next(
                    (message for _, message in observations.trajectories
                     if int(message.plan_id) == item["plan_id"]
                     and message.vehicles and message.trajectories), None),
                label + " trajectory", 10.0, run_demo, observations)
            hold = wait_for(
                lambda: observations.hold_end(item["plan_id"], after),
                label + " HOLD_END",
                float(trajectory.times_from_start[-1].to_sec()) + 20.0,
                run_demo, observations)[1]
            terminal_vehicles = {
                int(vehicle["vehicle_id"]): vehicle
                for vehicle in hold.get("vehicles", [])}
            goal_errors = []
            for vehicle_id, offset in zip(config["vehicle_ids"],
                                          config["offsets"]):
                expected = expected_centre(goal_pose, offset)
                actual = terminal_vehicles[vehicle_id]["center"]
                goal_errors.append(math.hypot(
                    float(actual[0]) - expected[0],
                    float(actual[1]) - expected[1]))
            check(summary, label + "_terminal_goal",
                  max(goal_errors) <= 1e-3,
                  "terminal centres={} max goal error={:.3g}".format(
                      [terminal_vehicles[vehicle_id]["center"]
                       for vehicle_id in config["vehicle_ids"]],
                      max(goal_errors)))
            return item, terminal, hold

        # ---- config / footprint / launch consistency ------------------
        metrics = footprint_metrics(CONFIG)
        check(summary, "one_config_for_area_tool",
              metrics["config_sha256"] == summary["config"]["sha256"]
              and metrics["formation_id"] == config["formation_id"]
              and metrics["vehicle_ids"] == config["vehicle_ids"]
              and metrics["offsets_m"] == config["offsets"],
              "footprint tool read the same YAML: sha256={}".format(
                  metrics["config_sha256"]))
        check(summary, "new_wedge_pair_distances",
              close(metrics["pair_distances_m"]["1-2"], math.sqrt(2.0))
              and close(metrics["pair_distances_m"]["1-3"], math.sqrt(2.0))
              and close(metrics["pair_distances_m"]["2-3"], 2.0),
              "pair distances={}".format(metrics["pair_distances_m"]))
        check(summary, "new_wedge_area_contract",
              close(metrics["common_heading_aabb_m"][0], 1.738)
              and close(metrics["common_heading_aabb_m"][1], 2.5)
              and close(metrics["common_heading_hull_area_m2"], 3.345)
              and close(metrics["heading_free_envelope_m2"],
                        6.5662084606402)
              and close(metrics["required_axis_scale_factor"], 5.0)
              and close(metrics["conservative_disk_scale_factor"],
                        5.782856135530851),
              "AABB={} hull={} envelope={} axis={} disk={}".format(
                  metrics["common_heading_aabb_m"],
                  metrics["common_heading_hull_area_m2"],
                  metrics["heading_free_envelope_m2"],
                  metrics["required_axis_scale_factor"],
                  metrics["conservative_disk_scale_factor"]))

        # ---- manual start preview ------------------------------------
        start_time = time.time()
        send_initial(initial_pub, 10.0, 10.0, 0.0)
        static_item = wait_for(
            lambda: next(
                ((wall, document) for wall, document
                 in observations.playback
                 if wall >= start_time
                 and document.get("state") == "STATIC"
                 and document.get("reason") == "START"
                 and document.get("plan_id") is None), None),
            "manual start static preview", 10.0, run_demo, observations)
        def static_markers_ready():
            item = observations.latest_markers(start_time)
            if item is None:
                return None
            if not any(marker.ns == "formation/cargo_wedge/vehicle_1"
                       and marker.type == 4 and marker.action == 0
                       for marker in item[1].markers):
                return None
            return item

        marker_item = wait_for(
            static_markers_ready, "three static vehicle OBBs",
            10.0, run_demo, observations)
        static_markers = body_geometry(
            marker_item[1], config["formation_id"], config["vehicle_ids"])
        static_errors = []
        for vehicle_id, offset in zip(config["vehicle_ids"],
                                      config["offsets"]):
            expected = expected_centre((10.0, 10.0, 0.0), offset)
            actual = static_markers[vehicle_id]["centre"]
            static_errors.append(math.hypot(actual[0] - expected[0],
                                            actual[1] - expected[1]))
            static_errors.append(abs(wrap(
                static_markers[vehicle_id]["heading"] - 0.0)))
            edges = static_markers[vehicle_id]["edges"]
            static_errors.append(abs(edges[0] - config["body_length_m"]))
            static_errors.append(abs(edges[1] - config["body_width_m"]))
        check(summary, "static_preview_uses_config_offsets_and_headings",
              max(static_errors) <= 1e-6,
              "max marker/config error={:.3g} m or rad; offsets={}".format(
                  max(static_errors), config["offsets"]))
        prefix = "formation_{}".format(config["formation_id"])
        expected_frames = {prefix + "_reference"} | {
            "{}_vehicle_{}_base_link".format(prefix, vehicle_id)
            for vehicle_id in config["vehicle_ids"]}

        def fresh_tf():
            now = time.time()
            if not all(frame in observations.tf for frame in expected_frames):
                return None
            if any(now - observations.tf[frame]["received"] > 0.6
                   for frame in expected_frames):
                return None
            return dict(observations.tf)

        tf_item = wait_for(fresh_tf, "fresh static playback TF",
                           10.0, run_demo, observations)
        tf_errors = []
        reference = tf_item[prefix + "_reference"]
        tf_errors.extend([
            reference["translation"][0] - 10.0,
            reference["translation"][1] - 10.0,
            wrap(reference["yaw"])])
        for vehicle_id in config["vehicle_ids"]:
            frame = "{}_vehicle_{}_base_link".format(prefix, vehicle_id)
            tf_errors.append(
                tf_item[frame]["translation"][0] -
                static_markers[vehicle_id]["centre"][0])
            tf_errors.append(
                tf_item[frame]["translation"][1] -
                static_markers[vehicle_id]["centre"][1])
            tf_errors.append(wrap(
                tf_item[frame]["yaw"] -
                static_markers[vehicle_id]["heading"]))
        check(summary, "static_tf_matches_markers_and_config",
              max(abs(value) for value in tf_errors) <= 1e-6,
              "TF reference=(10,10,0), body frames match marker OBBs; "
              "max error={:.3g}".format(
                  max(abs(value) for value in tf_errors)))
        summary["observed"]["static_preview"] = {
            "reference": [10.0, 10.0, 0.0],
            "vehicles": [
                {"vehicle_id": vehicle_id,
                 "centre": list(static_markers[vehicle_id]["centre"]),
                 "heading": static_markers[vehicle_id]["heading"]}
                for vehicle_id in config["vehicle_ids"]],
            "marker_count": len(marker_item[1].markers),
        }

        # Consume the manual-start override with a real bridge goal.  Later
        # continuous goals must read the actual playback state instead of
        # reusing a stale manual override.
        initial_forward_request, initial_forward_terminal, \
            initial_forward_hold = bridge_case(
                "E01_forward_initial", (13.0, 10.0, 0.0),
                (10.0, 10.0, 0.0))

        # ---- E05: legal distinct headings, +90 deg cargo rotation ----
        e05_time = time.time()
        e05_start = (10.0, 10.0, 0.0)
        e05_goal = (10.0, 10.0, math.pi / 2.0)
        e05_headings = {1: math.pi / 2.0, 2: math.pi, 3: 0.0}
        e05_request = make_plan_request(
            config, E05_PLAN_ID, e05_start, e05_goal, e05_headings)
        request_pub.publish(e05_request)
        e05_request_item = wait_for(
            lambda: next(
                ((wall, item) for wall, item in observations.requests
                 if wall >= e05_time and item["plan_id"] == E05_PLAN_ID),
                None), "E05 request on the wire", 10.0, run_demo,
            observations)
        e05_request = e05_request_item[1]
        requested_headings = [item["start_center"][2] for item in
                              e05_request["vehicles"]]
        check(summary, "e05_request_distinct_theta_from_psi",
              requested_headings == [math.pi / 2.0, math.pi, 0.0]
              and len({round(wrap(angle), 9)
                       for angle in requested_headings}) == 3
              and any(abs(wrap(angle - e05_start[2])) > 0.5
                      for angle in requested_headings),
              "theta_i={} psi_F={}".format(
                  requested_headings, e05_start[2]))
        e05_terminal = wait_for(
            lambda: observations.terminal(E05_PLAN_ID, e05_time),
            "E05 terminal optimizer status", 60.0, run_demo, observations)[1]
        check(summary, "e05_feasible_without_cargo_heading_lock",
              e05_terminal.get("status") == "FEASIBLE"
              and e05_terminal.get("trajectory_published") is True
              and e05_terminal.get("reason") not in
              ("CARGO_HEADING_MISMATCH",
               "CARGO_ROTATION_REQUIRES_GENERAL_4WIS"),
              "status={} reason={}".format(
                  e05_terminal.get("status"), e05_terminal.get("reason")))
        e05_trajectory = wait_for(
            lambda: next(
                (message for _, message in observations.trajectories
                 if int(message.plan_id) == E05_PLAN_ID
                 and message.vehicles and message.trajectories),
                None), "E05 published trajectory", 10.0, run_demo,
            observations)
        check(summary, "e05_trajectory_identity",
              e05_trajectory.header.frame_id == "map"
              and e05_trajectory.formation_id == config["formation_id"]
              and len(e05_trajectory.vehicles) == 3,
              "plan={} formation={} vehicles={}".format(
                  e05_trajectory.plan_id, e05_trajectory.formation_id,
                  len(e05_trajectory.vehicles)))
        e05_hold = wait_for(
            lambda: observations.hold_end(E05_PLAN_ID, e05_time),
            "E05 HOLD_END",
            float(e05_trajectory.times_from_start[-1].to_sec()) + 20.0,
            run_demo, observations)[1]
        e05_terminal_headings = {
            int(item["vehicle_id"]): float(item["theta"])
            for item in e05_hold.get("vehicles", [])}
        expected_tangent = {1: math.pi, 2: 3.0 * math.pi / 2.0,
                            3: math.pi / 2.0}
        check(summary, "e05_terminal_headings_are_the_orbit_tangents",
              all(close_angle(e05_terminal_headings[vehicle_id],
                              expected_tangent[vehicle_id], 1e-3)
                  for vehicle_id in expected_tangent)
              and len({round(wrap(value), 6)
                       for value in e05_terminal_headings.values()}) == 3,
              "terminal theta={} expected={}".format(
                  e05_terminal_headings, expected_tangent))
        def reference_at(goal, tolerance=1e-3):
            reference = observations.tf.get(prefix + "_reference")
            if reference is None or time.time() - reference["received"] > 0.6:
                return None
            if (math.hypot(
                    reference["translation"][0] - goal[0],
                    reference["translation"][1] - goal[1]) <= tolerance
                    and close_angle(reference["yaw"], goal[2], tolerance)):
                return reference
            return None

        e05_reference = wait_for(
            lambda: reference_at(e05_goal), "E05 terminal cargo TF",
            5.0, run_demo, observations)
        check(summary, "e05_terminal_cargo_pose",
              math.hypot(
                  e05_reference["translation"][0] - e05_goal[0],
                  e05_reference["translation"][1] - e05_goal[1]) <= 1e-3
              and close_angle(e05_reference["yaw"], e05_goal[2], 1e-3),
              "TF reference=({:.6f}, {:.6f}, {:.6f}) expected={}".format(
                  e05_reference["translation"][0],
                  e05_reference["translation"][1],
                  e05_reference["yaw"], e05_goal))
        summary["observed"]["e05"] = {
            "plan_id": E05_PLAN_ID,
            "request": e05_request,
            "terminal_optimizer_status": e05_terminal,
            "terminal_headings": e05_terminal_headings,
        }

        # ---- continuous bridge goal: actual playback headings ---------
        continuous_time = time.time()
        continuous_goal = (10.0, 13.0, math.pi / 2.0)
        goal_pub.publish(make_goal(*continuous_goal))
        continuous_request = wait_for(
            lambda: next(
                ((wall, item) for wall, item in observations.requests
                 if wall >= continuous_time
                 and close(item["formation_goal"][0], continuous_goal[0],
                           1e-6)
                 and close(item["formation_goal"][1], continuous_goal[1],
                           1e-6)),
                None), "continuous bridge request", 10.0, run_demo,
            observations)[1]
        check(summary, "continuous_request_starts_at_displayed_terminal",
              close(continuous_request["formation_start"][0], 10.0, 1e-6)
              and close(continuous_request["formation_start"][1], 10.0, 1e-6)
              and close_angle(continuous_request["formation_start"][2],
                              math.pi / 2.0, 1e-3),
              "start={} expected displayed E05 terminal".format(
                  continuous_request["formation_start"]))
        request_headings = {
            item["vehicle_id"]: item["start_center"][2]
            for item in continuous_request["vehicles"]}
        check(summary, "continuous_request_keeps_real_body_headings",
              all(close_angle(request_headings[vehicle_id],
                              e05_terminal_headings[vehicle_id], 1e-3)
                  for vehicle_id in request_headings)
              and any(abs(wrap(request_headings[vehicle_id] -
                               continuous_goal[2])) > 0.5
                      for vehicle_id in request_headings),
              "bridge start theta={} displayed terminal={} (cargo-heading "
              "policy would give all {:.3f})".format(
                  request_headings, e05_terminal_headings,
                  continuous_goal[2]))
        continuous_plan = continuous_request["plan_id"]
        continuous_terminal = wait_for(
            lambda: observations.terminal(continuous_plan, continuous_time),
            "continuous terminal status", 60.0, run_demo, observations)[1]
        check(summary, "continuous_translation_feasible",
              continuous_terminal.get("status") == "FEASIBLE"
              and continuous_terminal.get("trajectory_published") is True,
              "status={} reason={}".format(
                  continuous_terminal.get("status"),
                  continuous_terminal.get("reason")))
        continuous_trajectory = wait_for(
            lambda: next(
                (message for _, message in observations.trajectories
                 if int(message.plan_id) == continuous_plan
                 and message.vehicles and message.trajectories), None),
            "continuous published trajectory", 10.0, run_demo, observations)
        continuous_hold = wait_for(
            lambda: observations.hold_end(continuous_plan, continuous_time),
            "continuous HOLD_END",
            float(continuous_trajectory.times_from_start[-1].to_sec()) + 20.0,
            run_demo, observations)[1]
        continuous_end_headings = {
            int(item["vehicle_id"]): float(item["theta"])
            for item in continuous_hold.get("vehicles", [])}
        check(summary, "continuous_plan_keeps_body_headings",
              all(close_angle(continuous_end_headings[vehicle_id],
                              request_headings[vehicle_id], 1e-3)
                  for vehicle_id in request_headings),
              "end theta={} start theta={}".format(
                  continuous_end_headings, request_headings))
        continuous_reference = wait_for(
            lambda: reference_at(continuous_goal),
            "continuous terminal cargo TF", 5.0, run_demo, observations)
        check(summary, "continuous_terminal_cargo_pose",
              math.hypot(
                  continuous_reference["translation"][0] -
                  continuous_goal[0],
                  continuous_reference["translation"][1] -
                  continuous_goal[1]) <= 1e-3
              and close_angle(continuous_reference["yaw"],
                              continuous_goal[2], 1e-3),
              "TF reference=({:.6f}, {:.6f}, {:.6f})".format(
                  continuous_reference["translation"][0],
                  continuous_reference["translation"][1],
                  continuous_reference["yaw"]))
        summary["observed"]["continuous_after_e05"] = {
            "plan_id": continuous_plan,
            "request": continuous_request,
            "terminal_optimizer_status": continuous_terminal,
            "end_headings": continuous_end_headings,
        }

        # ---- manual reset is a new experiment -------------------------
        reset_time = time.time()
        send_initial(initial_pub, 10.0, 10.0, 0.0)
        reset_item = wait_for(
            lambda: next(
                ((wall, document) for wall, document
                 in observations.playback
                 if wall >= reset_time
                 and document.get("state") == "STATIC"
                 and document.get("reason") == "START"
                 and document.get("plan_id") is None
                 and document.get("trajectory_published") is False),
                None), "manual reset static preview", 10.0, run_demo,
            observations)[1]
        reset_markers = wait_for(
            lambda: observations.latest_markers(reset_time)
            if observations.latest_markers(reset_time) is not None
            and sum(marker.type == 4 and marker.action == 0
                    and "vehicle_" in marker.ns
                    for marker in observations.latest_markers(
                        reset_time)[1].markers) == 3
            else None, "reset static bodies", 10.0, run_demo, observations)[1]
        reset_bodies = body_geometry(
            reset_markers, config["formation_id"], config["vehicle_ids"])
        reset_heading_error = max(
            abs(wrap(reset_bodies[vehicle_id]["heading"]))
            for vehicle_id in config["vehicle_ids"])
        check(summary, "manual_reset_reinitialises_policy_headings",
              reset_heading_error <= 1e-6
              and reset_item.get("executable") is False,
              "reset reason=START, policy headings=0; max heading error="
              "{:.3g}".format(reset_heading_error))

        # ---- E06: independent headings are not silently faked --------
        e06_time = time.time()
        e06_goal = (10.0, 10.0, math.pi / 2.0)
        goal_pub.publish(make_goal(*e06_goal))
        e06_request = wait_for(
            lambda: next(
                ((wall, item) for wall, item in observations.requests
                 if wall >= e06_time
                 and close(item["formation_goal"][0], e06_goal[0], 1e-6)
                 and close(item["formation_goal"][1], e06_goal[1], 1e-6)
                 and close(item["formation_goal"][2], e06_goal[2], 1e-6)),
                None), "E06 bridge request", 10.0, run_demo,
            observations)[1]
        e06_terminal = wait_for(
            lambda: observations.terminal(e06_request["plan_id"], e06_time),
            "E06 terminal optimizer status", 60.0, run_demo, observations)[1]
        check(summary, "e06_rejected_by_independent_heading_check",
              e06_terminal.get("status") in ("INVALID", "REJECTED")
              and e06_terminal.get("trajectory_published") is False
              and e06_terminal.get("reason") ==
              "HEADING_BOUNDARY_MISMATCH",
              "status={} reason={}".format(
                  e06_terminal.get("status"),
                  e06_terminal.get("reason")))
        e06_failure = wait_for(
            lambda: next(
                ((wall, document) for wall, document
                 in observations.playback
                 if wall >= e06_time
                 and document.get("plan_id") == e06_request["plan_id"]
                 and document.get("state") == "STATIC"
                 and "HEADING_BOUNDARY_MISMATCH" in
                 str(document.get("reason", ""))), None),
            "E06 static failure display", 10.0, run_demo, observations)
        e06_bodies = wait_for(
            lambda: observations.latest_markers(e06_failure[0])
            if observations.latest_markers(e06_failure[0]) is not None
            and sum(marker.type == 4 and marker.action == 0
                    and "vehicle_" in marker.ns
                    for marker in observations.latest_markers(
                        e06_failure[0])[1].markers) == 3
            else None, "E06 three static bodies", 10.0, run_demo,
            observations)
        check(summary, "e06_keeps_static_bodies_without_old_path",
              e06_failure[1].get("executable") is False
              and sum(marker.type == 4 and marker.action == 0
                      and "vehicle_" in marker.ns
                      for marker in e06_bodies[1].markers) == 3,
              "state={} reason={} static vehicle OBBs={}".format(
                  e06_failure[1].get("state"),
                  e06_failure[1].get("reason"),
                  sum(marker.type == 4 and marker.action == 0
                      and "vehicle_" in marker.ns
                      for marker in e06_bodies[1].markers)))
        retired = ("CARGO_ROTATION_REQUIRES_GENERAL_4WIS",
                   "CARGO_HEADING_MISMATCH")
        check(summary, "retired_cargo_heading_lock_not_default",
              all(retired[0] not in str(document.get("reason", ""))
                  and retired[1] not in str(document.get("reason", ""))
                  for _, document in observations.optimizer),
              "no optimizer status cites {}".format(retired))
        summary["observed"]["e06_failure"] = {
            "plan_id": e06_request["plan_id"],
            "request": e06_request,
            "terminal_optimizer_status": e06_terminal,
            "playback_status": e06_failure[1],
        }

        # ---- basic forward and lateral goals after the failure --------
        forward_request, forward_terminal, forward_hold = bridge_case(
            "E01_forward", (13.0, 10.0, 0.0), (10.0, 10.0, 0.0))
        lateral_request, lateral_terminal, lateral_hold = bridge_case(
            "E02_lateral", (13.0, 13.0, 0.0), (13.0, 10.0, 0.0))
        summary["observed"]["post_failure_cases"] = {
            "E01": {"plan_id": forward_request["plan_id"],
                    "terminal_status": forward_terminal,
                    "end_headings": {
                        int(item["vehicle_id"]): item["theta"]
                        for item in forward_hold.get("vehicles", [])}},
            "E02": {"plan_id": lateral_request["plan_id"],
                    "terminal_status": lateral_terminal,
                    "request_start_headings": [
                        item["start_center"][2]
                        for item in lateral_request["vehicles"]],
                    "end_headings": {
                        int(item["vehicle_id"]): item["theta"]
                        for item in lateral_hold.get("vehicles", [])}},
        }

        # ---- request / marker / TF / config five-way agreement --------
        for label, request, terminal, hold in (
                ("E05", e05_request, e05_terminal, e05_hold),
                ("continuous", continuous_request, continuous_terminal,
                 continuous_hold),
                ("E01", forward_request, forward_terminal, forward_hold),
                ("E02", lateral_request, lateral_terminal, lateral_hold)):
            check(summary, label + "_request_uses_config",
                  request["formation_id"] == config["formation_id"]
                  and [item["vehicle_id"] for item in
                       request["vehicles"]] == config["vehicle_ids"]
                  and [item["offset"] for item in
                       request["vehicles"]] == config["offsets"]
                  and all(close(item["body_length_m"],
                                config["body_length_m"])
                          and close(item["body_width_m"],
                                    config["body_width_m"])
                          and item["goal_heading_free"] is
                          config["goal_heading_free"]
                          for item in request["vehicles"])
                  and request["allow_reverse"] == config["allow_reverse"],
                  "ids/offsets/dims/heading policy match transport YAML")

        summary["observed"]["tf_frame_count"] = len(observations.tf)
        summary["observed"]["marker_arrays"] = len(observations.markers)
        summary["observed"]["optimizer_status_count"] = len(
            observations.optimizer)
        summary["observed"]["playback_status_count"] = len(
            observations.playback)
        summary["observed"]["peak_tf_age_s"] = max(
            time.time() - item["received"] for item in observations.tf.values())
        summary["pass"] = True
    except Exception as exc:  # noqa: BLE001 - recorded as evidence
        summary["errors"].append({
            "type": type(exc).__name__,
            "detail": str(exc),
        })
        if observations is not None:
            summary["observed"]["failure_snapshot"] = observations.brief()
    finally:
        if rospy is not None:
            try:
                rospy.signal_shutdown("cargo smoke complete")
            except Exception:
                pass
        for process, label in ((run_demo, "run_demo"), (master, "roscore")):
            if process is None:
                continue
            exit_codes[label] = None
            if process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGINT)
                except (ProcessLookupError, PermissionError, OSError):
                    pass
            try:
                process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                except (ProcessLookupError, PermissionError, OSError):
                    pass
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    pass
            exit_codes[label] = process.poll()
        for handle in (demo_log, master_log):
            if handle is not None:
                handle.flush()
                handle.close()
        released = wait_port_closed(args.port, 10.0)
        summary["cleanup"] = {
            "exit_codes": exit_codes,
            "port_released": released,
        }
        summary["logs"] = {
            "run_demo": str(log_dir / "run_demo.log"),
            "roscore": str(log_dir / "roscore.log"),
            "ros_logs": str(log_dir / "ros_logs"),
        }
    summary["finished_at"] = utc_now()
    summary["duration_s"] = round(time.time() - started, 3)
    if not summary["checks"]:
        summary["pass"] = False
    write_json(output, summary)
    print("ros_cargo_smoke: {} ({} checks, {})".format(
        "PASS" if summary["pass"] else "FAIL",
        len(summary["checks"]), output))
    for name, item in summary["checks"].items():
        print("  [{}] {}: {}".format(
            "ok" if item["ok"] else "FAIL", name, item["detail"][:300]))
    if summary["errors"]:
        print("errors:", file=sys.stderr)
        for error in summary["errors"]:
            print("  {}: {}".format(error["type"], error["detail"]),
                  file=sys.stderr)
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())