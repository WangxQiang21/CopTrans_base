#!/usr/bin/env python3
"""S10 ROS integration smoke test for the multi-vehicle formation stack.

The test owns a private ROS master (default port 11399; the protected demo
ports 11311/11312/11320 are rejected), starts exactly one formation optimizer
and one formation playback node, and never starts a single-vehicle planner.
It then checks:

* the optimizer publishes a message-valid common-time trajectory only after
  the independent S9 evaluator reports FEASIBLE;
* every playback sample is interpolated on the trajectory's shared
  ``header.stamp``/``time_from_start`` clock and all vehicles stay rigid;
* vehicle marker namespaces contain both formation id and vehicle id, marker
  ids are unique, OBB/heading/mode bands/history/links/status are present,
  and the status label is ``FEASIBLE`` only with audited publication;
* the terminal frame is held exactly at the final sample;
* an invalid request clears the trajectory, a synthetic unaudited
  trajectory is labelled ``GEOMETRY_ONLY``, and an empty trajectory clears
  every marker;
* the private master port is released and the child nodes are stopped.

Usage (from the repository root)::

    /usr/bin/python3 tests/ros_formation_smoke.py

The script sources /opt/ros/noetic/setup.bash and catkin_ws/devel/setup.bash
for its child processes itself, so the calling shell does not need ROS on
PYTHONPATH.  It writes a strict-JSON report to
``evaluation/formation_s10/smoke/ros_formation_smoke.json`` unless --output
is given.
"""
import argparse
from collections import OrderedDict
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
OPTIMIZER_SCRIPT = (
    ROOT / "catkin_ws/src/fwis_v3_optimizer/scripts/"
    "formation_optimizer_node.py")
PLAYBACK_SCRIPT = (
    ROOT / "catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py")
SYSTEM_SETUP = Path("/opt/ros/noetic/setup.bash")
DEVEL_SETUP = ROOT / "catkin_ws/devel/setup.bash"

PROTECTED_PORTS = (11311, 11312, 11320)
FORMATION_ID = "smoke_wedge"
PLAN_ID = 424242
PROBE_PLAN_ID = 777001
PROBE_FORMATION_ID = "geometry_only_probe"
VEHICLE_IDS = (1, 2, 3)
OFFSETS = ((0.0, 0.0), (0.0, 1.2), (0.0, -1.2))
GOAL_X = 2.5
BODY_LENGTH_M = 0.738
BODY_WIDTH_M = 0.500
MODE_COLORS = {
    0: (0.10, 0.45, 1.00, 0.95),
    1: (0.10, 0.78, 0.30, 0.95),
    2: (1.00, 0.40, 0.10, 0.95),
}

rospy = None
String = None
TFMessage = None
OccupancyGrid = None
Pose2D = None
Point = None
Marker = None
MarkerArray = None
FormationPlanRequest = None
FormationPlanSeed = None
FormationTimedTrajectory = None
FormationTimedPoint = None
FormationVehicleSpec = None
FormationVehicleTrajectory = None
FormationPrimitive = None


class SmokeError(RuntimeError):
    """A smoke-test assertion or startup failure."""


def _utc_now():
    return datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def _json_safe(value):
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    return repr(value)


def _write_json(path, document):
    text = json.dumps(
        _json_safe(document), indent=2, sort_keys=False,
        allow_nan=False) + "\n"
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)
    return text


def _check(summary, name, ok, detail=""):
    ok = bool(ok)
    summary["checks"][name] = {"ok": ok, "detail": str(detail)}
    if not ok:
        summary["errors"].append({"check": name, "detail": str(detail)})
    return ok


def _port_open(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.2)
        return sock.connect_ex(("127.0.0.1", port)) == 0


def _wait_port(port, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if _port_open(port):
            return True
        time.sleep(0.05)
    raise SmokeError(
        "timeout waiting for {} on port {}".format(description, port))


def _wait_port_closed(port, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not _port_open(port):
            return True
        time.sleep(0.1)
    return False


def sourced_environment():
    """Return a child-process environment with ROS and catkin workspaces."""
    if not SYSTEM_SETUP.is_file():
        raise SmokeError(
            "ROS Noetic setup not found at {}".format(SYSTEM_SETUP))
    if not DEVEL_SETUP.is_file():
        raise SmokeError(
            "catkin devel setup not found at {}; build the workspace "
            "before running this smoke test".format(DEVEL_SETUP))
    command = (
        "set -e; source {} >/dev/null 2>&1; source {} >/dev/null 2>&1; "
        "env -0".format(
            shlex.quote(str(SYSTEM_SETUP)), shlex.quote(str(DEVEL_SETUP))))
    result = subprocess.run(
        ["bash", "-c", command], stdout=subprocess.PIPE,
        stderr=subprocess.PIPE)
    if result.returncode != 0:
        raise SmokeError(
            "failed to source ROS setup (exit {}): {}".format(
                result.returncode,
                result.stderr.decode("utf-8", "replace")[-2000:]))
    environment = {}
    for entry in result.stdout.split(b"\0"):
        if b"=" not in entry:
            continue
        key, value = entry.split(b"=", 1)
        environment[key.decode("utf-8", "replace")] = value.decode(
            "utf-8", "replace")
    return environment


def prepare_python_path(environment):
    for entry in reversed(
            environment.get("PYTHONPATH", "").split(os.pathsep)):
        if entry and entry not in sys.path:
            sys.path.insert(0, entry)


def load_ros_modules():
    global rospy, String, TFMessage, OccupancyGrid, Pose2D, Point
    global Marker, MarkerArray
    global FormationPlanRequest, FormationPlanSeed
    global FormationTimedTrajectory, FormationTimedPoint
    global FormationVehicleSpec, FormationVehicleTrajectory
    global FormationPrimitive
    try:
        import rospy as imported_rospy
        from std_msgs.msg import String as ImportedString
        from tf2_msgs.msg import TFMessage as ImportedTFMessage
        from nav_msgs.msg import OccupancyGrid as ImportedOccupancyGrid
        from geometry_msgs.msg import Pose2D as ImportedPose2D
        from geometry_msgs.msg import Point as ImportedPoint
        from visualization_msgs.msg import Marker as ImportedMarker
        from visualization_msgs.msg import MarkerArray as ImportedMarkerArray
        from fwis_v3_msgs.msg import (
            FormationPlanRequest as ImportedPlanRequest,
            FormationPlanSeed as ImportedPlanSeed,
            FormationTimedPoint as ImportedTimedPoint,
            FormationTimedTrajectory as ImportedTimedTrajectory,
            FormationVehicleSpec as ImportedVehicleSpec,
            FormationVehicleTrajectory as ImportedVehicleTrajectory,
            FormationPrimitive as ImportedPrimitive,
        )
    except Exception as exc:
        raise SmokeError(
            "cannot import ROS Python modules with {!r}: {!r}. Run with "
            "/usr/bin/python3 after building the workspace; the script "
            "sources setuptools itself for child processes.".format(
                sys.executable, exc)) from exc
    rospy = imported_rospy
    String = ImportedString
    TFMessage = ImportedTFMessage
    OccupancyGrid = ImportedOccupancyGrid
    Pose2D = ImportedPose2D
    Point = ImportedPoint
    Marker = ImportedMarker
    MarkerArray = ImportedMarkerArray
    FormationPlanRequest = ImportedPlanRequest
    FormationPlanSeed = ImportedPlanSeed
    FormationTimedTrajectory = ImportedTimedTrajectory
    FormationTimedPoint = ImportedTimedPoint
    FormationVehicleSpec = ImportedVehicleSpec
    FormationVehicleTrajectory = ImportedVehicleTrajectory
    FormationPrimitive = ImportedPrimitive


class NodeProcess:
    """One child process group with captured stdout/stderr."""

    def __init__(self, label, command, environment, log_dir):
        self.label = label
        self.path = Path(log_dir) / "{}.txt".format(label)
        self.handle = open(self.path, "w", encoding="utf-8")
        self.process = subprocess.Popen(
            command, cwd=str(ROOT), env=environment,
            stdout=self.handle, stderr=subprocess.STDOUT,
            start_new_session=True)
        self.exit_code = None

    def alive(self):
        return self.process.poll() is None

    def stop(self):
        if self.handle.closed:
            return self.exit_code
        if self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            except (ProcessLookupError, PermissionError, OSError):
                pass
        try:
            self.process.wait(timeout=6.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError, OSError):
                pass
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                pass
        self.exit_code = self.process.poll()
        self.handle.flush()
        self.handle.close()
        return self.exit_code

    def tail(self, limit=4000):
        try:
            text = self.path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return ""
        return text[-limit:]

    def describe(self):
        code = self.process.poll()
        return {
            "pid": int(self.process.pid),
            "running": code is None,
            "exit_code": code,
            "log": str(self.path),
        }


def ensure_alive(nodes):
    for node in nodes:
        if not node.alive():
            raise SmokeError(
                "{} exited before the smoke scenario completed "
                "(exit code {}); log tail:\n{}".format(
                    node.label, node.process.poll(), node.tail()))


def wait_for(predicate, timeout, description, nodes=(), observations=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ensure_alive(nodes)
        value = predicate()
        if value is not None and value is not False:
            return value
        time.sleep(0.05)
    detail = "" if observations is None else observations.brief()
    raise SmokeError(
        "timeout after {:.1f} s waiting for {}{}".format(
            timeout, description, detail))


class Observations:
    """Thread-safe-enough callback buffers for one smoke scenario."""

    def __init__(self):
        self.optimizer = []
        self.quality = []
        self.trajectories = []
        self.markers = []
        self.playback = []
        self.tf_frames = OrderedDict()
        self.tf_updates = 0

    @staticmethod
    def _parse(message):
        try:
            document = json.loads(message.data)
        except (TypeError, ValueError) as exc:
            return {"parse_error": repr(exc), "raw": str(message.data)[:500]}
        return document if isinstance(document, dict) else {
            "parse_error": "non-object JSON", "raw": str(message.data)[:500]}

    def add_optimizer_status(self, message):
        self.optimizer.append((time.time(), self._parse(message)))
        del self.optimizer[:-200]

    def add_quality(self, message):
        self.quality.append((time.time(), self._parse(message)))
        del self.quality[:-200]

    def add_trajectory(self, message):
        self.trajectories.append((time.time(), message))
        del self.trajectories[:-20]

    def add_markers(self, message):
        self.markers.append((time.time(), message))
        del self.markers[:-500]

    def add_playback_status(self, message):
        self.playback.append((time.time(), self._parse(message)))
        del self.playback[:-1000]

    def add_tf(self, message):
        self.tf_updates += 1
        for transform in message.transforms:
            child = transform.child_frame_id
            if not child:
                continue
            self.tf_frames[child] = {
                "parent": transform.header.frame_id,
                "stamp": time.time(),
                "translation": (
                    float(transform.transform.translation.x),
                    float(transform.transform.translation.y)),
                "rotation_z": float(transform.transform.rotation.z),
                "rotation_w": float(transform.transform.rotation.w),
            }

    def latest_optimizer(self, after=None):
        for wall, document in reversed(self.optimizer):
            if after is None or wall >= after:
                return wall, document
        return None

    def latest_quality(self, after=None):
        for wall, document in reversed(self.quality):
            if after is None or wall >= after:
                return wall, document
        return None

    def latest_playback(self, after=None):
        for wall, document in reversed(self.playback):
            if after is None or wall >= after:
                return wall, document
        return None

    def latest_markers(self, after=None):
        for wall, message in reversed(self.markers):
            if after is None or wall >= after:
                return wall, message
        return None

    def brief(self):
        optimizer = []
        for _, document in self.optimizer[-3:]:
            optimizer.append({
                "plan_id": document.get("plan_id"),
                "node_state": document.get("node_state"),
                "status": document.get("status"),
                "reason": document.get("reason"),
                "published": document.get("trajectory_published"),
            })
        playback = []
        for _, document in self.playback[-3:]:
            playback.append({
                "plan_id": document.get("plan_id"),
                "state": document.get("state"),
                "elapsed_s": document.get("elapsed_s"),
                "cleared": document.get("cleared"),
                "reason": document.get("reason"),
            })
        return " (optimizer={} playback={})".format(optimizer, playback)


def set_parameters():
    rospy.set_param("/formation_optimizer/enable_nlp", False)
    rospy.set_param("/formation_optimizer/max_candidates", 2)
    rospy.set_param("/formation_optimizer/max_schedule_combinations", 8)
    rospy.set_param("/formation_optimizer/max_schedules_per_block", 2)
    rospy.set_param("/formation_optimizer/optimizer_time_limit", 1.0)
    rospy.set_param("/formation_optimizer/require_collision_precheck", True)
    rospy.set_param("/formation_optimizer/collision_margin", 0.05)
    rospy.set_param("/formation_optimizer/sweep_spacing", 0.01)
    rospy.set_param("/formation_optimizer/publication_dt", 0.02)
    rospy.set_param("/formation_optimizer/planning_dt", 0.02)
    rospy.set_param("/formation_playback/frame_id", "map")
    rospy.set_param("/formation_playback/rate", 25.0)
    rospy.set_param("/formation_playback/publish_tf", True)


def make_map():
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.info.resolution = 0.05
    message.info.width = 320
    message.info.height = 240
    message.info.origin.position.x = -8.0
    message.info.origin.position.y = -6.0
    message.info.origin.position.z = 0.0
    message.info.origin.orientation.w = 1.0
    message.data = [0] * (320 * 240)
    return message


def make_request(plan_id=PLAN_ID, formation_id=FORMATION_ID,
                 nan_start=False):
    message = FormationPlanRequest()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.plan_id = int(plan_id)
    message.formation_id = formation_id
    message.formation_start = Pose2D(x=0.0, y=0.0, theta=0.0)
    message.formation_goal = Pose2D(x=GOAL_X, y=0.0, theta=0.0)
    message.allow_reverse = False
    for vehicle_id, (offset_x, offset_y) in zip(VEHICLE_IDS, OFFSETS):
        spec = FormationVehicleSpec()
        spec.vehicle_id = int(vehicle_id)
        spec.offset_in_formation = Point(x=offset_x, y=offset_y, z=0.0)
        spec.start_center = Pose2D(
            x=offset_x, y=offset_y, theta=0.0)
        spec.goal_center = Pose2D(
            x=GOAL_X + offset_x, y=offset_y, theta=0.0)
        spec.goal_heading_free = False
        spec.body_length_m = BODY_LENGTH_M
        spec.body_width_m = BODY_WIDTH_M
        message.vehicles.append(spec)
    if nan_start:
        message.vehicles[0].start_center.x = float("nan")
    return message


def make_seed(plan_id=PLAN_ID, formation_id=FORMATION_ID):
    message = FormationPlanSeed()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.plan_id = int(plan_id)
    message.formation_id = formation_id
    message.formation_start = Pose2D(x=0.0, y=0.0, theta=0.0)
    message.formation_goal = Pose2D(x=GOAL_X, y=0.0, theta=0.0)
    primitive = FormationPrimitive()
    primitive.geometry_type = FormationPrimitive.TRANSLATION
    primitive.dx = GOAL_X
    primitive.dy = 0.0
    primitive.dpsi = 0.0
    primitive.pivot_x = 0.0
    primitive.pivot_y = 0.0
    message.primitives.append(primitive)
    return message


def make_geometry_probe():
    message = FormationTimedTrajectory()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.plan_id = PROBE_PLAN_ID
    message.formation_id = PROBE_FORMATION_ID
    spec = FormationVehicleSpec()
    spec.vehicle_id = 1
    spec.offset_in_formation = Point(x=0.0, y=0.0, z=0.0)
    spec.start_center = Pose2D(x=0.0, y=0.0, theta=0.0)
    spec.goal_center = Pose2D(x=0.6, y=0.0, theta=0.0)
    spec.goal_heading_free = False
    spec.body_length_m = BODY_LENGTH_M
    spec.body_width_m = BODY_WIDTH_M
    message.vehicles.append(spec)
    message.formation_poses.append(Pose2D(x=0.0, y=0.0, theta=0.0))
    message.formation_poses.append(Pose2D(x=0.6, y=0.0, theta=0.0))
    message.times_from_start.append(rospy.Duration.from_sec(0.0))
    message.times_from_start.append(rospy.Duration.from_sec(0.6))
    trajectory = FormationVehicleTrajectory()
    trajectory.vehicle_id = 1
    for elapsed, x in ((0.0, 0.0), (0.6, 0.6)):
        point = FormationTimedPoint()
        point.time_from_start = rospy.Duration.from_sec(elapsed)
        point.center_pose = Pose2D(x=x, y=0.0, theta=0.0)
        point.center_speed_mps = 1.0
        point.center_yaw_rate_radps = 0.0
        point.mode = 0
        point.mode_control = 0.0
        trajectory.points.append(point)
    message.trajectories.append(trajectory)
    return message


def make_empty_trajectory():
    message = FormationTimedTrajectory()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.plan_id = 0
    message.formation_id = ""
    return message


def _duration_key(duration):
    return int(duration.secs), int(duration.nsecs)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _vehicle_namespace(formation_id, vehicle_id):
    return "formation/{}/vehicle_{}".format(formation_id, vehicle_id)


def _shared_namespace(formation_id, name):
    return "formation/{}/{}".format(formation_id, name)


def _markers_with_namespace(array, namespace):
    return [marker for marker in array.markers if marker.ns == namespace]


def _find_feasible_status(observations, plan_id):
    for wall, document in observations.optimizer:
        if (document.get("plan_id") == plan_id and
                document.get("status") == "FEASIBLE" and
                document.get("trajectory_published") is True and
                document.get("executable") is True and
                document.get("feasible") is True):
            return wall, document
    return None


def _find_feasible_quality(observations, plan_id):
    for wall, document in observations.quality:
        if (document.get("plan_id") == plan_id and
                document.get("feasible") is True and
                document.get("trajectory_published") is True and
                document.get("independent_audit") is True and
                document.get("collision_certificate") is True):
            return wall, document
    return None


def _find_published_trajectory(observations, plan_id):
    for _, message in reversed(observations.trajectories):
        if (int(message.plan_id) == plan_id and message.vehicles and
                message.trajectories):
            return message
    return None


def _playing_samples(observations, plan_id):
    samples = []
    for wall, document in observations.playback:
        if (document.get("plan_id") == plan_id and
                document.get("state") == "PLAYING" and
                document.get("terminal") is False):
            samples.append((wall, document))
    return samples


def _find_terminal_status(observations, plan_id):
    for wall, document in observations.playback:
        if (document.get("plan_id") == plan_id and
                document.get("state") == "HOLD_END" and
                document.get("terminal") is True):
            return wall, document
    return None


def _find_cleared_status(observations, after):
    for wall, document in reversed(observations.playback):
        if (wall >= after and document.get("state") == "CLEARED" and
                document.get("cleared") is True):
            return wall, document
    return None


def _find_deleteall(observations, after):
    for wall, array in reversed(observations.markers):
        if wall < after:
            continue
        if (len(array.markers) == 1 and
                array.markers[0].action == Marker.DELETEALL):
            return wall, array
    return None


def check_common_time_grid(summary, message):
    times = message.times_from_start
    keys = [_duration_key(value) for value in times]
    strictly_increasing = all(
        second > first for first, second in zip(keys, keys[1:]))
    _check(
        summary, "trajectory_common_time_grid",
        bool(times) and keys[0] == (0, 0) and strictly_increasing and
        all(len(trajectory.points) == len(times)
            for trajectory in message.trajectories) and
        all(_duration_key(point.time_from_start) == keys[index]
            for trajectory in message.trajectories
            for index, point in enumerate(trajectory.points)),
        "{} shared samples, first={} last={}, strictly increasing={}".format(
            len(times), keys[0] if keys else None, keys[-1] if keys else None,
            strictly_increasing))

    finite = True
    modes_ok = True
    for trajectory in message.trajectories:
        for point in trajectory.points:
            pose = point.center_pose
            values = (pose.x, pose.y, pose.theta,
                      point.center_speed_mps,
                      point.center_yaw_rate_radps, point.mode_control)
            finite = finite and all(math.isfinite(value) for value in values)
            modes_ok = modes_ok and point.mode in (0, 1, 2)
    _check(
        summary, "trajectory_finite_modes", finite and modes_ok,
        "all vehicle samples finite; modes in A/O/S")
    return keys


def check_trajectory_geometry(summary, message):
    offsets = {}
    for spec in message.vehicles:
        offsets[int(spec.vehicle_id)] = (
            float(spec.offset_in_formation.x),
            float(spec.offset_in_formation.y))
    _check(
        summary, "trajectory_vehicle_ids",
        set(offsets) == set(VEHICLE_IDS) and
        {int(trajectory.vehicle_id)
         for trajectory in message.trajectories} == set(VEHICLE_IDS),
        "vehicles={} trajectories={}".format(
            sorted(offsets),
            sorted(int(trajectory.vehicle_id)
                   for trajectory in message.trajectories)))
    _check(
        summary, "trajectory_offsets",
        all(abs(offsets.get(vehicle_id, (math.inf, math.inf))[axis] -
                OFFSETS[vehicle_id - 1][axis]) <= 1e-9
            for vehicle_id in VEHICLE_IDS for axis in (0, 1)),
        "offsets={}".format(
            {vehicle_id: offsets.get(vehicle_id)
             for vehicle_id in VEHICLE_IDS}))

    start_error = 0.0
    goal_error = 0.0
    for trajectory in message.trajectories:
        vehicle_id = int(trajectory.vehicle_id)
        offset_x, offset_y = offsets[vehicle_id]
        first = trajectory.points[0].center_pose
        last = trajectory.points[-1].center_pose
        start_error = max(
            start_error, abs(first.x - offset_x), abs(first.y - offset_y),
            abs(_wrap(first.theta)))
        goal_error = max(
            goal_error,
            abs(last.x - (GOAL_X + offset_x)),
            abs(last.y - offset_y), abs(_wrap(last.theta)))
    formation_start = message.formation_poses[0]
    formation_goal = message.formation_poses[-1]
    formation_error = max(
        abs(formation_start.x), abs(formation_start.y),
        abs(_wrap(formation_start.theta)),
        abs(formation_goal.x - GOAL_X), abs(formation_goal.y),
        abs(_wrap(formation_goal.theta)))
    _check(
        summary, "trajectory_rigid_endpoints",
        start_error <= 1e-6 and goal_error <= 1e-6 and
        formation_error <= 1e-6 and
        len(message.formation_poses) == len(message.times_from_start),
        "start_error={:.3g} goal_error={:.3g} formation_error={:.3g}".format(
            start_error, goal_error, formation_error))


def check_playback_samples(summary, samples, trajectory_message):
    times = trajectory_message.times_from_start
    duration = float(times[-1].to_sec())
    first_wall, first = samples[0]
    last_wall, last = samples[-1]
    elapsed_monotonic = all(
        second[1].get("elapsed_s") >= first_sample[1].get("elapsed_s")
        for first_sample, second in zip(samples, samples[1:]))
    elapsed_span = float(last["elapsed_s"]) - float(first["elapsed_s"])
    wall_span = last_wall - first_wall
    clock_ok = abs(elapsed_span - wall_span) <= 0.3
    clamped_ok = all(
        abs(float(document.get("clamped_elapsed_s", -1.0)) -
            min(float(document.get("elapsed_s", 0.0)), duration)) <= 1e-6
        for _, document in samples)
    stamp = trajectory_message.header.stamp
    stamp_ok = all(
        document.get("time_source") == "header_stamp_common_clock" and
        document.get("trajectory_stamp", {}).get("secs") == int(stamp.secs) and
        document.get("trajectory_stamp", {}).get("nsecs") == int(stamp.nsecs)
        for _, document in samples)
    _check(
        summary, "playback_common_clock",
        len(samples) >= 3 and elapsed_monotonic and
        elapsed_span > 0.08 and clock_ok and clamped_ok and stamp_ok,
        "{} PLAYING samples, elapsed {:.3f}->{:.3f} s, wall span {:.3f} s, "
        "stamp=({}, {})".format(
            len(samples), first["elapsed_s"], last["elapsed_s"],
            wall_span, stamp.secs, stamp.nsecs))

    rigid_error = 0.0
    heading_error = 0.0
    movement = {vehicle_id: 0.0 for vehicle_id in VEHICLE_IDS}
    for _, document in samples:
        vehicles = {int(item["vehicle_id"]): item
                    for item in document.get("vehicles", [])}
        reference_center = vehicles[VEHICLE_IDS[0]]["center"]
        for vehicle_id in VEHICLE_IDS:
            center = vehicles[vehicle_id]["center"]
            expected = (
                float(reference_center[0]) +
                OFFSETS[vehicle_id - 1][0] - OFFSETS[0][0],
                float(reference_center[1]) +
                OFFSETS[vehicle_id - 1][1] - OFFSETS[0][1])
            rigid_error = max(
                rigid_error,
                abs(float(center[0]) - expected[0]),
                abs(float(center[1]) - expected[1]))
            heading_error = max(
                heading_error, abs(_wrap(float(vehicles[vehicle_id]["theta"]))))
    first_vehicles = {
        int(item["vehicle_id"]): item for item in first.get("vehicles", [])}
    last_vehicles = {
        int(item["vehicle_id"]): item for item in last.get("vehicles", [])}
    for vehicle_id in VEHICLE_IDS:
        movement[vehicle_id] = math.hypot(
            float(last_vehicles[vehicle_id]["center"][0]) -
            float(first_vehicles[vehicle_id]["center"][0]),
            float(last_vehicles[vehicle_id]["center"][1]) -
            float(first_vehicles[vehicle_id]["center"][1]))
    _check(
        summary, "playback_rigid_centers",
        rigid_error <= 1e-6 and heading_error <= 1e-9 and
        all(value > 1e-3 for value in movement.values()),
        "rigidity_error={:.3g} heading_error={:.3g} movement={}".format(
            rigid_error, heading_error, movement))


def check_terminal(summary, terminal, trajectory_message):
    document = terminal[1]
    times = trajectory_message.times_from_start
    duration = float(times[-1].to_sec())
    clamped = float(document.get("clamped_elapsed_s", -1.0))
    center_error = document.get("terminal_center_error_m")
    heading_error = document.get("terminal_heading_error_rad")
    errors_ok = (isinstance(center_error, (int, float)) and
                 isinstance(heading_error, (int, float)) and
                 float(center_error) <= 1e-6 and
                 float(heading_error) <= 1e-6)
    _check(
        summary, "playback_terminal_hold",
        document.get("terminal") is True and document.get("state") == "HOLD_END"
        and abs(clamped - duration) <= 1e-6 and errors_ok,
        "state={} clamped={:.6f} duration={:.6f} center_error={} "
        "heading_error={}".format(
            document.get("state"), clamped, duration, center_error,
            heading_error))

    point_map = {
        int(item["vehicle_id"]): item for item in document.get("vehicles", [])}
    terminal_error = 0.0
    for trajectory in trajectory_message.trajectories:
        vehicle_id = int(trajectory.vehicle_id)
        last = trajectory.points[-1]
        item = point_map[vehicle_id]
        terminal_error = max(
            terminal_error,
            abs(float(item["center"][0]) - float(last.center_pose.x)),
            abs(float(item["center"][1]) - float(last.center_pose.y)),
            abs(_wrap(float(item["theta"]) - float(last.center_pose.theta))))
    _check(
        summary, "playback_terminal_message_match", terminal_error <= 1e-6,
        "terminal frame matches final common-time sample, error={:.3g}".format(
            terminal_error))
    return document


def check_markers(summary, array, terminal, trajectory_message):
    formation_id = trajectory_message.formation_id
    _check(
        summary, "marker_frame",
        bool(array.markers) and all(
            marker.header.frame_id == "map" for marker in array.markers),
        "{} markers on frame map".format(len(array.markers)))

    namespaces = [marker.ns for marker in array.markers]
    expected_vehicle = {
        vehicle_id: _vehicle_namespace(formation_id, vehicle_id)
        for vehicle_id in VEHICLE_IDS}
    _check(
        summary, "marker_vehicle_namespaces",
        all(expected_vehicle[vehicle_id] in namespaces
            for vehicle_id in VEHICLE_IDS) and
        all(namespace.startswith("formation/{}/".format(formation_id))
            for namespace in namespaces),
        "namespaces={}".format(sorted(set(namespaces))))

    ids = [int(marker.id) for marker in array.markers]
    _check(
        summary, "marker_unique_ids",
        len(ids) == len(set(ids)) and all(marker_id >= 0 for marker_id in ids),
        "{} markers, {} unique ids".format(len(ids), len(set(ids))))

    sample_count = len(trajectory_message.times_from_start)
    points_by_vehicle = {
        int(trajectory.vehicle_id): trajectory.points
        for trajectory in trajectory_message.trajectories}
    terminal_by_vehicle = {
        int(item["vehicle_id"]): item for item in terminal.get("vehicles", [])}
    path_point_map = {}
    obb_errors = []
    heading_errors = []
    color_errors = []
    type_errors = []
    history_errors = []
    link_errors = []
    link_fatal = False
    for vehicle_id in VEHICLE_IDS:
        namespace = expected_vehicle[vehicle_id]
        markers = _markers_with_namespace(array, namespace)
        by_type = {}
        for marker in markers:
            by_type.setdefault(marker.type, []).append(marker)
        if (len(markers) != 5 or
                len(by_type.get(Marker.LINE_STRIP, [])) != 3 or
                len(by_type.get(Marker.ARROW, [])) != 1 or
                len(by_type.get(Marker.TEXT_VIEW_FACING, [])) != 1):
            type_errors.append((vehicle_id, len(markers), {
                int(marker_type): len(items)
                for marker_type, items in by_type.items()}))
            continue
        line_strips = by_type[Marker.LINE_STRIP]
        obb = next((marker for marker in line_strips
                    if len(marker.points) == 5 and
                    abs(marker.scale.x - 0.03) <= 1e-9), None)
        body_path = next((marker for marker in line_strips
                          if len(marker.points) == sample_count and
                          abs(marker.scale.x - 0.05) <= 1e-9), None)
        history = next((marker for marker in line_strips
                        if abs(marker.scale.x - 0.015) <= 1e-9), None)
        if obb is None or body_path is None or history is None:
            type_errors.append(
                (vehicle_id, len(markers), "missing OBB/path/history"))
            continue
        corners = obb.points
        edge_lengths = [
            math.hypot(second.x - first.x, second.y - first.y)
            for first, second in zip(corners, corners[1:])]
        expected_edges = (BODY_LENGTH_M, BODY_WIDTH_M,
                          BODY_LENGTH_M, BODY_WIDTH_M)
        edge_error = max(abs(value - expected) for value, expected in
                         zip(edge_lengths, expected_edges))
        if abs(corners[0].x - corners[-1].x) > 1e-9 or abs(
                corners[0].y - corners[-1].y) > 1e-9:
            edge_error = max(edge_error, 1.0)
        obb_errors.append(edge_error)

        arrow = by_type[Marker.ARROW][0]
        if len(arrow.points) != 2:
            heading_errors.append(math.inf)
        else:
            dx = arrow.points[1].x - arrow.points[0].x
            dy = arrow.points[1].y - arrow.points[0].y
            heading_errors.append(abs(_wrap(
                math.atan2(dy, dx) -
                float(terminal_by_vehicle[vehicle_id]["theta"]))))

        points = points_by_vehicle[vehicle_id]
        if len(body_path.colors) != sample_count:
            color_errors.append((vehicle_id, "color count"))
        else:
            for index, (color, point) in enumerate(
                    zip(body_path.colors, points)):
                expected = MODE_COLORS[int(point.mode)]
                if max(abs(color.r - expected[0]),
                       abs(color.g - expected[1]),
                       abs(color.b - expected[2]),
                       abs(color.a - expected[3])) > 1e-6:
                    color_errors.append((vehicle_id, index, int(point.mode)))
                    break
        path_point_map[vehicle_id] = body_path
        if (len(history.points) != sample_count or len(body_path.points) !=
                sample_count or any(
                    abs(first.x - second.x) > 1e-9 or
                    abs(first.y - second.y) > 1e-9
                    for first, second in zip(history.points,
                                             body_path.points))):
            history_errors.append((vehicle_id, len(history.points)))

    _check(
        summary, "marker_obb_and_heading",
        not type_errors and
        max(obb_errors or [math.inf]) <= 1e-6 and
        max(heading_errors or [math.inf]) <= 1e-6,
        "types={} obb_max_error={} heading_max_error={}".format(
            type_errors, max(obb_errors or [math.inf]),
            max(heading_errors or [math.inf])))
    _check(
        summary, "marker_mode_color_bands",
        not color_errors,
        "every full-path color matches its A/O/S mode; errors={}".format(
            color_errors[:5]))
    _check(
        summary, "marker_history_and_paths",
        not type_errors and not history_errors and
        all(vehicle_id in path_point_map and
            len(path_point_map[vehicle_id].points) == sample_count
            for vehicle_id in VEHICLE_IDS),
        "full path and traveled history contain all shared samples; "
        "errors={}".format(history_errors[:5]))

    links = _markers_with_namespace(
        array, _shared_namespace(formation_id, "links"))
    if len(links) != 1 or links[0].type != Marker.LINE_LIST:
        link_fatal = True
    else:
        link = links[0]
        expected_pairs = ((1, 2), (1, 3), (2, 3))
        if len(link.points) != 2 * len(expected_pairs):
            link_fatal = True
        else:
            centers = {
                int(item["vehicle_id"]): item["center"]
                for item in terminal["vehicles"]}
            for pair_index, (first_id, second_id) in enumerate(expected_pairs):
                first = link.points[2 * pair_index]
                second = link.points[2 * pair_index + 1]
                link_errors.append(max(
                    abs(first.x - float(centers[first_id][0])),
                    abs(first.y - float(centers[first_id][1])),
                    abs(second.x - float(centers[second_id][0])),
                    abs(second.y - float(centers[second_id][1]))))
    _check(
        summary, "marker_intervehicle_links",
        not link_fatal and max(link_errors or [math.inf]) <= 1e-6,
        "pairwise links connect the terminal vehicle centers; errors={}".format(
            link_errors))

    reference = _markers_with_namespace(
        array, _shared_namespace(formation_id, "reference"))
    reference_ok = (
        len(reference) == 3 and
        any(marker.type == Marker.SPHERE for marker in reference) and
        any(marker.type == Marker.TEXT_VIEW_FACING and
            marker.text == "F" for marker in reference))
    trail = next((marker for marker in reference
                  if marker.type == Marker.LINE_STRIP), None)
    if trail is not None:
        goal = trajectory_message.formation_poses[-1]
        reference_ok = reference_ok and len(trail.points) == len(
            trajectory_message.formation_poses) and abs(
                trail.points[-1].x - goal.x) <= 1e-6 and abs(
                trail.points[-1].y - goal.y) <= 1e-6
    _check(
        summary, "marker_reference_point", reference_ok,
        "formation reference sphere/trail/F label at final formation pose")

    status_markers = _markers_with_namespace(
        array, _shared_namespace(formation_id, "status"))
    status_ok = (
        len(status_markers) == 1 and
        status_markers[0].type == Marker.TEXT_VIEW_FACING and
        "FEASIBLE" in status_markers[0].text and
        "GEOMETRY_ONLY" not in status_markers[0].text and
        "plan {}".format(PLAN_ID) in status_markers[0].text and
        "T_end=" in status_markers[0].text and
        "rigidity=" in status_markers[0].text and
        "obstacle=" in status_markers[0].text)
    _check(
        summary, "marker_status_feasible", status_ok,
        "status text={!r}".format(
            "" if not status_markers else status_markers[0].text[:300]))

    namespaces_documented = terminal.get("marker_namespaces") or []
    _check(
        summary, "marker_status_namespaces",
        sorted(namespaces_documented) == sorted(set(namespaces)),
        "playback status recorded {} namespaces".format(
            len(namespaces_documented)))


def check_tf(summary, observations, terminal, trajectory_message):
    expected = {
        "formation_{}_reference".format(FORMATION_ID),
        *{"formation_{}_vehicle_{}_base_link".format(FORMATION_ID, vehicle_id)
          for vehicle_id in VEHICLE_IDS},
    }
    present = set(observations.tf_frames)
    _check(
        summary, "tf_expected_unique_frames",
        expected.issubset(present) and len(expected) == 4 and
        all(observations.tf_frames[frame].get("parent") == "map"
            for frame in expected),
        "expected={} observed_formation={}".format(
            sorted(expected),
            sorted(frame for frame in present
                   if frame.startswith("formation_"))))
    centers = {
        int(item["vehicle_id"]): item["center"]
        for item in terminal.get("vehicles", [])}
    tf_errors = []
    for vehicle_id in VEHICLE_IDS:
        frame = "formation_{}_vehicle_{}_base_link".format(
            FORMATION_ID, vehicle_id)
        if frame not in observations.tf_frames:
            tf_errors.append((frame, "missing"))
            continue
        translation = observations.tf_frames[frame]["translation"]
        tf_errors.append(max(
            abs(translation[0] - float(centers[vehicle_id][0])),
            abs(translation[1] - float(centers[vehicle_id][1]))))
    _check(
        summary, "tf_body_frames_match_terminal",
        not tf_errors or max(float(item) for item in tf_errors) <= 1e-6,
        "formation TF translations match terminal centers; errors={}".format(
            tf_errors))


def run(args):
    started_wall = time.time()
    output_path = (ROOT / args.output).resolve() if not Path(
        args.output).is_absolute() else Path(args.output).resolve()
    log_dir = (ROOT / args.log_dir).resolve() if not Path(
        args.log_dir).is_absolute() else Path(args.log_dir).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    summary = OrderedDict([
        ("schema", "ros_formation_smoke/v1"),
        ("started_at", _utc_now()),
        ("finished_at", None),
        ("duration_s", None),
        ("pass", False),
        ("port", int(args.port)),
        ("scenario", {
            "plan_id": PLAN_ID,
            "formation_id": FORMATION_ID,
            "vehicle_ids": list(VEHICLE_IDS),
            "offsets": [list(offset) for offset in OFFSETS],
            "goal": [GOAL_X, 0.0, 0.0],
            "map": {"width": 320, "height": 240, "resolution": 0.05,
                    "origin": [-8.0, -6.0], "obstacles": []},
            "enable_nlp": False,
        }),
        ("environment", {}),
        ("checks", OrderedDict()),
        ("observed", {}),
        ("errors", []),
        ("processes", []),
        ("cleanup", {"stopped": False, "port_released": False}),
        ("logs", {}),
    ])
    nodes = []

    try:
        if args.port in PROTECTED_PORTS:
            raise SmokeError(
                "port {} is reserved for the interactive demo; choose a "
                "private port".format(args.port))
        if _port_open(args.port):
            raise SmokeError(
                "private master port {} is already in use; refusing to "
                "attach to an unknown ROS master".format(args.port))
        _check(summary, "private_master_port",
               True, "port {} is private and free".format(args.port))

        environment = sourced_environment()
        ros_home = log_dir / "ros_home"
        ros_home.mkdir(parents=True, exist_ok=True)
        environment.update({
            "ROS_MASTER_URI": "http://127.0.0.1:{}".format(args.port),
            "ROS_HOME": str(ros_home),
            "ROS_LOG_DIR": str(ros_home / "log"),
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
        load_ros_modules()
        summary["environment"] = {
            "python": sys.executable,
            "python_version": sys.version.split()[0],
            "ros_master_uri": environment["ROS_MASTER_URI"],
            "ros_distro": environment.get("ROS_DISTRO"),
            "ros_pythonpath_entries": [
                entry for entry in environment.get(
                    "PYTHONPATH", "").split(os.pathsep) if entry][:8],
        }

        node_python = ("/usr/bin/python3" if Path("/usr/bin/python3").exists()
                       else sys.executable)
        roscore = NodeProcess(
            "roscore", ["roscore", "-p", str(args.port)], environment,
            log_dir)
        nodes.append(roscore)
        wait_for(lambda: _port_open(args.port), 20.0,
                 "private roscore", nodes)
        rospy.init_node(
            "formation_ros_smoke", anonymous=True, disable_signals=True)
        set_parameters()

        observations = Observations()
        rospy.Subscriber(
            "/formation/optimizer_status", String,
            observations.add_optimizer_status, queue_size=50)
        rospy.Subscriber(
            "/formation/quality_report", String, observations.add_quality,
            queue_size=50)
        rospy.Subscriber(
            "/formation/timed_trajectory", FormationTimedTrajectory,
            observations.add_trajectory, queue_size=10)
        rospy.Subscriber(
            "/formation/markers", MarkerArray, observations.add_markers,
            queue_size=50)
        rospy.Subscriber(
            "/formation/playback_status", String,
            observations.add_playback_status, queue_size=100)
        rospy.Subscriber(
            "/tf", TFMessage, observations.add_tf, queue_size=100)

        optimizer = NodeProcess(
            "formation_optimizer",
            [node_python, str(OPTIMIZER_SCRIPT),
             "__name:=formation_optimizer"],
            environment, log_dir)
        nodes.append(optimizer)
        playback = NodeProcess(
            "formation_playback",
            [node_python, str(PLAYBACK_SCRIPT),
             "__name:=formation_playback"],
            environment, log_dir)
        nodes.append(playback)
        summary["processes"] = [node.describe() for node in nodes]

        wait_for(
            lambda: "/formation/optimizer_status" in
            {topic for topic, _ in rospy.get_published_topics()},
            20.0, "formation optimizer publisher", nodes, observations)
        wait_for(
            lambda: "/formation/markers" in
            {topic for topic, _ in rospy.get_published_topics()},
            20.0, "formation playback publisher", nodes, observations)
        _check(
            summary, "one_optimizer_and_playback_running",
            optimizer.alive() and playback.alive(),
            "optimizer pid={} playback pid={}".format(
                optimizer.process.pid, playback.process.pid))

        map_publisher = rospy.Publisher(
            "/map", OccupancyGrid, queue_size=1, latch=True)
        wait_for(lambda: map_publisher.get_num_connections() > 0, 10.0,
                 "optimizer map subscription", nodes, observations)
        map_publisher.publish(make_map())
        # The optimizer publishes no status for the first plain map event.
        # A short settle delay lets the exact grid land before the request;
        # the request/seed state machine also survives a late map.
        time.sleep(0.3)

        request_publisher = rospy.Publisher(
            "/formation/plan_request", FormationPlanRequest, queue_size=1)
        seed_publisher = rospy.Publisher(
            "/formation/plan_seed", FormationPlanSeed, queue_size=1,
            latch=True)
        wait_for(lambda: request_publisher.get_num_connections() > 0, 10.0,
                 "optimizer request subscription", nodes, observations)
        request_publisher.publish(make_request())
        time.sleep(0.1)
        seed_publisher.publish(make_seed())

        feasible_item = wait_for(
            lambda: _find_feasible_status(observations, PLAN_ID), 30.0,
            "FEASIBLE optimizer status", nodes, observations)
        summary["observed"]["optimizer_status"] = feasible_item[1]
        published_before = feasible_item[0]
        _check(
            summary, "optimizer_feasible_published",
            feasible_item[1].get("trajectory_published") is True and
            feasible_item[1].get("published_plan_id") == PLAN_ID,
            "status={} reason={!r} T_end={} publication_version={}".format(
                feasible_item[1].get("status"),
                feasible_item[1].get("reason"),
                feasible_item[1].get("T_end"),
                feasible_item[1].get("publication_version")))
        _check(
            summary, "map_ready",
            feasible_item[1].get("map_ready") is True and
            bool(feasible_item[1].get("map_signature")),
            "map_ready={} map_signature={}".format(
                feasible_item[1].get("map_ready"),
                feasible_item[1].get("map_signature")))

        quality_item = wait_for(
            lambda: _find_feasible_quality(observations, PLAN_ID), 10.0,
            "independent quality report", nodes, observations)
        summary["observed"]["quality_report"] = quality_item[1]
        _check(
            summary, "quality_independent_audit",
            quality_item[1].get("independent_audit") is True,
            "independent_audit={} event={}".format(
                quality_item[1].get("independent_audit"),
                quality_item[1].get("event")))
        _check(
            summary, "quality_collision_certificate",
            quality_item[1].get("collision_certificate") is True,
            "collision_certificate={}".format(
                quality_item[1].get("collision_certificate")))

        trajectory_message = wait_for(
            lambda: _find_published_trajectory(observations, PLAN_ID), 10.0,
            "published common-time trajectory", nodes, observations)
        summary["observed"]["trajectory"] = {
            "plan_id": int(trajectory_message.plan_id),
            "formation_id": trajectory_message.formation_id,
            "stamp": [int(trajectory_message.header.stamp.secs),
                      int(trajectory_message.header.stamp.nsecs)],
            "vehicle_ids": sorted(
                int(trajectory.vehicle_id)
                for trajectory in trajectory_message.trajectories),
            "sample_count": len(trajectory_message.times_from_start),
            "duration_s": float(
                trajectory_message.times_from_start[-1].to_sec()),
            "frame_id": trajectory_message.header.frame_id,
        }
        _check(
            summary, "trajectory_identity",
            int(trajectory_message.plan_id) == PLAN_ID and
            trajectory_message.formation_id == FORMATION_ID and
            trajectory_message.header.frame_id == "map" and
            trajectory_message.header.stamp.to_sec() > 0.0,
            "plan={} formation={} frame={}".format(
                trajectory_message.plan_id,
                trajectory_message.formation_id,
                trajectory_message.header.frame_id))
        check_common_time_grid(summary, trajectory_message)
        check_trajectory_geometry(summary, trajectory_message)

        first_playback = wait_for(
            lambda: observations.latest_playback()
            if observations.latest_playback() is not None and
            observations.latest_playback()[1].get("plan_id") == PLAN_ID
            else None, 10.0, "playback status for the plan", nodes,
            observations)
        _check(
            summary, "playback_accepted_plan",
            first_playback[1].get("formation_id") == FORMATION_ID and
            first_playback[1].get("vehicle_ids") == list(VEHICLE_IDS),
            "state={} formation={} vehicles={}".format(
                first_playback[1].get("state"),
                first_playback[1].get("formation_id"),
                first_playback[1].get("vehicle_ids")))

        def four_playing_samples():
            samples = _playing_samples(observations, PLAN_ID)
            if len(samples) < 4:
                return None
            span = (float(samples[-1][1].get("elapsed_s", 0.0)) -
                    float(samples[0][1].get("elapsed_s", 0.0)))
            return samples if span >= 0.12 else None

        samples = wait_for(four_playing_samples, 15.0,
                           "four PLAYING playback samples", nodes,
                           observations)
        check_playback_samples(summary, samples, trajectory_message)
        summary["observed"]["playback_samples"] = [
            {
                "elapsed_s": document.get("elapsed_s"),
                "clamped_elapsed_s": document.get("clamped_elapsed_s"),
                "state": document.get("state"),
                "vehicles": [
                    {"vehicle_id": item.get("vehicle_id"),
                     "center": item.get("center"),
                     "theta": item.get("theta"),
                     "mode": item.get("mode")}
                    for item in document.get("vehicles", [])],
            }
            for _, document in samples[:4]
        ]

        timeout = float(feasible_item[1].get("T_end") or 5.0) + 15.0
        terminal_item = wait_for(
            lambda: _find_terminal_status(observations, PLAN_ID), timeout,
            "terminal HOLD_END playback frame", nodes, observations)
        terminal = check_terminal(
            summary, terminal_item, trajectory_message)
        summary["observed"]["terminal"] = {
            "state": terminal.get("state"),
            "clamped_elapsed_s": terminal.get("clamped_elapsed_s"),
            "duration_s": terminal.get("duration_s"),
            "terminal_center_error_m": terminal.get(
                "terminal_center_error_m"),
            "terminal_heading_error_rad": terminal.get(
                "terminal_heading_error_rad"),
        }

        def terminal_marker_array():
            for wall, array in reversed(observations.markers):
                if wall >= terminal_item[0] and array.markers:
                    return wall, array
            return None

        terminal_marker_item = wait_for(
            terminal_marker_array, 5.0, "terminal marker array", nodes,
            observations)
        summary["observed"]["marker_namespaces"] = sorted({
            marker.ns for marker in terminal_marker_item[1].markers})
        summary["observed"]["marker_count"] = len(
            terminal_marker_item[1].markers)
        check_markers(
            summary, terminal_marker_item[1], terminal, trajectory_message)

        wait_for(
            lambda: all(frame in observations.tf_frames for frame in (
                "formation_{}_reference".format(FORMATION_ID),
                *{"formation_{}_vehicle_{}_base_link".format(
                    FORMATION_ID, vehicle_id)
                  for vehicle_id in VEHICLE_IDS})), 5.0,
            "formation TF frames", nodes, observations)
        check_tf(
            summary, observations, terminal, trajectory_message)
        summary["observed"]["tf_frames"] = sorted(
            frame for frame in observations.tf_frames
            if frame.startswith("formation_"))

        invalid_time = time.time()
        request_publisher.publish(make_request(
            plan_id=PLAN_ID + 1, nan_start=True))

        invalid_status_item = wait_for(
            lambda: next(
                ((wall, document) for wall, document
                 in observations.optimizer
                 if wall >= invalid_time and
                 document.get("status") == "INVALID" and
                 document.get("reason") == "INVALID_REQUEST"),
                None), 10.0, "INVALID_REQUEST optimizer status", nodes,
            observations)
        _check(
            summary, "invalid_request_rejected",
            invalid_status_item[1].get("trajectory_published") is False and
            invalid_status_item[1].get("plan_id") is None,
            "status={} reason={} field={}".format(
                invalid_status_item[1].get("status"),
                invalid_status_item[1].get("reason"),
                (invalid_status_item[1].get("failure") or {}).get("field")))

        clear_item = wait_for(
            lambda: next(
                ((wall, message) for wall, message
                 in observations.trajectories
                 if wall >= invalid_time and not message.vehicles and
                 not message.trajectories),
                None), 10.0, "empty trajectory clear", nodes, observations)
        _check(
            summary, "invalid_request_clears_trajectory",
            not clear_item[1].vehicles and not clear_item[1].trajectories,
            "clear message plan_id={} vehicles={} trajectories={}".format(
                clear_item[1].plan_id, len(clear_item[1].vehicles),
                len(clear_item[1].trajectories)))
        _check(
            summary, "invalid_request_clears_before_seed",
            clear_item[0] >= invalid_time,
            "clear arrived after the invalid request")

        cleared_item = wait_for(
            lambda: _find_cleared_status(observations, invalid_time), 10.0,
            "CLEARED playback status", nodes, observations)
        _check(
            summary, "playback_cleared_status",
            cleared_item[1].get("marker_count") == 0 and
            cleared_item[1].get("cleared") is True and
            cleared_item[1].get("plan_id") is None,
            "state={} marker_count={} reason={!r}".format(
                cleared_item[1].get("state"),
                cleared_item[1].get("marker_count"),
                cleared_item[1].get("reason")))

        deleteall_item = wait_for(
            lambda: _find_deleteall(observations, invalid_time), 10.0,
            "DELETEALL marker cleanup", nodes, observations)
        deleteall = deleteall_item[1].markers[0]
        _check(
            summary, "invalid_request_clears_markers",
            deleteall.ns == "formation" and deleteall.id == 0 and
            deleteall.action == Marker.DELETEALL,
            "DELETEALL ns={!r} id={}".format(deleteall.ns, deleteall.id))

        trajectory_publisher = rospy.Publisher(
            "/formation/timed_trajectory", FormationTimedTrajectory,
            queue_size=1, latch=True)
        wait_for(lambda: trajectory_publisher.get_num_connections() > 0,
                 10.0, "playback trajectory subscription", nodes,
                 observations)
        probe_time = time.time()
        trajectory_publisher.publish(make_geometry_probe())
        probe_status_item = wait_for(
            lambda: next(
                ((wall, document) for wall, document
                 in observations.playback
                 if wall >= probe_time and
                 document.get("plan_id") == PROBE_PLAN_ID),
                None), 10.0, "geometry-only playback status", nodes,
            observations)
        _check(
            summary, "geometry_only_label",
            probe_status_item[1].get("geometry_only") is True and
            probe_status_item[1].get("executable") is False and
            probe_status_item[1].get("certification") == "GEOMETRY_ONLY",
            "state={} certification={} audit_status={}".format(
                probe_status_item[1].get("state"),
                probe_status_item[1].get("certification"),
                probe_status_item[1].get("audit_status")))

        def probe_geometry_marker():
            namespace = _shared_namespace(PROBE_FORMATION_ID, "status")
            for wall, array in reversed(observations.markers):
                if wall < probe_time:
                    continue
                markers = _markers_with_namespace(array, namespace)
                if markers:
                    return wall, array, markers[0]
            return None

        probe_marker_item = wait_for(
            probe_geometry_marker, 10.0,
            "geometry-only status marker", nodes, observations)
        probe_text = probe_marker_item[2].text
        _check(
            summary, "geometry_only_marker_label",
            "GEOMETRY_ONLY" in probe_text and "FEASIBLE" not in probe_text and
            _vehicle_namespace(PROBE_FORMATION_ID, 1) in {
                marker.ns for marker in probe_marker_item[1].markers},
            "status marker text={!r}".format(probe_text[:300]))

        empty_time = time.time()
        trajectory_publisher.publish(make_empty_trajectory())
        empty_cleared_item = wait_for(
            lambda: _find_cleared_status(observations, empty_time), 10.0,
            "empty-trajectory CLEARED status", nodes, observations)
        empty_deleteall_item = wait_for(
            lambda: _find_deleteall(observations, empty_time), 10.0,
            "empty-trajectory DELETEALL", nodes, observations)
        _check(
            summary, "empty_trajectory_cleanup",
            empty_cleared_item[1].get("marker_count") == 0 and
            empty_deleteall_item[1].markers[0].action == Marker.DELETEALL,
            "empty trajectory cleared every marker")

        summary["observed"]["invalid_cleanup"] = {
            "optimizer_status": invalid_status_item[1],
            "playback_status": cleared_item[1],
            "marker_action": "DELETEALL",
        }

    except Exception as exc:
        summary["errors"].append({
            "check": "harness",
            "detail": "{}: {}".format(type(exc).__name__, exc),
        })
        if isinstance(exc, SmokeError):
            for node in nodes:
                if not node.alive():
                    summary["errors"].append({
                        "check": "process_exited",
                        "detail": "{} exit={} log tail:\n{}".format(
                            node.label, node.process.poll(), node.tail()),
                    })
    finally:
        try:
            rospy.signal_shutdown("formation smoke complete")
        except Exception:
            pass
        exit_codes = {}
        for node in reversed(nodes):
            exit_codes[node.label] = node.stop()
        stopped = all(not node.alive() for node in nodes)
        released = _wait_port_closed(args.port, 10.0)
        summary["cleanup"] = {
            "stopped": stopped,
            "port_released": released,
            "exit_codes": exit_codes,
        }
        _check(
            summary, "child_processes_stopped", stopped,
            "exit codes={}".format(exit_codes))
        _check(
            summary, "master_port_released", released,
            "port {} {}".format(
                args.port, "closed" if released else "still open"))
        summary["processes"] = [node.describe() for node in nodes]
        summary["logs"] = {
            node.label: {
                "path": str(node.path),
                "exit_code": exit_codes.get(node.label),
            }
            for node in nodes
        }

    summary["finished_at"] = _utc_now()
    summary["duration_s"] = round(time.time() - started_wall, 3)
    summary["pass"] = (
        not summary["errors"] and
        all(check["ok"] for check in summary["checks"].values()))
    _write_json(output_path, summary)
    return summary, output_path


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0])
    parser.add_argument(
        "--port", type=int, default=11399,
        help="private ROS master port (default 11399)")
    parser.add_argument(
        "--output",
        default="evaluation/formation_s10/smoke/ros_formation_smoke.json",
        help="strict-JSON smoke report path")
    parser.add_argument(
        "--log-dir", default="evaluation/formation_s10/logs/ros_smoke",
        help="directory for roscore/optimizer/playback logs")
    args = parser.parse_args(argv)

    summary, output_path = run(args)
    print("ros_formation_smoke: {} ({} checks, {})".format(
        "PASS" if summary["pass"] else "FAIL",
        len(summary["checks"]), output_path))
    for name, check in summary["checks"].items():
        print("  [{}] {}: {}".format(
            "ok" if check["ok"] else "FAIL", name, check["detail"]))
    if summary["errors"]:
        print("errors:", file=sys.stderr)
        for error in summary["errors"]:
            print("  {}: {}".format(error["check"], error["detail"]),
                  file=sys.stderr)
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())