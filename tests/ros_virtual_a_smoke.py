#!/usr/bin/env python3
"""G5-B live ROS driver for the virtual_rigid_a backend.

The demo launch is already running on a private ROS master.  This driver uses
only the two RViz-style user topics, ``/initialpose`` and
``/move_base_simple/goal``; it never publishes a reference trajectory or a
request to the planner for a positive case.  A direct busy probe injects one
``/formation/plan_request`` only to verify the planner's BUSY path.

Scenarios
---------
``main``     one nontrivial G4 curve (S1 FREE) or the explicit STRAIGHT
             S2 experiment with a second ordinary goal after the verified
             terminal; busy Goal/InitialPose probes run during playback.
             A FREE terminal that is not a supported continuation is refused
             with ``UNSUPPORTED_INITIAL_STATE`` and no envelope.
``failure``  one accepted STRAIGHT plan, then an admitted request to an
             occupied goal that produces no accepted candidate.  The old
             animation must stop with no envelope for the failed plan; an
             explicit reset and a new goal must recover.
``tamper``   a captured genuine envelope is replayed through
             ``/formation/playback_plan`` and mutated at the receiver only:
             unknown versions, wrong digest, position/heading/speed and time
             changes.  Every mutation must be rejected and clear the display.
``uturn``    the map05 parking-row U-turn with short and long approach lanes
             at scale 2.625. Both cases require certified playback and a
             terminal hold. Failed plans are recorded without aborting the set.

The driver independently rebuilds the expected request from the raw
``/formation/plan_request`` and the planner-published admission/job record,
then calls the real ``verify_virtual_a_playback_plan``.  It also compares
every published sample, TF, Marker and playback-status frame with the
authoritative ``VirtualATimedPlan`` at the message's actual quantized time.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = ROOT / "catkin_ws" / "src" / "fwis_v3_optimizer" / "src"
SCRIPTS = ROOT / "catkin_ws" / "src" / "fwis_v3_optimizer" / "scripts"
for entry in (str(PACKAGE_SRC), str(SCRIPTS)):
    if entry not in sys.path:
        sys.path.insert(0, entry)

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from fwis_v3_msgs.msg import (
    FormationPlaybackPlan,
    FormationPlanRequest,
    FormationVehicleSpec,
)
from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a.collision import load_map
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    GoalPolicy,
    collision_config_from_dict,
    geometry_audit_config_from_dict,
    grid_signature,
    planner_config_from_dict,
    planning_context_sha256,
    request_fingerprint,
    request_from_payload,
    timing_config_from_dict,
)
from fwis_v3_optimizer.formation_playback import (
    HOLD_END,
    HOLD_START,
    PLAYING,
    PlaybackValidationError,
    VIRTUAL_CERTIFICATION,
    VIRTUAL_PLAYBACK_BACKEND,
    VIRTUAL_PLAYBACK_SEMANTIC_VERSION,
    VIRTUAL_MODEL_VERSION,
    fields_from_timed_message,
    tf_frame_prefix,
    vehicle_namespace,
    verify_virtual_a_playback_plan,
    virtual_a_request_from_raw_request,
)

import formation_uniform_planner_node as planner_node

PROTECTED_PORTS = (11311, 11312, 11320)
MODEL_POSITION_TOL_M = 1e-7
MODEL_ANGLE_TOL_RAD = 1e-7
MODEL_SPEED_TOL = 1e-7
RIGIDITY_TOL_M = 1e-6
TERMINAL_POSITION_TOL_M = 0.005
TERMINAL_ANGLE_TOL_RAD = math.radians(0.2)
DISPLAY_MIN_SAMPLES = 100
MAP05_UTURN_MAP_SIGNATURE = "5a9ab3f50546275ad9b47ed0524e76431e2096c4c4f753dcf37243e4b293451f"
MAP05_UTURN_CASES = (
    ("short_approach", (10.841, 12.071, 0.000), (10.153, 2.076, -3.126)),
    ("long_approach", (-3.625, 11.771, -0.022), (-2.490, 2.309, 3.102)),
)
JOB_TOPIC = "/formation/job_state"
STATUS_TOPIC = "/formation/optimizer_status"
PLAYBACK_STATUS_TOPIC = "/formation/playback_status"
PLAN_TOPIC = "/formation/playback_plan"
REQUEST_TOPIC = "/formation/plan_request"
MARKER_TOPIC = "/formation/markers"
TF_TOPIC = "/tf"
GOAL_TOPIC = "/move_base_simple/goal"
INITIAL_POSE_TOPIC = "/initialpose"
MAP_TOPIC = "/map"


class SmokeError(RuntimeError):
    pass


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_value(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0),
                      w=math.cos(yaw / 2.0))


def duration_ns(value):
    return int(value.secs) * 10 ** 9 + int(value.nsecs)


def json_safe(value):
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    return repr(value)


def write_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(json_safe(document), indent=1, sort_keys=True,
                      allow_nan=False) + "\n"
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def note(checks, name, ok, detail="", *, required=True):
    checks[name] = {"ok": bool(ok), "detail": json_safe(detail)}
    if required and not ok:
        raise SmokeError("{}: {}".format(name, detail))


def _xdotool(environment, *arguments):
    """Run one xdotool call, returning stdout; failures are non-fatal."""
    try:
        completed = subprocess.run(
            ["xdotool"] + list(arguments), env=environment,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5.0)
    except (OSError, subprocess.SubprocessError):
        return ""
    if completed.returncode != 0:
        return ""
    return completed.stdout.decode("utf-8", "replace")


def _focus_rviz(environment):
    """Raise and maximize RViz so screenshots show the actual display."""
    # The ROS 1 end-of-life notice is a modal dialog owned by RViz; dismiss it
    # before capturing so it cannot cover the formation view.
    for window_id in _xdotool(
            environment, "search", "--name", "End-of-Life").split():
        _xdotool(environment, "windowactivate", window_id)
        _xdotool(environment, "key", "--window", window_id, "Return")
    time.sleep(0.3)
    window_id = None
    for candidate in _xdotool(
            environment, "search", "--name", "RViz").split():
        title = _xdotool(environment, "getwindowname", candidate).rstrip()
        if title.endswith("- RViz"):
            window_id = candidate
            break
    if window_id is None:
        return
    geometry = {}
    for line in _xdotool(
            environment, "getwindowgeometry", "--shell", window_id).splitlines():
        key, _, value = line.partition("=")
        geometry[key.strip()] = value.strip()
    try:
        x = int(geometry["X"])
        y = int(geometry["Y"])
        width = int(geometry["WIDTH"])
        height = int(geometry["HEIGHT"])
    except (KeyError, ValueError):
        x = y = 0
        width = height = 1
    center_x = x + width / 2.0
    center_y = y + height / 2.0
    monitor = None
    try:
        completed = subprocess.run(
            ["xrandr", "--query"], env=environment,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5.0)
        pattern = re.compile(
            r"^(\S+) connected (?:primary )?(\d+)x(\d+)\+(\d+)\+(\d+)")
        for line in completed.stdout.decode("utf-8", "replace").splitlines():
            matched = pattern.match(line)
            if not matched:
                continue
            mon_width = int(matched.group(2))
            mon_height = int(matched.group(3))
            mon_x = int(matched.group(4))
            mon_y = int(matched.group(5))
            if (mon_x <= center_x < mon_x + mon_width and
                    mon_y <= center_y < mon_y + mon_height):
                monitor = (mon_width, mon_height, mon_x, mon_y)
                break
    except (OSError, subprocess.SubprocessError):
        monitor = None
    if monitor is not None:
        mon_width, mon_height, mon_x, mon_y = monitor
        _xdotool(environment, "windowsize", window_id,
                 str(mon_width), str(mon_height))
        _xdotool(environment, "windowmove", window_id,
                 str(mon_x), str(mon_y))
    _xdotool(environment, "windowactivate", window_id)
    time.sleep(0.6)
    return window_id, width, height


def screenshot(path):
    """Best-effort image of the current X display; never fails a check."""
    path.parent.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ)
    environment.setdefault("DISPLAY", ":0")
    focused = _focus_rviz(environment)
    target = "root"
    zoomed = False
    if focused is not None:
        window_id, _width, _height = focused
        target = window_id
        # A few wheel steps enlarge the formation inside the fixed top-down
        # view; the same number of reverse steps restores the saved camera so
        # successive screenshots are comparable.
        _xdotool(environment, "mousemove", "--window", window_id,
                 str(max(1, _width // 2)), str(max(1, _height // 2)))
        for _ in range(3):
            _xdotool(environment, "click", "4")
            time.sleep(0.08)
        time.sleep(0.3)
        zoomed = True
    try:
        completed = subprocess.run(
            ["import", "-window", target, str(path)],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=15.0)
    except (OSError, subprocess.SubprocessError) as exc:
        return {"ok": False, "detail": "{}: {}".format(
            type(exc).__name__, exc)}
    finally:
        if zoomed:
            for _ in range(3):
                _xdotool(environment, "click", "5")
                time.sleep(0.08)
    return {
        "ok": completed.returncode == 0 and path.is_file(),
        "returncode": int(completed.returncode),
        "stderr": completed.stderr.decode("utf-8", "replace")[-400:],
    }


class MarkerRecord(object):
    __slots__ = ("stamp", "count", "arrows", "delete_all")

    def __init__(self, stamp, count, arrows, delete_all):
        self.stamp = stamp
        self.count = count
        self.arrows = arrows
        self.delete_all = delete_all


class Session(object):
    """Thread-safe recorder for the real demo topics."""

    def __init__(self):
        self.lock = threading.Lock()
        self.job_states = []
        self.statuses = []
        self.playback = []
        self.envelopes = {}
        self.envelope_records = []
        self.requests = []
        self.markers = []
        self.transforms = []
        self.maps = []
        self.capture_display = True
        self.started = rospy.Time.now().to_sec()
        self._subscribers = [
            rospy.Subscriber(JOB_TOPIC, String, self.on_job_state,
                             queue_size=50),
            rospy.Subscriber(STATUS_TOPIC, String, self.on_status,
                             queue_size=50),
            rospy.Subscriber(PLAYBACK_STATUS_TOPIC, String,
                             self.on_playback, queue_size=200),
            rospy.Subscriber(PLAN_TOPIC, FormationPlaybackPlan,
                             self.on_envelope, queue_size=20),
            rospy.Subscriber(REQUEST_TOPIC, FormationPlanRequest,
                             self.on_request, queue_size=50),
            rospy.Subscriber(MARKER_TOPIC, MarkerArray, self.on_markers,
                             queue_size=100),
            rospy.Subscriber(TF_TOPIC, TFMessage, self.on_tf,
                             queue_size=400),
            rospy.Subscriber(MAP_TOPIC, OccupancyGrid, self.on_map,
                             queue_size=1),
        ]

    @staticmethod
    def _document(message, label):
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            rospy.logwarn("ignoring invalid %s JSON", label)
            return None
        return value if isinstance(value, dict) else None

    def on_job_state(self, message):
        document = self._document(message, "job_state")
        if document is not None:
            with self.lock:
                self.job_states.append((rospy.Time.now().to_sec(),
                                        document))

    def on_status(self, message):
        document = self._document(message, "optimizer_status")
        if document is not None:
            with self.lock:
                self.statuses.append((rospy.Time.now().to_sec(), document))

    def on_playback(self, message):
        document = self._document(message, "playback_status")
        if document is not None:
            with self.lock:
                self.playback.append((rospy.Time.now().to_sec(), document))

    def on_envelope(self, message):
        now = rospy.Time.now().to_sec()
        with self.lock:
            self.envelopes[int(message.plan_id)] = message
            self.envelope_records.append((now, message))

    def on_request(self, message):
        with self.lock:
            self.requests.append((rospy.Time.now().to_sec(), message))

    def on_markers(self, message):
        if not self.capture_display:
            return
        markers = message.markers
        if not markers:
            return
        arrows = []
        delete_all = False
        for item in markers:
            if item.type == Marker.ARROW and len(item.points) >= 2:
                first, second = item.points[0], item.points[1]
                arrows.append((item.ns, (first.x, first.y),
                               (second.x, second.y)))
            elif item.action == Marker.DELETEALL:
                delete_all = True
        record = MarkerRecord(markers[0].header.stamp, len(markers),
                              arrows, delete_all)
        with self.lock:
            self.markers.append((rospy.Time.now().to_sec(), record))

    def on_tf(self, message):
        if not self.capture_display:
            return
        with self.lock:
            self.transforms.append((rospy.Time.now().to_sec(), message))

    def on_map(self, message):
        with self.lock:
            self.maps.append((rospy.Time.now().to_sec(), message))

    # ---- waiting helpers ---------------------------------------------

    def wait_job(self, predicate, timeout, description):
        deadline = rospy.Time.now().to_sec() + timeout
        while rospy.Time.now().to_sec() < deadline:
            with self.lock:
                for received, document in reversed(self.job_states):
                    if predicate(document):
                        return document
            rospy.sleep(0.02)
        raise SmokeError("timeout waiting for {}".format(description))

    def wait_envelope(self, plan_id, timeout):
        deadline = rospy.Time.now().to_sec() + timeout
        while rospy.Time.now().to_sec() < deadline:
            with self.lock:
                message = self.envelopes.get(plan_id)
            if message is not None and message.model_json and (
                    len(message.trajectory.times_from_start) > 0):
                return message
            rospy.sleep(0.02)
        raise SmokeError(
            "timeout waiting for executable envelope plan {}".format(plan_id))

    def wait_request_after(self, count, timeout):
        deadline = rospy.Time.now().to_sec() + timeout
        while rospy.Time.now().to_sec() < deadline:
            with self.lock:
                if len(self.requests) > count:
                    return self.requests[count][1]
            rospy.sleep(0.01)
        raise SmokeError("timeout waiting for a new formation plan request")

    def wait_playback(self, predicate, timeout, description, mark=0):
        deadline = rospy.Time.now().to_sec() + timeout
        while rospy.Time.now().to_sec() < deadline:
            with self.lock:
                for received, document in reversed(self.playback[mark:]):
                    if predicate(document):
                        return document
            rospy.sleep(0.01)
        raise SmokeError("timeout waiting for {}".format(description))

    def latest_playback(self):
        with self.lock:
            return self.playback[-1][1] if self.playback else None

    def job_history(self, plan_id):
        with self.lock:
            return [(t, d) for t, d in self.job_states
                    if d.get("plan_id") == plan_id]

    def playback_history(self, plan_id):
        with self.lock:
            return [(t, d) for t, d in self.playback
                    if d.get("plan_id") == plan_id]

    def request_count(self):
        with self.lock:
            return len(self.requests)

    def status_reason(self, plan_id):
        with self.lock:
            for _, document in reversed(self.statuses):
                if document.get("plan_id") == plan_id:
                    return document.get("reason")
        return None

    def latest_marker(self):
        with self.lock:
            return self.markers[-1][1] if self.markers else None

    def capture_start(self):
        with self.lock:
            return (len(self.playback), len(self.markers),
                    len(self.transforms))


def publish_initial_pose(x, y, theta):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.pose.pose.position.x = float(x)
    message.pose.pose.position.y = float(y)
    message.pose.pose.orientation = quaternion_value(theta)
    publisher = rospy.Publisher(INITIAL_POSE_TOPIC,
                                PoseWithCovarianceStamped, queue_size=1)
    rospy.sleep(0.3)
    publisher.publish(message)
    rospy.sleep(0.2)
    publisher.unregister()


def publish_goal(x, y, theta):
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.pose.position.x = float(x)
    message.pose.position.y = float(y)
    message.pose.orientation = quaternion_value(theta)
    publisher = rospy.Publisher(GOAL_TOPIC, PoseStamped, queue_size=1)
    rospy.sleep(0.3)
    publisher.publish(message)
    rospy.sleep(0.1)
    publisher.unregister()


def publish_envelope(message):
    publisher = rospy.Publisher(PLAN_TOPIC, FormationPlaybackPlan,
                                queue_size=1)
    rospy.sleep(0.2)
    publisher.publish(message)
    rospy.sleep(0.05)
    publisher.unregister()


def raw_request_document(message):
    vehicles = []
    for spec in message.vehicles:
        vehicles.append({
            "vehicle_id": int(spec.vehicle_id),
            "offset_x": float(spec.offset_in_formation.x),
            "offset_y": float(spec.offset_in_formation.y),
            "start_x": float(spec.start_center.x),
            "start_y": float(spec.start_center.y),
            "start_theta": float(spec.start_center.theta),
            "goal_x": float(spec.goal_center.x),
            "goal_y": float(spec.goal_center.y),
            "goal_theta": float(spec.goal_center.theta),
            "goal_heading_free": bool(spec.goal_heading_free),
            "body_length_m": float(spec.body_length_m),
            "body_width_m": float(spec.body_width_m),
        })
    return {
        "header": {
            "frame_id": message.header.frame_id,
            "secs": int(message.header.stamp.secs),
            "nsecs": int(message.header.stamp.nsecs),
        },
        "plan_id": int(message.plan_id),
        "formation_id": message.formation_id,
        "formation_start": {
            "x": float(message.formation_start.x),
            "y": float(message.formation_start.y),
            "theta": float(message.formation_start.theta),
        },
        "formation_goal": {
            "x": float(message.formation_goal.x),
            "y": float(message.formation_goal.y),
            "theta": float(message.formation_goal.theta),
        },
        "allow_reverse": bool(message.allow_reverse),
        "vehicles": vehicles,
    }


def envelope_document(message):
    fields = fields_from_timed_message(message.trajectory)
    return {
        "header": {
            "frame_id": message.header.frame_id,
            "secs": int(message.header.stamp.secs),
            "nsecs": int(message.header.stamp.nsecs),
        },
        "plan_id": int(message.plan_id),
        "formation_id": message.formation_id,
        "backend": message.backend,
        "semantic_version": message.semantic_version,
        "model_version": message.model_version,
        "model_sha256": message.model_sha256,
        "certification": message.certification,
        "model_json": message.model_json,
        "trajectory_fields": fields,
    }


def exact_duration(seconds):
    nanoseconds = int(round(float(seconds) * 1e9))
    secs, nsecs = divmod(nanoseconds, 10 ** 9)
    return rospy.Duration(secs, nsecs)


def build_envelope_message(document, *, fields=None, plan_id=None,
                           semantic_version=None, model_version=None,
                           model_sha256=None, model_json=None,
                           certification=None, embedded_stamp=None):
    fields = copy.deepcopy(document["trajectory_fields"]
                           if fields is None else fields)
    if embedded_stamp is not None:
        fields["stamp"] = list(embedded_stamp)
    message = FormationPlaybackPlan()
    header = document["header"]
    message.header.frame_id = header["frame_id"]
    message.header.stamp = rospy.Time(header["secs"], header["nsecs"])
    message.plan_id = int(document["plan_id"] if plan_id is None else plan_id)
    message.formation_id = document["formation_id"]
    message.backend = document["backend"]
    message.semantic_version = (
        document["semantic_version"] if semantic_version is None
        else semantic_version)
    message.model_version = (
        document["model_version"] if model_version is None
        else model_version)
    message.model_sha256 = (
        document["model_sha256"] if model_sha256 is None else model_sha256)
    message.model_json = (
        document["model_json"] if model_json is None else model_json)
    message.certification = (
        document["certification"] if certification is None
        else certification)
    message.trajectory = planner_node.timed_message(fields)
    for value, index in zip(fields["times_from_start"],
                            range(len(message.trajectory.times_from_start))):
        message.trajectory.times_from_start[index] = exact_duration(value)
    for values, trajectory in zip(fields["trajectories"],
                                  message.trajectory.trajectories):
        for (time, _x, _y, _theta, _speed, _yaw, _mode,
             _control), point in zip(values["points"], trajectory.points):
            point.time_from_start = exact_duration(time)
    return message


# ---- independent envelope checks -------------------------------------

def sample_times(message):
    times_ns = [duration_ns(value)
                for value in message.trajectory.times_from_start]
    times_s = [value / 1e9 for value in times_ns]
    return times_ns, times_s


def analyze_envelope(session, request_message, job, message, checks, prefix,
                     envelope_dir, request_dir):
    document = envelope_document(message)
    raw = raw_request_document(request_message)
    note(checks, prefix + "raw_request_plan_id",
         raw["plan_id"] == int(message.plan_id), raw["plan_id"])
    note(checks, prefix + "job_plan_id",
         job.get("plan_id") == int(message.plan_id), job.get("plan_id"))
    note(checks, prefix + "job_formation_id",
         job.get("formation_id") == message.formation_id,
         job.get("formation_id"))
    note(checks, prefix + "job_state_planning", job.get("state") == "PLANNING",
         job.get("state"))
    note(checks, prefix + "job_backend",
         job.get("backend") == VIRTUAL_PLAYBACK_BACKEND,
         job.get("backend"))
    initial = job.get("initial_context")
    note(checks, prefix + "job_initial_context",
         isinstance(initial, dict) and bool(initial.get("compatible")),
         initial)
    context = job.get("planning_context")
    note(checks, prefix + "job_planning_context",
         isinstance(context, dict), type(context).__name__)
    grid_sha = job.get("grid_sha256")
    note(checks, prefix + "job_grid_sha_shape",
         isinstance(grid_sha, str) and len(grid_sha) == 64, grid_sha)
    try:
        request = virtual_a_request_from_raw_request(
            raw, initial_source=initial.get("source"),
            initial_context_id=initial.get("context_id"))
    except Exception as exc:
        raise SmokeError("{} raw request rebuild failed: {}: {}".format(
            prefix, type(exc).__name__, exc))
    fingerprint = request_fingerprint(request)
    note(checks, prefix + "job_request_sha256",
         job.get("request_sha256") == fingerprint,
         {"job": job.get("request_sha256"), "rebuilt": fingerprint})
    note(checks, prefix + "job_request_sha256_context",
         context.get("request_sha256") == fingerprint,
         context.get("request_sha256"))
    note(checks, prefix + "job_initial_source",
         initial.get("source") == request.initial.source,
         {"job": initial.get("source"),
          "request": request.initial.source})
    note(checks, prefix + "job_initial_context_id",
         initial.get("context_id") == request.initial.context_id,
         {"job": initial.get("context_id"),
          "request": request.initial.context_id})
    note(checks, prefix + "job_goal_policy",
         str(job.get("virtual_a_goal_policy", "")).lower() ==
         request.goal_policy.value.lower(),
         {"job": job.get("virtual_a_goal_policy"),
          "request": request.goal_policy.value})
    request_path = request_dir / "plan_request_{:08d}.json".format(
        int(message.plan_id))
    write_json(request_path, raw)
    write_json(request_dir / "plan_request_{:08d}_job.json".format(
        int(message.plan_id)), job)

    geometry_payload = context.get("timing_config", {}).get(
        "geometry_audit_config")
    geometry_config = geometry_audit_config_from_dict(geometry_payload)
    note(checks, prefix + "context_sha256_matches_job",
         planning_context_sha256(context) ==
         job.get("planning_context_sha256"),
         {"computed": planning_context_sha256(context),
          "job": job.get("planning_context_sha256")})
    fields = fields_from_timed_message(message.trajectory)
    stamp = (int(message.header.stamp.secs),
             int(message.header.stamp.nsecs))
    note(checks, prefix + "envelope_stamp_equals_trajectory",
         fields["stamp"] == stamp, {"envelope": stamp,
                                    "trajectory": fields["stamp"]})
    note(checks, prefix + "envelope_backend",
         message.backend == VIRTUAL_PLAYBACK_BACKEND, message.backend)
    note(checks, prefix + "envelope_semantic_version",
         message.semantic_version == VIRTUAL_PLAYBACK_SEMANTIC_VERSION,
         message.semantic_version)
    note(checks, prefix + "envelope_model_version",
         message.model_version == VIRTUAL_MODEL_VERSION,
         message.model_version)
    note(checks, prefix + "envelope_certification",
         message.certification == VIRTUAL_CERTIFICATION,
         message.certification)

    trajectory, plan, digest = verify_virtual_a_playback_plan(
        plan_id=int(message.plan_id),
        formation_id=message.formation_id,
        backend=message.backend,
        semantic_version=message.semantic_version,
        model_version=message.model_version,
        model_sha256=message.model_sha256,
        model_json=message.model_json,
        certification=message.certification,
        trajectory_fields=fields,
        expected_request=request,
        expected_context=context,
        expected_grid_sha256=grid_sha,
        geometry_audit_config=geometry_config)
    note(checks, prefix + "verified_digest",
         digest == message.model_sha256, digest)
    note(checks, prefix + "trajectory_backend",
         trajectory.backend == VIRTUAL_PLAYBACK_BACKEND,
         trajectory.backend)
    note(checks, prefix + "trajectory_plan_identity",
         trajectory.plan_id == int(message.plan_id) and
         trajectory.formation_id == message.formation_id,
         [trajectory.plan_id, trajectory.formation_id])
    note(checks, prefix + "plan_request_sha256",
         plan.request_sha256 == fingerprint, plan.request_sha256)
    note(checks, prefix + "plan_context_sha256",
         plan.planning_context_sha256() == job.get(
             "planning_context_sha256"),
         plan.planning_context_sha256())

    times_ns, times_s = sample_times(message)
    note(checks, prefix + "sample_count", len(times_ns) >= 100,
         len(times_ns))
    note(checks, prefix + "sample_time_first_zero", times_ns[0] == 0,
         times_ns[0])
    note(checks, prefix + "sample_time_integer_ns",
         all(0 <= value < 2 ** 31 * 10 ** 9 for value in times_ns),
         times_ns[:3])
    note(checks, prefix + "sample_time_increasing",
         all(first < second for first, second in zip(times_ns, times_ns[1:])),
         "not strictly increasing")
    target = int(math.ceil(float(plan.duration_s) * 1e9))
    if target < 1:
        target = 1
    while target > 1 and (target - 1) / 1e9 >= float(plan.duration_s):
        target -= 1
    while target / 1e9 < float(plan.duration_s):
        target += 1
    note(checks, prefix + "sample_time_endpoint", times_ns[-1] == target,
         {"last_ns": times_ns[-1], "required_last_ns": target})
    note(checks, prefix + "sample_pose_count",
         len(fields["formation_poses"]) == len(times_ns),
         [len(fields["formation_poses"]), len(times_ns)])
    interior_steps = [second - first for first, second in
                      zip(times_ns, times_ns[1:])]
    nominal_step = interior_steps[0]
    note(checks, prefix + "sample_grid_dt",
         nominal_step > 0 and
         all(step == nominal_step for step in interior_steps[:-1]) and
         0 < interior_steps[-1] <= nominal_step,
         {"nominal_ns": nominal_step,
          "steps_unique": sorted(set(interior_steps))[:5],
          "last": interior_steps[-1]})

    worst = {
        "position_m": 0.0, "angle_rad": 0.0, "speed_mps": 0.0,
        "omega_radps": 0.0, "control": 0.0, "qf_position_m": 0.0,
        "qf_angle_rad": 0.0, "rigidity_m": 0.0,
    }
    interior = 0
    vehicle_ids = sorted(request.model.vehicle_ids)
    sample_tables = {
        int(item.vehicle_id): item for item in message.trajectory.trajectories}
    note(checks, prefix + "vehicle_sample_rows",
         sorted(sample_tables) == vehicle_ids,
         [sorted(sample_tables), vehicle_ids])
    for row in sample_tables.values():
        note(checks, prefix + "vehicle_point_count_for_{}".format(
            row.vehicle_id), len(row.points) == len(times_ns),
            [len(row.points), len(times_ns)])
    for index, (time_ns, time_s) in enumerate(zip(times_ns, times_s)):
        pose = fields["formation_poses"][index]
        # The verifier evaluates the authoritative model at the actual
        # quantized time, including the endpoint ceil(T); do the same here.
        state = plan.state_at(time_s)
        if 0 < time_s < float(plan.duration_s):
            interior += 1
        qf_position = math.hypot(pose[0] - state.qf.x, pose[1] - state.qf.y)
        qf_angle = abs(wrap(pose[2] - state.qf.psi))
        worst["qf_position_m"] = max(worst["qf_position_m"], qf_position)
        worst["qf_angle_rad"] = max(worst["qf_angle_rad"], qf_angle)
        centers = request.model.centers(FormationPose(pose[0], pose[1],
                                                      pose[2]))
        for vehicle_id, row in sample_tables.items():
            point = row.points[index]
            point_time = duration_ns(point.time_from_start)
            if point_time != time_ns:
                raise SmokeError(
                    "{} vehicle {} sample {} point time {} != shared {}"
                    .format(prefix, vehicle_id, index, point_time, time_ns))
            expected = state.vehicle(vehicle_id)
            measured = point.center_pose
            worst["position_m"] = max(
                worst["position_m"],
                math.hypot(measured.x - expected.x,
                           measured.y - expected.y))
            worst["angle_rad"] = max(
                worst["angle_rad"], abs(wrap(measured.theta - expected.theta)))
            worst["speed_mps"] = max(
                worst["speed_mps"],
                abs(point.center_speed_mps - expected.v_i))
            worst["omega_radps"] = max(
                worst["omega_radps"],
                abs(point.center_yaw_rate_radps - expected.omega_i))
            worst["control"] = max(
                worst["control"],
                abs(point.mode_control - expected.C_i))
            worst["rigidity_m"] = max(
                worst["rigidity_m"],
                math.hypot(measured.x - centers[vehicle_id][0],
                           measured.y - centers[vehicle_id][1]))
    note(checks, prefix + "interior_sample_count", interior >= 100, interior)
    note(checks, prefix + "sample_position_vs_model",
         worst["position_m"] <= MODEL_POSITION_TOL_M, worst["position_m"])
    note(checks, prefix + "sample_angle_vs_model",
         worst["angle_rad"] <= MODEL_ANGLE_TOL_RAD, worst["angle_rad"])
    note(checks, prefix + "sample_speed_vs_model",
         worst["speed_mps"] <= MODEL_SPEED_TOL, worst["speed_mps"])
    note(checks, prefix + "sample_omega_vs_model",
         worst["omega_radps"] <= MODEL_SPEED_TOL, worst["omega_radps"])
    note(checks, prefix + "sample_control_vs_model",
         worst["control"] <= MODEL_SPEED_TOL, worst["control"])
    note(checks, prefix + "sample_qf_position_vs_model",
         worst["qf_position_m"] <= MODEL_POSITION_TOL_M,
         worst["qf_position_m"])
    note(checks, prefix + "sample_qf_angle_vs_model",
         worst["qf_angle_rad"] <= MODEL_ANGLE_TOL_RAD, worst["qf_angle_rad"])
    note(checks, prefix + "sample_rigidity", worst["rigidity_m"]
         <= RIGIDITY_TOL_M, worst["rigidity_m"])
    note(checks, prefix + "sample_mode",
         all(point.mode == 0 for row in sample_tables.values()
             for point in row.points), "non-A sample")

    terminal = plan.state_at(float(plan.duration_s))
    terminal_position = math.hypot(terminal.qf.x - request.goal_qf.x,
                                   terminal.qf.y - request.goal_qf.y)
    heading_errors = {
        vehicle_id: abs(wrap(terminal.vehicle(vehicle_id).theta -
                             request.goal_qf.psi))
        for vehicle_id in vehicle_ids}
    terminal_angle = max(heading_errors.values())
    note(checks, prefix + "terminal_position",
         terminal_position <= TERMINAL_POSITION_TOL_M, terminal_position)
    if request.goal_policy == GoalPolicy.STRAIGHT:
        note(checks, prefix + "terminal_heading",
             terminal_angle <= TERMINAL_ANGLE_TOL_RAD, heading_errors)
    start = plan.state_at(0.0)
    start_centers = request.model.centers(request.start_qf)
    start_error = max(
        math.hypot(start.vehicle(vehicle_id).x - start_centers[vehicle_id][0],
                   start.vehicle(vehicle_id).y - start_centers[vehicle_id][1])
        for vehicle_id in vehicle_ids)
    note(checks, prefix + "start_position", start_error <= 1e-9, start_error)

    path = envelope_dir / "envelope_{:08d}.json".format(int(message.plan_id))
    write_json(path, document)
    summary = {
        "plan_id": int(message.plan_id),
        "formation_id": message.formation_id,
        "model_sha256": message.model_sha256,
        "request_sha256": fingerprint,
        "planning_context_sha256": job.get("planning_context_sha256"),
        "grid_sha256": grid_sha,
        "semantic_version": message.semantic_version,
        "model_version": message.model_version,
        "certification": message.certification,
        "stamp": {"secs": stamp[0], "nsecs": stamp[1]},
        "T_end_s": float(plan.duration_s),
        "sample_count": len(times_ns),
        "interior_sample_count": interior,
        "start_qf": [request.start_qf.x, request.start_qf.y,
                     request.start_qf.psi],
        "goal_qf": [request.goal_qf.x, request.goal_qf.y,
                    request.goal_qf.psi],
        "goal_policy": request.goal_policy.value,
        "initial_source": request.initial.source,
        "initial_context_id": request.initial.context_id,
        "terminal_qf": [terminal.qf.x, terminal.qf.y, terminal.qf.psi],
        "terminal_headings": {
            vehicle_id: terminal.vehicle(vehicle_id).theta
            for vehicle_id in vehicle_ids},
        "terminal_heading_error_rad": terminal_angle,
        "terminal_position_error_m": terminal_position,
        "worst": worst,
        "document_path": str(path),
        "request_document_path": str(request_path),
    }
    return {
        "message": message,
        "request": request,
        "job": job,
        "context": context,
        "plan": plan,
        "fields": fields,
        "stamp": stamp,
        "summary": summary,
    }


# ---- display checks ---------------------------------------------------

class VirtualDisplayAccumulator(object):
    def __init__(self, plan, fields, prefix):
        self.plan = plan
        self.fields = fields
        self.stamp = fields["stamp"]
        self.prefix = prefix
        self.vehicle_ids = sorted(plan._request.model.vehicle_ids)
        self.worst = {
            "tf_position_m": 0.0, "tf_angle_rad": 0.0,
            "marker_position_m": 0.0, "marker_angle_rad": 0.0,
            "status_position_m": 0.0, "status_angle_rad": 0.0,
        }
        self.samples = {"tf": 0, "marker": 0, "status": 0}
        self._stamps = {"tf": set(), "marker": set(), "status": set()}
        self.history = []

    def _elapsed(self, stamp):
        # Epoch-sized float subtraction loses ~2.4e-7 s at 2026 timestamps;
        # subtract the common clock in integer nanoseconds instead.
        elapsed_ns = (
            int(stamp.secs) * 1000000000 + int(stamp.nsecs)
            - (self.stamp[0] * 1000000000 + self.stamp[1]))
        elapsed = elapsed_ns * 1e-9
        if elapsed < -1e-6 or elapsed > float(self.plan.duration_s) + 0.05:
            return None
        return elapsed

    def _state(self, elapsed):
        return self.plan.state_at(min(max(elapsed, 0.0),
                                      float(self.plan.duration_s)))

    def add_tf(self, message):
        for transform in message.transforms:
            child = transform.child_frame_id
            if not child.startswith(self.prefix):
                continue
            elapsed = self._elapsed(transform.header.stamp)
            if elapsed is None:
                continue
            stamp_ns = (int(transform.header.stamp.secs) * 1000000000 +
                        int(transform.header.stamp.nsecs))
            key = (child, stamp_ns)
            if key in self._stamps["tf"]:
                continue
            self._stamps["tf"].add(key)
            self.samples["tf"] += 1
            state = self._state(elapsed)
            body = transform.transform
            if child == self.prefix + "_reference":
                position_error = math.hypot(
                    body.translation.x - state.qf.x,
                    body.translation.y - state.qf.y)
                angle_error = abs(wrap(quaternion_yaw(body.rotation) -
                                       state.qf.psi))
            else:
                try:
                    vehicle_id = int(child.split("_vehicle_")[1].split("_")[0])
                except (IndexError, ValueError):
                    continue
                expected = state.vehicle(vehicle_id)
                position_error = math.hypot(
                    body.translation.x - expected.x,
                    body.translation.y - expected.y)
                angle_error = abs(wrap(quaternion_yaw(body.rotation) -
                                       expected.theta))
            self.worst["tf_position_m"] = max(
                self.worst["tf_position_m"], position_error)
            self.worst["tf_angle_rad"] = max(
                self.worst["tf_angle_rad"], angle_error)
            self.history.append({
                "source": "tf", "elapsed_s": elapsed,
                "child_frame_id": child, "position_error_m": position_error,
                "angle_error_rad": angle_error,
            })

    def add_markers(self, record):
        if record.count == 0 or record.delete_all:
            return
        elapsed = self._elapsed(record.stamp)
        if elapsed is None:
            return
        key = (int(record.stamp.secs) * 1000000000 +
           int(record.stamp.nsecs))
        if key in self._stamps["marker"]:
            return
        self._stamps["marker"].add(key)
        self.samples["marker"] += 1
        state = self._state(elapsed)
        for vehicle_id in self.vehicle_ids:
            namespace = vehicle_namespace(self.plan._request.formation_id,
                                          vehicle_id)
            arrow = next(
                (item for item in record.arrows if item[0] == namespace), None)
            if arrow is None:
                raise SmokeError(
                    "marker arrow for vehicle {} missing at elapsed {:.6f}"
                    .format(vehicle_id, elapsed))
            first, second = arrow[1], arrow[2]
            expected = state.vehicle(vehicle_id)
            position_error = math.hypot(first[0] - expected.x,
                                        first[1] - expected.y)
            angle_error = abs(wrap(
                math.atan2(second[1] - first[1], second[0] - first[0])
                - expected.theta))
            self.worst["marker_position_m"] = max(
                self.worst["marker_position_m"], position_error)
            self.worst["marker_angle_rad"] = max(
                self.worst["marker_angle_rad"], angle_error)
        self.history.append({
            "source": "marker", "elapsed_s": elapsed,
            "position_error_m": self.worst["marker_position_m"],
            "angle_error_rad": self.worst["marker_angle_rad"],
        })

    def add_status(self, document):
        if document.get("plan_id") != self.plan._request.plan_id:
            return
        if document.get("state") not in (PLAYING, HOLD_END):
            return
        if document.get("model_sha256") != self.plan.model_sha256():
            return
        if document.get("backend") != VIRTUAL_PLAYBACK_BACKEND:
            return
        elapsed = document.get("elapsed_s")
        if not isinstance(elapsed, (int, float)):
            return
        if elapsed < -1e-6 or elapsed > float(self.plan.duration_s) + 0.05:
            return
        key = round(float(elapsed), 9)
        if key in self._stamps["status"]:
            return
        self._stamps["status"].add(key)
        self.samples["status"] += 1
        state = self._state(float(elapsed))
        for vehicle in document.get("vehicles", []):
            expected = state.vehicle(int(vehicle["vehicle_id"]))
            center = vehicle["center"]
            position_error = math.hypot(center[0] - expected.x,
                                        center[1] - expected.y)
            angle_error = abs(wrap(vehicle["theta"] - expected.theta))
            self.worst["status_position_m"] = max(
                self.worst["status_position_m"], position_error)
            self.worst["status_angle_rad"] = max(
                self.worst["status_angle_rad"], angle_error)
        self.history.append({
            "source": "status", "elapsed_s": float(elapsed),
            "position_error_m": self.worst["status_position_m"],
            "angle_error_rad": self.worst["status_angle_rad"],
        })

    def summary(self):
        return {
            "samples": dict(self.samples),
            "worst": dict(self.worst),
            "history": list(self.history),
        }


def display_checks(checks, accumulator, prefix):
    note(checks, prefix + "display_tf_samples",
         accumulator.samples["tf"] >= DISPLAY_MIN_SAMPLES,
         accumulator.samples["tf"])
    note(checks, prefix + "display_marker_samples",
         accumulator.samples["marker"] >= DISPLAY_MIN_SAMPLES,
         accumulator.samples["marker"])
    note(checks, prefix + "display_status_samples",
         accumulator.samples["status"] >= DISPLAY_MIN_SAMPLES,
         accumulator.samples["status"])
    note(checks, prefix + "display_tf_position",
         accumulator.worst["tf_position_m"] <= MODEL_POSITION_TOL_M,
         accumulator.worst["tf_position_m"])
    note(checks, prefix + "display_tf_angle",
         accumulator.worst["tf_angle_rad"] <= MODEL_ANGLE_TOL_RAD,
         accumulator.worst["tf_angle_rad"])
    note(checks, prefix + "display_marker_position",
         accumulator.worst["marker_position_m"] <= MODEL_POSITION_TOL_M,
         accumulator.worst["marker_position_m"])
    note(checks, prefix + "display_marker_angle",
         accumulator.worst["marker_angle_rad"] <= MODEL_ANGLE_TOL_RAD,
         accumulator.worst["marker_angle_rad"])
    note(checks, prefix + "display_status_position",
         accumulator.worst["status_position_m"] <= MODEL_POSITION_TOL_M,
         accumulator.worst["status_position_m"])
    note(checks, prefix + "display_status_angle",
         accumulator.worst["status_angle_rad"] <= MODEL_ANGLE_TOL_RAD,
         accumulator.worst["status_angle_rad"])


# ---- master checks ----------------------------------------------------

def scene_bundle(manifest_path, scene_key):
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    entry = manifest["scenes"][scene_key]
    scene_dir = Path(manifest_path).resolve().parent / entry["input_dir"]
    grid, metadata = load_map(scene_dir / "map.yaml")
    signature = grid_signature(grid)
    note({}, "scene_signature", signature == entry["map_sha256"],
         {"computed": signature, "manifest": entry["map_sha256"]})
    note({}, "scene_request_sha_manifest",
         isinstance(entry.get("request_sha256"), str),
         entry.get("request_sha256"))
    payload = json.loads((scene_dir / "request.json").read_text(
        encoding="utf-8"))
    request = request_from_payload(payload["request"])
    note({}, "scene_request_sha256",
         request_fingerprint(request) == entry["request_sha256"],
         {"computed": request_fingerprint(request),
          "manifest": entry["request_sha256"]})
    return {
        "manifest": manifest,
        "entry": entry,
        "scene_dir": scene_dir,
        "grid": grid,
        "metadata": metadata,
        "request": request,
        "payload": payload,
    }


def check_runtime_map(session, bundle, checks, prefix):
    with session.lock:
        entry = session.maps[-1] if session.maps else None
    note(checks, prefix + "runtime_map_received", entry is not None,
         "no /map message")
    if entry is None:
        return None
    _, message = entry
    width = int(message.info.width)
    height = int(message.info.height)
    note(checks, prefix + "runtime_map_size",
         (width, height) == bundle["grid"].occ.shape[::-1],
         {"runtime": [width, height],
          "expected": list(bundle["grid"].occ.shape[::-1])})
    # nav_msgs/OccupancyGrid.resolution is float32; compare it with the
    # float64 YAML resolution without pretending the message retained the
    # original binary64 value.  Rebuild the occupancy grid at the admitted
    # YAML resolution so its signature is comparable to the G4 manifest.
    resolution = float(message.info.resolution)
    resolution_error = abs(resolution - bundle["grid"].res)
    note(checks, prefix + "runtime_map_resolution",
         resolution_error <= 1e-6,
         {"runtime_float32": resolution, "expected_float64":
          bundle["grid"].res, "error": resolution_error})
    note(checks, prefix + "runtime_map_origin",
         abs(float(message.info.origin.position.x) - bundle["grid"].ox) <= 1e-12
         and abs(float(message.info.origin.position.y) -
                 bundle["grid"].oy) <= 1e-12,
         [message.info.origin.position.x, message.info.origin.position.y])
    occupancy = [value >= 65 if value >= 0 else True for value in message.data]
    runtime = Grid(
        [occupancy[index:index + width]
         for index in range(0, len(occupancy), width)],
        bundle["grid"].res,
        float(message.info.origin.position.x),
        float(message.info.origin.position.y),
        length=.738, width=.500)
    runtime_sha = grid_signature(runtime)
    note(checks, prefix + "runtime_map_sha256",
         runtime_sha == bundle["entry"]["map_sha256"],
         {"computed": runtime_sha,
          "manifest": bundle["entry"]["map_sha256"]})
    return {
        "width": width, "height": height,
        "resolution_m": float(message.info.resolution),
        "origin": [float(message.info.origin.position.x),
                   float(message.info.origin.position.y)],
        "runtime_grid_sha256": runtime_sha,
        "manifest_map_sha256": bundle["entry"]["map_sha256"],
    }


# ---- experiment helpers ------------------------------------------------

def wait_ready(session, timeout=30.0):
    return session.wait_job(
        lambda d: d.get("backend") == VIRTUAL_PLAYBACK_BACKEND and
        d.get("state") in ("IDLE", "HOLD_END", "FAILED"), timeout,
        "virtual planner ready job state")


def confirm_reset(session, pose, checks, prefix):
    before = session.request_count()
    with session.lock:
        previous_contexts = {document.get("initial_context_id")
                             for _, document in session.job_states}
    publish_initial_pose(*pose)
    job = session.wait_job(
        lambda d: d.get("initial_reset_ready") is True and
        isinstance(d.get("initial_context_id"), str) and
        d["initial_context_id"] not in previous_contexts, 5.0,
        "explicit virtual reset confirmation")
    note(checks, prefix + "reset_context_id",
         isinstance(job.get("initial_context_id"), str),
         job.get("initial_context_id"))
    note(checks, prefix + "reset_source",
         job.get("initial_context_source") == "rviz_reset",
         job.get("initial_context_source"))
    note(checks, prefix + "reset_no_request", session.request_count() == before,
         session.request_count() - before)
    return job


def drive_goal(session, goal, checks, prefix, planning_timeout):
    before = session.request_count()
    publish_goal(*goal)
    request = session.wait_request_after(before, 5.0)
    plan_id = int(request.plan_id)
    note(checks, prefix + "request_plan_id", plan_id > 0, plan_id)
    note(checks, prefix + "request_goal",
         abs(float(request.formation_goal.x) - float(goal[0])) <= 1e-9 and
         abs(float(request.formation_goal.y) - float(goal[1])) <= 1e-9 and
         abs(wrap(float(request.formation_goal.theta) - float(goal[2]))) <= 1e-9,
         [request.formation_goal.x, request.formation_goal.y,
          request.formation_goal.theta])
    job = session.wait_job(
        lambda d: d.get("plan_id") == plan_id and d.get("state") == "PLANNING"
        and isinstance(d.get("planning_context"), dict)
        and isinstance(d.get("initial_context"), dict), 10.0,
        "admitted PLANNING job for plan {}".format(plan_id))
    message = session.wait_envelope(plan_id, planning_timeout)
    return request, job, message


def wait_hold_end(session, plan_id, timeout):
    document = session.wait_playback(
        lambda d: d.get("plan_id") == plan_id and d.get("state") == HOLD_END,
        timeout, "playback HOLD_END for plan {}".format(plan_id))
    job = session.wait_job(
        lambda d: d.get("plan_id") == plan_id and d.get("state") == HOLD_END,
        timeout, "job HOLD_END for plan {}".format(plan_id))
    return document, job


def check_plan_halt(session, plan_id, checks, prefix):
    history = session.job_history(plan_id)
    states = [document.get("state") for _, document in history]
    note(checks, prefix + "job_sequence",
         "PLANNING" in states and "PLAYING" in states and
         states[-1] == "HOLD_END", states)
    playback = session.playback_history(plan_id)
    pstates = [document.get("state") for _, document in playback]
    playing_at = [index for index, state in enumerate(pstates)
                  if state == PLAYING]
    hold_end_at = [index for index, state in enumerate(pstates)
                   if state == HOLD_END]
    # The envelope stamp is already in the past when the receiver accepts it,
    # so a real run can begin on PLAYING without an elapsed<=0 HOLD_START
    # tick.  The required sequence is PLAYING followed by HOLD_END; the
    # pre-playback STATIC hold is not a failure.
    note(checks, prefix + "playback_sequence",
         bool(playing_at) and bool(hold_end_at) and
         min(hold_end_at) > min(playing_at), pstates)
    hold_times = [received for received, document in playback
                  if document.get("state") == HOLD_END]
    job_hold_times = [received for received, document in history
                      if document.get("state") == HOLD_END]
    note(checks, prefix + "hold_end_after_playback",
         bool(hold_times) and bool(job_hold_times) and
         min(job_hold_times) >= min(hold_times) - 0.05,
         {"playback": hold_times[:1], "job": job_hold_times[:1]})
    return {"job_states": states, "playback_states": sorted(set(pstates))}


def busy_probe(session, request_message, plan_id, checks, prefix):
    session.wait_job(
        lambda d: d.get("plan_id") == plan_id and
        d.get("state") in ("PLANNING", "PLAYING"), 2.0,
        "busy job state for plan {}".format(plan_id))
    with session.lock:
        request_before = len(session.requests)
        playback_before = len(session.playback)
    publish_goal(9.0, 9.0, 0.0)
    publish_initial_pose(8.0, 8.0, 0.0)
    rospy.sleep(0.8)
    with session.lock:
        new_requests = session.requests[request_before:]
        playback_after = session.playback[playback_before:]
        job_after = session.job_states[-1][1] if session.job_states else None
    note(checks, prefix + "busy_bridge_no_goal_or_reset_request",
         len(new_requests) == 0, len(new_requests))
    note(checks, prefix + "busy_job_unchanged",
         job_after is not None and job_after.get("plan_id") == plan_id and
         job_after.get("state") in ("PLANNING", "PLAYING"), job_after)
    note(checks, prefix + "busy_playback_still_running",
         any(document.get("plan_id") == plan_id and
             document.get("state") == PLAYING
             for _, document in playback_after),
         len(playback_after))

    # Direct planner injection bypasses the bridge.
    fake = copy.deepcopy(request_message)
    fake_id = plan_id + 500000
    fake.plan_id = fake_id
    fake.header.stamp = rospy.Time.now()
    publisher = rospy.Publisher(REQUEST_TOPIC, FormationPlanRequest,
                                queue_size=1)
    rospy.sleep(0.2)
    publisher.publish(fake)
    deadline = rospy.Time.now().to_sec() + 3.0
    busy_status = None
    while rospy.Time.now().to_sec() < deadline and busy_status is None:
        with session.lock:
            for _, document in reversed(session.statuses):
                if document.get("plan_id") == fake_id and (
                        document.get("reason") == "BUSY" or
                        document.get("status") == "BUSY_REJECTED"):
                    busy_status = document
                    break
        rospy.sleep(0.02)
    publisher.unregister()
    note(checks, prefix + "planner_busy_rejection", busy_status is not None,
         busy_status)
    note(checks, prefix + "planner_busy_no_envelope",
         fake_id not in session.envelopes, fake_id)
    with session.lock:
        job_final = session.job_states[-1][1] if session.job_states else None
    note(checks, prefix + "planner_busy_keeps_job",
         job_final is not None and job_final.get("plan_id") == plan_id and
         job_final.get("state") in ("PLANNING", "PLAYING"), job_final)
    return {
        "fake_plan_id": fake_id,
        "busy_status_reason": None if busy_status is None
        else busy_status.get("reason"),
        "new_requests": len(new_requests),
    }


def wait_playing(session, plan_id, message, timeout=30.0):
    """Wait until playback reports the exact certified plan as PLAYING.

    The plan clock starts when the planner stamps the trajectory.  Playback
    needs time to verify the envelope independently, and until it accepts the
    plan it keeps publishing the static hold with wall-clock stamps.  A hold
    stamp can therefore fall a few milliseconds inside the plan window while
    its geometry is still the t=0 start pose.  Only after the matching plan is
    PLAYING does the node publish frames generated from the trajectory model,
    so display agreement starts there.  Comparing the hold with a model that
    had not yet been accepted would be a harness bug, not a display error.
    """
    deadline = rospy.Time.now().to_sec() + timeout
    while rospy.Time.now().to_sec() < deadline:
        with session.lock:
            for received, document in session.playback:
                if (document.get("plan_id") == plan_id and
                        document.get("state") == PLAYING and
                        document.get("backend") == VIRTUAL_PLAYBACK_BACKEND and
                        document.get("model_sha256") ==
                        message.model_sha256):
                    return received, document
        rospy.sleep(0.01)
    raise SmokeError(
        "timeout waiting for playback PLAYING for plan {}".format(plan_id))


def run_verified_plan(session, bundle, goal, checks, prefix, args,
                      evidence_dir, reset=None, busy=False,
                      capture_dir=None):
    expected_initial = None
    if reset is not None:
        reset_job = confirm_reset(session, reset, checks, prefix)
        expected_initial = {
            "source": "rviz_reset",
            "context_id": reset_job.get("initial_context_id"),
            "qf": {"x": float(reset[0]), "y": float(reset[1]),
                   "psi": float(reset[2])},
        }
    request, job, message = drive_goal(
        session, goal, checks, prefix, args.planning_timeout)
    plan_id = int(message.plan_id)
    # Display agreement starts only when the receiver confirms this exact
    # plan as PLAYING.  Envelope publication alone is not enough: the node
    # verifies independently and may publish static hold ticks in the gap.
    wait_playing(session, plan_id, message)
    mark = session.capture_start()
    if busy:
        busy_result = busy_probe(session, request, plan_id, checks, prefix)
    else:
        busy_result = None
    verified = analyze_envelope(
        session, request, job, message, checks, prefix,
        evidence_dir / "envelopes", evidence_dir / "requests")
    accumulator = VirtualDisplayAccumulator(
        verified["plan"], verified["fields"],
        tf_frame_prefix(message.formation_id))
    note(checks, prefix + "initial_job_source",
         verified["request"].initial.source ==
         ("rviz_reset" if reset is not None else "verified_terminal"),
         verified["request"].initial.source)
    if expected_initial is not None:
        note(checks, prefix + "initial_reset_binding",
             verified["request"].initial.context_id ==
             expected_initial["context_id"] and
             abs(verified["request"].start_qf.x - expected_initial["qf"]["x"])
             <= 1e-9 and
             abs(verified["request"].start_qf.y - expected_initial["qf"]["y"])
             <= 1e-9 and
             abs(wrap(verified["request"].start_qf.psi -
                      expected_initial["qf"]["psi"])) <= 1e-9,
             expected_initial)
    capture_paths = []
    if capture_dir is not None:
        start_path = capture_dir / "{}_start.png".format(prefix)
        capture_paths.append({"label": "start", "path": str(start_path),
                              "result": screenshot(start_path)})
        mid_path = capture_dir / "{}_mid.png".format(prefix)

        def capture_mid():
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                latest = session.latest_playback()
                if latest is not None and latest.get("plan_id") == plan_id:
                    elapsed = latest.get("elapsed_s")
                    if isinstance(elapsed, (int, float)) and \
                            elapsed >= float(verified["plan"].duration_s) / 2.0:
                        break
                rospy.sleep(0.1)
            capture_paths.append({
                "label": "mid", "path": str(mid_path),
                "result": screenshot(mid_path)})

        thread = threading.Thread(target=capture_mid, name="g5-capture-mid")
        thread.start()
    hold, job_hold = wait_hold_end(
        session, plan_id, float(verified["plan"].duration_s) + 15.0)
    if capture_dir is not None:
        thread.join(timeout=5.0)
        end_path = capture_dir / "{}_end.png".format(prefix)
        capture_paths.append({"label": "end", "path": str(end_path),
                              "result": screenshot(end_path)})
    note(checks, prefix + "hold_end_identity",
         hold.get("plan_id") == plan_id and
         hold.get("formation_id") == message.formation_id and
         hold.get("model_sha256") == message.model_sha256 and
         hold.get("backend") == VIRTUAL_PLAYBACK_BACKEND, hold)
    note(checks, prefix + "job_hold_identity",
         job_hold.get("plan_id") == plan_id and
         job_hold.get("model_sha256") == message.model_sha256 and
         job_hold.get("backend") == VIRTUAL_PLAYBACK_BACKEND, job_hold)
    note(checks, prefix + "status_executable",
         hold.get("executable") is True and hold.get("geometry_only") is False,
         {"executable": hold.get("executable"),
          "geometry_only": hold.get("geometry_only")})
    note(checks, prefix + "terminal_errors",
         isinstance(hold.get("terminal_center_error_m"), (int, float)) and
         isinstance(hold.get("terminal_heading_error_rad"), (int, float)), hold)
    with session.lock:
        markers = [record for _, record in session.markers[mark[1]:]]
        transforms = [message for _, message in session.transforms[mark[2]:]]
        statuses = [document for _, document in session.playback[mark[0]:]]
    for record in markers:
        accumulator.add_markers(record)
    for transform in transforms:
        accumulator.add_tf(transform)
    for document in statuses:
        accumulator.add_status(document)
    display = accumulator.summary()
    write_json(evidence_dir / "display" / "{}_display.json".format(prefix),
               display)
    display_checks(checks, accumulator, prefix)
    sequence = check_plan_halt(session, plan_id, checks, prefix)
    return {
        "verified": verified,
        "busy": busy_result,
        "hold": hold,
        "job_hold": job_hold,
        "display": display,
        "sequence": sequence,
        "captures": capture_paths,
        "start_qf": [
            verified["request"].start_qf.x,
            verified["request"].start_qf.y,
            verified["request"].start_qf.psi],
        "terminal_qf": verified["summary"]["terminal_qf"],
        "terminal_headings": verified["summary"]["terminal_headings"],
        "plan_id": plan_id,
    }


def expect_rejection(session, goal, expected_reason, checks, prefix,
                     timeout=10.0):
    before = session.request_count()
    publish_goal(*goal)
    request = session.wait_request_after(before, 5.0)
    plan_id = int(request.plan_id)
    job = session.wait_job(
        lambda d: d.get("plan_id") == plan_id and d.get("state") == "FAILED",
        timeout, "planner rejection for plan {}".format(plan_id))
    note(checks, prefix + "rejection_plan_id", plan_id > 0, plan_id)
    note(checks, prefix + "rejection_reason",
         job.get("reason") == expected_reason,
         {"reason": job.get("reason"), "expected": expected_reason,
          "detail": job.get("detail")})
    note(checks, prefix + "rejection_no_envelope",
         plan_id not in session.envelopes, plan_id)
    with session.lock:
        playback = [(received, document)
                    for received, document in session.playback
                    if document.get("plan_id") == plan_id]
    animating = [
        document for _, document in playback
        if document.get("state") in (PLAYING, HOLD_START, HOLD_END) or
        document.get("executable") is True or
        document.get("trajectory_published") is True]
    note(checks, prefix + "rejection_no_playback", not animating,
         [document.get("state") for _, document in playback])
    return {"plan_id": plan_id, "job": job}


def failure_experiment(session, bundle, checks, args, evidence_dir,
                       capture_dir):
    first = run_verified_plan(
        session, bundle, args.valid_goal, checks, "plan1", args,
        evidence_dir, reset=(0.0, 0.0, 0.0), capture_dir=capture_dir)
    # The accepted STRAIGHT terminal is a supported continuation.
    note(checks, "failure_precondition_terminal",
         first["job_hold"].get("initial_context_source") ==
         "verified_terminal", first["job_hold"].get("initial_context_source"))
    clear_mark = session.capture_start()
    before = session.request_count()
    publish_goal(*args.failure_goal)
    failed_request = session.wait_request_after(before, 5.0)
    failed_id = int(failed_request.plan_id)
    job = session.wait_job(
        lambda d: d.get("plan_id") == failed_id and
        d.get("state") in ("PLANNING", "FAILED"), 10.0,
        "failure plan admitted")
    # A* may exhaust the grid within the same status cycle that admitted the
    # request, so the first observed job document can already be FAILED.  The
    # admission record and matching plan id, not the transient PLANNING label,
    # are what prove the failure was admitted rather than rejected.
    admission = job.get("admission") or {}
    note(checks, "failure_admitted",
         job.get("state") in ("PLANNING", "FAILED") and
         job.get("plan_id") == failed_id and
         admission.get("plan_id") == failed_id,
         job)
    failed = session.wait_job(
        lambda d: d.get("plan_id") == failed_id and d.get("state") == "FAILED",
        args.planning_timeout, "failure plan terminal FAILED")
    with session.lock:
        failed_envelope = session.envelopes.get(failed_id)
    executable = (
        failed_envelope is not None and bool(failed_envelope.model_json) and
        len(failed_envelope.trajectory.times_from_start) > 0)
    note(checks, "failure_no_executable_envelope", not executable,
         None if failed_envelope is None else {
             "model_json_length": len(failed_envelope.model_json),
             "sample_count": len(
                 failed_envelope.trajectory.times_from_start)})
    with session.lock:
        clears = [
            (received, document)
            for received, document in session.playback[clear_mark[0]:]
            if document.get("cleared") is True and
            (document.get("plan_id") == failed_id or
             "new experiment {} admitted".format(failed_id) in
             str(document.get("reason")) or
             "plan {}".format(failed_id) in str(document.get("reason")))]
    note(checks, "failure_clears_old_animation", bool(clears),
         [(document.get("plan_id"), document.get("reason"))
          for _, document in clears])
    clear = clears[0][1] if clears else {}
    # The cleared status is published just before the atomic DELETEALL+hold
    # marker array; sample the marker stream with a short bounded wait instead
    # of racing the in-flight marker message.
    deadline = time.monotonic() + 5.0
    marker_after = []
    while time.monotonic() < deadline:
        with session.lock:
            marker_after = session.markers[clear_mark[1]:]
        if any(record.delete_all for _, record in marker_after):
            break
        rospy.sleep(0.02)
    note(checks, "failure_marker_clear",
         bool(marker_after) and
         any(record.delete_all for _, record in marker_after),
         [(record.count, record.delete_all) for _, record in marker_after])
    clear_image = ((capture_dir if capture_dir is not None
                    else evidence_dir / "images") / "failure_cleared.png")
    clear_capture = screenshot(clear_image)
    note(checks, "failure_clear_screenshot", clear_capture.get("ok") is True,
         clear_capture)
    note(checks, "failure_failed_job",
         failed.get("reason") not in (None, ""), failed)

    recovery = run_verified_plan(
        session, bundle, args.valid_goal, checks, "recovery", args,
        evidence_dir, reset=(0.0, 0.0, 0.0), capture_dir=capture_dir)
    note(checks, "failure_reset_recovered", True,
         recovery["plan_id"])
    return {
        "first": first,
        "failed_plan_id": failed_id,
        "failed_job": failed,
        "clear_status": clear,
        "clear_capture": clear_capture,
        "recovery": recovery,
    }


# ---- tamper ------------------------------------------------------------

def tamper_fields(document):
    def fields():
        return copy.deepcopy(document["trajectory_fields"])

    target_vehicle = document["trajectory_fields"]["trajectories"][0]
    index = len(target_vehicle["points"]) // 2

    def mutable():
        value = fields()
        points = [list(point) for point in
                  value["trajectories"][0]["points"]]
        value["trajectories"][0]["points"] = points
        return value, points

    def position():
        value, points = mutable()
        points[index][1] += 1e-3
        return value

    def heading():
        value, points = mutable()
        points[index][3] += 1e-3
        return value

    def speed():
        value, points = mutable()
        points[index][4] += 1e-3
        return value

    digest = document["model_sha256"]
    flipped = ("0" if digest[0] != "0" else "1") + digest[1:]
    stamp = (document["header"]["secs"], document["header"]["nsecs"])
    return [
        ("unknown_semantic_version",
         lambda: build_envelope_message(
             document, semantic_version="virtual_rigid_a.playback.v0")),
        ("unknown_model_version",
         lambda: build_envelope_message(
             document, model_version="virtual_rigid_a.timed.v0")),
        ("wrong_digest",
         lambda: build_envelope_message(document, model_sha256=flipped)),
        ("tampered_position",
         lambda: build_envelope_message(document, fields=position())),
        ("tampered_heading",
         lambda: build_envelope_message(document, fields=heading())),
        ("tampered_speed",
         lambda: build_envelope_message(document, fields=speed())),
        ("tampered_time",
         lambda: tamper_time(document, stamp)),
    ]


def tamper_time(document, stamp):
    message = build_envelope_message(document)
    index = len(message.trajectory.times_from_start) // 2
    value = duration_ns(message.trajectory.times_from_start[index]) + 1
    secs, nsecs = divmod(value, 10 ** 9)
    message.trajectory.times_from_start[index] = rospy.Duration(secs, nsecs)
    for trajectory in message.trajectory.trajectories:
        trajectory.points[index].time_from_start = rospy.Duration(secs, nsecs)
    return message


def tamper_experiment(session, checks, document, evidence_dir):
    plan_id = int(document["plan_id"])
    initial_job = session.wait_job(
        lambda d: d.get("plan_id") == plan_id and
        d.get("state") == "HOLD_END", 10.0,
        "baseline HOLD_END for tamper")
    note(checks, "tamper_baseline_plan", plan_id > 0, plan_id)
    note(checks, "tamper_baseline_envelope",
         plan_id in session.envelopes, plan_id)
    results = []
    for name, factory in tamper_fields(document):
        mark = session.capture_start()
        message = factory()
        publish_envelope(message)
        try:
            status = session.wait_playback(
                lambda d, mark=mark: d.get("cleared") is True and
                d.get("state") in ("CLEARED", "INVALID"),
                3.0, "rejection/clear for {}".format(name), mark=mark[0])
        except SmokeError as exc:
            results.append({"case": name, "ok": False, "detail": str(exc)})
            note(checks, "tamper_" + name, False, str(exc))
            continue
        with session.lock:
            playback_after = session.playback[mark[0]:]
        tampered_playing = [
            document for _, document in playback_after
            if document.get("plan_id") == plan_id and
            document.get("state") in (PLAYING, HOLD_START)]
        note(checks, "tamper_{}_cleared".format(name),
             status.get("cleared") is True, status)
        note(checks, "tamper_{}_no_playing".format(name),
             not tampered_playing, len(tampered_playing))
        note(checks, "tamper_{}_reason_present".format(name),
             bool(status.get("reason")), status.get("reason"))
        results.append({
            "case": name,
            "ok": True,
            "status": status,
            "publisher": envelope_document(message)["model_sha256"],
        })
        rospy.sleep(0.2)
    baseline = build_envelope_message(document)
    mark = session.capture_start()
    publish_envelope(baseline)
    restored = session.wait_playback(
        lambda d: d.get("plan_id") == plan_id and
        d.get("state") == HOLD_END, 5.0,
        "genuine envelope accepted after tamper", mark=mark[0])
    note(checks, "tamper_genuine_replay_accepted",
         restored.get("model_sha256") == document["model_sha256"], restored)
    write_json(evidence_dir / "tamper_results.json", results)
    return {"baseline_plan_id": plan_id, "cases": results,
            "restored": restored}


# ---- scenario entry ----------------------------------------------------

def scenario_main(session, bundle, checks, args, evidence_dir, capture_dir):
    wait_ready(session)
    map_summary = check_runtime_map(session, bundle, checks, "main")
    policy = str(args.goal_policy).lower()
    note(checks, "main_policy", policy in ("free", "straight"), policy)
    first = run_verified_plan(
        session, bundle, args.main_goal, checks, "plan1", args,
        evidence_dir, reset=(0.0, 0.0, 0.0),
        busy=(policy == "straight"), capture_dir=capture_dir)
    note(checks, "main_policy_matches_job",
         str(first["verified"]["job"].get("virtual_a_goal_policy", "")).lower()
         == policy,
         first["verified"]["job"].get("virtual_a_goal_policy"))
    continuation = None
    if policy == "free":
        # S1/S2 FREE terminals have a non-zero C_i at kappa_F=0; the adapter
        # must refuse a normal continuation without treating the hold as a
        # successful request.
        continuation = expect_rejection(
            session, args.free_continuation_goal, "UNSUPPORTED_INITIAL_STATE",
            checks, "free_continuation")
        note(checks, "free_terminal_incompatible",
             first["job_hold"].get("initial_compatible") is False,
             first["job_hold"].get("initial_compatible"))
        note(checks, "free_hold_not_reported_as_success",
             first["verified"]["summary"]["goal_policy"] == "FREE",
             first["verified"]["summary"]["goal_policy"])
    else:
        second = run_verified_plan(
            session, bundle, args.straight_second_goal, checks, "plan2", args,
            evidence_dir, reset=None, capture_dir=capture_dir)
        note(checks, "straight_second_plan_id",
             second["plan_id"] > first["plan_id"], second["plan_id"])
        note(checks, "straight_second_start_matches_first_terminal",
             math.hypot(second["start_qf"][0] - first["terminal_qf"][0],
                        second["start_qf"][1] - first["terminal_qf"][1])
             <= 1e-6 and
             abs(wrap(second["start_qf"][2] - first["terminal_qf"][2])) <= 1e-6,
             {"first": first["terminal_qf"], "second": second["start_qf"]})
        note(checks, "straight_second_no_reset",
             second["verified"]["request"].initial.source ==
             "verified_terminal",
             second["verified"]["request"].initial.source)
        continuation = second
    return {
        "map": map_summary,
        "first": first,
        "continuation": continuation,
    }


def scenario_failure(session, bundle, checks, args, evidence_dir,
                     capture_dir):
    wait_ready(session)
    map_summary = check_runtime_map(session, bundle, checks, "failure")
    result = failure_experiment(
        session, bundle, checks, args, evidence_dir, capture_dir)
    result["map"] = map_summary
    return result


def scenario_tamper(session, checks, args, evidence_dir):
    wait_ready(session)
    path = Path(args.baseline_envelope)
    note(checks, "tamper_baseline_file", path.is_file(), str(path))
    document = json.loads(path.read_text(encoding="utf-8"))
    note(checks, "tamper_baseline_semantic_version",
         document.get("semantic_version") == VIRTUAL_PLAYBACK_SEMANTIC_VERSION,
         document.get("semantic_version"))
    note(checks, "tamper_baseline_backend",
         document.get("backend") == VIRTUAL_PLAYBACK_BACKEND,
         document.get("backend"))
    return tamper_experiment(session, checks, document, evidence_dir)


def map05_uturn_case(session, start, goal, checks, prefix, args,
                     evidence_dir, capture_dir):
    reset = confirm_reset(session, start, checks, prefix)
    before = session.request_count()
    publish_goal(*goal)
    request = session.wait_request_after(before, 5.0)
    plan_id = int(request.plan_id)
    planning = session.wait_job(
        lambda d: d.get("plan_id") == plan_id and d.get("state") == "PLANNING",
        10.0, "U-turn planning job")
    note(checks, prefix + "reset_used",
         planning.get("initial_context_id") == reset["initial_context_id"],
         planning.get("initial_context_id"))
    terminal = session.wait_job(
        lambda d: d.get("plan_id") == plan_id and
        d.get("state") in ("PLAYING", "HOLD_END", "FAILED"),
        args.planning_timeout, "U-turn publication or rejection")
    # The job and optimizer status use separate topics. Read the latched
    # result after the job transition instead of racing the recorder callback.
    status = json.loads(rospy.wait_for_message(STATUS_TOPIC, String, timeout=5.0).data)
    note(checks, prefix + "status_plan_id", status.get("plan_id") == plan_id,
         status.get("plan_id"))
    virtual = status.get("virtual_result") or {}
    audits = virtual.get("audits") or {}
    result = {
        "start_f_m_rad": start,
        "goal_f_m_rad": goal,
        "plan_id": plan_id,
        "grid_sha256": planning.get("grid_sha256"),
        "guidance": planning["planning_context"]["planner_config"]["guidance_mode"],
        "job_state": terminal["state"],
        "status": status.get("status"),
        "pipeline_status": status.get("pipeline_status"),
        "trajectory_published": status.get("trajectory_published"),
        "reason": terminal.get("reason") or status.get("reason"),
        "detail": terminal.get("detail") or status.get("detail"),
        "first_failure_layer": status.get("first_failure_layer"),
        "planning_wall_time_s": status.get("wall_time_s"),
        "audits_passed": {name: None if audit is None else audit["passed"]
                          for name, audit in audits.items()},
    }
    cover = virtual.get("search", {}).get("footprint_cover")
    if cover:
        result["footprint_cover"] = {key: value for key, value in cover.items()
                                     if key != "regions"}
    verification = status.get("verification")
    if verification:
        result["playback_confirmation_window_s"] = verification.get("playback_confirmation_window_s")
    print("{} {} {}".format(prefix, result["job_state"], result["reason"]),
          flush=True)
    note(checks, prefix + "certified_turn", terminal["state"] != "FAILED",
         result["reason"] or result["status"], required=False)
    if terminal["state"] == "FAILED":
        envelope = rospy.wait_for_message(PLAN_TOPIC, FormationPlaybackPlan, timeout=5.0)
        with session.lock:
            envelopes = [m for _, m in session.envelope_records if m.plan_id == plan_id]
        note(checks, prefix + "failure_no_executable_envelope",
             envelope.plan_id == plan_id and not any(
                 m.model_json or m.trajectory.times_from_start for m in [*envelopes, envelope]),
             {"latched_plan_id": envelope.plan_id, "failed_plan_id": plan_id})
        note(checks, prefix + "failure_no_playback", not any(
            d.get("executable") or d.get("state") in (PLAYING, HOLD_START, HOLD_END)
            for _, d in session.playback_history(plan_id)))
        if capture_dir is not None:
            result["capture"] = screenshot(capture_dir / (prefix + "failed.png"))
        return result

    message = session.wait_envelope(plan_id, 10.0)
    wait_playing(session, plan_id, message)
    if capture_dir is not None:
        result["capture"] = screenshot(capture_dir / (prefix + "playing.png"))
    verified = analyze_envelope(
        session, request, planning, message, checks, prefix,
        evidence_dir / "envelopes", evidence_dir / "requests")
    hold, job_hold = wait_hold_end(session, plan_id, verified["plan"].duration_s + 15.0)
    note(checks, prefix + "terminal_hold",
         hold.get("executable") is True and
         hold.get("model_sha256") == job_hold.get("model_sha256") == message.model_sha256,
         hold.get("state"))
    summary = verified["summary"]
    yaw_error = abs(wrap(summary["terminal_qf"][2] - goal[2]))
    note(checks, prefix + "terminal_f_heading", yaw_error <= TERMINAL_ANGLE_TOL_RAD,
         yaw_error)
    result.update(job_state=job_hold["state"], playback_state=hold["state"],
                  duration_s=summary["T_end_s"], terminal_f=summary["terminal_qf"],
                  terminal_f_yaw_error_rad=yaw_error, model_sha256=message.model_sha256)
    note(checks, prefix + "all_audits", result["audits_passed"] ==
         {"geometry": True, "dynamics": True, "collision": True}, result["audits_passed"])
    return result


def scenario_uturn(session, checks, args, evidence_dir, capture_dir):
    ready = session.wait_job(
        lambda d: d.get("backend") == VIRTUAL_PLAYBACK_BACKEND and
        d.get("state") == "IDLE" and d.get("map_ready"), 45.0,
        "Virtual A planner and map05 grid")
    if ready.get("map_signature") != MAP05_UTURN_MAP_SIGNATURE:
        raise SmokeError("U-turn benchmark requires map05 at scale 2.625")
    if str(ready.get("virtual_a_goal_policy", "")).lower() != "free":
        raise SmokeError("U-turn benchmark requires virtual_a_goal_policy:=free")
    results = {}
    for name, start, goal in MAP05_UTURN_CASES:
        try:
            results[name] = map05_uturn_case(
                session, start, goal, checks, name + "_", args, evidence_dir, capture_dir)
        except (SmokeError, PlaybackValidationError, rospy.ROSException) as exc:
            results[name] = {"start_f_m_rad": start, "goal_f_m_rad": goal,
                             "error": str(exc)}
            note(checks, name + "_completed", False, exc, required=False)
    return results


def parse_args():
    parser = argparse.ArgumentParser(
        description="G5-B real ROS virtual_rigid_a smoke driver")
    parser.add_argument("--scene-manifest")
    parser.add_argument("--scene-key", default="S1")
    parser.add_argument("--scenario", choices=("main", "failure", "tamper", "uturn"),
                        required=True)
    parser.add_argument("--goal-policy", choices=("free", "straight"),
                        default=None)
    parser.add_argument("--output", required=True)
    parser.add_argument("--capture-dir", default="")
    parser.add_argument("--baseline-envelope", default="")
    parser.add_argument("--planning-timeout", type=float, default=120.0)
    parser.add_argument("--main-goal", default="")
    parser.add_argument("--straight-second-goal", default="")
    parser.add_argument("--free-continuation-goal", default="")
    parser.add_argument("--valid-goal", default="")
    parser.add_argument("--failure-goal", default="")
    args = parser.parse_args()
    if args.scenario != "uturn" and not args.scene_manifest:
        parser.error("--scene-manifest is required for this scenario")
    return args


def parsed_goal(value, default):
    if not value:
        return default
    parts = [float(item) for item in value.split(",")]
    if len(parts) != 3:
        raise SystemExit("goal must be x,y,theta")
    return tuple(parts)


def main():
    args = parse_args()
    master = os.environ.get("ROS_MASTER_URI", "")
    port = None
    if master:
        try:
            port = int(master.rstrip("/").rsplit(":", 1)[1])
        except (IndexError, ValueError):
            port = None
    if port is None or port in PROTECTED_PORTS:
        raise SystemExit(
            "refusing protected/default master {!r}".format(master))
    rospy.init_node("virtual_a_smoke", anonymous=True)
    output = Path(args.output).resolve()
    evidence_dir = output.parent
    capture_dir = (Path(args.capture_dir).resolve()
                   if args.capture_dir else evidence_dir / "images")
    evidence_dir.mkdir(parents=True, exist_ok=True)
    bundle = (None if args.scenario == "uturn" else
              scene_bundle(args.scene_manifest, args.scene_key))
    defaults = {
        "S1": {
            "main": (8.819581504169413, 3.4858722492682084,
                     1.5707963267948968),
            "straight_second": (6.0, 1.0, 0.0),
            "free_continuation": (6.0, 1.0, 0.0),
            "valid": (8.819581504169413, 3.4858722492682084,
                      1.5707963267948968),
            "failure": (6.0, 1.0, 0.0),
        },
        "S2": {
            "main": (11.474331036127456, 0.0, 0.0),
            "straight_second": (13.0, 0.0, 0.0),
            "free_continuation": (13.0, 0.0, 0.0),
            "valid": (11.474331036127456, 0.0, 0.0),
            "failure": (15.5, 0.0, 0.0),
        },
    }
    values = defaults.get(args.scene_key, defaults["S2"])
    args.main_goal = parsed_goal(args.main_goal, values["main"])
    args.straight_second_goal = parsed_goal(
        args.straight_second_goal, values["straight_second"])
    args.free_continuation_goal = parsed_goal(
        args.free_continuation_goal, values["free_continuation"])
    args.valid_goal = parsed_goal(args.valid_goal, values["valid"])
    args.failure_goal = parsed_goal(args.failure_goal, values["failure"])
    if args.goal_policy is None:
        args.goal_policy = "free" if args.scene_key == "S1" else "straight"
    if args.scenario == "tamper" and not args.baseline_envelope:
        raise SystemExit("--scenario tamper requires --baseline-envelope")
    session = Session()
    checks = {}
    started = rospy.Time.now().to_sec()
    report = {
        "schema": "virtual_rigid_a.g5.real_smoke.v1",
        "ros_master_uri": master,
        "scenario": args.scenario,
        "goal_policy": args.goal_policy if bundle is not None else "free",
        "started_epoch_s": started,
        "checks": checks,
    }
    if bundle is None:
        report.update(scene_key="map05", map_scale=2.625)
    else:
        report.update(
            scene_key=args.scene_key,
            scene_manifest=str(Path(args.scene_manifest).resolve()),
            scene_manifest_sha256=__import__("hashlib").sha256(
                Path(args.scene_manifest).read_bytes()).hexdigest(),
            scene_manifest_entry=bundle["entry"], scene_input_dir=str(bundle["scene_dir"]))
    rospy.sleep(1.0)
    try:
        if args.scenario == "main":
            result = scenario_main(
                session, bundle, checks, args, evidence_dir, capture_dir)
        elif args.scenario == "failure":
            result = scenario_failure(
                session, bundle, checks, args, evidence_dir, capture_dir)
        elif args.scenario == "uturn":
            result = scenario_uturn(session, checks, args, evidence_dir,
                                    capture_dir if args.capture_dir else None)
        else:
            result = scenario_tamper(session, checks, args, evidence_dir)
        report["result"] = result
        report["passed"] = all(item["ok"] for item in checks.values())
    except Exception as exc:
        report["error"] = "{}: {}".format(type(exc).__name__, exc)
        report["passed"] = False
        raise
    finally:
        report["finished_epoch_s"] = rospy.Time.now().to_sec()
        report["elapsed_s"] = report["finished_epoch_s"] - started
        report["check_count"] = len(checks)
        report["failed_checks"] = [
            name for name, item in checks.items() if not item["ok"]]
        write_json(output, report)
    print("G5 virtual smoke {}: {} checks, {} failed".format(
        "PASS" if report["passed"] else "FAIL", len(checks),
        len(report["failed_checks"])))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(130)
