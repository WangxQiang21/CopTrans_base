#!/usr/bin/env python3
"""FR05 live ROS acceptance driver for the V0.2 atomic playback envelope.

This script attaches to a private ROS master where the real ``run_demo``
launch is already running (``formation_demo.launch``, ``backend:=uniform_v02``)
and drives the same RViz-style user flow with the standard topics:

* ``/initialpose``       2D Pose Estimate
* ``/move_base_simple/goal`` 2D Nav Goal

Scenarios
---------
``main``     forward A plan, O lateral plan, unified S->A plan with a
             non-zero cargo pivot and zero-angle vehicles, busy rejection of
             Goal/InitialPose during playback, and a second experiment after
             the authoritative HOLD_END.
``failure``  a reachable plan followed by an admitted planning failure on the
             blocked map; no executable envelope may remain, the old animation
             must stop, and the failed job must admit a new experiment.
``tamper``   a captured genuine envelope is replayed through
             ``/formation/playback_plan`` and then mutated (unknown
             semantic/model version, digest, model JSON, samples, stamps);
             playback must reject every mutation and clear the display.

Every scenario verifies, for each admitted plan:

* the envelope is self-consistent (stamp, identity, model digest, samples vs
  the authoritative model at >= 11 interior sample times per phase);
* each vehicle centre is derived from ``q_F`` and the model offsets;
* TF, MarkerArray and playback-status positions/angles agree with the model at
  the same wall-clock instants (common threshold);
* all states share the same job / plan_id / model hash;
* the job state leaves PLANNING/PLAYING only when playback itself reaches
  HOLD_END, never merely because a trajectory was published.

The driver does not start nodes, publish trajectories to the planner topic, or
kill any process.  It writes a strict-JSON report to ``--output`` and stores
each full envelope beside it.  Run it with the ROS environment sourced (the
``run_demo`` shell, or ``source /opt/ros/noetic/setup.bash`` plus
``catkin_ws/devel/setup.bash``).
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
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
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from fwis_v3_msgs.msg import (
    FormationPlaybackPlan,
    FormationPlanRequest,
    FormationTimedTrajectory,
    FormationVehicleSpec,
)
from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_playback import (
    HOLD_END,
    PLAYING,
    fields_from_timed_message,
    tf_frame_prefix,
    vehicle_namespace,
    verify_uniform_playback_plan,
)
from fwis_v3_optimizer.uniform_trajectory import (
    CERTIFICATION,
    MODEL_VERSION,
    UniformTimedPlan,
)

import formation_uniform_planner_node as planner_node

PROTECTED_PORTS = (11311, 11312, 11320)
POSITION_TOL_M = 1e-3
ANGLE_TOL_RAD = 1e-3
INTERIOR_MIN = 11
JOB_TOPIC = "/formation/job_state"
STATUS_TOPIC = "/formation/optimizer_status"
PLAYBACK_STATUS_TOPIC = "/formation/playback_status"
PLAN_TOPIC = "/formation/playback_plan"
REQUEST_TOPIC = "/formation/plan_request"
MARKER_TOPIC = "/formation/markers"
TF_TOPIC = "/tf"
GOAL_TOPIC = "/move_base_simple/goal"
INITIAL_POSE_TOPIC = "/initialpose"


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


def stamp_seconds(stamp):
    return int(stamp.secs) + int(stamp.nsecs) * 1e-9


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


def note(checks, name, ok, detail=""):
    checks[name] = {"ok": bool(ok), "detail": str(detail)}
    if not ok:
        raise SmokeError("{}: {}".format(name, detail))


class MarkerRecord(object):
    """Compact, lossless copy of the parts of a MarkerArray we check.

    Keeping the full message (20+ markers with trajectory-wide polylines at
    up to 100 ticks/s for every experiment) makes the recorder slower than
    the playback it observes, which biases the measured tick rate.  Only the
    stamp, the marker count and the vehicle arrows are needed.
    """

    __slots__ = ("stamp", "count", "arrows")

    def __init__(self, stamp, count, arrows):
        self.stamp = stamp
        self.count = count
        self.arrows = arrows


class Session(object):
    """Thread-safe recorder for the demo topics."""

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
        self.capture_display = True
        self.started = rospy.Time.now().to_sec()
        self._subscribers = [
            rospy.Subscriber(JOB_TOPIC, String, self.on_job_state,
                             queue_size=20),
            rospy.Subscriber(STATUS_TOPIC, String, self.on_status,
                             queue_size=20),
            rospy.Subscriber(PLAYBACK_STATUS_TOPIC, String,
                             self.on_playback, queue_size=100),
            rospy.Subscriber(PLAN_TOPIC, FormationPlaybackPlan,
                             self.on_envelope, queue_size=10),
            rospy.Subscriber(REQUEST_TOPIC, FormationPlanRequest,
                             self.on_request, queue_size=20),
            rospy.Subscriber(MARKER_TOPIC, MarkerArray, self.on_markers,
                             queue_size=100),
            rospy.Subscriber(TF_TOPIC, TFMessage, self.on_tf,
                             queue_size=200),
        ]

    def _document(self, message, label):
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
        for item in markers:
            if item.type != Marker.ARROW or len(item.points) < 2:
                continue
            first, second = item.points[0], item.points[1]
            arrows.append((item.ns, (first.x, first.y),
                           (second.x, second.y)))
        record = MarkerRecord(markers[0].header.stamp, len(markers), arrows)
        with self.lock:
            self.markers.append((rospy.Time.now().to_sec(), record))

    def on_tf(self, message):
        if not self.capture_display:
            return
        with self.lock:
            self.transforms.append((rospy.Time.now().to_sec(), message))

    # ---- waiting helpers ---------------------------------------------

    def latest_job(self):
        with self.lock:
            return self.job_states[-1][1] if self.job_states else None

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


def publish_initial_pose(x, y, theta):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.pose.pose.position.x = float(x)
    message.pose.pose.position.y = float(y)
    message.pose.pose.orientation = quaternion_value(theta)
    publisher = rospy.Publisher(INITIAL_POSE_TOPIC, PoseWithCovarianceStamped,
                                queue_size=1)
    rospy.sleep(0.4)
    publisher.publish(message)
    rospy.sleep(0.6)
    publisher.unregister()


def publish_goal(x, y, theta):
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.pose.position.x = float(x)
    message.pose.position.y = float(y)
    message.pose.orientation = quaternion_value(theta)
    publisher = rospy.Publisher(GOAL_TOPIC, PoseStamped, queue_size=1)
    rospy.sleep(0.4)
    publisher.publish(message)
    rospy.sleep(0.1)
    publisher.unregister()


# ---- envelope analysis ------------------------------------------------

def phase_index_of(plan, elapsed):
    """Phase index for ``elapsed``: left-closed, right-open, final endpoint."""
    for index, phase in enumerate(plan.phases):
        if float(phase.t0_s) <= elapsed < float(phase.end_time):
            return index
    if plan.phases and elapsed == float(plan.phases[-1].end_time):
        return len(plan.phases) - 1
    return None


def is_interior(plan, elapsed, index):
    if index is None:
        return False
    phase = plan.phases[index]
    return float(phase.t0_s) < elapsed < float(phase.end_time)


def phase_summary(plan):
    return [
        {
            "phase_id": phase.phase_id,
            "kind": getattr(phase, "kind", type(phase).__name__),
            "mode": getattr(phase, "mode", None),
            "t0_s": float(phase.t0_s),
            "end_s": float(phase.end_time),
        }
        for phase in plan.phases
    ]


def edge_summary(plan):
    summary = []
    for edge in plan.edges:
        control = edge.control
        summary.append({
            "primitive": edge.primitive,
            "mode": edge.mode,
            "pivot_x": float(getattr(control, "pivot_x", 0.0)),
            "pivot_y": float(getattr(control, "pivot_y", 0.0)),
            "delta_psi": float(getattr(control, "delta_psi", 0.0)),
        })
    return summary


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


def analyze_envelope(message, checks, envelope_dir):
    document = envelope_document(message)
    stamp = (document["header"]["secs"], document["header"]["nsecs"])
    fields = document["trajectory_fields"]

    note(checks, "envelope_stamp_equals_embedded_trajectory",
         fields["stamp"] == stamp,
         "envelope {} trajectory {}".format(stamp, fields["stamp"]))
    note(checks, "envelope_backend", message.backend == "uniform_v02",
         message.backend)
    note(checks, "envelope_semantic_version",
         message.semantic_version == "uniform_playback_plan/v1",
         message.semantic_version)
    note(checks, "envelope_model_version", message.model_version ==
         MODEL_VERSION, message.model_version)
    note(checks, "envelope_certification", message.certification ==
         CERTIFICATION, message.certification)
    note(checks, "envelope_digest_shape",
         len(message.model_sha256) == 64 and all(
             character in "0123456789abcdef"
             for character in message.model_sha256),
         message.model_sha256)

    trajectory, plan, digest = verify_uniform_playback_plan(
        plan_id=int(message.plan_id),
        formation_id=message.formation_id,
        backend=message.backend,
        semantic_version=message.semantic_version,
        model_version=message.model_version,
        model_sha256=message.model_sha256,
        model_json=message.model_json,
        certification=message.certification,
        trajectory_fields=fields)
    note(checks, "model_digest_verified", digest == message.model_sha256,
         digest)
    note(checks, "identity_plan_id", int(plan.plan_id) == int(message.plan_id),
         "{} vs {}".format(plan.plan_id, message.plan_id))
    note(checks, "identity_formation_id",
         plan.formation_id == message.formation_id, plan.formation_id)
    note(checks, "trajectory_plan_id",
         trajectory.plan_id == message.plan_id, trajectory.plan_id)

    sample_tables = {int(item["vehicle_id"]): item["points"]
                     for item in fields["trajectories"]}
    times = list(fields["times_from_start"])
    note(checks, "shared_time_axis_lengths",
         all(len(points) == len(times) for points in sample_tables.values()),
         {vehicle_id: len(points)
          for vehicle_id, points in sample_tables.items()})

    worst_sample_m = worst_sample_rad = 0.0
    worst_qf_m = 0.0
    phase_counts = {phase.phase_id: 0 for phase in plan.phases}
    for position, elapsed in enumerate(times):
        state = plan.state_at(min(elapsed, plan.duration_s))
        index = phase_index_of(plan, elapsed)
        if index is None:
            raise SmokeError(
                "published sample time {} is outside the model phases"
                .format(elapsed))
        if is_interior(plan, elapsed, index):
            phase_counts[plan.phases[index].phase_id] += 1
        poses = fields["formation_poses"][position]
        centers = plan.model.centers(FormationPose(*poses))
        for vehicle_id, points in sample_tables.items():
            _, x, y, theta = points[position][:4]
            expected = state.vehicle(vehicle_id)
            worst_sample_m = max(worst_sample_m,
                                 math.hypot(x - expected.x,
                                            y - expected.y))
            worst_sample_rad = max(worst_sample_rad,
                                   abs(wrap(theta - expected.theta)))
            qf_center = centers[vehicle_id]
            worst_qf_m = max(worst_qf_m,
                             math.hypot(x - qf_center[0],
                                        y - qf_center[1]))
    note(checks, "sample_vs_model_position",
         worst_sample_m <= POSITION_TOL_M, worst_sample_m)
    note(checks, "sample_vs_model_angle",
         worst_sample_rad <= ANGLE_TOL_RAD, worst_sample_rad)
    note(checks, "sample_centers_derived_from_qf",
         worst_qf_m <= POSITION_TOL_M, worst_qf_m)
    short = {key: value for key, value in phase_counts.items()
             if value < INTERIOR_MIN}
    note(checks, "interior_samples_per_phase",
         not short,
         {"counts": phase_counts, "short": short,
          "required": INTERIOR_MIN})

    model = UniformTimedPlan.from_model_json(message.model_json)
    note(checks, "model_rebuild_identity",
         model.plan_id == plan.plan_id and
         model.formation_id == plan.formation_id,
         "{} {}".format(model.plan_id, model.formation_id))

    path = envelope_dir / "envelope_{:08d}.json".format(int(message.plan_id))
    write_json(path, document)
    summary = {
        "plan_id": int(message.plan_id),
        "formation_id": message.formation_id,
        "model_sha256": message.model_sha256,
        "model_version": message.model_version,
        "semantic_version": message.semantic_version,
        "certification": message.certification,
        "stamp": {"secs": stamp[0], "nsecs": stamp[1]},
        "T_end_s": float(plan.duration_s),
        "sample_count": len(times),
        "phases": phase_summary(plan),
        "edges": edge_summary(plan),
        "start_qf": [plan.start_qf.x, plan.start_qf.y, plan.start_qf.psi],
        "goal_qf": [plan.goal_qf.x, plan.goal_qf.y, plan.goal_qf.psi],
        "start_headings": dict(plan.start_headings),
        "goal_headings": dict(plan.goal_headings),
        "interior_samples_per_phase": phase_counts,
        "worst_sample_position_m": worst_sample_m,
        "worst_sample_angle_rad": worst_sample_rad,
        "worst_qf_center_m": worst_qf_m,
        "document_path": str(path),
    }
    return plan, fields, summary, path


def head_delta(plan, vehicle_id, phase):
    """Body heading change over one motion phase."""
    start = plan.state_at(float(phase.t0_s), side="left").vehicle(vehicle_id)
    end = plan.state_at(float(phase.end_time)).vehicle(vehicle_id)
    return wrap(end.theta - start.theta)


def s_phase_zero_angle(plan):
    """Zero-angle vehicles inside the unified S motion phase, if any."""
    result = {}
    for phase in plan.phases:
        if getattr(phase, "mode", None) != "S" or not hasattr(
                phase, "t0_s"):
            continue
        if getattr(phase, "kind", None) != "motion":
            continue
        deltas = {vehicle_id: head_delta(plan, vehicle_id, phase)
                  for vehicle_id, _ in plan.start_headings}
        result[phase.phase_id] = deltas
    return result


# ---- display (TF / Marker / status) checks ---------------------------

class DisplayAccumulator(object):
    def __init__(self, plan, fields, trajectory, trajectory_ticks=None):
        self.plan = plan
        self.fields = fields
        self.trajectory = trajectory
        self.ticks = sorted(trajectory_ticks or ())
        self.stamp = fields["stamp"]
        self.prefix = tf_frame_prefix(plan.formation_id)
        self.vehicle_ids = [vehicle_id for vehicle_id, _ in
                            plan.start_headings]
        self.phase_counts = {
            source: {phase.phase_id: 0 for phase in plan.phases}
            for source in ("tf", "marker", "status")}
        self.worst = {
            "tf_position_m": 0.0, "tf_angle_rad": 0.0,
            "marker_position_m": 0.0, "marker_angle_rad": 0.0,
            "status_position_m": 0.0, "status_angle_rad": 0.0,
        }
        self.samples = {"tf": 0, "marker": 0, "status": 0}
        self._tf_stamps = set()
        self._marker_stamps = set()
        self._status_stamps = set()

    def _elapsed(self, stamp):
        return stamp_seconds(stamp) - stamp_seconds(self.fields_stamp())

    def fields_stamp(self):
        value = self.stamp
        return type("Stamp", (), {"secs": value[0], "nsecs": value[1]})()

    def _accept(self, source, stamp):
        elapsed = stamp_seconds(stamp) - (
            self.stamp[0] + self.stamp[1] * 1e-9)
        if elapsed < -1e-6 or elapsed > self.plan.duration_s + 0.02:
            return None
        if source in ("tf", "marker") and self.ticks:
            # Status and marker/TF messages belonging to one display tick are
            # stamped microseconds-to-a-millisecond apart (separate
            # rospy.Time.now() calls in the same loop iteration), so an exact
            # 10 us window dropped real trajectory frames and made the
            # per-phase tick count depend on scheduling jitter.  Half of the
            # 100 Hz display period keeps one tick from matching its neighbour
            # while still rejecting the pre-acceptance static hold.
            if not any(abs(elapsed - tick) <= 5e-3 for tick in self.ticks):
                return None
        key = round(stamp_seconds(stamp), 9)
        seen = {"tf": self._tf_stamps, "marker": self._marker_stamps,
                "status": self._status_stamps}[source]
        if key in seen:
            return None
        seen.add(key)
        self.samples[source] += 1
        clamped = min(max(elapsed, 0.0), self.plan.duration_s)
        index = phase_index_of(self.plan, elapsed)
        if elapsed >= 0.0 and is_interior(self.plan, elapsed, index):
            self.phase_counts[source][
                self.plan.phases[index].phase_id] += 1
        return elapsed, clamped

    def _state(self, elapsed):
        return self.plan.state_at(elapsed)

    def add_tf(self, message):
        for transform in message.transforms:
            child = transform.child_frame_id
            if not child.startswith(self.prefix):
                continue
            accepted = self._accept("tf", transform.header.stamp)
            if accepted is None:
                continue
            elapsed, clamped = accepted
            state = self._state(clamped)
            body = transform.transform
            if child == self.prefix + "_reference":
                position_error = math.hypot(
                    body.translation.x - state.qf.x,
                    body.translation.y - state.qf.y)
                angle_error = abs(wrap(
                    quaternion_yaw(body.rotation) - state.qf.psi))
            else:
                vehicle_id = int(child.split("_vehicle_")[1].split("_")[0])
                expected = state.vehicle(vehicle_id)
                position_error = math.hypot(
                    body.translation.x - expected.x,
                    body.translation.y - expected.y)
                angle_error = abs(wrap(
                    quaternion_yaw(body.rotation) - expected.theta))
            self.worst["tf_position_m"] = max(
                self.worst["tf_position_m"], position_error)
            self.worst["tf_angle_rad"] = max(
                self.worst["tf_angle_rad"], angle_error)

    def add_markers(self, record):
        if record.count == 0:
            return
        accepted = self._accept("marker", record.stamp)
        if accepted is None:
            return
        elapsed, clamped = accepted
        state = self._state(clamped)
        for vehicle_id in self.vehicle_ids:
            namespace = vehicle_namespace(self.plan.formation_id, vehicle_id)
            arrow = next(
                (item for item in record.arrows if item[0] == namespace), None)
            if arrow is None:
                raise SmokeError(
                    "marker arrow for vehicle {} is missing at elapsed {:.4f}"
                    .format(vehicle_id, clamped))
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

    def add_status(self, document):
        if document.get("state") not in (PLAYING, HOLD_END):
            return
        elapsed = document.get("elapsed_s")
        if not isinstance(elapsed, (int, float)):
            return
        if elapsed < -1e-6 or elapsed > self.plan.duration_s + 0.02:
            return
        key = round(float(elapsed), 9)
        if key in self._status_stamps:
            return
        self._status_stamps.add(key)
        self.samples["status"] += 1
        clamped = min(max(float(elapsed), 0.0), self.plan.duration_s)
        index = phase_index_of(self.plan, float(elapsed))
        if is_interior(self.plan, float(elapsed), index):
            self.phase_counts["status"][
                self.plan.phases[index].phase_id] += 1
        state = self._state(clamped)
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

    def summary(self):
        return {
            "samples": self.samples,
            "interior_per_phase": self.phase_counts,
            "worst": self.worst,
        }


def display_checks(checks, accumulator, prefix=""):
    short = {
        source: {key: value for key, value in counts.items()
                 if value < INTERIOR_MIN}
        for source, counts in accumulator.phase_counts.items()}
    short = {source: values for source, values in short.items() if values}
    note(checks, prefix + "interior_ticks_per_phase", not short,
         {"counts": accumulator.phase_counts, "short": short,
          "required_per_source": INTERIOR_MIN,
          "samples": accumulator.samples})
    note(checks, prefix + "tf_position", accumulator.worst["tf_position_m"]
         <= POSITION_TOL_M, accumulator.worst["tf_position_m"])
    note(checks, prefix + "tf_angle", accumulator.worst["tf_angle_rad"]
         <= ANGLE_TOL_RAD, accumulator.worst["tf_angle_rad"])
    note(checks, prefix + "marker_position",
         accumulator.worst["marker_position_m"] <= POSITION_TOL_M,
         accumulator.worst["marker_position_m"])
    note(checks, prefix + "marker_angle",
         accumulator.worst["marker_angle_rad"] <= ANGLE_TOL_RAD,
         accumulator.worst["marker_angle_rad"])
    note(checks, prefix + "status_position",
         accumulator.worst["status_position_m"] <= POSITION_TOL_M,
         accumulator.worst["status_position_m"])
    note(checks, prefix + "status_angle",
         accumulator.worst["status_angle_rad"] <= ANGLE_TOL_RAD,
         accumulator.worst["status_angle_rad"])


# ---- scenarios --------------------------------------------------------

def check_shared_identity(checks, session, plan_id):
    with session.lock:
        statuses = [d for _, d in session.statuses
                    if d.get("plan_id") == plan_id]
        jobs = [d for _, d in session.job_states
                if d.get("plan_id") == plan_id]
        envelope = session.envelopes.get(plan_id)
    note(checks, "optimizer_status_exists", bool(statuses),
         "{} status documents".format(len(statuses)))
    note(checks, "job_state_exists", bool(jobs),
         "{} job documents".format(len(jobs)))
    digest = envelope.model_sha256
    published = [document for document in statuses
                 if document.get("node_state") == "PUBLISHED"]
    note(checks, "status_model_digest_matches_envelope",
         bool(published) and all(
             document.get("model_sha256") == digest
             for document in published),
         "{}".format([document.get("model_sha256")
                      for document in published]))
    note(checks, "status_plan_id_matches_envelope",
         all(document.get("plan_id") == plan_id for document in statuses),
         "{}".format(sorted({document.get("plan_id")
                             for document in statuses})))
    terminal_jobs = [document for document in jobs
                     if document.get("state") in ("PLAYING", "HOLD_END")]
    note(checks, "job_model_digest_matches_envelope",
         bool(terminal_jobs) and all(
             document.get("model_sha256") == digest
             for document in terminal_jobs),
         "{}".format([document.get("model_sha256")
                      for document in jobs[-3:]]))
    note(checks, "job_plan_id_matches_envelope",
         all(document.get("plan_id") == plan_id for document in jobs),
         "{}".format(sorted({document.get("plan_id") for document in jobs})))


def run_experiment(session, checks, name, goal, initial=None,
                   busy_probe=False, planning_timeout=150.0,
                   envelope_dir=None, evidence=None):
    result = {"name": name, "goal": list(goal)}
    with session.lock:
        mark_tf = len(session.transforms)
        mark_marker = len(session.markers)
    if initial is not None:
        publish_initial_pose(*initial)
        result["initial_pose"] = list(initial)
    request_count = len(session.requests)
    result["request_count_before"] = request_count
    goal_time = rospy.Time.now().to_sec()
    publish_goal(*goal)
    request = session.wait_request_after(request_count, 5.0)
    plan_id = int(request.plan_id)
    result["plan_id"] = plan_id
    result["request_start"] = [request.formation_start.x,
                               request.formation_start.y,
                               request.formation_start.theta]
    session.wait_job(lambda d: d.get("plan_id") == plan_id and
                     d.get("state") == "PLANNING", 5.0,
                     "job PLANNING for plan {}".format(plan_id))
    envelope = session.wait_envelope(plan_id, planning_timeout)
    envelope_time = rospy.Time.now().to_sec()
    result["planning_wall_s"] = envelope_time - goal_time
    stamp = int(envelope.header.stamp.secs) + (
        int(envelope.header.stamp.nsecs) * 1e-9)
    result["stamp_to_receipt_s"] = envelope_time - stamp
    note(checks, name + "_stamp_not_after_receipt", stamp <= envelope_time +
         0.05, "stamp {:.6f} receipt {:.6f}".format(stamp, envelope_time))

    plan, fields, summary, path = analyze_envelope(
        envelope, checks, envelope_dir)
    result["envelope"] = summary
    result["envelope_path"] = str(path)

    accumulator = DisplayAccumulator(plan, fields, None)

    playing = session.wait_playback(
        lambda d: d.get("plan_id") == plan_id and d.get("state") == PLAYING,
        8.0, "playback PLAYING for plan {}".format(plan_id))
    result["playing_receipt_s"] = rospy.Time.now().to_sec()
    note(checks, name + "_playback_identity",
         playing.get("plan_id") == plan_id, playing.get("plan_id"))
    with session.lock:
        statuses = [(t, d) for t, d in session.statuses
                    if d.get("plan_id") == plan_id]
    published = [d for _, d in statuses
                 if d.get("node_state") == "PUBLISHED"]
    note(checks, name + "_planner_published", bool(published),
         [d.get("status") for _, d in statuses])
    if published:
        result["planner_wall_time_s"] = published[-1].get("wall_time_s")
        note(checks, name + "_planner_wall_matches_T_end",
             abs(float(published[-1].get("T_end")) -
                 float(plan.duration_s)) <= 1e-9,
             "{} vs {}".format(published[-1].get("T_end"),
                               plan.duration_s))

    if busy_probe:
        result["busy"] = busy_probe_experiment(session, checks, plan_id)

    terminal = session.wait_playback(
        lambda d: d.get("plan_id") == plan_id and d.get("state") == HOLD_END,
        plan.duration_s + 12.0, "playback HOLD_END for plan {}".format(
            plan_id))
    terminal_receipt = rospy.Time.now().to_sec()
    result["terminal_receipt_s"] = terminal_receipt
    result["terminal_minus_stamp_s"] = terminal_receipt - stamp
    note(checks, name + "_playback_terminal_state",
         terminal.get("state") == HOLD_END, terminal.get("state"))
    note(checks, name + "_terminal_position_error",
         (terminal.get("terminal_center_error_m") is None or
          float(terminal["terminal_center_error_m"]) <= POSITION_TOL_M),
         terminal.get("terminal_center_error_m"))
    note(checks, name + "_terminal_heading_error",
         (terminal.get("terminal_heading_error_rad") is None or
          float(terminal["terminal_heading_error_rad"]) <= ANGLE_TOL_RAD),
         terminal.get("terminal_heading_error_rad"))
    with session.lock:
        tf_messages = [message for _, message in
                       session.transforms[mark_tf:]]
        marker_messages = [message for _, message in
                           session.markers[mark_marker:]]
        playback_documents = [d for _, d in session.playback
                              if d.get("plan_id") == plan_id]
    accumulator.ticks = sorted({
        float(document["elapsed_s"]) for document in playback_documents
        if document.get("state") in (PLAYING, HOLD_END) and
        isinstance(document.get("elapsed_s"), (int, float))})
    for message in tf_messages:
        accumulator.add_tf(message)
    for message in marker_messages:
        accumulator.add_markers(message)
    for document in playback_documents:
        accumulator.add_status(document)
    display_checks(checks, accumulator, prefix=name + "_")
    result["display"] = accumulator.summary()

    job_end = session.wait_job(
        lambda d: d.get("plan_id") == plan_id and
        d.get("state") == "HOLD_END", 3.0,
        "job HOLD_END for plan {}".format(plan_id))
    job_end_time = rospy.Time.now().to_sec()
    result["job_hold_end_minus_playback_hold_end_s"] = (
        job_end_time - terminal_receipt)
    note(checks, name + "_busy_released_by_real_terminal",
         job_end_time >= terminal_receipt - 0.05,
         "job HOLD_END after playback HOLD_END by {:.3f} s".format(
             job_end_time - terminal_receipt))
    check_shared_identity(checks, session, plan_id)
    history = session.job_history(plan_id)
    result["job_states"] = [{"state": document.get("state"),
                             "received_s": received}
                            for received, document in history]
    states = [document.get("state") for _, document in history]
    note(checks, name + "_job_state_order",
         bool(states) and states[0] in ("PLANNING", "PLAYING") and
         "PLANNING" in states and "PLAYING" in states and
         "HOLD_END" in states and states[-1] == "HOLD_END",
         states)
    if evidence is not None:
        evidence.setdefault("experiments", []).append(result)
    return result


def busy_probe_experiment(session, checks, plan_id):
    """Reject Goal/InitialPose/plan_request while the job is busy."""
    result = {"active_plan_id": plan_id}
    session.wait_job(lambda d: d.get("plan_id") == plan_id and
                     d.get("state") in ("PLANNING", "PLAYING"), 2.0,
                     "busy job state before injection")
    with session.lock:
        request_count = len(session.requests)
        job_before = session.job_states[-1][1] if session.job_states else None
    note(checks, "busy_job_state_is_busy",
         job_before is not None and
         job_before.get("state") in ("PLANNING", "PLAYING"),
         job_before.get("state") if job_before else None)
    publish_goal(10.5, 10.5, 0.0)
    publish_initial_pose(9.0, 9.0, 0.0)
    rospy.sleep(1.2)
    with session.lock:
        new_requests = session.requests[request_count:]
        job_after = session.job_states[-1][1] if session.job_states else None
    note(checks, "busy_bridge_refuses_goal_and_initial_pose",
         len(new_requests) == 0,
         "{} new plan requests".format(len(new_requests)))
    note(checks, "busy_job_state_unchanged",
         job_after is not None and
         job_after.get("state") in ("PLANNING", "PLAYING") and
         job_after.get("plan_id") == plan_id,
         job_after)

    # Direct injection bypassing the bridge: the planner must also answer
    # BUSY and reserve nothing.
    with session.lock:
        request_message = session.requests[-1][1]
    fake_id = plan_id + 500000
    fake = copy.deepcopy(request_message)
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
    note(checks, "planner_busy_rejection", busy_status is not None,
         busy_status)
    with session.lock:
        envelope_seen = fake_id in session.envelopes
        job_final = session.job_states[-1][1] if session.job_states else None
    note(checks, "busy_planner_no_envelope", not envelope_seen, fake_id)
    note(checks, "busy_planner_keeps_job",
         job_final is not None and
         job_final.get("state") in ("PLANNING", "PLAYING") and
         job_final.get("plan_id") == plan_id, job_final)
    result.update({
        "fake_plan_id": fake_id,
        "new_requests": len(new_requests),
        "busy_status": busy_status,
        "job_before": job_before,
        "job_after": job_after,
    })
    return result


def scenario_main(session, evidence, envelope_dir, checks):
    # Fresh admission state: the bus must be free, not necessarily IDLE.
    session.wait_job(lambda d: d.get("state") in ("IDLE", "HOLD_END",
                                                  "FAILED"), 10.0,
                     "job admitting a new experiment")

    # 1. Forward experiment; busy probes run inside its playback.
    forward = run_experiment(
        session, checks, "forward",
        goal=(12.0, 10.0, 0.0), initial=(10.0, 10.0, 0.0),
        busy_probe=True, envelope_dir=envelope_dir, evidence=evidence)
    modes = [phase["mode"] for phase in forward["envelope"]["phases"]]
    note(checks, "forward_uses_A", "A" in modes, modes)

    # The second experiment must start from the real playback terminal, not
    # from the refused busy InitialPose.
    lateral = run_experiment(
        session, checks, "lateral",
        goal=(10.0, 12.0, 0.0), initial=None,
        envelope_dir=envelope_dir, evidence=evidence)
    start = lateral["envelope"]["start_qf"]
    note(checks, "second_request_started_from_playback_terminal",
         abs(start[0] - 12.0) <= 0.05 and abs(start[1] - 10.0) <= 0.05,
         start)
    lateral_modes = [phase["mode"] for phase in lateral["envelope"]["phases"]]
    note(checks, "lateral_uses_O", "O" in lateral_modes, lateral_modes)

    # 3. Arbitrary decimal headings as produced by an RViz drag (not the
    #    round 0/0.6 values used by the other experiments).  The search may
    #    legitimately stop within the frozen goal tolerances, so the published
    #    per-vehicle goal centres must be derived from the achieved model goal
    #    rather than the requested pose; deriving them from the request made
    #    the planner's own envelope audit reject every real RViz plan.
    rviz_style = run_experiment(
        session, checks, "rviz_style",
        goal=(12.0, 12.0, 0.018), initial=(10.0, 12.0, 0.013),
        envelope_dir=envelope_dir, evidence=evidence)
    achieved_goal = rviz_style["envelope"]["goal_qf"]
    achieved_delta = math.hypot(achieved_goal[0] - 12.0,
                                achieved_goal[1] - 12.0)
    rviz_style["requested_goal"] = [12.0, 12.0, 0.018]
    rviz_style["achieved_goal_delta_m"] = achieved_delta
    note(checks, "rviz_style_uses_nonzero_decimal_heading",
         abs(rviz_style["goal"][2] - 0.018) <= 1e-12,
         rviz_style["goal"])
    # This is evidence, not a pass condition: whether the search lands exactly
    # on the requested point depends on the primitives, but the envelope must
    # stay valid across the whole allowed tolerance band.
    note(checks, "rviz_style_achieved_goal_inside_frozen_tolerance",
         achieved_delta <= 0.16 + 1e-9,
         {"achieved": achieved_goal, "delta_m": achieved_delta})

    # 4. Unified S->A experiment with a non-zero cargo pivot.
    unified = run_experiment(
        session, checks, "s_to_a",
        goal=(12.0, 11.0, 0.6), initial=(10.0, 10.0, 0.0),
        envelope_dir=envelope_dir, evidence=evidence)
    unified_modes = [phase["mode"] for phase in unified["envelope"]["phases"]]
    note(checks, "unified_has_S_and_A",
         "S" in unified_modes and "A" in unified_modes, unified_modes)
    pivots = [(edge["pivot_x"], edge["pivot_y"])
              for edge in unified["envelope"]["edges"]
              if edge["primitive"] == "A_ARC"]
    note(checks, "non_zero_pivot_arc",
         any(abs(px) > 1e-9 or abs(py) > 1e-9 for px, py in pivots),
         pivots)
    plan = analyze_envelope(session.envelopes[
        unified["plan_id"]], checks, envelope_dir)[0]
    deltas = s_phase_zero_angle(plan)
    zero_angle = {phase_id: values for phase_id, values in deltas.items()
                  if any(abs(value) <= 1e-9 for value in values.values())}
    note(checks, "unified_S_phase_has_zero_angle_vehicle",
         bool(zero_angle),
         {"S_phase_heading_deltas": deltas, "zero_angle": zero_angle})
    unified["s_phase_heading_deltas"] = deltas

    # The final job state must be the playback terminal, accepting a next
    # experiment; no explicit preemption is implemented.
    final_job = session.latest_job()
    note(checks, "final_job_hold_end",
         final_job.get("state") == "HOLD_END", final_job)
    return evidence


def scenario_failure(session, evidence, envelope_dir, checks):
    session.wait_job(lambda d: d.get("state") in ("IDLE", "HOLD_END",
                                                  "FAILED"), 10.0,
                     "job admitting a new experiment before the failure run")
    success = run_experiment(
        session, checks, "failure_baseline",
        goal=(11.0, 10.0, 0.0), initial=(10.0, 10.0, 0.0),
        envelope_dir=envelope_dir, evidence=evidence)
    baseline_plan = success["plan_id"]
    with session.lock:
        marker_before = [m for _, m in session.markers[-3:]]
        tf_before = [m for _, m in session.transforms[-3:]]

    request_count = len(session.requests)
    with session.lock:
        failure_mark = len(session.playback)
    failure_goal_time = rospy.Time.now().to_sec()
    publish_goal(13.5, 10.0, 0.0)
    request = session.wait_request_after(request_count, 5.0)
    failed_id = int(request.plan_id)
    failed_job = session.wait_job(
        lambda d: d.get("plan_id") == failed_id and
        d.get("state") == "FAILED", 150.0,
        "job FAILED for plan {}".format(failed_id))
    failure_time = rospy.Time.now().to_sec()
    evidence["failure"] = {
        "plan_id": failed_id,
        "baseline_plan_id": baseline_plan,
        "wall_s": failure_time - failure_goal_time,
        "job": failed_job,
    }
    note(checks, "failure_job_failed", failed_job.get("state") == "FAILED",
         failed_job.get("reason"))

    with session.lock:
        statuses = [d for _, d in session.statuses
                    if d.get("plan_id") == failed_id]
        latched = session.envelopes.get(failed_id)
    reason = None
    pipeline = None
    for document in statuses:
        if document.get("node_state") in ("REJECTED", "INVALID"):
            reason = document.get("reason")
            pipeline = document.get("pipeline_status")
    evidence["failure"]["optimizer_status"] = statuses[-1] if statuses else None
    note(checks, "failure_no_executable_status",
         bool(statuses) and statuses[-1].get("executable") is False,
         statuses[-1].get("executable") if statuses else None)
    note(checks, "failure_reason_is_admitted_planning_failure",
         bool(reason) and
         pipeline in ("BUDGET_EXHAUSTED", "NO_SOLUTION", "AUDIT_REJECTED") and
         statuses[-1].get("admitted") is True,
         {"reason": reason, "pipeline_status": pipeline,
          "admitted": statuses[-1].get("admitted") if statuses else None})

    note(checks, "failure_latched_envelope_is_invalid",
         latched is not None and
         latched.plan_id == failed_id and
         latched.model_sha256 == "" and
         latched.model_json == "" and
         latched.certification == "" and
         len(latched.trajectory.times_from_start) == 0,
         None if latched is None else {
             "plan_id": latched.plan_id,
             "model_sha256": latched.model_sha256,
             "model_json_length": len(latched.model_json),
             "certification": latched.certification,
             "samples": len(latched.trajectory.times_from_start)})

    cleared = session.wait_playback(
        lambda d: d.get("state") in ("CLEARED", "INVALID") and
        d.get("executable") is False, 5.0,
        "playback clear after the failed plan", mark=failure_mark)
    evidence["failure"]["playback_after_failure"] = cleared
    note(checks, "failure_playback_cleared",
         cleared.get("state") in ("CLEARED", "INVALID") and
         cleared.get("cleared") is True, cleared.get("state"))
    note(checks, "failure_playback_not_executable",
         cleared.get("executable") is False, cleared.get("executable"))
    note(checks, "failure_clears_old_markers",
         cleared.get("marker_count") == 0, cleared.get("marker_count"))

    rospy.sleep(0.8)
    with session.lock:
        marker_after = [m for _, m in session.markers[-6:]]
        # tf2_ros emits one TransformStamped per message here, so four
        # messages make one complete hold tick; take several ticks.
        tf_after = [m for _, m in session.transforms[-24:]]
    # The playback keeps refreshing a static hold after a failed plan, which
    # is the intended display (RViz markers are republished every tick).  The
    # failure criterion is therefore that nothing *moves* any more, not that
    # no message arrives.
    prefix = tf_frame_prefix(success["envelope"]["formation_id"])
    tf_positions = {}
    for message in tf_after:
        for transform in message.transforms:
            child = transform.child_frame_id
            if not child.startswith(prefix):
                continue
            tf_positions.setdefault(child, []).append(
                (transform.transform.translation.x,
                 transform.transform.translation.y))

    def spread(values):
        return max(max(x for x, _ in values) - min(x for x, _ in values),
                   max(y for _, y in values) - min(y for _, y in values))

    tf_motion = max((spread(values) for values in tf_positions.values()),
                    default=0.0)
    marker_positions = {}
    for record in marker_after:
        for namespace, first, _ in record.arrows:
            marker_positions.setdefault(namespace, []).append(first)
    marker_motion = max(
        (spread(values) for values in marker_positions.values()),
        default=0.0)
    evidence["failure"]["post_failure_tf_messages"] = len(tf_after)
    evidence["failure"]["post_failure_tf_motion_m"] = tf_motion
    evidence["failure"]["post_failure_marker_motion_m"] = marker_motion
    note(checks, "failure_stops_old_animation",
         bool(tf_positions) and
         all(len(values) >= 2 for values in tf_positions.values()) and
         tf_motion <= POSITION_TOL_M and marker_motion <= POSITION_TOL_M,
         {"tf_messages": len(tf_after), "children": sorted(tf_positions),
          "tf_motion_m": tf_motion, "marker_motion_m": marker_motion})

    # The frozen hold must sit at the terminal frame the baseline playback
    # actually stopped at, not at the start pose or anywhere along the old
    # trajectory.
    reference = tf_positions.get(prefix + "_reference", [])
    goal_qf = success["envelope"]["goal_qf"]
    hold_error = (math.hypot(reference[-1][0] - goal_qf[0],
                             reference[-1][1] - goal_qf[1])
                  if reference else float("inf"))
    evidence["failure"]["hold_reference_error_m"] = hold_error
    note(checks, "failure_hold_is_at_stopped_frame",
         hold_error <= POSITION_TOL_M,
         {"hold_reference_error_m": hold_error,
          "goal_qf": goal_qf,
          "reference": reference[-1] if reference else None})

    # A failed job accepts the next experiment.  The start is the frozen
    # hold at the baseline terminal (11,10); the goal differs from it, so
    # the retry is a real motion plan rather than a NO_MOTION no-op.
    with session.lock:
        request_count = len(session.requests)
    publish_goal(10.0, 10.0, 0.0)
    retry_request = session.wait_request_after(request_count, 5.0)
    retry_id = int(retry_request.plan_id)
    note(checks, "failure_admits_new_experiment",
         retry_id > failed_id and
         abs(retry_request.formation_start.x - goal_qf[0]) <= 1e-6 and
         abs(retry_request.formation_start.y - goal_qf[1]) <= 1e-6,
         {"retry_plan_id": retry_id,
          # After a failed plan the playback keeps a static hold at the
          # frame where the last trajectory stopped.  The goal bridge uses
          # that fresh, rigid display snapshot as the next start (same rule
          # as after HOLD_END); it is not the pre-failure initial pose.
          "start": [retry_request.formation_start.x,
                    retry_request.formation_start.y],
          "hold": list(goal_qf[:2])})
    retry_envelope = session.wait_envelope(retry_id, 150.0)
    retry_plan, _, retry_summary, _ = analyze_envelope(
        retry_envelope, checks, envelope_dir)
    evidence["failure"]["retry"] = retry_summary
    session.wait_playback(
        lambda d: d.get("plan_id") == retry_id and d.get("state") == HOLD_END,
        retry_plan.duration_s + 12.0, "retry HOLD_END")
    session.wait_job(lambda d: d.get("plan_id") == retry_id and
                     d.get("state") == "HOLD_END", 3.0, "retry job HOLD_END")
    note(checks, "failure_retry_completed", True, retry_id)
    return evidence


# ---- tamper / late legacy --------------------------------------------

def build_request(document):
    fields = document["trajectory_fields"]
    request = FormationPlanRequest()
    request.header.frame_id = "map"
    request.header.stamp = rospy.Time.now()
    request.plan_id = int(document["plan_id"])
    request.formation_id = document["formation_id"]
    poses = fields["formation_poses"]
    start = poses[0] if poses else (0.0, 0.0, 0.0)
    request.formation_start.x = float(start[0])
    request.formation_start.y = float(start[1])
    request.formation_start.theta = float(start[2])
    goal = poses[-1] if poses else start
    request.formation_goal.x = float(goal[0])
    request.formation_goal.y = float(goal[1])
    request.formation_goal.theta = float(goal[2])
    request.allow_reverse = True
    for vehicle in fields["vehicles"]:
        spec = FormationVehicleSpec()
        spec.vehicle_id = int(vehicle["vehicle_id"])
        spec.offset_in_formation.x = float(vehicle["offset"][0])
        spec.offset_in_formation.y = float(vehicle["offset"][1])
        spec.offset_in_formation.z = 0.0
        for name in ("start_center", "goal_center"):
            values = vehicle[name]
            center = getattr(spec, name)
            center.x, center.y, center.theta = (
                float(values[0]), float(values[1]), float(values[2]))
        spec.goal_heading_free = bool(vehicle["goal_heading_free"])
        spec.body_length_m = float(vehicle["body_length_m"])
        spec.body_width_m = float(vehicle["body_width_m"])
        request.vehicles.append(spec)
    return request


def exact_duration(seconds):
    """Encode a saved time without a second floating-point truncation.

    The live planner already encodes with ``Duration.from_sec`` once; the
    captured JSON holds those exact nanoseconds as decimal seconds.  Calling
    ``from_sec`` again on the round-tripped float can land one nanosecond low
    (``6.245537336 * 1e9`` evaluates to ``...335.999`` and is truncated),
    which would make an otherwise genuine envelope fail the model-duration
    tolerance.  Round to the nearest nanosecond instead.
    """
    nanoseconds = int(round(float(seconds) * 1e9))
    secs, nsecs = divmod(nanoseconds, 10 ** 9)
    return rospy.Duration(secs, nsecs)


def build_envelope(document, stamp, fields=None, plan_id=None,
                   semantic_version=None, model_version=None,
                   model_sha256=None, model_json=None,
                   certification=None, envelope_stamp=None,
                   embedded_stamp=None):
    fields = copy.deepcopy(document["trajectory_fields"]
                           if fields is None else fields)
    # The embedded trajectory stamp normally equals the envelope stamp; the
    # explicit override exists only so a negative case can make them differ.
    fields["stamp"] = list(stamp if embedded_stamp is None else embedded_stamp)
    message = FormationPlaybackPlan()
    message.header.frame_id = "map"
    header = envelope_stamp if envelope_stamp is not None else stamp
    message.header.stamp = rospy.Time(header[0], header[1])
    message.plan_id = int(document["plan_id"] if plan_id is None else plan_id)
    message.formation_id = document["formation_id"]
    message.backend = "uniform_v02"
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


def publish_envelope(message):
    publisher = rospy.Publisher(PLAN_TOPIC, FormationPlaybackPlan,
                                queue_size=1)
    rospy.sleep(0.2)
    publisher.publish(message)
    rospy.sleep(0.05)
    publisher.unregister()


def wait_for_trajectory(session, plan_id, mark, timeout=3.0):
    return session.wait_playback(
        lambda d: d.get("plan_id") == plan_id and
        d.get("state") in (PLAYING, "HOLD_START"), timeout,
        "playback accepting plan {}".format(plan_id), mark=mark)


def tamper_cases(document, stamp):
    """Return [(name, expected_reason_fragment, message_factory)]."""
    def fields():
        return copy.deepcopy(document["trajectory_fields"])

    def tampered_model():
        model = json.loads(document["model_json"])
        model["requested_goal"]["qf"][0] += 0.25
        return json.dumps(model, sort_keys=True)

    def tampered_sample():
        value = fields()
        points = [list(point) for point in value["trajectories"][0]["points"]]
        index = len(points) // 2
        points[index][1] += 1e-3
        value["trajectories"][0]["points"] = [tuple(point)
                                             for point in points]
        return value

    digest = document["model_sha256"]
    flipped = ("0" if digest[0] != "0" else "1") + digest[1:]
    return [
        ("unknown_semantic_version", "semantic",
         lambda: build_envelope(document, stamp,
                                semantic_version="uniform_playback_plan/v0")),
        ("unknown_model_version", "model_version",
         lambda: build_envelope(document, stamp,
                                model_version="uniform_timed_plan.v0")),
        ("digest_mismatch", "sha256",
         lambda: build_envelope(document, stamp, model_sha256=flipped)),
        ("tampered_model", "model",
         lambda: build_envelope(document, stamp,
                                model_json=tampered_model())),
        ("tampered_sample", "sample",
         lambda: build_envelope(document, stamp, fields=tampered_sample())),
        ("stamp_mismatch", "stamp",
         lambda: build_envelope(
             document, stamp,
             embedded_stamp=[int(stamp[0]) + 1, int(stamp[1])])),
    ]


def scenario_tamper(session, evidence, checks, baseline_document):
    import time as wall_time
    stamp_now = rospy.Time.now()
    request = build_request(baseline_document)
    request.header.stamp = stamp_now
    publisher = rospy.Publisher(REQUEST_TOPIC, FormationPlanRequest,
                                queue_size=1)
    rospy.sleep(0.3)
    publisher.publish(request)
    rospy.sleep(0.3)
    publisher.unregister()

    def fresh_stamp():
        return (int(rospy.Time.now().secs), int(rospy.Time.now().nsecs))

    def publish_valid():
        stamp = fresh_stamp()
        with session.lock:
            mark = len(session.playback)
        publish_envelope(build_envelope(baseline_document, stamp))
        document = wait_for_trajectory(
            session, request.plan_id, mark, 3.0)
        return stamp, document

    stamp, accepted = publish_valid()
    evidence["tamper_baseline_accepted"] = accepted
    note(checks, "tamper_baseline_accepted",
         accepted.get("state") in (PLAYING, "HOLD_START") and
         accepted.get("model_sha256") == baseline_document["model_sha256"],
         accepted.get("state"))
    note(checks, "tamper_baseline_markers_present",
         accepted.get("marker_count", 0) > 0, accepted.get("marker_count"))

    results = []
    for name, fragment, factory in tamper_cases(baseline_document,
                                                fresh_stamp()):
        # Restore a genuine executable display before each mutation so the
        # rejection is observed against live animation.
        stamp, _ = publish_valid()
        with session.lock:
            mark = len(session.playback)
        message = factory()
        publish_envelope(message)
        deadline = rospy.Time.now().to_sec() + 3.0
        rejected = None
        while rospy.Time.now().to_sec() < deadline and rejected is None:
            with session.lock:
                for _, document in reversed(session.playback[mark:]):
                    if document.get("state") == "CLEARED" and (
                            "invalid playback plan" in str(
                                document.get("reason", ""))):
                        rejected = document
                        break
            rospy.sleep(0.02)
        ok = (rejected is not None and
              rejected.get("executable") is False and
              rejected.get("cleared") is True and
              rejected.get("marker_count") == 0 and
              fragment in str(rejected.get("reason", "")).lower())
        entry = {"name": name, "ok": ok, "rejected": rejected,
                 "expected_reason_fragment": fragment}
        results.append(entry)
        checks["tamper_" + name] = {"ok": bool(ok), "detail": rejected}
        if not ok:
            raise SmokeError(
                "tamper case {} was not rejected as expected: {}".format(
                    name, rejected))
    evidence["tamper_cases"] = results

    # Late legacy message: the uniform backend must not subscribe it.
    stamp, accepted = publish_valid()
    with session.lock:
        before = len(session.playback)
    legacy = FormationTimedTrajectory()
    legacy.header.frame_id = "map"
    legacy.header.stamp = rospy.Time.now()
    legacy.plan_id = 999999
    publisher = rospy.Publisher("/formation/timed_trajectory",
                                FormationTimedTrajectory, queue_size=1)
    rospy.sleep(0.2)
    publisher.publish(legacy)
    rospy.sleep(0.6)
    publisher.unregister()
    with session.lock:
        after_documents = [d for _, d in session.playback[before:]]
    playing_after = [d for d in after_documents
                     if d.get("state") in (PLAYING, "HOLD_START")]
    note(checks, "late_legacy_trajectory_is_ignored",
         bool(playing_after) and
         all(d.get("plan_id") == accepted.get("plan_id")
             for d in playing_after),
         {"documents": len(after_documents),
          "states": sorted({d.get("state") for d in after_documents})})
    evidence["late_legacy"] = {
        "plan_id": accepted.get("plan_id"),
        "documents_after_publish": len(after_documents),
        "states_after_publish": sorted(
            {d.get("state") for d in after_documents}),
    }
    return evidence


# ---- orchestration ----------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description="FR05 live uniform playback acceptance driver")
    parser.add_argument("--scenario", choices=("main", "failure", "tamper"),
                        required=True)
    parser.add_argument("--output", required=True,
                        help="strict-JSON report path")
    parser.add_argument("--baseline-envelope", default="",
                        help="envelope JSON captured by the main scenario")
    parser.add_argument("--planning-timeout", type=float, default=150.0)
    return parser.parse_args()


def main():
    arguments = parse_args()
    if "ROS_MASTER_URI" not in os.environ:
        raise SmokeError("ROS_MASTER_URI is not set; attach to the demo")
    port = os.environ["ROS_MASTER_URI"].rsplit(":", 1)[-1].rstrip("/")
    if port.isdigit() and int(port) in PROTECTED_PORTS:
        raise SmokeError(
            "refusing protected demo port {}; FR05 uses a private master"
            .format(port))

    rospy.init_node("fr05_uniform_playback_smoke", anonymous=True)
    session = Session()
    rospy.sleep(1.0)
    checks = {}
    output = Path(arguments.output)
    envelope_dir = output.parent / "envelopes"
    evidence = {
        "schema": "fr05.ros_uniform_playback_smoke.v1",
        "generated_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                          time.gmtime()),
        "ros_master_uri": os.environ["ROS_MASTER_URI"],
        "scenario": arguments.scenario,
        "topic_types": {
            topic: topic_type
            for topic, topic_type in rospy.get_published_topics()
        },
        "params": {
            "uniform_planner": rospy.get_param(
                "/formation_uniform_planner", {}),
            "playback": rospy.get_param("/formation_playback", {}),
        },
        "checks": checks,
    }
    try:
        if arguments.scenario == "main":
            scenario_main(session, evidence, envelope_dir, checks)
        elif arguments.scenario == "failure":
            scenario_failure(session, evidence, envelope_dir, checks)
        else:
            if not arguments.baseline_envelope:
                raise SmokeError(
                    "--baseline-envelope is required for the tamper scenario")
            document = json.loads(Path(arguments.baseline_envelope).read_text(
                encoding="utf-8"))
            scenario_tamper(session, evidence, checks, document)
        evidence["ok"] = True
    except Exception as exc:
        evidence["ok"] = False
        evidence["error"] = repr(exc)
        rospy.logerr("[fr05 smoke] %s", exc)
        raise
    finally:
        evidence["job_state_history"] = [
            {"received_s": received, "document": document}
            for received, document in session.job_states]
        evidence["request_history"] = [
            {"received_s": received,
             "plan_id": int(message.plan_id),
             "formation_start": [message.formation_start.x,
                                 message.formation_start.y,
                                 message.formation_start.theta],
             "formation_goal": [message.formation_goal.x,
                                message.formation_goal.y,
                                message.formation_goal.theta]}
            for received, message in session.requests]
        write_json(output, evidence)
        print("WROTE", output, "ok=", evidence.get("ok"))


if __name__ == "__main__":
    main()