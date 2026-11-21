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
``b03``      map05, scale 2.625, a fixed 3 m lateral goal and terminal hold.
             With the direct connector enabled this uses one audited O edge.
``b04``      map05, scale 2.625, a blocked diagonal with a verified A-to-O
             switch in one common-clock plan.

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
    PLAYBACK_PLAN_BACKEND,
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
# Frozen V0.2 contract (DECISIONS / FR03-FR04): goal and terminal-heading
# tolerance 0.02 m and 1 deg, search budget 10000 expansions / 30 s.  These
# literals are the acceptance target; they must not be replaced by whatever
# the running node happens to be configured with.
FROZEN_GOAL_XY_TOL_M = 0.02
FROZEN_GOAL_PSI_TOL_RAD = math.radians(1.0)
FROZEN_MAX_EXPANSIONS = 10000
FROZEN_MAX_SECONDS = 30.0
MAP05_SIGNATURE = "5a9ab3f50546275ad9b47ed0524e76431e2096c4c4f753dcf37243e4b293451f"
MAP15_SIGNATURE = "28f52629b8d298f7c4e4f72004402c02f02e6509d087781217f8c3f4fb59ff23"
MAP16_OBLIQUE_SIGNATURE = "1d64b50c64777f47a56de00968bbaa853c61678ac4dd0125401c68e0b9a01915"
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

    def wait_envelope(self, plan_id, timeout, *, executable=True):
        deadline = rospy.Time.now().to_sec() + timeout
        while rospy.Time.now().to_sec() < deadline:
            with self.lock:
                message = self.envelopes.get(plan_id)
            if message is not None and (not executable or (
                    message.model_json and len(message.trajectory.times_from_start) > 0)):
                return message
            rospy.sleep(0.02)
        raise SmokeError(
            "timeout waiting for {}envelope plan {}".format(
                "executable " if executable else "", plan_id))

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
        playback = [d for _, d in session.playback
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
    moving = [document for document in playback
              if document.get("state") in (PLAYING, HOLD_END)]
    note(checks, "playback_status_carries_envelope_identity",
         bool(moving) and all(
             document.get("model_sha256") == digest and
             document.get("formation_id") == envelope.formation_id and
             document.get("backend") == PLAYBACK_PLAN_BACKEND
             for document in moving),
         "{}".format([(document.get("plan_id"),
                       document.get("model_sha256"),
                       document.get("formation_id"))
                      for document in moving[:3]]))
    terminal = [document for document in moving
                if document.get("state") == HOLD_END]
    # The planner publishes its terminal status only after it receives the
    # playback HOLD_END, so the very first terminal tick can legitimately
    # still show AUDIT_PENDING (the pre-publication status carries no digest
    # yet).  What must hold is that certification *arrives* for exactly this
    # digest and never for a wrong or missing one; the cross-identity
    # negative case is covered by the unit tests.
    try:
        certified = session.wait_playback(
            lambda document: document.get("plan_id") == plan_id and
            document.get("state") == HOLD_END and
            document.get("executable") is True,
            2.0, "certified HOLD_END for plan {}".format(plan_id))
        certified_ok = (certified.get("model_sha256") == digest and
                        certified.get("formation_id") ==
                        envelope.formation_id)
        detail = {"terminal_documents": len(terminal),
                  "executable_flags": [document.get("executable")
                                       for document in terminal[:6]],
                  "certified_digest": certified.get("model_sha256")}
    except SmokeError as exc:
        certified_ok = False
        detail = {"terminal_documents": len(terminal),
                  "executable_flags": [document.get("executable")
                                       for document in terminal[:6]],
                  "error": str(exc)}
    note(checks, "playback_terminal_is_certified_for_that_digest",
         bool(terminal) and certified_ok, detail)


def run_experiment(session, checks, name, goal, initial=None,
                   busy_probe=False, planning_timeout=150.0,
                   envelope_dir=None, evidence=None, *, allow_planning_failure=False):
    result = {"name": name, "goal": list(goal)}
    with session.lock:
        mark_tf = len(session.transforms)
        mark_marker = len(session.markers)
    if initial is not None:
        publish_initial_pose(*initial)
        result["initial_pose"] = list(initial)
    with session.lock:
        mark_playback = len(session.playback)
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
    envelope = session.wait_envelope(plan_id, planning_timeout,
                                     executable=not allow_planning_failure)
    envelope_time = rospy.Time.now().to_sec()
    result["planning_wall_s"] = envelope_time - goal_time
    if allow_planning_failure and not envelope.model_json:
        failed = session.wait_job(lambda d: d.get("plan_id") == plan_id
                                  and d.get("state") == "FAILED", 5.0, "job FAILED")
        cleared = session.wait_playback(
            lambda d: d.get("state") == "CLEARED"
            and str(d.get("reason", "")).startswith("invalid playback plan:"),
            5.0, "playback CLEARED", mark=mark_playback)
        with session.lock:
            status = next(d for _, d in reversed(session.statuses)
                          if d.get("plan_id") == plan_id and d.get("node_state") == "REJECTED")
        note(checks, name + "_admitted_planning_failure", status.get("admitted") is True
             and status.get("pipeline_status") in ("BUDGET_EXHAUSTED", "NO_SOLUTION", "AUDIT_REJECTED"),
             status.get("reason"))
        note(checks, name + "_no_executable_plan_or_playback",
             not envelope.model_sha256 and not envelope.trajectory.times_from_start
             and cleared.get("executable") is False and status.get("executable") is False
             and not any(d.get("state") in (PLAYING, HOLD_END)
                         for _, d in session.playback_history(plan_id)), cleared.get("state"))
        result.update(failure=failed, search=status["search"], reason=status["reason"],
                      planner_wall_time_s=status["wall_time_s"])
        if evidence is not None:
            evidence.setdefault("experiments", []).append(result)
        return result
    stamp = int(envelope.header.stamp.secs) + (
        int(envelope.header.stamp.nsecs) * 1e-9)
    result["stamp_to_receipt_s"] = envelope_time - stamp
    note(checks, name + "_stamp_not_after_receipt", stamp <= envelope_time +
         0.05, "stamp {:.6f} receipt {:.6f}".format(stamp, envelope_time))

    plan, fields, summary, path = analyze_envelope(
        envelope, checks, envelope_dir)
    result["envelope"] = summary
    result["envelope_path"] = str(path)
    # The published model goal may be the actually achieved terminal, but a
    # successful plan must still satisfy the frozen tolerance against the
    # original request — rewriting the message goal must not launder a miss.
    requested = result["goal"]
    achieved = summary["goal_qf"]
    request_xy_error = math.hypot(achieved[0] - requested[0],
                                  achieved[1] - requested[1])
    request_psi_error = abs(wrap(achieved[2] - requested[2]))
    result["request_goal_error"] = {
        "xy_m": request_xy_error, "psi_rad": request_psi_error}
    note(checks, name + "_achieved_goal_within_frozen_request_tolerance",
         request_xy_error <= FROZEN_GOAL_XY_TOL_M + 1e-9 and
         request_psi_error <= FROZEN_GOAL_PSI_TOL_RAD + 1e-12,
         {"requested": requested, "achieved": achieved,
          "xy_error_m": request_xy_error,
          "psi_error_rad": request_psi_error,
          "frozen_xy_tolerance_m": FROZEN_GOAL_XY_TOL_M,
          "frozen_psi_tolerance_rad": FROZEN_GOAL_PSI_TOL_RAD})

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


def scenario_near_goal(session, evidence, envelope_dir, checks,
                       planning_timeout=150.0):
    """Real near-goal request from the Codex probe: (10,10,0)->(10.1,10,0.1).

    The requested offset (0.1 m / 0.1 rad) is outside the frozen 0.02 m / 1 deg
    contract.  A published plan therefore has to really reach that tolerance,
    and a NO_MOTION/publication that stays at the start is the historical
    misjudgment.  Honest failure is accepted: this negative case must not
    require new search ability.
    """
    session.wait_job(lambda d: d.get("state") in ("IDLE", "HOLD_END",
                                                  "FAILED"), 10.0,
                     "job admitting the near-goal probe")
    publish_initial_pose(10.0, 10.0, 0.0)
    rospy.sleep(0.6)
    request_count = len(session.requests)
    goal_time = rospy.Time.now().to_sec()
    publish_goal(10.1, 10.0, 0.1)
    request = session.wait_request_after(request_count, 5.0)
    plan_id = int(request.plan_id)
    requested = [10.1, 10.0, 0.1]
    started = [request.formation_start.x, request.formation_start.y,
               request.formation_start.theta]
    result = {"plan_id": plan_id, "requested_goal": requested,
              "request_start": started,
              "frozen_xy_tolerance_m": FROZEN_GOAL_XY_TOL_M,
              "frozen_psi_tolerance_rad": FROZEN_GOAL_PSI_TOL_RAD}
    note(checks, "near_goal_request_starts_at_the_probe_pose",
         math.hypot(started[0] - 10.0, started[1] - 10.0) <= 1e-9 and
         abs(wrap(started[2] - 0.0)) <= 1e-9, started)

    deadline = goal_time + planning_timeout
    envelope = None
    failure = None
    while rospy.Time.now().to_sec() < deadline:
        with session.lock:
            candidate = session.envelopes.get(plan_id)
            jobs = [d for _, d in session.job_states
                    if d.get("plan_id") == plan_id]
        # A failed admission deliberately overwrites the latched envelope with
        # an explicitly invalid placeholder (no model, no samples, no
        # certification).  That is not an executable plan; only a populated
        # envelope counts, and a FAILED job is checked first.
        if jobs and jobs[-1].get("state") == "FAILED":
            failure = jobs[-1]
            break
        if (candidate is not None and candidate.model_json and
                len(candidate.trajectory.times_from_start) > 0):
            envelope = candidate
            break
        rospy.sleep(0.05)
    result["wall_s"] = rospy.Time.now().to_sec() - goal_time

    if envelope is not None:
        plan, fields, summary, path = analyze_envelope(
            envelope, checks, envelope_dir)
        achieved = summary["goal_qf"]
        xy_error = math.hypot(achieved[0] - requested[0],
                              achieved[1] - requested[1])
        psi_error = abs(wrap(achieved[2] - requested[2]))
        result.update({"outcome": "published", "achieved_goal": achieved,
                       "xy_error_m": xy_error, "psi_error_rad": psi_error,
                       "envelope": summary, "envelope_path": str(path)})
        with session.lock:
            statuses = [d for _, d in session.statuses
                        if d.get("plan_id") == plan_id]
        published = [d for d in statuses
                     if d.get("node_state") == "PUBLISHED"]
        result["optimizer_status"] = published[-1] if published else None
        result["status_word"] = (
            published[-1].get("status") if published else None)
        note(checks, "near_goal_published_plan_really_reaches_tolerance",
             xy_error <= FROZEN_GOAL_XY_TOL_M + 1e-9 and
             psi_error <= FROZEN_GOAL_PSI_TOL_RAD + 1e-12,
             {"requested": requested, "achieved": achieved,
              "xy_error_m": xy_error, "psi_error_rad": psi_error,
              "status": result["status_word"]})
    elif failure is not None:
        with session.lock:
            statuses = [d for _, d in session.statuses
                        if d.get("plan_id") == plan_id]
        rejected = [d for d in statuses
                    if d.get("node_state") in ("REJECTED", "INVALID")]
        layer = None
        if rejected:
            layer = {
                "reason": rejected[-1].get("reason"),
                "first_failure_layer": rejected[-1].get(
                    "first_failure_layer"),
                "pipeline_status": rejected[-1].get("pipeline_status"),
                "search": rejected[-1].get("search"),
            }
        result.update({"outcome": "failed", "job": failure,
                       "failure_layer": layer})
        note(checks, "near_goal_failure_is_honest",
             failure.get("state") == "FAILED", failure)
        note(checks, "near_goal_failure_layer_recorded",
             layer is not None, layer)
    else:
        result["outcome"] = "timeout"
        raise SmokeError(
            "near-goal probe produced neither an envelope nor job FAILED "
            "within {:.0f} s".format(planning_timeout))
    evidence["near_goal"] = result
    return result


def check_frozen_parameters(checks, params):
    """The running node must expose the frozen contract, not a wider copy."""
    def number(name):
        value = params.get(name) if isinstance(params, dict) else None
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        return float(value)

    xy_tol = number("goal_xy_tol")
    psi_tol = number("goal_psi_tol")
    heading_tol = number("goal_heading_tol")
    expansions = number("max_expansions")
    seconds = number("max_seconds")
    note(checks, "frozen_goal_xy_tolerance",
         xy_tol is not None and
         abs(xy_tol - FROZEN_GOAL_XY_TOL_M) <= 1e-12,
         {"configured": xy_tol, "frozen": FROZEN_GOAL_XY_TOL_M})
    note(checks, "frozen_goal_psi_tolerance",
         psi_tol is not None and
         abs(psi_tol - FROZEN_GOAL_PSI_TOL_RAD) <= 1e-12,
         {"configured": psi_tol, "frozen": FROZEN_GOAL_PSI_TOL_RAD})
    note(checks, "frozen_terminal_heading_tolerance",
         heading_tol is not None and
         abs(heading_tol - FROZEN_GOAL_PSI_TOL_RAD) <= 1e-12,
         {"configured": heading_tol, "frozen": FROZEN_GOAL_PSI_TOL_RAD})
    note(checks, "frozen_search_expansion_budget",
         expansions is not None and
         int(expansions) == FROZEN_MAX_EXPANSIONS,
         {"configured": expansions, "frozen": FROZEN_MAX_EXPANSIONS})
    note(checks, "frozen_search_time_budget",
         seconds is not None and
         abs(seconds - FROZEN_MAX_SECONDS) <= 1e-12,
         {"configured": seconds, "frozen": FROZEN_MAX_SECONDS})


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
    #    round 0/0.6 values used by the other experiments).  The published
    #    per-vehicle goal centres may be derived from the achieved model goal,
    #    but the achieved terminal itself must satisfy the frozen 0.02 m /
    #    1 deg tolerance against the *requested* pose; deriving them from the
    #    request made the planner's own envelope audit reject every real RViz
    #    plan.
    rviz_style = run_experiment(
        session, checks, "rviz_style",
        goal=(12.0, 12.0, 0.018), initial=(10.0, 12.0, 0.013),
        envelope_dir=envelope_dir, evidence=evidence)
    achieved_goal = rviz_style["envelope"]["goal_qf"]
    achieved_delta = math.hypot(achieved_goal[0] - 12.0,
                                achieved_goal[1] - 12.0)
    achieved_heading_error = abs(wrap(achieved_goal[2] - 0.018))
    rviz_style["requested_goal"] = [12.0, 12.0, 0.018]
    rviz_style["achieved_goal_delta_m"] = achieved_delta
    rviz_style["achieved_heading_error_rad"] = achieved_heading_error
    note(checks, "rviz_style_uses_nonzero_decimal_heading",
         abs(rviz_style["goal"][2] - 0.018) <= 1e-12,
         rviz_style["goal"])
    note(checks, "rviz_style_achieved_goal_inside_frozen_tolerance",
         achieved_delta <= FROZEN_GOAL_XY_TOL_M + 1e-9 and
         achieved_heading_error <= FROZEN_GOAL_PSI_TOL_RAD + 1e-12,
         {"requested": [12.0, 12.0, 0.018], "achieved": achieved_goal,
          "delta_m": achieved_delta,
          "heading_error_rad": achieved_heading_error,
          "frozen_xy_tolerance_m": FROZEN_GOAL_XY_TOL_M,
          "frozen_heading_tolerance_rad": FROZEN_GOAL_PSI_TOL_RAD})

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

def fixed_map_experiment(session, evidence, envelope_dir, checks, name, start, goal,
                         planning_timeout, *, map_signature=MAP05_SIGNATURE,
                         allow_planning_failure=False):
    ready = session.wait_job(lambda d: d.get("state") == "IDLE" and d.get("map_ready"),
                             15.0, "ready planner for fixed map")
    note(checks, name + "_fixed_map", ready.get("map_signature") == map_signature,
         ready.get("map_signature"))
    result = run_experiment(
        session, checks, name, goal, initial=start,
        planning_timeout=planning_timeout, envelope_dir=envelope_dir, evidence=evidence,
        allow_planning_failure=allow_planning_failure)
    if "failure" in result:
        return result
    with session.lock:
        status = next(document for _, document in reversed(session.statuses)
                      if document.get("plan_id") == result["plan_id"]
                      and document.get("node_state") == "PUBLISHED")
    search = status["search"]
    evaluation = status["evaluation"]
    result["search"] = search
    result["evaluation"] = evaluation
    note(checks, name + "_independent_rigid_and_swept_audit",
         evaluation.get("independent_audit") is True
         and evaluation.get("collision_certificate") is True
         and evaluation["metrics"]["max_rigidity_error_m"] <= POSITION_TOL_M,
         evaluation["status"])
    return result


def scenario_b03(session, evidence, envelope_dir, checks, planning_timeout):
    result = fixed_map_experiment(
        session, evidence, envelope_dir, checks, "b03_lateral",
        (2.0, 0.0, 0.0), (2.0, 3.0, 0.0), planning_timeout)
    envelope = result["envelope"]
    headings = envelope["goal_headings"]
    note(checks, "b03_heading_preserved", headings == envelope["start_headings"], headings)
    edges = envelope["edges"]
    note(checks, "b03_uses_o", bool(edges) and
         all(edge["primitive"] == "O_TRANSLATION" for edge in edges), edges)
    search = result["search"]
    if evidence["params"]["uniform_planner"].get("direct_goal_translation"):
        note(checks, "b03_one_direct_edge", len(edges) == 1 and search["expansions"] == 0
             and (search.get("direct_goal_translation") or "").startswith("O_direct_goal"), search)


def scenario_b04(session, evidence, envelope_dir, checks, planning_timeout):
    result = fixed_map_experiment(
        session, evidence, envelope_dir, checks, "b04_switch",
        (-3.0, 2.5, 0.0), (2.0, -1.0, 0.0), planning_timeout)
    search = result["search"]
    note(checks, "b04_goal_connector", search["direct_goal_translation"] == "A_then_O_direct_goal"
         and search["expansions"] == 0, search)
    note(checks, "b04_direct_diagonal_blocked",
         search["rejection_histogram"].get("MAP_OBB_COLLISION", 0) > 0,
         search["rejection_histogram"])
    envelope = json.loads(Path(result["envelope_path"]).read_text())
    plan = UniformTimedPlan.from_model_json(envelope["model_json"])
    note(checks, "b04_phase_order", [(phase.kind, phase.mode) for phase in plan.phases]
         == [("motion", "A"), ("setup", "A"), ("motion", "O")], phase_summary(plan))
    setup = plan.phases[1]
    midpoint = (float(setup.t0_s) + float(setup.end_time)) / 2
    before = plan.state_at(float(setup.t0_s), side="left")
    during = plan.state_at(midpoint)
    after = plan.state_at(float(setup.end_time), side="right")
    for name, state in (("before", before), ("during", during), ("after", after)):
        note(checks, "b04_stationary_" + name, state.qf == FormationPose(2.0, 2.5, 0.0)
             and all(vehicle.center_speed_mps <= 1e-9 and abs(vehicle.theta) <= 1e-9
                     and max(map(abs, vehicle.wheel_rolling_speeds)) <= 1e-9
                     for vehicle in state.vehicles), state.as_dict())
    note(checks, "b04_steering_reconfigured_at_stop",
         during.from_mode == "A" and during.to_mode == "O" and all(
             max(map(abs, middle.wheel_angular_rates)) > 0.0
             and all(abs(abs(wrap(a - b)) - math.pi / 2) <= 1e-9
                     for a, b in zip(old.wheel_angles, new.wheel_angles))
             for old, middle, new in zip(before.vehicles, during.vehicles, after.vehicles)),
         during.as_dict())
    transition_ticks = [document for _, document in session.playback_history(result["plan_id"])
                        if document.get("setup_transition") == ["A", "O"]]
    note(checks, "b04_live_transition", bool(transition_ticks) and all(
         all(vehicle["center_speed_mps"] <= 1e-9 for vehicle in document["vehicles"])
         for document in transition_ticks), len(transition_ticks))


def scenario_b07(session, evidence, envelope_dir, checks, planning_timeout):
    start = (0.0, 2.5, 0.0)
    goal = (math.sqrt(3.0), 3.5, math.pi / 3.0)
    enabled = evidence["params"]["uniform_planner"].get("direct_goal_translation")
    result = fixed_map_experiment(session, evidence, envelope_dir, checks, "b07_pivot",
                                  start, goal, planning_timeout, allow_planning_failure=not enabled)
    if not enabled:
        return
    search = result["search"]
    note(checks, "b07_exact_goal_connector", search["expansions"] == 0
         and search["direct_goal_translation"] == "S_then_A_arc_direct_goal", search)
    envelope = json.loads(Path(result["envelope_path"]).read_text())
    plan = UniformTimedPlan.from_model_json(envelope["model_json"])
    note(checks, "b07_phase_order", [(phase.kind, phase.mode) for phase in plan.phases]
         == [("initial_setup", "A"), ("motion", "S"), ("setup", "S"), ("motion", "A")],
         phase_summary(plan))
    spin, arc = plan.edges
    note(checks, "b07_off_center_arc", spin.primitive == "S_SPIN" and arc.primitive == "A_ARC"
         and abs(arc.control.pivot_x) <= 1e-9 and abs(arc.control.pivot_y - 2.0) <= 1e-9,
         result["envelope"]["edges"])
    spin_phase, switch, arc_phase = plan.phases[1:]
    spin_start = plan.state_at(float(spin_phase.t0_s), side="right")
    spinning = plan.state_at((float(spin_phase.t0_s) + float(spin_phase.end_time)) / 2.0)
    stopped = plan.state_at((float(switch.t0_s) + float(switch.end_time)) / 2.0)
    driving = plan.state_at((float(arc_phase.t0_s) + float(arc_phase.end_time)) / 2.0)
    note(checks, "b07_spin_has_fixed_centers_and_actual_yaw", spin_start.qf == spinning.qf
         and all(math.hypot(a.x - b.x, a.y - b.y) <= 1e-9 and b.center_speed_mps <= 1e-9
                 for a, b in zip(spin_start.vehicles, spinning.vehicles))
         and any(abs(vehicle.yaw_rate_radps) > 0.1 for vehicle in spinning.vehicles),
         spinning.as_dict())
    note(checks, "b07_switch_stops_body_motion", stopped.from_mode == "S" and stopped.to_mode == "A"
         and all(vehicle.center_speed_mps <= 1e-9 and abs(vehicle.yaw_rate_radps) <= 1e-9
                 and max(map(abs, vehicle.wheel_rolling_speeds)) <= 1e-9
                 for vehicle in stopped.vehicles), stopped.as_dict())
    note(checks, "b07_a_arc_moves_every_center", all(
         math.hypot(a.x - b.x, a.y - b.y) > 0.1 and b.center_speed_mps > 0.1
         for a, b in zip(stopped.vehicles, driving.vehicles)), driving.as_dict())
    live = [document for _, document in session.playback_history(result["plan_id"])
            if document.get("state") == PLAYING and not document.get("setup_transition")]
    note(checks, "b07_live_spin_has_yaw_rate", any(
         vehicle["mode_name"] == "S" and abs(vehicle.get("yaw_rate_radps", 0.0)) > 0.1
         and vehicle["center_speed_mps"] <= 1e-9
         for document in live for vehicle in document["vehicles"]), len(live))


def scenario_map15_offset(session, evidence, envelope_dir, checks, planning_timeout):
    enabled = evidence["params"]["uniform_planner"].get("direct_goal_translation")
    start, goal = (-5.5, 0.16, 0.0), (14.0, 7.1, 0.0)
    result = fixed_map_experiment(
        session, evidence, envelope_dir, checks, "map15_offset", start, goal,
        planning_timeout, map_signature=MAP15_SIGNATURE, allow_planning_failure=not enabled)
    if not enabled:
        return
    search = result["search"]
    note(checks, "map15_offset_connector",
         search["direct_goal_translation"] == "A_then_O_then_A_direct_goal"
         and search["expansions"] == 0 and search["offset_preview_rejections"] > 0
         and search["rejection_histogram"].get("MAP_OBB_COLLISION", 0) >= 3, search)
    envelope = json.loads(Path(result["envelope_path"]).read_text())
    plan = UniformTimedPlan.from_model_json(envelope["model_json"])
    note(checks, "map15_offset_phase_order", [(phase.kind, phase.mode) for phase in plan.phases]
         == [("motion", "A"), ("setup", "A"), ("motion", "O"), ("setup", "O"), ("motion", "A")],
         phase_summary(plan))
    first, crossing, final = plan.edges
    note(checks, "map15_fixed_headings_and_exact_crossing",
         first.start_headings == crossing.start_headings == final.end_headings
         and abs(crossing.start_qf.x - crossing.end_qf.x) < 1e-9
         and abs(crossing.start_qf.y - start[1]) < 1e-9
         and abs(crossing.end_qf.y - goal[1]) < 1e-9,
         {name: [pose.x, pose.y, pose.psi] for name, pose in (
             ("start_f", first.start_qf), ("cross_f", crossing.start_qf),
             ("second_entry_f", final.start_qf))})
    # Nominal wall ends at this scale. Count the footprint padding so all
    # vehicles finish preparation outside the respective narrow lane.
    front_padding_m = (0.738 + 0.05) / 2
    for index, transition in ((1, ("A", "O")), (3, ("O", "A"))):
        phase = plan.phases[index]
        setup = plan.state_at((float(phase.t0_s) + float(phase.end_time)) / 2)
        xmin = min(vehicle.x for vehicle in setup.vehicles) - front_padding_m
        xmax = max(vehicle.x for vehicle in setup.vehicles) + front_padding_m
        note(checks, "map15_setup_{}_{}_in_open_area".format(*transition),
             -1.68 < xmin <= xmax < 3.90 and (setup.from_mode, setup.to_mode) == transition
             and all(abs(v.yaw_rate_radps) <= 1e-9 and v.center_speed_mps <= 1e-9
                     and max(map(abs, v.wheel_rolling_speeds)) <= 1e-9
                     for v in setup.vehicles)
             and any(max(map(abs, v.wheel_angular_rates)) > 0.0 for v in setup.vehicles),
             {"padded_formation_x_m": [xmin, xmax], "state": setup.as_dict()})
    live = [document for _, document in session.playback_history(result["plan_id"])
            if document.get("setup_transition")]
    note(checks, "map15_live_setups", {tuple(d["setup_transition"]) for d in live}
         == {("A", "O"), ("O", "A")} and all(
             all(v["center_speed_mps"] <= 1e-9 for v in d["vehicles"]) for d in live), len(live))


def scenario_map15_preparation(session, evidence, envelope_dir, checks, planning_timeout):
    from nav_msgs.msg import OccupancyGrid
    from formation_uniform_planner_node import grid_from_occupancy_grid
    from fwis_v3_optimizer.formation_collision import FormationCollisionChecker

    enabled = evidence["params"]["uniform_planner"].get("direct_goal_translation")
    yaw = math.radians(20.0)
    start, goal = (1.0, 3.0, yaw), (10.0, 7.3, yaw)
    result = fixed_map_experiment(
        session, evidence, envelope_dir, checks, "map15_preparation", start, goal,
        planning_timeout, map_signature=MAP15_SIGNATURE, allow_planning_failure=not enabled)
    if not enabled:
        return

    search = result["search"]
    note(checks, "map15_preparation_connector", search["expansions"] == 0
         and search["direct_goal_translation"] == "O_then_S_then_A_direct_goal"
         and search["straight_preparation_probes"] > 0, search)
    model_json = json.loads(Path(result["envelope_path"]).read_text())["model_json"]
    plan = UniformTimedPlan.from_model_json(model_json)
    note(checks, "map15_preparation_phase_order", [(p.kind, p.mode) for p in plan.phases]
         == [("initial_setup", "A"), ("motion", "O"), ("setup", "O"),
             ("motion", "S"), ("setup", "S"), ("motion", "A")], phase_summary(plan))
    incoming, spin, outgoing = plan.edges
    prep = spin.start_qf
    note(checks, "map15_approach_stops_and_aligns_on_lane_axis",
         incoming.end_qf == prep == spin.end_qf == outgoing.start_qf
         and abs(prep.x - start[0]) < 1e-9 and abs(prep.y - goal[1]) < 1e-9
         and abs(prep.psi - yaw) < 1e-9 and abs(outgoing.end_qf.psi - yaw) < 1e-9
         and all(abs(wrap(a - yaw)) < 1e-9 for _, a in spin.start_headings)
         and all(abs(wrap(a)) < 1e-9 for _, a in spin.end_headings), [prep.x, prep.y, prep.psi])
    for index, transition in ((2, ("O", "S")), (4, ("S", "A"))):
        phase = plan.phases[index]
        state = plan.state_at((float(phase.t0_s) + float(phase.end_time)) / 2.0)
        note(checks, "map15_stationary_setup_{}_{}".format(*transition), state.qf == prep
             and (state.from_mode, state.to_mode) == transition
             and all(abs(v.yaw_rate_radps) <= 1e-9 and v.center_speed_mps <= 1e-9
                     and max(map(abs, v.wheel_rolling_speeds)) <= 1e-9 for v in state.vehicles),
             state.as_dict())
    phase = plan.phases[3]
    spinning = plan.state_at((float(phase.t0_s) + float(phase.end_time)) / 2.0)
    margin = result["evaluation"]["metrics"]["collision_margin_m"]
    note(checks, "map15_body_spin_takes_place_before_entrance", spinning.qf == prep
         and all(v.center_speed_mps <= 1e-9 and abs(v.yaw_rate_radps) > 0.1
                 for v in spinning.vehicles)
         and all(v.x + math.hypot((plan.vehicle_shapes[v.vehicle_id].length_m + margin) / 2,
                                  (plan.vehicle_shapes[v.vehicle_id].width_m + margin) / 2) < 3.90
                 for v in spinning.vehicles), spinning.as_dict())

    grid, error = grid_from_occupancy_grid(rospy.wait_for_message("/map", OccupancyGrid, timeout=5.0))
    note(checks, "map15_independent_raster_loaded", grid is not None, error)
    obstacles = FormationCollisionChecker(grid, plan.model, plan.vehicle_shapes, margin)
    unaligned = obstacles.static_check(plan.goal_qf, dict(plan.start_headings))
    aligned = obstacles.static_check(plan.goal_qf, dict(plan.goal_headings))
    note(checks, "map15_incoming_body_orientation_blocked_at_goal",
         unaligned.reason == "MAP_OBB_COLLISION" and aligned.success,
         {"blocked_vehicle": unaligned.vehicle_id, "incoming_poses_clear": unaligned.success,
          "aligned_poses_clear": aligned.success})
    live = [document for _, document in session.playback_history(result["plan_id"])
            if document.get("state") == PLAYING and not document.get("setup_transition")]
    note(checks, "map15_playback_displays_stationary_body_spin", any(
         all(vehicle["mode_name"] == "S" and abs(vehicle.get("yaw_rate_radps", 0)) > .1
             and vehicle["center_speed_mps"] <= 1e-9 for vehicle in doc["vehicles"])
         for doc in live), len(live))


def scenario_b09(session, evidence, envelope_dir, checks, planning_timeout):
    yaw = math.pi / 3
    arc_goal = (math.sqrt(3), 3.5, yaw)
    reverse_goal = (arc_goal[0] - 1.0, arc_goal[1] - 1.0, yaw)
    results = [fixed_map_experiment(session, evidence, envelope_dir, checks, "b09_arc",
                                    (0.0, 2.5, 0.0), arc_goal, planning_timeout)]
    for name, goal in (("b09_reverse", reverse_goal), ("b09_forward", arc_goal)):
        rospy.sleep(0.5)
        results.append(run_experiment(session, checks, name, goal, planning_timeout=planning_timeout,
                                      envelope_dir=envelope_dir, evidence=evidence))
    plans = [UniformTimedPlan.from_model_json(json.loads(Path(item["envelope_path"]).read_text())["model_json"])
             for item in results]
    arc, reverse, forward = plans
    note(checks, "b09_arc_terminal_is_not_canonical", arc.initial_edge is None
         and arc.edges[-1].primitive == "A_ARC"
         and any(abs(wrap(theta - arc.goal_qf.psi)) > 0.1 for _, theta in arc.goal_headings),
         arc.goal_headings)
    for index, gear in ((1, -1), (2, 1)):
        prior, plan = plans[index - 1:index + 1]
        name = results[index]["name"]
        with session.lock:
            request = next(m for _, m in session.requests if m.plan_id == plan.plan_id)
            status = next(d for _, d in reversed(session.statuses)
                          if d.get("plan_id") == plan.plan_id and d.get("node_state") == "PUBLISHED")
        note(checks, name + "_binds_confirmed_predecessor",
             request.continuation_plan_id == prior.plan_id
             and request.continuation_model_sha256 == prior.model_sha256()
             and plan.initial_edge is not None and plan.initial_edge.primitive == prior.edges[-1].primitive
             and plan.start_qf == prior.goal_qf and plan.start_headings == prior.goal_headings,
             request.continuation_plan_id)
        stopped = prior.state_at(prior.duration_s)
        restart = plan.state_at(0.0)
        note(checks, name + "_inherits_stopped_wheels", restart.qf == stopped.qf and all(
            abs(a.theta - b.theta) <= 1e-9 and a.center_speed_mps <= 1e-9 and b.center_speed_mps <= 1e-9
            and max(map(abs, a.wheel_rolling_speeds)) <= 1e-9
            and max(map(abs, b.wheel_rolling_speeds)) <= 1e-9
            and all(abs(wrap(x - y)) <= 1e-9 for x, y in zip(a.wheel_angles, b.wheel_angles))
            for a, b in zip(stopped.vehicles, restart.vehicles)), restart.as_dict())
        note(checks, name + "_audited_signed_drive", len(plan.edges) == 1
             and plan.edges[0].primitive == "O_TRANSLATION"
             and all(direction == gear for _, direction in plan.edges[0].directions)
             and status["evaluation"].get("collision_certificate") is True,
             dict(plan.edges[0].directions))
        live = [d for _, d in session.playback_history(plan.plan_id) if d.get("state") == PLAYING]
        note(checks, name + "_live_signed_wheel_speeds", any(all(
            v.get("gear_direction") == gear
            and all(gear * speed > 0.1 for speed in v.get("wheel_rolling_speeds_mps", ()))
            and len(v.get("wheel_rolling_speeds_mps", ())) == 4 for v in d["vehicles"])
            for d in live), len(live))
        results[index]["continuation"] = {"plan_id": request.continuation_plan_id,
                                          "model_sha256": request.continuation_model_sha256}
        results[index]["search"] = status["search"]
    note(checks, "b09_setup_from_actual_arc_wheels", len(reverse.phases) == 2
         and reverse.phases[0].kind == "initial_setup"
         and all(d.from_mode == "A" and d.to_mode == "O" and d.to_direction == -1
                 for d in reverse.phases[0].demands), phase_summary(reverse))
    note(checks, "b09_no_redundant_setup_on_same_angles_after_reverse",
         len(forward.phases) == 1 and forward.phases[0].kind == "motion"
         and forward.initial_edge.directions == reverse.edges[-1].directions,
         phase_summary(forward))

    stale = copy.deepcopy(request)
    stale.header.stamp = rospy.Time.now()
    stale.plan_id = forward.plan_id + 100
    publisher = rospy.Publisher(REQUEST_TOPIC, FormationPlanRequest, queue_size=1)
    rospy.sleep(0.3)
    publisher.publish(stale)
    deadline = time.monotonic() + 5.0
    rejection = None
    while time.monotonic() < deadline:
        with session.lock:
            rejection = next((d for _, d in reversed(session.statuses)
                              if d.get("plan_id") == stale.plan_id), None)
        if rejection is not None:
            break
        rospy.sleep(0.02)
    publisher.unregister()
    job = session.latest_job()
    note(checks, "b09_stale_predecessor_rejected_without_releasing_hold",
         rejection is not None and rejection.get("reason") == "INVALID_CONTINUATION"
         and rejection.get("admitted") is False and stale.plan_id not in session.envelopes
         and job.get("plan_id") == forward.plan_id and job.get("state") == HOLD_END,
         rejection)


def scenario_map15_global(session, evidence, envelope_dir, checks, planning_timeout):
    params = evidence["params"]["uniform_planner"]
    enabled = params.get("direct_goal_translation") and params.get("global_translation_route")
    start, goal = (-5.5, 0.16, 0.0), (12.0, 10.8, 0.0)
    result = fixed_map_experiment(
        session, evidence, envelope_dir, checks, "map15_global", start, goal,
        planning_timeout, map_signature=MAP15_SIGNATURE, allow_planning_failure=not enabled)
    if not enabled:
        return
    search = result["search"]
    route = search["global_route"]
    note(checks, "map15_whole_formation_global_route_found",
         search["direct_goal_translation"] == "fixed_heading_global_route"
         and search["expansions"] == 0 and route["status"] == "FOUND"
         and route["expansions"] > 0 and search["offset_preview_rejections"] > 0,
         route)
    model = json.loads(Path(result["envelope_path"]).read_text())["model_json"]
    plan = UniformTimedPlan.from_model_json(model)
    note(checks, "map15_global_mixes_a_o_and_final_reverse",
         tuple(edge.mode for edge in plan.edges) == ("A", "O", "A", "O", "A")
         and all(direction < 0 for _, direction in plan.edges[-1].directions)
         and all(edge.start_headings == plan.start_headings == edge.end_headings
                 and edge.start_qf.psi == edge.end_qf.psi == 0.0 for edge in plan.edges),
         result["envelope"]["edges"])
    actual = [plan.start_qf] + [edge.end_qf for edge in plan.edges]
    note(checks, "map15_exact_route_reaches_second_passage_far_exit",
         len(route["waypoints"]) == len(actual)
         and all(math.hypot(x - pose.x, y - pose.y) <= 1e-9
                 for (x, y), pose in zip(route["waypoints"], actual))
         and 5.62 < actual[3].y < 8.59 and actual[3].x > 18.75
         and actual[4].y > 8.59 and actual[-1].x < actual[4].x, route["waypoints"])
    setup_count = 0
    for phase in plan.phases:
        if phase.kind != "setup":
            continue
        setup_count += 1
        state = plan.state_at(float(phase.t0_s + phase.end_time) / 2.0)
        note(checks, "map15_global_parked_" + phase.phase_id,
             state.from_mode == phase.from_mode and state.to_mode == phase.to_mode
             and all(abs(vehicle.center_speed_mps) <= 1e-9
                     and max(map(abs, vehicle.wheel_rolling_speeds)) <= 1e-9
                     and max(map(abs, vehicle.wheel_angular_rates)) > 0.0
                     for vehicle in state.vehicles), state.as_dict())
    note(checks, "map15_global_four_checked_switches", setup_count == 4, setup_count)


def scenario_map16_oblique(session, evidence, envelope_dir, checks, planning_timeout):
    params = evidence["params"]["uniform_planner"]
    heading_aligned = params.get("heading_aligned_route", True)
    yaw = 3 * math.pi / 4
    result = fixed_map_experiment(
        session, evidence, envelope_dir, checks, "map16_oblique",
        (-1.0, 0.0, yaw), (4.62, 14.65, yaw), planning_timeout,
        map_signature=MAP16_OBLIQUE_SIGNATURE)
    search = result["search"]
    route = search["global_route"]
    note(checks, "map16_selects_an_audited_global_route", route["status"] == "FOUND"
         and search["expansions"] == 0 and search["direct_goal_translation"] == "fixed_heading_global_route",
         search)
    model = json.loads(Path(result["envelope_path"]).read_text())["model_json"]
    plan = UniformTimedPlan.from_model_json(model)
    modes = tuple(edge.mode for edge in plan.edges)
    note(checks, "map16_body_headings_unchanged", all(
        abs(wrap(edge.start_qf.psi - yaw)) <= 1e-9 and abs(wrap(edge.end_qf.psi - yaw)) <= 1e-9
        and edge.start_headings == plan.start_headings == edge.end_headings for edge in plan.edges),
        result["envelope"]["edges"])
    if not heading_aligned:
        note(checks, "map16_axis_control_uses_o", route["frame_angle_rad"] == 0.0
             and modes == ("O",) * 4 and route["a_distance_m"] == 0.0, route)
        return
    native = next(candidate for candidate in route["alternatives"] if candidate["frame_angle_rad"] == 0.0)
    note(checks, "map16_heading_aligned_candidate_wins", abs(route["frame_angle_rad"]) > 0.1
         and native["status"] == "FOUND" and route["score_m"] < native["score_m"]
         and route["length_m"] < native["length_m"] and route["a_distance_m"] > 6.0, route)
    late = plan.edges[-1]
    bearing = math.atan2(late.end_qf.y - late.start_qf.y, late.end_qf.x - late.start_qf.x)
    note(checks, "map16_enters_diagonal_bay_in_a", modes == ("O", "A", "O", "A")
         and abs(wrap(bearing - yaw)) <= 1e-9
         and all(gear == 1 for _, gear in late.directions)
         and late.start_qf.y < 14.0 < late.end_qf.y, {"modes": modes, "bearing_rad": bearing})
    note(checks, "map16_both_mode_changes_parked", any(
        phase.kind == "setup" and (phase.from_mode, phase.to_mode) == ("O", "A")
        for phase in plan.phases) and any(
            phase.kind == "setup" and (phase.from_mode, phase.to_mode) == ("A", "O")
            for phase in plan.phases) and all(
                abs(v.center_speed_mps) <= 1e-9
                and max(map(abs, v.wheel_rolling_speeds)) <= 1e-9
                for phase in plan.phases if phase.kind == "setup"
                for v in plan.state_at((phase.t0_s + phase.end_time) / 2).vehicles), phase_summary(plan))


def parse_args():
    parser = argparse.ArgumentParser(
        description="FR05 live uniform playback acceptance driver")
    parser.add_argument(
        "--scenario", choices=("main", "failure", "tamper", "near_goal", "b03", "b04", "b07",
                               "map15_offset", "map15_preparation", "map15_global", "map16_oblique", "b09"),
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
        check_frozen_parameters(checks,
                                evidence["params"]["uniform_planner"])
        if arguments.scenario == "main":
            scenario_main(session, evidence, envelope_dir, checks)
        elif arguments.scenario == "near_goal":
            scenario_near_goal(session, evidence, envelope_dir, checks,
                               arguments.planning_timeout)
        elif arguments.scenario == "failure":
            scenario_failure(session, evidence, envelope_dir, checks)
        elif arguments.scenario == "b03":
            scenario_b03(session, evidence, envelope_dir, checks,
                         arguments.planning_timeout)
        elif arguments.scenario == "b04":
            scenario_b04(session, evidence, envelope_dir, checks,
                         arguments.planning_timeout)
        elif arguments.scenario == "b07":
            scenario_b07(session, evidence, envelope_dir, checks,
                         arguments.planning_timeout)
        elif arguments.scenario == "map15_offset":
            scenario_map15_offset(session, evidence, envelope_dir, checks,
                                  arguments.planning_timeout)
        elif arguments.scenario == "map15_preparation":
            scenario_map15_preparation(session, evidence, envelope_dir, checks,
                                       arguments.planning_timeout)
        elif arguments.scenario == "map15_global":
            scenario_map15_global(session, evidence, envelope_dir, checks, arguments.planning_timeout)
        elif arguments.scenario == "map16_oblique":
            scenario_map16_oblique(session, evidence, envelope_dir, checks, arguments.planning_timeout)
        elif arguments.scenario == "b09":
            scenario_b09(session, evidence, envelope_dir, checks, arguments.planning_timeout)
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
