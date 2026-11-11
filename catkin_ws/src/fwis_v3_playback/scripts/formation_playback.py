#!/usr/bin/env python3
"""S10 formation playback: common-clock RViz display for all vehicles.

This node never retimes, never certifies and never publishes a plan.  It
interpolates exactly one ``FormationTimedTrajectory`` from its common
``header.stamp`` and shared ``time_from_start`` axis, builds markers whose
namespaces contain both ``formation_id`` and ``vehicle_id``, and reports the
actual playback sample on ``/formation/playback_status``.
"""
import json
import math
import threading
from functools import wraps
from types import SimpleNamespace
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3, PoseStamped
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from fwis_v3_msgs.msg import FormationTimedTrajectory, FormationPlanRequest

from fwis_v3_optimizer.formation_playback import (
    CLEARED,
    INVALID,
    PlaybackMarkerOptions,
    PlaybackValidationError,
    build_markers,
    clear_marker_specs,
    frame_at,
    status_document,
    terminal_errors,
    tf_frame_prefix,
    trajectory_from_fields,
    PlaybackVehicleSpec,
    build_hold_markers,
)
from fwis_v3_optimizer.transport_contract import load_contract

MARKER_TOPIC = "/formation/markers"
STATUS_TOPIC = "/formation/playback_status"
TRAJECTORY_TOPIC = "/formation/timed_trajectory"
OPTIMIZER_STATUS_TOPIC = "/formation/optimizer_status"
QUALITY_TOPIC = "/formation/quality_report"


def synchronized(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return call


def _seconds(value):
    if hasattr(value, "to_sec"):
        return float(value.to_sec())
    return float(value)


def fields_from_message(message):
    """Convert the ROS message into plain display fields, failing closed."""
    if not hasattr(message, "header"):
        raise PlaybackValidationError("message has no header")
    vehicles = []
    for vehicle in message.vehicles:
        offset = vehicle.offset_in_formation
        vehicles.append({
            "vehicle_id": int(vehicle.vehicle_id),
            "offset": (float(offset.x), float(offset.y)),
            "body_length_m": float(vehicle.body_length_m),
            "body_width_m": float(vehicle.body_width_m),
        })
    trajectories = []
    for trajectory in message.trajectories:
        points = []
        for point in trajectory.points:
            pose = point.center_pose
            points.append((
                _seconds(point.time_from_start),
                float(pose.x), float(pose.y), float(pose.theta),
                float(point.center_speed_mps),
                float(point.center_yaw_rate_radps),
                int(point.mode), float(point.mode_control)))
        trajectories.append({
            "vehicle_id": int(trajectory.vehicle_id),
            "points": tuple(points),
        })
    return {
        "frame_id": message.header.frame_id,
        "stamp": (int(message.header.stamp.secs),
                  int(message.header.stamp.nsecs)),
        "plan_id": int(message.plan_id),
        "formation_id": message.formation_id,
        "vehicles": tuple(vehicles),
        "formation_poses": tuple(
            (float(pose.x), float(pose.y), float(pose.theta))
            for pose in message.formation_poses),
        "times_from_start": tuple(
            _seconds(value) for value in message.times_from_start),
        "trajectories": tuple(trajectories),
    }


class FormationPlaybackNode:

    def __init__(self):
        self._lock = threading.RLock()
        self.transport_ui = bool(rospy.get_param("~transport_ui", False))
        self.contract = None
        if self.transport_ui:
            try:
                self.contract = load_contract(
                    lambda name: rospy.get_param("~" + name))
            except KeyError as exc:
                raise ValueError(
                    "missing transport contract parameter {}; the launch "
                    "file must load config/transport_formation.yaml".format(
                        exc))
        self.hold = None
        self._active_plan = None
        self._hold_reason = "START"
        self._previewing = False
        self._frontend_reports = {}
        self.frame_id = rospy.get_param("~frame_id", "map")
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise ValueError("~frame_id must be a non-empty string")
        rate = float(rospy.get_param("~rate", 25.0))
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("~rate must be a positive finite value")
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))
        self.options = PlaybackMarkerOptions(
            band_width_m=float(rospy.get_param("~band_width", 0.05)),
            history_width_m=float(rospy.get_param("~history_width", 0.015)),
            body_width_m=float(rospy.get_param("~body_width", 0.03)),
            heading_width_m=float(rospy.get_param("~heading_width", 0.02)),
            link_width_m=float(rospy.get_param("~link_width", 0.01)),
            reference_scale_m=float(
                rospy.get_param("~reference_scale", 0.14)),
            text_size_m=float(rospy.get_param("~text_size", 0.16)),
            status_text_size_m=float(
                rospy.get_param("~status_text_size", 0.20)),
            text_height_m=float(rospy.get_param("~text_height", 0.35)),
            arrow_head_length_m=float(
                rospy.get_param("~arrow_head_length", 0.08)))
        self.trajectory = None
        self.optimizer_status = None
        self.quality_report = None
        self._last_state = None
        self._tf_prefix = None
        self.marker_pub = rospy.Publisher(
            MARKER_TOPIC, MarkerArray, queue_size=1)
        self.status_pub = rospy.Publisher(
            STATUS_TOPIC, String, queue_size=1, latch=True)
        self.tf_broadcaster = (TransformBroadcaster()
                               if self.publish_tf else None)
        rospy.Subscriber(
            TRAJECTORY_TOPIC, FormationTimedTrajectory,
            self.on_trajectory, queue_size=1)
        rospy.Subscriber(
            OPTIMIZER_STATUS_TOPIC, String, self.on_optimizer_status,
            queue_size=1)
        rospy.Subscriber(
            QUALITY_TOPIC, String, self.on_quality, queue_size=1)
        if self.transport_ui:
            rospy.Subscriber("/formation/display_pose", PoseStamped,
                             self.on_preview, queue_size=1)
            rospy.Subscriber("/formation/plan_request", FormationPlanRequest,
                             self.on_request, queue_size=1)
            rospy.Subscriber("/formation/status", String,
                             self.on_frontend_status, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.tick)
        rospy.loginfo(
            "[formation playback] common-clock display ready; "
            "GEOMETRY_ONLY is shown until an independent FEASIBLE status "
            "is received")

    # ---- subscriptions -------------------------------------------------

    @synchronized
    def on_trajectory(self, message):
        if self.transport_ui and self._previewing:
            return
        if (self.transport_ui and self._active_plan is not None and
                message.plan_id < self._active_plan):
            return
        if not message.vehicles or not message.trajectories:
            self._clear("empty trajectory clears previous markers")
            return
        try:
            fields = fields_from_message(message)
            if fields["frame_id"] != self.frame_id:
                raise PlaybackValidationError(
                    "trajectory frame_id {!r} differs from ~frame_id {!r}"
                    .format(fields["frame_id"], self.frame_id))
            trajectory = trajectory_from_fields(fields)
        except (PlaybackValidationError, AttributeError, TypeError,
                ValueError) as exc:
            rospy.logerr(
                "[formation playback] rejecting trajectory: %s", exc)
            self._clear("invalid trajectory: {}".format(exc))
            return
        self.trajectory = trajectory
        self.hold = None
        self._tf_prefix = tf_frame_prefix(trajectory.formation_id)
        self._last_state = None
        rospy.loginfo(
            "[formation playback] accepted plan %s formation %s: "
            "%d vehicles, %d shared samples, T_end=%.6f s from header.stamp",
            trajectory.plan_id, trajectory.formation_id,
            len(trajectory.vehicles), trajectory.sample_count,
            trajectory.duration)
        # Publish one frame immediately so a latched trajectory that is
        # already old is still visible without waiting for the next tick.
        self._publish_tick()

    @synchronized
    def on_optimizer_status(self, message):
        report = self._parse_json(message, "optimizer_status")
        if report is None:
            return
        # A non-published status is not used to clear here: the optimizer
        # publishes an explicit empty trajectory for every transactional
        # clear, and status delivery can race the trajectory on the wire.
        # Playback therefore clears only on the trajectory itself.
        self.optimizer_status = report
        if (self.transport_ui and report.get("plan_id") == self._active_plan
                and report.get("status") in ("INVALID", "REJECTED")):
            self._hold_reason = "{}: {}".format(report.get("status"),
                                                report.get("reason"))

    @synchronized
    def on_quality(self, message):
        report = self._parse_json(message, "quality_report")
        if report is not None:
            self.quality_report = report

    @synchronized
    def on_request(self, message):
        self._previewing = False
        self._active_plan = message.plan_id
        self._hold_reason = "PLANNING"
        qf = message.formation_start
        self.hold = (message.formation_id, (qf.x, qf.y, qf.theta), tuple(
            (PlaybackVehicleSpec(v.vehicle_id, v.offset_in_formation.x,
                                 v.offset_in_formation.y, v.body_length_m,
                                 v.body_width_m),
             v.start_center.x, v.start_center.y, v.start_center.theta)
            for v in message.vehicles))
        self.trajectory = None
        self.marker_pub.publish(self._marker_array(
            clear_marker_specs(self.frame_id), rospy.Time.now()))
        self._publish_hold()

    @synchronized
    def on_preview(self, message):
        if message.header.frame_id != self.frame_id:
            return
        p, q = message.pose.position, message.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        contract = self.contract
        headings = contract.headings_for_policy(yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        vehicles = tuple((PlaybackVehicleSpec(
                    vehicle_id, a, b, contract.body_length_m,
                    contract.body_width_m),
                          p.x + c * a - s * b,
                          p.y + s * a + c * b, headings[vehicle_id])
                         for vehicle_id, (a, b) in zip(
                             contract.vehicle_ids, contract.offsets))
        self.hold = (contract.formation_id, (p.x, p.y, yaw), vehicles)
        self.trajectory = None
        self._previewing = True
        self._active_plan = None
        self._hold_reason = "START"
        self.marker_pub.publish(self._marker_array(
            clear_marker_specs(self.frame_id), rospy.Time.now()))
        self._publish_hold()

    @synchronized
    def on_frontend_status(self, message):
        report = self._parse_json(message, "frontend_status")
        if report and isinstance(report.get("plan_id"), int):
            self._frontend_reports[report["plan_id"]] = report
            while len(self._frontend_reports) > 32:
                del self._frontend_reports[min(self._frontend_reports)]

    def _publish_hold(self):
        if self.hold is None:
            return
        formation_id, qf, vehicles = self.hold
        reason = self._hold_reason
        frontend = self._frontend_reports.get(self._active_plan, {})
        if frontend.get("reason"):
            prefix = "WAITING" if frontend["reason"] == "MAP_UNAVAILABLE" else "FAILED"
            reason = "{}: {}".format(prefix, frontend["reason"])
        specs = build_hold_markers(formation_id, qf, vehicles,
                                   reason, self.frame_id)
        self.marker_pub.publish(self._marker_array(specs, rospy.Time.now()))
        self._tf_prefix = tf_frame_prefix(formation_id)
        self._broadcast_tf(SimpleNamespace(
            formation_x=qf[0], formation_y=qf[1], formation_psi=qf[2],
            vehicles=tuple(SimpleNamespace(vehicle_id=v.vehicle_id,
                center_x=x, center_y=y, theta=theta)
                for v, x, y, theta in vehicles)), rospy.Time.now())
        self.status_pub.publish(String(data=json.dumps({
            "schema": "formation_playback/v1", "state": "STATIC",
            "plan_id": self._active_plan, "formation_id": formation_id,
            "reason": reason, "executable": False,
            "trajectory_published": False, "cleared": True})))

    @staticmethod
    def _parse_json(message, label):
        try:
            data = json.loads(message.data)
        except (TypeError, ValueError) as exc:
            rospy.logwarn(
                "[formation playback] ignoring invalid %s JSON: %s",
                label, exc)
            return None
        if not isinstance(data, dict):
            rospy.logwarn(
                "[formation playback] ignoring non-object %s JSON", label)
            return None
        return data

    # ---- display -------------------------------------------------------

    @synchronized
    def tick(self, _event=None):
        if self.trajectory is None:
            if self.transport_ui:
                self._publish_hold()
            return
        self._publish_tick()

    def _status_for_trajectory(self):
        report = self.optimizer_status
        if not (isinstance(report, dict) and report.get(
                "plan_id") == self.trajectory.plan_id):
            return None
        # The optimizer status is deliberately compact; the independent
        # evaluation metrics live in the latched quality report.  Merge that
        # report's pipeline block (without changing any certification field)
        # so the RViz status text can show constraint metrics.
        quality = self.quality_report
        if (isinstance(quality, dict) and
                quality.get("plan_id") == self.trajectory.plan_id and
                isinstance(quality.get("pipeline"), dict)):
            merged = dict(report)
            merged.setdefault("pipeline", quality["pipeline"])
            return merged
        return report

    def _publish_tick(self):
        if self.trajectory is None:
            return
        trajectory = self.trajectory
        now = rospy.Time.now()
        start = rospy.Time(trajectory.stamp[0], trajectory.stamp[1])
        elapsed = (now - start).to_sec()
        try:
            frame = frame_at(trajectory, elapsed)
            status = self._status_for_trajectory()
            markers, namespaces, audit_state, executable = build_markers(
                trajectory, frame, self.options, status)
        except (PlaybackValidationError, ValueError) as exc:
            rospy.logerr(
                "[formation playback] cannot build playback frame: %s", exc)
            self._clear("playback frame failed: {}".format(exc))
            return
        center_error = heading_error = None
        if frame.terminal:
            center_error, heading_error = terminal_errors(trajectory, frame)
        document = status_document(
            trajectory, frame, status, marker_count=len(markers),
            namespaces=namespaces,
            terminal_center_error_m=center_error,
            terminal_heading_error_rad=heading_error)
        try:
            payload = json.dumps(
                document, sort_keys=True, allow_nan=False)
        except (TypeError, ValueError) as exc:
            rospy.logerr(
                "[formation playback] status JSON failed: %s", exc)
            payload = json.dumps({
                "schema": "formation_playback/v1",
                "state": INVALID,
                "cleared": True,
                "reason": "playback status was not finite",
            }, sort_keys=True)
        self.marker_pub.publish(self._marker_array(markers, now))
        self.status_pub.publish(String(data=payload))
        if self.publish_tf:
            self._broadcast_tf(frame, now)
        if frame.state != self._last_state:
            rospy.loginfo(
                "[formation playback] plan=%s state=%s elapsed=%.6f "
                "clamped=%.6f fraction=%.6f executable=%s",
                trajectory.plan_id, frame.state, frame.elapsed_s,
                frame.clamped_elapsed_s, frame.fraction, executable)
            self._last_state = frame.state

    def _broadcast_tf(self, frame, stamp):
        if self.tf_broadcaster is None or self._tf_prefix is None:
            return
        reference = TransformStamped()
        reference.header.stamp = stamp
        reference.header.frame_id = self.frame_id
        reference.child_frame_id = "{}_reference".format(self._tf_prefix)
        reference.transform.translation = Point(
            x=frame.formation_x, y=frame.formation_y, z=0.0)
        reference.transform.rotation = self._quaternion(frame.formation_psi)
        self.tf_broadcaster.sendTransform(reference)
        for state in frame.vehicles:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.frame_id
            transform.child_frame_id = "{}_vehicle_{}_base_link".format(
                self._tf_prefix, state.vehicle_id)
            transform.transform.translation = Point(
                x=state.center_x, y=state.center_y, z=0.0)
            transform.transform.rotation = self._quaternion(state.theta)
            self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _quaternion(yaw):
        values = quaternion_from_euler(0.0, 0.0, yaw)
        return Quaternion(x=values[0], y=values[1],
                          z=values[2], w=values[3])

    def _marker_array(self, specs, stamp):
        array = MarkerArray()
        for spec in specs:
            marker = Marker()
            marker.header.frame_id = spec.frame_id
            marker.header.stamp = stamp
            marker.ns = spec.namespace
            marker.id = int(spec.marker_id)
            marker.type = getattr(Marker, spec.marker_type)
            marker.action = getattr(Marker, spec.action)
            marker.pose.position = Point(*spec.position)
            marker.pose.orientation = self._quaternion(spec.yaw)
            marker.scale = Vector3(*spec.scale)
            marker.color = ColorRGBA(*spec.color)
            marker.points = [Point(x=x, y=y, z=z) for x, y, z in spec.points]
            marker.colors = [ColorRGBA(*color) for color in spec.colors]
            marker.text = "Cargo" if self.transport_ui and spec.text == "F" else spec.text
            marker.lifetime = rospy.Duration(0)
            array.markers.append(marker)
        return array

    def _clear(self, reason):
        if self.transport_ui and self.trajectory is not None and self.hold is None:
            trajectory = self.trajectory
            elapsed = (rospy.Time.now() - rospy.Time(*trajectory.stamp)).to_sec()
            frame = frame_at(trajectory, elapsed)
            specs = trajectory.vehicle_map()
            self.hold = (trajectory.formation_id,
                         (frame.formation_x, frame.formation_y, frame.formation_psi),
                         tuple((specs[v.vehicle_id], v.center_x, v.center_y, v.theta)
                               for v in frame.vehicles))
            self._hold_reason = "CLEARED"
        self.trajectory = None
        self._last_state = CLEARED
        specs = clear_marker_specs(self.frame_id)
        now = rospy.Time.now()
        self.marker_pub.publish(self._marker_array(specs, now))
        document = status_document(
            cleared=True, invalid_reason=reason, frame_id=self.frame_id)
        self.status_pub.publish(String(data=json.dumps(
            document, sort_keys=True, allow_nan=False)))
        rospy.loginfo("[formation playback] cleared: %s", reason)
        if self.transport_ui:
            self._publish_hold()


def main():
    rospy.init_node("formation_playback")
    node = FormationPlaybackNode()
    try:
        rospy.spin()
    finally:
        node.timer.shutdown()


if __name__ == "__main__":
    main()
