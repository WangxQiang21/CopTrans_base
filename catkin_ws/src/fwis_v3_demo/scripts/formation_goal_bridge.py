#!/usr/bin/env python3
"""Turn an RViz 2D goal into a rigid-formation planning request.

The bridge is deliberately a thin request adapter.  It does not plan, retime,
evaluate or publish trajectories.  RViz's ``2D Nav Goal`` tool publishes the
cargo goal on ``/move_base_simple/goal``; this node converts it to a
``FormationPlanRequest`` on ``/formation/plan_request`` using the one
authoritative transport contract (ids, offsets, body dimensions, heading
policies) loaded from ``config/transport_formation.yaml``.

Body headings are independent.  A new experiment -- a manual
``2D Pose Estimate`` or the configured start fallback -- initialises
``theta_i(0)`` from the contract's ``initial_heading_policy``.  Later goals
never reset headings: the start pose and all three body headings are read
from one synchronized, sufficiently fresh playback TF snapshot.  Publication
of a trajectory is not treated as arrival; the live playback pose is.
"""
import json
import math

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from fwis_v3_msgs.msg import FormationPlanRequest, FormationVehicleSpec
from fwis_v3_optimizer.formation import (
    FormationModel, FormationPose, VehicleOffset)
from fwis_v3_optimizer.formation_playback import tf_frame_prefix
from fwis_v3_optimizer.transport_contract import (
    TransportContractError, load_contract)
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

GOAL_TOPIC = "/move_base_simple/goal"
INITIAL_POSE_TOPIC = "/initialpose"
REQUEST_TOPIC = "/formation/plan_request"
STATUS_TOPIC = "/formation/optimizer_status"
PLAN_ID_LIMIT = 2 ** 32 - 1


def quaternion_to_yaw(quaternion):
    """Return the planar yaw of a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FormationGoalBridge(object):
    def __init__(self):
        rospy.init_node("formation_goal_bridge")
        self.frame_id = self._text_param("~frame_id", "map")
        try:
            self.contract = load_contract(
                lambda name: rospy.get_param("~" + name))
        except KeyError as exc:
            raise ValueError(
                "missing transport contract parameter {}; the launch file "
                "must load config/transport_formation.yaml".format(exc))
        self.formation_id = self.contract.formation_id
        self.start_x = float(rospy.get_param("~start_x", 10.0))
        self.start_y = float(rospy.get_param("~start_y", 3.6))
        self.start_theta = float(rospy.get_param("~start_theta", 0.0))
        self.body_length_m = self.contract.body_length_m
        self.body_width_m = self.contract.body_width_m
        self.allow_reverse = self.contract.allow_reverse
        self.goal_heading_free = self.contract.goal_heading_free
        self.map_timeout_s = float(
            rospy.get_param("~map_timeout_s", 8.0))
        self.playback_tf_timeout_s = float(
            rospy.get_param("~playback_tf_timeout_s", 0.5))
        if (not math.isfinite(self.playback_tf_timeout_s)
                or self.playback_tf_timeout_s <= 0.0):
            raise ValueError(
                "~playback_tf_timeout_s must be a positive finite value")
        self.rigid_start_tolerance_m = float(
            rospy.get_param("~rigid_start_tolerance_m", 1e-3))
        self.initial_plan_id = int(
            rospy.get_param("~initial_plan_id", 600000))
        self.vehicle_ids = self.contract.vehicle_ids
        self.offsets = self.contract.offsets

        self._offset_by_id = dict(zip(self.vehicle_ids, self.offsets))
        self._model = FormationModel([
            VehicleOffset(vehicle_id, offset[0], offset[1])
            for vehicle_id, offset in zip(self.vehicle_ids, self.offsets)])
        self._tf_prefix = tf_frame_prefix(self.formation_id)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._plan_id = max(1, self.initial_plan_id)
        self._current_plan = None
        self._last_goal = None
        self._fallback_start = (self.start_x, self.start_y,
                                self.start_theta)
        self._start_override = None
        self._map_ready = False
        self._manual_start = False
        self._preview_pub = rospy.Publisher(
            "/formation/display_pose", PoseStamped, queue_size=1, latch=True)

        self._request_pub = rospy.Publisher(
            REQUEST_TOPIC, FormationPlanRequest, queue_size=1, latch=True)
        rospy.Subscriber(GOAL_TOPIC, PoseStamped, self.on_goal, queue_size=1)
        rospy.Subscriber(
            INITIAL_POSE_TOPIC, PoseWithCovarianceStamped,
            self.on_initial_pose, queue_size=1)
        rospy.Subscriber(
            STATUS_TOPIC, String, self.on_optimizer_status, queue_size=10)
        rospy.Subscriber("/start_pose", PoseStamped, self.on_map_start,
                         queue_size=1)

        try:
            rospy.wait_for_message(
                "/map", OccupancyGrid, timeout=self.map_timeout_s)
            self._map_ready = True
        except rospy.ROSException:
            rospy.logwarn(
                "[formation goal bridge] /map not received within %.1f s; "
                "the next goal will retry", self.map_timeout_s)

        rospy.loginfo(
            "[formation goal bridge] ready: formation=%s offsets=%s, "
            "start=(%.3f, %.3f, %.3f), goal topic=%s",
            self.formation_id, self.offsets, self.start_x, self.start_y,
            self.start_theta, GOAL_TOPIC)

    # ---- parameters ---------------------------------------------------

    @staticmethod
    def _text_param(name, default):
        value = rospy.get_param(name, default)
        if not isinstance(value, str) or not value.strip():
            raise ValueError("{} must be a non-empty string".format(name))
        return value.strip()

    # ---- TF / goal input ---------------------------------------------

    def _lookup(self, frame, stamp):
        try:
            return self._tf_buffer.lookup_transform(
                self.frame_id, frame, stamp, rospy.Duration(0.05))
        except (LookupException, ConnectivityException,
                ExtrapolationException):
            return None

    def _playback_snapshot(self):
        """One synchronized, fresh cargo + three-body playback state.

        The reference lookup chooses the display time; every body transform
        is then looked up at that same stamp, so the start pose and all three
        headings come from one playback instant rather than four separate
        ``Time(0)`` queries.  A stale snapshot is unused; a snapshot whose
        centres are not rigid with the configured offsets is rejected.
        """
        reference = self._lookup(
            self._tf_prefix + "_reference", rospy.Time(0))
        if reference is None:
            return None
        stamp = reference.header.stamp
        age = (rospy.Time.now() - stamp).to_sec()
        if (not math.isfinite(age)
                or age > self.playback_tf_timeout_s or age < -0.05):
            return None
        translation = reference.transform.translation
        heading = quaternion_to_yaw(reference.transform.rotation)
        start = (float(translation.x), float(translation.y), heading)
        expected_centres = self._model.centers(FormationPose(*start))
        headings = {}
        for vehicle_id in self.vehicle_ids:
            frame = "{}_vehicle_{}_base_link".format(
                self._tf_prefix, vehicle_id)
            transform = self._lookup(frame, stamp)
            if transform is None:
                return None
            body = transform.transform
            expected = expected_centres[vehicle_id]
            error = math.hypot(
                body.translation.x - expected[0],
                body.translation.y - expected[1])
            if error > self.rigid_start_tolerance_m:
                raise TransportContractError(
                    "playback TF for vehicle_id={} is not rigid with "
                    "formation {} (centre error {:.6g} m); refusing to start "
                    "from an inconsistent display".format(
                        vehicle_id, self.formation_id, error))
            headings[vehicle_id] = quaternion_to_yaw(body.rotation)
        return start, headings

    def _start_pose(self):
        """Return ``(start, body_headings, source, consume_override)``."""
        if self._start_override is not None:
            start = self._start_override
            return (start,
                    self.contract.headings_for_policy(start[2]),
                    "manual start", True)
        snapshot = self._playback_snapshot()
        if snapshot is not None:
            return snapshot[0], snapshot[1], "current playback", False
        start = self._fallback_start
        return (start,
                self.contract.headings_for_policy(start[2]),
                "configured fallback", False)

    def on_initial_pose(self, message):
        if message.header.frame_id not in ("", self.frame_id):
            rospy.logwarn(
                "[formation goal bridge] ignoring initial pose frame %r",
                message.header.frame_id)
            return
        pose = message.pose.pose
        self._start_override = (
            float(pose.position.x), float(pose.position.y),
            quaternion_to_yaw(pose.orientation))
        self._manual_start = True
        self._fallback_start = self._start_override
        # A manual reset starts a new experiment.  Statuses for the previous
        # plan must not be mistaken for this experiment, and the preview
        # below clears any old executable trajectory from the display.
        self._current_plan = None
        self._publish_preview(self._start_override)
        rospy.loginfo(
            "[formation goal bridge] 2D Pose Estimate recorded; new "
            "experiment policy=%s headings=%s",
            self.contract.initial_heading_policy,
            self.contract.headings_for_policy(self._start_override[2]))

    def on_map_start(self, message):
        if self._manual_start or self._current_plan is not None:
            return
        if message.header.frame_id not in ("", self.frame_id):
            return
        pose = message.pose
        start = (float(pose.position.x), float(pose.position.y),
                 quaternion_to_yaw(pose.orientation))
        self._fallback_start = start
        self._start_override = start
        self._publish_preview(start)

    def _publish_preview(self, start):
        message = PoseStamped()
        message.header.frame_id = self.frame_id
        message.header.stamp = rospy.Time.now()
        message.pose.position.x, message.pose.position.y = start[:2]
        message.pose.orientation.z = math.sin(start[2] / 2)
        message.pose.orientation.w = math.cos(start[2] / 2)
        self._preview_pub.publish(message)

    def on_goal(self, message):
        if message.header.frame_id not in ("", self.frame_id):
            rospy.logwarn(
                "[formation goal bridge] ignoring goal frame %r",
                message.header.frame_id)
            return
        if not self._map_ready:
            try:
                rospy.wait_for_message("/map", OccupancyGrid, timeout=2.0)
                self._map_ready = True
            except rospy.ROSException:
                rospy.logerr(
                    "[formation goal bridge] no /map; goal ignored")
                return

        goal_x = float(message.pose.position.x)
        goal_y = float(message.pose.position.y)
        goal_theta = quaternion_to_yaw(message.pose.orientation)
        try:
            start, body_headings, source, consume_override = self._start_pose()
        except (TransportContractError, ValueError) as exc:
            rospy.logerr(
                "[formation goal bridge] refusing goal: %s", exc)
            return
        if not self._publish_request(
                start, (goal_x, goal_y, goal_theta), body_headings):
            return
        if consume_override:
            self._start_override = None
        rospy.loginfo(
            "[formation goal bridge] start from %s; body headings %s",
            source, {vehicle_id: round(body_headings[vehicle_id], 6)
                     for vehicle_id in self.vehicle_ids})

    # ---- request construction ----------------------------------------

    def _next_plan_id(self):
        plan_id = self._plan_id
        self._plan_id = 1 if plan_id >= PLAN_ID_LIMIT else plan_id + 1
        return plan_id

    def _publish_request(self, start, goal, body_headings):
        try:
            start_centers = self._model.centers(FormationPose(*start))
            goal_centers = self._model.centers(FormationPose(*goal))
        except Exception as exc:
            rospy.logerr(
                "[formation goal bridge] cannot build rigid centers: %s", exc)
            return False

        plan_id = self._next_plan_id()
        request = FormationPlanRequest()
        request.header.frame_id = self.frame_id
        request.header.stamp = rospy.Time.now()
        request.plan_id = plan_id
        request.formation_id = self.formation_id
        request.formation_start.x = float(start[0])
        request.formation_start.y = float(start[1])
        request.formation_start.theta = float(start[2])
        request.formation_goal.x = float(goal[0])
        request.formation_goal.y = float(goal[1])
        request.formation_goal.theta = float(goal[2])
        request.allow_reverse = self.allow_reverse
        for vehicle_id in self.vehicle_ids:
            spec = FormationVehicleSpec()
            spec.vehicle_id = vehicle_id
            offset = self._offset_by_id[vehicle_id]
            spec.offset_in_formation.x = offset[0]
            spec.offset_in_formation.y = offset[1]
            spec.offset_in_formation.z = 0.0
            start_center = start_centers[vehicle_id]
            goal_center = goal_centers[vehicle_id]
            spec.start_center.x = float(start_center[0])
            spec.start_center.y = float(start_center[1])
            spec.start_center.theta = float(body_headings[vehicle_id])
            spec.goal_center.x = float(goal_center[0])
            spec.goal_center.y = float(goal_center[1])
            # Placeholder only when goal_heading_free is true; the backend
            # still checks cargo psi_F and the rigid goal centres.
            spec.goal_center.theta = float(goal[2])
            spec.goal_heading_free = self.goal_heading_free
            spec.body_length_m = self.body_length_m
            spec.body_width_m = self.body_width_m
            request.vehicles.append(spec)

        self._current_plan = plan_id
        self._last_goal = goal
        self._request_pub.publish(request)
        rospy.loginfo(
            "[formation goal bridge] plan %d: start=(%.3f, %.3f, %.3f) "
            "goal=(%.3f, %.3f, %.3f) vehicles=%d",
            plan_id, start[0], start[1], start[2],
            goal[0], goal[1], goal[2], len(self.vehicle_ids))
        return True

    def on_optimizer_status(self, message):
        try:
            document = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if document.get("plan_id") != self._current_plan:
            return
        status = document.get("status")
        if status not in ("FEASIBLE", "NO_MOTION", "INVALID", "REJECTED"):
            return
        if (status in ("FEASIBLE", "NO_MOTION") and
                document.get("trajectory_published")):
            # Publication is not arrival. Playback TF supplies the actual
            # current reference pose for subsequent goals.  A NO_MOTION hold
            # is a successful static publication, not a failed plan.
            rospy.loginfo(
                "[formation goal bridge] plan %d %s and published",
                self._current_plan, status)
        else:
            rospy.logwarn(
                "[formation goal bridge] plan %d status=%s reason=%s",
                self._current_plan, status, document.get("reason"))


def main():
    bridge = FormationGoalBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
