#!/usr/bin/env python3
"""Turn an RViz 2D goal into a rigid-formation planning request.

The bridge is deliberately a thin request adapter.  It does not plan, retime,
evaluate or publish trajectories.  RViz's ``2D Nav Goal`` tool publishes the
formation-center goal on ``/move_base_simple/goal``; this node converts it to
a ``FormationPlanRequest`` on ``/formation/plan_request`` with the same fixed
offsets and body dimensions for every click.  The C++ front end, S9 evaluator
and transactional publication remain unchanged.

The formation start is, in priority order:

1. the most recent ``2D Pose Estimate`` on ``/initialpose`` (one-shot override),
2. the current playback TF frame ``formation_<id>_reference`` if it exists,
3. the configured start pose (defaults to the clear corridor used by the
   real-map demonstration).

Vehicle body headings are also read from playback TF when available, so a
sequence of goals continues from the current independently-steered 4WIS
configuration instead of silently resetting body headings.
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
        self.formation_id = self._text_param("~formation_id", "rviz_goal")
        self.start_x = float(rospy.get_param("~start_x", 10.0))
        self.start_y = float(rospy.get_param("~start_y", 3.6))
        self.start_theta = float(rospy.get_param("~start_theta", 0.0))
        self.body_length_m = float(
            rospy.get_param("~body_length_m", 0.738))
        self.body_width_m = float(rospy.get_param("~body_width_m", 0.500))
        self.allow_reverse = self._bool_param("~allow_reverse", True)
        self.goal_heading_free = self._bool_param(
            "~goal_heading_free", False)
        self.map_timeout_s = float(
            rospy.get_param("~map_timeout_s", 8.0))
        self.initial_plan_id = int(
            rospy.get_param("~initial_plan_id", 600000))
        self.vehicle_ids = self._vehicle_ids_param()
        self.offsets = self._offsets_param()

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

        self._request_pub = rospy.Publisher(
            REQUEST_TOPIC, FormationPlanRequest, queue_size=1, latch=True)
        rospy.Subscriber(GOAL_TOPIC, PoseStamped, self.on_goal, queue_size=1)
        rospy.Subscriber(
            INITIAL_POSE_TOPIC, PoseWithCovarianceStamped,
            self.on_initial_pose, queue_size=1)
        rospy.Subscriber(
            STATUS_TOPIC, String, self.on_optimizer_status, queue_size=10)

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

    @staticmethod
    def _bool_param(name, default):
        value = rospy.get_param(name, default)
        if not isinstance(value, bool):
            raise ValueError("{} must be a bool".format(name))
        return value

    def _vehicle_ids_param(self):
        raw = rospy.get_param("~vehicle_ids", [1, 2, 3])
        if not isinstance(raw, list) or not raw:
            raise ValueError("~vehicle_ids must be a non-empty list")
        result = []
        for value in raw:
            vehicle_id = int(value)
            if vehicle_id < 0 or vehicle_id in result:
                raise ValueError(
                    "~vehicle_ids must contain unique non-negative ids")
            result.append(vehicle_id)
        return tuple(result)

    def _offsets_param(self):
        raw = rospy.get_param(
            "~offsets", [[0.0, 0.0], [0.0, 1.2], [0.0, -1.2]])
        if not isinstance(raw, list) or len(raw) != len(self.vehicle_ids):
            raise ValueError(
                "~offsets must contain one [x, y] pair per vehicle")
        result = []
        for pair in raw:
            if (not isinstance(pair, list) or len(pair) != 2):
                raise ValueError("each ~offsets entry must be [x, y]")
            result.append((float(pair[0]), float(pair[1])))
        return tuple(result)

    # ---- TF / goal input ---------------------------------------------

    def _latest_reference(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self.frame_id, self._tf_prefix + "_reference",
                rospy.Time(0), rospy.Duration(0.05))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        translation = transform.transform.translation
        yaw = quaternion_to_yaw(transform.transform.rotation)
        return (float(translation.x), float(translation.y), yaw)

    def _latest_body_headings(self):
        headings = {}
        for vehicle_id in self.vehicle_ids:
            frame = "{}_vehicle_{}_base_link".format(
                self._tf_prefix, vehicle_id)
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.frame_id, frame, rospy.Time(0),
                    rospy.Duration(0.05))
            except (LookupException, ConnectivityException,
                    ExtrapolationException):
                continue
            headings[vehicle_id] = quaternion_to_yaw(
                transform.transform.rotation)
        return headings

    def _start_pose(self):
        if self._start_override is not None:
            start = self._start_override
            self._start_override = None
            rospy.loginfo(
                "[formation goal bridge] using 2D Pose Estimate start "
                "(%.3f, %.3f, %.3f)", *start)
            return start, {}
        current = self._latest_reference()
        if current is not None:
            return current, self._latest_body_headings()
        return self._fallback_start, {}

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
        rospy.loginfo(
            "[formation goal bridge] 2D Pose Estimate recorded; it will be "
            "used by the next goal")

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
        start, body_headings = self._start_pose()
        self._publish_request(start, (goal_x, goal_y, goal_theta),
                              body_headings)

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
            return

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
            spec.start_center.theta = float(
                body_headings.get(vehicle_id, start[2]))
            spec.goal_center.x = float(goal_center[0])
            spec.goal_center.y = float(goal_center[1])
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

    def on_optimizer_status(self, message):
        try:
            document = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if document.get("plan_id") != self._current_plan:
            return
        status = document.get("status")
        if status not in ("FEASIBLE", "INVALID", "REJECTED"):
            return
        if status == "FEASIBLE" and document.get("trajectory_published"):
            if self._last_goal is not None:
                self._fallback_start = self._last_goal
            rospy.loginfo(
                "[formation goal bridge] plan %d FEASIBLE and published",
                self._current_plan)
        else:
            rospy.logwarn(
                "[formation goal bridge] plan %d status=%s reason=%s",
                self._current_plan, status, document.get("reason"))


def main():
    bridge = FormationGoalBridge()
    rospy.spin()


if __name__ == "__main__":
    main()