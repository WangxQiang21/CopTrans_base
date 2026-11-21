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
import sys
import tempfile
import threading
import time as _time
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from fwis_v3_msgs.msg import FormationPlanRequest, FormationVehicleSpec
from fwis_v3_optimizer.formation import (
    FormationModel, FormationPose, VehicleOffset)
from fwis_v3_optimizer.formation_collision import (
    FormationCollisionChecker, VehicleShape, grid_from_occupancy_grid)
from fwis_v3_optimizer.formation_playback import (
    PlaybackMarkerOptions, build_endpoint_markers, clear_marker_specs, tf_frame_prefix)
from fwis_v3_optimizer.formation_request import validate_formation_request
from fwis_v3_optimizer.formation_ros_markers import marker_array
from fwis_v3_optimizer.transport_contract import (
    TransportContractError, load_contract)
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

GOAL_TOPIC = "/move_base_simple/goal"
INITIAL_POSE_TOPIC = "/initialpose"
REQUEST_TOPIC = "/formation/plan_request"
STATUS_TOPIC = "/formation/optimizer_status"
JOB_STATE_TOPIC = "/formation/job_state"
ENDPOINTS_TOPIC = "/formation/endpoints"
PLAYBACK_PLAN_BACKEND = "uniform_v02"
VIRTUAL_BACKEND = "virtual_rigid_a"
ADMISSION_BACKENDS = ("legacy", PLAYBACK_PLAN_BACKEND, VIRTUAL_BACKEND)
PLAN_ID_LIMIT = 2 ** 32 - 1
BUSY_STATES = ("PLANNING", "PLAYING")


def quaternion_to_yaw(quaternion):
    """Return the planar yaw of a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def canonical_virtual_goal(start, goal):
    """Remove only floating-point roundoff from a repeated RViz goal."""
    rounding = 4 * sys.float_info.epsilon
    if not all(math.isfinite(value) for value in (*start, *goal)):
        return goal
    if (all(abs(a - b) <= rounding * max(1.0, abs(a), abs(b))
            for a, b in zip(start[:2], goal[:2]))
            and abs(math.remainder(goal[2] - start[2], math.tau)) <= rounding):
        return start
    return goal


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
        self.admission_backend = str(
            rospy.get_param("~admission_backend", "legacy"))
        if self.admission_backend not in ADMISSION_BACKENDS:
            raise ValueError(
                "~admission_backend must be one of {}, got {!r}".format(
                    ADMISSION_BACKENDS, self.admission_backend))
        if self.admission_backend == PLAYBACK_PLAN_BACKEND:
            self._cancel_pending_plan = rospy.ServiceProxy(
                "/formation/cancel_pending_plan", Trigger)
        case_dir = rospy.get_param("~case_collection_dir", "")
        self._case_dir = Path(case_dir) if case_dir else None
        if self._case_dir is not None:
            profile = Path(self._text_param("~case_collection_profile", ""))
            self._case_profile = json.loads(profile.read_text(encoding="utf-8"))
            self._case_dir.mkdir(parents=True, exist_ok=True)
            self._case_saved_pub = rospy.Publisher(
                "/formation/case_saved_goal", PoseStamped, queue_size=1, latch=True)
        self.virtual_a_goal_policy = None
        if self.admission_backend == VIRTUAL_BACKEND:
            policy = str(rospy.get_param(
                "~virtual_a_goal_policy", "free")).strip().lower()
            if policy not in ("free", "straight"):
                raise ValueError(
                    "~virtual_a_goal_policy must be 'free' or 'straight', "
                    "got {!r}".format(policy))
            self.virtual_a_goal_policy = policy
        self.job_state_timeout_s = float(
            rospy.get_param("~job_state_timeout_s", 3.0))
        if (not math.isfinite(self.job_state_timeout_s)
                or self.job_state_timeout_s <= 0.0):
            raise ValueError(
                "~job_state_timeout_s must be a positive finite value")
        self._job_state = None
        self._job_state_received = None
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
        self._confirmed_reset_id = None
        self._start_lock = threading.RLock()
        self._map_ready = False
        self._manual_start = False
        self._preview_pub = rospy.Publisher(
            "/formation/display_pose", PoseStamped, queue_size=1, latch=True)
        self._endpoint_margin = float(rospy.get_param("~endpoint_collision_margin", 0.05))
        self._endpoint_body_height = float(
            rospy.get_param("~body_height_m", PlaybackMarkerOptions.body_height_m))
        self._endpoint_shapes = {
            i: VehicleShape(self.body_length_m, self.body_width_m)
            for i in self.vehicle_ids}
        self._endpoint_checker = None
        self._endpoint_request = None
        self._endpoint_pub = rospy.Publisher(
            ENDPOINTS_TOPIC, MarkerArray, queue_size=1, latch=True)
        self._publish_endpoints()

        self._request_pub = rospy.Publisher(
            REQUEST_TOPIC, FormationPlanRequest, queue_size=1, latch=True)
        rospy.Subscriber("/map", OccupancyGrid, self.on_endpoint_map, queue_size=1)
        rospy.Subscriber(GOAL_TOPIC, PoseStamped, self.on_goal, queue_size=1)
        rospy.Subscriber(
            INITIAL_POSE_TOPIC, PoseWithCovarianceStamped,
            self.on_initial_pose, queue_size=1)
        rospy.Subscriber(
            STATUS_TOPIC, String, self.on_optimizer_status, queue_size=10)
        rospy.Subscriber(
            JOB_STATE_TOPIC, String, self.on_job_state, queue_size=10)
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
        if self.admission_backend in (
                PLAYBACK_PLAN_BACKEND, VIRTUAL_BACKEND):
            try:
                self.on_job_state(rospy.wait_for_message(
                    JOB_STATE_TOPIC, String, timeout=3.0))
            except rospy.ROSException:
                rospy.logwarn(
                    "[formation goal bridge] no %s within 3.0 s; goals and "
                    "initial poses are refused until the uniform planner "
                    "publishes a fresh job state", JOB_STATE_TOPIC)

        rospy.loginfo(
            "[formation goal bridge] ready: formation=%s offsets=%s, "
            "start=(%.3f, %.3f, %.3f), goal topic=%s",
            self.formation_id, self.offsets, self.start_x, self.start_y,
            self.start_theta, GOAL_TOPIC)
        if self._case_dir is not None:
            rospy.loginfo("[case collection] ready. Set a start and goal in RViz")

    # ---- parameters ---------------------------------------------------

    @staticmethod
    def _text_param(name, default):
        value = rospy.get_param(name, default)
        if not isinstance(value, str) or not value.strip():
            raise ValueError("{} must be a non-empty string".format(name))
        return value.strip()

    # ---- uniform admission --------------------------------------------

    def on_job_state(self, message):
        try:
            document = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(document, dict):
            return
        state = document.get("state")
        if state not in ("IDLE", "PLANNING", "PLAYING", "HOLD_END", "FAILED"):
            return
        with self._start_lock:
            if (self.admission_backend == VIRTUAL_BACKEND
                    and document.get("backend") == VIRTUAL_BACKEND
                    and state not in BUSY_STATES
                    and document.get("initial_reset_ready") is True):
                context = document.get("initial_context")
                if isinstance(context, dict) and context.get("source") == "rviz_reset":
                    identity = context.get("context_id")
                    pose = context.get("qf")
                    if (isinstance(identity, str) and identity
                            and identity != self._confirmed_reset_id
                            and isinstance(pose, dict)):
                        start = tuple(pose.get(key) for key in ("x", "y", "psi"))
                        if all(type(value) in (int, float) and math.isfinite(value)
                               for value in start):
                            self._confirmed_reset_id = identity
                            self._record_manual_start(start)
            self._job_state = document
            self._job_state_received = _time.monotonic()

    def _admission_busy(self):
        """Return True/False for a fresh uniform job, None when unavailable.

        Only the V0.2 backend owns admission.  The legacy backend keeps the
        historical behaviour of the bridge (no job state exists).
        """
        if self.admission_backend not in (
                PLAYBACK_PLAN_BACKEND, VIRTUAL_BACKEND):
            return False
        if self._job_state is None or self._job_state_received is None:
            return None
        if (_time.monotonic() - self._job_state_received >
                self.job_state_timeout_s):
            return None
        return self._job_state.get("state") in BUSY_STATES

    def _refuse_when_busy(self, action):
        busy = self._admission_busy()
        if busy is None:
            rospy.logerr(
                "[formation goal bridge] refusing %s: no fresh "
                "/formation/job_state from the uniform planner; the start "
                "override is not consumed", action)
            return True
        if busy:
            rospy.logwarn(
                "[formation goal bridge] refusing %s: job %s is %s; the "
                "start override and plan_id are not consumed and no request "
                "is published", action,
                self._job_state.get("plan_id"),
                self._job_state.get("state"))
            return True
        return False

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
        if self._case_dir is not None and not self._manual_start:
            raise TransportContractError("case collection requires 2D Pose Estimate first")
        if self._start_override is not None:
            start = self._start_override
            return (start,
                    self.contract.headings_for_policy(start[2]),
                    "manual start", self._case_dir is None)
        job = self._job_state or {}
        if (self.admission_backend == PLAYBACK_PLAN_BACKEND
                and job.get("state") == "HOLD_END"):
            terminal = job.get("uniform_terminal")
            if (job.get("backend") != PLAYBACK_PLAN_BACKEND
                    or job.get("formation_id") != self.formation_id or not terminal
                    or not job.get("plan_id") or not job.get("model_sha256")):
                raise TransportContractError("no confirmed uniform terminal state; set a new start pose")
            try:
                start = tuple(float(value) for value in terminal["qf"])
                headings = {i: float(terminal["headings"][str(i)]) for i in self.vehicle_ids}
            except (KeyError, TypeError, ValueError) as exc:
                raise TransportContractError("invalid confirmed uniform terminal state") from exc
            if len(start) != 3 or not all(math.isfinite(value) for value in (*start, *headings.values())):
                raise TransportContractError("invalid confirmed uniform terminal pose or headings")
            return start, headings, "confirmed terminal", False
        snapshot = self._playback_snapshot()
        if snapshot is not None:
            return snapshot[0], snapshot[1], "current playback", False
        start = self._fallback_start
        return (start,
                self.contract.headings_for_policy(start[2]),
                "configured fallback", False)

    def on_initial_pose(self, message):
        if self.admission_backend == VIRTUAL_BACKEND:
            # The planner confirms or rejects this command in job_state.
            return
        if message.header.frame_id not in ("", self.frame_id):
            rospy.logwarn(
                "[formation goal bridge] ignoring initial pose frame %r",
                message.header.frame_id)
            return
        pose = message.pose.pose
        start = (
            float(pose.position.x), float(pose.position.y),
            quaternion_to_yaw(pose.orientation))
        if not all(math.isfinite(value) for value in start):
            rospy.logwarn("[formation goal bridge] ignoring nonfinite initial pose")
            return
        with self._start_lock:
            job = self._job_state or {}
            if (self.admission_backend == PLAYBACK_PLAN_BACKEND and self._admission_busy()
                    and job.get("state") == "PLANNING"):
                try:
                    response = self._cancel_pending_plan()
                except rospy.ServiceException as exc:
                    rospy.logwarn("[formation goal bridge] reset service unavailable: %s", exc)
                    return
                if not response.success:
                    rospy.logwarn("[formation goal bridge] refusing reset: %s", response.message)
                    return
                self._job_state = dict(job, state="IDLE")
                self._job_state_received = _time.monotonic()
            elif self._refuse_when_busy("2D Pose Estimate"):
                return
            self._record_manual_start(start)

    def _record_manual_start(self, start):
        self._start_override = start
        self._manual_start = True
        self._fallback_start = self._start_override
        # A manual reset starts a new experiment.  Statuses for the previous
        # plan must not be mistaken for this experiment, and the preview
        # below clears any old executable trajectory from the display.
        self._current_plan = None
        self._endpoint_request = None
        self._publish_endpoints()
        self._publish_preview(self._start_override)
        rospy.loginfo(
            "[formation goal bridge] 2D Pose Estimate recorded; new "
            "experiment policy=%s headings=%s",
            self.contract.initial_heading_policy,
            self.contract.headings_for_policy(self._start_override[2]))

    def on_map_start(self, message):
        if message.header.frame_id not in ("", self.frame_id):
            return
        pose = message.pose
        start = (float(pose.position.x), float(pose.position.y),
                 quaternion_to_yaw(pose.orientation))
        with self._start_lock:
            if self._manual_start or self._current_plan is not None:
                return
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

    def on_endpoint_map(self, message):
        grid, _ = grid_from_occupancy_grid(message)
        checker = (FormationCollisionChecker(
            grid, self._model, self._endpoint_shapes, self._endpoint_margin)
            if grid is not None else None)
        with self._start_lock:
            self._endpoint_checker = checker
            if self._endpoint_request is not None:
                self._publish_endpoints()

    def _publish_endpoints(self):
        specs = clear_marker_specs(self.frame_id)
        if self._endpoint_request is not None:
            try:
                specs = build_endpoint_markers(
                    self._endpoint_request, self._endpoint_checker, self.frame_id,
                    body_height_m=self._endpoint_body_height)
            except (ValueError, OverflowError) as exc:
                rospy.logwarn("[formation goal bridge] endpoint display failed: %s", exc)
        self._endpoint_pub.publish(marker_array(specs, rospy.Time.now()))

    def on_goal(self, message):
        if message.header.frame_id not in ("", self.frame_id):
            rospy.logwarn(
                "[formation goal bridge] ignoring goal frame %r",
                message.header.frame_id)
            return
        if self._refuse_when_busy("2D Nav Goal"):
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
            with self._start_lock:
                start, body_headings, source, consume_override = self._start_pose()
                continuation = ((self._job_state["plan_id"], self._job_state["model_sha256"])
                                if source == "confirmed terminal" else None)
        except (TransportContractError, ValueError) as exc:
            rospy.logerr(
                "[formation goal bridge] refusing goal: %s", exc)
            return
        goal = (goal_x, goal_y, goal_theta)
        if self.admission_backend == VIRTUAL_BACKEND:
            goal = canonical_virtual_goal(start, goal)
        if not self._publish_request(start, goal, body_headings, continuation=continuation):
            return
        if consume_override:
            with self._start_lock:
                if self._start_override is start:
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

    def _publish_request(self, start, goal, body_headings, *, continuation=None):
        try:
            start_centers = self._model.centers(FormationPose(*start))
            goal_centers = self._model.centers(FormationPose(*goal))
        except Exception as exc:
            rospy.logerr(
                "[formation goal bridge] cannot build rigid centers: %s", exc)
            return False

        goal_heading_free = bool(self.goal_heading_free)
        if self.virtual_a_goal_policy is not None:
            # FREE and STRAIGHT are explicit launch choices.  The virtual
            # adapter still checks every flag and fixed heading; the bridge
            # only expresses the selected policy in the unchanged message.
            goal_heading_free = self.virtual_a_goal_policy == "free"
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
        if continuation is not None:
            request.continuation_plan_id, request.continuation_model_sha256 = continuation
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
            # The arrow sets the nominal body heading for display. STRAIGHT
            # also uses it as the fixed terminal body heading.
            spec.goal_center.theta = float(goal[2])
            spec.goal_heading_free = goal_heading_free
            spec.body_length_m = self.body_length_m
            spec.body_width_m = self.body_width_m
            request.vehicles.append(spec)

        if self._case_dir is not None:
            if not self._save_case(request):
                return False
        else:
            self._current_plan = plan_id
            self._last_goal = goal
            self._request_pub.publish(request)
            rospy.loginfo(
                "[formation goal bridge] plan %d: start=(%.3f, %.3f, %.3f) "
                "goal=(%.3f, %.3f, %.3f) vehicles=%d",
                plan_id, start[0], start[1], start[2],
                goal[0], goal[1], goal[2], len(self.vehicle_ids))
            if self.virtual_a_goal_policy is not None:
                rospy.loginfo(
                    "[formation goal bridge] plan %d virtual_a_goal_policy=%s "
                    "goal_headings=%s", plan_id, self.virtual_a_goal_policy,
                    {vehicle_id: round(goal[2], 6)
                     for vehicle_id in self.vehicle_ids})
        with self._start_lock:
            if self._manual_start:
                self._endpoint_request = validate_formation_request(request).snapshot
                self._publish_endpoints()
        return True

    def _save_case(self, request):
        with self._start_lock:
            job = self._job_state or {}
            signature = job.get("map_signature")
        if not job.get("map_ready") or not signature or job.get("backend") != self.admission_backend:
            rospy.logwarn("[case collection] planner map is not ready. Send the goal again")
            return False

        created = datetime.now(timezone.utc)
        map_name = Path(self._case_profile["map"]["directory"]).name
        case_id = "{}_{}_{}".format(
            map_name.split("-")[0], created.strftime("%Y%m%dT%H%M%S%fZ"),
            uuid4().hex[:8])
        path = self._case_dir / (case_id + ".json")
        fields = yaml.safe_load(str(request))
        del fields["plan_id"], fields["header"]["seq"], fields["header"]["stamp"]
        document = {
            **self._case_profile,
            "case_id": case_id,
            "name": case_id,
            "notes": "",
            "captured_at_utc": created.isoformat(),
            "map": {**self._case_profile["map"], "grid_sha256": signature},
            "request": fields,
            "expect": {"outcome": "success"},
        }
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                    mode="w", encoding="utf-8", suffix=".tmp",
                    dir=self._case_dir, delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(document, stream, indent=2, ensure_ascii=False, allow_nan=False)
                stream.write("\n")
            temporary.replace(path)
        except (OSError, ValueError) as exc:
            if temporary is not None and temporary.exists():
                temporary.unlink()
            rospy.logerr("[case collection] could not save %s: %s", path, exc)
            return False
        rospy.loginfo("[case collection] saved %s", path)
        goal = request.formation_goal
        marker = PoseStamped()
        marker.header = request.header
        marker.pose.position.x = goal.x
        marker.pose.position.y = goal.y
        marker.pose.position.z = 0.07
        marker.pose.orientation.z = math.sin(goal.theta / 2)
        marker.pose.orientation.w = math.cos(goal.theta / 2)
        self._case_saved_pub.publish(marker)
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
