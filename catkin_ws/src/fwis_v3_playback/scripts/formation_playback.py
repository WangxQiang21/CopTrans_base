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
from collections.abc import Mapping
from functools import wraps
from types import SimpleNamespace
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, Quaternion, TransformStamped, PoseStamped
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from fwis_v3_msgs.msg import (
    FormationPlaybackPlan, FormationTimedTrajectory, FormationPlanRequest)
from fwis_v3_optimizer.formation_ros_markers import marker_array

from fwis_v3_optimizer.formation_playback import (
    CLEARED,
    INVALID,
    PLAYBACK_PLAN_BACKEND,
    PLAYBACK_PLAN_SCHEMA,
    PLANNING_FAILED_STATE,
    VIRTUAL_CERTIFICATION,
    VIRTUAL_PLAYBACK_BACKEND,
    VIRTUAL_PLAYBACK_SEMANTIC_VERSION,
    VIRTUAL_MODEL_VERSION,
    PlaybackMarkerOptions,
    PlaybackValidationError,
    build_candidate_display,
    build_corridor_markers,
    build_uniform_sweep_markers,
    build_guide_display,
    build_markers,
    clear_marker_specs,
    corridor_display_matches,
    fields_from_timed_message,
    frame_at,
    hud_text,
    planning_diagnostics_view,
    quality_identity_matches,
    status_document,
    terminal_errors,
    tf_frame_prefix,
    trajectory_from_fields,
    verify_playback_plan,
    verify_uniform_playback_plan,
    virtual_a_request_from_raw_request,
    PlaybackVehicleSpec,
    build_hold_markers,
    vehicle_path_samples,
)
try:
    from jsk_rviz_plugins.msg import OverlayText
except ImportError as _hud_import_error:
    OverlayText = None
else:
    _hud_import_error = None
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    INITIAL_HEADING_TOLERANCE_RAD,
    geometry_audit_config_from_dict,
    request_fingerprint,
)
from fwis_v3_optimizer.transport_contract import load_contract


def _reject_json_constant(value):
    raise ValueError("non-finite JSON constant {!r}".format(value))


MARKER_TOPIC = "/formation/markers"
VEHICLE_PATH_TOPIC = "/formation/path/vehicle_{}"
F_CANDIDATE_PATH_TOPIC = "/formation/path/f_candidate"
CANDIDATE_VEHICLE_PATH_TOPIC = "/formation/path/candidate_vehicle_{}"
ASTAR_PATH_TOPIC = "/formation/path/astar_raw_{}"
GUIDE_PATH_TOPIC = "/formation/path/portal_guide_{}"
ASTAR_ACTIVE_PATH_TOPIC = "/formation/path/astar_active"
GUIDE_ACTIVE_PATH_TOPIC = "/formation/path/portal_guide_active"
HUD_TOPIC = "/formation/hud"
CORRIDOR_TOPIC = "/formation/corridors"
GUIDES_TOPIC = "/formation/guides"
CANDIDATES_TOPIC = "/formation/planning_candidates"
PLANNING_DEBUG_TOPIC = "/formation/planning_debug"
STATUS_TOPIC = "/formation/playback_status"
TRAJECTORY_TOPIC = "/formation/timed_trajectory"
PLAYBACK_PLAN_TOPIC = "/formation/playback_plan"
JOB_STATE_TOPIC = "/formation/job_state"
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


def active_route_index(document):
    """The real selected route index, or None when the document cannot say.

    Both fields come from the verified planning-debug document written by the
    pipeline.  There is deliberately no ``route 0`` default: an absent or
    inconsistent pair must clear the active aliases instead of presenting a
    guess as the selected route.
    """
    if not isinstance(document, dict):
        return None
    route_index = document.get("route_index")
    route_count = document.get("route_count")
    if isinstance(route_index, bool) or not isinstance(route_index, int):
        return None
    if isinstance(route_count, bool) or not isinstance(route_count, int):
        return None
    if route_count < 1 or route_index < 0 or route_index >= route_count:
        return None
    return route_index


def fields_from_message(message):
    """Convert the ROS message into plain display fields, failing closed."""
    return fields_from_timed_message(message)


class FormationPlaybackNode:

    # Defaults used by the ROS-independent test harness, which constructs the
    # instance with __new__.  __init__ overwrites every mutable display field.
    show_cargo_reason = True
    options = PlaybackMarkerOptions()
    hud_pub = None
    _job_state = "IDLE"
    _job_state_plan = None
    _last_hud_publish = None
    _last_hud_signature = None
    _virtual_jobs = {}
    quality_report = None
    corridor_pub = None
    guide_pub = None
    candidate_pub = None
    path_pubs = {}
    # Dynamic latched Path publishers for diagnostic layers.  A* routes and
    # guides are route-suffixed, so these cannot be created from the vehicle
    # contract at startup.
    guide_path_pubs = {}
    candidate_path_pubs = {}
    _planning_progress = None

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
        # The static hold uses marker ids that do not all collide with the
        # trajectory markers (the cargo label, for example), so accepting a
        # plan must clear the hold before the first trajectory frame.  The
        # deletion is prepended to that frame atomically; a separate message
        # could be dropped by the queue_size=1 marker publisher.
        self._clear_markers_pending = False
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
                rospy.get_param("~arrow_head_length", 0.08)),
            body_height_m=float(rospy.get_param("~body_height_m",
                                               self.options.body_height_m)),
            show_footprints=rospy.get_param("~show_footprints", False),
            show_status_text=bool(
                rospy.get_param("~show_status_text", True)))
        self.show_cargo_reason = bool(
            rospy.get_param("~show_cargo_reason", True))
        self.backend = str(rospy.get_param("~backend", "legacy"))
        if self.backend not in (
                "legacy", PLAYBACK_PLAN_BACKEND, VIRTUAL_PLAYBACK_BACKEND):
            raise ValueError(
                "~backend must be 'legacy', {!r} or {!r}, got {!r}".format(
                    PLAYBACK_PLAN_BACKEND, VIRTUAL_PLAYBACK_BACKEND,
                    self.backend))
        self._job_state = "IDLE"
        self._job_state_plan = None
        self._playback_model_sha256 = None
        self._playback_semantic_version = None
        self._raw_requests = {}
        self._virtual_jobs = {}
        self._virtual_terminals = {}
        self._planning_progress = None
        self.trajectory = None
        self._last_frame = None
        self.optimizer_status = None
        self._optimizer_status_by_plan = {}
        self.quality_report = None
        self._last_state = None
        self._tf_prefix = None
        self.hud_enabled = bool(rospy.get_param("~hud_enabled", True))
        self.hud_left = int(rospy.get_param("~hud_left", 900))
        self.hud_top = int(rospy.get_param("~hud_top", 20))
        self.hud_width = int(rospy.get_param("~hud_width", 360))
        self.hud_height = int(rospy.get_param("~hud_height", 270))
        if (self.hud_left < 0 or self.hud_top < 0 or
                self.hud_width <= 0 or self.hud_height <= 0):
            raise ValueError(
                "~hud_left/~hud_top must be non-negative and "
                "~hud_width/~hud_height must be positive")
        self.hud_pub = None
        self._last_hud_publish = None
        self._last_hud_signature = None
        if self.hud_enabled and OverlayText is None:
            self.hud_enabled = False
            rospy.logwarn(
                "[formation playback] HUD requested but "
                "jsk_rviz_plugins.msg.OverlayText is unavailable (%s). "
                "Install ros-noetic-jsk-rviz-plugins for ROS Noetic, then "
                "restart; the marker display and playback status continue "
                "without the HUD. Real HUD GUI verification is NOT_RUN.",
                _hud_import_error)
        self.marker_pub = rospy.Publisher(
            MARKER_TOPIC, MarkerArray, queue_size=1, latch=True)
        self.marker_pub.publish(self._marker_array(
            clear_marker_specs(self.frame_id), rospy.Time.now()))
        # One latched nav_msgs/Path per vehicle: RViz Path displays can change
        # colour and line width, unlike a MarkerArray LINE_STRIP.  The topics
        # start empty so a restart cannot leave a stale path on screen.
        self.path_pubs = {}
        self.guide_path_pubs = {}
        self.candidate_path_pubs = {}
        vehicle_ids = (tuple(self.contract.vehicle_ids)
                       if self.contract is not None else (1, 2, 3))
        for vehicle_id in vehicle_ids:
            vehicle_id = int(vehicle_id)
            self.path_pubs[vehicle_id] = rospy.Publisher(
                VEHICLE_PATH_TOPIC.format(vehicle_id), Path,
                queue_size=1, latch=True)
        self._clear_paths()
        self.corridor_pub = None
        self.guide_pub = None
        self.candidate_pub = None
        if self.backend in (PLAYBACK_PLAN_BACKEND, VIRTUAL_PLAYBACK_BACKEND):
            self.corridor_pub = rospy.Publisher(
                CORRIDOR_TOPIC, MarkerArray, queue_size=1, latch=True)
        if self.backend == VIRTUAL_PLAYBACK_BACKEND:
            self.guide_pub = rospy.Publisher(
                GUIDES_TOPIC, MarkerArray, queue_size=1)
            self.candidate_pub = rospy.Publisher(
                CANDIDATES_TOPIC, MarkerArray, queue_size=1)
            # Active aliases copy the one route selected by the real
            # route_index in the verified planning document.  They are
            # created latched here so subscription callbacks never create a
            # publisher on the wire.
            for topic in (ASTAR_ACTIVE_PATH_TOPIC, GUIDE_ACTIVE_PATH_TOPIC):
                self.guide_path_pubs[topic] = rospy.Publisher(
                    topic, Path, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(
            STATUS_TOPIC, String, queue_size=1, latch=True)
        if self.hud_enabled:
            self.hud_pub = rospy.Publisher(
                HUD_TOPIC, OverlayText, queue_size=1, latch=True)
        self.tf_broadcaster = (TransformBroadcaster()
                               if self.publish_tf else None)
        if self.backend in (PLAYBACK_PLAN_BACKEND, VIRTUAL_PLAYBACK_BACKEND):
            rospy.Subscriber(
                PLAYBACK_PLAN_TOPIC, FormationPlaybackPlan,
                self.on_playback_plan, queue_size=1)
            rospy.Subscriber(
                JOB_STATE_TOPIC, String, self.on_job_state, queue_size=1)
        else:
            rospy.Subscriber(
                TRAJECTORY_TOPIC, FormationTimedTrajectory,
                self.on_trajectory, queue_size=1)
        rospy.Subscriber(
            OPTIMIZER_STATUS_TOPIC, String, self.on_optimizer_status,
            queue_size=1)
        rospy.Subscriber(
            QUALITY_TOPIC, String, self.on_quality, queue_size=1)
        if self.transport_ui or self.backend == VIRTUAL_PLAYBACK_BACKEND:
            # The virtual receiver must retain the raw request even when the
            # optional transport preview UI is disabled.
            rospy.Subscriber("/formation/plan_request", FormationPlanRequest,
                             self.on_request, queue_size=1)
        if self.backend == VIRTUAL_PLAYBACK_BACKEND:
            # Progress documents arrive in bursts: a stage's "complete"
            # document is immediately followed by the next stage's
            # "running" document while this callback rebuilds and publishes
            # the display layers (and the 10 Hz tick holds the same lock).
            # A one-slot subscriber queue drops those complete documents
            # (verified live on the map05 run: ASTAR/CORRIDOR/PORTAL_GUIDE
            # completions never reached the HUD or the diagnostics view).
            # A bounded multi-slot queue keeps order and delivery; it does
            # not change identity, admission, or publish gates.
            rospy.Subscriber(
                PLANNING_DEBUG_TOPIC, String, self.on_planning_debug,
                queue_size=50)
        if self.transport_ui:
            rospy.Subscriber("/formation/display_pose", PoseStamped,
                             self.on_preview, queue_size=1)
            rospy.Subscriber("/formation/status", String,
                             self.on_frontend_status, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.tick)
        if self.hud_pub is not None:
            self._publish_hud(
                status_document(
                    cleared=True, frame_id=self.frame_id,
                    backend=self.backend),
                quality=None, force=True)
        if self.corridor_pub is not None:
            # RViz keeps lifetime-zero markers across a node restart; start
            # from empty diagnostic layers instead of a stale chain.
            self._clear_diagnostics()
        rospy.loginfo(
            "[formation playback] common-clock display ready; "
            "GEOMETRY_ONLY is shown until an independent FEASIBLE status "
            "is received")
        if self.contract is not None:
            rospy.loginfo(
                "[formation playback] backend=%s formation=%s body_m=%.3fx%.3fx%.3f "
                "initial_heading_policy=%s show_footprints=%s",
                self.backend, self.contract.formation_id,
                self.contract.body_length_m, self.contract.body_width_m,
                self.options.body_height_m, self.contract.initial_heading_policy,
                self.options.show_footprints)

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
        self._clear_markers_pending = True
        self._tf_prefix = tf_frame_prefix(trajectory.formation_id)
        self._last_state = None
        rospy.loginfo(
            "[formation playback] accepted plan %s formation %s: "
            "%d vehicles, %d shared samples, T_end=%.6f s from header.stamp",
            trajectory.plan_id, trajectory.formation_id,
            len(trajectory.vehicles), trajectory.sample_count,
            trajectory.duration)
        # Publish the full path once as nav_msgs/Path (RViz-adjustable),
        # then one frame immediately so a latched trajectory that is already
        # old is still visible without waiting for the next tick.
        self._publish_paths(trajectory)
        self._publish_tick()

    @staticmethod
    def _raw_request_document(message):
        """Neutral, strict-JSON snapshot of the raw transport request."""
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
            "plan_id": int(message.plan_id),
            "formation_id": str(message.formation_id),
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

    def _record_raw_request(self, message):
        if self.backend != VIRTUAL_PLAYBACK_BACKEND:
            return
        try:
            document = self._raw_request_document(message)
        except (AttributeError, TypeError, ValueError) as exc:
            rospy.logwarn(
                "[formation playback] cannot retain raw request %s: %s",
                getattr(message, "plan_id", None), exc)
            return
        self._raw_requests[document["plan_id"]] = document
        while len(self._raw_requests) > 64:
            del self._raw_requests[min(self._raw_requests)]

    def _record_virtual_job(self, report):
        if self.backend != VIRTUAL_PLAYBACK_BACKEND:
            return
        plan_id = report.get("plan_id")
        if not isinstance(plan_id, int) or isinstance(plan_id, bool):
            return
        if report.get("backend") != VIRTUAL_PLAYBACK_BACKEND:
            return
        try:
            record = json.loads(json.dumps(
                report, sort_keys=True, allow_nan=False))
        except (TypeError, ValueError):
            return
        if report.get("state") == "PLANNING":
            self._virtual_jobs[plan_id] = record
            while len(self._virtual_jobs) > 64:
                del self._virtual_jobs[min(self._virtual_jobs)]
        elif plan_id not in self._virtual_jobs:
            self._virtual_jobs[plan_id] = record
            while len(self._virtual_jobs) > 64:
                del self._virtual_jobs[min(self._virtual_jobs)]
        if (report.get("state") == "HOLD_END" and
                isinstance(report.get("initial_context"), dict)):
            self._virtual_terminals[plan_id] = report["initial_context"]
            while len(self._virtual_terminals) > 64:
                del self._virtual_terminals[min(self._virtual_terminals)]

    @synchronized
    def on_job_state(self, message):
        report = self._parse_json(message, "job_state")
        if report is None:
            return
        state = report.get("state")
        if state not in ("IDLE", "PLANNING", "PLAYING", "HOLD_END", "FAILED"):
            return
        plan_id = report.get("plan_id")
        previous_plan = self._job_state_plan
        new_admission = (
            state == "PLANNING" and plan_id is not None and
            plan_id != previous_plan)
        self._job_state = state
        self._job_state_plan = plan_id
        self._record_virtual_job(report)
        if new_admission:
            # A new admission is the only event that empties the progressive
            # diagnostic layers.  Raw requests and busy rejections never do.
            self._clear_diagnostics()
            self._planning_progress = None
        if (state == "PLANNING" and plan_id is not None and
                self.trajectory is not None and
                self.trajectory.plan_id != plan_id):
            # A new experiment was admitted.  Clear the previous animation
            # even when this node is not subscribed to /formation/plan_request.
            self._clear(
                "new experiment {} admitted".format(plan_id),
                freeze=True, hold_reason="PLANNING")
            return
        if (state == "FAILED" and plan_id == self._active_plan and
                self.trajectory is not None):
            self._clear(
                "job failed: {}".format(
                    report.get("reason") or report.get("detail") or state),
                freeze=True, hold_reason="FAILED: {}".format(
                    report.get("reason") or report.get("detail") or state))
            return
        if state == "FAILED" and self.trajectory is None:
            self._hold_reason = "FAILED: {}".format(
                report.get("reason") or report.get("detail") or "planning")

    @synchronized
    def on_playback_plan(self, message):
        """Accept only an atomic, model-bound V0.2 playback envelope."""
        if self._previewing:
            rospy.logwarn_throttle(
                5.0,
                "[formation playback] ignoring playback plan %s while a "
                "transport preview is active",
                getattr(message, "plan_id", None))
            return
        if (self._active_plan is not None and
                message.plan_id < self._active_plan):
            return
        try:
            if message.header.frame_id != self.frame_id:
                raise PlaybackValidationError(
                    "playback plan frame_id {!r} differs from ~frame_id {!r}"
                    .format(message.header.frame_id, self.frame_id))
            envelope_stamp = (int(message.header.stamp.secs),
                              int(message.header.stamp.nsecs))
            fields = fields_from_timed_message(message.trajectory)
            if fields["stamp"] != envelope_stamp:
                raise PlaybackValidationError(
                    "envelope header stamp differs from the embedded "
                    "trajectory stamp")
            if self.backend == PLAYBACK_PLAN_BACKEND:
                if message.backend != PLAYBACK_PLAN_BACKEND:
                    raise PlaybackValidationError(
                        "envelope backend {!r} differs from the node's "
                        "configured {!r}".format(
                            message.backend, PLAYBACK_PLAN_BACKEND))
                if (message.semantic_version == PLAYBACK_PLAN_SCHEMA
                        and not (message.model_json or message.model_sha256 or message.certification
                                 or fields["times_from_start"] or fields["trajectories"]
                                 or fields["formation_poses"] or fields["vehicles"])):
                    self._clear("planner published no executable model", freeze=False)
                    return
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
            elif self.backend == VIRTUAL_PLAYBACK_BACKEND:
                if message.backend != VIRTUAL_PLAYBACK_BACKEND:
                    raise PlaybackValidationError(
                        "envelope backend {!r} differs from the node's "
                        "configured {!r}".format(
                            message.backend, VIRTUAL_PLAYBACK_BACKEND))
                trajectory, plan, digest = self._verify_virtual_envelope(
                    message, fields)
            else:
                raise PlaybackValidationError(
                    "legacy playback does not accept atomic envelopes")
        except (PlaybackValidationError, AttributeError, TypeError,
                ValueError, KeyError, IndexError) as exc:
            rospy.logerr(
                "[formation playback] rejecting playback plan %s: %s",
                getattr(message, "plan_id", None), exc)
            self._clear("invalid playback plan: {}".format(exc), freeze=False)
            return
        self.trajectory = trajectory
        self.hold = None
        self._clear_markers_pending = True
        self._active_plan = trajectory.plan_id
        self._tf_prefix = tf_frame_prefix(trajectory.formation_id)
        self._playback_model_sha256 = digest
        self._playback_semantic_version = message.semantic_version
        self._last_state = None
        rospy.loginfo(
            "[formation playback] accepted %s plan %s formation %s "
            "model_sha256=%s: %d vehicles, %d shared samples, T_end=%.6f s",
            self.backend, trajectory.plan_id, trajectory.formation_id,
            digest, len(trajectory.vehicles), trajectory.sample_count,
            trajectory.duration)
        self._publish_paths(trajectory)
        self._publish_tick()
        if (self.backend == PLAYBACK_PLAN_BACKEND and self.corridor_pub is not None
                and self.trajectory is trajectory):
            try:
                markers = build_uniform_sweep_markers(trajectory)
                self.corridor_pub.publish(self._marker_array(markers, rospy.Time.now()))
            except (PlaybackValidationError, ValueError) as exc:
                rospy.logwarn("[formation playback] cannot draw uniform sweep: %s", exc)
                self._clear_corridors()

    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _check_virtual_initial(self, request, context_record):
        """Bind the raw request start to the planner's confirmed context."""
        if not isinstance(context_record, dict):
            raise PlaybackValidationError(
                "no confirmed initial_context is retained for this plan")
        if not bool(context_record.get("compatible")):
            raise PlaybackValidationError(
                "the retained initial context is not compatible: {}".format(
                    context_record.get("detail")))
        source = context_record.get("source")
        context_id = context_record.get("context_id")
        if source not in ("rviz_reset", "verified_terminal"):
            raise PlaybackValidationError(
                "retained initial source {!r} is not a ROS experiment "
                "source".format(source))
        if not isinstance(context_id, str) or not context_id.strip():
            raise PlaybackValidationError(
                "retained initial context has no context_id")
        if request.initial.source != source or \
                request.initial.context_id != context_id:
            raise PlaybackValidationError(
                "raw request initial source/context differs from the "
                "retained confirmed record")
        qf = context_record.get("qf")
        if not isinstance(qf, dict):
            raise PlaybackValidationError(
                "retained initial context has no F pose")
        try:
            qf_values = (float(qf["x"]), float(qf["y"]), float(qf["psi"]))
        except (KeyError, TypeError, ValueError) as exc:
            raise PlaybackValidationError(
                "retained initial F pose is invalid: {}".format(exc)) from exc
        position_error = math.hypot(
            request.start_qf.x - qf_values[0],
            request.start_qf.y - qf_values[1])
        heading_error = abs(self._wrap(request.start_qf.psi - qf_values[2]))
        if position_error > 1e-6 or heading_error > 1e-6:
            raise PlaybackValidationError(
                "raw request start differs from the retained initial F pose "
                "by {:.6g} m / {:.6g} rad".format(
                    position_error, heading_error))
        if source == "rviz_reset":
            for vehicle_id in sorted(request.start_headings):
                if abs(self._wrap(
                        request.start_headings[vehicle_id]
                        - qf_values[2])) > INITIAL_HEADING_TOLERANCE_RAD:
                    raise PlaybackValidationError(
                        "vehicle_id={} start heading is not parallel to the "
                        "confirmed reset heading".format(vehicle_id))
            return
        headings = context_record.get("headings")
        achieved = {}
        if isinstance(headings, list):
            for item in headings:
                if not isinstance(item, dict):
                    continue
                vehicle_id = item.get("vehicle_id")
                value = item.get("heading_rad")
                if isinstance(vehicle_id, int) and not isinstance(
                        vehicle_id, bool):
                    try:
                        achieved[int(vehicle_id)] = float(value)
                    except (TypeError, ValueError):
                        continue
        if set(achieved) != set(request.start_headings):
            raise PlaybackValidationError(
                "retained verified-terminal headings do not cover the "
                "request vehicle ids")
        for vehicle_id in sorted(request.start_headings):
            if abs(self._wrap(
                    request.start_headings[vehicle_id]
                    - achieved[vehicle_id])) > 1e-9:
                raise PlaybackValidationError(
                    "vehicle_id={} start heading differs from the retained "
                    "verified terminal".format(vehicle_id))

    def _verify_virtual_envelope(self, message, fields):
        plan_id = int(message.plan_id)
        raw = self._raw_requests.get(plan_id)
        if raw is None:
            raise PlaybackValidationError(
                "no raw /formation/plan_request is retained for plan {}; "
                "the virtual envelope cannot be accepted".format(plan_id))
        if raw.get("formation_id") != message.formation_id:
            raise PlaybackValidationError(
                "retained raw request formation_id differs from the "
                "envelope")
        job = self._virtual_jobs.get(plan_id)
        if job is None:
            raise PlaybackValidationError(
                "no admitted virtual job record is retained for plan {}; "
                "wait for an explicit job_state/reset match".format(plan_id))
        if job.get("formation_id") != message.formation_id:
            raise PlaybackValidationError(
                "retained job formation_id differs from the envelope")
        if job.get("state") not in ("PLANNING", "PLAYING", "HOLD_END"):
            raise PlaybackValidationError(
                "retained job state {!r} cannot certify a playback "
                "envelope".format(job.get("state")))
        context_record = job.get("initial_context")
        try:
            request = virtual_a_request_from_raw_request(
                raw,
                initial_source=(None if context_record is None
                                else context_record.get("source")),
                initial_context_id=(None if context_record is None
                                    else context_record.get("context_id")))
        except PlaybackValidationError:
            raise
        except Exception as exc:
            raise PlaybackValidationError(
                "raw request cannot rebuild the expected request: {}: {}"
                .format(type(exc).__name__, exc)) from exc
        self._check_virtual_initial(request, context_record)
        fingerprint = request_fingerprint(request)
        if job.get("request_sha256") != fingerprint:
            raise PlaybackValidationError(
                "retained admission request_sha256 differs from the raw "
                "request rebuilt by the receiver")
        if str(job.get("virtual_a_goal_policy", "")).lower() != \
                request.goal_policy.value.lower():
            raise PlaybackValidationError(
                "retained goal policy differs from the raw request flags")
        context = job.get("planning_context")
        if not isinstance(context, dict):
            raise PlaybackValidationError(
                "no admitted planning context is retained for this plan")
        if context.get("request_sha256") != fingerprint:
            raise PlaybackValidationError(
                "admitted planning context request_sha256 differs from the "
                "raw request")
        grid_sha = job.get("grid_sha256")
        if not isinstance(grid_sha, str) or len(grid_sha) != 64:
            raise PlaybackValidationError(
                "no admitted grid_sha256 is retained for this plan")
        timing = context.get("timing_config")
        geometry_payload = (None if not isinstance(timing, dict)
                            else timing.get("geometry_audit_config"))
        try:
            geometry_config = geometry_audit_config_from_dict(
                geometry_payload)
        except Exception as exc:
            raise PlaybackValidationError(
                "admitted geometry audit config is invalid: {}: {}".format(
                    type(exc).__name__, exc)) from exc
        binding_json = str(
            getattr(message, "formation_binding_json", "") or "").strip()
        expected_formation = None
        if binding_json:
            try:
                expected_formation = json.loads(
                    binding_json, parse_constant=_reject_json_constant)
            except (TypeError, ValueError) as exc:
                raise PlaybackValidationError(
                    "envelope formation_binding_json is not finite JSON: {}"
                    .format(exc)) from exc
            if not isinstance(expected_formation, Mapping):
                raise PlaybackValidationError(
                    "envelope formation_binding_json must decode to a JSON "
                    "object")
        return verify_playback_plan(
            backend=message.backend,
            plan_id=plan_id,
            formation_id=message.formation_id,
            semantic_version=message.semantic_version,
            model_version=message.model_version,
            model_sha256=message.model_sha256,
            model_json=message.model_json,
            certification=message.certification,
            trajectory_fields=fields,
            expected_request=request,
            expected_context=context,
            expected_grid_sha256=grid_sha,
            expected_formation=expected_formation,
            geometry_audit_config=geometry_config,
            position_tolerance=1e-9,
            angle_tolerance=1e-9,
            speed_tolerance=1e-9)

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
        plan_id = report.get("plan_id")
        if isinstance(plan_id, int) and not isinstance(plan_id, bool):
            statuses = getattr(self, "_optimizer_status_by_plan", None)
            if statuses is None:
                statuses = self._optimizer_status_by_plan = {}
            previous = statuses.get(int(plan_id))
            # A rejected busy reset is labelled with the active plan id but is
            # not a statement about that plan's envelope; it must not erase the
            # certification already published for the playing trajectory.
            if not (previous is not None and
                    previous.get("trajectory_published") and
                    report.get("admitted") is False and
                    not report.get("trajectory_published")):
                statuses[int(plan_id)] = report
            while len(statuses) > 32:
                del statuses[min(statuses)]
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
    def on_planning_debug(self, message):
        """Layer-preserving progressive planning display (TASK 01 Step 1).

        Absent layer keys preserve the existing markers; an explicit ``null``
        clears exactly that layer.  A document that fails identity, state or
        parse checks changes nothing.  A builder failure clears only its own
        layer.  ``CLEARED`` is the only payload that empties all diagnostics.
        """
        document = self._parse_json(message, "planning_debug")
        if document is None:
            return
        plan_id = document.get("plan_id")
        if plan_id != self._job_state_plan:
            rospy.logwarn_throttle(
                5.0,
                "[formation playback] ignoring planning payload for plan %s: "
                "current job is plan %s", plan_id, self._job_state_plan)
            return
        job = (self._virtual_jobs.get(plan_id)
               if isinstance(plan_id, int) else None)
        if isinstance(job, dict) and job.get("model_sha256") is None:
            # The planner freezes its active model digest immediately before
            # publishing the envelope, so the PLANNING job record may still
            # carry the admission-time None until the next heartbeat.  This
            # receiver has already verified the envelope for the same plan;
            # only that missing field is supplied, never a reported digest.
            trajectory = getattr(self, "trajectory", None)
            verified = getattr(self, "_playback_model_sha256", None)
            if (trajectory is not None and
                    trajectory.plan_id == plan_id and
                    isinstance(verified, str) and verified):
                job = dict(job)
                job["model_sha256"] = verified
        if not corridor_display_matches(document, job):
            rospy.logwarn_throttle(
                5.0,
                "[formation playback] ignoring planning payload for plan %s: "
                "identity does not match the current admission", plan_id)
            return
        if document.get("state") == CLEARED:
            self._clear_diagnostics()
            self._planning_progress = None
            return
        failed_document = (
            document.get("state") == PLANNING_FAILED_STATE)
        if self._job_state not in ("PLANNING", "PLAYING") and not (
                failed_document and self._job_state == PLANNING_FAILED_STATE):
            # The final FAILED document may be processed after the matching
            # FAILED job report; it must still freeze the completed layers.
            rospy.logwarn(
                "[formation playback] ignoring planning payload for plan %s "
                "while job state is %s", plan_id, self._job_state)
            return
        stamp = rospy.Time.now()
        self._record_planning_progress(document)
        corridors_present = "corridors" in document
        corridors = document.get("corridors")
        if corridors_present:
            if corridors is None:
                self._clear_corridors()
                if document.get("astar"):
                    self._publish_guide_layer(document, stamp)
                else:
                    self._clear_guides()
            else:
                self._publish_corridor_layer(document, stamp)
                self._publish_guide_layer(document, stamp)
        elif "astar" in document:
            if document.get("astar") is None:
                self._clear_guides()
            else:
                self._publish_guide_layer(document, stamp)
        if "candidates" in document:
            candidates = document.get("candidates")
            if candidates is None and document.get("failure") is None:
                self._clear_candidates()
            else:
                try:
                    specs, paths = build_candidate_display(
                        document, self.frame_id)
                except (TypeError, ValueError) as exc:
                    rospy.logerr(
                        "[formation playback] rejecting candidate payload "
                        "for plan %s: %s", plan_id, exc)
                    self._clear_candidates()
                else:
                    self._publish_candidate_paths(paths, stamp)
                    if self.candidate_pub is not None:
                        self.candidate_pub.publish(
                            self._marker_array(specs, stamp))
        rospy.loginfo(
            "[formation playback] planning display plan=%s generation=%s "
            "stage=%s/%s layers=%s",
            plan_id, document.get("generation"),
            document.get("stage"), document.get("stage_index"),
            ",".join(sorted(
                key for key in ("astar", "corridors", "candidates")
                if key in document)) or "identity-only")

    def _publish_corridor_layer(self, document, stamp):
        if self.corridor_pub is None:
            return
        try:
            specs = build_corridor_markers(document, self.frame_id)
        except (TypeError, ValueError) as exc:
            rospy.logerr(
                "[formation playback] rejecting corridor payload: %s", exc)
            self._clear_corridors()
            return
        self.corridor_pub.publish(self._marker_array(specs, stamp))

    def _path_registry(self, name):
        registry = getattr(self, name, None)
        if registry is None:
            registry = {}
            setattr(self, name, registry)
        return registry

    def _path_publisher(self, registry_name, topic):
        registry = self._path_registry(registry_name)
        publisher = registry.get(topic)
        if publisher is None:
            publisher = rospy.Publisher(
                topic, Path, queue_size=1, latch=True)
            registry[topic] = publisher
        return publisher

    def _topic_path_message(self, samples, stamp):
        message = Path()
        message.header.frame_id = self.frame_id
        message.header.stamp = stamp
        for x, y in samples:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = stamp
            pose.pose.position = Point(x=float(x), y=float(y), z=0.0)
            pose.pose.orientation = Quaternion(w=1.0)
            message.poses.append(pose)
        return message

    def _publish_topic_paths(self, registry_name, entries, stamp):
        for topic, samples in entries:
            self._path_publisher(registry_name, topic).publish(
                self._topic_path_message(samples, stamp))

    def _clear_topic_paths(self, registry_name):
        """Empty every latched Path in one diagnostic-layer registry.

        Clearing before each new payload removes route topics that are no
        longer present, so a shorter route list cannot leave stale geometry
        behind on the map.
        """
        now = rospy.Time.now()
        for publisher in self._path_registry(registry_name).values():
            message = Path()
            message.header.frame_id = self.frame_id
            message.header.stamp = now
            publisher.publish(message)

    def _publish_guide_paths(self, paths, stamp, route_index=None):
        self._clear_topic_paths("guide_path_pubs")
        entries = []
        for (kind, route_id), samples in paths.items():
            if kind == "astar_raw":
                topic = ASTAR_PATH_TOPIC.format(route_id)
            elif kind == "portal_guide":
                topic = GUIDE_PATH_TOPIC.format(route_id)
            else:
                raise PlaybackValidationError(
                    "unknown guide path kind {!r}".format(kind))
            entries.append((topic, samples))
        self._publish_topic_paths("guide_path_pubs", entries, stamp)
        self._publish_active_guide_paths(paths, stamp, route_index)

    def _publish_active_guide_paths(self, paths, stamp, route_index):
        """Copy only the route the planner reports as currently selected.

        The aliases never fall back to route 0.  A missing, malformed or
        out-of-range ``route_index`` leaves both active topics empty because
        ``_clear_topic_paths`` has already emptied this registry.
        """
        if isinstance(route_index, bool) or not isinstance(route_index, int):
            return
        registry = self._path_registry("guide_path_pubs")
        for topic, kind in ((ASTAR_ACTIVE_PATH_TOPIC, "astar_raw"),
                            (GUIDE_ACTIVE_PATH_TOPIC, "portal_guide")):
            publisher = registry.get(topic)
            if publisher is None:
                continue
            samples = paths.get((kind, route_index))
            if samples:
                publisher.publish(self._topic_path_message(samples, stamp))

    def _publish_candidate_paths(self, paths, stamp):
        self._clear_topic_paths("candidate_path_pubs")
        entries = []
        for key, samples in paths.items():
            if key == "f_candidate":
                topic = F_CANDIDATE_PATH_TOPIC
            elif (isinstance(key, tuple) and len(key) == 2
                  and key[0] == "candidate_vehicle"):
                topic = CANDIDATE_VEHICLE_PATH_TOPIC.format(key[1])
            else:
                raise PlaybackValidationError(
                    "unknown candidate path key {!r}".format(key))
            entries.append((topic, samples))
        self._publish_topic_paths("candidate_path_pubs", entries, stamp)

    def _publish_guide_layer(self, document, stamp):
        try:
            specs, paths = build_guide_display(document, self.frame_id)
        except (TypeError, ValueError) as exc:
            rospy.logerr(
                "[formation playback] rejecting guide payload: %s", exc)
            self._clear_guides()
            return
        self._publish_guide_paths(paths, stamp,
                                  active_route_index(document))
        if self.guide_pub is not None:
            self.guide_pub.publish(self._marker_array(specs, stamp))

    def _clear_corridors(self):
        if self.corridor_pub is None:
            return
        self.corridor_pub.publish(self._marker_array(
            clear_marker_specs(self.frame_id), rospy.Time.now()))

    def _clear_guides(self):
        self._clear_topic_paths("guide_path_pubs")
        if self.guide_pub is None:
            return
        self.guide_pub.publish(self._marker_array(
            clear_marker_specs(self.frame_id), rospy.Time.now()))

    def _clear_candidates(self):
        self._clear_topic_paths("candidate_path_pubs")
        if self.candidate_pub is None:
            return
        self.candidate_pub.publish(self._marker_array(
            clear_marker_specs(self.frame_id), rospy.Time.now()))

    def _clear_paths(self):
        """Empty every latched vehicle Path display."""
        now = rospy.Time.now()
        for publisher in getattr(self, "path_pubs", {}).values():
            message = Path()
            message.header.frame_id = self.frame_id
            message.header.stamp = now
            publisher.publish(message)

    def _publish_paths(self, trajectory):
        """Publish the verified full centre path of each vehicle once."""
        publishers = getattr(self, "path_pubs", None)
        if publishers is None:
            publishers = {}
            self.path_pubs = publishers
        stamp = rospy.Time(*trajectory.stamp)
        for vehicle_id, samples in vehicle_path_samples(trajectory).items():
            vehicle_id = int(vehicle_id)
            publisher = publishers.get(vehicle_id)
            if publisher is None:
                publisher = rospy.Publisher(
                    VEHICLE_PATH_TOPIC.format(vehicle_id), Path,
                    queue_size=1, latch=True)
                publishers[vehicle_id] = publisher
            message = Path()
            message.header.frame_id = trajectory.frame_id
            message.header.stamp = stamp
            for x, y, theta in samples:
                pose = PoseStamped()
                pose.header.frame_id = trajectory.frame_id
                pose.header.stamp = stamp
                pose.pose.position = Point(x=x, y=y, z=0.0)
                pose.pose.orientation = self._quaternion(theta)
                message.poses.append(pose)
            publisher.publish(message)

    def _clear_diagnostics(self):
        """Clear every non-hold planning layer, including the full paths."""
        self._clear_corridors()
        self._clear_guides()
        self._clear_candidates()
        self._clear_paths()

    def _record_planning_progress(self, document):
        """Retain the stage fields; geometry stays in the topics.

        Diagnostic counts are copied only from this document's real
        ``metrics``.  A later stage of the same generation/round keeps the
        last completed assessment until a new one arrives; a new round or
        plan never inherits another round's counts.
        """
        view = planning_diagnostics_view(document)
        previous = getattr(self, "_planning_progress", None)
        same_context = (
            isinstance(previous, dict)
            and previous.get("plan_id") == document.get("plan_id")
            and previous.get("generation") == document.get("generation")
            and previous.get("round_index") == document.get("round_index"))
        view_has_data = isinstance(view, dict) and (
            view.get("guide_source") is not None
            or view.get("footprint_region_count") is not None
            or isinstance(view.get("assessment"), Mapping))
        if isinstance(view, dict) and not view_has_data and same_context:
            retained = previous.get("diagnostics")
            if retained is not None:
                view = retained
        self._planning_progress = {
            "plan_id": document.get("plan_id"),
            "formation_id": document.get("formation_id"),
            "generation": document.get("generation"),
            "stage": document.get("stage"),
            "stage_index": document.get("stage_index"),
            "state": document.get("state"),
            "elapsed_s": document.get("elapsed_s"),
            "route_index": document.get("route_index"),
            "route_count": document.get("route_count"),
            "round_index": document.get("round_index"),
            "reason": document.get("reason"),
            "detail": document.get("detail"),
            "diagnostics": view,
            "received_wall": rospy.Time.now().to_sec(),
        }

    def _hud_progress(self):
        """HUD view with a 10 Hz elapsed extrapolation while planning."""
        progress = getattr(self, "_planning_progress", None)
        if not isinstance(progress, dict):
            return None
        result = dict(progress)
        received = result.pop("received_wall", None)
        if (result.get("state") == "PLANNING" and
                isinstance(received, (int, float)) and
                math.isfinite(float(received))):
            elapsed = result.get("elapsed_s")
            if isinstance(elapsed, (int, float)) and not isinstance(
                    elapsed, bool):
                result["elapsed_s"] = max(
                    0.0, float(elapsed) +
                    max(0.0, rospy.Time.now().to_sec() - float(received)))
        return result

    @synchronized
    def on_request(self, message):
        self._record_raw_request(message)
        if self.backend == VIRTUAL_PLAYBACK_BACKEND:
            # Raw message receipt is not admission; only the planner's
            # job_state PLANNING admits an experiment.  Never clear the active
            # replay, the hold or a diagnostic layer from this callback.
            # It does end a transport preview: the goal bridge latches
            # /formation/display_pose before the goal, and leaving
            # ``_previewing`` set would make ``on_playback_plan`` silently
            # drop the atomic envelope for this experiment.  A busy request
            # for another admitted plan keeps the preview, matching the
            # non-virtual ordering above.
            if (self._job_state not in ("PLANNING", "PLAYING") or
                    message.plan_id == self._job_state_plan):
                self._previewing = False
            return
        if (self.backend in (PLAYBACK_PLAN_BACKEND, VIRTUAL_PLAYBACK_BACKEND)
                and self._job_state in ("PLANNING", "PLAYING") and
                message.plan_id != self._job_state_plan):
            # A busy-admission rejection must not clear or restart the active
            # replay.  The planner never admits this request; a direct
            # injection reaches playback only as an ignored request.
            rospy.logwarn(
                "[formation playback] ignoring busy plan_request %s while "
                "job %s is %s", message.plan_id, self._job_state_plan,
                self._job_state)
            return
        if (self.backend in (PLAYBACK_PLAN_BACKEND, VIRTUAL_PLAYBACK_BACKEND)
                and self.trajectory is not None and
                self.trajectory.plan_id == message.plan_id):
            # The atomic envelope for this admitted request may arrive before
            # the request itself.  Never clear a trajectory that is already
            # playing the same plan.
            return
        if (self.backend in (PLAYBACK_PLAN_BACKEND, VIRTUAL_PLAYBACK_BACKEND)
                and self._active_plan is not None and
                message.plan_id < self._active_plan):
            # A stale latched request must not replace a newer experiment.
            return
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
        self._clear_markers_pending = False
        self._clear_corridors()
        self._clear_guides()
        self._publish_hold(clear=True)

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
        self._clear_markers_pending = False
        self._previewing = True
        self._active_plan = None
        self._hold_reason = "START"
        # A manual 2D Pose Estimate starts a new experiment.  The previous
        # plan's candidate, corridor, guide and full-path layers belong to
        # that old experiment and must disappear immediately; only a raw
        # request (which is not admission) is forbidden from clearing them.
        self._clear_diagnostics()
        self._publish_hold(clear=True)

    @synchronized
    def on_frontend_status(self, message):
        report = self._parse_json(message, "frontend_status")
        if report and isinstance(report.get("plan_id"), int):
            self._frontend_reports[report["plan_id"]] = report
            while len(self._frontend_reports) > 32:
                del self._frontend_reports[min(self._frontend_reports)]

    def _publish_hold(self, clear=False, stamp=None):
        if self.hold is None:
            return
        formation_id, qf, vehicles = self.hold
        reason = self._hold_reason
        frontend = self._frontend_reports.get(self._active_plan, {})
        if frontend.get("reason"):
            prefix = "WAITING" if frontend["reason"] == "MAP_UNAVAILABLE" else "FAILED"
            reason = "{}: {}".format(prefix, frontend["reason"])
        specs = build_hold_markers(
            formation_id, qf, vehicles, reason, self.frame_id,
            self.show_cargo_reason, options=self.options)
        if clear:
            # A DELETEALL sent as its own marker message can be dropped by the
            # queue_size=1 publisher when the static hold follows immediately,
            # leaving the old path visible in RViz.  Send the deletion and the
            # replacement hold atomically, in that order.
            specs = clear_marker_specs(self.frame_id) + specs
        stamp = stamp if stamp is not None else rospy.Time.now()
        self.marker_pub.publish(self._marker_array(specs, stamp))
        self._tf_prefix = tf_frame_prefix(formation_id)
        self._broadcast_tf(SimpleNamespace(
            formation_x=qf[0], formation_y=qf[1], formation_psi=qf[2],
            vehicles=tuple(SimpleNamespace(vehicle_id=v.vehicle_id,
                center_x=x, center_y=y, theta=theta)
                for v, x, y, theta in vehicles)), stamp)
        document = {
            "schema": "formation_playback/v1", "state": "STATIC",
            "plan_id": self._active_plan, "formation_id": formation_id,
            "backend": self.backend,
            "model_sha256": self._playback_model_sha256,
            "request_sha256": None,
            "planning_context_sha256": None,
            "vehicles": [
                {"vehicle_id": v.vehicle_id} for v, _x, _y, _t in vehicles],
            "reason": reason, "executable": False,
            "trajectory_published": False, "cleared": True}
        self.status_pub.publish(String(data=json.dumps(document)))
        self._publish_hud(document, quality=None, force=True)

    def _publish_hud(self, document, quality=None, force=False):
        """Publish the fixed-corner overlay at 10 Hz or on a state change."""
        if self.hud_pub is None:
            return
        now = rospy.Time.now()
        progress = self._hud_progress()
        signature = (
            document.get("plan_id"), document.get("state"), self._job_state,
            None if progress is None else (
                progress.get("stage"), progress.get("stage_index"),
                progress.get("route_index"), progress.get("round_index"),
                progress.get("state"), progress.get("reason")))
        changed = signature != self._last_hud_signature
        if (not force and not changed and
                self._last_hud_publish is not None and
                (now - self._last_hud_publish).to_sec() < 0.095):
            return
        message = OverlayText()
        message.action = OverlayText.ADD
        message.width = self.hud_width
        message.height = self.hud_height
        message.left = self.hud_left
        message.top = self.hud_top
        message.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.55)
        message.line_width = 2
        message.text_size = 13.0
        message.font = "DejaVu Sans Mono"
        message.fg_color = ColorRGBA(1.0, 1.0, 1.0, 0.95)
        message.text = hud_text(
            document, self._job_state, quality, self.show_cargo_reason,
            progress=progress)
        self.hud_pub.publish(message)
        self._last_hud_publish = now
        self._last_hud_signature = signature

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
            self._publish_progress_hud()
            return
        self._publish_tick()

    def _publish_progress_hud(self):
        """Refresh the fixed-corner HUD while planning has no trajectory."""
        if self.hud_pub is None:
            return
        progress = self._hud_progress()
        if progress is None:
            return
        state = progress.get("state")
        formation_id = progress.get("formation_id")
        if not isinstance(formation_id, str) or not formation_id:
            formation_id = None if self.hold is None else self.hold[0]
        document = {
            "schema": "formation_playback/v1",
            "state": state if isinstance(state, str) else "UNKNOWN",
            "cleared": False,
            "plan_id": self._job_state_plan,
            "formation_id": formation_id,
            "backend": self.backend,
            "model_sha256": self._playback_model_sha256,
            "request_sha256": None,
            "planning_context_sha256": None,
            "executable": False,
        }
        self._publish_hud(document, quality=None)

    def _status_for_trajectory(self):
        # A busy rejection (or a newly admitted plan) publishes its own status
        # while this plan is still playing.  Certification must come from the
        # status retained for this exact plan, never from whichever report
        # happened to arrive last.
        report = getattr(self, "_optimizer_status_by_plan", {}).get(
            self.trajectory.plan_id)
        if not (isinstance(report, dict) and report.get(
                "plan_id") == self.trajectory.plan_id):
            return None
        # A verified uniform_v02 envelope carries its authoritative model
        # digest.  The audit status must belong to exactly that model; a
        # status for another digest/formation (or one missing the identity)
        # must never certify this replay.  Legacy trajectories keep the
        # historical plan_id-only rule.
        digest = self.trajectory.model_sha256
        audit_backend = self.trajectory.backend or PLAYBACK_PLAN_BACKEND
        if digest is not None and (
                report.get("model_sha256") != digest or
                report.get("formation_id") != self.trajectory.formation_id or
                report.get("backend") != audit_backend):
            rospy.logwarn_throttle(
                5.0,
                "[formation playback] ignoring audit status for plan %s: "
                "identity differs from the verified envelope "
                "(model_sha256/formation_id/backend)",
                self.trajectory.plan_id)
            return None
        # The optimizer status is deliberately compact; the independent
        # evaluation metrics live in the latched quality report.  Merge that
        # report's pipeline block only when its full identity matches this
        # playback: a quality block for an older plan/formation/backend/model
        # (or for another virtual request/planning context) must not leak into
        # the displayed metrics.
        quality = self.quality_report
        identity = {
            "plan_id": self.trajectory.plan_id,
            "formation_id": self.trajectory.formation_id,
            "backend": audit_backend,
            "model_sha256": digest,
        }
        if digest is not None and audit_backend == VIRTUAL_PLAYBACK_BACKEND:
            job = self._virtual_jobs.get(self.trajectory.plan_id, {})
            identity["request_sha256"] = job.get("request_sha256")
            identity["planning_context_sha256"] = job.get(
                "planning_context_sha256")
        if (quality_identity_matches(identity, quality) and
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
            publish_markers = markers
            if self._clear_markers_pending:
                publish_markers = (
                    clear_marker_specs(self.frame_id) + tuple(markers))
                self._clear_markers_pending = False
        except (PlaybackValidationError, ValueError) as exc:
            rospy.logerr(
                "[formation playback] cannot build playback frame: %s", exc)
            self._clear("playback frame failed: {}".format(exc))
            return
        self._last_frame = (trajectory, frame)
        center_error = heading_error = None
        if frame.terminal:
            center_error, heading_error = terminal_errors(trajectory, frame)
        request_sha = context_sha = None
        if (self._playback_model_sha256 is not None and
                self.backend == VIRTUAL_PLAYBACK_BACKEND):
            job = self._virtual_jobs.get(trajectory.plan_id, {})
            request_sha = job.get("request_sha256")
            context_sha = job.get("planning_context_sha256")
        document = status_document(
            trajectory, frame, status, marker_count=len(markers),
            namespaces=namespaces,
            terminal_center_error_m=center_error,
            terminal_heading_error_rad=heading_error,
            model_sha256=self._playback_model_sha256,
            backend=self.backend,
            semantic_version=self._playback_semantic_version,
            request_sha256=request_sha,
            planning_context_sha256=context_sha)
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
        self.marker_pub.publish(self._marker_array(publish_markers, now))
        self.status_pub.publish(String(data=payload))
        self._publish_hud(document, quality=self.quality_report)
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
        return marker_array(specs, stamp)

    def _clear(self, reason, freeze=True, hold_reason=None):
        if (freeze and self.transport_ui and self.trajectory is not None and
                self.hold is None):
            trajectory = self.trajectory
            elapsed = (rospy.Time.now() - rospy.Time(*trajectory.stamp)).to_sec()
            try:
                frame = frame_at(trajectory, elapsed)
            except PlaybackValidationError:
                frame = (self._last_frame[1] if self._last_frame is not None
                         and self._last_frame[0] is trajectory else None)
            if frame is not None:
                specs = trajectory.vehicle_map()
                self.hold = (trajectory.formation_id,
                             (frame.formation_x, frame.formation_y, frame.formation_psi),
                             tuple((specs[v.vehicle_id], v.center_x, v.center_y, v.theta)
                                   for v in frame.vehicles))
                self._hold_reason = hold_reason or "CLEARED"
        self.trajectory = None
        self._last_frame = None
        self._clear_markers_pending = False
        self._last_state = CLEARED
        now = rospy.Time.now()
        document = status_document(
            cleared=True, invalid_reason=reason, frame_id=self.frame_id,
            model_sha256=self._playback_model_sha256,
            backend=self.backend,
            semantic_version=self._playback_semantic_version)
        self.status_pub.publish(String(data=json.dumps(
            document, sort_keys=True, allow_nan=False)))
        self._publish_hud(document, quality=None, force=True)
        # Diagnostic layers are cleared only by CLEARED or a new admission;
        # a playback clear must leave a completed FAILED plan visible.  The
        # Path layers are tied to this playback trajectory, so they are
        # emptied here rather than left on screen.
        self._clear_paths()
        if self.backend == PLAYBACK_PLAN_BACKEND:
            self._clear_corridors()
        rospy.loginfo("[formation playback] cleared: %s", reason)
        if self.transport_ui and freeze and self.hold is not None:
            self._publish_hold(clear=True, stamp=now)
        else:
            specs = clear_marker_specs(self.frame_id)
            self.marker_pub.publish(self._marker_array(specs, now))


def main():
    rospy.init_node("formation_playback")
    node = FormationPlaybackNode()
    try:
        rospy.spin()
    finally:
        node.timer.shutdown()


if __name__ == "__main__":
    main()
