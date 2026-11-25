#!/usr/bin/env python3
"""FR05 thin ROS boundary around the autonomous V0.2 uniform planner.

This node is the only V0.2 publisher.  It accepts one ``FormationPlanRequest``
at a time, runs ``plan_uniform`` from FR04 in a worker thread, and publishes an
atomic ``FormationPlaybackPlan`` only when the FR02 audit reports an executable
plan.  The envelope embeds the complete versioned ``UniformTimedPlan`` model;
the sample trajectory inside it is a projection of that model, never the
authority.  Playback re-derives every display frame analytically from
``model_json``.

The node also owns the five admission states IDLE / PLANNING / PLAYING /
HOLD_END / FAILED.  ``/formation/job_state`` exposes that state so the goal
bridge can reject a Goal or InitialPose before it consumes a start override,
and playback can ignore a non-admitted request.  Busy release is driven by the
real playback terminal state (``/formation/playback_status`` reaching
HOLD_END) or by an explicit failure -- not by "a trajectory was published".
"""
from collections.abc import Mapping
from dataclasses import replace
from fwis_ros.node import run_node, time_message, duration_message, duration_from_seconds, elapsed

import json
import hashlib
import math
import threading
import time as _time

from geometry_msgs.msg import Point, Pose2D, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
from std_msgs.msg import String

from fwis_v3_msgs.msg import (
    FormationPlanRequest,
    FormationPlaybackPlan,
    FormationTimedPoint,
    FormationTimedTrajectory as FormationTimedTrajectoryMsg,
    FormationVehicleSpec,
    FormationVehicleTrajectory,
)

from fwis_backends.multi_4wis.formation_collision import CargoFootprint, grid_from_occupancy_grid
from fwis_backends.multi_4wis.formation import FormationModel
from fwis_backends.multi_4wis.formation_continuous_a.contracts import (
    PROGRESS_FAILED_STAGE,
    PROGRESS_SCHEMA,
    PROGRESS_STAGE_INDEX,
    PROGRESS_STAGES,
    PROGRESS_STATES,
)
from fwis_backends.multi_4wis.formation_playback import (
    CANDIDATE_CERTIFIED,
    CANDIDATE_UNVERIFIED,
    PLANNING_DEBUG_SCHEMA,
    PLANNING_FAILED_STATE,
    PLANNING_PUBLISHED_STATE,
    PLAYBACK_PLAN_BACKEND,
    PLAYBACK_PLAN_SCHEMA,
    failure_point_from_result,
    fields_from_timed_message,
    sample_candidate_path,
    verify_uniform_playback_plan,
)
from fwis_backends.multi_4wis.formation_quality import grid_signature
from fwis_backends.multi_4wis.formation_request import (
    validate_formation_request,
    validate_formation_timed_trajectory,
)
from fwis_backends.multi_4wis.formation_retiming import (
    FormationTimedTrajectory,
    FormationTimingLimits,
)
from fwis_backends.multi_4wis.uniform_pipeline import (
    PIPELINE_REQUEST_REJECTED,
    UniformPlanningRequest,
    plan_uniform,
    refine_certified_plan,
)
from fwis_backends.multi_4wis.uniform_search import (
    SEARCH_AUDIT_REJECTED,
    SEARCH_BUDGET_EXHAUSTED,
    SEARCH_NO_MOTION,
    SEARCH_NO_SOLUTION,
    SEARCH_SUCCESS,
    GraphSearchConfig,
    SuccessorConfig,
)
from fwis_backends.multi_4wis.uniform_trajectory import CERTIFICATION, MODEL_VERSION

STATUS_SCHEMA = "formation_optimizer_status/v1"
QUALITY_SCHEMA = "formation_quality_report/v1"
JOB_SCHEMA = "formation_job_state/v1"

MAP_TOPIC = "/map"
REQUEST_TOPIC = "/formation/plan_request"
STATUS_TOPIC = "/formation/optimizer_status"
QUALITY_TOPIC = "/formation/quality_report"
PLAYBACK_PLAN_TOPIC = "/formation/playback_plan"
PLAYBACK_STATUS_TOPIC = "/formation/playback_status"
JOB_STATE_TOPIC = "/formation/job_state"
PLANNING_DEBUG_TOPIC = "/formation/planning_debug"

IDLE = "IDLE"
PLANNING = "PLANNING"
PLAYING = "PLAYING"
HOLD_END = "HOLD_END"
FAILED = "FAILED"
STATES = (IDLE, PLANNING, PLAYING, HOLD_END, FAILED)
BUSY_STATES = (PLANNING, PLAYING)
MAX_PLAYBACK_CONFIRMATION_GRACE_S = 30.0

_PIPELINE_TO_STATUS = {
    SEARCH_SUCCESS: "FEASIBLE",
    SEARCH_NO_MOTION: "NO_MOTION",
    SEARCH_AUDIT_REJECTED: "REJECTED",
    SEARCH_BUDGET_EXHAUSTED: "REJECTED",
    SEARCH_NO_SOLUTION: "REJECTED",
    PIPELINE_REQUEST_REJECTED: "INVALID",
}


def _status_word(pipeline_status):
    return _PIPELINE_TO_STATUS.get(pipeline_status, "REJECTED")


def _json(data):
    return json.dumps(data, sort_keys=True, allow_nan=False)


def _is_sha256_text(value):
    return (isinstance(value, str) and len(value) == 64
            and all(character in "0123456789abcdef" for character in value))


def _progress_metrics_copy(metrics):
    """JSON-safe copy of real progress metrics, or ``None`` if not finite."""
    if not isinstance(metrics, Mapping):
        return None
    try:
        return json.loads(json.dumps(dict(metrics), allow_nan=False))
    except (TypeError, ValueError):
        return None


def _finite(value, context):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("{} must be finite: {!r}".format(context, value)) from exc
    if not math.isfinite(result):
        raise ValueError("{} must be finite: {!r}".format(context, value))
    return result


def timed_trajectory_fields(snapshot, timed, stamp, goal_qf=None,
                            goal_headings=None):
    """Build ROS-message field values from the FR02 projection.

    ``snapshot`` is the validated ``FormationRequestSnapshot``; ``timed`` is
    the ``FormationTimedTrajectory`` carried by ``plan.sample()``.  The values
    are re-derived from the request/model, not copied from the sample table.

    ``goal_qf``/``goal_headings`` are the plan's authoritative terminal pose
    and body headings.  The search may legitimately stop within the frozen
    goal tolerances, so the achieved terminal state can differ from the
    requested one by more than the envelope's strict 1e-9 identity tolerance.
    The envelope verifier compares these fields against
    ``plan.model.centers(plan.goal_qf)`` and ``plan.goal_headings``, so a real
    plan must pass its achieved terminal state here; omitting the arguments
    keeps the pure-request behavior used by unit tests.
    """
    if not isinstance(timed, FormationTimedTrajectory):
        raise ValueError("expected FormationTimedTrajectory")
    if (snapshot.plan_id != timed.plan_id or
            snapshot.formation_id != timed.formation_id):
        raise ValueError("timed trajectory identity differs from the request")
    if not isinstance(stamp, tuple) or len(stamp) != 2:
        raise ValueError("stamp must be (secs, nsecs)")
    secs, nsecs = int(stamp[0]), int(stamp[1])
    if secs < 0 or not 0 <= nsecs < 1000000000 or (secs == 0 and nsecs == 0):
        raise ValueError("publication stamp must be an explicit non-zero time")
    model = snapshot.model
    if not isinstance(model, FormationModel):
        raise ValueError("request model is not a FormationModel")
    offsets = tuple(model.offsets)
    if tuple(timed.offsets) != offsets:
        raise ValueError("timed offsets differ from the request model")
    start_centers = model.centers(snapshot.start)
    goal_pose = snapshot.goal if goal_qf is None else goal_qf
    goal_centers = model.centers(goal_pose)
    start_headings = dict(snapshot.start_headings)
    goal_heading_values = dict(
        snapshot.goal_headings if goal_headings is None else goal_headings)
    free_flags = dict(snapshot.goal_heading_free)
    shapes = dict(snapshot.vehicle_shapes)
    times = tuple(float(value) for value in timed.times_from_start)
    if (not times or times[0] != 0.0 or any(
            second <= first for first, second in zip(times, times[1:]))):
        raise ValueError("times_from_start is not strictly increasing from 0")
    if len(timed.formation_poses) != len(times):
        raise ValueError("formation pose count differs from the time count")
    if set(timed.trajectories) != set(timed.vehicle_ids):
        raise ValueError("timed trajectory vehicle ids differ")
    vehicles = []
    trajectories = []
    for offset in offsets:
        vehicle_id = offset.vehicle_id
        shape = shapes.get(vehicle_id)
        if shape is None:
            raise ValueError("missing vehicle shape {}".format(vehicle_id))
        vehicles.append({
            "vehicle_id": vehicle_id,
            "offset": (float(offset.x), float(offset.y), 0.0),
            "start_center": (float(start_centers[vehicle_id][0]),
                             float(start_centers[vehicle_id][1]),
                             float(start_headings[vehicle_id])),
            "goal_center": (float(goal_centers[vehicle_id][0]),
                            float(goal_centers[vehicle_id][1]),
                            float(goal_heading_values[vehicle_id])),
            "goal_heading_free": bool(free_flags.get(vehicle_id, False)),
            "body_length_m": float(shape.length_m),
            "body_width_m": float(shape.width_m),
        })
        points = tuple(timed.trajectories[vehicle_id])
        if len(points) != len(times):
            raise ValueError(
                "vehicle {} point count differs from the time count".format(
                    vehicle_id))
        converted = []
        for index, point in enumerate(points):
            if abs(float(point.time_from_start) - times[index]) > 1e-9:
                raise ValueError(
                    "vehicle {} point {} time differs from the shared axis"
                    .format(vehicle_id, index))
            converted.append((
                float(point.time_from_start),
                float(point.center_x), float(point.center_y),
                float(point.theta),
                float(point.center_speed_mps),
                float(point.center_yaw_rate_radps),
                int(point.mode), float(point.mode_control)))
        trajectories.append({"vehicle_id": vehicle_id,
                             "points": converted})
    return {
        "frame_id": "map",
        "stamp": (secs, nsecs),
        "plan_id": int(timed.plan_id),
        "formation_id": timed.formation_id,
        "vehicles": vehicles,
        "formation_poses": [
            (float(pose.x), float(pose.y), float(pose.psi))
            for pose in timed.formation_poses],
        "times_from_start": list(times),
        "trajectories": trajectories,
    }


def timed_message(fields):
    """Convert plain field values into a ``FormationTimedTrajectory``."""
    message = FormationTimedTrajectoryMsg()
    message.header.frame_id = fields["frame_id"]
    secs, nsecs = fields["stamp"]
    message.header.stamp = time_message(secs, nsecs)
    message.plan_id = int(fields["plan_id"])
    message.formation_id = fields["formation_id"]
    for values in fields["vehicles"]:
        spec = FormationVehicleSpec()
        spec.vehicle_id = int(values["vehicle_id"])
        spec.offset_in_formation = Point(
            x=values["offset"][0], y=values["offset"][1], z=0.0)
        spec.start_center = Pose2D(
            x=values["start_center"][0], y=values["start_center"][1],
            theta=values["start_center"][2])
        spec.goal_center = Pose2D(
            x=values["goal_center"][0], y=values["goal_center"][1],
            theta=values["goal_center"][2])
        spec.goal_heading_free = bool(values["goal_heading_free"])
        spec.body_length_m = float(values["body_length_m"])
        spec.body_width_m = float(values["body_width_m"])
        message.vehicles.append(spec)
    for x, y, theta in fields["formation_poses"]:
        message.formation_poses.append(Pose2D(x=x, y=y, theta=theta))
    times_ns = fields.get("times_ns")
    if times_ns is not None:
        if len(times_ns) != len(fields["times_from_start"]):
            raise ValueError("times_ns differs from the common time axis")
        for value in times_ns:
            secs, nsecs = divmod(int(value), 1000000000)
            message.times_from_start.append(duration_message(secs, nsecs))
    else:
        for value in fields["times_from_start"]:
            message.times_from_start.append(
                duration_from_seconds(float(value)))
    for values in fields["trajectories"]:
        trajectory = FormationVehicleTrajectory()
        trajectory.vehicle_id = int(values["vehicle_id"])
        point_times_ns = values.get("point_times_ns")
        if point_times_ns is not None and \
                len(point_times_ns) != len(values["points"]):
            raise ValueError("point_times_ns differs from the point count")
        for index, (time, x, y, theta, speed, yaw_rate, mode,
                    control) in enumerate(values["points"]):
            point = FormationTimedPoint()
            if point_times_ns is not None:
                secs, nsecs = divmod(int(point_times_ns[index]),
                                     1000000000)
                point.time_from_start = duration_message(secs, nsecs)
            else:
                point.time_from_start = duration_from_seconds(float(time))
            point.center_pose = Pose2D(x=float(x), y=float(y),
                                       theta=float(theta))
            point.center_speed_mps = float(speed)
            point.center_yaw_rate_radps = float(yaw_rate)
            point.mode = int(mode)
            point.mode_control = float(control)
            trajectory.points.append(point)
        message.trajectories.append(trajectory)
    return message


def _load_limits(ros, parameter):
    values = ros.param(parameter, {})
    if not isinstance(values, dict):
        raise ValueError("{} must be a dictionary".format(parameter))
    return FormationTimingLimits(**values)


class UniformPlannerNode(object):
    """Admission state machine, worker thread and the atomic envelope."""

    def __init__(self, ros, plan=plan_uniform, *, backend_hooks=None):
        self.ros = ros
        self._lock = threading.RLock()
        self._plan = plan
        if backend_hooks is None:
            backend_hooks = _backend_hooks_from_parameters(ros)
        self._backend_hooks = backend_hooks
        if backend_hooks is None:
            self.backend = PLAYBACK_PLAN_BACKEND
            self.model_version = MODEL_VERSION
            self.semantic_version = PLAYBACK_PLAN_SCHEMA
            self.certification = CERTIFICATION
        else:
            self.backend = str(backend_hooks.backend)
            self.model_version = str(backend_hooks.model_version)
            self.semantic_version = str(backend_hooks.semantic_version)
            self.certification = str(
                getattr(backend_hooks, "certification", ""))
        self.frame_id = self.ros.param("~frame_id", "map")
        if self.frame_id != "map":
            raise ValueError("~frame_id must be 'map'")
        self.timing_limits = _load_limits(self.ros, "~timing_limits")
        cargo = self.ros.param("~cargo_footprint", {})
        if not isinstance(cargo, dict):
            raise ValueError("~cargo_footprint must be a dictionary")
        self.cargo_footprint = CargoFootprint(**cargo) if cargo else None
        self.successor_template = SuccessorConfig(
            xy_bin_m=float(self.ros.param("~xy_bin", 0.20)),
            psi_bin_rad=float(self.ros.param("~psi_bin", math.radians(10.0))),
            heading_bin_rad=float(self.ros.param(
                "~heading_bin", math.radians(10.0))),
            control_bin=float(self.ros.param("~control_bin", 0.01)),
            arc_step_rad=float(self.ros.param(
                "~arc_step_rad", math.radians(10.0))),
            translation_step_m=float(self.ros.param(
                "~translation_step", 1.0)),
            # Frozen contract: 0.02 m / 1 deg goal and terminal heading
            # tolerances.  Reference the FR03/FR04 configuration defaults
            # instead of repeating the numbers in this node.
            goal_xy_tol_m=float(self.ros.param(
                "~goal_xy_tol", SuccessorConfig.goal_xy_tol_m)),
            goal_formation_heading_free=self.ros.param("~goal_formation_heading_free", False),
            goal_psi_tol_rad=float(self.ros.param(
                "~goal_psi_tol", SuccessorConfig.goal_psi_tol_rad)),
            goal_heading_tol_rad=float(self.ros.param(
                "~goal_heading_tol", SuccessorConfig.goal_heading_tol_rad)),
            sweep_spacing_m=float(self.ros.param("~sweep_spacing", 0.01)),
            max_samples_per_segment=int(self.ros.param(
                "~max_samples_per_segment", 200000)),
            collision_margin_m=float(self.ros.param(
                "~collision_margin", 0.05)),
            timing_limits=self.timing_limits)
        # Frozen FR04 search budget: 10000 expansions / 30 s.
        self.refinement_budget_s = float(self.ros.param("~background_refinement_budget_s", 0.0))
        if not math.isfinite(self.refinement_budget_s) or self.refinement_budget_s < 0.0:
            raise ValueError("background_refinement_budget_s must be nonnegative and finite")
        self.reuse_terminal_certificate = bool(self.ros.param("~reuse_terminal_certificate", False))
        self.reachability_retime = bool(self.ros.param("~reachability_retime", False))
        self.execution_without_playback = (
            self.backend == PLAYBACK_PLAN_BACKEND
            and bool(self.ros.param("~execution_without_playback", False)))
        self._refinement_thread = None
        self.search_config = GraphSearchConfig(
            max_expansions=int(self.ros.param(
                "~max_expansions", GraphSearchConfig.max_expansions)),
            max_seconds=float(self.ros.param(
                "~max_seconds", GraphSearchConfig.max_seconds)),
            heuristic_weight=float(self.ros.param(
                "~heuristic_weight", 3.0)),
            rotation_weight=float(self.ros.param("~rotation_weight", 3.0)),
            direct_goal_translation=self.ros.param("~direct_goal_translation", False),
            global_translation_route=self.ros.param("~global_translation_route", True),
            heading_aligned_route=self.ros.param("~heading_aligned_route", True),
            heading_transition_route=self.ros.param("~heading_transition_route", True),
            continuous_a_refinement=(self.refinement_budget_s == 0.0
                                     and self.ros.param("~continuous_a_refinement", False)),
            formation_raster=self.ros.param("~formation_raster", True),
            native_frontier=self.ros.param("~native_frontier", False),
            convex_corridor=self.ros.param("~convex_corridor", False),
            prefer_ackermann=self.ros.param("~prefer_ackermann", True),
            preview_checks=self.ros.param("~preview_checks", True))
        self.playback_start_timeout_s = float(self.ros.param(
            "~playback_start_timeout_s", 5.0))
        self.playback_finish_grace_s = float(self.ros.param(
            "~playback_finish_grace_s", 3.0))
        self.planning_timeout_s = float(self.ros.param(
            "~planning_timeout_s", 0.0))
        if self.planning_timeout_s <= 0.0:
            self.planning_timeout_s = self.search_config.max_seconds + 15.0
        self.virtual_a_planning_budget_s = None
        if self._backend_hooks is not None:
            # The G4 solve has a 60 s soft internal budget.  The watchdog is an
            # explicit wall-clock tolerance outside that budget; it is not a
            # second, shorter budget that would reject a still-valid solve.
            self.virtual_a_planning_budget_s = float(self.ros.param(
                "~virtual_a_planning_budget_s", 60.0))
            self.planning_timeout_s = float(self.ros.param(
                "~virtual_a_planning_timeout_s",
                self.virtual_a_planning_budget_s + 15.0))
            if (not math.isfinite(self.virtual_a_planning_budget_s) or
                    self.virtual_a_planning_budget_s <= 0.0 or
                    not math.isfinite(self.planning_timeout_s) or
                    self.planning_timeout_s <= 0.0):
                raise ValueError(
                    "virtual_rigid_a planning budgets must be positive and "
                    "finite")
        self.job_state_heartbeat_s = float(self.ros.param(
            "~job_state_heartbeat_s", 1.0))
        self.publication_dt = float(self.ros.param("~publication_dt", 0.02))
        self.max_publication_samples = int(self.ros.param(
            "~max_publication_samples", 200001))
        self.watchdog_s = float(self.ros.param("~watchdog_period", 0.5))
        for name in ("playback_start_timeout_s", "playback_finish_grace_s",
                     "planning_timeout_s", "job_state_heartbeat_s",
                     "publication_dt", "watchdog_s"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("~{} must be positive and finite".format(
                    name))

        self._state = IDLE
        self._state_plan = None
        self._generation = 0
        self._last_accepted_plan = None
        self._requests_seen = 0
        self._busy_rejections = 0
        self._grid = None
        self._map_signature = None
        self._map_frame_id = None
        self._map_error = None
        self._request_map_signature = None
        self._active_model_sha256 = None
        self._active_request_sha256 = None
        self._active_digest = None
        self._active_formation_id = None
        self._active_status_word = None
        self._published_wall = None
        self._published_duration = None
        self._published_stamp = None
        self._playback_seen = False
        self._playback_started_wall = None
        self._plan_started_wall = None
        self._last_status = None
        self._last_quality = None
        self._last_result = None
        self._last_heartbeat = 0.0
        # G5 virtual_rigid_a experiment state.  ``_initial_context`` is the
        # planner-owned reset or verified-terminal record used by the next
        # request; it is never inferred from appearance.
        self._initial_context = None
        self._reset_serial = 0
        self._initial_reset_ready = False
        self._worker_active = False
        self._virtual_snapshot = None
        self._virtual_request = None
        self._virtual_map_request = None
        self._virtual_context = None
        self._virtual_context_sha256 = None
        self._virtual_grid_sha256 = None
        self._virtual_admission = None
        # TASK 01 progressive planning display.  Each accepted stage document
        # replaces only its own layer; previously completed display data is
        # retained so a FAILED document freezes the last real geometry.
        self._progress_display_errors = 0
        self._progress_display_disabled = set()
        self._progress_layers = {}
        self._planning_progress = None

        self.status_pub = self.ros.publisher(
            STATUS_TOPIC, String, queue_size=1, latch=True)
        self.quality_pub = self.ros.publisher(
            QUALITY_TOPIC, String, queue_size=1, latch=True)
        self.job_pub = self.ros.publisher(
            JOB_STATE_TOPIC, String, queue_size=1, latch=True)
        self.playback_plan_pub = self.ros.publisher(
            PLAYBACK_PLAN_TOPIC, FormationPlaybackPlan, queue_size=1,
            latch=True)
        self.corridor_pub = (self.ros.publisher(
            "/formation/convex_corridor_guide", MarkerArray, queue_size=1, latch=True)
            if self.search_config.convex_corridor else None)
        self.uniform_progress_pub = (self.ros.publisher(
            "/formation/uniform_progress", String, queue_size=10, latch=True)
            if self.ros.param("~publish_uniform_progress", False) else None)
        # Progress stages are published in bursts: the "complete" document of
        # a stage is immediately followed by the "running" document of the
        # next one, and each carries the retained geometry.  A single-slot
        # send queue drops those complete documents (verified on the real
        # map05 run: guide/selection diagnostics never reached the playback
        # HUD).  A bounded multi-slot queue preserves order and delivery
        # without changing any admission, planning or publish gate; only the
        # latest document remains latched.
        self.planning_debug_pub = self.ros.publisher(
            PLANNING_DEBUG_TOPIC, String, queue_size=20, latch=True)
        self.ros.subscribe(MAP_TOPIC, OccupancyGrid, self.on_map, queue_size=1)
        self.ros.subscribe(REQUEST_TOPIC, FormationPlanRequest,
                         self.on_request, queue_size=1)
        self.ros.subscribe(PLAYBACK_STATUS_TOPIC, String,
                         self.on_playback_status, queue_size=10)
        if self._backend_hooks is not None:
            # Only the new backend owns an explicit experiment reset.  The old
            # uniform path keeps its historical automatic fallback and never
            # claims to have observed a manual reset.
            self.ros.subscribe("/initialpose", PoseWithCovarianceStamped,
                             self.on_initial_pose, queue_size=1)
        else:
            self._cancel_planning_service = self.ros.create_service(
                Trigger, "/formation/cancel_pending_plan", self.cancel_pending_plan)
        self.timer = self.ros.create_timer(self.watchdog_s,
                                 lambda: self.on_timer(None))
        self._publish_job_state("ready")
        self.ros.log("info",
            "[uniform planner] autonomous V0.2 planner ready; backend=%s "
            "budget=%d expansions / %.1f s",
            self.backend, self.search_config.max_expansions,
            self.search_config.max_seconds)

    # ---- parameters / helpers -----------------------------------------

    def _now(self):
        try:
            now = self.ros.stamp()
            if now.sec or now.nanosec:
                return int(now.sec), int(now.nanosec)
        except Exception:
            pass
        return 1, 0

    def _map_payload(self):
        return {
            "map_ready": bool(self._grid is not None and
                              self._map_error is None),
            "map_signature": self._map_signature,
            "request_map_signature": self._request_map_signature,
            "map_frame_id": self._map_frame_id,
            "map_error": self._map_error,
        }

    def _base_status(self, node_state, status, reason, detail, *,
                     plan_id=None, formation_id=None, feasible=False,
                     executable=False, published=False, extra=None):
        document = {
            "schema": STATUS_SCHEMA,
            "backend": self.backend,
            "node_state": node_state,
            "job_state": self._state,
            "plan_id": (self._state_plan if plan_id is None else plan_id),
            "formation_id": formation_id,
            "status": status,
            "feasible": bool(feasible),
            "executable": bool(executable),
            "trajectory_published": bool(published),
            "playback_confirmed": bool(self._playback_seen),
            "reason": reason,
            "detail": detail,
            "requests_seen": int(self._requests_seen),
            "busy_rejections": int(self._busy_rejections),
            "model_version": (getattr(self, "_active_model_version", None)
                              or getattr(self, "model_version", MODEL_VERSION)),
            "model_sha256": self._active_model_sha256,
            "semantic_version": getattr(
                self, "semantic_version", PLAYBACK_PLAN_SCHEMA),
            "request_sha256": self._active_request_sha256,
        }
        document.update(self._map_payload())
        if extra:
            document.update(extra)
        return document

    def _base_quality(self, event, status, reason, detail, *,
                      plan_id=None, formation_id=None, feasible=False,
                      executable=False, published=False, extra=None):
        document = {
            "schema": QUALITY_SCHEMA,
            "event": event,
            "backend": self.backend,
            "job_state": self._state,
            "plan_id": (self._state_plan if plan_id is None else plan_id),
            "formation_id": formation_id,
            "status": status,
            "reason": reason,
            "detail": detail,
            "feasible": bool(feasible),
            "executable": bool(executable),
            "trajectory_published": bool(published),
            "playback_confirmed": bool(self._playback_seen),
            "model_version": (getattr(self, "_active_model_version", None)
                              or getattr(self, "model_version", MODEL_VERSION)),
            "model_sha256": self._active_model_sha256,
            "semantic_version": getattr(
                self, "semantic_version", PLAYBACK_PLAN_SCHEMA),
            "request_sha256": self._active_request_sha256,
            "publication": self._publication_summary(),
        }
        document.update(self._map_payload())
        if extra:
            document.update(extra)
        return document

    def _publication_summary(self):
        if self._published_wall is None:
            return None
        return {
            "stamp": (None if self._published_stamp is None else {
                "secs": self._published_stamp[0],
                "nsecs": self._published_stamp[1]}),
            "duration_s": self._published_duration,
            "model_sha256": self._active_model_sha256,
            "playback_confirmed": bool(self._playback_seen),
        }

    def _publish_status(self, document):
        self._last_status = document
        self.status_pub.publish(String(data=_json(document)))

    def _publish_quality(self, document):
        self._last_quality = document
        self.quality_pub.publish(String(data=_json(document)))

    def _publish_planning_debug(self, document):
        self.planning_debug_pub.publish(String(data=_json(document)))

    def _clear_planning_debug(self, plan_id, generation, formation_id):
        # Replace the latched diagnostic with an explicit empty generation.
        # Playback also clears on the existing admit/fail/clear callbacks; this
        # payload prevents a restarted or late subscriber from redrawing a
        # corridor chain that no longer belongs to the current admission.
        self._publish_planning_debug({
            "state": "CLEARED",
            "plan_id": plan_id,
            "formation_id": formation_id,
            "backend": self.backend,
            "generation": int(generation),
            "grid_sha256": None,
            "planning_context_sha256": None,
            "model_sha256": None,
            "corridors": [],
        })

    @staticmethod
    def _display_from_corridor_round(identity, result):
        """Build the V2 display from the actual C2 round the solver used."""
        from fwis_backends.multi_4wis.formation_continuous_a.corridor import (
            CORRIDOR_DISPLAY_STATE,
            corridor_chain_from_payload,
            corridor_display_fields,
            corridor_portal_diagnostics,
        )
        footprint_cover = result.search.get("footprint_cover")
        regions = (footprint_cover.get("regions")
                   if isinstance(footprint_cover, dict) else None)
        if regions:
            return dict(
                identity, state=CORRIDOR_DISPLAY_STATE, route_id=0,
                route_source="footprint_seed", corridor_used=True,
                chain_digest=hashlib.sha256(_json(regions).encode()).hexdigest(),
                regions=[dict(region, region_id=index)
                         for index, region in enumerate(regions)])
        corridor = result.search.get("corridor")
        rounds = (corridor.get("rounds")
                  if isinstance(corridor, dict) else None)
        if not isinstance(rounds, (list, tuple)) or not rounds:
            return None
        current = rounds[-1]
        if not isinstance(current, dict):
            return None
        route_index = current.get("route_index")
        routes = getattr(result, "routes", ())
        if (not isinstance(route_index, int) or isinstance(route_index, bool)
                or route_index < 0 or route_index >= len(routes)):
            return None
        raw_route = routes[route_index]
        guide = current.get("guide")
        if not isinstance(guide, dict):
            return None
        if (raw_route.grid_sha256 != result.grid_sha256
                or guide.get("grid_sha256") != result.grid_sha256):
            return None
        chain = corridor_chain_from_payload(current.get("chain"))
        identity = dict(identity)
        identity["route_id"] = int(route_index)
        display = corridor_display_fields(
            chain, identity,
            raw_route_points=raw_route.points,
            guide_points=guide.get("points"),
            guide_quality=guide.get("quality"),
            guide_source=guide.get("source"),
            corridor_used=True)
        diagnostics = corridor_portal_diagnostics(
            current,
            guide_points=guide.get("points"),
            guide_source=guide.get("source"))
        if diagnostics is not None:
            display["portal_diagnostics"] = diagnostics
        return display

    # ---- progressive planning display (TASK 01 Step 1) ----------------

    def _progress_disabled_set(self):
        disabled = getattr(self, "_progress_display_disabled", None)
        if not isinstance(disabled, set):
            disabled = set()
            self._progress_display_disabled = disabled
        return disabled

    def _progress_layers_for(self, generation):
        layers = getattr(self, "_progress_layers", None)
        if not isinstance(layers, dict):
            layers = {}
            self._progress_layers = layers
        generation = int(generation)
        current = layers.get(generation)
        if not isinstance(current, dict):
            current = {"astar": None, "corridors": None, "candidates": None}
            layers[generation] = current
        return current

    def _record_progress_display_error(self, generation, plan_id, layer, exc):
        self._progress_display_errors = int(
            getattr(self, "_progress_display_errors", 0)) + 1
        self.ros.log("error",
            "[uniform planner] PROGRESS_DISPLAY_ERROR plan %s generation %s "
            "layer=%s: %s: %s; planning and audits continue",
            plan_id, generation, layer, type(exc).__name__, exc)

    def _planning_identity(self, generation, plan_id, snapshot, state,
                           model_sha256=None):
        """Admission identity for one progressive planning document."""
        admission = getattr(self, "_virtual_admission", None)
        if not isinstance(admission, Mapping):
            return None
        if snapshot is None or snapshot.formation_id != admission.get(
                "formation_id"):
            return None
        for name in ("request_sha256", "grid_sha256",
                     "planning_context_sha256"):
            if not _is_sha256_text(admission.get(name)):
                return None
        return {
            "schema": PLANNING_DEBUG_SCHEMA,
            "state": state,
            "plan_id": int(plan_id),
            "formation_id": snapshot.formation_id,
            "backend": self.backend,
            "generation": int(generation),
            "request_sha256": admission["request_sha256"],
            "grid_sha256": admission["grid_sha256"],
            "planning_context_sha256": admission["planning_context_sha256"],
            "model_sha256": model_sha256,
        }

    def _corridor_displays(self, generation, plan_id, snapshot, view,
                           identity):
        """The actual display payload for the current corridor round.

        Reuses ``_display_from_corridor_round()`` so corridor geometry is never
        copied here.  Returns ``None`` (and records PROGRESS_DISPLAY_ERROR) if
        the display cannot be built; the caller then clears only the
        corridor/guide layer and planning continues.
        """
        from fwis_backends.multi_4wis.formation_continuous_a.corridor import (
            CORRIDOR_DISPLAYS_KEY,
            CORRIDOR_DISPLAY_STATE,
        )
        try:
            if view is None:
                raise ValueError("corridor display needs a real result view")
            for name in ("grid_sha256", "planning_context_sha256"):
                value = getattr(view, name, None)
                if value is not None and value != identity[name]:
                    raise ValueError(
                        "display view {} differs from the admission".format(
                            name))
            display_identity = dict(identity)
            display_identity["state"] = CORRIDOR_DISPLAY_STATE
            display = self._display_from_corridor_round(
                display_identity, view)
            displays = ([] if display is None else [display])
            if not displays:
                displays = view.search.get(CORRIDOR_DISPLAYS_KEY)
            if not isinstance(displays, (list, tuple)) or not displays:
                raise ValueError(
                    "no actual corridor display exists for this round")
            checked = []
            for entry in displays:
                if not isinstance(entry, Mapping):
                    raise ValueError("corridor display must be an object")
                if entry.get("state") != CORRIDOR_DISPLAY_STATE:
                    raise ValueError("corridor display state differs")
                for name in ("plan_id", "formation_id", "backend",
                             "generation", "grid_sha256",
                             "planning_context_sha256", "model_sha256"):
                    if entry.get(name) != identity[name]:
                        raise ValueError(
                            "corridor display {} differs from the "
                            "admission".format(name))
                if entry.get("chain_digest") is None:
                    raise ValueError("corridor display lacks a chain digest")
                if not entry.get("regions"):
                    raise ValueError("corridor display has no regions")
                checked.append(dict(entry))
        except (KeyError, TypeError, ValueError) as exc:
            self._record_progress_display_error(
                generation, plan_id, "corridors", exc)
            return None
        return checked

    @staticmethod
    def _astar_payload(astar):
        """JSON-safe copy of the real raw A* routes (display only)."""
        if not isinstance(astar, (list, tuple)):
            raise ValueError("astar payload must be a list")
        entries = []
        for entry in astar:
            if not isinstance(entry, Mapping):
                raise ValueError("astar route must be an object")
            route_id = entry.get("route_id")
            if (not isinstance(route_id, int) or isinstance(route_id, bool)
                    or route_id < 0):
                raise ValueError("astar route_id must be non-negative")
            points = entry.get("points")
            if not isinstance(points, (list, tuple)) or len(points) < 2:
                raise ValueError("astar route needs at least two points")
            copied = []
            for point in points:
                if not isinstance(point, (list, tuple)) or len(point) != 2:
                    raise ValueError("astar point must be a pair")
                copied.append([
                    _finite(point[0], "astar point x"),
                    _finite(point[1], "astar point y")])
            entries.append({"route_id": int(route_id), "points": copied})
        return entries

    def _candidate_payload(self, path, classification, plan_id, status,
                           failure):
        shapes = getattr(self._virtual_request, "vehicle_shapes", None)
        return sample_candidate_path(
            path, shapes, classification=classification,
            source="plan {} {}".format(plan_id, self.backend),
            status=str(status or ""), failure=failure)

    def _progress_document(self, generation, plan_id, snapshot, event):
        """Validate one pipeline event and build its progressive document.

        Identity, stage order and admission digests fail closed: a late or
        mismatched event returns ``None`` and never clears an existing layer.
        Display-only construction failures clear only the affected layer.
        """
        if not isinstance(event, Mapping):
            return None
        if event.get("schema") != PROGRESS_SCHEMA:
            return None
        stage = event.get("stage")
        state = event.get("state")
        if state not in (PLANNING, PLANNING_FAILED_STATE):
            # PUBLISHED is the ROS node's statement after the atomic envelope
            # is actually published; the pipeline may never claim it.
            return None
        if stage not in PROGRESS_STAGE_INDEX \
                and stage != PROGRESS_FAILED_STAGE:
            return None
        if (state == PLANNING_FAILED_STATE) != (
                stage == PROGRESS_FAILED_STAGE):
            return None
        stage_index = event.get("stage_index")
        if isinstance(stage_index, bool) or not isinstance(stage_index, int):
            return None
        if stage == PROGRESS_FAILED_STAGE:
            if not 0 <= stage_index <= len(PROGRESS_STAGES) - 1:
                return None
        elif stage_index != PROGRESS_STAGE_INDEX[stage]:
            return None
        if int(generation) != int(getattr(self, "_generation", -1)):
            return None
        if getattr(self, "_state", None) != PLANNING:
            return None
        if getattr(self, "_state_plan", None) != plan_id:
            return None
        if getattr(self, "_virtual_snapshot", None) is not snapshot:
            return None
        admission = getattr(self, "_virtual_admission", None)
        if not isinstance(admission, Mapping):
            return None
        for name in ("request_sha256", "grid_sha256",
                     "planning_context_sha256"):
            value = event.get(name)
            if not _is_sha256_text(value) or value != admission.get(name):
                return None
        map_signature = getattr(self, "_map_signature", None)
        request_signature = getattr(self, "_request_map_signature", None)
        if (map_signature is not None and request_signature is not None
                and map_signature != request_signature):
            return None
        elapsed = event.get("elapsed_s")
        if (isinstance(elapsed, bool)
                or not isinstance(elapsed, (int, float))
                or not math.isfinite(float(elapsed))
                or float(elapsed) < 0.0):
            elapsed = 0.0
        route_index = event.get("route_index")
        if isinstance(route_index, bool) or not isinstance(route_index, int):
            route_index = None
        route_count = event.get("route_count")
        if (isinstance(route_count, bool)
                or not isinstance(route_count, int) or route_count < 0):
            route_count = 0
        round_index = event.get("round_index")
        if isinstance(round_index, bool) or not isinstance(round_index, int):
            round_index = None
        metrics = _progress_metrics_copy(event.get("metrics"))
        if metrics is None:
            self._record_progress_display_error(
                generation, plan_id, "metrics",
                ValueError("progress metrics are not finite JSON"))
            metrics = {}
        reason = str(metrics.get("reason") or "")
        detail = str(metrics.get("detail") or "")
        if state == PLANNING_FAILED_STATE and not reason:
            reason = str(metrics.get("status") or "FAILED")
        identity = self._planning_identity(
            generation, plan_id, snapshot, state)
        if identity is None:
            return None
        document = dict(identity)
        document.update({
            "stage": stage,
            "stage_index": int(stage_index),
            "elapsed_s": float(elapsed),
            "route_index": route_index,
            "route_count": int(route_count),
            "round_index": round_index,
            "reason": reason,
            "detail": detail,
            "metrics": metrics,
        })
        layers = self._progress_layers_for(generation)
        display = event.get("diagnostic_display")
        if isinstance(display, Mapping):
            astar = display.get("astar")
            if astar is not None:
                try:
                    layers["astar"] = self._astar_payload(astar)
                except (TypeError, ValueError) as exc:
                    self._record_progress_display_error(
                        generation, plan_id, "astar", exc)
                    layers["astar"] = None
                    document["astar"] = None
            view = display.get("result")
            if view is not None:
                displays = self._corridor_displays(
                    generation, plan_id, snapshot, view, identity)
                layers["corridors"] = displays
                document["corridors"] = displays
        if "astar" not in document and layers["astar"] is not None:
            document["astar"] = layers["astar"]
        if "corridors" not in document and layers["corridors"] is not None:
            document["corridors"] = layers["corridors"]
        path = event.get("diagnostic_path")
        if path is not None and callable(getattr(path, "state_at_s", None)):
            try:
                payload = self._candidate_payload(
                    path, CANDIDATE_UNVERIFIED, plan_id,
                    metrics.get("status"), None)
            except (TypeError, ValueError) as exc:
                self._record_progress_display_error(
                    generation, plan_id, "candidates", exc)
                layers["candidates"] = None
                document["candidates"] = None
            else:
                layers["candidates"] = payload
                document["candidates"] = payload
        elif layers["candidates"] is not None:
            document["candidates"] = layers["candidates"]
        self._planning_progress = {
            "stage": stage,
            "stage_index": int(stage_index),
            "state": state,
            "elapsed_s": float(elapsed),
            "route_index": route_index,
            "route_count": int(route_count),
            "round_index": round_index,
            "reason": reason,
        }
        return document

    def _on_virtual_progress(self, generation, plan_id, snapshot, event):
        """Worker-thread progress callback: display only, never planning."""
        disabled = self._progress_disabled_set()
        if int(generation) in disabled:
            return
        try:
            with self._lock:
                if int(generation) in disabled:
                    return
                document = self._progress_document(
                    generation, plan_id, snapshot, event)
                if document is None:
                    return
                self._publish_planning_debug(document)
        except Exception as exc:
            # An unexpected document-level failure disables later display
            # callbacks for this request only; the pipeline and its audits
            # continue and the final terminal document still publishes.
            disabled.add(int(generation))
            self._progress_display_errors = int(
                getattr(self, "_progress_display_errors", 0)) + 1
            self.ros.log("error",
                "[uniform planner] PROGRESS_DISPLAY_ERROR plan %s "
                "generation %s: %s: %s; further progress display for this "
                "request is disabled, planning and audits continue",
                plan_id, generation, type(exc).__name__, exc)

    def _emit_request_admitted(self, generation, plan_id, snapshot, admission):
        event = {
            "schema": PROGRESS_SCHEMA,
            "stage": "REQUEST_ADMITTED",
            "stage_index": int(PROGRESS_STAGE_INDEX["REQUEST_ADMITTED"]),
            "state": PLANNING,
            "elapsed_s": 0.0,
            "route_index": None,
            "route_count": 0,
            "round_index": None,
            "request_sha256": admission["request_sha256"],
            "grid_sha256": admission["grid_sha256"],
            "planning_context_sha256": admission["planning_context_sha256"],
            "metrics": {"phase": "admitted"},
            "diagnostic_path": None,
            "diagnostic_display": {"astar": None, "result": None},
        }
        self._on_virtual_progress(generation, plan_id, snapshot, event)

    def _progress_stage_snapshot(self):
        progress = getattr(self, "_planning_progress", None)
        if not isinstance(progress, Mapping):
            return 0, None, 0, None
        stage_index = progress.get("stage_index")
        if (isinstance(stage_index, int) and not isinstance(stage_index, bool)
                and 0 <= stage_index <= len(PROGRESS_STAGES) - 1):
            pass
        else:
            stage_index = 0
        route_index = progress.get("route_index")
        if isinstance(route_index, bool) or not isinstance(route_index, int):
            route_index = None
        route_count = progress.get("route_count")
        if (isinstance(route_count, bool)
                or not isinstance(route_count, int) or route_count < 0):
            route_count = 0
        round_index = progress.get("round_index")
        if isinstance(round_index, bool) or not isinstance(round_index, int):
            round_index = None
        return int(stage_index), route_index, int(route_count), round_index

    def _publish_planning_failed(self, generation, plan_id, snapshot, result,
                                 reason, detail, layer, wall_time_s):
        """Final FAILED document: freeze every layer already drawn."""
        if getattr(self, "_backend_hooks", None) is None:
            return
        identity = self._planning_identity(
            generation, plan_id, snapshot, PLANNING_FAILED_STATE)
        if identity is None:
            return
        stage_index, route_index, route_count, round_index = \
            self._progress_stage_snapshot()
        wall_time_s = float(wall_time_s)
        document = dict(identity)
        document.update({
            "stage": PROGRESS_FAILED_STAGE,
            "stage_index": int(stage_index),
            "elapsed_s": wall_time_s,
            "route_index": route_index,
            "route_count": int(route_count),
            "round_index": round_index,
            "reason": str(reason or ""),
            "detail": str(detail or ""),
            "metrics": {
                "phase": "failed",
                "status": str(getattr(result, "status", "") or ""),
                "reason": str(reason or ""),
                "detail": str(detail or ""),
                "first_failure_layer": str(layer or ""),
                "wall_time_s": wall_time_s,
            },
        })
        layers = self._progress_layers_for(generation)
        if layers["astar"] is not None:
            document["astar"] = layers["astar"]
        if layers["corridors"] is not None:
            document["corridors"] = layers["corridors"]
        candidates = layers["candidates"]
        failure = failure_point_from_result(result)
        path = getattr(result, "path", None)
        if path is not None and callable(getattr(path, "state_at_s", None)):
            try:
                candidates = self._candidate_payload(
                    path, CANDIDATE_UNVERIFIED, plan_id,
                    getattr(result, "status", ""), failure)
                layers["candidates"] = candidates
            except (TypeError, ValueError) as exc:
                self._record_progress_display_error(
                    generation, plan_id, "candidates", exc)
                candidates = None
        elif candidates is not None and failure is not None:
            # The retained candidate is real; the failure point is a separate
            # real audit coordinate.  Keep both, labelled by their sources.
            candidates = dict(candidates)
            candidates["failure"] = failure
        if candidates is not None:
            document["candidates"] = candidates
        else:
            document["candidates"] = None
            if failure is not None:
                document["failure"] = failure
        self._planning_progress = {
            "stage": PROGRESS_FAILED_STAGE,
            "stage_index": int(stage_index),
            "state": PLANNING_FAILED_STATE,
            "elapsed_s": wall_time_s,
            "route_index": route_index,
            "route_count": int(route_count),
            "round_index": round_index,
            "reason": str(reason or ""),
        }
        try:
            self._publish_planning_debug(document)
        except (TypeError, ValueError) as exc:
            self._record_progress_display_error(
                generation, plan_id, "document", exc)

    def _publish_planning_published(self, generation, plan_id, snapshot,
                                    result, model_sha256, wall_time_s):
        """PUBLISHED document, after the atomic playback envelope exists."""
        identity = self._planning_identity(
            generation, plan_id, snapshot, PLANNING_PUBLISHED_STATE,
            model_sha256=model_sha256)
        if identity is None:
            return
        stage_index, route_index, route_count, round_index = \
            self._progress_stage_snapshot()
        wall_time_s = float(wall_time_s)
        document = dict(identity)
        document.update({
            "stage": PLANNING_PUBLISHED_STATE,
            "stage_index": int(PROGRESS_STAGE_INDEX["PUBLISHED"]),
            "elapsed_s": wall_time_s,
            "route_index": route_index,
            "route_count": int(route_count),
            "round_index": round_index,
            "reason": str(getattr(result, "reason", "") or ""),
            "detail": str(getattr(result, "detail", "") or ""),
            "metrics": {
                "phase": "published",
                "status": str(getattr(result, "status", "") or ""),
                "wall_time_s": wall_time_s,
            },
        })
        metrics = getattr(result, "metrics", {})
        if "footprint_region_count" in metrics:
            for name in ("footprint_region_count", "footprint_obb_rows",
                         "footprint_selected"):
                document["metrics"][name] = metrics[name]
        layers = self._progress_layers_for(generation)
        if layers["astar"] is not None:
            document["astar"] = layers["astar"]
        displays = None
        if getattr(result, "status", None) != "NO_MOTION":
            displays = self._corridor_displays(
                generation, plan_id, snapshot, result, identity)
        document["corridors"] = displays
        layers["corridors"] = displays
        candidates = None
        path = getattr(result, "path", None)
        if path is not None and callable(getattr(path, "state_at_s", None)):
            try:
                candidates = self._candidate_payload(
                    path, CANDIDATE_CERTIFIED, plan_id,
                    getattr(result, "status", ""), None)
            except (TypeError, ValueError) as exc:
                self._record_progress_display_error(
                    generation, plan_id, "candidates", exc)
        document["candidates"] = candidates
        layers["candidates"] = candidates
        self._planning_progress = {
            "stage": PLANNING_PUBLISHED_STATE,
            "stage_index": int(PROGRESS_STAGE_INDEX["PUBLISHED"]),
            "state": PLANNING_PUBLISHED_STATE,
            "elapsed_s": wall_time_s,
            "route_index": route_index,
            "route_count": int(route_count),
            "round_index": round_index,
            "reason": str(getattr(result, "reason", "") or ""),
        }
        try:
            self._publish_planning_debug(document)
        except (TypeError, ValueError) as exc:
            self._record_progress_display_error(
                generation, plan_id, "document", exc)

    def _job_document(self, reason="", detail=""):
        document = {
            "schema": JOB_SCHEMA,
            "state": self._state,
            "backend": self.backend,
            "generation": int(self._generation),
            "plan_id": self._state_plan,
            "formation_id": self._active_formation_id,
            "model_sha256": self._active_model_sha256,
            "request_sha256": self._active_request_sha256,
            "reason": reason,
            "detail": detail,
        }
        document.update(self._map_payload())
        if self.execution_without_playback:
            document["execution_verified"] = bool(
                self._state == PLAYING and self._published_wall is not None
                and self._request_map_signature == self._map_signature)
        plan = getattr(getattr(self, "_last_result", None), "plan", None)
        if self.backend == PLAYBACK_PLAN_BACKEND and self._state == HOLD_END and plan is not None:
            terminal = plan.goal_qf
            document["uniform_terminal"] = {
                "qf": [terminal.x, terminal.y, terminal.psi],
                "headings": dict(plan.goal_headings),
            }
        if getattr(self, "_backend_hooks", None) is not None:
            context = getattr(self, "_initial_context", None)
            settings = getattr(self._backend_hooks, "settings", None)
            document.update({
                "initial_reset_ready": bool(getattr(
                    self, "_initial_reset_ready", False)),
                "initial_context_id": (
                    None if context is None else context.context_id),
                "initial_context_source": (
                    None if context is None else context.source),
                "initial_compatible": (
                    None if context is None else bool(context.compatible)),
                "worker_active": bool(getattr(
                    self, "_worker_active", False)),
                "planning_window_s": getattr(
                    self, "planning_timeout_s", None),
                "virtual_a_goal_policy": (
                    None if settings is None else settings.goal_policy),
            })
            request = getattr(self, "_virtual_request", None)
            map_request = getattr(self, "_virtual_map_request", None)
            context_record = getattr(self, "_initial_context", None)
            if request is not None:
                document.update({
                    "request_sha256": getattr(
                        map_request, "request_sha256", None),
                    "planning_context_sha256": getattr(
                        self, "_virtual_context_sha256", None),
                    "grid_sha256": getattr(
                        self, "_virtual_grid_sha256", None),
                    "planning_context": (
                        None if getattr(self, "_virtual_context", None)
                        is None else dict(self._virtual_context)),
                    "admission": (
                        None if getattr(self, "_virtual_admission", None)
                        is None else dict(self._virtual_admission)),
                })
            if context_record is not None:
                document["initial_context"] = context_record.as_dict()
        return document

    def _publish_job_state(self, reason="", detail=""):
        self.job_pub.publish(String(data=_json(
            self._job_document(reason, detail))))
        self._last_heartbeat = _time.monotonic()

    def _publication_times(self, plan):
        """Choose ROS nanoseconds before evaluating the analytic samples."""
        if plan.duration_s <= 0.0:
            return (0,), (0.0,)
        epsilon = 1e-6
        duration = float(plan.duration_s)
        times_ns = [0]
        for phase in plan.phases:
            t0 = min(max(float(phase.t0_s), 0.0), duration)
            end = min(max(float(phase.end_time), 0.0), duration)
            if end - t0 <= 2.0 * epsilon:
                continue
            # Keep each sample inside its phase after quantization.
            value = t0 + epsilon
            while value < end - epsilon:
                tick = int(round(value * 1e9))
                if tick - times_ns[-1] > 1:
                    times_ns.append(tick)
                value += self.publication_dt
        times_ns.append(int(round(duration * 1e9)))
        if len(times_ns) > self.max_publication_samples:
            raise ValueError(
                "publication grid has {} samples at dt={} s, above the "
                "{} sample limit".format(
                    len(times_ns), self.publication_dt,
                    self.max_publication_samples))
        times = [secs + nsecs / 1e9 for secs, nsecs in
                 (divmod(tick, 1000000000) for tick in times_ns)]
        times[-1] = min(times[-1], duration)
        return tuple(times_ns), tuple(times)

    # ---- map -----------------------------------------------------------

    def on_map(self, message):
        grid, error = grid_from_occupancy_grid(message)
        with self._lock:
            frame_id = getattr(message.header, "frame_id", None)
            if grid is None or error:
                self._grid = None
                self._map_signature = None
                self._map_frame_id = frame_id
                self._map_error = error or "invalid map"
                self.ros.log("error", "[uniform planner] /map rejected: %s",
                             self._map_error)
                return
            try:
                signature = grid_signature(grid)
            except Exception as exc:
                self._grid = None
                self._map_signature = None
                self._map_frame_id = frame_id
                self._map_error = "invalid map: {!r}".format(exc)
                self.ros.log("error", "[uniform planner] /map rejected: %s",
                             self._map_error)
                return
            if self._state in BUSY_STATES:
                self.ros.log("warning",
                    "[uniform planner] /map changed while job %s is %s; the "
                    "current job keeps the map snapshot it was admitted with",
                    self._state_plan, self._state)
            self._grid = grid
            self._map_signature = signature
            self._map_frame_id = frame_id
            self._map_error = None
            self.ros.log("info",
                "[uniform planner] exact map %dx%d res=%.3f origin=(%.2f, "
                "%.2f) signature=%s",
                grid.width, grid.height, grid.res, grid.ox, grid.oy,
                signature[:16])

    # ---- virtual experiment reset -------------------------------------

    def _next_context_id(self, kind):
        self._reset_serial = int(getattr(self, "_reset_serial", 0)) + 1
        return "virtual-a-{}-{}-{}".format(
            kind, self._state_plan, self._reset_serial)

    def on_initial_pose(self, message):
        """Record an explicit 2D Pose Estimate for the new A experiment.

        This is an observation of the user's command, not a measurement of the
        physical wheel state.  It is accepted only while the node is not busy;
        a rejected pose never replaces the current confirmed context.
        """
        if getattr(self, "_backend_hooks", None) is None:
            return
        from fwis_backends.multi_4wis.formation import FormationPose
        from fwis_backends.multi_4wis.formation_continuous_a.ros_adapter import (
            VirtualAInitialContext)
        with self._lock:
            if (self._state in BUSY_STATES or
                    bool(getattr(self, "_worker_active", False))):
                detail = ("a job is already planning or playing; the 2D Pose "
                          "Estimate was not consumed and the active plan is "
                          "unchanged")
                self._busy_rejections += 1
                self._publish_status(self._base_status(
                    "RESET_REJECTED", "REJECTED", "BUSY", detail,
                    plan_id=self._state_plan, feasible=False,
                    executable=False, published=False,
                    extra={"admitted": False, "busy_rejected": True,
                           "reset_consumed": False}))
                self.ros.log("warning", "[uniform planner] %s", detail)
                return
            frame = str(getattr(message.header, "frame_id", "") or "")
            if frame not in ("", self.frame_id):
                detail = "initial pose frame {!r} is not {!r}".format(
                    frame, self.frame_id)
                self._publish_status(self._base_status(
                    "RESET_REJECTED", "REJECTED", "RESET_FRAME_REJECTED",
                    detail, plan_id=self._state_plan,
                    feasible=False, executable=False, published=False,
                    extra={"admitted": False, "reset_consumed": False}))
                self.ros.log("warning", "[uniform planner] %s", detail)
                return
            pose = message.pose.pose
            try:
                x = _finite(pose.position.x, "initialpose x")
                y = _finite(pose.position.y, "initialpose y")
                qx = _finite(pose.orientation.x, "initialpose qx")
                qy = _finite(pose.orientation.y, "initialpose qy")
                qz = _finite(pose.orientation.z, "initialpose qz")
                qw = _finite(pose.orientation.w, "initialpose qw")
                norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
                if norm < 1e-9:
                    raise ValueError("initialpose quaternion is degenerate")
                qx, qy, qz, qw = (qx / norm, qy / norm,
                                  qz / norm, qw / norm)
                sinr = 2.0 * (qw * qx + qy * qz)
                cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
                roll = math.atan2(sinr, cosr)
                sinp = 2.0 * (qw * qy - qz * qx)
                pitch = math.asin(max(-1.0, min(1.0, sinp)))
                if abs(roll) > 1e-6 or abs(pitch) > 1e-6:
                    raise ValueError(
                        "initialpose is not planar (roll={:.3g}, "
                        "pitch={:.3g})".format(roll, pitch))
                yaw = math.atan2(
                    2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz))
            except (TypeError, ValueError, AttributeError) as exc:
                detail = "invalid 2D Pose Estimate: {}".format(exc)
                self._publish_status(self._base_status(
                    "RESET_REJECTED", "REJECTED", "RESET_POSE_REJECTED",
                    detail, plan_id=self._state_plan,
                    feasible=False, executable=False, published=False,
                    extra={"admitted": False, "reset_consumed": False}))
                self.ros.log("warning", "[uniform planner] %s", detail)
                return
            context_id = self._next_context_id("reset")
            context = VirtualAInitialContext(
                source="rviz_reset", context_id=context_id,
                qf=FormationPose(x, y, yaw))
            self._initial_context = context
            self._initial_reset_ready = True
            detail = ("explicit virtual experiment reset recorded at "
                      "F=({:.6g}, {:.6g}, {:.6g}); this is a declared "
                      "A-straight stationary initial state, not a physical "
                      "wheel measurement".format(x, y, yaw))
            self._publish_status(self._base_status(
                "RESET_CONFIRMED", "IDLE", "OK", detail,
                plan_id=self._state_plan, feasible=False,
                executable=False, published=False,
                extra={"admitted": False, "reset_consumed": True,
                       "initial_reset_ready": True,
                       "initial_context_id": context_id,
                       "initial_context_source": "rviz_reset",
                       "initial_pose": {"x": x, "y": y, "yaw": yaw}}))
            self._publish_job_state("RESET_CONFIRMED", detail)
            self.ros.log("info", "[uniform planner] %s", detail)

    # ---- request admission --------------------------------------------

    def _uniform_continuation(self, message, snapshot):
        parent_id = getattr(message, "continuation_plan_id", 0)
        digest = getattr(message, "continuation_model_sha256", "")
        if not parent_id and not digest:
            return snapshot, None
        plan = getattr(getattr(self, "_last_result", None), "plan", None)
        if (not parent_id or not digest or plan is None or self._state != HOLD_END
                or parent_id != self._state_plan or parent_id != plan.plan_id
                or digest != self._active_model_sha256
                or snapshot.formation_id != plan.formation_id
                or snapshot.model.offsets != plan.model.offsets
                or dict(snapshot.vehicle_shapes) != dict(plan.vehicle_shapes)):
            raise ValueError("continuation must identify the current confirmed terminal plan and formation")

        terminal = plan.goal_qf
        headings = dict(plan.goal_headings)

        def angle_error(a, b):
            return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

        if (math.hypot(snapshot.start.x - terminal.x, snapshot.start.y - terminal.y) > 1e-7
                or angle_error(snapshot.start.psi, terminal.psi) > 1e-7
                or any(angle_error(snapshot.start_headings[i], headings[i]) > 1e-7
                       for i in plan.vehicle_ids)):
            raise ValueError("continuation start does not match the confirmed terminal pose and headings")
        edge = plan.edges[-1] if plan.edges else plan.initial_edge
        return replace(snapshot, start=terminal, start_headings=headings), edge

    def cancel_pending_plan(self, _request, _response):
        """Allow an explicit RViz reset to supersede an unpublished search."""
        with self._lock:
            if self._state == PLAYING or (
                    self._state == PLANNING and self._published_wall is not None):
                return Trigger.Response(success=False, message="playback has already been published")
            if self._state != PLANNING:
                return Trigger.Response(success=True, message="no pending plan")
            plan_id = self._state_plan
            self._planning_cancel.set()
            self._generation += 1
            self._state = IDLE
            self._request_map_signature = None
            self._publish_no_envelope(plan_id, self._active_formation_id, "MANUAL_RESET")
            self._publish_job_state("MANUAL_RESET", "operator reset cancelled the pending search")
            self.ros.log("info", "[uniform planner] cancelled pending plan %s on manual reset", plan_id)
            return Trigger.Response(success=True, message="pending plan cancelled")

    def on_request(self, message):
        with self._lock:
            self._requests_seen += 1
            request_plan_id = getattr(message, "plan_id", None)
            if not isinstance(request_plan_id, int):
                request_plan_id = None
            if (self._state in BUSY_STATES or
                    bool(getattr(self, "_worker_active", False))):
                self._busy_rejections += 1
                self.ros.log("warning",
                    "[uniform planner] busy: rejecting plan %s while job %s "
                    "is %s; no preemption and no override is consumed",
                    request_plan_id, self._state_plan, self._state)
                status = self._base_status(
                    "BUSY_REJECTED", "REJECTED", "BUSY",
                    "a job is already planning or playing; this request was "
                    "not admitted and no start override was consumed",
                    plan_id=request_plan_id,
                    formation_id=getattr(message, "formation_id", None),
                    extra={"admitted": False, "busy_rejected": True,
                           "rejected_plan_id": request_plan_id,
                           "active_plan_id": self._state_plan,
                           "active_job_state": self._state})
                self._publish_status(status)
                self._publish_quality(self._base_quality(
                    "request", "REJECTED", "BUSY", status["detail"],
                    plan_id=request_plan_id,
                    formation_id=getattr(message, "formation_id", None),
                    extra={"admitted": False, "busy_rejected": True}))
                return
            validation = validate_formation_request(message)
            if not validation.valid:
                self._state = FAILED
                self._state_plan = request_plan_id
                reason = validation.reason or "INVALID_REQUEST"
                detail = "{} {}".format(validation.field, validation.detail)
                self.ros.log("error",
                    "[uniform planner] plan %s rejected before planning: %s",
                    request_plan_id, detail)
                self._publish_status(self._base_status(
                    "REQUEST_REJECTED", "INVALID", reason, detail,
                    plan_id=request_plan_id,
                    formation_id=getattr(message, "formation_id", None),
                    extra={"admitted": False, "valid": False,
                           "failure": {
                               "code": reason, "field": validation.field,
                               "vehicle_id": validation.vehicle_id,
                               "detail": validation.detail}}))
                self._publish_quality(self._base_quality(
                    "request", "INVALID", reason, detail,
                    plan_id=request_plan_id,
                    formation_id=getattr(message, "formation_id", None),
                    extra={"admitted": False, "valid": False}))
                self._publish_job_state(reason, detail)
                return
            snapshot = validation.snapshot
            plan_id = snapshot.plan_id
            if (self._last_accepted_plan is not None and
                    plan_id <= self._last_accepted_plan):
                self.ros.log("warning",
                    "[uniform planner] rejecting plan %s: not newer than the "
                    "last accepted plan %s", plan_id,
                    self._last_accepted_plan)
                self._publish_status(self._base_status(
                    "REQUEST_REJECTED", "REJECTED",
                    "DUPLICATE_OR_STALE_PLAN_ID",
                    "plan_id must identify the next experiment",
                    plan_id=plan_id, formation_id=snapshot.formation_id,
                    extra={"admitted": False, "valid": True,
                           "last_accepted_plan_id": self._last_accepted_plan}))
                return
            if getattr(self, "_backend_hooks", None) is not None:
                self._admit_virtual(snapshot)
                return
            free_flags = dict(snapshot.goal_heading_free)
            if any(free_flags.values()) and not all(free_flags.values()):
                self._state = FAILED
                self._state_plan = plan_id
                detail = ("mixed goal_heading_free is not representable by "
                          "the V0.2 uniform request contract; every vehicle "
                          "must share the same terminal-heading policy")
                self._publish_status(self._base_status(
                    "REQUEST_REJECTED", "INVALID", "MIXED_GOAL_HEADING_FREE",
                    detail, plan_id=plan_id,
                    formation_id=snapshot.formation_id,
                    extra={"admitted": False, "valid": True}))
                self._publish_quality(self._base_quality(
                    "request", "INVALID", "MIXED_GOAL_HEADING_FREE", detail,
                    plan_id=plan_id, formation_id=snapshot.formation_id,
                    extra={"admitted": False}))
                self._publish_job_state("MIXED_GOAL_HEADING_FREE", detail)
                return
            if self._grid is None or self._map_error is not None:
                self._state = FAILED
                self._state_plan = plan_id
                detail = self._map_error or "no exact /map has been received"
                self._publish_status(self._base_status(
                    "MAP_UNAVAILABLE", "INVALID", "MAP_UNAVAILABLE", detail,
                    plan_id=plan_id, formation_id=snapshot.formation_id,
                    extra={"admitted": False, "valid": True,
                           "planning_started": False}))
                self._publish_quality(self._base_quality(
                    "map", "INVALID", "MAP_UNAVAILABLE", detail,
                    plan_id=plan_id, formation_id=snapshot.formation_id,
                    extra={"admitted": False}))
                self._publish_no_envelope(plan_id, snapshot.formation_id,
                                          "MAP_UNAVAILABLE")
                self._publish_job_state("MAP_UNAVAILABLE", detail)
                return

            all_free = all(free_flags.values())
            try:
                snapshot, initial_edge = self._uniform_continuation(message, snapshot)
            except ValueError as exc:
                self._publish_status(self._base_status(
                    "REQUEST_REJECTED", "INVALID", "INVALID_CONTINUATION", str(exc),
                    plan_id=plan_id, formation_id=snapshot.formation_id,
                    extra={"admitted": False, "planning_started": False}))
                return
            successor_config = replace(
                self.successor_template,
                allow_reverse=snapshot.allow_reverse,
                goal_heading_free=all_free,
                timing_limits=self.timing_limits)
            self._planning_cancel = threading.Event()
            planning_request = UniformPlanningRequest(
                grid=self._grid,
                model=snapshot.model,
                vehicle_shapes=dict(snapshot.vehicle_shapes),
                start_qf=snapshot.start,
                start_headings=dict(snapshot.start_headings),
                goal_qf=snapshot.goal,
                goal_headings=(None if all_free
                               else dict(snapshot.goal_headings)),
                plan_id=plan_id,
                formation_id=snapshot.formation_id,
                successor_config=successor_config,
                search_config=self.search_config, initial_edge=initial_edge,
                cancelled=self._planning_cancel.is_set, cargo_footprint=self.cargo_footprint,
                reuse_terminal_certificate=self.reuse_terminal_certificate,
                reachability_retime=self.reachability_retime)

            self._generation += 1
            generation = self._generation
            self._state = PLANNING
            self._state_plan = plan_id
            self._last_accepted_plan = plan_id
            self._request_map_signature = self._map_signature
            self._active_model_sha256 = None
            self._active_model_version = None
            self._active_request_sha256 = None
            self._active_formation_id = snapshot.formation_id
            self._active_status_word = None
            self._playback_seen = False
            self._published_wall = None
            self._published_duration = None
            self._published_stamp = None
            self._playback_started_wall = None
            self._last_result = None
            self.ros.log("info",
                "[uniform planner] plan %s admitted from %s: start=(%.3f, "
                "%.3f, %.3f) goal=(%.3f, %.3f, %.3f) free_headings=%s "
                "reverse=%s", plan_id, snapshot.formation_id,
                snapshot.start.x, snapshot.start.y, snapshot.start.psi,
                snapshot.goal.x, snapshot.goal.y, snapshot.goal.psi,
                all_free, snapshot.allow_reverse)
            self._publish_job_state("planning")
            self._publish_status(self._base_status(
                "REQUEST_VALIDATED", "PLANNING", "OK",
                "request validated; the autonomous V0.2 search is running",
                plan_id=plan_id, formation_id=snapshot.formation_id,
                extra={"admitted": True, "valid": True,
                       "planning_started": True,
                       "continuation_plan_id": getattr(message, "continuation_plan_id", 0),
                       "wall_time_s": None}))
            self._publish_quality(self._base_quality(
                "request", "PLANNING", "OK",
                "request validated; no envelope exists before the audit",
                plan_id=plan_id, formation_id=snapshot.formation_id,
                extra={"admitted": True, "valid": True}))
            self._plan_started_wall = _time.monotonic()
            thread = threading.Thread(
                target=self._plan_worker,
                args=(generation, plan_id, snapshot, planning_request),
                name="uniform-plan-{}".format(plan_id), daemon=True)
            self._plan_thread = thread
            thread.start()

    # ---- planning worker ----------------------------------------------

    def _plan_worker(self, generation, plan_id, snapshot, request):
        started = _time.monotonic()
        publisher = getattr(self, "uniform_progress_pub", None)
        last_progress = {}

        def report_progress(stage, metrics):
            last_progress.update(metrics)
            if generation == self._generation:
                publisher.publish(String(data=_json(dict(last_progress, stage=stage, plan_id=plan_id))))

        try:
            result = self._plan(request, progress=report_progress) if publisher else self._plan(request)
        except Exception as exc:
            self._finish_exception(generation, plan_id, snapshot, started, exc)
            return
        with self._lock:
            if generation != self._generation or self._state != PLANNING:
                self.ros.log("warning",
                    "[uniform planner] discarding result for stale job %s",
                    plan_id)
                return
            if (self._request_map_signature is not None and
                    self._map_signature != self._request_map_signature):
                self._finish_failure(
                    plan_id, snapshot, result, started,
                    "MAP_VERSION_MISMATCH",
                    "the exact map changed while the plan was being built; "
                    "the result is not published",
                    layer="map")
                return
            if not result.executable:
                self._finish_failure(
                    plan_id, snapshot, result, started, result.reason,
                    result.detail, layer=result.first_failure_layer)
                return
            if publisher:
                report_progress("publication", {"elapsed_s": _time.monotonic() - started})
            self._finish_success(generation, plan_id, snapshot, result,
                                 started)
            if (self._published_wall is not None and self.refinement_budget_s > 0.0
                    and result.status == SEARCH_SUCCESS
                    and (self._refinement_thread is None or not self._refinement_thread.is_alive())):
                self._refinement_thread = threading.Thread(
                    target=self._refine_worker, args=(generation, request, result.plan),
                    name="uniform-refine-{}".format(plan_id), daemon=True)
                self._refinement_thread.start()

    def _refine_worker(self, generation, request, plan):
        """Record certified quality experiments without changing the executing plan."""
        started = _time.perf_counter()
        original_cancelled = request.cancelled
        admitted_map = request.grid

        def obsolete():
            return (generation != self._generation or self._grid is not admitted_map
                    or bool(original_cancelled and original_cancelled()))

        try:
            candidate = refine_certified_plan(
                replace(request, cancelled=obsolete), plan, started + self.refinement_budget_s,
                research_worker=True)
            outcome = "no_accepted_improvement"
            if obsolete():
                outcome = "stale"
            elif _time.perf_counter() >= started + self.refinement_budget_s:
                outcome = "budget_exhausted"
            elif candidate is not None:
                outcome = "certified_candidate_discarded_without_safe_switch"
            self.ros.log("info", "[uniform planner] background refinement for %s: %s wall=%.4f s",
                         request.plan_id, outcome, _time.perf_counter() - started)
        except Exception as exc:
            self.ros.log("warning", "[uniform planner] background refinement for %s failed: %r",
                         request.plan_id, exc)

    def _publish_corridor_guide(self, search, stamp):
        if self.corridor_pub is None:
            return
        guide = getattr(search, "corridor_guide", None) or {}
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = time_message(*stamp)
        marker.ns = "convex_corridor_guide"
        marker.type = Marker.LINE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = .04
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .1, .75, .8, .65
        for x0, y0, x1, y1 in guide.get("convex_regions_m", ()):
            ring = ((x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0))
            for start, end in zip(ring, ring[1:]):
                marker.points.extend(Point(x=x, y=y, z=.06) for x, y in (start, end))
        marker.action = Marker.ADD if marker.points else Marker.DELETE
        self.corridor_pub.publish(MarkerArray(markers=[marker]))

    # ---- virtual_rigid_a admission and worker -------------------------

    @staticmethod
    def _virtual_status_word(status):
        if status == "SUCCESS":
            return "FEASIBLE"
        if status == "NO_MOTION":
            return "NO_MOTION"
        return "REJECTED"

    def _admit_virtual(self, snapshot):
        """Adapt one raw request through the virtual hook surface.

        A structured rejection keeps the actual confirmed terminal or reset
        record, never consumes a start override (the bridge owns that) and
        does not start a worker.  No prior executable envelope is treated as
        the result of this request.
        """
        from fwis_backends.multi_4wis.formation_continuous_a.contracts import (
            VirtualAContractError)
        from fwis_backends.multi_4wis.formation_continuous_a.ros_adapter import (
            request_from_snapshot)
        plan_id = snapshot.plan_id
        context_sha = None
        try:
            map_request = self._backend_hooks.request_from_snapshot(
                snapshot, self._grid,
                getattr(self._backend_hooks, "settings", None),
                getattr(self, "_initial_context", None))
            request = map_request.request
            context = map_request.context()
            from fwis_backends.multi_4wis.formation_continuous_a.contracts import (
                planning_context_sha256, request_fingerprint)
            context = dict(context)
            context_sha = planning_context_sha256(context)
            grid_sha = map_request.grid_sha256
            request_sha = map_request.request_sha256
            if request_sha != request_fingerprint(request):
                raise VirtualAContractError(
                    "REQUEST_DIGEST_MISMATCH",
                    "map request digest differs from the physical request")
            admission = {
                "plan_id": request.plan_id,
                "formation_id": request.formation_id,
                "request_sha256": request_sha,
                "planning_context_sha256": context_sha,
                "grid_sha256": grid_sha,
                "initial_source": request.initial.source,
                "initial_context_id": request.initial.context_id,
                "goal_policy": request.goal_policy.value,
            }
        except VirtualAContractError as exc:
            self._state = FAILED
            self._state_plan = plan_id
            terminal = exc.location.get("terminal")
            extra = {
                "admitted": False, "valid": True,
                "planning_started": False, "retry_allowed": True,
                "rejection": exc.as_dict(),
            }
            if terminal is not None:
                extra["terminal"] = terminal
            self._publish_status(self._base_status(
                "REQUEST_REJECTED", "REJECTED", exc.reason, exc.detail,
                plan_id=plan_id, formation_id=snapshot.formation_id,
                feasible=False, executable=False, published=False,
                extra=extra))
            self._publish_quality(self._base_quality(
                "request", "REJECTED", exc.reason, exc.detail,
                plan_id=plan_id, formation_id=snapshot.formation_id,
                extra={"admitted": False, "retry_allowed": True}))
            self._publish_job_state(exc.reason, exc.detail)
            self.ros.log("error",
                "[uniform planner] plan %s virtual admission rejected: "
                "%s: %s", plan_id, exc.reason, exc.detail)
            return
        if request.plan_id != plan_id:
            raise RuntimeError("hook changed the admitted plan identity")
        self._generation += 1
        generation = self._generation
        self._state = PLANNING
        self._state_plan = plan_id
        self._last_accepted_plan = plan_id
        self._request_map_signature = self._map_signature
        self._active_model_sha256 = None
        self._active_request_sha256 = None
        self._active_formation_id = snapshot.formation_id
        self._active_status_word = None
        self._playback_seen = False
        self._published_wall = None
        self._published_duration = None
        self._published_stamp = None
        self._playback_started_wall = None
        self._last_result = None
        self._virtual_snapshot = snapshot
        self._virtual_request = request
        self._virtual_map_request = map_request
        self._virtual_context = context
        self._virtual_context_sha256 = context_sha
        self._virtual_grid_sha256 = grid_sha
        self._virtual_admission = admission
        self._worker_active = True
        # A just-admitted request has no executable result yet.  Clear the
        # latched display without pretending plan_id=0 belongs to this job.
        self._publish_no_envelope(0, "", "ADMITTED_PLANNING")
        self._clear_planning_debug(
            plan_id, generation, snapshot.formation_id)
        # A new admission is the only event that empties the progressive
        # planning layers; busy/stale rejections never touch them.
        self._progress_layers = {}
        self._planning_progress = None
        self._emit_request_admitted(
            generation, plan_id, snapshot, admission)
        self.ros.log("info",
            "[uniform planner] plan %s admitted for virtual_rigid_a: "
            "start=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f) "
            "initial=%s/%s policy=%s", plan_id, request.start_qf.x,
            request.start_qf.y, request.start_qf.psi, request.goal_qf.x,
            request.goal_qf.y, request.goal_qf.psi,
            request.initial.source, request.initial.context_id,
            request.goal_policy.value)
        self._publish_job_state("planning")
        self._publish_status(self._base_status(
            "REQUEST_VALIDATED", "PLANNING", "OK",
            "request validated; the autonomous G4 A-only solve is running",
            plan_id=plan_id, formation_id=snapshot.formation_id,
            extra={"admitted": True, "valid": True,
                   "planning_started": True, "wall_time_s": None,
                   "initial_source": request.initial.source,
                   "initial_context_id": request.initial.context_id,
                   "goal_policy": request.goal_policy.value,
                   "planning_window_s": self.planning_timeout_s}))
        self._publish_quality(self._base_quality(
            "request", "PLANNING", "OK",
            "request validated; no virtual envelope exists before the three "
            "G4 audits", plan_id=plan_id,
            formation_id=snapshot.formation_id,
            extra={"admitted": True, "valid": True}))
        self._plan_started_wall = _time.monotonic()
        thread = threading.Thread(
            target=self._virtual_plan_worker,
            args=(generation, plan_id, snapshot, map_request, admission),
            name="virtual-a-plan-{}".format(plan_id), daemon=True)
        self._plan_thread = thread
        thread.start()

    def _virtual_plan_worker(self, generation, plan_id, snapshot,
                             map_request, admission):
        started = _time.monotonic()
        callback = lambda event: self._on_virtual_progress(
            generation, plan_id, snapshot, event)
        try:
            result = self._backend_hooks.solve(
                map_request, progress_callback=callback)
        except Exception as exc:
            with self._lock:
                self._worker_active = False
            self._finish_exception(generation, plan_id, snapshot, started, exc)
            return
        with self._lock:
            self._worker_active = False
            if generation != self._generation or self._state != PLANNING:
                self.ros.log("warning",
                    "[uniform planner] discarding virtual result for stale "
                    "job %s", plan_id)
                return
            if (self._request_map_signature is not None and
                    self._map_signature != self._request_map_signature):
                self._finish_virtual_failure(
                    plan_id, snapshot, result, started,
                    "MAP_VERSION_MISMATCH",
                    "the exact map changed while the virtual plan was being "
                    "built; the result is not published", layer="map")
                return
            if not bool(getattr(result, "executable", False)):
                self._finish_virtual_failure(
                    plan_id, snapshot, result, started,
                    getattr(result, "reason", "") or "NOT_EXECUTABLE",
                    getattr(result, "detail", ""),
                    layer=getattr(result, "first_failure_layer", ""))
                return
            self._finish_virtual_success(
                generation, plan_id, snapshot, result, started)

    def _finish_virtual_failure(self, plan_id, snapshot, result, started,
                                reason, detail, layer="none"):
        self._state = FAILED
        self._state_plan = plan_id
        wall = _time.monotonic() - started
        status_word = self._virtual_status_word(
            str(getattr(result, "status", "REJECTED")))
        self.ros.log("error",
            "[uniform planner] virtual plan %s not executable: status=%s "
            "reason=%s layer=%s wall=%.3fs", plan_id,
            getattr(result, "status", None), reason, layer, wall)
        extra = {
            "admitted": True,
            "planning_started": True,
            "pipeline_status": getattr(result, "status", None),
            "first_failure_layer": layer,
            "wall_time_s": wall,
            "request_sha256": getattr(result, "request_sha256", ""),
            "planning_context_sha256": getattr(
                result, "planning_context_sha256", ""),
            "grid_sha256": getattr(result, "grid_sha256", ""),
            "virtual_result": self._backend_hooks.result_summary(result),
            "collision_claim": (
                "no executable envelope; the failed result does not certify "
                "geometry, dynamics or map collision"),
        }
        self._publish_status(self._base_status(
            "REJECTED", status_word, reason, detail,
            plan_id=plan_id, formation_id=snapshot.formation_id,
            feasible=False, executable=False, published=False, extra=extra))
        self._publish_quality(self._base_quality(
            "plan", status_word, reason, detail,
            plan_id=plan_id, formation_id=snapshot.formation_id,
            feasible=False, executable=False, published=False, extra=extra))
        self._publish_no_envelope(plan_id, snapshot.formation_id, reason)
        # Freeze the completed layers in one final FAILED document; do not
        # clear them (TASK 01 Step 1 failure-freeze rule).
        self._publish_planning_failed(
            self._generation, plan_id, snapshot, result, reason, detail,
            layer, wall)
        self._publish_job_state(reason, detail)

    def _finish_virtual_success(self, generation, plan_id, snapshot, result,
                                started):
        plan = result.plan
        verification_started = _time.monotonic()
        try:
            provisional = self._now()
            fields = self._backend_hooks.publication_fields(
                snapshot, result, provisional, self.publication_dt)
            if (int(fields.get("plan_id", -1)) != plan_id or
                    fields.get("formation_id") != snapshot.formation_id):
                raise ValueError(
                    "projected fields changed the plan identity")
            message = timed_message(fields)
            validation = validate_formation_timed_trajectory(message)
            if not validation.valid:
                raise ValueError(
                    "converted trajectory violates the ROS contract: {} {}"
                    .format(validation.field, validation.detail))
            model_json = plan.to_model_json()
            digest = result.model_sha256
            envelope = FormationPlaybackPlan()
            envelope.header.frame_id = fields["frame_id"]
            envelope.header.stamp = time_message(*fields["stamp"])
            envelope.plan_id = int(plan_id)
            envelope.formation_id = snapshot.formation_id
            envelope.backend = self.backend
            envelope.semantic_version = self.semantic_version
            envelope.model_version = self.model_version
            envelope.model_sha256 = digest
            envelope.model_json = model_json
            envelope.certification = self.certification
            envelope.trajectory = message
            embedded = fields_from_timed_message(message)
            if embedded["stamp"] != fields["stamp"]:
                raise ValueError("envelope stamp differs from its trajectory")
            expected_formation = None
            if str(getattr(result, "formation_corridor_scope", "f_only")) == \
                    "three_vehicle_union_v1":
                audit = dict(
                    getattr(result, "formation_corridor_audit", None) or {})
                search = result.search if isinstance(
                    result.search, dict) else {}
                corridor_search = search.get("corridor") or {}
                rounds = (corridor_search.get("rounds")
                          if isinstance(corridor_search, dict) else None)
                if not rounds:
                    raise ValueError(
                        "union result carries no corridor round chain")
                expected_formation = {
                    "scope": "three_vehicle_union_v1",
                    "planning_context_sha256":
                        result.planning_context_sha256,
                    "template": audit.get("template"),
                    "chain": rounds[-1].get("chain"),
                    "certificate": audit.get("certificate"),
                }
            # The sealed binding travels inside the atomic envelope: the
            # receiver re-verifies it against this model and the admitted
            # context, so it must not be a second message the transport can
            # reorder or drop.
            envelope.formation_binding_json = (
                "" if expected_formation is None
                else _json(expected_formation))
            verified = self._backend_hooks.verify_envelope_fields({
                "plan_id": int(plan_id),
                "formation_id": snapshot.formation_id,
                "backend": self.backend,
                "semantic_version": self.semantic_version,
                "model_version": self.model_version,
                "model_sha256": digest,
                "model_json": model_json,
                "certification": self.certification,
                "trajectory_fields": embedded,
                "expected_request": self._virtual_request,
                "expected_context": self._virtual_context,
                "expected_grid_sha256": self._virtual_grid_sha256,
                "expected_admission": self._virtual_admission,
                "expected_formation": expected_formation,
            })
            if verified.model_sha256 != digest:
                raise ValueError("verifier returned a different model digest")
            stamp = self._now()
            fields = dict(fields, stamp=stamp)
            message.header.stamp = time_message(*stamp)
            envelope.header.stamp = time_message(*stamp)
            envelope.trajectory = message
            if ((int(message.header.stamp.sec),
                 int(message.header.stamp.nanosec)) != stamp or
                    (int(envelope.header.stamp.sec),
                     int(envelope.header.stamp.nanosec)) != stamp):
                raise ValueError(
                    "restamped envelope header is inconsistent with the "
                    "common start")
            self._publication_delay_s = (
                stamp[0] + stamp[1] * 1e-9
                - (provisional[0] + provisional[1] * 1e-9))
        except Exception as exc:
            self._finish_virtual_failure(
                plan_id, snapshot, result, started, "ENVELOPE_INVALID",
                "the executable virtual plan could not be enveloped and "
                "re-verified: {!r}".format(exc), layer="envelope")
            return
        self._active_model_sha256 = verified.model_sha256
        self._active_request_sha256 = result.request_sha256
        self._active_digest = verified.model_sha256
        self._active_status_word = self._virtual_status_word(result.status)
        self._last_result = result
        self._published_wall = _time.monotonic()
        # The receiver independently rebuilds and checks the same trajectory.
        # Allow time for that work and ROS transport, bounded per publication.
        self._playback_confirmation_grace_s = min(
            MAX_PLAYBACK_CONFIRMATION_GRACE_S,
            2 * (self._published_wall - verification_started))
        self._published_duration = float(plan.duration_s)
        self._published_stamp = fields["stamp"]
        self._playback_seen = False
        self.playback_plan_pub.publish(envelope)
        # PUBLISHED is emitted only now: the atomic replay envelope has
        # actually been published by this ROS node.
        self._publish_planning_published(
            generation, plan_id, snapshot, result, verified.model_sha256,
            float(getattr(result, "wall_time_s", 0.0)))
        status_word = self._active_status_word
        wall = float(getattr(result, "wall_time_s", 0.0))
        extra = {
            "admitted": True,
            "planning_started": True,
            "pipeline_status": result.status,
            "first_failure_layer": result.first_failure_layer,
            "wall_time_s": wall,
            "T_end": float(plan.duration_s),
            "duration_s": float(plan.duration_s),
            "trajectory_stamp": {"secs": int(fields["stamp"][0]),
                                 "nsecs": int(fields["stamp"][1])},
            "verification": {
                "samples": len(fields["times_from_start"]),
                "model_sha256": verified.model_sha256,
                "certification": self.certification,
                "content_verified": True,
                "stamp_set_at_publication": True,
                "playback_confirmation_window_s": self._playback_confirmation_window(),
                "verification_to_publication_stamp_delay_s":
                    getattr(self, "_publication_delay_s", None),
            },
            "request_sha256": result.request_sha256,
            "planning_context_sha256": result.planning_context_sha256,
            "grid_sha256": result.grid_sha256,
            "initial_source": self._virtual_request.initial.source,
            "initial_context_id": self._virtual_request.initial.context_id,
            "goal_policy": self._virtual_request.goal_policy.value,
            "virtual_result": self._backend_hooks.result_summary(result),
            "collision_claim": (
                "certification applies to the three G4 audits frozen with "
                "this planning context; playback does not rerun map collision"),
        }
        self._publish_status(self._base_status(
            "PUBLISHED", status_word, result.reason, result.detail,
            plan_id=plan_id, formation_id=snapshot.formation_id,
            feasible=True, executable=True, published=True, extra=extra))
        self._publish_quality(self._base_quality(
            "plan", status_word, result.reason, result.detail,
            plan_id=plan_id, formation_id=snapshot.formation_id,
            feasible=True, executable=True, published=True, extra=extra))
        self.ros.log("info",
            "[uniform planner] virtual plan %s published: T_end=%.3f s "
            "model_sha256=%s samples=%d wall=%.3f s "
            "verification_to_stamp_delay=%.4f s",
            plan_id, plan.duration_s, verified.model_sha256,
            len(fields["times_from_start"]), wall,
            getattr(self, "_publication_delay_s", 0.0))

    def _finish_exception(self, generation, plan_id, snapshot, started, exc):
        with self._lock:
            if generation != self._generation or self._state != PLANNING:
                self.ros.log("warning",
                    "[uniform planner] discarding exception for stale job %s",
                    plan_id)
                return
            self._state = FAILED
            self._state_plan = plan_id
            wall = _time.monotonic() - started
            detail = "planner raised: {!r}".format(exc)
            self.ros.log("error", "[uniform planner] %s", detail)
            self._publish_status(self._base_status(
                "INTERNAL_ERROR", "REJECTED", "INTERNAL_ERROR", detail,
                plan_id=plan_id, formation_id=snapshot.formation_id,
                extra={"admitted": True, "planning_started": True,
                       "wall_time_s": wall}))
            self._publish_quality(self._base_quality(
                "plan", "REJECTED", "INTERNAL_ERROR", detail,
                plan_id=plan_id, formation_id=snapshot.formation_id,
                extra={"admitted": True, "wall_time_s": wall}))
            self._publish_no_envelope(plan_id, snapshot.formation_id,
                                      "INTERNAL_ERROR")
            if getattr(self, "_backend_hooks", None) is not None:
                # The worker raised: keep every layer already drawn and state
                # the failure in a final FAILED document.
                self._publish_planning_failed(
                    generation, plan_id, snapshot, None, "INTERNAL_ERROR",
                    detail, "internal", wall)
            self._publish_job_state("INTERNAL_ERROR", detail)

    def _publish_no_envelope(self, plan_id, formation_id, reason):
        """Overwrite the latched envelope with an explicitly invalid one.

        A failed experiment must not leave the previous executable envelope
        on ``/formation/playback_plan``: a playback node started later would
        otherwise accept an old plan.  This message carries the failed plan id
        but no model, no samples and no certification, so the receiver's
        verifier rejects it and clears the display.
        """
        envelope = FormationPlaybackPlan()
        envelope.header.frame_id = self.frame_id
        envelope.header.stamp = time_message(*self._now())
        envelope.plan_id = (int(plan_id) if isinstance(plan_id, int)
                            else 0)
        envelope.formation_id = formation_id or ""
        envelope.backend = self.backend
        envelope.semantic_version = getattr(
            self, "semantic_version", PLAYBACK_PLAN_SCHEMA)
        envelope.model_version = getattr(
            self, "model_version", MODEL_VERSION)
        envelope.model_sha256 = ""
        envelope.model_json = ""
        envelope.certification = ""
        envelope.trajectory = FormationTimedTrajectoryMsg()
        envelope.trajectory.header.frame_id = self.frame_id
        envelope.trajectory.header.stamp = envelope.header.stamp
        envelope.trajectory.plan_id = envelope.plan_id
        envelope.trajectory.formation_id = envelope.formation_id
        self.playback_plan_pub.publish(envelope)
        self.ros.log("warning",
            "[uniform planner] plan %s has no executable envelope (%s)",
            envelope.plan_id, reason)

    def _finish_failure(self, plan_id, snapshot, result, started, reason,
                        detail, layer="none"):
        self._state = FAILED
        self._state_plan = plan_id
        wall = _time.monotonic() - started
        status_word = _status_word(result.status)
        self.ros.log("error",
            "[uniform planner] plan %s not executable: status=%s reason=%s "
            "layer=%s wall=%.3fs", plan_id, result.status, reason, layer,
            wall)
        extra = {
            "admitted": True,
            "planning_started": True,
            "pipeline_status": result.status,
            "first_failure_layer": layer,
            "wall_time_s": wall,
            "search": self._search_summary(result),
            "evaluation": self._evaluation_summary(result),
            "projection": result.projection_summary(),
        }
        self._publish_status(self._base_status(
            "REJECTED", status_word, reason, detail,
            plan_id=plan_id, formation_id=snapshot.formation_id,
            feasible=False, executable=False, published=False, extra=extra))
        self._publish_quality(self._base_quality(
            "plan", status_word, reason, detail,
            plan_id=plan_id, formation_id=snapshot.formation_id,
            feasible=False, executable=False, published=False, extra=extra))
        self._publish_no_envelope(plan_id, snapshot.formation_id, reason)
        self._publish_job_state(reason, detail)

    def _finish_success(self, generation, plan_id, snapshot, result, started):
        plan = result.plan
        verification_started = _time.monotonic()
        try:
            # The model and the sample carrier do not depend on the wall-clock
            # start.  Build and fully verify them under a provisional stamp,
            # then restamp at the last instant before publication so playback
            # starts at the beginning of the schedule instead of mid-way
            # through the first phase (sample construction and model
            # verification take a measurable fraction of a second).
            provisional = self._now()
            times_ns, times = self._publication_times(plan)
            projection = plan.sample(times)
            fields = timed_trajectory_fields(
                snapshot, projection.timed_trajectory, provisional,
                goal_qf=plan.goal_qf, goal_headings=plan.goal_headings)
            fields["times_ns"] = times_ns
            for vehicle in fields["trajectories"]:
                vehicle["point_times_ns"] = times_ns
            message = timed_message(fields)
            validation = validate_formation_timed_trajectory(message)
            if not validation.valid:
                raise ValueError(
                    "converted trajectory violates the ROS contract: {} {}"
                    .format(validation.field, validation.detail))
            model_json = plan.to_model_json()
            digest = plan.model_sha256()
            envelope = FormationPlaybackPlan()
            envelope.header.frame_id = fields["frame_id"]
            envelope.header.stamp = time_message(*fields["stamp"])
            envelope.plan_id = int(plan.plan_id)
            envelope.formation_id = plan.formation_id
            envelope.backend = self.backend
            envelope.semantic_version = PLAYBACK_PLAN_SCHEMA
            envelope.model_version = projection.model_version
            envelope.model_sha256 = digest
            envelope.model_json = model_json
            envelope.certification = CERTIFICATION
            envelope.trajectory = message
            embedded = fields_from_timed_message(message)
            if embedded["stamp"] != fields["stamp"]:
                raise ValueError("envelope stamp differs from its trajectory")
            verified, rebuilt, verified_digest = verify_uniform_playback_plan(
                plan_id=int(plan.plan_id),
                formation_id=plan.formation_id,
                backend=self.backend,
                semantic_version=PLAYBACK_PLAN_SCHEMA,
                model_version=envelope.model_version,
                model_sha256=digest,
                model_json=model_json,
                certification=CERTIFICATION,
                trajectory_fields=embedded)
            # Only the common wall-clock start changes after verification;
            # the verified model, digest, identity, modes and every sample
            # remain byte-identical.  Restamping is therefore two header
            # assignments; verify their exact values before publishing.
            stamp = self._now()
            fields = dict(fields, stamp=stamp)
            message.header.stamp = time_message(*stamp)
            envelope.header.stamp = time_message(*stamp)
            envelope.trajectory = message
            if ((int(message.header.stamp.sec),
                 int(message.header.stamp.nanosec)) != stamp or
                    (int(envelope.header.stamp.sec),
                     int(envelope.header.stamp.nanosec)) != stamp):
                raise ValueError(
                    "restamped envelope header is inconsistent with the "
                    "common start")
            self._publication_delay_s = (
                stamp[0] + stamp[1] * 1e-9
                - (provisional[0] + provisional[1] * 1e-9))
        except Exception as exc:
            self._finish_failure(
                plan_id, snapshot, result, started, "ENVELOPE_INVALID",
                "the executable plan could not be enveloped and re-verified: "
                "{!r}".format(exc), layer="envelope")
            return
        self._active_model_sha256 = verified_digest
        self._active_model_version = envelope.model_version
        self._active_request_sha256 = result.request_sha256
        self._active_digest = verified_digest
        self._active_status_word = _status_word(result.status)
        self._last_result = result
        self._published_wall = _time.monotonic()
        # Playback independently rebuilds and verifies the same sample carrier.
        self._playback_confirmation_grace_s = min(
            MAX_PLAYBACK_CONFIRMATION_GRACE_S,
            2 * (self._published_wall - verification_started))
        self._published_duration = float(rebuilt.duration_s)
        self._published_stamp = fields["stamp"]
        self._playback_seen = False
        self.playback_plan_pub.publish(envelope)
        if self.execution_without_playback:
            self._state = PLAYING
            self._publish_job_state("execution envelope independently verified")
        status_word = self._active_status_word
        wall = result.wall_time_s
        extra = {
            "admitted": True,
            "planning_started": True,
            "pipeline_status": result.status,
            "first_failure_layer": result.first_failure_layer,
            "wall_time_s": wall,
            "planning_stages": getattr(result, "stages", {}),
            "envelope_verification_wall_s": self._published_wall - verification_started,
            "T_end": float(rebuilt.duration_s),
            "duration_s": float(rebuilt.duration_s),
            "trajectory_stamp": {"secs": int(fields["stamp"][0]),
                                 "nsecs": int(fields["stamp"][1])},
            "verification": {
                "samples": int(verified.sample_count),
                "model_sha256": verified_digest,
                "certification": CERTIFICATION,
                "content_verified": True,
                "stamp_set_at_publication": True,
                "verification_to_publication_stamp_delay_s":
                    getattr(self, "_publication_delay_s", None),
            },
            "search": self._search_summary(result),
            "evaluation": self._evaluation_summary(result),
            "projection": {
                "source": "FR02 plan.sample over the publication grid",
                "semantic_version": projection.semantic_version,
                "model_version": projection.model_version,
                "model_sha256": projection.model_sha256,
                "sample_count": len(projection.times),
                "publication_dt_s": self.publication_dt,
                "phase_count": len(plan.phases),
            },
            "audit_projection": result.projection_summary(),
        }
        self._publish_status(self._base_status(
            "PUBLISHED", status_word, result.reason, result.detail,
            plan_id=plan_id, formation_id=snapshot.formation_id,
            feasible=True, executable=True, published=True, extra=extra))
        self._publish_quality(self._base_quality(
            "plan", status_word, result.reason, result.detail,
            plan_id=plan_id, formation_id=snapshot.formation_id,
            feasible=True, executable=True, published=True, extra=extra))
        self.ros.log("info",
            "[uniform planner] plan %s published: T_end=%.3f s "
            "model_sha256=%s samples=%d wall=%.3f s "
            "verification_to_stamp_delay=%.4f s",
            plan_id, rebuilt.duration_s, verified_digest,
            verified.sample_count, wall,
            getattr(self, "_publication_delay_s", 0.0))
        if self.corridor_pub is not None:
            try:
                self._publish_corridor_guide(result.search, stamp)
            except Exception as exc:
                self.ros.log("warning", "[uniform planner] corridor display failed: %r", exc)

    @staticmethod
    def _search_summary(result):
        if result.search is None:
            return None
        data = result.search.as_dict(include_path=False)
        data.pop("terminal_audits", None)
        return data

    @staticmethod
    def _evaluation_summary(result):
        evaluation = result.evaluation
        if evaluation is None:
            return None
        document = evaluation.as_status()
        document["metrics"] = dict(evaluation.metrics)
        document["checks"] = int(evaluation.checks)
        return document

    # ---- playback feedback -------------------------------------------

    def on_playback_status(self, message):
        if self.execution_without_playback:
            return
        try:
            report = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(report, dict):
            return
        plan_id = report.get("plan_id")
        state = report.get("state")
        with self._lock:
            if plan_id != self._state_plan:
                return
            if state not in ("HOLD_START", "PLAYING", "HOLD_END",
                             "CLEARED", "INVALID"):
                # STATIC holds carry no model identity and are not feedback
                # about a published plan; ignore them without logging.
                return
            if state in ("HOLD_START", "PLAYING", "HOLD_END"):
                # A playback confirmation is only meaningful for a model this
                # job has actually published.  Before publication
                # ``_active_model_sha256`` is None, and comparing it with a
                # missing or explicit-null report digest would count as a
                # match (None == None), releasing the busy job before any
                # envelope exists.  Require a real published identity first.
                # This is not a blanket rejection of PLANNING: after the
                # envelope is published the job stays PLANNING until the
                # first playback confirmation, and a zero-duration hold may
                # report HOLD_END directly; both must keep working.
                if not self._active_model_sha256:
                    self.ros.warn_throttle(
                        5.0,
                        "[uniform planner] ignoring playback feedback for "
                        "plan %s: no model has been published for this job "
                        "yet", plan_id)
                    return
            # Feedback is authoritative only when it belongs to the model this
            # planner published.  A HOLD_END for another digest (or one
            # missing the identity fields) must not release the busy job or
            # upgrade its status to executable.
            if (report.get("model_sha256") != self._active_model_sha256 or
                    report.get("formation_id") != self._active_formation_id or
                    report.get("backend") != self.backend):
                self.ros.warn_throttle(
                    5.0,
                    "[uniform planner] ignoring playback feedback for plan "
                    "%s: identity differs from the published model "
                    "(model_sha256/formation_id/backend)", plan_id)
                return
            if state in ("HOLD_START", "PLAYING"):
                if not self._playback_seen:
                    self._playback_seen = True
                    self._playback_started_wall = _time.monotonic()
                    if self._state == PLANNING and self._published_wall:
                        self._state = PLAYING
                        self._publish_job_state(
                            "playback accepted the atomic envelope")
                        self.ros.log("info",
                            "[uniform planner] plan %s playback confirmed",
                            plan_id)
            elif state == "HOLD_END":
                if self._state in (PLANNING, PLAYING):
                    self._playback_seen = True
                    self._state = HOLD_END
                    terminal_extra = {"terminal": True,
                                      "playback_confirmed": True}
                    if (getattr(self, "_backend_hooks", None) is not None and
                            getattr(self, "_last_result", None) is not None
                            and getattr(self._last_result, "plan",
                                        None) is not None):
                        try:
                            from fwis_backends.multi_4wis.formation_continuous_a\
                                .ros_adapter import terminal_context_from_plan
                            context_id = self._next_context_id("terminal")
                            terminal = terminal_context_from_plan(
                                self._last_result.plan,
                                self._virtual_request, context_id)
                            self._initial_context = terminal
                            self._initial_reset_ready = False
                            terminal_extra.update({
                                "terminal_context_id": context_id,
                                "terminal_compatible": bool(
                                    terminal.compatible),
                                "terminal_detail": terminal.detail,
                                "terminal_state": dict(terminal.terminal),
                            })
                            self.ros.log("info",
                                "[uniform planner] plan %s terminal record "
                                "%s compatible=%s: %s", plan_id, context_id,
                                terminal.compatible, terminal.detail)
                        except Exception as exc:
                            terminal_extra["terminal_record_error"] = (
                                "{}: {}".format(type(exc).__name__, exc))
                            self.ros.log("error",
                                "[uniform planner] cannot record terminal "
                                "state for plan %s: %r", plan_id, exc)
                    self._publish_job_state(
                        "playback reached its terminal hold")
                    self._publish_status(self._base_status(
                        "HOLD_END",
                        self._active_status_word or "FEASIBLE", "OK",
                        "playback reached the authoritative final frame; "
                        "the next experiment is admitted",
                        plan_id=plan_id,
                        formation_id=self._active_formation_id,
                        feasible=True, executable=True, published=True,
                        extra=terminal_extra))
                    self.ros.log("info",
                        "[uniform planner] plan %s HOLD_END; busy released",
                        plan_id)
            elif (state in ("CLEARED", "INVALID") and
                  self._state in (PLANNING, PLAYING)):
                reason = str(report.get("reason") or state)
                self._state = FAILED
                self._publish_job_state("PLAYBACK_REJECTED", reason)
                self.ros.log("error",
                    "[uniform planner] playback cleared plan %s: %s",
                    plan_id, reason)

    # ---- watchdog ------------------------------------------------------

    def _playback_confirmation_window(self):
        return max(self.playback_start_timeout_s,
                   getattr(self, "_playback_confirmation_grace_s", 0.0))

    def on_timer(self, _event=None):
        with self._lock:
            now = _time.monotonic()
            if (now - self._last_heartbeat >= self.job_state_heartbeat_s):
                self._publish_job_state()
            if self._state == PLANNING:
                if self._published_wall is not None:
                    start_window = self._playback_confirmation_window()
                    if (not self._playback_seen and
                            now - self._published_wall > start_window):
                        self._playback_failure(
                            "PLAYBACK_START_TIMEOUT",
                            "playback did not confirm the atomic envelope "
                            "within {:.1f} s".format(
                                start_window))
                elif now - self._plan_started_wall > self.planning_timeout_s:
                    self._state = FAILED
                    self._generation += 1
                    detail = ("planning exceeded the watchdog window "
                              "{:.1f} s").format(self.planning_timeout_s)
                    self._publish_status(self._base_status(
                        "PLANNING_TIMEOUT", "REJECTED",
                        "PLANNING_TIMEOUT", detail,
                        formation_id=self._active_formation_id,
                        extra={"admitted": True, "planning_started": True}))
                    self._publish_quality(self._base_quality(
                        "plan", "REJECTED", "PLANNING_TIMEOUT", detail,
                        formation_id=self._active_formation_id,
                        extra={"admitted": True}))
                    self._publish_no_envelope(
                        self._state_plan, self._active_formation_id,
                        "PLANNING_TIMEOUT")
                    self._publish_job_state("PLANNING_TIMEOUT", detail)
                    self.ros.log("error", "[uniform planner] %s", detail)
            elif (self._state == PLAYING and self._published_stamp is not None
                  and not self.execution_without_playback):
                secs, nsecs = self._now()
                start_secs, start_nsecs = self._published_stamp
                elapsed = (secs - start_secs) + (nsecs - start_nsecs) * 1e-9
                if elapsed > (self._published_duration or 0.0) + self.playback_finish_grace_s:
                    self._playback_failure(
                        "PLAYBACK_FINISH_TIMEOUT",
                        "playback did not reach HOLD_END within T_end + "
                        "{:.1f} s".format(self.playback_finish_grace_s))

    def _playback_failure(self, reason, detail):
        self._state = FAILED
        self.ros.log("error", "[uniform planner] plan %s: %s", self._state_plan,
                     detail)
        self._publish_status(self._base_status(
            "PLAYBACK_NOT_CONFIRMED", "REJECTED", reason, detail,
            feasible=True, executable=False, published=True,
            extra={"admitted": True, "playback_confirmed": False}))
        self._publish_quality(self._base_quality(
            "playback", "REJECTED", reason, detail,
            feasible=True, executable=False, published=True,
            extra={"admitted": True, "playback_confirmed": False}))
        self._publish_no_envelope(self._state_plan, self._active_formation_id,
                                  reason)
        self._publish_job_state(reason, detail)


def _backend_hooks_from_parameters(ros):
    backend = str(ros.param("~backend", PLAYBACK_PLAN_BACKEND))
    hooks = None
    if backend == "virtual_rigid_a":
        from fwis_backends.multi_4wis.formation_continuous_a.contracts import (
            research_planner_config)
        from fwis_backends.multi_4wis.formation_continuous_a.ros_adapter import (
            VirtualABackendHooks, VirtualABackendSettings)
        planner_config = research_planner_config(
            guidance_mode=str(ros.param(
                "~virtual_a_guidance_mode", "corridor")),
            corridor_enforcement=str(ros.param(
                "~virtual_a_corridor_enforcement", "hard")),
            formation_corridor_scope=str(ros.param(
                "~virtual_a_formation_corridor_scope", "f_only")))
        ros.log("info",
            "[uniform planner] virtual_rigid_a validated planner config: "
            "guidance_mode=%s corridor_enforcement=%s "
            "formation_corridor_scope=%s as_dict=%s",
            planner_config.guidance_mode,
            planner_config.corridor_enforcement,
            planner_config.formation_corridor_scope,
            planner_config.as_dict())
        settings = VirtualABackendSettings(
            planner_config=planner_config,
            vehicle_limits=_load_limits(ros, "~timing_limits"),
            goal_policy=str(ros.param(
                "~virtual_a_goal_policy", "free")),
            frame_id=str(ros.param("~frame_id", "map")),
            max_publication_samples=int(ros.param(
                "~max_publication_samples", 200001)))
        hooks = VirtualABackendHooks(settings)
    elif backend != PLAYBACK_PLAN_BACKEND:
        raise ValueError(
            "~backend={!r}; this node starts only {!r} or "
            "'virtual_rigid_a'".format(backend, PLAYBACK_PLAN_BACKEND))
    return hooks


if __name__ == "__main__":
    run_node("formation_uniform_planner", UniformPlannerNode)
