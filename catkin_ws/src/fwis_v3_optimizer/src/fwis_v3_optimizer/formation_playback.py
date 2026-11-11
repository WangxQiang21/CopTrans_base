"""S10 ROS-free formation playback: one common clock and RViz marker plans.

Every vehicle of a ``FormationTimedTrajectory`` is interpolated on the same
``header.stamp``-based clock and the same ``time_from_start`` samples.  There
is deliberately no per-vehicle clock, no "start when this vehicle's message was
received" fallback and no independent retiming.

The module builds plain :class:`MarkerSpec` values; the ROS adapter in
``fwis_v3_playback/scripts/formation_playback.py`` converts them to
``visualization_msgs/Marker`` messages.  Marker namespaces for vehicle markers
are ``formation/<formation_id>/vehicle_<vehicle_id>`` and marker ids are unique
across the whole marker array.  Playback is a display layer: it never turns a
geometry-only trajectory into an executable one and it never claims the
independent S9 audit passed on its own.
"""
from bisect import bisect_right
from dataclasses import dataclass
import math
from typing import Mapping, Optional, Tuple


ACKERMANN = 0
OBLIQUE = 1
SPIN = 2
MODE_NAMES = {ACKERMANN: "A", OBLIQUE: "O", SPIN: "S"}

MODE_COLORS = {
    ACKERMANN: (0.10, 0.45, 1.00, 0.95),
    OBLIQUE: (0.10, 0.78, 0.30, 0.95),
    SPIN: (1.00, 0.40, 0.10, 0.95),
}
WHITE = (0.95, 0.95, 0.95, 1.00)
DIM_WHITE = (0.70, 0.70, 0.70, 0.65)
LINK_COLOR = (0.80, 0.80, 0.35, 0.55)
STATUS_OK_COLOR = (0.20, 1.00, 0.35, 1.00)
STATUS_FAIL_COLOR = (1.00, 0.25, 0.20, 1.00)

PLAYBACK_SCHEMA = "formation_playback/v1"
TIME_SOURCE = "header_stamp_common_clock"
GEOMETRY_ONLY_LABEL = "GEOMETRY_ONLY"
CLEARED = "CLEARED"
INVALID = "INVALID"
HOLD_START = "HOLD_START"
PLAYING = "PLAYING"
HOLD_END = "HOLD_END"

_NS_ROOT = "formation"
_TIME_TOL = 1e-9
_VALUE_TOL = 1e-9
_UINT32_MAX = 2 ** 32 - 1


class PlaybackValidationError(ValueError):
    """A trajectory/field set cannot be displayed safely."""


def _finite(value, context):
    if isinstance(value, bool) or isinstance(value, (str, bytes)):
        raise PlaybackValidationError(
            "{} must be a finite number, got {!r}".format(context, value))
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise PlaybackValidationError(
            "{} must be a finite number, got {!r}".format(
                context, value)) from exc
    if not math.isfinite(result):
        raise PlaybackValidationError(
            "{} must be finite, got {!r}".format(context, value))
    return result


def _positive(value, context):
    result = _finite(value, context)
    if result <= 0.0:
        raise PlaybackValidationError(
            "{} must be > 0, got {:.12g}".format(context, result))
    return result


def _integer(value, context, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise PlaybackValidationError(
            "{} must be an integer, got {!r}".format(context, value))
    if minimum is not None and value < minimum:
        raise PlaybackValidationError(
            "{} must be >= {}, got {}".format(context, minimum, value))
    if maximum is not None and value > maximum:
        raise PlaybackValidationError(
            "{} must be <= {}, got {}".format(context, maximum, value))
    return value


def _text(value, context):
    if not isinstance(value, str) or not value.strip():
        raise PlaybackValidationError(
            "{} must be a non-empty string".format(context))
    return value


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _angle_error(first, second):
    return abs(_wrap(first - second))


def _interpolate_angle(first, second, fraction):
    return first + fraction * _wrap(second - first)


def normalize_formation_id(formation_id):
    """Return a TF-safe token that still makes the frame unique per formation.

    Formation ids that are already TF-safe are used unchanged.  Ids that need
    sanitising get a deterministic hash suffix so ``wedge/3`` and ``wedge_3``
    cannot silently share one TF frame.
    """
    import hashlib
    text = _text(formation_id, "formation_id")
    safe = "".join(character if (
        character.isalnum() or character in ("_", "-")) else "_"
        for character in text)
    safe = safe.strip("_")
    if not safe:
        safe = "formation"
    if safe != text:
        digest = hashlib.sha1(text.encode("utf-8")).hexdigest()[:8]
        safe = "{}_{}".format(safe, digest)
    return safe


def vehicle_namespace(formation_id, vehicle_id):
    return "{}/{}/vehicle_{}".format(
        _NS_ROOT, _text(formation_id, "formation_id"),
        _integer(vehicle_id, "vehicle_id", minimum=0))


def shared_namespace(formation_id, name):
    return "{}/{}/{}".format(
        _NS_ROOT, _text(formation_id, "formation_id"),
        _text(name, "namespace"))


def tf_frame_prefix(formation_id):
    return "formation_{}".format(normalize_formation_id(formation_id))


@dataclass(frozen=True)
class PlaybackVehicleSpec:
    """One rigid formation member as published in the trajectory message."""

    vehicle_id: int
    offset_x: float
    offset_y: float
    body_length_m: float
    body_width_m: float

    def __post_init__(self):
        object.__setattr__(self, "vehicle_id", _integer(
            self.vehicle_id, "PlaybackVehicleSpec.vehicle_id", minimum=0))
        object.__setattr__(self, "offset_x", _finite(
            self.offset_x, "PlaybackVehicleSpec.offset_x"))
        object.__setattr__(self, "offset_y", _finite(
            self.offset_y, "PlaybackVehicleSpec.offset_y"))
        object.__setattr__(self, "body_length_m", _positive(
            self.body_length_m, "PlaybackVehicleSpec.body_length_m"))
        object.__setattr__(self, "body_width_m", _positive(
            self.body_width_m, "PlaybackVehicleSpec.body_width_m"))

    def obb_corners(self, center_x, center_y, theta):
        """Return the closed body OBB outline in the map frame."""
        half_length = 0.5 * self.body_length_m
        half_width = 0.5 * self.body_width_m
        cosine, sine = math.cos(theta), math.sin(theta)
        corners = []
        for local_x, local_y in (
                (half_length, half_width),
                (-half_length, half_width),
                (-half_length, -half_width),
                (half_length, -half_width),
                (half_length, half_width)):
            corners.append((
                center_x + cosine * local_x - sine * local_y,
                center_y + sine * local_x + cosine * local_y,
                0.0))
        return tuple(corners)


@dataclass(frozen=True)
class PlaybackPoint:
    """One common-time sample for one vehicle (display subset)."""

    time_from_start: float
    center_x: float
    center_y: float
    theta: float
    mode: int
    mode_control: float
    center_speed_mps: float

    def __post_init__(self):
        for name in ("time_from_start", "center_x", "center_y", "theta",
                     "mode_control", "center_speed_mps"):
            object.__setattr__(self, name, _finite(
                getattr(self, name), "PlaybackPoint." + name))
        _integer(self.mode, "PlaybackPoint.mode", minimum=ACKERMANN,
                 maximum=SPIN)


@dataclass(frozen=True)
class FormationPlaybackTrajectory:
    """Validated common-time trajectory prepared for display."""

    plan_id: int
    formation_id: str
    frame_id: str
    stamp: Tuple[int, int]
    vehicles: Tuple[PlaybackVehicleSpec, ...]
    formation_poses: Tuple[Tuple[float, float, float], ...]
    times_from_start: Tuple[float, ...]
    points: Mapping[int, Tuple[PlaybackPoint, ...]]

    def __post_init__(self):
        object.__setattr__(self, "plan_id", _integer(
            self.plan_id, "plan_id", minimum=0, maximum=_UINT32_MAX))
        object.__setattr__(self, "formation_id", _text(
            self.formation_id, "formation_id"))
        object.__setattr__(self, "frame_id", _text(
            self.frame_id, "frame_id"))
        try:
            secs, nsecs = self.stamp
        except (TypeError, ValueError) as exc:
            raise PlaybackValidationError(
                "stamp must be (secs, nsecs)") from exc
        secs = _integer(secs, "stamp.secs", minimum=0,
                        maximum=_UINT32_MAX)
        nsecs = _integer(nsecs, "stamp.nsecs", minimum=0)
        if nsecs >= 1000000000:
            raise PlaybackValidationError(
                "stamp.nsecs must be < 1e9")
        if secs == 0 and nsecs == 0:
            raise PlaybackValidationError(
                "stamp must be an explicit non-zero common time")
        object.__setattr__(self, "stamp", (secs, nsecs))
        try:
            vehicles = tuple(self.vehicles)
            poses = tuple(tuple(item) for item in self.formation_poses)
            times = tuple(float(item) for item in self.times_from_start)
        except (TypeError, ValueError) as exc:
            raise PlaybackValidationError(
                "invalid playback trajectory components") from exc
        if not vehicles:
            raise PlaybackValidationError(
                "a display trajectory needs at least one vehicle")
        checked_vehicles = []
        vehicle_ids = set()
        for vehicle in vehicles:
            if not isinstance(vehicle, PlaybackVehicleSpec):
                raise PlaybackValidationError(
                    "vehicles must contain PlaybackVehicleSpec values")
            if vehicle.vehicle_id in vehicle_ids:
                raise PlaybackValidationError(
                    "duplicate vehicle_id {}".format(vehicle.vehicle_id))
            vehicle_ids.add(vehicle.vehicle_id)
            checked_vehicles.append(vehicle)
        object.__setattr__(self, "vehicles", tuple(checked_vehicles))
        if not times:
            raise PlaybackValidationError("times_from_start is empty")
        if abs(times[0]) > _TIME_TOL:
            raise PlaybackValidationError(
                "times_from_start must begin at zero")
        for first, second in zip(times, times[1:]):
            if not second - first > _TIME_TOL:
                raise PlaybackValidationError(
                    "times_from_start must be strictly increasing")
        if len(poses) != len(times):
            raise PlaybackValidationError(
                "formation pose count differs from the time count")
        checked_poses = []
        for index, pose in enumerate(poses):
            if len(pose) != 3:
                raise PlaybackValidationError(
                    "formation_poses[{}] is not (x, y, psi)".format(index))
            checked_poses.append((
                _finite(pose[0], "formation_poses[{}].x".format(index)),
                _finite(pose[1], "formation_poses[{}].y".format(index)),
                _finite(pose[2], "formation_poses[{}].theta".format(index))))
        object.__setattr__(self, "formation_poses", tuple(checked_poses))
        object.__setattr__(self, "times_from_start", times)
        try:
            points = dict(self.points)
        except (TypeError, ValueError) as exc:
            raise PlaybackValidationError(
                "points must be a vehicle_id -> samples mapping") from exc
        if set(points) != vehicle_ids:
            raise PlaybackValidationError(
                "point vehicle ids {} differ from spec ids {}".format(
                    sorted(points), sorted(vehicle_ids)))
        checked_points = {}
        for vehicle_id, samples in points.items():
            samples = tuple(samples)
            if len(samples) != len(times):
                raise PlaybackValidationError(
                    "vehicle {} has {} samples, expected {}".format(
                        vehicle_id, len(samples), len(times)))
            for index, sample in enumerate(samples):
                if not isinstance(sample, PlaybackPoint):
                    raise PlaybackValidationError(
                        "vehicle {} sample {} is not PlaybackPoint".format(
                            vehicle_id, index))
                if abs(sample.time_from_start - times[index]) > _TIME_TOL:
                    raise PlaybackValidationError(
                        "vehicle {} sample {} time differs from the shared "
                        "time axis".format(vehicle_id, index))
            checked_points[vehicle_id] = samples
        object.__setattr__(self, "points", checked_points)

    @property
    def duration(self):
        return self.times_from_start[-1]

    @property
    def vehicle_ids(self):
        return tuple(vehicle.vehicle_id for vehicle in self.vehicles)

    @property
    def stamp_seconds(self):
        return self.stamp[0] + self.stamp[1] * 1e-9

    @property
    def sample_count(self):
        return len(self.times_from_start)

    def vehicle_map(self):
        return {vehicle.vehicle_id: vehicle for vehicle in self.vehicles}


def _field(owner, name, context):
    if isinstance(owner, Mapping):
        if name not in owner:
            raise PlaybackValidationError(
                "{} missing field {!r}".format(context, name))
        return owner[name]
    if not hasattr(owner, name):
        raise PlaybackValidationError(
            "{} missing field {!r}".format(context, name))
    return getattr(owner, name)


def _sequence(value, context):
    if not isinstance(value, (list, tuple)):
        raise PlaybackValidationError(
            "{} must be a sequence, got {!r}".format(
                context, type(value).__name__))
    return tuple(value)


def trajectory_from_fields(fields):
    """Build a display trajectory from plain ROS-message field values.

    ``fields`` follows ``timed_trajectory_fields`` from
    ``formation_optimizer_node.py``: ``frame_id``, ``stamp``, ``plan_id``,
    ``formation_id``, ``vehicles``, ``formation_poses``, ``times_from_start``
    and ``trajectories``.  Only display fields are required; missing optional
    start/goal metadata is ignored.
    """
    try:
        frame_id = _field(fields, "frame_id", "trajectory")
        stamp = _field(fields, "stamp", "trajectory")
        plan_id = _field(fields, "plan_id", "trajectory")
        formation_id = _field(fields, "formation_id", "trajectory")
        times = tuple(_finite(value, "times_from_start")
                      for value in _sequence(
                          _field(fields, "times_from_start", "trajectory"),
                          "times_from_start"))
        poses = []
        for index, pose in enumerate(_sequence(
                _field(fields, "formation_poses", "trajectory"),
                "formation_poses")):
            pose = _sequence(pose, "formation_poses[{}]".format(index))
            if len(pose) != 3:
                raise PlaybackValidationError(
                    "formation_poses[{}] must be (x, y, psi)".format(index))
            poses.append(tuple(_finite(value, "formation_poses[{}]".format(
                index)) for value in pose))
        vehicles = []
        for index, item in enumerate(_sequence(
                _field(fields, "vehicles", "trajectory"), "vehicles")):
            vehicle_id = _field(item, "vehicle_id", "vehicles")
            offset = _field(item, "offset", "vehicles")
            offset = _sequence(offset, "vehicles.offset")
            if len(offset) < 2:
                raise PlaybackValidationError(
                    "vehicles.offset must contain x and y")
            vehicles.append(PlaybackVehicleSpec(
                vehicle_id=vehicle_id,
                offset_x=offset[0],
                offset_y=offset[1],
                body_length_m=_field(item, "body_length_m", "vehicles"),
                body_width_m=_field(item, "body_width_m", "vehicles")))
        point_map = {}
        for item in _sequence(
                _field(fields, "trajectories", "trajectory"),
                "trajectories"):
            vehicle_id = _field(item, "vehicle_id", "trajectories")
            samples = []
            for index, sample in enumerate(_sequence(
                    _field(item, "points", "trajectories"),
                    "trajectories.points")):
                sample = _sequence(
                    sample, "trajectories[{}].points[{}]".format(
                        vehicle_id, index))
                if len(sample) < 7:
                    raise PlaybackValidationError(
                        "trajectory point must have at least 7 fields")
                samples.append(PlaybackPoint(
                    time_from_start=sample[0],
                    center_x=sample[1],
                    center_y=sample[2],
                    theta=sample[3],
                    center_speed_mps=sample[4],
                    mode=sample[6],
                    mode_control=sample[7] if len(sample) > 7 else 0.0))
            if vehicle_id in point_map:
                raise PlaybackValidationError(
                    "duplicate trajectory vehicle_id {}".format(vehicle_id))
            point_map[vehicle_id] = tuple(samples)
        return FormationPlaybackTrajectory(
            plan_id=plan_id,
            formation_id=formation_id,
            frame_id=frame_id,
            stamp=stamp,
            vehicles=tuple(vehicles),
            formation_poses=tuple(poses),
            times_from_start=times,
            points=point_map)
    except PlaybackValidationError:
        raise
    except (AttributeError, IndexError, KeyError, TypeError,
            ValueError, OverflowError) as exc:
        raise PlaybackValidationError(
            "trajectory field conversion failed: {!r}".format(exc)) from exc


@dataclass(frozen=True)
class PlaybackVehicleState:
    vehicle_id: int
    center_x: float
    center_y: float
    theta: float
    mode: int
    mode_control: float
    center_speed_mps: float
    offset_x: float
    offset_y: float
    body_length_m: float
    body_width_m: float

    @property
    def mode_name(self):
        return MODE_NAMES.get(self.mode, "?")


@dataclass(frozen=True)
class PlaybackFrame:
    """Interpolated state on the common clock."""

    plan_id: int
    formation_id: str
    frame_id: str
    stamp: Tuple[int, int]
    duration_s: float
    elapsed_s: float
    clamped_elapsed_s: float
    state: str
    lower_index: int
    upper_index: int
    fraction: float
    formation_x: float
    formation_y: float
    formation_psi: float
    vehicles: Tuple[PlaybackVehicleState, ...]

    @property
    def terminal(self):
        return self.state == HOLD_END

    @property
    def vehicle_ids(self):
        return tuple(vehicle.vehicle_id for vehicle in self.vehicles)

    def vehicle_map(self):
        return {vehicle.vehicle_id: vehicle for vehicle in self.vehicles}


def frame_at(trajectory, elapsed_s):
    """Interpolate every vehicle on the same clock at ``elapsed_s``.

    ``elapsed_s`` is measured from the trajectory's common ``header.stamp``.
    Values outside ``[0, duration]`` are held, never restarted per vehicle.
    """
    if not isinstance(trajectory, FormationPlaybackTrajectory):
        raise PlaybackValidationError(
            "expected FormationPlaybackTrajectory")
    elapsed = _finite(elapsed_s, "elapsed_s")
    times = trajectory.times_from_start
    duration = trajectory.duration
    if elapsed <= 0.0:
        lower = 0
        upper = 0
        fraction = 0.0
        state = HOLD_START
        clamped = 0.0
    elif elapsed >= duration:
        lower = len(times) - 1
        upper = lower
        fraction = 0.0
        state = HOLD_END
        clamped = duration
    else:
        upper = bisect_right(times, elapsed)
        lower = upper - 1
        span = times[upper] - times[lower]
        if span <= 0.0:
            raise PlaybackValidationError(
                "shared time axis is not strictly increasing")
        fraction = (elapsed - times[lower]) / span
        if fraction <= 0.0:
            fraction = 0.0
        state = PLAYING
        clamped = elapsed
    formation = trajectory.formation_poses
    lower_pose = formation[lower]
    upper_pose = formation[upper]
    formation_x = lower_pose[0] + fraction * (
        upper_pose[0] - lower_pose[0])
    formation_y = lower_pose[1] + fraction * (
        upper_pose[1] - lower_pose[1])
    formation_psi = _interpolate_angle(
        lower_pose[2], upper_pose[2], fraction)
    states = []
    for vehicle in trajectory.vehicles:
        samples = trajectory.points[vehicle.vehicle_id]
        first = samples[lower]
        second = samples[upper]
        states.append(PlaybackVehicleState(
            vehicle_id=vehicle.vehicle_id,
            center_x=first.center_x + fraction * (
                second.center_x - first.center_x),
            center_y=first.center_y + fraction * (
                second.center_y - first.center_y),
            theta=_interpolate_angle(
                first.theta, second.theta, fraction),
            mode=first.mode,
            mode_control=first.mode_control,
            center_speed_mps=first.center_speed_mps + fraction * (
                second.center_speed_mps - first.center_speed_mps),
            offset_x=vehicle.offset_x,
            offset_y=vehicle.offset_y,
            body_length_m=vehicle.body_length_m,
            body_width_m=vehicle.body_width_m))
    return PlaybackFrame(
        plan_id=trajectory.plan_id,
        formation_id=trajectory.formation_id,
        frame_id=trajectory.frame_id,
        stamp=trajectory.stamp,
        duration_s=duration,
        elapsed_s=elapsed,
        clamped_elapsed_s=clamped,
        state=state,
        lower_index=lower,
        upper_index=upper,
        fraction=fraction,
        formation_x=formation_x,
        formation_y=formation_y,
        formation_psi=formation_psi,
        vehicles=tuple(states))


def terminal_errors(trajectory, frame):
    """Return max center/heading error of a held final frame vs the last sample."""
    last = len(trajectory.times_from_start) - 1
    center_error = 0.0
    heading_error = 0.0
    for vehicle in trajectory.vehicles:
        point = trajectory.points[vehicle.vehicle_id][last]
        state = frame.vehicle_map()[vehicle.vehicle_id]
        center_error = max(center_error, math.hypot(
            state.center_x - point.center_x,
            state.center_y - point.center_y))
        heading_error = max(heading_error, _angle_error(
            state.theta, point.theta))
    return center_error, heading_error


@dataclass(frozen=True)
class PlaybackMarkerOptions:
    """Display sizes; all values are metres or metres/second."""

    band_width_m: float = 0.05
    history_width_m: float = 0.015
    body_width_m: float = 0.03
    heading_width_m: float = 0.02
    link_width_m: float = 0.01
    reference_scale_m: float = 0.14
    text_size_m: float = 0.16
    status_text_size_m: float = 0.20
    text_height_m: float = 0.35
    arrow_head_length_m: float = 0.08

    def __post_init__(self):
        for name in (
                "band_width_m", "history_width_m", "body_width_m",
                "heading_width_m", "link_width_m", "reference_scale_m",
                "text_size_m", "status_text_size_m", "text_height_m",
                "arrow_head_length_m"):
            object.__setattr__(self, name, _positive(
                getattr(self, name), "PlaybackMarkerOptions." + name))


@dataclass(frozen=True)
class MarkerSpec:
    """One ROS-independent marker request."""

    namespace: str
    marker_id: int
    marker_type: str
    frame_id: str
    points: Tuple[Tuple[float, float, float], ...] = ()
    colors: Tuple[Tuple[float, float, float, float], ...] = ()
    color: Tuple[float, float, float, float] = WHITE
    scale: Tuple[float, float, float] = (0.1, 0.1, 0.1)
    text: str = ""
    action: str = "ADD"
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw: float = 0.0

    def __post_init__(self):
        _text(self.namespace, "MarkerSpec.namespace")
        _integer(self.marker_id, "MarkerSpec.marker_id", minimum=0)
        _text(self.marker_type, "MarkerSpec.marker_type")
        _text(self.frame_id, "MarkerSpec.frame_id")
        if self.action not in ("ADD", "DELETE", "DELETEALL"):
            raise PlaybackValidationError(
                "unsupported marker action {!r}".format(self.action))


def _marker(namespace, marker_id, marker_type, frame_id, **values):
    return MarkerSpec(
        namespace=namespace, marker_id=marker_id,
        marker_type=marker_type, frame_id=frame_id, **values)


def _mode_text(state):
    direction = "-" if state.center_speed_mps < -_VALUE_TOL else (
        "+" if state.mode != SPIN else "")
    if state.mode == ACKERMANN:
        detail = "k={:+.3f} 1/m".format(state.mode_control)
    elif state.mode == OBLIQUE:
        detail = "phi={:+.1f} deg".format(
            math.degrees(state.mode_control))
    else:
        detail = "center fixed"
    return "V{} {}{} {}".format(
        state.vehicle_id, state.mode_name, direction, detail)


def _status_label(status, plan_id):
    """Interpret the optimizer status without ever upgrading its claims."""
    if not isinstance(status, Mapping):
        return "AUDIT_PENDING", False, GEOMETRY_ONLY_LABEL, "no status report"
    status_plan = status.get("plan_id")
    executable = bool(
        status.get("trajectory_published") and status.get("executable")
        and status.get("feasible") and status_plan == plan_id)
    if executable:
        if status.get("status") == "NO_MOTION":
            return "NO_MOTION", True, "NO_MOTION", (
                "audited zero-motion hold; static state published")
        return "FEASIBLE", True, "FEASIBLE", (
            "independent audit and transactional publication")
    reason = status.get("reason") or status.get("status") or "UNKNOWN"
    if status.get("status") == "FEASIBLE":
        return "GEOMETRY_ONLY", False, GEOMETRY_ONLY_LABEL, str(reason)
    return "GEOMETRY_ONLY", False, GEOMETRY_ONLY_LABEL, (
        "{}: {}".format(status.get("status", "UNKNOWN"), reason))


def _selected_metrics(status):
    if not isinstance(status, Mapping):
        return {}
    pipeline = status.get("pipeline")
    if not isinstance(pipeline, Mapping):
        return {}
    selection = pipeline.get("selection")
    if not isinstance(selection, Mapping):
        return {}
    selected_id = selection.get("selected_candidate_id")
    evaluations = selection.get("evaluations")
    if not isinstance(evaluations, list):
        return {}
    for evaluation in evaluations:
        if not isinstance(evaluation, Mapping):
            continue
        if selected_id is not None and evaluation.get(
                "plan_id") == status.get("plan_id") and evaluation.get(
                "status") == "FEASIBLE":
            metrics = evaluation.get("metrics")
            return metrics if isinstance(metrics, Mapping) else {}
    for evaluation in evaluations:
        if isinstance(evaluation, Mapping) and evaluation.get(
                "status") == "FEASIBLE":
            metrics = evaluation.get("metrics")
            return metrics if isinstance(metrics, Mapping) else {}
    return {}


def _metrics_text(status):
    metrics = _selected_metrics(status)
    if not metrics:
        return "no independent metrics reported"
    parts = []
    for key, label in (
            ("duration_s", "T_end"),
            ("max_rigidity_error_m", "rigidity"),
            ("minimum_obstacle_clearance_m", "obstacle"),
            ("minimum_intervehicle_clearance_m", "intervehicle"),
            ("dynamic_peak_ratio", "dynamic_ratio"),
            ("mode_switch_count", "mode_switch")):
        value = metrics.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            parts.append("{}={:.3g}".format(label, value))
    return " ".join(parts)


def build_markers(trajectory, frame, options=None, status=None):
    """Plan the full marker array for one playback frame.

    Vehicle marker namespaces always contain the formation id and vehicle id.
    Marker ids are unique across the returned tuple.
    """
    if not isinstance(trajectory, FormationPlaybackTrajectory):
        raise PlaybackValidationError(
            "expected FormationPlaybackTrajectory")
    if not isinstance(frame, PlaybackFrame):
        raise PlaybackValidationError("expected PlaybackFrame")
    if frame.plan_id != trajectory.plan_id or (
            frame.formation_id != trajectory.formation_id):
        raise PlaybackValidationError(
            "frame identity differs from the trajectory")
    options = options or PlaybackMarkerOptions()
    if not isinstance(options, PlaybackMarkerOptions):
        raise PlaybackValidationError("expected PlaybackMarkerOptions")
    frame_id = trajectory.frame_id
    formation_id = trajectory.formation_id
    markers = []
    next_id = [1]

    def add(namespace, marker_type, **values):
        marker_id = next_id[0]
        next_id[0] += 1
        markers.append(_marker(
            namespace, marker_id, marker_type, frame_id, **values))

    specification = trajectory.vehicle_map()
    states = frame.vehicle_map()
    for vehicle in trajectory.vehicles:
        state = states[vehicle.vehicle_id]
        namespace = vehicle_namespace(
            formation_id, vehicle.vehicle_id)
        mode_color = MODE_COLORS[state.mode]
        # Current body OBB outline.
        add(namespace, "LINE_STRIP",
            points=vehicle.obb_corners(
                state.center_x, state.center_y, state.theta),
            color=mode_color,
            scale=(options.body_width_m, 0.0, 0.0))
        # Body-heading arrow, never the formation heading.
        heading_length = 0.65 * vehicle.body_length_m
        add(namespace, "ARROW",
            points=((state.center_x, state.center_y, 0.03),
                    (state.center_x + heading_length * math.cos(state.theta),
                     state.center_y + heading_length * math.sin(state.theta),
                     0.03)),
            color=WHITE,
            scale=(options.heading_width_m,
                   2.5 * options.heading_width_m,
                   options.arrow_head_length_m))
        # A/O/S mode-coloured full path (colour per sample).
        samples = trajectory.points[vehicle.vehicle_id]
        add(namespace, "LINE_STRIP",
            points=tuple((sample.center_x, sample.center_y, 0.01)
                         for sample in samples),
            colors=tuple(MODE_COLORS[sample.mode] for sample in samples),
            scale=(options.band_width_m, 0.0, 0.0))
        # Traveled history up to the current interpolated point.
        history = [
            (sample.center_x, sample.center_y, 0.02)
            for sample in samples[:frame.lower_index + 1]]
        if frame.fraction > 0.0 and frame.lower_index + 1 < len(samples):
            history.append((state.center_x, state.center_y, 0.02))
        if len(history) == 1:
            history.append(history[0])
        add(namespace, "LINE_STRIP",
            points=tuple(history),
            color=DIM_WHITE,
            scale=(options.history_width_m, 0.0, 0.0))
        # Mode/control label at the current body.
        add(namespace, "TEXT_VIEW_FACING",
            position=(state.center_x, state.center_y,
                      options.text_height_m),
            text=_mode_text(state),
            color=mode_color,
            scale=(0.0, 0.0, options.text_size_m))

    # Pairwise inter-vehicle links at the current frame.
    ordered = [states[vehicle.vehicle_id] for vehicle in trajectory.vehicles]
    link_points = []
    for first_index in range(len(ordered)):
        for second_index in range(first_index + 1, len(ordered)):
            first = ordered[first_index]
            second = ordered[second_index]
            link_points.append((first.center_x, first.center_y, 0.015))
            link_points.append((second.center_x, second.center_y, 0.015))
    if link_points:
        add(shared_namespace(formation_id, "links"), "LINE_LIST",
            points=tuple(link_points),
            color=LINK_COLOR,
            scale=(options.link_width_m, 0.0, 0.0))

    # Formation reference point and its trail.
    reference_ns = shared_namespace(formation_id, "reference")
    add(reference_ns, "SPHERE",
        position=(frame.formation_x, frame.formation_y, 0.10),
        color=WHITE,
        scale=(options.reference_scale_m,
               options.reference_scale_m,
               options.reference_scale_m))
    add(reference_ns, "LINE_STRIP",
        points=tuple((x, y, 0.01)
                     for x, y, _ in trajectory.formation_poses),
        color=DIM_WHITE,
        scale=(options.history_width_m, 0.0, 0.0))
    add(reference_ns, "TEXT_VIEW_FACING",
        position=(frame.formation_x, frame.formation_y,
                  2.2 * options.text_height_m),
        text="F",
        color=WHITE,
        scale=(0.0, 0.0, 1.2 * options.text_size_m))

    # Honest status label: GEOMETRY_ONLY until the S9 status proves otherwise.
    audit_state, executable, label, detail = _status_label(
        status, trajectory.plan_id)
    status_color = STATUS_OK_COLOR if executable else STATUS_FAIL_COLOR
    status_text = (
        "plan {} | {} | {}\nT_end={:.3f} s | {} / {}\n{}".format(
            trajectory.plan_id, frame.state, label,
            trajectory.duration, audit_state, detail,
            _metrics_text(status)))
    add(shared_namespace(formation_id, "status"), "TEXT_VIEW_FACING",
        position=(frame.formation_x, frame.formation_y,
                  3.4 * options.text_height_m),
        text=status_text,
        color=status_color,
        scale=(0.0, 0.0, options.status_text_size_m))

    namespaces = tuple(sorted({marker.namespace for marker in markers}))
    return tuple(markers), namespaces, audit_state, executable


def clear_marker_specs(frame_id="map"):
    """One DELETEALL marker that removes every previous formation marker."""
    return (_marker(
        _NS_ROOT, 0, "DELETEALL", _text(frame_id, "frame_id"),
        action="DELETEALL"),)


def build_hold_markers(formation_id, qf, vehicles, reason, frame_id="map"):
    """Static bodies and cargo reference only; never an executable path.

    ``vehicles`` contains (PlaybackVehicleSpec, x, y, body_heading) tuples.
    Caller must clear obsolete path markers before showing this static state.
    """
    markers = []
    color = (0.65, 0.75, 0.85, 1.0)
    for spec, x, y, theta in vehicles:
        ns = vehicle_namespace(formation_id, spec.vehicle_id)
        markers.append(_marker(ns, len(markers) + 1, "LINE_STRIP", frame_id,
            points=spec.obb_corners(x, y, theta), color=color,
            scale=(0.035, 0.0, 0.0)))
        markers.append(_marker(ns, len(markers) + 1, "ARROW", frame_id,
            points=((x, y, 0.04), (x + 0.5 * math.cos(theta),
                                    y + 0.5 * math.sin(theta), 0.04)),
            color=color, scale=(0.025, 0.06, 0.08)))
        markers.append(_marker(ns, len(markers) + 1, "TEXT_VIEW_FACING", frame_id,
            position=(x, y, 0.4), text="car{}".format(spec.vehicle_id),
            color=color, scale=(0.0, 0.0, 0.18)))
    x, y, yaw = qf
    ns = shared_namespace(formation_id, "cargo")
    markers.append(_marker(ns, len(markers) + 1, "SPHERE", frame_id,
        position=(x, y, 0.1), color=WHITE, scale=(0.16, 0.16, 0.16)))
    markers.append(_marker(ns, len(markers) + 1, "ARROW", frame_id,
        points=((x, y, 0.06), (x + 0.7 * math.cos(yaw),
                                y + 0.7 * math.sin(yaw), 0.06)),
        color=WHITE, scale=(0.035, 0.08, 0.12)))
    markers.append(_marker(ns, len(markers) + 1, "TEXT_VIEW_FACING", frame_id,
        position=(x, y, 0.85), text="Cargo | {}\nSTATIC / no executable trajectory".format(reason),
        color=WHITE, scale=(0.0, 0.0, 0.20)))
    return tuple(markers)


def status_document(trajectory=None, frame=None, status=None, *,
                    cleared=False, marker_count=0, namespaces=(),
                    invalid_reason="", frame_id="map",
                    terminal_center_error_m=None,
                    terminal_heading_error_rad=None):
    """Strict-JSON playback status for smoke tests and RViz-side inspection."""
    if cleared:
        return {
            "schema": PLAYBACK_SCHEMA,
            "state": CLEARED,
            "cleared": True,
            "frame_id": frame_id,
            "plan_id": None,
            "formation_id": None,
            "time_source": TIME_SOURCE,
            "marker_count": 0,
            "marker_namespaces": [],
            "executable": False,
            "geometry_only": True,
            "reason": invalid_reason or "empty trajectory clears all markers",
        }
    if invalid_reason:
        return {
            "schema": PLAYBACK_SCHEMA,
            "state": INVALID,
            "cleared": True,
            "frame_id": frame_id,
            "plan_id": (None if trajectory is None
                        else trajectory.plan_id),
            "formation_id": (None if trajectory is None
                             else trajectory.formation_id),
            "time_source": TIME_SOURCE,
            "marker_count": 0,
            "marker_namespaces": [],
            "executable": False,
            "geometry_only": True,
            "reason": invalid_reason,
        }
    if not isinstance(trajectory, FormationPlaybackTrajectory) or not (
            isinstance(frame, PlaybackFrame)):
        raise PlaybackValidationError(
            "status_document requires a trajectory and frame")
    audit_state, executable, label, detail = _status_label(
        status, trajectory.plan_id)
    return {
        "schema": PLAYBACK_SCHEMA,
        "state": frame.state,
        "cleared": False,
        "frame_id": trajectory.frame_id,
        "plan_id": trajectory.plan_id,
        "formation_id": trajectory.formation_id,
        "time_source": TIME_SOURCE,
        "trajectory_stamp": {
            "secs": int(trajectory.stamp[0]),
            "nsecs": int(trajectory.stamp[1]),
        },
        "duration_s": frame.duration_s,
        "elapsed_s": frame.elapsed_s,
        "clamped_elapsed_s": frame.clamped_elapsed_s,
        "terminal": bool(frame.terminal),
        "lower_index": frame.lower_index,
        "upper_index": frame.upper_index,
        "fraction": frame.fraction,
        "sample_count": trajectory.sample_count,
        "vehicle_ids": list(trajectory.vehicle_ids),
        "vehicles": [
            {
                "vehicle_id": state.vehicle_id,
                "center": [state.center_x, state.center_y],
                "theta": state.theta,
                "mode": state.mode,
                "mode_name": state.mode_name,
                "mode_control": state.mode_control,
                "center_speed_mps": state.center_speed_mps,
                "namespace": vehicle_namespace(
                    trajectory.formation_id, state.vehicle_id),
            }
            for state in frame.vehicles
        ],
        "marker_count": int(marker_count),
        "marker_namespaces": list(namespaces),
        "certification": label,
        "audit_status": audit_state,
        "executable": bool(executable),
        "geometry_only": not executable,
        "audit_detail": detail,
        "terminal_center_error_m": (
            None if terminal_center_error_m is None
            else float(terminal_center_error_m)),
        "terminal_heading_error_rad": (
            None if terminal_heading_error_rad is None
            else float(terminal_heading_error_rad)),
    }
