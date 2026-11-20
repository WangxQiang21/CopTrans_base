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
from dataclasses import dataclass, replace
import hashlib
import json
import math
from typing import Mapping, Optional, Tuple

from .formation_continuous_a.contracts import (
    COLLISION_SCOPE,
    DYNAMICS_SCOPE,
    GEOMETRY_SCOPE,
    PROGRESS_SCHEMA,
    PROGRESS_STAGES,
)
from .formation_continuous_a.corridor import CORRIDOR_DISPLAY_STATE


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
PLAYBACK_PLAN_SCHEMA = "uniform_playback_plan/v1"
PLAYBACK_PLAN_BACKEND = "uniform_v02"
VIRTUAL_PLAYBACK_BACKEND = "virtual_rigid_a"
VIRTUAL_PLAYBACK_SEMANTIC_VERSION = "virtual_rigid_a.playback.v1"
VIRTUAL_MODEL_VERSION = "virtual_rigid_a.timed.v1"
VIRTUAL_CERTIFICATION = "virtual_a_geometry_dynamics_collision/v1"
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
    # Present only for the uniform_v02 atomic playback envelope.  When set,
    # frame_at derives every display value from this FR02 analytic plan
    # instead of linearly interpolating the published sample columns.
    uniform_plan: Optional[object] = None
    # Present only for the virtual_rigid_a atomic playback envelope.  When set,
    # frame_at derives every display value from the G4 timed model at the
    # actual quantised common-clock time instead of interpolating samples.
    virtual_a_plan: Optional[object] = None
    # Authoritative backend, set by a successful envelope verification.  Legacy
    # trajectories leave it None; the display then keeps the historical
    # uniform_v02 audit-label behavior.
    backend: Optional[str] = None
    # Authoritative model digest, set only by a verified envelope.  Legacy
    # trajectories leave it None and keep the historical plan_id-only audit
    # label behavior.  The display binds its audit label to this digest, so an
    # audit status for another model with the same plan_id can never mark this
    # display executable.
    model_sha256: Optional[str] = None

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
        if self.uniform_plan is not None and self.virtual_a_plan is not None:
            raise PlaybackValidationError(
                "a trajectory cannot carry both a uniform and a virtual plan")
        backend = self.backend
        if backend is not None:
            if not isinstance(backend, str) or not backend.strip():
                raise PlaybackValidationError(
                    "backend must be a non-empty string")
            if self.virtual_a_plan is not None and \
                    backend != VIRTUAL_PLAYBACK_BACKEND:
                raise PlaybackValidationError(
                    "virtual plan backend {!r} is not {!r}".format(
                        backend, VIRTUAL_PLAYBACK_BACKEND))
            if self.uniform_plan is not None and \
                    backend != PLAYBACK_PLAN_BACKEND:
                raise PlaybackValidationError(
                    "uniform plan backend {!r} is not {!r}".format(
                        backend, PLAYBACK_PLAN_BACKEND))
        if self.virtual_a_plan is not None and self.backend is None:
            raise PlaybackValidationError(
                "a virtual plan requires its verified backend identity")
        digest = self.model_sha256
        if digest is not None:
            if (not isinstance(digest, str) or len(digest) != 64 or
                    any(character not in "0123456789abcdef"
                        for character in digest)):
                raise PlaybackValidationError(
                    "model_sha256 must be 64 lowercase hex digits")

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


def virtual_a_request_from_raw_request(document, *, initial_source,
                                       initial_context_id):
    """Rebuild the physical ``VirtualARequest`` from a retained raw request.

    ``document`` is the receiver's neutral snapshot of the ROS
    ``FormationPlanRequest``.  It keeps the message's vehicle order so the
    rebuilt model and request digest match the planner's normalised snapshot.
    This function does not read ``model_json`` and does not trust planner
    derived fields: every centre, flag, heading, offset and shape is checked
    against the raw message values before the request is returned.
    """
    from .formation import FormationModel, FormationPose, VehicleOffset
    from .formation_continuous_a import contracts

    if not isinstance(document, Mapping):
        raise PlaybackValidationError("raw request must be a mapping")
    required = {
        "plan_id", "formation_id", "formation_start", "formation_goal",
        "vehicles",
    }
    missing = sorted(required - set(document))
    if missing:
        raise PlaybackValidationError(
            "raw request is missing {}".format(missing))
    plan_id = _integer(document["plan_id"], "plan_id", 0, _UINT32_MAX)
    formation_id = document["formation_id"]
    if not isinstance(formation_id, str) or not formation_id.strip():
        raise PlaybackValidationError(
            "formation_id must be a non-empty string")

    def pose(name):
        value = document[name]
        if not isinstance(value, Mapping):
            raise PlaybackValidationError(
                "{} must be a mapping".format(name))
        return FormationPose(
            _finite(value.get("x"), name + ".x"),
            _finite(value.get("y"), name + ".y"),
            _finite(value.get("theta"), name + ".theta"))

    start = pose("formation_start")
    goal = pose("formation_goal")
    raw_vehicles = document["vehicles"]
    if not isinstance(raw_vehicles, (list, tuple)) or not raw_vehicles:
        raise PlaybackValidationError("raw request has no vehicles")
    rows = []
    seen = set()
    for index, raw in enumerate(raw_vehicles):
        label = "vehicles[{}]".format(index)
        if not isinstance(raw, Mapping):
            raise PlaybackValidationError(label + " must be a mapping")
        vehicle_id = _integer(raw.get("vehicle_id"),
                              label + ".vehicle_id", 0, _UINT32_MAX)
        if vehicle_id in seen:
            raise PlaybackValidationError(
                "duplicate vehicle_id={}".format(vehicle_id))
        seen.add(vehicle_id)
        free = raw.get("goal_heading_free")
        if not isinstance(free, bool):
            raise PlaybackValidationError(
                label + ".goal_heading_free must be a bool")
        row = {
            "vehicle_id": vehicle_id,
            "offset_x": _finite(raw.get("offset_x"),
                                label + ".offset_x"),
            "offset_y": _finite(raw.get("offset_y"),
                                label + ".offset_y"),
            "start_x": _finite(raw.get("start_x"), label + ".start_x"),
            "start_y": _finite(raw.get("start_y"), label + ".start_y"),
            "start_theta": _finite(raw.get("start_theta"),
                                   label + ".start_theta"),
            "goal_x": _finite(raw.get("goal_x"), label + ".goal_x"),
            "goal_y": _finite(raw.get("goal_y"), label + ".goal_y"),
            "goal_theta": _finite(raw.get("goal_theta"),
                                  label + ".goal_theta"),
            "goal_heading_free": free,
            "body_length_m": _positive(raw.get("body_length_m"),
                                       label + ".body_length_m"),
            "body_width_m": _positive(raw.get("body_width_m"),
                                      label + ".body_width_m"),
        }
        rows.append(row)

    model = FormationModel([
        VehicleOffset(row["vehicle_id"], row["offset_x"], row["offset_y"])
        for row in rows])
    shapes = {
        row["vehicle_id"]: contracts.VehicleShape(
            row["body_length_m"], row["body_width_m"])
        for row in rows}
    start_centers = model.centers(start)
    goal_centers = model.centers(goal)
    for index, row in enumerate(rows):
        label = "vehicles[{}]".format(index)
        vehicle_id = row["vehicle_id"]
        if math.hypot(row["start_x"] - start_centers[vehicle_id][0],
                      row["start_y"] - start_centers[vehicle_id][1]) > 1e-6:
            raise PlaybackValidationError(
                label + ".start_center is not rigid with formation_start")
        if math.hypot(row["goal_x"] - goal_centers[vehicle_id][0],
                      row["goal_y"] - goal_centers[vehicle_id][1]) > 1e-6:
            raise PlaybackValidationError(
                label + ".goal_center is not rigid with formation_goal")

    flags = [row["goal_heading_free"] for row in rows]
    if all(flags):
        policy = "FREE"
    elif not any(flags):
        policy = "STRAIGHT"
        for index, row in enumerate(rows):
            error = abs(math.atan2(
                math.sin(row["goal_theta"] - goal.psi),
                math.cos(row["goal_theta"] - goal.psi)))
            if error > 1e-9:
                raise PlaybackValidationError(
                    "vehicles[{}].goal_theta differs from goal psi by "
                    "{:.6g} rad; a STRAIGHT request needs fixed headings "
                    "equal to the cargo goal heading".format(index, error))
    else:
        raise PlaybackValidationError(
            "mixed goal_heading_free is not a supported virtual_a request")

    payload = {
        "schema": contracts.REQUEST_SCHEMA,
        "plan_id": plan_id,
        "formation_id": formation_id.strip(),
        "model": [[row["vehicle_id"], row["offset_x"], row["offset_y"]]
                  for row in rows],
        "vehicle_shapes": [
            [row["vehicle_id"], row["body_length_m"], row["body_width_m"]]
            for row in rows],
        "start_qf": [start.x, start.y, start.psi],
        "start_headings": [[row["vehicle_id"], row["start_theta"]]
                           for row in rows],
        "goal_qf": [goal.x, goal.y, goal.psi],
        "initial": {
            "stationary": True,
            "mode": "A",
            "wheels_straight": True,
            "source": initial_source,
            "context_id": initial_context_id,
        },
        "goal_policy": policy,
        "limits": contracts.FormationTimingLimits().as_dict(),
    }
    try:
        request = contracts.request_from_payload(payload)
    except Exception as exc:
        raise PlaybackValidationError(
            "raw request cannot be normalised: {}: {}".format(
                type(exc).__name__, exc)) from exc
    validation = contracts.validate_virtual_a_request(request)
    if not validation.passed:
        raise PlaybackValidationError(
            "raw request is invalid: {}: {}".format(
                validation.reason, validation.detail))
    return request


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


def _message_seconds(value):
    if hasattr(value, "to_sec"):
        return float(value.to_sec())
    return float(value)


def fields_from_timed_message(message):
    """Convert a ``FormationTimedTrajectory`` message into plain fields.

    The same converter is used by the legacy playback subscription and by the
    uniform_v02 planner before it publishes an atomic playback envelope, so
    both sides validate identical values.  Start/goal vehicle metadata is
    retained for model/trajectory identity checks when present.
    """
    if not hasattr(message, "header"):
        raise PlaybackValidationError("message has no header")
    vehicles = []
    for vehicle in message.vehicles:
        offset = vehicle.offset_in_formation
        start = getattr(vehicle, "start_center", None)
        goal = getattr(vehicle, "goal_center", None)
        vehicles.append({
            "vehicle_id": int(vehicle.vehicle_id),
            "offset": (float(offset.x), float(offset.y)),
            "body_length_m": float(vehicle.body_length_m),
            "body_width_m": float(vehicle.body_width_m),
            "start_center": (None if start is None else (
                float(start.x), float(start.y), float(start.theta))),
            "goal_center": (None if goal is None else (
                float(goal.x), float(goal.y), float(goal.theta))),
            "goal_heading_free": bool(getattr(
                vehicle, "goal_heading_free", False)),
        })
    trajectories = []
    for trajectory in message.trajectories:
        points = []
        for point in trajectory.points:
            pose = point.center_pose
            points.append((
                _message_seconds(point.time_from_start),
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
            _message_seconds(value) for value in message.times_from_start),
        "trajectories": tuple(trajectories),
    }


def _check_close(actual, expected, context, tolerance):
    if not math.isfinite(actual) or not math.isfinite(expected):
        raise PlaybackValidationError(
            "{} is not finite".format(context))
    error = abs(actual - expected)
    if error > tolerance:
        raise PlaybackValidationError(
            "{}={:.17g} differs from the model {:.17g} by {:.6g} > {:.6g}"
            .format(context, actual, expected, error, tolerance))


def verify_uniform_playback_plan(*, plan_id, formation_id, backend,
                                 semantic_version, model_version,
                                 model_sha256, model_json, certification,
                                 trajectory_fields,
                                 position_tolerance=1e-9,
                                 angle_tolerance=1e-9,
                                 speed_tolerance=1e-9):
    """Validate one atomic V0.2 envelope against its authoritative model.

    The embedded model is the authority: the plan is rebuilt from
    ``model_json`` with the FR02 model loader, its versioned digest must equal
    the transmitted digest, and every published sample must equal
    ``state_at(sample_time)``.  Locations, headings, modes, controls, speeds
    and yaw rates all participate; equal endpoints or a plausible sample
    table alone are not accepted.
    """
    from .uniform_trajectory import MODEL_VERSION, UniformTimedPlan

    if backend != PLAYBACK_PLAN_BACKEND:
        raise PlaybackValidationError(
            "backend {!r} is not {!r}".format(
                backend, PLAYBACK_PLAN_BACKEND))
    if semantic_version != PLAYBACK_PLAN_SCHEMA:
        raise PlaybackValidationError(
            "unsupported playback semantic_version {!r}; expected {!r}"
            .format(semantic_version, PLAYBACK_PLAN_SCHEMA))
    if model_version != MODEL_VERSION:
        raise PlaybackValidationError(
            "unsupported model_version {!r}; expected {!r}".format(
                model_version, MODEL_VERSION))
    if not isinstance(model_sha256, str) or len(model_sha256) != 64:
        raise PlaybackValidationError("model_sha256 must be 64 hex digits")
    if not isinstance(model_json, str) or not model_json.strip():
        raise PlaybackValidationError("model_json must be a non-empty string")
    if not isinstance(certification, str) or not certification:
        raise PlaybackValidationError("certification must be a non-empty string")

    if plan_id != trajectory_fields.get("plan_id"):
        raise PlaybackValidationError(
            "envelope plan_id={!r} differs from the trajectory {!r}".format(
                plan_id, trajectory_fields.get("plan_id")))
    if formation_id != trajectory_fields.get("formation_id"):
        raise PlaybackValidationError(
            "envelope formation_id={!r} differs from the trajectory {!r}"
            .format(formation_id, trajectory_fields.get("formation_id")))

    try:
        plan = UniformTimedPlan.from_model_json(model_json)
    except (ValueError, TypeError, KeyError) as exc:
        raise PlaybackValidationError(
            "model_json does not rebuild a UniformTimedPlan: {}".format(exc)
        ) from exc
    except Exception as exc:  # FR02 model errors subclass ValueError
        raise PlaybackValidationError(
            "model_json was rejected: {}".format(exc)) from exc
    if plan.plan_id != plan_id or plan.formation_id != formation_id:
        raise PlaybackValidationError(
            "model identity plan_id={} formation_id={!r} differs from the "
            "envelope {} / {!r}".format(
                plan.plan_id, plan.formation_id, plan_id, formation_id))
    digest = plan.model_sha256()
    if digest != model_sha256:
        raise PlaybackValidationError(
            "model_sha256 mismatch: message={} rebuilt={}".format(
                model_sha256, digest))
    if plan.to_model_dict().get("certification") != certification:
        raise PlaybackValidationError(
            "certification differs from the authoritative model")

    try:
        trajectory = trajectory_from_fields(trajectory_fields)
    except PlaybackValidationError:
        raise
    except Exception as exc:
        raise PlaybackValidationError(
            "trajectory fields are invalid: {}".format(exc)) from exc
    _check_close(trajectory.duration, plan.duration_s,
                 "trajectory duration", position_tolerance)

    model_offsets = {
        offset.vehicle_id: (float(offset.x), float(offset.y))
        for offset in plan.model.offsets}
    model_shapes = {
        vehicle_id: (float(shape.length_m), float(shape.width_m))
        for vehicle_id, shape in plan.vehicle_shapes.items()}
    field_vehicles = trajectory_fields.get("vehicles", ())
    field_by_id = {}
    for item in field_vehicles:
        vehicle_id = int(item["vehicle_id"])
        if vehicle_id in field_by_id:
            raise PlaybackValidationError(
                "duplicate vehicle_id {} in the envelope".format(vehicle_id))
        field_by_id[vehicle_id] = item
    if set(field_by_id) != set(model_offsets):
        raise PlaybackValidationError(
            "envelope vehicle_ids {} differ from the model {}".format(
                sorted(field_by_id), sorted(model_offsets)))

    start_centers = plan.model.centers(plan.start_qf)
    goal_centers = plan.model.centers(plan.goal_qf)
    start_headings = dict(plan.start_headings)
    goal_headings = dict(plan.goal_headings)
    for vehicle_id in sorted(model_offsets):
        item = field_by_id[vehicle_id]
        offset = item["offset"]
        _check_close(offset[0], model_offsets[vehicle_id][0],
                     "vehicle {} offset_x".format(vehicle_id),
                     position_tolerance)
        _check_close(offset[1], model_offsets[vehicle_id][1],
                     "vehicle {} offset_y".format(vehicle_id),
                     position_tolerance)
        shape = model_shapes[vehicle_id]
        _check_close(float(item["body_length_m"]), shape[0],
                     "vehicle {} body_length_m".format(vehicle_id),
                     position_tolerance)
        _check_close(float(item["body_width_m"]), shape[1],
                     "vehicle {} body_width_m".format(vehicle_id),
                     position_tolerance)
        start = item.get("start_center")
        if start is not None:
            _check_close(start[0], start_centers[vehicle_id][0],
                         "vehicle {} start_center.x".format(vehicle_id),
                         position_tolerance)
            _check_close(start[1], start_centers[vehicle_id][1],
                         "vehicle {} start_center.y".format(vehicle_id),
                         position_tolerance)
            _check_close(math.atan2(math.sin(start[2] - start_headings[
                vehicle_id]), math.cos(start[2] - start_headings[
                    vehicle_id])), 0.0,
                "vehicle {} start heading".format(vehicle_id),
                angle_tolerance)
        goal = item.get("goal_center")
        if goal is not None:
            _check_close(goal[0], goal_centers[vehicle_id][0],
                         "vehicle {} goal_center.x".format(vehicle_id),
                         position_tolerance)
            _check_close(goal[1], goal_centers[vehicle_id][1],
                         "vehicle {} goal_center.y".format(vehicle_id),
                         position_tolerance)
            if not bool(item.get("goal_heading_free", False)):
                _check_close(math.atan2(math.sin(
                    goal[2] - goal_headings[vehicle_id]),
                    math.cos(goal[2] - goal_headings[vehicle_id])), 0.0,
                    "vehicle {} goal heading".format(vehicle_id),
                    angle_tolerance)

    if not trajectory.formation_poses:
        raise PlaybackValidationError("trajectory has no formation poses")
    first_pose = trajectory.formation_poses[0]
    last_pose = trajectory.formation_poses[-1]
    _check_close(first_pose[0], plan.start_qf.x, "start formation x",
                 position_tolerance)
    _check_close(first_pose[1], plan.start_qf.y, "start formation y",
                 position_tolerance)
    _check_close(math.atan2(math.sin(first_pose[2] - plan.start_qf.psi),
                            math.cos(first_pose[2] - plan.start_qf.psi)),
                 0.0, "start formation psi", angle_tolerance)
    _check_close(last_pose[0], plan.goal_qf.x, "goal formation x",
                 position_tolerance)
    _check_close(last_pose[1], plan.goal_qf.y, "goal formation y",
                 position_tolerance)
    _check_close(math.atan2(math.sin(last_pose[2] - plan.goal_qf.psi),
                            math.cos(last_pose[2] - plan.goal_qf.psi)),
                 0.0, "goal formation psi", angle_tolerance)

    raw_points = {
        int(item["vehicle_id"]): tuple(item["points"])
        for item in trajectory_fields.get("trajectories", ())
    }
    if set(raw_points) != set(model_offsets):
        raise PlaybackValidationError(
            "trajectory vehicle_ids {} differ from the model {}".format(
                sorted(raw_points), sorted(model_offsets)))
    times = trajectory.times_from_start
    for index, time_s in enumerate(times):
        state = plan.state_at(time_s)
        _check_close(state.qf.x, trajectory.formation_poses[index][0],
                     "formation_poses[{}].x".format(index),
                     position_tolerance)
        _check_close(state.qf.y, trajectory.formation_poses[index][1],
                     "formation_poses[{}].y".format(index),
                     position_tolerance)
        _check_close(math.atan2(math.sin(
            state.qf.psi - trajectory.formation_poses[index][2]),
            math.cos(state.qf.psi - trajectory.formation_poses[index][2])),
            0.0, "formation_poses[{}].psi".format(index), angle_tolerance)
        state_by_id = {vehicle.vehicle_id: vehicle
                       for vehicle in state.vehicles}
        for vehicle_id in sorted(model_offsets):
            sample = raw_points[vehicle_id][index]
            vehicle = state_by_id[vehicle_id]
            _check_close(float(sample[0]), time_s,
                         "vehicle {} sample {} time".format(
                             vehicle_id, index), 1e-9)
            _check_close(float(sample[1]), vehicle.x,
                         "vehicle {} sample {} x".format(vehicle_id, index),
                         position_tolerance)
            _check_close(float(sample[2]), vehicle.y,
                         "vehicle {} sample {} y".format(vehicle_id, index),
                         position_tolerance)
            _check_close(math.atan2(math.sin(float(sample[3]) - vehicle.theta),
                                    math.cos(float(sample[3]) - vehicle.theta)),
                         0.0,
                         "vehicle {} sample {} theta".format(
                             vehicle_id, index), angle_tolerance)
            _check_close(float(sample[4]), vehicle.center_speed_mps,
                         "vehicle {} sample {} speed".format(
                             vehicle_id, index), speed_tolerance)
            _check_close(float(sample[5]), vehicle.yaw_rate_radps,
                         "vehicle {} sample {} yaw rate".format(
                             vehicle_id, index), speed_tolerance)
            expected_mode = {"A": ACKERMANN, "O": OBLIQUE,
                             "S": SPIN}[state.mode]
            if int(sample[6]) != expected_mode:
                raise PlaybackValidationError(
                    "vehicle {} sample {} mode {} differs from the model {}"
                    .format(vehicle_id, index, sample[6], expected_mode))
            _check_close(float(sample[7]), vehicle.mode_control,
                         "vehicle {} sample {} mode_control".format(
                             vehicle_id, index), speed_tolerance)

    return replace(trajectory, uniform_plan=plan,
                   model_sha256=digest), plan, digest


_UNIFORM_VERIFY_FIELDS = frozenset((
    "plan_id", "formation_id", "semantic_version", "model_version",
    "model_sha256", "model_json", "certification", "trajectory_fields",
    "position_tolerance", "angle_tolerance", "speed_tolerance",
))
_VIRTUAL_VERIFY_FIELDS = _UNIFORM_VERIFY_FIELDS | frozenset((
    "expected_request", "expected_context", "expected_grid_sha256",
    "geometry_audit_config", "dynamic_audit_config", "expected_formation",
))


def verify_playback_plan(*, backend, **fields):
    """Finite backend dispatch for atomic playback-envelope verification.

    ``uniform_v02`` keeps the unchanged FR02 verifier.  ``virtual_rigid_a``
    uses the G5 verifier.  Any other backend, and any field that does not
    belong to the selected backend, is rejected explicitly.
    """
    if backend == PLAYBACK_PLAN_BACKEND:
        unknown = sorted(set(fields) - _UNIFORM_VERIFY_FIELDS)
        if unknown:
            raise PlaybackValidationError(
                "uniform_v02 verification does not accept {}".format(unknown))
        return verify_uniform_playback_plan(backend=backend, **fields)
    if backend == VIRTUAL_PLAYBACK_BACKEND:
        unknown = sorted(set(fields) - _VIRTUAL_VERIFY_FIELDS)
        if unknown:
            raise PlaybackValidationError(
                "virtual_rigid_a verification does not accept {}".format(
                    unknown))
        return verify_virtual_a_playback_plan(backend=backend, **fields)
    raise PlaybackValidationError(
        "unsupported playback backend {!r}; expected {!r} or {!r}".format(
            backend, PLAYBACK_PLAN_BACKEND, VIRTUAL_PLAYBACK_BACKEND))


def _canonical_mapping(value, label):
    if not isinstance(value, Mapping):
        raise PlaybackValidationError("{} must be a mapping".format(label))
    try:
        return json.loads(json.dumps(
            value, sort_keys=True, separators=(",", ":"),
            ensure_ascii=False, allow_nan=False))
    except (TypeError, ValueError) as exc:
        raise PlaybackValidationError(
            "{} is not finite JSON: {}".format(label, exc)) from exc


def _verify_union_formation_binding(binding, plan, request, context_sha,
                                    grid_sha):
    """Re-verify one retained union-scope template/certificate binding.

    The binding is planner evidence, never transport content: the sealed
    template and chain are rebuilt from their canonical payloads, the chain
    must belong to the admitted grid, and the continuous certificate must
    re-verify against the published path and the admitted planning context.
    """
    from .formation_continuous_a.corridor import corridor_chain_from_payload
    from .formation_continuous_a.formation_certificate import (
        verify_formation_certificate)
    from .formation_continuous_a.formation_corridor import (
        formation_template_from_payload)

    expected = {"scope", "planning_context_sha256", "template", "chain",
                "certificate"}
    if set(binding) != expected:
        raise PlaybackValidationError(
            "formation corridor binding keys missing={} extra={}".format(
                sorted(expected - set(binding)),
                sorted(set(binding) - expected)))
    if binding["scope"] != "three_vehicle_union_v1":
        raise PlaybackValidationError(
            "formation corridor binding scope={!r} is not the union scope"
            .format(binding["scope"]))
    if binding["planning_context_sha256"] != context_sha:
        raise PlaybackValidationError(
            "formation corridor binding planning context differs from the "
            "admitted context")
    try:
        chain = corridor_chain_from_payload(binding["chain"])
        template = formation_template_from_payload(binding["template"])
    except Exception as exc:
        raise PlaybackValidationError(
            "formation corridor binding payload was rejected: {}: {}"
            .format(type(exc).__name__, exc)) from exc
    if chain.grid_sha256 != grid_sha:
        raise PlaybackValidationError(
            "formation corridor chain grid differs from the admitted grid")
    verification = verify_formation_certificate(
        binding["certificate"], plan.path, request, chain, template,
        planning_context_sha256=context_sha)
    if not bool(verification.get("passed")):
        raise PlaybackValidationError(
            "formation corridor certificate failed: {}: {}".format(
                verification.get("reason", ""),
                verification.get("detail", "")))


def verify_virtual_a_playback_plan(*, plan_id, formation_id, backend,
                                   semantic_version, model_version,
                                   model_sha256, model_json, certification,
                                   trajectory_fields, expected_request=None,
                                   expected_context=None,
                                   expected_grid_sha256=None,
                                   expected_formation=None,
                                   geometry_audit_config=None,
                                   dynamic_audit_config=None,
                                   position_tolerance=1e-9,
                                   angle_tolerance=1e-9,
                                   speed_tolerance=1e-9):
    """Validate one atomic virtual_rigid_a envelope against its authority.

    The envelope is rebuilt from its canonical model JSON with the G4 loader.
    The transmitted digest must equal both the SHA-256 of the transmitted
    canonical bytes and ``plan.model_sha256()``.  The original physical request
    must be supplied from the retained raw ``/formation/plan_request``; the
    admitted planning context must be supplied from the planner's frozen
    admission record.  Geometry and dynamics are re-audited independently from
    the model, and every published sample is compared with
    ``state_at(actual_quantised_time)``.  A missing request or context is a
    rejection, never a default acceptance.  A plan whose admitted planner
    config declares ``three_vehicle_union_v1`` additionally requires the
    retained ``expected_formation`` binding (sealed template, corridor chain
    and continuous certificate); the certificate is re-verified against the
    published path, and a missing or stale binding is a rejection.
    """
    from .formation_continuous_a.audit import audit_dynamics, audit_geometry
    from .formation_continuous_a.contracts import (
        BACKEND, DynamicAuditConfig, G4_PLANNING_SCHEMA, GeometryAuditConfig,
        TIMED_MODEL_VERSION, grid_summary_sha256, planning_context_sha256,
        request_fingerprint, request_from_payload, request_payload,
        validate_virtual_a_request, VirtualARequest)
    from .formation_continuous_a.trajectory import VirtualATimedPlan

    if backend != VIRTUAL_PLAYBACK_BACKEND:
        raise PlaybackValidationError(
            "backend {!r} is not {!r}".format(
                backend, VIRTUAL_PLAYBACK_BACKEND))
    if semantic_version != VIRTUAL_PLAYBACK_SEMANTIC_VERSION:
        raise PlaybackValidationError(
            "unsupported playback semantic_version {!r}; expected {!r}"
            .format(semantic_version, VIRTUAL_PLAYBACK_SEMANTIC_VERSION))
    if model_version != TIMED_MODEL_VERSION:
        raise PlaybackValidationError(
            "unsupported model_version {!r}; expected {!r}".format(
                model_version, TIMED_MODEL_VERSION))
    if certification != VIRTUAL_CERTIFICATION:
        raise PlaybackValidationError(
            "unsupported certification {!r}; expected {!r}".format(
                certification, VIRTUAL_CERTIFICATION))
    if not isinstance(model_sha256, str) or len(model_sha256) != 64 or any(
            character not in "0123456789abcdef" for character in model_sha256):
        raise PlaybackValidationError(
            "model_sha256 must be 64 lowercase hex digits")
    if not isinstance(model_json, str) or not model_json.strip():
        raise PlaybackValidationError(
            "model_json must be a non-empty string")

    if plan_id != trajectory_fields.get("plan_id"):
        raise PlaybackValidationError(
            "envelope plan_id={!r} differs from the trajectory {!r}".format(
                plan_id, trajectory_fields.get("plan_id")))
    if formation_id != trajectory_fields.get("formation_id"):
        raise PlaybackValidationError(
            "envelope formation_id={!r} differs from the trajectory {!r}"
            .format(formation_id, trajectory_fields.get("formation_id")))

    if expected_request is None:
        raise PlaybackValidationError(
            "no normalized raw /formation/plan_request is retained for this "
            "plan; the virtual envelope cannot be accepted")
    if isinstance(expected_request, VirtualARequest):
        request = expected_request
    elif isinstance(expected_request, Mapping):
        try:
            request = request_from_payload(expected_request)
        except Exception as exc:
            raise PlaybackValidationError(
                "expected_request payload is invalid: {}: {}".format(
                    type(exc).__name__, exc)) from exc
    else:
        raise PlaybackValidationError(
            "expected_request must be a VirtualARequest or its payload")
    validation = validate_virtual_a_request(request)
    if not validation.passed:
        raise PlaybackValidationError(
            "expected_request is invalid: {}: {}".format(
                validation.reason, validation.detail))
    request_sha = request_fingerprint(request)

    try:
        plan = VirtualATimedPlan.from_model_json(
            model_json, expected_request=request)
    except Exception as exc:
        raise PlaybackValidationError(
            "model_json was rejected: {}: {}".format(
                type(exc).__name__, exc)) from exc
    canonical = plan.to_model_json()
    if model_json != canonical:
        raise PlaybackValidationError(
            "model_json is not the canonical authoritative model JSON")
    actual_digest = hashlib.sha256(
        model_json.encode("utf-8")).hexdigest()
    if actual_digest != model_sha256:
        raise PlaybackValidationError(
            "model_sha256 mismatch: message={} transmitted-bytes={}".format(
                model_sha256, actual_digest))
    if plan.model_sha256() != model_sha256:
        raise PlaybackValidationError(
            "model_sha256 mismatch: message={} rebuilt={}".format(
                model_sha256, plan.model_sha256()))
    if plan.request_sha256 != request_sha:
        raise PlaybackValidationError(
            "model request digest differs from the retained raw request")
    if request.plan_id != plan_id or request.formation_id != formation_id:
        raise PlaybackValidationError(
            "model request identity plan_id={} formation_id={!r} differs "
            "from the envelope {} / {!r}".format(
                request.plan_id, request.formation_id, plan_id, formation_id))

    context = plan.planning_context
    if not isinstance(context, Mapping):
        raise PlaybackValidationError(
            "virtual model carries no planning context")
    if context.get("schema") != G4_PLANNING_SCHEMA or \
            context.get("stage") != "G4":
        raise PlaybackValidationError(
            "virtual model planning context schema/stage are not G4")
    if context.get("request_sha256") != request_sha:
        raise PlaybackValidationError(
            "model planning context request_sha256 differs from the request")
    context_sha = planning_context_sha256(context)
    grid = context.get("grid")
    if not isinstance(grid, Mapping):
        raise PlaybackValidationError(
            "model planning context carries no grid summary")
    grid_sha = grid_summary_sha256(grid)
    if expected_context is None:
        raise PlaybackValidationError(
            "no admitted planning context is retained for this plan; the "
            "virtual envelope cannot be accepted")
    expected_context = _canonical_mapping(
        expected_context, "expected planning context")
    if expected_context.get("schema") != G4_PLANNING_SCHEMA or \
            expected_context.get("stage") != "G4":
        raise PlaybackValidationError(
            "expected planning context schema/stage are not G4")
    if expected_context.get("request_sha256") != request_sha:
        raise PlaybackValidationError(
            "expected planning context request_sha256 differs from the "
            "retained request")
    if _canonical_mapping(context, "model planning context") != \
            expected_context:
        raise PlaybackValidationError(
            "model planning context differs from the admitted context")
    if planning_context_sha256(expected_context) != context_sha:
        raise PlaybackValidationError(
            "planning context digest differs from the admitted context")
    expected_grid = expected_context.get("grid")
    if expected_grid_sha256 is not None:
        if expected_grid_sha256 != grid_sha:
            raise PlaybackValidationError(
                "expected grid digest differs from the model planning grid")
    elif grid_summary_sha256(expected_grid) != grid_sha:
        raise PlaybackValidationError(
            "admitted grid digest differs from the model planning grid")

    planner_config = expected_context.get("planner_config")
    if not isinstance(planner_config, Mapping):
        raise PlaybackValidationError(
            "expected planning context carries no planner config")
    model_scope = planner_config.get(
        "formation_corridor_scope", "f_only")
    if not isinstance(model_scope, str):
        raise PlaybackValidationError(
            "planning context formation_corridor_scope is not a string")
    model_scope = model_scope.strip().lower()
    if model_scope not in ("f_only", "three_vehicle_union_v1"):
        raise PlaybackValidationError(
            "planning context formation_corridor_scope={!r} is not a known "
            "scope".format(model_scope))
    if model_scope == "three_vehicle_union_v1":
        if expected_formation is None:
            raise PlaybackValidationError(
                "union-scope plan has no retained formation corridor binding; "
                "the virtual envelope cannot be accepted")
        expected_formation = _canonical_mapping(
            expected_formation, "expected formation corridor binding")
        _verify_union_formation_binding(
            expected_formation, plan, request, context_sha, grid_sha)
    elif expected_formation is not None and expected_formation != {}:
        raise PlaybackValidationError(
            "f_only plan carries an unexpected formation corridor binding")

    if geometry_audit_config is None:
        geometry_audit_config = GeometryAuditConfig()
    if not isinstance(geometry_audit_config, GeometryAuditConfig):
        raise PlaybackValidationError(
            "geometry_audit_config must be GeometryAuditConfig")
    if dynamic_audit_config is None:
        dynamic_audit_config = DynamicAuditConfig(
            geometry_audit_config=geometry_audit_config)
    if not isinstance(dynamic_audit_config, DynamicAuditConfig):
        raise PlaybackValidationError(
            "dynamic_audit_config must be DynamicAuditConfig")
    geometry = audit_geometry(
        request, plan.path, geometry_audit_config,
        planning_context=expected_context)
    if not bool(getattr(geometry, "passed", False)):
        raise PlaybackValidationError(
            "independent geometry audit failed: {}: {}".format(
                getattr(geometry, "reason", ""),
                getattr(geometry, "detail", "")))
    dynamics = audit_dynamics(
        request, plan, dynamic_audit_config,
        expected_planning_context=expected_context)
    if not bool(getattr(dynamics, "passed", False)):
        raise PlaybackValidationError(
            "independent dynamics audit failed: {}: {}".format(
                getattr(dynamics, "reason", ""),
                getattr(dynamics, "detail", "")))
    for report, label in ((geometry, "geometry"), (dynamics, "dynamics")):
        digest = getattr(report, "planning_context_sha256", "")
        if digest and digest != context_sha:
            raise PlaybackValidationError(
                "{} audit planning context digest differs".format(label))
        digest = getattr(report, "request_sha256", "")
        if digest and digest != request_sha:
            raise PlaybackValidationError(
                "{} audit request digest differs".format(label))

    try:
        trajectory = trajectory_from_fields(trajectory_fields)
    except PlaybackValidationError:
        raise
    except Exception as exc:
        raise PlaybackValidationError(
            "trajectory fields are invalid: {}".format(exc)) from exc
    if trajectory.frame_id != "map":
        raise PlaybackValidationError(
            "virtual trajectory frame_id={!r}; expected 'map'".format(
                trajectory.frame_id))
    times = trajectory.times_from_start
    duration_t = plan.duration_s
    if plan.kind == "hold":
        if plan.total_S_m != 0.0 or duration_t != 0.0:
            raise PlaybackValidationError(
                "hold model must have S=T=0")
        if len(times) != 1 or abs(times[0]) > 1e-12:
            raise PlaybackValidationError(
                "a hold plan publishes exactly one sample at t=0")
    else:
        if not times:
            raise PlaybackValidationError("trajectory has no samples")
        if not (duration_t - 1e-12 <= times[-1] <= duration_t + 2e-9):
            raise PlaybackValidationError(
                "trajectory duration {:.17g} is not the smallest integer-"
                "nanosecond endpoint of T={:.17g}".format(
                    times[-1], duration_t))
        for index, value in enumerate(times):
            scaled = value * 1e9
            if not math.isfinite(scaled) or \
                    abs(scaled - round(scaled)) > 5e-6:
                raise PlaybackValidationError(
                    "sample {} time {:.17g} is not an integer-nanosecond "
                    "time".format(index, value))
            if index > 0 and not times[index] > times[index - 1]:
                raise PlaybackValidationError(
                    "sample times are not strictly increasing")
    for index, value in enumerate(times):
        scaled = value * 1e9
        if not math.isfinite(scaled) or \
                abs(scaled - round(scaled)) > 5e-6:
            raise PlaybackValidationError(
                "sample {} time {:.17g} is not an integer-nanosecond time"
                .format(index, value))
    declared_ns = trajectory_fields.get("times_ns")
    if declared_ns is not None:
        declared_ns = tuple(declared_ns)
        if len(declared_ns) != len(times):
            raise PlaybackValidationError(
                "times_ns count differs from the common time axis")
        for index, (value, declared) in enumerate(zip(times, declared_ns)):
            if isinstance(declared, bool) or not isinstance(declared, int):
                raise PlaybackValidationError(
                    "times_ns[{}] is not an integer".format(index))
            if declared < 0 or declared != int(round(value * 1e9)):
                raise PlaybackValidationError(
                    "times_ns[{}]={} differs from the actual quantised time "
                    "{:.17g}".format(index, declared, value))
        for item in trajectory_fields.get("trajectories", ()):
            vehicle_id = int(item["vehicle_id"])
            point_ns = item.get("point_times_ns")
            if point_ns is None:
                raise PlaybackValidationError(
                    "vehicle {} has times_ns but no point_times_ns"
                    .format(vehicle_id))
            point_ns = tuple(point_ns)
            if point_ns != declared_ns:
                raise PlaybackValidationError(
                    "vehicle {} point_times_ns differs from times_ns"
                    .format(vehicle_id))

    model_offsets = {
        offset.vehicle_id: (float(offset.x), float(offset.y))
        for offset in request.model.offsets}
    model_shapes = {
        vehicle_id: (float(shape.length_m), float(shape.width_m))
        for vehicle_id, shape in request.vehicle_shapes.items()}
    field_vehicles = trajectory_fields.get("vehicles", ())
    field_by_id = {}
    for item in field_vehicles:
        vehicle_id = int(item["vehicle_id"])
        if vehicle_id in field_by_id:
            raise PlaybackValidationError(
                "duplicate vehicle_id {} in the envelope".format(vehicle_id))
        field_by_id[vehicle_id] = item
    if set(field_by_id) != set(model_offsets):
        raise PlaybackValidationError(
            "envelope vehicle_ids {} differ from the model {}".format(
                sorted(field_by_id), sorted(model_offsets)))

    start_centers = request.model.centers(request.start_qf)
    goal_centers = request.model.centers(request.goal_qf)
    policy = request.goal_policy
    policy_value = str(getattr(policy, "value", policy)).lower()
    free_flags = {}
    for vehicle_id in sorted(model_offsets):
        item = field_by_id[vehicle_id]
        free_flags[vehicle_id] = bool(item.get("goal_heading_free", False))
        offset = item["offset"]
        _check_close(offset[0], model_offsets[vehicle_id][0],
                     "vehicle {} offset_x".format(vehicle_id),
                     position_tolerance)
        _check_close(offset[1], model_offsets[vehicle_id][1],
                     "vehicle {} offset_y".format(vehicle_id),
                     position_tolerance)
        shape = model_shapes[vehicle_id]
        _check_close(float(item["body_length_m"]), shape[0],
                     "vehicle {} body_length_m".format(vehicle_id),
                     position_tolerance)
        _check_close(float(item["body_width_m"]), shape[1],
                     "vehicle {} body_width_m".format(vehicle_id),
                     position_tolerance)
        start = item.get("start_center")
        if start is None:
            raise PlaybackValidationError(
                "vehicle {} carries no start_center".format(vehicle_id))
        _check_close(start[0], start_centers[vehicle_id][0],
                     "vehicle {} start_center.x".format(vehicle_id),
                     position_tolerance)
        _check_close(start[1], start_centers[vehicle_id][1],
                     "vehicle {} start_center.y".format(vehicle_id),
                     position_tolerance)
        _check_close(math.atan2(math.sin(
            start[2] - request.start_headings[vehicle_id]),
            math.cos(start[2] - request.start_headings[vehicle_id])), 0.0,
            "vehicle {} real start heading".format(vehicle_id),
            angle_tolerance)
        goal = item.get("goal_center")
        if goal is None:
            raise PlaybackValidationError(
                "vehicle {} carries no goal_center".format(vehicle_id))
        _check_close(goal[0], goal_centers[vehicle_id][0],
                     "vehicle {} goal_center.x".format(vehicle_id),
                     position_tolerance)
        _check_close(goal[1], goal_centers[vehicle_id][1],
                     "vehicle {} goal_center.y".format(vehicle_id),
                     position_tolerance)
        if not free_flags[vehicle_id]:
            _check_close(math.atan2(math.sin(
                goal[2] - request.goal_qf.psi),
                math.cos(goal[2] - request.goal_qf.psi)), 0.0,
                "vehicle {} fixed goal heading".format(vehicle_id),
                angle_tolerance)
    if policy_value == "free":
        if not all(free_flags.values()):
            raise PlaybackValidationError(
                "FREE model request carries a fixed goal heading flag")
    elif policy_value == "straight":
        if any(free_flags.values()):
            raise PlaybackValidationError(
                "STRAIGHT model request carries a free goal heading flag")
        for vehicle_id in sorted(model_offsets):
            _check_close(math.atan2(math.sin(
                float(field_by_id[vehicle_id]["goal_center"][2])
                - request.goal_qf.psi),
                math.cos(float(field_by_id[vehicle_id]["goal_center"][2])
                         - request.goal_qf.psi)), 0.0,
                "vehicle {} straight goal heading".format(vehicle_id),
                angle_tolerance)
    else:
        raise PlaybackValidationError(
            "unknown model goal policy {!r}".format(policy_value))

    if not trajectory.formation_poses:
        raise PlaybackValidationError("trajectory has no formation poses")
    start_state = plan.state_at(0.0)
    terminal_state = plan.state_at(duration_t)
    first_pose = trajectory.formation_poses[0]
    last_pose = trajectory.formation_poses[-1]
    _check_close(first_pose[0], start_state.qf.x, "start formation x",
                 position_tolerance)
    _check_close(first_pose[1], start_state.qf.y, "start formation y",
                 position_tolerance)
    _check_close(math.atan2(math.sin(first_pose[2] - start_state.qf.psi),
                            math.cos(first_pose[2] - start_state.qf.psi)),
                 0.0, "start formation psi", angle_tolerance)
    _check_close(last_pose[0], terminal_state.qf.x,
                 "terminal formation x", position_tolerance)
    _check_close(last_pose[1], terminal_state.qf.y,
                 "terminal formation y", position_tolerance)
    _check_close(math.atan2(math.sin(
        last_pose[2] - terminal_state.qf.psi),
        math.cos(last_pose[2] - terminal_state.qf.psi)),
        0.0, "terminal formation psi", angle_tolerance)
    _check_close(math.hypot(
        terminal_state.qf.x - request.goal_qf.x,
        terminal_state.qf.y - request.goal_qf.y), 0.0,
        "terminal vs requested goal position",
        float(geometry_audit_config.position_tolerance_m))
    _check_close(math.atan2(math.sin(
        terminal_state.qf.psi - request.goal_qf.psi),
        math.cos(terminal_state.qf.psi - request.goal_qf.psi)), 0.0,
        "terminal vs requested goal heading",
        float(geometry_audit_config.heading_tolerance_rad))

    raw_points = {
        int(item["vehicle_id"]): tuple(item["points"])
        for item in trajectory_fields.get("trajectories", ())}
    if set(raw_points) != set(model_offsets):
        raise PlaybackValidationError(
            "trajectory vehicle_ids {} differ from the model {}".format(
                sorted(raw_points), sorted(model_offsets)))
    for index, time_s in enumerate(times):
        state = plan.state_at(time_s)
        _check_close(state.qf.x, trajectory.formation_poses[index][0],
                     "formation_poses[{}].x".format(index),
                     position_tolerance)
        _check_close(state.qf.y, trajectory.formation_poses[index][1],
                     "formation_poses[{}].y".format(index),
                     position_tolerance)
        _check_close(math.atan2(math.sin(
            state.qf.psi - trajectory.formation_poses[index][2]),
            math.cos(state.qf.psi - trajectory.formation_poses[index][2])),
            0.0, "formation_poses[{}].psi".format(index), angle_tolerance)
        state_by_id = {vehicle.vehicle_id: vehicle
                       for vehicle in state.vehicles}
        for vehicle_id in sorted(model_offsets):
            samples = raw_points[vehicle_id]
            if len(samples) != len(times):
                raise PlaybackValidationError(
                    "vehicle {} has {} samples, expected {}".format(
                        vehicle_id, len(samples), len(times)))
            sample = samples[index]
            if len(sample) < 8:
                raise PlaybackValidationError(
                    "vehicle {} sample {} has fewer than 8 fields".format(
                        vehicle_id, index))
            vehicle = state_by_id[vehicle_id]
            _check_close(float(sample[0]), time_s,
                         "vehicle {} sample {} time".format(
                             vehicle_id, index), 1e-9)
            _check_close(float(sample[1]), vehicle.x,
                         "vehicle {} sample {} x".format(vehicle_id, index),
                         position_tolerance)
            _check_close(float(sample[2]), vehicle.y,
                         "vehicle {} sample {} y".format(vehicle_id, index),
                         position_tolerance)
            _check_close(math.atan2(math.sin(
                float(sample[3]) - vehicle.theta),
                math.cos(float(sample[3]) - vehicle.theta)), 0.0,
                "vehicle {} sample {} theta".format(vehicle_id, index),
                angle_tolerance)
            _check_close(float(sample[4]), vehicle.v_i,
                         "vehicle {} sample {} center speed".format(
                             vehicle_id, index), speed_tolerance)
            _check_close(float(sample[5]), vehicle.omega_i,
                         "vehicle {} sample {} yaw rate".format(
                             vehicle_id, index), speed_tolerance)
            if int(sample[6]) != ACKERMANN:
                raise PlaybackValidationError(
                    "vehicle {} sample {} mode {} differs from the unified A "
                    "mode {}".format(vehicle_id, index, sample[6], ACKERMANN))
            _check_close(float(sample[7]), vehicle.C_i,
                         "vehicle {} sample {} mode_control".format(
                             vehicle_id, index), speed_tolerance)

    return replace(
        trajectory, virtual_a_plan=plan, model_sha256=model_sha256,
        backend=VIRTUAL_PLAYBACK_BACKEND), plan, model_sha256


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
    if trajectory.virtual_a_plan is not None:
        return _virtual_frame_at(trajectory, elapsed)
    if trajectory.uniform_plan is not None:
        return _uniform_frame_at(trajectory, elapsed)
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


def _uniform_frame_at(trajectory, elapsed):
    """Derive the display frame from the FR02 analytic model.

    No published point column is interpolated here.  Position, heading, mode,
    control and speed all come from ``UniformTimedPlan.state_at`` at the same
    common clock; non-zero-pivot cargo arcs therefore stay exact between
    samples, and each vehicle centre is derived from ``q_F`` by the model.
    """
    times = trajectory.times_from_start
    duration = trajectory.duration
    if elapsed <= 0.0:
        lower = upper = 0
        fraction = 0.0
        state_name = HOLD_START
        clamped = 0.0
    elif elapsed >= duration:
        lower = upper = len(times) - 1
        fraction = 0.0
        state_name = HOLD_END
        clamped = duration
    else:
        upper = bisect_right(times, elapsed)
        lower = max(0, upper - 1)
        span = times[upper] - times[lower]
        if span <= 0.0:
            raise PlaybackValidationError(
                "shared time axis is not strictly increasing")
        fraction = (elapsed - times[lower]) / span
        if fraction <= 0.0:
            fraction = 0.0
        state_name = PLAYING
        clamped = elapsed
    try:
        state = trajectory.uniform_plan.state_at(clamped)
    except Exception as exc:
        raise PlaybackValidationError(
            "authoritative model cannot evaluate t={:.17g}: {}".format(
                clamped, exc)) from exc
    if state.qf is None:
        raise PlaybackValidationError("model state has no formation pose")
    specs = trajectory.vehicle_map()
    states = []
    for vehicle in state.vehicles:
        spec = specs.get(vehicle.vehicle_id)
        if spec is None:
            raise PlaybackValidationError(
                "model vehicle_id={} is absent from the envelope".format(
                    vehicle.vehicle_id))
        mode = {"A": ACKERMANN, "O": OBLIQUE, "S": SPIN}[state.mode]
        states.append(PlaybackVehicleState(
            vehicle_id=vehicle.vehicle_id,
            center_x=float(vehicle.x),
            center_y=float(vehicle.y),
            theta=float(vehicle.theta),
            mode=mode,
            mode_control=float(vehicle.mode_control),
            center_speed_mps=float(vehicle.center_speed_mps),
            offset_x=spec.offset_x,
            offset_y=spec.offset_y,
            body_length_m=spec.body_length_m,
            body_width_m=spec.body_width_m))
    return PlaybackFrame(
        plan_id=trajectory.plan_id,
        formation_id=trajectory.formation_id,
        frame_id=trajectory.frame_id,
        stamp=trajectory.stamp,
        duration_s=duration,
        elapsed_s=elapsed,
        clamped_elapsed_s=clamped,
        state=state_name,
        lower_index=lower,
        upper_index=upper,
        fraction=fraction,
        formation_x=float(state.qf.x),
        formation_y=float(state.qf.y),
        formation_psi=float(state.qf.psi),
        vehicles=tuple(states))


def _virtual_frame_at(trajectory, elapsed):
    """Derive the display frame from the G4 authoritative timed model.

    Every position, heading, speed, yaw rate and curvature comes from one
    ``VirtualATimedPlan.state_at`` call at the same common-clock time.  No
    published sample column is interpolated between samples, so a curvature
    arc between two 20 ms samples is still the model's arc.
    """
    times = trajectory.times_from_start
    duration = trajectory.duration
    if elapsed <= 0.0:
        lower = upper = 0
        fraction = 0.0
        state_name = HOLD_START
        clamped = 0.0
    elif elapsed >= duration:
        lower = upper = len(times) - 1
        fraction = 0.0
        state_name = HOLD_END
        clamped = duration
    else:
        upper = bisect_right(times, elapsed)
        lower = max(0, upper - 1)
        span = times[upper] - times[lower]
        if span <= 0.0:
            raise PlaybackValidationError(
                "shared time axis is not strictly increasing")
        fraction = (elapsed - times[lower]) / span
        if fraction <= 0.0:
            fraction = 0.0
        state_name = PLAYING
        clamped = elapsed
    try:
        state = trajectory.virtual_a_plan.state_at(clamped)
    except Exception as exc:
        raise PlaybackValidationError(
            "virtual model cannot evaluate t={:.17g}: {}".format(
                clamped, exc)) from exc
    specs = trajectory.vehicle_map()
    states = []
    for vehicle in state.vehicles:
        spec = specs.get(vehicle.vehicle_id)
        if spec is None:
            raise PlaybackValidationError(
                "model vehicle_id={} is absent from the envelope".format(
                    vehicle.vehicle_id))
        states.append(PlaybackVehicleState(
            vehicle_id=vehicle.vehicle_id,
            center_x=float(vehicle.x),
            center_y=float(vehicle.y),
            theta=float(vehicle.theta),
            mode=ACKERMANN,
            mode_control=float(vehicle.C_i),
            center_speed_mps=float(vehicle.v_i),
            offset_x=spec.offset_x,
            offset_y=spec.offset_y,
            body_length_m=spec.body_length_m,
            body_width_m=spec.body_width_m))
    return PlaybackFrame(
        plan_id=trajectory.plan_id,
        formation_id=trajectory.formation_id,
        frame_id=trajectory.frame_id,
        stamp=trajectory.stamp,
        duration_s=duration,
        elapsed_s=elapsed,
        clamped_elapsed_s=clamped,
        state=state_name,
        lower_index=lower,
        upper_index=upper,
        fraction=fraction,
        formation_x=float(state.qf.x),
        formation_y=float(state.qf.y),
        formation_psi=float(state.qf.psi),
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
    # The long map-anchored status block is useful in the legacy demo but
    # obscures the fixed-corner HUD on a 1280x720 render viewport.  Both
    # switches default to the historical full display.
    show_status_text: bool = True

    def __post_init__(self):
        for name in (
                "band_width_m", "history_width_m", "body_width_m",
                "heading_width_m", "link_width_m", "reference_scale_m",
                "text_size_m", "status_text_size_m", "text_height_m",
                "arrow_head_length_m"):
            object.__setattr__(self, name, _positive(
                getattr(self, name), "PlaybackMarkerOptions." + name))
        if not isinstance(self.show_status_text, bool):
            raise PlaybackValidationError(
                "PlaybackMarkerOptions.show_status_text must be a bool")


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


def _status_label(status, plan_id, model_sha256=None, formation_id=None,
                  backend=None):
    """Interpret the optimizer status without ever upgrading its claims.

    ``model_sha256`` is the digest of the verified uniform_v02 envelope this
    display belongs to.  When it is present the audit status must carry the
    same plan_id, formation_id, backend and digest; a status for another
    model, or one missing the identity fields, can never certify this
    display.  Legacy trajectories pass None and keep the historical
    plan_id-only rule.
    """
    if not isinstance(status, Mapping):
        return "AUDIT_PENDING", False, GEOMETRY_ONLY_LABEL, "no status report"
    status_plan = status.get("plan_id")
    identity_ok = True
    if model_sha256 is not None:
        identity_ok = bool(
            status.get("model_sha256") == model_sha256
            and status.get("formation_id") == formation_id
            and status.get("backend") == backend)
    executable = bool(
        status.get("trajectory_published") and status.get("executable")
        and status.get("feasible") and status_plan == plan_id
        and identity_ok)
    if executable:
        if status.get("status") == "NO_MOTION":
            return "NO_MOTION", True, "NO_MOTION", (
                "audited zero-motion hold; static state published")
        return "FEASIBLE", True, "FEASIBLE", (
            "independent audit and transactional publication")
    if not identity_ok:
        return "GEOMETRY_ONLY", False, GEOMETRY_ONLY_LABEL, (
            "audit status identity does not match the displayed model "
            "(plan_id/formation_id/backend/model_sha256)")
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


def quality_identity_matches(playback_status, quality):
    """Whether a quality report belongs to exactly this displayed model.

    The planner's quality reports can arrive before or after the plan they
    describe, and their metrics are copied into the status label.  A report
    with the same plan number but another formation, backend, model digest or
    planning context is stale for this display.  Legacy trajectories without a
    bound model never accept a quality block.
    """
    if not isinstance(playback_status, Mapping) or not isinstance(
            quality, Mapping):
        return False
    plan_id = playback_status.get("plan_id")
    formation_id = playback_status.get("formation_id")
    backend = playback_status.get("backend")
    model_sha256 = playback_status.get("model_sha256")
    if (not isinstance(plan_id, int) or isinstance(plan_id, bool) or
            not isinstance(formation_id, str) or not formation_id or
            backend not in (PLAYBACK_PLAN_BACKEND,
                            VIRTUAL_PLAYBACK_BACKEND) or
            not isinstance(model_sha256, str) or not model_sha256):
        return False
    if (quality.get("plan_id") != plan_id or
            quality.get("formation_id") != formation_id or
            quality.get("backend") != backend or
            quality.get("model_sha256") != model_sha256):
        return False
    if backend == VIRTUAL_PLAYBACK_BACKEND:
        request_sha = playback_status.get("request_sha256")
        context_sha = playback_status.get("planning_context_sha256")
        if (not isinstance(request_sha, str) or not request_sha or
                not isinstance(context_sha, str) or not context_sha):
            return False
        if (quality.get("request_sha256") != request_sha or
                quality.get("planning_context_sha256") != context_sha):
            return False
    return True


def _audit_columns(playback_status, quality):
    """Three independent audit columns from a matching three-layer report.

    ``PASS`` is written only when the report identity matches this playback
    and the layer's own scope and ``passed`` flag are present.  An IPOPT
    success or a planning-stage message can never populate these columns.
    """
    columns = {
        "geometry": "UNKNOWN",
        "dynamics": "UNKNOWN",
        "collision": "UNKNOWN",
    }
    if not quality_identity_matches(playback_status, quality):
        return columns
    virtual_result = quality.get("virtual_result")
    if not isinstance(virtual_result, Mapping):
        return columns
    audits = virtual_result.get("audits")
    if not isinstance(audits, Mapping):
        return columns
    for key, scope in (
            ("geometry", GEOMETRY_SCOPE),
            ("dynamics", DYNAMICS_SCOPE),
            ("collision", COLLISION_SCOPE)):
        report = audits.get(key)
        if not isinstance(report, Mapping):
            columns[key] = "PENDING"
            continue
        if report.get("scope") != scope:
            continue
        passed = report.get("passed")
        if passed is True:
            columns[key] = "PASS"
        elif passed is False:
            columns[key] = "FAIL"
        else:
            columns[key] = "PENDING"
    return columns


def _hud_vehicle_line(vehicle, static=False):
    if not isinstance(vehicle, Mapping):
        raise PlaybackValidationError("HUD vehicle entry must be an object")
    vehicle_id = vehicle.get("vehicle_id")
    if not isinstance(vehicle_id, int) or isinstance(vehicle_id, bool):
        raise PlaybackValidationError("HUD vehicle_id must be an integer")
    if static:
        mode = "STATIC"
        speed = 0.0
    else:
        mode = vehicle.get("mode_name")
        if mode not in MODE_NAMES.values():
            raise PlaybackValidationError(
                "HUD mode_name must be A, O or S")
        speed = _finite(vehicle.get("center_speed_mps"),
                        "HUD center_speed_mps")
    if abs(speed) < 0.005:
        speed = 0.0
    return "car{} | {} | v={:.2f} m/s".format(vehicle_id, mode, speed)


def _hud_diagnostic_count(value):
    """Reject a malformed count; display-only fields never invent a number."""
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return int(value)


def planning_diagnostics_view(document):
    """Bounded diagnostics copied from one real progress document.

    Missing or malformed diagnostic fields stay ``None``; the HUD then prints
    ``unknown`` rather than a fabricated count or percentage.  No geometry is
    copied here; markers/paths remain in their own topic layers.
    """
    if not isinstance(document, Mapping):
        return None
    metrics = document.get("metrics")
    if not isinstance(metrics, Mapping):
        return None
    view = {}
    source = metrics.get("guide_source")
    if isinstance(source, str) and source:
        view["guide_source"] = source[:80]
    origin = metrics.get("selected_candidate_origin")
    if isinstance(origin, str) and origin:
        view["selected_candidate_origin"] = origin[:80]
    index = _hud_diagnostic_count(metrics.get("selected_candidate_index"))
    if index is not None:
        view["selected_candidate_index"] = index
    count = _hud_diagnostic_count(metrics.get("guide_candidate_count"))
    if count is not None:
        view["guide_candidate_count"] = count
    rule = metrics.get("guide_selection_rule")
    if isinstance(rule, str) and rule:
        view["guide_selection_rule"] = rule[:80]
    assessment = metrics.get("assessment")
    if isinstance(assessment, Mapping):
        if assessment.get("available") is True:
            copied = {"available": True}
            status = assessment.get("map_sample_status")
            if isinstance(status, str) and status:
                copied["map_sample_status"] = status[:64]
            for name in ("station_count", "map_collision_count",
                         "single_region_count", "union_covered_count",
                         "union_gap_count", "union_unknown_count"):
                value = _hud_diagnostic_count(assessment.get(name))
                if value is not None:
                    copied[name] = value
            view["assessment"] = copied
        else:
            view["assessment"] = {"available": False}
    elif assessment is None:
        view["assessment"] = None
    return view


def _hud_diagnostics_line(diagnostics):
    """Two compact true-count lines; absent metrics are printed as unknown."""
    if diagnostics is None:
        return None
    if not isinstance(diagnostics, Mapping):
        raise PlaybackValidationError(
            "HUD diagnostics must be an object or null")
    source = diagnostics.get("guide_source")
    parts = ["guide={}".format(
        source if isinstance(source, str) and source else "unknown")]
    origin = diagnostics.get("selected_candidate_origin")
    if isinstance(origin, str) and origin:
        parts.append("origin={}".format(origin))
    index = _hud_diagnostic_count(
        diagnostics.get("selected_candidate_index"))
    count = _hud_diagnostic_count(
        diagnostics.get("guide_candidate_count"))
    if index is not None:
        parts.append("selected={}/{}".format(
            index, "?" if count is None else count))
    rule = diagnostics.get("guide_selection_rule")
    if isinstance(rule, str) and rule:
        parts.append(rule)
    assessment = diagnostics.get("assessment")
    if isinstance(assessment, Mapping) and \
            assessment.get("available") is True:
        parts = ["diagnostics | " + " | ".join(parts)]
        parts.append(
            "samples={} | map_collisions={} | "
            "single={} | union={} | gap={} | unknown={} | status={}".format(
                _unknown_count(assessment.get("station_count")),
                _unknown_count(assessment.get("map_collision_count")),
                _unknown_count(assessment.get("single_region_count")),
                _unknown_count(assessment.get("union_covered_count")),
                _unknown_count(assessment.get("union_gap_count")),
                _unknown_count(assessment.get("union_unknown_count")),
                assessment.get("map_sample_status") or "UNKNOWN"))
        return "\n".join(parts)
    parts = ["diagnostics | " + " | ".join(parts)]
    parts.append(
        "samples=unknown | map_collisions=unknown | "
        "single=unknown | union=unknown | gap=unknown | unknown=unknown")
    return "\n".join(parts)


def _unknown_count(value):
    return "unknown" if value is None else str(value)


def _hud_progress_line(progress):
    """One compact planning line; large geometry is never repeated here."""
    if progress is None:
        return None
    if not isinstance(progress, Mapping):
        raise PlaybackValidationError("HUD progress must be an object or null")
    stage = progress.get("stage")
    if not isinstance(stage, str) or stage not in tuple(PROGRESS_STAGES) + (
            "FAILED",):
        stage = "UNKNOWN"
    parts = ["planning {}".format(stage)]
    stage_index = progress.get("stage_index")
    if (isinstance(stage_index, int) and not isinstance(stage_index, bool)
            and 0 <= stage_index <= 8):
        parts.append("step {}/8".format(stage_index))
    route_index = progress.get("route_index")
    route_count = progress.get("route_count")
    if (isinstance(route_index, int) and not isinstance(route_index, bool)
            and isinstance(route_count, int)
            and not isinstance(route_count, bool) and route_count > 0):
        parts.append("route {}/{}".format(route_index, route_count))
    elif isinstance(route_count, int) and not isinstance(route_count, bool) \
            and route_count > 0:
        parts.append("routes {}".format(route_count))
    round_index = progress.get("round_index")
    if isinstance(round_index, int) and not isinstance(round_index, bool):
        parts.append("round {}".format(round_index))
    elapsed = progress.get("elapsed_s")
    if (isinstance(elapsed, (int, float)) and not isinstance(elapsed, bool)
            and math.isfinite(float(elapsed)) and float(elapsed) >= 0.0):
        # The playback node extrapolates this value at its 10 Hz refresh; it
        # carries no geometry and no planner-budget claim.
        parts.append("elapsed {:.1f} s".format(float(elapsed)))
    reason = progress.get("reason")
    if isinstance(reason, str) and reason:
        parts.append(reason[:80])
    line = " | ".join(parts)
    diagnostics = _hud_diagnostics_line(progress.get("diagnostics"))
    if diagnostics is None and stage in ("CORRIDOR_BUILD", "PORTAL_GUIDE"):
        diagnostics = _hud_diagnostics_line({})
    return line if diagnostics is None else line + "\n" + diagnostics


def hud_text(playback_status, job_state, quality, show_cargo_reason=True,
             progress=None):
    """Fixed-corner HUD text for the current playback sample and audit state.

    ``playback_status`` is the strict playback document produced by
    :func:`status_document`.  Its ``audit_status``/``certification`` already
    came from :func:`_status_label`, so this formatter reuses that identity
    decision rather than introducing a second certification rule.  The
    quality block is used only for the three audit columns and only after
    :func:`quality_identity_matches` accepts its full identity.

    A STATIC hold repeats the white Cargo label in the fixed corner.  The
    optional reason follows the existing :func:`build_hold_markers` switch so
    the HUD and the map label show the same text in the interactive demo.
    """
    if not isinstance(playback_status, Mapping):
        raise PlaybackValidationError("HUD playback_status must be an object")
    if not isinstance(job_state, str):
        raise PlaybackValidationError("HUD job_state must be a string")
    if quality is not None and not isinstance(quality, Mapping):
        raise PlaybackValidationError("HUD quality must be an object or null")
    if not isinstance(show_cargo_reason, bool):
        raise PlaybackValidationError(
            "HUD show_cargo_reason must be a bool")
    plan_id = playback_status.get("plan_id")
    formation_id = playback_status.get("formation_id")
    backend = playback_status.get("backend")
    state = playback_status.get("state", "UNKNOWN")
    lines = [
        "FORMATION HUD | plan {}".format(
            "-" if plan_id is None else plan_id),
        "formation {} | {}".format(
            "-" if not formation_id else formation_id,
            "-" if not backend else backend),
        "job {} | playback {}".format(job_state or "UNKNOWN", state),
    ]
    progress_line = _hud_progress_line(progress)
    if progress_line is not None:
        lines.append(progress_line)
    duration = playback_status.get("duration_s")
    elapsed = playback_status.get("elapsed_s")
    if isinstance(duration, (int, float)) and not isinstance(duration, bool):
        lines.append("T_end {:.2f} s | elapsed {:.2f} s".format(
            float(duration),
            float(elapsed) if isinstance(elapsed, (int, float)) and
            not isinstance(elapsed, bool) else 0.0))

    certification = playback_status.get("certification") or "AUDIT_PENDING"
    if playback_status.get("executable") is True:
        lines.append("audit {}".format(certification))
    else:
        lines.append("audit {} (GEOMETRY_ONLY)".format(certification))

    vehicles = playback_status.get("vehicles")
    if isinstance(vehicles, (list, tuple)) and vehicles:
        for vehicle in sorted(
                vehicles,
                key=lambda item: (item.get("vehicle_id")
                                  if isinstance(item, Mapping) else -1)):
            lines.append(_hud_vehicle_line(
                vehicle, static=(state == "STATIC")))
    else:
        lines.append("STATIC | v=0.00 m/s")

    if state == "STATIC":
        cargo_text = "Cargo"
        reason = playback_status.get("reason")
        if show_cargo_reason and reason:
            cargo_text += " | " + str(reason)
        lines.append(cargo_text)
        lines.append("STATIC / no executable trajectory")

    columns = _audit_columns(playback_status, quality)
    if (quality is None and plan_id is not None and
            columns["geometry"] == "UNKNOWN" and
            columns["dynamics"] == "UNKNOWN" and
            columns["collision"] == "UNKNOWN" and
            job_state in ("PLANNING", "PLAYING")):
        # No report has arrived for this plan yet.  A received but
        # mismatching report stays UNKNOWN: its stages do not describe this
        # playback.
        columns = {key: "PENDING" for key in columns}
    lines.append("AUDIT geometry={} dynamics={}".format(
        columns["geometry"], columns["dynamics"]))
    lines.append("      collision={}".format(columns["collision"]))
    return "\n".join(lines)


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
        # The full path is published separately as nav_msgs/Path per vehicle
        # so RViz can control its colour and line width.  A MarkerArray
        # LINE_STRIP cannot expose either property.
        samples = trajectory.points[vehicle.vehicle_id]
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
        # Concise body label: physical vehicle id, mode and centre speed.
        speed = state.center_speed_mps
        if abs(speed) < 0.005:
            speed = 0.0
        add(namespace, "TEXT_VIEW_FACING",
            position=(state.center_x, state.center_y,
                      options.text_height_m),
            text="car{} | {} | v={:.2f} m/s".format(
                state.vehicle_id, state.mode_name, speed),
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
    # A verified envelope additionally binds the label to its own backend and
    # model digest; legacy trajectories keep the plan_id-only rule.
    audit_digest = trajectory.model_sha256
    audit_backend = trajectory.backend or PLAYBACK_PLAN_BACKEND
    audit_state, executable, label, detail = _status_label(
        status, trajectory.plan_id, audit_digest,
        trajectory.formation_id if audit_digest is not None else None,
        audit_backend if audit_digest is not None else None)
    status_color = STATUS_OK_COLOR if executable else STATUS_FAIL_COLOR
    status_text = (
        "plan {} | {} | {}\nT_end={:.3f} s | {} / {}\n{}".format(
            trajectory.plan_id, frame.state, label,
            trajectory.duration, audit_state, detail,
            _metrics_text(status)))
    if options.show_status_text:
        add(shared_namespace(formation_id, "status"), "TEXT_VIEW_FACING",
            position=(frame.formation_x, frame.formation_y,
                      3.4 * options.text_height_m),
            text=status_text,
            color=status_color,
            scale=(0.0, 0.0, options.status_text_size_m))

    namespaces = tuple(sorted({marker.namespace for marker in markers}))
    return tuple(markers), namespaces, audit_state, executable


def vehicle_path_samples(trajectory):
    """Full centre path of every vehicle as ``(x, y, theta)`` tuples.

    RViz ``nav_msgs/Path`` displays expose colour and line width; the marker
    array carries only the current pose, OBB, heading, traveled history and
    labels.  The returned samples are the authoritative common-time samples
    already verified by the envelope; no interpolation is added here.
    """
    if not isinstance(trajectory, FormationPlaybackTrajectory):
        raise PlaybackValidationError(
            "expected FormationPlaybackTrajectory")
    return {
        vehicle.vehicle_id: tuple(
            (sample.center_x, sample.center_y, sample.theta)
            for sample in trajectory.points[vehicle.vehicle_id])
        for vehicle in trajectory.vehicles}


def clear_marker_specs(frame_id="map"):
    """One DELETEALL marker that removes every previous formation marker."""
    return (_marker(
        _NS_ROOT, 0, "DELETEALL", _text(frame_id, "frame_id"),
        action="DELETEALL"),)


CORRIDOR_FILL_Z_M = 0.005
CORRIDOR_EDGE_Z_M = 0.01
CORRIDOR_FILL_ALPHA = 0.09
CORRIDOR_EDGE_ALPHA = 0.55
# TASK 01: one fixed colour for every corridor polygon.  The layer is a
# candidate F-point workspace display, not an execution certificate.  The
# soft red fill and deep-rose edge keep the corridor behind every path.
CORRIDOR_FILL_COLOR = (232 / 255, 121 / 255, 134 / 255)
CORRIDOR_EDGE_COLOR = (180 / 255, 83 / 255, 98 / 255)
CORRIDOR_FILL_NAMESPACE = "formation/planning/corridor_f_only/fill"
CORRIDOR_EDGE_NAMESPACE = "formation/planning/corridor_f_only/edge"

# Progressive planning debug document (TASK 01 Step 1).  PLANNING is emitted
# while the solver is still running; FAILED freezes the last real layers;
# PUBLISHED exists only after the node published the atomic envelope.
PLANNING_DEBUG_SCHEMA = "virtual_a_planning_debug/v1"
PLANNING_STATE = "PLANNING"
PLANNING_FAILED_STATE = "FAILED"
PLANNING_PUBLISHED_STATE = "PUBLISHED"
PLANNING_DEBUG_STATES = (
    PLANNING_STATE, PLANNING_FAILED_STATE, PLANNING_PUBLISHED_STATE)


def _sha256_text(value):
    return (isinstance(value, str) and len(value) == 64
            and all(character in "0123456789abcdef" for character in value))


def _planning_document_identity_ok(document, job):
    """Shared identity rule for progressive planning documents."""
    if document.get("schema") != PLANNING_DEBUG_SCHEMA:
        return False
    if job.get("backend") != VIRTUAL_PLAYBACK_BACKEND:
        return False
    for name in ("plan_id", "formation_id", "backend", "generation"):
        if document.get(name) != job.get(name):
            return False
    for name in ("request_sha256", "grid_sha256",
                 "planning_context_sha256"):
        if not _sha256_text(document.get(name)):
            return False
        if document.get(name) != job.get(name):
            return False
    model_sha256 = document.get("model_sha256")
    if document.get("state") == PLANNING_PUBLISHED_STATE:
        if not _sha256_text(model_sha256):
            return False
        if job.get("model_sha256") not in (None, model_sha256):
            return False
    elif model_sha256 is not None:
        # Before the envelope is published no authoritative model exists.
        return False
    return True


def corridor_display_matches(document, job):
    """Whether one planning-debug payload belongs to the current admission.

    The planner publishes corridor geometry after the admission job report.
    A payload from another plan, formation, generation, grid or planning
    context must never reach the display, even when the chains are valid.
    ``CLEARED`` payloads carry no geometry hashes; they may only delete, so
    plan/formation/backend/generation identity is sufficient for them.

    Progressive TASK 01 documents (``PLANNING``/``FAILED``/``PUBLISHED``)
    carry request/grid/context digests and a ``null`` model until the node has
    actually published the atomic envelope.  A ``CERTIFIED`` candidate or a
    non-null model is refused before that point; late or mismatched documents
    never clear or replace an existing layer.
    """
    if not isinstance(document, Mapping) or not isinstance(job, Mapping):
        return False
    state = document.get("state")
    if state == CLEARED:
        if job.get("backend") != VIRTUAL_PLAYBACK_BACKEND:
            return False
        for name in ("plan_id", "formation_id", "backend", "generation"):
            if document.get(name) != job.get(name):
                return False
        return True
    if state == CORRIDOR_DISPLAY_STATE:
        # Legacy V2 corridor document: unchanged full-identity rule.
        if job.get("backend") != VIRTUAL_PLAYBACK_BACKEND:
            return False
        for name in ("plan_id", "formation_id", "backend", "generation"):
            if document.get(name) != job.get(name):
                return False
        for name in ("grid_sha256", "planning_context_sha256",
                     "model_sha256"):
            if document.get(name) != job.get(name):
                return False
        corridors = document.get("corridors")
        if not isinstance(corridors, (list, tuple)) or not corridors:
            return False
        for corridor in corridors:
            if not isinstance(corridor, Mapping):
                return False
            if corridor.get("state") != CORRIDOR_DISPLAY_STATE:
                return False
            for name in ("plan_id", "formation_id", "backend", "generation",
                         "grid_sha256", "planning_context_sha256",
                         "model_sha256"):
                if corridor.get(name) != document.get(name):
                    return False
            regions = corridor.get("regions")
            if not isinstance(regions, (list, tuple)) or not regions:
                return False
        return True
    if state not in PLANNING_DEBUG_STATES:
        return False
    if not _planning_document_identity_ok(document, job):
        return False
    corridors = document.get("corridors")
    if corridors is None:
        return True
    if not isinstance(corridors, (list, tuple)):
        return False
    for corridor in corridors:
        if not isinstance(corridor, Mapping):
            return False
        if corridor.get("state") != CORRIDOR_DISPLAY_STATE:
            return False
        for name in ("plan_id", "formation_id", "backend", "generation",
                     "grid_sha256", "planning_context_sha256",
                     "model_sha256"):
            if corridor.get(name) != document.get(name):
                return False
        regions = corridor.get("regions")
        if not isinstance(regions, (list, tuple)) or not regions:
            return False
    return True


def _corridor_region(region):
    """Validate one display polygon and return (region_id, 2-D vertices)."""
    if not isinstance(region, Mapping):
        raise PlaybackValidationError("corridor region must be a mapping")
    region_id = region.get("region_id")
    if (not isinstance(region_id, int) or isinstance(region_id, bool) or
            region_id < 0):
        raise PlaybackValidationError(
            "corridor region_id must be a non-negative integer")
    vertices = region.get("vertices")
    if not isinstance(vertices, (list, tuple)) or len(vertices) < 3:
        raise PlaybackValidationError(
            "corridor region needs at least three vertices")
    points = []
    for vertex in vertices:
        if not isinstance(vertex, (list, tuple)) or len(vertex) != 2:
            raise PlaybackValidationError(
                "corridor vertex must be a coordinate pair")
        x, y = float(vertex[0]), float(vertex[1])
        if not (math.isfinite(x) and math.isfinite(y)):
            raise PlaybackValidationError("corridor vertex is not finite")
        points.append((x, y))
    twice_area = sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]))
    if not twice_area > 0.0:
        raise PlaybackValidationError(
            "corridor vertices must be counter-clockwise")
    scale = max(1.0, max(
        (bx - ax) ** 2 + (by - ay) ** 2
        for (ax, ay), (bx, by) in zip(points, points[1:] + points[:1])))
    for index in range(len(points)):
        ax, ay = points[index]
        bx, by = points[(index + 1) % len(points)]
        cx, cy = points[(index + 2) % len(points)]
        cross = (bx - ax) * (cy - by) - (by - ay) * (cx - bx)
        if cross < -1e-12 * scale:
            raise PlaybackValidationError(
                "corridor vertices must form a convex polygon")
    return region_id, points


def _corridor_route_ids(corridors):
    """Validate route ids and return them in payload order."""
    route_ids = []
    for corridor in corridors:
        if not isinstance(corridor, Mapping):
            raise PlaybackValidationError("corridor route must be a mapping")
        route_id = corridor.get("route_id")
        if (not isinstance(route_id, int) or isinstance(route_id, bool) or
                route_id < 0):
            raise PlaybackValidationError(
                "corridor route_id must be a non-negative integer")
        route_ids.append(route_id)
    if len(set(route_ids)) != len(route_ids):
        raise PlaybackValidationError(
            "corridor route ids must be unique within one document")
    return route_ids


def build_corridor_markers(document, frame_id="map"):
    """Draw the original chain: fan triangles plus closed polygon edges.

    The vertices are copied from the payload without refitting or enlarging
    them.  Every returned array starts with its own DELETEALL marker, so a
    replacement cannot leave stale route/region ids visible.  The payload is
    a candidate reference-point corridor only; this function never emits an
    execution or three-vehicle safety claim.  Both the legacy V2 display state
    and the progressive PLANNING/FAILED/PUBLISHED documents are accepted; the
    corridor colour is fixed red and the namespace never carries the
    formation id.
    """
    if not isinstance(document, Mapping):
        raise PlaybackValidationError("corridor display must be an object")
    state = document.get("state")
    if state not in (CORRIDOR_DISPLAY_STATE,) + PLANNING_DEBUG_STATES:
        raise PlaybackValidationError(
            "corridor display state must be the C2 display state or one of "
            "{}".format(PLANNING_DEBUG_STATES))
    if state == CORRIDOR_DISPLAY_STATE and document.get(
            "used_by_optimizer"):
        raise PlaybackValidationError(
            "V1 corridor display cannot carry used_by_optimizer")
    _text(document.get("formation_id"), "formation_id")
    corridors = document.get("corridors")
    if not isinstance(corridors, (list, tuple)) or not corridors:
        raise PlaybackValidationError(
            "corridor display needs at least one route")
    route_ids = _corridor_route_ids(corridors)
    markers = list(clear_marker_specs(frame_id))
    for route_id, corridor in zip(route_ids, corridors):
        regions = corridor.get("regions")
        if not isinstance(regions, (list, tuple)) or not regions:
            raise PlaybackValidationError(
                "corridor route needs at least one region")
        for region in regions:
            region_id, points = _corridor_region(region)
            # The stable namespace is shared by every route in a document;
            # ids stay unique when more than one route is displayed.
            marker_id = route_id * 1000 + region_id
            triangles = []
            for index in range(1, len(points) - 1):
                for x, y in (points[0], points[index], points[index + 1]):
                    triangles.append((x, y, CORRIDOR_FILL_Z_M))
            markers.append(_marker(
                CORRIDOR_FILL_NAMESPACE, marker_id, "TRIANGLE_LIST",
                frame_id,
                points=tuple(triangles),
                color=CORRIDOR_FILL_COLOR + (CORRIDOR_FILL_ALPHA,),
                # RViz scales triangle-list points by Marker.scale; the
                # default 0.1 would shrink every corridor to 10% at the
                # origin instead of drawing the audited vertices.
                scale=(1.0, 1.0, 1.0)))
            edge = tuple(
                (x, y, CORRIDOR_EDGE_Z_M) for x, y in points)
            edge += ((points[0][0], points[0][1], CORRIDOR_EDGE_Z_M),)
            markers.append(_marker(
                CORRIDOR_EDGE_NAMESPACE, marker_id, "LINE_STRIP", frame_id,
                points=edge,
                color=CORRIDOR_EDGE_COLOR + (CORRIDOR_EDGE_ALPHA,),
                scale=(0.013, 0.0, 0.0)))
    return tuple(markers)


PLANNING_ASTAR_NAMESPACE = "formation/planning/astar_raw"
PLANNING_GUIDE_NAMESPACE = "formation/planning/portal_guide"
GUIDE_Z_M = 0.012
# Legacy marker widths, retained for older verification scripts.  The guide
# and A* layers are now nav_msgs/Path topics, whose colour/width are owned by
# the RViz Path displays rather than these values.
GUIDE_ASTAR_WIDTH_M = 0.026
GUIDE_CENTERLINE_WIDTH_M = 0.032
GUIDE_ASTAR_COLOR = (1.0, 0.55, 0.0, 0.95)
GUIDE_CENTERLINE_COLOR = (0.05, 0.82, 0.25, 0.95)
GUIDE_UNVALIDATED_COLOR = (1.0, 0.78, 0.0, 0.95)

PORTAL_DIAGNOSTICS_SCHEMA = "virtual_a_portal_diagnostics_v1"
PORTAL_WINDOW_NAMESPACE = "formation/planning/portal_window"
MAP_COLLISION_SAMPLE_NAMESPACE = "formation/planning/map_collision_sample"
COVERAGE_GAP_NAMESPACE = "formation/planning/coverage_gap"
PORTAL_WINDOW_COLOR = (0.20, 0.80, 0.95, 0.55)
MAP_COLLISION_SAMPLE_COLOR = (236 / 255, 72 / 255, 153 / 255, 1.0)
COVERAGE_GAP_COLOR = (1.0, 0.55, 0.05, 1.0)
PORTAL_DIAGNOSTIC_Z_M = 0.03
PORTAL_DIAGNOSTIC_TEXT_Z_M = 0.20
PORTAL_DIAGNOSTIC_COLLISION_LIMIT = 5
PORTAL_DIAGNOSTIC_GAP_LIMIT = 6
PORTAL_DIAGNOSTIC_WINDOW_LIMIT = 3


def _guide_polyline(points, name):
    """Copy an actual route/guide polyline without resampling it."""
    if not isinstance(points, (list, tuple)) or len(points) < 2:
        raise PlaybackValidationError(
            "{} needs at least two points".format(name))
    copied = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise PlaybackValidationError(
                "{} point must be a coordinate pair".format(name))
        x, y = float(point[0]), float(point[1])
        if not (math.isfinite(x) and math.isfinite(y)):
            raise PlaybackValidationError(
                "{} point is not finite".format(name))
        copied.append((x, y, GUIDE_Z_M))
    return tuple(copied)




def guide_quality_valid(quality, chain_sha256, region_count):
    """Whether the C2-A quality report fully describes this guide.

    This is a display classification only.  The C2-A guide is an executable
    planning primitive, but this layer remains a reference polyline: it is
    never labelled as a smooth three-vehicle path.
    """
    if not isinstance(quality, Mapping):
        return False
    if quality.get("stage") != "C2_A":
        return False
    if quality.get("chain_digest") != chain_sha256:
        return False
    if quality.get("is_executable_path") is not False:
        return False
    if quality.get("rounding") != "PL_ONLY_NO_BEZIER":
        return False
    if quality.get("smoothness_claim") != "NONE":
        return False
    if (not isinstance(region_count, int) or isinstance(region_count, bool)
            or region_count < 1):
        return False
    if quality.get("region_count") != region_count:
        return False
    if quality.get("segment_corridor_indices") != list(
            range(region_count)):
        return False
    warnings = quality.get("warnings")
    if not isinstance(warnings, list) or warnings:
        return False
    return True


def _guide_free_text(markers, namespace, marker_id, point, text, color,
                     frame_id):
    markers.append(_marker(
        namespace, marker_id, "TEXT_VIEW_FACING", frame_id,
        position=(point[0], point[1], 0.45),
        text=text, color=color, scale=(0.0, 0.0, 0.22)))


def build_guide_display(document, frame_id="map"):
    """Validate the guide payload and return ``(markers, paths)``.

    ``paths`` maps ``("astar_raw", route_id)`` / ``("portal_guide", route_id)``
    to the verbatim ``(x, y)`` polyline samples.  The caller publishes those
    as ``nav_msgs/Path`` so RViz can change their colour and width; this
    function only keeps the text labels in the marker array.  Neither
    polyline is resampled, and the layer never upgrades the guide to a safety
    certificate.  Both the legacy V2 state and the progressive TASK 01
    planning states are accepted; namespaces are the stable
    ``formation/planning/*`` ones and do not carry the formation id or a
    used/candidate variant.
    """
    if not isinstance(document, Mapping):
        raise PlaybackValidationError("guide display must be an object")
    state = document.get("state")
    if state not in (CORRIDOR_DISPLAY_STATE,) + PLANNING_DEBUG_STATES:
        raise PlaybackValidationError(
            "guide display state must be the C2 display state or one of "
            "{}".format(PLANNING_DEBUG_STATES))
    if state == CORRIDOR_DISPLAY_STATE and document.get(
            "used_by_optimizer"):
        raise PlaybackValidationError(
            "V2 guide display cannot carry used_by_optimizer")
    _text(document.get("formation_id"), "formation_id")
    if state in PLANNING_DEBUG_STATES:
        # The admission request digest is checked here as well; corridor
        # entries cannot carry it because corridor_display_fields() emits the
        # legacy identity, whose planning_context already binds the request.
        if not _sha256_text(document.get("request_sha256")):
            raise PlaybackValidationError(
                "planning guide display needs a request SHA-256")
    corridors = document.get("corridors")
    astar = document.get("astar")
    if corridors is not None and not isinstance(corridors, (list, tuple)):
        raise PlaybackValidationError(
            "guide corridors must be a list or null")
    corridors = () if corridors is None else corridors
    if astar is not None and not isinstance(astar, (list, tuple)):
        raise PlaybackValidationError("guide astar must be a list or null")
    v2_keys = ("raw_route_points", "guide_points", "guide_quality",
               "chain_sha256", "corridor_used")
    has_v2 = any(isinstance(corridor, Mapping)
                 and any(key in corridor for key in v2_keys)
                 for corridor in corridors)
    if not corridors and astar is None:
        return tuple(clear_marker_specs(frame_id)), {}
    if astar is not None and not (corridors and has_v2):
        # A*-complete stage: the corridor chain does not exist yet.  Only the
        # real raw routes are published, one Path per route, and the text
        # labels stay in the marker layer.
        markers = list(clear_marker_specs(frame_id))
        paths = {}
        for entry in astar:
            if not isinstance(entry, Mapping):
                raise PlaybackValidationError(
                    "guide astar route must be a mapping")
            route_id = entry.get("route_id")
            if (not isinstance(route_id, int) or isinstance(route_id, bool)
                    or route_id < 0):
                raise PlaybackValidationError(
                    "guide astar route_id must be a non-negative integer")
            points = _guide_polyline(entry.get("points"), "astar points")
            text_id = 2 * route_id + 1
            paths[("astar_raw", route_id)] = tuple(
                (x, y) for x, y, _ in points)
            _guide_free_text(
                markers, PLANNING_ASTAR_NAMESPACE, text_id, points[0],
                "A* ROUTE | REFERENCE_ONLY | CANDIDATE",
                GUIDE_ASTAR_COLOR, frame_id)
        return tuple(markers), paths
    if not has_v2:
        # V1 payloads carry only corridor regions.  Clearing is the correct
        # guide layer for them; stale guides from an earlier plan must not
        # survive.
        return tuple(clear_marker_specs(frame_id)), {}
    identity_names = ["plan_id", "formation_id", "backend", "generation",
                      "grid_sha256", "planning_context_sha256",
                      "model_sha256"]
    markers = list(clear_marker_specs(frame_id))
    paths = {}
    for corridor in corridors:
        if not isinstance(corridor, Mapping):
            raise PlaybackValidationError("guide route must be a mapping")
        for name in identity_names:
            if corridor.get(name) != document.get(name):
                raise PlaybackValidationError(
                    "guide route identity does not match its document")
        route_id = corridor.get("route_id")
        if (not isinstance(route_id, int) or isinstance(route_id, bool)
                or route_id < 0):
            raise PlaybackValidationError(
                "guide route_id must be a non-negative integer")
        chain_sha256 = corridor.get("chain_sha256")
        if (not isinstance(chain_sha256, str)
                or len(chain_sha256) != 64
                or any(character not in "0123456789abcdef"
                       for character in chain_sha256)):
            raise PlaybackValidationError(
                "guide chain_sha256 must be a lowercase SHA-256")
        corridor_used = corridor.get("corridor_used")
        if not isinstance(corridor_used, bool):
            raise PlaybackValidationError(
                "guide corridor_used must be a bool")
        raw_route = _guide_polyline(
            corridor.get("raw_route_points"), "raw_route_points")
        guide = _guide_polyline(
            corridor.get("guide_points"), "guide_points")
        regions = corridor.get("regions")
        if not isinstance(regions, (list, tuple)) or not regions:
            raise PlaybackValidationError(
                "guide route needs at least one corridor region")
        route_suffix = "" if corridor_used else " | CANDIDATE"
        text_id = 2 * route_id + 1
        paths[("astar_raw", route_id)] = tuple(
            (x, y) for x, y, _ in raw_route)
        _guide_free_text(
            markers, PLANNING_ASTAR_NAMESPACE, text_id, raw_route[0],
            "A* ROUTE{} | REFERENCE_ONLY".format(route_suffix),
            GUIDE_ASTAR_COLOR, frame_id)
        validated = guide_quality_valid(
            corridor.get("guide_quality"), chain_sha256, len(regions))
        color = (GUIDE_CENTERLINE_COLOR if validated
                 else GUIDE_UNVALIDATED_COLOR)
        label = ("PL_GUIDE | REFERENCE_ONLY" if validated else
                 "UNVALIDATED_GUIDE | PL_GUIDE | REFERENCE_ONLY")
        paths[("portal_guide", route_id)] = tuple(
            (x, y) for x, y, _ in guide)
        _guide_free_text(
            markers, PLANNING_GUIDE_NAMESPACE, text_id,
            guide[len(guide) // 2], label + route_suffix, color, frame_id)
    _append_portal_diagnostics(markers, corridors, document, frame_id)
    return tuple(markers), paths


def _portal_point(value, context):
    """Validate one finite display coordinate from the diagnostics payload."""
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise PlaybackValidationError(
            "{} must be a coordinate pair".format(context))
    return (_finite(value[0], "{} x".format(context)),
            _finite(value[1], "{} y".format(context)))


def _portal_count(value, context):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise PlaybackValidationError(
            "{} must be a non-negative integer".format(context))
    return int(value)


def _portal_diagnostic_payload(diagnostics, corridor):
    """Validate one display-only diagnostic block and bind its identity."""
    if not isinstance(diagnostics, Mapping):
        raise PlaybackValidationError(
            "portal diagnostics must be an object")
    if diagnostics.get("schema") != PORTAL_DIAGNOSTICS_SCHEMA:
        raise PlaybackValidationError(
            "portal diagnostics schema differs")
    if diagnostics.get("certification_claim") != "NONE":
        raise PlaybackValidationError(
            "portal diagnostics cannot claim certification")
    if diagnostics.get("continuously_certified") is not False:
        raise PlaybackValidationError(
            "portal diagnostics cannot certify a continuous transfer")
    if diagnostics.get("formation_obb_hard_constraints") is not False:
        raise PlaybackValidationError(
            "portal diagnostics cannot claim OBB hard constraints")
    chain_digest = diagnostics.get("chain_digest")
    if chain_digest is not None and chain_digest != corridor.get(
            "chain_sha256"):
        raise PlaybackValidationError(
            "portal diagnostics chain differs from the corridor")
    guide_source = diagnostics.get("guide_source")
    if not isinstance(guide_source, str):
        raise PlaybackValidationError(
            "portal diagnostics guide_source must be a string")
    if corridor.get("guide_source") and guide_source and \
            guide_source != corridor.get("guide_source"):
        raise PlaybackValidationError(
            "portal diagnostics guide differs from the corridor")
    counts = diagnostics.get("counts")
    if not isinstance(counts, Mapping):
        raise PlaybackValidationError(
            "portal diagnostics counts must be an object")
    for name in ("map_collision_total", "single_region_total",
                 "union_covered_total", "union_gap_total",
                 "union_unknown_total", "portal_window_total"):
        _portal_count(counts.get(name), "portal diagnostics {}".format(name))
    samples = diagnostics.get("map_collision_samples")
    gaps = diagnostics.get("coverage_gaps")
    windows = diagnostics.get("portal_windows")
    for entries, limit, name in (
            (samples, PORTAL_DIAGNOSTIC_COLLISION_LIMIT,
             "map_collision_samples"),
            (gaps, PORTAL_DIAGNOSTIC_GAP_LIMIT, "coverage_gaps"),
            (windows, PORTAL_DIAGNOSTIC_WINDOW_LIMIT, "portal_windows")):
        if not isinstance(entries, (list, tuple)):
            raise PlaybackValidationError(
                "portal diagnostics {} must be a list".format(name))
        if len(entries) > limit:
            raise PlaybackValidationError(
                "portal diagnostics {} exceed the marker cap".format(name))
    return diagnostics


def _append_portal_diagnostics(markers, corridors, document, frame_id):
    """Append bounded real map-collision/gap/Portal diagnostics.

    These are sample annotations from the finite assessment, never a
    collision certificate and never part of the executable path.  Bad data
    fails the whole guide layer closed; marker text deliberately separates
    ``MAP COLLISION (sample)`` from ``CORRIDOR COVERAGE GAP``.
    """
    for corridor in corridors:
        if not isinstance(corridor, Mapping):
            raise PlaybackValidationError("guide route must be a mapping")
        diagnostics = corridor.get("portal_diagnostics")
        if diagnostics is None:
            continue
        _portal_diagnostic_payload(diagnostics, corridor)
        route_id = corridor.get("route_id")
        if (not isinstance(route_id, int) or isinstance(route_id, bool)
                or route_id < 0):
            raise PlaybackValidationError(
                "guide route_id must be a non-negative integer")
        base_id = 1000 + 100 * int(route_id)
        samples = diagnostics.get("map_collision_samples")
        gaps = diagnostics.get("coverage_gaps")
        windows = diagnostics.get("portal_windows")
        for index, sample in enumerate(samples):
            if not isinstance(sample, Mapping):
                raise PlaybackValidationError(
                    "map collision sample must be an object")
            x, y = _portal_point(
                sample.get("point"), "map collision sample point")
            sample_s = _finite(
                sample.get("s_m"), "map collision sample s_m")
            vehicle_id = sample.get("vehicle_id")
            if vehicle_id is not None and (
                    isinstance(vehicle_id, bool)
                    or not isinstance(vehicle_id, int) or vehicle_id < 1):
                raise PlaybackValidationError(
                    "map collision sample vehicle_id is invalid")
            marker_id = base_id + index
            size = 0.07
            markers.append(_marker(
                MAP_COLLISION_SAMPLE_NAMESPACE, marker_id, "LINE_LIST",
                frame_id,
                points=(
                    (x - size, y, PORTAL_DIAGNOSTIC_Z_M),
                    (x + size, y, PORTAL_DIAGNOSTIC_Z_M),
                    (x, y - size, PORTAL_DIAGNOSTIC_Z_M),
                    (x, y + size, PORTAL_DIAGNOSTIC_Z_M)),
                color=MAP_COLLISION_SAMPLE_COLOR,
                scale=(0.018, 0.0, 0.0)))
            car = "?" if vehicle_id is None else str(vehicle_id)
            markers.append(_marker(
                MAP_COLLISION_SAMPLE_NAMESPACE, base_id + 500 + index,
                "TEXT_VIEW_FACING", frame_id,
                position=(x, y, PORTAL_DIAGNOSTIC_TEXT_Z_M),
                text="MAP COLLISION (sample) | car{} | s={:.2f} | {}".format(
                    car, sample_s, str(sample.get("reason") or "")[:40]),
                color=MAP_COLLISION_SAMPLE_COLOR,
                scale=(0.0, 0.0, 0.22)))
        for index, gap in enumerate(gaps):
            if not isinstance(gap, Mapping):
                raise PlaybackValidationError(
                    "coverage gap sample must be an object")
            x, y = _portal_point(
                gap.get("point"), "coverage gap sample point")
            gap_s = _finite(gap.get("s_m"), "coverage gap sample s_m")
            vehicle_id = gap.get("vehicle_id")
            if isinstance(vehicle_id, bool) or not isinstance(
                    vehicle_id, int) or vehicle_id < 1:
                raise PlaybackValidationError(
                    "coverage gap sample vehicle_id is invalid")
            marker_id = base_id + index
            size = 0.07
            markers.append(_marker(
                COVERAGE_GAP_NAMESPACE, marker_id, "LINE_STRIP", frame_id,
                points=(
                    (x - size, y, PORTAL_DIAGNOSTIC_Z_M),
                    (x, y + size, PORTAL_DIAGNOSTIC_Z_M),
                    (x + size, y, PORTAL_DIAGNOSTIC_Z_M),
                    (x, y - size, PORTAL_DIAGNOSTIC_Z_M),
                    (x - size, y, PORTAL_DIAGNOSTIC_Z_M)),
                color=COVERAGE_GAP_COLOR,
                scale=(0.016, 0.0, 0.0)))
            markers.append(_marker(
                COVERAGE_GAP_NAMESPACE, base_id + 500 + index,
                "TEXT_VIEW_FACING", frame_id,
                position=(x, y, PORTAL_DIAGNOSTIC_TEXT_Z_M),
                text=(
                    "CORRIDOR COVERAGE GAP (sample) | car{} | s={:.2f}"
                    " | {}".format(
                        vehicle_id, gap_s,
                        str(gap.get("reason") or "")[:40])),
                color=COVERAGE_GAP_COLOR,
                scale=(0.0, 0.0, 0.22)))
        for index, window in enumerate(windows):
            if not isinstance(window, Mapping):
                raise PlaybackValidationError(
                    "portal window sample must be an object")
            start = _portal_point(
                window.get("start"), "portal window start")
            end = _portal_point(window.get("end"), "portal window end")
            portal_index = window.get("portal_index")
            if portal_index is not None and (
                    isinstance(portal_index, bool)
                    or not isinstance(portal_index, int)
                    or portal_index < 0):
                raise PlaybackValidationError(
                    "portal window index is invalid")
            target = None if portal_index is None else portal_index + 1
            label = "PORTAL {}->{} | {}".format(
                "?" if portal_index is None else portal_index,
                "?" if target is None else target,
                str(window.get("status") or "UNKNOWN")[:48])
            marker_id = base_id + index
            markers.append(_marker(
                PORTAL_WINDOW_NAMESPACE, marker_id, "LINE_STRIP", frame_id,
                points=(
                    (start[0], start[1], PORTAL_DIAGNOSTIC_Z_M),
                    (end[0], end[1], PORTAL_DIAGNOSTIC_Z_M)),
                color=PORTAL_WINDOW_COLOR,
                scale=(0.021, 0.0, 0.0)))
            seed = window.get("portal_seed_point")
            if seed is not None:
                sx, sy = _portal_point(seed, "portal seed point")
                markers.append(_marker(
                    PORTAL_WINDOW_NAMESPACE, base_id + 200 + index,
                    "SPHERE", frame_id,
                    position=(sx, sy, PORTAL_DIAGNOSTIC_Z_M),
                    color=PORTAL_WINDOW_COLOR,
                    scale=(0.045, 0.045, 0.045)))
            markers.append(_marker(
                PORTAL_WINDOW_NAMESPACE, base_id + 500 + index,
                "TEXT_VIEW_FACING", frame_id,
                position=(
                    (start[0] + end[0]) / 2.0,
                    (start[1] + end[1]) / 2.0,
                    PORTAL_DIAGNOSTIC_TEXT_Z_M),
                text=label,
                color=PORTAL_WINDOW_COLOR,
                scale=(0.0, 0.0, 0.20)))


def build_guide_markers(document, frame_id="map"):
    """Display-only wrapper returning just the guide text markers."""
    return build_guide_display(document, frame_id)[0]


# ---------------------------------------------------------------------------
# Candidate display (TASK 01 Step 1): unverified solver candidates only
# ---------------------------------------------------------------------------
CANDIDATE_SCHEMA = "virtual_a_candidate/v1"
CANDIDATE_UNVERIFIED = "UNVERIFIED_CANDIDATE"
CANDIDATE_CERTIFIED = "CERTIFIED"
CANDIDATE_DIAGNOSTIC_LABEL = "DIAGNOSTIC_ONLY"
CANDIDATE_POINT_LIMIT = 400
CANDIDATE_OBB_LIMIT = 5
CANDIDATE_SAMPLE_SPACING_M = 0.05
CANDIDATE_F_NAMESPACE = "formation/planning/f_candidate"
CANDIDATE_CENTER_NAMESPACE = "formation/planning/car{}_center_candidate"
CANDIDATE_OBB_NAMESPACE = "formation/planning/car{}_obb_candidate"
PLANNING_FAILURE_NAMESPACE = "formation/planning/failure"
CANDIDATE_F_COLOR = (1.00, 0.85, 0.05, 0.95)
CANDIDATE_CENTER_COLOR = (0.10, 0.45, 1.00, 0.95)
CANDIDATE_OBB_COLOR = (148 / 255, 163 / 255, 184 / 255, 0.38)
PLANNING_FAILURE_COLOR = (236 / 255, 72 / 255, 153 / 255, 1.00)
CANDIDATE_OBB_Z_M = 0.028
PLANNING_FAILURE_Z_M = 0.16
CANDIDATE_CLASSIFICATIONS = (CANDIDATE_UNVERIFIED, CANDIDATE_CERTIFIED)


def _candidate_shape(vehicle_shapes, vehicle_id):
    if not isinstance(vehicle_shapes, Mapping):
        raise PlaybackValidationError(
            "candidate vehicle_shapes must be a mapping")
    shape = vehicle_shapes.get(vehicle_id)
    if shape is None:
        raise PlaybackValidationError(
            "candidate vehicle {} has no shape".format(vehicle_id))
    length = _positive(
        getattr(shape, "length_m", None),
        "candidate vehicle {} length_m".format(vehicle_id))
    width = _positive(
        getattr(shape, "width_m", None),
        "candidate vehicle {} width_m".format(vehicle_id))
    return length, width


def _obb_outline(center_x, center_y, theta, length_m, width_m):
    """Closed five-point body outline at the actual vehicle heading."""
    half_length = 0.5 * length_m
    half_width = 0.5 * width_m
    cosine, sine = math.cos(theta), math.sin(theta)
    points = []
    for local_x, local_y in (
            (half_length, half_width),
            (-half_length, half_width),
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width)):
        points.append([
            center_x + cosine * local_x - sine * local_y,
            center_y + sine * local_x + cosine * local_y])
    return points


def sample_candidate_path(path, vehicle_shapes,
                          classification=CANDIDATE_UNVERIFIED,
                          source="", status="", failure=None,
                          point_limit=CANDIDATE_POINT_LIMIT,
                          obb_limit=CANDIDATE_OBB_LIMIT):
    """Read-only equal-arc display sampling of one real ``VirtualAPath``.

    Every F point and every vehicle centre comes from the same real
    ``path.state_at_s(s)`` call; every OBB uses that state's actual body
    heading and the request's shape dimensions (never ``psi_F``).  Stations
    include ``s=0`` and ``s=S`` and are capped by the display limits.  No
    interpolation, model or new trajectory is constructed here.
    """
    if classification not in CANDIDATE_CLASSIFICATIONS:
        raise PlaybackValidationError(
            "candidate classification must be one of {}".format(
                CANDIDATE_CLASSIFICATIONS))
    if path is None or not callable(getattr(path, "state_at_s", None)):
        raise PlaybackValidationError(
            "candidate display needs a real VirtualAPath")
    length = _finite(getattr(path, "length_m", None),
                     "candidate path length_m")
    if length < 0.0:
        raise PlaybackValidationError(
            "candidate path length_m must be non-negative")
    point_limit = int(point_limit)
    obb_limit = int(obb_limit)
    if point_limit < 2 or obb_limit < 1:
        raise PlaybackValidationError(
            "candidate display limits are too small")
    if length <= 0.0:
        count = 2
    else:
        count = min(point_limit, max(
            2, int(math.ceil(length / CANDIDATE_SAMPLE_SPACING_M)) + 1))
    if count == 2:
        stations = (0.0, length)
    else:
        stations = tuple(
            length * index / float(count - 1) for index in range(count))
    obb_count = min(obb_limit, count)
    if obb_count == 1:
        obb_indices = (0,)
    else:
        obb_indices = tuple(sorted({
            int(round(index * (count - 1) / float(obb_count - 1)))
            for index in range(obb_count)}))
    f_points = []
    vehicle_points = {}
    vehicle_obbs = {}
    vehicle_ids = None
    for index, station in enumerate(stations):
        state = path.state_at_s(station)
        qf = getattr(state, "qf", None)
        if qf is None:
            raise PlaybackValidationError(
                "candidate state_at_s returned no formation pose")
        f_points.append([_finite(getattr(qf, "x", None),
                                 "candidate F x"),
                         _finite(getattr(qf, "y", None),
                                 "candidate F y")])
        vehicles = tuple(getattr(state, "vehicles", ()) or ())
        ids = tuple(getattr(vehicle, "vehicle_id", None)
                    for vehicle in vehicles)
        if vehicle_ids is None:
            vehicle_ids = ids
            if (ids != tuple(sorted(ids)) or len(set(ids)) != len(ids)):
                raise PlaybackValidationError(
                    "candidate vehicle ids must be unique and sorted")
            for vehicle_id in ids:
                vehicle_points[vehicle_id] = []
                vehicle_obbs[vehicle_id] = []
        elif ids != vehicle_ids:
            raise PlaybackValidationError(
                "candidate state_at_s changed the vehicle set")
        for vehicle in vehicles:
            x = _finite(getattr(vehicle, "x", None),
                        "candidate vehicle x")
            y = _finite(getattr(vehicle, "y", None),
                        "candidate vehicle y")
            vehicle_points[vehicle.vehicle_id].append([x, y])
            if index in obb_indices:
                theta = _finite(getattr(vehicle, "theta", None),
                                "candidate vehicle heading")
                length_m, width_m = _candidate_shape(
                    vehicle_shapes, vehicle.vehicle_id)
                vehicle_obbs[vehicle.vehicle_id].append(
                    _obb_outline(x, y, theta, length_m, width_m))
    vehicles_payload = []
    for vehicle_id in vehicle_ids or ():
        length_m, width_m = _candidate_shape(vehicle_shapes, vehicle_id)
        vehicles_payload.append({
            "vehicle_id": int(vehicle_id),
            "body_length_m": float(length_m),
            "body_width_m": float(width_m),
            "center_points": vehicle_points[vehicle_id],
            "obb_corners": vehicle_obbs[vehicle_id],
        })
    return {
        "schema": CANDIDATE_SCHEMA,
        "classification": classification,
        "certified": classification == CANDIDATE_CERTIFIED,
        "diagnostic_label": (
            "" if classification == CANDIDATE_CERTIFIED
            else CANDIDATE_DIAGNOSTIC_LABEL),
        "source": str(source or ""),
        "status": str(status or ""),
        "s_total_m": float(length),
        "point_count": len(f_points),
        "obb_count": (len(vehicle_obbs[vehicle_ids[0]])
                      if vehicle_ids else 0),
        "f_points": f_points,
        "vehicles": vehicles_payload,
        "failure": _candidate_failure(failure),
    }


def _candidate_failure(failure):
    """Only a real finite world coordinate may become a failure marker."""
    if not isinstance(failure, Mapping):
        return None
    try:
        x = float(failure.get("x"))
        y = float(failure.get("y"))
    except (TypeError, ValueError, OverflowError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return {
        "x": x, "y": y,
        "reason": str(failure.get("reason", "") or ""),
        "source": str(failure.get("source", "") or ""),
    }


def _failure_point_from_location(location, path):
    """Extract a real map-frame point from an audit failure location.

    A ``qf`` array is already a world pose.  ``station``/``s_m`` values are
    usable only when the same candidate path can be evaluated there; no
    coordinate is guessed.
    """
    if not isinstance(location, Mapping):
        return None
    qf = location.get("qf")
    if isinstance(qf, (list, tuple)) and len(qf) >= 2:
        try:
            x, y = float(qf[0]), float(qf[1])
        except (TypeError, ValueError, OverflowError):
            x = y = None
        if x is not None and math.isfinite(x) and math.isfinite(y):
            return {"x": x, "y": y}
    try:
        x = float(location.get("x"))
        y = float(location.get("y"))
    except (TypeError, ValueError, OverflowError):
        x = y = None
    if x is not None and math.isfinite(x) and math.isfinite(y):
        return {"x": x, "y": y}
    if path is None or not callable(getattr(path, "state_at_s", None)):
        return None
    for key in ("station", "s_m", "failure_s_m"):
        value = location.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            continue
        station = float(value)
        if not math.isfinite(station):
            continue
        length = _finite(getattr(path, "length_m", None),
                         "candidate path length_m")
        if station < -1e-9 or station > length + 1e-9:
            continue
        state = path.state_at_s(min(max(station, 0.0), length))
        qf = getattr(state, "qf", None)
        if qf is None:
            continue
        return {"x": _finite(getattr(qf, "x", None), "failure x"),
                "y": _finite(getattr(qf, "y", None), "failure y")}
    return None


def failure_point_from_result(result):
    """The first real failure location of a non-executable result, or None.

    Only audit reports and the actual corridor containment record are read.
    A constraint name without a coordinate is not a failure point.
    """
    if result is None:
        return None
    path = getattr(result, "path", None)
    for source, report in (
            ("COLLISION", getattr(result, "collision_audit", None)),
            ("DYNAMICS", getattr(result, "dynamic_audit", None)),
            ("GEOMETRY", getattr(result, "geometry_audit", None))):
        if report is None or bool(getattr(report, "passed", False)):
            continue
        point = _failure_point_from_location(
            getattr(report, "location", None), path)
        if point is not None:
            point["reason"] = str(getattr(report, "reason", "") or "")
            point["source"] = source
            return point
    search = getattr(result, "search", None)
    corridor = (search.get("corridor")
                if isinstance(search, Mapping) else None)
    rounds = corridor.get("rounds") if isinstance(corridor, Mapping) else None
    if isinstance(rounds, (list, tuple)) and rounds:
        containment = rounds[-1].get("containment") \
            if isinstance(rounds[-1], Mapping) else None
        if isinstance(containment, Mapping) and not containment.get("passed"):
            point = _failure_point_from_location(
                containment.get("first_failure"), path)
            if point is not None:
                point["reason"] = str(containment.get("reason", "") or "")
                point["source"] = "CORRIDOR_CONTAINMENT"
                return point
    return None


def _candidate_point(point, context):
    if not isinstance(point, (list, tuple)) or len(point) != 2:
        raise PlaybackValidationError(
            "{} must be a coordinate pair".format(context))
    x = _finite(point[0], context + " x")
    y = _finite(point[1], context + " y")
    return (x, y)


def _failure_marker_specs(failure, frame_id):
    """A purple-red failure marker only for a real finite map coordinate."""
    if failure is None:
        return ()
    text = "FAILURE | {} | {}".format(
        failure["source"] or "AUDIT", failure["reason"] or "rejected")
    specs = [
        _marker(
            PLANNING_FAILURE_NAMESPACE, 1, "SPHERE", frame_id,
            position=(failure["x"], failure["y"], PLANNING_FAILURE_Z_M),
            color=PLANNING_FAILURE_COLOR, scale=(0.22, 0.22, 0.22)),
        _marker(
            PLANNING_FAILURE_NAMESPACE, 2, "TEXT_VIEW_FACING", frame_id,
            position=(failure["x"], failure["y"],
                      PLANNING_FAILURE_Z_M + 0.25),
            text=text, color=PLANNING_FAILURE_COLOR,
            scale=(0.0, 0.0, 0.22)),
    ]
    return tuple(specs)


def build_candidate_display(document, frame_id="map"):
    """Validate the candidate payload and return ``(markers, paths)``.

    ``paths`` maps ``"f_candidate"`` and ``("candidate_vehicle", vehicle_id)``
    to the verbatim ``(x, y)`` sample polylines so the node can publish them
    as ``nav_msgs/Path``.  OBB outlines and every text label stay in the
    marker array.  Namespaces are exactly the stable TASK 01 ones.  The
    builder never upgrades a classification: a ``CERTIFIED`` payload is
    accepted only in a ``PUBLISHED`` document, and any other payload keeps
    the ``DIAGNOSTIC_ONLY`` label.  Failure markers exist only with a real
    finite map coordinate.  A payload that cannot be validated raises, so the
    caller can clear only this layer; it never becomes execution input.
    """
    if not isinstance(document, Mapping):
        raise PlaybackValidationError("candidate display must be an object")
    state = document.get("state")
    if state not in PLANNING_DEBUG_STATES:
        raise PlaybackValidationError(
            "candidate display state must be one of {}".format(
                PLANNING_DEBUG_STATES))
    candidates = document.get("candidates")
    if candidates is None:
        # A failure without any constructed candidate; only a real audit
        # coordinate may be drawn, and the candidate layer stays empty.
        failure = _candidate_failure(document.get("failure"))
        if failure is None:
            raise PlaybackValidationError(
                "candidate display needs candidates or a real failure point")
        return (tuple(list(clear_marker_specs(frame_id)) +
                      list(_failure_marker_specs(failure, frame_id))), {})
    if not isinstance(candidates, Mapping):
        raise PlaybackValidationError("candidates must be an object")
    if candidates.get("schema") != CANDIDATE_SCHEMA:
        raise PlaybackValidationError(
            "candidate schema must be {!r}".format(CANDIDATE_SCHEMA))
    classification = candidates.get("classification")
    if classification not in CANDIDATE_CLASSIFICATIONS:
        raise PlaybackValidationError(
            "candidate classification must be one of {}".format(
                CANDIDATE_CLASSIFICATIONS))
    certified = candidates.get("certified")
    if not isinstance(certified, bool):
        raise PlaybackValidationError(
            "candidate certified flag must be a bool")
    if certified != (classification == CANDIDATE_CERTIFIED):
        raise PlaybackValidationError(
            "candidate certified flag disagrees with its classification")
    if classification == CANDIDATE_CERTIFIED and \
            state != PLANNING_PUBLISHED_STATE:
        raise PlaybackValidationError(
            "CERTIFIED requires the atomic envelope to have been published")
    diagnostic_label = ("" if classification == CANDIDATE_CERTIFIED
                        else CANDIDATE_DIAGNOSTIC_LABEL)
    s_total_m = _finite(candidates.get("s_total_m"),
                        "candidate s_total_m")
    if s_total_m < 0.0:
        raise PlaybackValidationError(
            "candidate s_total_m must be non-negative")
    f_points = candidates.get("f_points")
    if not isinstance(f_points, (list, tuple)) or len(f_points) < 2:
        raise PlaybackValidationError(
            "candidate f_points needs at least two points")
    if len(f_points) > CANDIDATE_POINT_LIMIT:
        raise PlaybackValidationError(
            "candidate f_points exceeds the display limit")
    f_points = [_candidate_point(point, "candidate f_point")
                for point in f_points]
    vehicles = candidates.get("vehicles")
    if not isinstance(vehicles, (list, tuple)) or not vehicles:
        raise PlaybackValidationError(
            "candidate vehicles must be a non-empty list")
    vehicle_ids = []
    checked_vehicles = []
    for vehicle in vehicles:
        if not isinstance(vehicle, Mapping):
            raise PlaybackValidationError(
                "candidate vehicle must be a mapping")
        vehicle_id = vehicle.get("vehicle_id")
        if (not isinstance(vehicle_id, int) or isinstance(vehicle_id, bool)
                or vehicle_id < 1):
            raise PlaybackValidationError(
                "candidate vehicle_id must be a positive integer")
        if vehicle_id in vehicle_ids:
            raise PlaybackValidationError(
                "candidate vehicle ids must be unique")
        vehicle_ids.append(vehicle_id)
        body_length = _positive(
            vehicle.get("body_length_m"), "candidate body_length_m")
        body_width = _positive(
            vehicle.get("body_width_m"), "candidate body_width_m")
        center_points = vehicle.get("center_points")
        if (not isinstance(center_points, (list, tuple))
                or len(center_points) != len(f_points)):
            raise PlaybackValidationError(
                "candidate centre points must match the F sample count")
        center_points = [
            _candidate_point(point, "candidate centre point")
            for point in center_points]
        obbs = vehicle.get("obb_corners")
        if not isinstance(obbs, (list, tuple)):
            raise PlaybackValidationError(
                "candidate obb_corners must be a list")
        if len(obbs) > CANDIDATE_OBB_LIMIT:
            raise PlaybackValidationError(
                "candidate OBB count exceeds the display limit")
        checked_obbs = []
        for index, outline in enumerate(obbs):
            if not isinstance(outline, (list, tuple)) or len(outline) != 5:
                raise PlaybackValidationError(
                    "candidate OBB outline must have five closed corners")
            corners = [_candidate_point(
                corner, "candidate OBB corner") for corner in outline]
            if corners[0] != corners[-1]:
                raise PlaybackValidationError(
                    "candidate OBB outline must be closed")
            checked_obbs.append(corners)
        checked_vehicles.append((
            vehicle_id, body_length, body_width, center_points,
            checked_obbs))
    if vehicle_ids != sorted(vehicle_ids):
        raise PlaybackValidationError(
            "candidate vehicles must be sorted by vehicle_id")
    failure = candidates.get("failure")
    failure = _candidate_failure(failure)
    source = str(candidates.get("source", "") or "")
    status = str(candidates.get("status", "") or "")
    label_parts = [
        "F candidate", classification,
        "S={:.2f} m".format(s_total_m)]
    if diagnostic_label:
        label_parts.insert(1, diagnostic_label)
    if failure is not None and failure["reason"]:
        label_parts.append(failure["reason"])
    markers = list(clear_marker_specs(frame_id))
    paths = {"f_candidate": tuple(f_points)}
    markers.append(_marker(
        CANDIDATE_F_NAMESPACE, 2, "TEXT_VIEW_FACING", frame_id,
        position=(f_points[len(f_points) // 2][0],
                  f_points[len(f_points) // 2][1], 0.40),
        text=" | ".join(label_parts),
        color=CANDIDATE_F_COLOR, scale=(0.0, 0.0, 0.14)))
    for vehicle_id, _length, _width, center_points, obbs in \
            checked_vehicles:
        center_namespace = CANDIDATE_CENTER_NAMESPACE.format(vehicle_id)
        obb_namespace = CANDIDATE_OBB_NAMESPACE.format(vehicle_id)
        paths[("candidate_vehicle", vehicle_id)] = tuple(center_points)
        markers.append(_marker(
            center_namespace, 2, "TEXT_VIEW_FACING", frame_id,
            position=(center_points[len(center_points) // 2][0],
                      center_points[len(center_points) // 2][1], 0.34),
            text="car{} centre | {}".format(
                vehicle_id,
                classification if classification == CANDIDATE_CERTIFIED
                else diagnostic_label),
            color=CANDIDATE_CENTER_COLOR, scale=(0.0, 0.0, 0.14)))
        for index, corners in enumerate(obbs):
            markers.append(_marker(
                obb_namespace, index, "LINE_STRIP", frame_id,
                points=tuple((x, y, CANDIDATE_OBB_Z_M)
                             for x, y in corners),
                color=CANDIDATE_OBB_COLOR, scale=(0.010, 0.0, 0.0)))
    markers.extend(_failure_marker_specs(failure, frame_id))
    return tuple(markers), paths


def build_candidate_markers(document, frame_id="map"):
    """Display-only wrapper returning just the candidate label/OBB markers."""
    return build_candidate_display(document, frame_id)[0]


def build_hold_markers(formation_id, qf, vehicles, reason, frame_id="map",
                       show_cargo_reason=True):
    """Static bodies and cargo reference only; never an executable path.

    ``vehicles`` contains (PlaybackVehicleSpec, x, y, body_heading) tuples.
    Caller must clear obsolete path markers before showing this static state.
    The compact labelling used by the normal HUD also reports the static speed
    as exactly zero; there is no model, so no wheel or mode confirmation is
    invented.  The historical cargo reason remains the default display.
    """
    if not isinstance(show_cargo_reason, bool):
        raise PlaybackValidationError(
            "build_hold_markers.show_cargo_reason must be a bool")
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
            position=(x, y, 0.4),
            text="car{} | STATIC | v=0.00 m/s".format(spec.vehicle_id),
            color=color, scale=(0.0, 0.0, 0.18)))
    x, y, yaw = qf
    ns = shared_namespace(formation_id, "cargo")
    markers.append(_marker(ns, len(markers) + 1, "SPHERE", frame_id,
        position=(x, y, 0.1), color=WHITE, scale=(0.16, 0.16, 0.16)))
    markers.append(_marker(ns, len(markers) + 1, "ARROW", frame_id,
        points=((x, y, 0.06), (x + 0.7 * math.cos(yaw),
                                y + 0.7 * math.sin(yaw), 0.06)),
        color=WHITE, scale=(0.035, 0.08, 0.12)))
    cargo_text = "Cargo"
    if show_cargo_reason:
        cargo_text += " | " + reason
    cargo_text += "\nSTATIC / no executable trajectory"
    markers.append(_marker(ns, len(markers) + 1, "TEXT_VIEW_FACING", frame_id,
        position=(x, y, 0.85), text=cargo_text,
        color=WHITE, scale=(0.0, 0.0, 0.20)))
    return tuple(markers)


def status_document(trajectory=None, frame=None, status=None, *,
                    cleared=False, marker_count=0, namespaces=(),
                    invalid_reason="", frame_id="map",
                    terminal_center_error_m=None,
                    terminal_heading_error_rad=None,
                    model_sha256=None, backend=None,
                    semantic_version=None,
                    request_sha256=None, planning_context_sha256=None):
    """Strict-JSON playback status for smoke tests and RViz-side inspection."""
    if cleared:
        return {
            "schema": PLAYBACK_SCHEMA,
            "state": CLEARED,
            "cleared": True,
            "frame_id": frame_id,
            "plan_id": None,
            "formation_id": None,
            "backend": backend,
            "model_sha256": model_sha256,
            "request_sha256": request_sha256,
            "planning_context_sha256": planning_context_sha256,
            "semantic_version": semantic_version,
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
            "backend": backend,
            "model_sha256": model_sha256,
            "request_sha256": request_sha256,
            "planning_context_sha256": planning_context_sha256,
            "semantic_version": semantic_version,
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
    audit_digest = trajectory.model_sha256
    if audit_digest is not None:
        if model_sha256 is not None and model_sha256 != audit_digest:
            raise PlaybackValidationError(
                "status_document model_sha256 differs from the verified "
                "playback envelope")
        # The verified envelope is the authority for the advertised identity.
        model_sha256 = audit_digest
    audit_backend = trajectory.backend or PLAYBACK_PLAN_BACKEND
    audit_state, executable, label, detail = _status_label(
        status, trajectory.plan_id, audit_digest,
        trajectory.formation_id if audit_digest is not None else None,
        audit_backend if audit_digest is not None else None)
    return {
        "schema": PLAYBACK_SCHEMA,
        "state": frame.state,
        "cleared": False,
        "frame_id": trajectory.frame_id,
        "plan_id": trajectory.plan_id,
        "formation_id": trajectory.formation_id,
        "backend": backend,
        "model_sha256": model_sha256,
        "request_sha256": request_sha256,
        "planning_context_sha256": planning_context_sha256,
        "semantic_version": semantic_version,
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
