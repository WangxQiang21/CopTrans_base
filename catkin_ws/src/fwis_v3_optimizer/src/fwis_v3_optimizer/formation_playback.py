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
    "geometry_audit_config", "dynamic_audit_config",
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


def verify_virtual_a_playback_plan(*, plan_id, formation_id, backend,
                                   semantic_version, model_version,
                                   model_sha256, model_json, certification,
                                   trajectory_fields, expected_request=None,
                                   expected_context=None,
                                   expected_grid_sha256=None,
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
    rejection, never a default acceptance.
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
                    terminal_heading_error_rad=None,
                    model_sha256=None, backend=None,
                    semantic_version=None):
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
