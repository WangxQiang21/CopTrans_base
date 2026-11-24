"""ROS-independent validation of S3 formation request and trajectory messages.

This module freezes the input contract only.  It rebuilds a ``FormationModel``
and ``VehicleShape`` map from message content instead of trusting the message,
but it never searches for a path, assigns A/O/S modes, retimes a trajectory,
checks map collision, or certifies executability.

``plan_id`` identifies one planning request; ``formation_id`` identifies the
fixed offset configuration.  They are deliberately separate fields and neither
is used as the other's identity.
"""
from collections.abc import Mapping as MappingABC
from dataclasses import dataclass
import math
from numbers import Integral
from types import MappingProxyType
from typing import Mapping, Optional, Tuple

from .formation import (FormationModel, FormationPose,
                        FormationValidationError, VehicleOffset)
from .formation_collision import VehicleShape


UINT32_MAX = 2**32 - 1
INVALID_REQUEST = "INVALID_REQUEST"
INVALID_TIMED_TRAJECTORY = "INVALID_TIMED_TRAJECTORY"
DEFAULT_POSITION_TOLERANCE_M = 1e-6
_NANOSECONDS_PER_SECOND = 1000000000


class _Invalid(Exception):
    """Structured first failure found while normalising a message."""

    def __init__(self, field, detail, vehicle_id=None):
        super().__init__("{}: {}".format(field, detail))
        self.field = field
        self.detail = detail
        self.vehicle_id = vehicle_id


def _field(owner, name, field):
    if isinstance(owner, MappingABC):
        if name not in owner:
            raise _Invalid(field, "missing field")
        return owner[name]
    if not hasattr(owner, name):
        raise _Invalid(field, "missing field")
    return getattr(owner, name)


def _sequence(owner, name, field):
    value = _field(owner, name, field)
    if not isinstance(value, (list, tuple)):
        raise _Invalid(field, "expected a sequence, got {!r}".format(type(value).__name__))
    return value


def _float(value, field, vehicle_id=None, positive=False):
    if isinstance(value, (bool, str, bytes)):
        raise _Invalid(field, "expected a finite number, got {!r}".format(value),
                       vehicle_id)
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise _Invalid(field, "expected a finite number, got {!r}".format(value),
                       vehicle_id) from exc
    if not math.isfinite(result):
        raise _Invalid(field, "expected a finite number, got {!r}".format(value),
                       vehicle_id)
    if positive and result <= 0.0:
        raise _Invalid(field, "expected a strictly positive value, got {}".format(
            result), vehicle_id)
    return result


def _integer(value, field, vehicle_id=None):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise _Invalid(field, "expected an integer, got {!r}".format(value),
                       vehicle_id)
    return int(value)


def _uint32(value, field, vehicle_id=None):
    result = _integer(value, field, vehicle_id)
    if not 0 <= result <= UINT32_MAX:
        raise _Invalid(field, "uint32 out of range: {}".format(result),
                       vehicle_id)
    return result


def _boolean(value, field, vehicle_id=None):
    if not isinstance(value, bool):
        raise _Invalid(field, "expected a bool, got {!r}".format(value),
                       vehicle_id)
    return value


def _text(value, field):
    if not isinstance(value, str):
        raise _Invalid(field, "expected a string, got {!r}".format(type(value).__name__))
    if not value.strip():
        raise _Invalid(field, "expected a non-empty string")
    return value


def _point(owner, name, field, vehicle_id=None):
    raw = _field(owner, name, field)
    values = []
    for axis in ("x", "y", "z"):
        axis_field = "{}.{}".format(field, axis)
        values.append(_float(_field(raw, axis, axis_field), axis_field,
                             vehicle_id))
    return tuple(values)


def _pose2d(owner, name, field, vehicle_id=None):
    raw = _field(owner, name, field)
    values = []
    for axis in ("x", "y", "theta"):
        axis_field = "{}.{}".format(field, axis)
        values.append(_float(_field(raw, axis, axis_field), axis_field,
                             vehicle_id))
    return tuple(values)


def _time_parts(value, field):
    # ROS2 messages use sec/nanosec; saved ROS1 fixtures keep secs/nsecs.
    if hasattr(value, "nanosec"):
        return _integer(value.sec, field + ".sec"), _integer(value.nanosec, field + ".nanosec")
    return (_integer(_field(value, "secs", field + ".secs"), field + ".secs"),
            _integer(_field(value, "nsecs", field + ".nsecs"), field + ".nsecs"))


def _stamp(owner, field, name="stamp"):
    raw = _field(owner, name, field)
    secs, nsecs = _time_parts(raw, field)
    if not 0 <= secs <= UINT32_MAX:
        raise _Invalid(field, "secs outside uint32: {}".format(secs))
    if not 0 <= nsecs < _NANOSECONDS_PER_SECOND:
        raise _Invalid(field, "nsecs outside [0, 1e9): {}".format(nsecs))
    if secs == 0 and nsecs == 0:
        raise _Invalid(field, "zero stamp is not an explicit common time")
    return secs, nsecs


def _header(owner):
    raw = _field(owner, "header", "header")
    frame_id = _text(_field(raw, "frame_id", "header.frame_id"),
                     "header.frame_id")
    if frame_id != "map":
        raise _Invalid("header.frame_id",
                       "expected 'map', got {!r}".format(frame_id))
    return _stamp(raw, "header.stamp")


@dataclass(frozen=True)
class VehicleRequest:
    """One normalised vehicle spec with independent body headings."""

    vehicle_id: int
    request_index: int
    offset_x: float
    offset_y: float
    start_x: float
    start_y: float
    start_theta: float
    goal_x: float
    goal_y: float
    goal_theta: float
    goal_heading_free: bool
    body_length_m: float
    body_width_m: float


@dataclass(frozen=True)
class FormationRequestSnapshot:
    """Message content rebuilt into S1/S2 objects, with both IDs preserved."""

    plan_id: int
    formation_id: str
    header_stamp: Tuple[int, int]
    start: FormationPose
    goal: FormationPose
    model: FormationModel
    vehicle_shapes: Mapping[int, VehicleShape]
    vehicles: Tuple[VehicleRequest, ...]
    start_headings: Mapping[int, float]
    goal_headings: Mapping[int, float]
    goal_heading_free: Mapping[int, bool]
    allow_reverse: bool

    @property
    def vehicle_ids(self):
        return self.model.vehicle_ids


@dataclass(frozen=True)
class RequestValidation:
    """Structured validation outcome for a ``FormationPlanRequest``."""

    valid: bool
    reason: str = ""
    field: str = ""
    vehicle_id: Optional[int] = None
    detail: str = ""
    snapshot: Optional[FormationRequestSnapshot] = None

    def as_status(self):
        return {
            "schema": "fwis_v3/formation_status",
            "valid": bool(self.valid),
            "reason": self.reason,
            "field": self.field,
            "vehicle_id": self.vehicle_id,
            "detail": self.detail,
            "search_started": False,
            "trajectory_published": False,
            "executable": False,
        }


@dataclass(frozen=True)
class TimedTrajectorySnapshot:
    """Structurally validated common-time trajectory; not an audit certificate."""

    plan_id: int
    formation_id: str
    header_stamp: Tuple[int, int]
    vehicle_ids: Tuple[int, ...]
    times_from_start: Tuple[Tuple[int, int], ...]
    sample_count: int
    vehicle_count: int


@dataclass(frozen=True)
class TimedTrajectoryValidation:
    """Structured validation outcome for a ``FormationTimedTrajectory``."""

    valid: bool
    reason: str = ""
    field: str = ""
    vehicle_id: Optional[int] = None
    detail: str = ""
    snapshot: Optional[TimedTrajectorySnapshot] = None

    def as_status(self):
        return {
            "schema": "fwis_v3/formation_trajectory_status",
            "valid": bool(self.valid),
            "reason": self.reason,
            "field": self.field,
            "vehicle_id": self.vehicle_id,
            "detail": self.detail,
            "executable": False,
            "independent_audit": False,
        }


def _normalise_vehicle_spec(raw, index):
    prefix = "vehicles[{}]".format(index)
    vehicle_id = _uint32(_field(raw, "vehicle_id", prefix + ".vehicle_id"),
                         prefix + ".vehicle_id")
    offset_x, offset_y, offset_z = _point(
        raw, "offset_in_formation", prefix + ".offset_in_formation", vehicle_id)
    if offset_z != 0.0:
        raise _Invalid(prefix + ".offset_in_formation.z",
                       "z must be exactly 0, got {}".format(offset_z), vehicle_id)
    start_x, start_y, start_theta = _pose2d(
        raw, "start_center", prefix + ".start_center", vehicle_id)
    goal_x, goal_y, goal_theta = _pose2d(
        raw, "goal_center", prefix + ".goal_center", vehicle_id)
    goal_heading_free = _boolean(
        _field(raw, "goal_heading_free", prefix + ".goal_heading_free"),
        prefix + ".goal_heading_free", vehicle_id)
    body_length_m = _float(
        _field(raw, "body_length_m", prefix + ".body_length_m"),
        prefix + ".body_length_m", vehicle_id, positive=True)
    body_width_m = _float(
        _field(raw, "body_width_m", prefix + ".body_width_m"),
        prefix + ".body_width_m", vehicle_id, positive=True)
    return VehicleRequest(
        vehicle_id=vehicle_id, request_index=index,
        offset_x=offset_x, offset_y=offset_y,
        start_x=start_x, start_y=start_y, start_theta=start_theta,
        goal_x=goal_x, goal_y=goal_y, goal_theta=goal_theta,
        goal_heading_free=goal_heading_free,
        body_length_m=body_length_m, body_width_m=body_width_m)


def _build_model_and_shapes(vehicles):
    model = FormationModel([
        VehicleOffset(vehicle.vehicle_id, vehicle.offset_x, vehicle.offset_y)
        for vehicle in vehicles
    ])
    shapes = MappingProxyType({
        vehicle.vehicle_id: VehicleShape(vehicle.body_length_m,
                                         vehicle.body_width_m)
        for vehicle in vehicles
    })
    return model, shapes


def _position_error(expected, actual, tolerance):
    return math.hypot(actual[0] - expected[0], actual[1] - expected[1]) > tolerance


def _normalise_request(request, tolerance):
    if request is None:
        raise _Invalid("request", "missing FormationPlanRequest")
    header_stamp = _header(request)
    plan_id = _uint32(_field(request, "plan_id", "plan_id"), "plan_id")
    formation_id = _text(_field(request, "formation_id", "formation_id"),
                         "formation_id")
    start_values = _pose2d(request, "formation_start", "formation_start")
    goal_values = _pose2d(request, "formation_goal", "formation_goal")
    start = FormationPose(*start_values)
    goal = FormationPose(*goal_values)
    allow_reverse = _boolean(_field(request, "allow_reverse", "allow_reverse"),
                             "allow_reverse")
    raw_vehicles = _sequence(request, "vehicles", "vehicles")
    if not raw_vehicles:
        raise _Invalid("vehicles", "empty vehicle list")

    vehicles = []
    seen_ids = set()
    for index, raw in enumerate(raw_vehicles):
        vehicle = _normalise_vehicle_spec(raw, index)
        if vehicle.vehicle_id in seen_ids:
            raise _Invalid(
                "vehicles[{}].vehicle_id".format(index),
                "duplicate vehicle_id={}".format(vehicle.vehicle_id),
                vehicle.vehicle_id)
        seen_ids.add(vehicle.vehicle_id)
        vehicles.append(vehicle)

    try:
        model, shapes = _build_model_and_shapes(vehicles)
    except FormationValidationError as exc:
        raise _Invalid("vehicles.offset_in_formation",
                       "invalid fixed offsets: {}".format(exc)) from exc

    start_centers = model.centers(start)
    goal_centers = model.centers(goal)
    for vehicle in vehicles:
        prefix = "vehicles[{}]".format(vehicle.request_index)
        if _position_error(start_centers[vehicle.vehicle_id],
                           (vehicle.start_x, vehicle.start_y), tolerance):
            raise _Invalid(
                prefix + ".start_center",
                "center does not equal formation_start + R(psi_F) r_i",
                vehicle.vehicle_id)
        if _position_error(goal_centers[vehicle.vehicle_id],
                           (vehicle.goal_x, vehicle.goal_y), tolerance):
            raise _Invalid(
                prefix + ".goal_center",
                "center does not equal formation_goal + R(psi_F) r_i; "
                "goal_heading_free does not relax center rigidity",
                vehicle.vehicle_id)

    ordered = tuple(sorted(vehicles, key=lambda item: item.vehicle_id))
    return FormationRequestSnapshot(
        plan_id=plan_id,
        formation_id=formation_id,
        header_stamp=header_stamp,
        start=start,
        goal=goal,
        model=model,
        vehicle_shapes=shapes,
        vehicles=ordered,
        start_headings=MappingProxyType(
            {item.vehicle_id: item.start_theta for item in ordered}),
        goal_headings=MappingProxyType(
            {item.vehicle_id: item.goal_theta for item in ordered}),
        goal_heading_free=MappingProxyType(
            {item.vehicle_id: item.goal_heading_free for item in ordered}),
        allow_reverse=allow_reverse)


def validate_formation_request(request, position_tolerance_m=DEFAULT_POSITION_TOLERANCE_M):
    """Validate an S3 request and rebuild S1/S2 objects, or fail structurally."""
    try:
        tolerance = _float(position_tolerance_m, "position_tolerance_m")
        if tolerance < 0.0:
            raise _Invalid("position_tolerance_m",
                           "expected >= 0, got {}".format(tolerance))
        return RequestValidation(True, snapshot=_normalise_request(request, tolerance))
    except _Invalid as exc:
        return RequestValidation(False, INVALID_REQUEST, exc.field,
                                 exc.vehicle_id, exc.detail)
    except FormationValidationError as exc:
        return RequestValidation(False, INVALID_REQUEST,
                                 "vehicles.offset_in_formation", None, str(exc))
    except Exception as exc:  # fail closed on unexpectedly malformed messages
        return RequestValidation(False, INVALID_REQUEST, "request", None,
                                 "unexpected validation failure: {!r}".format(exc))


def _time_key(value, field):
    secs, nsecs = _time_parts(value, field)
    if secs < 0:
        raise _Invalid(field, "negative duration secs={}".format(secs))
    if not 0 <= nsecs < _NANOSECONDS_PER_SECOND:
        raise _Invalid(field, "nsecs outside [0, 1e9): {}".format(nsecs))
    return secs, nsecs


def _normalise_timed_trajectory(message):
    if message is None:
        raise _Invalid("trajectory", "missing FormationTimedTrajectory")
    header_stamp = _header(message)
    plan_id = _uint32(_field(message, "plan_id", "plan_id"), "plan_id")
    formation_id = _text(_field(message, "formation_id", "formation_id"),
                         "formation_id")
    raw_vehicles = _sequence(message, "vehicles", "vehicles")
    if not raw_vehicles:
        raise _Invalid("vehicles", "empty vehicle list")

    vehicles = []
    seen_ids = set()
    for index, raw in enumerate(raw_vehicles):
        vehicle = _normalise_vehicle_spec(raw, index)
        if vehicle.vehicle_id in seen_ids:
            raise _Invalid(
                "vehicles[{}].vehicle_id".format(index),
                "duplicate vehicle_id={}".format(vehicle.vehicle_id),
                vehicle.vehicle_id)
        seen_ids.add(vehicle.vehicle_id)
        vehicles.append(vehicle)
    try:
        _, _ = _build_model_and_shapes(vehicles)
    except FormationValidationError as exc:
        raise _Invalid("vehicles.offset_in_formation",
                       "invalid fixed offsets: {}".format(exc)) from exc

    raw_times = _sequence(message, "times_from_start", "times_from_start")
    if not raw_times:
        raise _Invalid("times_from_start", "empty common time grid")
    times = []
    previous = None
    for index, raw in enumerate(raw_times):
        field = "times_from_start[{}]".format(index)
        current = _time_key(raw, field)
        if previous is not None and current <= previous:
            raise _Invalid(field,
                           "time grid is not strictly increasing: {} then {}".format(
                               previous, current))
        times.append(current)
        previous = current

    raw_poses = _sequence(message, "formation_poses", "formation_poses")
    if len(raw_poses) != len(times):
        raise _Invalid("formation_poses",
                       "length {} does not match times_from_start length {}".format(
                           len(raw_poses), len(times)))
    for index, raw in enumerate(raw_poses):
        _float(_field(raw, "x", "formation_poses[{}].x".format(index)),
               "formation_poses[{}].x".format(index))
        _float(_field(raw, "y", "formation_poses[{}].y".format(index)),
               "formation_poses[{}].y".format(index))
        _float(_field(raw, "theta", "formation_poses[{}].theta".format(index)),
               "formation_poses[{}].theta".format(index))

    raw_trajectories = _sequence(message, "trajectories", "trajectories")
    if len(raw_trajectories) != len(vehicles):
        raise _Invalid("trajectories",
                       "length {} does not match vehicles length {}".format(
                           len(raw_trajectories), len(vehicles)))
    expected_ids = {vehicle.vehicle_id for vehicle in vehicles}
    trajectory_ids = set()
    for index, raw in enumerate(raw_trajectories):
        prefix = "trajectories[{}]".format(index)
        vehicle_id = _uint32(_field(raw, "vehicle_id", prefix + ".vehicle_id"),
                             prefix + ".vehicle_id")
        if vehicle_id in trajectory_ids:
            raise _Invalid(prefix + ".vehicle_id",
                           "duplicate trajectory vehicle_id={}".format(vehicle_id),
                           vehicle_id)
        trajectory_ids.add(vehicle_id)
        raw_points = _sequence(raw, "points", prefix + ".points")
        if len(raw_points) != len(times):
            raise _Invalid(
                prefix + ".points",
                "length {} does not match common time grid length {}".format(
                    len(raw_points), len(times)), vehicle_id)
        for point_index, point in enumerate(raw_points):
            point_field = "{}.points[{}]".format(prefix, point_index)
            point_time = _time_key(
                _field(point, "time_from_start", point_field + ".time_from_start"),
                point_field + ".time_from_start")
            if point_time != times[point_index]:
                raise _Invalid(
                    point_field + ".time_from_start",
                    "timestamp {} differs from common grid {}".format(
                        point_time, times[point_index]), vehicle_id)
            _pose2d(point, "center_pose", point_field + ".center_pose", vehicle_id)
            _float(_field(point, "center_speed_mps", point_field + ".center_speed_mps"),
                   point_field + ".center_speed_mps", vehicle_id)
            _float(_field(point, "center_yaw_rate_radps",
                          point_field + ".center_yaw_rate_radps"),
                   point_field + ".center_yaw_rate_radps", vehicle_id)
            mode = _integer(_field(point, "mode", point_field + ".mode"),
                            point_field + ".mode", vehicle_id)
            if mode not in (0, 1, 2):
                raise _Invalid(point_field + ".mode",
                               "mode must be 0/1/2 (A/O/S), got {}".format(mode),
                               vehicle_id)
            _float(_field(point, "mode_control", point_field + ".mode_control"),
                   point_field + ".mode_control", vehicle_id)

    if trajectory_ids != expected_ids:
        missing = sorted(expected_ids - trajectory_ids)
        extra = sorted(trajectory_ids - expected_ids)
        raise _Invalid("trajectories",
                       "vehicle ID set mismatch missing={} extra={}".format(
                           missing, extra))

    return TimedTrajectorySnapshot(
        plan_id=plan_id,
        formation_id=formation_id,
        header_stamp=header_stamp,
        vehicle_ids=tuple(sorted(expected_ids)),
        times_from_start=tuple(times),
        sample_count=len(times),
        vehicle_count=len(vehicles))


def validate_formation_timed_trajectory(message):
    """Check the S3 common-time structural contract, not executable safety."""
    try:
        return TimedTrajectoryValidation(
            True, snapshot=_normalise_timed_trajectory(message))
    except _Invalid as exc:
        return TimedTrajectoryValidation(
            False, INVALID_TIMED_TRAJECTORY, exc.field, exc.vehicle_id, exc.detail)
    except FormationValidationError as exc:
        return TimedTrajectoryValidation(False, INVALID_TIMED_TRAJECTORY,
                                         "vehicles.offset_in_formation", None,
                                         str(exc))
    except Exception as exc:
        return TimedTrajectoryValidation(
            False, INVALID_TIMED_TRAJECTORY, "trajectory", None,
            "unexpected validation failure: {!r}".format(exc))
