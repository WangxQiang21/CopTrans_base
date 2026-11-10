"""S6 continuous rigid-formation geometry and independent plan audit.

The S4 seed contains piecewise formation-reference primitives.  This module
turns them into ``FormationBlock`` controls with a block-local progress
``lambda`` and per-vehicle A/O/S segments assigned by S5.  Vehicle centers are
never stored as independent curves: every sample is recomputed from
``p_i(lambda) = p_F(lambda) + R(psi_F(lambda)) r_i`` and an independent body
heading function.

The highest certification produced here is ``KINEMATIC_VALID``.  It is a
geometry/mode candidate only, not a collision, timing, actuator or execution
certificate.  ``ContinuousPlan`` for a single vehicle is neither imported nor
modified.
"""
from dataclasses import dataclass
import math
from typing import Mapping, Optional, Tuple

from .formation import (
    FormationModel,
    FormationPose,
    FormationValidationError,
    VehicleOffset,
)
from .formation_collision import (
    CollisionReport,
    FormationCollisionChecker,
    FormationState,
    SweepPolicy,
)
from .formation_kinematics import (
    COMPOSITE,
    KinematicLimits,
    OBLIQUE,
    ROTATION,
    TRANSLATION,
    FormationKinematicBlock,
    FormationModeAssignment,
    VehicleModeSegment,
    audit_mode_assignment,
    enumerate_mode_assignments,
    normalize_allowed_modes,
)


GEOMETRY_VALID = "GEOMETRY_VALID"
KINEMATIC_VALID = "KINEMATIC_VALID"
VALID_CERTIFICATIONS = (GEOMETRY_VALID, KINEMATIC_VALID)

_GEOMETRY_TOL = 1e-7
_UINT32_MAX = 2**32 - 1


def _finite(value, context, positive=False, nonnegative=False):
    if isinstance(value, bool) or isinstance(value, (str, bytes)):
        raise FormationValidationError(
            "NONFINITE_VALUE context={} value={!r}".format(context, value))
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise FormationValidationError(
            "NONFINITE_VALUE context={} value={!r}".format(context, value)) from exc
    if not math.isfinite(result):
        raise FormationValidationError(
            "NONFINITE_VALUE context={} value={!r}".format(context, value))
    if positive and result <= 0.0:
        raise FormationValidationError(
            "NONPOSITIVE_VALUE context={} value={!r}".format(context, value))
    if nonnegative and result < 0.0:
        raise FormationValidationError(
            "NEGATIVE_VALUE context={} value={!r}".format(context, value))
    return result


def _uint32(value, context):
    if isinstance(value, bool) or not isinstance(value, int):
        raise FormationValidationError(
            "INVALID_PLAN_ID context={} value={!r}: expected uint32".format(
                context, value))
    if not 0 <= value <= _UINT32_MAX:
        raise FormationValidationError(
            "INVALID_PLAN_ID context={} value={!r}: outside uint32".format(
                context, value))
    return value


def _text(value, context):
    if not isinstance(value, str) or not value:
        raise FormationValidationError(
            "INVALID_TEXT context={} value={!r}: expected a non-empty string"
            .format(context, value))
    return value


def _attribute(owner, name, context):
    try:
        return getattr(owner, name)
    except AttributeError as exc:
        raise FormationValidationError(
            "MISSING_FIELD context={} field={}".format(context, name)) from exc


def _pose_from(value, context):
    if isinstance(value, FormationPose):
        return value
    if isinstance(value, Mapping):
        if "psi" in value:
            psi = value["psi"]
        elif "theta" in value:
            psi = value["theta"]
        else:
            raise FormationValidationError(
                "INVALID_FORMATION_POSE context={} missing psi".format(context))
        return FormationPose(
            _finite(value.get("x"), context + ".x"),
            _finite(value.get("y"), context + ".y"),
            _finite(psi, context + ".psi"))
    try:
        owner_x = getattr(value, "x")
        owner_y = getattr(value, "y")
        if hasattr(value, "psi"):
            psi = getattr(value, "psi")
        elif hasattr(value, "theta"):
            psi = getattr(value, "theta")
        else:
            raise FormationValidationError(
                "INVALID_FORMATION_POSE context={} missing psi/theta".format(
                    context))
    except AttributeError as exc:
        raise FormationValidationError(
            "INVALID_FORMATION_POSE context={} expected x/y/psi".format(
                context)) from exc
    return FormationPose(
        _finite(owner_x, context + ".x"),
        _finite(owner_y, context + ".y"),
        _finite(psi, context + ".psi"))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _angle_error(first, second):
    return abs(_wrap(first - second))


def _heading_pairs(values, vehicle_ids, context):
    if isinstance(values, Mapping):
        items = list(values.items())
    else:
        try:
            items = list(values)
        except TypeError as exc:
            raise FormationValidationError(
                "INVALID_BODY_HEADINGS context={}".format(context)) from exc
    result = {}
    expected = set(vehicle_ids)
    for item in items:
        if isinstance(item, Mapping):
            vehicle_id = item.get("vehicle_id")
            theta = item.get("theta")
        else:
            try:
                vehicle_id, theta = item
            except (TypeError, ValueError) as exc:
                raise FormationValidationError(
                    "INVALID_BODY_HEADINGS context={} expected pairs".format(
                        context)) from exc
        vehicle_id = _uint32(vehicle_id, context + ".vehicle_id")
        if vehicle_id not in expected:
            raise FormationValidationError(
                "UNKNOWN_VEHICLE_ID context={} vehicle_id={}".format(
                    context, vehicle_id))
        if vehicle_id in result:
            raise FormationValidationError(
                "DUPLICATE_VEHICLE_ID context={} vehicle_id={}".format(
                    context, vehicle_id))
        result[vehicle_id] = _finite(theta, "{} theta".format(context))
    missing = sorted(expected - set(result))
    if missing:
        raise FormationValidationError(
            "MISSING_VEHICLE_HEADING context={} missing={}".format(
                context, missing))
    return tuple(sorted(result.items()))


def _heading_map(pairs):
    result = {}
    for vehicle_id, theta in pairs:
        if vehicle_id in result:
            raise FormationValidationError(
                "DUPLICATE_VEHICLE_ID context=headings vehicle_id={}".format(
                    vehicle_id))
        result[vehicle_id] = theta
    return result


@dataclass(frozen=True)
class FormationPathControl:
    """One piecewise formation-reference control on local ``lambda``."""

    geometry_type: str
    parameter_length: float
    start_qf: FormationPose
    end_qf: FormationPose
    pivot_x: float = 0.0
    pivot_y: float = 0.0

    def __post_init__(self):
        if self.geometry_type not in (TRANSLATION, ROTATION):
            raise FormationValidationError(
                "UNSUPPORTED_GEOMETRY_TYPE {}".format(self.geometry_type))
        object.__setattr__(self, "parameter_length", _finite(
            self.parameter_length, "FormationPathControl.parameter_length",
            positive=True))
        object.__setattr__(self, "start_qf", _pose_from(
            self.start_qf, "FormationPathControl.start_qf"))
        object.__setattr__(self, "end_qf", _pose_from(
            self.end_qf, "FormationPathControl.end_qf"))
        object.__setattr__(self, "pivot_x", _finite(
            self.pivot_x, "FormationPathControl.pivot_x"))
        object.__setattr__(self, "pivot_y", _finite(
            self.pivot_y, "FormationPathControl.pivot_y"))
        if self.geometry_type == TRANSLATION:
            if not _angle_error(self.start_qf.psi, self.end_qf.psi) <= (
                    _GEOMETRY_TOL):
                raise FormationValidationError(
                    "INVALID_TRANSLATION_CONTROL psi_F must remain constant")
            displacement = math.hypot(
                self.end_qf.x - self.start_qf.x,
                self.end_qf.y - self.start_qf.y)
            if abs(displacement - self.parameter_length) > (
                    _GEOMETRY_TOL +
                    1e-9 * max(1.0, self.parameter_length)):
                raise FormationValidationError(
                    "INVALID_TRANSLATION_CONTROL parameter_length mismatch")
            return
        delta_psi = self.delta_psi
        if abs(abs(delta_psi) - self.parameter_length) > (
                _GEOMETRY_TOL +
                1e-9 * max(1.0, self.parameter_length)):
            raise FormationValidationError(
                "INVALID_ROTATION_CONTROL parameter_length must equal |dpsi|")
        expected = self.qf_at(self.parameter_length)
        if math.hypot(
                expected.x - self.end_qf.x,
                expected.y - self.end_qf.y) > _GEOMETRY_TOL:
            raise FormationValidationError(
                "INVALID_ROTATION_CONTROL endpoint does not match the pivot")

    @property
    def delta_psi(self):
        return self.end_qf.psi - self.start_qf.psi

    def qf_at(self, station):
        """Recompute the formation reference pose at local progress lambda."""
        station = _finite(station, "FormationPathControl.qf_at")
        if station < -_GEOMETRY_TOL or station > (
                self.parameter_length + _GEOMETRY_TOL):
            raise FormationValidationError(
                "INVALID_BLOCK_STATION {} outside [0, {}]".format(
                    station, self.parameter_length))
        fraction = station / self.parameter_length
        if self.geometry_type == TRANSLATION:
            return FormationPose(
                self.start_qf.x + fraction * (
                    self.end_qf.x - self.start_qf.x),
                self.start_qf.y + fraction * (
                    self.end_qf.y - self.start_qf.y),
                self.start_qf.psi)
        cosine = math.cos(self.start_qf.psi)
        sine = math.sin(self.start_qf.psi)
        pivot_world_x = (
            self.start_qf.x + cosine * self.pivot_x -
            sine * self.pivot_y)
        pivot_world_y = (
            self.start_qf.y + sine * self.pivot_x +
            cosine * self.pivot_y)
        psi = self.start_qf.psi + fraction * self.delta_psi
        cn, sn = math.cos(psi), math.sin(psi)
        return FormationPose(
            pivot_world_x - cn * self.pivot_x + sn * self.pivot_y,
            pivot_world_y - sn * self.pivot_x - cn * self.pivot_y,
            psi)

    def dqf_dlambda(self, station):
        """Exact first derivative of ``q_F(lambda)``, including unwrapped psi."""
        station = _finite(station, "FormationPathControl.dqf_dlambda")
        if station < -_GEOMETRY_TOL or station > (
                self.parameter_length + _GEOMETRY_TOL):
            raise FormationValidationError(
                "INVALID_BLOCK_STATION {} outside [0, {}]".format(
                    station, self.parameter_length))
        if self.geometry_type == TRANSLATION:
            return (
                (self.end_qf.x - self.start_qf.x) / self.parameter_length,
                (self.end_qf.y - self.start_qf.y) / self.parameter_length,
                0.0)
        fraction = station / self.parameter_length
        psi = self.start_qf.psi + fraction * self.delta_psi
        cosine, sine = math.cos(psi), math.sin(psi)
        # q_F = pivot_world - R(psi) p; d/dlambda = -(dpsi/L) R(psi) J p.
        jx, jy = -self.pivot_y, self.pivot_x
        rotated_x = cosine * jx - sine * jy
        rotated_y = sine * jx + cosine * jy
        scale = self.delta_psi / self.parameter_length
        return (-scale * rotated_x, -scale * rotated_y, scale)

    def as_dict(self):
        return {
            "geometry_type": self.geometry_type,
            "parameter_length": self.parameter_length,
            "start_qf": [self.start_qf.x, self.start_qf.y,
                         self.start_qf.psi],
            "end_qf": [self.end_qf.x, self.end_qf.y, self.end_qf.psi],
            "pivot_x": self.pivot_x,
            "pivot_y": self.pivot_y,
        }


@dataclass(frozen=True)
class FormationBlock:
    """Structured formation control plus one A/O/S segment per vehicle."""

    geometry_type: str
    parameter_length: float
    formation_controls: Tuple[FormationPathControl, ...]
    vehicle_mode_segments: Tuple[VehicleModeSegment, ...]
    block_index: int = 0

    def __post_init__(self):
        if self.geometry_type not in (TRANSLATION, ROTATION):
            raise FormationValidationError(
                "UNSUPPORTED_GEOMETRY_TYPE {}".format(self.geometry_type))
        object.__setattr__(self, "parameter_length", _finite(
            self.parameter_length, "FormationBlock.parameter_length",
            positive=True))
        if isinstance(self.block_index, bool) or not isinstance(
                self.block_index, int) or self.block_index < 0:
            raise FormationValidationError(
                "INVALID_BLOCK_INDEX {}".format(self.block_index))
        try:
            controls = tuple(self.formation_controls)
            segments = tuple(self.vehicle_mode_segments)
        except TypeError as exc:
            raise FormationValidationError(
                "INVALID_BLOCK_COMPONENTS") from exc
        if len(controls) != 1:
            raise FormationValidationError(
                "INVALID_FORMATION_CONTROLS expected exactly one control")
        control = controls[0]
        if not isinstance(control, FormationPathControl):
            raise FormationValidationError(
                "INVALID_FORMATION_CONTROLS expected FormationPathControl")
        if control.geometry_type != self.geometry_type:
            raise FormationValidationError(
                "BLOCK_CONTROL_TYPE_MISMATCH")
        if abs(control.parameter_length - self.parameter_length) > _GEOMETRY_TOL:
            raise FormationValidationError(
                "BLOCK_CONTROL_LENGTH_MISMATCH")
        segment_map = {}
        for segment in segments:
            if not isinstance(segment, VehicleModeSegment):
                raise FormationValidationError(
                    "INVALID_MODE_SEGMENT expected VehicleModeSegment")
            if segment.vehicle_id in segment_map:
                raise FormationValidationError(
                    "DUPLICATE_MODE_SEGMENT vehicle_id={}".format(
                        segment.vehicle_id))
            segment_map[segment.vehicle_id] = segment
            if segment.start_station != 0.0 or abs(
                    segment.end_station - self.parameter_length) > _GEOMETRY_TOL:
                raise FormationValidationError(
                    "MODE_SEGMENT_STATION_MISMATCH vehicle_id={}".format(
                        segment.vehicle_id))
        if not segments:
            raise FormationValidationError(
                "EMPTY_MODE_SEGMENTS")
        object.__setattr__(self, "formation_controls", (control,))
        object.__setattr__(
            self, "vehicle_mode_segments",
            tuple(sorted(segments, key=lambda item: item.vehicle_id)))

    @property
    def control(self):
        return self.formation_controls[0]

    @property
    def segment_map(self):
        return {segment.vehicle_id: segment
                for segment in self.vehicle_mode_segments}

    @property
    def vehicle_ids(self):
        return tuple(segment.vehicle_id
                     for segment in self.vehicle_mode_segments)

    @property
    def start_headings(self):
        return tuple((segment.vehicle_id, segment.theta_start)
                     for segment in self.vehicle_mode_segments)

    @property
    def end_headings(self):
        return tuple((segment.vehicle_id, segment.theta_end)
                     for segment in self.vehicle_mode_segments)

    @property
    def mode_word(self):
        return tuple((segment.vehicle_id, segment.mode, segment.direction)
                     for segment in self.vehicle_mode_segments)

    def body_heading_at(self, vehicle_id, station):
        if isinstance(vehicle_id, bool) or not isinstance(vehicle_id, int):
            raise FormationValidationError("INVALID_VEHICLE_ID")
        try:
            segment = self.segment_map[vehicle_id]
        except KeyError as exc:
            raise FormationValidationError(
                "UNKNOWN_VEHICLE_ID vehicle_id={}".format(vehicle_id)) from exc
        station = _finite(station, "FormationBlock.body_heading_at")
        fraction = station / self.parameter_length
        return segment.theta_start + fraction * (
            segment.theta_end - segment.theta_start)

    def body_headings_at(self, station):
        return {segment.vehicle_id: self.body_heading_at(
            segment.vehicle_id, station)
            for segment in self.vehicle_mode_segments}

    def qf_at(self, station):
        return self.control.qf_at(station)

    def candidate_key(self):
        return (
            self.geometry_type,
            round(self.parameter_length, 12),
            tuple(round(value, 12) for value in (
                self.control.start_qf.x, self.control.start_qf.y,
                self.control.start_qf.psi,
                self.control.end_qf.x, self.control.end_qf.y,
                self.control.end_qf.psi,
                self.control.pivot_x, self.control.pivot_y)),
            tuple(
                (segment.vehicle_id, segment.mode, segment.direction,
                 round(segment.control, 12),
                 round(segment.theta_start, 12),
                 round(segment.theta_end, 12),
                 round(segment.travel_distance, 12))
                for segment in self.vehicle_mode_segments),
        )

    def as_dict(self):
        return {
            "block_index": self.block_index,
            "geometry_type": self.geometry_type,
            "parameter_length": self.parameter_length,
            "formation_controls": [self.control.as_dict()],
            "vehicle_mode_segments": [
                {
                    "vehicle_id": segment.vehicle_id,
                    "start_station": segment.start_station,
                    "end_station": segment.end_station,
                    "mode": segment.mode,
                    "direction": segment.direction,
                    "theta_start": segment.theta_start,
                    "theta_end": segment.theta_end,
                    "control": segment.control,
                    "travel_distance": segment.travel_distance,
                }
                for segment in self.vehicle_mode_segments
            ],
        }


@dataclass(frozen=True)
class FormationContinuousPlan:
    """Continuous multi-vehicle geometry candidate; never an execution plan."""

    plan_id: int
    formation_id: str
    start: FormationPose
    goal: FormationPose
    offsets: Tuple[VehicleOffset, ...]
    vehicle_start_headings: Tuple[Tuple[int, float], ...]
    vehicle_goal_headings: Tuple[Tuple[int, float], ...]
    blocks: Tuple[FormationBlock, ...]
    allow_reverse: bool = False
    goal_heading_free: Tuple[Tuple[int, bool], ...] = ()
    certification: str = GEOMETRY_VALID
    dynamic_profile: str = "static_geometry"
    source_signature: Tuple = ()

    def __post_init__(self):
        _uint32(self.plan_id, "FormationContinuousPlan.plan_id")
        _text(self.formation_id, "FormationContinuousPlan.formation_id")
        object.__setattr__(self, "start", _pose_from(
            self.start, "FormationContinuousPlan.start"))
        object.__setattr__(self, "goal", _pose_from(
            self.goal, "FormationContinuousPlan.goal"))
        try:
            offsets = tuple(self.offsets)
            blocks = tuple(self.blocks)
        except TypeError as exc:
            raise FormationValidationError(
                "INVALID_PLAN_COMPONENTS") from exc
        model = FormationModel(offsets)
        sorted_offsets = tuple(sorted(offsets, key=lambda item: item.vehicle_id))
        if offsets != sorted_offsets:
            raise FormationValidationError(
                "UNSORTED_OFFSETS plan offsets must be ordered by id")
        object.__setattr__(self, "offsets", sorted_offsets)
        ids = model.vehicle_ids
        object.__setattr__(
            self, "vehicle_start_headings",
            _heading_pairs(self.vehicle_start_headings, ids,
                           "plan.vehicle_start_headings"))
        object.__setattr__(
            self, "vehicle_goal_headings",
            _heading_pairs(self.vehicle_goal_headings, ids,
                           "plan.vehicle_goal_headings"))
        if not isinstance(self.allow_reverse, bool):
            raise FormationValidationError(
                "INVALID_ALLOW_REVERSE expected bool")
        free = {}
        if self.goal_heading_free:
            supplied = dict(self.goal_heading_free)
            for vehicle_id, flag in supplied.items():
                checked_id = _uint32(vehicle_id, "plan.goal_heading_free.id")
                if checked_id not in set(ids):
                    raise FormationValidationError(
                        "UNKNOWN_VEHICLE_ID context=goal_heading_free "
                        "vehicle_id={}".format(checked_id))
                if not isinstance(flag, bool):
                    raise FormationValidationError(
                        "INVALID_HEADING_FREE vehicle_id={}".format(checked_id))
                free[checked_id] = flag
        object.__setattr__(
            self, "goal_heading_free",
            tuple((vehicle_id, free.get(vehicle_id, False))
                  for vehicle_id in ids))
        if self.certification not in VALID_CERTIFICATIONS:
            raise FormationValidationError(
                "INVALID_CERTIFICATION {}".format(self.certification))
        _text(self.dynamic_profile, "FormationContinuousPlan.dynamic_profile")
        if not blocks:
            raise FormationValidationError("EMPTY_FORMATION_PLAN")
        control_start = blocks[0].control.start_qf
        if math.hypot(
                control_start.x - self.start.x,
                control_start.y - self.start.y) > _GEOMETRY_TOL or (
                _angle_error(control_start.psi, self.start.psi) > _GEOMETRY_TOL):
            raise FormationValidationError(
                "PLAN_START_MISMATCH")
        for index, block in enumerate(blocks):
            if not isinstance(block, FormationBlock):
                raise FormationValidationError(
                    "INVALID_FORMATION_BLOCK index={}".format(index))
            if block.block_index != index:
                raise FormationValidationError(
                    "BLOCK_INDEX_MISMATCH index={} stored={}".format(
                        index, block.block_index))
            if set(block.vehicle_ids) != set(ids):
                raise FormationValidationError(
                    "BLOCK_VEHICLE_SET_MISMATCH index={}".format(index))
            if index == 0:
                continue
            previous = blocks[index - 1]
            gap = math.hypot(
                block.control.start_qf.x - previous.control.end_qf.x,
                block.control.start_qf.y - previous.control.end_qf.y)
            if gap > _GEOMETRY_TOL or _angle_error(
                    block.control.start_qf.psi,
                    previous.control.end_qf.psi) > _GEOMETRY_TOL:
                raise FormationValidationError(
                    "BLOCK_CONTINUITY_MISMATCH index={}".format(index))
        object.__setattr__(self, "blocks", blocks)

    @property
    def vehicle_ids(self):
        return tuple(offset.vehicle_id for offset in self.offsets)

    @property
    def start_heading_map(self):
        return _heading_map(self.vehicle_start_headings)

    @property
    def goal_heading_map(self):
        return _heading_map(self.vehicle_goal_headings)

    @property
    def heading_free_map(self):
        return dict(self.goal_heading_free)

    @property
    def total_parameter_length(self):
        return sum(block.parameter_length for block in self.blocks)

    @property
    def mode_switch_count(self):
        count = 0
        previous = None
        for block in self.blocks:
            for segment in block.vehicle_mode_segments:
                current = (segment.mode, segment.direction,
                           round(segment.control, 12))
                if previous is not None and (
                        previous[segment.vehicle_id] != current):
                    count += 1
            previous = {
                segment.vehicle_id: (segment.mode, segment.direction,
                                     round(segment.control, 12))
                for segment in block.vehicle_mode_segments}
        return count

    def mode_word(self):
        return tuple(block.mode_word for block in self.blocks)

    def candidate_key(self):
        return (
            ("ids", self.plan_id, self.formation_id),
            ("offsets", tuple(
                (offset.vehicle_id, round(offset.x, 12),
                 round(offset.y, 12)) for offset in self.offsets)),
            ("start", round(self.start.x, 12), round(self.start.y, 12),
             round(self.start.psi, 12)),
            ("goal", round(self.goal.x, 12), round(self.goal.y, 12),
             round(self.goal.psi, 12)),
            ("start_headings", tuple(
                (vehicle_id, round(theta, 12))
                for vehicle_id, theta in self.vehicle_start_headings)),
            ("goal_headings", tuple(
                (vehicle_id, round(theta, 12))
                for vehicle_id, theta in self.vehicle_goal_headings)),
            ("goal_heading_free", tuple(self.goal_heading_free)),
            ("allow_reverse", self.allow_reverse),
            ("dynamic_profile", self.dynamic_profile),
            ("blocks", tuple(block.candidate_key() for block in self.blocks)),
        )

    def as_status(self):
        return {
            "schema": "formation_continuous_plan/v1",
            "plan_id": self.plan_id,
            "formation_id": self.formation_id,
            "certification": self.certification,
            "geometry_only": self.certification == GEOMETRY_VALID,
            "dynamic_profile": self.dynamic_profile,
            "blocks": len(self.blocks),
            "total_parameter_length": self.total_parameter_length,
            "mode_switches": self.mode_switch_count,
            "executable": False,
            "trajectory_published": False,
        }


@dataclass(frozen=True)
class FormationPlanRejection:
    block_index: Optional[int]
    vehicle_id: Optional[int]
    reason: str
    detail: str
    primitive_index: Optional[int] = None

    def __post_init__(self):
        if self.block_index is not None:
            if isinstance(self.block_index, bool) or not isinstance(
                    self.block_index, int):
                raise FormationValidationError(
                    "INVALID_BLOCK_INDEX")
        if self.vehicle_id is not None:
            _uint32(self.vehicle_id, "FormationPlanRejection.vehicle_id")
        _text(self.reason, "FormationPlanRejection.reason")
        if not isinstance(self.detail, str):
            raise FormationValidationError(
                "INVALID_REJECTION_DETAIL expected string")

    def as_dict(self):
        return {
            "block_index": self.block_index,
            "primitive_index": self.primitive_index,
            "vehicle_id": self.vehicle_id,
            "reason": self.reason,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class FormationPlanBuild:
    """Result of converting one S4 seed into up to K continuous plans."""

    success: bool
    reason: str
    detail: str
    plans: Tuple[FormationContinuousPlan, ...]
    rejections: Tuple[FormationPlanRejection, ...]
    map_version_checked: bool = False
    terminal_max_xy_m: float = 0.0
    terminal_max_yaw_rad: float = 0.0
    certification: str = GEOMETRY_VALID

    def as_status(self):
        return {
            "schema": "formation_plan_build/v1",
            "success": self.success,
            "reason": self.reason,
            "detail": self.detail,
            "certification": self.certification,
            "plans": len(self.plans),
            "map_version_checked": self.map_version_checked,
            "terminal_max_xy_m": self.terminal_max_xy_m,
            "terminal_max_yaw_rad": self.terminal_max_yaw_rad,
            "rejections": [item.as_dict() for item in self.rejections],
            "executable": False,
            "trajectory_published": False,
        }


def _primitive_type(primitive):
    value = _attribute(primitive, "geometry_type", "primitive")
    if isinstance(value, bool):
        raise FormationValidationError(
            "INVALID_PRIMITIVE_TYPE expected uint8 geometry_type")
    if isinstance(value, str):
        lowered = value.lower()
        if lowered == TRANSLATION:
            return TRANSLATION
        if lowered == ROTATION:
            return ROTATION
        if lowered == COMPOSITE:
            return COMPOSITE
        raise FormationValidationError(
            "INVALID_PRIMITIVE_TYPE {}".format(value))
    if not isinstance(value, int) or value not in (0, 1, 2):
        raise FormationValidationError(
            "INVALID_PRIMITIVE_TYPE {}".format(value))
    return (TRANSLATION, ROTATION, COMPOSITE)[value]


def _seed_header(seed):
    header = _attribute(seed, "header", "seed")
    frame_id = _attribute(header, "frame_id", "seed.header")
    if frame_id != "map":
        raise FormationValidationError(
            "SEED_FRAME_MISMATCH expected 'map', got {!r}".format(frame_id))
    stamp = _attribute(header, "stamp", "seed.header")
    secs = _attribute(stamp, "secs", "seed.header.stamp")
    nsecs = _attribute(stamp, "nsecs", "seed.header.stamp")
    if isinstance(secs, bool) or not isinstance(secs, int) or secs < 0:
        raise FormationValidationError("INVALID_SEED_STAMP secs")
    if isinstance(nsecs, bool) or not isinstance(nsecs, int) or not (
            0 <= nsecs < 1000000000):
        raise FormationValidationError("INVALID_SEED_STAMP nsecs")
    if secs == 0 and nsecs == 0:
        raise FormationValidationError(
            "INVALID_SEED_STAMP zero stamp is not explicit")


def _seed_signature(seed):
    signature = getattr(seed, "source_signature", None)
    if signature is None:
        return ()
    try:
        return tuple(signature)
    except TypeError:
        return (signature,)


def _path_control(kinematic):
    return FormationPathControl(
        kinematic.geometry_type,
        kinematic.parameter_length,
        kinematic.start_qf,
        kinematic.end_qf,
        kinematic.pivot_x,
        kinematic.pivot_y)


def _block_from_assignment(kinematic, assignment, block_index):
    return FormationBlock(
        geometry_type=kinematic.geometry_type,
        parameter_length=kinematic.parameter_length,
        formation_controls=(_path_control(kinematic),),
        vehicle_mode_segments=assignment.segments,
        block_index=block_index)


def _empty_build(reason, detail, rejection=None, map_checked=False,
                 certification=GEOMETRY_VALID):
    rejections = () if rejection is None else (rejection,)
    return FormationPlanBuild(
        success=False, reason=reason, detail=detail, plans=(),
        rejections=rejections, map_version_checked=map_checked,
        certification=certification)


def _validate_seed(snapshot, seed, expected_map_signature):
    plan_id = _uint32(_attribute(seed, "plan_id", "seed"), "seed.plan_id")
    formation_id = _attribute(seed, "formation_id", "seed")
    if not isinstance(formation_id, str) or not formation_id:
        raise FormationValidationError(
            "INVALID_SEED_FORMATION_ID")
    if plan_id != snapshot.plan_id:
        raise FormationValidationError(
            "SEED_PLAN_ID_MISMATCH seed={} request={}".format(
                plan_id, snapshot.plan_id))
    if formation_id != snapshot.formation_id:
        raise FormationValidationError(
            "SEED_FORMATION_ID_MISMATCH seed={!r} request={!r}".format(
                formation_id, snapshot.formation_id))
    _seed_header(seed)
    start = _pose_from(_attribute(seed, "formation_start", "seed"),
                       "seed.formation_start")
    goal = _pose_from(_attribute(seed, "formation_goal", "seed"),
                      "seed.formation_goal")
    for name, actual, expected in (
            ("formation_start", start, snapshot.start),
            ("formation_goal", goal, snapshot.goal)):
        xy = math.hypot(actual.x - expected.x, actual.y - expected.y)
        yaw = _angle_error(actual.psi, expected.psi)
        if xy > _GEOMETRY_TOL or yaw > _GEOMETRY_TOL:
            raise FormationValidationError(
                "SEED_{}_MISMATCH xy={:.12g} yaw={:.12g}".format(
                    name.upper(), xy, yaw))
    primitives = list(_attribute(seed, "primitives", "seed"))
    if not primitives:
        raise FormationValidationError("EMPTY_PRIMITIVE_SEQUENCE")

    map_checked = False
    map_detail = "seed carries no map signature"
    seed_signature = getattr(seed, "map_signature", None)
    if expected_map_signature is not None:
        if seed_signature is None:
            map_detail = "expected a map signature but the seed has none"
        else:
            map_checked = True
            if seed_signature != expected_map_signature:
                raise FormationValidationError(
                    "MAP_VERSION_MISMATCH seed={!r} expected={!r}".format(
                        seed_signature, expected_map_signature))
    return plan_id, formation_id, start, goal, primitives, map_checked, map_detail


def _terminal_goal_error(control, goal):
    endpoint = control.qf_at(control.parameter_length)
    return math.hypot(endpoint.x - goal.x, endpoint.y - goal.y), (
        _angle_error(endpoint.psi, goal.psi))


def _fit_terminal_control(kinematic, goal, start_headings, expected_end,
                          goal_heading_free, allow_reverse, limits,
                          max_schedules, allowed_modes=None):
    """Return ``(variants, failure_reason, failure_detail)``.

    Each variant is ``(kinematic, assignment)``; ``assignment=None`` means the
    original block already reaches the explicit goal and its stored schedule
    should be retained.  Otherwise only the last endpoint is changed: a
    translation absorbs the residual with unchanged ``psi_F``, while a rotation
    keeps its explicit pivot and changes only its yaw increment.  No other
    block or vehicle center is independently moved.
    """
    goal_heading_free = goal_heading_free or {}
    xy_error, yaw_error = _terminal_goal_error(
        _path_control(kinematic), goal)
    if xy_error <= _GEOMETRY_TOL and yaw_error <= _GEOMETRY_TOL:
        delta = (kinematic.delta_psi
                 if kinematic.geometry_type == ROTATION else 0.0)
        for vehicle_id, theta in start_headings.items():
            if expected_end is None or goal_heading_free.get(vehicle_id, False):
                continue
            error = _angle_error(theta + delta, expected_end[vehicle_id])
            if error > limits.heading_tolerance_rad:
                return (), "GOAL_HEADING_MISMATCH", (
                    "vehicle_id={} terminal body heading differs from the "
                    "explicit goal by {:.12g} rad without changing position"
                    .format(vehicle_id, error))
        return ((kinematic, None),), None, ""

    if kinematic.geometry_type == TRANSLATION:
        if _angle_error(kinematic.start_qf.psi, goal.psi) > _GEOMETRY_TOL:
            return (), "TERMINAL_FIT_FAILED", (
                "a translation cannot change psi_F from {:.12g} to {:.12g}"
                .format(kinematic.start_qf.psi, goal.psi))
        length = math.hypot(
            goal.x - kinematic.start_qf.x,
            goal.y - kinematic.start_qf.y)
        if length <= _GEOMETRY_TOL:
            return (), "TERMINAL_FIT_FAILED", (
                "the fitted translation has zero length")
        fitted = FormationKinematicBlock(
            TRANSLATION, length, kinematic.start_qf, goal, 0.0, 0.0,
            kinematic.offsets)
    else:
        delta_psi = goal.psi - kinematic.start_qf.psi
        if abs(delta_psi) <= _GEOMETRY_TOL:
            return (), "TERMINAL_FIT_FAILED", (
                "the fitted rotation has zero angle")
        start = kinematic.start_qf
        cosine, sine = math.cos(start.psi), math.sin(start.psi)
        pivot_w = (
            start.x + cosine * kinematic.pivot_x -
            sine * kinematic.pivot_y,
            start.y + sine * kinematic.pivot_x +
            cosine * kinematic.pivot_y)
        next_psi = start.psi + delta_psi
        cn, sn = math.cos(next_psi), math.sin(next_psi)
        endpoint = FormationPose(
            pivot_w[0] - cn * kinematic.pivot_x + sn * kinematic.pivot_y,
            pivot_w[1] - sn * kinematic.pivot_x - cn * kinematic.pivot_y,
            next_psi)
        residual = math.hypot(endpoint.x - goal.x, endpoint.y - goal.y)
        if residual > _GEOMETRY_TOL:
            return (), "TERMINAL_FIT_FAILED", (
                "the explicit pivot cannot reach the goal; residual={:.12g}"
                .format(residual))
        fitted = FormationKinematicBlock(
            ROTATION, abs(delta_psi), start, endpoint,
            kinematic.pivot_x, kinematic.pivot_y, kinematic.offsets)

    assignments = enumerate_mode_assignments(
        fitted, start_headings, limits=limits,
        allow_reverse=allow_reverse,
        expected_end_headings=expected_end,
        goal_heading_free=goal_heading_free,
        max_schedules=max_schedules,
        allowed_modes=allowed_modes)
    variants = []
    for assignment in assignments:
        audit = audit_mode_assignment(
            assignment, limits=limits,
            expected_end_headings=expected_end,
            goal_heading_free=goal_heading_free,
            allow_reverse=allow_reverse)
        if assignment.success and audit.success:
            variants.append((fitted, assignment))
    if not variants:
        reasons = sorted(set(
            getattr(assignment, "reason", "MODE_ASSIGNMENT_FAILED")
            for assignment in assignments))
        return (), "MODE_ASSIGNMENT_FAILED", (
            "no audited terminal mode schedule; reasons={}".format(
                reasons or ["NONE"]))
    return tuple(variants), None, ""


def _schedule_combinations(choice_sets, max_combinations):
    partial = [(0.0, ())]
    for choices in choice_sets:
        expanded = []
        for cost, combination in partial:
            for choice_cost, block in choices:
                expanded.append((cost + choice_cost,
                                 combination + (block,)))
        expanded.sort(key=lambda item: (
            item[0], tuple(block.candidate_key() for block in item[1])))
        partial = expanded[:max_combinations]
        if not partial:
            break
    return partial


def build_continuous_plans(
        snapshot, seed, limits=None, max_candidates=4,
        max_schedule_combinations=64, max_schedules_per_block=4,
        expected_map_signature=None, terminal_tolerance_m=_GEOMETRY_TOL,
        terminal_tolerance_rad=_GEOMETRY_TOL, allowed_modes=None):
    """Convert one S4 geometry-only seed into up to K continuous candidates."""
    limits = limits or KinematicLimits()
    allowed_modes = normalize_allowed_modes(allowed_modes)
    if not isinstance(limits, KinematicLimits):
        raise FormationValidationError(
            "INVALID_KINEMATIC_LIMITS expected KinematicLimits")
    if isinstance(max_candidates, bool) or not isinstance(
            max_candidates, int) or max_candidates <= 0:
        raise FormationValidationError("INVALID_MAX_CANDIDATES")
    if isinstance(max_schedule_combinations, bool) or not isinstance(
            max_schedule_combinations, int) or max_schedule_combinations <= 0:
        raise FormationValidationError("INVALID_MAX_SCHEDULE_COMBINATIONS")
    terminal_tolerance_m = _finite(
        terminal_tolerance_m, "terminal_tolerance_m", nonnegative=True)
    terminal_tolerance_rad = _finite(
        terminal_tolerance_rad, "terminal_tolerance_rad", nonnegative=True)

    try:
        (plan_id, formation_id, seed_start, seed_goal, primitives,
         map_checked, _map_detail) = _validate_seed(
            snapshot, seed, expected_map_signature)
    except FormationValidationError as exc:
        return _empty_build("INVALID_SEED", str(exc))

    offsets = tuple(snapshot.model.offsets)
    model = snapshot.model
    headings = dict(snapshot.start_headings)
    start_headings = tuple(sorted(headings.items()))
    goal_headings = dict(snapshot.goal_headings)
    free_flags = dict(snapshot.goal_heading_free)
    allow_reverse = bool(snapshot.allow_reverse)

    qf = FormationPose(snapshot.start.x, snapshot.start.y,
                       snapshot.start.psi)
    block_choice_sets = []
    rejections = []
    skipped = 0
    for primitive_index, primitive in enumerate(primitives):
        try:
            kind = _primitive_type(primitive)
        except FormationValidationError as exc:
            rejections.append(FormationPlanRejection(
                len(block_choice_sets), None, "INVALID_PRIMITIVE",
                str(exc), primitive_index))
            return _empty_build(
                "INVALID_PRIMITIVE", str(exc), rejections[-1], map_checked)
        if kind == COMPOSITE:
            rejection = FormationPlanRejection(
                len(block_choice_sets), None,
                "COMPOSITE_PRIMITIVE_UNSUPPORTED",
                "composite primitive is not defined in S6",
                primitive_index)
            rejections.append(rejection)
            return _empty_build(
                "COMPOSITE_PRIMITIVE_UNSUPPORTED",
                rejection.detail, rejection, map_checked)
        try:
            kinematic = FormationKinematicBlock.from_primitive(
                qf, primitive, offsets)
        except FormationValidationError as exc:
            message = str(exc)
            if "ZERO_LENGTH_TRANSLATION_BLOCK" in message or (
                    "ZERO_ROTATION_BLOCK" in message):
                skipped += 1
                continue
            rejection = FormationPlanRejection(
                len(block_choice_sets), None, "INVALID_PRIMITIVE_GEOMETRY",
                message, primitive_index)
            rejections.append(rejection)
            return _empty_build(
                rejection.reason, message, rejection, map_checked)
        block_index = len(block_choice_sets)
        # The last block is fitted to the explicit goal after construction;
        # its terminal headings are checked there, because extending a short
        # rotation changes theta_i while preserving the rigid centers.
        try:
            assignments = enumerate_mode_assignments(
                kinematic, headings, limits=limits,
                allow_reverse=allow_reverse,
                max_schedules=max_schedules_per_block,
                allowed_modes=allowed_modes)
        except FormationValidationError as exc:
            rejection = FormationPlanRejection(
                block_index, None, "MODE_ASSIGNMENT_FAILED", str(exc),
                primitive_index)
            rejections.append(rejection)
            return _empty_build(
                rejection.reason, rejection.detail, rejection, map_checked)
        choices = []
        block_rejections = []
        for assignment in assignments:
            if not assignment.success:
                block_rejections.append(FormationPlanRejection(
                    block_index,
                    (assignment.rejections[0].vehicle_id
                     if assignment.rejections else None),
                    assignment.reason, assignment.detail, primitive_index))
                continue
            audit = audit_mode_assignment(
                assignment, limits=limits,
                allow_reverse=allow_reverse)
            if not audit.success:
                block_rejections.append(FormationPlanRejection(
                    block_index, None, audit.reason, audit.detail,
                    primitive_index))
                continue
            block = _block_from_assignment(kinematic, assignment, block_index)
            choices.append((assignment.schedule_cost, block))
        if not choices:
            rejections.extend(block_rejections)
            first = block_rejections[0] if block_rejections else (
                FormationPlanRejection(
                    block_index, None, "MODE_ASSIGNMENT_FAILED",
                    "no audited mode schedule", primitive_index))
            return _empty_build(
                first.reason, first.detail, first, map_checked)
        choices.sort(key=lambda item: (
            item[0], item[1].candidate_key()))
        block_choice_sets.append(choices)
        headings = dict(choices[0][1].end_headings)
        qf = kinematic.end_qf

    if not block_choice_sets:
        return _empty_build(
            "EMPTY_PLAN", "all S4 primitives were zero length",
            FormationPlanRejection(None, None, "EMPTY_PLAN",
                                   "no non-zero primitive"), map_checked)

    combinations = _schedule_combinations(
        block_choice_sets, max_schedule_combinations)
    source_signature = _seed_signature(seed)
    plan_candidates = []
    terminal_max_xy = 0.0
    terminal_max_yaw = 0.0
    for _, combination in combinations:
        last_block = combination[-1]
        last_kinematic = FormationKinematicBlock(
            last_block.geometry_type,
            last_block.parameter_length,
            last_block.control.start_qf,
            last_block.control.end_qf,
            last_block.control.pivot_x,
            last_block.control.pivot_y,
            offsets)
        last_start_headings = dict(last_block.start_headings)
        # The last block is the only one whose endpoint is fitted.  Its start
        # headings were already propagated from the preceding block.
        fit_variants, fit_reason, fit_detail = _fit_terminal_control(
            last_kinematic, snapshot.goal, last_start_headings,
            goal_headings, free_flags, allow_reverse, limits,
            max_schedules_per_block, allowed_modes=allowed_modes)
        if not fit_variants:
            rejections.append(FormationPlanRejection(
                last_block.block_index, None,
                fit_reason or "TERMINAL_FIT_FAILED",
                fit_detail or "terminal fit produced no audited schedule"))
            continue
        for fitted, fitted_assignment in fit_variants:
            if fitted_assignment is None:
                fitted_block = last_block
            else:
                fitted_block = _block_from_assignment(
                    fitted, fitted_assignment, last_block.block_index)
            blocks = combination[:-1] + (fitted_block,)
            try:
                plan = FormationContinuousPlan(
                    plan_id=plan_id,
                    formation_id=formation_id,
                    start=snapshot.start,
                    goal=snapshot.goal,
                    offsets=offsets,
                    vehicle_start_headings=start_headings,
                    vehicle_goal_headings=tuple(sorted(goal_headings.items())),
                    blocks=blocks,
                    allow_reverse=allow_reverse,
                    goal_heading_free=tuple(sorted(free_flags.items())),
                    certification=GEOMETRY_VALID,
                    dynamic_profile="static_geometry",
                    source_signature=source_signature,
                )
            except FormationValidationError as exc:
                rejections.append(FormationPlanRejection(
                    last_block.block_index, None, "INVALID_CONTINUOUS_PLAN",
                    str(exc)))
                continue
            endpoint = plan.blocks[-1].control.qf_at(
                plan.blocks[-1].parameter_length)
            xy = math.hypot(
                endpoint.x - snapshot.goal.x,
                endpoint.y - snapshot.goal.y)
            yaw = _angle_error(endpoint.psi, snapshot.goal.psi)
            terminal_max_xy = max(terminal_max_xy, xy)
            terminal_max_yaw = max(terminal_max_yaw, yaw)
            if xy > terminal_tolerance_m or yaw > terminal_tolerance_rad:
                rejections.append(FormationPlanRejection(
                    last_block.block_index, None, "TERMINAL_ENDPOINT_MISMATCH",
                    "terminal xy={:.12g} yaw={:.12g}".format(xy, yaw)))
                continue
            plan_candidates.append((plan_candidate_score(plan), plan))

    if not plan_candidates:
        if rejections:
            first = rejections[0]
            return _empty_build(
                first.reason, first.detail, first, map_checked)
        return _empty_build(
            "NO_CONTINUOUS_CANDIDATE",
            "no mode schedule reached the explicit formation goal",
            None, map_checked)
    plan_candidates.sort(key=lambda item: (
        item[0], item[1].candidate_key()))
    unique = []
    seen = set()
    for _, plan in plan_candidates:
        key = plan.candidate_key()
        if key in seen:
            continue
        seen.add(key)
        unique.append(plan)
        if len(unique) >= max_candidates:
            break
    detail = (
        "GEOMETRY_BUILT_KINEMATIC_CANDIDATE_NOT_EXECUTABLE; "
        "map_version_checked={}; skipped_zero_primitives={}".format(
            map_checked, skipped))
    return FormationPlanBuild(
        success=True,
        reason="OK",
        detail=detail,
        plans=tuple(unique),
        rejections=tuple(rejections),
        map_version_checked=map_checked,
        terminal_max_xy_m=terminal_max_xy,
        terminal_max_yaw_rad=terminal_max_yaw,
        certification=GEOMETRY_VALID,
    )


@dataclass(frozen=True)
class FormationPlanAudit:
    """Independent geometry/mode audit; never a collision or timing certificate."""

    success: bool
    reason: str
    detail: str
    certification: str
    checks: int
    block_count: int
    sample_count: int
    max_rigidity_error_m: float
    max_terminal_error_xy_m: float
    max_terminal_error_yaw_rad: float
    max_heading_error_rad: float
    collision_prechecked: bool
    collision_success: bool
    collision_reason: str
    collision_detail: str
    rejections: Tuple[FormationPlanRejection, ...] = ()

    def as_status(self):
        return {
            "schema": "formation_plan_audit/v1",
            "success": self.success,
            "reason": self.reason,
            "detail": self.detail,
            "certification": self.certification,
            "checks": self.checks,
            "block_count": self.block_count,
            "sample_count": self.sample_count,
            "max_rigidity_error_m": self.max_rigidity_error_m,
            "max_terminal_error_xy_m": self.max_terminal_error_xy_m,
            "max_terminal_error_yaw_rad": self.max_terminal_error_yaw_rad,
            "max_heading_error_rad": self.max_heading_error_rad,
            "collision_prechecked": self.collision_prechecked,
            "collision_success": self.collision_success,
            "collision_reason": self.collision_reason,
            "collision_detail": self.collision_detail,
            "rejections": [item.as_dict() for item in self.rejections],
            "executable": False,
            "trajectory_published": False,
            "independent_timing_audit": False,
        }


def _audit_failure(reason, detail, rejections=(), checks=0,
                   block_count=0, sample_count=0,
                   max_rigidity=0.0, terminal_xy=0.0, terminal_yaw=0.0,
                   max_heading=0.0, collision_prechecked=False,
                   collision_success=False, collision_reason="",
                   collision_detail=""):
    return FormationPlanAudit(
        success=False, reason=reason, detail=detail,
        certification="REJECTED", checks=checks, block_count=block_count,
        sample_count=sample_count,
        max_rigidity_error_m=max_rigidity,
        max_terminal_error_xy_m=terminal_xy,
        max_terminal_error_yaw_rad=terminal_yaw,
        max_heading_error_rad=max_heading,
        collision_prechecked=collision_prechecked,
        collision_success=collision_success,
        collision_reason=collision_reason,
        collision_detail=collision_detail,
        rejections=tuple(rejections))


@dataclass(frozen=True)
class CollisionPrecheck:
    success: bool
    report: CollisionReport
    samples: int
    blocks: int
    policy: SweepPolicy
    failing_block_index: Optional[int] = None


def _collision_sample_count(block, snapshot, margin_m, policy):
    if block.geometry_type == TRANSLATION:
        bound = block.parameter_length
    else:
        reference_radius = max(
            math.hypot(offset.x, offset.y) for offset in snapshot.model.offsets)
        corner_radius = max(
            math.hypot(
                (shape.length_m + margin_m) / 2.0,
                (shape.width_m + margin_m) / 2.0)
            for shape in snapshot.vehicle_shapes.values())
        bound = (reference_radius + corner_radius) * block.parameter_length
        # Every vehicle's body heading also rotates by the same |dpsi|.
    return max(1, int(math.ceil(
        bound / policy.max_corner_step_m)))


def precheck_plan_collision(
        plan, snapshot, grid, margin_m, policy=None,
        max_samples_per_block=20000):
    """Dense rigid-state OBB precheck before local optimization.

    This is a conservative initial screen, not S9's independent certificate:
    states are sampled, then ``FormationCollisionChecker.swept_check`` adds its
    own discrete motion padding.  A success here never means FEASIBLE.
    """
    policy = policy or SweepPolicy(max_corner_step_m=0.02)
    if not isinstance(policy, SweepPolicy):
        raise FormationValidationError(
            "INVALID_SWEEP_POLICY expected SweepPolicy")
    margin_m = _finite(margin_m, "margin_m", nonnegative=True)
    if isinstance(max_samples_per_block, bool) or not isinstance(
            max_samples_per_block, int) or max_samples_per_block <= 0:
        raise FormationValidationError("INVALID_MAX_SAMPLES_PER_BLOCK")
    try:
        checker = FormationCollisionChecker(
            grid, snapshot.model, snapshot.vehicle_shapes, margin_m)
    except FormationValidationError as exc:
        return CollisionPrecheck(False, CollisionReport(
            False, reason="INVALID_COLLISION_INPUT", detail=str(exc)),
            0, 0, policy)

    states = []
    state_blocks = []
    cumulative = 0.0
    samples = 0
    for block in plan.blocks:
        count = _collision_sample_count(block, snapshot, margin_m, policy)
        if count > max_samples_per_block:
            report = CollisionReport(
                False, reason="SWEEP_SAMPLE_BUDGET_EXCEEDED",
                segment_index=block.block_index,
                detail="block needs {} samples > {}".format(
                    count, max_samples_per_block))
            return CollisionPrecheck(
                False, report, samples, len(plan.blocks), policy,
                block.block_index)
        for index in range(count + 1):
            if index == 0 and states:
                continue
            station = cumulative + block.parameter_length * index / count
            qf = block.qf_at(block.parameter_length * index / count)
            headings = block.body_headings_at(
                block.parameter_length * index / count)
            states.append(FormationState(station, qf, headings))
            state_blocks.append(block.block_index)
            samples += 1
        cumulative += block.parameter_length
    if not states:
        report = CollisionReport(
            False, reason="EMPTY_STATES",
            detail="no collision states were generated")
        return CollisionPrecheck(False, report, 0, len(plan.blocks), policy)
    report = checker.swept_check(states, policy)
    if report.success:
        return CollisionPrecheck(
            True, report, samples, len(plan.blocks), policy)
    # ``swept_check`` reports a state-pair index.  A sample after the first
    # state belongs to the pair's second state; a sample zero belongs to the
    # pair's first state.
    state_index = report.segment_index + (1 if report.sample_index else 0)
    state_index = max(0, min(state_index, len(state_blocks) - 1))
    return CollisionPrecheck(
        False, report, samples, len(plan.blocks), policy,
        state_blocks[state_index])


def audit_continuous_plan(
        plan, snapshot, limits=None, grid=None, margin_m=0.05,
        sweep_policy=None, terminal_tolerance_m=_GEOMETRY_TOL,
        terminal_tolerance_rad=_GEOMETRY_TOL):
    """Recompute a continuous plan and audit every block from raw geometry."""
    if not isinstance(plan, FormationContinuousPlan):
        raise FormationValidationError(
            "INVALID_CONTINUOUS_PLAN expected FormationContinuousPlan")
    limits = limits or KinematicLimits()
    if not isinstance(limits, KinematicLimits):
        raise FormationValidationError(
            "INVALID_KINEMATIC_LIMITS expected KinematicLimits")
    terminal_tolerance_m = _finite(
        terminal_tolerance_m, "terminal_tolerance_m", nonnegative=True)
    terminal_tolerance_rad = _finite(
        terminal_tolerance_rad, "terminal_tolerance_rad", nonnegative=True)
    rejections = []
    checks = 0
    sample_count = 0
    max_rigidity = 0.0
    max_terminal_xy = 0.0
    max_terminal_yaw = 0.0
    max_heading = 0.0

    if plan.plan_id != snapshot.plan_id:
        return _audit_failure(
            "PLAN_ID_MISMATCH", "plan={} request={}".format(
                plan.plan_id, snapshot.plan_id))
    if plan.formation_id != snapshot.formation_id:
        return _audit_failure(
            "FORMATION_ID_MISMATCH", "plan={!r} request={!r}".format(
                plan.formation_id, snapshot.formation_id))
    try:
        plan_offsets = tuple(plan.offsets)
        snapshot_offsets = tuple(snapshot.model.offsets)
        if plan_offsets != snapshot_offsets:
            return _audit_failure(
                "OFFSET_VERSION_MISMATCH",
                "plan offsets are not the request's fixed offsets")
        if math.hypot(
                plan.start.x - snapshot.start.x,
                plan.start.y - snapshot.start.y) > _GEOMETRY_TOL or (
                _angle_error(plan.start.psi, snapshot.start.psi) >
                _GEOMETRY_TOL):
            return _audit_failure(
                "PLAN_START_MISMATCH", "plan start differs from request")
        if math.hypot(
                plan.goal.x - snapshot.goal.x,
                plan.goal.y - snapshot.goal.y) > _GEOMETRY_TOL or (
                _angle_error(plan.goal.psi, snapshot.goal.psi) >
                _GEOMETRY_TOL):
            return _audit_failure(
                "PLAN_GOAL_MISMATCH", "plan goal differs from request")
        if plan.allow_reverse != bool(snapshot.allow_reverse):
            return _audit_failure(
                "ALLOW_REVERSE_MISMATCH",
                "plan reverse policy differs from request")
        if dict(plan.goal_heading_free) != dict(snapshot.goal_heading_free):
            return _audit_failure(
                "GOAL_HEADING_FREE_MISMATCH",
                "plan heading-free policy differs from request")
        start_headings = _heading_map(plan.vehicle_start_headings)
        if any(
                _angle_error(start_headings[vehicle_id],
                             snapshot.start_headings[vehicle_id]) >
                _GEOMETRY_TOL
                for vehicle_id in snapshot.model.vehicle_ids):
            return _audit_failure(
                "PLAN_START_HEADING_MISMATCH",
                "plan start headings differ from request")
        goal_headings_map = _heading_map(plan.vehicle_goal_headings)
        if any(
                _angle_error(goal_headings_map[vehicle_id],
                             snapshot.goal_headings[vehicle_id]) >
                _GEOMETRY_TOL
                for vehicle_id in snapshot.model.vehicle_ids):
            return _audit_failure(
                "PLAN_GOAL_HEADING_MISMATCH",
                "plan goal headings differ from request")
        model = FormationModel(plan_offsets)
        current_headings = dict(start_headings)
        last_end_headings = None
        for index, block in enumerate(plan.blocks):
            try:
                kinematic = FormationKinematicBlock(
                    block.geometry_type,
                    block.parameter_length,
                    block.control.start_qf,
                    block.control.end_qf,
                    block.control.pivot_x,
                    block.control.pivot_y,
                    plan_offsets)
            except FormationValidationError as exc:
                return _audit_failure(
                    "INVALID_BLOCK_GEOMETRY",
                    "block={} {}".format(index, exc))
            if set(block.vehicle_ids) != set(model.vehicle_ids):
                return _audit_failure(
                    "BLOCK_VEHICLE_SET_MISMATCH", "block={}".format(index))
            is_last = index == len(plan.blocks) - 1
            expected_end = (
                snapshot.goal_headings if is_last else None)
            assignment = FormationModeAssignment(
                success=True,
                reason="OK",
                detail="stored_segments_for_independent_audit",
                block=kinematic,
                start_headings=tuple(sorted(current_headings.items())),
                end_headings=tuple(sorted({
                    vehicle_id: current_headings[vehicle_id] + (
                        kinematic.delta_psi
                        if kinematic.geometry_type == ROTATION else 0.0)
                    for vehicle_id in model.vehicle_ids}.items())),
                segments=block.vehicle_mode_segments,
                rejections=(),
                schedule_index=0,
                schedule_cost=0.0,
                goal_headings_checked=is_last,
            )
            audit = audit_mode_assignment(
                assignment, limits=limits,
                expected_end_headings=expected_end,
                goal_heading_free=(
                    snapshot.goal_heading_free if is_last else None),
                allow_reverse=plan.allow_reverse)
            if not audit.success:
                rejection = FormationPlanRejection(
                    index, None, audit.reason, audit.detail)
                rejections.append(rejection)
                return _audit_failure(
                    audit.reason, "block={} {}".format(index, audit.detail),
                    rejections, checks, len(plan.blocks), sample_count,
                    max_rigidity, max_terminal_xy, max_terminal_yaw,
                    max_heading)
            checks += 1
            if is_last:
                last_end_headings = dict(assignment.end_headings)

            # Independent rigid-center sampling at every block.  Centers are
            # always freshly derived from q_F; no stored center curve exists.
            for sample_index in range(17):
                station = block.parameter_length * sample_index / 16.0
                qf = block.qf_at(station)
                try:
                    centers = model.centers(qf)
                    residual = model.rigidity_residual(centers)
                except FormationValidationError as exc:
                    return _audit_failure(
                        "FORMATION_RIGIDITY_VIOLATION",
                        "block={} station={} {}".format(
                            index, station, exc), rejections, checks,
                        len(plan.blocks), sample_count, max_rigidity,
                        max_terminal_xy, max_terminal_yaw, max_heading)
                max_rigidity = max(max_rigidity, residual)
                sample_count += 1
                if residual > limits.geometry_tolerance_m:
                    return _audit_failure(
                        "FORMATION_RIGIDITY_VIOLATION",
                        "block={} station={} residual={:.12g}".format(
                            index, station, residual), rejections, checks,
                        len(plan.blocks), sample_count, max_rigidity,
                        max_terminal_xy, max_terminal_yaw, max_heading)
            for segment in block.vehicle_mode_segments:
                max_heading = max(max_heading, _angle_error(
                    block.body_heading_at(segment.vehicle_id, 0.0),
                    current_headings[segment.vehicle_id]))
                max_heading = max(max_heading, _angle_error(
                    block.body_heading_at(
                        segment.vehicle_id, block.parameter_length),
                    segment.theta_end))
                checks += 1
            if any(
                    _angle_error(
                        block.body_heading_at(vehicle_id, 0.0),
                        current_headings[vehicle_id]) >
                    limits.heading_tolerance_rad
                    for vehicle_id in model.vehicle_ids):
                return _audit_failure(
                    "BLOCK_HEADING_BOUNDARY_MISMATCH",
                    "block={} stored theta_start is inconsistent".format(
                        index), rejections, checks, len(plan.blocks),
                    sample_count, max_rigidity, max_terminal_xy,
                    max_terminal_yaw, max_heading)
            current_headings = dict(assignment.end_headings)

        if any(
                _angle_error(last_end_headings[vehicle_id],
                             snapshot.goal_headings[vehicle_id]) >
                limits.heading_tolerance_rad
                for vehicle_id in model.vehicle_ids
                if not snapshot.goal_heading_free[vehicle_id]):
            return _audit_failure(
                "GOAL_HEADING_MISMATCH",
                "plan terminal headings differ from the explicit goal",
                rejections, checks, len(plan.blocks), sample_count,
                max_rigidity, max_terminal_xy, max_terminal_yaw, max_heading)
        endpoint = plan.blocks[-1].control.qf_at(
            plan.blocks[-1].control.parameter_length)
        terminal_xy = math.hypot(
            endpoint.x - snapshot.goal.x,
            endpoint.y - snapshot.goal.y)
        terminal_yaw = _angle_error(endpoint.psi, snapshot.goal.psi)
        max_terminal_xy = max(max_terminal_xy, terminal_xy)
        max_terminal_yaw = max(max_terminal_yaw, terminal_yaw)
        if terminal_xy > terminal_tolerance_m or (
                terminal_yaw > terminal_tolerance_rad):
            return _audit_failure(
                "TERMINAL_ENDPOINT_MISMATCH",
                "terminal xy={:.12g} yaw={:.12g}".format(
                    terminal_xy, terminal_yaw), rejections, checks,
                len(plan.blocks), sample_count, max_rigidity,
                max_terminal_xy, max_terminal_yaw, max_heading)

        collision_prechecked = False
        collision_success = False
        collision_reason = ""
        collision_detail = ""
        if grid is not None:
            collision_prechecked = True
            precheck = precheck_plan_collision(
                plan, snapshot, grid, margin_m, sweep_policy)
            collision_success = precheck.success
            collision_reason = precheck.report.reason
            collision_detail = precheck.report.detail
            sample_count += precheck.samples
            if not precheck.success:
                qf_text = ""
                if precheck.report.qf is not None:
                    qf_text = " qf=({:.6g},{:.6g},{:.6g})".format(
                        precheck.report.qf.x, precheck.report.qf.y,
                        precheck.report.qf.psi)
                detail = "block={} vehicle={} {} {}{}".format(
                    precheck.failing_block_index,
                    precheck.report.vehicle_id,
                    precheck.report.reason,
                    precheck.report.detail or "", qf_text)
                collision_detail = detail
                rejection = FormationPlanRejection(
                    precheck.failing_block_index,
                    precheck.report.vehicle_id,
                    "COLLISION_PRECHECK_FAILED", detail)
                rejections.append(rejection)
                return _audit_failure(
                    "COLLISION_PRECHECK_FAILED",
                    "swept precheck failed: " + detail,
                    rejections, checks, len(plan.blocks), sample_count,
                    max_rigidity, max_terminal_xy, max_terminal_yaw,
                    max_heading, collision_prechecked, collision_success,
                    collision_reason, collision_detail)
        return FormationPlanAudit(
            success=True,
            reason="OK",
            detail="KINEMATIC_ONLY_UNCERTIFIED_SAFETY",
            certification=KINEMATIC_VALID,
            checks=checks,
            block_count=len(plan.blocks),
            sample_count=sample_count,
            max_rigidity_error_m=max_rigidity,
            max_terminal_error_xy_m=max_terminal_xy,
            max_terminal_error_yaw_rad=max_terminal_yaw,
            max_heading_error_rad=max_heading,
            collision_prechecked=collision_prechecked,
            collision_success=collision_success,
            collision_reason=collision_reason,
            collision_detail=collision_detail,
            rejections=tuple(rejections),
        )
    except FormationValidationError as exc:
        return _audit_failure(
            "INVALID_CONTINUOUS_PLAN", str(exc), tuple(rejections), checks,
            len(plan.blocks), sample_count, max_rigidity, max_terminal_xy,
            max_terminal_yaw, max_heading)


def plan_candidate_score(plan):
    """Deterministic ranking that never substitutes for an independent audit."""
    mode_cost = 0.0
    previous = None
    for block in plan.blocks:
        for segment in block.vehicle_mode_segments:
            mode_cost += 0.01 * abs(segment.control)
            if segment.mode == OBLIQUE:
                mode_cost += 0.1
            current = (segment.mode, segment.direction,
                       round(segment.control, 12))
            if previous is not None and previous[segment.vehicle_id] != current:
                mode_cost += 0.25
        previous = {
            segment.vehicle_id: (segment.mode, segment.direction,
                                 round(segment.control, 12))
            for segment in block.vehicle_mode_segments}
    return plan.total_parameter_length + mode_cost