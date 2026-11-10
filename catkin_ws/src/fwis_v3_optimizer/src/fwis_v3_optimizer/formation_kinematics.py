"""S5 per-vehicle A/O/S assignment for a rigid formation block.

A formation block is parameterised by a local progress ``lambda``.  For a
translation block ``lambda`` is the formation-frame translation arc length;
for a pure rotation block it is the absolute formation yaw increment.  A
per-vehicle mode segment therefore stores both:

* ``start_station``/``end_station`` in the block-local ``lambda`` domain; and
* ``travel_distance``, the vehicle-center arc length used by the A/O mode
  equations.  For an A rotation this is ``radius * lambda`` and for S it is
  exactly zero.

The module is deliberately ROS-free and does not modify the single-vehicle
``ContinuousPlan``.  Assignments are candidates: a separate
``audit_mode_assignment`` function recomputes every segment from the rigid
formation geometry and body heading, without trusting the stored controls.
No function here certifies collision safety or an executable trajectory.
"""
from dataclasses import dataclass
import itertools
import math
from typing import Mapping, Optional, Tuple

from .formation import (
    FormationModel,
    FormationPose,
    FormationValidationError,
    VehicleOffset,
)


ACKERMANN = 0
OBLIQUE = 1
SPIN = 2
MODE_NAMES = {ACKERMANN: "A", OBLIQUE: "O", SPIN: "S"}

TRANSLATION = "translation"
ROTATION = "rotation"
COMPOSITE = "composite"

# A mode candidate is a kinematic candidate, never an execution certificate.
CERTIFICATION_CANDIDATE = "KINEMATIC_CANDIDATE"
CERTIFICATION_AUDITED = "KINEMATIC_AUDITED"

_GEOMETRY_TOL = 1e-7


def _finite(value, context, positive=False):
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
    return result


def _integer(value, context, allowed=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise FormationValidationError(
            "INVALID_INTEGER context={} value={!r}".format(context, value))
    if allowed is not None and value not in allowed:
        raise FormationValidationError(
            "INVALID_INTEGER context={} value={!r} allowed={}".format(
                context, value, tuple(allowed)))
    return value


def wrap_angle(angle):
    """Return the equivalent angle in ``(-pi, pi]``."""
    value = _finite(angle, "wrap_angle")
    return math.atan2(math.sin(value), math.cos(value))


def _angle_error(first, second):
    return abs(wrap_angle(first - second))


def _angle_near(first, second, tolerance):
    return _angle_error(first, second) <= tolerance


def _mapping_to_headings(values, vehicle_ids, context):
    if not isinstance(values, Mapping):
        raise FormationValidationError(
            "INVALID_BODY_HEADINGS context={} expected a vehicle_id mapping".format(
                context))
    result = {}
    expected = set(vehicle_ids)
    for vehicle_id, theta in values.items():
        checked_id = _integer(vehicle_id, context + ".vehicle_id")
        if checked_id not in expected:
            raise FormationValidationError(
                "UNKNOWN_VEHICLE_ID context={} vehicle_id={}".format(
                    context, checked_id))
        if checked_id in result:
            raise FormationValidationError(
                "DUPLICATE_VEHICLE_ID context={} vehicle_id={}".format(
                    context, checked_id))
        result[checked_id] = _finite(
            theta, "{} vehicle_id={}".format(context, checked_id))
    missing = sorted(expected - set(result))
    if missing:
        raise FormationValidationError(
            "MISSING_VEHICLE_HEADING context={} missing={}".format(
                context, missing))
    return result


def _mapping_to_flags(values, vehicle_ids, context):
    if values is None:
        return {vehicle_id: False for vehicle_id in vehicle_ids}
    if not isinstance(values, Mapping):
        raise FormationValidationError(
            "INVALID_HEADING_FREE context={} expected a vehicle_id mapping".format(
                context))
    result = {vehicle_id: False for vehicle_id in vehicle_ids}
    expected = set(vehicle_ids)
    for vehicle_id, flag in values.items():
        checked_id = _integer(vehicle_id, context + ".vehicle_id")
        if checked_id not in expected:
            raise FormationValidationError(
                "UNKNOWN_VEHICLE_ID context={} vehicle_id={}".format(
                    context, checked_id))
        if not isinstance(flag, bool):
            raise FormationValidationError(
                "INVALID_HEADING_FREE context={} vehicle_id={} expected bool".format(
                    context, checked_id))
        result[checked_id] = flag
    return result


@dataclass(frozen=True)
class KinematicLimits:
    """Vehicle-independent limits used by the S5 assignment/audit.

    The values mirror ``retiming.MotionLimits`` at the time S5 was written.
    S5 does not import the retimer, so a later S8 layer must pass its own
    limits explicitly rather than assuming these values certify dynamics.
    """

    r_min_ackermann: float = 0.810
    oblique_angle_max: float = math.pi / 2.0
    heading_tolerance_rad: float = 1e-6
    geometry_tolerance_m: float = 1e-7
    pivot_tolerance_m: float = 1e-9

    def __post_init__(self):
        object.__setattr__(self, "r_min_ackermann", _finite(
            self.r_min_ackermann, "KinematicLimits.r_min_ackermann", positive=True))
        object.__setattr__(self, "oblique_angle_max", _finite(
            self.oblique_angle_max, "KinematicLimits.oblique_angle_max",
            positive=True))
        if self.oblique_angle_max > math.pi:
            raise FormationValidationError(
                "INVALID_KINEMATIC_LIMIT oblique_angle_max must be <= pi")
        for name in (
                "heading_tolerance_rad", "geometry_tolerance_m",
                "pivot_tolerance_m"):
            value = _finite(getattr(self, name), "KinematicLimits." + name)
            if value < 0.0:
                raise FormationValidationError(
                    "INVALID_KINEMATIC_LIMIT {} must be >= 0".format(name))
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class VehicleModeSegment:
    """One vehicle's constant-control A/O/S segment in a formation block."""

    vehicle_id: int
    start_station: float
    end_station: float
    mode: int
    direction: int
    theta_start: float
    theta_end: float
    control: float
    travel_distance: float
    start_center_x: float
    start_center_y: float
    end_center_x: float
    end_center_y: float
    pivot_x: float
    pivot_y: float

    def __post_init__(self):
        _integer(self.vehicle_id, "VehicleModeSegment.vehicle_id")
        _finite(self.start_station, "VehicleModeSegment.start_station")
        _finite(self.end_station, "VehicleModeSegment.end_station")
        if self.end_station < self.start_station:
            raise FormationValidationError(
                "INVALID_MODE_SEGMENT station range is reversed")
        _integer(self.mode, "VehicleModeSegment.mode",
                 allowed=(ACKERMANN, OBLIQUE, SPIN))
        _integer(self.direction, "VehicleModeSegment.direction",
                 allowed=(-1, 1))
        for name in (
                "theta_start", "theta_end", "control", "travel_distance",
                "start_center_x", "start_center_y",
                "end_center_x", "end_center_y", "pivot_x", "pivot_y"):
            object.__setattr__(
                self, name, _finite(
                    getattr(self, name), "VehicleModeSegment." + name))
        if self.travel_distance < 0.0:
            raise FormationValidationError(
                "INVALID_MODE_SEGMENT travel_distance must be >= 0")
        if self.mode == SPIN and self.travel_distance != 0.0:
            raise FormationValidationError(
                "INVALID_MODE_SEGMENT a spin segment cannot have travel")


@dataclass(frozen=True)
class VehicleModeRejection:
    vehicle_id: Optional[int]
    reason: str
    detail: str

    def __post_init__(self):
        if self.vehicle_id is not None:
            _integer(self.vehicle_id, "VehicleModeRejection.vehicle_id")
        if not isinstance(self.reason, str) or not self.reason:
            raise FormationValidationError(
                "INVALID_REJECTION reason must be a non-empty string")
        if not isinstance(self.detail, str):
            raise FormationValidationError(
                "INVALID_REJECTION detail must be a string")


@dataclass(frozen=True)
class FormationKinematicBlock:
    """Geometry inputs for one local-``lambda`` rigid formation block.

    ``offsets`` are the fixed vehicle-center offsets in the formation frame.
    ``pivot_x``/``pivot_y`` are also expressed in the formation frame.
    """

    geometry_type: str
    parameter_length: float
    start_qf: FormationPose
    end_qf: FormationPose
    pivot_x: float
    pivot_y: float
    offsets: Tuple[VehicleOffset, ...]

    def __post_init__(self):
        if self.geometry_type not in (TRANSLATION, ROTATION, COMPOSITE):
            raise FormationValidationError(
                "INVALID_GEOMETRY_TYPE {}".format(self.geometry_type))
        object.__setattr__(self, "parameter_length", _finite(
            self.parameter_length, "FormationKinematicBlock.parameter_length",
            positive=True))
        if not isinstance(self.start_qf, FormationPose):
            raise FormationValidationError(
                "INVALID_FORMATION_POSE start_qf")
        if not isinstance(self.end_qf, FormationPose):
            raise FormationValidationError(
                "INVALID_FORMATION_POSE end_qf")
        object.__setattr__(self, "pivot_x", _finite(
            self.pivot_x, "FormationKinematicBlock.pivot_x"))
        object.__setattr__(self, "pivot_y", _finite(
            self.pivot_y, "FormationKinematicBlock.pivot_y"))
        try:
            supplied = tuple(self.offsets)
        except TypeError as exc:
            raise FormationValidationError(
                "INVALID_OFFSETS expected VehicleOffset tuple") from exc
        FormationModel(supplied)  # validates unique IDs and coincident offsets
        offsets = tuple(sorted(supplied, key=lambda item: item.vehicle_id))
        if tuple(self.offsets) != offsets:
            raise FormationValidationError(
                "UNSORTED_OFFSETS offsets must be ordered by vehicle_id")
        object.__setattr__(self, "offsets", offsets)

        if self.geometry_type == COMPOSITE:
            return
        if self.geometry_type == TRANSLATION:
            if not _angle_near(
                    self.start_qf.psi, self.end_qf.psi,
                    self.geometry_tolerance):
                raise FormationValidationError(
                    "INVALID_TRANSLATION_BLOCK psi_F must remain constant")
            world = math.hypot(
                self.end_qf.x - self.start_qf.x,
                self.end_qf.y - self.start_qf.y)
            if abs(world - self.parameter_length) > (
                    self.geometry_tolerance +
                    1e-9 * max(1.0, self.parameter_length)):
                raise FormationValidationError(
                    "INVALID_TRANSLATION_BLOCK parameter_length does not match "
                    "the rigid translation")
            return

        delta_psi = self.end_qf.psi - self.start_qf.psi
        if abs(abs(delta_psi) - self.parameter_length) > (
                self.geometry_tolerance +
                1e-9 * max(1.0, self.parameter_length)):
            raise FormationValidationError(
                "INVALID_ROTATION_BLOCK parameter_length must equal |dpsi_F|")
        cosine = math.cos(self.start_qf.psi)
        sine = math.sin(self.start_qf.psi)
        pivot_world_x = (
            self.start_qf.x + cosine * self.pivot_x -
            sine * self.pivot_y)
        pivot_world_y = (
            self.start_qf.y + sine * self.pivot_x +
            cosine * self.pivot_y)
        next_psi = self.start_qf.psi + delta_psi
        cn, sn = math.cos(next_psi), math.sin(next_psi)
        expected_x = (
            pivot_world_x - cn * self.pivot_x + sn * self.pivot_y)
        expected_y = (
            pivot_world_y - sn * self.pivot_x - cn * self.pivot_y)
        if math.hypot(
                expected_x - self.end_qf.x,
                expected_y - self.end_qf.y) > self.geometry_tolerance:
            raise FormationValidationError(
                "INVALID_ROTATION_BLOCK end_qf is not the explicit-pivot "
                "rigid rotation of start_qf")

    @property
    def geometry_tolerance(self):
        return _GEOMETRY_TOL

    @property
    def vehicle_ids(self):
        return tuple(offset.vehicle_id for offset in self.offsets)

    @property
    def delta_psi(self):
        return self.end_qf.psi - self.start_qf.psi

    def offset_map(self):
        return {offset.vehicle_id: (offset.x, offset.y)
                for offset in self.offsets}

    def centers(self, qf):
        model = FormationModel(self.offsets)
        return model.centers(qf)

    def qf_at(self, station):
        """Recompute ``q_F(lambda)`` from the stored rigid block definition."""
        station = _finite(station, "FormationKinematicBlock.qf_at")
        if station < -self.geometry_tolerance or station > (
                self.parameter_length + self.geometry_tolerance):
            raise FormationValidationError(
                "INVALID_BLOCK_STATION {} outside [0, {}]".format(
                    station, self.parameter_length))
        fraction = 0.0 if self.parameter_length == 0.0 else (
            station / self.parameter_length)
        if self.geometry_type == TRANSLATION:
            return FormationPose(
                self.start_qf.x + fraction * (
                    self.end_qf.x - self.start_qf.x),
                self.start_qf.y + fraction * (
                    self.end_qf.y - self.start_qf.y),
                self.start_qf.psi)
        if self.geometry_type == ROTATION:
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
        raise FormationValidationError(
            "COMPOSITE_BLOCK_UNSUPPORTED blocking primitive is not defined")

    @classmethod
    def from_primitive(cls, start_qf, primitive, offsets,
                       geometry_tolerance_m=_GEOMETRY_TOL):
        """Build a block from an S4 primitive-like object (ROS message safe)."""
        if not isinstance(start_qf, FormationPose):
            raise FormationValidationError(
                "INVALID_FORMATION_POSE start_qf")
        geometry_type = getattr(primitive, "geometry_type", None)
        if isinstance(geometry_type, str):
            kind = geometry_type.lower()
        elif geometry_type == 0:
            kind = TRANSLATION
        elif geometry_type == 1:
            kind = ROTATION
        elif geometry_type == 2:
            kind = COMPOSITE
        else:
            raise FormationValidationError(
                "INVALID_GEOMETRY_TYPE {!r}".format(geometry_type))
        tolerance = _finite(
            geometry_tolerance_m, "geometry_tolerance_m", positive=True)
        if kind == TRANSLATION:
            dx = _finite(getattr(primitive, "dx", None), "primitive.dx")
            dy = _finite(getattr(primitive, "dy", None), "primitive.dy")
            length = math.hypot(dx, dy)
            if length <= tolerance:
                raise FormationValidationError(
                    "ZERO_LENGTH_TRANSLATION_BLOCK")
            cosine, sine = math.cos(start_qf.psi), math.sin(start_qf.psi)
            end_qf = FormationPose(
                start_qf.x + cosine * dx - sine * dy,
                start_qf.y + sine * dx + cosine * dy,
                start_qf.psi)
            return cls(kind, length, start_qf, end_qf, 0.0, 0.0, offsets)
        if kind == ROTATION:
            dpsi = _finite(getattr(primitive, "dpsi", None), "primitive.dpsi")
            if abs(dpsi) <= tolerance:
                raise FormationValidationError(
                    "ZERO_ROTATION_BLOCK")
            pivot_x = _finite(getattr(primitive, "pivot_x", None),
                              "primitive.pivot_x")
            pivot_y = _finite(getattr(primitive, "pivot_y", None),
                              "primitive.pivot_y")
            cosine, sine = math.cos(start_qf.psi), math.sin(start_qf.psi)
            pivot_world_x = (
                start_qf.x + cosine * pivot_x - sine * pivot_y)
            pivot_world_y = (
                start_qf.y + sine * pivot_x + cosine * pivot_y)
            next_psi = start_qf.psi + dpsi
            cn, sn = math.cos(next_psi), math.sin(next_psi)
            end_qf = FormationPose(
                pivot_world_x - cn * pivot_x + sn * pivot_y,
                pivot_world_y - sn * pivot_x - cn * pivot_y,
                next_psi)
            return cls(kind, abs(dpsi), start_qf, end_qf,
                       pivot_x, pivot_y, offsets)
        return cls(kind, tolerance, start_qf, start_qf, 0.0, 0.0, offsets)


@dataclass(frozen=True)
class FormationModeAssignment:
    """One joint mode schedule for a single formation block."""

    success: bool
    reason: str
    detail: str
    block: Optional[FormationKinematicBlock]
    start_headings: Tuple[Tuple[int, float], ...]
    end_headings: Tuple[Tuple[int, float], ...]
    segments: Tuple[VehicleModeSegment, ...]
    rejections: Tuple[VehicleModeRejection, ...]
    schedule_index: int
    schedule_cost: float
    goal_headings_checked: bool
    stop_required_at_boundaries: bool = True
    certification: str = CERTIFICATION_CANDIDATE

    @property
    def end_heading_map(self):
        return dict(self.end_headings)

    @property
    def segment_map(self):
        return {segment.vehicle_id: segment for segment in self.segments}

    @property
    def modes(self):
        return tuple((segment.vehicle_id, segment.mode)
                     for segment in self.segments)

    def as_status(self):
        return {
            "schema": "formation_mode_assignment/v1",
            "success": self.success,
            "reason": self.reason,
            "detail": self.detail,
            "certification": self.certification,
            "schedule_index": self.schedule_index,
            "schedule_cost": self.schedule_cost,
            "goal_headings_checked": self.goal_headings_checked,
            "stop_required_at_boundaries": self.stop_required_at_boundaries,
            "geometry_type": (
                None if self.block is None else self.block.geometry_type),
            "segments": [
                {
                    "vehicle_id": segment.vehicle_id,
                    "mode": segment.mode,
                    "direction": segment.direction,
                    "theta_start": segment.theta_start,
                    "theta_end": segment.theta_end,
                    "control": segment.control,
                    "travel_distance": segment.travel_distance,
                }
                for segment in self.segments
            ],
            "rejections": [
                {
                    "vehicle_id": rejection.vehicle_id,
                    "reason": rejection.reason,
                    "detail": rejection.detail,
                }
                for rejection in self.rejections
            ],
            "executable": False,
        }


@dataclass(frozen=True)
class ModeAudit:
    """Independent recomputation result for a saved mode assignment."""

    success: bool
    reason: str
    detail: str
    certification: str
    checks: int
    max_center_error_m: float
    max_heading_error_rad: float
    max_curvature: float
    max_oblique_angle_rad: float
    max_spin_center_motion_m: float
    goal_headings_checked: bool

    def as_status(self):
        return {
            "schema": "formation_mode_audit/v1",
            "success": self.success,
            "reason": self.reason,
            "detail": self.detail,
            "certification": self.certification,
            "checks": self.checks,
            "max_center_error_m": self.max_center_error_m,
            "max_heading_error_rad": self.max_heading_error_rad,
            "max_curvature": self.max_curvature,
            "max_oblique_angle_rad": self.max_oblique_angle_rad,
            "max_spin_center_motion_m": self.max_spin_center_motion_m,
            "goal_headings_checked": self.goal_headings_checked,
            "executable": False,
        }


@dataclass(frozen=True)
class _Choice:
    mode: int
    direction: int
    control: float
    mode_cost: float


def _translation_choices(theta, motion_angle, allow_reverse, limits):
    choices = []
    delta = wrap_angle(motion_angle - theta)
    tolerance = limits.heading_tolerance_rad
    reverse_possible = False

    if abs(delta) <= tolerance:
        choices.append(_Choice(ACKERMANN, 1, 0.0, 0.0))
    elif abs(abs(delta) - math.pi) <= tolerance:
        reverse_possible = True
        if allow_reverse:
            choices.append(_Choice(ACKERMANN, -1, 0.0, 2.0))

    forward_phi = wrap_angle(delta)
    if abs(forward_phi) <= limits.oblique_angle_max + tolerance:
        choices.append(_Choice(
            OBLIQUE, 1, forward_phi, 0.35 + 0.25 * abs(forward_phi)))

    reverse_phi = wrap_angle(delta + math.pi)
    if abs(reverse_phi) <= limits.oblique_angle_max + tolerance:
        reverse_possible = True
        if allow_reverse:
            choices.append(_Choice(
                OBLIQUE, -1, reverse_phi,
                2.35 + 0.25 * abs(reverse_phi)))

    # An exact A alignment is preferred over O; otherwise O is the only
    # admissible constant-heading translation mode.
    choices.sort(key=lambda item: (
        item.mode_cost, item.mode, abs(item.control), item.direction))
    if choices:
        return tuple(choices), None
    if reverse_possible:
        return (), VehicleModeRejection(
            None, "REVERSE_NOT_ALLOWED",
            "the only admissible translation direction is reverse; "
            "request allow_reverse is false")
    return (), VehicleModeRejection(
        None, "OBLIQUE_ANGLE_LIMIT",
        "translation direction cannot be represented within "
        "|phi| <= {:.12g}".format(limits.oblique_angle_max))


def _rotation_choices(offset, pivot_x, pivot_y, theta, dpsi,
                       allow_reverse, limits, start_psi=0.0):
    rx = offset.x - pivot_x
    ry = offset.y - pivot_y
    radius = math.hypot(rx, ry)
    if radius <= limits.pivot_tolerance_m:
        return ((
            _Choice(SPIN, 1 if dpsi >= 0.0 else -1, 0.0, 0.0),
        ), None, radius)
    # Offsets and pivots are expressed in the formation frame; body headings
    # are world headings.  The forward tangent therefore includes psi_F.
    tangent = math.atan2(rx, -ry) + start_psi
    direction = None
    if _angle_near(theta, tangent, limits.heading_tolerance_rad):
        direction = 1
    elif _angle_near(theta, tangent + math.pi,
                     limits.heading_tolerance_rad):
        direction = -1
    if direction is None:
        return (), VehicleModeRejection(
            offset.vehicle_id, "HEADING_BOUNDARY_MISMATCH",
            "vehicle_id={} body heading {:.12g} is neither the forward "
            "tangent {:.12g} nor the reverse tangent {:.12g}".format(
                offset.vehicle_id, theta, tangent,
                wrap_angle(tangent + math.pi))), radius
    if direction == -1 and not allow_reverse:
        return (), VehicleModeRejection(
            offset.vehicle_id, "REVERSE_NOT_ALLOWED",
            "vehicle_id={} A rotation would require reverse travel".format(
                offset.vehicle_id)), radius
    curvature = (1.0 if dpsi >= 0.0 else -1.0) * direction / radius
    if abs(curvature) > 1.0 / limits.r_min_ackermann + (
            limits.heading_tolerance_rad):
        return (), VehicleModeRejection(
            offset.vehicle_id, "CURVATURE_LIMIT",
            "vehicle_id={} A rotation radius {:.12g} m requires "
            "|kappa|={:.12g} > 1/Rmin={:.12g}".format(
                offset.vehicle_id, radius, abs(curvature),
                1.0 / limits.r_min_ackermann)), radius
    return ((
        _Choice(ACKERMANN, direction, curvature, 0.0),
    ), None, radius)


def _build_segment(block, offset, choice, start_theta, end_theta):
    start_centers = block.centers(block.start_qf)
    end_centers = block.centers(block.end_qf)
    if block.geometry_type == TRANSLATION:
        travel_distance = block.parameter_length
    elif choice.mode == SPIN:
        travel_distance = 0.0
    else:
        radius = math.hypot(
            offset.x - block.pivot_x, offset.y - block.pivot_y)
        travel_distance = radius * block.parameter_length
    return VehicleModeSegment(
        vehicle_id=offset.vehicle_id,
        start_station=0.0,
        end_station=block.parameter_length,
        mode=choice.mode,
        direction=choice.direction,
        theta_start=start_theta,
        theta_end=end_theta,
        control=choice.control,
        travel_distance=travel_distance,
        start_center_x=start_centers[offset.vehicle_id][0],
        start_center_y=start_centers[offset.vehicle_id][1],
        end_center_x=end_centers[offset.vehicle_id][0],
        end_center_y=end_centers[offset.vehicle_id][1],
        pivot_x=block.pivot_x,
        pivot_y=block.pivot_y,
    )


def _assignment_failure(block, headings, rejection, reason=None,
                        detail=None, schedule_index=0):
    return FormationModeAssignment(
        success=False,
        reason=reason or rejection.reason,
        detail=detail if detail is not None else rejection.detail,
        block=block,
        start_headings=tuple(sorted(headings.items())),
        end_headings=(),
        segments=(),
        rejections=(rejection,),
        schedule_index=schedule_index,
        schedule_cost=float("inf"),
        goal_headings_checked=False,
    )


def enumerate_mode_assignments(
        block, start_headings, limits=None, allow_reverse=False,
        expected_end_headings=None, goal_heading_free=None,
        max_schedules=4, max_combinations=4096):
    """Return up to ``max_schedules`` best joint A/O/S schedules.

    The Cartesian product is over per-vehicle admissible constant-control
    choices.  A rejected geometry produces one failed assignment explaining
    the first (or per-vehicle) rejection.  Successful entries are still
    candidates and must be passed through :func:`audit_mode_assignment`.
    """
    if not isinstance(block, FormationKinematicBlock):
        raise FormationValidationError(
            "INVALID_FORMATION_BLOCK expected FormationKinematicBlock")
    limits = limits or KinematicLimits()
    if not isinstance(limits, KinematicLimits):
        raise FormationValidationError(
            "INVALID_KINEMATIC_LIMITS expected KinematicLimits")
    if isinstance(max_schedules, bool) or not isinstance(max_schedules, int):
        raise FormationValidationError(
            "INVALID_MAX_SCHEDULES expected a positive integer")
    if max_schedules <= 0:
        raise FormationValidationError(
            "INVALID_MAX_SCHEDULES expected a positive integer")
    if isinstance(max_combinations, bool) or not isinstance(
            max_combinations, int) or max_combinations <= 0:
        raise FormationValidationError(
            "INVALID_MAX_COMBINATIONS expected a positive integer")
    allow_reverse = bool(allow_reverse)

    headings = _mapping_to_headings(
        start_headings, block.vehicle_ids, "start_headings")
    if expected_end_headings is None:
        expected = None
        free_flags = {vehicle_id: False
                      for vehicle_id in block.vehicle_ids}
    else:
        expected = _mapping_to_headings(
            expected_end_headings, block.vehicle_ids,
            "expected_end_headings")
        free_flags = _mapping_to_flags(
            goal_heading_free, block.vehicle_ids, "goal_heading_free")

    if block.geometry_type == COMPOSITE:
        rejection = VehicleModeRejection(
            None, "COMPOSITE_BLOCK_UNSUPPORTED",
            "composite formation blocks are not defined in S5")
        return (_assignment_failure(block, headings, rejection),)
    if block.geometry_type not in (TRANSLATION, ROTATION):
        rejection = VehicleModeRejection(
            None, "UNSUPPORTED_GEOMETRY_TYPE", block.geometry_type)
        return (_assignment_failure(block, headings, rejection),)

    if block.geometry_type == TRANSLATION:
        local_dx = math.cos(block.start_qf.psi) * (
            block.end_qf.x - block.start_qf.x) + math.sin(
            block.start_qf.psi) * (
            block.end_qf.y - block.start_qf.y)
        local_dy = -math.sin(block.start_qf.psi) * (
            block.end_qf.x - block.start_qf.x) + math.cos(
            block.start_qf.psi) * (
            block.end_qf.y - block.start_qf.y)
        # local_dx/local_dy are in the formation frame while theta_i is a
        # world heading; rotate the local motion direction back to world.
        motion_angle = block.start_qf.psi + math.atan2(local_dy, local_dx)
    else:
        motion_angle = None
        dpsi = block.delta_psi

    per_vehicle = []
    rejections = []
    for offset in block.offsets:
        theta = headings[offset.vehicle_id]
        if block.geometry_type == TRANSLATION:
            choices, rejection = _translation_choices(
                theta, motion_angle, allow_reverse, limits)
            radius = None
        else:
            choices, rejection, radius = _rotation_choices(
                offset, block.pivot_x, block.pivot_y, theta, dpsi,
                allow_reverse, limits, block.start_qf.psi)
        if rejection is not None:
            if rejection.vehicle_id is None:
                rejection = VehicleModeRejection(
                    offset.vehicle_id, rejection.reason,
                    "vehicle_id={} {}".format(
                        offset.vehicle_id, rejection.detail))
            rejections.append(rejection)
            choices = ()
        per_vehicle.append((offset, theta, choices, radius))

    if rejections:
        first = rejections[0]
        return (_assignment_failure(
            block, headings, first, detail=first.detail),)

    combination_count = 1
    for _, _, choices, _ in per_vehicle:
        combination_count *= len(choices)
    if combination_count > max_combinations:
        rejection = VehicleModeRejection(
            None, "MODE_ENUMERATION_BUDGET",
            "{} combinations exceed the cap {}".format(
                combination_count, max_combinations))
        return (_assignment_failure(block, headings, rejection),)

    schedules = []
    for combination in itertools.product(
            *(choices for _, _, choices, _ in per_vehicle)):
        segments = []
        schedule_cost = 0.0
        end_headings = {}
        for (offset, theta, _, _), choice in zip(per_vehicle, combination):
            if block.geometry_type == ROTATION:
                end_theta = theta + block.delta_psi
            else:
                end_theta = theta
            segments.append(_build_segment(
                block, offset, choice, theta, end_theta))
            end_headings[offset.vehicle_id] = end_theta
            schedule_cost += choice.mode_cost

        expected_checked = False
        if expected is not None:
            mismatch = None
            for vehicle_id, end_theta in end_headings.items():
                if free_flags[vehicle_id]:
                    continue
                expected_checked = True
                if not _angle_near(
                        end_theta, expected[vehicle_id],
                        limits.heading_tolerance_rad):
                    mismatch = VehicleModeRejection(
                        vehicle_id, "GOAL_HEADING_MISMATCH",
                        "assigned theta_end {:.12g} differs from the explicit "
                        "goal {:.12g} by {:.12g} rad".format(
                            end_theta, expected[vehicle_id],
                            _angle_error(end_theta, expected[vehicle_id])))
                    break
            if mismatch is not None:
                continue

        key = tuple(
            (segment.vehicle_id, segment.mode, segment.direction,
             round(segment.control, 12),
             round(segment.theta_start, 12),
             round(segment.theta_end, 12))
            for segment in segments)
        schedules.append((
            schedule_cost, key, tuple(segments), tuple(
                sorted(end_headings.items())), expected_checked))

    if not schedules:
        # Every combination was rejected by the explicit goal heading check.
        first_vehicle = block.vehicle_ids[0]
        rejection = VehicleModeRejection(
            first_vehicle, "GOAL_HEADING_MISMATCH",
            "no admissible mode schedule reaches the explicit goal headings")
        return (_assignment_failure(block, headings, rejection),)

    schedules.sort(key=lambda item: (item[0], item[1]))
    unique = []
    seen = set()
    for item in schedules:
        if item[1] in seen:
            continue
        seen.add(item[1])
        unique.append(item)
        if len(unique) >= max_schedules:
            break

    assignments = []
    for index, (cost, _, segments, ends, checked) in enumerate(unique):
        assignments.append(FormationModeAssignment(
            success=True,
            reason="OK",
            detail="KINEMATIC_CANDIDATE_NOT_AUDITED",
            block=block,
            start_headings=tuple(sorted(headings.items())),
            end_headings=ends,
            segments=segments,
            rejections=(),
            schedule_index=index,
            schedule_cost=cost,
            goal_headings_checked=checked,
        ))
    return tuple(assignments)


def assign_formation_modes(
        block, start_headings, limits=None, allow_reverse=False,
        expected_end_headings=None, goal_heading_free=None,
        max_schedules=1):
    """Return the cheapest joint schedule (a candidate, not an audit)."""
    assignments = enumerate_mode_assignments(
        block, start_headings, limits=limits, allow_reverse=allow_reverse,
        expected_end_headings=expected_end_headings,
        goal_heading_free=goal_heading_free,
        max_schedules=max_schedules)
    return assignments[0]


def _audit_failure(reason, detail, checks=0, goal_checked=False):
    return ModeAudit(
        success=False,
        reason=reason,
        detail=detail,
        certification="NONE",
        checks=checks,
        max_center_error_m=0.0,
        max_heading_error_rad=0.0,
        max_curvature=0.0,
        max_oblique_angle_rad=0.0,
        max_spin_center_motion_m=0.0,
        goal_headings_checked=goal_checked,
    )


def audit_mode_assignment(
        assignment, limits=None, expected_end_headings=None,
        goal_heading_free=None, allow_reverse=False):
    """Recompute and independently audit a saved mode assignment.

    Stored segment centers, headings, travel lengths and controls are not
    trusted.  Every value is recomputed from the block's rigid geometry,
    fixed offsets, body headings and declared mode.  Passing this audit still
    does not certify collision avoidance, timing, actuator limits or an
    executable trajectory.
    """
    if not isinstance(assignment, FormationModeAssignment):
        raise FormationValidationError(
            "INVALID_MODE_ASSIGNMENT expected FormationModeAssignment")
    limits = limits or KinematicLimits()
    if not isinstance(limits, KinematicLimits):
        raise FormationValidationError(
            "INVALID_KINEMATIC_LIMITS expected KinematicLimits")
    if not assignment.success:
        return _audit_failure(
            assignment.reason, assignment.detail,
            goal_checked=assignment.goal_headings_checked)

    block = assignment.block
    if not isinstance(block, FormationKinematicBlock):
        return _audit_failure(
            "INVALID_MODE_ASSIGNMENT", "missing formation block")
    if block.geometry_type == COMPOSITE:
        return _audit_failure(
            "COMPOSITE_BLOCK_UNSUPPORTED",
            "composite blocks have no S5 mode equations")
    try:
        headings = _mapping_to_headings(
            dict(assignment.start_headings), block.vehicle_ids,
            "assignment.start_headings")
        segments = tuple(assignment.segments)
        segment_map = {}
        for segment in segments:
            if not isinstance(segment, VehicleModeSegment):
                return _audit_failure(
                    "INVALID_MODE_SEGMENT",
                    "assignment contains a non-VehicleModeSegment")
            if segment.vehicle_id in segment_map:
                return _audit_failure(
                    "DUPLICATE_MODE_SEGMENT",
                    "vehicle_id={}".format(segment.vehicle_id))
            segment_map[segment.vehicle_id] = segment
        if set(segment_map) != set(block.vehicle_ids):
            return _audit_failure(
                "MODE_SEGMENT_SET_MISMATCH",
                "segments={} vehicles={}".format(
                    sorted(segment_map), list(block.vehicle_ids)))
        start_centers = block.centers(block.start_qf)
        end_centers = block.centers(block.end_qf)
        start_qf = block.qf_at(0.0)
        end_qf = block.qf_at(block.parameter_length)
        if math.hypot(
                start_qf.x - block.start_qf.x,
                start_qf.y - block.start_qf.y) > limits.geometry_tolerance_m:
            return _audit_failure(
                "BLOCK_ENDPOINT_MISMATCH", "qf(0) does not equal start_qf")
        if math.hypot(
                end_qf.x - block.end_qf.x,
                end_qf.y - block.end_qf.y) > limits.geometry_tolerance_m:
            return _audit_failure(
                "BLOCK_ENDPOINT_MISMATCH", "qf(L) does not equal end_qf")

        if expected_end_headings is not None:
            expected = _mapping_to_headings(
                expected_end_headings, block.vehicle_ids,
                "expected_end_headings")
            free_flags = _mapping_to_flags(
                goal_heading_free, block.vehicle_ids, "goal_heading_free")
        else:
            expected = None
            free_flags = None

        checks = 0
        max_center_error = 0.0
        max_heading_error = 0.0
        max_curvature = 0.0
        max_oblique = 0.0
        max_spin_motion = 0.0
        goal_checked = False

        for offset in block.offsets:
            vehicle_id = offset.vehicle_id
            segment = segment_map[vehicle_id]
            theta_start = headings[vehicle_id]
            expected_start = start_centers[vehicle_id]
            expected_end = end_centers[vehicle_id]

            center_error = max(
                math.hypot(
                    segment.start_center_x - expected_start[0],
                    segment.start_center_y - expected_start[1]),
                math.hypot(
                    segment.end_center_x - expected_end[0],
                    segment.end_center_y - expected_end[1]))
            max_center_error = max(max_center_error, center_error)
            if center_error > limits.geometry_tolerance_m:
                return _audit_failure(
                    "CONSTRUCTION_MISMATCH",
                    "vehicle_id={} stored center differs by {:.12g} m".format(
                        vehicle_id, center_error), checks, goal_checked)

            heading_error = _angle_error(segment.theta_start, theta_start)
            expected_end_theta = (
                theta_start + block.delta_psi
                if block.geometry_type == ROTATION else theta_start)
            heading_error = max(
                heading_error,
                _angle_error(segment.theta_end, expected_end_theta))
            max_heading_error = max(max_heading_error, heading_error)
            if heading_error > limits.heading_tolerance_rad:
                return _audit_failure(
                    "HEADING_CONTINUITY_VIOLATION",
                    "vehicle_id={} heading error {:.12g} rad".format(
                        vehicle_id, heading_error), checks, goal_checked)

            if (segment.start_station != 0.0 or
                    abs(segment.end_station - block.parameter_length) >
                    limits.geometry_tolerance_m):
                return _audit_failure(
                    "MODE_SEGMENT_STATION_MISMATCH",
                    "vehicle_id={} segment does not span the block".format(
                        vehicle_id), checks, goal_checked)

            if (segment.mode in (ACKERMANN, OBLIQUE) and
                    segment.direction == -1 and not allow_reverse):
                return _audit_failure(
                    "REVERSE_NOT_ALLOWED",
                    "vehicle_id={} mode={} uses reverse travel while "
                    "allow_reverse is false".format(
                        vehicle_id, MODE_NAMES[segment.mode]), checks,
                    goal_checked)

            if block.geometry_type == TRANSLATION:
                if segment.mode == SPIN:
                    return _audit_failure(
                        "NONSTATIONARY_SPIN",
                        "vehicle_id={} spin in a non-zero translation block"
                        .format(vehicle_id), checks, goal_checked)
                if segment.travel_distance != block.parameter_length:
                    return _audit_failure(
                        "MODE_TRAVEL_MISMATCH",
                        "vehicle_id={} translation travel mismatch".format(
                            vehicle_id), checks, goal_checked)
                if segment.direction not in (-1, 1):
                    return _audit_failure(
                        "INVALID_MODE_DIRECTION",
                        "vehicle_id={} direction={}".format(
                            vehicle_id, segment.direction), checks,
                        goal_checked)
                if segment.mode == ACKERMANN:
                    curvature = segment.control
                    max_curvature = max(max_curvature, abs(curvature))
                    if abs(curvature) > 1e-12:
                        return _audit_failure(
                            "ACKERMANN_TRANSLATION_CURVATURE",
                            "vehicle_id={} straight translation has kappa="
                            "{:.12g}".format(vehicle_id, curvature), checks,
                            goal_checked)
                    if abs(curvature) > 1.0 / limits.r_min_ackermann + (
                            limits.heading_tolerance_rad):
                        return _audit_failure(
                            "CURVATURE_LIMIT",
                            "vehicle_id={} |kappa| exceeds 1/Rmin".format(
                                vehicle_id), checks, goal_checked)
                    angle = segment.theta_start
                    expected_dx = (
                        segment.travel_distance * segment.direction *
                        math.cos(angle))
                    expected_dy = (
                        segment.travel_distance * segment.direction *
                        math.sin(angle))
                elif segment.mode == OBLIQUE:
                    oblique = abs(segment.control)
                    max_oblique = max(max_oblique, oblique)
                    if oblique > limits.oblique_angle_max + (
                            limits.heading_tolerance_rad):
                        return _audit_failure(
                            "OBLIQUE_ANGLE_LIMIT",
                            "vehicle_id={} |phi|={:.12g} exceeds limit".format(
                                vehicle_id, oblique), checks, goal_checked)
                    angle = segment.theta_start + segment.control
                    expected_dx = (
                        segment.travel_distance * segment.direction *
                        math.cos(angle))
                    expected_dy = (
                        segment.travel_distance * segment.direction *
                        math.sin(angle))
                else:
                    return _audit_failure(
                        "UNSUPPORTED_MODE",
                        "vehicle_id={} mode={}".format(
                            vehicle_id, segment.mode), checks, goal_checked)
                displacement_error = math.hypot(
                    segment.end_center_x - segment.start_center_x - expected_dx,
                    segment.end_center_y - segment.start_center_y - expected_dy)
                max_center_error = max(max_center_error, displacement_error)
                if displacement_error > limits.geometry_tolerance_m:
                    return _audit_failure(
                        "MODE_DISPLACEMENT_MISMATCH",
                        "vehicle_id={} translation displacement error "
                        "{:.12g} m".format(vehicle_id, displacement_error),
                        checks, goal_checked)
            else:
                rx = offset.x - block.pivot_x
                ry = offset.y - block.pivot_y
                radius = math.hypot(rx, ry)
                dpsi = block.delta_psi
                if abs(dpsi) <= limits.heading_tolerance_rad:
                    return _audit_failure(
                        "ZERO_ROTATION_BLOCK",
                        "rotation block has no progress", checks, goal_checked)
                if segment.mode == SPIN:
                    spin_motion = math.hypot(
                        segment.end_center_x - segment.start_center_x,
                        segment.end_center_y - segment.start_center_y)
                    max_spin_motion = max(max_spin_motion, spin_motion)
                    if radius > limits.pivot_tolerance_m:
                        return _audit_failure(
                            "NON_PIVOT_SPIN",
                            "vehicle_id={} has radius {:.12g} m but was "
                            "assigned S in a ROTATE block".format(
                                vehicle_id, radius), checks, goal_checked)
                    if spin_motion > limits.geometry_tolerance_m:
                        return _audit_failure(
                            "SPIN_CENTER_MOTION",
                            "vehicle_id={} spin center moved {:.12g} m".format(
                                vehicle_id, spin_motion), checks, goal_checked)
                    if abs(segment.control) > 1e-12:
                        return _audit_failure(
                            "SPIN_CONTROL_NONZERO",
                            "vehicle_id={} spin control must be zero".format(
                                vehicle_id), checks, goal_checked)
                    expected_direction = 1 if dpsi > 0.0 else -1
                    if segment.direction != expected_direction:
                        return _audit_failure(
                            "SPIN_DIRECTION_MISMATCH",
                            "vehicle_id={} direction={} sign(dpsi)={}".format(
                                vehicle_id, segment.direction,
                                expected_direction), checks, goal_checked)
                    if segment.travel_distance != 0.0:
                        return _audit_failure(
                            "SPIN_TRAVEL_NONZERO",
                            "vehicle_id={} spin travel={}".format(
                                vehicle_id, segment.travel_distance), checks,
                            goal_checked)
                elif segment.mode == ACKERMANN:
                    if radius <= limits.pivot_tolerance_m:
                        return _audit_failure(
                            "PIVOT_VEHICLE_NON_SPIN",
                            "vehicle_id={} lies on the pivot but was assigned "
                            "A".format(vehicle_id), checks, goal_checked)
                    tangent = math.atan2(rx, -ry) + block.start_qf.psi
                    if segment.direction == 1:
                        tangent_expected = tangent
                    elif segment.direction == -1:
                        tangent_expected = tangent + math.pi
                    else:
                        return _audit_failure(
                            "INVALID_MODE_DIRECTION",
                            "vehicle_id={} direction={}".format(
                                vehicle_id, segment.direction), checks,
                            goal_checked)
                    tangent_error = _angle_error(
                        theta_start, tangent_expected)
                    if tangent_error > limits.heading_tolerance_rad:
                        return _audit_failure(
                            "HEADING_BOUNDARY_MISMATCH",
                            "vehicle_id={} heading is not aligned with the "
                            "rotation tangent".format(vehicle_id), checks,
                            goal_checked)
                    expected_travel = radius * block.parameter_length
                    if abs(segment.travel_distance - expected_travel) > (
                            limits.geometry_tolerance_m +
                            1e-9 * max(1.0, expected_travel)):
                        return _audit_failure(
                            "MODE_TRAVEL_MISMATCH",
                            "vehicle_id={} A rotation travel {:.12g} != "
                            "{:.12g}".format(
                                vehicle_id, segment.travel_distance,
                                expected_travel), checks, goal_checked)
                    expected_curvature = (
                        (1.0 if dpsi > 0.0 else -1.0) *
                        segment.direction / radius)
                    max_curvature = max(
                        max_curvature, abs(segment.control))
                    curvature_error = abs(
                        segment.control - expected_curvature)
                    if curvature_error > (
                            limits.heading_tolerance_rad +
                            1e-9 * max(1.0, abs(expected_curvature))):
                        return _audit_failure(
                            "MODE_CURVATURE_MISMATCH",
                            "vehicle_id={} stored kappa {:.12g} != recomputed "
                            "{:.12g}".format(
                                vehicle_id, segment.control,
                                expected_curvature), checks, goal_checked)
                    if abs(expected_curvature) > (
                            1.0 / limits.r_min_ackermann +
                            limits.heading_tolerance_rad):
                        return _audit_failure(
                            "CURVATURE_LIMIT",
                            "vehicle_id={} |kappa|={:.12g} exceeds 1/Rmin="
                            "{:.12g}".format(
                                vehicle_id, abs(expected_curvature),
                                1.0 / limits.r_min_ackermann), checks,
                            goal_checked)
                    # The endpoint displacement is already rigidly implied by
                    # the block; verify the exact circular chord too.
                    expected_dx = (
                        segment.end_center_x - segment.start_center_x)
                    expected_dy = (
                        segment.end_center_y - segment.start_center_y)
                    chord_radius = math.hypot(expected_dx, expected_dy)
                    if radius > limits.pivot_tolerance_m:
                        half_angle = abs(dpsi) / 2.0
                        expected_chord = 2.0 * radius * abs(
                            math.sin(half_angle))
                        if abs(chord_radius - expected_chord) > (
                                limits.geometry_tolerance_m +
                                1e-9 * max(1.0, expected_chord)):
                            return _audit_failure(
                                "MODE_DISPLACEMENT_MISMATCH",
                                "vehicle_id={} chord {:.12g} != circular chord "
                                "{:.12g}".format(
                                    vehicle_id, chord_radius, expected_chord),
                                checks, goal_checked)
                elif segment.mode == OBLIQUE:
                    return _audit_failure(
                        "ROTATION_OBLIQUE_UNSUPPORTED",
                        "vehicle_id={} a pure rotation cannot use constant O "
                        "heading in S5".format(vehicle_id), checks,
                        goal_checked)
                else:
                    return _audit_failure(
                        "UNSUPPORTED_MODE",
                        "vehicle_id={} mode={}".format(
                            vehicle_id, segment.mode), checks, goal_checked)

            if expected is not None and not free_flags[vehicle_id]:
                goal_checked = True
                goal_error = _angle_error(
                    expected_end_theta, expected[vehicle_id])
                max_heading_error = max(max_heading_error, goal_error)
                if goal_error > limits.heading_tolerance_rad:
                    return _audit_failure(
                        "GOAL_HEADING_MISMATCH",
                        "vehicle_id={} theta_end differs from goal by "
                        "{:.12g} rad".format(vehicle_id, goal_error),
                        checks, goal_checked)
            checks += 1

        return ModeAudit(
            success=True,
            reason="OK",
            detail="KINEMATIC_AUDIT_PASSED_NOT_EXECUTABLE",
            certification=CERTIFICATION_AUDITED,
            checks=checks,
            max_center_error_m=max_center_error,
            max_heading_error_rad=max_heading_error,
            max_curvature=max_curvature,
            max_oblique_angle_rad=max_oblique,
            max_spin_center_motion_m=max_spin_motion,
            goal_headings_checked=goal_checked,
        )
    except FormationValidationError as exc:
        return _audit_failure("INVALID_MODE_ASSIGNMENT", str(exc))


def mode_word(assignment):
    """Return a stable, hashable vehicle-mode ordering for deduplication."""
    if not isinstance(assignment, FormationModeAssignment):
        raise FormationValidationError(
            "INVALID_MODE_ASSIGNMENT expected FormationModeAssignment")
    return tuple((segment.vehicle_id, segment.mode, segment.direction)
                 for segment in assignment.segments)