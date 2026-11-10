"""S8 common-time retiming for rigid formation geometry.

Every vehicle shares one monotonic formation progress ``lambda(t)``.  Vehicle
centers are recomputed from ``q_F(lambda)`` and fixed offsets; body headings
are the S5 block-local heading functions.  Each formation block is one
rest-to-rest quintic progress phase.  A block boundary is always a zero-speed
point; if any wheel angle would change, the whole formation inserts one
stationary reconfiguration event before the next block.

The output is ``FormationTimedTrajectory`` with certification
``GEOMETRY_ONLY``.  It is not executable, is not published, has no collision
certificate, and has not been audited by the future S9 independent evaluator.
No status in this module is ``FEASIBLE``.
"""
from bisect import bisect_left
from dataclasses import asdict, dataclass
import math
from typing import Mapping, Optional, Tuple

import numpy as np

from .core import Action
from .formation import (
    FormationModel,
    FormationPose,
    FormationValidationError,
    VehicleOffset,
)
from .formation_geometry import (
    FormationBlock,
    FormationContinuousPlan,
    TRANSLATION,
)
from .formation_kinematics import (
    ACKERMANN,
    OBLIQUE,
    SPIN,
    VehicleModeSegment,
)
from .mode_switch_timing import mode_switch_event
from .retiming import MotionLimits, quintic, wheel_configuration


TIMED_GEOMETRY_ONLY = "TIMED_GEOMETRY_ONLY"
GEOMETRY_ONLY = "GEOMETRY_ONLY"
NONE = "NONE"
REJECTED = "REJECTED"
FORMATION_RETIMING_INFEASIBLE = "FORMATION_RETIMING_INFEASIBLE"
TIMING_AUDIT_FAILED = "TIMING_AUDIT_FAILED"

_DYNAMIC_PROFILE = "formation_piecewise_quintic_v1"
_GEOMETRY_TOL = 1e-7
_TIME_TOL = 1e-9
_WHEEL_ANGLE_TOL = 1e-9
_DWELL_TOL = 1e-9
_SAFETY_FACTOR = 1.005
_PROFILE_SAMPLE_COUNT = 401
_UINT32_MAX = 2 ** 32 - 1

_DYNAMIC_LIMITS = (
    ("speed", "v_max", 1),
    ("acceleration", "a_max", 2),
    ("jerk", "jerk_max", 3),
    ("yaw_rate", "yaw_rate_max", 1),
    ("yaw_acceleration", "yaw_accel_max", 2),
    ("yaw_jerk", "yaw_jerk_max", 3),
    ("wheel_speed", "wheel_speed_max", 1),
    ("lateral_acceleration", "lateral_accel_max", 2),
)


def _finite(value, context, positive=False, nonnegative=False):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise FormationValidationError(
            "NONFINITE_VALUE {}={!r}".format(context, value)) from exc
    if not math.isfinite(result):
        raise FormationValidationError(
            "NONFINITE_VALUE {}={!r}".format(context, value))
    if positive and result <= 0.0:
        raise FormationValidationError(
            "INVALID_VALUE {} must be > 0".format(context))
    if nonnegative and result < 0.0:
        raise FormationValidationError(
            "INVALID_VALUE {} must be >= 0".format(context))
    return result


def _integer(value, context, minimum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise FormationValidationError(
            "INVALID_INTEGER {}={!r}".format(context, value))
    if minimum is not None and value < minimum:
        raise FormationValidationError(
            "INVALID_INTEGER {}={!r}".format(context, value))
    return value


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _angle_error(first, second):
    return abs(_wrap(first - second))


def _rotate(psi, x, y):
    cosine, sine = math.cos(psi), math.sin(psi)
    return cosine * x - sine * y, sine * x + cosine * y


def _s3(tau):
    return 60.0 - 360.0 * tau + 360.0 * tau * tau


@dataclass(frozen=True)
class FormationTimingLimits:
    """Per-vehicle dynamic and mode-residency limits for S8.

    These are independent from ``retiming.MotionLimits``.  A compatible
    ``MotionLimits`` is constructed only where the existing wheel-angle and
    mode-switch timing helpers are reused.
    """

    v_max: float = 0.8
    a_max: float = 0.6
    jerk_max: float = 2.0
    yaw_rate_max: float = 0.8
    yaw_accel_max: float = 0.8
    yaw_jerk_max: float = 3.0
    lateral_accel_max: float = 0.6
    wheel_speed_max: float = 1.0
    steer_angle_max: float = math.pi / 2.0
    steer_rate_max: float = 0.8
    steer_accel_max: float = 1.5
    mode_setup_time: float = 0.15
    mode_switch_duration: float = 0.3
    min_mode_dwell_s: float = 0.3
    wheelbase: float = 0.494
    track: float = 0.364
    r_min_ackermann: float = 0.810
    max_block_duration_s: float = 120.0

    def __post_init__(self):
        for name in ("v_max", "a_max", "jerk_max", "yaw_rate_max",
                     "yaw_accel_max", "yaw_jerk_max", "lateral_accel_max",
                     "wheel_speed_max", "steer_angle_max", "steer_rate_max",
                     "steer_accel_max", "wheelbase", "track",
                     "r_min_ackermann", "max_block_duration_s"):
            object.__setattr__(self, name, _finite(
                getattr(self, name), "FormationTimingLimits." + name,
                positive=True))
        for name in ("mode_setup_time", "mode_switch_duration",
                     "min_mode_dwell_s"):
            object.__setattr__(self, name, _finite(
                getattr(self, name), "FormationTimingLimits." + name,
                nonnegative=True))
        if self.steer_angle_max > math.pi:
            raise FormationValidationError(
                "INVALID_TIMING_LIMIT steer_angle_max must be <= pi")
        if self.mode_switch_duration <= 0.0:
            raise FormationValidationError(
                "INVALID_TIMING_LIMIT mode_switch_duration must be > 0")
        if self.max_block_duration_s < self.min_mode_dwell_s:
            raise FormationValidationError(
                "INVALID_TIMING_LIMIT max_block_duration_s must be >= "
                "min_mode_dwell_s")

    def as_dict(self):
        return asdict(self)


def _motion_limits(limits):
    return MotionLimits(
        v_max=limits.v_max,
        a_max=limits.a_max,
        jerk_max=limits.jerk_max,
        lateral_a_max=limits.lateral_accel_max,
        omega_max=limits.yaw_rate_max,
        alpha_max=limits.yaw_accel_max,
        angular_jerk_max=limits.yaw_jerk_max,
        steer_rate_max=limits.steer_rate_max,
        steer_accel_max=limits.steer_accel_max,
        steer_angle_max=limits.steer_angle_max,
        wheel_speed_max=limits.wheel_speed_max,
        mode_setup_time=limits.mode_setup_time,
        wheelbase=limits.wheelbase,
        track=limits.track,
        r_min_ack=limits.r_min_ackermann,
        translation_speed_policy="restricted",
        mode_switch_duration=limits.mode_switch_duration)


@dataclass(frozen=True)
class FormationTimedPoint:
    """One common-time sample for one vehicle.

    ``center_speed_mps`` is the signed speed along the commanded vehicle
    motion direction: negative for authorized A/O reverse travel and zero for
    Spin.  ``theta`` is the body heading, never the formation heading.
    """

    time_from_start: float
    center_x: float
    center_y: float
    theta: float
    center_speed_mps: float
    center_yaw_rate_radps: float
    mode: int
    mode_control: float
    block_index: int
    station: float
    stationary: bool = False
    center_acceleration_mps2: float = 0.0
    center_jerk_mps3: float = 0.0
    wheel_speed_mps: float = 0.0
    lateral_acceleration_mps2: float = 0.0

    def __post_init__(self):
        for name in ("time_from_start", "center_x", "center_y", "theta",
                     "center_speed_mps", "center_yaw_rate_radps",
                     "mode_control", "station",
                     "center_acceleration_mps2", "center_jerk_mps3",
                     "wheel_speed_mps", "lateral_acceleration_mps2"):
            object.__setattr__(self, name, _finite(
                getattr(self, name), "FormationTimedPoint." + name))
        if self.mode not in (ACKERMANN, OBLIQUE, SPIN):
            raise FormationValidationError(
                "INVALID_MODE {}".format(self.mode))
        object.__setattr__(self, "block_index", _integer(
            self.block_index, "FormationTimedPoint.block_index", minimum=0))
        if not isinstance(self.stationary, bool):
            raise FormationValidationError(
                "INVALID_STATIONARY_FLAG")
        if self.station < 0.0:
            raise FormationValidationError(
                "INVALID_STATION " + repr(self.station))

    def as_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class FormationTimingReport:
    dynamic_profile: str
    dt: float
    duration_s: float
    moving_seconds: float
    stationary_seconds: float
    block_durations: Tuple[dict, ...]
    phases: Tuple[dict, ...]
    stationary_events: Tuple[dict, ...]
    direction_changes: Tuple[dict, ...]
    vehicle_peaks: Mapping[int, dict]
    active_constraints: Tuple[dict, ...]
    mode_intervals: Mapping[int, Tuple[dict, ...]]
    limits: Mapping[str, object]
    certification: str = GEOMETRY_ONLY
    timing_audit: str = "SELF_CONSISTENCY_ONLY"
    executable: bool = False
    trajectory_published: bool = False
    feasible: bool = False
    independent_timing_audit: bool = False
    collision_certificate: bool = False

    def as_dict(self):
        return {
            "dynamic_profile": self.dynamic_profile,
            "dt": self.dt,
            "duration_s": self.duration_s,
            "moving_seconds": self.moving_seconds,
            "stationary_seconds": self.stationary_seconds,
            "block_durations": [dict(item) for item in self.block_durations],
            "phases": [dict(item) for item in self.phases],
            "stationary_events": [
                dict(item) for item in self.stationary_events],
            "direction_changes": [
                dict(item) for item in self.direction_changes],
            "vehicle_peaks": {
                int(vehicle_id): dict(peaks)
                for vehicle_id, peaks in self.vehicle_peaks.items()},
            "active_constraints": [
                dict(item) for item in self.active_constraints],
            "mode_intervals": {
                int(vehicle_id): [dict(item) for item in intervals]
                for vehicle_id, intervals in self.mode_intervals.items()},
            "limits": dict(self.limits),
            "certification": self.certification,
            "timing_audit": self.timing_audit,
            "executable": self.executable,
            "trajectory_published": self.trajectory_published,
            "feasible": self.feasible,
            "independent_timing_audit": self.independent_timing_audit,
            "collision_certificate": self.collision_certificate,
        }


@dataclass(frozen=True)
class FormationTimedTrajectory:
    """One common-time multi-vehicle timed trajectory; geometry-only."""

    plan_id: int
    formation_id: str
    offsets: Tuple[VehicleOffset, ...]
    formation_poses: Tuple[FormationPose, ...]
    times_from_start: Tuple[float, ...]
    trajectories: Mapping[int, Tuple[FormationTimedPoint, ...]]
    phases: Tuple[dict, ...]
    timing_report: FormationTimingReport
    certification: str = GEOMETRY_ONLY
    dynamic_profile: str = _DYNAMIC_PROFILE

    def __post_init__(self):
        if isinstance(self.plan_id, bool) or not isinstance(self.plan_id, int) or (
                self.plan_id < 0 or self.plan_id > _UINT32_MAX):
            raise FormationValidationError(
                "INVALID_PLAN_ID {}".format(self.plan_id))
        if not isinstance(self.formation_id, str) or not self.formation_id:
            raise FormationValidationError("INVALID_FORMATION_ID")
        try:
            offsets = tuple(self.offsets)
            poses = tuple(self.formation_poses)
            times = tuple(self.times_from_start)
        except TypeError as exc:
            raise FormationValidationError(
                "INVALID_TIMED_TRAJECTORY_COMPONENTS") from exc
        FormationModel(offsets)
        object.__setattr__(self, "offsets", offsets)
        object.__setattr__(self, "formation_poses", poses)
        object.__setattr__(self, "times_from_start", times)
        if len(poses) != len(times) or not times:
            raise FormationValidationError(
                "TIMED_TRAJECTORY_LENGTH_MISMATCH")
        if abs(times[0]) > _TIME_TOL:
            raise FormationValidationError("TIMES_MUST_START_AT_ZERO")
        for first, second in zip(times, times[1:]):
            if not second - first > _TIME_TOL:
                raise FormationValidationError(
                    "TIMES_NOT_STRICTLY_INCREASING")
        try:
            trajectories = dict(self.trajectories)
        except (TypeError, ValueError) as exc:
            raise FormationValidationError(
                "INVALID_TRAJECTORY_MAP") from exc
        if set(trajectories) != {offset.vehicle_id for offset in offsets}:
            raise FormationValidationError(
                "TRAJECTORY_VEHICLE_IDS_MISMATCH")
        checked = {}
        for vehicle_id, points in trajectories.items():
            points = tuple(points)
            if len(points) != len(times):
                raise FormationValidationError(
                    "TRAJECTORY_POINT_COUNT_MISMATCH vehicle_id={}".format(
                        vehicle_id))
            for index, (point, time) in enumerate(zip(points, times)):
                if abs(point.time_from_start - time) > _TIME_TOL:
                    raise FormationValidationError(
                        "TRAJECTORY_TIME_MISMATCH vehicle_id={} index={}".format(
                            vehicle_id, index))
            checked[vehicle_id] = points
        object.__setattr__(self, "trajectories", checked)
        if self.certification != GEOMETRY_ONLY:
            raise FormationValidationError(
                "INVALID_TIMED_CERTIFICATION {}".format(self.certification))
        if self.dynamic_profile != _DYNAMIC_PROFILE:
            raise FormationValidationError(
                "INVALID_DYNAMIC_PROFILE {}".format(self.dynamic_profile))

    @property
    def duration(self):
        return self.times_from_start[-1]

    @property
    def vehicle_ids(self):
        return tuple(offset.vehicle_id for offset in self.offsets)

    def report(self):
        return self.timing_report.as_dict()

    def as_status(self):
        return {
            "plan_id": self.plan_id,
            "formation_id": self.formation_id,
            "certification": self.certification,
            "dynamic_profile": self.dynamic_profile,
            "executable": False,
            "trajectory_published": False,
            "feasible": False,
            "independent_timing_audit": False,
            "collision_certificate": False,
        }


@dataclass(frozen=True)
class FormationTimingAudit:
    success: bool
    reason: str
    detail: str
    checks: int = 0
    max_rigidity_error_m: float = 0.0
    max_speed_mps: float = 0.0
    max_acceleration_mps2: float = 0.0
    max_jerk_mps3: float = 0.0
    max_yaw_rate_radps: float = 0.0
    max_wheel_speed_mps: float = 0.0
    max_lateral_acceleration_mps2: float = 0.0

    def as_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class FormationTimingFailure:
    code: str
    constraint: str
    vehicle_id: Optional[int]
    block_index: Optional[int]
    detail: str

    def as_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class FormationRetimingResult:
    status: str
    reason: str
    detail: str
    timed: Optional[FormationTimedTrajectory]
    audit: Optional[FormationTimingAudit]
    failure: Optional[FormationTimingFailure]

    def as_status(self):
        return {
            "status": self.status,
            "reason": self.reason,
            "detail": self.detail,
            "certification": ("NONE" if self.timed is None
                              else self.timed.certification),
            "executable": False,
            "trajectory_published": False,
            "feasible": False,
            "independent_timing_audit": False,
            "collision_certificate": False,
            "timing_audit_passed": bool(
                self.audit is not None and self.audit.success),
        }


@dataclass(frozen=True)
class _VehicleContext:
    vehicle_id: int
    offset_x: float
    offset_y: float
    segment: VehicleModeSegment
    mode: int
    direction: int
    control: float
    theta_rate: float
    wheel_angles: Tuple[float, ...]
    wheel_factors: Tuple[float, ...]
    max_wheel_angle: float


@dataclass(frozen=True)
class _AnalyzedBlock:
    block: FormationBlock
    contexts: Tuple[_VehicleContext, ...]
    duration: float
    required_duration: float
    driver: dict
    limiting: Tuple[dict, ...]
    peaks: Mapping[int, dict]
    report: dict


class _TimingInfeasible(Exception):
    def __init__(self, constraint, vehicle_id, block_index, detail):
        super().__init__(detail)
        self.constraint = constraint
        self.vehicle_id = vehicle_id
        self.block_index = block_index
        self.detail = detail


def _profile_taus():
    values = set(float(value) for value in np.linspace(
        0.0, 1.0, _PROFILE_SAMPLE_COUNT))
    values.update((0.5, 0.5 - math.sqrt(3.0) / 6.0,
                   0.5 + math.sqrt(3.0) / 6.0))
    return tuple(sorted(values))


_TAUS = _profile_taus()


def _formation_derivatives(control, station):
    """Exact first three derivatives of q_F with respect to lambda."""
    length = control.parameter_length
    if control.geometry_type == TRANSLATION:
        return (
            (control.end_qf.x - control.start_qf.x) / length,
            (control.end_qf.y - control.start_qf.y) / length,
            0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, (
                control.start_qf.psi)
    slope = control.delta_psi / length
    psi = control.start_qf.psi + (station / length) * control.delta_psi
    pivot_x, pivot_y = control.pivot_x, control.pivot_y
    first_x, first_y = _rotate(psi, -pivot_y, pivot_x)
    second_x, second_y = _rotate(psi, pivot_x, pivot_y)
    third_x, third_y = _rotate(psi, -pivot_y, pivot_x)
    return (
        (-slope * first_x, -slope * first_y, slope),
        (slope * slope * second_x, slope * slope * second_y, 0.0),
        (slope ** 3 * third_x, slope ** 3 * third_y, 0.0),
        slope, psi)


def _vehicle_derivatives(control, context, station):
    q_first, q_second, q_third, slope, _ = _formation_derivatives(
        control, station)
    rotated_r_x, rotated_r_y = _rotate(
        control.start_qf.psi + (station / control.parameter_length
                                ) * control.delta_psi,
        context.offset_x, context.offset_y)
    rotated_jr_x, rotated_jr_y = _rotate(
        control.start_qf.psi + (station / control.parameter_length
                                ) * control.delta_psi,
        -context.offset_y, context.offset_x)
    first = (q_first[0] + slope * rotated_jr_x,
             q_first[1] + slope * rotated_jr_y)
    second = (q_second[0] - slope * slope * rotated_r_x,
              q_second[1] - slope * slope * rotated_r_y)
    third = (q_third[0] - slope ** 3 * rotated_jr_x,
             q_third[1] - slope ** 3 * rotated_jr_y)
    return first, second, third


def _vector_norm(vector):
    return math.hypot(vector[0], vector[1])


def _wheel_data(segment, motion_limits):
    angles, factors = wheel_configuration(
        Action(segment.mode, 0.0, segment.control), motion_limits)
    return (tuple(float(value) for value in angles),
            tuple(float(value) for value in factors))


def _context_for(segment, offset, block, limits, motion_limits):
    length = block.parameter_length
    theta_rate = (segment.theta_end - segment.theta_start) / length
    if block.geometry_type == TRANSLATION:
        if abs(theta_rate) > 1e-6:
            raise _TimingInfeasible(
                "translation_body_heading_change", segment.vehicle_id,
                block.block_index,
                "translation block changes body heading by {:.12g} rad".format(
                    segment.theta_end - segment.theta_start))
    else:
        expected = block.control.delta_psi / length
        if abs(theta_rate - expected) > 1e-6:
            raise _TimingInfeasible(
                "rotation_heading_rate_mismatch", segment.vehicle_id,
                block.block_index,
                "rotation theta rate {:.12g} != dpsi/L {:.12g}".format(
                    theta_rate, expected))
    if segment.mode == ACKERMANN:
        if abs(segment.control) > 1.0 / limits.r_min_ackermann + 1e-9:
            raise _TimingInfeasible(
                "curvature_limit", segment.vehicle_id, block.block_index,
                "|kappa|={:.12g} exceeds 1/r_min".format(
                    abs(segment.control)))
    elif segment.mode == OBLIQUE:
        if abs(segment.control) > math.pi / 2.0 + 1e-9:
            raise _TimingInfeasible(
                "oblique_angle_limit", segment.vehicle_id, block.block_index,
                "|phi|={:.12g} exceeds pi/2".format(abs(segment.control)))
    elif abs(segment.control) > 1e-12:
        raise _TimingInfeasible(
            "spin_control_not_zero", segment.vehicle_id, block.block_index,
            "spin control is {:.12g}".format(segment.control))
    wheel_angles, wheel_factors = _wheel_data(segment, motion_limits)
    max_wheel_angle = max(abs(value) for value in wheel_angles)
    if max_wheel_angle > limits.steer_angle_max + 1e-9:
        raise _TimingInfeasible(
            "steer_angle_limit", segment.vehicle_id, block.block_index,
            "wheel angle {:.12g} rad exceeds {:.12g}".format(
                max_wheel_angle, limits.steer_angle_max))
    return _VehicleContext(
        vehicle_id=segment.vehicle_id,
        offset_x=offset.x,
        offset_y=offset.y,
        segment=segment,
        mode=segment.mode,
        direction=segment.direction,
        control=segment.control,
        theta_rate=theta_rate,
        wheel_angles=wheel_angles,
        wheel_factors=wheel_factors,
        max_wheel_angle=max_wheel_angle)


def _quantities(context, block, station, lambda_dot, lambda_ddot,
                lambda_dddot):
    first, second, third = _vehicle_derivatives(
        block.control, context, station)
    p_dot_x = first[0] * lambda_dot
    p_dot_y = first[1] * lambda_dot
    p_ddot_x = (second[0] * lambda_dot * lambda_dot +
                first[0] * lambda_ddot)
    p_ddot_y = (second[1] * lambda_dot * lambda_dot +
                first[1] * lambda_ddot)
    p_dddot_x = (third[0] * lambda_dot ** 3 +
                 3.0 * second[0] * lambda_dot * lambda_ddot +
                 first[0] * lambda_dddot)
    p_dddot_y = (third[1] * lambda_dot ** 3 +
                 3.0 * second[1] * lambda_dot * lambda_ddot +
                 first[1] * lambda_dddot)
    speed = math.hypot(p_dot_x, p_dot_y)
    acceleration = math.hypot(p_ddot_x, p_ddot_y)
    jerk = math.hypot(p_dddot_x, p_dddot_y)
    yaw_rate = abs(context.theta_rate * lambda_dot)
    yaw_acceleration = abs(context.theta_rate * lambda_ddot)
    yaw_jerk = abs(context.theta_rate * lambda_dddot)
    wheel_speed = (max(context.wheel_factors) *
                   (speed if context.mode != SPIN else yaw_rate))
    lateral = (abs(context.control) * speed * speed
               if context.mode == ACKERMANN else 0.0)
    return {
        "speed": speed,
        "acceleration": acceleration,
        "jerk": jerk,
        "yaw_rate": yaw_rate,
        "yaw_acceleration": yaw_acceleration,
        "yaw_jerk": yaw_jerk,
        "wheel_speed": wheel_speed,
        "lateral_acceleration": lateral,
    }


def _peak_table(contexts, block, limits):
    peaks = {}
    for context in contexts:
        peaks[context.vehicle_id] = {
            name: 0.0 for name, _, _ in _DYNAMIC_LIMITS}
        peaks[context.vehicle_id]["_tau"] = {
            name: 0.0 for name, _, _ in _DYNAMIC_LIMITS}
    length = block.parameter_length
    for tau in _TAUS:
        progress, velocity, acceleration = quintic(tau)
        lambda_dot = length * velocity
        lambda_ddot = length * acceleration
        lambda_dddot = length * _s3(tau)
        station = length * progress
        for context in contexts:
            values = _quantities(
                context, block, station, lambda_dot, lambda_ddot,
                lambda_dddot)
            vehicle_peaks = peaks[context.vehicle_id]
            for name, _, _ in _DYNAMIC_LIMITS:
                value = values[name]
                if value > vehicle_peaks[name]:
                    vehicle_peaks[name] = value
                    vehicle_peaks["_tau"][name] = tau
    return peaks


def _analyze_block(block, offsets, limits, motion_limits):
    if not isinstance(block, FormationBlock):
        raise FormationValidationError(
            "INVALID_BLOCK_TYPE {}".format(type(block).__name__))
    offset_map = {offset.vehicle_id: offset for offset in offsets}
    if set(block.vehicle_ids) != set(offset_map):
        raise FormationValidationError(
            "BLOCK_VEHICLE_IDS_MISMATCH")
    contexts = tuple(_context_for(
        block.segment_map[vehicle_id], offset_map[vehicle_id], block,
        limits, motion_limits)
        for vehicle_id in sorted(offset_map))
    peaks = _peak_table(contexts, block, limits)
    required = 0.0
    contributions = []
    for context in contexts:
        for name, limit_name, order in _DYNAMIC_LIMITS:
            limit = getattr(limits, limit_name)
            peak = peaks[context.vehicle_id][name]
            if peak <= 0.0:
                continue
            if order == 1:
                duration = peak / limit
            elif order == 2:
                duration = math.sqrt(peak / limit)
            else:
                duration = (peak / limit) ** (1.0 / 3.0)
            contributions.append({
                "constraint": name,
                "vehicle_id": context.vehicle_id,
                "block_index": block.block_index,
                "peak_at_unit_duration": peak,
                "limit": limit,
                "required_duration_s": duration,
                "tau": peaks[context.vehicle_id]["_tau"][name],
            })
            required = max(required, duration)
    if not math.isfinite(required):
        raise _TimingInfeasible(
            "nonfinite_duration", None, block.block_index,
            "required duration is not finite")
    dynamic_duration = required * _SAFETY_FACTOR
    if required > 0.0 and dynamic_duration > limits.max_block_duration_s:
        worst = max(contributions, key=lambda item: item[
            "required_duration_s"])
        raise _TimingInfeasible(
            worst["constraint"], worst["vehicle_id"], block.block_index,
            "{} requires {:.6f} s, exceeds max_block_duration_s={:.6f}".format(
                worst["constraint"], worst["required_duration_s"],
                limits.max_block_duration_s))
    duration = max(dynamic_duration, limits.min_mode_dwell_s)
    if duration <= 0.0:
        raise _TimingInfeasible(
            "nonpositive_duration", None, block.block_index,
            "block duration is not positive")
    utilizations = []
    for context in contexts:
        for name, limit_name, order in _DYNAMIC_LIMITS:
            limit = getattr(limits, limit_name)
            unit_peak = peaks[context.vehicle_id][name]
            actual_peak = unit_peak / (duration ** order) if unit_peak else 0.0
            peaks[context.vehicle_id][name] = actual_peak
            if limit > 0.0:
                utilizations.append({
                    "constraint": name,
                    "vehicle_id": context.vehicle_id,
                    "block_index": block.block_index,
                    "peak": actual_peak,
                    "limit": limit,
                    "utilization": actual_peak / limit,
                    "tau": peaks[context.vehicle_id]["_tau"][name],
                })
    for context in contexts:
        peaks[context.vehicle_id].pop("_tau", None)
        peaks[context.vehicle_id]["steer_rate"] = 0.0
        peaks[context.vehicle_id]["steer_acceleration"] = 0.0
        peaks[context.vehicle_id]["wheel_angle"] = context.max_wheel_angle
    if dynamic_duration < limits.min_mode_dwell_s:
        driver = {
            "constraint": "mode_dwell",
            "vehicle_id": None,
            "block_index": block.block_index,
            "peak": duration,
            "limit": limits.min_mode_dwell_s,
            "utilization": 1.0,
            "tau": None,
        }
        limiting = ()
    else:
        utilizations.sort(
            key=lambda item: item["utilization"], reverse=True)
        driver = dict(utilizations[0])
        limiting = tuple(
            dict(item) for item in utilizations[:3]
            if item["utilization"] >= 0.95 * max(0.0, driver["utilization"]))
    report = {
        "block_index": block.block_index,
        "geometry_type": block.geometry_type,
        "parameter_length": block.parameter_length,
        "duration_s": duration,
        "dynamic_duration_s": dynamic_duration,
        "duration_driver": dict(driver),
        "limiting_constraints": [dict(item) for item in limiting],
        "vehicle_peaks": {
            context.vehicle_id: dict(peaks[context.vehicle_id])
            for context in contexts},
    }
    return _AnalyzedBlock(
        block=block,
        contexts=contexts,
        duration=duration,
        required_duration=required,
        driver=driver,
        limiting=limiting,
        peaks=peaks,
        report=report)


def _wheel_delta(first, second):
    return max(
        _angle_error(a, b)
        for a, b in zip(first.wheel_angles, second.wheel_angles))


def _boundary_event(previous, current, limits, motion_limits, time_s):
    previous_map = {context.vehicle_id: context
                    for context in previous.contexts}
    current_map = {context.vehicle_id: context
                   for context in current.contexts}
    vehicle_ids = sorted(previous_map)
    details = []
    mode_changed = False
    wheel_changed = False
    for vehicle_id in vehicle_ids:
        before = previous_map[vehicle_id]
        after = current_map[vehicle_id]
        delta = _wheel_delta(before, after)
        event = mode_switch_event(delta, motion_limits)
        if before.mode != after.mode:
            mode_changed = True
        if delta > _WHEEL_ANGLE_TOL:
            wheel_changed = True
        details.append({
            "vehicle_id": vehicle_id,
            "from_mode": before.mode,
            "to_mode": after.mode,
            "from_direction": before.direction,
            "to_direction": after.direction,
            "wheel_angles_start_rad": list(before.wheel_angles),
            "wheel_angles_end_rad": list(after.wheel_angles),
            "max_wheel_angle_delta_rad": delta,
            "setup_s": event["setup_s"],
            "steering_s": event["steering_s"],
            "steering_profile": event["steering_profile"],
            "steer_rate_peak_radps": event["steer_rate_peak"],
            "steer_accel_peak_radps2": event["steer_accel_peak"],
        })
    if not mode_changed and not wheel_changed:
        return None
    duration = max(item["setup_s"] + item["steering_s"]
                   for item in details)
    kind = "mode_switch" if mode_changed else "wheel_reconfiguration"
    return {
        "kind": kind,
        "t0_s": time_s,
        "duration_s": duration,
        "block_index": current.block.block_index,
        "from_block_index": previous.block.block_index,
        "pose": [current.block.control.start_qf.x,
                 current.block.control.start_qf.y,
                 current.block.control.start_qf.psi],
        "headings": {
            int(segment.vehicle_id): float(segment.theta_start)
            for segment in current.block.vehicle_mode_segments},
        "max_wheel_angle_delta_rad": max(
            item["max_wheel_angle_delta_rad"] for item in details),
        "vehicles": details,
    }


def _direction_changes(previous, current, time_s):
    events = []
    previous_map = {context.vehicle_id: context
                    for context in previous.contexts}
    for context in current.contexts:
        before = previous_map[context.vehicle_id]
        if before.direction != context.direction:
            events.append({
                "kind": "direction_reversal",
                "vehicle_id": context.vehicle_id,
                "from_mode": before.mode,
                "to_mode": context.mode,
                "mode": context.mode,
                "from_direction": before.direction,
                "to_direction": context.direction,
                "time_s": time_s,
                "duration_s": 0.0,
                "at_rest": True,
            })
    return events


def _mode_intervals(phases, analyzed):
    intervals = {}
    block_map = {item.block.block_index: item for item in analyzed}
    for item in analyzed:
        for segment in item.block.vehicle_mode_segments:
            intervals.setdefault(segment.vehicle_id, [])
    for vehicle_id in sorted(intervals):
        runs = []
        for phase in phases:
            block = block_map[phase["block_index"]].block
            segment = block.segment_map[vehicle_id]
            start = float(phase["t0_s"])
            end = start + float(phase["duration_s"])
            mode = segment.mode
            if runs and runs[-1]["mode"] == mode and abs(
                    runs[-1]["end_s"] - start) <= 1e-9:
                runs[-1]["end_s"] = end
                runs[-1]["duration_s"] = end - runs[-1]["start_s"]
            else:
                runs.append({
                    "vehicle_id": vehicle_id,
                    "mode": mode,
                    "start_s": start,
                    "end_s": end,
                    "duration_s": end - start,
                })
        intervals[vehicle_id] = tuple(runs)
    return intervals


def _vehicle_peaks(analyzed, stationary_events):
    peaks = {}
    for item in analyzed:
        for vehicle_id, values in item.peaks.items():
            current = peaks.setdefault(vehicle_id, {
                name: 0.0 for name, _, _ in _DYNAMIC_LIMITS})
            current.setdefault("steer_rate", 0.0)
            current.setdefault("steer_acceleration", 0.0)
            current.setdefault("wheel_angle", 0.0)
            for name, _, _ in _DYNAMIC_LIMITS:
                current[name] = max(current[name], values[name])
            current["wheel_angle"] = max(
                current["wheel_angle"], values["wheel_angle"])
    for event in stationary_events:
        for detail in event["vehicles"]:
            vehicle_id = detail["vehicle_id"]
            current = peaks.setdefault(vehicle_id, {
                name: 0.0 for name, _, _ in _DYNAMIC_LIMITS})
            current.setdefault("steer_rate", 0.0)
            current.setdefault("steer_acceleration", 0.0)
            current.setdefault("wheel_angle", 0.0)
            current["steer_rate"] = max(
                current["steer_rate"], detail["steer_rate_peak_radps"])
            current["steer_acceleration"] = max(
                current["steer_acceleration"],
                detail["steer_accel_peak_radps2"])
    return peaks


def _sample_motion(block, analyzed, start_time, dt):
    points = []
    duration = analyzed.duration
    length = block.parameter_length
    count = max(1, int(math.ceil(duration / dt)))
    for index in range(1, count + 1):
        tau = index / count
        progress, velocity, acceleration = quintic(tau)
        lambda_dot = length * velocity / duration
        lambda_ddot = length * acceleration / duration ** 2
        lambda_dddot = length * _s3(tau) / duration ** 3
        station = min(length, max(0.0, length * progress))
        qf = block.qf_at(station)
        time_s = start_time + duration * tau
        for context in analyzed.contexts:
            values = _quantities(
                context, block, station, lambda_dot, lambda_ddot,
                lambda_dddot)
            center_x, center_y = _rotate(
                qf.psi, context.offset_x, context.offset_y)
            center_x += qf.x
            center_y += qf.y
            signed_speed = (
                context.direction * values["speed"]
                if context.mode != SPIN else 0.0)
            points.append(FormationTimedPoint(
                time_from_start=time_s,
                center_x=center_x,
                center_y=center_y,
                theta=block.body_heading_at(context.vehicle_id, station),
                center_speed_mps=signed_speed,
                center_yaw_rate_radps=context.theta_rate * lambda_dot,
                mode=context.mode,
                mode_control=context.control,
                block_index=block.block_index,
                station=station,
                stationary=False,
                center_acceleration_mps2=values["acceleration"],
                center_jerk_mps3=values["jerk"],
                wheel_speed_mps=values["wheel_speed"],
                lateral_acceleration_mps2=values["lateral_acceleration"]))
    return points


def _sample_stationary(block, analyzed, start_time, duration, dt):
    points = []
    qf = block.qf_at(0.0)
    count = max(1, int(math.ceil(duration / dt)))
    for index in range(1, count + 1):
        time_s = start_time + duration * index / count
        for context in analyzed.contexts:
            center_x, center_y = _rotate(
                qf.psi, context.offset_x, context.offset_y)
            center_x += qf.x
            center_y += qf.y
            points.append(FormationTimedPoint(
                time_from_start=time_s,
                center_x=center_x,
                center_y=center_y,
                theta=block.body_heading_at(context.vehicle_id, 0.0),
                center_speed_mps=0.0,
                center_yaw_rate_radps=0.0,
                mode=context.mode,
                mode_control=context.control,
                block_index=block.block_index,
                station=0.0,
                stationary=True,
                center_acceleration_mps2=0.0,
                center_jerk_mps3=0.0,
                wheel_speed_mps=0.0,
                lateral_acceleration_mps2=0.0))
    return points


def _build_timed(plan, analyzed, phases, stationary_events,
                 direction_changes, limits, dt):
    first = analyzed[0]
    first_block = first.block
    qf = first_block.qf_at(0.0)
    times = [0.0]
    poses = [qf]
    per_vehicle = {
        context.vehicle_id: [FormationTimedPoint(
            time_from_start=0.0,
            center_x=qf.x + _rotate(
                qf.psi, context.offset_x, context.offset_y)[0],
            center_y=qf.y + _rotate(
                qf.psi, context.offset_x, context.offset_y)[1],
            theta=first_block.body_heading_at(context.vehicle_id, 0.0),
            center_speed_mps=0.0,
            center_yaw_rate_radps=0.0,
            mode=context.mode,
            mode_control=context.control,
            block_index=first_block.block_index,
            station=0.0,
            stationary=False,
            center_acceleration_mps2=0.0,
            center_jerk_mps3=0.0,
            wheel_speed_mps=0.0,
            lateral_acceleration_mps2=0.0)]
        for context in first.contexts}
    block_map = {item.block.block_index: item for item in analyzed}
    for phase in phases:
        item = block_map[phase["block_index"]]
        if phase["kind"] == "motion":
            new_points = _sample_motion(
                item.block, item, phase["t0_s"], dt)
        else:
            new_points = _sample_stationary(
                item.block, item, phase["t0_s"], phase["duration_s"], dt)
        contexts = item.contexts
        count = len(new_points) // len(contexts)
        by_vehicle = {vehicle_id: [] for vehicle_id in per_vehicle}
        for index in range(count):
            for offset_index, context in enumerate(contexts):
                point = new_points[index * len(contexts) + offset_index]
                by_vehicle[context.vehicle_id].append(point)
        for vehicle_id, values in by_vehicle.items():
            per_vehicle[vehicle_id].extend(values)
        first_vehicle_points = by_vehicle[contexts[0].vehicle_id]
        times.extend(point.time_from_start for point in first_vehicle_points)
        if phase["kind"] == "motion":
            poses.extend(item.block.qf_at(point.station)
                         for point in first_vehicle_points)
        else:
            poses.extend(item.block.qf_at(0.0) for _ in range(count))
    trajectories = {
        vehicle_id: tuple(points)
        for vehicle_id, points in per_vehicle.items()}
    moving_seconds = sum(item.duration for item in analyzed)
    stationary_seconds = sum(
        phase["duration_s"] for phase in phases
        if phase["kind"] == "stationary")
    peaks = _vehicle_peaks(analyzed, stationary_events)
    intervals = _mode_intervals(phases, analyzed)
    active = [
        dict(item.driver)
        for item in analyzed
        if item.driver.get("constraint") != "mode_dwell"]
    active.extend(
        dict(constraint)
        for item in analyzed for constraint in item.limiting)
    report = FormationTimingReport(
        dynamic_profile=_DYNAMIC_PROFILE,
        dt=dt,
        duration_s=times[-1],
        moving_seconds=moving_seconds,
        stationary_seconds=stationary_seconds,
        block_durations=tuple(item.report for item in analyzed),
        phases=tuple(dict(phase) for phase in phases),
        stationary_events=tuple(
            dict(event) for event in stationary_events),
        direction_changes=tuple(
            dict(event) for event in direction_changes),
        vehicle_peaks=peaks,
        active_constraints=tuple(active),
        mode_intervals=intervals,
        limits=limits.as_dict())
    return FormationTimedTrajectory(
        plan_id=plan.plan_id,
        formation_id=plan.formation_id,
        offsets=tuple(plan.offsets),
        formation_poses=tuple(poses),
        times_from_start=tuple(times),
        trajectories=trajectories,
        phases=tuple(dict(phase) for phase in phases),
        timing_report=report)


def retime_formation_plan(plan, limits=None, dt=0.02):
    """Build one common-time, geometry-only timed trajectory.

    Invalid input types raise ``FormationValidationError``.  Dynamic or static
    infeasibility returns a ``FormationRetimingResult`` whose status is
    ``FORMATION_RETIMING_INFEASIBLE``.  A failed self-audit never produces a
    certified timed trajectory.
    """
    if not isinstance(plan, FormationContinuousPlan):
        raise FormationValidationError(
            "INVALID_FORMATION_PLAN expected FormationContinuousPlan")
    limits = limits or FormationTimingLimits()
    if not isinstance(limits, FormationTimingLimits):
        raise FormationValidationError(
            "INVALID_FORMATION_TIMING_LIMITS")
    dt = _finite(dt, "dt", positive=True)
    if not plan.blocks:
        raise FormationValidationError("EMPTY_FORMATION_PLAN")
    blocks = tuple(sorted(plan.blocks, key=lambda block: block.block_index))
    indices = [block.block_index for block in blocks]
    if indices != list(range(len(blocks))):
        raise FormationValidationError(
            "INVALID_BLOCK_SEQUENCE {}".format(indices))
    offsets = tuple(sorted(plan.offsets, key=lambda item: item.vehicle_id))
    model = FormationModel(offsets)
    if tuple(plan.vehicle_ids) != model.vehicle_ids:
        raise FormationValidationError(
            "PLAN_VEHICLE_IDS_MISMATCH")
    motion_limits = _motion_limits(limits)
    analyzed = []
    try:
        for block in blocks:
            analyzed.append(_analyze_block(
                block, offsets, limits, motion_limits))
    except _TimingInfeasible as failure:
        detail = failure.detail
        return FormationRetimingResult(
            status=FORMATION_RETIMING_INFEASIBLE,
            reason=FORMATION_RETIMING_INFEASIBLE,
            detail=detail,
            timed=None,
            audit=None,
            failure=FormationTimingFailure(
                code=FORMATION_RETIMING_INFEASIBLE,
                constraint=failure.constraint,
                vehicle_id=failure.vehicle_id,
                block_index=failure.block_index,
                detail=detail))
    phases = []
    stationary_events = []
    direction_changes = []
    elapsed = 0.0
    for index, item in enumerate(analyzed):
        if index > 0:
            event = _boundary_event(
                analyzed[index - 1], item, limits, motion_limits, elapsed)
            direction_changes.extend(
                _direction_changes(analyzed[index - 1], item, elapsed))
            if event is not None:
                stationary_events.append(event)
                phases.append({
                    "kind": "stationary",
                    "block_index": item.block.block_index,
                    "from_block_index": analyzed[index - 1].block.block_index,
                    "t0_s": elapsed,
                    "duration_s": event["duration_s"],
                    "event_kind": event["kind"],
                    "headings": dict(event["headings"]),
                    "max_wheel_angle_delta_rad": event[
                        "max_wheel_angle_delta_rad"],
                })
                elapsed += event["duration_s"]
        phases.append({
            "kind": "motion",
            "block_index": item.block.block_index,
            "t0_s": elapsed,
            "duration_s": item.duration,
            "geometry_type": item.block.geometry_type,
            "parameter_length": item.block.parameter_length,
            "profile": "piecewise_quintic",
            "mode_word": tuple(
                (segment.vehicle_id, segment.mode, segment.direction)
                for segment in item.block.vehicle_mode_segments),
        })
        elapsed += item.duration
    timed = _build_timed(
        plan, analyzed, phases, stationary_events, direction_changes,
        limits, dt)
    audit = audit_formation_timing(plan, timed, limits)
    if not audit.success:
        detail = "{} {}".format(audit.reason, audit.detail).strip()
        return FormationRetimingResult(
            status=REJECTED,
            reason=TIMING_AUDIT_FAILED,
            detail=detail,
            timed=None,
            audit=audit,
            failure=FormationTimingFailure(
                code=TIMING_AUDIT_FAILED,
                constraint="timing_audit",
                vehicle_id=None,
                block_index=None,
                detail=detail))
    return FormationRetimingResult(
        status=TIMED_GEOMETRY_ONLY,
        reason="OK",
        detail="GEOMETRY_ONLY: common-time dynamics only; S9 independent "
               "audit and collision certification are still required",
        timed=timed,
        audit=audit,
        failure=None)


def _audit_result(success, reason, detail, checks, metrics=None):
    metrics = metrics or {}
    return FormationTimingAudit(
        success=success,
        reason=reason,
        detail=detail,
        checks=checks,
        max_rigidity_error_m=metrics.get("max_rigidity_error_m", 0.0),
        max_speed_mps=metrics.get("max_speed_mps", 0.0),
        max_acceleration_mps2=metrics.get("max_acceleration_mps2", 0.0),
        max_jerk_mps3=metrics.get("max_jerk_mps3", 0.0),
        max_yaw_rate_radps=metrics.get("max_yaw_rate_radps", 0.0),
        max_wheel_speed_mps=metrics.get("max_wheel_speed_mps", 0.0),
        max_lateral_acceleration_mps2=metrics.get(
            "max_lateral_acceleration_mps2", 0.0))


def _limit_value(limits, name):
    return getattr(limits, name)


def audit_formation_timing(plan, timed, limits=None):
    """Recompute rigid centers, headings, modes, dwell and dynamic caps.

    This is an S8 self-consistency audit.  It is intentionally not the S9
    independent evaluator: it consumes the S8 sample grid and phase report,
    does not perform collision certification, and never emits ``FEASIBLE``.
    """
    if not isinstance(plan, FormationContinuousPlan):
        raise FormationValidationError(
            "INVALID_FORMATION_PLAN expected FormationContinuousPlan")
    if not isinstance(timed, FormationTimedTrajectory):
        raise FormationValidationError(
            "INVALID_TIMED_TRAJECTORY expected FormationTimedTrajectory")
    limits = limits or FormationTimingLimits()
    if not isinstance(limits, FormationTimingLimits):
        raise FormationValidationError(
            "INVALID_FORMATION_TIMING_LIMITS")
    model = FormationModel(plan.offsets)
    vehicle_ids = model.vehicle_ids
    checks = 0
    metrics = {
        "max_rigidity_error_m": 0.0,
        "max_speed_mps": 0.0,
        "max_acceleration_mps2": 0.0,
        "max_jerk_mps3": 0.0,
        "max_yaw_rate_radps": 0.0,
        "max_wheel_speed_mps": 0.0,
        "max_lateral_acceleration_mps2": 0.0,
    }

    def fail(reason, detail):
        return _audit_result(False, reason, detail, checks, metrics)

    if timed.plan_id != plan.plan_id:
        return fail("PLAN_ID_MISMATCH", "timed plan_id does not match plan")
    checks += 1
    if timed.formation_id != plan.formation_id:
        return fail("FORMATION_ID_MISMATCH",
                    "timed formation_id does not match plan")
    checks += 1
    if tuple(sorted(plan.offsets, key=lambda item: item.vehicle_id)) != tuple(
            timed.offsets):
        return fail("OFFSET_MISMATCH", "timed offsets differ from plan")
    checks += 1
    times = timed.times_from_start
    poses = timed.formation_poses
    if len(times) != len(poses) or not times:
        return fail("TIME_GRID_MISMATCH",
                    "formation_poses and times_from_start differ in length")
    checks += 1
    if abs(times[0]) > _TIME_TOL:
        return fail("TIME_GRID_MISMATCH", "times_from_start must start at 0")
    checks += 1
    for first, second in zip(times, times[1:]):
        if not second - first > _TIME_TOL:
            return fail("NON_MONOTONIC_TIME",
                        "times_from_start is not strictly increasing near "
                        "t={:.12g}".format(first))
        checks += 1
    duration = times[-1]
    if abs(timed.timing_report.duration_s - duration) > _TIME_TOL:
        return fail("REPORT_DURATION_MISMATCH",
                    "timing report duration differs from the time grid")
    checks += 1
    trajectories = timed.trajectories
    if set(trajectories) != set(vehicle_ids):
        return fail("TRAJECTORY_VEHICLE_IDS_MISMATCH",
                    "trajectories do not cover the plan vehicles")
    checks += 1
    blocks = {block.block_index: block
              for block in sorted(plan.blocks, key=lambda item: item.block_index)}
    for vehicle_id in vehicle_ids:
        points = trajectories[vehicle_id]
        if len(points) != len(times):
            return fail(
                "TRAJECTORY_POINT_COUNT_MISMATCH",
                "vehicle_id={} point count differs from the time grid".format(
                    vehicle_id))
        checks += 1
        for index, (point, time_s) in enumerate(zip(points, times)):
            if abs(point.time_from_start - time_s) > _TIME_TOL:
                return fail(
                    "TRAJECTORY_TIME_MISMATCH",
                    "vehicle_id={} index={} time differs".format(
                        vehicle_id, index))
            checks += 1
    for index, (time_s, pose) in enumerate(zip(times, poses)):
        if not isinstance(pose, FormationPose):
            return fail("INVALID_FORMATION_POSE",
                        "index={} is not FormationPose".format(index))
        first_point = trajectories[vehicle_ids[0]][index]
        block_index = first_point.block_index
        if block_index not in blocks:
            return fail("UNKNOWN_BLOCK_INDEX",
                        "index={} block_index={}".format(index, block_index))
        block = blocks[block_index]
        stationary = first_point.stationary
        station = first_point.station
        if stationary:
            expected_pose = block.qf_at(0.0)
        else:
            if station < -_GEOMETRY_TOL or station > (
                    block.parameter_length + _GEOMETRY_TOL):
                return fail(
                    "INVALID_STATION",
                    "index={} station={} outside block".format(
                        index, station))
            expected_pose = block.qf_at(station)
        checks += 1
        pose_error = math.hypot(
            pose.x - expected_pose.x, pose.y - expected_pose.y)
        pose_angle_error = _angle_error(pose.psi, expected_pose.psi)
        if pose_error > _GEOMETRY_TOL or pose_angle_error > _GEOMETRY_TOL:
            return fail(
                "FORMATION_POSE_MISMATCH",
                "index={} pose error {:.12g} m {:.12g} rad".format(
                    index, pose_error, pose_angle_error))
        checks += 1
        expected_centers = model.centers(pose)
        for vehicle_id in vehicle_ids:
            point = trajectories[vehicle_id][index]
            if point.block_index != block_index or (
                    abs(point.station - station) > _TIME_TOL) or (
                    point.stationary != stationary):
                return fail(
                    "COMMON_TIME_STATE_MISMATCH",
                    "index={} vehicle_id={} common block/station differs".format(
                        index, vehicle_id))
            checks += 1
            center_error = math.hypot(
                point.center_x - expected_centers[vehicle_id][0],
                point.center_y - expected_centers[vehicle_id][1])
            if center_error > max(metrics["max_rigidity_error_m"],
                                  0.0):
                metrics["max_rigidity_error_m"] = max(
                    metrics["max_rigidity_error_m"], center_error)
            if center_error > _GEOMETRY_TOL:
                return fail(
                    "RIGIDITY_VIOLATION",
                    "index={} vehicle_id={} center error {:.12g} m".format(
                        index, vehicle_id, center_error))
            checks += 1
            segment = block.segment_map[vehicle_id]
            expected_theta = block.body_heading_at(
                vehicle_id, 0.0 if stationary else station)
            if _angle_error(point.theta, expected_theta) > 1e-6:
                return fail(
                    "HEADING_MISMATCH",
                    "index={} vehicle_id={} body heading differs".format(
                        index, vehicle_id))
            checks += 1
            if point.mode != segment.mode or abs(
                    point.mode_control - segment.control) > 1e-9:
                return fail(
                    "MODE_MISMATCH",
                    "index={} vehicle_id={} mode/control differs".format(
                        index, vehicle_id))
            checks += 1
            if stationary:
                if any(abs(value) > 1e-9 for value in (
                        point.center_speed_mps, point.center_yaw_rate_radps,
                        point.center_acceleration_mps2,
                        point.center_jerk_mps3, point.wheel_speed_mps,
                        point.lateral_acceleration_mps2)):
                    return fail(
                        "STATIONARY_MOTION",
                        "index={} vehicle_id={} stationary sample has "
                        "motion".format(index, vehicle_id))
                checks += 1
            if segment.mode == SPIN and abs(point.center_speed_mps) > 1e-9:
                return fail(
                    "SPIN_CENTER_MOVED",
                    "index={} vehicle_id={} spin speed={:.12g}".format(
                        index, vehicle_id, point.center_speed_mps))
            checks += 1
            if (segment.mode != SPIN and
                    abs(point.center_speed_mps) > 1e-9 and
                    int(math.copysign(1.0, point.center_speed_mps)) !=
                    segment.direction):
                return fail(
                    "DIRECTION_SIGN_MISMATCH",
                    "index={} vehicle_id={} speed={:.12g} direction={}".format(
                        index, vehicle_id, point.center_speed_mps,
                        segment.direction))
            checks += 1
            speed = abs(point.center_speed_mps)
            acceleration = abs(point.center_acceleration_mps2)
            jerk = abs(point.center_jerk_mps3)
            yaw = abs(point.center_yaw_rate_radps)
            wheel = abs(point.wheel_speed_mps)
            lateral = abs(point.lateral_acceleration_mps2)
            metrics["max_speed_mps"] = max(
                metrics["max_speed_mps"], speed)
            metrics["max_acceleration_mps2"] = max(
                metrics["max_acceleration_mps2"], acceleration)
            metrics["max_jerk_mps3"] = max(metrics["max_jerk_mps3"], jerk)
            metrics["max_yaw_rate_radps"] = max(
                metrics["max_yaw_rate_radps"], yaw)
            metrics["max_wheel_speed_mps"] = max(
                metrics["max_wheel_speed_mps"], wheel)
            metrics["max_lateral_acceleration_mps2"] = max(
                metrics["max_lateral_acceleration_mps2"], lateral)
            for value, limit, reason in (
                    (speed, limits.v_max, "SPEED_LIMIT"),
                    (acceleration, limits.a_max, "ACCELERATION_LIMIT"),
                    (jerk, limits.jerk_max, "JERK_LIMIT"),
                    (yaw, limits.yaw_rate_max, "YAW_RATE_LIMIT"),
                    (wheel, limits.wheel_speed_max, "WHEEL_SPEED_LIMIT"),
                    (lateral, limits.lateral_accel_max,
                     "LATERAL_ACCELERATION_LIMIT")):
                if value > limit + 1e-9:
                    return fail(
                        reason,
                        "index={} vehicle_id={} value={:.12g} limit={:.12g}"
                        .format(index, vehicle_id, value, limit))
                checks += 1
    start_pose = poses[0]
    if math.hypot(start_pose.x - plan.start.x,
                  start_pose.y - plan.start.y) > _GEOMETRY_TOL or (
            _angle_error(start_pose.psi, plan.start.psi) > _GEOMETRY_TOL):
        return fail("ENDPOINT_MISMATCH", "start formation pose differs")
    checks += 1
    goal_pose = poses[-1]
    if math.hypot(goal_pose.x - plan.goal.x,
                  goal_pose.y - plan.goal.y) > _GEOMETRY_TOL or (
            _angle_error(goal_pose.psi, plan.goal.psi) > _GEOMETRY_TOL):
        return fail("ENDPOINT_MISMATCH", "goal formation pose differs")
    checks += 1
    start_centers = model.centers(start_pose)
    goal_centers = model.centers(goal_pose)
    start_headings = plan.start_heading_map
    goal_headings = plan.goal_heading_map
    free = plan.heading_free_map
    for vehicle_id in vehicle_ids:
        first = trajectories[vehicle_id][0]
        last = trajectories[vehicle_id][-1]
        if math.hypot(first.center_x - start_centers[vehicle_id][0],
                      first.center_y - start_centers[vehicle_id][1]) > (
                _GEOMETRY_TOL):
            return fail("ENDPOINT_MISMATCH",
                        "vehicle_id={} start center differs".format(
                            vehicle_id))
        checks += 1
        if math.hypot(last.center_x - goal_centers[vehicle_id][0],
                      last.center_y - goal_centers[vehicle_id][1]) > (
                _GEOMETRY_TOL):
            return fail("ENDPOINT_MISMATCH",
                        "vehicle_id={} goal center differs".format(
                            vehicle_id))
        checks += 1
        if _angle_error(first.theta, start_headings[vehicle_id]) > 1e-6:
            return fail("ENDPOINT_MISMATCH",
                        "vehicle_id={} start heading differs".format(
                            vehicle_id))
        checks += 1
        if not free[vehicle_id] and _angle_error(
                last.theta, goal_headings[vehicle_id]) > 1e-6:
            return fail("ENDPOINT_MISMATCH",
                        "vehicle_id={} goal heading differs".format(
                            vehicle_id))
        checks += 1
    intervals = timed.timing_report.mode_intervals
    for vehicle_id in vehicle_ids:
        runs = intervals.get(vehicle_id, ())
        if not runs:
            return fail("MODE_INTERVAL_MISSING",
                        "vehicle_id={} has no mode intervals".format(
                            vehicle_id))
        if abs(runs[0]["start_s"]) > _TIME_TOL or abs(
                runs[-1]["end_s"] - duration) > _TIME_TOL:
            return fail("MODE_INTERVAL_GAP",
                        "vehicle_id={} intervals do not cover the "
                        "trajectory".format(vehicle_id))
        checks += 1
        for first, second in zip(runs, runs[1:]):
            if abs(first["end_s"] - second["start_s"]) > _TIME_TOL:
                return fail("MODE_INTERVAL_GAP",
                            "vehicle_id={} intervals are not contiguous".format(
                                vehicle_id))
            checks += 1
        for run in runs:
            if run["duration_s"] + 1e-9 < limits.min_mode_dwell_s:
                return fail(
                    "MODE_DWELL_VIOLATION",
                    "vehicle_id={} mode={} duration={:.12g} < {:.12g}".format(
                        vehicle_id, run["mode"], run["duration_s"],
                        limits.min_mode_dwell_s))
            checks += 1
    for change in timed.timing_report.direction_changes:
        index = bisect_left(times, change["time_s"])
        if index >= len(times) or abs(times[index] - change["time_s"]) > (
                _TIME_TOL):
            return fail("DIRECTION_CHANGE_TIME_MISSING",
                        "direction change at t={:.12g} has no sample".format(
                            change["time_s"]))
        checks += 1
        vehicle_id = change["vehicle_id"]
        point = trajectories[vehicle_id][index]
        if abs(point.center_speed_mps) > 1e-9:
            return fail(
                "DIRECTION_REVERSAL_NOT_AT_REST",
                "vehicle_id={} t={:.12g} speed={:.12g}".format(
                    vehicle_id, change["time_s"],
                    point.center_speed_mps))
        checks += 1
    peaks = timed.timing_report.vehicle_peaks
    for vehicle_id in vehicle_ids:
        values = peaks.get(vehicle_id, {})
        for name, limit_name, _ in _DYNAMIC_LIMITS:
            limit = _limit_value(limits, limit_name)
            if abs(values.get(name, 0.0)) > limit + 1e-9:
                return fail(
                    "REPORTED_PEAK_LIMIT",
                    "vehicle_id={} {} peak={:.12g} limit={:.12g}".format(
                        vehicle_id, name, values.get(name, 0.0), limit))
            checks += 1
        if values.get("steer_rate", 0.0) > limits.steer_rate_max + 1e-9:
            return fail("STEER_RATE_LIMIT",
                        "vehicle_id={} steering rate exceeds limit".format(
                            vehicle_id))
        checks += 1
        if values.get("steer_acceleration", 0.0) > (
                limits.steer_accel_max + 1e-9):
            return fail("STEER_ACCELERATION_LIMIT",
                        "vehicle_id={} steering acceleration exceeds "
                        "limit".format(vehicle_id))
        checks += 1
        if values.get("wheel_angle", 0.0) > limits.steer_angle_max + 1e-9:
            return fail("STEER_ANGLE_LIMIT",
                        "vehicle_id={} wheel angle exceeds limit".format(
                            vehicle_id))
        checks += 1
    if abs(timed.duration - duration) > _TIME_TOL:
        return fail("DURATION_MISMATCH", "timed duration differs")
    checks += 1
    return _audit_result(
        True, "OK",
        "GEOMETRY_ONLY common-time self-audit; S9 independent evaluation "
        "is still required",
        checks, metrics)