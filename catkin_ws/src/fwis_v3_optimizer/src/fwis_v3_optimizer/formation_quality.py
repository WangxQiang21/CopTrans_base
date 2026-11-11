"""S9 independent formation evaluator, candidate selection and publication gate.

This module intentionally recomputes every safety-relevant quantity from the
request, the continuous plan and the timed trajectory.  It does not consume
the S8 retiming peak report as evidence, does not reuse the S6 collision
precheck as a certificate, and never returns ``FEASIBLE`` unless request,
trajectory, rigidity, collision, kinematics, timing and publication sampling
all pass.

The evaluator is ROS-free.  ``scripts/formation_optimizer_node.py`` is the
thin ROS boundary that publishes a timed trajectory only after this module
returns ``FEASIBLE`` for the current request.
"""
from bisect import bisect_left, bisect_right
from dataclasses import dataclass, field
import hashlib
import math
from numbers import Integral, Real
from typing import Mapping, Optional, Tuple

import numpy as np

from .core import Action, Grid, Pose
from .formation import (
    FormationModel,
    FormationPose,
    FormationValidationError,
    VehicleOffset,
)
from .formation_collision import (
    FormationCollisionChecker,
    FormationState,
    SweepPolicy,
    VehicleShape,
)
from .formation_geometry import (
    FormationContinuousPlan,
    audit_continuous_plan,
)
from .formation_kinematics import (
    ACKERMANN,
    OBLIQUE,
    SPIN,
    KinematicLimits,
)
from .formation_retiming import (
    GEOMETRY_ONLY,
    FormationTimedPoint,
    FormationTimedTrajectory,
    FormationTimingLimits,
)
from .mode_switch_timing import mode_switch_event
from .quality import FootprintDistance
from .retiming import MotionLimits, wheel_configuration


FEASIBLE = "FEASIBLE"
INVALID = "INVALID"
REJECTED = "REJECTED"
GEOMETRY_ONLY_NOT_EXECUTABLE = "GEOMETRY_ONLY_NOT_EXECUTABLE"

INVALID_REQUEST = "INVALID_REQUEST"
INVALID_TIMED_TRAJECTORY = "INVALID_TIMED_TRAJECTORY"
START_FORMATION_COLLISION = "START_FORMATION_COLLISION"
GOAL_FORMATION_COLLISION = "GOAL_FORMATION_COLLISION"
FORMATION_NO_GEOMETRIC_PATH = "FORMATION_NO_GEOMETRIC_PATH"
FORMATION_SWEEP_COLLISION = "FORMATION_SWEEP_COLLISION"
KINEMATIC_INFEASIBLE = "KINEMATIC_INFEASIBLE"
MODE_SCHEDULE_INFEASIBLE = "MODE_SCHEDULE_INFEASIBLE"
FORMATION_NLP_INFEASIBLE = "FORMATION_NLP_INFEASIBLE"
FORMATION_RETIMING_INFEASIBLE = "FORMATION_RETIMING_INFEASIBLE"
FORMATION_RIGIDITY_VIOLATION = "FORMATION_RIGIDITY_VIOLATION"
PUBLICATION_SWEEP_INVALID = "PUBLICATION_SWEEP_INVALID"

_GEOMETRY_TOL = 1e-7
_TIME_TOL = 1e-9
_WHEEL_ANGLE_TOL = 1e-9
_DYNAMIC_TOL = 1e-6
_PROFILE_SAMPLES = 801
_SAFETY_FACTOR = 1.005

_DYNAMIC_LIMITS = (
    ("speed", "v_max"),
    ("acceleration", "a_max"),
    ("jerk", "jerk_max"),
    ("yaw_rate", "yaw_rate_max"),
    ("yaw_acceleration", "yaw_accel_max"),
    ("yaw_jerk", "yaw_jerk_max"),
    ("wheel_speed", "wheel_speed_max"),
    ("lateral_acceleration", "lateral_accel_max"),
)


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


def _integer(value, context, minimum=None):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise FormationValidationError(
            "INVALID_INTEGER context={} value={!r}".format(context, value))
    result = int(value)
    if minimum is not None and result < minimum:
        raise FormationValidationError(
            "INVALID_INTEGER context={} value={} minimum={}".format(
                context, result, minimum))
    return result


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _angle_error(first, second):
    return abs(_wrap(first - second))


def _rotate(psi, x, y):
    cosine, sine = math.cos(psi), math.sin(psi)
    return cosine * x - sine * y, sine * x + cosine * y


def _quintic(tau):
    value = tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))
    velocity = 30.0 * tau * tau * (1.0 + tau * (-2.0 + tau))
    acceleration = 60.0 * tau * (1.0 + tau * (-3.0 + 2.0 * tau))
    return value, velocity, acceleration


def _jerk_profile(tau):
    return 60.0 - 360.0 * tau + 360.0 * tau * tau


def _profile_taus():
    values = [index / float(_PROFILE_SAMPLES - 1)
              for index in range(_PROFILE_SAMPLES)]
    values.extend((0.5, 0.5 - math.sqrt(3.0) / 6.0,
                   0.5 + math.sqrt(3.0) / 6.0))
    return tuple(sorted(set(values)))


_TAUS = _profile_taus()


def grid_signature(grid):
    """Deterministic fingerprint of the exact occupancy grid used by S9."""
    if not isinstance(grid, Grid):
        raise FormationValidationError("INVALID_GRID expected core.Grid")
    digest = hashlib.sha256()
    digest.update(np.ascontiguousarray(grid.occ, dtype=np.bool_).tobytes())
    digest.update(np.asarray(
        (grid.res, grid.ox, grid.oy), dtype=np.float64).tobytes())
    digest.update(np.asarray((grid.height, grid.width),
                             dtype=np.int64).tobytes())
    return digest.hexdigest()


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
class FormationQualityConfig:
    """Independent evaluator limits and sampling policy."""

    timing_limits: FormationTimingLimits = field(
        default_factory=FormationTimingLimits)
    kinematic_limits: KinematicLimits = field(
        default_factory=KinematicLimits)
    collision_margin_m: float = 0.05
    sweep_spacing_m: float = 0.01
    publication_dt_s: float = 0.02
    max_samples_per_segment: int = 200000
    position_tolerance_m: float = 1e-6
    heading_tolerance_rad: float = 1e-6
    dynamics_tolerance_ratio: float = 1e-6
    wheel_angle_tolerance_rad: float = 1e-9
    expected_map_signature: Optional[str] = None

    def __post_init__(self):
        if not isinstance(self.timing_limits, FormationTimingLimits):
            raise FormationValidationError(
                "INVALID_FORMATION_TIMING_LIMITS")
        if not isinstance(self.kinematic_limits, KinematicLimits):
            raise FormationValidationError(
                "INVALID_KINEMATIC_LIMITS")
        for name in ("collision_margin_m", "sweep_spacing_m",
                     "publication_dt_s", "position_tolerance_m",
                     "heading_tolerance_rad", "dynamics_tolerance_ratio",
                     "wheel_angle_tolerance_rad"):
            object.__setattr__(self, name, _finite(
                getattr(self, name), "FormationQualityConfig." + name))
        object.__setattr__(self, "collision_margin_m", _finite(
            self.collision_margin_m, "collision_margin_m", nonnegative=True))
        for name in ("sweep_spacing_m", "publication_dt_s"):
            object.__setattr__(self, name, _finite(
                getattr(self, name), name, positive=True))
        object.__setattr__(self, "position_tolerance_m", _finite(
            self.position_tolerance_m, "position_tolerance_m",
            nonnegative=True))
        object.__setattr__(self, "heading_tolerance_rad", _finite(
            self.heading_tolerance_rad, "heading_tolerance_rad",
            nonnegative=True))
        object.__setattr__(self, "dynamics_tolerance_ratio", _finite(
            self.dynamics_tolerance_ratio, "dynamics_tolerance_ratio",
            nonnegative=True))
        object.__setattr__(self, "wheel_angle_tolerance_rad", _finite(
            self.wheel_angle_tolerance_rad, "wheel_angle_tolerance_rad",
            nonnegative=True))
        object.__setattr__(self, "max_samples_per_segment", _integer(
            self.max_samples_per_segment, "max_samples_per_segment",
            minimum=1))
        if self.expected_map_signature is not None and (
                not isinstance(self.expected_map_signature, str) or
                not self.expected_map_signature):
            raise FormationValidationError(
                "INVALID_MAP_SIGNATURE expected a non-empty string or None")


@dataclass(frozen=True)
class FormationEvaluationFailure:
    """Structured first failure; diagnostic fields are serialisable."""

    code: str
    field: str = ""
    vehicle_id: Optional[int] = None
    other_vehicle_id: Optional[int] = None
    pair_id: Optional[str] = None
    block_index: Optional[int] = None
    segment_index: Optional[int] = None
    sample_index: Optional[int] = None
    time_s: Optional[float] = None
    station: Optional[float] = None
    constraint: str = ""
    max_error: Optional[float] = None
    detail: str = ""

    def as_dict(self):
        return {
            "code": self.code,
            "field": self.field,
            "vehicle_id": self.vehicle_id,
            "other_vehicle_id": self.other_vehicle_id,
            "pair_id": self.pair_id,
            "block_index": self.block_index,
            "segment_index": self.segment_index,
            "sample_index": self.sample_index,
            "time_s": self.time_s,
            "station": self.station,
            "constraint": self.constraint,
            "max_error": self.max_error,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class FormationEvaluation:
    """Final S9 result.  ``FEASIBLE`` is the only executable status."""

    plan_id: int
    formation_id: str
    status: str
    feasible: bool
    reason: str
    detail: str
    failure: Optional[FormationEvaluationFailure]
    metrics: Mapping
    checks: int

    def __post_init__(self):
        if self.status not in (FEASIBLE, INVALID, REJECTED,
                               GEOMETRY_ONLY_NOT_EXECUTABLE):
            raise FormationValidationError(
                "INVALID_EVALUATION_STATUS {}".format(self.status))
        if self.feasible != (self.status == FEASIBLE):
            raise FormationValidationError(
                "INCONSISTENT_FEASIBLE_FLAG")
        if self.feasible and self.failure is not None:
            raise FormationValidationError(
                "FEASIBLE_WITH_FAILURE")
        if not self.feasible and self.failure is None:
            raise FormationValidationError(
                "NONFEASIBLE_WITHOUT_FAILURE")

    @property
    def certification(self):
        return FEASIBLE if self.feasible else "NONE"

    @property
    def executable(self):
        return bool(self.feasible)

    @property
    def trajectory_published(self):
        return False

    @property
    def independent_audit(self):
        return True

    @property
    def collision_certificate(self):
        return bool(self.feasible)

    def as_status(self):
        return {
            "schema": "formation_evaluation/v1",
            "plan_id": self.plan_id,
            "formation_id": self.formation_id,
            "status": self.status,
            "feasible": bool(self.feasible),
            "reason": self.reason,
            "detail": self.detail,
            "certification": self.certification,
            "executable": self.executable,
            "trajectory_published": False,
            "independent_audit": True,
            "collision_certificate": bool(self.feasible),
            "failure": (None if self.failure is None
                        else self.failure.as_dict()),
            "metrics": dict(self.metrics),
            "checks": int(self.checks),
        }


@dataclass(frozen=True)
class FormationTimedCandidate:
    """One S6/S7 plan with its S8 timed trajectory, before S9 selection."""

    candidate_id: int
    plan: FormationContinuousPlan
    timed: FormationTimedTrajectory

    def __post_init__(self):
        if isinstance(self.candidate_id, bool) or not isinstance(
                self.candidate_id, int) or self.candidate_id <= 0:
            raise FormationValidationError(
                "INVALID_CANDIDATE_ID {}".format(self.candidate_id))
        if not isinstance(self.plan, FormationContinuousPlan):
            raise FormationValidationError(
                "INVALID_CANDIDATE_PLAN")
        if not isinstance(self.timed, FormationTimedTrajectory):
            raise FormationValidationError(
                "INVALID_CANDIDATE_TRAJECTORY")


@dataclass(frozen=True)
class FormationSelection:
    """Evaluated candidates and the single feasible selection."""

    status: str
    selected_candidate_id: Optional[int]
    selected_plan: Optional[FormationContinuousPlan]
    selected_timed: Optional[FormationTimedTrajectory]
    selected_evaluation: Optional[FormationEvaluation]
    evaluations: Tuple[FormationEvaluation, ...]
    rejections: Tuple[dict, ...]
    candidate_count: int
    feasible_count: int

    def as_status(self):
        return {
            "schema": "formation_selection/v1",
            "status": self.status,
            "selected_candidate_id": self.selected_candidate_id,
            "T_end": (None if self.selected_timed is None
                      else self.selected_timed.duration),
            "candidate_count": int(self.candidate_count),
            "feasible_count": int(self.feasible_count),
            "evaluations": [item.as_status() for item in self.evaluations],
            "rejections": [dict(item) for item in self.rejections],
            "executable": self.status == FEASIBLE,
            "trajectory_published": False,
        }


@dataclass(frozen=True)
class PublicationDecision:
    """Transactional publication decision for one request."""

    publish: bool
    clear: bool
    plan_id: Optional[int]
    trajectory: Optional[FormationTimedTrajectory]
    status: str
    reason: str
    detail: str

    def as_status(self):
        return {
            "publish": bool(self.publish),
            "clear": bool(self.clear),
            "plan_id": self.plan_id,
            "status": self.status,
            "reason": self.reason,
            "detail": self.detail,
            "trajectory_published": bool(self.publish),
        }


class FormationPublicationGate:
    """Only the current plan's FEASIBLE trajectory may be published.

    A new request immediately invalidates the previous publication.  A failed
    or stale result cannot leave an old successful trajectory visible.
    """

    def __init__(self):
        self._request_plan_id = None
        self._published_plan_id = None
        self._version = 0

    @property
    def request_plan_id(self):
        return self._request_plan_id

    @property
    def published_plan_id(self):
        return self._published_plan_id

    @property
    def version(self):
        return self._version

    def begin_request(self, plan_id):
        plan_id = _integer(plan_id, "plan_id", minimum=0)
        self._request_plan_id = plan_id
        self._published_plan_id = None
        self._version += 1
        return PublicationDecision(
            False, True, plan_id, None, "CLEAR",
            "NEW_REQUEST",
            "new request {} supersedes any previous publication".format(
                plan_id))

    def offer(self, plan_id, timed, evaluation):
        plan_id = _integer(plan_id, "plan_id", minimum=0)
        if plan_id != self._request_plan_id:
            return PublicationDecision(
                False, False, plan_id, None, "STALE_PLAN",
                "STALE_PLAN",
                "plan {} is not the current request {}".format(
                    plan_id, self._request_plan_id))
        if not isinstance(evaluation, FormationEvaluation):
            return self.fail(plan_id, "INVALID_EVALUATION",
                             "offer requires a FormationEvaluation")
        if not evaluation.feasible:
            return self.fail(plan_id, evaluation.reason, evaluation.detail)
        if not isinstance(timed, FormationTimedTrajectory):
            return self.fail(plan_id, "INVALID_TRAJECTORY",
                             "feasible evaluation has no timed trajectory")
        if (timed.plan_id != plan_id or
                timed.formation_id != evaluation.formation_id):
            return self.fail(plan_id, "PLAN_ID_MISMATCH",
                             "trajectory identity differs from evaluation")
        if self._published_plan_id == plan_id:
            return PublicationDecision(
                True, False, plan_id, timed, "PUBLISH",
                "OK", "refreshing the current plan")
        self._published_plan_id = plan_id
        return PublicationDecision(
            True, False, plan_id, timed, "PUBLISH", "OK",
            "FEASIBLE independent evaluation for the current request")

    def fail(self, plan_id, reason, detail):
        plan_id = _integer(plan_id, "plan_id", minimum=0)
        clear = plan_id == self._request_plan_id
        if clear:
            self._published_plan_id = None
        return PublicationDecision(
            False, clear, plan_id, None, "REJECTED", reason, detail)

    def invalidate(self):
        self._published_plan_id = None
        self._version += 1
        return PublicationDecision(
            False, True, self._request_plan_id, None, "CLEAR",
            "INVALIDATED", "publication state cleared")


class _EvalError(Exception):
    def __init__(self, code, detail, **fields):
        super().__init__(detail)
        self.code = code
        self.detail = detail
        self.fields = fields


def _status_for_code(code):
    if code in (INVALID_REQUEST, INVALID_TIMED_TRAJECTORY):
        return INVALID
    if code == FORMATION_RETIMING_INFEASIBLE:
        return GEOMETRY_ONLY_NOT_EXECUTABLE
    return REJECTED


def _failure_evaluation(plan_id, formation_id, code, detail,
                        metrics=None, checks=0, **fields):
    return FormationEvaluation(
        plan_id=plan_id,
        formation_id=formation_id,
        status=_status_for_code(code),
        feasible=False,
        reason=code,
        detail=detail,
        failure=FormationEvaluationFailure(code=code, detail=detail,
                                           **fields),
        metrics=dict(metrics or {}),
        checks=int(checks))


def _identity(value):
    plan_id = getattr(value, "plan_id", -1)
    formation_id = getattr(value, "formation_id", "")
    if isinstance(plan_id, bool) or not isinstance(plan_id, int):
        plan_id = -1
    if not isinstance(formation_id, str):
        formation_id = ""
    return plan_id, formation_id


def _require_attr(owner, name, context, code=INVALID_REQUEST):
    try:
        return getattr(owner, name)
    except AttributeError as exc:
        raise _EvalError(code, "{} is missing field {!r}".format(
            context, name), field=name) from exc


def _check_heading_map(value, vehicle_ids, context, code=INVALID_REQUEST):
    if not isinstance(value, Mapping):
        raise _EvalError(code, "{} must be a vehicle_id -> heading map".format(
            context), field=context)
    try:
        items = dict(value)
    except (TypeError, ValueError) as exc:
        raise _EvalError(code, "{} is not a heading map".format(context),
                         field=context) from exc
    if set(items) != set(vehicle_ids):
        raise _EvalError(code, "{} vehicle ids differ from the formation".format(
            context), field=context)
    result = {}
    for vehicle_id in vehicle_ids:
        raw = items[vehicle_id]
        if isinstance(raw, bool) or not isinstance(raw, Real):
            raise _EvalError(code, "{}[{}] is not numeric".format(
                context, vehicle_id), field=context, vehicle_id=vehicle_id)
        number = float(raw)
        if not math.isfinite(number):
            raise _EvalError(code, "{}[{}] is not finite".format(
                context, vehicle_id), field=context, vehicle_id=vehicle_id)
        result[vehicle_id] = number
    return result


def _check_heading_pairs(value, vehicle_ids, context,
                         code=INVALID_REQUEST):
    if isinstance(value, Mapping):
        return _check_heading_map(value, vehicle_ids, context, code)
    try:
        pairs = list(value)
    except TypeError as exc:
        raise _EvalError(code, "{} must be heading pairs or a map".format(
            context), field=context) from exc
    result = {}
    for index, pair in enumerate(pairs):
        try:
            vehicle_id, heading = pair
        except (TypeError, ValueError) as exc:
            raise _EvalError(code, "{}[{}] is not a (id, heading) pair".format(
                context, index), field=context) from exc
        if isinstance(vehicle_id, bool) or not isinstance(
                vehicle_id, Integral):
            raise _EvalError(code, "{}[{}] vehicle id is invalid".format(
                context, index), field=context)
        vehicle_id = int(vehicle_id)
        if vehicle_id in result:
            raise _EvalError(code, "{} has a duplicate vehicle id".format(
                context), field=context, vehicle_id=vehicle_id)
        if isinstance(heading, bool) or not isinstance(heading, Real):
            raise _EvalError(code, "{}[{}] heading is not numeric".format(
                context, index), field=context, vehicle_id=vehicle_id)
        heading = float(heading)
        if not math.isfinite(heading):
            raise _EvalError(code, "{}[{}] heading is not finite".format(
                context, index), field=context, vehicle_id=vehicle_id)
        result[vehicle_id] = heading
    if set(result) != set(vehicle_ids):
        raise _EvalError(code, "{} vehicle ids differ from the formation".format(
            context), field=context)
    return result


def _check_bool_map(value, vehicle_ids, context, code=INVALID_REQUEST):
    if not isinstance(value, Mapping):
        raise _EvalError(code, "{} must be a vehicle_id -> bool map".format(
            context), field=context)
    try:
        items = dict(value)
    except (TypeError, ValueError) as exc:
        raise _EvalError(code, "{} is not a bool map".format(context),
                         field=context) from exc
    if set(items) != set(vehicle_ids):
        raise _EvalError(code, "{} vehicle ids differ from the formation".format(
            context), field=context)
    if any(not isinstance(items[vehicle_id], bool)
           for vehicle_id in vehicle_ids):
        raise _EvalError(code, "{} values must be bool".format(context),
                         field=context)
    return {vehicle_id: items[vehicle_id] for vehicle_id in vehicle_ids}


def _check_stamp(snapshot):
    code = INVALID_REQUEST
    raw = getattr(snapshot, "header_stamp", None)
    if raw is None:
        header = getattr(snapshot, "header", None)
        stamp = getattr(header, "stamp", None) if header is not None else None
        if stamp is None:
            raise _EvalError(code, "request has no header stamp",
                             field="header_stamp")
        raw = (getattr(stamp, "secs", None), getattr(stamp, "nsecs", None))
    try:
        secs, nsecs = raw
    except (TypeError, ValueError) as exc:
        raise _EvalError(code, "header stamp must be (secs, nsecs)",
                         field="header_stamp") from exc
    for name, value in (("secs", secs), ("nsecs", nsecs)):
        if isinstance(value, bool) or not isinstance(value, Integral):
            raise _EvalError(code, "header stamp {} is not an integer".format(
                name), field="header_stamp")
    secs, nsecs = int(secs), int(nsecs)
    if secs < 0 or nsecs < 0 or nsecs >= 1000000000:
        raise _EvalError(code, "header stamp is outside ROS time",
                         field="header_stamp")
    if secs == 0 and nsecs == 0:
        raise _EvalError(code, "header stamp must be non-zero",
                         field="header_stamp")
    return secs, nsecs


def _pose_equal(first, second, position_tolerance, heading_tolerance):
    return (math.hypot(first.x - second.x, first.y - second.y) <=
            position_tolerance and
            _angle_error(first.psi, second.psi) <= heading_tolerance)


def _angle_difference(a, b):
    return abs(_wrap(float(a) - float(b)))


def _segment_wheel_data(segment, motion_limits):
    angles, factors = wheel_configuration(
        Action(segment.mode, 0.0, segment.control), motion_limits)
    return (tuple(float(value) for value in angles),
            tuple(float(value) for value in factors))


def _formation_derivatives(control, station):
    """Independent first three lambda-derivatives of q_F."""
    length = control.parameter_length
    if control.geometry_type == "translation":
        return (
            ((control.end_qf.x - control.start_qf.x) / length,
             (control.end_qf.y - control.start_qf.y) / length,
             0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            0.0)
    slope = control.delta_psi / length
    psi = control.start_qf.psi + control.delta_psi * station / length
    first_x, first_y = _rotate(psi, -control.pivot_y, control.pivot_x)
    second_x, second_y = _rotate(psi, control.pivot_x, control.pivot_y)
    return (
        (-slope * first_x, -slope * first_y, slope),
        (slope * slope * second_x, slope * slope * second_y, 0.0),
        (slope ** 3 * first_x, slope ** 3 * first_y, 0.0),
        slope)


def _vehicle_derivatives(control, offset, station):
    q_first, q_second, q_third, slope = _formation_derivatives(
        control, station)
    psi = control.start_qf.psi + control.delta_psi * station / (
        control.parameter_length)
    rotated_x, rotated_y = _rotate(psi, offset.x, offset.y)
    rotated_jx, rotated_jy = _rotate(psi, -offset.y, offset.x)
    first = (q_first[0] + slope * rotated_jx,
             q_first[1] + slope * rotated_jy)
    second = (q_second[0] - slope * slope * rotated_x,
              q_second[1] - slope * slope * rotated_y)
    third = (q_third[0] - slope ** 3 * rotated_jx,
             q_third[1] - slope ** 3 * rotated_jy)
    return first, second, third


def _dynamic_values(block, segment, offset, duration, tau, motion_limits,
                    wheel_factors=None):
    progress, velocity, acceleration = _quintic(tau)
    station = block.parameter_length * progress
    lambda_dot = block.parameter_length * velocity / duration
    lambda_ddot = block.parameter_length * acceleration / (duration * duration)
    lambda_dddot = block.parameter_length * _jerk_profile(tau) / (
        duration * duration * duration)
    first, second, third = _vehicle_derivatives(
        block.control, offset, station)
    p_dot = (first[0] * lambda_dot, first[1] * lambda_dot)
    p_ddot = (
        second[0] * lambda_dot * lambda_dot + first[0] * lambda_ddot,
        second[1] * lambda_dot * lambda_dot + first[1] * lambda_ddot)
    p_dddot = (
        third[0] * lambda_dot ** 3 +
        3.0 * second[0] * lambda_dot * lambda_ddot +
        first[0] * lambda_dddot,
        third[1] * lambda_dot ** 3 +
        3.0 * second[1] * lambda_dot * lambda_ddot +
        first[1] * lambda_dddot)
    speed = math.hypot(p_dot[0], p_dot[1])
    theta_rate = (segment.theta_end - segment.theta_start) / (
        block.parameter_length)
    yaw_rate = abs(theta_rate * lambda_dot)
    yaw_acceleration = abs(theta_rate * lambda_ddot)
    yaw_jerk = abs(theta_rate * lambda_dddot)
    if wheel_factors is None:
        _, wheel_factors = _segment_wheel_data(segment, motion_limits)
    wheel_speed = max(wheel_factors) * (
        yaw_rate if segment.mode == SPIN else speed)
    lateral = (abs(segment.control) * speed * speed
               if segment.mode == ACKERMANN else 0.0)
    heading = block.body_heading_at(segment.vehicle_id, station)
    if segment.mode == SPIN:
        signed_speed = 0.0
        lateral_speed = 0.0
    else:
        signed_speed = (p_dot[0] * math.cos(heading) +
                        p_dot[1] * math.sin(heading))
        lateral_speed = (-p_dot[0] * math.sin(heading) +
                         p_dot[1] * math.cos(heading))
    return {
        "speed_signed": signed_speed,
        "lateral_speed_signed": lateral_speed,
        "speed": speed,
        "acceleration": math.hypot(p_ddot[0], p_ddot[1]),
        "jerk": math.hypot(p_dddot[0], p_dddot[1]),
        "yaw_rate": yaw_rate,
        "yaw_acceleration": yaw_acceleration,
        "yaw_jerk": yaw_jerk,
        "wheel_speed": wheel_speed,
        "lateral_acceleration": lateral,
        "station": station,
    }


def _close(actual, expected, ratio, absolute=_DYNAMIC_TOL):
    scale = max(abs(float(expected)), 1.0)
    return abs(float(actual) - float(expected)) <= (
        absolute + float(ratio) * scale)


@dataclass(frozen=True)
class _PhaseInfo:
    index: int
    kind: str
    block_index: int
    from_block_index: Optional[int]
    t0_s: float
    duration_s: float


class _IndependentEvaluator:
    def __init__(self, snapshot, plan, timed, grid, config):
        self.snapshot = snapshot
        self.plan = plan
        self.timed = timed
        self.grid = grid
        self.config = config
        self.plan_id, self.formation_id = _identity(snapshot)
        self.metrics = {}
        self.checks = 0
        self.model = None
        self.vehicle_ids = ()
        self.shapes = {}
        self.offset_map = {}
        self.blocks = ()
        self.phases = ()
        self._wheels = {}
        self._times = ()
        self._expected_qf = []
        self._expected_headings = []
        self._expected_block = []
        self._expected_station = []
        self._expected_stationary = []
        self._sample_phase = []
        self._sample_peak = {
            "speed": 0.0, "acceleration": 0.0, "jerk": 0.0,
            "yaw_rate": 0.0, "yaw_acceleration": 0.0, "yaw_jerk": 0.0,
            "wheel_speed": 0.0, "lateral_acceleration": 0.0,
        }
        self._max_rigidity = 0.0
        self._map_signature = None

    def _fail(self, code, detail, **fields):
        raise _EvalError(code, detail, **fields)

    def _count(self, amount=1):
        self.checks += int(amount)

    def run(self):
        try:
            self._check_grid_and_signature()
            self._check_request()
            self._check_plan()
            self._check_timed_contract()
            self._parse_phases()
            self._assign_samples()
            self._check_samples()
            self._check_mode_schedule()
            self._check_dynamics_peaks()
            self._check_static_collisions()
            self._check_sweeps()
            self._check_publication()
            return self._success()
        except _EvalError as exc:
            return _failure_evaluation(
                self.plan_id, self.formation_id, exc.code, exc.detail,
                metrics=self.metrics, checks=self.checks, **exc.fields)
        except FormationValidationError as exc:
            return _failure_evaluation(
                self.plan_id, self.formation_id, INVALID_TIMED_TRAJECTORY,
                "independent evaluation input became invalid: {}".format(exc),
                metrics=self.metrics, checks=self.checks)

    def _check_grid_and_signature(self):
        if not isinstance(self.grid, Grid):
            self._fail(INVALID_REQUEST,
                       "an exact occupancy map is required",
                       field="map")
        self._map_signature = grid_signature(self.grid)
        self.metrics["map_signature"] = self._map_signature
        expected = self.config.expected_map_signature
        if expected is not None and expected != self._map_signature:
            self._fail(INVALID_REQUEST,
                       "map signature differs from the expected grid",
                       field="map_signature")
        self._count()

    def _check_request(self):
        snapshot = self.snapshot
        plan_id = _require_attr(snapshot, "plan_id", "request")
        if isinstance(plan_id, bool) or not isinstance(plan_id, int) or (
                plan_id < 0 or plan_id > 2 ** 32 - 1):
            self._fail(INVALID_REQUEST, "plan_id is outside uint32",
                       field="plan_id")
        formation_id = _require_attr(snapshot, "formation_id", "request")
        if not isinstance(formation_id, str) or not formation_id:
            self._fail(INVALID_REQUEST, "formation_id must be non-empty",
                       field="formation_id")
        self.plan_id, self.formation_id = plan_id, formation_id
        model = _require_attr(snapshot, "model", "request")
        if not isinstance(model, FormationModel):
            self._fail(INVALID_REQUEST, "request.model must be FormationModel",
                       field="model")
        self.model = model
        self.vehicle_ids = tuple(model.vehicle_ids)
        self.offset_map = {offset.vehicle_id: offset
                           for offset in model.offsets}
        start = _require_attr(snapshot, "start", "request")
        goal = _require_attr(snapshot, "goal", "request")
        if not isinstance(start, FormationPose):
            self._fail(INVALID_REQUEST, "request.start must be FormationPose",
                       field="start")
        if not isinstance(goal, FormationPose):
            self._fail(INVALID_REQUEST, "request.goal must be FormationPose",
                       field="goal")
        self._check_stamp_runtime = _check_stamp(snapshot)
        allow_reverse = _require_attr(snapshot, "allow_reverse", "request")
        if not isinstance(allow_reverse, bool):
            self._fail(INVALID_REQUEST, "allow_reverse must be bool",
                       field="allow_reverse")
        self.start_headings = _check_heading_map(
            _require_attr(snapshot, "start_headings", "request"),
            self.vehicle_ids, "start_headings")
        self.goal_headings = _check_heading_map(
            _require_attr(snapshot, "goal_headings", "request"),
            self.vehicle_ids, "goal_headings")
        self.goal_heading_free = _check_bool_map(
            _require_attr(snapshot, "goal_heading_free", "request"),
            self.vehicle_ids, "goal_heading_free")
        shapes = _require_attr(snapshot, "vehicle_shapes", "request")
        if not isinstance(shapes, Mapping):
            self._fail(INVALID_REQUEST,
                       "vehicle_shapes must be a vehicle_id -> VehicleShape map",
                       field="vehicle_shapes")
        shapes = dict(shapes)
        if set(shapes) != set(self.vehicle_ids):
            self._fail(INVALID_REQUEST,
                       "vehicle_shapes ids differ from the formation",
                       field="vehicle_shapes")
        for vehicle_id in self.vehicle_ids:
            if not isinstance(shapes[vehicle_id], VehicleShape):
                self._fail(INVALID_REQUEST,
                           "vehicle shape is not a VehicleShape",
                           field="vehicle_shapes", vehicle_id=vehicle_id)
        self.shapes = shapes
        self._count(len(self.vehicle_ids) + 8)

    def _check_plan(self):
        plan = self.plan
        if not isinstance(plan, FormationContinuousPlan):
            self._fail(INVALID_REQUEST,
                       "plan must be a FormationContinuousPlan",
                       field="plan")
        if plan.plan_id != self.plan_id:
            self._fail(INVALID_REQUEST, "plan_id differs from the request",
                       field="plan_id")
        if plan.formation_id != self.formation_id:
            self._fail(INVALID_REQUEST,
                       "formation_id differs from the request",
                       field="formation_id")
        if tuple(plan.offsets) != tuple(self.model.offsets):
            self._fail(INVALID_REQUEST,
                       "plan offsets differ from the fixed request offsets",
                       field="offsets")
        start = _require_attr(self.snapshot, "start", "request")
        goal = _require_attr(self.snapshot, "goal", "request")
        if not _pose_equal(plan.start, start, _GEOMETRY_TOL, _GEOMETRY_TOL):
            self._fail(INVALID_REQUEST, "plan start differs from the request",
                       field="start")
        if not _pose_equal(plan.goal, goal, _GEOMETRY_TOL, _GEOMETRY_TOL):
            self._fail(INVALID_REQUEST, "plan goal differs from the request",
                       field="goal")
        if dict(plan.goal_heading_free) != self.goal_heading_free:
            self._fail(INVALID_REQUEST,
                       "plan heading-free policy differs from the request",
                       field="goal_heading_free")
        if bool(plan.allow_reverse) != bool(
                getattr(self.snapshot, "allow_reverse", False)):
            self._fail(INVALID_REQUEST,
                       "plan reverse policy differs from the request",
                       field="allow_reverse")
        plan_start_headings = _check_heading_pairs(
            plan.vehicle_start_headings, self.vehicle_ids,
            "plan start headings", INVALID_REQUEST)
        plan_goal_headings = _check_heading_pairs(
            plan.vehicle_goal_headings, self.vehicle_ids,
            "plan goal headings", INVALID_REQUEST)
        if any(_angle_error(plan_start_headings[vehicle_id],
                            self.start_headings[vehicle_id]) > _GEOMETRY_TOL
               for vehicle_id in self.vehicle_ids):
            self._fail(INVALID_REQUEST,
                       "plan start headings differ from the request",
                       field="start_headings")
        if any(_angle_error(plan_goal_headings[vehicle_id],
                            self.goal_headings[vehicle_id]) > _GEOMETRY_TOL
               for vehicle_id in self.vehicle_ids):
            self._fail(INVALID_REQUEST,
                       "plan goal headings differ from the request",
                       field="goal_headings")
        blocks = tuple(plan.blocks)
        if not blocks:
            self._fail(INVALID_REQUEST, "plan has no blocks", field="blocks")
        indices = [block.block_index for block in blocks]
        if indices != list(range(len(blocks))):
            self._fail(INVALID_REQUEST,
                       "block indices are not 0..N-1", field="blocks")
        self.blocks = blocks
        for block in blocks:
            if tuple(block.vehicle_ids) != self.vehicle_ids:
                self._fail(INVALID_REQUEST,
                           "block vehicle ids differ from the formation",
                           field="blocks", block_index=block.block_index)
            if block.parameter_length <= 0.0:
                self._fail(INVALID_REQUEST,
                           "block parameter_length must be positive",
                           field="parameter_length",
                           block_index=block.block_index)
        audit = audit_continuous_plan(
            plan, self.snapshot, limits=self.config.kinematic_limits,
            grid=None)
        self.checks += int(audit.checks)
        if not audit.success:
            reason = audit.reason
            if reason in (
                    "PLAN_ID_MISMATCH", "FORMATION_ID_MISMATCH",
                    "OFFSET_VERSION_MISMATCH", "PLAN_START_MISMATCH",
                    "PLAN_GOAL_MISMATCH", "ALLOW_REVERSE_MISMATCH",
                    "GOAL_HEADING_FREE_MISMATCH",
                    "PLAN_START_HEADING_MISMATCH",
                    "PLAN_GOAL_HEADING_MISMATCH",
                    "BLOCK_HEADING_BOUNDARY_MISMATCH",
                    "GOAL_HEADING_MISMATCH",
                    "TERMINAL_ENDPOINT_MISMATCH",
                    "INVALID_BLOCK_GEOMETRY", "INVALID_CONTINUOUS_PLAN",
                    "BLOCK_VEHICLE_SET_MISMATCH"):
                code = INVALID_REQUEST
            elif reason == "FORMATION_RIGIDITY_VIOLATION":
                code = FORMATION_RIGIDITY_VIOLATION
            elif "REVERSE" in reason or "MODE" in reason:
                code = MODE_SCHEDULE_INFEASIBLE
            else:
                code = KINEMATIC_INFEASIBLE
            self._fail(code, "continuous plan audit: {} {}".format(
                reason, audit.detail), field=reason.replace(" ", "_"))
        self.metrics["plan_checks"] = int(audit.checks)
        self.metrics["plan_max_rigidity_error_m"] = float(
            audit.max_rigidity_error_m)

        motion_limits = _motion_limits(self.config.timing_limits)
        max_curvature = 0.0
        max_oblique = 0.0
        mode_switches = 0
        previous_modes = None
        current_headings = dict(self.start_headings)
        for block in blocks:
            self._check_block_kinematics(
                block, motion_limits, current_headings)
            modes = {segment.vehicle_id: segment.mode
                     for segment in block.vehicle_mode_segments}
            if previous_modes is not None:
                mode_switches += sum(
                    1 for vehicle_id in self.vehicle_ids
                    if modes[vehicle_id] != previous_modes[vehicle_id])
            previous_modes = modes
            for segment in block.vehicle_mode_segments:
                if segment.mode == ACKERMANN:
                    max_curvature = max(max_curvature, abs(segment.control))
                elif segment.mode == OBLIQUE:
                    max_oblique = max(max_oblique, abs(segment.control))
            # Headings are continuous across every block boundary.
            for segment in block.vehicle_mode_segments:
                if _angle_difference(
                        segment.theta_start,
                        current_headings[segment.vehicle_id]) > (
                        self.config.heading_tolerance_rad + _GEOMETRY_TOL):
                    self._fail(KINEMATIC_INFEASIBLE,
                               "block theta_start differs from the previous "
                               "block end",
                               field="theta_start",
                               vehicle_id=segment.vehicle_id,
                               block_index=block.block_index)
                current_headings[segment.vehicle_id] = segment.theta_end
            self._count(len(block.vehicle_mode_segments))
        self.metrics["mode_switch_count"] = int(mode_switches)
        self.metrics["peak_A_curvature_1pm"] = float(max_curvature)
        self.metrics["peak_O_phi_rad"] = float(max_oblique)
        # Initial and terminal request headings.
        first = blocks[0]
        for segment in first.vehicle_mode_segments:
            if _angle_difference(
                    segment.theta_start,
                    self.start_headings[segment.vehicle_id]) > (
                    self.config.heading_tolerance_rad + _GEOMETRY_TOL):
                self._fail(INVALID_REQUEST,
                           "first block does not start at the request heading",
                           field="start_headings",
                           vehicle_id=segment.vehicle_id)
        last = blocks[-1]
        for segment in last.vehicle_mode_segments:
            if not self.goal_heading_free[segment.vehicle_id] and (
                    _angle_difference(
                        segment.theta_end,
                        self.goal_headings[segment.vehicle_id]) > (
                        self.config.heading_tolerance_rad + _GEOMETRY_TOL)):
                self._fail(INVALID_REQUEST,
                           "last block does not end at the request heading",
                           field="goal_headings",
                           vehicle_id=segment.vehicle_id)
        endpoint = last.control.qf_at(last.parameter_length)
        if not _pose_equal(endpoint, goal, _GEOMETRY_TOL, _GEOMETRY_TOL):
            self._fail(INVALID_REQUEST,
                       "plan geometry does not reach the request goal",
                       field="goal")

    def _check_block_kinematics(self, block, motion_limits, current_headings):
        length = block.parameter_length
        start_centers = self.model.centers(block.control.start_qf)
        end_centers = self.model.centers(block.control.end_qf)
        expected_theta_rate = (
            block.control.delta_psi / length
            if block.geometry_type == "rotation" else 0.0)
        for segment in block.vehicle_mode_segments:
            vehicle_id = segment.vehicle_id
            angles, factors = _segment_wheel_data(segment, motion_limits)
            self._wheels[(block.block_index, vehicle_id)] = (angles, factors)
            max_angle = max(abs(value) for value in angles)
            if max_angle > self.config.timing_limits.steer_angle_max + 1e-9:
                self._fail(KINEMATIC_INFEASIBLE,
                           "wheel angle exceeds steer_angle_max",
                           field="steer_angle",
                           vehicle_id=vehicle_id,
                           block_index=block.block_index,
                           max_error=max_angle,
                           constraint="steer_angle_max")
            if segment.mode == ACKERMANN:
                if abs(segment.control) > (
                        1.0 / self.config.timing_limits.r_min_ackermann +
                        1e-9):
                    self._fail(
                        KINEMATIC_INFEASIBLE,
                        "A curvature exceeds 1/r_min",
                        field="curvature", vehicle_id=vehicle_id,
                        block_index=block.block_index,
                        max_error=abs(segment.control),
                        constraint="r_min_ackermann")
            elif segment.mode == OBLIQUE:
                if abs(segment.control) > (
                        self.config.kinematic_limits.oblique_angle_max +
                        1e-9):
                    self._fail(
                        KINEMATIC_INFEASIBLE,
                        "O oblique angle exceeds the limit",
                        field="oblique_angle", vehicle_id=vehicle_id,
                        block_index=block.block_index,
                        max_error=abs(segment.control),
                        constraint="oblique_angle_max")
                if abs(segment.theta_end - segment.theta_start) > (
                        self.config.heading_tolerance_rad + _GEOMETRY_TOL):
                    self._fail(
                        KINEMATIC_INFEASIBLE,
                        "O body heading must stay constant",
                        field="theta", vehicle_id=vehicle_id,
                        block_index=block.block_index,
                        max_error=abs(
                            segment.theta_end - segment.theta_start))
            elif segment.mode == SPIN:
                if abs(segment.control) > 1e-12:
                    self._fail(
                        KINEMATIC_INFEASIBLE,
                        "Spin control must be zero",
                        field="control", vehicle_id=vehicle_id,
                        block_index=block.block_index,
                        max_error=abs(segment.control))
                if segment.travel_distance > 1e-9:
                    self._fail(
                        KINEMATIC_INFEASIBLE,
                        "Spin centre travel must be zero",
                        field="travel_distance", vehicle_id=vehicle_id,
                        block_index=block.block_index,
                        max_error=segment.travel_distance)
                if math.hypot(
                        segment.end_center_x - segment.start_center_x,
                        segment.end_center_y - segment.start_center_y) > (
                        self.config.position_tolerance_m + _GEOMETRY_TOL):
                    self._fail(
                        KINEMATIC_INFEASIBLE,
                        "Spin centre moves in the plan",
                        field="center", vehicle_id=vehicle_id,
                        block_index=block.block_index)
            # Independent first-order differential contract.  The centre
            # velocity is derived from q_F, the offset and the pivot; its
            # projection on the body heading gives the signed speed, and the
            # curvature is theta'/v rather than a direction label.
            control = block.control
            if block.geometry_type == "translation":
                velocity_x = (
                    control.end_qf.x - control.start_qf.x) / length
                velocity_y = (
                    control.end_qf.y - control.start_qf.y) / length
            else:
                slope = control.delta_psi / length
                offset = self.offset_map[vehicle_id]
                rx = offset.x - control.pivot_x
                ry = offset.y - control.pivot_y
                local_vx = -slope * ry
                local_vy = slope * rx
                cosine = math.cos(control.start_qf.psi)
                sine = math.sin(control.start_qf.psi)
                velocity_x = cosine * local_vx - sine * local_vy
                velocity_y = sine * local_vx + cosine * local_vy
            signed_speed = (
                velocity_x * math.cos(segment.theta_start) +
                velocity_y * math.sin(segment.theta_start))
            lateral_speed = (
                -velocity_x * math.sin(segment.theta_start) +
                velocity_y * math.cos(segment.theta_start))
            if (segment.mode in (ACKERMANN, OBLIQUE) and
                    abs(signed_speed) > _GEOMETRY_TOL):
                expected_direction = 1 if signed_speed > 0.0 else -1
                if segment.direction != expected_direction:
                    self._fail(
                        KINEMATIC_INFEASIBLE,
                        "centre velocity sign contradicts segment direction",
                        field="direction", vehicle_id=vehicle_id,
                        block_index=block.block_index,
                        max_error=abs(signed_speed))
            if segment.mode == ACKERMANN:
                if abs(lateral_speed) > (
                        self.config.position_tolerance_m + _GEOMETRY_TOL):
                    self._fail(
                        KINEMATIC_INFEASIBLE,
                        "A centre velocity has a lateral component",
                        field="lateral_speed", vehicle_id=vehicle_id,
                        block_index=block.block_index,
                        max_error=abs(lateral_speed))
                if (block.geometry_type == "rotation" and
                        abs(signed_speed) > _GEOMETRY_TOL):
                    curvature = (
                        (segment.theta_end - segment.theta_start) / length
                    ) / signed_speed
                    if abs(curvature - segment.control) > (
                            self.config.heading_tolerance_rad +
                            1e-9 * max(1.0, abs(curvature))):
                        self._fail(
                            KINEMATIC_INFEASIBLE,
                            "A curvature contradicts theta'/v",
                            field="curvature", vehicle_id=vehicle_id,
                            block_index=block.block_index,
                            max_error=abs(curvature - segment.control),
                            constraint="r_min_ackermann")
            elif (segment.mode == OBLIQUE and
                  abs(signed_speed) > _GEOMETRY_TOL):
                observed_phi = math.atan2(lateral_speed, signed_speed)
                if expected_direction < 0:
                    observed_phi = _wrap(observed_phi - math.pi)
                if _angle_error(observed_phi, segment.control) > (
                        self.config.heading_tolerance_rad + _GEOMETRY_TOL):
                    self._fail(
                        KINEMATIC_INFEASIBLE,
                        "O oblique angle contradicts centre velocity",
                        field="oblique_angle", vehicle_id=vehicle_id,
                        block_index=block.block_index,
                        max_error=_angle_error(
                            observed_phi, segment.control))
            if segment.direction == -1 and not bool(
                    getattr(self.snapshot, "allow_reverse", False)):
                self._fail(
                    MODE_SCHEDULE_INFEASIBLE,
                    "reverse direction is not authorized by the request",
                    field="direction", vehicle_id=vehicle_id,
                    block_index=block.block_index)
            theta_rate = (segment.theta_end - segment.theta_start) / length
            if block.geometry_type == "translation":
                if abs(theta_rate) > (
                        self.config.heading_tolerance_rad + _GEOMETRY_TOL):
                    self._fail(
                        KINEMATIC_INFEASIBLE,
                        "translation block changes body heading",
                        field="theta", vehicle_id=vehicle_id,
                        block_index=block.block_index,
                        max_error=abs(theta_rate))
            else:
                if segment.mode in (ACKERMANN, SPIN):
                    if abs(theta_rate - expected_theta_rate) > (
                            self.config.heading_tolerance_rad + _GEOMETRY_TOL):
                        self._fail(
                            KINEMATIC_INFEASIBLE,
                            "rotation body-heading rate differs from dpsi/L",
                            field="theta", vehicle_id=vehicle_id,
                            block_index=block.block_index,
                            max_error=abs(
                                theta_rate - expected_theta_rate))
            if math.hypot(
                    segment.start_center_x - start_centers[vehicle_id][0],
                    segment.start_center_y - start_centers[vehicle_id][1]) > (
                    self.config.position_tolerance_m + _GEOMETRY_TOL) or (
                    math.hypot(
                        segment.end_center_x - end_centers[vehicle_id][0],
                        segment.end_center_y - end_centers[vehicle_id][1]) > (
                        self.config.position_tolerance_m + _GEOMETRY_TOL)):
                self._fail(
                    FORMATION_RIGIDITY_VIOLATION,
                    "stored segment centres differ from q_F + R r_i",
                    field="segment_center", vehicle_id=vehicle_id,
                    block_index=block.block_index)
            self._count(7)

    def _check_timed_contract(self):
        timed = self.timed
        if not isinstance(timed, FormationTimedTrajectory):
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "timed must be a FormationTimedTrajectory",
                       field="timed")
        if timed.plan_id != self.plan_id:
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "timed plan_id differs from the plan",
                       field="plan_id")
        if timed.formation_id != self.formation_id:
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "timed formation_id differs from the plan",
                       field="formation_id")
        if tuple(timed.offsets) != tuple(self.model.offsets):
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "timed offsets differ from the fixed offsets",
                       field="offsets")
        if timed.certification != GEOMETRY_ONLY:
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "timed trajectory is not GEOMETRY_ONLY",
                       field="certification")
        times = tuple(timed.times_from_start)
        poses = tuple(timed.formation_poses)
        if len(times) < 2 or len(times) != len(poses):
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "timed time and pose grids differ or are too short",
                       field="times_from_start")
        if abs(times[0]) > _TIME_TOL:
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "timed time grid must start at zero",
                       field="times_from_start")
        previous = None
        for index, time in enumerate(times):
            if isinstance(time, bool) or not isinstance(time, Real) or (
                    not math.isfinite(float(time))):
                self._fail(INVALID_TIMED_TRAJECTORY,
                           "timed time is not finite",
                           field="times_from_start", sample_index=index)
            if previous is not None and not float(time) - previous > _TIME_TOL:
                self._fail(INVALID_TIMED_TRAJECTORY,
                           "timed times are not strictly increasing",
                           field="times_from_start", sample_index=index)
            previous = float(time)
        self._times = tuple(float(value) for value in times)
        trajectories = dict(timed.trajectories)
        if set(trajectories) != set(self.vehicle_ids):
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "timed vehicle id set differs from the formation",
                       field="trajectories")
        for vehicle_id in self.vehicle_ids:
            points = tuple(trajectories[vehicle_id])
            if len(points) != len(self._times):
                self._fail(INVALID_TIMED_TRAJECTORY,
                           "vehicle point count differs from the time grid",
                           field="trajectories", vehicle_id=vehicle_id)
            for index, point in enumerate(points):
                if not isinstance(point, FormationTimedPoint):
                    self._fail(INVALID_TIMED_TRAJECTORY,
                               "trajectory point is not FormationTimedPoint",
                               field="trajectories",
                               vehicle_id=vehicle_id, sample_index=index)
                if not _close(point.time_from_start, self._times[index],
                              self.config.dynamics_tolerance_ratio,
                              absolute=_TIME_TOL):
                    self._fail(INVALID_TIMED_TRAJECTORY,
                               "point time differs from the shared grid",
                               field="time_from_start",
                               vehicle_id=vehicle_id, sample_index=index)
        self._count(len(self.vehicle_ids) * len(self._times) + 8)

    def _parse_phases(self):
        raw_phases = tuple(self.timed.phases)
        if not raw_phases:
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "timed trajectory has no phase records",
                       field="phases")
        phases = []
        elapsed = 0.0
        motion_index = 0
        previous_kind = None
        for index, raw in enumerate(raw_phases):
            if not isinstance(raw, Mapping):
                self._fail(INVALID_TIMED_TRAJECTORY,
                           "phase record is not a mapping",
                           field="phases", sample_index=index)
            try:
                kind = raw["kind"]
                block_index = raw["block_index"]
                t0 = float(raw["t0_s"])
                duration = float(raw["duration_s"])
            except (KeyError, TypeError, ValueError) as exc:
                self._fail(INVALID_TIMED_TRAJECTORY,
                           "phase record is incomplete",
                           field="phases", sample_index=index)
            if kind not in ("motion", "stationary"):
                self._fail(INVALID_TIMED_TRAJECTORY,
                           "unknown phase kind", field="kind",
                           sample_index=index)
            if isinstance(block_index, bool) or not isinstance(
                    block_index, Integral):
                self._fail(INVALID_TIMED_TRAJECTORY,
                           "phase block_index is not an integer",
                           field="block_index", sample_index=index)
            block_index = int(block_index)
            if not math.isfinite(t0) or not math.isfinite(duration) or (
                    duration <= 0.0):
                self._fail(INVALID_TIMED_TRAJECTORY,
                           "phase time is not finite/positive",
                           field="duration_s", sample_index=index)
            if abs(t0 - elapsed) > 1e-7:
                self._fail(INVALID_TIMED_TRAJECTORY,
                           "phase times are not contiguous",
                           field="t0_s", sample_index=index)
            from_block = raw.get("from_block_index")
            if kind == "motion":
                if block_index != motion_index:
                    self._fail(INVALID_TIMED_TRAJECTORY,
                               "motion phases are not in block order",
                               field="block_index", sample_index=index)
                if from_block is not None:
                    self._fail(INVALID_TIMED_TRAJECTORY,
                               "motion phase must not have from_block_index",
                               field="from_block_index", sample_index=index)
                motion_index += 1
            else:
                if block_index != motion_index:
                    self._fail(INVALID_TIMED_TRAJECTORY,
                               "stationary phase must precede its next block",
                               field="block_index", sample_index=index)
                if isinstance(from_block, bool) or not isinstance(
                        from_block, Integral) or int(from_block) != (
                        block_index - 1):
                    self._fail(INVALID_TIMED_TRAJECTORY,
                               "stationary from_block_index is inconsistent",
                               field="from_block_index",
                               sample_index=index)
                if previous_kind == "stationary":
                    self._fail(INVALID_TIMED_TRAJECTORY,
                               "two stationary phases are adjacent",
                               field="phases", sample_index=index)
            phases.append(_PhaseInfo(index, kind, block_index,
                                     None if from_block is None
                                     else int(from_block), t0, duration))
            elapsed += duration
            previous_kind = kind
        if motion_index != len(self.blocks):
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "phase motion count differs from the block count",
                       field="phases")
        if abs(elapsed - self._times[-1]) > 1e-7:
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "phase duration differs from the timed grid",
                       field="phases")
        self.phases = tuple(phases)
        self.metrics["stationary_phase_count"] = sum(
            1 for phase in phases if phase.kind == "stationary")
        self._count(len(phases) * 2)

    def _assign_samples(self):
        count = len(self._times)
        self._sample_phase = [None] * count
        sample_count_total = 0
        phases = {phase.block_index: phase for phase in self.phases
                  if phase.kind == "motion"}
        for phase in self.phases:
            start = bisect_left(self._times, phase.t0_s)
            while start < count and self._times[start] <= (
                    phase.t0_s + _TIME_TOL):
                start += 1
            end_time = phase.t0_s + phase.duration_s
            end = bisect_right(self._times, end_time + _TIME_TOL)
            if end < start:
                end = start
            if end >= count:
                end = count
            if end <= start:
                self._fail(INVALID_TIMED_TRAJECTORY,
                           "phase has no time samples",
                           field="phases", sample_index=phase.index)
            if abs(self._times[end - 1] - end_time) > 1e-7:
                self._fail(INVALID_TIMED_TRAJECTORY,
                           "phase endpoint is absent from the time grid",
                           field="phases", sample_index=phase.index)
            for sample_index in range(start, end):
                if self._sample_phase[sample_index] is not None:
                    self._fail(INVALID_TIMED_TRAJECTORY,
                               "time sample belongs to two phases",
                               field="phases", sample_index=sample_index)
                self._sample_phase[sample_index] = phase
            sample_count_total += end - start
        if any(phase is None for phase in self._sample_phase[1:]):
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "a time sample is not covered by any phase",
                       field="phases")
        if sample_count_total != count - 1:
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "phase sample ownership is incomplete",
                       field="phases")
        # The initial sample is the first block at lambda=0.
        first = self.blocks[0]
        self._expected_qf.append(first.qf_at(0.0))
        self._expected_headings.append(
            {segment.vehicle_id: segment.theta_start
             for segment in first.vehicle_mode_segments})
        self._expected_block.append(first.block_index)
        self._expected_station.append(0.0)
        self._expected_stationary.append(False)
        for index in range(1, count):
            phase = self._sample_phase[index]
            block = self.blocks[phase.block_index]
            if phase.kind == "motion":
                tau = (self._times[index] - phase.t0_s) / phase.duration_s
                tau = min(1.0, max(0.0, tau))
                progress, _, _ = _quintic(tau)
                station = block.parameter_length * progress
                stationary = False
            else:
                station = 0.0
                stationary = True
            qf = block.qf_at(station)
            self._expected_qf.append(qf)
            self._expected_headings.append(
                block.body_headings_at(station))
            self._expected_block.append(block.block_index)
            self._expected_station.append(station)
            self._expected_stationary.append(stationary)
        if not _pose_equal(
                self._expected_qf[0], self.snapshot.start,
                self.config.position_tolerance_m,
                self.config.heading_tolerance_rad):
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "timed trajectory does not start at the request pose",
                       field="start")
        self._sample_phase[0] = None
        self._count(count + 4)

    def _check_samples(self):
        motion_limits = _motion_limits(self.config.timing_limits)
        trajectories = {vehicle_id: tuple(self.timed.trajectories[vehicle_id])
                        for vehicle_id in self.vehicle_ids}
        for index in range(len(self._times)):
            expected_qf = self._expected_qf[index]
            expected_headings = self._expected_headings[index]
            block = self.blocks[self._expected_block[index]]
            zero_dynamics = (index == 0 or self._expected_stationary[index])
            trajectory_pose = self.timed.formation_poses[index]
            if not _pose_equal(
                    trajectory_pose, expected_qf,
                    self.config.position_tolerance_m,
                    self.config.heading_tolerance_rad):
                self._fail(
                    INVALID_TIMED_TRAJECTORY,
                    "formation pose differs from the common progress q_F",
                    field="formation_poses", sample_index=index,
                    time_s=self._times[index])
            expected_centers = self.model.centers(expected_qf)
            actual_centers = {}
            for vehicle_id in self.vehicle_ids:
                point = trajectories[vehicle_id][index]
                actual_centers[vehicle_id] = (point.center_x, point.center_y)
                if point.block_index != self._expected_block[index]:
                    self._fail(
                        INVALID_TIMED_TRAJECTORY,
                        "point block_index differs from its phase",
                        field="block_index", vehicle_id=vehicle_id,
                        sample_index=index, time_s=self._times[index])
                if abs(point.station - self._expected_station[index]) > (
                        self.config.position_tolerance_m + _GEOMETRY_TOL):
                    self._fail(
                        INVALID_TIMED_TRAJECTORY,
                        "point station differs from the phase progress",
                        field="station", vehicle_id=vehicle_id,
                        sample_index=index, time_s=self._times[index])
                if bool(point.stationary) != bool(
                        self._expected_stationary[index]):
                    self._fail(
                        INVALID_TIMED_TRAJECTORY,
                        "point stationary flag differs from its phase",
                        field="stationary", vehicle_id=vehicle_id,
                        sample_index=index, time_s=self._times[index])
                segment = block.segment_map[vehicle_id]
                if point.mode != segment.mode:
                    self._fail(
                        KINEMATIC_INFEASIBLE,
                        "point mode differs from the planned mode",
                        field="mode", vehicle_id=vehicle_id,
                        sample_index=index, block_index=block.block_index,
                        time_s=self._times[index])
                if not _close(point.mode_control, segment.control,
                              self.config.dynamics_tolerance_ratio):
                    self._fail(
                        KINEMATIC_INFEASIBLE,
                        "point mode_control differs from the planned control",
                        field="mode_control", vehicle_id=vehicle_id,
                        sample_index=index, block_index=block.block_index,
                        time_s=self._times[index])
                heading_error = _angle_error(
                    point.theta, expected_headings[vehicle_id])
                if heading_error > self.config.heading_tolerance_rad + (
                        _GEOMETRY_TOL):
                    self._fail(
                        KINEMATIC_INFEASIBLE,
                        "body heading differs from theta_i(lambda)",
                        field="theta", vehicle_id=vehicle_id,
                        sample_index=index, block_index=block.block_index,
                        time_s=self._times[index], max_error=heading_error)
                expected_center = expected_centers[vehicle_id]
                center_error = math.hypot(
                    point.center_x - expected_center[0],
                    point.center_y - expected_center[1])
                if center_error > self.config.position_tolerance_m + (
                        _GEOMETRY_TOL):
                    self._fail(
                        FORMATION_RIGIDITY_VIOLATION,
                        "vehicle centre is not q_F + R(psi_F) r_i",
                        field="center", vehicle_id=vehicle_id,
                        sample_index=index, time_s=self._times[index],
                        max_error=center_error)
                self._max_rigidity = max(self._max_rigidity, center_error)
            pair_error, pair = self._pair_residual(actual_centers)
            self._max_rigidity = max(self._max_rigidity, pair_error)
            if pair_error > self.config.position_tolerance_m + (
                    _GEOMETRY_TOL):
                self._fail(
                    FORMATION_RIGIDITY_VIOLATION,
                    "pairwise formation distance drifted from the fixed "
                    "reference distance",
                    field="rigidity", pair_id=self._pair_text(*pair),
                    sample_index=index, time_s=self._times[index],
                    max_error=pair_error)
            if zero_dynamics:
                for vehicle_id in self.vehicle_ids:
                    point = trajectories[vehicle_id][index]
                    values = {
                        "center_speed_mps": point.center_speed_mps,
                        "center_yaw_rate_radps": point.center_yaw_rate_radps,
                        "center_acceleration_mps2":
                            point.center_acceleration_mps2,
                        "center_jerk_mps3": point.center_jerk_mps3,
                        "wheel_speed_mps": point.wheel_speed_mps,
                        "lateral_acceleration_mps2":
                            point.lateral_acceleration_mps2,
                    }
                    for field, value in values.items():
                        if abs(value) > _DYNAMIC_TOL:
                            self._fail(
                                FORMATION_RETIMING_INFEASIBLE,
                                "stationary sample has non-zero {}".format(
                                    field), field=field,
                                vehicle_id=vehicle_id,
                                sample_index=index,
                                block_index=block.block_index,
                                time_s=self._times[index],
                                constraint="stationary_dynamics",
                                max_error=abs(value))
                self._count(len(self.vehicle_ids) * 7 + 3)
                continue
            phase = self._sample_phase[index]
            tau = (self._times[index] - phase.t0_s) / phase.duration_s
            tau = min(1.0, max(0.0, tau))
            for vehicle_id in self.vehicle_ids:
                point = trajectories[vehicle_id][index]
                segment = block.segment_map[vehicle_id]
                _, factors = self._wheels[
                    (block.block_index, vehicle_id)]
                expected = _dynamic_values(
                    block, segment, self.offset_map[vehicle_id],
                    phase.duration_s, tau, motion_limits, factors)
                signed_error = abs(
                    point.center_speed_mps - expected["speed_signed"])
                if segment.mode == SPIN and abs(
                        point.center_speed_mps) > _DYNAMIC_TOL:
                    self._fail(
                        KINEMATIC_INFEASIBLE,
                        "Spin centre speed is non-zero",
                        field="center_speed_mps", vehicle_id=vehicle_id,
                        sample_index=index, time_s=self._times[index],
                        max_error=abs(point.center_speed_mps))
                if signed_error > max(
                        _DYNAMIC_TOL,
                        self.config.dynamics_tolerance_ratio *
                        max(1.0, expected["speed"])):
                    code = (KINEMATIC_INFEASIBLE
                            if segment.mode == SPIN or (
                                expected["speed"] > _DYNAMIC_TOL and
                                point.center_speed_mps *
                                expected["speed_signed"] < -_DYNAMIC_TOL)
                            else FORMATION_RETIMING_INFEASIBLE)
                    self._fail(
                        code,
                        "sample signed speed differs from the commanded "
                        "A/O/S motion",
                        field="center_speed_mps", vehicle_id=vehicle_id,
                        sample_index=index, time_s=self._times[index],
                        constraint="speed", max_error=signed_error)
                # The stored scalar is the signed component along the body
                # heading.  For O the centre travels along theta+phi, so the
                # full speed is reconstructed with the independently derived
                # lateral component before it is compared with |v|.
                speed_actual = abs(point.center_speed_mps)
                if segment.mode == OBLIQUE:
                    speed_actual = math.hypot(
                        point.center_speed_mps,
                        expected["lateral_speed_signed"])
                comparisons = (
                    ("center_speed_mps", speed_actual,
                     expected["speed"]),
                    ("center_acceleration_mps2",
                     point.center_acceleration_mps2,
                     expected["acceleration"]),
                    ("center_jerk_mps3", point.center_jerk_mps3,
                     expected["jerk"]),
                    ("center_yaw_rate_radps",
                     abs(point.center_yaw_rate_radps),
                     expected["yaw_rate"]),
                    ("wheel_speed_mps", point.wheel_speed_mps,
                     expected["wheel_speed"]),
                    ("lateral_acceleration_mps2",
                     point.lateral_acceleration_mps2,
                     expected["lateral_acceleration"]),
                )
                for field, actual, wanted in comparisons:
                    if not _close(actual, wanted,
                                  self.config.dynamics_tolerance_ratio):
                        self._fail(
                            FORMATION_RETIMING_INFEASIBLE,
                            "sample {} differs from the independent chain "
                            "rule".format(field), field=field,
                            vehicle_id=vehicle_id, sample_index=index,
                            block_index=block.block_index,
                            time_s=self._times[index],
                            constraint=field, max_error=abs(
                                actual - wanted))
                    self._sample_peak[
                        "speed" if field == "center_speed_mps" else
                        "acceleration" if field ==
                        "center_acceleration_mps2" else
                        "jerk" if field == "center_jerk_mps3" else
                        "yaw_rate" if field ==
                        "center_yaw_rate_radps" else
                        "wheel_speed" if field == "wheel_speed_mps" else
                        "lateral_acceleration"] = max(
                            self._sample_peak[
                                "speed" if field == "center_speed_mps"
                                else "acceleration" if field ==
                                "center_acceleration_mps2" else
                                "jerk" if field ==
                                "center_jerk_mps3" else
                                "yaw_rate" if field ==
                                "center_yaw_rate_radps" else
                                "wheel_speed" if field ==
                                "wheel_speed_mps" else
                                "lateral_acceleration"],
                            float(actual))
                self._count(len(comparisons) + 6)
        endpoint = self._expected_qf[-1]
        goal = self.snapshot.goal
        if not _pose_equal(
                endpoint, goal, self.config.position_tolerance_m,
                self.config.heading_tolerance_rad):
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "timed trajectory does not end at the request goal",
                       field="goal")
        for vehicle_id in self.vehicle_ids:
            if self.goal_heading_free[vehicle_id]:
                continue
            final_heading = self._expected_headings[-1][vehicle_id]
            if _angle_error(
                    final_heading, self.goal_headings[vehicle_id]) > (
                    self.config.heading_tolerance_rad + _GEOMETRY_TOL):
                self._fail(
                    INVALID_TIMED_TRAJECTORY,
                    "timed terminal body heading differs from the explicit "
                    "goal", field="goal_headings", vehicle_id=vehicle_id)
        self.metrics["duration_s"] = float(self._times[-1])
        self.metrics["sample_count"] = len(self._times)
        self.metrics["vehicle_count"] = len(self.vehicle_ids)
        self.metrics["max_rigidity_error_m"] = float(self._max_rigidity)
        self.metrics["sample_peaks"] = {
            key: float(value) for key, value in self._sample_peak.items()}
        self._count(4 + len(self.vehicle_ids))

    def _pair_residual(self, centers):
        maximum = 0.0
        worst = (self.vehicle_ids[0], self.vehicle_ids[-1])
        for index, first in enumerate(self.vehicle_ids):
            for second in self.vehicle_ids[index + 1:]:
                reference = math.hypot(
                    self.offset_map[second].x - self.offset_map[first].x,
                    self.offset_map[second].y - self.offset_map[first].y)
                actual = math.hypot(
                    centers[second][0] - centers[first][0],
                    centers[second][1] - centers[first][1])
                error = abs(actual - reference)
                if error > maximum:
                    maximum, worst = error, (first, second)
        return maximum, worst

    @staticmethod
    def _pair_text(first, second):
        return "({}, {})".format(first, second)

    def _check_mode_schedule(self):
        motion_limits = _motion_limits(self.config.timing_limits)
        intervals = {vehicle_id: [] for vehicle_id in self.vehicle_ids}
        for phase in self.phases:
            block = self.blocks[phase.block_index]
            for segment in block.vehicle_mode_segments:
                mode = segment.mode
                start = phase.t0_s
                end = phase.t0_s + phase.duration_s
                values = intervals[segment.vehicle_id]
                if values and values[-1][0] == mode and abs(
                        values[-1][2] - start) <= _TIME_TOL:
                    values[-1] = (mode, values[-1][1], end)
                else:
                    values.append((mode, start, end))
        min_dwell = self.config.timing_limits.min_mode_dwell_s
        shortest = float("inf")
        for vehicle_id, values in intervals.items():
            if not values:
                self._fail(MODE_SCHEDULE_INFEASIBLE,
                           "vehicle has no mode interval",
                           vehicle_id=vehicle_id, field="mode_intervals")
            if abs(values[0][1]) > _TIME_TOL or abs(
                    values[-1][2] - self._times[-1]) > 1e-7:
                self._fail(MODE_SCHEDULE_INFEASIBLE,
                           "mode intervals do not cover the trajectory",
                           vehicle_id=vehicle_id, field="mode_intervals")
            for previous, current in zip(values, values[1:]):
                if abs(previous[2] - current[1]) > 1e-7:
                    self._fail(MODE_SCHEDULE_INFEASIBLE,
                               "mode intervals are not contiguous",
                               vehicle_id=vehicle_id,
                               field="mode_intervals")
            for mode, start, end in values:
                duration = end - start
                shortest = min(shortest, duration)
                if duration + 1e-9 < min_dwell:
                    self._fail(
                        MODE_SCHEDULE_INFEASIBLE,
                        "mode {} dwell is {:.12g} s, below {:.12g} s".format(
                            mode, duration, min_dwell),
                        vehicle_id=vehicle_id, field="mode_dwell",
                        constraint="mode_dwell", time_s=start,
                        max_error=duration)
        self.metrics["minimum_mode_dwell_s"] = float(shortest)

        stationary_by_edge = {
            (phase.from_block_index, phase.block_index): phase
            for phase in self.phases if phase.kind == "stationary"}
        direction_changes = 0
        for previous, current in zip(self.blocks, self.blocks[1:]):
            previous_segments = previous.segment_map
            current_segments = current.segment_map
            deltas = {}
            mode_changed = False
            wheel_changed = False
            required = 0.0
            for vehicle_id in self.vehicle_ids:
                before = previous_segments[vehicle_id]
                after = current_segments[vehicle_id]
                before_angles, _ = self._wheels[
                    (previous.block_index, vehicle_id)]
                after_angles, _ = self._wheels[
                    (current.block_index, vehicle_id)]
                delta = max(
                    _angle_error(first, second)
                    for first, second in zip(before_angles, after_angles))
                deltas[vehicle_id] = delta
                event = mode_switch_event(delta, motion_limits)
                required = max(required, event["setup_s"] +
                               event["steering_s"])
                mode_changed = mode_changed or before.mode != after.mode
                wheel_changed = wheel_changed or delta > (
                    self.config.wheel_angle_tolerance_rad)
            event_phase = stationary_by_edge.get(
                (previous.block_index, current.block_index))
            if (mode_changed or wheel_changed) and event_phase is None:
                self._fail(
                    MODE_SCHEDULE_INFEASIBLE,
                    "mode/wheel change without a common stationary hold",
                    field="stationary_hold",
                    block_index=current.block_index,
                    constraint="hold_required")
            if event_phase is not None:
                if event_phase.duration_s + 1e-9 < required:
                    self._fail(
                        MODE_SCHEDULE_INFEASIBLE,
                        "stationary hold is shorter than the wheel "
                        "reconfiguration needs",
                        field="stationary_hold",
                        block_index=current.block_index,
                        constraint="wheel_reconfiguration",
                        max_error=max(0.0, required -
                                      event_phase.duration_s))
                for vehicle_id in self.vehicle_ids:
                    if deltas[vehicle_id] > self.config.wheel_angle_tolerance_rad or (
                            previous_segments[vehicle_id].mode !=
                            current_segments[vehicle_id].mode):
                        self._count()
            for vehicle_id in self.vehicle_ids:
                if previous_segments[vehicle_id].direction == current_segments[
                        vehicle_id].direction:
                    continue
                direction_changes += 1
                boundary_time = None
                if event_phase is not None:
                    boundary_time = event_phase.t0_s
                else:
                    for phase in self.phases:
                        if phase.kind == "motion" and phase.block_index == (
                                current.block_index):
                            boundary_time = phase.t0_s
                            break
                if boundary_time is None:
                    self._fail(MODE_SCHEDULE_INFEASIBLE,
                               "cannot locate a direction reversal in time",
                               vehicle_id=vehicle_id,
                               field="direction_reversal")
                check_times = [boundary_time]
                if event_phase is not None:
                    check_times.append(
                        event_phase.t0_s + event_phase.duration_s)
                for check_time in check_times:
                    pivot = bisect_left(self._times, check_time)
                    found = False
                    for sample_index in range(
                            max(0, pivot - 1),
                            min(len(self._times), pivot + 2)):
                        if abs(self._times[sample_index] - check_time) > 1e-7:
                            continue
                        found = True
                        point = self.timed.trajectories[vehicle_id][
                            sample_index]
                        if abs(point.center_speed_mps) > _DYNAMIC_TOL:
                            self._fail(
                                MODE_SCHEDULE_INFEASIBLE,
                                "direction reversal does not occur at rest",
                                field="center_speed_mps",
                                vehicle_id=vehicle_id,
                                sample_index=sample_index,
                                time_s=self._times[sample_index],
                                constraint="reversal_at_rest",
                                max_error=abs(point.center_speed_mps))
                    if not found:
                        self._fail(
                            MODE_SCHEDULE_INFEASIBLE,
                            "no sample exists at the direction reversal",
                            field="direction_reversal",
                            vehicle_id=vehicle_id, time_s=check_time)
                self._count(2)
        self.metrics["direction_change_count"] = int(direction_changes)
        self._count(len(self.phases) * 3 + len(self.blocks) * 2)

    def _check_dynamics_peaks(self):
        motion_limits = _motion_limits(self.config.timing_limits)
        peaks = {vehicle_id: {} for vehicle_id in self.vehicle_ids}
        maximum_ratio = 0.0
        for phase in self.phases:
            if phase.kind != "motion":
                continue
            block = self.blocks[phase.block_index]
            if phase.duration_s > (
                    self.config.timing_limits.max_block_duration_s + 1e-9):
                self._fail(
                    FORMATION_RETIMING_INFEASIBLE,
                    "block duration exceeds max_block_duration_s",
                    block_index=block.block_index,
                    constraint="max_block_duration_s",
                    time_s=phase.t0_s,
                    max_error=phase.duration_s)
            for vehicle_id in self.vehicle_ids:
                segment = block.segment_map[vehicle_id]
                _, factors = self._wheels[
                    (block.block_index, vehicle_id)]
                vehicle_peaks = {
                    name: 0.0 for name, _ in _DYNAMIC_LIMITS}
                vehicle_peaks["_tau"] = {
                    name: 0.0 for name, _ in _DYNAMIC_LIMITS}
                for tau in _TAUS:
                    values = _dynamic_values(
                        block, segment, self.offset_map[vehicle_id],
                        phase.duration_s, tau, motion_limits, factors)
                    for name, _ in _DYNAMIC_LIMITS:
                        if values[name] > vehicle_peaks[name]:
                            vehicle_peaks[name] = values[name]
                            vehicle_peaks["_tau"][name] = tau
                for name, limit_name in _DYNAMIC_LIMITS:
                    limit = getattr(self.config.timing_limits, limit_name)
                    peak = vehicle_peaks[name]
                    ratio = peak / limit
                    maximum_ratio = max(maximum_ratio, ratio)
                    if ratio > 1.0 + self.config.dynamics_tolerance_ratio:
                        self._fail(
                            FORMATION_RETIMING_INFEASIBLE,
                            "{} peak {:.12g} exceeds limit {:.12g}".format(
                                name, peak, limit),
                            constraint=name, vehicle_id=vehicle_id,
                            block_index=block.block_index,
                            time_s=phase.t0_s,
                            station=vehicle_peaks["_tau"][name],
                            max_error=peak)
                vehicle_peaks.pop("_tau", None)
                for name, _ in _DYNAMIC_LIMITS:
                    peaks[vehicle_id][name] = max(
                        peaks[vehicle_id].get(name, 0.0),
                        vehicle_peaks[name])
                self._count(len(_DYNAMIC_LIMITS) + _TAUS.__len__())
            self._count(2)
        self.metrics["dynamic_peak_ratio"] = float(maximum_ratio)
        self.metrics["vehicle_peaks"] = {
            vehicle_id: {key: float(value)
                         for key, value in values.items()}
            for vehicle_id, values in peaks.items()}

    def _collision_failure(self, report, code, prefix, timed=True,
                           time_override=None):
        if report.reason in ("MAP_OBB_COLLISION", "VEHICLE_OBB_COLLISION"):
            failure_code = code
        elif report.reason and "COLLISION" in report.reason:
            failure_code = code
        else:
            failure_code = INVALID_TIMED_TRAJECTORY
        time_s = time_override
        if time_s is None and timed and len(self._times):
            index = min(max(0, int(report.segment_index)), len(self._times) - 1)
            time_s = self._times[index]
        pair_id = None
        if report.vehicle_id is not None and report.other_vehicle_id is not None:
            pair_id = self._pair_text(
                report.vehicle_id, report.other_vehicle_id)
        detail = "{} {} block/sample {}/{}{}".format(
            prefix, report.reason or failure_code,
            report.segment_index, report.sample_index,
            "" if not report.detail else ": " + report.detail)
        self._fail(
            failure_code, detail, field="collision",
            vehicle_id=report.vehicle_id,
            other_vehicle_id=report.other_vehicle_id,
            pair_id=pair_id,
            segment_index=report.segment_index,
            sample_index=report.sample_index,
            time_s=time_s)

    def _check_static_collisions(self):
        checker = FormationCollisionChecker(
            self.grid, self.model, self.shapes,
            self.config.collision_margin_m)
        start = self.snapshot.start
        start_report = checker.static_check(
            start, dict(self.start_headings))
        self.checks += 1
        if not start_report.success:
            self._collision_failure(
                start_report, START_FORMATION_COLLISION, "start")
        goal_report = checker.static_check(
            self._expected_qf[-1], dict(self._expected_headings[-1]))
        self.checks += 1
        if not goal_report.success:
            self._collision_failure(
                goal_report, GOAL_FORMATION_COLLISION, "goal", timed=False,
                time_override=self._times[-1])
        self._checker = checker
        self._checker_statistics = checker.statistics()
        self._count(2)

    def _check_sweeps(self):
        states = [
            FormationState(self._times[index], self._expected_qf[index],
                           self._expected_headings[index])
            for index in range(len(self._times))]
        policy = SweepPolicy(
            max_corner_step_m=self.config.sweep_spacing_m,
            max_samples_per_segment=self.config.max_samples_per_segment)
        try:
            report = self._checker.swept_check(states, policy)
        except FormationValidationError as exc:
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "timed sweep input is invalid: {}".format(exc),
                       field="swept_check")
        self._record_collision_statistics()
        if not report.success:
            if report.reason in ("MAP_OBB_COLLISION",
                                 "VEHICLE_OBB_COLLISION"):
                self._collision_failure(
                    report, FORMATION_SWEEP_COLLISION, "timed sweep")
            self._fail(
                INVALID_TIMED_TRAJECTORY,
                "timed sweep could not certify the trajectory: {}".format(
                    report.reason),
                field="swept_check", vehicle_id=report.vehicle_id,
                segment_index=report.segment_index)
        self._count(len(self._times))

    def _record_collision_statistics(self):
        current = self._checker.statistics()
        delta = sum(max(0, current.get(key, 0) - self._checker_statistics.get(
            key, 0)) for key in current)
        self.checks += int(delta)
        self._checker_statistics = current

    def _publication_times(self):
        duration = self._times[-1]
        step = self.config.publication_dt_s
        values = list(self._times)
        count = int(math.floor(duration / step))
        for index in range(1, count + 1):
            values.append(index * step)
        values.append(duration)
        result = []
        for value in sorted(values):
            if not result or value - result[-1] > _TIME_TOL:
                result.append(float(value))
        return tuple(result)

    def _state_at(self, time):
        index = bisect_left(self._times, time)
        if index <= 0:
            return self._expected_qf[0], self._expected_headings[0]
        if index >= len(self._times):
            return self._expected_qf[-1], self._expected_headings[-1]
        if abs(self._times[index] - time) <= _TIME_TOL:
            return self._expected_qf[index], self._expected_headings[index]
        first = index - 1
        second = index
        fraction = (time - self._times[first]) / (
            self._times[second] - self._times[first])
        first_qf, first_headings = self._expected_qf[first], (
            self._expected_headings[first])
        second_qf, second_headings = self._expected_qf[second], (
            self._expected_headings[second])
        qf = FormationPose(
            first_qf.x + fraction * (second_qf.x - first_qf.x),
            first_qf.y + fraction * (second_qf.y - first_qf.y),
            first_qf.psi + fraction * (second_qf.psi - first_qf.psi))
        headings = {
            vehicle_id: first_headings[vehicle_id] + fraction * (
                second_headings[vehicle_id] - first_headings[vehicle_id])
            for vehicle_id in self.vehicle_ids}
        return qf, headings

    def _check_publication(self):
        publication_times = self._publication_times()
        states = []
        for time in publication_times:
            qf, headings = self._state_at(time)
            states.append(FormationState(time, qf, headings))
        policy = SweepPolicy(
            max_corner_step_m=self.config.sweep_spacing_m,
            max_samples_per_segment=self.config.max_samples_per_segment)
        try:
            report = self._checker.swept_check(states, policy)
        except FormationValidationError as exc:
            self._fail(INVALID_TIMED_TRAJECTORY,
                       "publication sweep input is invalid: {}".format(exc),
                       field="publication_sweep")
        self._record_collision_statistics()
        if not report.success:
            self._collision_failure(
                report, PUBLICATION_SWEEP_INVALID, "publication sweep")
        minimum_obstacle, minimum_intervehicle = self._clearance_metrics(
            states)
        self.metrics["publication_dt_s"] = float(self.config.publication_dt_s)
        self.metrics["publication_sample_count"] = len(publication_times)
        self.metrics["minimum_obstacle_clearance_m"] = float(
            minimum_obstacle)
        self.metrics["minimum_intervehicle_clearance_m"] = float(
            minimum_intervehicle)
        self._count(len(states) * (len(self.vehicle_ids) + 1))

    def _clearance_metrics(self, states):
        base = FootprintDistance(self.grid)
        distance_objects = {}
        for vehicle_id in self.vehicle_ids:
            shape = self.shapes[vehicle_id]
            distance_objects[vehicle_id] = FootprintDistance(
                self.grid, shape.length_m, shape.width_m, obstacle_index=base)
        minimum_obstacle = float("inf")
        minimum_intervehicle = float("inf")
        for state in states:
            centers = self.model.centers(state.qf)
            poses = {}
            for vehicle_id in self.vehicle_ids:
                theta = state.body_headings[vehicle_id]
                poses[vehicle_id] = Pose(
                    centers[vehicle_id][0], centers[vehicle_id][1], theta)
                distance = distance_objects[vehicle_id].distance(
                    poses[vehicle_id])
                if distance == 0.0:
                    self._fail(
                        FORMATION_SWEEP_COLLISION,
                        "vehicle OBB touches the map or its boundary",
                        field="obstacle_clearance", vehicle_id=vehicle_id,
                        time_s=state.station,
                        max_error=0.0)
                minimum_obstacle = min(minimum_obstacle, float(distance))
            for index, first in enumerate(self.vehicle_ids):
                for second in self.vehicle_ids[index + 1:]:
                    shape_a = self.shapes[first]
                    shape_b = self.shapes[second]
                    gap = _obb_distance(
                        poses[first], shape_a.length_m, shape_a.width_m,
                        poses[second], shape_b.length_m, shape_b.width_m)
                    if gap <= 0.0:
                        self._fail(
                            FORMATION_SWEEP_COLLISION,
                            "vehicle OBBs touch during publication sampling",
                            field="intervehicle_clearance",
                            vehicle_id=first, other_vehicle_id=second,
                            pair_id=self._pair_text(first, second),
                            time_s=state.station, max_error=gap)
                    minimum_intervehicle = min(
                        minimum_intervehicle, float(gap))
            self._count(len(self.vehicle_ids) + len(self.vehicle_ids) * (
                len(self.vehicle_ids) - 1) // 2)
        if not math.isfinite(minimum_obstacle):
            minimum_obstacle = 0.0
        if not math.isfinite(minimum_intervehicle):
            minimum_intervehicle = 0.0
        return minimum_obstacle, minimum_intervehicle

    def _success(self):
        self.metrics["checks"] = int(self.checks)
        self.metrics["certification"] = FEASIBLE
        self.metrics["independent_audit"] = True
        self.metrics["collision_certificate"] = True
        self.metrics["map_version_checked"] = True
        if not all(math.isfinite(float(value)) for value in (
                self.metrics.get("duration_s", 0.0),
                self.metrics.get("max_rigidity_error_m", 0.0),
                self.metrics.get("dynamic_peak_ratio", 0.0),
                self.metrics.get("minimum_obstacle_clearance_m", 0.0),
                self.metrics.get("minimum_intervehicle_clearance_m", 0.0))):
            self._fail(FORMATION_RETIMING_INFEASIBLE,
                       "evaluation metrics are not finite",
                       field="metrics")
        return FormationEvaluation(
            plan_id=self.plan_id,
            formation_id=self.formation_id,
            status=FEASIBLE,
            feasible=True,
            reason="OK",
            detail=("independent request/trajectory/rigidity/collision/"
                    "kinematics/timing/publication audit passed"),
            failure=None,
            metrics=dict(self.metrics),
            checks=self.checks)


def _segment_distance(first, second, third, fourth):
    def point_segment(point, start, end):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-24:
            return math.hypot(point[0] - start[0], point[1] - start[1])
        fraction = ((point[0] - start[0]) * dx +
                    (point[1] - start[1]) * dy) / length_sq
        fraction = min(1.0, max(0.0, fraction))
        return math.hypot(
            point[0] - (start[0] + fraction * dx),
            point[1] - (start[1] + fraction * dy))

    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (
            b[1] - a[1]) * (c[0] - a[0])

    def on_segment(a, b, c):
        return (min(a[0], b[0]) - 1e-12 <= c[0] <= max(a[0], b[0]) + 1e-12
                and min(a[1], b[1]) - 1e-12 <= c[1] <= max(
                    a[1], b[1]) + 1e-12)

    def intersects(a, b, c, d):
        o1 = orient(a, b, c)
        o2 = orient(a, b, d)
        o3 = orient(c, d, a)
        o4 = orient(c, d, b)
        if ((o1 > 0 and o2 < 0) or (o1 < 0 and o2 > 0)) and (
                (o3 > 0 and o4 < 0) or (o3 < 0 and o4 > 0)):
            return True
        if abs(o1) <= 1e-12 and on_segment(a, b, c):
            return True
        if abs(o2) <= 1e-12 and on_segment(a, b, d):
            return True
        if abs(o3) <= 1e-12 and on_segment(c, d, a):
            return True
        if abs(o4) <= 1e-12 and on_segment(c, d, b):
            return True
        return False

    if intersects(first, second, third, fourth):
        return 0.0
    return min(
        point_segment(first, third, fourth),
        point_segment(second, third, fourth),
        point_segment(third, first, second),
        point_segment(fourth, first, second))


def _obb_corners(pose, length, width):
    half_length, half_width = length / 2.0, width / 2.0
    cosine, sine = math.cos(pose.th), math.sin(pose.th)
    result = []
    for x, y in ((-half_length, -half_width),
                 (-half_length, half_width),
                 (half_length, half_width),
                 (half_length, -half_width)):
        result.append((pose.x + cosine * x - sine * y,
                       pose.y + sine * x + cosine * y))
    return result


def _obb_distance(first_pose, first_length, first_width,
                  second_pose, second_length, second_width):
    first = _obb_corners(first_pose, first_length, first_width)
    second = _obb_corners(second_pose, second_length, second_width)
    axes = (
        (math.cos(first_pose.th), math.sin(first_pose.th)),
        (-math.sin(first_pose.th), math.cos(first_pose.th)),
        (math.cos(second_pose.th), math.sin(second_pose.th)),
        (-math.sin(second_pose.th), math.cos(second_pose.th)),
    )
    for axis in axes:
        first_projection = [point[0] * axis[0] + point[1] * axis[1]
                            for point in first]
        second_projection = [point[0] * axis[0] + point[1] * axis[1]
                             for point in second]
        if min(first_projection) > max(second_projection) + 1e-12 or (
                min(second_projection) > max(first_projection) + 1e-12):
            break
    else:
        return 0.0
    result = float("inf")
    first_edges = list(zip(first, first[1:] + first[:1]))
    second_edges = list(zip(second, second[1:] + second[:1]))
    for a, b in first_edges:
        for c, d in second_edges:
            result = min(result, _segment_distance(a, b, c, d))
    return result


def _selection_key(candidate, evaluation):
    metrics = evaluation.metrics
    return (
        round(float(candidate.timed.duration), 9),
        -round(float(metrics.get("minimum_obstacle_clearance_m", 0.0)), 9),
        -round(float(metrics.get(
            "minimum_intervehicle_clearance_m", 0.0)), 9),
        int(metrics.get("mode_switch_count", 0)),
        round(float(metrics.get("dynamic_peak_ratio", 0.0)), 12),
        int(candidate.candidate_id))


def evaluate_formation_trajectory(snapshot, plan, timed, grid, config=None):
    """Independently evaluate one candidate; no input is trusted as evidence."""
    if config is None:
        config = FormationQualityConfig()
    if not isinstance(config, FormationQualityConfig):
        raise FormationValidationError(
            "INVALID_FORMATION_QUALITY_CONFIG")
    if not isinstance(plan, FormationContinuousPlan):
        raise FormationValidationError(
            "INVALID_FORMATION_PLAN expected FormationContinuousPlan")
    if not isinstance(timed, FormationTimedTrajectory):
        raise FormationValidationError(
            "INVALID_TIMED_TRAJECTORY expected FormationTimedTrajectory")
    return _IndependentEvaluator(
        snapshot, plan, timed, grid, config).run()


def select_formation_candidate(snapshot, candidates, grid, config=None):
    """Evaluate every candidate, then select the least-time feasible one.

    All candidates are evaluated regardless of early success.  A candidate is
    executable only when its independent evaluation is ``FEASIBLE``.
    """
    if config is None:
        config = FormationQualityConfig()
    if not isinstance(config, FormationQualityConfig):
        raise FormationValidationError(
            "INVALID_FORMATION_QUALITY_CONFIG")
    try:
        supplied = tuple(candidates)
    except TypeError as exc:
        raise FormationValidationError(
            "INVALID_FORMATION_CANDIDATES expected an iterable") from exc
    if not supplied:
        raise FormationValidationError("EMPTY_FORMATION_CANDIDATES")
    seen = set()
    checked = []
    for candidate in supplied:
        if not isinstance(candidate, FormationTimedCandidate):
            raise FormationValidationError(
                "INVALID_FORMATION_CANDIDATE")
        if candidate.candidate_id in seen:
            raise FormationValidationError(
                "DUPLICATE_CANDIDATE_ID {}".format(candidate.candidate_id))
        seen.add(candidate.candidate_id)
        checked.append(candidate)
    evaluations = []
    feasible = []
    rejections = []
    for candidate in checked:
        evaluation = evaluate_formation_trajectory(
            snapshot, candidate.plan, candidate.timed, grid, config)
        evaluations.append(evaluation)
        if evaluation.feasible:
            feasible.append((candidate, evaluation))
        else:
            rejections.append({
                "candidate_id": candidate.candidate_id,
                "feasibility_status": evaluation.status,
                "duration": float(candidate.timed.duration),
                "rejection_reason": evaluation.reason,
                "detail": evaluation.detail,
                "failure_code": (None if evaluation.failure is None
                                 else evaluation.failure.code),
            })
    if not feasible:
        return FormationSelection(
            status=REJECTED,
            selected_candidate_id=None,
            selected_plan=None,
            selected_timed=None,
            selected_evaluation=None,
            evaluations=tuple(evaluations),
            rejections=tuple(rejections),
            candidate_count=len(checked),
            feasible_count=0)
    feasible.sort(key=lambda item: _selection_key(item[0], item[1]))
    selected_candidate, selected_evaluation = feasible[0]
    return FormationSelection(
        status=FEASIBLE,
        selected_candidate_id=selected_candidate.candidate_id,
        selected_plan=selected_candidate.plan,
        selected_timed=selected_candidate.timed,
        selected_evaluation=selected_evaluation,
        evaluations=tuple(evaluations),
        rejections=tuple(rejections),
        candidate_count=len(checked),
        feasible_count=len(feasible))