"""FR02-A authoritative analytic time model and versioned sampled projection.

A :class:`UniformTimedPlan` evaluates the original FR01 edge under the common
quintic clock.  Setup phases keep every body pose fixed and evaluate one common
quintic wheel-angle curve after the configured setup overhead.  Vehicle
centres are never independently interpolated; in particular, nonzero-pivot
arcs remain on their exact analytic arcs at every query time.

``to_model_dict``/``from_model_dict`` form a strict, data-only V1 protocol.
Decoding rebuilds FR01 edges through their public factories and restores the
stored positive phase durations without retiming them.  It checks structure,
redundant values, continuity and setup demands; independent dynamic/collision
certification is intentionally FR02-B scope.
"""
from dataclasses import asdict, dataclass
import copy
import hashlib
import json
import math
from types import MappingProxyType
from typing import Mapping, Optional, Tuple

from .core import Action
from .formation import (
    FormationModel,
    FormationPose,
    FormationValidationError,
    VehicleOffset,
)
from .formation_collision import VehicleShape
from .formation_kinematics import ACKERMANN
from .formation_retiming import (
    FormationTimedPoint,
    FormationTimedTrajectory,
    FormationTimingLimits,
    FormationTimingReport,
)
from .mode_switch_timing import mode_switch_event
from .retiming import MotionLimits, wheel_configuration
from .uniform_motion import (
    MODE_A,
    MODE_O,
    MODE_S,
    PRIMITIVE_A_ARC,
    PRIMITIVE_A_STRAIGHT,
    PRIMITIVE_O_TRANSLATION,
    PRIMITIVE_S_SPIN,
    UniformMotionError,
    a_arc_edge,
    a_straight_edge,
    o_translation_edge,
    s_spin_edge,
)
from .uniform_timing import (
    HEADING_CONTINUITY_TOL_RAD,
    GEOMETRY_CONTINUITY_TOL_M,
    MotionTiming,
    SetupTiming,
    UniformTimingError,
    UniformTimingSchedule,
    VehicleSetupDemand,
    legacy_mode_word,
    motion_derivatives,
    restore_motion_timing,
    schedule_phase_totals,
    schedule_uniform_edges,
    timing_motion_limits,
    validate_uniform_edge_sequence,
)

MODEL_VERSION = "uniform_timed_plan.v1"
PROJECTION_SEMANTICS = "uniform_v02_signed_center_speed.v1"
MODEL_SEMANTICS = "formation_uniform_v02"
CERTIFICATION = "GEOMETRY_ONLY_NO_S9_NO_COLLISION"
TIME_CONTINUITY_TOL_S = 1e-9
# Numerical support domain for stored phase clocks, not a physical-duration
# policy.  FR02-B owns dynamic feasibility.  The A evaluator forms
# progress_rate**3 and length / duration**3, so clocks outside this interval
# cannot be stably evaluated with the current float arithmetic.
MIN_STABLE_PHASE_DURATION_S = 1e-100
MAX_STABLE_PHASE_DURATION_S = 1e100
FIELD_TOL = 1e-9
UINT32_MAX = 2 ** 32 - 1


class UniformTrajectoryError(UniformMotionError):
    """Strict time/model/projection rejection with a stable reason."""

    def __init__(self, reason, detail, edge_index=None, vehicle_id=None,
                 phase_id=None):
        self.edge_index = edge_index
        self.phase_id = phase_id
        super().__init__(reason, detail, vehicle_id=vehicle_id)


def _finite(value, context):
    if isinstance(value, (bool, str, bytes)):
        raise UniformTrajectoryError(
            "NONFINITE_VALUE", "{}={!r} is not finite numeric data".format(
                context, value))
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise UniformTrajectoryError(
            "NONFINITE_VALUE", "{}={!r} is not finite numeric data".format(
                context, value)) from exc
    if not math.isfinite(result):
        raise UniformTrajectoryError(
            "NONFINITE_VALUE", "{}={!r} is not finite".format(context, value))
    return result


def _stable_phase_duration(value, context, phase_id=None):
    """Require a stored phase clock inside the A numerical support domain."""
    result = _finite(value, context)
    if not MIN_STABLE_PHASE_DURATION_S <= result <= MAX_STABLE_PHASE_DURATION_S:
        raise UniformTrajectoryError(
            "PHASE_DURATION_UNSUPPORTED",
            "{}={:.17g} s is outside numeric support [{:.0e}, {:.0e}] s".format(
                context, result, MIN_STABLE_PHASE_DURATION_S,
                MAX_STABLE_PHASE_DURATION_S),
            phase_id=phase_id)
    return result


def _integer(value, context, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise UniformTrajectoryError(
            "INVALID_INTEGER", "{}={!r} must be an integer".format(
                context, value))
    if minimum is not None and value < minimum or (
            maximum is not None and value > maximum):
        raise UniformTrajectoryError(
            "INVALID_INTEGER", "{}={} is outside the allowed range".format(
                context, value))
    return value


def _string(value, context):
    if not isinstance(value, str) or not value.strip():
        raise UniformTrajectoryError(
            "INVALID_STRING", "{} must be a non-empty string".format(context))
    return value


def _mapping(value, context):
    if not isinstance(value, Mapping):
        raise UniformTrajectoryError(
            "INVALID_MODEL_STRUCTURE", "{} must be a mapping".format(context))
    return value


def _sequence(value, context):
    if isinstance(value, (str, bytes)) or not isinstance(value, (list, tuple)):
        raise UniformTrajectoryError(
            "INVALID_MODEL_STRUCTURE", "{} must be an array".format(context))
    return value


def _pick(values, key, context):
    _mapping(values, context)
    if key not in values:
        raise UniformTrajectoryError(
            "MISSING_MODEL_FIELD", "{} is missing {!r}".format(context, key))
    return values[key]


def _pose(value, context):
    items = _sequence(value, context)
    if len(items) != 3:
        raise UniformTrajectoryError(
            "INVALID_MODEL_STRUCTURE", "{} must contain x/y/psi".format(context))
    return FormationPose(*(
        _finite(item, "{}[{}]".format(context, index))
        for index, item in enumerate(items)))


def _pose_list(qf):
    return [qf.x, qf.y, qf.psi]


def _heading_records(headings):
    return [
        {"vehicle_id": int(vehicle_id), "theta": float(theta)}
        for vehicle_id, theta in headings]


def _headings(value, vehicle_ids, context):
    records = _sequence(value, context)
    result = {}
    order = []
    for index, record in enumerate(records):
        vehicle_id = _integer(
            _pick(record, "vehicle_id", "{}[{}]".format(context, index)),
            "{}[{}].vehicle_id".format(context, index), minimum=0,
            maximum=UINT32_MAX)
        if vehicle_id in result:
            raise UniformTrajectoryError(
                "DUPLICATE_VEHICLE_ID", "{} contains vehicle_id={} twice"
                .format(context, vehicle_id), vehicle_id=vehicle_id)
        theta = _finite(
            _pick(record, "theta", "{}[{}]".format(context, index)),
            "{}[{}].theta".format(context, index))
        result[vehicle_id] = theta
        order.append(vehicle_id)
    if set(result) != set(vehicle_ids) or len(result) != len(vehicle_ids):
        raise UniformTrajectoryError(
            "INVALID_VEHICLE_ID_SET",
            "{} IDs {} != {}".format(context, sorted(result),
                                     sorted(vehicle_ids)))
    if tuple(order) != tuple(sorted(vehicle_ids)):
        raise UniformTrajectoryError(
            "UNSORTED_VEHICLE_IDS", "{} must be sorted by vehicle_id".format(
                context))
    return tuple((vehicle_id, result[vehicle_id])
                 for vehicle_id in sorted(vehicle_ids))


def _heading_input(value, vehicle_ids, context):
    if isinstance(value, Mapping):
        records = [
            {"vehicle_id": vehicle_id, "theta": value[vehicle_id]}
            for vehicle_id in sorted(value)]
        return _headings(records, vehicle_ids, context)
    items = _sequence(value, context)
    if all(isinstance(item, (list, tuple)) and len(item) == 2
           for item in items):
        items = [
            {"vehicle_id": item[0], "theta": item[1]} for item in items]
    return _headings(items, vehicle_ids, context)


def _pair_map(pairs):
    return {vehicle_id: value for vehicle_id, value in pairs}


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _assert_finite_json(value, context="model"):
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return
    if isinstance(value, (int, float)):
        _finite(value, context)
        return
    if isinstance(value, Mapping):
        for key, item in value.items():
            if not isinstance(key, str):
                raise UniformTrajectoryError(
                    "INVALID_MODEL_STRUCTURE",
                    "{} contains non-string key {!r}".format(context, key))
            _assert_finite_json(item, "{}.{}".format(context, key))
        return
    if isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _assert_finite_json(item, "{}[{}]".format(context, index))
        return
    raise UniformTrajectoryError(
        "INVALID_MODEL_STRUCTURE", "{} contains unsupported {}".format(
            context, type(value).__name__))


def _stable_value(value):
    """Round floats for a factory-rebuild-stable semantic digest only."""
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise UniformTrajectoryError(
                "NONFINITE_VALUE", "digest input is not finite")
        if value == 0.0:
            return 0.0
        return float("{:.12g}".format(value))
    if isinstance(value, Mapping):
        return {key: _stable_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_stable_value(item) for item in value]
    raise UniformTrajectoryError(
        "INVALID_MODEL_STRUCTURE", "digest contains unsupported {}".format(
            type(value).__name__))


def _compare_json(actual, expected, context="model", tolerance=FIELD_TOL):
    if isinstance(expected, bool) or expected is None or isinstance(
            expected, str):
        if type(actual) is not type(expected) or actual != expected:
            raise UniformTrajectoryError(
                "REDUNDANT_VALUE_MISMATCH",
                "{}: {!r} != {!r}".format(context, actual, expected))
        return
    if isinstance(expected, int):
        if isinstance(actual, bool) or not isinstance(actual, int) or (
                actual != expected):
            raise UniformTrajectoryError(
                "REDUNDANT_VALUE_MISMATCH",
                "{}: {!r} != {!r}".format(context, actual, expected))
        return
    if isinstance(expected, float):
        if isinstance(actual, (bool, str, bytes)) or not isinstance(
                actual, (int, float)):
            raise UniformTrajectoryError(
                "REDUNDANT_VALUE_MISMATCH",
                "{}: {!r} is not numeric".format(context, actual))
        actual_value = _finite(actual, context)
        if abs(actual_value - expected) > tolerance:
            raise UniformTrajectoryError(
                "REDUNDANT_VALUE_MISMATCH",
                "{}: {:.17g} != recomputed {:.17g}".format(
                    context, actual_value, expected))
        return
    if isinstance(expected, Mapping):
        if not isinstance(actual, Mapping) or set(actual) != set(expected):
            raise UniformTrajectoryError(
                "REDUNDANT_VALUE_MISMATCH",
                "{} mapping keys differ".format(context))
        for key in expected:
            _compare_json(actual[key], expected[key],
                          "{}.{}".format(context, key), tolerance)
        return
    if isinstance(expected, (list, tuple)):
        if not isinstance(actual, (list, tuple)) or len(actual) != len(expected):
            raise UniformTrajectoryError(
                "REDUNDANT_VALUE_MISMATCH",
                "{} array length differs".format(context))
        for index, (actual_item, expected_item) in enumerate(
                zip(actual, expected)):
            _compare_json(actual_item, expected_item,
                          "{}[{}]".format(context, index), tolerance)
        return
    raise UniformTrajectoryError(
        "INVALID_MODEL_STRUCTURE", "{} has unsupported expected type {}"
        .format(context, type(expected).__name__))


def _quintic_basic(tau):
    """Unit quintic and its first three derivatives in factorized form."""
    return (
        tau ** 3 * (10.0 - 15.0 * tau + 6.0 * tau * tau),
        30.0 * tau * tau * (1.0 - tau) * (1.0 - tau),
        60.0 * tau * (1.0 - tau) * (1.0 - 2.0 * tau),
        60.0 - 360.0 * tau + 360.0 * tau * tau,
    )


def _stable_quintic(tau):
    """Evaluate the unit quintic without endpoint cancellation.

    The quintic satisfies ``s(tau) = 1 - s(1-tau)``, while its first and
    third derivatives are symmetric and its second derivative is
    antisymmetric.  Evaluating the small complement in the second half keeps
    the progress from rounding just above one near the motion endpoint.
    """
    if tau <= 0.0:
        return 0.0, 0.0, 0.0, 60.0
    if tau >= 1.0:
        return 1.0, 0.0, 0.0, 60.0
    if tau > 0.5:
        complement = 1.0 - tau
        progress, first, second, third = _quintic_basic(complement)
        return 1.0 - progress, first, -second, third
    return _quintic_basic(tau)


def _wheel_positions(limits):
    """Wheel coordinates in the same order as ``wheel_configuration``."""
    return tuple(
        (x, y)
        for x in (limits.wheelbase / 2.0, -limits.wheelbase / 2.0)
        for y in (limits.track / 2.0, -limits.track / 2.0))


def _normalise_shapes(shapes, vehicle_ids):
    if not isinstance(shapes, Mapping):
        raise UniformTrajectoryError(
            "INVALID_VEHICLE_SHAPES", "vehicle_shapes must be an ID mapping")
    if set(shapes) != set(vehicle_ids) or len(shapes) != len(vehicle_ids):
        raise UniformTrajectoryError(
            "INVALID_VEHICLE_ID_SET", "vehicle shape IDs do not match offsets")
    result = {}
    for vehicle_id in vehicle_ids:
        value = shapes[vehicle_id]
        if not isinstance(value, VehicleShape):
            try:
                value = VehicleShape(float(value[0]), float(value[1]))
            except (TypeError, ValueError, IndexError) as exc:
                raise UniformTrajectoryError(
                    "INVALID_VEHICLE_SHAPES",
                    "vehicle_id={} shape must be VehicleShape or (length, width)"
                    .format(vehicle_id), vehicle_id=vehicle_id) from exc
        result[vehicle_id] = value
    return MappingProxyType(result)


@dataclass(frozen=True)
class UniformVehicleTimeState:
    vehicle_id: int
    x: float
    y: float
    theta: float
    velocity: Tuple[float, float]
    acceleration: Tuple[float, float]
    jerk: Tuple[float, float]
    yaw_rate_radps: float
    yaw_acceleration_radps2: float
    yaw_jerk_radps3: float
    center_speed_mps: float
    longitudinal_speed_mps: float
    lateral_speed_mps: float
    center_acceleration_mps2: float
    center_jerk_mps3: float
    wheel_speed_mps: float
    lateral_acceleration_mps2: float
    direction: int
    mode_control: float
    wheel_angles: Tuple[float, float, float, float]
    wheel_angular_rates: Tuple[float, float, float, float]
    wheel_angular_accelerations: Tuple[float, float, float, float]
    wheel_rolling_speeds: Tuple[float, float, float, float]

    def as_dict(self):
        return {
            "vehicle_id": self.vehicle_id,
            "center": [self.x, self.y],
            "theta": self.theta,
            "velocity": list(self.velocity),
            "acceleration": list(self.acceleration),
            "jerk": list(self.jerk),
            "yaw_rate_radps": self.yaw_rate_radps,
            "yaw_acceleration_radps2": self.yaw_acceleration_radps2,
            "yaw_jerk_radps3": self.yaw_jerk_radps3,
            "center_speed_mps": self.center_speed_mps,
            "longitudinal_speed_mps": self.longitudinal_speed_mps,
            "lateral_speed_mps": self.lateral_speed_mps,
            "center_acceleration_mps2": self.center_acceleration_mps2,
            "center_jerk_mps3": self.center_jerk_mps3,
            "wheel_speed_mps": self.wheel_speed_mps,
            "lateral_acceleration_mps2": self.lateral_acceleration_mps2,
            "direction": self.direction,
            "mode_control": self.mode_control,
            "wheel_angles_rad": list(self.wheel_angles),
            "wheel_angular_rates_radps": list(self.wheel_angular_rates),
            "wheel_angular_accelerations_radps2": list(
                self.wheel_angular_accelerations),
            "wheel_rolling_speeds_mps": list(self.wheel_rolling_speeds),
        }


@dataclass(frozen=True)
class UniformTimeState:
    time_s: float
    phase_id: str
    phase_kind: str
    phase_index: int
    edge_index: Optional[int]
    mode: str
    qf: FormationPose
    vehicles: Tuple[UniformVehicleTimeState, ...]
    progress: float
    progress_unit: str
    progress_rate: float
    progress_acceleration: float
    progress_jerk: float
    qf_velocity: Tuple[float, float, float]
    qf_acceleration: Tuple[float, float, float]
    qf_jerk: Tuple[float, float, float]
    from_mode: Optional[str] = None
    to_mode: Optional[str] = None

    @property
    def centers(self):
        return tuple((vehicle.vehicle_id, vehicle.x, vehicle.y)
                     for vehicle in self.vehicles)

    @property
    def headings(self):
        return tuple((vehicle.vehicle_id, vehicle.theta)
                     for vehicle in self.vehicles)

    @property
    def stationary(self):
        return self.phase_kind in ("initial_setup", "setup", "hold")

    def vehicle(self, vehicle_id):
        for vehicle in self.vehicles:
            if vehicle.vehicle_id == vehicle_id:
                return vehicle
        raise UniformTrajectoryError(
            "INVALID_VEHICLE_ID_SET", "state has no vehicle_id={}".format(
                vehicle_id), vehicle_id=vehicle_id)

    def as_dict(self):
        return {
            "time_s": self.time_s,
            "phase_id": self.phase_id,
            "phase_kind": self.phase_kind,
            "phase_index": self.phase_index,
            "edge_index": self.edge_index,
            "mode": self.mode,
            "qf": _pose_list(self.qf),
            "centers": [
                {"vehicle_id": vehicle_id, "x": x, "y": y}
                for vehicle_id, x, y in self.centers],
            "headings": [
                {"vehicle_id": vehicle_id, "theta": theta}
                for vehicle_id, theta in self.headings],
            "progress": self.progress,
            "progress_unit": self.progress_unit,
            "progress_rate": self.progress_rate,
            "progress_acceleration": self.progress_acceleration,
            "progress_jerk": self.progress_jerk,
            "qf_velocity": list(self.qf_velocity),
            "qf_acceleration": list(self.qf_acceleration),
            "qf_jerk": list(self.qf_jerk),
            "stationary": self.stationary,
            "from_mode": self.from_mode,
            "to_mode": self.to_mode,
            "vehicles": [vehicle.as_dict() for vehicle in self.vehicles],
        }


@dataclass(frozen=True)
class UniformSampleProjection:
    """Versioned V0.2 envelope around the legacy numeric sample carrier."""

    semantic_version: str
    model_version: str
    plan_id: int
    model_sha256: str
    states: Tuple[UniformTimeState, ...]
    timed_trajectory: FormationTimedTrajectory
    certification: str = CERTIFICATION

    @property
    def times(self):
        return self.timed_trajectory.times_from_start

    def as_dict(self, include_states=True):
        result = {
            "semantic_version": self.semantic_version,
            "model_version": self.model_version,
            "plan_id": self.plan_id,
            "model_sha256": self.model_sha256,
            "certification": self.certification,
            "formation_id": self.timed_trajectory.formation_id,
            "times_from_start": list(self.times),
            "formation_poses": [
                _pose_list(qf)
                for qf in self.timed_trajectory.formation_poses],
            "offsets": [
                {"vehicle_id": offset.vehicle_id, "x": offset.x, "y": offset.y}
                for offset in self.timed_trajectory.offsets],
            "trajectories": {
                str(vehicle_id): [point.as_dict() for point in points]
                for vehicle_id, points in sorted(
                    self.timed_trajectory.trajectories.items())},
            "phases": copy.deepcopy(self.timed_trajectory.phases),
            "timing_report": self.timed_trajectory.timing_report.as_dict(),
        }
        if include_states:
            result["analytic_states"] = [state.as_dict() for state in self.states]
        return result


@dataclass(frozen=True)
class UniformTimedPlan:
    plan_id: int
    formation_id: str
    model: FormationModel
    vehicle_shapes: Mapping[int, VehicleShape]
    edges: Tuple[object, ...]
    phases: Tuple[object, ...]
    limits: FormationTimingLimits
    motion_limits: MotionLimits
    start_qf: FormationPose
    start_headings: Tuple[Tuple[int, float], ...]
    goal_qf: FormationPose
    goal_headings: Tuple[Tuple[int, float], ...]
    duration_s: float

    def __post_init__(self):
        _integer(self.plan_id, "plan_id", minimum=0, maximum=UINT32_MAX)
        _string(self.formation_id, "formation_id")
        if not isinstance(self.model, FormationModel):
            raise UniformTrajectoryError("INVALID_MODEL", "expected FormationModel")
        if not isinstance(self.start_qf, FormationPose) or not isinstance(
                self.goal_qf, FormationPose):
            raise UniformTrajectoryError(
                "INVALID_POSE", "start_qf/goal_qf must be FormationPose")
        if not isinstance(self.limits, FormationTimingLimits):
            raise UniformTrajectoryError(
                "INVALID_TIMING_LIMITS", "expected FormationTimingLimits")
        if not isinstance(self.motion_limits, MotionLimits):
            raise UniformTrajectoryError(
                "INVALID_LIMITS", "expected restricted MotionLimits")
        object.__setattr__(self, "edges", tuple(self.edges))
        object.__setattr__(self, "phases", tuple(self.phases))
        object.__setattr__(self, "vehicle_shapes", _normalise_shapes(
            self.vehicle_shapes, self.model.vehicle_ids))
        object.__setattr__(self, "start_headings", tuple(self.start_headings))
        object.__setattr__(self, "goal_headings", tuple(self.goal_headings))
        object.__setattr__(self, "duration_s", _finite(
            self.duration_s, "duration_s"))
        if self.duration_s < 0.0:
            raise UniformTrajectoryError(
                "NEGATIVE_PLAN_DURATION", "duration_s must be nonnegative")

    @classmethod
    def from_edges(cls, edges, vehicle_shapes, plan_id=0,
                   formation_id="uniform_v02", timing_limits=None, model=None,
                   start_qf=None, start_headings=None, expected_end_qf=None,
                   expected_end_headings=None):
        """Build and time a manual sequence, or an explicit zero-motion hold."""
        try:
            sequence = tuple(edges)
        except TypeError as exc:
            raise UniformTrajectoryError(
                "INVALID_EDGE_SEQUENCE", "edges must be iterable") from exc
        if timing_limits is None:
            timing_limits = FormationTimingLimits()
        if sequence:
            if model is None:
                model = sequence[0].model
            if not isinstance(model, FormationModel):
                raise UniformTrajectoryError(
                    "INVALID_MODEL", "model must be FormationModel")
            if tuple((item.vehicle_id, item.x, item.y)
                     for item in model.offsets) != tuple(
                         (item.vehicle_id, item.x, item.y)
                         for item in sequence[0].model.offsets):
                raise UniformTrajectoryError(
                    "EDGE_OFFSETS_MISMATCH", "supplied model differs from edges")
            initial_qf = sequence[0].start_qf if start_qf is None else start_qf
            initial_headings = (sequence[0].start_headings
                                if start_headings is None else _heading_input(
                                    start_headings, model.vehicle_ids,
                                    "start_headings"))
            goal_qf = (sequence[-1].end_qf if expected_end_qf is None
                       else expected_end_qf)
            goal_headings = (sequence[-1].end_headings
                             if expected_end_headings is None else _heading_input(
                                 expected_end_headings, model.vehicle_ids,
                                 "expected_end_headings"))
            schedule = schedule_uniform_edges(sequence, timing_limits)
            plan = cls(
                plan_id=plan_id, formation_id=formation_id, model=model,
                vehicle_shapes=vehicle_shapes, edges=schedule.edges,
                phases=schedule.phases, limits=timing_limits,
                motion_limits=schedule.motion_limits, start_qf=initial_qf,
                start_headings=initial_headings, goal_qf=goal_qf,
                goal_headings=goal_headings, duration_s=schedule.duration)
        else:
            if not isinstance(model, FormationModel):
                raise UniformTrajectoryError(
                    "ZERO_MOTION_MODEL_REQUIRED",
                    "an empty sequence needs an explicit FormationModel")
            if start_qf is None or start_headings is None or (
                    expected_end_qf is None) or expected_end_headings is None:
                raise UniformTrajectoryError(
                    "ZERO_MOTION_REQUEST_REQUIRED",
                    "empty edges require explicit start/goal q_F and headings")
            initial_qf = start_qf
            initial_headings = _heading_input(
                start_headings, model.vehicle_ids, "start_headings")
            goal_qf = expected_end_qf
            goal_headings = _heading_input(
                expected_end_headings, model.vehicle_ids,
                "expected_end_headings")
            plan = cls(
                plan_id=plan_id, formation_id=formation_id, model=model,
                vehicle_shapes=vehicle_shapes, edges=(), phases=(),
                limits=timing_limits,
                motion_limits=timing_motion_limits(timing_limits),
                start_qf=initial_qf, start_headings=initial_headings,
                goal_qf=goal_qf, goal_headings=goal_headings, duration_s=0.0)
        _validate_plan_structure(plan)
        return plan

    @property
    def duration(self):
        return self.duration_s

    @property
    def vehicle_ids(self):
        return self.model.vehicle_ids

    @property
    def schedule(self):
        if not self.edges:
            raise UniformTrajectoryError(
                "EMPTY_EDGE_SEQUENCE", "a zero-motion hold has no timing schedule")
        return UniformTimingSchedule(
            edges=self.edges, phases=self.phases, limits=self.limits,
            motion_limits=self.motion_limits)

    def phase_table(self):
        if not self.edges:
            return []
        return self.schedule.phase_table()

    def phase_totals(self):
        if not self.edges:
            return {
                "initial_setup_s": 0.0, "inter_edge_setup_s": 0.0,
                "motion_s": 0.0, "duration_s": 0.0,
                "initial_setup_count": 0, "inter_edge_setup_count": 0,
                "motion_count": 0,
            }
        return schedule_phase_totals(self.schedule)

    def _phase_at(self, time_s, side="auto"):
        if side not in ("auto", "left", "right"):
            raise UniformTrajectoryError(
                "INVALID_BOUNDARY_SIDE", "side={!r}".format(side))
        value = _finite(time_s, "state_at.time_s")
        if value < 0.0 or value > self.duration_s:
            raise UniformTrajectoryError(
                "TIME_OUT_OF_RANGE",
                "time={:.17g} is outside [0, {:.17g}]".format(
                    value, self.duration_s))
        if not self.phases:
            return value, None, -1
        right_index = None
        for index, phase in enumerate(self.phases):
            if phase.t0_s <= value < phase.end_time:
                right_index = index
                break
        if right_index is None:
            if value == self.duration_s:
                right_index = len(self.phases) - 1
            else:
                raise UniformTrajectoryError(
                    "PHASE_TIME_DISCONTINUITY",
                    "no phase contains t={:.17g}".format(value))
        if side == "left" and value > 0.0:
            for index, phase in enumerate(self.phases):
                if value == phase.end_time:
                    return value, phase, index
        return value, self.phases[right_index], right_index

    def state_at(self, time_s, side="auto"):
        """Exact analytic state; boundaries are left-closed/right-open.

        ``side='left'`` is an audit-only query for the old-mode state exactly
        at a phase boundary.  The default always selects the right phase, and
        ``t=T_end`` selects the final stationary endpoint of the last motion.
        """
        value, phase, phase_index = self._phase_at(time_s, side)
        if phase is None:
            return self._hold_state(value)
        if isinstance(phase, MotionTiming):
            return self._motion_state(value, phase, phase_index)
        if isinstance(phase, SetupTiming):
            return self._setup_state(value, phase, phase_index)
        raise UniformTrajectoryError(
            "INVALID_PHASE_TYPE", "unsupported phase {}".format(type(phase)))

    def _zero_vehicle(self, vehicle_id, center, theta, direction, control,
                      angles):
        return UniformVehicleTimeState(
            vehicle_id=vehicle_id, x=center[0], y=center[1], theta=theta,
            velocity=(0.0, 0.0), acceleration=(0.0, 0.0), jerk=(0.0, 0.0),
            yaw_rate_radps=0.0, yaw_acceleration_radps2=0.0,
            yaw_jerk_radps3=0.0, center_speed_mps=0.0,
            longitudinal_speed_mps=0.0, lateral_speed_mps=0.0,
            center_acceleration_mps2=0.0, center_jerk_mps3=0.0,
            wheel_speed_mps=0.0, lateral_acceleration_mps2=0.0,
            direction=direction, mode_control=control,
            wheel_angles=tuple(angles),
            wheel_angular_rates=(0.0, 0.0, 0.0, 0.0),
            wheel_angular_accelerations=(0.0, 0.0, 0.0, 0.0),
            wheel_rolling_speeds=(0.0, 0.0, 0.0, 0.0))

    def _hold_state(self, time_s):
        centers = self.model.centers(self.start_qf)
        headings = _pair_map(self.start_headings)
        angles, _ = wheel_configuration(
            Action(ACKERMANN, 0.0, 0.0), self.motion_limits)
        vehicles = tuple(self._zero_vehicle(
            vehicle_id, centers[vehicle_id], headings[vehicle_id], 1, 0.0,
            tuple(float(value) for value in angles))
            for vehicle_id in self.vehicle_ids)
        return UniformTimeState(
            time_s=time_s, phase_id="hold", phase_kind="hold", phase_index=-1,
            edge_index=None, mode=MODE_A, qf=self.start_qf, vehicles=vehicles,
            progress=0.0, progress_unit="m", progress_rate=0.0,
            progress_acceleration=0.0, progress_jerk=0.0,
            qf_velocity=(0.0, 0.0, 0.0),
            qf_acceleration=(0.0, 0.0, 0.0), qf_jerk=(0.0, 0.0, 0.0),
            from_mode=MODE_A, to_mode=MODE_A)

    def _setup_state(self, time_s, phase, phase_index):
        centers = self.model.centers(phase.qf)
        headings = _pair_map(phase.headings)
        local = time_s - phase.t0_s
        steering_tau = ((local - phase.setup_overhead_s)
                        / phase.steering_duration_s
                        if phase.steering_duration_s > 0.0 else 1.0)
        if steering_tau <= 0.0:
            tau = 0.0
            rate_factor = 0.0
            accel_factor = 0.0
        elif steering_tau >= 1.0:
            tau = 1.0
            rate_factor = 0.0
            accel_factor = 0.0
        else:
            tau = steering_tau
            _, first, second, _ = _stable_quintic(tau)
            rate_factor = first / phase.steering_duration_s
            accel_factor = second / phase.steering_duration_s ** 2
        progress, _, _, _ = _stable_quintic(tau)
        vehicles = []
        for demand in phase.demands:
            angles = tuple(
                start + delta * progress for start, delta in zip(
                    demand.wheel_angles_start, demand.wheel_angle_deltas))
            rates = tuple(delta * rate_factor
                          for delta in demand.wheel_angle_deltas)
            accelerations = tuple(delta * accel_factor
                                  for delta in demand.wheel_angle_deltas)
            vehicle = self._zero_vehicle(
                demand.vehicle_id, centers[demand.vehicle_id],
                headings[demand.vehicle_id], demand.from_direction,
                demand.from_control, angles)
            object.__setattr__(vehicle, "wheel_angular_rates", rates)
            object.__setattr__(vehicle, "wheel_angular_accelerations",
                               accelerations)
            vehicles.append(vehicle)
        return UniformTimeState(
            time_s=time_s, phase_id=phase.phase_id, phase_kind=phase.kind,
            phase_index=phase_index, edge_index=phase.edge_index,
            mode=phase.from_mode, qf=phase.qf, vehicles=tuple(vehicles),
            progress=0.0,
            progress_unit=self.edges[phase.edge_index].progress_unit,
            progress_rate=0.0, progress_acceleration=0.0, progress_jerk=0.0,
            qf_velocity=(0.0, 0.0, 0.0),
            qf_acceleration=(0.0, 0.0, 0.0), qf_jerk=(0.0, 0.0, 0.0),
            from_mode=phase.from_mode, to_mode=phase.to_mode)

    def _motion_state(self, time_s, phase, phase_index):
        duration = phase.duration_s
        if time_s == phase.t0_s:
            tau = 0.0
        elif time_s == phase.end_time:
            tau = 1.0
        else:
            tau = (time_s - phase.t0_s) / duration
            # The requested time is already inside the closed phase interval.
            # A would-be 1+ulp quotient is only floating subtraction rounding,
            # not permission to accept an out-of-domain time.
            if tau < 0.0:
                tau = 0.0
            elif tau > 1.0:
                tau = 1.0
        progress_unitless, first, second, third = _stable_quintic(tau)
        length = phase.edge.control.parameter_length
        progress = length * progress_unitless
        progress_rate = length * first / duration
        progress_acceleration = length * second / duration ** 2
        progress_jerk = length * third / duration ** 3
        derivatives = motion_derivatives(phase, progress)
        geometric = phase.edge.state_at(progress)
        centers = {
            vehicle_id: (x, y) for vehicle_id, x, y in geometric.centers}
        headings = _pair_map(geometric.headings)

        def chained(first_vector, second_vector, third_vector):
            velocity = tuple(item * progress_rate for item in first_vector)
            acceleration = tuple(
                second_item * progress_rate ** 2
                + first_item * progress_acceleration
                for first_item, second_item in zip(first_vector, second_vector))
            jerk = tuple(
                third_item * progress_rate ** 3
                + 3.0 * second_item * progress_rate * progress_acceleration
                + first_item * progress_jerk
                for first_item, second_item, third_item in zip(
                    first_vector, second_vector, third_vector))
            return velocity, acceleration, jerk

        qf_velocity, qf_acceleration, qf_jerk = chained(
            derivatives.q_first, derivatives.q_second, derivatives.q_third)
        wheel_positions = _wheel_positions(self.motion_limits)
        vehicles = []
        for item in derivatives.vehicles:
            velocity, acceleration, jerk = chained(
                item.first, item.second, item.third)
            speed = math.hypot(*velocity)
            acceleration_norm = math.hypot(*acceleration)
            jerk_norm = math.hypot(*jerk)
            yaw_rate = item.theta_rate * progress_rate
            yaw_acceleration = item.theta_rate * progress_acceleration
            yaw_jerk = item.theta_rate * progress_jerk
            theta = headings[item.vehicle_id]
            if phase.edge.mode == MODE_S:
                center_speed = 0.0
                longitudinal = 0.0
                lateral_speed = 0.0
                # Canonical S wheel angles are folded into [-pi/2, pi/2].
                # The per-wheel factor is an unsigned magnitude, so project
                # the body-spin wheel-plane velocity onto the actual folded
                # rolling direction to retain the needed sign changes.
                rolling = tuple(
                    (-yaw_rate * position_y) * math.cos(angle)
                    + (yaw_rate * position_x) * math.sin(angle)
                    for (position_x, position_y), angle in zip(
                        wheel_positions, item.wheel_angles))
                lateral_acceleration = 0.0
            else:
                center_speed = item.direction * speed
                longitudinal = (velocity[0] * math.cos(theta)
                                + velocity[1] * math.sin(theta))
                lateral_speed = (-velocity[0] * math.sin(theta)
                                 + velocity[1] * math.cos(theta))
                rolling = tuple(factor * center_speed
                                for factor in item.wheel_factors)
                lateral_acceleration = (abs(item.control) * speed ** 2
                                        if phase.edge.mode == MODE_A else 0.0)
            vehicles.append(UniformVehicleTimeState(
                vehicle_id=item.vehicle_id,
                x=centers[item.vehicle_id][0], y=centers[item.vehicle_id][1],
                theta=theta, velocity=velocity, acceleration=acceleration,
                jerk=jerk, yaw_rate_radps=yaw_rate,
                yaw_acceleration_radps2=yaw_acceleration,
                yaw_jerk_radps3=yaw_jerk, center_speed_mps=center_speed,
                longitudinal_speed_mps=longitudinal,
                lateral_speed_mps=lateral_speed,
                center_acceleration_mps2=acceleration_norm,
                center_jerk_mps3=jerk_norm,
                wheel_speed_mps=max(abs(value) for value in rolling),
                lateral_acceleration_mps2=lateral_acceleration,
                direction=item.direction, mode_control=item.control,
                wheel_angles=tuple(item.wheel_angles),
                wheel_angular_rates=(0.0, 0.0, 0.0, 0.0),
                wheel_angular_accelerations=(0.0, 0.0, 0.0, 0.0),
                wheel_rolling_speeds=rolling))
        return UniformTimeState(
            time_s=time_s, phase_id=phase.phase_id, phase_kind="motion",
            phase_index=phase_index, edge_index=phase.edge_index,
            mode=phase.edge.mode, qf=geometric.qf, vehicles=tuple(vehicles),
            progress=progress, progress_unit=phase.edge.progress_unit,
            progress_rate=progress_rate,
            progress_acceleration=progress_acceleration,
            progress_jerk=progress_jerk, qf_velocity=qf_velocity,
            qf_acceleration=qf_acceleration, qf_jerk=qf_jerk,
            from_mode=phase.edge.mode, to_mode=phase.edge.mode)

    def default_sample_times(self):
        if not self.phases:
            return (0.0,)
        values = {0.0, self.duration_s}
        for phase in self.phases:
            values.add(phase.t0_s)
            values.add(phase.end_time)
        return tuple(sorted(values))

    def _mode_intervals(self):
        if not self.phases:
            return []
        intervals = []
        for phase in self.phases:
            mode = legacy_mode_word(phase.mode)
            if intervals and intervals[-1]["mode"] == mode and abs(
                    intervals[-1]["end_time_s"] - phase.t0_s) <= (
                    TIME_CONTINUITY_TOL_S):
                intervals[-1]["end_time_s"] = phase.end_time
                intervals[-1]["duration_s"] = (
                    intervals[-1]["end_time_s"] - intervals[-1]["t0_s"])
            else:
                intervals.append({
                    "mode": mode, "t0_s": phase.t0_s,
                    "end_time_s": phase.end_time,
                    "duration_s": phase.duration_s,
                })
        return intervals

    def _aggregate_peaks(self):
        names = (
            "speed", "acceleration", "jerk", "yaw_rate",
            "yaw_acceleration", "yaw_jerk", "wheel_speed",
            "lateral_acceleration", "steer_rate", "steer_acceleration",
            "wheel_angle")
        result = {vehicle_id: {name: 0.0 for name in names}
                  for vehicle_id in self.vehicle_ids}
        for phase in self.phases:
            if isinstance(phase, MotionTiming):
                for vehicle_id, values in phase.peaks.items():
                    for name, value in values.items():
                        if name in result[vehicle_id]:
                            result[vehicle_id][name] = max(
                                result[vehicle_id][name], float(value))
            elif isinstance(phase, SetupTiming):
                for demand in phase.demands:
                    rate = 0.0
                    accel = 0.0
                    for delta in demand.wheel_angle_deltas:
                        rate = max(rate, 1.875 * abs(delta) / max(
                            phase.steering_duration_s, 1e-12))
                        accel = max(accel, (10.0 / math.sqrt(3.0)) * abs(delta)
                                    / max(phase.steering_duration_s ** 2, 1e-12))
                    values = result[demand.vehicle_id]
                    values["steer_rate"] = max(values["steer_rate"], rate)
                    values["steer_acceleration"] = max(
                        values["steer_acceleration"], accel)
                    values["wheel_angle"] = max(
                        values["wheel_angle"],
                        *(abs(value) for value in demand.wheel_angles_start),
                        *(abs(value) for value in demand.wheel_angles_end))
        return result

    def sample(self, times=None):
        """Project a common time grid; every value comes from ``state_at``."""
        values = self.default_sample_times() if times is None else tuple(times)
        if not values:
            raise UniformTrajectoryError(
                "EMPTY_SAMPLE_TIMES", "sample times must not be empty")
        checked = []
        for index, value in enumerate(values):
            time_s = _finite(value, "sample.times[{}]".format(index))
            if time_s < 0.0 or time_s > self.duration_s:
                raise UniformTrajectoryError(
                    "TIME_OUT_OF_RANGE", "sample time {} is outside the plan"
                    .format(time_s))
            checked.append(time_s)
        if abs(checked[0]) > TIME_CONTINUITY_TOL_S:
            raise UniformTrajectoryError(
                "PROJECTION_TIMES_MUST_START_AT_ZERO",
                "a common sampled projection starts at t=0; use state_at for "
                "an arbitrary isolated time")
        for first, second in zip(checked, checked[1:]):
            if second - first <= TIME_CONTINUITY_TOL_S:
                raise UniformTrajectoryError(
                    "TIME_GRID_NOT_INCREASING",
                    "sample times must increase by more than 1e-9 s")
        states = tuple(self.state_at(value) for value in checked)
        trajectories = {vehicle_id: [] for vehicle_id in self.vehicle_ids}
        for state in states:
            for vehicle in state.vehicles:
                trajectories[vehicle.vehicle_id].append(FormationTimedPoint(
                    time_from_start=state.time_s, center_x=vehicle.x,
                    center_y=vehicle.y, theta=vehicle.theta,
                    center_speed_mps=vehicle.center_speed_mps,
                    center_yaw_rate_radps=vehicle.yaw_rate_radps,
                    mode=legacy_mode_word(state.mode),
                    mode_control=vehicle.mode_control,
                    block_index=(0 if state.edge_index is None
                                 else state.edge_index),
                    station=state.progress, stationary=state.stationary,
                    center_acceleration_mps2=vehicle.center_acceleration_mps2,
                    center_jerk_mps3=vehicle.center_jerk_mps3,
                    wheel_speed_mps=vehicle.wheel_speed_mps,
                    lateral_acceleration_mps2=vehicle.lateral_acceleration_mps2))
        totals = self.phase_totals()
        phases = self.phase_table()
        stationary_events = [phase.as_dict() for phase in self.phases
                             if isinstance(phase, SetupTiming)]
        if not self.phases:
            stationary_events = [{
                "kind": "NO_MOTION", "t0_s": 0.0, "duration_s": 0.0,
                "qf": _pose_list(self.start_qf),
                "headings": _heading_records(self.start_headings),
            }]
        intervals = self._mode_intervals()
        report = FormationTimingReport(
            dynamic_profile="formation_piecewise_quintic_v1",
            dt=(min((b - a for a, b in zip(checked, checked[1:])), default=0.0)),
            duration_s=self.duration_s, moving_seconds=totals["motion_s"],
            stationary_seconds=(totals["initial_setup_s"]
                                + totals["inter_edge_setup_s"]),
            block_durations=tuple(phase.report for phase in self.phases
                                  if isinstance(phase, MotionTiming)),
            phases=tuple(copy.deepcopy(phases)),
            stationary_events=tuple(copy.deepcopy(stationary_events)),
            direction_changes=(), vehicle_peaks=self._aggregate_peaks(),
            active_constraints=tuple(dict(phase.driver) for phase in self.phases
                                     if isinstance(phase, MotionTiming)),
            mode_intervals={vehicle_id: tuple(copy.deepcopy(intervals))
                            for vehicle_id in self.vehicle_ids},
            limits=dict(self.limits.as_dict(),
                        uniform_semantics=PROJECTION_SEMANTICS))
        timed = FormationTimedTrajectory(
            plan_id=self.plan_id, formation_id=self.formation_id,
            offsets=self.model.offsets,
            formation_poses=tuple(state.qf for state in states),
            times_from_start=tuple(checked),
            trajectories={vehicle_id: tuple(points)
                          for vehicle_id, points in trajectories.items()},
            phases=tuple(copy.deepcopy(phases)), timing_report=report)
        return UniformSampleProjection(
            semantic_version=PROJECTION_SEMANTICS, model_version=MODEL_VERSION,
            plan_id=self.plan_id, model_sha256=self.model_sha256(),
            states=states, timed_trajectory=timed)

    def _initial_wheel_records(self):
        angles, factors = wheel_configuration(
            Action(ACKERMANN, 0.0, 0.0), self.motion_limits)
        return [{
            "vehicle_id": vehicle_id,
            "wheel_angles_rad": [float(value) for value in angles],
            "wheel_factors": [float(value) for value in factors],
        } for vehicle_id in self.vehicle_ids]

    def to_model_dict(self):
        return {
            "model_version": MODEL_VERSION,
            "semantics": MODEL_SEMANTICS,
            "plan_id": self.plan_id,
            "formation_id": self.formation_id,
            "offsets": [
                {"vehicle_id": offset.vehicle_id, "x": offset.x, "y": offset.y}
                for offset in self.model.offsets],
            "vehicle_shapes": [
                {"vehicle_id": vehicle_id,
                 "length_m": self.vehicle_shapes[vehicle_id].length_m,
                 "width_m": self.vehicle_shapes[vehicle_id].width_m}
                for vehicle_id in self.vehicle_ids],
            "timing_limits": self.limits.as_dict(),
            "s8_motion_limits": asdict(self.motion_limits),
            "initial_state": {
                "qf": _pose_list(self.start_qf),
                "headings": _heading_records(self.start_headings),
                "mode": MODE_A,
                "wheel_configurations": self._initial_wheel_records(),
            },
            "requested_goal": {
                "qf": _pose_list(self.goal_qf),
                "headings": _heading_records(self.goal_headings),
            },
            "edges": [_edge_record(edge) for edge in self.edges],
            "phases": [phase.as_dict() for phase in self.phases],
            "duration_s": self.duration_s,
            "certification": CERTIFICATION,
        }

    def model_sha256(self):
        """Factory-rebuild-stable semantic digest, not an equality proof.

        ``to_model_dict`` includes redundant values recomputed by FR01
        factories.  Rebuilding an edge from its factory parameters can change
        those diagnostics by a few ULPs.  The digest therefore rounds floats
        to 12 significant digits; the authoritative roundtrip check is the
        field-by-field comparison in ``from_model_dict`` and the tests.
        """
        payload = json.dumps(
            _stable_value(self.to_model_dict()), sort_keys=True,
            separators=(",", ":"), ensure_ascii=False, allow_nan=False
        ).encode("utf-8")
        return hashlib.sha256(payload).hexdigest()

    def to_model_json(self, indent=2):
        return json.dumps(
            self.to_model_dict(), sort_keys=True, indent=indent,
            ensure_ascii=False, allow_nan=False)

    @classmethod
    def from_model_json(cls, payload):
        if not isinstance(payload, str):
            raise UniformTrajectoryError(
                "INVALID_JSON", "model JSON payload must be text")
        def unique_object(pairs):
            result = {}
            for key, value in pairs:
                if key in result:
                    raise UniformTrajectoryError(
                        "DUPLICATE_JSON_KEY", "duplicate key {!r}".format(key))
                result[key] = value
            return result
        try:
            values = json.loads(payload, object_pairs_hook=unique_object)
        except UniformTrajectoryError:
            raise
        except (TypeError, ValueError) as exc:
            raise UniformTrajectoryError(
                "INVALID_JSON", str(exc)) from exc
        return cls.from_model_dict(values)

    @classmethod
    def from_model_dict(cls, values):
        """Rebuild factories and restore stored phases; never re-time them."""
        _mapping(values, "model")
        _assert_finite_json(values)
        if _pick(values, "model_version", "model") != MODEL_VERSION:
            raise UniformTrajectoryError(
                "UNKNOWN_MODEL_VERSION", "model_version={!r}; expected {}".format(
                    values.get("model_version"), MODEL_VERSION))
        if _pick(values, "semantics", "model") != MODEL_SEMANTICS:
            raise UniformTrajectoryError(
                "INVALID_MODEL_SEMANTICS", "semantics={!r}".format(
                    values.get("semantics")))
        try:
            offsets = []
            for index, record in enumerate(_sequence(
                    _pick(values, "offsets", "model"), "model.offsets")):
                offsets.append(VehicleOffset(
                    _integer(_pick(record, "vehicle_id", "offset"),
                             "offsets[{}].vehicle_id".format(index),
                             minimum=0, maximum=UINT32_MAX),
                    _finite(_pick(record, "x", "offset"),
                            "offsets[{}].x".format(index)),
                    _finite(_pick(record, "y", "offset"),
                            "offsets[{}].y".format(index))))
            model = FormationModel(offsets)
            shapes = {}
            for record in _sequence(
                    _pick(values, "vehicle_shapes", "model"),
                    "model.vehicle_shapes"):
                vehicle_id = _integer(
                    _pick(record, "vehicle_id", "vehicle_shape"),
                    "vehicle_shape.vehicle_id", minimum=0, maximum=UINT32_MAX)
                if vehicle_id in shapes:
                    raise UniformTrajectoryError(
                        "DUPLICATE_VEHICLE_ID",
                        "duplicate shape vehicle_id={}".format(vehicle_id))
                shapes[vehicle_id] = VehicleShape(
                    _finite(_pick(record, "length_m", "vehicle_shape"),
                            "vehicle_shape.length_m"),
                    _finite(_pick(record, "width_m", "vehicle_shape"),
                            "vehicle_shape.width_m"))
            timing_values = _mapping(
                _pick(values, "timing_limits", "model"), "timing_limits")
            limits = FormationTimingLimits(**dict(timing_values))
            motion_limits = timing_motion_limits(limits)
            _compare_json(
                asdict(motion_limits),
                _pick(values, "s8_motion_limits", "model"),
                "model.s8_motion_limits")
            initial_values = _pick(values, "initial_state", "model")
            start_qf = _pose(
                _pick(initial_values, "qf", "initial_state"), "initial.qf")
            start_headings = _headings(
                _pick(initial_values, "headings", "initial_state"),
                model.vehicle_ids, "initial.headings")
            goal_values = _pick(values, "requested_goal", "model")
            goal_qf = _pose(
                _pick(goal_values, "qf", "requested_goal"), "goal.qf")
            goal_headings = _headings(
                _pick(goal_values, "headings", "requested_goal"),
                model.vehicle_ids, "goal.headings")
            edges = tuple(_decode_edge(record, model) for record in _sequence(
                _pick(values, "edges", "model"), "model.edges"))
            if edges:
                validate_uniform_edge_sequence(edges, limits)
            phases = []
            for record in _sequence(
                    _pick(values, "phases", "model"), "model.phases"):
                kind = _pick(record, "kind", "phase")
                if kind == "motion":
                    phases.append(_decode_motion(record, edges, limits,
                                                 motion_limits))
                elif kind in ("initial_setup", "setup"):
                    phases.append(_decode_setup(record))
                else:
                    raise UniformTrajectoryError(
                        "INVALID_PHASE_KIND", "kind={!r}".format(kind))
            plan = cls(
                plan_id=_integer(_pick(values, "plan_id", "model"), "plan_id",
                                 minimum=0, maximum=UINT32_MAX),
                formation_id=_string(
                    _pick(values, "formation_id", "model"), "formation_id"),
                model=model, vehicle_shapes=shapes, edges=edges,
                phases=tuple(phases), limits=limits,
                motion_limits=motion_limits, start_qf=start_qf,
                start_headings=start_headings, goal_qf=goal_qf,
                goal_headings=goal_headings,
                duration_s=_finite(_pick(values, "duration_s", "model"),
                                   "model.duration_s"))
            _validate_plan_structure(plan)
            _compare_json(plan.to_model_dict(), values, "model")
            return plan
        except UniformMotionError:
            raise
        except (FormationValidationError, KeyError, TypeError, ValueError) as exc:
            raise UniformTrajectoryError(
                "INVALID_MODEL_STRUCTURE", str(exc)) from exc


def _motion_limits_from_dict(values):
    allowed = set(MotionLimits.__dataclass_fields__)
    _mapping(values, "edge.limits")
    if set(values) != allowed:
        raise UniformTrajectoryError(
            "INVALID_LIMITS", "MotionLimits fields differ: missing={} extra={}"
            .format(sorted(allowed - set(values)), sorted(set(values) - allowed)))
    return MotionLimits(**dict(values))


def _edge_record(edge):
    start_headings = _heading_records(edge.start_headings)
    directions = [
        {"vehicle_id": vehicle_id, "direction": direction}
        for vehicle_id, direction in edge.directions]
    factory = {
        "primitive": edge.primitive,
        "start_qf": _pose_list(edge.start_qf),
        "start_headings": start_headings,
        "directions": directions,
        "limits": asdict(edge.limits),
        "allow_reverse": edge.allow_reverse,
    }
    if edge.primitive in (PRIMITIVE_A_STRAIGHT, PRIMITIVE_O_TRANSLATION):
        factory["displacement"] = [
            edge.end_qf.x - edge.start_qf.x,
            edge.end_qf.y - edge.start_qf.y]
    if edge.primitive == PRIMITIVE_A_ARC:
        factory["delta_psi"] = edge.control.delta_psi
        factory["pivot"] = [edge.control.pivot_x, edge.control.pivot_y]
    if edge.primitive == PRIMITIVE_O_TRANSLATION:
        factory["oblique_angles"] = [
            {"vehicle_id": vehicle_id, "phi": phi}
            for vehicle_id, phi in edge.oblique_angles]
    if edge.primitive == PRIMITIVE_S_SPIN:
        starts = _pair_map(edge.start_headings)
        ends = _pair_map(edge.end_headings)
        factory["delta_thetas"] = [
            {"vehicle_id": vehicle_id,
             "delta_theta": ends[vehicle_id] - starts[vehicle_id]}
            for vehicle_id in edge.model.vehicle_ids]
    return {"factory": factory, "redundant": edge.as_dict()}


def _direction_input(records, vehicle_ids):
    result = {}
    for record in _sequence(records, "edge.directions"):
        vehicle_id = _integer(
            _pick(record, "vehicle_id", "direction"), "direction.vehicle_id")
        if vehicle_id in result:
            raise UniformTrajectoryError(
                "DUPLICATE_VEHICLE_ID", "duplicate direction ID {}".format(
                    vehicle_id))
        result[vehicle_id] = _integer(
            _pick(record, "direction", "direction"), "direction.direction")
    if set(result) != set(vehicle_ids):
        raise UniformTrajectoryError(
            "INVALID_VEHICLE_ID_SET", "edge direction IDs differ")
    return result


def _decode_edge(record, model):
    factory = _pick(record, "factory", "edge")
    primitive = _pick(factory, "primitive", "edge.factory")
    qf = _pose(_pick(factory, "start_qf", "edge.factory"), "edge.start_qf")
    headings = dict(_headings(
        _pick(factory, "start_headings", "edge.factory"), model.vehicle_ids,
        "edge.start_headings"))
    directions = _direction_input(
        _pick(factory, "directions", "edge.factory"), model.vehicle_ids)
    limits = _motion_limits_from_dict(_pick(factory, "limits", "edge.factory"))
    allow_reverse = _pick(factory, "allow_reverse", "edge.factory")
    if not isinstance(allow_reverse, bool):
        raise UniformTrajectoryError(
            "INVALID_ALLOW_REVERSE", "allow_reverse must be bool")
    if primitive == PRIMITIVE_A_STRAIGHT:
        edge = a_straight_edge(
            model, qf, headings,
            _pick(factory, "displacement", "edge.factory"), directions,
            limits=limits, allow_reverse=allow_reverse)
    elif primitive == PRIMITIVE_A_ARC:
        edge = a_arc_edge(
            model, qf, headings,
            _pick(factory, "delta_psi", "edge.factory"),
            _pick(factory, "pivot", "edge.factory"), directions,
            limits=limits, allow_reverse=allow_reverse)
    elif primitive == PRIMITIVE_O_TRANSLATION:
        phis = {}
        for item in _sequence(
                _pick(factory, "oblique_angles", "edge.factory"),
                "edge.oblique_angles"):
            vehicle_id = _integer(
                _pick(item, "vehicle_id", "oblique"), "oblique.vehicle_id")
            if vehicle_id in phis:
                raise UniformTrajectoryError(
                    "DUPLICATE_VEHICLE_ID", "duplicate oblique ID {}".format(
                        vehicle_id))
            phis[vehicle_id] = _finite(
                _pick(item, "phi", "oblique"), "oblique.phi")
        if set(phis) != set(model.vehicle_ids):
            raise UniformTrajectoryError(
                "INVALID_VEHICLE_ID_SET", "oblique IDs differ")
        edge = o_translation_edge(
            model, qf, headings,
            _pick(factory, "displacement", "edge.factory"), directions, phis,
            limits=limits, allow_reverse=allow_reverse)
    elif primitive == PRIMITIVE_S_SPIN:
        deltas = {}
        for item in _sequence(
                _pick(factory, "delta_thetas", "edge.factory"),
                "edge.delta_thetas"):
            vehicle_id = _integer(
                _pick(item, "vehicle_id", "delta_theta"),
                "delta_theta.vehicle_id")
            if vehicle_id in deltas:
                raise UniformTrajectoryError(
                    "DUPLICATE_VEHICLE_ID", "duplicate spin ID {}".format(
                        vehicle_id))
            deltas[vehicle_id] = _finite(
                _pick(item, "delta_theta", "delta_theta"),
                "delta_theta.delta_theta")
        if set(deltas) != set(model.vehicle_ids):
            raise UniformTrajectoryError(
                "INVALID_VEHICLE_ID_SET", "spin IDs differ")
        edge = s_spin_edge(model, qf, headings, deltas, limits=limits)
    else:
        raise UniformTrajectoryError(
            "INVALID_PRIMITIVE", "primitive={!r}".format(primitive))
    _compare_json(_edge_record(edge), record, "edge")
    return edge


def _decode_motion(record, edges, limits, motion_limits):
    edge_index = _integer(
        _pick(record, "edge_index", "motion"), "motion.edge_index", minimum=0)
    if edge_index >= len(edges):
        raise UniformTrajectoryError(
            "INVALID_EDGE_INDEX", "motion edge_index={}".format(edge_index))
    peaks = {}
    for item in _sequence(
            _pick(record, "vehicle_peaks", "motion"), "motion.vehicle_peaks"):
        vehicle_id = _integer(
            _pick(item, "vehicle_id", "vehicle_peaks"),
            "vehicle_peaks.vehicle_id")
        if vehicle_id in peaks:
            raise UniformTrajectoryError(
                "DUPLICATE_VEHICLE_ID", "duplicate motion peak ID {}".format(
                    vehicle_id))
        peaks[vehicle_id] = dict(_mapping(
            _pick(item, "peaks", "vehicle_peaks"), "vehicle_peaks.values"))
    return restore_motion_timing(
        edges[edge_index], edge_index,
        _string(_pick(record, "phase_id", "motion"), "motion.phase_id"),
        _finite(_pick(record, "t0_s", "motion"), "motion.t0_s"),
        _finite(_pick(record, "duration_s", "motion"), "motion.duration_s"),
        _finite(_pick(record, "required_duration_s", "motion"),
                "motion.required_duration_s"),
        _finite(_pick(record, "dynamic_duration_s", "motion"),
                "motion.dynamic_duration_s"),
        dict(_mapping(_pick(record, "duration_driver", "motion"),
                      "motion.duration_driver")),
        tuple(dict(_mapping(item, "limiting_constraint")) for item in _sequence(
            _pick(record, "limiting_constraints", "motion"),
            "motion.limiting_constraints")),
        peaks, limits, motion_limits)


def _decode_setup(record):
    demands = []
    for item in _sequence(
            _pick(record, "demands", "setup"), "setup.demands"):
        def four(name):
            values = tuple(_finite(value, "setup." + name) for value in _sequence(
                _pick(item, name, "setup.demand"), "setup." + name))
            if len(values) != 4:
                raise UniformTrajectoryError(
                    "INVALID_PAIR_FIELD", "{} must contain four values".format(name))
            return values
        demands.append(VehicleSetupDemand(
            vehicle_id=_integer(_pick(item, "vehicle_id", "demand"),
                                "demand.vehicle_id"),
            from_mode=_string(_pick(item, "from_mode", "demand"), "from_mode"),
            to_mode=_string(_pick(item, "to_mode", "demand"), "to_mode"),
            from_direction=_integer(_pick(item, "from_direction", "demand"),
                                    "from_direction"),
            to_direction=_integer(_pick(item, "to_direction", "demand"),
                                  "to_direction"),
            from_control=_finite(_pick(item, "from_control", "demand"),
                                 "from_control"),
            to_control=_finite(_pick(item, "to_control", "demand"), "to_control"),
            wheel_angles_start=four("wheel_angles_start_rad"),
            wheel_angles_end=four("wheel_angles_end_rad"),
            wheel_angle_deltas=four("wheel_angle_deltas_rad"),
            max_wheel_angle_delta_rad=_finite(
                _pick(item, "max_wheel_angle_delta_rad", "demand"),
                "max_wheel_angle_delta_rad"),
            event_duration_s=_finite(_pick(item, "event_duration_s", "demand"),
                                     "event_duration_s"),
            setup_s=_finite(_pick(item, "setup_s", "demand"), "setup_s"),
            steering_s=_finite(_pick(item, "steering_s", "demand"), "steering_s"),
            steering_profile=_string(
                _pick(item, "steering_profile", "demand"), "steering_profile"),
            event_steer_rate_peak_radps=_finite(
                _pick(item, "event_steer_rate_peak_radps", "demand"),
                "event_steer_rate_peak_radps"),
            event_steer_accel_peak_radps2=_finite(
                _pick(item, "event_steer_accel_peak_radps2", "demand"),
                "event_steer_accel_peak_radps2")))
    return SetupTiming(
        phase_id=_string(_pick(record, "phase_id", "setup"), "setup.phase_id"),
        edge_index=_integer(_pick(record, "edge_index", "setup"),
                            "setup.edge_index", minimum=0),
        t0_s=_finite(_pick(record, "t0_s", "setup"), "setup.t0_s"),
        duration_s=_finite(_pick(record, "duration_s", "setup"),
                           "setup.duration_s"),
        setup_overhead_s=_finite(
            _pick(record, "setup_overhead_s", "setup"), "setup_overhead_s"),
        steering_duration_s=_finite(
            _pick(record, "steering_duration_s", "setup"),
            "steering_duration_s"),
        initial=_pick(record, "initial", "setup"),
        from_mode=_string(_pick(record, "from_mode", "setup"), "from_mode"),
        to_mode=_string(_pick(record, "to_mode", "setup"), "to_mode"),
        qf=_pose(_pick(record, "qf", "setup"), "setup.qf"),
        headings=tuple(
            (item["vehicle_id"], item["theta"])
            for item in _pick(record, "headings", "setup")),
        demands=tuple(demands),
        common_steer_rate_peak_radps=_finite(
            _pick(record, "common_steer_rate_peak_radps", "setup"),
            "common_steer_rate_peak_radps"),
        common_steer_accel_peak_radps2=_finite(
            _pick(record, "common_steer_accel_peak_radps2", "setup"),
            "common_steer_accel_peak_radps2"))


def _check_pose_continuity(actual, expected, context, tolerance_m=None,
                           tolerance_rad=HEADING_CONTINUITY_TOL_RAD):
    if tolerance_m is None:
        tolerance_m = GEOMETRY_CONTINUITY_TOL_M
    errors = (abs(actual.x - expected.x), abs(actual.y - expected.y),
              abs(actual.psi - expected.psi))
    if errors[0] > tolerance_m or errors[1] > tolerance_m or (
            errors[2] > tolerance_rad):
        raise UniformTrajectoryError(
            "REQUESTED_POSE_MISMATCH",
            "{} error={} exceeds raw-pose continuity".format(context, errors))


def _check_heading_continuity(actual, expected, context):
    expected_map = _pair_map(expected)
    for vehicle_id, theta in actual:
        if abs(theta - expected_map[vehicle_id]) > HEADING_CONTINUITY_TOL_RAD:
            raise UniformTrajectoryError(
                "REQUESTED_HEADING_MISMATCH",
                "{} vehicle_id={} unwrapped heading mismatch".format(
                    context, vehicle_id), vehicle_id=vehicle_id)


def _validate_setup(plan, phase, phase_index):
    edge = plan.edges[phase.edge_index]
    if (len(phase.headings) != len(plan.vehicle_ids) or
            tuple(vehicle_id for vehicle_id, _ in phase.headings) !=
            plan.vehicle_ids):
        raise UniformTrajectoryError(
            "INVALID_VEHICLE_ID_SET",
            "setup headings must contain every vehicle exactly once in order",
            phase_id=phase.phase_id)
    if phase.setup_overhead_s < 0.0 or phase.steering_duration_s <= 0.0:
        raise UniformTrajectoryError(
            "NONPOSITIVE_SETUP_COMPONENT",
            "setup overhead must be nonnegative and steering positive",
            phase_id=phase.phase_id)
    _stable_phase_duration(
        phase.steering_duration_s, "phase.steering_duration_s", phase.phase_id)
    if phase.qf != edge.start_qf:
        _check_pose_continuity(phase.qf, edge.start_qf, "setup.qf")
    _check_heading_continuity(phase.headings, edge.start_headings, "setup")
    if phase.initial:
        previous_mode = MODE_A
        previous_segments = None
        angles, _ = wheel_configuration(
            Action(ACKERMANN, 0.0, 0.0), plan.motion_limits)
        previous_angles = {vehicle_id: tuple(float(value) for value in angles)
                           for vehicle_id in plan.vehicle_ids}
        previous_directions = {vehicle_id: 1 for vehicle_id in plan.vehicle_ids}
        previous_controls = {vehicle_id: 0.0 for vehicle_id in plan.vehicle_ids}
    else:
        previous = plan.edges[phase.edge_index - 1]
        previous_mode = previous.mode
        previous_segments = {segment.vehicle_id: segment
                             for segment in previous.segments}
        previous_angles = {configuration.vehicle_id: configuration.angles
                           for configuration in previous.wheel_configurations(
                               at_end=True)}
        previous_directions = _pair_map(previous.directions)
        previous_controls = {vehicle_id: segment.control
                             for vehicle_id, segment in previous_segments.items()}
    current_angles = {configuration.vehicle_id: configuration.angles
                      for configuration in edge.wheel_configurations(at_end=False)}
    current_directions = _pair_map(edge.directions)
    current_controls = {segment.vehicle_id: segment.control
                        for segment in edge.segments}
    if phase.from_mode != previous_mode or phase.to_mode != edge.mode:
        raise UniformTrajectoryError(
            "SETUP_MODE_MISMATCH", "setup from/to modes do not match its edges",
            edge_index=phase.edge_index, phase_id=phase.phase_id)
    demands = {demand.vehicle_id: demand for demand in phase.demands}
    if set(demands) != set(plan.vehicle_ids) or len(demands) != len(plan.vehicle_ids):
        raise UniformTrajectoryError(
            "INVALID_VEHICLE_ID_SET", "setup demand IDs differ",
            phase_id=phase.phase_id)
    any_change = previous_mode != edge.mode
    for vehicle_id, demand in demands.items():
        expected_deltas = tuple(
            _wrap(end - start) for start, end in zip(
                previous_angles[vehicle_id], current_angles[vehicle_id]))
        delta = max(abs(value) for value in expected_deltas)
        any_change = any_change or delta > 1e-9
        expected_event = mode_switch_event(delta, plan.motion_limits)
        expected = VehicleSetupDemand(
            vehicle_id=vehicle_id, from_mode=previous_mode, to_mode=edge.mode,
            from_direction=previous_directions[vehicle_id],
            to_direction=current_directions[vehicle_id],
            from_control=previous_controls[vehicle_id],
            to_control=current_controls[vehicle_id],
            wheel_angles_start=tuple(previous_angles[vehicle_id]),
            wheel_angles_end=tuple(current_angles[vehicle_id]),
            wheel_angle_deltas=expected_deltas,
            max_wheel_angle_delta_rad=delta,
            event_duration_s=float(expected_event["duration"]),
            setup_s=float(expected_event["setup_s"]),
            steering_s=float(expected_event["steering_s"]),
            steering_profile=str(expected_event["steering_profile"]),
            event_steer_rate_peak_radps=float(expected_event["steer_rate_peak"]),
            event_steer_accel_peak_radps2=float(
                expected_event["steer_accel_peak"]))
        _compare_json(demand.as_dict(), expected.as_dict(),
                      "setup.vehicle_{}".format(vehicle_id))
    if not any_change:
        raise UniformTrajectoryError(
            "UNNECESSARY_SETUP_PHASE", "same mode/configuration needs no setup",
            phase_id=phase.phase_id)
    if abs(phase.duration_s - phase.setup_overhead_s
           - phase.steering_duration_s) > TIME_CONTINUITY_TOL_S:
        raise UniformTrajectoryError(
            "SETUP_DURATION_DECOMPOSITION_MISMATCH",
            "duration != setup overhead + steering", phase_id=phase.phase_id)
    rate = 0.0
    accel = 0.0
    for demand in phase.demands:
        for delta in demand.wheel_angle_deltas:
            rate = max(rate, 1.875 * abs(delta) / max(
                phase.steering_duration_s, 1e-12))
            accel = max(accel, (10.0 / math.sqrt(3.0)) * abs(delta) / max(
                phase.steering_duration_s ** 2, 1e-12))
    if abs(rate - phase.common_steer_rate_peak_radps) > FIELD_TOL or abs(
            accel - phase.common_steer_accel_peak_radps2) > FIELD_TOL:
        raise UniformTrajectoryError(
            "SETUP_PEAK_MISMATCH", "stored common setup peaks are inconsistent",
            phase_id=phase.phase_id)


def _validate_plan_structure(plan):
    ids = plan.vehicle_ids
    if tuple(vehicle_id for vehicle_id, _ in plan.start_headings) != ids or (
            tuple(vehicle_id for vehicle_id, _ in plan.goal_headings) != ids):
        raise UniformTrajectoryError(
            "INVALID_VEHICLE_ID_SET", "start/goal heading IDs differ")
    if not plan.edges:
        if plan.phases or plan.duration_s != 0.0:
            raise UniformTrajectoryError(
                "INVALID_ZERO_MOTION_PLAN",
                "an empty edge sequence must have zero duration and no phases")
        _check_pose_continuity(plan.start_qf, plan.goal_qf, "zero-motion request")
        _check_heading_continuity(
            plan.start_headings, plan.goal_headings, "zero-motion request")
        return
    validate_uniform_edge_sequence(plan.edges, plan.limits)
    _check_pose_continuity(plan.start_qf, plan.edges[0].start_qf, "requested start")
    _check_heading_continuity(
        plan.start_headings, plan.edges[0].start_headings, "requested start")
    _check_pose_continuity(plan.goal_qf, plan.edges[-1].end_qf, "requested goal")
    _check_heading_continuity(
        plan.goal_headings, plan.edges[-1].end_headings, "requested goal")
    if not plan.phases:
        raise UniformTrajectoryError(
            "MISSING_MOTION_PHASE", "nonempty edges require motion phases")
    cursor = 0.0
    expected_edge = 0
    setup_allowed = True
    phase_ids = set()
    for index, phase in enumerate(plan.phases):
        if phase.phase_id in phase_ids:
            raise UniformTrajectoryError(
                "DUPLICATE_PHASE_ID", "phase_id={!r}".format(phase.phase_id))
        phase_ids.add(phase.phase_id)
        if phase.duration_s <= 0.0:
            raise UniformTrajectoryError(
                "NONPOSITIVE_PHASE_DURATION",
                "phase {} duration={}".format(phase.phase_id, phase.duration_s),
                phase_id=phase.phase_id)
        _finite(phase.t0_s, "phase.t0_s")
        _finite(phase.duration_s, "phase.duration_s")
        _stable_phase_duration(
            phase.duration_s, "phase.duration_s", phase.phase_id)
        end_time = phase.end_time
        if not math.isfinite(end_time) or end_time <= phase.t0_s:
            raise UniformTrajectoryError(
                "PHASE_TIME_UNREPRESENTABLE",
                "phase {} t0={:.17g} duration={:.17g} end={!r}".format(
                    phase.phase_id, phase.t0_s, phase.duration_s, end_time),
                phase_id=phase.phase_id)
        if isinstance(phase, MotionTiming):
            if phase.required_duration_s < 0.0 or phase.dynamic_duration_s <= 0.0:
                raise UniformTrajectoryError(
                    "NONPOSITIVE_MOTION_COMPONENT",
                    "required duration must be nonnegative and dynamic positive",
                    phase_id=phase.phase_id)
            if set(phase.peaks) != set(plan.vehicle_ids) or len(
                    phase.peaks) != len(plan.vehicle_ids):
                raise UniformTrajectoryError(
                    "INVALID_VEHICLE_ID_SET", "motion peak IDs differ",
                    phase_id=phase.phase_id)
        # A persisted phases axis must cover [0, T_end] exactly.  The old
        # tolerance accepted real gaps/overlaps that _phase_at cannot evaluate.
        difference = phase.t0_s - cursor
        if difference != 0.0:
            raise UniformTrajectoryError(
                "PHASE_TIME_DISCONTINUITY",
                "phase {} has {} {:.12g} s".format(
                    phase.phase_id, "gap" if difference > 0.0 else "overlap",
                    abs(difference)), phase_id=phase.phase_id)
        if isinstance(phase, SetupTiming):
            if phase.edge_index != expected_edge or not setup_allowed:
                raise UniformTrajectoryError(
                    "SETUP_PHASE_ORDER", "setup {} is not before edge {}".format(
                        phase.phase_id, expected_edge), phase_id=phase.phase_id)
            if phase.initial != (index == 0 and expected_edge == 0):
                raise UniformTrajectoryError(
                    "INITIAL_SETUP_MISPLACED", "initial setup must be first",
                    phase_id=phase.phase_id)
            if not isinstance(phase.initial, bool):
                raise UniformTrajectoryError(
                    "INVALID_INITIAL_FLAG", "initial must be bool")
            _validate_setup(plan, phase, index)
            setup_allowed = False
        elif isinstance(phase, MotionTiming):
            if phase.edge_index != expected_edge:
                raise UniformTrajectoryError(
                    "MOTION_PHASE_ORDER", "motion edge {} expected {}".format(
                        phase.edge_index, expected_edge), phase_id=phase.phase_id)
            if phase.edge is not plan.edges[expected_edge]:
                raise UniformTrajectoryError(
                    "MOTION_EDGE_IDENTITY", "motion does not reference its edge",
                    phase_id=phase.phase_id)
            expected_edge += 1
            setup_allowed = True
        else:
            raise UniformTrajectoryError(
                "INVALID_PHASE_TYPE", "unsupported phase type")
        cursor = end_time
    if expected_edge != len(plan.edges):
        raise UniformTrajectoryError(
            "MISSING_MOTION_PHASE", "decoded phases do not cover every edge")
    if cursor != plan.duration_s:
        raise UniformTrajectoryError(
            "PLAN_DURATION_MISMATCH",
            "phase sum {:.17g} != duration {:.17g}".format(
                cursor, plan.duration_s))


def build_uniform_timed_plan(edges, vehicle_shapes, **kwargs):
    """Functional convenience wrapper around :meth:`UniformTimedPlan.from_edges`."""
    return UniformTimedPlan.from_edges(
        edges, vehicle_shapes=vehicle_shapes, **kwargs)


def model_from_json(payload):
    return UniformTimedPlan.from_model_json(payload)
