"""FR02-A common timing for authoritative uniform-mode analytic edges.

This module is the only FR02 adapter that calls private S8 analysis helpers.
It converts each FR01 edge directly to a legacy :class:`FormationBlock` view,
uses ``_analyze_block`` for the all-vehicle common motion duration, and adds
initial/inter-edge stationary wheel reconfiguration.  It does not call the
legacy full retimer, choose geometry, or reinterpret the V0.2 speed fields.

The returned schedule is a timing/analysis object.  The authoritative time
 evaluator and versioned model live in :mod:`uniform_trajectory`.
"""
from dataclasses import dataclass
import math
from typing import Mapping, Optional, Tuple

from .core import Action
from .formation import FormationPose, FormationValidationError
from .formation_geometry import FormationBlock
from .formation_kinematics import ACKERMANN, OBLIQUE, SPIN
from .formation_retiming import (
    FormationTimingLimits,
    _TimingInfeasible,
    _analyze_block,
    _context_for,
    _formation_derivatives,
    _motion_limits,
    _vehicle_derivatives,
)
from .mode_switch_timing import mode_switch_event
from .retiming import MotionLimits, wheel_configuration
from .uniform_motion import (
    MODE_A,
    MODE_O,
    MODE_S,
    UniformMotionEdge,
    UniformMotionError,
    validate_uniform_edge,
)

GEOMETRY_CONTINUITY_TOL_M = 1e-7
HEADING_CONTINUITY_TOL_RAD = 1e-7
LIMIT_MATCH_TOL = 1e-9
WHEEL_CHANGE_TOL_RAD = 1e-9
SETUP_LIMIT_TOL = 1e-9

_MODE_TO_LEGACY = {MODE_A: ACKERMANN, MODE_O: OBLIQUE, MODE_S: SPIN}
_LEGACY_TO_MODE = {value: key for key, value in _MODE_TO_LEGACY.items()}


class UniformTimingError(UniformMotionError):
    """Explicit common-timing rejection with structured location fields."""

    def __init__(self, reason, detail, edge_index=None, vehicle_id=None,
                 phase_id=None, constraint=None):
        self.edge_index = edge_index
        self.phase_id = phase_id
        self.constraint = constraint
        super().__init__(reason, detail, vehicle_id=vehicle_id)


def _finite(value, context):
    if isinstance(value, (bool, str, bytes)):
        raise UniformTimingError(
            "NONFINITE_VALUE", "{}={!r} is not a finite number".format(
                context, value))
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise UniformTimingError(
            "NONFINITE_VALUE", "{}={!r} is not a finite number".format(
                context, value)) from exc
    if not math.isfinite(result):
        raise UniformTimingError(
            "NONFINITE_VALUE", "{}={!r} is not finite".format(context, value))
    return result


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _pair_map(pairs):
    return {vehicle_id: value for vehicle_id, value in pairs}


def _pose_values(qf):
    return (qf.x, qf.y, qf.psi)


def _mode_word(mode):
    if mode not in _MODE_TO_LEGACY:
        raise UniformTimingError(
            "INVALID_MODE", "mode={!r} is not A/O/S".format(mode))
    return _MODE_TO_LEGACY[mode]


def legacy_mode_word(mode):
    """Public A/O/S to legacy numeric mode conversion for projections."""
    return _mode_word(mode)


def timing_motion_limits(limits):
    """Construct S8's restricted MotionLimits at the adapter boundary."""
    if not isinstance(limits, FormationTimingLimits):
        raise UniformTimingError(
            "INVALID_TIMING_LIMITS", "expected FormationTimingLimits")
    return _motion_limits(limits)


def uniform_mode_word(legacy_mode):
    if legacy_mode not in _LEGACY_TO_MODE:
        raise UniformTimingError(
            "INVALID_MODE", "legacy mode={!r} is not A/O/S".format(
                legacy_mode))
    return _LEGACY_TO_MODE[legacy_mode]


def edge_to_block(edge, block_index):
    """Direct FR01 -> S8 view; no route/mode/heading construction occurs."""
    if not isinstance(edge, UniformMotionEdge):
        raise UniformTimingError(
            "INVALID_EDGE_TYPE", "expected UniformMotionEdge, got {}".format(
                type(edge).__name__), edge_index=block_index)
    if isinstance(block_index, bool) or not isinstance(block_index, int) or (
            block_index < 0):
        raise UniformTimingError(
            "INVALID_EDGE_INDEX", "block_index={!r}".format(block_index))
    return FormationBlock(
        geometry_type=edge.control.geometry_type,
        parameter_length=edge.control.parameter_length,
        formation_controls=(edge.control,),
        vehicle_mode_segments=tuple(edge.segments),
        block_index=block_index)


def _offsets_key(edge):
    return tuple(
        (offset.vehicle_id, offset.x, offset.y)
        for offset in edge.model.offsets)


def _check_geometry_limits(edge, limits, edge_index):
    expected = (
        ("wheelbase", edge.limits.wheelbase, limits.wheelbase),
        ("track", edge.limits.track, limits.track),
        ("r_min_ack", edge.limits.r_min_ack, limits.r_min_ackermann),
        ("steer_angle_max", edge.limits.steer_angle_max,
         limits.steer_angle_max),
    )
    for name, actual, required in expected:
        if abs(actual - required) > LIMIT_MATCH_TOL:
            raise UniformTimingError(
                "TIMING_GEOMETRY_LIMIT_MISMATCH",
                "edge {} {}={:.17g} does not match TimingLimits {:.17g}"
                .format(edge_index, name, actual, required),
                edge_index=edge_index, constraint=name)


def validate_uniform_edge_sequence(edges, limits):
    """Validate exact IDs/offsets, raw continuity, and timing compatibility."""
    try:
        sequence = tuple(edges)
    except TypeError as exc:
        raise UniformTimingError(
            "INVALID_EDGE_SEQUENCE", "edges must be an iterable") from exc
    if not sequence:
        raise UniformTimingError(
            "EMPTY_EDGE_SEQUENCE", "timing a hold uses the explicit hold API")
    if not isinstance(limits, FormationTimingLimits):
        raise UniformTimingError(
            "INVALID_TIMING_LIMITS", "expected FormationTimingLimits")
    if limits.min_mode_dwell_s <= 0.0:
        raise UniformTimingError(
            "MIN_MODE_DWELL_REQUIRED",
            "V0.2 requires a positive min_mode_dwell_s; got {}".format(
                limits.min_mode_dwell_s))
    reference = None
    previous = None
    for index, edge in enumerate(sequence):
        if not isinstance(edge, UniformMotionEdge):
            raise UniformTimingError(
                "INVALID_EDGE_TYPE", "edge {} has type {}".format(
                    index, type(edge).__name__), edge_index=index)
        validate_uniform_edge(edge)
        _check_geometry_limits(edge, limits, index)
        if reference is None:
            reference = _offsets_key(edge)
        else:
            actual = _offsets_key(edge)
            reference_ids = tuple(item[0] for item in reference)
            actual_ids = tuple(item[0] for item in actual)
            if actual_ids != reference_ids:
                raise UniformTimingError(
                    "EDGE_VEHICLE_IDS_MISMATCH",
                    "edge {} IDs {} != {}".format(
                        index, actual_ids, reference_ids), edge_index=index)
            if actual != reference:
                raise UniformTimingError(
                    "EDGE_OFFSETS_MISMATCH",
                    "edge {} offsets differ from edge 0".format(index),
                    edge_index=index)
        if previous is not None:
            qf_error = tuple(
                abs(a - b) for a, b in zip(
                    _pose_values(previous.end_qf),
                    _pose_values(edge.start_qf)))
            if qf_error[0] > GEOMETRY_CONTINUITY_TOL_M or (
                    qf_error[1] > GEOMETRY_CONTINUITY_TOL_M):
                raise UniformTimingError(
                    "EDGE_QF_DISCONTINUITY",
                    "edge {} position jump=({:.12g}, {:.12g}) m".format(
                        index, qf_error[0], qf_error[1]), edge_index=index)
            if qf_error[2] > HEADING_CONTINUITY_TOL_RAD:
                raise UniformTimingError(
                    "EDGE_QF_DISCONTINUITY",
                    "edge {} raw psi jump={:.12g} rad".format(
                        index, qf_error[2]), edge_index=index)
            previous_headings = _pair_map(previous.end_headings)
            current_headings = _pair_map(edge.start_headings)
            for vehicle_id in previous.model.vehicle_ids:
                error = abs(previous_headings[vehicle_id]
                            - current_headings[vehicle_id])
                if error > HEADING_CONTINUITY_TOL_RAD:
                    raise UniformTimingError(
                        "EDGE_HEADING_DISCONTINUITY",
                        "edge {} vehicle_id={} unwrapped heading jump={:.12g} "
                        "rad; setup cannot repair it".format(
                            index, vehicle_id, error), edge_index=index,
                        vehicle_id=vehicle_id)
        previous = edge
    return sequence


@dataclass(frozen=True)
class VehicleSetupDemand:
    vehicle_id: int
    from_mode: str
    to_mode: str
    from_direction: int
    to_direction: int
    from_control: float
    to_control: float
    wheel_angles_start: Tuple[float, float, float, float]
    wheel_angles_end: Tuple[float, float, float, float]
    wheel_angle_deltas: Tuple[float, float, float, float]
    max_wheel_angle_delta_rad: float
    event_duration_s: float
    setup_s: float
    steering_s: float
    steering_profile: str
    event_steer_rate_peak_radps: float
    event_steer_accel_peak_radps2: float

    def as_dict(self):
        return {
            "vehicle_id": self.vehicle_id,
            "from_mode": self.from_mode,
            "to_mode": self.to_mode,
            "from_direction": self.from_direction,
            "to_direction": self.to_direction,
            "from_control": self.from_control,
            "to_control": self.to_control,
            "wheel_angles_start_rad": list(self.wheel_angles_start),
            "wheel_angles_end_rad": list(self.wheel_angles_end),
            "wheel_angle_deltas_rad": list(self.wheel_angle_deltas),
            "max_wheel_angle_delta_rad": self.max_wheel_angle_delta_rad,
            "event_duration_s": self.event_duration_s,
            "setup_s": self.setup_s,
            "steering_s": self.steering_s,
            "steering_profile": self.steering_profile,
            "event_steer_rate_peak_radps": self.event_steer_rate_peak_radps,
            "event_steer_accel_peak_radps2":
                self.event_steer_accel_peak_radps2,
        }


@dataclass(frozen=True)
class SetupTiming:
    phase_id: str
    edge_index: int
    t0_s: float
    duration_s: float
    setup_overhead_s: float
    steering_duration_s: float
    initial: bool
    from_mode: str
    to_mode: str
    qf: FormationPose
    headings: Tuple[Tuple[int, float], ...]
    demands: Tuple[VehicleSetupDemand, ...]
    common_steer_rate_peak_radps: float
    common_steer_accel_peak_radps2: float

    @property
    def kind(self):
        return "initial_setup" if self.initial else "setup"

    @property
    def mode(self):
        # Phases are left-closed/right-open: the old mode remains authoritative.
        return self.from_mode

    @property
    def end_time(self):
        return self.t0_s + self.duration_s

    def demand(self, vehicle_id):
        for demand in self.demands:
            if demand.vehicle_id == vehicle_id:
                return demand
        raise UniformTimingError(
            "INVALID_VEHICLE_ID_SET",
            "setup {} has no vehicle_id={}".format(
                self.phase_id, vehicle_id), phase_id=self.phase_id)

    def as_dict(self):
        return {
            "kind": self.kind,
            "phase_id": self.phase_id,
            "edge_index": self.edge_index,
            "t0_s": self.t0_s,
            "duration_s": self.duration_s,
            "end_time_s": self.end_time,
            "setup_overhead_s": self.setup_overhead_s,
            "steering_duration_s": self.steering_duration_s,
            "initial": self.initial,
            "from_mode": self.from_mode,
            "to_mode": self.to_mode,
            "mode_during_phase": self.mode,
            "qf": list(_pose_values(self.qf)),
            "headings": [
                {"vehicle_id": vehicle_id, "theta": theta}
                for vehicle_id, theta in self.headings],
            "common_steer_rate_peak_radps":
                self.common_steer_rate_peak_radps,
            "common_steer_accel_peak_radps2":
                self.common_steer_accel_peak_radps2,
            "demands": [demand.as_dict() for demand in self.demands],
        }


@dataclass(frozen=True)
class VehicleDerivativeVectors:
    vehicle_id: int
    first: Tuple[float, float]
    second: Tuple[float, float]
    third: Tuple[float, float]
    theta_rate: float
    direction: int
    control: float
    wheel_angles: Tuple[float, ...]
    wheel_factors: Tuple[float, ...]


@dataclass(frozen=True)
class EdgeDerivativeVectors:
    progress: float
    q_first: Tuple[float, float, float]
    q_second: Tuple[float, float, float]
    q_third: Tuple[float, float, float]
    psi: float
    vehicles: Tuple[VehicleDerivativeVectors, ...]


@dataclass(frozen=True)
class MotionTiming:
    phase_id: str
    edge_index: int
    edge: UniformMotionEdge
    block: FormationBlock
    contexts: Tuple[object, ...]
    t0_s: float
    duration_s: float
    required_duration_s: float
    dynamic_duration_s: float
    driver: dict
    limiting: Tuple[dict, ...]
    peaks: Mapping[int, dict]

    @property
    def kind(self):
        return "motion"

    @property
    def mode(self):
        return self.edge.mode

    @property
    def end_time(self):
        return self.t0_s + self.duration_s

    @property
    def report(self):
        return {
            "block_index": self.edge_index,
            "geometry_type": self.edge.control.geometry_type,
            "parameter_length": self.edge.control.parameter_length,
            "duration_s": self.duration_s,
            "dynamic_duration_s": self.dynamic_duration_s,
            "duration_driver": dict(self.driver),
            "limiting_constraints": [dict(item) for item in self.limiting],
            "vehicle_peaks": {
                int(vehicle_id): dict(values)
                for vehicle_id, values in self.peaks.items()},
        }

    def as_dict(self):
        return {
            "kind": self.kind,
            "phase_id": self.phase_id,
            "edge_index": self.edge_index,
            "t0_s": self.t0_s,
            "duration_s": self.duration_s,
            "end_time_s": self.end_time,
            "mode": self.mode,
            "progress_unit": self.edge.progress_unit,
            "required_duration_s": self.required_duration_s,
            "dynamic_duration_s": self.dynamic_duration_s,
            "duration_driver": dict(self.driver),
            "limiting_constraints": [dict(item) for item in self.limiting],
            "vehicle_peaks": [
                {"vehicle_id": int(vehicle_id), "peaks": dict(values)}
                for vehicle_id, values in sorted(self.peaks.items())],
        }


@dataclass(frozen=True)
class UniformTimingSchedule:
    edges: Tuple[UniformMotionEdge, ...]
    phases: Tuple[object, ...]
    limits: FormationTimingLimits
    motion_limits: MotionLimits

    @property
    def duration(self):
        return self.phases[-1].end_time

    @property
    def motions(self):
        return tuple(phase for phase in self.phases
                     if isinstance(phase, MotionTiming))

    @property
    def setups(self):
        return tuple(phase for phase in self.phases
                     if isinstance(phase, SetupTiming))

    @property
    def initial_setup(self):
        for phase in self.phases:
            if isinstance(phase, SetupTiming) and phase.initial:
                return phase
        return None

    def motion_for_edge(self, edge_index):
        for motion in self.motions:
            if motion.edge_index == edge_index:
                return motion
        raise UniformTimingError(
            "INVALID_EDGE_INDEX", "no motion for edge {}".format(edge_index))

    def phase_table(self):
        records = []
        for phase in self.phases:
            record = {
                "phase_id": phase.phase_id,
                "kind": phase.kind,
                "edge_index": phase.edge_index,
                "t0_s": phase.t0_s,
                "end_time_s": phase.end_time,
                "duration_s": phase.duration_s,
                "mode": phase.mode,
            }
            if isinstance(phase, MotionTiming):
                record.update({
                    "primitive": phase.edge.primitive,
                    "progress_unit": phase.edge.progress_unit,
                    "required_duration_s": phase.required_duration_s,
                    "dynamic_duration_s": phase.dynamic_duration_s,
                    "duration_driver": dict(phase.driver),
                })
            else:
                record.update({
                    "initial": phase.initial,
                    "from_mode": phase.from_mode,
                    "to_mode": phase.to_mode,
                    "setup_overhead_s": phase.setup_overhead_s,
                    "steering_duration_s": phase.steering_duration_s,
                    "common_steer_rate_peak_radps":
                        phase.common_steer_rate_peak_radps,
                    "common_steer_accel_peak_radps2":
                        phase.common_steer_accel_peak_radps2,
                })
            records.append(record)
        return records


def _context_view(edge, edge_index, limits, motion_limits):
    block = edge_to_block(edge, edge_index)
    offset_map = {offset.vehicle_id: offset for offset in edge.model.offsets}
    try:
        contexts = tuple(_context_for(
            block.segment_map[vehicle_id], offset_map[vehicle_id], block,
            limits, motion_limits) for vehicle_id in sorted(offset_map))
    except _TimingInfeasible as exc:
        raise UniformTimingError(
            "S8_CONTEXT_REJECTED", exc.detail, edge_index=edge_index,
            vehicle_id=exc.vehicle_id, constraint=exc.constraint) from exc
    return block, contexts


def restore_motion_timing(edge, edge_index, phase_id, t0_s, duration_s,
                          required_duration_s, dynamic_duration_s, driver,
                          limiting, peaks, limits, motion_limits=None):
    """Restore a decoded timing record without recomputing its duration."""
    if motion_limits is None:
        motion_limits = _motion_limits(limits)
    block, contexts = _context_view(edge, edge_index, limits, motion_limits)
    return MotionTiming(
        phase_id=phase_id, edge_index=edge_index, edge=edge, block=block,
        contexts=contexts, t0_s=t0_s, duration_s=duration_s,
        required_duration_s=required_duration_s,
        dynamic_duration_s=dynamic_duration_s, driver=dict(driver),
        limiting=tuple(dict(item) for item in limiting), peaks=peaks)


def motion_derivatives(motion, progress):
    """Exact first three geometric derivatives from the existing S8 kernel."""
    if not isinstance(motion, MotionTiming):
        raise UniformTimingError(
            "INVALID_MOTION_TIMING", "expected MotionTiming")
    value = _finite(progress, "motion progress")
    length = motion.edge.control.parameter_length
    if value < 0.0 or value > length:
        raise UniformTimingError(
            "PROGRESS_OUT_OF_RANGE",
            "progress={:.17g} is outside [0, {:.17g}]".format(value, length),
            edge_index=motion.edge_index, phase_id=motion.phase_id)
    q_first, q_second, q_third, _, psi = _formation_derivatives(
        motion.block.control, value)
    vectors = []
    for context in motion.contexts:
        first, second, third = _vehicle_derivatives(
            motion.block.control, context, value)
        vectors.append(VehicleDerivativeVectors(
            vehicle_id=context.vehicle_id,
            first=(float(first[0]), float(first[1])),
            second=(float(second[0]), float(second[1])),
            third=(float(third[0]), float(third[1])),
            theta_rate=float(context.theta_rate),
            direction=int(context.direction),
            control=float(context.control),
            wheel_angles=tuple(context.wheel_angles),
            wheel_factors=tuple(context.wheel_factors)))
    return EdgeDerivativeVectors(
        progress=value,
        q_first=tuple(float(item) for item in q_first),
        q_second=tuple(float(item) for item in q_second),
        q_third=tuple(float(item) for item in q_third),
        psi=float(psi), vehicles=tuple(vectors))


def _initial_descriptors(edge, motion_limits):
    descriptors = {}
    for segment in edge.segments:
        angles, _ = wheel_configuration(
            Action(ACKERMANN, 0.0, 0.0), motion_limits)
        descriptors[segment.vehicle_id] = {
            "mode": MODE_A, "direction": 1, "control": 0.0,
            "angles": tuple(float(value) for value in angles),
        }
    return descriptors


def _edge_descriptors(edge, at_end):
    configurations = {
        configuration.vehicle_id: configuration
        for configuration in edge.wheel_configurations(at_end=at_end)}
    directions = _pair_map(edge.directions)
    descriptors = {}
    for segment in edge.segments:
        descriptors[segment.vehicle_id] = {
            "mode": edge.mode,
            "direction": directions[segment.vehicle_id],
            "control": float(segment.control),
            "angles": tuple(configurations[segment.vehicle_id].angles),
        }
    return descriptors


def _make_setup(previous, current_edge, previous_edge, edge_index, t0_s,
                limits, motion_limits, initial=False):
    current = _edge_descriptors(current_edge, at_end=False)
    demands = []
    mode_changed = False
    wheel_changed = False
    for vehicle_id in current_edge.model.vehicle_ids:
        before = previous[vehicle_id]
        after = current[vehicle_id]
        signed_deltas = tuple(
            _wrap(end - start)
            for start, end in zip(before["angles"], after["angles"]))
        delta = max(abs(value) for value in signed_deltas)
        event = mode_switch_event(delta, motion_limits)
        if before["mode"] != after["mode"]:
            mode_changed = True
        if delta > WHEEL_CHANGE_TOL_RAD:
            wheel_changed = True
        demands.append(VehicleSetupDemand(
            vehicle_id=vehicle_id,
            from_mode=before["mode"], to_mode=after["mode"],
            from_direction=before["direction"],
            to_direction=after["direction"],
            from_control=before["control"], to_control=after["control"],
            wheel_angles_start=tuple(before["angles"]),
            wheel_angles_end=tuple(after["angles"]),
            wheel_angle_deltas=signed_deltas,
            max_wheel_angle_delta_rad=delta,
            event_duration_s=float(event["duration"]),
            setup_s=float(event["setup_s"]),
            steering_s=float(event["steering_s"]),
            steering_profile=str(event["steering_profile"]),
            event_steer_rate_peak_radps=float(event["steer_rate_peak"]),
            event_steer_accel_peak_radps2=float(event["steer_accel_peak"])))
    if not mode_changed and not wheel_changed:
        return None
    duration = max(demand.event_duration_s for demand in demands)
    overhead = max(demand.setup_s for demand in demands)
    steering_duration = duration - overhead
    if steering_duration < -1e-12:
        raise UniformTimingError(
            "INVALID_SETUP_TIMING",
            "common steering duration is negative", edge_index=edge_index)
    steering_duration = max(0.0, steering_duration)
    rate_peak = 0.0
    accel_peak = 0.0
    for demand in demands:
        for delta in demand.wheel_angle_deltas:
            rate_peak = max(
                rate_peak, 1.875 * abs(delta) / max(steering_duration, 1e-12))
            accel_peak = max(
                accel_peak, (10.0 / math.sqrt(3.0)) * abs(delta)
                / max(steering_duration ** 2, 1e-12))
    if rate_peak > limits.steer_rate_max * (1.0 + SETUP_LIMIT_TOL):
        raise UniformTimingError(
            "SETUP_CURVE_RATE_LIMIT",
            "actual common steering rate {:.12g} exceeds {:.12g}".format(
                rate_peak, limits.steer_rate_max), edge_index=edge_index,
            constraint="steer_rate")
    if accel_peak > limits.steer_accel_max * (1.0 + SETUP_LIMIT_TOL):
        raise UniformTimingError(
            "SETUP_CURVE_ACCEL_LIMIT",
            "actual common steering acceleration {:.12g} exceeds {:.12g}"
            .format(accel_peak, limits.steer_accel_max),
            edge_index=edge_index, constraint="steer_acceleration")
    from_modes = {demand.from_mode for demand in demands}
    to_modes = {demand.to_mode for demand in demands}
    if len(from_modes) != 1 or len(to_modes) != 1:
        raise UniformTimingError(
            "NONUNIFORM_SETUP_MODE",
            "V0.2 requires one global from/to mode", edge_index=edge_index)
    phase_id = ("initial_setup_0000" if initial
                else "setup_{:04d}".format(edge_index))
    return SetupTiming(
        phase_id=phase_id, edge_index=edge_index, t0_s=t0_s,
        duration_s=duration, setup_overhead_s=overhead,
        steering_duration_s=steering_duration, initial=initial,
        from_mode=next(iter(from_modes)), to_mode=next(iter(to_modes)),
        qf=current_edge.start_qf,
        headings=tuple(current_edge.start_headings), demands=tuple(demands),
        common_steer_rate_peak_radps=rate_peak,
        common_steer_accel_peak_radps2=accel_peak)


def schedule_uniform_edges(edges, limits=None, *, initial_edge=None):
    """Return common initial/setup/motion timing for a manual edge sequence."""
    if limits is None:
        limits = FormationTimingLimits()
    sequence = validate_uniform_edge_sequence(edges, limits)
    motion_limits = _motion_limits(limits)
    analyzed = []
    for index, edge in enumerate(sequence):
        block = edge_to_block(edge, index)
        try:
            item = _analyze_block(
                block, edge.model.offsets, limits, motion_limits)
        except _TimingInfeasible as exc:
            reason = ("MOTION_DURATION_LIMIT"
                      if "exceeds max_block_duration_s" in exc.detail
                      else "S8_TIMING_REJECTED")
            raise UniformTimingError(
                reason, exc.detail, edge_index=index,
                vehicle_id=exc.vehicle_id, phase_id="motion_{:04d}".format(index),
                constraint=exc.constraint) from exc
        except FormationValidationError as exc:
            raise UniformTimingError(
                "S8_ANALYSIS_REJECTED", str(exc), edge_index=index) from exc
        analyzed.append(item)

    phases = []
    time_s = 0.0
    previous_descriptors = (_initial_descriptors(sequence[0], motion_limits)
                            if initial_edge is None else _edge_descriptors(initial_edge, at_end=True))
    previous_edge = initial_edge
    for index, (edge, item) in enumerate(zip(sequence, analyzed)):
        setup = _make_setup(
            previous_descriptors, edge, previous_edge, index, time_s,
            limits, motion_limits, initial=index == 0)
        if setup is not None:
            phases.append(setup)
            time_s = setup.end_time
        motion = MotionTiming(
            phase_id="motion_{:04d}".format(index), edge_index=index,
            edge=edge, block=item.block, contexts=item.contexts,
            t0_s=time_s, duration_s=float(item.duration),
            required_duration_s=float(item.required_duration),
            dynamic_duration_s=float(item.report["dynamic_duration_s"]),
            driver=dict(item.driver),
            limiting=tuple(dict(record) for record in item.limiting),
            peaks={int(vehicle_id): dict(values)
                   for vehicle_id, values in item.peaks.items()})
        phases.append(motion)
        time_s = motion.end_time
        previous_descriptors = _edge_descriptors(edge, at_end=True)
        previous_edge = edge
    return UniformTimingSchedule(
        edges=sequence, phases=tuple(phases), limits=limits,
        motion_limits=motion_limits)


def schedule_phase_totals(schedule):
    """Initial setup, other setup, and motion seconds without double counting."""
    initial = sum(phase.duration_s for phase in schedule.setups if phase.initial)
    other = sum(phase.duration_s for phase in schedule.setups if not phase.initial)
    motion = sum(phase.duration_s for phase in schedule.motions)
    return {
        "initial_setup_s": initial,
        "inter_edge_setup_s": other,
        "motion_s": motion,
        "duration_s": initial + other + motion,
        "initial_setup_count": sum(
            1 for phase in schedule.setups if phase.initial),
        "inter_edge_setup_count": sum(
            1 for phase in schedule.setups if not phase.initial),
        "motion_count": len(schedule.motions),
    }
