"""Production-eligible common-time reachability and jerk-smooth retiming."""
from dataclasses import asdict, dataclass, fields, replace
import json
import math
from types import MappingProxyType

from ..path_reachability import ReachabilityProfile, VehicleDerivativeBounds, parameterize, progress_bounds
from .core import Action
from .formation_retiming import REACHABLE_DYNAMIC_PROFILE, _DYNAMIC_LIMITS
from .retiming import wheel_configuration
from .uniform_motion import PRIMITIVE_A_ARC, PRIMITIVE_S_SPIN
from .uniform_progress_clock import JerkClock, reachability_clock
from .uniform_timing import MotionTiming
from .uniform_trajectory import CERTIFICATION, UniformTimedPlan, _validate_plan_structure


REACHABILITY_MODEL_VERSION = "uniform_trajectory.reachable_jerk/v1"
_MARGIN = 0.995


def _geometry_rates(edge):
    """Return bounds per unit of progress from the independently built edge."""
    length = edge.control.parameter_length
    offsets = {item.vehicle_id: item for item in edge.model.offsets}
    begins, ends = dict(edge.start_headings), dict(edge.end_headings)
    arc = edge.primitive == PRIMITIVE_A_ARC
    spin = edge.primitive == PRIMITIVE_S_SPIN
    for segment in edge.segments:
        vehicle_id = segment.vehicle_id
        yaw = abs(ends[vehicle_id] - begins[vehicle_id]) / length
        if arc:
            offset = offsets[vehicle_id]
            radius = math.hypot(offset.x - edge.control.pivot_x, offset.y - edge.control.pivot_y)
            rotation = abs(edge.control.delta_psi) / length
            d1, d2, d3 = radius * rotation, radius * rotation**2, radius * rotation**3
        elif spin:
            d1 = d2 = d3 = 0.0
        else:
            d1 = math.hypot(edge.end_qf.x - edge.start_qf.x,
                            edge.end_qf.y - edge.start_qf.y) / length
            d2 = d3 = 0.0
        angles, factors = wheel_configuration(Action(segment.mode, 0.0, segment.control), edge.limits)
        wheel = max(map(abs, factors)) * (yaw if spin else d1)
        lateral = abs(segment.control) * d1*d1 if edge.mode == "A" else 0.0
        yield vehicle_id, (d1, d2, d3, yaw, wheel, lateral, max(map(abs, angles)))


def _full_interval_bounds(rates, velocity, acceleration, jerk):
    d1, d2, d3, yaw, wheel, lateral, angle = rates
    # For an arc, the tangential and normal vectors are orthogonal. The
    # absolute derivative suprema conservatively cover every jerk lobe.
    return {
        "speed": d1 * velocity,
        "acceleration": math.hypot(d1 * acceleration, d2 * velocity**2),
        "jerk": math.hypot(d1 * jerk + d3 * velocity**3, 3 * d2 * velocity * acceleration),
        "yaw_rate": yaw * velocity,
        "yaw_acceleration": yaw * acceleration,
        "yaw_jerk": yaw * jerk,
        "wheel_speed": wheel * velocity,
        "lateral_acceleration": lateral * velocity**2,
        "steer_rate": 0.0, "steer_acceleration": 0.0, "wheel_angle": angle,
    }


def certify_clock(phase, clock, limits):
    """Check all lobe suprema against each vehicle's original dynamic contract."""
    if not limits.min_mode_dwell_s <= clock.duration_s <= limits.max_block_duration_s:
        raise ValueError("common clock violates residence or phase-duration limit")
    length = phase.edge.control.parameter_length
    # Derive the suprema independently of the clock sampler.
    r, h, c = clock.ramp_s, clock.hold_s, clock.cruise_s
    speed = length / (r + r + h + c)
    acceleration = speed / (r + h)
    jerk = speed / (r * (r + h))
    bounds = {}
    for vehicle_id, rates in _geometry_rates(phase.edge):
        report = _full_interval_bounds(rates, speed, acceleration, jerk)
        for name, limit_name, _ in _DYNAMIC_LIMITS:
            if not math.isfinite(report[name]) or report[name] > getattr(limits, limit_name):
                raise ValueError("clock exceeds {} for vehicle {}".format(name, vehicle_id))
        if report["wheel_angle"] > limits.steer_angle_max + 1e-9:
            raise ValueError("clock phase wheel angle exceeds limit")
        bounds[vehicle_id] = report
    return bounds


def _choose_clock(phase, limits):
    edge = phase.edge
    geometry = tuple(rates for _, rates in _geometry_rates(edge))
    velocity = math.inf
    for d1, _, _, yaw, wheel, lateral, _ in geometry:
        if d1:
            velocity = min(velocity, limits.v_max / d1)
        if yaw:
            velocity = min(velocity, limits.yaw_rate_max / yaw)
        if wheel:
            velocity = min(velocity, limits.wheel_speed_max / wheel)
        if lateral:
            velocity = min(velocity, math.sqrt(limits.lateral_accel_max / lateral))

    arc = edge.primitive == PRIMITIVE_A_ARC
    best = None
    for speed_ratio in ((1.0, .90, .80, .65, .5) if arc else (1.0,)):
        speed = velocity * speed_ratio * _MARGIN
        accel_limit = math.inf
        for d1, d2, _, yaw, _, _, _ in geometry:
            if d1:
                normal = d2 * speed**2
                accel_limit = min(accel_limit, math.sqrt(max(0.0, limits.a_max**2 - normal**2)) / d1)
            if yaw:
                accel_limit = min(accel_limit, limits.yaw_accel_max / yaw)
        if not accel_limit > 0:
            continue
        for accel_ratio in ((1.0, .8, .6) if arc else (1.0,)):
            acceleration = accel_limit * accel_ratio * _MARGIN
            jerk_limit = math.inf
            for d1, d2, d3, yaw, _, _, _ in geometry:
                if d1:
                    normal = 3 * d2 * speed * acceleration
                    tangential = math.sqrt(max(0.0, limits.jerk_max**2 - normal**2)) - d3 * speed**3
                    jerk_limit = min(jerk_limit, tangential / d1)
                if yaw:
                    jerk_limit = min(jerk_limit, limits.yaw_jerk_max / yaw)
            if jerk_limit <= 0:
                continue
            try:
                candidate = reachability_clock(
                    edge.control.parameter_length, speed, acceleration, jerk_limit * _MARGIN,
                    min_duration_s=math.nextafter(limits.min_mode_dwell_s, math.inf))
                certify_clock(phase, candidate, limits)
            except (ValueError, ArithmeticError):
                continue
            if candidate.duration_s < (phase.duration_s if best is None else best.duration_s):
                best = candidate
    return best


@dataclass(frozen=True, kw_only=True)
class UniformReachabilityTimedPlan(UniformTimedPlan):
    """Versioned jerk clocks over a separately audited original geometry."""

    model_version = REACHABILITY_MODEL_VERSION
    dynamic_profile = REACHABLE_DYNAMIC_PROFILE
    baseline: UniformTimedPlan
    clocks: tuple

    @classmethod
    def from_clocks(cls, baseline, clocks):
        if type(baseline) is not UniformTimedPlan:
            raise ValueError("clock model requires an original uniform plan")
        clocks = tuple(clocks)
        by_phase = dict(clocks)
        phase_order = tuple(phase.phase_id for phase in baseline.phases
                            if isinstance(phase, MotionTiming) and phase.phase_id in by_phase)
        if not clocks or tuple(key for key, _ in clocks) != phase_order:
            raise ValueError("invalid motion clock phase identifiers")
        phases, cursor = [], 0.0
        for phase in baseline.phases:
            clock = by_phase.get(phase.phase_id)
            if clock is not None:
                bounds = certify_clock(phase, clock, baseline.limits)
                if clock.duration_s >= phase.duration_s:
                    raise ValueError("selected clock must improve its original phase")
                phase = replace(
                    phase, duration_s=clock.duration_s, required_duration_s=clock.duration_s,
                    dynamic_duration_s=clock.duration_s, driver={"constraint": "shared_reachability"},
                    limiting=(), peaks=bounds)
            shifted = replace(phase, t0_s=cursor)
            phases.append(shifted)
            cursor = shifted.end_time
        options = {item.name: getattr(baseline, item.name) for item in fields(UniformTimedPlan)}
        options.update(phases=tuple(phases), duration_s=cursor)
        plan = cls(**options, baseline=baseline, clocks=clocks)
        _validate_plan_structure(plan)
        object.__setattr__(plan, "_clocks_by_phase", MappingProxyType(by_phase))
        return plan

    def _motion_state(self, time_s, phase, phase_index):
        clock = self._clocks_by_phase.get(phase.phase_id)
        if clock is None:
            return super()._motion_state(time_s, phase, phase_index)
        local = (clock.duration_s if time_s == phase.end_time else
                 min(clock.duration_s, max(0.0, time_s - phase.t0_s)))
        values = clock.sample(local, phase.edge.control.parameter_length)
        return self._motion_state_for_progress(time_s, phase, phase_index, *values)

    def to_model_dict(self):
        return {
            "model_version": self.model_version, "certification": CERTIFICATION,
            "baseline": self.baseline.to_model_dict(),
            "clocks": [{"phase_id": phase_id, **asdict(clock)} for phase_id, clock in self.clocks],
        }

    @classmethod
    def from_model_dict(cls, values):
        fields_expected = {"model_version", "certification", "baseline", "clocks"}
        if (not isinstance(values, dict) or values.keys() != fields_expected
                or values["model_version"] != REACHABILITY_MODEL_VERSION
                or values["certification"] != CERTIFICATION or not isinstance(values["clocks"], list)):
            raise ValueError("invalid reachability model fields")
        baseline = UniformTimedPlan.from_model_dict(values["baseline"])
        clocks = []
        for entry in values["clocks"]:
            if (not isinstance(entry, dict) or entry.keys() != {"phase_id", "ramp_s", "hold_s", "cruise_s"}
                    or not isinstance(entry["phase_id"], str)):
                raise ValueError("invalid clock record")
            clocks.append((entry["phase_id"], JerkClock(entry["ramp_s"], entry["hold_s"], entry["cruise_s"])))
        return cls.from_clocks(baseline, clocks)


def retime_certified_uniform(baseline, evaluation):
    """Retain independent geometry/rigidity/sweep proof and certify the new clock."""
    baseline_digest = baseline.model_sha256()
    if (type(baseline) is not UniformTimedPlan or not evaluation.feasible
            or not evaluation.independent_audit or not evaluation.collision_certificate
            or evaluation.metrics.get("model_sha256") != baseline_digest
            or evaluation.metrics.get("collision_certificate") is not True
            or evaluation.metrics.get("map_version_checked") is not True
            or evaluation.metrics.get("dynamic_continuous_certificate") is not True):
        return None
    clocks = []
    for phase in baseline.phases:
        if isinstance(phase, MotionTiming):
            clock = _choose_clock(phase, baseline.limits)
            if clock is not None:
                clocks.append((phase.phase_id, clock))
    if not clocks:
        return None
    plan = UniformReachabilityTimedPlan.from_clocks(baseline, clocks)
    # All edge progress maps are monotone and traverse exactly the same closed
    # intervals; setup geometry, duration and order are unchanged. The static
    # swept-OBB result therefore carries over, independently of timing.
    ratios, checks = [], 0
    by_phase = dict(clocks)
    for phase in plan.phases:
        if phase.phase_id in by_phase:
            for bounds in certify_clock(phase, by_phase[phase.phase_id], plan.limits).values():
                ratios.extend(bounds[name] / getattr(plan.limits, limit_name)
                              for name, limit_name, _ in _DYNAMIC_LIMITS)
                checks += len(_DYNAMIC_LIMITS)
    metrics = {"certification": evaluation.certification, "independent_audit": True,
               "collision_certificate": True, "map_version_checked": True,
               "model_sha256": plan.model_sha256(), "model_version": plan.model_version,
               "duration_s": plan.duration_s, "baseline_duration_s": baseline.duration_s,
               "baseline_geometry_sha256": baseline_digest,
               "dynamic_continuous_certificate": True,
               "replacement_clock_max_dynamic_ratio": max(ratios),
               "retimed_phases": [phase_id for phase_id, _ in clocks]}
    for field in ("cargo_footprint", "collision_margin_m", "map_signature"):
        if field in evaluation.metrics:
            metrics[field] = evaluation.metrics[field]
    audited = replace(evaluation, detail="unchanged swept geometry and independently bounded common jerk clock",
                      metrics=metrics, checks=evaluation.checks + checks)
    return plan, plan.sample(), audited


@dataclass(frozen=True)
class ReachabilityPhase:
    phase_id: str
    kind: str
    mode: str
    t0_s: float
    baseline_duration_s: float
    duration_s: float
    profile: ReachabilityProfile | None = None


def ackermann_progress_bounds(edge):
    """Intersect body and wheel bounds in metres or radians of edge progress."""
    if edge.mode != "A":
        raise ValueError("reachability progress limits require an A primitive")
    limits = edge.limits
    wheel_factors = dict(edge.wheel_factors_start)
    arc = edge.primitive == PRIMITIVE_A_ARC
    vehicles = []
    for vehicle_id, signed_rate in edge.signed_progress_speeds:
        rate = abs(signed_rate)
        factor = max(abs(value) for value in wheel_factors[vehicle_id])
        vehicles.append(VehicleDerivativeBounds(
            length_rate=rate, length_second_rate=rate if arc else 0.0,
            yaw_rate=1.0 if arc else 0.0, yaw_second_rate=0.0,
            max_speed=min(limits.target_v, limits.wheel_speed_max / factor),
            max_acceleration=limits.a_max, max_yaw_rate=limits.translation_omega_max,
            max_yaw_acceleration=limits.translation_alpha_max))
    return progress_bounds(vehicles)


def compare_uniform_timing(plan, *, intervals=32):
    """Build a non-executable comparison on the original mixed mode sequence."""
    if intervals < 2:
        raise ValueError("at least two reachability intervals are required")
    elapsed, timings = 0.0, []
    for phase in plan.phases:
        duration, profile = phase.duration_s, None
        if isinstance(phase, MotionTiming) and phase.mode == "A":
            size = phase.edge.control.parameter_length
            knots = tuple(size * index / intervals for index in range(intervals + 1))
            bound = ackermann_progress_bounds(phase.edge)
            profile = parameterize(knots, (bound,) * intervals)
            duration = profile.duration_s
        if not math.isfinite(duration) or duration < 0.0:
            raise ValueError("invalid research phase duration")
        timings.append(ReachabilityPhase(phase.phase_id, phase.kind, phase.mode, elapsed,
                                         phase.duration_s, duration, profile))
        elapsed += duration
    return tuple(timings)
