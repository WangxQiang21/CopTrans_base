"""Versioned common timeline for audited A/O/S pieces with one C2 A span."""
from bisect import bisect_left, bisect_right
from dataclasses import dataclass, replace
import hashlib
import json
import math

from .formation_continuous_a.contracts import request_from_payload
from .formation_continuous_a.trajectory import VirtualATimedPlan
from .uniform_timing import SetupTiming
from .uniform_trajectory import (
    CERTIFICATION,
    UniformTimedPlan,
    UniformTimeState,
    UniformTrajectoryError,
    UniformVehicleTimeState,
)

MIXED_MODEL_VERSION = "uniform_trajectory.c2_span/v1"
BOUNDARY_TOL = 1e-9


@dataclass(frozen=True)
class CurvePhase:
    t0_s: float
    duration_s: float
    edge_index: int
    kind: str = "motion"
    mode: str = "A"
    phase_id: str = "continuous_A"

    @property
    def end_time(self):
        return self.t0_s + self.duration_s

    def as_dict(self):
        return {
            "phase_id": self.phase_id, "kind": self.kind, "edge_index": self.edge_index,
            "t0_s": self.t0_s, "end_time_s": self.end_time, "duration_s": self.duration_s,
            "mode": self.mode, "primitive": "C2_CURVATURE",
        }


def _angle_error(a, b):
    return abs(math.remainder(a - b, math.tau))


def _validate_join(first, second):
    """Check the stopped splice, including real orientation and wheel state."""
    if (first.mode != second.mode or first.mode != "A"
            or math.hypot(first.qf.x - second.qf.x, first.qf.y - second.qf.y) > BOUNDARY_TOL
            or _angle_error(first.qf.psi, second.qf.psi) > BOUNDARY_TOL):
        raise UniformTrajectoryError("MIXED_POSE_DISCONTINUITY", "C2 A join must be a shared A pose")
    for left, right in zip(first.vehicles, second.vehicles):
        if (left.vehicle_id != right.vehicle_id
                or _angle_error(left.theta, right.theta) > BOUNDARY_TOL
                or any(abs(value) > BOUNDARY_TOL for vehicle in (left, right)
                       for value in (vehicle.mode_control, vehicle.center_speed_mps,
                                     vehicle.yaw_rate_radps, vehicle.yaw_acceleration_radps2,
                                     *vehicle.acceleration, *vehicle.wheel_angles,
                                     *vehicle.wheel_rolling_speeds, *vehicle.wheel_angular_rates,
                                     *vehicle.wheel_angular_accelerations))):
            raise UniformTrajectoryError(
                "MIXED_WHEEL_DISCONTINUITY", "C2 A join requires a stopped straight wheel state",
                vehicle_id=left.vehicle_id)


class UniformMixedTimedPlan:
    """Three joined pieces on one clock, with standalone analytic C2 reconstruction."""

    def __init__(self, prefix, curve, suffix):
        middle = request_from_payload(curve.to_model_dict()["request_inputs"])
        if (not suffix.edges or not middle.initial.wheels_straight
                or middle.initial.source != "verified_terminal"
                or middle.initial.context_id != prefix.model_sha256()
                or curve.kind != "motion" or suffix.initial_edge is not None
                or prefix.plan_id != middle.plan_id or suffix.plan_id != middle.plan_id
                or prefix.formation_id != middle.formation_id or suffix.formation_id != middle.formation_id
                or prefix.model.offsets != middle.model.offsets or suffix.model.offsets != middle.model.offsets
                or dict(prefix.vehicle_shapes) != middle.vehicle_shapes
                or dict(suffix.vehicle_shapes) != middle.vehicle_shapes
                or prefix.limits != middle.limits or suffix.limits != middle.limits):
            raise UniformTrajectoryError("MIXED_PIECE_MISMATCH", "incompatible C2 span pieces")

        self.prefix, self.curve, self.suffix = prefix, curve, suffix
        self.plan_id = prefix.plan_id
        self.formation_id = prefix.formation_id
        self.model = prefix.model
        self.vehicle_ids = prefix.vehicle_ids
        self.vehicle_shapes = prefix.vehicle_shapes
        self.limits = prefix.limits
        self.motion_limits = prefix.motion_limits
        self.start_qf, self.start_headings = prefix.start_qf, prefix.start_headings
        self.goal_qf, self.goal_headings = suffix.goal_qf, suffix.goal_headings
        self.edges = prefix.edges + suffix.edges
        self.initial_edge = prefix.initial_edge
        self.curve_phase = CurvePhase(prefix.duration_s, curve.duration_s, len(prefix.edges))
        suffix_time = self.curve_phase.end_time
        # Suffix initial wheel setup is internal to the common timeline.
        self.phases = prefix.phases + (self.curve_phase,) + tuple(
            replace(phase, t0_s=phase.t0_s + suffix_time,
                    phase_id="suffix_" + phase.phase_id,
                    edge_index=phase.edge_index + len(prefix.edges) + 1,
                    **({"initial": False} if isinstance(phase, SetupTiming) else {}))
            for phase in suffix.phases)
        self._ends = (prefix.duration_s, suffix_time, suffix_time + suffix.duration_s)
        self.duration_s = self._ends[-1]
        _validate_join(prefix.state_at(prefix.duration_s), self._curve_state(0.0))
        _validate_join(self._curve_state(curve.duration_s), suffix.state_at(0.0))

    def _curve_state(self, elapsed, side="auto"):
        state = self.curve.state_at(elapsed, side)
        phase = self.curve_phase
        vehicles = tuple(
            UniformVehicleTimeState(
                vehicle_id=v.vehicle_id, x=v.x, y=v.y, theta=v.theta,
                velocity=v.velocity, acceleration=v.acceleration, jerk=v.jerk,
                yaw_rate_radps=v.omega_i, yaw_acceleration_radps2=v.yaw_accel,
                yaw_jerk_radps3=v.yaw_jerk, center_speed_mps=abs(v.v_i),
                longitudinal_speed_mps=v.v_i, lateral_speed_mps=0.0,
                center_acceleration_mps2=math.hypot(*v.acceleration),
                center_jerk_mps3=math.hypot(*v.jerk),
                wheel_speed_mps=max(map(abs, v.wheel_speeds)),
                lateral_acceleration_mps2=abs(v.lateral_accel),
                direction=1, mode_control=v.C_i, wheel_angles=v.wheel_angles,
                wheel_angular_rates=v.steer_rates, wheel_angular_accelerations=v.steer_accels,
                wheel_rolling_speeds=v.wheel_speeds)
            for v in state.vehicles)
        psi = state.qf.psi
        t, n = (math.cos(psi), math.sin(psi)), (-math.sin(psi), math.cos(psi))
        jet = self.curve.path.profile.jet_at(state.s, side)
        k, eta, kss = jet.kappa, jet.eta, jet.u
        w, a, j = state.w, state.a, state.j
        return UniformTimeState(
            phase.t0_s + state.t, phase.phase_id, "motion", len(self.prefix.phases),
            phase.edge_index, "A", state.qf, vehicles, state.s, "m", w, a, j,
            (w * t[0], w * t[1], k * w),
            (a * t[0] + k * w*w * n[0], a * t[1] + k * w*w * n[1], eta*w*w + k*a),
            ((j-k*k*w**3)*t[0] + (eta*w**3+3*k*w*a)*n[0],
             (j-k*k*w**3)*t[1] + (eta*w**3+3*k*w*a)*n[1], kss*w**3+3*eta*w*a+k*j),
            from_mode="A", to_mode="A")

    def state_at(self, time_s, side="auto"):
        if not math.isfinite(time_s) or side not in ("auto", "left", "right"):
            raise UniformTrajectoryError("INVALID_MIXED_TIME", "invalid time or side")
        elapsed = min(self.duration_s, max(0.0, time_s))
        index = min(2, (bisect_left if side == "left" else bisect_right)(self._ends, elapsed))
        if index == 0:
            return self.prefix.state_at(elapsed, side)
        if index == 1:
            return self._curve_state(elapsed - self._ends[0], side)
        local = min(self.suffix.duration_s, elapsed - self._ends[1])
        state = self.suffix.state_at(local, side)
        phase_index = state.phase_index + len(self.prefix.phases) + 1
        return replace(state, time_s=elapsed, phase_id="suffix_" + state.phase_id,
                       phase_index=phase_index, phase_kind=self.phases[phase_index].kind,
                       edge_index=state.edge_index + len(self.prefix.edges) + 1)

    default_sample_times = UniformTimedPlan.default_sample_times
    _mode_intervals = UniformTimedPlan._mode_intervals
    def phase_table(self):
        return [phase.as_dict() for phase in self.phases]

    def phase_totals(self):
        initial = sum(phase.duration_s for phase in self.phases if phase.kind == "initial_setup")
        setup = sum(phase.duration_s for phase in self.phases if phase.kind == "setup")
        return {"initial_setup_s": initial, "inter_edge_setup_s": setup,
                "motion_s": self.duration_s - initial - setup}

    def sample(self, times=None):
        projection = UniformTimedPlan.sample(self, times)
        return replace(projection, model_version=MIXED_MODEL_VERSION)

    def _aggregate_peaks(self):
        peaks = UniformTimedPlan._aggregate_peaks(self)
        selection = self.curve.to_model_dict()["timing"]["selection"]
        # The retimer records whole-formation bounds. Use them as conservative
        # upper bounds for each car in the projection report.
        bounds = {
            name: item["upper_at_T0_s"] / self.curve.duration_s ** item["order"]
            for name, item in selection["metrics_at_T0_s"].items()
        }
        bounds["wheel_angle"] = selection["wheel_angle_bound_rad"]
        for values in peaks.values():
            for name, bound in bounds.items():
                values[name] = max(values[name], bound)
        return peaks

    def to_model_dict(self):
        return {
            "model_version": MIXED_MODEL_VERSION, "certification": CERTIFICATION,
            "prefix": self.prefix.to_model_dict(), "curve": self.curve.to_model_dict(),
            "suffix": self.suffix.to_model_dict(),
        }

    def to_model_json(self):
        return json.dumps(self.to_model_dict(), sort_keys=True, separators=(",", ":"), allow_nan=False)

    def model_sha256(self):
        digest = hashlib.sha256()
        for word in (MIXED_MODEL_VERSION, self.prefix.model_sha256(),
                     self.curve.model_sha256(), self.suffix.model_sha256()):
            digest.update(word.encode("ascii"))
        return digest.hexdigest()

    @classmethod
    def from_model_json(cls, payload):
        data = json.loads(payload)
        if (not isinstance(data, dict)
                or set(data) != {"model_version", "certification", "prefix", "curve", "suffix"}
                or data["model_version"] != MIXED_MODEL_VERSION
                or data["certification"] != CERTIFICATION):
            raise UniformTrajectoryError("INVALID_MIXED_MODEL", "invalid mixed model schema")
        return cls(
            UniformTimedPlan.from_model_dict(data["prefix"]),
            VirtualATimedPlan.from_model_json(json.dumps(data["curve"], allow_nan=False)),
            UniformTimedPlan.from_model_dict(data["suffix"]))
