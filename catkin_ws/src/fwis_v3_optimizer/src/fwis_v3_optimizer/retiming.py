"""Acceleration-bounded exact-primitive timing with stationary reconfiguration.

The limits are an explicit kinematic simulation profile, not calibrated
hardware limits. Every change of constant wheel configuration stops first.
The phase record is the interpolation contract; ROS samples are its display.
"""
from dataclasses import asdict, dataclass
import math

import numpy as np

from .core import Action, Pose, advance, collapse_exact_runs, wrap


@dataclass(frozen=True)
class MotionLimits:
    profile: str = "kinematic_sim_v1"
    v_max: float = .8
    a_max: float = .6
    jerk_max: float = 2.0
    lateral_a_max: float = .6
    omega_max: float = .8
    alpha_max: float = .8
    angular_jerk_max: float = 3.0
    steer_rate_max: float = .8
    steer_accel_max: float = 1.5
    steer_angle_max: float = math.pi / 2
    wheel_speed_max: float = 1.0
    mode_setup_time: float = .15
    wheelbase: float = .494
    track: float = .364
    r_min_ack: float = .810
    target_v: object = None
    # -1 preserves legacy constructor behavior; explicit None removes a
    # separate body-yaw bound during translation. Spin retains omega/alpha.
    translation_omega_max: object = -1.
    translation_alpha_max: object = -1.
    translation_angular_jerk_max: object = -1.
    # The interactive kinematic demo cruises independently of curvature.
    # Restricted timing remains available for capability-limit comparisons.
    translation_speed_policy: str = "restricted"
    mode_switch_duration: object = None

    def __post_init__(self):
        if self.translation_speed_policy not in ("restricted", "mode_switch_only"):
            raise ValueError("invalid_translation_speed_policy")
        if self.translation_speed_policy == "mode_switch_only" and self.mode_switch_duration is None:
            raise ValueError("mode_switch_only_requires_switch_duration")
        if self.target_v is None:
            object.__setattr__(self, "target_v", self.v_max)
        for field, old in (("translation_omega_max", "omega_max"),
                           ("translation_alpha_max", "alpha_max"),
                           ("translation_angular_jerk_max", "angular_jerk_max")):
            if getattr(self, field) == -1.:
                object.__setattr__(self, field, getattr(self, old))
            elif getattr(self, field) == 0.:
                object.__setattr__(self, field, None)
        for name, value in asdict(self).items():
            if name in ("profile", "translation_speed_policy"):
                continue
            if name == "mode_switch_duration" and value is None:
                continue
            if name.startswith("translation_") and value is None:
                continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError("invalid_motion_limit:" + name)
        if self.target_v > self.v_max:
            raise ValueError("target_speed_exceeds_maximum")


@dataclass
class TimedPlan:
    samples: list
    phases: list
    duration: float
    moving_seconds: float
    spin_seconds: float
    reconfiguration_seconds: float
    reconfiguration_count: int
    limits: MotionLimits

    def report(self):
        return {"profile": asdict(self.limits),
                "interpolation": "exact_primitive_quintic_v1",
                "display_interpolation": "linear_pose_samples",
                "duration_s": self.duration,
                "translation_s": self.moving_seconds,
                "spin_s": self.spin_seconds,
                "reconfiguration_s": self.reconfiguration_seconds,
                "other_holds_s": max(0., self.duration-self.moving_seconds-self.spin_seconds-self.reconfiguration_seconds),
                "reconfiguration_count": self.reconfiguration_count,
                "phases": self.phases}


def wheel_configuration(action, limits):
    """Canonical four wheel angles and wheel-speed / center-rate factors."""
    angles, factors = [], []
    for x in (limits.wheelbase / 2, -limits.wheelbase / 2):
        for y in (limits.track / 2, -limits.track / 2):
            if action.mode == 0:
                vx, vy = 1 - action.control * y, action.control * x
            elif action.mode == 1:
                vx, vy = math.cos(action.control), math.sin(action.control)
            elif action.mode == 2:
                vx, vy = -y, x
            else:
                raise ValueError("unknown_mode")
            angle = math.atan2(vy, vx)
            # In Spin opposite wheels may roll with opposite signs. The body
            # does not translate; the forward-body constraint is unaffected.
            if action.mode == 2:
                if angle > math.pi / 2:
                    angle -= math.pi
                if angle < -math.pi / 2:
                    angle += math.pi
            angles.append(angle)
            factors.append(math.hypot(vx, vy))
    return np.asarray(angles), np.asarray(factors)


def quintic(tau):
    """Unit displacement and first two derivatives wrt normalized time."""
    return (tau ** 3 * (10 - 15 * tau + 6 * tau ** 2),
            30 * tau ** 2 * (1 - tau) ** 2,
            60 * tau * (1 - tau) * (1 - 2 * tau))


def bounded_duration(amount, rate, acceleration, jerk=None):
    # Exact maxima of |s'|, |s''| and |s'''| for the unit quintic.
    values = [1.875 * amount / rate,
              math.sqrt((10 / math.sqrt(3)) * amount / acceleration)]
    if jerk is not None:
        values.append((60 * amount / jerk) ** (1 / 3))
    return max(values)


def steering_rest_to_rest_duration(amount, rate, acceleration):
    """Minimum triangular/trapezoidal steering time with zero endpoint rate.

    The simulation has steering rate/acceleration limits and no steering jerk
    limit. Body-motion jerk constraints are separate and remain unchanged.
    """
    amount=abs(float(amount))
    if amount<=1e-12:
        return 0.
    if amount<=rate*rate/acceleration:
        return 2*math.sqrt(amount/acceleration)
    return amount/rate+rate/acceleration


class PrimitiveTimingCache:
    """Exact duration-only legacy proxy, reusing primitive and steering costs.

    Candidate ranking shares most primitives across suffix replacements. The
    executable continuous retimer and its independent audit remain separate.
    """
    def __init__(self, limits, allow_reverse=False, dt=.5):
        self.limits, self.allow_reverse, self.dt = limits, allow_reverse, dt
        self.actions, self.transitions = {}, {}
        self.stats = {"primitive_hits": 0, "primitive_misses": 0,
                      "steering_hits": 0, "steering_misses": 0}

    def __call__(self, actions):
        elapsed, old_mode, old_angles = 0., 0, (0.,)*4
        for action in collapse_exact_runs(list(actions)):
            if action not in self.actions:
                # Reuse all validation and timing formulae from the reference
                # implementation instead of introducing a second motion model.
                single = retime(Pose(0.,0.,0.), [action], self.limits, self.dt,
                                allow_reverse=self.allow_reverse, build_samples=False)
                motion = next((p for p in single.phases
                               if p["kind"] in ("translation", "spin")), None)
                self.actions[action] = ((motion["duration"], tuple(motion["wheel_angles"]))
                                        if motion is not None else None)
                self.stats["primitive_misses"] += 1
            else:
                self.stats["primitive_hits"] += 1
            cached = self.actions[action]
            if cached is None:
                continue
            duration, angles = cached
            key = (old_mode, action.mode, old_angles, angles)
            if key not in self.transitions:
                delta = max(abs(a-b) for a,b in zip(old_angles,angles))
                steering = (bounded_duration(delta,self.limits.steer_rate_max,
                            self.limits.steer_accel_max) if delta > 1e-8 else 0.)
                setup = self.limits.mode_setup_time if old_mode != action.mode else 0.
                self.transitions[key] = steering+setup
                self.stats["steering_misses"] += 1
            else:
                self.stats["steering_hits"] += 1
            elapsed += self.transitions[key]+duration
            old_mode, old_angles = action.mode, angles
        return elapsed if elapsed else self.dt


def retime(start, actions, limits=None, dt=.05, allow_reverse=False,
           build_samples=True):
    limits = limits or MotionLimits()
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("invalid_sampling_interval")
    actions = collapse_exact_runs(list(actions))
    for a in actions:
        if not all(math.isfinite(z) for z in (a.amount, a.control)):
            raise ValueError("nonfinite_action")
        if a.mode not in (0, 1, 2):
            raise ValueError("unknown_mode")
        if not allow_reverse and a.mode != 2 and a.amount < -1e-10:
            raise ValueError("reverse_motion_forbidden")
        if a.mode == 0 and abs(a.control) > 1 / limits.r_min_ack + 1e-9:
            raise ValueError("ackermann_curvature_limit")
        if a.mode == 1 and abs(a.control) > math.pi / 2 + 1e-9:
            raise ValueError("oblique_forward_angle_limit")

    phases, samples = [], [(0., start, 0., actions[0].mode if actions else 0)]
    p, elapsed, old_mode = start, 0., 0
    wheel_angles = np.zeros(4)
    moving_seconds = spin_seconds = config_seconds = 0.
    config_count = 0
    for index, a in enumerate(actions):
        if abs(a.amount) < 1e-10:
            continue
        angles, factors = wheel_configuration(a, limits)
        if np.max(np.abs(angles)) > limits.steer_angle_max + 1e-9:
            raise ValueError("wheel_angle_limit")
        delta = float(np.max(np.abs(angles - wheel_angles)))
        steer_time = (bounded_duration(delta, limits.steer_rate_max,
                                       limits.steer_accel_max)
                      if delta > 1e-8 else 0.)
        setup_time = limits.mode_setup_time if a.mode != old_mode else 0.
        hold_time = steer_time + setup_time  # declared sequential setup
        if hold_time > 1e-9:
            phases.append({"kind": "reconfiguration", "action_index": index,
                           "mode": a.mode, "t0": elapsed, "duration": hold_time,
                           "setup_s": setup_time, "steering_s": steer_time,
                           "wheel_angles_start": wheel_angles.tolist(),
                           "wheel_angles_end": angles.tolist(),
                           "pose": [p.x, p.y, p.th],
                           "steer_rate_peak": 1.875 * delta / max(steer_time, 1e-12),
                           "steer_accel_peak": (10 / math.sqrt(3)) * delta /
                           max(steer_time ** 2, 1e-12)})
            if build_samples:
                n = max(1, math.ceil(hold_time / dt))
                samples.extend((elapsed + hold_time * i / n, p, 0., a.mode)
                               for i in range(1, n + 1))
            elapsed += hold_time
            config_seconds += hold_time
            config_count += 1
        wheel_angles, old_mode = angles, a.mode
        amount = abs(a.amount)
        if a.mode == 2:
            rate = min(limits.omega_max, limits.wheel_speed_max / max(factors))
            acceleration, jerk = limits.alpha_max, limits.angular_jerk_max
        else:
            rate = min(limits.target_v, limits.wheel_speed_max / max(factors))
            if a.mode == 0 and abs(a.control) > 1e-9:
                rate = min(rate, math.sqrt(limits.lateral_a_max / abs(a.control)))
            acceleration, jerk = limits.a_max, limits.jerk_max
            if a.mode == 0 and abs(a.control) > 1e-9:
                rate = min(rate, limits.omega_max/abs(a.control))
                acceleration = min(acceleration, limits.alpha_max/abs(a.control))
                jerk = min(jerk, limits.angular_jerk_max/abs(a.control))
        duration = max(1e-6, bounded_duration(amount, rate, acceleration, jerk))
        phases.append({"kind": "spin" if a.mode == 2 else "translation",
                       "action_index": index, "mode": a.mode,
                       "amount": a.amount, "control": a.control,
                       "start": [p.x, p.y, p.th], "t0": elapsed,
                       "duration": duration, "rate_peak": 1.875 * amount / duration,
                       "acceleration_peak": (10 / math.sqrt(3)) * amount / duration ** 2,
                       "jerk_peak": 60 * amount / duration ** 3,
                       "wheel_angles": angles.tolist(), "wheel_rate_factors": factors.tolist()})
        n = max(2, math.ceil(duration / dt)) if build_samples else 1
        for i in range(1, n + 1):
            tau = i / n
            s, speed, _ = quintic(tau)
            q = advance(p, Action(a.mode, a.amount * s, a.control))
            # Preserve unwrapped yaw for rotation and interpolation clients.
            th = p.th + (a.amount * s if a.mode == 2 else
                         a.amount * a.control * s if a.mode == 0 else 0.)
            q = Pose(q.x, q.y, th)
            v = 0. if a.mode == 2 else a.amount * speed / duration
            samples.append((elapsed + duration * tau, q, v, a.mode))
        elapsed += duration
        if a.mode == 2:
            spin_seconds += duration
        else:
            moving_seconds += duration
        p = samples[-1][1]
    if len(samples) == 1:
        samples.append((dt, start, 0., 0))
        elapsed = dt
    return TimedPlan(samples, phases, elapsed, moving_seconds, spin_seconds,
                     config_seconds, config_count, limits)
