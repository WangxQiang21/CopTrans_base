"""Project a planned mode and constrain the final published Twist."""
from dataclasses import dataclass
import math

from .model import Mode, Twist, ZERO


@dataclass(frozen=True)
class Limits:
    max_speed: float = .28
    max_yaw_rate: float = .55
    max_accel: float = .45
    max_yaw_accel: float = .70
    mechanical_radius: float = .810
    switch_radius: float = .47644
    radius_margin: float = .02
    parallel_min_vy: float = .00001

    def __post_init__(self):
        if not all(math.isfinite(value) and value > 0 for value in vars(self).values()):
            raise ValueError("velocity limits, radii and margins must be finite and positive")
        if self.parallel_min_vy >= self.max_speed / 2:
            raise ValueError("parallel_min_vy must be well below max_speed")

    @property
    def guard_radius(self):
        return max(self.mechanical_radius, self.switch_radius) + self.radius_margin


@dataclass(frozen=True)
class Filtered:
    command: Twist
    reasons: tuple = ()


def clamp(value, limit):
    return max(-limit, min(limit, value))


class SafetyFilter:
    def __init__(self, limits):
        self.limits = limits
        self.reset()

    def reset(self):
        self.previous = ZERO
        self.mode = None

    def apply(self, raw, mode, dt, direction=0.0, oblique_direction=0.0):
        if not all(math.isfinite(value) for value in (*raw.values(), dt)) or dt <= 0:
            self.reset()
            return Filtered(ZERO, ("nonfinite_or_invalid_interval",))
        reasons = []
        prev = self.previous
        if prev != ZERO and self.mode != mode:
            reasons.append("mode_change_requires_zero")
            raw, mode = ZERO, self.mode

        vx, vy, wz = raw.values()
        if mode == Mode.ACKERMANN:
            if vy != 0:
                reasons.append("ackermann_no_lateral")
            vy = 0.0
            if direction and vx * direction < 0:
                vx = 0.0
                reasons.append("ackermann_direction")
        elif mode == Mode.OBLIQUE:
            if wz != 0:
                reasons.append("oblique_no_yaw")
            wz = 0.0
        else:
            if vx != 0 or vy != 0:
                reasons.append("spin_no_translation")
            vx = vy = 0.0

        speed = math.hypot(vx, vy)
        if speed > self.limits.max_speed:
            factor = self.limits.max_speed / speed
            vx, vy = vx * factor, vy * factor
            reasons.append("speed")
        if abs(wz) > self.limits.max_yaw_rate:
            wz = clamp(wz, self.limits.max_yaw_rate)
            reasons.append("yaw_rate")

        if mode == Mode.ACKERMANN:
            safe_wz = clamp(wz, abs(vx) / self.limits.guard_radius)
            if safe_wz != wz:
                reasons.append("ackermann_radius")
            wz = safe_wz
            # Each half of the Ackermann envelope is convex. Pass through zero
            # for a reversal so simultaneous rate and radius bounds still hold.
            if prev.vx * vx < 0:
                vx = wz = 0.0
                reasons.append("ackermann_reversal_through_zero")
        elif mode == Mode.OBLIQUE:
            # A nonzero longitudinal-only Twist would select Ackermann.
            if vx and abs(vy) < self.limits.parallel_min_vy:
                vy = math.copysign(self.limits.parallel_min_vy, vy or oblique_direction or prev.vy or 1.)
                vx = clamp(vx, math.sqrt(self.limits.max_speed ** 2 - vy ** 2))
                reasons.append("oblique_encoding")
            if prev.vy * vy < 0:
                vx = vy = 0.0
                reasons.append("oblique_reversal_through_zero")

        desired = Twist(vx, vy, wz)
        linear_delta = math.hypot(vx - prev.vx, vy - prev.vy)
        angular_delta = abs(wz - prev.wz)
        linear_fraction = self.limits.max_accel * dt / linear_delta if linear_delta else 1.
        angular_fraction = self.limits.max_yaw_accel * dt / angular_delta if angular_delta else 1.
        if linear_fraction < 1:
            reasons.append("acceleration")
        if angular_fraction < 1:
            reasons.append("angular_acceleration")
        step = min(1., linear_fraction, angular_fraction)
        self.previous = desired if step == 1. else prev.interpolate(desired, step)
        self.mode = mode
        return Filtered(self.previous, tuple(reasons))
