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
    high_steer_speed: float = 0.
    high_steer_angle: float = .3490658503988659
    wheelbase: float = .494
    track: float = .364

    def __post_init__(self):
        if not all(math.isfinite(value) and value > 0
                   for name, value in vars(self).items() if name != "high_steer_speed"):
            raise ValueError("velocity limits, radii and margins must be finite and positive")
        if not 0 <= self.high_steer_speed <= self.max_speed or self.high_steer_angle >= math.pi / 2:
            raise ValueError("invalid steering speed envelope")
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

    def apply(self, raw, mode, dt, direction=0.0, oblique_direction=None, continuous_oblique=False):
        if (not all(math.isfinite(value) for value in (*raw.values(), dt)) or dt <= 0
                or (oblique_direction is not None and not math.isfinite(oblique_direction))):
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
            last_branch = prev.vy or 1.
            if oblique_direction is None:
                branch = vy or last_branch
            else:
                branch = oblique_direction if abs(oblique_direction) > 1e-9 else last_branch
            # Constrain lateral feedback to the half-plane of the reference.
            # A nonzero longitudinal-only Twist would select Ackermann.
            crosses_reference = vy * branch < 0.
            if crosses_reference or (vx and abs(vy) < self.limits.parallel_min_vy):
                vy = math.copysign(self.limits.parallel_min_vy, branch)
                vx = clamp(vx, math.sqrt(self.limits.max_speed ** 2 - vy ** 2))
                reasons.append("oblique_reference_branch" if crosses_reference else "oblique_encoding")
            forward_progress = prev.vx * vx > 0 and prev.vx * vx + prev.vy * vy > 0
            if prev.vy * vy < 0 and not (continuous_oblique and forward_progress):
                vx = vy = 0.0
                reasons.append("oblique_reversal_through_zero")

        steering_cap = self.limits.high_steer_speed
        if steering_cap > 0 and mode != Mode.SPIN:
            tangent = math.tan(self.limits.high_steer_angle)
            ackermann_bound = 2 * tangent / (self.limits.wheelbase + self.limits.track * tangent)

            def high_steer(x, y, yaw):
                if mode == Mode.ACKERMANN:
                    return abs(yaw) > ackermann_bound * abs(x)
                return abs(y) > tangent * abs(x)

            target_speed = math.hypot(vx, vy)
            if prev.speed > steering_cap and high_steer(vx, vy, wz):
                # Brake in the previous direction before increasing steering.
                factor = min(steering_cap, target_speed) / prev.speed
                vx, vy, wz = prev.vx * factor, prev.vy * factor, prev.wz * factor
                reasons.append("high_steer_speed")
            elif target_speed > steering_cap and (high_steer(*prev.values()) or high_steer(vx, vy, wz)):
                factor = steering_cap / target_speed
                vx, vy, wz = vx * factor, vy * factor, wz * factor
                reasons.append("high_steer_speed")

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
        command = desired if step == 1. else prev.interpolate(desired, step)
        if (continuous_oblique and mode == Mode.OBLIQUE and prev.vy * vy < 0
                and command.vx and abs(command.vy) < self.limits.parallel_min_vy):
            encoding = min(self.limits.parallel_min_vy, abs(prev.vy), abs(vy))
            lateral = math.copysign(encoding, vy)
            if math.hypot(command.vx - prev.vx, lateral - prev.vy) > self.limits.max_accel * dt:
                lateral = math.copysign(encoding, prev.vy)
            command = Twist(command.vx, lateral, command.wz)
        self.previous = command
        self.mode = mode
        return Filtered(self.previous, tuple(reasons))
