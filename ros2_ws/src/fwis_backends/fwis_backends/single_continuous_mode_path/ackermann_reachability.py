"""Research timing profiles for a fixed single-vehicle Ackermann path."""

from bisect import bisect_right
from dataclasses import dataclass
import math

import numpy as np

from fwis_backends.path_reachability import (VehicleDerivativeBounds, parameterize,
                                              progress_bounds)
from .core import Action
from .retiming import wheel_configuration


def _hermite_bounds(left, right):
    """Maximum absolute curvature and spatial derivative on a Hermite interval."""
    width = right.s - left.s
    slope_l, slope_r = width * left.derivative, width * right.derivative
    c0, c1 = left.value, slope_l
    c2 = 3.0 * (right.value - left.value) - 2.0 * slope_l - slope_r
    c3 = 2.0 * (left.value - right.value) + slope_l + slope_r
    stations = [0.0, 1.0]
    for root in np.polynomial.polynomial.polyroots((c1, 2.0 * c2, 3.0 * c3)):
        if np.isreal(root) and 0 < root < 1:
            stations.append(float(root))
    inflection = -c2 / (3.0 * c3) if c3 else 0.0
    slopes = [0.0, 1.0]
    if 0 < inflection < 1:
        slopes.append(inflection)
    curvature = max(abs(c0 + u * (c1 + u * (c2 + u * c3))) for u in stations)
    derivative = max(abs(c1 + u * (2.0 * c2 + u * 3.0 * c3)) / width for u in slopes)
    return curvature, derivative


def _vehicle_bounds(curvature, curvature_rate, limits):
    if curvature > 1.0 / limits.r_min_ack + 1e-10:
        raise ValueError("Ackermann curvature exceeds the minimum-radius contract")
    speed = limits.target_v
    if curvature:
        speed = min(speed, math.sqrt(limits.lateral_a_max / curvature))
    wheel_rate_factor = 0.0
    maximum_steer_rate_per_progress = 0.0
    for sign in (-1.0, 1.0):
        angles, factors = wheel_configuration(Action(0, 1.0, sign * curvature), limits)
        if max(abs(angle) for angle in angles) > limits.steer_angle_max + 1e-10:
            raise ValueError("Ackermann wheel angle exceeds limit")
        wheel_rate_factor = max(wheel_rate_factor, max(factors))
    if curvature_rate:
        denominator = max(1e-9, 1.0 - 0.5 * limits.track * curvature) ** 2
        maximum_steer_rate_per_progress = 0.5 * limits.wheelbase * curvature_rate / denominator
    if maximum_steer_rate_per_progress:
        speed = min(speed, limits.steer_rate_max / maximum_steer_rate_per_progress)
    return VehicleDerivativeBounds(
        length_rate=1.0, length_second_rate=0.0,
        yaw_rate=curvature, yaw_second_rate=curvature_rate,
        max_speed=min(speed, limits.wheel_speed_max / wheel_rate_factor),
        max_acceleration=limits.a_max,
        max_yaw_rate=limits.translation_omega_max or limits.omega_max,
        max_yaw_acceleration=limits.translation_alpha_max or limits.alpha_max,
    )


def pure_ackermann_profile(block, limits, *, start_speed=0.0, end_speed=0.0):
    """Stop-to-stop scalar reachability over one continuous A block.

    Uses exact curvature extrema and conservative angular-acceleration bounds.
    Acceleration transitions are discontinuous. This research profile is not an
    independently certified trajectory and does not satisfy finite jerk limits.
    """
    if block.mode != 0 or block.length <= 0:
        raise ValueError("need a nonempty Ackermann block with fixed travel direction")
    progress = tuple(knot.s for knot in block.knots)
    intervals = [progress_bounds((_vehicle_bounds(*_hermite_bounds(left, right), limits),))
                 for left, right in zip(block.knots[:-1], block.knots[1:], strict=True)]
    return parameterize(progress, intervals, start_speed=start_speed, end_speed=end_speed)


@dataclass(frozen=True)
class JerkProfile:
    progress: tuple[float, ...]
    times_s: tuple[float, ...]
    speeds: tuple[float, ...]
    accelerations: tuple[float, ...]
    jerks: tuple[float, ...]

    @property
    def duration_s(self):
        return self.times_s[-1]

    def state_at(self, time_s):
        """Sample (progress, speed, acceleration, jerk) from the profile."""
        if not math.isfinite(time_s) or not 0.0 <= time_s <= self.duration_s:
            raise ValueError("time outside jerk-limited profile")
        if time_s == self.duration_s:
            return self.progress[-1], 0.0, 0.0, 0.0
        i = bisect_right(self.times_s, time_s) - 1
        dt = time_s - self.times_s[i]
        v, a, j = self.speeds[i], self.accelerations[i], self.jerks[i]
        return (self.progress[i] + dt * (v + dt * (a / 2.0 + j * dt / 6.0)),
                v + dt * (a + j * dt / 2.0), a + j * dt, j)


def constant_ackermann_jerk_profile(action, limits):
    """Zero endpoint speed/acceleration S-curve for one constant-curvature A edge."""
    if (action.mode != 0 or not math.isfinite(action.amount) or not math.isfinite(action.control)
            or action.amount == 0.0):
        raise ValueError("need a finite nonzero constant Ackermann action")
    curvature, distance = abs(action.control), abs(action.amount)
    bounds = progress_bounds((_vehicle_bounds(curvature, 0.0, limits),))
    acceleration = min(bounds.max_acceleration, -bounds.min_acceleration)
    angular_jerk = limits.translation_angular_jerk_max or limits.angular_jerk_max
    jerk = min(limits.jerk_max, angular_jerk / curvature) if curvature else limits.jerk_max

    def ramp_times(max_speed):
        ramp = min(acceleration / jerk, math.sqrt(max_speed / jerk))
        hold = max(0.0, max_speed / (jerk * ramp) - ramp) if ramp else 0.0
        return ramp, hold

    def moving_distance(max_speed):
        ramp, hold = ramp_times(max_speed)
        return max_speed * (2.0 * ramp + hold)

    peak_speed = bounds.max_speed
    has_cruise = moving_distance(peak_speed) <= distance
    if not has_cruise:
        lower, upper = 0.0, peak_speed
        for _ in range(60):
            test = (lower + upper) / 2.0
            if moving_distance(test) > distance:
                upper = test
            else:
                lower = test
        peak_speed = lower
    ramp, hold = ramp_times(peak_speed)
    cruise = max(0.0, (distance - moving_distance(peak_speed)) / peak_speed) if has_cruise else 0.0
    durations = (ramp, hold, ramp, cruise, ramp, hold, ramp)
    slopes = (jerk, 0.0, -jerk, 0.0, -jerk, 0.0, jerk)
    points, times, speeds, accelerations, jerks = [0.0], [0.0], [0.0], [0.0], []
    for dt, j in zip(durations, slopes, strict=True):
        if dt <= math.ulp(times[-1]):
            continue
        v, a = speeds[-1], accelerations[-1]
        points.append(points[-1] + dt * (v + dt * (a / 2.0 + j * dt / 6.0)))
        speeds.append(v + dt * (a + j * dt / 2.0))
        accelerations.append(a + j * dt)
        times.append(times[-1] + dt)
        jerks.append(j)
    points[-1], speeds[-1], accelerations[-1] = distance, 0.0, 0.0
    return JerkProfile(tuple(points), tuple(times), tuple(speeds), tuple(accelerations), tuple(jerks))
