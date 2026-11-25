"""Common rest-to-rest reachability clock for an immutable formation edge."""
from dataclasses import dataclass
import math

from ..path_reachability import ProgressBounds, parameterize


@dataclass(frozen=True)
class JerkClock:
    """Seven symmetric constant-jerk lobes, with optional holds and cruise.

    Persist only the three durations, in seconds. The edge supplies the
    progression length in metres or radians. Position, speed and acceleration
    are continuous across lobes, and the jerk is bounded on each side of a join.
    """

    ramp_s: float
    hold_s: float
    cruise_s: float

    def __post_init__(self):
        durations = (self.ramp_s, self.hold_s, self.cruise_s)
        if (not all(isinstance(value, (int, float)) and not isinstance(value, bool)
                    and math.isfinite(value) and 0 <= value <= 3600 for value in durations)
                or self.ramp_s < 1e-6):
            raise ValueError("invalid jerk clock durations")

    @property
    def duration_s(self):
        return 4 * self.ramp_s + 2 * self.hold_s + self.cruise_s

    def suprema(self, length):
        """Progress speed, acceleration and absolute jerk suprema."""
        r, h, c = self.ramp_s, self.hold_s, self.cruise_s
        velocity = length / (2 * r + h + c)
        acceleration = velocity / (r + h)
        return velocity, acceleration, acceleration / r

    def sample(self, time_s, length):
        """Sample progress and three derivatives on the closed time domain."""
        duration = self.duration_s
        if not math.isfinite(time_s) or not 0 <= time_s <= duration:
            raise ValueError("time outside the jerk clock")
        r, h = self.ramp_s, self.hold_s
        top_speed, top_acceleration, jerk = self.suprema(length)
        if time_s == 0.0:
            return 0.0, 0.0, 0.0, jerk
        if time_s == duration:
            return length, 0.0, 0.0, jerk
        reverse = time_s > duration / 2
        t = duration - time_s if reverse else time_s
        ramp_position = top_acceleration * r * r / 6
        ramp_speed = top_acceleration * r / 2
        if t < r:
            position, speed, acceleration, local_jerk = jerk * t**3 / 6, jerk * t*t / 2, jerk*t, jerk
        elif t < r + h:
            dt = t - r
            position = ramp_position + dt * (ramp_speed + top_acceleration * dt / 2)
            speed, acceleration, local_jerk = ramp_speed + top_acceleration * dt, top_acceleration, 0.0
        elif t < 2 * r + h:
            # Integrate backwards from the exact speed at the start of cruise.
            tail = 2 * r + h - t
            position = top_speed * ((2 * r + h) / 2 - tail) + jerk * tail**3 / 6
            speed, acceleration, local_jerk = top_speed - jerk * tail*tail / 2, jerk*tail, -jerk
        else:
            position = length / 2 - top_speed * (duration / 2 - t)
            speed, acceleration, local_jerk = top_speed, 0.0, 0.0
        if reverse:
            return length - position, speed, -acceleration, local_jerk
        return position, speed, acceleration, local_jerk


def reachability_clock(length, velocity, acceleration, jerk, *, min_duration_s=0.0):
    """Jerk-smooth the reachable speed envelope for constant derivative bounds."""
    if not all(math.isfinite(x) and x > 0 for x in (length, velocity, acceleration, jerk)):
        raise ValueError("progress and derivative caps must be positive")
    reach_bounds = ProgressBounds(velocity, -acceleration, acceleration)
    reachable = parameterize((0.0, length / 2, length), (reach_bounds,) * 2)
    upper = min(velocity, max(reachable.speeds))

    def ramp_times(speed):
        ramp = min(acceleration / jerk, math.sqrt(speed / jerk))
        return ramp, max(0.0, speed / (jerk * ramp) - ramp)

    def stop_distance(speed):
        ramp, hold = ramp_times(speed)
        return speed * (2 * ramp + hold)

    lower = 0.0
    for _ in range(48):
        speed = (lower + upper) / 2
        if stop_distance(speed) > length:
            upper = speed
        else:
            lower = speed
    speed = lower
    ramp, hold = ramp_times(speed)
    cruise = max(0.0, length / speed - 2 * ramp - hold)
    scale = max(1.0, min_duration_s / (4 * ramp + 2 * hold + cruise))
    return JerkClock(ramp * scale, hold * scale, cruise * scale)
