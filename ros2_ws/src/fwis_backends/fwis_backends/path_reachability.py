"""Scalar reachability timing for fixed paths with certified interval bounds.

Progress s has one consistent unit throughout a profile, usually metres
or radians for a formation arc. Path derivative bounds are nonnegative
interval suprema in the matching coordinate. The routine solves constraints in x = ds/dt squared, where
dx/ds = 2 d²s/dt². The returned acceleration can jump at phase boundaries.
Callers requiring jerk bounds must smooth and independently re-audit it.
"""

from bisect import bisect_right
from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class ProgressBounds:
    max_speed: float
    min_acceleration: float
    max_acceleration: float

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in (self.max_speed, self.min_acceleration,
                                              self.max_acceleration))
                or self.max_speed <= 0.0
                or not self.min_acceleration < 0.0 < self.max_acceleration):
            raise ValueError("progress bounds must have finite positive speed and bidirectional acceleration")


@dataclass(frozen=True)
class VehicleDerivativeBounds:
    length_rate: float
    length_second_rate: float
    yaw_rate: float
    yaw_second_rate: float
    max_speed: float
    max_acceleration: float
    max_yaw_rate: float
    max_yaw_acceleration: float

    def __post_init__(self):
        derivatives = (self.length_rate, self.length_second_rate, self.yaw_rate, self.yaw_second_rate)
        limits = (self.max_speed, self.max_acceleration, self.max_yaw_rate, self.max_yaw_acceleration)
        if (not all(math.isfinite(v) and v >= 0.0 for v in derivatives)
                or not all(math.isfinite(v) and v > 0.0 for v in limits)):
            raise ValueError("derivative bounds must be finite and nonnegative, limits finite and positive")


@dataclass(frozen=True)
class ReachabilityProfile:
    progress: tuple[float, ...]
    times_s: tuple[float, ...]
    speeds: tuple[float, ...]
    accelerations: tuple[float, ...]

    @property
    def duration_s(self):
        return self.times_s[-1]

    def state_at(self, time_s):
        """Return progress, speed and outgoing acceleration for a time in seconds."""
        if not math.isfinite(time_s) or not 0.0 <= time_s <= self.duration_s:
            raise ValueError("sample time lies outside this profile")
        if time_s == self.duration_s:
            return self.progress[-1], self.speeds[-1], 0.0
        index = bisect_right(self.times_s, time_s) - 1
        dt = time_s - self.times_s[index]
        speed = self.speeds[index]
        acceleration = self.accelerations[index]
        return (self.progress[index] + dt * (speed + 0.5 * acceleration * dt),
                speed + acceleration * dt, acceleration)


def progress_bounds(vehicles: Sequence[VehicleDerivativeBounds]) -> ProgressBounds:
    """Reduce all per-vehicle limits to one conservative progress envelope.

    If both derivative terms in a vehicle's acceleration are nonzero,
    allocate half the acceleration limit to each term using the triangle
    inequality. The reduction does not depend on derivative signs.
    """
    speed, acceleration = math.inf, math.inf
    for vehicle in vehicles:
        if vehicle.length_rate:
            speed = min(speed, vehicle.max_speed / vehicle.length_rate)
        if vehicle.yaw_rate:
            speed = min(speed, vehicle.max_yaw_rate / vehicle.yaw_rate)
        for first, second, limit in (
            (vehicle.length_rate, vehicle.length_second_rate, vehicle.max_acceleration),
            (vehicle.yaw_rate, vehicle.yaw_second_rate, vehicle.max_yaw_acceleration),
        ):
            if first:
                acceleration = min(acceleration, limit / ((2.0 if second else 1.0) * first))
            if second:
                speed = min(speed, math.sqrt(limit / ((2.0 if first else 1.0) * second)))
    return ProgressBounds(speed, -acceleration, acceleration)


def _forward_envelope(knots, intervals, first_speed, last_speed):
    # Adjacent intervals both constrain the speed at their common knot.
    caps = [intervals[0].max_speed ** 2]
    caps.extend(min(before.max_speed, after.max_speed) ** 2
                for before, after in zip(intervals[:-1], intervals[1:]))
    caps.append(intervals[-1].max_speed ** 2)
    first, last = first_speed ** 2, last_speed ** 2
    if first > caps[0] or last > caps[-1]:
        raise ValueError("endpoint speed exceeds an interval speed cap")
    caps[-1] = last
    lengths = [right - left for left, right in zip(knots[:-1], knots[1:])]
    for index in reversed(range(len(intervals))):
        caps[index] = min(caps[index], caps[index + 1] - 2.0 * intervals[index].min_acceleration * lengths[index])
    if first > caps[0]:
        raise ValueError("start speed cannot brake to the requested end speed")
    squares = [first]
    for cap, length, bounds in zip(caps[1:], lengths, intervals, strict=True):
        squares.append(min(cap, squares[-1] + 2.0 * bounds.max_acceleration * length))
    tolerance = 1e-12 * max(last, squares[-1])
    if last - squares[-1] > tolerance:
        raise ValueError("requested end speed lies outside the reachable set")
    squares[-1] = last
    return squares, lengths


def parameterize(progress: Sequence[float], intervals: Sequence[ProgressBounds], *,
                 start_speed=0.0, end_speed=0.0) -> ReachabilityProfile:
    """Integrate the scalar backward controllable sets and forward speed envelope.

    Knots must increase strictly, and each interval needs a speed cap and
    acceleration bounds that straddle zero. The algorithm also inserts
    exact acceleration/cruise/braking switches inside each interval.
    Endpoint speed values and the original knots are preserved.
    """
    knots = tuple(progress)
    if (len(knots) < 2 or len(intervals) != len(knots) - 1
            or any(not math.isfinite(s) for s in knots)
            or any(left >= right for left, right in zip(knots[:-1], knots[1:]))
            or not all(math.isfinite(v) and v >= 0.0 for v in (start_speed, end_speed))):
        raise ValueError("invalid progress knots, interval count or endpoint speed")
    squares, lengths = _forward_envelope(knots, intervals, start_speed, end_speed)
    offsets, times, speeds, accelerations = [knots[0]], [0.0], [start_speed], []

    def append(offset, speed):
        length = offset - offsets[-1]
        if length <= 0 or speed + speeds[-1] <= 0:
            raise ValueError("non-positive distance or traversal speed")
        dt = 2.0 * length / (speeds[-1] + speed)
        end_time = times[-1] + dt
        if not math.isfinite(end_time) or end_time <= times[-1]:
            raise ValueError("segment duration is outside the representable range")
        accelerations.append((speed - speeds[-1]) / dt)
        offsets.append(offset)
        times.append(end_time)
        speeds.append(speed)

    for index, bounds in enumerate(intervals):
        length = lengths[index]
        first, last = squares[index:index + 2]
        accelerate, brake = bounds.max_acceleration, -bounds.min_acceleration
        peak = min(bounds.max_speed ** 2,
                   (brake * first + accelerate * last + 2.0 * brake * accelerate * length)
                   / (accelerate + brake))
        endpoint_peak = max(first, last)
        # Degenerate triangular profiles can acquire a tiny artificial brake
        # or ramp from the closed-form intersection's rounding error.
        if peak - endpoint_peak <= 32.0 * max(math.ulp(peak), math.ulp(endpoint_peak)):
            peak = endpoint_peak
        first_piece = len(accelerations)
        acceleration_end = knots[index] + (peak - first) / (2.0 * accelerate)
        braking_start = knots[index + 1] - (peak - last) / (2.0 * brake)
        peak_speed = math.sqrt(peak)
        # Omit coincident switch points at the triangular and saturated boundaries.
        rounding = 8.0 * max(math.ulp(knots[index]), math.ulp(knots[index + 1]))
        if offsets[-1] + rounding < acceleration_end < knots[index + 1] - rounding:
            append(acceleration_end, peak_speed)
        if offsets[-1] + rounding < braking_start < knots[index + 1] - rounding:
            append(braking_start, peak_speed)
        append(knots[index + 1], math.sqrt(last))
        epsilon = 1e-9 * max(1.0, -bounds.min_acceleration, bounds.max_acceleration)
        if any(not bounds.min_acceleration - epsilon <= a <= bounds.max_acceleration + epsilon
               for a in accelerations[first_piece:]):
            raise ValueError("floating-point timing cannot preserve the acceleration envelope")

    return ReachabilityProfile(tuple(offsets), tuple(times), tuple(speeds), tuple(accelerations))
