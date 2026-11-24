"""Check and sample the unmodified CopTrans time-parameterized trajectory."""
from bisect import bisect_right
from dataclasses import dataclass
import math

from .model import Mode, Reference, Twist, ZERO, advance, between, wrap


@dataclass(frozen=True)
class PreflightSettings:
    min_step: float = .0005
    max_step: float = .25
    stationary_position: float = .00000001
    stationary_yaw: float = .00000001
    stop_declared_speed: float = .005
    velocity_tolerance: float = .0003
    oblique_yaw_tolerance: float = .003
    spin_position_tolerance: float = .003
    declared_speed_warn: float = .01
    declared_speed_tolerance: float = .025
    acceleration_tolerance: float = .035

    def __post_init__(self):
        if not all(math.isfinite(value) and value > 0 for value in vars(self).values()):
            raise ValueError("preflight limits must be finite and positive")
        if self.max_step <= self.min_step or self.declared_speed_tolerance <= self.declared_speed_warn:
            raise ValueError("invalid preflight intervals or speed tolerances")


@dataclass(frozen=True)
class Issue:
    level: str
    code: str
    start: float
    end: float
    value: float = 0.
    limit: float = 0.


@dataclass(frozen=True)
class Switch:
    time: float
    resume: float
    mode: Mode


@dataclass(frozen=True)
class Interval:
    start: float
    end: float
    mode: Mode
    velocity: Twist
    stationary: bool

    @property
    def middle(self):
        return (self.start + self.end) * .5


@dataclass(frozen=True)
class Preflight:
    issues: tuple
    switches: tuple
    sampler: object = None

    @property
    def result(self):
        if not self.issues:
            return "PASS"
        return "FAIL" if self.sampler is None else "WARN"


class Sampler:
    def __init__(self, trajectory, intervals):
        self.trajectory = trajectory
        self.intervals = tuple(intervals)
        self.times = tuple(point.time for point in trajectory.points)

    def sample(self, time):
        time = max(0., min(time, self.times[-1]))
        index = bisect_right(self.times, time) - 1
        point = self.trajectory.points[index]
        if index == len(self.intervals):
            return Reference(time, point.pose, ZERO, Mode(point.mode))
        current = self.intervals[index]
        if current.stationary:
            return Reference(time, point.pose, ZERO, current.mode)
        # Recover the interval pose from its endpoints. Smooth the feed-forward
        # between interval midpoints within a continuous run of the same mode.
        pose = advance(point.pose, current.velocity, time - point.time)
        adjacent_index = index - 1 if time < current.middle else index + 1
        anchor_time = current.start if time < current.middle else current.end
        anchor_velocity = ZERO
        if 0 <= adjacent_index < len(self.intervals):
            adjacent = self.intervals[adjacent_index]
            if adjacent.mode == current.mode and not adjacent.stationary:
                anchor_time, anchor_velocity = adjacent.middle, adjacent.velocity
        fraction = (time - current.middle) / (anchor_time - current.middle)
        return Reference(time, pose, current.velocity.interpolate(anchor_velocity, fraction),
                         current.mode)


def preflight(trajectory, limits, settings, switch_budget, initial_mode=Mode.ACKERMANN,
              *, allow_axial_oblique=False):
    issues, switches, intervals = [], [], []
    points = trajectory.points

    def issue(code, a=0., b=0., value=0., limit=0., level="FAIL"):
        issues.append(Issue(level, code, a, b, value, limit))

    if (type(trajectory.plan_id) is not int or not 0 < trajectory.plan_id < 2**32
            or type(trajectory.stamp_ns) is not int or trajectory.stamp_ns <= 0
            or trajectory.frame != "map"):
        issue("invalid_header")
    if len(points) < 2:
        issue("too_few_points")
    for point in points:
        if (type(point.time_ns) is not int
                or not 0 <= point.time_ns < 2**63
                or point.mode not in tuple(Mode)
                or not all(map(math.isfinite, (point.pose.x, point.pose.y, point.pose.yaw, point.speed)))):
            issue("invalid_point")
            break
    if issues:
        return Preflight(tuple(issues), ())
    if points[0].time_ns != 0:
        issue("nonzero_start_time", points[0].time)
    for a, b in zip(points, points[1:]):
        dt = (b.time_ns - a.time_ns) * 1e-9
        if not settings.min_step <= dt <= settings.max_step:
            issue("sample_interval", a.time, b.time, dt,
                  settings.max_step if dt > settings.max_step else settings.min_step)
    if issues:
        return Preflight(tuple(issues), ())

    def check_rate(a, b, ta, tb):
        linear = math.hypot(b.vx - a.vx, b.vy - a.vy) / (tb - ta)
        angular = abs(b.wz - a.wz) / (tb - ta)
        if linear > limits.max_accel + settings.acceleration_tolerance:
            issue("linear_acceleration", ta, tb, linear, limits.max_accel)
        if angular > limits.max_yaw_accel + settings.acceleration_tolerance:
            issue("angular_acceleration", ta, tb, angular, limits.max_yaw_accel)

    for endpoint in (points[0], points[-1]):
        if abs(endpoint.speed) > settings.stop_declared_speed:
            issue("endpoint_speed", endpoint.time, endpoint.time, abs(endpoint.speed),
                  settings.stop_declared_speed)

    mode_origin = points[0].pose
    for a, b in zip(points, points[1:]):
        start, end = a.time, b.time
        duration = end - start
        mode = Mode(a.mode)
        chord = math.hypot(b.pose.x - a.pose.x, b.pose.y - a.pose.y)
        raw_yaw_delta = b.pose.yaw - a.pose.yaw
        if not math.isfinite(chord) or not math.isfinite(raw_yaw_delta):
            issue("pose_delta_overflow", start, end)
            return Preflight(tuple(issues), ())
        yaw_delta = abs(wrap(raw_yaw_delta))
        if abs(raw_yaw_delta) > 2 * math.pi + settings.stationary_yaw:
            issue("yaw_discontinuity", start, end, abs(raw_yaw_delta), 2 * math.pi)
        stationary = chord <= settings.stationary_position and yaw_delta <= settings.stationary_yaw
        if mode != b.mode and (not stationary or
                               max(abs(a.speed), abs(b.speed)) > settings.stop_declared_speed):
            issue("moving_mode_boundary", start, end)
        velocity = ZERO if stationary else between(a.pose, b.pose, duration)
        if not all(map(math.isfinite, (*velocity.values(), velocity.speed))):
            issue("nonfinite_interval_velocity", start, end)
            return Preflight(tuple(issues), ())
        intervals.append(Interval(start, end, mode, velocity, stationary))
        vx, vy, wz = velocity.values()
        linear = velocity.speed
        tol = settings.velocity_tolerance
        if linear > limits.max_speed + tol:
            issue("linear_speed", start, end, linear, limits.max_speed)
        if abs(wz) > limits.max_yaw_rate + tol:
            issue("yaw_rate", start, end, abs(wz), limits.max_yaw_rate)
        declared = a.speed * .5 + b.speed * .5
        if mode == Mode.ACKERMANN:
            discrepancy = abs(vx - declared)
            if abs(vy) > tol:
                issue("ackermann_lateral_velocity", start, end, abs(vy), tol)
            min_vx = limits.guard_radius * abs(wz)
            # Compare velocities directly to keep the check defined at a stop.
            if min_vx > abs(vx) * 1.001 + 1e-8:
                issue("ackermann_radius", start, end, abs(vx) / abs(wz), limits.guard_radius)
        elif mode == Mode.OBLIQUE:
            discrepancy = abs(linear - abs(declared))
            if abs(vx) > tol and vx * declared < 0:
                issue("oblique_direction", start, end, vx, declared)
            if (not allow_axial_oblique and not stationary and linear > tol
                    and abs(vy) < limits.parallel_min_vy):
                issue("oblique_twist_encodes_ackermann", start, end, abs(vy), limits.parallel_min_vy)
            drift = abs(wrap(b.pose.yaw - mode_origin.yaw))
            if abs(wz) > tol or drift > settings.oblique_yaw_tolerance:
                issue("oblique_yaw_change", start, end, drift, settings.oblique_yaw_tolerance)
        else:
            discrepancy = max(abs(a.speed), abs(b.speed))
            drift = math.hypot(b.pose.x - mode_origin.x, b.pose.y - mode_origin.y)
            if linear > tol or drift > settings.spin_position_tolerance:
                issue("spin_translation", start, end, drift, settings.spin_position_tolerance)
        if discrepancy > settings.declared_speed_warn:
            level = "FAIL" if discrepancy > settings.declared_speed_tolerance else "WARN"
            issue("declared_speed_mismatch", start, end, discrepancy,
                  settings.declared_speed_tolerance, level)
        if a.mode != b.mode:
            mode_origin = b.pose

    for index, current in enumerate(intervals):
        if current.stationary:
            continue
        previous = intervals[index - 1] if index else None
        following = intervals[index + 1] if index + 1 < len(intervals) else None
        if previous is None or previous.stationary or previous.mode != current.mode:
            check_rate(ZERO, current.velocity, current.start, current.middle)
        else:
            check_rate(previous.velocity, current.velocity, previous.middle, current.middle)
        if following is None or following.stationary or following.mode != current.mode:
            check_rate(current.velocity, ZERO, current.middle, current.end)

    old_mode = initial_mode
    for index, point in enumerate(points):
        if point.mode == old_mode:
            continue
        target = Mode(point.mode)
        tail = index
        while (tail < len(intervals) and intervals[tail].mode == target
               and intervals[tail].stationary and points[tail + 1].mode == target
               and max(abs(points[tail].speed), abs(points[tail + 1].speed))
               <= settings.stop_declared_speed):
            tail += 1
        window = points[tail].time - point.time
        if window + 1e-9 < switch_budget:
            issue("insufficient_mode_hold", point.time, points[tail].time, window, switch_budget)
        switches.append(Switch(point.time, points[tail].time, target))
        old_mode = target

    if any(item.level == "FAIL" for item in issues):
        return Preflight(tuple(issues), tuple(switches))
    return Preflight(tuple(issues), tuple(switches), Sampler(trajectory, intervals))
