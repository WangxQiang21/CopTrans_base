"""Independent parametrized Ranger test trajectories."""
from dataclasses import asdict, dataclass, replace
import math

from fwis_control.model import Mode, Point, Pose, Trajectory, Twist, advance, wrap


A, O, S = Mode
FIXTURE_STAMP = 1_000_000_000


@dataclass(frozen=True)
class Leg:
    mode: Mode
    distance: float  # m in A/O, rad in S
    speed: float
    shape: str = "constant"
    parameter: float = 0.


@dataclass(frozen=True)
class Scenario:
    name: str
    source: str
    family: str
    trajectory: Trajectory
    expected_pass: bool = True
    provenance: object = None

    def __post_init__(self):
        alphabet = "abcdefghijklmnopqrstuvwxyz0123456789_-"
        if not self.name or any(c not in alphabet for c in self.name):
            raise ValueError(f"invalid scenario name: {self.name!r}")

    def document(self):
        t = self.trajectory
        return {"name": self.name, "source": self.source, "family": self.family,
                "expected_pass": self.expected_pass, "provenance": self.provenance,
                "stamp_ns": t.stamp_ns, "frame_id": t.frame, "plan_id": t.plan_id,
                "points": [dict(t_ns=p.time_ns, x=p.pose.x, y=p.pose.y, yaw=p.pose.yaw,
                                v=p.speed, mode=p.mode) for p in t.points]}


def from_document(raw):
    points = tuple(Point(p["t_ns"], Pose(p["x"], p["y"], p["yaw"]), p["v"], p["mode"])
                   for p in raw["points"])
    tr = Trajectory(raw["stamp_ns"], raw["frame_id"], raw["plan_id"], points)
    modes = {p.mode for p in points}
    family = Mode(next(iter(modes))).name if len(modes) == 1 and modes <= set(Mode) else "MIXED"
    return Scenario(raw["name"], raw["source"], raw.get("family", family), tr,
                    raw.get("expected_pass", True), raw.get("provenance"))


def shape(leg, progress):
    if leg.shape == "s":
        return leg.parameter * math.sin(2. * math.pi * progress)
    if leg.shape == "bend":
        return leg.parameter * math.sin(math.pi * progress) ** 2
    if leg.shape == "composite":
        if progress < .15 or progress > .88:
            return 0.
        u = (progress - .15) / .73
        return leg.parameter * math.sin(2 * math.pi * u) * math.sin(math.pi * u)
    return leg.parameter


def accelerating_profile(length, peak, accel):
    peak = min(peak, math.sqrt(2 * length * accel / math.pi), length / .60)
    ramp = max(.60, peak * math.pi / (2 * accel))
    duration = length / peak + ramp

    def speed(time):
        fraction = min(1., max(0., min(time, duration - time) / ramp))
        return peak * (1. - math.cos(math.pi * fraction)) * .5

    return duration, speed


def sample_case(name, legs, *, hold=.80, expected_pass=True, plan_id=1, yaw=.0):
    pose = Pose(0., 0., yaw)
    points = [Point(0, replace(pose, yaw=wrap(pose.yaw)), 0., legs[0].mode)]
    time = 0.
    previous_mode = A
    for leg in legs:
        if previous_mode != leg.mode and hold:
            steps = math.ceil(hold / .039)
            for index in range(1, steps + 1):
                points.append(Point(round((time + index * hold / steps) * 1e9),
                                    replace(pose, yaw=wrap(pose.yaw)), 0., leg.mode))
            time += hold
        magnitude = abs(leg.distance)
        direction = math.copysign(1., leg.distance)
        duration, speed = accelerating_profile(magnitude, leg.speed, .42 if leg.mode == S else .27)
        steps = math.ceil(duration / .04)
        offsets = [duration * index / steps + .005 * math.sin(math.pi * index * .5)
                   for index in range(steps + 1)]
        offsets[-1] = duration
        station = 0.
        for left, right in zip(offsets, offsets[1:]):
            dt = right - left
            value = speed((left + right) * .5)
            u = min(1., (station + value * dt * .5) / magnitude)
            signed = direction * value
            if leg.mode == A:
                twist = Twist(signed, 0., signed * shape(leg, u))
            elif leg.mode == O:
                twist = Twist(signed * math.cos(leg.parameter),
                              signed * math.sin(leg.parameter), 0.)
            else:
                twist = Twist(wz=signed)
            pose = advance(pose, twist, dt)
            station += value * dt
            points.append(Point(round((time + right) * 1e9), replace(pose, yaw=wrap(pose.yaw)),
                                0. if leg.mode == S else direction * speed(right), leg.mode))
        time += duration
        previous_mode = leg.mode

    family = legs[0].mode.name if len({leg.mode for leg in legs}) == 1 else "MIXED"
    return Scenario(name, "synthetic", family, Trajectory(FIXTURE_STAMP, "map", plan_id, tuple(points)),
                    expected_pass, {"legs": [asdict(leg) for leg in legs], "hold_s": hold,
                                    "initial_yaw_rad": yaw})


def suite():
    straight = Leg(A, .60, .18)
    turn = Leg(A, .82, .18, parameter=.82)
    oblique = Leg(O, .38, .15, parameter=.65)
    spin = Leg(S, .80, .38)
    cases = (
        ("ack_straight_speed_profile", [Leg(A, 1.25, .20)], {}),
        ("ack_reverse", [Leg(A, -.80, .18, parameter=.55)], {}),
        ("ack_arc_left", [Leg(A, 1.20, .19, parameter=1 / 1.30)], {}),
        ("ack_arc_right", [Leg(A, 1.30, .20, parameter=-1 / 1.60)], {}),
        ("ack_slow_s", [Leg(A, 1.60, .14, "s", .92)], {}),
        ("ack_fast_s", [Leg(A, 1.60, .24, "s", .92)], {}),
        ("ack_variable_curvature", [Leg(A, 1.12, .20, "bend", .95)], {}),
        ("ack_composite", [Leg(A, 1.65, .21, "composite", 1.05)], {}),
        ("ack_short_near_boundary", [Leg(A, .48, .16, parameter=1 / .85)], {}),
        ("oblique_lateral_left", [Leg(O, .72, .19, parameter=math.pi / 2)], {}),
        ("oblique_lateral_right", [Leg(O, .72, .19, parameter=-math.pi / 2)], {}),
        ("oblique_30deg", [Leg(O, .82, .21, parameter=math.pi / 6)], {}),
        ("oblique_minus60deg", [Leg(O, .85, .20, parameter=-math.pi / 3)], {}),
        ("oblique_direction_change", [oblique, Leg(O, .52, .17, parameter=-.78)], {}),
        ("spin_90", [Leg(S, math.pi / 2, .40)], {}),
        ("spin_180_reverse", [Leg(S, -math.pi, .45)], {}),
        ("spin_270", [Leg(S, 1.5 * math.pi, .46)], {"yaw": 2.9}),
        ("spin_two_turns", [Leg(S, 4 * math.pi, .49)], {"yaw": -2.9}),
        ("mixed_aoa", [straight, oblique, turn], {}),
        ("mixed_asa", [straight, spin, turn], {}),
        ("mixed_oso", [oblique, spin, Leg(O, .50, .17, parameter=-.8)], {}),
        ("mixed_aosa", [turn, oblique, spin, straight], {}),
        ("mixed_near_hold_limit", [straight, oblique, spin, turn], {"hold": .59}),
        ("reject_missing_hold", [straight, spin, straight], {"hold": 0., "expected_pass": False}),
        ("reject_tight_radius", [Leg(A, .55, .19, parameter=1 / .60)], {"expected_pass": False}),
    )
    return [sample_case(name, legs, plan_id=index, **options)
            for index, (name, legs, options) in enumerate(cases, start=1)]
