"""ROS-independent planar vehicle state and trajectory contract."""
from dataclasses import dataclass
from enum import IntEnum
import math


class Mode(IntEnum):
    ACKERMANN = 0
    OBLIQUE = 1
    SPIN = 2


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

    def error_to(self, reference):
        dx, dy = reference.x - self.x, reference.y - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy, wrap(reference.yaw - self.yaw)


@dataclass(frozen=True)
class Twist:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)

    def values(self):
        return self.vx, self.vy, self.wz

    def interpolate(self, other, fraction):
        return Twist(*(a + fraction * (b - a) for a, b in zip(self.values(), other.values())))


ZERO = Twist()


def between(a, b, dt):
    """Recover a constant body twist from a short SE(2) increment."""
    dyaw = wrap(b.yaw - a.yaw)
    half = dyaw * 0.5
    dx, dy = b.x - a.x, b.y - a.y
    c, s = math.cos(a.yaw + half), math.sin(a.yaw + half)
    scale = math.sin(half) / half if half else 1.0
    return Twist((c * dx + s * dy) / (dt * scale),
                 (-s * dx + c * dy) / (dt * scale), dyaw / dt)


def advance(pose, twist, dt):
    half = twist.wz * dt * 0.5
    scale = math.sin(half) / half if half else 1.0
    c, s = math.cos(pose.yaw + half), math.sin(pose.yaw + half)
    return Pose(pose.x + dt * scale * (c * twist.vx - s * twist.vy),
                pose.y + dt * scale * (s * twist.vx + c * twist.vy),
                pose.yaw + twist.wz * dt)


@dataclass(frozen=True)
class Point:
    time_ns: int
    pose: Pose
    speed: float
    mode: int

    @property
    def time(self):
        return self.time_ns * 1e-9


@dataclass(frozen=True)
class Trajectory:
    stamp_ns: int
    frame: str
    plan_id: int
    points: tuple

    @property
    def duration(self):
        return self.points[-1].time


@dataclass(frozen=True)
class Reference:
    time: float
    pose: Pose
    twist: Twist
    mode: Mode


@dataclass(frozen=True)
class Observation:
    pose: Pose
    twist: Twist
    stamp: float
    mode: Mode
    mode_stamp: float
