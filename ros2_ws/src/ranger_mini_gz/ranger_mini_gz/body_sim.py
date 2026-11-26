"""Ranger body-level response, with observation from actual Gazebo link poses."""
from dataclasses import dataclass
import math

from gz.math7 import Pose3d
from gz.sim8 import Link, Model


ACKERMANN, OBLIQUE, SPIN = range(3)
ZERO = (0., 0., 0.)


@dataclass(frozen=True)
class PlantSettings:
    frequency: float = 50.
    linear_tau: float = .075
    angular_tau: float = .095
    max_accel: float = .65
    max_yaw_accel: float = .9
    max_speed: float = .28
    max_yaw_rate: float = .55
    switch_radius: float = .47644
    switch_delay: float = .10
    command_timeout: float = .25
    stopped_speed: float = .010
    stopped_yaw_rate: float = .016
    high_steer_speed: float = 0.
    high_steer_angle: float = math.radians(20)
    wheelbase: float = .494
    track: float = .364

    def __post_init__(self):
        if not all(math.isfinite(x) and x > 0 for name, x in vars(self).items() if name != "high_steer_speed"):
            raise ValueError("plant settings must be finite and positive")
        if not 0 <= self.high_steer_speed <= self.max_speed or self.high_steer_angle >= math.pi / 2:
            raise ValueError("invalid plant steering speed envelope")


def classify_twist(command, radius):
    x, y, yaw = command
    if y != 0.:
        return OBLIQUE, (x, y, 0.)
    if abs(yaw) < 1e-6:
        return ACKERMANN, (x, 0., 0.)
    if abs(x) < radius * abs(yaw):
        return SPIN, (0., 0., yaw)
    return ACKERMANN, command


class Response:
    def __init__(self, settings, managed):
        self.settings = settings
        self.managed = managed
        self.velocity = ZERO
        self.mode = ACKERMANN
        self.pending_mode = None
        self.pending_since = None

    def can_stop_this_step(self, dt):
        # Snap only where the acceleration allowance includes the whole
        # residual velocity. This also provides a finite-time reversal.
        return (math.hypot(*self.velocity[:2])
                <= min(self.settings.stopped_speed, self.settings.max_accel * dt)
                and abs(self.velocity[2]) <= min(self.settings.stopped_yaw_rate,
                                                self.settings.max_yaw_accel * dt))

    def update(self, time, dt, command, request):
        if not all(math.isfinite(x) for x in command):
            command = ZERO
        if request not in (ACKERMANN, OBLIQUE, SPIN):
            request = None
        if command != ZERO:
            chosen, target = classify_twist(command, self.settings.switch_radius)
            if not self.managed:
                request = chosen
            if chosen != self.mode:
                target = ZERO
        else:
            target = ZERO

        if request is None or request == self.mode:
            self.pending_mode = self.pending_since = None
        else:
            target = ZERO
            if not self.can_stop_this_step(dt):
                self.pending_mode = self.pending_since = None
            elif self.pending_mode != request:
                self.pending_mode, self.pending_since = request, time
            elif time - self.pending_since + 1e-9 >= self.settings.switch_delay:
                self.mode = request
                self.velocity = ZERO
                self.pending_mode = self.pending_since = None
                return self.velocity

        previous = self.velocity
        x, y, yaw = target
        if self.mode == ACKERMANN and previous[0] * x < 0:
            x = yaw = 0.
        if (x, y, yaw) == ZERO and self.can_stop_this_step(dt):
            self.velocity = ZERO
            return self.velocity
        speed = math.hypot(x, y)
        if speed > self.settings.max_speed:
            factor = self.settings.max_speed / speed
            x, y = x * factor, y * factor
        yaw = max(-self.settings.max_yaw_rate, min(self.settings.max_yaw_rate, yaw))
        if self.mode == ACKERMANN:
            bound = abs(x) / self.settings.switch_radius
            yaw = max(-bound, min(bound, yaw))
        steering_cap = self.settings.high_steer_speed
        if steering_cap > 0 and self.mode != SPIN:
            tangent = math.tan(self.settings.high_steer_angle)
            ackermann_bound = 2 * tangent / (self.settings.wheelbase + self.settings.track * tangent)

            def high_steer(twist):
                vx, vy, wz = twist
                if self.mode == ACKERMANN:
                    return abs(wz) > ackermann_bound * abs(vx)
                return abs(vy) > tangent * abs(vx)

            speed, previous_speed = math.hypot(x, y), math.hypot(*previous[:2])
            if previous_speed > steering_cap and high_steer((x, y, yaw)):
                # Brake along the current direction before increasing steering.
                factor = min(steering_cap, speed) / previous_speed
                x, y, yaw = (value * factor for value in previous)
            elif speed > steering_cap and (high_steer(previous) or high_steer((x, y, yaw))):
                factor = steering_cap / speed
                x, y, yaw = x * factor, y * factor, yaw * factor
        delta_linear = math.hypot(x - previous[0], y - previous[1])
        delta_yaw = abs(yaw - previous[2])
        t_linear = min(-math.expm1(-dt / self.settings.linear_tau),
                       self.settings.max_accel * dt / delta_linear) if delta_linear else 1.
        t_yaw = min(-math.expm1(-dt / self.settings.angular_tau),
                    self.settings.max_yaw_accel * dt / delta_yaw) if delta_yaw else 1.
        if self.mode == ACKERMANN:
            t_linear = t_yaw = min(t_linear, t_yaw)
        self.velocity = (previous[0] + t_linear * (x - previous[0]),
                         previous[1] + t_linear * (y - previous[1]),
                         previous[2] + t_yaw * (yaw - previous[2]))
        return self.velocity


@dataclass(frozen=True)
class Measurement:
    stamp: float
    pose: object
    velocity: tuple
    mode: int


def measured_twist(old, new, elapsed):
    angle = new.rot().yaw() - old.rot().yaw()
    angle = math.atan2(math.sin(angle), math.cos(angle))
    half = angle * .5
    scale = math.sin(half) / half if half else 1.
    c, s = math.cos(old.rot().yaw() + half), math.sin(old.rot().yaw() + half)
    dx = new.pos().x() - old.pos().x()
    dy = new.pos().y() - old.pos().y()
    return (c * dx + s * dy) / (elapsed * scale), (-s * dx + c * dy) / (elapsed * scale), angle / elapsed


class GazeboBody:
    """Use ECM pose feedback in both the live system and accelerated trials."""
    def __init__(self, entity, ecm, settings, managed=True):
        self.model = Model(entity)
        self.link = Link(self.model.canonical_link(ecm))
        self.response = Response(settings, managed)
        self.period = 1. / settings.frequency
        self.last = None
        self.sample_due = False

    def pre_update(self, time, step, ecm, command=ZERO, request=None):
        if self.last is not None and time < self.last.stamp - 1e-8:
            self.response = Response(self.response.settings, self.response.managed)
            self.last = None
        self.sample_due = self.last is None or time - self.last.stamp + 1e-8 >= self.period
        if not self.sample_due or self.last is None:
            return
        dt = max(step, time - self.last.stamp)
        vx, vy, yaw_rate = self.response.update(time, dt, command, request)
        if (vx, vy, yaw_rate) == ZERO:
            return
        p = self.last.pose
        yaw = p.rot().yaw()
        half = yaw_rate * dt * .5
        scale = math.sin(half) / half if half else 1.
        c, s = math.cos(yaw + half), math.sin(yaw + half)
        self.model.set_world_pose_cmd(ecm, Pose3d(
            p.pos().x() + dt * scale * (c * vx - s * vy),
            p.pos().y() + dt * scale * (s * vx + c * vy),
            p.pos().z(), 0., 0., yaw + 2 * half))

    def post_update(self, time, ecm):
        if not self.sample_due:
            return None
        pose = self.link.world_pose(ecm)
        if pose is None:
            return None
        velocity = ZERO
        if self.last is not None and time > self.last.stamp:
            velocity = measured_twist(self.last.pose, pose, time - self.last.stamp)
        self.last = Measurement(time, pose, velocity, self.response.mode)
        return self.last
