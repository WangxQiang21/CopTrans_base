"""Two feedback laws sharing the reference and vehicle-state contract."""
from dataclasses import dataclass
import math

from .model import Mode, Twist, wrap
from .safety import clamp


def project_feedforward(reference, state, yaw_error):
    # The planned world velocity is preserved if the current vehicle heading
    # differs from the constant heading of an oblique reference.
    c, s = math.cos(yaw_error), math.sin(yaw_error)
    vx, vy, _ = reference.twist.values()
    return c * vx - s * vy, s * vx + c * vy


@dataclass(frozen=True)
class NonlinearGains:
    ack_x: float = 1.35
    ack_y: float = 9.0
    ack_yaw: float = 6.0
    oblique_xy: float = 1.65
    spin_yaw: float = 2.0

    def __post_init__(self):
        if not all(math.isfinite(x) and x > 0 for x in vars(self).values()):
            raise ValueError("feedback gains must be finite and positive")


class NonlinearFeedback:
    def __init__(self, gains):
        self.gains = gains

    def reset(self):
        pass

    def update(self, reference, state, dt, direction):
        ex, ey, et = state.pose.error_to(reference.pose)
        vx, _, wz = reference.twist.values()
        if reference.mode == Mode.ACKERMANN:
            # Body-frame Kanayama form. |vx| in the heading damping term
            # keeps its sign for reverse tracking.
            return Twist(vx * math.cos(et) + self.gains.ack_x * ex, 0.,
                         wz + self.gains.ack_y * vx * ey
                         + self.gains.ack_yaw * abs(vx) * math.sin(et))
        if reference.mode == Mode.OBLIQUE:
            fx, fy = project_feedforward(reference, state, et)
            return Twist(fx + self.gains.oblique_xy * ex, fy + self.gains.oblique_xy * ey, 0.)
        return Twist(wz=wz + self.gains.spin_yaw * et)

    def accept(self, applied):
        pass


@dataclass(frozen=True)
class PIDGains:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    derivative_tau: float

    def __post_init__(self):
        if (not all(math.isfinite(x) and x >= 0 for x in vars(self).values())
                or self.derivative_tau == 0):
            raise ValueError("PID gains must be finite and derivative_tau positive")


class PIDAxis:
    def __init__(self, gains, angular=False):
        self.gains = gains
        self.angular = angular
        self.integral = self.previous_integral = 0.
        self.derivative = 0.
        self.previous_error = None

    def update(self, error, dt):
        if self.previous_error is not None:
            change = error - self.previous_error
            derivative = (wrap(change) if self.angular else change) / dt
            fraction = -math.expm1(-dt / self.gains.derivative_tau)
            self.derivative += fraction * (derivative - self.derivative)
        self.previous_error = error
        self.previous_integral = self.integral
        self.integral = clamp(self.integral + self.gains.ki * error * dt,
                              self.gains.integral_limit)
        return self.gains.kp * error + self.integral + self.gains.kd * self.derivative

    def accept(self, saturation):
        # Keep integral unwind and suppress increments aggravating saturation.
        if (self.integral - self.previous_integral) * saturation > 1e-12:
            self.integral = self.previous_integral


class PIDFeedback:
    def __init__(self, gains):
        self.gains = gains
        self.reset()

    def reset(self):
        self.axes = {}
        self.key = None
        self.contributions = ()
        self.raw = Twist()

    def update(self, reference, state, dt, direction):
        key = reference.mode, direction if reference.mode == Mode.ACKERMANN else 0.
        if key != self.key:
            self.reset()
            self.key = key
        ex, ey, et = state.pose.error_to(reference.pose)
        vx, _, wz = reference.twist.values()
        if reference.mode == Mode.ACKERMANN:
            terms = (("ack_x", ex, 0), ("ack_y", direction * ey, 2), ("ack_yaw", et, 2))
            output = [vx * math.cos(et), 0., wz]
        elif reference.mode == Mode.OBLIQUE:
            terms = (("oblique", ex, 0), ("oblique", ey, 1))
            output = [*project_feedforward(reference, state, et), 0.]
        else:
            terms = (("spin", et, 2),)
            output = [0., 0., wz]
        contributions = []
        for preset, error, channel in terms:
            axis_key = preset, channel
            if axis_key not in self.axes:
                self.axes[axis_key] = PIDAxis(self.gains[preset], preset in ("ack_yaw", "spin"))
            axis = self.axes[axis_key]
            output[channel] += axis.update(error, dt)
            contributions.append((axis, channel))
        self.contributions = tuple(contributions)
        self.raw = Twist(*output)
        return self.raw

    def accept(self, applied):
        for axis, channel in self.contributions:
            axis.accept(self.raw.values()[channel] - applied.values()[channel])
