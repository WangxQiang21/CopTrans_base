"""Follow a checked trajectory and execute its mode changes at rest."""
from dataclasses import dataclass
import math

from .model import Mode, ZERO
from .safety import SafetyFilter
from .trajectory import preflight


@dataclass(frozen=True)
class Timing:
    frequency: float = 25.
    state_timeout: float = .20
    mode_timeout: float = .20
    plan_admission_timeout: float = .70
    clock_tolerance: float = .04
    approach_time: float = .35
    stable_time: float = .08
    switch_timeout: float = .30
    settle_time: float = .08
    stop_speed: float = .012
    stop_yaw_rate: float = .020
    final_stop_timeout: float = 1.5

    def __post_init__(self):
        if not all(math.isfinite(x) and x > 0 for x in vars(self).values()):
            raise ValueError("timing settings must be finite and positive")

    @property
    def period(self):
        return 1. / self.frequency

    @property
    def minimum_hold(self):
        return self.stable_time + self.switch_timeout + self.settle_time + 2 * self.period


@dataclass(frozen=True)
class Output:
    phase: str
    command: object = ZERO
    raw: object = ZERO
    reference: object = None
    requested_mode: object = None
    reasons: tuple = ()


class Execution:
    def __init__(self, controller, limits, preflight_settings, timing):
        self.controller = controller
        self.filter = SafetyFilter(limits)
        self.preflight_settings = preflight_settings
        self.timing = timing
        self.sampler = None
        self.phase = "IDLE"
        self.result = None

    def admit(self, trajectory, initial_mode, now, *, replay_start=None, future_window=0.,
              allow_axial_oblique=False):
        self.controller.reset()
        self.filter.reset()
        self.sampler = None
        self.result = preflight(trajectory, self.filter.limits, self.preflight_settings,
                                self.timing.minimum_hold, initial_mode,
                                allow_axial_oblique=allow_axial_oblique)
        self.start = trajectory.stamp_ns * 1e-9 if replay_start is None else replay_start
        if self.result.sampler is None:
            self.phase = "REJECTED"
            return self.result
        age = now - self.start
        if not -max(self.timing.clock_tolerance, future_window) <= age <= self.timing.plan_admission_timeout:
            self.phase = "STALE_TRAJECTORY"
            return self.result
        if age >= trajectory.duration:
            self.phase = "STALE_TRAJECTORY"
            return self.result
        self.sampler = self.result.sampler
        self.switches = self.result.switches
        self.switch_index = 0
        self.expected_mode = initial_mode
        self.last_tick = None
        self.still_since = None
        self.request_at = None
        self.settle_from = None
        self.direction = 0.
        self.phase = "TRACKING"
        return self.result

    def abort(self, reason, reference=None):
        self.sampler = None
        self.filter.reset()
        self.controller.reset()
        self.phase = "STOPPED"
        return Output(self.phase, reference=reference, reasons=(reason,))

    def feedback_is_fresh(self, observation, now):
        return (observation is not None
                and all(math.isfinite(x) for x in (observation.pose.x, observation.pose.y,
                        observation.pose.yaw, *observation.twist.values(), observation.stamp,
                        observation.mode_stamp))
                and observation.mode in tuple(Mode)
                and -self.timing.clock_tolerance <= now - observation.stamp <= self.timing.state_timeout
                and -self.timing.clock_tolerance <= now - observation.mode_stamp <= self.timing.mode_timeout)

    def _stopped(self, observation):
        return (observation.twist.speed <= self.timing.stop_speed
                and abs(observation.twist.wz) <= self.timing.stop_yaw_rate
                and self.filter.previous == ZERO)

    def _brake(self, phase, reference, mode, dt, requested_mode=None):
        self.phase = phase
        result = self.filter.apply(ZERO, mode, dt)
        return Output(phase, result.command, ZERO, reference, requested_mode, result.reasons)

    def tick(self, now, observation):
        if self.sampler is None:
            return Output(self.phase)
        if not math.isfinite(now) or not self.feedback_is_fresh(observation, now):
            return self.abort("missing_stale_or_invalid_feedback")
        dt = self.timing.period if self.last_tick is None else now - self.last_tick
        if not 0 < dt <= self.timing.state_timeout:
            return self.abort("control_clock_jump_or_overrun")
        self.last_tick = now
        elapsed = now - self.start
        if elapsed < 0:
            return self._brake("WAIT_START", None, self.expected_mode, dt)

        reference = self.sampler.sample(elapsed)
        event = (self.switches[self.switch_index]
                 if self.switch_index < len(self.switches) else None)
        if event is not None and elapsed + 1e-8 >= event.time:
            transition = self._transition(event, now, elapsed, observation, reference, dt)
            if transition is not None:
                return transition
            event = None

        if observation.mode != self.expected_mode:
            return self.abort("unexpected_feedback_mode", reference)
        if elapsed + 1e-8 >= self.sampler.trajectory.duration:
            if self._stopped(observation):
                self.phase = "COMPLETE"
                self.sampler = None
                return Output(self.phase, reference=reference)
            if elapsed > self.sampler.trajectory.duration + self.timing.final_stop_timeout:
                return self.abort("final_stop_timeout", reference)
            return self._brake("FINAL_STOP", reference, self.expected_mode, dt)

        if reference.mode != self.expected_mode:
            return self.abort("unconfirmed_planned_mode", reference)
        self.phase = ("APPROACH_SWITCH" if event and elapsed >= event.time - self.timing.approach_time
                      else "TRACKING")
        if reference.twist == ZERO:
            self.controller.reset()
            return self._brake(self.phase, reference, self.expected_mode, dt)

        if reference.mode == Mode.ACKERMANN and abs(reference.twist.vx) > 1e-9:
            self.direction = math.copysign(1., reference.twist.vx)
        raw = self.controller.update(reference, observation, dt, self.direction)
        result = self.filter.apply(raw, reference.mode, dt, self.direction, reference.twist.vy)
        if "nonfinite_or_invalid_interval" in result.reasons:
            return self.abort("invalid_controller_output", reference)
        self.controller.accept(result.command)
        return Output(self.phase, result.command, raw, reference, reasons=result.reasons)

    def _transition(self, event, now, elapsed, observation, reference, dt):
        if self.settle_from is not None:
            if observation.mode != event.mode or not self._stopped(observation):
                return self.abort("mode_or_stop_lost_during_settle", reference)
            if now - self.settle_from + 1e-8 >= self.timing.settle_time:
                self.switch_index += 1
                self.expected_mode = event.mode
                self.still_since = self.request_at = self.settle_from = None
                self.direction = 0.
                self.controller.reset()
                return None
            if elapsed + 1e-8 >= event.resume:
                return self.abort("planned_hold_exhausted", reference)
            return self._brake("SETTLE", reference, event.mode, dt)

        if elapsed + 1e-8 >= event.resume:
            return self.abort("planned_hold_exhausted", reference)
        if self.request_at is not None:
            if observation.mode == event.mode and observation.mode_stamp + 1e-8 >= self.request_at:
                self.settle_from = now
                return self._brake("SETTLE", reference, event.mode, dt)
            if now - self.request_at > self.timing.switch_timeout:
                return self.abort("mode_confirmation_timeout", reference)
            if observation.mode != self.expected_mode:
                return self.abort("unexpected_feedback_mode", reference)
            return self._brake("WAIT_MODE_CONFIRMATION", reference, self.expected_mode, dt, event.mode)

        if observation.mode != self.expected_mode:
            return self.abort("unexpected_feedback_mode", reference)
        if not self._stopped(observation):
            self.still_since = None
        elif self.still_since is None:
            self.still_since = observation.stamp
        elif observation.stamp - self.still_since + 1e-8 >= self.timing.stable_time:
            self.request_at = now
            return self._brake("REQUEST_MODE_SWITCH", reference, self.expected_mode, dt, event.mode)
        return self._brake("STOPPING", reference, self.expected_mode, dt)
