from dataclasses import replace
import math

import pytest

from fwis_control.config import load
from fwis_control.controllers import PIDAxis, PIDGains
from fwis_control.model import Mode, Observation, Pose, Twist, ZERO
from fwis_control.safety import Limits, SafetyFilter
from fwis_control.trajectory import preflight
from scenarios import A, O, S, Leg, sample_case, suite

from pathlib import Path

CONFIG = load(Path(__file__).resolve().parent.parent / "config/ranger_tracking.yaml")


def check(case):
    return preflight(case.trajectory, CONFIG.limits, CONFIG.preflight, CONFIG.timing.minimum_hold)


def test_preflight_accepts_only_feasible_synthetic_cases():
    for case in suite():
        assert (check(case).sampler is not None) == case.expected_pass, case.name


def test_preflight_rejects_invalid_header_and_timestamps():
    case = sample_case("straight", [Leg(A, .4, .15)])
    t = case.trajectory
    assert check(replace(case, trajectory=replace(t, frame="odom"))).result == "FAIL"
    duplicate = t.points[:1] + (replace(t.points[1], time_ns=0),) + t.points[2:]
    assert check(replace(case, trajectory=replace(t, points=duplicate))).result == "FAIL"


def test_coordinated_admission_holds_the_common_future_epoch():
    trajectory = sample_case("scheduled", [Leg(A, .4, .15)]).trajectory
    start = trajectory.stamp_ns * 1e-9
    early = start - .5
    engine = CONFIG.execution("nonlinear")
    engine.admit(trajectory, A, early)
    assert engine.phase == "STALE_TRAJECTORY"
    engine.admit(trajectory, A, early, future_window=1.)
    before = Observation(trajectory.points[0].pose, ZERO, start - .02, A, start - .02)
    output = engine.tick(start - .02, before)
    assert output.phase == "WAIT_START" and output.command == ZERO
    after = replace(before, stamp=start + .02, mode_stamp=start + .02)
    assert engine.tick(start + .02, after).phase == "TRACKING"


def test_preflight_rejects_unphysical_mode_geometry():
    for mode, error_code in ((O, "oblique_yaw_change"), (S, "spin_translation")):
        case = sample_case("bad", [Leg(mode, .6, .15, parameter=.6)])
        t = case.trajectory
        points = list(t.points)
        index = len(points) // 2
        p = points[index].pose
        p = replace(p, yaw=p.yaw+.02) if mode == O else replace(p, x=p.x+.02)
        points[index] = replace(points[index], pose=p)
        report = check(replace(case, trajectory=replace(t, points=tuple(points))))
        assert report.result == "FAIL"
        assert error_code in {issue.code for issue in report.issues}


def test_coordinated_oblique_admits_longitudinal_body_reference():
    case = sample_case("axial_oblique", [Leg(O, .6, .15)])
    assert "oblique_twist_encodes_ackermann" in {issue.code for issue in check(case).issues}
    engine = CONFIG.execution("nonlinear")
    start = case.trajectory.stamp_ns * 1e-9
    admitted = engine.admit(case.trajectory, A, start, allow_axial_oblique=True)
    assert admitted.result == "PASS"
    assert [switch.mode for switch in admitted.switches] == [O]


def test_preflight_wrap_and_sampling_use_reference_time():
    case = sample_case("wrapped_spin", [Leg(S, 4*math.pi, .43)], yaw=3.1)
    report = check(case)
    assert report.result == "PASS"
    previous = None
    for interval in report.sampler.intervals:
        if not interval.stationary:
            ref = report.sampler.sample(interval.middle)
            assert 0. < ref.twist.wz <= CONFIG.limits.max_yaw_rate
            assert ref.pose.x == ref.pose.y == 0.
            if previous is not None:
                assert abs(previous.pose.error_to(ref.pose)[2]) < .04
            previous = ref
    assert report.sampler.sample(case.trajectory.duration + 10).twist == ZERO


def test_preflight_holds_new_mode_until_switch_window():
    case = sample_case("change", [Leg(A, .5, .16), Leg(O, .5, .16, parameter=.5)])
    report = check(case)
    event = report.switches[0]
    assert report.sampler.sample(event.time - .001).mode == A
    assert report.sampler.sample(event.time).mode == O
    assert report.sampler.sample(event.resume - .001).twist == ZERO


def test_safety_preserves_radius_and_rates_through_reversal():
    limits = Limits()
    saf = SafetyFilter(limits)
    previous = ZERO
    for raw in [Twist(.17, .1, .38)] * 15 + [Twist(-.12, 0., -.4)] * 28 + [ZERO] * 15:
        result = saf.apply(raw, Mode.ACKERMANN, .04)
        cmd = result.command
        assert abs(cmd.wz) * limits.guard_radius <= abs(cmd.vx) + 1e-12
        assert cmd.vy == 0.
        assert math.hypot(cmd.vx-previous.vx, cmd.vy-previous.vy) <= .04*limits.max_accel+1e-12
        assert abs(cmd.wz-previous.wz) <= .04*limits.max_yaw_accel+1e-12
        assert cmd.vx * previous.vx >= 0.
        previous = cmd
    assert previous == ZERO


def test_safety_preserves_parallel_encoding_and_spin_constraints():
    saf = SafetyFilter(Limits())
    for raw in [Twist(.5, 0., .9)] * 18 + [Twist(.1, -.1, 0.)] * 35:
        cmd = saf.apply(raw, O, .04, oblique_direction=1.).command
        assert cmd.wz == 0.
        assert cmd.vy or cmd == ZERO
        assert cmd.speed <= saf.limits.max_speed + 1e-12
    while saf.previous != ZERO:
        saf.apply(ZERO, O, .04)
    cmd = saf.apply(Twist(.1, .1, .3), S, .04).command
    assert cmd.vx == cmd.vy == 0. and cmd.wz > 0.
    assert saf.apply(Twist(wz=math.nan), S, .04).command == ZERO


def test_pid_integrator_antiwindup_and_wrapped_derivative():
    controller = PIDAxis(PIDGains(1., .5, .1, .15, .08), angular=True)
    controller.update(math.pi-.005, .04)
    controller.accept(10.)
    assert controller.integral == 0.
    controller.update(-math.pi+.005, .04)
    assert 0 < controller.derivative < .25
    for _ in range(100):
        controller.update(-.3, .04)
    assert controller.integral == pytest.approx(-.15)


@pytest.mark.parametrize("algorithm", ["nonlinear", "pid"])
def test_execution_failsafe_for_stale_feedback_and_plan(algorithm):
    engine = CONFIG.execution(algorithm)
    t = sample_case("straight", [Leg(A, 1., .16)]).trajectory
    now = t.stamp_ns * 1e-9
    assert engine.admit(t, A, now + 1.).result == "PASS"
    assert engine.phase == "STALE_TRAJECTORY"
    engine.admit(t, A, now)
    state = Observation(t.points[0].pose, ZERO, now - .4, A, now)
    outcome = engine.tick(now, state)
    assert outcome.command == ZERO and "missing_stale_or_invalid_feedback" in outcome.reasons


def test_execution_waits_for_measured_stop_and_confirmation():
    engine = CONFIG.execution("nonlinear")
    t = sample_case("spin", [Leg(S, 1., .35)]).trajectory
    start = t.stamp_ns * 1e-9
    engine.admit(t, A, start)
    requested = []
    rows = []
    for step in range(15):
        now = start + step * CONFIG.timing.period
        twist = Twist(.08, 0., 0.) if step < 4 else ZERO
        state = Observation(Pose(0., 0., 0.), twist, now, A, now)
        out = engine.tick(now, state)
        rows.append(out)
        if out.requested_mode is not None:
            requested.append(step)
        assert out.command == ZERO
    assert requested and requested[0] >= 6
    assert rows[-1].phase == "STOPPED"
    assert rows[-1].reasons == ("mode_confirmation_timeout",)


def test_frozen_odometry_cannot_confirm_continuous_stop():
    engine = CONFIG.execution("pid")
    t = sample_case("spin", [Leg(S, 1., .35)]).trajectory
    start = t.stamp_ns * 1e-9
    engine.admit(t, A, start)
    for i in range(5):
        now = start + i*.04
        out = engine.tick(now, Observation(Pose(0., 0., 0.), ZERO, start, A, now))
        assert out.phase == "STOPPING" and out.requested_mode is None
