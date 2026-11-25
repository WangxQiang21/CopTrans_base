from bisect import bisect_left
from itertools import accumulate
import math
import random

import numpy as np
import pytest

from fwis_backends.path_reachability import (
    ProgressBounds, VehicleDerivativeBounds, parameterize, progress_bounds,
)
from fwis_backends.single_continuous_mode_path.ackermann_reachability import (
    constant_ackermann_jerk_profile, pure_ackermann_profile,
)
from fwis_backends.single_continuous_mode_path.continuous_geometry import ControlKnot, MotionBlock
from fwis_backends.single_continuous_mode_path.core import Action, Pose
from fwis_backends.single_continuous_mode_path.retiming import MotionLimits, retime


def test_two_metre_trapezoid():
    profile = parameterize([0., 2.], [ProgressBounds(1., -1., 1.)])
    assert profile.progress == (0., .5, 1.5, 2.)
    assert profile.speeds == (0., 1., 1., 0.)
    assert profile.duration_s == pytest.approx(3.)
    assert profile.state_at(1.5) == pytest.approx((1., 1., 0.))
    assert profile.state_at(profile.duration_s) == pytest.approx((2., 0., 0.))


def test_backward_pass_respects_a_slow_future_interval():
    profile = parameterize([0., 10., 10.1, 15.], [
        ProgressBounds(3., -1., 1.), ProgressBounds(.25, -.3, .3), ProgressBounds(2., -.6, .7)])
    i, j = [profile.progress.index(x) for x in (10., 10.1)]
    assert profile.speeds[i] <= .25 and profile.speeds[j] <= .25
    assert profile.speeds[0] == profile.speeds[-1] == 0.
    assert all(b > a for a, b in zip(profile.times_s[:-1], profile.times_s[1:]))
    with pytest.raises(ValueError, match="end speed"):
        parameterize([0., .01], [ProgressBounds(2., -1., 1.)], end_speed=1.)



def test_nearly_degenerate_switch_does_not_create_ghost_braking():
    first = ProgressBounds(1.1219121910514127, -0.048236161737882484, 0.7737872563582363)
    second = ProgressBounds(1.417415371536549, -1.1519785210916746, 1.6261569000574247)
    boundary = .2831005899670028
    profile = parameterize([0., boundary, 4.731505339187552], [first, second])
    assert profile.progress[1] == boundary
    assert profile.accelerations[0] == pytest.approx(first.max_acceleration)


def test_randomized_intervals_preserve_profile_envelopes():
    rng = random.Random(332245)
    for _ in range(3000):
        count = rng.randrange(1, 9)
        knots = list(accumulate([0.] + [rng.uniform(.001, 5.) for _ in range(count)]))
        bounds = [ProgressBounds(rng.uniform(.001, 1.5), -rng.uniform(.01, 2.), rng.uniform(.01, 2.))
                  for _ in range(count)]
        profile = parameterize(knots, bounds)
        assert profile.speeds[0] == profile.speeds[-1] == 0.
        for i, acceleration in enumerate(profile.accelerations):
            midpoint = .5 * (profile.progress[i] + profile.progress[i + 1])
            envelope = bounds[bisect_left(knots, midpoint) - 1]
            assert envelope.min_acceleration - 1e-9 <= acceleration <= envelope.max_acceleration + 1e-9
            assert max(profile.speeds[i:i + 2]) <= envelope.max_speed + 1e-9
            assert profile.times_s[i] < profile.times_s[i + 1]

def test_formation_intersects_every_vehicle_derivative_envelope():
    bounds = progress_bounds([
        VehicleDerivativeBounds(1., 0., .3, 0., .6, .5, .5, .4),
        VehicleDerivativeBounds(2., 0., .3, 3., .6, .5, .5, .4),
    ])
    assert bounds.max_speed == pytest.approx(math.sqrt(.4 / 6.))
    assert bounds.max_acceleration == .25
    assert bounds.min_acceleration == -.25
    with pytest.raises(ValueError, match="progress bounds"):
        progress_bounds([])


@pytest.mark.parametrize("blocked", [True, False])
def test_variable_curvature_is_bounded_using_interval_extrema(blocked):
    knots = (ControlKnot(0., .2, 1.8 if blocked else .18),
             ControlKnot(.6, .2, -1.8 if blocked else -.18),
             ControlKnot(2., -.4, 0.))
    block = MotionBlock(0, 1, 2., knots)
    limits = MotionLimits(v_max=.45, r_min_ack=2.5 if blocked else .9,
                         omega_max=.24, alpha_max=.32)
    if blocked:
        with pytest.raises(ValueError, match="minimum-radius"):
            pure_ackermann_profile(block, limits)
        return
    profile = pure_ackermann_profile(block, limits)
    for t in np.linspace(0., profile.duration_s, 1001):
        station, speed, acceleration = profile.state_at(t)
        kappa, d_kappa, _ = block.control(station)
        assert abs(kappa) <= 1 / limits.r_min_ack
        assert abs(kappa * speed) <= limits.omega_max + 1e-10
        assert abs(kappa * acceleration + d_kappa * speed ** 2) <= limits.alpha_max + 1e-10
    assert profile.speeds[0] == profile.speeds[-1] == 0.


@pytest.mark.parametrize("length,kappa", [(.02, .0), (.08, .5), (1., 1.), (3., -.8), (-2., .3)])
def test_constant_ackermann_jerk_bounded_and_faster_than_quintic_motion(length, kappa):
    limits = MotionLimits(v_max=.16, a_max=.22, jerk_max=.8, omega_max=.40,
                          alpha_max=.45, angular_jerk_max=1., r_min_ack=.90)
    action = Action(0, length, kappa)
    profile = constant_ackermann_jerk_profile(action, limits)
    legacy = retime(Pose(2., 2., 0.), [action], limits, allow_reverse=True)
    assert profile.state_at(0.)[:3] == pytest.approx((0., 0., 0.))
    assert profile.state_at(profile.duration_s) == pytest.approx((abs(length), 0., 0., 0.))
    assert profile.duration_s < legacy.moving_seconds
    assert all(after > before for before, after in zip(profile.times_s[:-1], profile.times_s[1:]))
    for t in np.linspace(0., profile.duration_s, 1001):
        station, speed, acceleration, jerk = profile.state_at(t)
        assert -1e-10 <= station <= abs(length) + 1e-10
        assert -1e-10 <= speed <= limits.v_max + 1e-10
        assert abs(acceleration) <= limits.a_max + 1e-10
        assert abs(jerk) <= limits.jerk_max + 1e-10
        assert abs(kappa * speed) <= limits.omega_max + 1e-10
        assert abs(kappa * acceleration) <= limits.alpha_max + 1e-10
        assert abs(kappa * jerk) <= limits.angular_jerk_max + 1e-10


@pytest.mark.parametrize("length,curvature", [(.02, 0.), (.08, .5), (1., 1.), (3., -.8), (-2., .3)])
def test_analytic_ackermann_passes_independent_wire_geometry_and_timing_audit(length, curvature):
    from fwis_backends.single_continuous_mode_path.continuous_geometry import plan_from_actions
    from fwis_backends.single_continuous_mode_path.continuous_retiming import retime_continuous
    from fwis_backends.single_continuous_mode_path.core import Grid, advance
    from fwis_backends.single_continuous_mode_path.quality import TrajectoryEvaluator

    limits = MotionLimits(v_max=.16, a_max=.22, jerk_max=.8, omega_max=.40,
                          alpha_max=.45, angular_jerk_max=1., r_min_ack=.90)
    start = Pose(3., 3., 0.)
    action = Action(0, length, curvature)
    goal = advance(start, action)
    geometry = plan_from_actions(start, goal, (action,))
    grid = Grid(np.zeros((150, 150), dtype=bool), .05, 0., 0.)
    evaluator = TrajectoryEvaluator(grid, start, goal, limits,
                                    xy_tol=.02, yaw_tol=math.radians(1), allow_reverse=True)
    legacy = retime_continuous(geometry, limits)
    candidate = retime_continuous(geometry, limits, analytic_ackermann=True)
    assert all(p["timing_source"] == "constant_ackermann_s_curve_v1" for p in candidate.phases)
    audit = evaluator.evaluate_continuous_geometry(geometry, candidate)
    assert audit["status"] == "FEASIBLE", audit
    assert candidate.duration < .95 * legacy.duration
    assert all(b[0] - a[0] >= .0005 for a, b in zip(candidate.samples, candidate.samples[1:]))
    corrupted = candidate.phases[0]
    corrupted["coefficients"][3] *= 1.2
    assert evaluator.evaluate_continuous_geometry(geometry, candidate)["status"] != "FEASIBLE"
