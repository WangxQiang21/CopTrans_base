"""Contract tests for continuous-mode time scaling and transition holds."""
import math
from pathlib import Path
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import Action, Pose, replay
from fwis_v3_optimizer.continuous_geometry import ControlKnot, MotionBlock, plan_from_actions, sample_block
from fwis_v3_optimizer.continuous_retiming import (
    _continuous_geometry_table, _low_speed_intervals, retime_continuous)
from fwis_v3_optimizer.retiming import (MotionLimits, steering_rest_to_rest_duration,
                                       bounded_duration, PrimitiveTimingCache, retime)


class ContinuousRetiming(unittest.TestCase):
    def setUp(self):
        self.start = Pose(2., 2., 0.)
        self.limits = MotionLimits()

    def interactive_limits(self):
        return MotionLimits(v_max=2., target_v=2.,
            translation_speed_policy="mode_switch_only", mode_switch_duration=.3)

    def test_cached_primitive_duration_matches_reference_across_modes_and_reverse(self):
        import numpy as np
        rng = np.random.default_rng(4418)
        limits = self.interactive_limits()
        cache = PrimitiveTimingCache(limits,allow_reverse=True,dt=.5)
        bank = [Action(0,d,k) for d in (-.4,.5,1.) for k in (-.8,0.,.8)]
        bank += [Action(1,d,p) for d in (-.4,.7) for p in (-.7,0.,.7)]
        bank += [Action(2,d,0.) for d in (-.4,.7)]
        for _ in range(80):
            actions = [bank[int(i)] for i in rng.integers(0,len(bank),size=12)]
            expected = retime(self.start,actions,limits,.5,allow_reverse=True,
                              build_samples=False).duration
            self.assertAlmostEqual(cache(actions),expected,places=10)
        self.assertGreater(cache.stats["primitive_hits"],cache.stats["primitive_misses"])
        self.assertAlmostEqual(cache([]),.5)

    def test_interactive_timing_is_identical_for_straight_and_maximum_curvature(self):
        limits = self.interactive_limits()
        results = []
        for curvature in (0., 1./limits.r_min_ack):
            actions = [Action(0, 12., curvature)]
            plan = plan_from_actions(self.start, replay(self.start, actions)[-1], actions)
            results.append(retime_continuous(plan, limits, dt=.02))
        self.assertAlmostEqual(results[0].duration, results[1].duration, places=8)
        self.assertAlmostEqual(max(v for _, _, v, _ in results[1].samples), 2., places=8)
        self.assertTrue(all(row["active_speed_caps"] == ["target_speed"]
                            for row in results[1].speed_constraints))
        # Demand remains visible even when the kinematic policy does not cap it.
        self.assertGreater(max(p["dynamic_peaks"]["lateral_acceleration"]
                               for p in results[1].phases), limits.lateral_a_max)

    def test_interactive_variable_control_has_no_interior_braking(self):
        actions = [Action(0, 4., 0.), Action(0, 2., -1.2),
                   Action(0, 2., 1.2), Action(0, 4., 0.)]
        plan = plan_from_actions(self.start, replay(self.start, actions)[-1], actions)
        timed = retime_continuous(plan, self.interactive_limits(), dt=.02)
        signs = [p["end_speed"]-p["start_speed"] for p in timed.phases]
        # After braking starts, no later acceleration is allowed in one block.
        first_brake = next(i for i, change in enumerate(signs) if change < -1e-7)
        self.assertTrue(all(change <= 1e-7 for change in signs[first_brake:]))
        self.assertEqual(timed.mode_switch_count, 0)
        self.assertAlmostEqual(max(v for _, _, v, _ in timed.samples), 2., places=8)

    def test_interactive_large_wheel_reconfigurations_take_exactly_point_three_seconds(self):
        actions = [Action(0, 1., .3), Action(1, 1., -1.4),
                   Action(2, .5, 0.), Action(1, 1., 1.4)]
        plan = plan_from_actions(self.start, replay(self.start, actions)[-1], actions)
        limits = self.interactive_limits()
        timed = retime_continuous(plan, limits, dt=.02)
        holds = [p for p in timed.phases if p["kind"] == "mode_switch"]
        self.assertEqual(len(holds), 3)
        self.assertTrue(all(p["duration"] == .3 for p in holds))
        self.assertAlmostEqual(timed.mode_switch_hold_seconds, .9)
        self.assertTrue(all(p["setup_overlaps_steering"] for p in holds))
        self.assertGreater(max(p["max_wheel_angle_delta_rad"] for p in holds), 1.4)
        restricted = retime_continuous(plan,
            replace(limits, translation_speed_policy="restricted", mode_switch_duration=None))
        self.assertGreater(restricted.mode_switch_hold_seconds, 3.)

    def test_interactive_short_curve_does_not_use_restricted_peak_stretch(self):
        limits = self.interactive_limits()
        results = []
        for curvature in (0., 1.2):
            actions = [Action(0, .08, curvature)]
            results.append(retime_continuous(plan_from_actions(self.start,
                replay(self.start, actions)[-1], actions), limits))
        self.assertAlmostEqual(results[0].duration, results[1].duration, places=8)

    def test_rest_to_rest_steering_meets_rate_acceleration_and_shortens_quintic_hold(self):
        import numpy as np
        rate,acc=.8,1.5
        for delta in (.05,.4,.8,1.5):
            duration=steering_rest_to_rest_duration(delta,rate,acc)
            peak=min(rate,math.sqrt(delta*acc))
            ramp=peak/acc
            times=np.linspace(0.,duration,10001)
            speeds=np.minimum(np.minimum(acc*times,acc*(duration-times)),peak)
            self.assertAlmostEqual(float(np.trapz(speeds,x=times)),delta,places=6)
            self.assertLessEqual(float(np.max(speeds)),rate+1e-9)
            accelerations=np.diff(speeds)/np.diff(times)
            self.assertLessEqual(float(np.max(np.abs(accelerations))),acc+1e-8)
            self.assertAlmostEqual(speeds[0],0.)
            self.assertAlmostEqual(speeds[-1],0.)
            self.assertLess(duration,bounded_duration(delta,rate,acc))

    def test_target2_straight_cruises_with_shared_nonzero_acceleration(self):
        limits = MotionLimits(v_max=2., target_v=2., lateral_a_max=2.,
            wheel_speed_max=3., translation_omega_max=2.5,
            translation_alpha_max=0., translation_angular_jerk_max=0.)
        actions = [Action(0, 20., 0.)]
        timed = retime_continuous(plan_from_actions(self.start,
            replay(self.start, actions)[-1], actions), limits, dt=.05)
        self.assertGreater(max(v for _, _, v, _ in timed.samples), 1.9)
        self.assertLess(timed.duration, 16.)
        self.assertTrue(any(abs(p["start_acceleration"]) > .2 for p in timed.phases))
        for left, right in zip(timed.phases, timed.phases[1:]):
            self.assertAlmostEqual(left["end_acceleration"], right["start_acceleration"], places=8)
        self.assertLessEqual(max(p["dynamic_peaks"]["jerk"] for p in timed.phases), limits.jerk_max)

    def test_report_keeps_interval_caps_and_classifies_endpoint_low_speed(self):
        limits = MotionLimits(v_max=2., target_v=2., lateral_a_max=2.,
            wheel_speed_max=3., translation_omega_max=2.5,
            translation_alpha_max=0., translation_angular_jerk_max=0.)
        actions = [Action(0, 20., 0.)]
        plan = plan_from_actions(self.start,
            replay(self.start, actions)[-1], actions)
        report = retime_continuous(plan, limits, dt=.02).report()
        self.assertTrue(report["speed_constraint_intervals"])
        interval = report["speed_constraint_intervals"][0]
        self.assertIn("target_speed", interval["active_speed_caps"])
        self.assertGreaterEqual(interval["peak_sampling_points"], 65)
        self.assertIn("scheduled_anchor_speed_mps", interval)
        low = report["low_speed_intervals"]
        self.assertTrue(any(row["classification"] == "start_acceleration"
                            for row in low), low)
        self.assertTrue(any(row["classification"] == "end_braking"
                            for row in low), low)

    def test_short_block_low_speed_nearer_goal_is_end_braking(self):
        phases = [{"kind": "translation", "profile": "linear",
                   "block_index": 0, "mode": 0, "direction": 1,
                   "t0": 0., "duration": 2., "s0": 0., "s1": .8,
                   "global_s0": 0., "speed_envelope_active_limits": []}]
        samples = [(1., Pose(.4, 2., 0.), .1, 0.),
                   (2., Pose(.8, 2., 0.), .1, 0.)]
        low = _low_speed_intervals(samples, phases, threshold=.2,
                                   minimum_duration=.5,
                                   endpoint_exclusion_m=.5)
        self.assertEqual(len(low), 1)
        self.assertEqual(low[0]["classification"], "end_braking")

    def test_constant_corner_does_not_use_spin_angular_rate_cap(self):
        limits = MotionLimits(v_max=2., target_v=2., lateral_a_max=2.,
            wheel_speed_max=3., translation_omega_max=2.5,
            translation_alpha_max=0., translation_angular_jerk_max=0.)
        actions = [Action(0, 12., .5)]
        timed = retime_continuous(plan_from_actions(self.start,
            replay(self.start, actions)[-1], actions), limits, dt=.05)
        peaks = [p["dynamic_peaks"] for p in timed.phases]
        self.assertGreater(max(p["speed"] for p in peaks), 1.85)
        self.assertGreater(max(p["angular_speed"] for p in peaks), limits.omega_max)
        self.assertLessEqual(max(p["lateral_acceleration"] for p in peaks), limits.lateral_a_max)

    def test_submillimetre_terminal_turn_does_not_slow_the_straight_route(self):
        actions = [Action(0, 5., 0.), Action(0, .0007, -1.2)]
        plan = plan_from_actions(self.start, replay(self.start, actions)[-1], actions)
        timed = retime_continuous(plan, self.limits, dt=.05)
        cruise = [abs(v) for _, pose, v, _ in timed.samples
                  if 1. < pose.x-self.start.x < 3.]
        self.assertGreater(max(cruise), .7)
        self.assertLess(timed.duration, 20.)
        self.assertFalse(any(abs(v) < 1e-8 for _, _, v, _ in timed.samples[1:-1]))
        self.assertEqual(timed.mode_switch_count, 0)

    def test_nearly_coincident_metric_station_has_no_fake_steering_acceleration(self):
        import numpy as np
        block = MotionBlock(0, 1, 2., tuple(
            ControlKnot(s, .2, 0.) for s in (0., .4+1e-12, 2.)))
        *_, first, second = _continuous_geometry_table(block, self.limits)
        self.assertLess(np.max(np.abs(first)), 1e-10)
        self.assertLess(np.max(np.abs(second)), 1e-10)

    def test_curvature_knots_do_not_create_stops_or_reconfiguration(self):
        actions = [Action(0, .7, -.3), Action(0, .7, .3)]
        plan = plan_from_actions(self.start, replay(self.start, actions)[-1], actions)
        timed = retime_continuous(plan, self.limits, dt=.01)
        motion = [p for p in timed.phases if p["kind"] == "translation"]
        self.assertGreater(len(motion), 1)
        self.assertEqual({p["block_index"] for p in motion}, {0})
        self.assertEqual(timed.mode_switch_count, 0)
        self.assertEqual(timed.same_mode_hold_seconds, 0.)
        self.assertEqual(timed.gear_change_hold_seconds, 0.)
        for left, right in zip(motion, motion[1:]):
            self.assertAlmostEqual(left["end_speed"], right["start_speed"], places=8)
            self.assertGreater(left["end_speed"], .05)
        knot_pose = min(sample_block(plan.blocks[0], self.start, .005),
                        key=lambda row: abs(row[0]-.7))[1]
        knot_sample = min(timed.samples, key=lambda row:
            (row[1].x-knot_pose.x)**2+(row[1].y-knot_pose.y)**2)
        self.assertGreater(abs(knot_sample[2]), .05)
        for metric, limit in (("speed", self.limits.v_max),
                              ("acceleration", self.limits.a_max),
                              ("jerk", self.limits.jerk_max),
                              ("wheel_speed", self.limits.wheel_speed_max),
                              ("lateral_acceleration", self.limits.lateral_a_max),
                              ("angular_speed", self.limits.omega_max),
                              ("angular_acceleration", self.limits.alpha_max),
                              ("angular_jerk", self.limits.angular_jerk_max),
                              ("steer_rate", self.limits.steer_rate_max),
                              ("steer_acceleration", self.limits.steer_accel_max)):
            self.assertLessEqual(max(p["dynamic_peaks"][metric] for p in motion),
                                 limit+1e-5, metric)

    def test_oblique_control_respects_steering_and_normal_acceleration(self):
        actions = [Action(1, .8, -.5), Action(1, .8, .6)]
        plan = plan_from_actions(self.start, replay(self.start, actions)[-1], actions)
        timed = retime_continuous(plan, self.limits, dt=.01)
        peaks = [p["dynamic_peaks"] for p in timed.phases
                 if p["kind"] == "translation"]
        self.assertLessEqual(max(p["steer_rate"] for p in peaks),
                             self.limits.steer_rate_max+1e-5)
        self.assertLessEqual(max(p["steer_acceleration"] for p in peaks),
                             self.limits.steer_accel_max+1e-5)
        self.assertLessEqual(max(p["oblique_normal_acceleration"] for p in peaks),
                             self.limits.lateral_a_max+1e-5)
        self.assertTrue(all(p["angular_speed"] == 0. and
                            p["angular_acceleration"] == 0. for p in peaks))

    def test_only_mode_change_inserts_stationary_reconfiguration(self):
        actions = [Action(0, 1., .1), Action(1, .6, .2), Action(0, .7, -.1)]
        plan = plan_from_actions(self.start, replay(self.start, actions)[-1], actions)
        timed = retime_continuous(plan, self.limits, dt=.02)
        holds = [phase for phase in timed.phases if phase["kind"] == "mode_switch"]
        self.assertEqual(len(holds), 2)
        self.assertEqual(timed.mode_switch_count, 2)
        self.assertGreater(timed.mode_switch_hold_seconds, 0.)
        self.assertEqual(timed.same_mode_hold_seconds, 0.)
        self.assertEqual(timed.gear_change_hold_seconds, 0.)
        for phase in holds:
            self.assertGreater(phase["duration"], 0.)

    def test_direction_cusp_has_zero_speed_point_without_dwell(self):
        actions = [Action(0, .8, .1), Action(0, -.4, .1)]
        plan = plan_from_actions(self.start, replay(self.start, actions)[-1], actions)
        timed = retime_continuous(plan, self.limits, dt=.01)
        phases = [p for p in timed.phases if p["kind"] == "translation"]
        self.assertEqual({p["block_index"] for p in phases}, {0, 1})
        self.assertEqual(timed.mode_switch_count, 0)
        self.assertEqual(timed.gear_change_hold_seconds, 0.)
        self.assertFalse(any(p["kind"] == "mode_switch" for p in timed.phases))
        first_block = [p for p in phases if p["block_index"] == 0]
        cusp_t = first_block[-1]["t0"]+first_block[-1]["duration"]
        cusp = min(timed.samples, key=lambda row: abs(row[0]-cusp_t))
        self.assertLess(abs(cusp[2]), 1e-9)
        self.assertEqual(cusp[0], cusp_t)

    def test_short_translation_has_finite_acceleration_and_zero_end_speed(self):
        actions = [Action(1, .08, .15)]
        plan = plan_from_actions(self.start, replay(self.start, actions)[-1], actions)
        timed = retime_continuous(plan, self.limits, dt=.005)
        self.assertGreater(timed.duration, .2)
        self.assertEqual(timed.samples[0][2], 0.)
        self.assertAlmostEqual(timed.samples[-1][2], 0.)
        self.assertGreater(max(abs(row[2]) for row in timed.samples), .01)
        self.assertLessEqual(timed.phases[0]["dynamic_peaks"]["acceleration"],
                             self.limits.a_max+1e-5)

    def test_spin_centers_stay_fixed_and_has_finite_angular_profile(self):
        actions = [Action(2, .6, 0.), Action(2, .4, 0.)]
        plan = plan_from_actions(self.start, replay(self.start, actions)[-1], actions)
        timed = retime_continuous(plan, self.limits, dt=.01)
        self.assertEqual(len([p for p in timed.phases if p["kind"] == "spin"]), 1)
        self.assertTrue(all(row[1].x == self.start.x and row[1].y == self.start.y
                            and row[2] == 0. for row in timed.samples))
        self.assertGreater(timed.spin_seconds, 0.)
        self.assertTrue(math.isfinite(timed.duration))


if __name__ == "__main__":
    unittest.main()
