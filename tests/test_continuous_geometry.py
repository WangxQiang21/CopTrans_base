"""Behavior checks for the versioned continuous arc-length geometry kernel."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import Action, Pose, advance, replay
from fwis_v3_optimizer.continuous_geometry import (
    ContinuousPlan, blocks_from_actions, fit_continuous_endpoint,
    plan_from_actions, replay_continuous, sample_block, _rk4_step,
    smooth_control_candidate, fit_continuous_suffix, fit_mode_boundaries)


class ContinuousGeometry(unittest.TestCase):
    def test_feasible_projection_stop_keeps_existing_strict_terminal_accuracy(self):
        start = Pose(2.,2.,0.)
        actions = [Action(0,1.,.2),Action(0,1.,-.2)]
        endpoint = replay(start,actions)[-1]
        goal = Pose(endpoint.x+.07,endpoint.y-.05,endpoint.th+.03)
        plan = plan_from_actions(start,goal,actions)
        fitted,report = fit_continuous_endpoint(plan,max_iterations=80,
            terminal_xy_tol=1e-7,terminal_yaw_tol=1e-7,stop_on_feasible=True)
        self.assertTrue(report["success"],report)
        self.assertTrue(report["stopped_on_feasible"],report)
        actual = replay_continuous(fitted,.025)[-1][1]
        self.assertLess(math.hypot(actual.x-goal.x,actual.y-goal.y),2e-5)
        self.assertLess(abs(actual.th-goal.th),2e-5)

    def test_oblique_endpoint_fit_omits_invariant_yaw_constraint(self):
        start = Pose(2., 2., .7)
        plan = plan_from_actions(start, Pose(3., 3., .7),
                                 [Action(1, 1., -.3), Action(1, .6, .3)])
        fitted, report = fit_continuous_endpoint(plan)
        self.assertTrue(report["success"], report)
        self.assertEqual(report["endpoint_constraint_components"], ["x", "y"])
        end = replay_continuous(fitted, .005)[-1][1]
        self.assertLess(math.hypot(end.x-3., end.y-3.), 2e-5)
        self.assertEqual(end.th, start.th)
        self.assertEqual(len(fitted.blocks), 1)

    def test_oblique_endpoint_fit_rejects_unreachable_yaw(self):
        start = Pose(2., 2., .7)
        plan = plan_from_actions(start, Pose(3., 3., .9), [Action(1, 1., 0.)])
        fitted, report = fit_continuous_endpoint(plan)
        self.assertIs(fitted, plan)
        self.assertFalse(report["success"])
        self.assertEqual(report["status"], "invariant_terminal_mismatch")

    def test_spin_endpoint_fit_only_changes_yaw_at_fixed_position(self):
        start = Pose(2., 3., .7)
        plan = plan_from_actions(start, Pose(2., 3., 1.3), [Action(2, .3, 0.)])
        fitted, report = fit_continuous_endpoint(plan)
        self.assertTrue(report["success"], report)
        self.assertEqual(report["endpoint_constraint_components"], ["yaw"])
        end = replay_continuous(fitted, .005)[-1][1]
        self.assertEqual((end.x, end.y), (2., 3.))
        self.assertLess(abs(end.th-1.3), 2e-5)
        invalid = ContinuousPlan(start, plan.blocks, Pose(2.1, 3., 1.3))
        _, report = fit_continuous_endpoint(invalid)
        self.assertEqual(report["status"], "invariant_terminal_mismatch")

    def test_mode_boundary_projection_preserves_mode_events_and_true_goal(self):
        start=Pose(2.,2.,.2)
        actions=[Action(0,1.,.4),Action(0,1.,0.),Action(2,-.3,0.),Action(1,.6,.5)]
        coarse=replay(start,actions)
        goal=Pose(coarse[-1].x+.02,coarse[-1].y-.01,coarse[-1].th)
        seed=smooth_control_candidate(plan_from_actions(start,goal,actions),.2,.2)
        fitted,report=fit_mode_boundaries(seed,actions)
        self.assertTrue(report["success"])
        self.assertEqual([b.mode for b in fitted.blocks],[0,2,1])
        pose=sample_block(fitted.blocks[0],start,.005)[-1][1]
        self.assertLess(math.hypot(pose.x-coarse[2].x,pose.y-coarse[2].y),2e-5)
        end=replay_continuous(fitted,.005)[-1][1]
        self.assertLess(math.hypot(end.x-goal.x,end.y-goal.y),2e-5)
        self.assertLess(abs(end.th-goal.th),2e-5)
        self.assertEqual(len(fitted.blocks[-1].knots),2)
        self.assertEqual(fitted.blocks[-1].knots[0].value,fitted.blocks[-1].knots[1].value)

    def test_suffix_projection_preserves_prefix_pose_and_control_without_stop_blocks(self):
        import numpy as np
        start = Pose(2.,2.,.1)
        actions = [Action(0,6.,0.),Action(1,1.,.4)]
        coarse_goal = replay(start,actions)[-1]
        goal = Pose(coarse_goal.x,coarse_goal.y,coarse_goal.th-.1)
        seed = smooth_control_candidate(plan_from_actions(start,goal,actions),.4,.4)
        fitted,report = fit_continuous_suffix(seed,2.,
            terminal_xy_tol=1e-7,terminal_yaw_tol=1e-7,max_iterations=80)
        self.assertTrue(report["success"],report)
        self.assertEqual([(b.mode,b.direction) for b in fitted.blocks],
                         [(b.mode,b.direction) for b in seed.blocks])
        prefix = report["fixed_prefix_length_m"]
        stations = np.linspace(0.,prefix,101)
        original,changed = seed.blocks[0],fitted.blocks[0]
        self.assertEqual(original.length,changed.length)
        # C1 is the control contract. The suffix may change the right-hand
        # second derivative at the boundary while value/slope stay identical.
        for old,new in zip(original.control_many(stations)[:2],changed.control_many(stations)[:2]):
            np.testing.assert_allclose(old,new,atol=1e-10)
        np.testing.assert_allclose(original.control_integral(stations),
                                   changed.control_integral(stations),atol=1e-10)
        # The added knots are control support, not additional motion blocks.
        self.assertGreater(len(changed.knots),len(original.knots))
        end = replay_continuous(fitted,.005)[-1][1]
        self.assertLess(math.hypot(end.x-goal.x,end.y-goal.y),2e-5)
        self.assertLess(abs(end.th-goal.th),2e-5)

    def test_suffix_projection_validates_window_and_deadline(self):
        import time
        start = Pose(2.,2.,0.)
        plan = plan_from_actions(start,Pose(5.,2.,.1),[Action(0,3.,0.)])
        with self.assertRaisesRegex(ValueError,"invalid_suffix_window"):
            fit_continuous_suffix(plan,0.)
        with self.assertRaisesRegex(ValueError,"budget"):
            fit_continuous_suffix(plan,2.,deadline=time.perf_counter()-1.)

    def test_smoothed_endpoint_fit_preserves_curvature_bounds_between_knots(self):
        import numpy as np
        start = Pose(2., 2., 0.)
        kmax = 1./.810
        actions = [Action(0, 1.6, kmax), Action(0, 1.3, 0.), Action(0, 1.5, -kmax)]
        goal = replay(start, actions)[-1]
        seed = smooth_control_candidate(plan_from_actions(start, goal, actions), .18)
        fitted, meta = fit_continuous_endpoint(seed, max_iterations=80)
        self.assertTrue(meta["success"], meta)
        self.assertEqual(fitted.blocks[0].slope_policy, "monotone")
        block = fitted.blocks[0]
        samples = block.control_many(np.linspace(0., block.length, 2001))[0]
        self.assertLessEqual(float(np.max(np.abs(samples))), kmax+1e-9)

    def test_smoothing_weights_a_tiny_terminal_turn_by_its_physical_length(self):
        start = Pose(2., 2., 0.)
        actions = [Action(0, 5., 0.), Action(0, .0007, -1.2)]
        raw = plan_from_actions(start, replay(start, actions)[-1], actions)
        smooth = smooth_control_candidate(raw, .18)
        self.assertEqual([(b.mode, b.direction) for b in raw.blocks],
                         [(b.mode, b.direction) for b in smooth.blocks])
        self.assertLess(max(abs(k.value) for k in smooth.blocks[0].knots), .01)
        end = replay_continuous(smooth, .005)[-1][1]
        self.assertLess(math.hypot(end.x-raw.goal.x, end.y-raw.goal.y), .005)
        self.assertLess(abs(end.th-raw.goal.th), .002)

    def test_endpoint_fitter_obeys_its_wall_clock_deadline(self):
        import time
        start = Pose(2., 2., 0.)
        actions = [Action(0, .7, -.3), Action(0, .7, .3)]
        seed = plan_from_actions(start, replay(start, actions)[-1], actions)
        with self.assertRaisesRegex(ValueError, "continuous_endpoint_fit_budget"):
            fit_continuous_endpoint(seed, terminal_xy_tol=1e-8,
                                    terminal_yaw_tol=1e-8, deadline=time.perf_counter()-1.)

    def test_batched_integration_matches_independent_small_step_rk4(self):
        import numpy as np
        start = Pose(2., 3., .4)
        for mode in (0, 1):
            for direction in (-1, 1):
                actions = [Action(mode, direction*.4, -.3),
                           Action(mode, direction*.7, .5)]
                block = blocks_from_actions(actions)[0]
                state = np.asarray([start.x, start.y, start.th])
                stations = sorted(set(np.linspace(0., block.length, 1101)).union(
                    k.s for k in block.knots))
                for left, right in zip(stations, stations[1:]):
                    state = _rk4_step(block, state, left, right-left)
                end = sample_block(block, start, .025)[-1][1]
                self.assertLess(np.linalg.norm(state-[end.x, end.y, end.th]), 1e-8)

    def test_constant_ackermann_matches_legacy_exact_arc(self):
        start = Pose(2., 3., .4)
        actions = [Action(0, 2.3, .45)]
        block = blocks_from_actions(actions)[0]
        plan = plan_from_actions(start, replay(start, actions)[-1], actions)
        end = replay_continuous(plan, .01)[-1][1]
        expected = advance(start, actions[0])
        self.assertAlmostEqual(end.x, expected.x, places=9)
        self.assertAlmostEqual(end.y, expected.y, places=9)
        self.assertAlmostEqual(end.th, expected.th, places=9)
        self.assertEqual(block.mode, 0)
        self.assertEqual(block.knots[0].value, block.knots[-1].value)

    def test_control_is_continuous_at_internal_knot(self):
        actions = [Action(0, .8, -.35), Action(0, 1.1, .4), Action(0, .6, .1)]
        block = blocks_from_actions(actions)[0]
        for left, right in zip(block.knots, block.knots[1:]):
            value_l, slope_l, _ = block.control(left.s)
            value_r, slope_r, _ = block.control(right.s)
            self.assertTrue(math.isfinite(value_l+value_r+slope_l+slope_r))
        # Sampling exactly at a knot selects the following interval, whose
        # Hermite endpoint derivative must equal the preceding endpoint.
        for knot in block.knots[1:-1]:
            value, derivative, _ = block.control(knot.s)
            eps = 1e-7*min(knot.s-block.knots[0].s,
                           block.knots[-1].s-knot.s)
            before = block.control(knot.s-eps)[0]
            after = block.control(knot.s+eps)[0]
            self.assertLess(abs(before-value), 2e-5)
            self.assertLess(abs(after-value), 2e-5)
            self.assertAlmostEqual((value-before)/eps, derivative, delta=2e-4)
            self.assertAlmostEqual((after-value)/eps, derivative, delta=2e-4)

    def test_oblique_body_heading_stays_fixed_for_both_directions(self):
        for amount in (.7, -.7):
            start = Pose(2., 2., .7)
            actions = [Action(1, amount, -.3)]
            plan = plan_from_actions(start, replay(start, actions)[-1], actions)
            samples = replay_continuous(plan, .02)
            self.assertTrue(all(abs(row[1].th-start.th) < 1e-12 for row in samples))
            end = samples[-1][1]
            expected = advance(start, actions[0])
            self.assertAlmostEqual(end.x, expected.x, places=12)
            self.assertAlmostEqual(end.y, expected.y, places=12)

    def test_direction_cusp_and_mode_switch_are_distinct_blocks(self):
        actions = [Action(0, .8, .1), Action(0, -.3, .1),
                   Action(1, .5, .2), Action(2, math.pi/2, 0.)]
        blocks = blocks_from_actions(actions)
        self.assertEqual([(b.mode, b.direction) for b in blocks],
                         [(0, 1), (0, -1), (1, 1), (2, 1)])
        self.assertEqual(blocks[0].mode, blocks[1].mode)  # cusp, no mode change

    def test_spin_keeps_unwrapped_yaw(self):
        start = Pose(1., 2., 3.1)
        actions = [Action(2, .3, 0.), Action(2, -.1, 0.)]
        goal = replay(start, actions)[-1]
        plan = ContinuousPlan(start, blocks_from_actions(actions), goal)
        end = replay_continuous(plan)[-1][1]
        self.assertAlmostEqual(end.th, 3.3, places=12)
        self.assertAlmostEqual(end.th-start.th, .2, places=12)

    def test_refinement_spacing_converges(self):
        start = Pose(1., 1., -.2)
        actions = [Action(0, 2., .9), Action(0, 1.5, -.7)]
        plan = plan_from_actions(start, replay(start, actions)[-1], actions)
        end_coarse = replay_continuous(plan, .04)[-1][1]
        end_fine = replay_continuous(plan, .01)[-1][1]
        self.assertLess(math.hypot(end_coarse.x-end_fine.x,
                                   end_coarse.y-end_fine.y), 2e-6)
        self.assertLess(abs(end_coarse.th-end_fine.th), 2e-6)

    def test_endpoint_fitter_restores_goal_after_control_smoothing(self):
        start = Pose(2., 2., 0.)
        actions = [Action(0, .7, -.3), Action(0, .7, .3)]
        goal = replay(start, actions)[-1]
        seed = plan_from_actions(start, goal, actions)
        unsolved_end = replay_continuous(seed, .02)[-1][1]
        self.assertLess(math.hypot(unsolved_end.x-goal.x,
                                   unsolved_end.y-goal.y), .01)
        fitted, meta = fit_continuous_endpoint(
            seed, terminal_xy_tol=1e-6, terminal_yaw_tol=1e-6)
        end = replay_continuous(fitted, .005)[-1][1]
        self.assertTrue(meta["success"], meta)
        self.assertLess(math.hypot(end.x-goal.x, end.y-goal.y), 2e-5)
        self.assertLess(abs(end.th-goal.th), 2e-5)
        self.assertEqual(meta["mode_direction_word"], [[0, 1]])


if __name__ == "__main__":
    unittest.main()
