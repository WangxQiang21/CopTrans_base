import dataclasses
import json
import math
import time
import unittest
from unittest import mock

import numpy as np
from gymnasium.utils.env_checker import check_env

from coptrans_rl.action_library import ActionLibrary
from coptrans_rl.action_mask import valid_actions
from coptrans_rl.benchmark import engineering_tasks
from coptrans_rl.environment import EnvironmentConfig, PlanningEnv
from coptrans_rl.kernel_adapter import (
    Action, Grid, Pose, advance, replay, load_contract, FinalizationConfig,
    finalize_candidate, finalize_candidate_set)
from coptrans_rl.observation import ObservationBuilder
from coptrans_rl.configuration import load_settings


class FoundationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = load_contract()
        cls.library = ActionLibrary.from_contract(cls.contract)

    def problem(self, actions=(), **kwargs):
        grid = Grid(np.zeros((160, 160), dtype=bool), .05, 0., 0.,
                    self.contract.length, self.contract.width, self.contract.margin)
        start = Pose(3., 3., 0.)
        return self.contract.problem(grid, start, replay(start, actions)[-1], **kwargs)

    def index(self, mode, amount, control=0.):
        return next(i for i, a in enumerate(self.library.actions)
                    if a.mode == mode and abs(a.amount-amount) < 1e-8 and
                    abs(a.control-control) < 1e-8)

    def test_library_identity_and_indices(self):
        self.assertEqual(len(self.library), 80)
        self.assertEqual([sum(a.mode == m for a in self.library.actions)
                          for m in range(3)], [30, 42, 8])
        self.assertEqual(len(set(self.library.actions)), 80)
        self.assertEqual(self.library.identity,
                         ActionLibrary.from_contract(self.contract).identity)
        for bad in (-1, 80, True, 1.5, float("nan"), "1"):
            with self.assertRaisesRegex(ValueError, "invalid_action_id"):
                self.library[bad]

    def test_configuration_identity(self):
        settings = load_settings()
        self.assertEqual(settings.library.identity, self.library.identity)
        self.assertEqual(settings.contract.identity, self.contract.identity)
        self.assertEqual(settings.environment.max_decisions, 256)
        self.assertEqual(settings.problem_kwargs["xy_tol"], .02)
        self.assertEqual(len(settings.identity), 64)

    def test_gymnasium_api_and_seeded_determinism(self):
        p = self.problem([Action(0, 2., 0.)])
        env = PlanningEnv(lambda rng: p, self.library)
        check_env(env, skip_render_check=True)
        for bad in (0, -1, 2.3, float("nan"), True):
            with self.assertRaises(ValueError):
                EnvironmentConfig(max_decisions=bad)

    def test_forward_and_shared_history_mask(self):
        problem = self.problem([Action(0, 1., 0.)], allow_reverse=False)
        mask, _ = valid_actions(problem, problem.start, [], self.library)
        for i, a in enumerate(self.library.actions):
            if a.mode != 2 and a.amount < 0:
                self.assertFalse(mask[i])
        self.assertTrue(mask[self.index(2, math.pi/2)])
        problem = dataclasses.replace(problem, allow_reverse=True)
        reverse = self.index(0, -.25*self.contract.length)
        cases = [([Action(0, .5, 0.)], False),
                 ([Action(0, .5, 0.), Action(2, .26, 0.)], True),
                 ([Action(0, .5, 0.), Action(2, .26, 0.), Action(2, -.26, 0.)], False)]
        for prefix, expected in cases:
            pose = replay(problem.start, prefix)[-1]
            mask, _ = valid_actions(problem, pose, prefix, self.library)
            self.assertEqual(bool(mask[reverse]), expected)

    def test_spin_collision_at_intermediate_angle(self):
        occupied = np.zeros((400, 400), dtype=bool)
        occupied[260, 266] = True
        grid = Grid(occupied, .005, -1., -1., self.contract.length,
                    self.contract.width, self.contract.margin)
        p = Pose(0., 0., 0.)
        spin = Action(2, math.pi/2, 0.)
        self.assertTrue(grid.pose_clear(p))
        self.assertTrue(grid.pose_clear(advance(p, spin)))
        self.assertFalse(grid.action_clear(p, spin, .0025))
        problem = self.contract.problem(grid, p, p)
        mask, _ = valid_actions(problem, p, [], self.library)
        self.assertFalse(mask[self.index(2, math.pi/2)])

    def test_mask_allowed_actions_have_finer_sweep(self):
        problem = self.problem([Action(0, 1., 0.)])
        problem.grid.occ[65:85, 70:75] = True
        # Grid caches occupancy; construct a fresh instance after changing input.
        problem = dataclasses.replace(problem, grid=Grid(
            problem.grid.occ.copy(), .05, 0., 0., self.contract.length,
            self.contract.width, self.contract.margin))
        mask, _ = valid_actions(problem, problem.start, [], self.library)
        self.assertTrue(mask.any())
        self.assertTrue(all(problem.grid.action_clear(problem.start, a, .0025)
                            for i, a in enumerate(self.library.actions) if mask[i]))

    def test_vector_mask_matches_scalar_on_real_maps(self):
        rng = np.random.default_rng(20261026)
        for problem, _ in engineering_tasks(self.contract):
            for index in range(4):
                p = problem.start if index == 0 else Pose(
                    problem.start.x+rng.uniform(-.3, .3),
                    problem.start.y+rng.uniform(-.3, .3), rng.uniform(-math.pi, math.pi))
                quick, _ = valid_actions(problem, p, [], self.library)
                reference, _ = valid_actions(problem, p, [], self.library, reference=True)
                np.testing.assert_array_equal(quick, reference, err_msg=problem.task_id)

    def test_zero_motion_and_invalid_inputs(self):
        p = self.problem()
        r = finalize_candidate(p, [])
        self.assertTrue(r.feasible, r.reason)
        self.assertEqual(r.timed.duration, 0.)
        self.assertEqual(r.timed.mode_switch_count, 0)
        bads = [([Action(0, float("nan"), 0.)], "nonfinite_action"),
                ([Action(0, 0., 0.)], "zero_amount_action"),
                ([Action(3, 1., 0.)], "unknown_mode"),
                ([Action(0, .5, 2.)], "ackermann_curvature_limit")]
        for actions, reason in bads:
            r = finalize_candidate(p, actions)
            self.assertFalse(r.feasible)
            self.assertEqual(r.reason, reason)
            json.dumps(r.report(), allow_nan=False)
        for deadline in (time.perf_counter()-1, float("nan")):
            r = finalize_candidate(p, [], deadline=deadline)
            self.assertFalse(r.feasible)
        p2 = dataclasses.replace(p, goal=Pose(3.1, 3., 0.))
        self.assertEqual(finalize_candidate(p2, []).reason,
                         "empty_path_does_not_reach_goal")

    def test_continuous_chunks_do_not_add_stops(self):
        p = self.problem([Action(0, 2., 0.)])
        one = finalize_candidate(p, [Action(0, 2., 0.)])
        split = finalize_candidate(p, [Action(0, .5, 0.)]*4)
        for r in (one, split):
            self.assertTrue(r.feasible, (r.reason, r.audit, r.detail))
            self.assertEqual(r.timed.mode_switch_count, 0)
            self.assertEqual(r.timed.same_mode_hold_seconds, 0.)
            self.assertTrue(all(b[0] > a[0] for a, b in
                                zip(r.timed.samples, r.timed.samples[1:])))
        self.assertAlmostEqual(one.timed.duration, split.timed.duration, places=10)

    def test_finalizer_rejects_history_direction_and_unreached_goal(self):
        reverse = [Action(0, -.5, 0.)]
        p = self.problem(reverse, allow_reverse=False)
        self.assertEqual(finalize_candidate(p, reverse).reason, "reverse_motion_forbidden")
        forbidden = [Action(0, .5, 0.), Action(0, -.25, 0.)]
        r = finalize_candidate(self.problem(forbidden), forbidden)
        self.assertFalse(r.feasible)
        self.assertIn("translation_reversal_without_spin", r.reason)
        p = self.problem([Action(0, 2., 0.)])
        r = finalize_candidate(p, [Action(0, 1., 0.)])
        self.assertEqual(r.reason, "candidate_outside_terminal_projection_budget")

    def test_initial_oblique_counts_one_switch_and_time_selection(self):
        p = self.problem([Action(0, 2., 0.)])
        winner, results = finalize_candidate_set(p, [
            ("A", [Action(0, 2., 0.)]), ("O", [Action(1, 2., 0.)])])
        self.assertTrue(all(r.feasible for r in results), [r.reason for r in results])
        self.assertEqual(winner.source, "A")
        other = results[1]
        self.assertEqual(other.timed.mode_switch_count, 1)
        self.assertAlmostEqual(other.timed.mode_switch_hold_seconds, .3, places=10)
        self.assertAlmostEqual(other.timed.duration-winner.timed.duration, .3, places=8)

    def test_initial_spin_counts_one_switch(self):
        actions = [Action(2, math.pi/2, 0.)]
        result = finalize_candidate(self.problem(actions), actions)
        self.assertTrue(result.feasible, result.reason)
        self.assertEqual(result.timed.mode_switch_count, 1)
        self.assertAlmostEqual(result.timed.mode_switch_hold_seconds, .3, places=10)
        self.assertGreater(result.timed.spin_seconds, 0.)

    def test_boundary_recovery_preserves_mode_word_and_is_audited(self):
        actions = [Action(0, .738, 0.), Action(1, .738, math.pi/2), Action(0, .738, 0.)]
        p = self.problem(actions)
        from fwis_v3_optimizer.continuous_geometry import plan_from_actions
        failed = (plan_from_actions(p.start, p.goal, actions), {"success": False})
        with mock.patch("fwis_v3_optimizer.candidate_finalizer.fit_continuous_endpoint", return_value=failed):
            result = finalize_candidate(p, actions)
        self.assertTrue(result.feasible, result.reason)
        self.assertTrue(result.detail["boundary_attempts"][0]["success"])
        self.assertEqual(result.detail["mode_direction_word"], [[0, 1], [1, 1], [0, 1]])
        self.assertEqual(result.audit["dynamic_status"], "PASS")

    def test_lateral_corridor_requires_oblique_library_action(self):
        occ = np.ones((400, 200), dtype=bool)
        occ[20:380, 57:143] = False
        grid = Grid(occ, .01, -1., -1., self.contract.length,
                    self.contract.width, self.contract.margin)
        actions = [Action(1, self.contract.length, math.pi/2)]*2
        start = Pose(0., 0., 0.)
        p = self.contract.problem(grid, start, replay(start, actions)[-1])
        mask, _ = valid_actions(p, start, [], self.library)
        self.assertTrue(mask.any())
        self.assertTrue(all(a.mode == 1 for i, a in enumerate(self.library.actions) if mask[i]))
        result = finalize_candidate(p, actions)
        self.assertTrue(result.feasible, (result.reason, result.audit))

    def test_ackermann_arc_has_no_forced_spin(self):
        actions = [Action(0, self.contract.length, 1./self.contract.limits.r_min_ack)]*2
        result = finalize_candidate(self.problem(actions), actions)
        self.assertTrue(result.feasible, result.reason)
        self.assertEqual(result.timed.mode_switch_count, 0)
        self.assertTrue(all(block.mode == 0 for block in result.plan.blocks))

    def test_env_transfer_reward_and_observation(self):
        L = self.contract.length
        actions = [Action(0, L, 0.), Action(2, math.pi/2, 0.)]
        p = self.problem(actions)
        env = PlanningEnv(lambda rng: p, self.library)
        obs, _ = env.reset(seed=71)
        self.assertTrue(env.observation_space.contains(obs))
        phi0 = env.builder.potential(p.start)
        total = 0.
        for j, a in enumerate(actions):
            obs, reward, done, truncated, info = env.step(self.index(a.mode, a.amount, a.control))
            total += reward
            self.assertEqual(env.pose, replay(p.start, actions[:j+1])[-1])
            self.assertTrue(env.observation_space.contains(obs))
            self.assertFalse(truncated)
        self.assertTrue(done)
        self.assertTrue(info["is_success"], info)
        self.assertAlmostEqual(total, 50.-info["execution_seconds"]/10.-phi0, places=8)
        self.assertEqual(env.last_finalization.timed.mode_switch_count, 1)
        with self.assertRaisesRegex(RuntimeError, "step_after_episode_end"):
            env.step(0)

    def test_env_invalid_and_budget_failure(self):
        p = self.problem([Action(0, 2., 0.)], allow_reverse=False)
        env = PlanningEnv(lambda rng: p, self.library, EnvironmentConfig(max_decisions=1))
        env.reset(seed=2)
        _, _, done, truncated, info = env.step(-1)
        self.assertTrue(done)
        self.assertFalse(truncated)
        self.assertEqual(info["reason"], "invalid_action_id")
        env.reset(seed=2)
        _, _, done, _, info = env.step(self.index(0, .25*self.contract.length))
        self.assertTrue(done)
        self.assertEqual(info["reason"], "decision_budget_exceeded")

    def test_all_masked_and_reset_footprint(self):
        occ = np.ones((120, 120), dtype=bool)
        occ[30:90, 18:102] = False
        grid = Grid(occ, .01, -.6, -.6, self.contract.length,
                    self.contract.width, self.contract.margin)
        p = self.contract.problem(grid, Pose(0., 0., 0.), Pose(.025, 0., 0.))
        self.assertTrue(grid.pose_clear(p.start))
        self.assertTrue(grid.pose_clear(p.goal))
        mask, _ = valid_actions(p, p.start, [], self.library)
        self.assertFalse(mask.any())
        with self.assertRaisesRegex(ValueError, "no_legal_action_at_reset"):
            PlanningEnv(lambda rng: p, self.library).reset()
        bad = dataclasses.replace(p, start=Pose(99., 0., 0.))
        with self.assertRaisesRegex(ValueError, "invalid_start_or_goal"):
            PlanningEnv(lambda rng: bad, self.library).reset()

    def test_zero_motion_and_contract_mismatch_are_explicit(self):
        p = self.problem()
        with self.assertRaisesRegex(ValueError, "zero_motion_task_requires_direct_finalization"):
            PlanningEnv(lambda rng: p, self.library).reset()
        altered = dataclasses.replace(p, limits=dataclasses.replace(p.limits, r_min_ack=.9))
        with self.assertRaisesRegex(ValueError, "action_library_vehicle_contract_mismatch"):
            PlanningEnv(lambda rng: altered, self.library).reset()

    def test_real_map_manifest_has_eleven_valid_tasks(self):
        entries = [entry for _, entry in engineering_tasks(self.contract)]
        self.assertEqual(len(entries), 11)
        self.assertTrue(all(e["start_footprint_valid"] and e["goal_footprint_valid"]
                            for e in entries), entries)
        self.assertEqual([e["scale"] for e in entries],
                         [.75, 1., .7, .5, .7, 1., .8, 1.2, 1., .8, 1.])


if __name__ == "__main__":
    unittest.main()
