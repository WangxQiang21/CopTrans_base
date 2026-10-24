"""Behavior contracts for the refactored quality/mode-path backend."""
import math
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import Action, Grid, Pose, replay, wrap
from fwis_v3_optimizer.continuous_geometry import (fit_continuous_endpoint,
                                                   fit_continuous_suffix,
                                                   plan_from_actions,
                                                   replay_continuous,
                                                   sample_block)
from fwis_v3_optimizer.quality import (FootprintDistance, QualityProfile,
                                       TrajectoryEvaluator)
from fwis_v3_optimizer.retiming import MotionLimits, retime
from fwis_v3_optimizer.continuous_retiming import retime_continuous
from fwis_v3_optimizer.continuous_optimizer import optimize_continuous_geometry
from fwis_v3_optimizer.continuous_optimizer import _constrain_terminal_pose
from fwis_v3_optimizer.mode_master import (ModeMasterConfig, ModePathMaster,
                                          duration_guard_candidates,
                                          forward_connections, optimize_geometry,
                                          reverse_connections, structural_proposals)
from fwis_v3_optimizer.mode_master import late_reverse_proposals
from fwis_v3_optimizer.mode_master import (mode_anchor_budget_trigger,
                                           mode_anchor_switch_count)
from fwis_v3_optimizer.quality import action_shunting_violation
from fwis_v3_optimizer.legacy_geometry_provider import LegacyGeometryCandidateProvider
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"tools"))
from trajectory_direction_audit import audit_direction


class QualityContracts(unittest.TestCase):
    def setUp(self):
        self.grid = Grid(np.zeros((160, 160), bool), .05, 0, 0)
        self.start = Pose(2, 2, 0)
        self.limits = MotionLimits()

    def test_interactive_policy_independently_audits_fixed_switch_time(self):
        limits = MotionLimits(v_max=2., target_v=2.,
            translation_speed_policy="mode_switch_only", mode_switch_duration=.3)
        actions = [Action(0, .8, .5), Action(1, .8, -.8)]
        goal = replay(self.start, actions)[-1]
        plan = plan_from_actions(self.start, goal, actions)
        timed = retime_continuous(plan, limits)
        evaluator = TrajectoryEvaluator(self.grid, self.start, goal, limits)
        audit = evaluator.evaluate_continuous_geometry(plan, timed)
        self.assertEqual(audit["status"], "FEASIBLE", audit)
        self.assertIn("diagnostic_only", audit["timing"]["translation_dynamic_contract"])
        hold = next(p for p in timed.phases if p["kind"] == "mode_switch")
        hold["duration"] += .01
        with self.assertRaisesRegex(ValueError, "continuous_mode_switch_fixed_duration"):
            evaluator._audit_continuous_timing(plan, timed, .02, math.pi/180.)

    def test_interactive_policy_still_rejects_speed_and_geometric_violations(self):
        limits = MotionLimits(v_max=2., target_v=2.,
            translation_speed_policy="mode_switch_only", mode_switch_duration=.3)
        actions = [Action(0, 4., 0.)]
        goal = replay(self.start, actions)[-1]
        plan = plan_from_actions(self.start, goal, actions)
        timed = retime_continuous(plan, limits)
        evaluator = TrajectoryEvaluator(self.grid, self.start, goal, limits)
        t, pose, _, mode = timed.samples[len(timed.samples)//2]
        timed.samples[len(timed.samples)//2] = (t, pose, 2.1, mode)
        self.assertEqual(evaluator.evaluate_continuous_geometry(plan, timed)["dynamic_status"], "FAIL")
        self.grid.occ[40,80] = True
        evaluator = TrajectoryEvaluator(self.grid, self.start, goal, limits)
        timed = retime_continuous(plan, limits)
        self.assertEqual(evaluator.evaluate_continuous_geometry(plan, timed)["geometry_status"], "FAIL")

    def test_continuous_nlp_checks_invariant_terminal_without_constant_constraint(self):
        import casadi as ca
        opt = ca.Opti()
        x, y = opt.variable(), opt.variable()
        names = _constrain_terminal_pose(opt, (x, y, .7), (3., 4., .7))
        self.assertEqual(names, ["x", "y"])
        self.assertEqual(opt.g.numel(), 2)
        with self.assertRaisesRegex(ValueError, "invariant_terminal_yaw"):
            _constrain_terminal_pose(opt, (x, y, .7), (3., 4., .8))

    def test_published_sample_audit_independently_enforces_no_shunting(self):
        for actions,expected in (
            ([Action(0,.6,0.),Action(0,-.3,0.)],False),
            ([Action(0,.6,0.),Action(2,math.pi,0.),Action(0,-.3,0.)],True),
            ([Action(0,-.6,0.)],True)):
            plan=plan_from_actions(self.start,replay(self.start,actions)[-1],actions)
            timed=retime_continuous(plan,self.limits)
            result=audit_direction([(t,p.x,p.y,p.th,v) for t,p,v,mode in timed.samples],
                                   allow_reverse=True,allow_shunting=False)
            self.assertEqual(result["pass_check"],expected,result)

    def test_boundary_distance_matches_all_occupied_cells_including_solid_interiors(self):
        rng=np.random.default_rng(20261012)
        for occupied in (rng.random((42,48))<.3,
                         np.pad(np.ones((22,28),bool),10),
                         np.ones((42,48),bool)):
            grid=Grid(occupied,.08,-.4,-.3)
            distance=FootprintDistance(grid)
            iy,ix=np.nonzero(occupied)
            all_centers=np.column_stack((grid.ox+(ix+.5)*grid.res,
                                         grid.oy+(iy+.5)*grid.res))
            for _ in range(80):
                p=Pose(float(rng.uniform(-.5,3.5)),float(rng.uniform(-.4,3.1)),
                       float(rng.uniform(-math.pi,math.pi)))
                c,s=abs(math.cos(p.th)),abs(math.sin(p.th))
                rx=distance.hl*c+distance.hw*s
                ry=distance.hl*s+distance.hw*c
                border=min(p.x-rx-grid.ox,p.y-ry-grid.oy,
                           grid.ox+grid.width*grid.res-p.x-rx,
                           grid.oy+grid.height*grid.res-p.y-ry)
                expected=(0. if border<=0 else min(border,
                    float(np.min(distance._cell_distances(p,all_centers)))))
                self.assertAlmostEqual(distance.distance(p),expected,places=10)

    def test_batched_clearance_matches_scalar_and_full_cell_reference(self):
        rng = np.random.default_rng(20261018)
        for occupied in (rng.random((42,48)) < .17,
                         np.pad(np.ones((22,28),bool),10),
                         np.ones((42,48),bool), np.zeros((42,48),bool)):
            grid = Grid(occupied,.08,-.4,-.3)
            distance = FootprintDistance(grid)
            poses = [Pose(float(rng.uniform(-.7,4.)),float(rng.uniform(-.6,3.5)),
                          float(rng.uniform(-math.pi,math.pi))) for _ in range(240)]
            batched = distance.distances(poses,chunk_size=37)
            expected = [distance.distance(p) for p in poses]
            np.testing.assert_allclose(batched,expected,rtol=0.,atol=1e-12)

    def test_batched_swept_sat_matches_full_grid_at_random_poses_and_contacts(self):
        rng = np.random.default_rng(9817)
        for occupied in (rng.random((42,48)) < .03,
                         np.pad(np.ones((22,28),bool),10),
                         np.ones((42,48),bool), np.zeros((42,48),bool)):
            grid = Grid(occupied,.08,-.4,-.3)
            distance = FootprintDistance(grid,2*grid.half_l,2*grid.half_w)
            poses = [Pose(float(rng.uniform(-.7,4.)),float(rng.uniform(-.6,3.5)),
                          float(rng.uniform(-math.pi,math.pi))) for _ in range(500)]
            poses += [Pose(grid.ox+grid.half_l,1.,0.),
                      Pose(grid.ox+grid.half_l-1e-8,1.,0.),
                      Pose(grid.ox+grid.half_l+1e-8,1.,0.)]
            np.testing.assert_array_equal(distance.poses_clear(poses,chunk_size=37),
                                           [grid.pose_clear(p) for p in poses])

    def test_sweep_certificate_reuse_is_scoped_to_exact_block_start_and_spacing(self):
        actions = [Action(0,1.,.15)]
        goal = replay(self.start,actions)[-1]
        evaluator = TrajectoryEvaluator(self.grid,self.start,goal,self.limits)
        plan = plan_from_actions(self.start,goal,actions)
        self.assertIsNone(evaluator.continuous_collision(plan))
        checked = evaluator.sweep_stats["checked_blocks"]
        self.assertIsNone(evaluator.continuous_collision(plan))
        self.assertEqual(evaluator.sweep_stats["checked_blocks"],checked)
        self.assertGreater(evaluator.sweep_stats["reused_blocks"],0)
        block = plan.blocks[0]
        moved = sample_block(block,Pose(2.1,2.,0.),.005)
        self.assertIsNone(evaluator._continuous_block_collision(moved,0,0.,block))
        self.assertEqual(evaluator.sweep_stats["checked_blocks"],checked+1)
        evaluator.profile = replace(evaluator.profile,sweep_spacing=.004)
        rows = sample_block(block,self.start,.004)
        self.assertIsNone(evaluator._continuous_block_collision(rows,0,0.,block))
        self.assertEqual(evaluator.sweep_stats["checked_blocks"],checked+2)

    def test_no_shunting_policy_rejects_plain_and_repeated_reversals(self):
        plain=[Action(0,.8,0.),Action(0,-.4,0.)]
        self.assertEqual(action_shunting_violation(plain)["reason"],
                         "translation_reversal_without_spin")
        repeated=[Action(0,.8,0.),Action(2,math.pi,0.),Action(0,-.4,0.),
                  Action(2,math.pi,0.),Action(0,.4,0.)]
        self.assertEqual(action_shunting_violation(repeated)["reason"],
                         "repeated_translation_reversals")
        for actions in (plain,repeated):
            evaluator=self.evaluator(actions)
            evaluator.allow_shunting=False
            plan=plan_from_actions(self.start,evaluator.goal,actions)
            audit=evaluator.evaluate_continuous_geometry(plan,allow_reverse=True)
            self.assertEqual(audit["status"],"INVALID")
            self.assertTrue(audit["reason"].startswith("shunting_forbidden:"))

    def test_no_shunting_policy_keeps_full_reverse_and_spin_reverse_eligible(self):
        for actions in ([Action(0,-.8,0.)],
                        [Action(0,.8,0.),Action(2,math.pi,0.),Action(0,-.4,0.)]):
            self.assertIsNone(action_shunting_violation(actions))
            evaluator=self.evaluator(actions)
            evaluator.allow_shunting=False
            plan=plan_from_actions(self.start,evaluator.goal,actions)
            audit=evaluator.evaluate_continuous_geometry(plan,
                retime_continuous(plan,self.limits),allow_reverse=True)
            self.assertEqual(audit["status"],"FEASIBLE",audit)

    def test_mode_switch_audit_rejects_understated_steering_time(self):
        actions=[Action(0,.8,0.),Action(1,.6,.5)]
        evaluator=self.evaluator(actions)
        plan=plan_from_actions(self.start,evaluator.goal,actions)
        timed=retime_continuous(plan,self.limits)
        phase=next(p for p in timed.phases if p["kind"]=="mode_switch")
        phase["steering_s"]*=.5
        audit=evaluator.evaluate_continuous_geometry(plan,timed)
        self.assertEqual(audit["reason"],"continuous_mode_switch_steering_rate_accel")

    def test_duration_guard_excludes_a_better_score_that_slows_route_too_much(self):
        candidates = [
            {"id": 0, "timed": SimpleNamespace(duration=20.),
             "audit": {"score": 8.}},
            {"id": 1, "timed": SimpleNamespace(duration=26.),
             "audit": {"score": 5.}},
        ]
        eligible, report = duration_guard_candidates(candidates, .10)
        self.assertEqual([item["id"] for item in eligible], [0])
        self.assertEqual(report["excluded_candidate_ids"], [1])
        self.assertAlmostEqual(report["duration_cap_s"], 22.)

    def test_anchor_reserve_covers_shunting_seed_repaired_to_high_switch_incumbent(self):
        config = ModeMasterConfig(continuous_motion_v2=True,
                                  mode_anchor_min_switches=3,
                                  allow_shunting=False)
        shunting_seed = [Action(0, .8, 0.), Action(0, -.4, 0.)]
        repaired_incumbent = [Action(mode, .2, 0.)
                              for mode in (0, 1, 0, 1)]
        self.assertEqual(mode_anchor_budget_trigger(config, shunting_seed),
                         "shunting_seed_repair")
        self.assertIsNone(mode_anchor_switch_count(config, shunting_seed))
        self.assertEqual(mode_anchor_switch_count(config, repaired_incumbent), 3)
        self.assertEqual(mode_anchor_budget_trigger(config, repaired_incumbent),
                         "high_switch_seed")

    def test_numerical_score_ties_keep_straight_ackermann_without_excluding_oblique(self):
        seed = [Action(0, 2., 0.)]
        evaluator = self.evaluator(seed)
        bank = lambda *args, **kwargs: iter([([Action(1, 2., 0.)],
            {"operation": "exact_mode_merge"})])
        with patch("fwis_v3_optimizer.mode_master.structural_proposals", side_effect=bank), \
             patch("fwis_v3_optimizer.mode_master.upstream_heading_proposals", return_value=iter([])):
            winner, report = ModePathMaster(evaluator, ModeMasterConfig(
                budget_s=1.5, validation_reserve_s=.1, max_nlp_calls=0,
                max_candidates=10, continuous_motion_v2=True)).solve(seed)
        self.assertTrue(any(c["source"] == "exact_mode_merge" for c in report["candidates"]))
        self.assertEqual(winner["continuous_plan"].blocks[0].mode, 0)

    def test_structural_search_does_not_repeat_for_continuous_variants_of_same_actions(self):
        seed = [Action(0,1.,.1),Action(0,1.,-.1)]
        evaluator = self.evaluator(seed)
        with patch("fwis_v3_optimizer.mode_master.structural_proposals",return_value=iter([])) as bank, \
             patch("fwis_v3_optimizer.mode_master.mode_schedule_neighbor_candidates",return_value=[]), \
             patch("fwis_v3_optimizer.mode_master.upstream_heading_proposals",return_value=iter([])):
            winner,report = ModePathMaster(evaluator,ModeMasterConfig(
                budget_s=2.,validation_reserve_s=.1,max_nlp_calls=0,
                max_candidates=10,continuous_motion_v2=True)).solve(seed)
        self.assertGreaterEqual(len(report["candidates"]),2)
        self.assertEqual(bank.call_count,1)
        self.assertEqual(report["structural_action_parents_expanded"],1)

    def test_fast_search_returns_certified_incumbent_after_a_nonimproving_expansion(self):
        seed = [Action(0,2.,0.)]
        evaluator = self.evaluator(seed)
        with patch("fwis_v3_optimizer.mode_master.structural_proposals",
                   return_value=iter([([Action(1,2.,0.)],{"operation":"exact_mode_merge"})])), \
             patch("fwis_v3_optimizer.mode_master.mode_schedule_neighbor_candidates",return_value=[]), \
             patch("fwis_v3_optimizer.mode_master.upstream_heading_proposals",return_value=iter([])):
            winner,report = ModePathMaster(evaluator,ModeMasterConfig(
                budget_s=2.,validation_reserve_s=.1,max_nlp_calls=0,
                max_stagnant_expansions=1,continuous_motion_v2=True)).solve(seed)
        self.assertEqual(winner["audit"]["status"],"FEASIBLE")
        self.assertEqual(winner["continuous_plan"].blocks[0].mode,0)
        self.assertEqual(report["search_stop_reason"],"stagnant_expansion_limit")

    def test_fast_anchor_stop_keeps_full_certificate_and_skips_remaining_search(self):
        seed = [Action(0,1.,0.),Action(1,1.,0.),Action(0,1.,0.)]
        evaluator = self.evaluator(seed)
        original_audit = evaluator.evaluate_continuous_geometry
        state = {"refined":False}
        def refine(plan,*args,**kwargs):
            state["refined"] = True
            return plan,{"attempted":True,"status":"solver_feasible_candidate"}
        def audit(plan,timed=None,*args,**kwargs):
            checked = original_audit(plan,timed,*args,**kwargs)
            if state["refined"] and checked["status"] == "FEASIBLE":
                checked["score"] *= .8  # controlled improvement for the stop-policy test
            return checked
        with patch.object(evaluator,"evaluate_continuous_geometry",side_effect=audit), \
             patch("fwis_v3_optimizer.mode_master.mode_schedule_neighbor_candidates",return_value=[]), \
             patch("fwis_v3_optimizer.mode_master.anchor_mode_sequences",
                   return_value=[([Action(0,3.,0.)],0.)]), \
             patch("fwis_v3_optimizer.mode_master.optimize_continuous_geometry",side_effect=refine), \
             patch("fwis_v3_optimizer.mode_master.structural_proposals",return_value=iter([])) as bank:
            winner,report = ModePathMaster(evaluator,ModeMasterConfig(
                budget_s=2.,validation_reserve_s=.1,max_nlp_calls=1,
                mode_anchor_min_switches=1,continuous_motion_v2=True,
                anchor_refinement_early_accept_relative_gain=.01)).solve(seed)
        self.assertEqual(winner["audit"]["geometry_status"],"PASS")
        self.assertEqual(winner["audit"]["timing_status"],"PASS")
        self.assertEqual(report["search_stop_reason"],"improving_anchor_refinement")
        self.assertEqual(bank.call_count,0)

    def test_legal_auxiliary_is_audited_before_repairing_an_illegal_seed(self):
        seed = [Action(0,1.,0.),Action(0,-.5,0.)]
        evaluator = self.evaluator(seed)
        evaluator.allow_shunting = False
        with patch("fwis_v3_optimizer.mode_master.shunt_free_seed_proposals") as repair, \
             patch("fwis_v3_optimizer.mode_master.structural_proposals",return_value=iter([])), \
             patch("fwis_v3_optimizer.mode_master.upstream_heading_proposals",return_value=iter([])), \
             patch("fwis_v3_optimizer.mode_master.mode_schedule_neighbor_candidates",return_value=[]), \
             patch("fwis_v3_optimizer.mode_master.anchor_mode_sequences") as anchors:
            winner,report = ModePathMaster(evaluator,ModeMasterConfig(
                budget_s=2.,validation_reserve_s=.1,max_nlp_calls=0,
                allow_shunting=False,strict_auxiliary_first=True,
                mode_anchor_min_switches=1,
                max_stagnant_expansions=1,continuous_motion_v2=True)).solve(
                    seed,auxiliary_seeds=[([Action(0,.25,0.),Action(1,.25,0.)],
                                           "planner_strict_primary")])
        self.assertEqual(winner["audit"]["status"],"FEASIBLE")
        self.assertTrue(report["strict_auxiliary_first_used"])
        repair.assert_not_called()
        anchors.assert_not_called()
        self.assertIsNone(report["mode_anchor_base_switches"])

    def test_late_reverse_cost_prefers_the_same_reverse_distance_at_the_end(self):
        profile = QualityProfile(reverse_early=4., reverse_start=3., reverse_yaw_early=2.)
        early = [Action(0, -.5, 0.), Action(0, 2., 0.)]
        late = [Action(0, 2., 0.), Action(0, -.5, 0.)]
        goal = replay(self.start, early)[-1]
        evaluator = TrajectoryEvaluator(self.grid, self.start, goal, self.limits,
                                        profile=profile, allow_reverse=True)
        results = [evaluator.evaluate(a, retime(self.start, a, self.limits,
                   allow_reverse=True), allow_reverse=True) for a in (early, late)]
        self.assertTrue(all(r["status"] == "FEASIBLE" for r in results), results)
        self.assertLess(results[1]["score"], results[0]["score"])
        self.assertEqual(results[0]["metrics"]["first_reverse_progress"], 0.)
        self.assertAlmostEqual(results[1]["metrics"]["first_reverse_progress"], .8)

    def test_late_reverse_preference_keeps_an_extreme_all_reverse_route_eligible(self):
        actions = [Action(0, -.5, 0.)]
        goal = replay(self.start, actions)[-1]
        profile = QualityProfile(reverse_early=4., reverse_start=3., reverse_yaw_early=2.)
        evaluator = TrajectoryEvaluator(self.grid, self.start, goal, self.limits,
                                        profile=profile, allow_reverse=True)
        plan = plan_from_actions(self.start, goal, actions)
        result = evaluator.evaluate_continuous_geometry(plan,
            retime_continuous(plan, self.limits), allow_reverse=True)
        self.assertEqual(result["status"], "FEASIBLE", result)
        self.assertGreater(result["contributions"]["reverse_start"], 0.)

    def test_late_reverse_cost_cannot_be_bypassed_by_a_millimetre_forward_preamble(self):
        profile = QualityProfile(reverse_early=4., reverse_start=3., reverse_yaw_early=2.)
        direct = [Action(0, -1., 0.)]
        preamble = [Action(0, .004, 0.), Action(0, -1.004, 0.)]
        goal = replay(self.start, direct)[-1]
        evaluator = TrajectoryEvaluator(self.grid, self.start, goal,
            self.limits, profile=profile, allow_reverse=True)
        checked = []
        for actions in (direct, preamble):
            plan = plan_from_actions(self.start, goal, actions)
            checked.append(evaluator.evaluate_continuous_geometry(plan,
                retime_continuous(plan, self.limits), allow_reverse=True))
        self.assertTrue(all(r["status"] == "FEASIBLE" for r in checked), checked)
        self.assertGreater(checked[1]["metrics"]["reverse_start"], .98)
        self.assertGreater(checked[1]["score"], checked[0]["score"])

    def test_nearly_collinear_structural_candidates_have_bounded_route_length(self):
        actions = [Action(0, .004, 0.), Action(0, -1.004, -1.6e-8)]
        proposed = list(structural_proposals(self.start, actions))
        self.assertTrue(proposed)
        bound = 1.8*sum(abs(a.amount) for a in actions)+.3
        self.assertTrue(all(sum(abs(a.amount) for a in seq if a.mode != 2) <= bound+1e-8
                            for seq, _ in proposed))

    def test_duration_estimate_does_not_materialize_a_long_playback(self):
        actions = [Action(0, 1e8, 1.6e-8)]
        estimate = retime(self.start, actions, self.limits, build_samples=False)
        self.assertEqual(len(estimate.samples), 2)
        self.assertGreater(estimate.duration, 1e8)

    def test_late_reverse_candidate_preserves_exact_goal_and_has_a_forward_prefix(self):
        actions = [Action(2, .4, 0.), Action(0, -1., .2),
                   Action(0, -2., 0.), Action(0, -.5, -.2)]
        expected = replay(self.start, actions)[-1]
        candidate, _ = next(late_reverse_proposals(self.start, actions))
        end = replay(self.start, candidate)[-1]
        self.assertLess(math.hypot(end.x-expected.x, end.y-expected.y), 1e-8)
        self.assertLess(abs(wrap(end.th-expected.th)), 1e-8)
        translations = [a for a in candidate if a.mode != 2]
        self.assertGreater(translations[0].amount, 0.)
        self.assertLess(translations[-1].amount, 0.)

    def test_direction_policy_constructor_alias_and_conflict(self):
        self.assertTrue(ModeMasterConfig().allow_reverse)
        self.assertTrue(ModeMasterConfig(allow_reverse_fallback=True).allow_reverse)
        self.assertFalse(ModeMasterConfig(allow_reverse_fallback=False).allow_reverse)
        self.assertFalse(ModeMasterConfig(allow_reverse=False).allow_reverse)
        with self.assertRaisesRegex(ValueError, "conflicting_reverse_policy_aliases"):
            ModeMasterConfig(allow_reverse=True, allow_reverse_fallback=False)
        with self.assertRaisesRegex(ValueError, "legacy_reverse_policy_requires_boolean"):
            ModeMasterConfig(allow_reverse_fallback="false")

    def test_continuous_clearance_and_sweep_use_separate_sampling_rates(self):
        actions = [Action(0, .4, 0.)]
        goal = replay(self.start, actions)[-1]
        profile = QualityProfile(metric_spacing=.05, sweep_spacing=.01)
        evaluator = TrajectoryEvaluator(self.grid, self.start, goal,
                                        self.limits, profile=profile)
        plan = plan_from_actions(self.start, goal, actions)
        timed = retime_continuous(plan, self.limits, dt=.01)
        counts = {"distance": 0, "sweep": 0}
        original_distance = evaluator.distance.distances

        def count_distance(poses):
            counts["distance"] += len(poses)
            return original_distance(poses)

        with patch.object(evaluator.distance, "distances", count_distance):
            checked = evaluator.evaluate_continuous_geometry(plan, timed)
        self.assertEqual(checked["status"], "FEASIBLE", checked)
        self.assertEqual(checked["metrics"]["clearance_metric_spacing_m"], .05)
        self.assertGreater(evaluator.sweep_stats["checked_poses"], counts["distance"])

    def test_clearance_integral_uses_nonuniform_control_knot_positions(self):
        occ = np.zeros((160, 160), bool)
        occ[47, 48] = True
        grid = Grid(occ, .05, 0, 0)
        actions = [Action(0, .43, .25), Action(0, .57, -.2)]
        goal = replay(self.start, actions)[-1]
        profile = QualityProfile(metric_spacing=.13, sweep_spacing=.01)
        evaluator = TrajectoryEvaluator(grid, self.start, goal, self.limits,
                                        profile=profile)
        plan = plan_from_actions(self.start, goal, actions)
        plan, fit = fit_continuous_endpoint(plan)
        self.assertTrue(fit["success"], fit)
        timed = retime_continuous(plan, self.limits, dt=.01)
        checked = evaluator.evaluate_continuous_geometry(plan, timed)
        self.assertEqual(checked["status"], "FEASIBLE", checked)
        block = plan.blocks[0]
        rows = sample_block(block, plan.start, profile.metric_spacing)
        positions = np.asarray([row[0] for row in rows])
        self.assertGreater(np.ptp(np.diff(positions)), 1e-5)
        preferred = profile.preferred_clearance_factor*evaluator.width
        risks = [max(0., (preferred-evaluator.distance.distance(row[1]))/
                     preferred)**2 for row in rows]
        expected = (np.trapz(risks, x=positions)/evaluator.L0+
                    .5*max(risks))
        self.assertAlmostEqual(checked["metrics"]["J_clear"], expected,
                               places=9)

    def evaluator(self, actions, goal=None):
        return TrajectoryEvaluator(self.grid, self.start,
                                   goal or replay(self.start, actions)[-1], self.limits)

    def test_cost_time_reference_is_stable_when_target_speed_is_overridden(self):
        profile = QualityProfile(
            name="quality_cost_v2_prototype_uncalibrated",
            reference_speed_mps=2., reference_spin_omega_radps=.8)
        actions = [Action(0, 2., 0.)]
        goal = replay(self.start, actions)[-1]
        slower_limits = replace(self.limits, target_v=.4)
        normal = TrajectoryEvaluator(self.grid, self.start, goal, self.limits,
                                     profile=profile)
        slower = TrajectoryEvaluator(self.grid, self.start, goal, slower_limits,
                                     profile=profile)
        self.assertEqual(normal.T0, slower.T0)
        self.assertEqual(normal.T0, 1.)
        plan = plan_from_actions(self.start, goal, actions)
        timed = retime_continuous(plan, slower_limits)
        result = slower.evaluate_continuous_geometry(plan, timed)
        self.assertEqual(result["status"], "FEASIBLE", result)
        self.assertEqual(result["cost_profile"], profile.name)
        self.assertEqual(result["reference_scales"]["T0"], 1.)

    def test_continuous_geometry_requires_exact_terminal_and_swept_clearance(self):
        actions = [Action(0, .7, -.3), Action(0, .7, .3)]
        goal = replay(self.start, actions)[-1]
        evaluator = self.evaluator(actions, goal)
        seed = plan_from_actions(self.start, goal, actions)
        raw = evaluator.evaluate_continuous_geometry(
            seed, xy_tol=1e-5, yaw_tol=1e-5, allow_reverse=True)
        self.assertEqual(raw["reason"], "strict_terminal_failed")
        fitted, fit_report = fit_continuous_endpoint(seed)
        self.assertTrue(fit_report["success"])
        checked = evaluator.evaluate_continuous_geometry(fitted, allow_reverse=True)
        self.assertEqual(checked["status"], "GEOMETRY_VALID", checked)
        self.assertIsNone(checked["score"])
        self.assertTrue(checked["direction_policy"]["all_reverse_is_eligible"])
        timed = retime_continuous(fitted, self.limits, dt=.01)
        complete = evaluator.evaluate_continuous_geometry(
            fitted, timed, allow_reverse=True)
        self.assertEqual(complete["status"], "FEASIBLE", complete)
        self.assertEqual(complete["timing_status"], "PASS")
        self.assertEqual(complete["timing"]["same_mode_hold_seconds"], 0.)
        self.assertEqual(complete["timing"]["gear_change_hold_seconds"], 0.)
        timed.samples[10] = (timed.samples[10][0], timed.samples[10][1], 1.2,
                             timed.samples[10][3])
        tampered = evaluator.evaluate_continuous_geometry(
            fitted, timed, allow_reverse=True)
        self.assertEqual(tampered["status"], "INVALID")
        self.assertEqual(tampered["timing_status"], "FAIL")

    def test_continuous_pure_reverse_is_not_rejected_by_direction_audit(self):
        actions = [Action(1, -.5, .2)]
        goal = replay(self.start, actions)[-1]
        evaluator = self.evaluator(actions, goal)
        plan = plan_from_actions(self.start, goal, actions)
        checked = evaluator.evaluate_continuous_geometry(plan, allow_reverse=True)
        self.assertEqual(checked["status"], "GEOMETRY_VALID", checked)
        self.assertAlmostEqual(checked["metrics"]["reverse_length_m"], .5)

    def test_continuous_gear_cusp_keeps_steering_continuous_without_waiting(self):
        actions = [Action(0, .8, .3), Action(0, -.5, -.3)]
        goal = replay(self.start, actions)[-1]
        evaluator = self.evaluator(actions, goal)
        raw = plan_from_actions(self.start, goal, actions)
        self.assertEqual(evaluator.evaluate_continuous_geometry(raw, allow_reverse=True)["reason"],
                         "continuous_same_mode_steering_jump")
        fitted, meta = fit_continuous_endpoint(raw, max_iterations=80)
        self.assertTrue(meta["success"], meta)
        left, right = fitted.blocks
        self.assertAlmostEqual(left.control(left.length)[0], right.control(0.)[0], places=9)
        timed = retime_continuous(fitted, self.limits)
        checked = evaluator.evaluate_continuous_geometry(fitted, timed, allow_reverse=True)
        self.assertEqual(checked["status"], "FEASIBLE", checked)
        self.assertEqual(timed.gear_change_hold_seconds, 0.)

    def test_continuous_timing_audit_accepts_only_actual_mode_switch_holds(self):
        actions = [Action(0, .6, 0.), Action(1, .6, .2), Action(0, .4, 0.)]
        goal = replay(self.start, actions)[-1]
        evaluator = self.evaluator(actions, goal)
        plan = plan_from_actions(self.start, goal, actions)
        timed = retime_continuous(plan, self.limits, dt=.01)
        checked = evaluator.evaluate_continuous_geometry(
            plan, timed, allow_reverse=True)
        self.assertEqual(checked["status"], "FEASIBLE", checked)
        self.assertEqual(timed.mode_switch_count, 2)
        self.assertEqual(checked["timing"]["phases_checked"], len(timed.phases))
        self.assertEqual(checked["timing"]["same_mode_hold_seconds"], 0.)

    def test_short_oblique_has_bounded_acceleration_and_zero_end_speeds(self):
        actions = [Action(1, .5, .3)]
        timed = retime(self.start, actions, self.limits, .01)
        motion = [p for p in timed.phases if p["kind"] == "translation"][0]
        self.assertLessEqual(motion["acceleration_peak"], self.limits.a_max+1e-9)
        self.assertLessEqual(motion["jerk_peak"], self.limits.jerk_max+1e-9)
        self.assertGreater(motion["duration"], 1.)
        self.assertEqual(timed.samples[0][2], 0)
        self.assertAlmostEqual(timed.samples[-1][2], 0)
        self.assertEqual(self.evaluator(actions).evaluate(actions, timed)["status"], "FEASIBLE")

    def test_ackermann_yaw_rate_and_acceleration_respect_profile(self):
        curvature = 1/self.limits.r_min_ack
        actions = [Action(0, 1.5, curvature)]
        timed = retime(self.start, actions, self.limits)
        phase = [p for p in timed.phases if p["kind"] == "translation"][0]
        self.assertLessEqual(phase["rate_peak"]*curvature, self.limits.omega_max+1e-9)
        self.assertLessEqual(phase["acceleration_peak"]*curvature, self.limits.alpha_max+1e-9)
        self.assertLessEqual(phase["jerk_peak"]*curvature, self.limits.angular_jerk_max+1e-9)
        self.assertEqual(self.evaluator(actions).evaluate(actions, timed)["status"], "FEASIBLE")

    def test_micro_oblique_updates_accumulate_instead_of_resetting_noise_history(self):
        actions = [Action(1, .15, i*.00006) for i in range(7)]
        timed = retime(self.start, actions, self.limits)
        audit = self.evaluator(actions).evaluate(actions, timed)
        self.assertEqual(audit["status"], "FEASIBLE")
        self.assertEqual(audit["metrics"]["oblique_adjustments"], 3)
        self.assertGreater(audit["metrics"]["oblique_tv_rad"], 0)
        self.assertGreaterEqual(timed.reconfiguration_count, 6)

    def test_oblique_angle_change_stops_and_reconfigures_even_without_mode_change(self):
        actions = [Action(1, .7, .2), Action(1, .7, -.2)]
        timed = retime(self.start, actions, self.limits)
        holds = [p for p in timed.phases if p["kind"] == "reconfiguration"]
        self.assertEqual(len(holds), 2)  # initial steering and actual O adjustment
        boundary = replay(self.start, actions)[1]
        samples = [s for s in timed.samples if math.hypot(s[1].x-boundary.x, s[1].y-boundary.y) < 1e-8]
        self.assertGreater(len(samples), 2)
        self.assertTrue(all(abs(s[2]) < 1e-8 for s in samples))
        record = self.evaluator(actions).evaluate(actions, timed)
        self.assertEqual(record["metrics"]["oblique_adjustments"], 1)

    def test_reverse_and_oblique_reverse_disguise_are_rejected(self):
        for actions in ([Action(0, -.5, 0)], [Action(1, .5, math.pi)]):
            with self.assertRaises(ValueError):
                retime(self.start, actions, self.limits)
            self.assertEqual(self.evaluator(actions).evaluate(actions)["status"], "INVALID")

    def test_signed_local_reverse_has_zero_speed_at_cusp_and_bounded_dynamics(self):
        actions = [Action(0, 1., .2), Action(0, -.4, .2), Action(1, .5, .3)]
        timed = retime(self.start, actions, self.limits, .01, allow_reverse=True)
        audit = self.evaluator(actions).evaluate(actions, timed, allow_reverse=True)
        self.assertEqual(audit["status"], "FEASIBLE")
        self.assertAlmostEqual(audit["metrics"]["reverse_length_m"], .4)
        self.assertEqual(audit["metrics"]["direction_switches"], 2)
        self.assertGreater(audit["contributions"]["reverse"], 0)
        self.assertLess(min(s[2] for s in timed.samples), 0)
        cusp_time = next(p["t0"] for p in timed.phases if p.get("amount", 0) < 0)
        self.assertAlmostEqual(next(s[2] for s in timed.samples if abs(s[0]-cusp_time)<1e-9), 0)
        samples = [(t,p.x,p.y,p.th,v) for t,p,v,_ in timed.samples]
        direction = audit_direction(samples, True,
                     {"reverse_fallback_activated": True, "forward_candidate_count": 0})
        self.assertTrue(direction["pass_check"])
        self.assertFalse(audit_direction(samples, False)["pass_check"])
        self.assertTrue(audit_direction(samples, True,
                        {"reverse_fallback_activated": False,
                         "forward_candidate_count": 1})["pass_check"])
        peaks = [abs((b[2]-a[2])/(b[0]-a[0])) for a,b in zip(timed.samples,timed.samples[1:])]
        self.assertLessEqual(max(peaks), self.limits.a_max+1e-6)

    def test_all_reverse_with_spin_is_eligible_when_config_allows_it(self):
        actions = [Action(0, -.5, 0), Action(2, .2, 0)]
        timed = retime(self.start, actions, self.limits, allow_reverse=True)
        audit = self.evaluator(actions).evaluate(actions, timed, allow_reverse=True)
        self.assertEqual(audit["status"], "FEASIBLE", audit)
        samples = [(t,p.x,p.y,p.th,v) for t,p,v,_ in timed.samples]
        direction = audit_direction(samples, True)
        self.assertTrue(direction["pass_check"])
        self.assertTrue(direction["all_reverse_route"])
        disguised = [(t,x,y,th,abs(v)) for t,x,y,th,v in samples]
        self.assertGreater(audit_direction(disguised)["signed_direction_mismatches"], 0)

    def test_local_reverse_lateral_motion_uses_signed_speed_without_yaw_flip(self):
        seed = [Action(1,.8,math.pi/2), Action(1,-.3,math.pi/2), Action(0,.4,0)]
        evaluator = self.evaluator(seed)
        timed = retime(self.start,seed,self.limits,allow_reverse=True)
        audit = evaluator.evaluate(seed,timed,allow_reverse=True)
        self.assertEqual(audit["status"], "FEASIBLE")
        self.assertTrue(all(abs(p.th-self.start.th)<1e-9 for _,p,_,_ in timed.samples))
        self.assertLess(min(v for _,_,v,_ in timed.samples),0)
        samples = [(t,p.x,p.y,p.th,v) for t,p,v,_ in timed.samples]
        direction = audit_direction(samples,True,
            {"reverse_fallback_activated": True,"forward_candidate_count": 0})
        self.assertTrue(direction["pass_check"])
        self.assertAlmostEqual(direction["reverse_distance_m"],.3)

    def test_reverse_connections_replay_to_exact_pose(self):
        start=Pose(3.0,2.0,.3)
        goal=Pose(2.0,2.4,-.2)
        routes=reverse_connections(start,goal,self.limits.r_min_ack)
        self.assertTrue(routes)
        for route in routes:
            self.assertTrue(all(action.amount<0 for action in route))
            end=replay(start,route)[-1]
            self.assertLess(math.hypot(end.x-goal.x,end.y-goal.y),1e-7)
            self.assertLess(abs(wrap(end.th-goal.th)),1e-7)

    def test_route_level_mode_fit_survives_short_local_window_limit(self):
        controls = [.1, .2, .3, .4, 0., -.4, -.3, -.2, -.1, 0.]
        seed = [Action(0, .1, control) for control in controls]
        goal = replay(self.start, seed)[-1]
        proposals = list(structural_proposals(self.start, seed, max_window=2,
                                              radius=self.limits.r_min_ack))
        route_modes = [proposal for proposal, detail in proposals
                       if detail["operation"] == "route_level_mode_fit"]
        oblique = [proposal for proposal in route_modes
                   if len(proposal) == 1 and proposal[0].mode == 1]
        self.assertTrue(oblique)
        endpoint = replay(self.start, oblique[0])[-1]
        self.assertLess(math.hypot(endpoint.x-goal.x, endpoint.y-goal.y), 1e-7)
        self.assertLess(abs(wrap(endpoint.th-goal.th)), 1e-7)

    def test_pure_reverse_candidate_competes_without_fallback_provenance(self):
        start=Pose(3.0,2.0,0.0)
        seed=[Action(0,-1.0,0.0)]
        evaluator=TrajectoryEvaluator(self.grid,start,replay(start,seed)[-1],
                                      self.limits)
        winner,report=ModePathMaster(evaluator,ModeMasterConfig(
            budget_s=.8,validation_reserve_s=.1,max_nlp_calls=0,
            max_candidates=8)).solve(seed)
        self.assertEqual(winner["audit"]["status"],"FEASIBLE")
        self.assertTrue(any(action.amount<0 for action in winner["actions"]))
        self.assertEqual(report["direction_policy"],
                         "bidirectional_joint_candidate_competition")
        self.assertGreater(report["reverse_candidate_count"],0)
        self.assertTrue(report["selected_has_reverse"])
        self.assertFalse(report["empty_pool_recovery_activated"])

    def test_reverse_geometry_refinement_preserves_signed_kinematics(self):
        seed = [Action(0, 1., .15), Action(0, -.4, .15), Action(1, .5, .2)]
        evaluator = self.evaluator(seed)
        refined, _ = optimize_geometry(self.start, evaluator.goal, seed, self.grid,
                                       evaluator, time_limit=2., allow_reverse=True)
        self.assertTrue(any(a.mode != 2 and a.amount < 0 for a in refined))
        audit = evaluator.evaluate(refined, retime(self.start, refined, self.limits,
                                   allow_reverse=True), allow_reverse=True)
        self.assertEqual(audit["status"], "FEASIBLE")

    def test_master_forward_route_beats_reverse_seed_even_with_fallback_enabled(self):
        seed = [Action(0, 2, 0), Action(0, -1, 0)]
        evaluator = self.evaluator(seed)
        winner, report = ModePathMaster(evaluator, ModeMasterConfig(
            budget_s=1, validation_reserve_s=.1, max_nlp_calls=0, max_candidates=4)).solve(seed)
        self.assertTrue(all(a.amount >= 0 or a.mode == 2 for a in winner["actions"]))
        self.assertFalse(report["reverse_fallback_activated"])
        self.assertGreater(report["forward_candidate_count"], 0)

    def test_dead_end_reverse_recovery_and_disable_switch(self):
        occ = np.ones((120, 160), bool)
        occ[10:110, 10:70] = False  # room
        occ[53:67, 70:120] = False  # narrow dead end, no initial Spin
        grid = Grid(occ, .05, 0, 0)
        start = Pose(5.3, 3, 0)
        seed = [Action(0, -2.3, 0), Action(2, math.pi, 0), Action(0, 1.5, 0)]
        evaluator = TrajectoryEvaluator(grid, start, replay(start,seed)[-1], self.limits)
        self.assertFalse(grid.trajectory_clear(replay(start,[Action(2,math.pi,0)]),
                                              [Action(2,math.pi,0)], .01))
        config = ModeMasterConfig(budget_s=2, validation_reserve_s=.1, max_nlp_calls=0, max_candidates=6)
        with patch("fwis_v3_optimizer.mode_master.optimize_geometry", side_effect=ValueError("unavailable")):
            winner, report = ModePathMaster(evaluator, config).solve(seed)
            self.assertTrue(report["selected_has_reverse"])
            self.assertEqual(report["forward_candidate_count"], 0)
            self.assertGreater(winner["audit"]["metrics"]["reverse_length_m"], 0)
            config.allow_reverse = False
            with self.assertRaisesRegex(ValueError, "no_feasible_candidate"):
                ModePathMaster(evaluator, config).solve(seed)

    def test_upstream_heading_replan_competes_in_normal_candidate_pool(self):
        occ = np.ones((120, 160), bool)
        occ[10:110, 10:70] = False
        occ[53:67, 70:120] = False
        grid = Grid(occ, .05, 0, 0)
        start = Pose(5.3, 3, 0)
        r = self.limits.r_min_ack
        # Swept-safe A-only cusp route, obtained from the generic C++ lattice.
        seed = [Action(0,-1.9,0), Action(0,-.3,-1/(2*r)),
                Action(0,-1.1127687882703863,-1/r),
                Action(0,1.3474773474341766,1/r), Action(0,.06555608629683014,-1/r)]
        evaluator = TrajectoryEvaluator(grid,start,replay(start,seed)[-1],self.limits)
        baseline = evaluator.evaluate(seed, retime(start,seed,self.limits,allow_reverse=True),allow_reverse=True)
        winner, report = ModePathMaster(evaluator, ModeMasterConfig(
            budget_s=3, validation_reserve_s=.1, max_nlp_calls=0, max_candidates=12, allow_shunting=True)).solve(seed)
        self.assertEqual(winner["audit"]["status"], "FEASIBLE")
        self.assertFalse(report["empty_pool_recovery_activated"])
        self.assertGreater(report["reverse_candidate_count"],0)
        self.assertLess(winner["audit"]["score"], .95*baseline["score"])

    def test_missing_timing_does_not_report_a_complete_score(self):
        actions = [Action(0, 1, 0)]
        audit = self.evaluator(actions).evaluate(actions)
        self.assertEqual(audit["status"], "GEOMETRY_VALID")
        self.assertIsNone(audit["score"])

    def test_legacy_geometry_provider_returns_immutable_strictly_audited_geometry(self):
        seed = [Action(0, 1., 0.)]
        alternative = [Action(1, 1., 0.)]
        evaluator = self.evaluator(seed)
        with patch("fwis_v3_optimizer.legacy_geometry_provider.anchor_mode_sequences",
                   return_value=[(alternative, 1.)]):
            candidates, report = LegacyGeometryCandidateProvider(
                evaluator, max_candidates=1).generate(
                    [(seed, "planner_primary_seed")], budget_s=.25)
        self.assertEqual(len(candidates), 1)
        candidate = candidates[0]
        self.assertEqual(candidate.source, "legacy_anchor_connection")
        self.assertIsInstance(candidate.actions, tuple)
        self.assertEqual(candidate.audit["geometry_status"], "PASS")
        self.assertEqual(candidate.audit["status"], "GEOMETRY_VALID")
        self.assertIsNone(candidate.audit["score"])
        self.assertEqual(candidate.geometry_version, "legacy_geometry_provider_v1")
        self.assertEqual(len(candidate.map_hash), 64)
        self.assertEqual(report["candidate_count"], 1)
        self.assertEqual(report["seed_audits"][0]["geometry_status"], "PASS")
        import json
        json.dumps(report, allow_nan=False)

    def test_legacy_geometry_provider_repairs_terminal_only_seed_in_a_safe_suffix(self):
        seed = [Action(0, 1.5, 0.)]
        goal = Pose(3.496, 2.112, .1)
        evaluator = TrajectoryEvaluator(
            self.grid, self.start, goal, self.limits,
            xy_tol=.02, yaw_tol=math.radians(1),
            allow_reverse=True, allow_shunting=False)
        raw = evaluator.evaluate(seed, timed=None, allow_reverse=True)
        self.assertEqual(raw.get("reason"), "terminal")
        self.assertTrue(evaluator.sweep_grid.trajectory_clear(
            replay(self.start, seed), seed, evaluator.profile.sweep_spacing))
        candidates, report = LegacyGeometryCandidateProvider(
            evaluator, max_candidates=1).generate(
                [(seed, "legacy_terminal_miss")], budget_s=1.5)
        self.assertEqual(len(candidates), 1, report)
        candidate = candidates[0]
        self.assertEqual(candidate.source, "legacy_terminal_suffix_repair")
        self.assertEqual(candidate.parent_source, "legacy_terminal_miss")
        self.assertIsInstance(candidate.actions, tuple)
        self.assertIsNotNone(candidate.continuous_plan)
        self.assertEqual(candidate.audit["geometry_status"], "PASS")
        self.assertIsNone(candidate.audit["metrics"])
        self.assertEqual(report["status"], "GEOMETRY_CANDIDATES")
        self.assertTrue(report["seed_audits"][0]["terminal_repair_eligible"])
        self.assertEqual(report["generated"][0]["has_certified_continuous_seed"], True)

        timed = retime_continuous(candidate.continuous_plan, self.limits)
        final = evaluator.evaluate_continuous_geometry(
            candidate.continuous_plan, timed,
            xy_tol=.02, yaw_tol=math.radians(1), allow_reverse=True)
        self.assertEqual(final["status"], "FEASIBLE", final)

    def test_mode_master_preserves_provider_certified_suffix_through_shared_retiming(self):
        seed = [Action(0, 1.5, 0.)]
        goal = Pose(3.496, 2.112, .1)
        evaluator = TrajectoryEvaluator(
            self.grid, self.start, goal, self.limits,
            xy_tol=.02, yaw_tol=math.radians(1),
            allow_reverse=True, allow_shunting=False)
        with patch("fwis_v3_optimizer.mode_master.fit_continuous_endpoint",
                   side_effect=ValueError("synthetic_initial_fit_rejection")), \
             patch("fwis_v3_optimizer.mode_master.structural_proposals",
                   return_value=iter([])), \
             patch("fwis_v3_optimizer.mode_master.upstream_heading_proposals",
                   return_value=iter([])):
            winner, report = ModePathMaster(evaluator, ModeMasterConfig(
                budget_s=4., validation_reserve_s=.1,
                candidate_evaluation_reserve_s=.1, max_nlp_calls=0,
                max_candidates=2, continuous_motion_v2=True,
                allow_shunting=False)).solve(seed)
        self.assertEqual(winner["audit"]["status"], "FEASIBLE", winner["audit"])
        self.assertEqual(winner["source"], "legacy_terminal_suffix_repair")
        self.assertEqual(winner["fit_report"]["status"],
                         "provider_geometry_already_certified")
        provider = report["legacy_geometry_provider"]
        self.assertEqual(provider["status"], "GEOMETRY_CANDIDATES")
        self.assertEqual(provider["execution_evaluations"][0]["status"],
                         "FEASIBLE")

    def test_mode_master_retimes_legacy_geometry_candidate_under_shared_contract(self):
        seed = [Action(0, 1., 0.)]
        alternative = [Action(1, 1., 0.)]
        evaluator = self.evaluator(seed)
        original_retime = retime_continuous

        def reject_ackermann(plan, limits, dt=.05):
            if any(block.mode == 0 for block in plan.blocks):
                raise ValueError("synthetic_seed_timing_rejection")
            return original_retime(plan, limits, dt)

        with patch("fwis_v3_optimizer.mode_master.retime_continuous",
                   side_effect=reject_ackermann), \
             patch("fwis_v3_optimizer.legacy_geometry_provider.anchor_mode_sequences",
                   return_value=[(alternative, 1.)]), \
             patch("fwis_v3_optimizer.mode_master.structural_proposals",
                   return_value=iter([])), \
             patch("fwis_v3_optimizer.mode_master.upstream_heading_proposals",
                   return_value=iter([])):
            winner, report = ModePathMaster(evaluator, ModeMasterConfig(
                budget_s=1.2, validation_reserve_s=.1,
                candidate_evaluation_reserve_s=.2, max_nlp_calls=0,
                max_candidates=2, continuous_motion_v2=True)).solve(seed)
        self.assertTrue(all(action.mode == 1 for action in winner["actions"]))
        provider = report["legacy_geometry_provider"]
        self.assertEqual(provider["status"], "GEOMETRY_CANDIDATES")
        self.assertEqual(provider["execution_evaluations"][0]["status"], "FEASIBLE")
        self.assertEqual(report["selected"]["status"], "FEASIBLE")

    def test_wrong_timing_contract_is_detected(self):
        actions = [Action(0, 1, 0)]
        timed = retime(self.start, actions, self.limits)
        timed.samples[2] = (timed.samples[2][0], Pose(2.1, 2.2, 0), .3, 0)
        audit = self.evaluator(actions).evaluate(actions, timed)
        self.assertEqual(audit["status"], "INVALID")
        self.assertEqual(audit["reason"], "sample_interpolation_contract")

    def test_equivalent_splitting_and_resampling_leave_score_unchanged(self):
        whole, split = [Action(0, 2, .2)], [Action(0, .5, .2)]*4
        evaluator = self.evaluator(whole)
        a = evaluator.evaluate(whole, retime(self.start, whole, self.limits, .1))
        b = evaluator.evaluate(split, retime(self.start, split, self.limits, .025))
        self.assertEqual(a["status"], "FEASIBLE")
        self.assertEqual(b["status"], "FEASIBLE")
        self.assertAlmostEqual(a["score"], b["score"], places=10)

    def test_body_clearance_is_exact_to_wall_and_changes_with_heading(self):
        occ = np.zeros((160, 160), bool)
        occ[:, 80:] = True  # wall x = 4.0
        distance = FootprintDistance(Grid(occ, .05, 0, 0))
        self.assertAlmostEqual(distance.distance(Pose(3, 3, 0)), 1-.738/2)
        self.assertAlmostEqual(distance.distance(Pose(3, 3, math.pi/2)), 1-.5/2)
        angle = .6
        support = .738/2*abs(math.cos(angle))+.5/2*abs(math.sin(angle))
        self.assertAlmostEqual(distance.distance(Pose(3, 3, angle)), 1-support)

    def test_long_body_edge_collision_without_interior_vertices_is_detected(self):
        grid = Grid(np.zeros((20, 20), bool), .1, 0, 0)
        distance = FootprintDistance(grid, length=1.5, width=.05)
        p = Pose(1, 1, math.pi/4)
        # A cell intersects the middle of the long body edge, not its corners.
        self.assertEqual(distance._cell_distances(p, np.array([[1., 1.]]))[0], 0.)

    def test_more_clearance_is_preferred_for_equal_straight_routes(self):
        occ = np.zeros((160, 160), bool)
        occ[0:20, :] = True  # top of obstacle row y = 1.0
        grid = Grid(occ, .05, 0, 0)
        actions = [Action(0, 2, 0)]
        values = []
        for y in (1.29, 1.40):
            start = Pose(2, y, 0)
            evaluator = TrajectoryEvaluator(grid, start, replay(start, actions)[-1], self.limits)
            record = evaluator.evaluate(actions, retime(start, actions, self.limits))
            self.assertEqual(record["status"], "FEASIBLE")
            values.append(record["score"])
        self.assertLess(values[1], values[0])

    def test_necessary_spin_is_valid_and_has_angular_limits(self):
        actions = [Action(2, math.pi/2, 0)]
        timed = retime(self.start, actions, self.limits)
        audit = self.evaluator(actions).evaluate(actions, timed)
        self.assertEqual(audit["status"], "FEASIBLE")
        self.assertTrue(all(s[2] == 0 for s in timed.samples))
        self.assertGreater(timed.spin_seconds, 0)
        self.assertEqual(audit["metrics"]["length_m"], 0)

    def test_forward_connector_endpoints_and_direction(self):
        rng = np.random.default_rng(104)
        for _ in range(80):
            p, q = (Pose(float(rng.uniform(1, 5)), float(rng.uniform(1, 5)),
                         float(rng.uniform(-math.pi, math.pi))) for _ in range(2))
            options = forward_connections(p, q)
            self.assertTrue(options)
            for actions in options:
                self.assertTrue(all(a.amount > 0 and a.mode == 0 for a in actions))
                end = replay(p, actions)[-1]
                self.assertLess(math.hypot(end.x-q.x, end.y-q.y), 1e-7)
                self.assertLess(abs(wrap(end.th-q.th)), 1e-7)

    def test_master_replaces_unneeded_oblique_without_legacy_cost_gate(self):
        actions = [Action(1, 2, 0)]
        evaluator = self.evaluator(actions)
        with patch("fwis_v3_optimizer.mode_master.optimize_geometry", side_effect=ValueError("timeout")):
            winner, report = ModePathMaster(evaluator, ModeMasterConfig(
                budget_s=2, max_nlp_calls=1, max_candidates=8)).solve(actions)
        self.assertEqual([a.mode for a in winner["actions"]], [0])
        self.assertTrue(any(c["source"] == "coarse_incumbent" for c in report["candidates"]))
        self.assertTrue(report["rejections"])

    def test_mode_master_continuous_v2_returns_audited_continuous_candidate(self):
        actions = [Action(0, .7, -.3), Action(0, .7, .3)]
        evaluator = self.evaluator(actions)
        winner, report = ModePathMaster(evaluator, ModeMasterConfig(
            budget_s=1., validation_reserve_s=.1, max_nlp_calls=0,
            max_candidates=1, continuous_motion_v2=True)).solve(actions)
        self.assertEqual(report["motion_representation"], "continuous_motion_v2")
        self.assertIsNotNone(winner.get("continuous_plan"))
        self.assertEqual(winner["timed"].interpolation_version,
                         "continuous_motion_v2_piecewise_quintic_time_v3")
        self.assertEqual(winner["audit"]["status"], "FEASIBLE")
        self.assertEqual(winner["audit"]["timing"]["same_mode_hold_seconds"], 0.)
        self.assertEqual(set(("continuous_plan_build", "endpoint_fit", "retime",
                              "independent_audit", "proposal_generation",
                              "proposal_ranking", "nlp_solve")) -
                         set(report["stage_timings_s"]), set())
        self.assertGreaterEqual(report["proposal_funnel"]["candidate_evaluation_attempted"], 1)
        self.assertGreaterEqual(report["proposal_funnel"]["candidate_feasible"], 1)
        self.assertGreaterEqual(report["stage_timings_s"]["endpoint_fit"], 0.)
        self.assertGreaterEqual(report["stage_call_counts"]["independent_audit"], 1)
        self.assertIn("stage_timings_s", report["candidates"][0])
        motion = [p for p in winner["timed"].phases if p["kind"] == "translation"]
        self.assertGreater(len(motion), 1)
        self.assertTrue(all(left["end_speed"] > .01 and
                            abs(left["end_speed"]-right["start_speed"]) < 1e-8
                            for left, right in zip(motion, motion[1:])))

    def test_suffix_repair_is_reserved_for_an_empty_candidate_pool(self):
        actions = [Action(0,1.,0.)]
        evaluator = self.evaluator(actions)
        calls = 0
        def screen(plan):
            nonlocal calls
            calls += 1
            return None if calls == 1 else {"global_s_interval_m": [0.,.005]}
        proposal = [Action(1,1.,0.)]
        with patch.object(evaluator,"continuous_collision",side_effect=screen), \
             patch("fwis_v3_optimizer.mode_master.structural_proposals",
                   return_value=iter([(proposal,{"operation":"exact_mode_merge"})])), \
             patch("fwis_v3_optimizer.mode_master.upstream_heading_proposals",return_value=iter([])), \
             patch("fwis_v3_optimizer.mode_master.fit_continuous_suffix") as suffix:
            winner,report = ModePathMaster(evaluator,ModeMasterConfig(
                budget_s=2.,max_nlp_calls=0,max_candidates=2,
                continuous_motion_v2=True)).solve(actions)
        suffix.assert_not_called()
        self.assertEqual(winner["actions"][0].mode,0)
        self.assertGreater(calls,1)
        self.assertEqual(winner["audit"]["status"],"FEASIBLE")

    def test_late_structural_window_survives_raw_connector_family_scan(self):
        seed = [Action(0, 1., 0.)]
        # Early windows can emit many unusable connector families. Their
        # count must not consume the much smaller full-audit quota.
        proposals = [([Action(0, -1., 0.)], {"operation": "early_reverse"})]*170
        proposals.append(([Action(1, 1., 0.)],
                          {"operation": "late_window_connector"}))
        with patch("fwis_v3_optimizer.mode_master.structural_proposals",
                   return_value=iter(proposals)), \
             patch("fwis_v3_optimizer.mode_master.mode_schedule_neighbor_candidates",
                   return_value=[]), \
             patch("fwis_v3_optimizer.mode_master.upstream_heading_proposals",
                   return_value=iter([])):
            _, report = ModePathMaster(self.evaluator(seed), ModeMasterConfig(
                budget_s=2., max_nlp_calls=0, max_candidates=2,
                continuous_motion_v2=True, allow_reverse=False)).solve(seed)
        self.assertIn("late_window_connector", [c["source"] for c in report["candidates"]])
        self.assertFalse(report["budget_exceeded"])

    def test_suffix_repair_does_not_hide_a_feasible_second_initializer(self):
        actions = [Action(0,.7,-.3),Action(0,.7,.3)]
        evaluator = self.evaluator(actions)
        calls = 0
        def screen(plan):
            nonlocal calls
            calls += 1
            return {"global_s_interval_m": [0.,.005]} if calls <= 2 else None
        with patch.object(evaluator,"continuous_collision",side_effect=screen), \
             patch("fwis_v3_optimizer.mode_master.fit_continuous_suffix") as suffix:
            winner,report = ModePathMaster(evaluator,ModeMasterConfig(
                budget_s=2.,max_nlp_calls=0,max_candidates=1,
                continuous_motion_v2=True)).solve(actions)
        suffix.assert_not_called()
        self.assertAlmostEqual(winner["detail"]["bandwidth_m"],.3)
        self.assertAlmostEqual(winner["detail"]["control_spacing_m"],.2)
        self.assertEqual(winner["audit"]["status"],"FEASIBLE")

    def test_collision_screen_reports_same_first_interval_as_final_geometry_audit(self):
        occupied = np.zeros((160,160),bool)
        occupied[37:43,70:73] = True
        grid = Grid(occupied,.05,0.,0.)
        actions = [Action(0,3.,0.)]
        evaluator = TrajectoryEvaluator(grid,self.start,replay(self.start,actions)[-1],self.limits)
        plan = plan_from_actions(self.start,evaluator.goal,actions)
        collision = evaluator.continuous_collision(plan)
        audit = evaluator.evaluate_continuous_geometry(plan)
        self.assertEqual(audit["reason"],"continuous_swept_collision")
        self.assertEqual(collision,audit["first_collision"])
        self.assertEqual(collision["block_index"],0)
        self.assertLessEqual(collision["global_s_interval_m"][1]-
                             collision["global_s_interval_m"][0],.005+1e-9)
        self.assertEqual(len(collision["inflated_footprint_corners_world_m"]),4)

    def test_scipy_candidate_statuses_reach_independent_audit(self):
        actions = [Action(0, .7, -.3), Action(0, .7, .3)]
        for status in ("budget_feasible_iterate", "converged_feasible", "iteration_limit_feasible"):
            with self.subTest(status=status):
                def optimized(plan, evaluator, **kwargs):
                    return plan, {"attempted": True, "status": status}
                with patch("fwis_v3_optimizer.mode_master.optimize_continuous_geometry",
                           side_effect=optimized):
                    winner, report = ModePathMaster(self.evaluator(actions), ModeMasterConfig(
                        budget_s=3., validation_reserve_s=.1, max_nlp_calls=1,
                        max_candidates=2, continuous_motion_v2=True)).solve(actions)
                accepted = [c for c in report["candidates"]
                            if c["source"] == "continuous_geometry_nlp"]
                self.assertTrue(accepted, report)
                self.assertEqual(accepted[0]["audit"]["status"], "FEASIBLE")
                self.assertEqual(accepted[0]["detail"]["status"], status)

    def test_mixed_spin_continuous_optimizer_clearance_contract(self):
        actions = [Action(0, .7, -.3), Action(2, .2, 0.), Action(0, .7, .3)]
        evaluator = self.evaluator(actions)
        plan, fit = fit_continuous_endpoint(plan_from_actions(
            self.start, evaluator.goal, actions), terminal_xy_tol=1e-7,
            terminal_yaw_tol=1e-7)
        self.assertTrue(fit["success"])
        optimized, report = optimize_continuous_geometry(plan, evaluator, time_limit=1.)
        self.assertIn("status", report)
        self.assertEqual([b.mode for b in optimized.blocks], [0, 2, 0])

    def test_failed_endpoint_fits_keep_timings_and_all_rejections(self):
        actions = [Action(0, 1., 0.)]
        with patch("fwis_v3_optimizer.mode_master.fit_continuous_endpoint",
                   side_effect=lambda plan, *args, **kwargs: (plan, {"success": False})), \
             patch("fwis_v3_optimizer.mode_master.fit_continuous_suffix",
                   side_effect=lambda plan, *args, **kwargs: (plan, {"success": False})), \
             self.assertRaises(ValueError) as caught:
            ModePathMaster(self.evaluator(actions), ModeMasterConfig(
                budget_s=.5, validation_reserve_s=.05, max_nlp_calls=0,
                continuous_motion_v2=True)).solve(actions)
        report = caught.exception.report
        self.assertEqual(report["status"], "REJECTED")
        self.assertFalse(report["infeasibility_proof"])
        self.assertGreater(report["stage_call_counts"]["endpoint_fit"], 0)
        self.assertGreater(report["stage_timings_s"]["endpoint_fit"], 0.)
        fits = [r for r in report["rejections"] if r["reason"] == "continuous_endpoint_fit_failed"]
        self.assertGreater(len(fits), 3)
        self.assertTrue(all("stage_timings_s" in r for r in fits))

    def test_direct_continuous_optimizer_preserves_topology_and_audits(self):
        actions = [Action(0, .7, -.3), Action(0, .7, .3)]
        goal = replay(self.start, actions)[-1]
        evaluator = TrajectoryEvaluator(self.grid, self.start, goal, self.limits,
                                        xy_tol=.02, yaw_tol=.02, allow_reverse=True)
        seed, fit = fit_continuous_endpoint(
            plan_from_actions(self.start, goal, actions),
            terminal_xy_tol=1e-7, terminal_yaw_tol=1e-7)
        self.assertTrue(fit["success"], fit)
        optimized, report = optimize_continuous_geometry(
            seed, evaluator, time_limit=1.25)
        self.assertIn(report["status"], ("solver_feasible_candidate", "no_solution",
                                        "budget_exhausted", "no_continuous_control_dofs"))
        if report.get("status") == "solver_feasible_candidate":
            self.assertEqual(report["endpoint_fit"]["status"],
                             "already_within_release_tolerance")
            self.assertGreater(
                report["clearance_proxy_geometric_overcoverage_m"], 0.1)
            self.assertTrue(report["final_swept_audit_required"])
        self.assertEqual([(b.mode, b.direction) for b in optimized.blocks],
                         [(b.mode, b.direction) for b in seed.blocks])
        end = replay_continuous(optimized, .01)[-1][1]
        self.assertLess(math.hypot(end.x-goal.x, end.y-goal.y), .005)
        self.assertLess(abs(end.th-goal.th), math.radians(.2))
        timed = retime_continuous(optimized, self.limits)
        audit = evaluator.evaluate_continuous_geometry(
            optimized, timed, allow_reverse=True)
        self.assertEqual(audit["status"], "FEASIBLE", audit.get("reason"))

    def test_whole_route_geometry_keeps_only_start_and_goal_fixed(self):
        actions = [Action(0, 1., .2), Action(0, 1., -.2)]
        goal = replay(self.start, actions)[-1]
        evaluator = self.evaluator(actions)
        refined, meta = optimize_geometry(self.start, goal, actions, self.grid, evaluator, time_limit=2.)
        end = replay(self.start, refined)[-1]
        self.assertLess(math.hypot(end.x-goal.x, end.y-goal.y), 1e-4)
        self.assertEqual(meta["fixed_boundary_poses"], 2)
        self.assertTrue(meta["objective_is_proxy"])

    def test_nonexact_A_prototype_can_change_mode_before_geometry_is_feasible(self):
        seed = [Action(1, 2, .15)]
        proposed = [(actions, details) for actions, details in structural_proposals(self.start, seed)
                    if details["operation"] == "free_yaw_A_prototype"]
        self.assertTrue(proposed)
        actions = proposed[0][0]
        self.assertTrue(all(a.mode == 0 for a in actions))
        goal = replay(self.start, seed)[-1]
        evaluator = self.evaluator(seed)
        self.assertEqual(evaluator.evaluate(actions)["reason"], "terminal")
        refined, meta = optimize_geometry(self.start, goal, actions, self.grid, evaluator, time_limit=2.)
        audit = evaluator.evaluate(refined, retime(self.start, refined, self.limits))
        self.assertEqual(audit["status"], "FEASIBLE")
        self.assertGreaterEqual(meta["continuous_nodes"], 2)


if __name__ == "__main__":
    unittest.main()
