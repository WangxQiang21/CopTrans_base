"""A geometric fallback must survive retiming failure without execution claims."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.continuous_geometry import plan_from_actions
from fwis_v3_optimizer.core import Action, Grid, Pose, replay
from fwis_v3_optimizer.geometry_incumbent import GeometryIncumbentStore
from fwis_v3_optimizer.quality import TrajectoryEvaluator
from fwis_v3_optimizer.retiming import MotionLimits
from fwis_v3_optimizer.continuous_retiming import retime_continuous
from fwis_v3_optimizer.mode_master import ModeMasterConfig, ModePathMaster, ModeMasterFailure


class GeometryIncumbentContracts(unittest.TestCase):
    def evaluator(self, occupied=None, goal=None):
        grid = Grid(np.zeros((120, 120), bool) if occupied is None else occupied,
                    .05, 0., 0.)
        start = Pose(2., 2., 0.)
        return TrajectoryEvaluator(grid, start, goal or Pose(3., 2., 0.),
                                   MotionLimits(), allow_shunting=False)

    def test_snapshot_is_independent_of_later_reports_and_is_not_executable(self):
        evaluator = self.evaluator()
        actions = [Action(0, 1., 0.)]
        plan = plan_from_actions(evaluator.start, evaluator.goal, actions)
        store = GeometryIncumbentStore(evaluator)
        self.assertTrue(store.preserve(actions, "safe_seed", plan=plan,
                                      execution_error="retimer_unavailable", found_at_s=.1))
        saved = store.report()
        incumbent = saved["geometry_incumbent"]
        self.assertEqual(incumbent["geometry_audit"]["geometry_status"], "PASS")
        self.assertIsNone(incumbent["geometry_audit"]["score"])
        self.assertEqual(incumbent["execution_status"], "NOT_CERTIFIED")
        self.assertFalse(incumbent["publishable_as_timed_trajectory"])
        self.assertGreater(len(incumbent["spatial_path"]), 2)
        self.assertEqual(incumbent["spatial_path"][-1], [3., 2., 0.])
        signature = incumbent["geometry_sha256"]
        incumbent["geometry"]["plan"]["start"]["x"] = 99.
        self.assertEqual(store.report()["geometry_incumbent"]["geometry"]["plan"]["start"]["x"], 2.)
        self.assertTrue(store.preserve(actions, "duplicate", plan=plan))
        self.assertEqual(len(store.records), 1)
        self.assertEqual(store.records[0]["geometry_sha256"], signature)
        json.dumps(store.report(), allow_nan=False)

    def test_collision_or_loose_terminal_pass_is_never_a_retained_incumbent(self):
        occupied = np.zeros((120, 120), bool)
        occupied[35:45, 48:51] = True
        for evaluator in (self.evaluator(occupied=occupied),
                          self.evaluator(goal=Pose(3.1, 2., 0.))):
            actions = [Action(0, 1., 0.)]
            store = GeometryIncumbentStore(evaluator)
            self.assertFalse(store.preserve(actions, "invalid"))
            self.assertIsNone(store.report()["geometry_incumbent"])

    def test_all_retimers_failing_returns_geometry_only_with_audited_snapshot(self):
        evaluator = self.evaluator()
        with patch("fwis_v3_optimizer.mode_master.retime_continuous",
                   side_effect=ValueError("test_retimer_unavailable")), \
             patch("fwis_v3_optimizer.mode_master.structural_proposals",return_value=iter([])), \
             patch("fwis_v3_optimizer.mode_master.upstream_heading_proposals",return_value=iter([])), \
             patch("fwis_v3_optimizer.mode_master.LegacyGeometryCandidateProvider.generate",
                   return_value=([], {"status": "NO_GEOMETRY_CANDIDATE"})), \
             self.assertRaises(ModeMasterFailure) as caught:
            ModePathMaster(evaluator, ModeMasterConfig(
                continuous_motion_v2=True, budget_s=.8, validation_reserve_s=.05,
                max_nlp_calls=0)).solve([Action(0, 1., 0.)])
        report = caught.exception.report
        self.assertEqual(report["status"], "GEOMETRY_ONLY")
        self.assertEqual(report["geometry_status"], "PASS")
        self.assertEqual(report["execution_status"], "NOT_CERTIFIED")
        self.assertIsNone(report["selected_id"])
        self.assertEqual(report["candidates"], [])
        self.assertEqual(report["geometry_incumbent"]["geometry_audit"]["geometry_status"], "PASS")
        self.assertFalse(report["geometry_incumbent"]["publishable_as_timed_trajectory"])
        json.dumps(report, allow_nan=False)

    def test_later_retimer_failure_cannot_replace_an_executable_incumbent(self):
        evaluator = self.evaluator()
        calls = 0
        def retimer(*args, **kwargs):
            nonlocal calls
            calls += 1
            if calls > 1:
                raise ValueError("test_later_retimer_failure")
            return retime_continuous(*args, **kwargs)
        with patch("fwis_v3_optimizer.mode_master.retime_continuous",side_effect=retimer), \
             patch("fwis_v3_optimizer.mode_master.structural_proposals",return_value=iter([
                 ([Action(1, 1., 0.)], {"operation": "test_neighbor"})])), \
             patch("fwis_v3_optimizer.mode_master.mode_schedule_neighbor_candidates",return_value=[]), \
             patch("fwis_v3_optimizer.mode_master.upstream_heading_proposals",return_value=iter([])):
            winner, report = ModePathMaster(evaluator, ModeMasterConfig(
                continuous_motion_v2=True, budget_s=1., validation_reserve_s=.05,
                max_nlp_calls=0)).solve([Action(0, 1., 0.)])
        self.assertEqual(winner["audit"]["status"], "FEASIBLE")
        self.assertEqual(winner["actions"][0].mode, 0)
        self.assertIsNotNone(report["geometry_incumbent"])


if __name__ == "__main__":
    unittest.main()
