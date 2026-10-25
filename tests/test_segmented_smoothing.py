"""Mode-word, shared-boundary, safety and benefit checks for paper-08 QP."""
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import Grid, Pose
from fwis_v3_optimizer.continuous_geometry import (
    ContinuousPlan, ControlKnot, MotionBlock, sample_block)
from fwis_v3_optimizer.continuous_retiming import retime_continuous
from fwis_v3_optimizer.quality import TrajectoryEvaluator
from fwis_v3_optimizer.retiming import MotionLimits
from fwis_v3_optimizer.segmented_smoothing import (
    improve_segmented_trajectory, quadratic_smooth_segments)


class SegmentQuadraticSmoothing(unittest.TestCase):
    def setUp(self):
        self.start = Pose(2., 2., 0.)
        xs = np.linspace(0., 4., 9)
        values = np.array([0, 1, -1, 1, -1, 1, -1, 1, 0])*.3
        self.block = MotionBlock(0, 1, 4., tuple(
            ControlKnot(float(s), float(c), 0.) for s, c in zip(xs, values)))
        goal = sample_block(self.block, self.start, .005)[-1][1]
        self.plan = ContinuousPlan(self.start, (self.block,), goal)
        self.limits = MotionLimits(v_max=2., target_v=2.)
        self.evaluator = TrajectoryEvaluator(
            Grid(np.zeros((200, 200), bool), .05, 0, 0),
            self.start, goal, self.limits)

    def test_qp_reduces_roughness_preserves_yaw_wheel_endpoints_and_bounds(self):
        plan, report = quadratic_smooth_segments(self.plan)
        block = plan.blocks[0]
        self.assertLess(report["blocks"][0]["second_difference_after"],
                        report["blocks"][0]["second_difference_before"]*.1)
        self.assertAlmostEqual(block.control_integral(block.length),
                               self.block.control_integral(4.), places=9)
        self.assertEqual((block.mode, block.direction), (0, 1))
        self.assertAlmostEqual(block.knots[0].value, self.block.knots[0].value)
        self.assertAlmostEqual(block.knots[-1].value, self.block.knots[-1].value)
        self.assertLessEqual(max(abs(k.value) for k in block.knots), 1./.810)

    def test_certified_result_is_faster_and_reaches_the_same_shared_boundary(self):
        timed = retime_continuous(self.plan, self.limits)
        audit = self.evaluator.evaluate_continuous_geometry(self.plan, timed)
        self.assertEqual(audit["status"], "FEASIBLE")
        plan, improved, checked, report = improve_segmented_trajectory(
            self.plan, timed, audit, self.evaluator, budget_s=2.)
        self.assertTrue(report["selected"], report)
        self.assertEqual(checked["status"], "FEASIBLE", checked)
        self.assertLess(improved.duration, .8*timed.duration)
        last = sample_block(plan.blocks[0], plan.start, .005)[-1][1]
        self.assertLess(math.hypot(last.x-plan.goal.x, last.y-plan.goal.y), 1e-4)
        self.assertLess(abs(last.th-plan.goal.th), 1e-5)
        self.assertEqual(improved.mode_switch_count, timed.mode_switch_count)

    def test_oblique_and_spin_are_not_smoothed_or_relabelled(self):
        oblique = MotionBlock(1, -1, 1., (ControlKnot(0., .2, 0.),
                                         ControlKnot(1., .2, 0.)))
        spin = MotionBlock(2, 1, 0., (), .5)
        plan = ContinuousPlan(self.start, (self.block, spin, oblique), self.plan.goal)
        result, _ = quadratic_smooth_segments(plan)
        self.assertIs(result.blocks[1], spin)
        self.assertIs(result.blocks[2], oblique)

    def test_failed_swept_check_retains_identical_incumbent_objects(self):
        timed = retime_continuous(self.plan, self.limits)
        audit = self.evaluator.evaluate_continuous_geometry(self.plan, timed)
        with patch.object(self.evaluator, "continuous_collision", return_value={"collision": True}):
            plan, selected, checked, report = improve_segmented_trajectory(
                self.plan, timed, audit, self.evaluator, budget_s=2.)
        self.assertFalse(report["selected"])
        self.assertIs(plan, self.plan)
        self.assertIs(selected, timed)
        self.assertIs(checked, audit)


if __name__ == "__main__":
    unittest.main()
