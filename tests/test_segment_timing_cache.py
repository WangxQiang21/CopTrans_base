"""Segment reuse must preserve execution and never substitute for an audit."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import Action, Grid, Pose, replay
from fwis_v3_optimizer.continuous_geometry import (
    ContinuousPlan, plan_from_actions)
from fwis_v3_optimizer.continuous_retiming import retime_continuous, SegmentTimingCache
from fwis_v3_optimizer.quality import TrajectoryEvaluator
from fwis_v3_optimizer.retiming import MotionLimits


class SegmentTimingReuse(unittest.TestCase):
    def setUp(self):
        self.start = Pose(2., 2., 0.)
        self.limits = MotionLimits(v_max=2., target_v=2.,
            translation_speed_policy="mode_switch_only", mode_switch_duration=.3)
        self.actions = [Action(0, 3., .2), Action(1, 1., .4)]
        self.plan = plan_from_actions(self.start,
            replay(self.start, self.actions)[-1], self.actions)

    def assert_same_execution(self, before, after):
        self.assertAlmostEqual(before.duration, after.duration, places=10)
        self.assertEqual(before.mode_switch_count, after.mode_switch_count)
        self.assertAlmostEqual(before.mode_switch_hold_seconds,
                               after.mode_switch_hold_seconds, places=12)
        trace = lambda t: np.asarray([[s, p.x, p.y, p.th, v, m]
                                      for s, p, v, m in t.samples])
        np.testing.assert_allclose(trace(before), trace(after), rtol=1e-10, atol=1e-10)

    def test_shared_blocks_rebase_time_distance_pose_and_mode_holds(self):
        cache = SegmentTimingCache()
        retime_continuous(self.plan, self.limits, segment_cache=cache)
        # Put the same A/O blocks after a new Spin, at another world pose.
        spin = plan_from_actions(self.start, self.start, [Action(2, .4, 0.)]).blocks[0]
        plan = ContinuousPlan(Pose(4., 3., .6), (spin,)+self.plan.blocks, self.plan.goal)
        before = retime_continuous(plan, self.limits)
        after = retime_continuous(plan, self.limits, segment_cache=cache)
        self.assert_same_execution(before, after)
        self.assertEqual(cache.hits, 2)
        for left, right in zip(before.phases, after.phases):
            self.assertEqual(left.get("block_index"), right.get("block_index"))
            self.assertAlmostEqual(left["t0"], right["t0"], places=10)

    def test_exact_keys_invalidate_on_profile_and_control_change(self):
        cache = SegmentTimingCache()
        block = self.plan.blocks[0]
        cache.translation(block, self.limits, 0, 0., 0.)
        cache.translation(block, replace(self.limits, target_v=1.), 0, 0., 0.)
        knots = list(block.knots)
        knots[0] = replace(knots[0], value=knots[0].value+1e-10)
        changed = replace(block, knots=tuple(knots))
        cache.translation(changed, self.limits, 0, 0., 0.)
        self.assertEqual(cache.misses, 3)
        self.assertEqual(cache.hits, 0)

    def test_returned_phase_mutation_does_not_poison_later_candidates(self):
        cache = SegmentTimingCache()
        first = retime_continuous(self.plan, self.limits, segment_cache=cache)
        translation = next(p for p in first.phases if p["kind"] == "translation")
        translation["dynamic_peaks"]["speed"] = 999.
        translation["coefficients"] = [999.]*6
        second = retime_continuous(self.plan, self.limits, segment_cache=cache)
        reference = retime_continuous(self.plan, self.limits)
        self.assert_same_execution(reference, second)
        self.assertLess(second.phases[0]["dynamic_peaks"]["speed"], 3.)

    def test_cache_capacity_is_bounded_and_evicted_block_is_resolved(self):
        cache = SegmentTimingCache(max_entries=2)
        block = self.plan.blocks[0]
        for speed in (2., 1.5, 1.):
            cache.translation(block, replace(self.limits, target_v=speed), 0, 0., 0.)
        self.assertEqual(cache.stats["entries"], 2)
        self.assertEqual(cache.evictions, 1)
        cache.translation(block, self.limits, 0, 0., 0.)
        self.assertEqual(cache.misses, 4)

    def test_reusing_timing_cannot_hide_collision_in_a_new_map(self):
        cache = SegmentTimingCache()
        retime_continuous(self.plan, self.limits, segment_cache=cache)
        timed = retime_continuous(self.plan, self.limits, segment_cache=cache)
        occupied = np.zeros((200, 200), bool)
        occupied[35:45, 35:45] = True  # includes the initial vehicle footprint
        evaluator = TrajectoryEvaluator(Grid(occupied, .05, 0, 0),
            self.plan.start, self.plan.goal, self.limits)
        self.assertNotEqual(evaluator.evaluate_continuous_geometry(
            self.plan, timed)["status"], "FEASIBLE")
        self.assertFalse(cache.stats["collision_certificates_reused"])


if __name__ == "__main__":
    unittest.main()
