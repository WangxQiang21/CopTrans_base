"""S2 contracts for exact formation OBBs and conservative sweep checks."""
import math
from pathlib import Path
import random
from types import SimpleNamespace
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import Grid, Pose
from fwis_v3_optimizer.formation import (FormationModel, FormationPose,
                                         FormationValidationError, VehicleOffset,
                                         VehiclePose)
from fwis_v3_optimizer.formation_collision import (
    CollisionReport,
    FormationCollisionChecker,
    FormationState,
    SweepPolicy,
    VehicleShape,
)


class FormationCollisionContracts(unittest.TestCase):
    @staticmethod
    def make_grid(occupied=None, size=160, resolution=0.05):
        occupancy = (np.zeros((size, size), bool) if occupied is None
                     else occupied)
        return Grid(occupancy, resolution, 0.0, 0.0)

    @staticmethod
    def make_checker(offsets, shapes=None, margin=0.0, grid=None):
        model = FormationModel([
            VehicleOffset(index, x, y)
            for index, (x, y) in enumerate(offsets)
        ])
        if shapes is None:
            shapes = {index: VehicleShape(0.5, 0.3)
                      for index in range(len(offsets))}
        return FormationCollisionChecker(
            grid if grid is not None else FormationCollisionContracts.make_grid(),
            model, shapes, margin)

    @staticmethod
    def headings(checker, theta):
        return {vehicle_id: theta for vehicle_id in checker.model.vehicle_ids}

    @staticmethod
    def state(station, x, y, psi, theta_by_id):
        return FormationState(
            station, FormationPose(x, y, psi), theta_by_id)

    def test_single_vehicle_matches_existing_grid_clearance_golden(self):
        occupancy = np.zeros((120, 120), bool)
        occupancy[10:20, 15:60] = True
        occupancy[40:45, 45:80] = True
        occupancy[80:112, 90:105] = True
        grid = Grid(occupancy, 0.05, 0.0, 0.0)
        shape = VehicleShape(2 * grid.half_l, 2 * grid.half_w)
        checker = self.make_checker([(0.0, 0.0)], {0: shape}, 0.0, grid)
        rng = random.Random(17)
        mismatches = []
        for _ in range(100):
            pose = Pose(rng.uniform(0.6, 5.4), rng.uniform(0.6, 5.4),
                        rng.uniform(-math.pi, math.pi))
            report = checker.static_check(
                FormationPose(pose.x, pose.y, pose.th), {0: pose.th})
            legacy = grid.pose_clear(pose)
            if report.success != legacy:
                mismatches.append((pose, report, legacy))
        self.assertEqual(mismatches, [])
        self.assertEqual(checker.statistics()["map_queries"], 100)

    def test_n3_vehicle_1_hits_wall_and_report_identifies_it(self):
        occupancy = np.zeros((120, 120), bool)
        occupancy[59:62, 59:62] = True  # only around vehicle 1 at (3,3)
        checker = self.make_checker(
            [(-1.0, 0.0), (0.0, 0.0), (1.0, 0.0)],
            grid=self.make_grid(occupancy, 120))
        report = checker.static_check(
            FormationPose(3.0, 3.0, 0.0), self.headings(checker, 0.0))
        self.assertFalse(report.success)
        self.assertEqual(report.reason, "MAP_OBB_COLLISION")
        self.assertEqual(report.vehicle_id, 1)
        self.assertEqual(report.segment_index, 0)
        self.assertEqual(report.sample_index, 0)

    def test_fixed_centers_can_overlap_only_after_heading_rotation(self):
        checker = self.make_checker(
            [(0.0, 0.0), (0.0, 0.5)],
            {0: VehicleShape(0.8, 0.2), 1: VehicleShape(0.8, 0.2)})
        endpoints = self.headings(checker, 0.0)
        self.assertGreater(0.5, 0.2)  # center-distance shortcut would pass
        self.assertTrue(checker.static_check(
            FormationPose(4.0, 4.0, 0.0), endpoints).success)
        report = checker.static_check(
            FormationPose(4.0, 4.0, 0.0),
            self.headings(checker, math.pi / 2))
        self.assertFalse(report.success)
        self.assertEqual(report.reason, "VEHICLE_OBB_COLLISION")
        self.assertEqual(report.vehicle_id, 0)
        self.assertEqual(report.other_vehicle_id, 1)

    def test_touching_obbs_are_conservatively_a_collision(self):
        checker = self.make_checker(
            [(0.0, 0.0), (0.5, 0.0)],
            {0: VehicleShape(0.5, 0.5), 1: VehicleShape(0.5, 0.5)})
        report = checker.static_check(
            FormationPose(4.0, 4.0, 0.0), self.headings(checker, 0.0))
        self.assertFalse(report.success)
        self.assertEqual(report.reason, "VEHICLE_OBB_COLLISION")

    def test_map_boundary_touch_is_collision_but_inside_is_clear(self):
        checker = self.make_checker(
            [(0.0, 0.0)], {0: VehicleShape(0.738, 0.5)})
        touch = checker.static_check(
            FormationPose(0.369, 4.0, 0.0), {0: 0.0})
        self.assertFalse(touch.success)
        self.assertEqual(touch.reason, "MAP_OBB_COLLISION")
        self.assertEqual(touch.vehicle_id, 0)
        inside = checker.static_check(
            FormationPose(0.370, 4.0, 0.0), {0: 0.0})
        self.assertTrue(inside.success)

    def test_narrow_door_uses_margin_not_center_footprint(self):
        occupancy = np.zeros((120, 120), bool)
        occupancy[:, 79:81] = True
        occupancy[54:66, 79:81] = False  # 0.60 m doorway
        grid = self.make_grid(occupancy, 120)

        def verdict(margin):
            checker = self.make_checker(
                [(0.0, 0.0)], {0: VehicleShape(0.738, 0.5)}, margin, grid)
            return checker.static_check(
                FormationPose(4.0, 3.0, 0.0), {0: 0.0})

        self.assertTrue(verdict(0.08).success)
        ambiguous = verdict(0.10)  # expanded width exactly equals doorway
        self.assertFalse(ambiguous.success)
        self.assertEqual(ambiguous.reason, "MAP_OBB_COLLISION")
        self.assertEqual(ambiguous.vehicle_id, 0)

    def test_vehicle_shape_configuration_is_strict(self):
        model = FormationModel([VehicleOffset(0, 0.0, 0.0),
                                VehicleOffset(1, 1.0, 0.0)])
        grid = self.make_grid()
        for shapes in ({0: VehicleShape(0.5, 0.3)},
                       {0: VehicleShape(0.5, 0.3),
                        1: VehicleShape(0.5, 0.3),
                        2: VehicleShape(0.5, 0.3)}):
            with self.subTest(shapes=shapes):
                with self.assertRaisesRegex(
                        FormationValidationError, "VEHICLE_ID_SET_MISMATCH"):
                    FormationCollisionChecker(grid, model, shapes, 0.0)
        with self.assertRaises(FormationValidationError):
            FormationCollisionChecker(
                grid, model, {0: VehicleShape(0.0, 0.3),
                              1: VehicleShape(0.5, 0.3)}, 0.0)
        with self.assertRaises(FormationValidationError):
            FormationCollisionChecker(
                grid, model, {0: VehicleShape(0.5, 0.3),
                              1: VehicleShape(0.5, 0.3)}, -0.1)
        with self.assertRaises(FormationValidationError):
            FormationCollisionChecker("not-grid", model, {}, 0.0)

    def test_static_input_failures_are_structured(self):
        checker = self.make_checker([(0.0, 0.0), (1.0, 0.0)])
        qf = FormationPose(4.0, 4.0, 0.0)
        missing = checker.static_check(qf, {0: 0.0})
        self.assertFalse(missing.success)
        self.assertEqual(missing.reason, "VEHICLE_ID_SET_MISMATCH")
        self.assertEqual(missing.vehicle_id, 1)
        self.assertEqual(missing.segment_index, 0)
        extra = checker.static_check(qf, {0: 0.0, 1: 0.0, 9: 0.0})
        self.assertEqual(extra.reason, "VEHICLE_ID_SET_MISMATCH")
        self.assertEqual(extra.vehicle_id, 9)
        nan = checker.static_check(qf, {0: 0.0, 1: math.nan})
        self.assertFalse(nan.success)
        self.assertEqual(nan.reason, "NONFINITE_VALUE")
        self.assertEqual(nan.vehicle_id, 1)
        self.assertEqual(nan.segment_index, 0)
        self.assertEqual(
            checker.static_check(None, {0: 0.0, 1: 0.0}).reason,
            "INVALID_FORMATION_POSE")

    def test_static_statistics_count_map_and_pair_queries(self):
        checker = self.make_checker([(-0.7, 0.0), (0.7, 0.0)])
        report = checker.static_check(
            FormationPose(4.0, 4.0, 0.0), self.headings(checker, 0.0))
        self.assertTrue(report.success)
        stats = checker.statistics()
        self.assertEqual(stats["static_checks"], 1)
        self.assertEqual(stats["states_checked"], 1)
        self.assertEqual(stats["map_queries"], 2)
        self.assertEqual(stats["obb_pair_queries"], 1)
        stats["map_queries"] = 999
        self.assertEqual(checker.statistics()["map_queries"], 2)

    def test_n3_multisegment_straight_sweep_in_empty_map_succeeds(self):
        checker = self.make_checker(
            [(-0.6, 0.0), (0.0, 0.0), (0.6, 0.0)],
            {0: VehicleShape(0.5, 0.3),
             1: VehicleShape(0.5, 0.3),
             2: VehicleShape(0.5, 0.3)})
        headings = self.headings(checker, -0.7)
        states = [
            FormationState(0.0, FormationPose(2.0, 4.0, 0.2), headings),
            FormationState(1.5, FormationPose(3.5, 4.0, 0.2), headings),
            FormationState(3.0, FormationPose(5.0, 4.0, 0.2), headings),
        ]
        report = checker.swept_check(states, SweepPolicy(0.05))
        self.assertTrue(report.success, msg=report)
        stats = checker.statistics()
        self.assertEqual(stats["swept_checks"], 1)
        self.assertEqual(stats["segments_checked"], 2)
        self.assertGreater(stats["swept_samples_checked"], 60)
        self.assertGreater(stats["map_queries"], 60)

    def test_endpoint_clear_but_rotation_hits_wall_mid_sweep(self):
        occupancy = np.zeros((160, 160), bool)
        occupancy[:54, :] = True
        occupancy[66:, :] = True  # free band is y=2.7..3.3
        checker = self.make_checker(
            [(0.0, 0.0)], {0: VehicleShape(0.738, 0.5)}, 0.0,
            self.make_grid(occupancy))
        states = [
            FormationState(0.0, FormationPose(4.0, 3.0, 0.0), {0: 0.0}),
            FormationState(1.0, FormationPose(4.0, 3.0, 0.0),
                           {0: 2.0 * math.pi}),
        ]
        self.assertTrue(checker.static_check(
            states[0].qf, states[0].body_headings).success)
        self.assertTrue(checker.static_check(
            states[1].qf, states[1].body_headings).success)
        report = checker.swept_check(states, SweepPolicy(0.01))
        self.assertFalse(report.success)
        self.assertEqual(report.reason, "MAP_OBB_COLLISION")
        self.assertEqual(report.vehicle_id, 0)
        self.assertEqual(report.segment_index, 0)
        self.assertGreater(report.sample_index, 0)

    def test_endpoint_clear_but_two_vehicle_obbs_overlap_mid_sweep(self):
        checker = self.make_checker(
            [(0.0, 0.0), (0.0, 0.5)],
            {0: VehicleShape(0.8, 0.2), 1: VehicleShape(0.8, 0.2)})
        qf = FormationPose(4.0, 4.0, 0.0)
        states = [
            FormationState(0.0, qf, self.headings(checker, 0.0)),
            FormationState(1.0, qf, self.headings(checker, 2.0 * math.pi)),
        ]
        self.assertTrue(checker.static_check(
            states[0].qf, states[0].body_headings).success)
        self.assertTrue(checker.static_check(
            states[1].qf, states[1].body_headings).success)
        report = checker.swept_check(states, SweepPolicy(0.01))
        self.assertFalse(report.success)
        self.assertEqual(report.reason, "VEHICLE_OBB_COLLISION")
        self.assertEqual(report.vehicle_id, 0)
        self.assertEqual(report.other_vehicle_id, 1)
        self.assertEqual(report.segment_index, 0)
        self.assertGreater(report.sample_index, 0)

    def test_unwrapped_formation_rotation_over_pi_is_sampled(self):
        checker = self.make_checker(
            [(0.3, 0.0)], {0: VehicleShape(0.4, 0.2)})
        states = [
            FormationState(0.0, FormationPose(4.0, 4.0, 0.0), {0: 0.0}),
            FormationState(1.0, FormationPose(4.0, 4.0, 2.0 * math.pi),
                           {0: 0.0}),
        ]
        report = checker.swept_check(states, SweepPolicy(0.01))
        self.assertTrue(report.success, msg=report)
        stats = checker.statistics()
        self.assertEqual(stats["segments_checked"], 1)
        self.assertGreater(stats["swept_samples_checked"], 1)

    def test_empty_or_single_state_and_policy_forms(self):
        checker = self.make_checker([(0.0, 0.0)])
        empty = checker.swept_check([], SweepPolicy())
        self.assertFalse(empty.success)
        self.assertEqual(empty.reason, "EMPTY_STATES")
        one = FormationState(
            0.0, FormationPose(4.0, 4.0, 0.0), {0: 0.0})
        self.assertTrue(checker.swept_check([one], 0.05).success)
        with self.assertRaises(FormationValidationError):
            SweepPolicy(0.0)
        self.assertFalse(checker.swept_check(
            [None], SweepPolicy()).success)

    def test_bad_states_are_rejected(self):
        checker = self.make_checker([(0.0, 0.0), (0.8, 0.0)])
        headings = self.headings(checker, 0.0)
        qf = FormationPose(4.0, 4.0, 0.0)
        bad_station = checker.swept_check([
            FormationState(float("nan"), qf, headings),
            FormationState(1.0, qf, headings)], SweepPolicy())
        self.assertFalse(bad_station.success)
        self.assertEqual(bad_station.reason, "NONFINITE_VALUE")
        bad_heading = checker.swept_check([
            FormationState(0.0, qf, {0: 0.0, 1: math.nan}),
            FormationState(1.0, qf, headings)], SweepPolicy())
        self.assertFalse(bad_heading.success)
        self.assertEqual(bad_heading.reason, "NONFINITE_VALUE")
        self.assertEqual(bad_heading.vehicle_id, 1)
        decreasing = checker.swept_check([
            FormationState(1.0, qf, headings),
            FormationState(0.0, qf, headings)], SweepPolicy())
        self.assertFalse(decreasing.success)
        self.assertEqual(decreasing.reason, "NONMONOTONIC_STATIONS")
        missing = checker.swept_check([
            FormationState(0.0, qf, headings),
            FormationState(1.0, qf, {0: 0.0})], SweepPolicy())
        self.assertFalse(missing.success)
        self.assertEqual(missing.reason, "VEHICLE_ID_SET_MISMATCH")
        self.assertEqual(missing.vehicle_id, 1)
        malformed = checker.swept_check([object()], SweepPolicy())
        self.assertEqual(malformed.reason, "INVALID_FORMATION_STATE")
        bad_qf = checker.swept_check([
            SimpleNamespace(station=0.0, qf=None, body_headings=headings)],
            SweepPolicy())
        self.assertEqual(bad_qf.reason, "INVALID_FORMATION_POSE")
        self.assertEqual(
            checker.swept_check(None, SweepPolicy()).reason,
            "INVALID_STATES")

    def test_invalid_policy_and_sample_budget_fail_closed(self):
        checker = self.make_checker([(0.3, 0.0)])
        states = [
            FormationState(0.0, FormationPose(4.0, 4.0, 0.0), {0: 0.0}),
            FormationState(1.0, FormationPose(4.0, 4.0, 2.0 * math.pi),
                           {0: 0.0}),
        ]
        invalid = checker.swept_check(states, 0.0)
        self.assertEqual(invalid.reason, "INVALID_SWEEP_POLICY")
        budget = checker.swept_check(
            states, SweepPolicy(0.01, max_samples_per_segment=1))
        self.assertFalse(budget.success)
        self.assertEqual(
            budget.reason, "SWEEP_SAMPLE_BUDGET_EXCEEDED")
        self.assertEqual(budget.segment_index, 0)

    def test_vehicle_pose_adapter_keeps_independent_heading(self):
        checker = self.make_checker([(0.0, 0.0)])
        qf = FormationPose(4.0, 4.0, 0.0)
        report = checker.static_check(qf, {0: math.pi / 3.0})
        self.assertTrue(report.success)
        center = checker.model.centers(qf)[0]
        vehicle_pose = VehiclePose(0, center[0], center[1], math.pi / 3.0)
        self.assertNotEqual(vehicle_pose.theta, qf.psi)
        self.assertIsInstance(report, CollisionReport)
        report = checker.static_check(qf, {0: 0.0})


if __name__ == "__main__":
    unittest.main()
