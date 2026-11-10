"""S1 rigid-formation contracts; deliberately independent of ROS and maps."""
from dataclasses import FrozenInstanceError
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import Pose
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    FormationValidationError,
    VehicleOffset,
    VehiclePose,
)


def angle_error(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


class FormationModelContracts(unittest.TestCase):
    def setUp(self):
        # Non-collinear, scalene and deliberately supplied out of ID order.
        self.offsets = (
            VehicleOffset(20, 0.4, -0.7),
            VehicleOffset(3, 0.0, 0.0),
            VehicleOffset(11, 1.8, 0.2),
        )
        self.model = FormationModel(self.offsets)
        self.qf = FormationPose(4.2, -3.1, 0.37)

    def assert_pose_almost_equal(self, actual, expected, places=10):
        self.assertAlmostEqual(actual.x, expected.x, places=places)
        self.assertAlmostEqual(actual.y, expected.y, places=places)
        self.assertAlmostEqual(angle_error(actual.psi, expected.psi), 0.0,
                               places=places)

    def test_core_pose_th_is_explicitly_converted_to_unwrapped_psi(self):
        converted = FormationPose.from_core(Pose(1.2, -3.4, 7.25))
        self.assertEqual(converted, FormationPose(1.2, -3.4, 7.25))
        self.assertEqual(Pose(1.2, -3.4, 7.25).th, 7.25)

    def test_center_mapping_at_cardinal_rotations(self):
        model = FormationModel([
            VehicleOffset(0, 1.25, -0.4),
            VehicleOffset(1, -0.7, 2.0),
        ])
        expected = {
            0.0: {0: (4.25, -2.4), 1: (2.3, 0.0)},
            math.pi / 2: {0: (3.4, -0.75), 1: (1.0, -2.7)},
            -math.pi / 2: {0: (2.6, -3.25), 1: (5.0, -1.3)},
            math.pi: {0: (1.75, -1.6), 1: (3.7, -4.0)},
        }
        for psi, expected_centers in expected.items():
            with self.subTest(psi=psi):
                actual = model.centers(FormationPose(3.0, -2.0, psi))
                self.assertEqual(tuple(actual), (0, 1))
                for vehicle_id in (0, 1):
                    self.assertAlmostEqual(
                        actual[vehicle_id][0], expected_centers[vehicle_id][0],
                        places=12)
                    self.assertAlmostEqual(
                        actual[vehicle_id][1], expected_centers[vehicle_id][1],
                        places=12)

    def test_n1_n2_n3_have_exact_forward_and_inverse_geometry(self):
        for count in (1, 2, 3):
            with self.subTest(vehicle_count=count):
                model = FormationModel(self.offsets[:count])
                qf = FormationPose(-2.3, 8.7, 5 * math.pi + 0.3)
                centers = model.centers(qf)
                report = model.validate_vehicle_centers(qf, centers, 1e-10)
                self.assertEqual(report.vehicle_count, count)
                self.assertLess(report.max_pairwise_error_m, 1e-10)
                self.assertLess(report.max_reconstruction_error_m, 1e-10)
                fitted = model.fit_formation_pose(
                    centers, psi=qf.psi if count == 1 else None,
                    tolerance_m=1e-10)
                self.assert_pose_almost_equal(fitted, qf)

    def test_pure_float_rigidity_error_is_below_1e_10(self):
        centers = self.model.centers(self.qf)
        self.assertLess(self.model.rigidity_residual(centers), 1e-10)
        fitted = self.model.fit_formation_pose(centers, tolerance_m=1e-10)
        self.assertLess(abs(fitted.x - self.qf.x), 1e-10)
        self.assertLess(abs(fitted.y - self.qf.y), 1e-10)
        self.assertLess(abs(angle_error(fitted.psi, self.qf.psi)), 1e-10)

    def test_offsets_are_sorted_and_immutable(self):
        self.assertEqual(self.model.vehicle_ids, (3, 11, 20))
        self.assertEqual(tuple(self.model.pairwise_reference_distances()),
                         ((3, 11), (3, 20), (11, 20)))
        with self.assertRaises(FrozenInstanceError):
            self.model.offsets[0].x = 9.0
        with self.assertRaises(AttributeError):
            self.model.offsets = ()

    def test_pairwise_reference_distances_are_correct_and_copied(self):
        distances = self.model.pairwise_reference_distances()
        self.assertAlmostEqual(distances[(3, 11)], math.hypot(1.8, 0.2))
        self.assertAlmostEqual(distances[(3, 20)], math.hypot(0.4, -0.7))
        self.assertAlmostEqual(distances[(11, 20)], math.hypot(-1.4, -0.9))
        distances[(3, 11)] = 999.0
        self.assertNotEqual(
            self.model.pairwise_reference_distances()[(3, 11)], 999.0)

    def test_body_headings_are_independent_of_formation_psi(self):
        headings = {20: -2.0, 3: 8.1, 11: 0.4}
        poses = self.model.vehicle_poses(self.qf, headings)
        centers = self.model.centers(self.qf)
        self.assertEqual(tuple(pose.vehicle_id for pose in poses), (3, 11, 20))
        for pose in poses:
            self.assertEqual((pose.x, pose.y), centers[pose.vehicle_id])
            self.assertEqual(pose.theta, headings[pose.vehicle_id])
            self.assertNotEqual(pose.theta, self.qf.psi)

    def test_body_headings_require_exact_ids_and_finite_values(self):
        invalid = (
            {3: 0.0, 11: 0.0},
            {3: 0.0, 11: 0.0, 20: 0.0, 99: 0.0},
            {3: 0.0, 11: math.nan, 20: 0.0},
        )
        for headings in invalid:
            with self.subTest(headings=headings):
                with self.assertRaises(FormationValidationError):
                    self.model.vehicle_poses(self.qf, headings)
        with self.assertRaises(FormationValidationError):
            self.model.vehicle_poses(self.qf, [0.0, 0.0, 0.0])

    def test_vehicle_pose_iterable_preserves_ids_for_validation(self):
        headings = {3: -1.0, 11: 0.2, 20: 3.0}
        poses = self.model.vehicle_poses(self.qf, headings)
        report = self.model.validate_vehicle_centers(self.qf, poses, 1e-10)
        self.assertEqual(report.vehicle_count, 3)

    def test_design_skeleton_validate_centers_name_is_supported(self):
        centers = self.model.centers(self.qf)
        report = self.model.validate_centers(self.qf, centers, 1e-10)
        self.assertEqual(report.vehicle_count, 3)

    def test_empty_offsets_fail(self):
        with self.assertRaisesRegex(FormationValidationError, "EMPTY_FORMATION"):
            FormationModel([])

    def test_duplicate_offset_id_fails_with_context(self):
        with self.assertRaisesRegex(
                FormationValidationError,
                "DUPLICATE_VEHICLE_ID.*vehicle_id=7"):
            FormationModel([VehicleOffset(7, 0.0, 0.0),
                            VehicleOffset(7, 1.0, 0.0)])

    def test_all_coincident_offset_pairs_fail_not_only_adjacent_ids(self):
        with self.assertRaisesRegex(
                FormationValidationError, "COINCIDENT_OFFSETS.*1, 3"):
            FormationModel([VehicleOffset(1, 0.0, 0.0),
                            VehicleOffset(2, 2.0, 0.0),
                            VehicleOffset(3, 0.0, 0.0)])

    def test_invalid_vehicle_ids_fail(self):
        for vehicle_id in (True, -1, 2**32, 1.5):
            with self.subTest(vehicle_id=vehicle_id):
                with self.assertRaisesRegex(
                        FormationValidationError, "INVALID_VEHICLE_ID"):
                    VehicleOffset(vehicle_id, 0.0, 0.0)

    def test_nonfinite_offsets_and_poses_fail(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                with self.assertRaisesRegex(
                        FormationValidationError, "NONFINITE_VALUE"):
                    VehicleOffset(0, value, 0.0)
                with self.assertRaisesRegex(
                        FormationValidationError, "NONFINITE_VALUE"):
                    FormationPose(0.0, 0.0, value)

    def test_missing_and_extra_vehicles_fail_with_id_context(self):
        centers = self.model.centers(self.qf)
        missing = dict(centers)
        del missing[11]
        extra = dict(centers)
        extra[99] = (0.0, 0.0)
        both = dict(missing)
        both[99] = (0.0, 0.0)
        for value, missing_text, extra_text in (
                (missing, "[11]", "[]"),
                (extra, "[]", "[99]"),
                (both, "[11]", "[99]")):
            with self.subTest(value=value):
                with self.assertRaises(FormationValidationError) as caught:
                    self.model.validate_vehicle_centers(self.qf, value)
                self.assertIn("missing={}".format(missing_text),
                              str(caught.exception))
                self.assertIn("extra={}".format(extra_text),
                              str(caught.exception))

    def test_duplicate_ids_in_center_sequence_fail_before_dict_conversion(self):
        model = FormationModel([VehicleOffset(0, 0.0, 0.0),
                                VehicleOffset(1, 1.0, 0.0)])
        duplicate = [VehiclePose(0, 0.0, 0.0, 0.0),
                     VehiclePose(0, 1.0, 0.0, 0.0),
                     VehiclePose(1, 1.0, 0.0, 0.0)]
        with self.assertRaisesRegex(
                FormationValidationError,
                "DUPLICATE_VEHICLE_ID.*vehicle_id=0"):
            model.validate_vehicle_centers(FormationPose(0.0, 0.0, 0.0), duplicate)

    def test_nonfinite_centers_fail_with_vehicle_context(self):
        centers = self.model.centers(self.qf)
        centers[11] = (math.nan, 0.0)
        with self.assertRaisesRegex(
                FormationValidationError, "NONFINITE_VALUE.*vehicle_id=11"):
            self.model.rigidity_residual(centers)

    def test_rigidity_violation_reports_pair_and_tolerance(self):
        centers = self.model.centers(self.qf)
        centers[11] = (centers[11][0] + 0.1, centers[11][1])
        with self.assertRaises(FormationValidationError) as caught:
            self.model.validate_vehicle_centers(self.qf, centers, 1e-6)
        self.assertIn("FORMATION_RIGIDITY_VIOLATION", str(caught.exception))
        self.assertIn("pair_id=", str(caught.exception))
        self.assertIn("tolerance_m=", str(caught.exception))

    def test_reflection_has_zero_distance_residual_but_fails_rigid_contract(self):
        centers = self.model.centers(self.qf)
        reflected = {vehicle_id: (x, -y)
                     for vehicle_id, (x, y) in centers.items()}
        self.assertLess(self.model.rigidity_residual(reflected), 1e-10)
        with self.assertRaisesRegex(
                FormationValidationError, "FORMATION_CENTER_MISMATCH"):
            self.model.validate_vehicle_centers(self.qf, reflected, 1e-9)
        with self.assertRaises(FormationValidationError):
            self.model.fit_formation_pose(reflected, tolerance_m=1e-9)

    def test_swapped_vehicle_ids_fail_even_for_equilateral_distances(self):
        model = FormationModel([
            VehicleOffset(0, 0.0, 0.0),
            VehicleOffset(1, 2.0, 0.0),
            VehicleOffset(2, 1.0, math.sqrt(3.0)),
        ])
        qf = FormationPose(2.0, -4.0, 0.4)
        centers = model.centers(qf)
        centers[1], centers[2] = centers[2], centers[1]
        self.assertLess(model.rigidity_residual(centers), 1e-10)
        with self.assertRaisesRegex(
                FormationValidationError, "FORMATION_CENTER_MISMATCH"):
            model.validate_vehicle_centers(qf, centers, 1e-9)

    def test_fit_rejects_nonrigid_stretch(self):
        centers = self.model.centers(self.qf)
        stretched = {vehicle_id: (1.01 * x, y)
                     for vehicle_id, (x, y) in centers.items()}
        with self.assertRaises(FormationValidationError):
            self.model.fit_formation_pose(stretched, tolerance_m=1e-8)

    def test_single_vehicle_orientation_requires_explicit_psi(self):
        model = FormationModel([VehicleOffset(4, 0.8, -0.3)])
        centers = model.centers(self.qf)
        with self.assertRaisesRegex(
                FormationValidationError,
                "FORMATION_ORIENTATION_UNOBSERVABLE"):
            model.fit_formation_pose(centers)
        fitted = model.fit_formation_pose(centers, psi=7.2, tolerance_m=1e-10)
        self.assertEqual(fitted.psi, 7.2)
        report = model.validate_vehicle_centers(fitted, centers, 1e-10)
        self.assertLess(report.max_reconstruction_error_m, 1e-10)

    def test_degenerate_orientation_requires_explicit_psi(self):
        model = FormationModel([VehicleOffset(0, 0.0, 0.0),
                                VehicleOffset(1, 1.0, 0.0)])
        collapsed = {0: (2.0, 3.0), 1: (2.0, 3.0)}
        with self.assertRaisesRegex(
                FormationValidationError,
                "FORMATION_ORIENTATION_UNOBSERVABLE"):
            model.fit_formation_pose(collapsed)
        with self.assertRaises(FormationValidationError):
            model.fit_formation_pose(collapsed, psi=0.0, tolerance_m=1e-8)

    def test_invalid_tolerances_and_explicit_psi_fail(self):
        centers = self.model.centers(self.qf)
        for tolerance in (-1.0, math.nan, math.inf):
            with self.subTest(tolerance=tolerance):
                with self.assertRaises(FormationValidationError):
                    self.model.validate_vehicle_centers(
                        self.qf, centers, tolerance)
        with self.assertRaises(FormationValidationError):
            self.model.fit_formation_pose(centers, psi=math.nan)

    def test_unwrapped_heading_is_not_pre_wrapped_before_rotation(self):
        unwrapped = 16 * math.pi + 0.2
        direct = self.model.centers(FormationPose(1.0, 2.0, 0.2))
        actual = self.model.centers(FormationPose(1.0, 2.0, unwrapped))
        for vehicle_id in self.model.vehicle_ids:
            self.assertAlmostEqual(actual[vehicle_id][0], direct[vehicle_id][0],
                                   places=10)
            self.assertAlmostEqual(actual[vehicle_id][1], direct[vehicle_id][1],
                                   places=10)


if __name__ == "__main__":
    unittest.main()
