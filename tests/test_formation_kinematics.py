"""S5 mode-assignment and independent kinematic-audit tests.

These tests deliberately use no ROS, map, collision checker or retimer.  They
verify only the per-vehicle A/O/S equations and the rigid heading boundary
contract.
"""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.formation import (
    FormationPose,
    FormationValidationError,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_kinematics import (
    ACKERMANN,
    CERTIFICATION_AUDITED,
    COMPOSITE,
    KinematicLimits,
    OBLIQUE,
    ROTATION,
    SPIN,
    TRANSLATION,
    FormationKinematicBlock,
    VehicleModeSegment,
    assign_formation_modes,
    audit_mode_assignment,
    enumerate_mode_assignments,
    mode_word,
)


def primitive(geometry_type, dx=0.0, dy=0.0, dpsi=0.0,
              pivot_x=0.0, pivot_y=0.0):
    return type("Primitive", (), {
        "geometry_type": geometry_type,
        "dx": dx,
        "dy": dy,
        "dpsi": dpsi,
        "pivot_x": pivot_x,
        "pivot_y": pivot_y,
    })()


def translation_block(displacement, offsets, start_psi=0.0):
    length = math.hypot(*displacement)
    start = FormationPose(2.0, 3.0, start_psi)
    return FormationKinematicBlock.from_primitive(
        start, primitive(0, dx=displacement[0], dy=displacement[1]),
        offsets)


def rotation_block(dpsi, offsets, pivot=(0.0, 0.0), start_psi=0.0):
    return FormationKinematicBlock.from_primitive(
        FormationPose(6.0, 7.0, start_psi),
        primitive(1, dpsi=dpsi, pivot_x=pivot[0], pivot_y=pivot[1]),
        offsets)


def headings(**values):
    return values


class FormationKinematicsTests(unittest.TestCase):
    def setUp(self):
        self.limits = KinematicLimits()
        self.wedge = (
            VehicleOffset(0, 0.0, 0.0),
            VehicleOffset(1, 0.0, 1.2),
            VehicleOffset(2, 0.0, -1.2),
        )

    def assert_audit_passes(self, assignment, expected_end=None,
                            free=None, allow_reverse=False):
        audit = audit_mode_assignment(
            assignment, limits=self.limits,
            expected_end_headings=expected_end,
            goal_heading_free=free, allow_reverse=allow_reverse)
        self.assertTrue(audit.success, audit.reason + " " + audit.detail)
        self.assertEqual(audit.certification, CERTIFICATION_AUDITED)
        self.assertEqual(audit.checks, len(assignment.segments))
        self.assertFalse(audit.as_status()["executable"])
        return audit

    def test_translation_a_forward_and_reverse_are_signed_consistently(self):
        block = translation_block((3.0, 0.0), self.wedge)
        forward = assign_formation_modes(
            block, {0: 0.0, 1: 0.0, 2: 0.0})
        self.assertTrue(forward.success)
        for segment in forward.segments:
            self.assertEqual(segment.mode, ACKERMANN)
            self.assertEqual(segment.direction, 1)
            self.assertEqual(segment.control, 0.0)
            self.assertAlmostEqual(segment.travel_distance, 3.0)
            self.assertAlmostEqual(
                segment.end_center_x - segment.start_center_x, 3.0)
        self.assert_audit_passes(forward)

        reverse_block = translation_block((-3.0, 0.0), self.wedge)
        blocked = assign_formation_modes(
            reverse_block, {0: 0.0, 1: 0.0, 2: 0.0})
        self.assertFalse(blocked.success)
        self.assertEqual(blocked.reason, "REVERSE_NOT_ALLOWED")

        reverse = assign_formation_modes(
            reverse_block, {0: 0.0, 1: 0.0, 2: 0.0},
            allow_reverse=True)
        self.assertTrue(reverse.success, reverse.detail)
        for segment in reverse.segments:
            self.assertEqual(segment.mode, ACKERMANN)
            self.assertEqual(segment.direction, -1)
            self.assertAlmostEqual(
                segment.end_center_x - segment.start_center_x,
                -3.0, places=12)
        self.assert_audit_passes(reverse, allow_reverse=True)
        reverse_policy = audit_mode_assignment(reverse)
        self.assertFalse(reverse_policy.success)
        self.assertEqual(reverse_policy.reason, "REVERSE_NOT_ALLOWED")

    def test_oblique_translation_holds_body_heading_and_uses_signed_phi(self):
        block = translation_block(
            (2.0 * math.cos(math.pi / 3.0),
             2.0 * math.sin(math.pi / 3.0)), self.wedge)
        assignment = assign_formation_modes(
            block, {0: 0.0, 1: 0.0, 2: 0.0})
        self.assertTrue(assignment.success, assignment.detail)
        for segment in assignment.segments:
            self.assertEqual(segment.mode, OBLIQUE)
            self.assertEqual(segment.direction, 1)
            self.assertAlmostEqual(segment.control, math.pi / 3.0)
            self.assertAlmostEqual(segment.theta_end, segment.theta_start)
        audit = self.assert_audit_passes(assignment)
        self.assertAlmostEqual(audit.max_oblique_angle_rad, math.pi / 3.0)

    def test_oblique_reverse_is_not_hidden_in_a_huge_forward_angle(self):
        motion = 2.0 * math.pi / 3.0
        block = translation_block(
            (2.0 * math.cos(motion), 2.0 * math.sin(motion)), self.wedge)
        assignment = assign_formation_modes(
            block, {0: 0.0, 1: 0.0, 2: 0.0}, allow_reverse=True)
        self.assertTrue(assignment.success, assignment.detail)
        for segment in assignment.segments:
            self.assertEqual(segment.mode, OBLIQUE)
            self.assertEqual(segment.direction, -1)
            self.assertAlmostEqual(segment.control, -math.pi / 3.0)
            self.assertLessEqual(
                abs(segment.control), self.limits.oblique_angle_max)
            self.assertTrue(audit_mode_assignment(
                assignment, allow_reverse=True).success)

    def test_rotation_pivot_is_spin_and_nonpivot_vehicles_use_a(self):
        block = rotation_block(math.pi / 2.0, self.wedge)
        start = {0: 0.0, 1: 0.0, 2: 0.0}
        assignment = assign_formation_modes(
            block, start, allow_reverse=True)
        self.assertTrue(assignment.success, assignment.detail)
        modes = {segment.vehicle_id: segment.mode
                 for segment in assignment.segments}
        self.assertEqual(modes, {0: SPIN, 1: ACKERMANN, 2: ACKERMANN})
        pivot = assignment.segment_map[0]
        self.assertEqual(pivot.travel_distance, 0.0)
        self.assertEqual(pivot.direction, 1)
        self.assertAlmostEqual(pivot.end_center_x, pivot.start_center_x)
        self.assertAlmostEqual(pivot.end_center_y, pivot.start_center_y)

        upper = assignment.segment_map[1]
        lower = assignment.segment_map[2]
        self.assertEqual(upper.direction, -1)
        self.assertAlmostEqual(upper.control, -1.0 / 1.2)
        self.assertEqual(lower.direction, 1)
        self.assertAlmostEqual(lower.control, 1.0 / 1.2)
        self.assertAlmostEqual(upper.theta_end, math.pi / 2.0)
        audit = self.assert_audit_passes(assignment, allow_reverse=True)
        self.assertAlmostEqual(audit.max_curvature, 1.0 / 1.2)

    def test_nonpivot_spin_assignment_is_rejected_by_independent_audit(self):
        block = rotation_block(math.pi / 2.0, self.wedge)
        assignment = assign_formation_modes(
            block, {0: 0.0, 1: 0.0, 2: 0.0}, allow_reverse=True)
        tampered = []
        for segment in assignment.segments:
            if segment.vehicle_id == 1:
                tampered.append(replace(
                    segment, mode=SPIN, direction=1, control=0.0,
                    travel_distance=0.0))
            else:
                tampered.append(segment)
        bad = replace(assignment, segments=tuple(tampered))
        audit = audit_mode_assignment(bad)
        self.assertFalse(audit.success)
        self.assertEqual(audit.reason, "NON_PIVOT_SPIN")
        self.assertIn("vehicle_id=1", audit.detail)

    def test_rotation_radius_below_rmin_is_rejected(self):
        offsets = (
            VehicleOffset(0, 0.0, 0.0),
            VehicleOffset(1, 0.0, 0.5),
        )
        block = rotation_block(math.pi / 2.0, offsets)
        # Forward tangent of (0, +0.5) is +pi; use it to isolate curvature.
        assignment = assign_formation_modes(
            block, {0: 0.0, 1: math.pi})
        self.assertFalse(assignment.success)
        self.assertEqual(assignment.reason, "CURVATURE_LIMIT")
        self.assertIn("vehicle_id=1", assignment.detail)

    def test_rotation_requires_heading_to_match_a_tangent(self):
        block = rotation_block(math.pi / 2.0, self.wedge)
        assignment = assign_formation_modes(
            block, {0: 0.0, 1: 0.3, 2: 0.0}, allow_reverse=True)
        self.assertFalse(assignment.success)
        self.assertEqual(assignment.reason, "HEADING_BOUNDARY_MISMATCH")
        self.assertIn("vehicle_id=1", assignment.detail)

    def test_rotation_reverse_travel_requires_allow_reverse(self):
        block = rotation_block(math.pi / 2.0, self.wedge)
        assignment = assign_formation_modes(
            block, {0: 0.0, 1: 0.0, 2: 0.0})
        self.assertFalse(assignment.success)
        self.assertEqual(assignment.reason, "REVERSE_NOT_ALLOWED")
        self.assertIn("vehicle_id=1", assignment.detail)

    def test_goal_heading_constraint_is_explicit_and_heading_free_only_skips(self):
        block = translation_block((2.0, 0.0), self.wedge[:2])
        goal = {0: 0.0, 1: 0.0}
        assignment = assign_formation_modes(
            block, {0: 0.0, 1: 0.0}, expected_end_headings=goal)
        self.assertTrue(assignment.success)
        self.assertTrue(assignment.goal_headings_checked)
        self.assertTrue(assignment.end_headings)
        audit = self.assert_audit_passes(assignment, expected_end=goal)
        self.assertTrue(audit.goal_headings_checked)

        mismatch = assign_formation_modes(
            block, {0: 0.0, 1: 0.0},
            expected_end_headings={0: 0.2, 1: 0.0})
        self.assertFalse(mismatch.success)
        self.assertEqual(mismatch.reason, "GOAL_HEADING_MISMATCH")

        freed = assign_formation_modes(
            block, {0: 0.0, 1: 0.0},
            expected_end_headings={0: 0.2, 1: 0.0},
            goal_heading_free={0: True, 1: False})
        self.assertTrue(freed.success, freed.detail)
        self.assertTrue(freed.goal_headings_checked)
        self.assertTrue(audit_mode_assignment(
            freed, expected_end_headings={0: 0.2, 1: 0.0},
            goal_heading_free={0: True, 1: False}).success)

    def test_independent_audit_recomputes_centers_heading_and_control(self):
        block = translation_block(
            (2.0 * math.cos(math.pi / 3.0),
             2.0 * math.sin(math.pi / 3.0)), self.wedge)
        assignment = assign_formation_modes(
            block, {0: 0.0, 1: 0.0, 2: 0.0})
        self.assertTrue(assignment.success)
        good = audit_mode_assignment(assignment)
        self.assertTrue(good.success)

        shifted = replace(
            assignment.segments[0],
            start_center_x=assignment.segments[0].start_center_x + 0.01)
        bad_center = replace(assignment, segments=(
            shifted,) + assignment.segments[1:])
        audit = audit_mode_assignment(bad_center)
        self.assertFalse(audit.success)
        self.assertEqual(audit.reason, "CONSTRUCTION_MISMATCH")

        turned = replace(assignment.segments[0], control=0.2)
        bad_control = replace(assignment, segments=(
            turned,) + assignment.segments[1:])
        audit = audit_mode_assignment(bad_control)
        self.assertFalse(audit.success)
        self.assertEqual(audit.reason, "MODE_DISPLACEMENT_MISMATCH")

        drifted = replace(assignment.segments[0], theta_end=0.2)
        bad_heading = replace(assignment, segments=(
            drifted,) + assignment.segments[1:])
        audit = audit_mode_assignment(bad_heading)
        self.assertFalse(audit.success)
        self.assertEqual(audit.reason, "HEADING_CONTINUITY_VIOLATION")

    def test_spin_requires_an_identically_fixed_center(self):
        block = rotation_block(math.pi / 2.0, self.wedge)
        assignment = assign_formation_modes(
            block, {0: 0.0, 1: 0.0, 2: 0.0}, allow_reverse=True)
        pivot = assignment.segment_map[0]
        moved = replace(pivot, end_center_x=pivot.end_center_x + 0.001)
        bad = replace(
            assignment,
            segments=tuple(moved if s.vehicle_id == 0 else s
                           for s in assignment.segments))
        audit = audit_mode_assignment(bad, allow_reverse=True)
        self.assertFalse(audit.success)
        self.assertIn(audit.reason, ("CONSTRUCTION_MISMATCH",
                                     "SPIN_CENTER_MOTION"))

    def test_joint_enumeration_retains_k_and_all_are_auditable(self):
        block = translation_block((2.0, 0.0), self.wedge[:2])
        assignments = enumerate_mode_assignments(
            block, {0: 0.0, 1: 0.0}, max_schedules=4)
        self.assertEqual(len(assignments), 4)
        words = [mode_word(item) for item in assignments]
        self.assertEqual(len(set(words)), 4)
        self.assertEqual(words[0], ((0, ACKERMANN, 1), (1, ACKERMANN, 1)))
        for assignment in assignments:
            self.assertTrue(assignment.success)
            audit = audit_mode_assignment(assignment)
            self.assertTrue(audit.success, audit.detail)

    def test_composite_and_zero_length_are_rejected_fail_closed(self):
        try:
            TransitionPrimitive = primitive(COMPOSITE)
            block = FormationKinematicBlock.from_primitive(
                FormationPose(0.0, 0.0, 0.0), TransitionPrimitive,
                self.wedge)
        except FormationValidationError:
            self.fail("composite block construction should be representable")
        self.assertEqual(block.geometry_type, COMPOSITE)
        assignment = assign_formation_modes(
            block, {0: 0.0, 1: 0.0, 2: 0.0})
        self.assertFalse(assignment.success)
        self.assertEqual(assignment.reason, "COMPOSITE_BLOCK_UNSUPPORTED")
        self.assertFalse(audit_mode_assignment(assignment).success)

        with self.assertRaises(FormationValidationError):
            FormationKinematicBlock.from_primitive(
                FormationPose(0.0, 0.0, 0.0),
                primitive(0, dx=0.0, dy=0.0), self.wedge)
        with self.assertRaises(FormationValidationError):
            FormationKinematicBlock.from_primitive(
                FormationPose(0.0, 0.0, 0.0),
                primitive(1, dpsi=0.0, pivot_x=0.0, pivot_y=0.0),
                self.wedge)

    def test_status_never_claims_execution(self):
        block = rotation_block(math.pi / 2.0, self.wedge)
        assignment = assign_formation_modes(
            block, {0: 0.0, 1: 0.0, 2: 0.0}, allow_reverse=True)
        audit = audit_mode_assignment(assignment, allow_reverse=True)
        for status in (assignment.as_status(), audit.as_status()):
            self.assertFalse(status["executable"])
            self.assertEqual(status["certification"],
                             "KINEMATIC_CANDIDATE" if "schedule_index" in status
                             else "KINEMATIC_AUDITED")
            self.assertNotEqual(status.get("reason"), "FEASIBLE")


if __name__ == "__main__":
    unittest.main()