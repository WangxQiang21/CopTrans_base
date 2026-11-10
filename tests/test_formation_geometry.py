"""S6 continuous formation geometry, audit, collision precheck and master tests.

The tests use plain duck-typed snapshots/seeds and no ROS process.  They verify
only rigid-center reconstruction from ``q_F``, independent mode replay, dense
collision prechecking and fixed-K candidate management.  No test may claim
``FEASIBLE`` or executable timing.
"""
from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_geometry import (
    GEOMETRY_VALID,
    KINEMATIC_VALID,
    FormationContinuousPlan,
    FormationPathControl,
    audit_continuous_plan,
    build_continuous_plans,
    precheck_plan_collision,
)
from fwis_v3_optimizer.formation_kinematics import (
    ACKERMANN,
    OBLIQUE,
    SPIN,
)
from fwis_v3_optimizer.formation_master import (
    FormationMasterConfig,
    FormationModeMaster,
)


def primitive(geometry_type, dx=0.0, dy=0.0, dpsi=0.0,
              pivot_x=0.0, pivot_y=0.0):
    return SimpleNamespace(
        geometry_type=geometry_type, dx=dx, dy=dy, dpsi=dpsi,
        pivot_x=pivot_x, pivot_y=pivot_y)


def header():
    return SimpleNamespace(
        frame_id="map", stamp=SimpleNamespace(secs=123, nsecs=456))


def snapshot(offsets, start, goal, start_headings, goal_headings,
             free=None, allow_reverse=False, plan_id=17,
             formation_id="s6_test", shapes=None):
    model = FormationModel(offsets)
    if free is None:
        free = {offset.vehicle_id: False for offset in offsets}
    if shapes is None:
        shapes = {offset.vehicle_id: VehicleShape(0.738, 0.500)
                  for offset in offsets}
    return SimpleNamespace(
        plan_id=plan_id, formation_id=formation_id,
        start=start, goal=goal, model=model,
        vehicle_shapes=shapes,
        start_headings=dict(start_headings),
        goal_headings=dict(goal_headings),
        goal_heading_free=dict(free),
        allow_reverse=allow_reverse,
        vehicle_ids=model.vehicle_ids)


def seed_for(request, primitives, start=None, goal=None, plan_id=None,
             formation_id=None, map_signature=None):
    values = dict(
        header=header(),
        plan_id=request.plan_id if plan_id is None else plan_id,
        formation_id=(request.formation_id if formation_id is None
                      else formation_id),
        formation_start=request.start if start is None else start,
        formation_goal=request.goal if goal is None else goal,
        primitives=tuple(primitives))
    if map_signature is not None:
        values["map_signature"] = map_signature
    return SimpleNamespace(**values)


def straight_request(offsets, length=3.0, start=None):
    start = start or FormationPose(2.0, 3.0, 0.2)
    goal = FormationPose(
        start.x + length * math.cos(start.psi),
        start.y + length * math.sin(start.psi),
        start.psi)
    headings = {offset.vehicle_id: start.psi for offset in offsets}
    return snapshot(offsets, start, goal, headings, headings)


def straight_seed(request, length=3.0):
    return seed_for(
        request, [primitive(0, dx=length, dy=0.0)])


def plan_with_first_segment_change(plan, **changes):
    block = plan.blocks[0]
    segments = tuple(
        replace(segment, **changes) if segment.vehicle_id == 0 else segment
        for segment in block.vehicle_mode_segments)
    return replace(
        plan, blocks=(replace(block, vehicle_mode_segments=segments),)
        + plan.blocks[1:])


class FormationGeometryTests(unittest.TestCase):
    def setUp(self):
        self.offsets = (
            VehicleOffset(0, 0.0, 0.0),
            VehicleOffset(1, 0.0, 1.2),
        )

    def build_straight(self):
        request = straight_request(self.offsets)
        build = build_continuous_plans(
            request, straight_seed(request), max_candidates=4)
        self.assertTrue(build.success, build.reason + " " + build.detail)
        return request, build

    def test_formation_control_derivative_matches_finite_difference(self):
        translation = FormationPathControl(
            "translation", 2.0,
            FormationPose(1.0, 2.0, 0.4),
            FormationPose(1.0 + 2.0 * math.cos(0.4),
                          2.0 + 2.0 * math.sin(0.4), 0.4))
        start = FormationPose(6.0, 7.0, 0.3)
        pivot_x, pivot_y, dpsi = 0.5, 0.2, 0.7
        c, s = math.cos(start.psi), math.sin(start.psi)
        pivot_world = (start.x + c * pivot_x - s * pivot_y,
                       start.y + s * pivot_x + c * pivot_y)
        next_psi = start.psi + dpsi
        cn, sn = math.cos(next_psi), math.sin(next_psi)
        end = FormationPose(
            pivot_world[0] - cn * pivot_x + sn * pivot_y,
            pivot_world[1] - sn * pivot_x - cn * pivot_y,
            next_psi)
        rotation = FormationPathControl(
            "rotation", abs(dpsi), start, end, pivot_x, pivot_y)
        step = 1e-6
        for control, stations in ((translation, (0.2, 1.0, 1.8)),
                                  (rotation, (0.2, 0.35, 0.6))):
            for station in stations:
                before = control.qf_at(station - step)
                after = control.qf_at(station + step)
                numeric = (
                    (after.x - before.x) / (2.0 * step),
                    (after.y - before.y) / (2.0 * step),
                    (after.psi - before.psi) / (2.0 * step))
                exact = control.dqf_dlambda(station)
                for computed, expected in zip(numeric, exact):
                    self.assertAlmostEqual(computed, expected, places=6)

    def test_straight_translation_reconstructs_rigid_centers_and_audits(self):
        request, build = self.build_straight()
        self.assertEqual(build.certification, GEOMETRY_VALID)
        self.assertEqual(len(build.plans), 4)
        plan = build.plans[0]
        self.assertEqual(plan.certification, GEOMETRY_VALID)
        self.assertEqual(len(plan.blocks), 1)
        block = plan.blocks[0]
        self.assertEqual(block.geometry_type, "translation")
        self.assertAlmostEqual(block.parameter_length, 3.0)

        start_centers = request.model.centers(request.start)
        end_centers = request.model.centers(request.goal)
        for segment in block.vehicle_mode_segments:
            self.assertEqual(segment.mode, ACKERMANN)
            self.assertAlmostEqual(
                segment.end_center_x - segment.start_center_x,
                end_centers[segment.vehicle_id][0] -
                start_centers[segment.vehicle_id][0])
            self.assertAlmostEqual(
                segment.end_center_y - segment.start_center_y,
                end_centers[segment.vehicle_id][1] -
                start_centers[segment.vehicle_id][1])

        audit = audit_continuous_plan(plan, request)
        self.assertTrue(audit.success, audit.reason + " " + audit.detail)
        self.assertEqual(audit.certification, KINEMATIC_VALID)
        self.assertEqual(audit.block_count, 1)
        self.assertGreater(audit.sample_count, 0)
        self.assertLess(audit.max_rigidity_error_m, 1e-9)
        self.assertLess(audit.max_terminal_error_xy_m, 1e-9)
        self.assertFalse(audit.collision_prechecked)

    def test_terminal_translation_fit_changes_only_last_endpoint(self):
        request = straight_request(self.offsets, length=3.0)
        seed = seed_for(request, [primitive(0, dx=2.8, dy=0.0)])
        build = build_continuous_plans(request, seed)
        self.assertTrue(build.success, build.reason + " " + build.detail)
        plan = build.plans[0]
        endpoint = plan.blocks[-1].control.qf_at(
            plan.blocks[-1].parameter_length)
        self.assertAlmostEqual(endpoint.x, request.goal.x, places=12)
        self.assertAlmostEqual(endpoint.y, request.goal.y, places=12)
        self.assertAlmostEqual(endpoint.psi, request.goal.psi, places=12)
        self.assertAlmostEqual(plan.blocks[-1].parameter_length, 3.0)
        self.assertTrue(audit_continuous_plan(plan, request).success)

    def test_reverse_translation_is_signed_and_policy_checked(self):
        start = FormationPose(2.0, 3.0, 0.3)
        goal = FormationPose(
            start.x - 2.0 * math.cos(start.psi),
            start.y - 2.0 * math.sin(start.psi),
            start.psi)
        headings = {0: start.psi, 1: start.psi}
        blocked = snapshot(
            self.offsets, start, goal, headings, headings,
            allow_reverse=False)
        build = build_continuous_plans(
            blocked, seed_for(blocked, [primitive(0, dx=-2.0)]))
        self.assertFalse(build.success)
        self.assertEqual(build.reason, "REVERSE_NOT_ALLOWED")

        allowed = snapshot(
            self.offsets, start, goal, headings, headings,
            allow_reverse=True)
        build = build_continuous_plans(
            allowed, seed_for(allowed, [primitive(0, dx=-2.0)]))
        self.assertTrue(build.success, build.reason + " " + build.detail)
        plan = build.plans[0]
        for segment in plan.blocks[0].vehicle_mode_segments:
            self.assertEqual(segment.mode, ACKERMANN)
            self.assertEqual(segment.direction, -1)
            self.assertAlmostEqual(
                segment.end_center_x - segment.start_center_x,
                -2.0 * math.cos(start.psi))
            self.assertAlmostEqual(
                segment.end_center_y - segment.start_center_y,
                -2.0 * math.sin(start.psi))
        audit = audit_continuous_plan(plan, allowed)
        self.assertTrue(audit.success, audit.reason + " " + audit.detail)

    def test_pure_rotation_uses_spin_pivot_and_ackermann_ring(self):
        offsets = (VehicleOffset(0, 0.0, 0.0),
                   VehicleOffset(1, 1.2, 0.0))
        start = FormationPose(6.0, 7.0, 0.0)
        goal = FormationPose(6.0, 7.0, math.pi / 2.0)
        request = snapshot(
            offsets, start, goal,
            start_headings={0: 0.0, 1: math.pi / 2.0},
            goal_headings={0: math.pi / 2.0, 1: math.pi})
        seed = seed_for(
            request, [primitive(1, dpsi=math.pi / 2.0,
                                pivot_x=0.0, pivot_y=0.0)])
        build = build_continuous_plans(request, seed)
        self.assertTrue(build.success, build.reason + " " + build.detail)
        plan = build.plans[0]
        modes = {segment.vehicle_id: segment.mode
                 for segment in plan.blocks[0].vehicle_mode_segments}
        self.assertEqual(modes, {0: SPIN, 1: ACKERMANN})
        pivot = plan.blocks[0].segment_map[0]
        self.assertEqual(pivot.travel_distance, 0.0)
        self.assertEqual(pivot.start_center_x, request.model.centers(
            request.start)[0][0])
        self.assertEqual(pivot.start_center_y, request.model.centers(
            request.start)[0][1])
        self.assertEqual(pivot.end_center_x, pivot.start_center_x)
        self.assertEqual(pivot.end_center_y, pivot.start_center_y)
        self.assertAlmostEqual(
            plan.blocks[0].segment_map[1].theta_end, math.pi)
        audit = audit_continuous_plan(plan, request)
        self.assertTrue(audit.success, audit.reason + " " + audit.detail)

    def test_terminal_rotation_fit_extends_dpsi_without_moving_centers(self):
        offsets = (VehicleOffset(0, 0.0, 0.0),
                   VehicleOffset(1, 1.2, 0.0))
        start = FormationPose(6.0, 7.0, 0.0)
        goal = FormationPose(6.0, 7.0, math.pi / 2.0)
        request = snapshot(
            offsets, start, goal,
            start_headings={0: 0.0, 1: math.pi / 2.0},
            goal_headings={0: math.pi / 2.0, 1: math.pi})
        seed = seed_for(
            request, [primitive(1, dpsi=1.2, pivot_x=0.0, pivot_y=0.0)])
        build = build_continuous_plans(request, seed)
        self.assertTrue(build.success, build.reason + " " + build.detail)
        plan = build.plans[0]
        self.assertAlmostEqual(
            plan.blocks[-1].parameter_length, math.pi / 2.0)
        end_centers = request.model.centers(goal)
        for segment in plan.blocks[-1].vehicle_mode_segments:
            self.assertAlmostEqual(
                segment.end_center_x, end_centers[segment.vehicle_id][0])
            self.assertAlmostEqual(
                segment.end_center_y, end_centers[segment.vehicle_id][1])
        audit = audit_continuous_plan(plan, request)
        self.assertTrue(audit.success, audit.reason + " " + audit.detail)

    def test_two_block_chain_propagates_body_headings(self):
        offsets = (VehicleOffset(0, 0.0, 0.0),
                   VehicleOffset(1, 1.2, 0.0))
        start = FormationPose(0.0, 0.0, 0.0)
        goal = FormationPose(0.0, 1.0, math.pi / 2.0)
        request = snapshot(
            offsets, start, goal,
            start_headings={0: 0.0, 1: math.pi / 2.0},
            goal_headings={0: math.pi / 2.0, 1: math.pi})
        seed = seed_for(request, [
            primitive(1, dpsi=math.pi / 2.0, pivot_x=0.0, pivot_y=0.0),
            primitive(0, dx=1.0, dy=0.0),
        ])
        build = build_continuous_plans(request, seed)
        self.assertTrue(build.success, build.reason + " " + build.detail)
        plan = build.plans[0]
        self.assertEqual(len(plan.blocks), 2)
        first = plan.blocks[0]
        second = plan.blocks[1]
        self.assertAlmostEqual(first.control.delta_psi, math.pi / 2.0)
        self.assertEqual(first.segment_map[0].mode, SPIN)
        self.assertEqual(second.segment_map[0].mode, ACKERMANN)
        self.assertEqual(second.segment_map[1].mode, OBLIQUE)
        self.assertAlmostEqual(
            second.segment_map[0].theta_start, math.pi / 2.0)
        self.assertAlmostEqual(
            second.segment_map[1].theta_start, math.pi)
        self.assertAlmostEqual(
            second.segment_map[1].control, -math.pi / 2.0)
        audit = audit_continuous_plan(plan, request)
        self.assertTrue(audit.success, audit.reason + " " + audit.detail)
        self.assertEqual(audit.block_count, 2)

        tampered = replace(
            plan,
            blocks=(plan.blocks[0],
                    replace(
                        plan.blocks[1],
                        vehicle_mode_segments=tuple(
                            replace(segment, theta_start=math.pi / 2.0)
                            if segment.vehicle_id == 1 else segment
                            for segment in
                            plan.blocks[1].vehicle_mode_segments))))
        bad = audit_continuous_plan(tampered, request)
        self.assertFalse(bad.success)
        self.assertIn(bad.reason, (
            "HEADING_CONTINUITY_VIOLATION",
            "BLOCK_HEADING_BOUNDARY_MISMATCH",
            "MODE_DISPLACEMENT_MISMATCH"))

    def test_goal_heading_free_is_explicit_and_does_not_move_centers(self):
        offsets = (VehicleOffset(0, 0.0, 0.0),
                   VehicleOffset(1, 0.0, 1.2))
        start = FormationPose(1.0, 2.0, 0.0)
        goal = FormationPose(3.0, 2.0, 0.0)
        blocked = snapshot(
            offsets, start, goal,
            start_headings={0: 0.0, 1: 0.2},
            goal_headings={0: 0.0, 1: 0.0})
        seed = seed_for(
            blocked, [primitive(0, dx=2.0, dy=0.0)])
        failed = build_continuous_plans(blocked, seed)
        self.assertFalse(failed.success)
        self.assertEqual(failed.reason, "GOAL_HEADING_MISMATCH")

        freed = snapshot(
            offsets, start, goal,
            start_headings={0: 0.0, 1: 0.2},
            goal_headings={0: 0.0, 1: 0.0},
            free={0: False, 1: True})
        build = build_continuous_plans(
            freed, seed_for(freed, [primitive(0, dx=2.0, dy=0.0)]))
        self.assertTrue(build.success, build.reason + " " + build.detail)
        plan = build.plans[0]
        endpoint = plan.blocks[-1].control.qf_at(
            plan.blocks[-1].parameter_length)
        self.assertAlmostEqual(endpoint.x, goal.x)
        self.assertAlmostEqual(endpoint.y, goal.y)
        self.assertAlmostEqual(endpoint.psi, goal.psi)
        # The freed vehicle still reaches the exact rigid center; only its
        # terminal body yaw is exempt from the explicit goal heading.
        end_centers = freed.model.centers(goal)
        segment = plan.blocks[-1].segment_map[1]
        self.assertAlmostEqual(segment.end_center_x, end_centers[1][0])
        self.assertAlmostEqual(segment.end_center_y, end_centers[1][1])
        audit = audit_continuous_plan(plan, freed)
        self.assertTrue(audit.success, audit.reason + " " + audit.detail)

    def test_composite_and_seed_identity_fail_closed(self):
        request, _ = self.build_straight()
        composite = seed_for(
            request, [primitive(2, dx=0.0, dy=0.0, dpsi=0.0)])
        build = build_continuous_plans(request, composite)
        self.assertFalse(build.success)
        self.assertEqual(build.reason, "COMPOSITE_PRIMITIVE_UNSUPPORTED")

        wrong_id = seed_for(
            request, [primitive(0, dx=3.0)], plan_id=request.plan_id + 1)
        build = build_continuous_plans(request, wrong_id)
        self.assertFalse(build.success)
        self.assertEqual(build.reason, "INVALID_SEED")
        self.assertIn("SEED_PLAN_ID_MISMATCH", build.detail)

        wrong_start = seed_for(
            request, [primitive(0, dx=3.0)],
            start=FormationPose(request.start.x + 0.01,
                                request.start.y, request.start.psi))
        build = build_continuous_plans(request, wrong_start)
        self.assertFalse(build.success)
        self.assertIn("SEED_FORMATION_START_MISMATCH", build.detail)

    def test_map_version_is_explicitly_unchecked_when_seed_has_no_signature(self):
        request = straight_request(self.offsets)
        without = build_continuous_plans(
            request, straight_seed(request),
            expected_map_signature="map-v1")
        self.assertTrue(without.success, without.detail)
        self.assertFalse(without.map_version_checked)
        self.assertIn("map_version_checked=False", without.detail)

        mismatch = seed_for(
            request, [primitive(0, dx=3.0)], map_signature="map-v2")
        rejected = build_continuous_plans(
            request, mismatch, expected_map_signature="map-v1")
        self.assertFalse(rejected.success)
        self.assertIn("MAP_VERSION_MISMATCH", rejected.detail)

        matched = seed_for(
            request, [primitive(0, dx=3.0)], map_signature="map-v1")
        checked = build_continuous_plans(
            request, matched, expected_map_signature="map-v1")
        self.assertTrue(checked.success, checked.detail)
        self.assertTrue(checked.map_version_checked)

    def test_audit_rejects_tampered_center_heading_and_modes(self):
        request, build = self.build_straight()
        good = build.plans[0]
        self.assertTrue(audit_continuous_plan(good, request).success)

        moved = plan_with_first_segment_change(
            good, end_center_x=good.blocks[0].segment_map[0].end_center_x
            + 0.01)
        bad_center = audit_continuous_plan(moved, request)
        self.assertFalse(bad_center.success)
        self.assertEqual(bad_center.reason, "CONSTRUCTION_MISMATCH")

        turned = plan_with_first_segment_change(
            good, theta_end=good.blocks[0].segment_map[0].theta_end + 0.1)
        bad_heading = audit_continuous_plan(turned, request)
        self.assertFalse(bad_heading.success)
        self.assertEqual(
            bad_heading.reason, "HEADING_CONTINUITY_VIOLATION")

        spun = plan_with_first_segment_change(
            good, mode=SPIN, travel_distance=0.0)
        bad_mode = audit_continuous_plan(spun, request)
        self.assertFalse(bad_mode.success)
        self.assertIn(bad_mode.reason, (
            "MODE_DISPLACEMENT_MISMATCH", "SPIN_CENTER_MOTION",
            "NON_PIVOT_SPIN", "NONSTATIONARY_SPIN"))

    def test_dense_collision_precheck_rejects_mid_path_obstacle(self):
        request, build = self.build_straight()
        plan = build.plans[0]
        empty = Grid(
            occupied=[[False] * 100 for _ in range(100)],
            resolution=0.1, origin_x=0.0, origin_y=0.0)
        clean = precheck_plan_collision(plan, request, empty, margin_m=0.05)
        self.assertTrue(clean.success, clean.report.reason)
        self.assertGreater(clean.samples, 10)

        occupied = [[False] * 100 for _ in range(100)]
        # Vehicle 1 crosses roughly (3.23, 4.47) halfway through the 3 m
        # formation translation; the obstacle is outside both endpoints.
        for ix in range(30, 35):
            for iy in range(43, 47):
                occupied[iy][ix] = True
        blocked = Grid(
            occupied=occupied, resolution=0.1,
            origin_x=0.0, origin_y=0.0)
        precheck = precheck_plan_collision(
            plan, request, blocked, margin_m=0.05)
        self.assertFalse(precheck.success)
        self.assertIn(precheck.report.reason, (
            "MAP_OBB_COLLISION", "VEHICLE_OBB_COLLISION"))
        audit = audit_continuous_plan(
            plan, request, grid=blocked, margin_m=0.05)
        self.assertFalse(audit.success)
        self.assertEqual(audit.reason, "COLLISION_PRECHECK_FAILED")
        self.assertTrue(audit.collision_prechecked)
        self.assertEqual(audit.certification, "REJECTED")

    def test_master_retains_fixed_k_deduplicates_and_reports_rejections(self):
        request, build = self.build_straight()
        config = FormationMasterConfig(
            max_candidates=2, max_schedule_combinations=16,
            max_schedules_per_block=4, sweep_spacing_m=0.02,
            max_samples_per_block=20000)
        master = FormationModeMaster(config)
        admitted = master.consider_build(build, request, source="seed")
        self.assertEqual(len(admitted), 2)
        result = master.result()
        self.assertEqual(result.status, KINEMATIC_VALID)
        self.assertEqual(result.certification, KINEMATIC_VALID)
        self.assertIsNotNone(result.selected)
        self.assertEqual(len(result.candidates), 2)
        for candidate in result.candidates:
            self.assertEqual(candidate.plan.certification, KINEMATIC_VALID)
            self.assertTrue(candidate.audit.success)
            self.assertFalse(candidate.as_status()["executable"])
        self.assertTrue(any(
            rejection.reason == "CANDIDATE_LIMIT_REACHED"
            for rejection in result.rejections))

        duplicate = master.consider(
            build.plans[0], request, source="duplicate")
        self.assertIsNone(duplicate)
        self.assertEqual(master.result().duplicates, 1)
        self.assertTrue(any(
            rejection.reason == "DUPLICATE_CANDIDATE"
            for rejection in master.result().rejections))

    def test_master_never_admits_failed_audit_or_claims_feasible(self):
        request, build = self.build_straight()
        broken = plan_with_first_segment_change(
            build.plans[0],
            end_center_x=build.plans[0].blocks[0].segment_map[0]
            .end_center_x + 0.1)
        master = FormationModeMaster(FormationMasterConfig(max_candidates=2))
        candidate = master.consider(broken, request, source="broken")
        self.assertIsNone(candidate)
        result = master.result()
        self.assertEqual(result.status, "REJECTED")
        self.assertEqual(result.certification, "NONE")
        self.assertIsNone(result.selected)
        self.assertTrue(any(
            rejection.reason == "CONSTRUCTION_MISMATCH"
            for rejection in result.rejections))
        status = result.as_status()
        self.assertFalse(status["executable"])
        self.assertFalse(status["trajectory_published"])
        self.assertNotEqual(status["status"], "FEASIBLE")

    def test_status_is_geometry_or_kinematic_only(self):
        request, build = self.build_straight()
        plan = build.plans[0]
        geometry_status = plan.as_status()
        self.assertEqual(geometry_status["certification"], GEOMETRY_VALID)
        self.assertTrue(geometry_status["geometry_only"])
        self.assertFalse(geometry_status["executable"])
        audit = audit_continuous_plan(plan, request)
        audit_status = audit.as_status()
        self.assertEqual(audit_status["certification"], KINEMATIC_VALID)
        self.assertFalse(audit_status["executable"])
        self.assertFalse(audit_status["independent_timing_audit"])
        self.assertNotIn("FEASIBLE", str(build.as_status()))
        self.assertNotIn("FEASIBLE", str(audit_status))


if __name__ == "__main__":
    unittest.main()