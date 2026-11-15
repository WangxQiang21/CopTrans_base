"""S9 independent formation evaluator, selection and publication tests.

The evaluator must recompute the request, trajectory, rigidity, collision,
kinematics, dynamics and publication sampling from raw inputs.  A successful
S8 timed trajectory is only an input candidate here; only a full independent
pass may produce ``FEASIBLE``.
"""
from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    FormationValidationError,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_geometry import (
    build_continuous_plans,
)
from fwis_v3_optimizer.formation_kinematics import KinematicLimits
from fwis_v3_optimizer.formation_quality import (
    FEASIBLE,
    GEOMETRY_ONLY_NOT_EXECUTABLE,
    GOAL_FORMATION_COLLISION,
    FORMATION_RETIMING_INFEASIBLE,
    FORMATION_RIGIDITY_VIOLATION,
    FORMATION_SWEEP_COLLISION,
    INVALID,
    INVALID_REQUEST,
    INVALID_TIMED_TRAJECTORY,
    KINEMATIC_INFEASIBLE,
    MODE_SCHEDULE_INFEASIBLE,
    REJECTED,
    PUBLICATION_SWEEP_INVALID,
    START_FORMATION_COLLISION,
    FormationPublicationGate,
    FormationQualityConfig,
    FormationTimedCandidate,
    evaluate_formation_trajectory,
    grid_signature,
    select_formation_candidate,
)
from fwis_v3_optimizer.formation_retiming import (
    FormationTimedTrajectory,
    FormationTimingLimits,
    retime_formation_plan,
)


def primitive(geometry_type, dx=0.0, dy=0.0, dpsi=0.0,
              pivot_x=0.0, pivot_y=0.0):
    return SimpleNamespace(
        geometry_type=geometry_type, dx=dx, dy=dy, dpsi=dpsi,
        pivot_x=pivot_x, pivot_y=pivot_y)


def header():
    return SimpleNamespace(frame_id="map",
                           stamp=SimpleNamespace(secs=123, nsecs=456))


def snapshot(offsets, start, goal, start_headings, goal_headings,
             free=None, allow_reverse=False, plan_id=17,
             formation_id="s9_test"):
    model = FormationModel(offsets)
    if free is None:
        free = {offset.vehicle_id: False for offset in offsets}
    return SimpleNamespace(
        plan_id=plan_id, formation_id=formation_id,
        header_stamp=(123, 456),
        header=header(),
        start=start, goal=goal, model=model,
        vehicle_shapes={offset.vehicle_id: VehicleShape(0.738, 0.500)
                        for offset in offsets},
        start_headings=dict(start_headings),
        goal_headings=dict(goal_headings),
        goal_heading_free=dict(free),
        allow_reverse=allow_reverse,
        vehicle_ids=model.vehicle_ids)


def seed_for(request, primitives):
    return SimpleNamespace(
        header=header(),
        plan_id=request.plan_id,
        formation_id=request.formation_id,
        formation_start=request.start,
        formation_goal=request.goal,
        primitives=tuple(primitives))


def build_plan(request, primitives):
    build = build_continuous_plans(
        request, seed_for(request, primitives), max_candidates=4)
    if not build.plans:
        raise AssertionError(
            "seed build failed: {} {}".format(build.reason, build.detail))
    return build.plans[0]


def retimed(plan, **kwargs):
    result = retime_formation_plan(plan, **kwargs)
    if result.timed is None:
        raise AssertionError(
            "retiming failed: {} {}".format(result.reason, result.detail))
    return result.timed


def empty_grid():
    # 8 m x 6 m map, origin (-2, -1.5); free space starts at x=0, y=0.
    return Grid(np.zeros((120, 160), dtype=bool), 0.05, -2.0, -1.5)


def straight_request(plan_id=17):
    offsets = (VehicleOffset(0, 0.0, 0.0), VehicleOffset(1, 1.2, 0.0))
    return snapshot(
        offsets, FormationPose(0.0, 0.0, 0.0), FormationPose(2.0, 0.0, 0.0),
        {0: 0.0, 1: 0.0}, {0: 0.0, 1: 0.0},
        plan_id=plan_id, formation_id="s9_straight")


def straight_plan(plan_id=17):
    request = straight_request(plan_id)
    return request, build_plan(request, (primitive(0, dx=2.0),))


def rotation_request(plan_id=17):
    offsets = (VehicleOffset(0, 0.0, 0.0), VehicleOffset(1, 1.2, 0.0))
    dpsi = 0.5
    return snapshot(
        offsets, FormationPose(0.0, 0.0, 0.0), FormationPose(0.0, 0.0, dpsi),
        {0: 0.0, 1: math.pi / 2.0},
        {0: dpsi, 1: math.pi / 2.0 + dpsi},
        plan_id=plan_id, formation_id="s9_rotation")


def rotation_plan(plan_id=17):
    request = rotation_request(plan_id)
    return request, build_plan(request, (primitive(1, dpsi=0.5),))


def cw_rotation_request(plan_id=17):
    offsets = (VehicleOffset(0, 0.0, 0.0), VehicleOffset(1, 1.2, 0.0))
    dpsi = -0.5
    return snapshot(
        offsets, FormationPose(0.0, 0.0, 0.0), FormationPose(0.0, 0.0, dpsi),
        {0: 0.0, 1: math.pi / 2.0},
        {0: dpsi, 1: math.pi / 2.0 + dpsi},
        allow_reverse=True, plan_id=plan_id,
        formation_id="s9_cw_rotation")


def cw_rotation_plan(plan_id=17):
    request = cw_rotation_request(plan_id)
    return request, build_plan(request, (primitive(1, dpsi=-0.5),))


def chain_request(plan_id=17):
    offsets = (VehicleOffset(0, 0.0, 0.0), VehicleOffset(1, 1.2, 0.0))
    dpsi = 0.3
    goal = FormationPose(2.0 * math.cos(dpsi), 2.0 * math.sin(dpsi), dpsi)
    return snapshot(
        offsets, FormationPose(0.0, 0.0, 0.0), goal,
        {0: 0.0, 1: math.pi / 2.0},
        {0: dpsi, 1: math.pi / 2.0 + dpsi},
        allow_reverse=True, plan_id=plan_id, formation_id="s9_chain")


def chain_plan(plan_id=17):
    request = chain_request(plan_id)
    return request, build_plan(request, (
        primitive(1, dpsi=0.3),
        primitive(0, dx=1.0, dy=0.3),
        primitive(0, dx=1.0, dy=-0.3)))


def raw_timed(timed, **changes):
    """Bypass the frozen constructor to model an externally corrupted message."""
    result = object.__new__(FormationTimedTrajectory)
    fields = ("plan_id", "formation_id", "offsets", "formation_poses",
              "times_from_start", "trajectories", "phases", "timing_report",
              "certification", "dynamic_profile")
    for name in fields:
        object.__setattr__(result, name, changes.get(
            name, getattr(timed, name)))
    return result


def tamper_point(timed, vehicle_id, index, **changes):
    trajectories = dict(timed.trajectories)
    points = list(trajectories[vehicle_id])
    points[index] = replace(points[index], **changes)
    trajectories[vehicle_id] = tuple(points)
    return replace(timed, trajectories=trajectories)


class FormationQualityTest(unittest.TestCase):

    def test_valid_straight_is_feasible_with_independent_metrics(self):
        request, plan = straight_plan()
        timed = retimed(plan)
        grid = empty_grid()
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, grid)
        self.assertTrue(evaluation.feasible, evaluation.detail)
        self.assertEqual(evaluation.status, FEASIBLE)
        self.assertIsNone(evaluation.failure)
        self.assertEqual(evaluation.certification, FEASIBLE)
        self.assertTrue(evaluation.executable)
        self.assertFalse(evaluation.trajectory_published)
        metrics = evaluation.metrics
        self.assertAlmostEqual(metrics["duration_s"], timed.duration,
                               places=9)
        self.assertEqual(metrics["sample_count"], len(timed.times_from_start))
        self.assertLess(metrics["max_rigidity_error_m"], 1e-9)
        self.assertGreater(metrics["minimum_obstacle_clearance_m"], 0.0)
        self.assertGreater(metrics["minimum_intervehicle_clearance_m"], 0.0)
        self.assertLessEqual(metrics["dynamic_peak_ratio"], 1.0 + 1e-6)
        self.assertEqual(metrics["map_signature"], grid_signature(grid))
        self.assertGreater(metrics["publication_sample_count"],
                           metrics["sample_count"])
        status = evaluation.as_status()
        self.assertEqual(status["status"], FEASIBLE)
        self.assertTrue(status["collision_certificate"])
        self.assertTrue(status["independent_audit"])
        self.assertFalse(status["trajectory_published"])

    def test_valid_rotation_and_chain_are_feasible(self):
        request, plan = rotation_plan()
        timed = retimed(plan)
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, empty_grid())
        self.assertTrue(evaluation.feasible, evaluation.detail)
        # The pivot spins in place; its center must stay fixed.
        pivot = timed.trajectories[0]
        self.assertLess(max(math.hypot(point.center_x, point.center_y)
                            for point in pivot), 1e-9)
        request, plan = chain_plan()
        timed = retimed(plan)
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, empty_grid())
        self.assertTrue(evaluation.feasible, evaluation.detail)
        self.assertEqual(evaluation.metrics["mode_switch_count"], 2)
        self.assertEqual(evaluation.metrics["direction_change_count"], 1)

    def test_clockwise_rotation_is_feasible_with_reverse_speeds(self):
        request, plan = cw_rotation_plan()
        timed = retimed(plan)
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, empty_grid())
        self.assertTrue(evaluation.feasible, evaluation.detail)
        follower = timed.trajectories[1]
        self.assertTrue(all(point.mode == 0 for point in follower))
        signed = [point.center_speed_mps for point in follower
                  if abs(point.center_speed_mps) > 1e-6]
        self.assertTrue(signed)
        # Clockwise rotation drags the tangent-aligned body backwards.
        self.assertTrue(all(value < 0.0 for value in signed))
        ys = [point.center_y for point in follower]
        self.assertLess(ys[-1], ys[0] - 0.5)
        for point in follower:
            self.assertGreaterEqual(point.wheel_speed_mps, 0.0)
        segment = plan.blocks[0].segment_map[1]
        self.assertEqual(segment.direction, -1)
        self.assertAlmostEqual(segment.control, 1.0 / 1.2, places=12)

    def test_tampered_reverse_direction_is_rejected_by_s9(self):
        request, plan = cw_rotation_plan()
        timed = retimed(plan)
        blocks = list(plan.blocks)
        segments = list(blocks[0].vehicle_mode_segments)
        index = next(index for index, segment in enumerate(segments)
                     if segment.vehicle_id == 1)
        segments[index] = replace(segments[index], direction=1)
        blocks[0] = replace(blocks[0], vehicle_mode_segments=tuple(segments))
        bad_plan = replace(plan, blocks=tuple(blocks))
        passing = SimpleNamespace(
            success=True, checks=0, reason=None, detail="",
            max_rigidity_error_m=0.0)
        with mock.patch(
                "fwis_v3_optimizer.formation_quality.audit_continuous_plan",
                return_value=passing):
            evaluation = evaluate_formation_trajectory(
                request, bad_plan, timed, empty_grid())
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.reason, KINEMATIC_INFEASIBLE)
        self.assertEqual(evaluation.failure.field, "direction")
        self.assertEqual(evaluation.failure.vehicle_id, 1)

    def test_tampered_rotation_curvature_is_rejected_by_s9(self):
        request, plan = cw_rotation_plan()
        timed = retimed(plan)
        blocks = list(plan.blocks)
        segments = list(blocks[0].vehicle_mode_segments)
        index = next(index for index, segment in enumerate(segments)
                     if segment.vehicle_id == 1)
        segments[index] = replace(
            segments[index], control=-segments[index].control)
        blocks[0] = replace(blocks[0], vehicle_mode_segments=tuple(segments))
        bad_plan = replace(plan, blocks=tuple(blocks))
        passing = SimpleNamespace(
            success=True, checks=0, reason=None, detail="",
            max_rigidity_error_m=0.0)
        with mock.patch(
                "fwis_v3_optimizer.formation_quality.audit_continuous_plan",
                return_value=passing):
            evaluation = evaluate_formation_trajectory(
                request, bad_plan, timed, empty_grid())
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.reason, KINEMATIC_INFEASIBLE)
        self.assertEqual(evaluation.failure.field, "curvature")
        self.assertEqual(evaluation.failure.vehicle_id, 1)

    def test_tampered_signed_speed_is_rejected_by_s9(self):
        request, plan = cw_rotation_plan()
        timed = retimed(plan)
        index = next(
            index for index, point in enumerate(timed.trajectories[1])
            if point.center_speed_mps < -1e-3)
        point = timed.trajectories[1][index]
        halved = tamper_point(
            timed, 1, index,
            center_speed_mps=0.5 * point.center_speed_mps)
        evaluation = evaluate_formation_trajectory(
            request, plan, halved, empty_grid())
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.reason, FORMATION_RETIMING_INFEASIBLE)
        self.assertEqual(evaluation.failure.field, "center_speed_mps")
        self.assertEqual(evaluation.failure.vehicle_id, 1)
        flipped = tamper_point(
            timed, 1, index,
            center_speed_mps=-point.center_speed_mps)
        evaluation = evaluate_formation_trajectory(
            request, plan, flipped, empty_grid())
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.reason, KINEMATIC_INFEASIBLE)
        self.assertEqual(evaluation.failure.field, "center_speed_mps")

    def test_tampered_center_is_rigidity_violation(self):
        request, plan = straight_plan()
        timed = retimed(plan)
        bad = tamper_point(timed, 1, 10, center_x=(
            timed.trajectories[1][10].center_x + 0.1))
        evaluation = evaluate_formation_trajectory(
            request, plan, bad, empty_grid())
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.status, REJECTED)
        self.assertEqual(evaluation.reason, FORMATION_RIGIDITY_VIOLATION)
        self.assertEqual(evaluation.failure.vehicle_id, 1)
        self.assertEqual(evaluation.failure.sample_index, 10)
        self.assertAlmostEqual(evaluation.failure.max_error, 0.1, places=9)

    def test_tampered_formation_pose_is_invalid_trajectory(self):
        request, plan = straight_plan()
        timed = retimed(plan)
        poses = list(timed.formation_poses)
        poses[20] = replace(poses[20], x=poses[20].x + 0.1)
        bad = replace(timed, formation_poses=tuple(poses))
        evaluation = evaluate_formation_trajectory(
            request, plan, bad, empty_grid())
        self.assertEqual(evaluation.status, INVALID)
        self.assertEqual(evaluation.reason, INVALID_TIMED_TRAJECTORY)
        self.assertEqual(evaluation.failure.field, "formation_poses")

    def test_tampered_heading_is_kinematically_infeasible(self):
        request, plan = straight_plan()
        timed = retimed(plan)
        bad = tamper_point(
            timed, 0, 20,
            theta=timed.trajectories[0][20].theta + 0.2)
        evaluation = evaluate_formation_trajectory(
            request, plan, bad, empty_grid())
        self.assertEqual(evaluation.reason, KINEMATIC_INFEASIBLE)
        self.assertEqual(evaluation.failure.field, "theta")
        self.assertEqual(evaluation.failure.vehicle_id, 0)

    def test_inconsistent_vehicle_time_axis_is_rejected(self):
        request, plan = straight_plan()
        timed = retimed(plan)
        points = list(timed.trajectories[0])
        points[-1] = replace(points[-1],
                             time_from_start=points[-1].time_from_start + 0.1)
        trajectories = dict(timed.trajectories)
        trajectories[0] = tuple(points)
        bad = raw_timed(timed, trajectories=trajectories)
        evaluation = evaluate_formation_trajectory(
            request, plan, bad, empty_grid())
        self.assertEqual(evaluation.reason, INVALID_TIMED_TRAJECTORY)
        self.assertEqual(evaluation.failure.field, "time_from_start")

    def test_nonmonotonic_shared_time_grid_is_rejected(self):
        request, plan = straight_plan()
        timed = retimed(plan)
        times = list(timed.times_from_start)
        times[5] = times[4]
        bad = raw_timed(timed, times_from_start=tuple(times))
        evaluation = evaluate_formation_trajectory(
            request, plan, bad, empty_grid())
        self.assertEqual(evaluation.reason, INVALID_TIMED_TRAJECTORY)
        self.assertEqual(evaluation.failure.field, "times_from_start")

    def test_spin_center_motion_and_oblique_heading_change_rejected(self):
        request, plan = rotation_plan()
        timed = retimed(plan)
        blocks = list(plan.blocks)
        segments = list(blocks[0].vehicle_mode_segments)
        segments[0] = replace(
            segments[0], end_center_y=segments[0].start_center_y + 0.1)
        blocks[0] = replace(blocks[0], vehicle_mode_segments=tuple(segments))
        bad_plan = replace(plan, blocks=tuple(blocks))
        evaluation = evaluate_formation_trajectory(
            request, bad_plan, timed, empty_grid())
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.reason, KINEMATIC_INFEASIBLE)

        request, plan = straight_plan()
        timed = retimed(plan)
        blocks = list(plan.blocks)
        segments = list(blocks[0].vehicle_mode_segments)
        segments[0] = replace(
            segments[0], mode=1, control=0.3,
            theta_end=segments[0].theta_start + 0.1)
        blocks[0] = replace(blocks[0], vehicle_mode_segments=tuple(segments))
        bad_plan = replace(plan, blocks=tuple(blocks))
        evaluation = evaluate_formation_trajectory(
            request, bad_plan, timed, empty_grid())
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.reason, KINEMATIC_INFEASIBLE)

    def test_ackermann_curvature_uses_independent_limits(self):
        request, plan = rotation_plan()
        timed = retimed(plan)
        config = FormationQualityConfig(
            kinematic_limits=KinematicLimits(r_min_ackermann=1.5))
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, empty_grid(), config)
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.reason, KINEMATIC_INFEASIBLE)

    def test_short_mode_dwell_is_mode_schedule_infeasible(self):
        request, plan = straight_plan()
        timed = retimed(plan)
        config = FormationQualityConfig(
            timing_limits=FormationTimingLimits(min_mode_dwell_s=10.0))
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, empty_grid(), config)
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.reason, MODE_SCHEDULE_INFEASIBLE)
        self.assertEqual(evaluation.failure.constraint, "mode_dwell")

    def test_short_hold_is_mode_schedule_infeasible(self):
        request, plan = chain_plan()
        timed = retimed(plan)
        config = FormationQualityConfig(
            timing_limits=FormationTimingLimits(mode_setup_time=10.0))
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, empty_grid(), config)
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.reason, MODE_SCHEDULE_INFEASIBLE)
        self.assertEqual(evaluation.failure.constraint,
                         "wheel_reconfiguration")

    def test_start_goal_and_sweep_collisions_have_distinct_codes(self):
        request, plan = straight_plan()
        timed = retimed(plan)
        grid = empty_grid()
        start_grid = empty_grid()
        start_grid.occ[30, 40] = True
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, start_grid)
        self.assertEqual(evaluation.reason, START_FORMATION_COLLISION)
        goal_grid = empty_grid()
        goal_grid.occ[30, 80] = True
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, goal_grid)
        self.assertEqual(evaluation.reason, GOAL_FORMATION_COLLISION)
        mid_grid = empty_grid()
        mid_grid.occ[30, 54] = True
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, mid_grid)
        self.assertEqual(evaluation.reason, FORMATION_SWEEP_COLLISION)
        self.assertIsNotNone(evaluation.failure.segment_index)

    def test_publication_sample_sweep_is_recomputed(self):
        request, plan = straight_plan()
        timed = retimed(plan)
        grid = empty_grid()
        # A coarse publication grid must not replace the continuous sweep.
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, grid,
            FormationQualityConfig(publication_dt_s=1.0))
        self.assertTrue(evaluation.feasible, evaluation.detail)
        self.assertEqual(evaluation.metrics["publication_dt_s"], 1.0)
        self.assertGreaterEqual(
            evaluation.metrics["publication_sample_count"],
            math.floor(timed.duration / 1.0) + 2)

    def test_selection_evaluates_all_and_uses_common_t_end(self):
        request, plan = straight_plan()
        fast = retimed(plan)
        slow = retimed(plan, limits=FormationTimingLimits(v_max=0.2))
        bad = tamper_point(
            fast, 1, 10,
            center_x=fast.trajectories[1][10].center_x + 0.1)
        selection = select_formation_candidate(
            request, (
                FormationTimedCandidate(1, plan, fast),
                FormationTimedCandidate(2, plan, slow),
                FormationTimedCandidate(3, plan, bad)),
            empty_grid())
        self.assertEqual(selection.status, FEASIBLE)
        self.assertEqual(selection.candidate_count, 3)
        self.assertEqual(selection.feasible_count, 2)
        self.assertEqual(selection.selected_candidate_id, 1)
        self.assertAlmostEqual(selection.selected_timed.duration,
                               fast.duration, places=9)
        self.assertEqual(len(selection.evaluations), 3)
        self.assertEqual([item.status for item in selection.evaluations],
                         [FEASIBLE, FEASIBLE, REJECTED])
        self.assertEqual(selection.rejections[0]["candidate_id"], 3)
        self.assertEqual(
            selection.rejections[0]["rejection_reason"],
            FORMATION_RIGIDITY_VIOLATION)
        status = selection.as_status()
        self.assertEqual(status["status"], FEASIBLE)
        self.assertTrue(status["executable"])
        self.assertFalse(status["trajectory_published"])

    def test_publication_gate_is_transactional(self):
        request, plan = straight_plan()
        timed = retimed(plan)
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, empty_grid())
        gate = FormationPublicationGate()
        decision = gate.begin_request(17)
        self.assertFalse(decision.publish)
        self.assertTrue(decision.clear)
        bad_evaluation = evaluate_formation_trajectory(
            request, plan, tamper_point(
                timed, 1, 10,
                center_x=timed.trajectories[1][10].center_x + 0.1),
            empty_grid())
        decision = gate.offer(17, timed, bad_evaluation)
        self.assertFalse(decision.publish)
        self.assertTrue(decision.clear)
        self.assertIsNone(gate.published_plan_id)
        decision = gate.offer(17, timed, evaluation)
        self.assertTrue(decision.publish)
        self.assertFalse(decision.clear)
        self.assertEqual(gate.published_plan_id, 17)
        self.assertIs(decision.trajectory, timed)

        decision = gate.begin_request(18)
        self.assertTrue(decision.clear)
        self.assertIsNone(gate.published_plan_id)
        decision = gate.offer(17, timed, evaluation)
        self.assertFalse(decision.publish)
        self.assertEqual(decision.status, "STALE_PLAN")
        self.assertIsNone(gate.published_plan_id)
        decision = gate.fail(17, "BAD", "old plan failure")
        self.assertFalse(decision.publish)
        self.assertFalse(decision.clear)
        self.assertIsNone(gate.published_plan_id)

    def test_map_signature_and_missing_map_fail_closed(self):
        request, plan = straight_plan()
        timed = retimed(plan)
        grid = empty_grid()
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, None)
        self.assertEqual(evaluation.status, INVALID)
        self.assertEqual(evaluation.reason, INVALID_REQUEST)
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, grid,
            FormationQualityConfig(expected_map_signature="0" * 64))
        self.assertEqual(evaluation.status, INVALID)
        self.assertEqual(evaluation.reason, INVALID_REQUEST)
        evaluation = evaluate_formation_trajectory(
            request, plan, timed, grid,
            FormationQualityConfig(expected_map_signature=grid_signature(grid)))
        self.assertTrue(evaluation.feasible, evaluation.detail)

    def test_invalid_inputs_and_candidates_fail_closed(self):
        request, plan = straight_plan()
        timed = retimed(plan)
        with self.assertRaises(FormationValidationError):
            evaluate_formation_trajectory(
                request, object(), timed, empty_grid())
        with self.assertRaises(FormationValidationError):
            evaluate_formation_trajectory(
                request, plan, object(), empty_grid())
        with self.assertRaises(FormationValidationError):
            evaluate_formation_trajectory(
                request, plan, timed, empty_grid(), object())
        with self.assertRaises(FormationValidationError):
            select_formation_candidate(request, (), empty_grid())
        with self.assertRaises(FormationValidationError):
            select_formation_candidate(
                request, (FormationTimedCandidate(1, plan, timed),
                          FormationTimedCandidate(1, plan, timed)),
                empty_grid())
        with self.assertRaises(FormationValidationError):
            FormationTimedCandidate(0, plan, timed)

    def test_nonfeasible_status_never_claims_feasible_or_certificate(self):
        request, plan = straight_plan()
        timed = retimed(plan)
        evaluations = (
            evaluate_formation_trajectory(
                request, plan, tamper_point(
                    timed, 1, 10,
                    center_x=timed.trajectories[1][10].center_x + 0.1),
                empty_grid()),
            evaluate_formation_trajectory(
                request, plan, timed, None),
            evaluate_formation_trajectory(
                request, plan, timed, empty_grid(),
                FormationQualityConfig(
                    timing_limits=FormationTimingLimits(
                        min_mode_dwell_s=10.0))),
        )
        for evaluation in evaluations:
            status = evaluation.as_status()
            self.assertFalse(status["feasible"])
            self.assertFalse(status["executable"])
            self.assertFalse(status["trajectory_published"])
            self.assertFalse(status["collision_certificate"])
            self.assertEqual(status["certification"], "NONE")
            self.assertNotEqual(status["status"], FEASIBLE)
            self.assertIsNotNone(status["failure"])
        self.assertEqual(evaluations[0].status, REJECTED)
        self.assertEqual(evaluations[1].status, INVALID)
        self.assertEqual(evaluations[2].status, REJECTED)


if __name__ == "__main__":
    unittest.main()