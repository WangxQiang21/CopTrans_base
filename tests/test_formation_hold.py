"""Zero-motion hold contract: builder, independent audit and seed detection.

The hold is the only successful representation of a request whose start and
goal formation poses are identical.  These tests tamper every safety-relevant
field and require the independent auditor to reject it; they also prove that a
normal empty seed with a different goal is not misclassified as a hold.
"""
from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))

from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_hold import (
    NO_MOTION,
    build_hold_timed_trajectory,
    evaluate_formation_hold,
    zero_motion_seed,
)
from fwis_v3_optimizer.formation_quality import (
    FEASIBLE,
    FormationQualityConfig,
    grid_signature,
)
from fwis_v3_optimizer.formation_request import validate_formation_request

OFFSETS = (
    VehicleOffset(1, 1.0, 0.0),
    VehicleOffset(2, 0.0, 1.0),
    VehicleOffset(3, 0.0, -1.0),
)
START = (10.0, 10.0, 0.0)


def pose(x, y, theta=0.0):
    return SimpleNamespace(x=x, y=y, theta=theta)


def header(secs=123, nsecs=456):
    return SimpleNamespace(
        frame_id="map", stamp=SimpleNamespace(secs=secs, nsecs=nsecs))


def vehicle_spec(vehicle_id, offset_x, offset_y, start, goal):
    return SimpleNamespace(
        vehicle_id=vehicle_id,
        offset_in_formation=SimpleNamespace(
            x=offset_x, y=offset_y, z=0.0),
        start_center=pose(*start),
        goal_center=pose(*goal),
        goal_heading_free=True,
        body_length_m=0.738,
        body_width_m=0.500)


def request_message(start=START, goal=START, plan_id=81):
    model = FormationModel(OFFSETS)
    start_centers = model.centers(FormationPose(*start))
    goal_centers = model.centers(FormationPose(*goal))
    vehicles = []
    for offset in OFFSETS:
        vehicles.append(vehicle_spec(
            offset.vehicle_id, offset.x, offset.y,
            (start_centers[offset.vehicle_id][0],
             start_centers[offset.vehicle_id][1], 0.0),
            (goal_centers[offset.vehicle_id][0],
             goal_centers[offset.vehicle_id][1], 0.0)))
    return SimpleNamespace(
        header=header(), plan_id=plan_id, formation_id="hold_test",
        formation_start=pose(*start), formation_goal=pose(*goal),
        vehicles=vehicles, allow_reverse=True)


def seed_message(request, primitives=()):
    return SimpleNamespace(
        header=header(), plan_id=request.plan_id,
        formation_id=request.formation_id,
        formation_start=request.formation_start,
        formation_goal=request.formation_goal,
        primitives=tuple(primitives))


def empty_grid(width=600, height=600, resolution=0.05):
    return Grid(np.zeros((height, width), dtype=bool), resolution, 0.0, 0.0)


def config(**overrides):
    values = {
        "collision_margin_m": 0.05,
        "sweep_spacing_m": 0.01,
        "publication_dt_s": 0.02,
    }
    values.update(overrides)
    return FormationQualityConfig(**values)


class ZeroMotionHoldTest(unittest.TestCase):

    def setUp(self):
        self.request = request_message()
        self.snapshot = validate_formation_request(self.request).snapshot
        self.grid = empty_grid()
        self.timed = build_hold_timed_trajectory(self.snapshot)

    def test_static_wedge_hold_is_feasible_and_strictly_static(self):
        evaluation = evaluate_formation_hold(
            self.snapshot, self.timed, self.grid, config())
        self.assertTrue(evaluation.feasible, evaluation.detail)
        self.assertEqual(evaluation.status, FEASIBLE)
        self.assertEqual(evaluation.reason, NO_MOTION)
        self.assertGreater(evaluation.checks, 0)
        self.assertEqual(self.timed.times_from_start, (0.0,))
        self.assertEqual(self.timed.duration, 0.0)
        model = FormationModel(OFFSETS)
        centers = model.centers(FormationPose(*START))
        for vehicle_id in (1, 2, 3):
            points = self.timed.trajectories[vehicle_id]
            self.assertEqual(len(points), 1)
            point = points[0]
            self.assertTrue(point.stationary)
            self.assertEqual(point.center_speed_mps, 0.0)
            self.assertEqual(point.center_yaw_rate_radps, 0.0)
            self.assertEqual(point.wheel_speed_mps, 0.0)
            self.assertEqual(point.mode, 0)
            self.assertEqual(point.mode_control, 0.0)
            self.assertAlmostEqual(point.center_x, centers[vehicle_id][0])
            self.assertAlmostEqual(point.center_y, centers[vehicle_id][1])
            self.assertAlmostEqual(point.theta, 0.0)
        self.assertTrue(evaluation.metrics["no_motion"])

    def test_hold_rejects_nonzero_center_speed(self):
        points = tuple(self.timed.trajectories[1])
        tampered = replace(
            self.timed, trajectories={
                **self.timed.trajectories,
                1: (replace(points[0], center_speed_mps=0.1),)})
        evaluation = evaluate_formation_hold(
            self.snapshot, tampered, self.grid, config())
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.failure.field, "center_speed_mps")
        self.assertEqual(evaluation.failure.vehicle_id, 1)

    def test_hold_rejects_shifted_center(self):
        points = tuple(self.timed.trajectories[2])
        tampered = replace(
            self.timed, trajectories={
                **self.timed.trajectories,
                2: (replace(points[0], center_y=points[0].center_y + 0.01),)})
        evaluation = evaluate_formation_hold(
            self.snapshot, tampered, self.grid, config())
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.failure.code,
                         "FORMATION_RIGIDITY_VIOLATION")
        self.assertEqual(evaluation.failure.field, "center")

    def test_hold_rejects_wrong_body_heading(self):
        points = tuple(self.timed.trajectories[3])
        tampered = replace(
            self.timed, trajectories={
                **self.timed.trajectories,
                3: (replace(points[0], theta=math.pi / 2.0),)})
        evaluation = evaluate_formation_hold(
            self.snapshot, tampered, self.grid, config())
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.failure.field, "theta")

    def test_hold_rejects_fake_spin_mode(self):
        points = tuple(self.timed.trajectories[1])
        tampered = replace(
            self.timed, trajectories={
                **self.timed.trajectories,
                1: (replace(points[0], mode=2, mode_control=0.5),)})
        evaluation = evaluate_formation_hold(
            self.snapshot, tampered, self.grid, config())
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.failure.code, "KINEMATIC_INFEASIBLE")
        self.assertEqual(evaluation.failure.field, "mode")

    def test_hold_rejects_multiple_samples(self):
        tampered = replace(
            self.timed,
            formation_poses=(self.timed.formation_poses[0],
                             self.timed.formation_poses[0]),
            times_from_start=(0.0, 1.0),
            trajectories={
                vehicle_id: (points[0], replace(points[0], time_from_start=1.0))
                for vehicle_id, points in self.timed.trajectories.items()})
        evaluation = evaluate_formation_hold(
            self.snapshot, tampered, self.grid, config())
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.failure.field, "times_from_start")

    def test_hold_rejects_map_signature_mismatch(self):
        signature = grid_signature(self.grid)
        evaluation = evaluate_formation_hold(
            self.snapshot, self.timed, self.grid,
            config(expected_map_signature="0" * 64))
        self.assertFalse(evaluation.feasible)
        self.assertEqual(evaluation.failure.field, "map_signature")
        self.assertNotEqual(signature, "0" * 64)

    def test_hold_rejects_colliding_start_pose(self):
        grid = empty_grid()
        grid.occ[200, 220] = True  # x=11.0, y=10.0: vehicle 1 centre
        evaluation = evaluate_formation_hold(
            self.snapshot, self.timed, grid, config())
        self.assertFalse(evaluation.feasible)
        self.assertIn(evaluation.failure.code,
                      ("START_FORMATION_COLLISION", "MAP_OBB_COLLISION"))
        self.assertFalse(evaluation.feasible)


class ZeroMotionSeedTest(unittest.TestCase):

    def setUp(self):
        self.request = request_message()
        self.snapshot = validate_formation_request(self.request).snapshot

    def test_empty_seed_is_a_zero_motion_candidate(self):
        report = zero_motion_seed(
            self.snapshot, seed_message(self.request))
        self.assertIsNotNone(report)
        self.assertEqual(report["primitives"], ())

    def test_zero_length_primitives_are_a_zero_motion_candidate(self):
        primitives = (
            SimpleNamespace(geometry_type=0, dx=0.0, dy=0.0, dpsi=0.0,
                            pivot_x=0.0, pivot_y=0.0),
            SimpleNamespace(geometry_type=1, dx=0.0, dy=0.0, dpsi=0.0,
                            pivot_x=0.0, pivot_y=0.0),
        )
        report = zero_motion_seed(
            self.snapshot, seed_message(self.request, primitives))
        self.assertIsNotNone(report)

    def test_nonzero_primitive_is_not_a_hold(self):
        primitive = SimpleNamespace(
            geometry_type=0, dx=0.5, dy=0.0, dpsi=0.0,
            pivot_x=0.0, pivot_y=0.0)
        self.assertIsNone(zero_motion_seed(
            self.snapshot, seed_message(self.request, (primitive,))))

    def test_distinct_goal_is_not_a_hold(self):
        other = request_message(goal=(13.0, 10.0, 0.0))
        snapshot = validate_formation_request(other).snapshot
        self.assertIsNone(zero_motion_seed(
            snapshot, seed_message(other)))

    def test_seed_identity_mismatch_is_not_a_hold(self):
        seed = seed_message(self.request)
        seed.plan_id = 999
        self.assertIsNone(zero_motion_seed(self.snapshot, seed))

    def test_seed_start_mismatch_is_not_a_hold(self):
        seed = seed_message(self.request)
        seed.formation_start = pose(10.5, 10.0, 0.0)
        self.assertIsNone(zero_motion_seed(self.snapshot, seed))


if __name__ == "__main__":
    unittest.main()