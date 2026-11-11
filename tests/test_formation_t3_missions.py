"""T3 mission acceptance: E00-E04 through S6 build, S8 retiming, S9 audit.

This is the ROS-independent half of the T3 acceptance.  It pins the exact
mission table used by the real-entry smoke test:

* E00: start == goal, zero motion, audited static hold (``NO_MOTION``);
* E01: straight +x by 3 m;
* E02: lateral +y by 3 m;
* E03: oblique (+3, +2) m;
* E04: reverse -x by 3 m with ``allow_reverse``.

Every case starts at formation pose ``(10, 10, 0)`` with all three body
headings equal to zero, free goal headings, the rigid wedge offsets
``(1,0)``, ``(0,1)``, ``(0,-1)`` and the unmodified default kinematic,
timing, collision and sampling limits.  The tests require that the mode
assignment (S5), signed speeds (S8) and the independent evaluator (S9) agree
with the mission geometry, and that the goal is reached without moving the
body headings for the lateral and oblique cases.
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
from fwis_v3_optimizer.formation_geometry import build_continuous_plans
from fwis_v3_optimizer.formation_hold import (
    NO_MOTION,
    build_hold_timed_trajectory,
    evaluate_formation_hold,
    zero_motion_seed,
)
from fwis_v3_optimizer.formation_quality import (
    FEASIBLE,
    FormationQualityConfig,
    FormationTimedCandidate,
    select_formation_candidate,
)
from fwis_v3_optimizer.formation_request import validate_formation_request
from fwis_v3_optimizer.formation_retiming import (
    ACKERMANN,
    OBLIQUE,
    FormationTimingLimits,
    retime_formation_plan,
)

OFFSETS = (
    VehicleOffset(1, 1.0, 0.0),
    VehicleOffset(2, 0.0, 1.0),
    VehicleOffset(3, 0.0, -1.0),
)
START = (10.0, 10.0, 0.0)
MISSIONS = {
    "E01_straight": (13.0, 10.0, 0.0),
    "E02_lateral": (10.0, 13.0, 0.0),
    "E03_oblique": (13.0, 12.0, 0.0),
    "E04_reverse": (7.0, 10.0, 0.0),
}
GEOMETRY_TOL = 1.0e-6


def pose(x, y, theta=0.0):
    return SimpleNamespace(x=x, y=y, theta=theta)


def header(secs=700, nsecs=123):
    return SimpleNamespace(
        frame_id="map", stamp=SimpleNamespace(secs=secs, nsecs=nsecs))


def make_request(goal, plan_id=7000):
    model = FormationModel(OFFSETS)
    start_centers = model.centers(FormationPose(*START))
    goal_centers = model.centers(FormationPose(*goal))
    vehicles = []
    for offset in OFFSETS:
        vehicles.append(SimpleNamespace(
            vehicle_id=offset.vehicle_id,
            offset_in_formation=SimpleNamespace(
                x=offset.x, y=offset.y, z=0.0),
            start_center=pose(*start_centers[offset.vehicle_id], 0.0),
            goal_center=pose(*goal_centers[offset.vehicle_id], 0.0),
            goal_heading_free=True,
            body_length_m=0.738,
            body_width_m=0.500))
    return SimpleNamespace(
        header=header(), plan_id=plan_id, formation_id="wedge_3",
        formation_start=pose(*START), formation_goal=pose(*goal),
        vehicles=vehicles, allow_reverse=True)


def seed_for(request):
    return SimpleNamespace(
        header=header(), plan_id=request.plan_id,
        formation_id=request.formation_id,
        formation_start=request.formation_start,
        formation_goal=request.formation_goal,
        primitives=(SimpleNamespace(
            geometry_type=0,
            dx=request.formation_goal.x - request.formation_start.x,
            dy=request.formation_goal.y - request.formation_start.y,
            dpsi=0.0, pivot_x=0.0, pivot_y=0.0),))


def empty_grid(width=600, height=600, resolution=0.05):
    return Grid(np.zeros((height, width), dtype=bool), resolution, 0.0, 0.0)


class T3MissionTest(unittest.TestCase):

    def setUp(self):
        self.grid = empty_grid()
        self.quality = FormationQualityConfig()
        self.limits = FormationTimingLimits()

    def _select(self, goal):
        request = make_request(goal)
        snapshot = validate_formation_request(request).snapshot
        build = build_continuous_plans(snapshot, seed_for(request))
        self.assertTrue(build.success, build.detail)
        self.assertTrue(build.plans)
        candidates = []
        for candidate_id, plan in enumerate(build.plans, 1):
            timing = retime_formation_plan(
                plan, limits=self.limits, dt=0.02)
            self.assertIsNotNone(timing.timed, timing.detail)
            candidates.append(FormationTimedCandidate(
                candidate_id, plan, timing.timed))
        selection = select_formation_candidate(
            snapshot, candidates, self.grid, self.quality)
        return snapshot, build, selection

    def _assert_terminal_and_rigidity(self, snapshot, selection):
        timed = selection.selected_timed
        self.assertIsNotNone(timed)
        model = snapshot.model
        self.assertGreater(timed.duration, 0.0)
        self.assertGreater(selection.selected_evaluation.checks, 0)
        for index, pose_value in enumerate(timed.formation_poses):
            expected = model.centers(
                FormationPose(pose_value.x, pose_value.y, pose_value.psi))
            for vehicle_id, points in timed.trajectories.items():
                self.assertAlmostEqual(
                    points[index].center_x, expected[vehicle_id][0],
                    places=9)
                self.assertAlmostEqual(
                    points[index].center_y, expected[vehicle_id][1],
                    places=9)
        final_pose = timed.formation_poses[-1]
        self.assertAlmostEqual(final_pose.x, snapshot.goal.x, delta=GEOMETRY_TOL)
        self.assertAlmostEqual(final_pose.y, snapshot.goal.y, delta=GEOMETRY_TOL)
        self.assertAlmostEqual(final_pose.psi, snapshot.goal.psi,
                               delta=GEOMETRY_TOL)
        goal_centers = model.centers(snapshot.goal)
        for vehicle_id, points in timed.trajectories.items():
            self.assertAlmostEqual(
                points[-1].center_x, goal_centers[vehicle_id][0],
                delta=GEOMETRY_TOL)
            self.assertAlmostEqual(
                points[-1].center_y, goal_centers[vehicle_id][1],
                delta=GEOMETRY_TOL)
            self.assertAlmostEqual(points[-1].center_speed_mps, 0.0,
                                   delta=1.0e-3)
        self.assertLessEqual(
            selection.selected_evaluation.metrics["max_rigidity_error_m"],
            GEOMETRY_TOL)

    def test_e00_zero_motion_is_an_audited_hold(self):
        request = make_request(START)
        snapshot = validate_formation_request(request).snapshot
        empty = SimpleNamespace(
            header=header(), plan_id=request.plan_id,
            formation_id=request.formation_id,
            formation_start=request.formation_start,
            formation_goal=request.formation_goal, primitives=())
        build = build_continuous_plans(snapshot, empty)
        self.assertFalse(build.success)
        self.assertEqual(build.reason, "INVALID_SEED")
        self.assertIn("EMPTY_PRIMITIVE_SEQUENCE", build.detail)
        report = zero_motion_seed(snapshot, empty)
        self.assertIsNotNone(report)
        hold = build_hold_timed_trajectory(snapshot)
        evaluation = evaluate_formation_hold(
            snapshot, hold, self.grid, self.quality)
        self.assertTrue(evaluation.feasible, evaluation.detail)
        self.assertEqual(evaluation.reason, NO_MOTION)
        self.assertTrue(evaluation.metrics["no_motion"])
        self.assertEqual(hold.duration, 0.0)

    def test_e01_straight_uses_ackermann_with_positive_speed(self):
        snapshot, build, selection = self._select(MISSIONS["E01_straight"])
        self.assertEqual(selection.status, FEASIBLE)
        self.assertEqual(selection.feasible_count, selection.candidate_count)
        self._assert_terminal_and_rigidity(snapshot, selection)
        timed = selection.selected_timed
        for vehicle_id, points in timed.trajectories.items():
            moving = [point for point in points
                      if abs(point.center_speed_mps) > 1.0e-6]
            self.assertTrue(moving)
            self.assertTrue(all(point.mode == ACKERMANN for point in moving))
            self.assertTrue(all(point.center_speed_mps > 0.0
                                for point in moving))
            self.assertAlmostEqual(points[-1].theta, 0.0, delta=GEOMETRY_TOL)
            self.assertEqual(points[-1].mode, ACKERMANN)
        self.assertGreater(
            selection.selected_evaluation.metrics["sample_peaks"]["speed"],
            0.5)

    def test_e02_lateral_is_oblique_with_unchanged_body_headings(self):
        snapshot, build, selection = self._select(MISSIONS["E02_lateral"])
        self.assertEqual(selection.status, FEASIBLE)
        self.assertEqual(selection.feasible_count, selection.candidate_count)
        self._assert_terminal_and_rigidity(snapshot, selection)
        timed = selection.selected_timed
        for vehicle_id, points in timed.trajectories.items():
            moving = [point for point in points
                      if point.wheel_speed_mps > 1.0e-9]
            self.assertTrue(moving)
            self.assertTrue(all(point.mode == OBLIQUE for point in moving))
            for point in moving:
                # A pure +y translation from body heading zero is a 90 deg
                # oblique command.  The per-vehicle direction sign is not
                # observable in the signed body-forward speed (it is exactly
                # zero), so S5 may label it forward or reverse; both reach
                # +y with the same |phi| and must keep the body fixed.
                self.assertAlmostEqual(
                    abs(point.mode_control), math.pi / 2.0,
                    delta=GEOMETRY_TOL)
                self.assertAlmostEqual(
                    point.theta, 0.0, delta=GEOMETRY_TOL)
            # The body does not rotate: signed body-forward speed is zero
            # while the centre truly travels +y and the wheels are driven.
            self.assertTrue(all(abs(point.center_speed_mps) <= 1.0e-6
                                for point in moving))
            self.assertGreater(points[-1].center_y, points[0].center_y + 2.9)
            self.assertAlmostEqual(points[-1].center_x, points[0].center_x,
                                   delta=GEOMETRY_TOL)
        self.assertGreater(
            selection.selected_evaluation.metrics["sample_peaks"][
                "wheel_speed"], 0.5)

    def test_e03_oblique_is_oblique_forward_along_the_body_angle(self):
        snapshot, build, selection = self._select(MISSIONS["E03_oblique"])
        self.assertEqual(selection.status, FEASIBLE)
        self._assert_terminal_and_rigidity(snapshot, selection)
        timed = selection.selected_timed
        expected_phi = math.atan2(2.0, 3.0)
        for vehicle_id, points in timed.trajectories.items():
            moving = [point for point in points
                      if abs(point.center_speed_mps) > 1.0e-6]
            self.assertTrue(moving)
            self.assertTrue(all(point.mode == OBLIQUE for point in moving))
            for point in moving:
                self.assertAlmostEqual(
                    point.mode_control, expected_phi, delta=GEOMETRY_TOL)
                self.assertAlmostEqual(
                    point.theta, 0.0, delta=GEOMETRY_TOL)
                self.assertGreater(point.center_speed_mps, 0.0)
            self.assertAlmostEqual(
                points[-1].center_x - points[0].center_x, 3.0,
                delta=GEOMETRY_TOL)
            self.assertAlmostEqual(
                points[-1].center_y - points[0].center_y, 2.0,
                delta=GEOMETRY_TOL)

    def test_e04_reverse_uses_ackermann_with_negative_signed_speed(self):
        snapshot, build, selection = self._select(MISSIONS["E04_reverse"])
        self.assertEqual(selection.status, FEASIBLE)
        self.assertEqual(selection.feasible_count, selection.candidate_count)
        self._assert_terminal_and_rigidity(snapshot, selection)
        timed = selection.selected_timed
        for vehicle_id, points in timed.trajectories.items():
            moving = [point for point in points
                      if abs(point.center_speed_mps) > 1.0e-6]
            self.assertTrue(moving)
            self.assertTrue(all(point.mode == ACKERMANN for point in moving))
            self.assertTrue(all(point.center_speed_mps < 0.0
                                for point in moving))
            self.assertAlmostEqual(points[-1].theta, 0.0, delta=GEOMETRY_TOL)
            self.assertAlmostEqual(
                points[-1].center_x - points[0].center_x, -3.0,
                delta=GEOMETRY_TOL)
        # The evaluator's peak table stores magnitudes; the sign of the
        # commanded travel is asserted on every moving sample above.


if __name__ == "__main__":
    unittest.main()