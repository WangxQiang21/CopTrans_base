"""T4 acceptance: stationary cargo, independent vehicle Spin, then A orbit.

The T4 completion standard is explicit:

* with all three initial body headings at zero, the cargo can rotate +-90
  degrees in place on an empty map;
* the three vehicle centres keep the rigid wedge configuration for the whole
  plan;
* at least one test shows a vehicle spinning about its own centre and then
  moving with an A-mode arc around the cargo.

No external seed or ROS message is extended here.  ``body_spin`` is an
internal S6 geometry type built from either an explicitly requested terminal
body heading (empty S4 seed) or a tangent/anti-tangent pre-rotation scheme.
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
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/scripts"))

from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import (
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import FormationCollisionChecker
from fwis_v3_optimizer.formation_geometry import (
    BODY_SPIN,
    ROTATION,
    audit_continuous_plan,
    build_continuous_plans,
    precheck_plan_collision,
)
from fwis_v3_optimizer.formation_hold import zero_motion_seed
from fwis_v3_optimizer.formation_kinematics import ACKERMANN, SPIN
from fwis_v3_optimizer.formation_quality import (
    FEASIBLE,
    FormationQualityConfig,
    FormationTimedCandidate,
    select_formation_candidate,
)
from fwis_v3_optimizer.formation_request import validate_formation_request
from fwis_v3_optimizer.formation_retiming import (
    FormationTimingLimits,
    retime_formation_plan,
)
from fwis_v3_optimizer.formation_playback import (
    frame_at,
    trajectory_from_fields,
)
from fwis_v3_optimizer.formation_spin import rotation_schemes
from formation_optimizer_node import timed_trajectory_fields
from tests.test_formation_t3_missions import (
    GEOMETRY_TOL,
    START,
    empty_grid,
    header,
    make_request,
)

ANGLE = math.pi / 2.0


def rotation_seed(request, dpsi):
    return SimpleNamespace(
        header=header(),
        plan_id=request.plan_id,
        formation_id=request.formation_id,
        formation_start=request.formation_start,
        formation_goal=request.formation_goal,
        primitives=(SimpleNamespace(
            geometry_type=1, dx=0.0, dy=0.0, dpsi=dpsi,
            pivot_x=0.0, pivot_y=0.0),))


def empty_seed(request):
    return SimpleNamespace(
        header=header(),
        plan_id=request.plan_id,
        formation_id=request.formation_id,
        formation_start=request.formation_start,
        formation_goal=request.formation_goal,
        primitives=())


def explicit_heading_request(headings):
    """Cargo-stationary request with an explicit per-vehicle goal heading."""
    request = make_request(START)
    for vehicle in request.vehicles:
        vehicle.goal_heading_free = False
        vehicle.goal_center.theta = headings[vehicle.vehicle_id]
    return request


def centres_at(snapshot, pose):
    return snapshot.model.centers(pose)


class T4SpinTest(unittest.TestCase):

    def setUp(self):
        self.limits = FormationTimingLimits()
        self.quality = FormationQualityConfig()

    def _timed_candidates(self, build):
        candidates = []
        for candidate_id, plan in enumerate(build.plans, 1):
            timing = retime_formation_plan(
                plan, limits=self.limits, dt=0.02)
            self.assertEqual(
                timing.status, "TIMED_GEOMETRY_ONLY", timing.detail)
            candidates.append(FormationTimedCandidate(
                candidate_id, plan, timing.timed))
        return candidates

    def _selection(self, snapshot, build, grid):
        candidates = self._timed_candidates(build)
        selection = select_formation_candidate(
            snapshot, candidates, grid, self.quality)
        self.assertIsNotNone(selection.selected_evaluation)
        self.assertTrue(
            selection.selected_evaluation.feasible,
            selection.selected_evaluation.detail)
        self.assertEqual(selection.selected_evaluation.status, FEASIBLE)
        return selection

    def _select(self, snapshot, build, grid):
        return self._selection(snapshot, build, grid).selected_timed

    def test_heading_only_request_is_a_body_spin_not_a_hold(self):
        request = explicit_heading_request(
            {1: ANGLE, 2: -math.pi / 4.0, 3: 0.0})
        snapshot = validate_formation_request(request).snapshot
        seed = empty_seed(request)

        # The identical q_F pose must not be classified as NO_MOTION when an
        # explicit body heading still differs from the start heading.
        self.assertIsNone(zero_motion_seed(snapshot, seed))

        build = build_continuous_plans(snapshot, seed)
        self.assertTrue(build.success, build.detail)
        self.assertIn("BODY_SPIN_ONLY", build.detail)
        self.assertEqual(len(build.plans), 1)
        plan = build.plans[0]
        self.assertEqual(len(plan.blocks), 1)
        block = plan.blocks[0]
        self.assertEqual(block.geometry_type, BODY_SPIN)
        self.assertAlmostEqual(
            block.parameter_length, ANGLE, delta=GEOMETRY_TOL)

        segments = block.segment_map
        self.assertEqual(segments[1].mode, SPIN)
        self.assertEqual(segments[1].direction, 1)
        self.assertAlmostEqual(segments[1].theta_end, ANGLE)
        self.assertEqual(segments[2].mode, SPIN)
        self.assertEqual(segments[2].direction, -1)
        self.assertAlmostEqual(segments[2].theta_end, -math.pi / 4.0)
        self.assertEqual(segments[3].mode, ACKERMANN)
        self.assertAlmostEqual(segments[3].theta_end, 0.0)
        for segment in segments.values():
            self.assertEqual(segment.control, 0.0)
            self.assertEqual(segment.travel_distance, 0.0)
            self.assertAlmostEqual(
                segment.start_center_x, segment.end_center_x)
            self.assertAlmostEqual(
                segment.start_center_y, segment.end_center_y)
        # q_F is fixed throughout the joint event.
        self.assertAlmostEqual(block.control.start_qf.x, block.control.end_qf.x)
        self.assertAlmostEqual(block.control.start_qf.y, block.control.end_qf.y)
        self.assertAlmostEqual(
            block.control.start_qf.psi, block.control.end_qf.psi)

        audit = audit_continuous_plan(plan, snapshot)
        self.assertTrue(audit.success, audit.reason + " " + audit.detail)

        timed = self._select(snapshot, build, empty_grid())
        self.assertGreater(timed.duration, 0.0)
        start_centres = centres_at(snapshot, snapshot.start)
        for vehicle_id, points in timed.trajectories.items():
            self.assertEqual(len(points), len(timed.times_from_start))
            for point in points:
                self.assertAlmostEqual(
                    point.center_x, start_centres[vehicle_id][0],
                    delta=GEOMETRY_TOL)
                self.assertAlmostEqual(
                    point.center_y, start_centres[vehicle_id][1],
                    delta=GEOMETRY_TOL)
            if vehicle_id == 1:
                self.assertAlmostEqual(points[-1].theta, ANGLE, delta=1e-6)
            elif vehicle_id == 2:
                self.assertAlmostEqual(
                    points[-1].theta, -math.pi / 4.0, delta=1e-6)
            else:
                self.assertAlmostEqual(points[-1].theta, 0.0, delta=1e-6)
        peaks = timed.timing_report.vehicle_peaks
        for vehicle_id in (1, 2, 3):
            self.assertLessEqual(
                peaks[vehicle_id]["yaw_rate"],
                self.limits.yaw_rate_max + 1e-9)
            self.assertLessEqual(
                peaks[vehicle_id]["yaw_acceleration"],
                self.limits.yaw_accel_max + 1e-9)
            self.assertLessEqual(
                peaks[vehicle_id]["yaw_jerk"],
                self.limits.yaw_jerk_max + 1e-9)
        # One common event duration: vehicle 1 rotates twice as far as vehicle
        # 2, so its peak yaw rate is the binding constraint.
        self.assertGreater(
            peaks[1]["yaw_rate"], peaks[2]["yaw_rate"] + 0.1)

    def test_rotation_schemes_are_tangent_anti_tangent_with_finite_budget(self):
        offsets = (
            VehicleOffset(1, 1.0, 0.0),
            VehicleOffset(2, 0.0, 1.0),
            VehicleOffset(3, 0.0, -1.0),
        )
        qf = FormationPose(START[0], START[1], 0.0)
        headings = {1: 0.0, 2: 0.0, 3: 0.0}
        schemes = rotation_schemes(qf, headings, offsets, ANGLE)
        self.assertEqual(len(schemes), 8)
        for scheme in schemes:
            targets = dict(scheme.end_headings)
            # Vehicle 1 offset (1, 0): tangents are +90 and -90 degrees.
            self.assertTrue(
                min(abs(targets[1] - ANGLE),
                    abs(targets[1] + ANGLE)) <= 1e-9)
            # Vehicle 2 offset (0, 1): tangents are 0 and +180 degrees.
            self.assertTrue(
                min(abs(targets[2]), abs(abs(targets[2]) - math.pi)) <= 1e-9)
            # Vehicle 3 offset (0, -1): tangents are 0 and +180 degrees.
            self.assertTrue(
                min(abs(targets[3]), abs(abs(targets[3]) - math.pi)) <= 1e-9)
        self.assertEqual(
            [scheme.cost for scheme in schemes],
            sorted(scheme.cost for scheme in schemes))
        limited = rotation_schemes(
            qf, headings, offsets, ANGLE, max_schemes=3)
        self.assertEqual(len(limited), 3)

        # An explicit terminal heading selects the pre-rotation tangent
        # required by the rigid rotation; an inconsistent one yields no
        # scheme instead of silently clamping.
        goals = {
            1: ANGLE + ANGLE,
            2: math.pi + ANGLE,
            3: 0.0 + ANGLE,
        }
        fixed = {1: False, 2: False, 3: False}
        forced = rotation_schemes(
            qf, headings, offsets, ANGLE, goal_headings=goals,
            goal_heading_free=fixed)
        self.assertEqual(len(forced), 1)
        self.assertEqual(
            tuple(round(value, 9) for _, value in forced[0].end_headings),
            (round(ANGLE, 9), round(math.pi, 9), 0.0))
        impossible = dict(goals)
        impossible[1] = ANGLE + ANGLE + 0.3
        self.assertEqual(
            rotation_schemes(
                qf, headings, offsets, ANGLE, goal_headings=impossible,
                goal_heading_free=fixed), ())

    def test_cargo_plus90_spins_then_drives_A_arc_around_cargo(self):
        self._check_cargo_rotation(ANGLE)

    def test_cargo_minus90_spins_then_drives_A_arc_around_cargo(self):
        self._check_cargo_rotation(-ANGLE)

    def _check_cargo_rotation(self, dpsi):
        request = make_request((START[0], START[1], dpsi))
        snapshot = validate_formation_request(request).snapshot
        build = build_continuous_plans(
            snapshot, rotation_seed(request, dpsi), max_candidates=8)
        self.assertTrue(build.success, build.detail)
        self.assertTrue(build.plans)

        spin_index = None
        plan = build.plans[0]
        for index, block in enumerate(plan.blocks):
            if block.geometry_type == BODY_SPIN:
                spin_index = index
                break
        self.assertIsNotNone(spin_index)
        spin_block = plan.blocks[spin_index]
        self.assertEqual(spin_block.control.start_qf.x, snapshot.start.x)
        self.assertEqual(spin_block.control.start_qf.y, snapshot.start.y)
        self.assertAlmostEqual(
            spin_block.control.start_qf.psi, snapshot.start.psi)
        for segment in spin_block.vehicle_mode_segments:
            self.assertAlmostEqual(
                segment.start_center_x, segment.end_center_x)
            self.assertAlmostEqual(
                segment.start_center_y, segment.end_center_y)
        self.assertTrue(any(
            segment.mode == SPIN
            for segment in spin_block.vehicle_mode_segments))

        # Spin is followed by the rigid rotation primitive represented by A
        # arcs about the cargo pivot.
        orbit = plan.blocks[spin_index + 1:]
        self.assertTrue(orbit)
        self.assertEqual(orbit[0].geometry_type, ROTATION)
        a_segments = [
            segment for segment in orbit[0].vehicle_mode_segments
            if segment.mode == ACKERMANN]
        self.assertTrue(a_segments)
        self.assertTrue(any(
            math.hypot(
                segment.end_center_x - segment.start_center_x,
                segment.end_center_y - segment.start_center_y) > 0.1
            for segment in a_segments))

        audit = audit_continuous_plan(plan, snapshot)
        self.assertTrue(audit.success, audit.reason + " " + audit.detail)

        timed = self._select(snapshot, build, empty_grid())
        self.assertGreater(timed.duration, 0.0)
        final_pose = timed.formation_poses[-1]
        self.assertAlmostEqual(final_pose.x, snapshot.goal.x, delta=1e-6)
        self.assertAlmostEqual(final_pose.y, snapshot.goal.y, delta=1e-6)
        self.assertAlmostEqual(final_pose.psi, snapshot.goal.psi, delta=1e-6)

        # All vehicles share one strictly increasing time axis.
        times = timed.times_from_start
        for vehicle_id, points in timed.trajectories.items():
            self.assertEqual(len(points), len(times))
            for index, point in enumerate(points):
                self.assertAlmostEqual(point.time_from_start, times[index])
            # During the spin block every centre is fixed.
            spin_points = [
                point for point in points
                if point.block_index == spin_block.block_index and (
                    not point.stationary)]
            self.assertTrue(spin_points)
            first = spin_points[0]
            for point in spin_points:
                self.assertAlmostEqual(point.center_x, first.center_x,
                                       delta=GEOMETRY_TOL)
                self.assertAlmostEqual(point.center_y, first.center_y,
                                       delta=GEOMETRY_TOL)
            self.assertGreater(
                abs(spin_points[-1].theta - spin_points[0].theta) + 1e-12,
                0.0)
            # No heading jump at a block boundary or inside the spin.
            for first_point, second_point in zip(points, points[1:]):
                self.assertLessEqual(
                    abs(math.atan2(
                        math.sin(second_point.theta - first_point.theta),
                        math.cos(second_point.theta - first_point.theta))),
                    self.limits.yaw_rate_max * 0.02 + 1e-6)
        # The rigid centres are reconstructed from q_F at every sample.
        for index, pose in enumerate(timed.formation_poses):
            expected = centres_at(snapshot, FormationPose(
                pose.x, pose.y, pose.psi))
            for vehicle_id, points in timed.trajectories.items():
                self.assertAlmostEqual(
                    points[index].center_x, expected[vehicle_id][0],
                    delta=GEOMETRY_TOL)
                self.assertAlmostEqual(
                    points[index].center_y, expected[vehicle_id][1],
                    delta=GEOMETRY_TOL)
        self.assertLessEqual(
            timed.timing_report.vehicle_peaks[
                spin_block.vehicle_mode_segments[0].vehicle_id]["yaw_rate"],
            self.limits.yaw_rate_max + 1e-9)

    def test_spin_time_and_mode_switch_dwell_are_separate(self):
        request = make_request((START[0], START[1], ANGLE))
        snapshot = validate_formation_request(request).snapshot
        build = build_continuous_plans(
            snapshot, rotation_seed(request, ANGLE), max_candidates=1)
        self.assertTrue(build.success, build.detail)
        plan = build.plans[0]
        spin = plan.blocks[0]
        self.assertEqual(spin.geometry_type, BODY_SPIN)
        timing = retime_formation_plan(
            plan, limits=self.limits, dt=0.02)
        self.assertEqual(timing.status, "TIMED_GEOMETRY_ONLY", timing.detail)
        timed = timing.timed
        motion = [
            phase for phase in timed.phases
            if phase["kind"] == "motion" and phase["block_index"] == 0]
        stationary = [
            phase for phase in timed.phases
            if phase["kind"] == "stationary" and (
                phase["from_block_index"] == 0)]
        self.assertEqual(len(motion), 1)
        self.assertEqual(len(stationary), 1)
        self.assertGreater(motion[0]["duration_s"], 0.0)
        self.assertGreater(stationary[0]["duration_s"], 0.0)
        self.assertEqual(stationary[0]["event_kind"], "mode_switch")
        # The dwell starts after the spin, never inside it.
        self.assertAlmostEqual(
            stationary[0]["t0_s"],
            motion[0]["t0_s"] + motion[0]["duration_s"], delta=1e-9)
        # The mode interval for the spinning vehicle is the actual spin
        # duration, not spin plus dwell.
        vehicle_id = spin.vehicle_mode_segments[0].vehicle_id
        runs = timed.timing_report.mode_intervals[vehicle_id]
        self.assertEqual(runs[0]["mode"], SPIN)
        self.assertAlmostEqual(
            runs[0]["duration_s"], motion[0]["duration_s"], delta=1e-9)
        self.assertGreaterEqual(
            runs[0]["duration_s"], self.limits.min_mode_dwell_s)
        self.assertGreaterEqual(
            runs[1]["duration_s"], self.limits.min_mode_dwell_s)

    def test_playback_shares_one_clock_and_shows_independent_headings(self):
        request = make_request((START[0], START[1], ANGLE))
        snapshot = validate_formation_request(request).snapshot
        build = build_continuous_plans(
            snapshot, rotation_seed(request, ANGLE), max_candidates=4)
        self.assertTrue(build.success, build.detail)
        selection = self._selection(snapshot, build, empty_grid())
        timed = selection.selected_timed
        fields = timed_trajectory_fields(snapshot, timed, (9000, 0))
        playback = trajectory_from_fields(fields)

        motion_phases = [
            phase for phase in timed.phases if phase["kind"] == "motion"]
        spin_phase = next(
            phase for phase in motion_phases if phase["block_index"] == 0)
        self.assertEqual(
            selection.selected_plan.blocks[0].geometry_type, BODY_SPIN)
        orbit_phase = next(
            phase for phase in motion_phases if phase["block_index"] == 1)
        dwell = next(
            phase for phase in timed.phases
            if phase["kind"] == "stationary" and
            phase["from_block_index"] == 0)
        self.assertGreater(dwell["duration_s"], 0.0)

        start_centres = centres_at(snapshot, snapshot.start)
        mid_spin = spin_phase["t0_s"] + 0.5 * spin_phase["duration_s"]
        frame = frame_at(playback, mid_spin)
        states = frame.vehicle_map()
        for vehicle_id, state in states.items():
            self.assertAlmostEqual(
                state.center_x, start_centres[vehicle_id][0],
                delta=GEOMETRY_TOL)
            self.assertAlmostEqual(
                state.center_y, start_centres[vehicle_id][1],
                delta=GEOMETRY_TOL)
        # All vehicles use the same clock and the same fraction.
        self.assertEqual(
            len({state.vehicle_id for state in frame.vehicles}), 3)
        self.assertGreater(
            abs(math.atan2(
                math.sin(states[1].theta - states[2].theta),
                math.cos(states[1].theta - states[2].theta))), 0.3)

        # The mode-switch dwell does not start the orbit early: q_F and every
        # centre remain fixed just before the rotation motion begins.
        before_orbit = orbit_phase["t0_s"] - 1e-6
        hold_frame = frame_at(playback, before_orbit)
        self.assertEqual(hold_frame.state, "PLAYING")
        self.assertAlmostEqual(
            hold_frame.formation_x, snapshot.start.x, delta=GEOMETRY_TOL)
        self.assertAlmostEqual(
            hold_frame.formation_y, snapshot.start.y, delta=GEOMETRY_TOL)
        self.assertAlmostEqual(
            hold_frame.formation_psi, snapshot.start.psi, delta=GEOMETRY_TOL)
        for vehicle_id, state in hold_frame.vehicle_map().items():
            self.assertAlmostEqual(
                state.center_x, start_centres[vehicle_id][0],
                delta=GEOMETRY_TOL)
            self.assertAlmostEqual(
                state.center_y, start_centres[vehicle_id][1],
                delta=GEOMETRY_TOL)
        after_orbit = orbit_phase["t0_s"] + 0.25 * orbit_phase["duration_s"]
        moved = frame_at(playback, after_orbit)
        self.assertTrue(any(
            math.hypot(
                state.center_x - start_centres[state.vehicle_id][0],
                state.center_y - start_centres[state.vehicle_id][1]) > 0.05
            for state in moved.vehicles))

    def test_body_spin_obb_sweep_rejects_obstacle_outside_both_endpoints(self):
        request = explicit_heading_request(
            {1: ANGLE, 2: 0.0, 3: 0.0})
        snapshot = validate_formation_request(request).snapshot
        build = build_continuous_plans(snapshot, empty_seed(request))
        self.assertTrue(build.success, build.detail)
        plan = build.plans[0]

        clean = empty_grid()
        clean_audit = audit_continuous_plan(
            plan, snapshot, grid=clean, margin_m=0.05)
        self.assertTrue(clean_audit.success, clean_audit.detail)
        clean_precheck = precheck_plan_collision(
            plan, snapshot, clean, margin_m=0.05)
        self.assertTrue(clean_precheck.success, clean_precheck.report.reason)

        occupied = np.zeros((600, 600), dtype=bool)
        # Vehicle 1 starts at (11, 10) heading 0 and spins to 90 degrees in
        # place.  The cell is outside the expanded OBB at 0 and 90 degrees but
        # is swept by a corner near 35 degrees.
        occupied[int(10.0 / 0.05), int(11.46 / 0.05)] = True
        blocked = Grid(occupied, 0.05, 0.0, 0.0)

        checker = FormationCollisionChecker(
            blocked, snapshot.model, snapshot.vehicle_shapes, 0.05)
        start_headings = dict(snapshot.start_headings)
        self.assertTrue(
            checker.static_check(snapshot.start, start_headings).success)
        end_headings = dict(start_headings)
        end_headings[1] = ANGLE
        self.assertTrue(
            checker.static_check(snapshot.start, end_headings).success)

        precheck = precheck_plan_collision(
            plan, snapshot, blocked, margin_m=0.05)
        self.assertFalse(precheck.success)
        self.assertIn(precheck.report.reason, (
            "MAP_OBB_COLLISION", "VEHICLE_OBB_COLLISION"))
        self.assertGreater(precheck.samples, 1)
        audit = audit_continuous_plan(
            plan, snapshot, grid=blocked, margin_m=0.05)
        self.assertFalse(audit.success)
        self.assertEqual(audit.reason, "COLLISION_PRECHECK_FAILED")
        self.assertTrue(audit.collision_prechecked)

    def test_tampered_spin_target_is_rejected_by_independent_audit(self):
        request = explicit_heading_request(
            {1: ANGLE, 2: 0.0, 3: 0.0})
        snapshot = validate_formation_request(request).snapshot
        build = build_continuous_plans(snapshot, empty_seed(request))
        self.assertTrue(build.success, build.detail)
        plan = build.plans[0]
        self.assertTrue(audit_continuous_plan(plan, snapshot).success)
        block = plan.blocks[0]
        tampered_segments = tuple(
            replace(segment, theta_end=segment.theta_end + 0.1)
            if segment.vehicle_id == 1 else segment
            for segment in block.vehicle_mode_segments)
        tampered = replace(
            plan, blocks=(replace(
                block, vehicle_mode_segments=tampered_segments),))
        audit = audit_continuous_plan(tampered, snapshot)
        self.assertFalse(audit.success)
        self.assertIn(audit.reason, (
            "GOAL_HEADING_MISMATCH", "HEADING_CONTINUITY_VIOLATION"))

    def test_stored_spin_centres_are_recomputed_from_qf(self):
        request = explicit_heading_request(
            {1: ANGLE, 2: 0.0, 3: 0.0})
        snapshot = validate_formation_request(request).snapshot
        build = build_continuous_plans(snapshot, empty_seed(request))
        self.assertTrue(build.success, build.detail)
        plan = build.plans[0]
        block = plan.blocks[0]
        shifted = tuple(
            replace(
                segment,
                start_center_x=segment.start_center_x + 0.1,
                start_center_y=segment.start_center_y + 0.1,
                end_center_x=segment.end_center_x + 0.1,
                end_center_y=segment.end_center_y + 0.1)
            if segment.vehicle_id == 1 else segment
            for segment in block.vehicle_mode_segments)
        tampered = replace(plan, blocks=(replace(
            block, vehicle_mode_segments=shifted),))
        audit = audit_continuous_plan(tampered, snapshot)
        self.assertFalse(audit.success)
        self.assertEqual(audit.reason, "CONSTRUCTION_MISMATCH")

    def test_orbiting_vehicle_mislabelled_as_spin_is_rejected(self):
        request = make_request((START[0], START[1], ANGLE))
        snapshot = validate_formation_request(request).snapshot
        build = build_continuous_plans(
            snapshot, rotation_seed(request, ANGLE), max_candidates=1)
        self.assertTrue(build.success, build.detail)
        plan = build.plans[0]
        rotation = plan.blocks[1]
        self.assertEqual(rotation.geometry_type, ROTATION)
        segments = rotation.vehicle_mode_segments
        target = next(
            segment for segment in segments
            if segment.mode == ACKERMANN and math.hypot(
                segment.start_center_x - snapshot.start.x,
                segment.start_center_y - snapshot.start.y) > 0.5)
        falsified = replace(
            target, mode=SPIN, direction=1, control=0.0,
            travel_distance=0.0)
        tampered = replace(plan, blocks=(
            plan.blocks[0],
            replace(rotation, vehicle_mode_segments=tuple(
                falsified if segment.vehicle_id == target.vehicle_id
                else segment for segment in segments))))
        audit = audit_continuous_plan(tampered, snapshot)
        self.assertFalse(audit.success)
        self.assertEqual(audit.reason, "NON_PIVOT_SPIN")


if __name__ == "__main__":
    unittest.main()