"""S8 common-time formation retiming tests.

These tests exercise the ROS-free retiming library only.  They verify that a
single monotonic formation progress lambda(t) drives every vehicle center via
``q_F(lambda) + R(psi_F(lambda)) r_i``, that each block is a rest-to-rest
quintic phase, that mode switches and wheel reconfigurations insert a full
stationary hold, and that the output is never claimed executable, published,
collision-certified, or ``FEASIBLE`` before S9.
"""
from pathlib import Path
import math
from types import SimpleNamespace
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    FormationValidationError,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_geometry import (
    audit_continuous_plan,
    build_continuous_plans,
)
from fwis_v3_optimizer.formation_retiming import (
    FORMATION_RETIMING_INFEASIBLE,
    GEOMETRY_ONLY,
    TIMED_GEOMETRY_ONLY,
    FormationTimingLimits,
    audit_formation_timing,
    retime_formation_plan,
)

TOL = 1e-9


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
             formation_id="s8_test"):
    model = FormationModel(offsets)
    if free is None:
        free = {offset.vehicle_id: False for offset in offsets}
    return SimpleNamespace(
        plan_id=plan_id, formation_id=formation_id,
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
    plan = build.plans[0]
    audit = audit_continuous_plan(plan, request)
    if not audit.success:
        raise AssertionError(
            "seed audit failed: {} {}".format(audit.reason, audit.detail))
    return plan


def straight_plan(plan_id=17):
    offsets = (VehicleOffset(0, 0.0, 0.0), VehicleOffset(1, 1.2, 0.0))
    request = snapshot(
        offsets, FormationPose(0.0, 0.0, 0.0), FormationPose(2.0, 0.0, 0.0),
        {0: 0.0, 1: 0.0}, {0: 0.0, 1: 0.0},
        plan_id=plan_id, formation_id="s8_straight")
    return build_plan(request, (primitive(0, dx=2.0),))


def rotation_plan(plan_id=17):
    offsets = (VehicleOffset(0, 0.0, 0.0), VehicleOffset(1, 1.2, 0.0))
    dpsi = 0.5
    request = snapshot(
        offsets, FormationPose(0.0, 0.0, 0.0),
        FormationPose(0.0, 0.0, dpsi),
        {0: 0.0, 1: math.pi / 2.0},
        {0: dpsi, 1: math.pi / 2.0 + dpsi},
        plan_id=plan_id, formation_id="s8_rotation")
    return build_plan(request, (primitive(1, dpsi=dpsi),))


def cw_rotation_plan(plan_id=17):
    offsets = (VehicleOffset(0, 0.0, 0.0), VehicleOffset(1, 1.2, 0.0))
    dpsi = -0.5
    request = snapshot(
        offsets, FormationPose(0.0, 0.0, 0.0),
        FormationPose(0.0, 0.0, dpsi),
        {0: 0.0, 1: math.pi / 2.0},
        {0: dpsi, 1: math.pi / 2.0 + dpsi},
        allow_reverse=True, plan_id=plan_id,
        formation_id="s8_cw_rotation")
    return build_plan(request, (primitive(1, dpsi=dpsi),))


def chain_plan(plan_id=17):
    offsets = (VehicleOffset(0, 0.0, 0.0), VehicleOffset(1, 1.2, 0.0))
    dpsi = 0.3
    start = FormationPose(0.0, 0.0, 0.0)
    goal = FormationPose(2.0 * math.cos(dpsi), 2.0 * math.sin(dpsi), dpsi)
    request = snapshot(
        offsets, start, goal,
        {0: 0.0, 1: math.pi / 2.0},
        {0: dpsi, 1: math.pi / 2.0 + dpsi},
        allow_reverse=True, plan_id=plan_id, formation_id="s8_chain")
    return build_plan(request, (
        primitive(1, dpsi=dpsi),
        primitive(0, dx=1.0, dy=0.3),
        primitive(0, dx=1.0, dy=-0.3)))


def retimed(plan, **kwargs):
    result = retime_formation_plan(plan, **kwargs)
    if result.timed is None:
        raise AssertionError(
            "retiming failed: {} {}".format(result.reason, result.detail))
    return result


class FormationRetimingTest(unittest.TestCase):

    def test_translation_common_grid_and_rigid_centers(self):
        plan = straight_plan()
        result = retimed(plan)
        self.assertEqual(result.status, TIMED_GEOMETRY_ONLY)
        self.assertTrue(result.audit.success, result.audit.detail)
        timed = result.timed
        self.assertEqual(timed.certification, GEOMETRY_ONLY)

        times = timed.times_from_start
        self.assertEqual(times[0], 0.0)
        for first, second in zip(times, times[1:]):
            self.assertGreater(second, first)
        self.assertEqual(len(timed.formation_poses), len(times))
        self.assertEqual(set(timed.trajectories), {0, 1})
        for points in timed.trajectories.values():
            self.assertEqual(len(points), len(times))
            for point, time in zip(points, times):
                self.assertAlmostEqual(point.time_from_start, time, places=9)

        model = FormationModel(timed.offsets)
        for index, pose in enumerate(timed.formation_poses):
            expected = model.centers(pose)
            for vehicle_id, center in expected.items():
                point = timed.trajectories[vehicle_id][index]
                self.assertAlmostEqual(point.center_x, center[0], places=9)
                self.assertAlmostEqual(point.center_y, center[1], places=9)

        start, goal = plan.start, plan.goal
        self.assertAlmostEqual(timed.formation_poses[0].x, start.x, places=9)
        self.assertAlmostEqual(timed.formation_poses[0].y, start.y, places=9)
        self.assertAlmostEqual(timed.formation_poses[-1].x, goal.x, places=9)
        self.assertAlmostEqual(timed.formation_poses[-1].y, goal.y, places=9)
        for points in timed.trajectories.values():
            self.assertAlmostEqual(points[0].center_speed_mps, 0.0, places=9)
            self.assertAlmostEqual(points[-1].center_speed_mps, 0.0, places=9)
            self.assertAlmostEqual(points[0].center_acceleration_mps2,
                                   0.0, places=9)
            self.assertAlmostEqual(points[-1].center_acceleration_mps2,
                                   0.0, places=9)

        report = timed.timing_report
        self.assertAlmostEqual(
            report.moving_seconds + report.stationary_seconds,
            timed.duration, places=9)
        self.assertAlmostEqual(report.duration_s, timed.duration, places=9)
        self.assertEqual(report.stationary_seconds, 0.0)

    def test_speed_limit_sets_duration(self):
        plan = straight_plan()
        default = retimed(plan)
        tight = retimed(plan, limits=FormationTimingLimits(v_max=0.2))
        self.assertGreater(tight.timed.duration, default.timed.duration)
        driver = tight.timed.timing_report.block_durations[0][
            "duration_driver"]
        self.assertEqual(driver["constraint"], "speed")
        self.assertLessEqual(driver["peak"], 0.2 + TOL)
        self.assertEqual(driver["vehicle_id"], 0)

    def test_jerk_limit_sets_duration(self):
        plan = straight_plan()
        tight = retimed(plan, limits=FormationTimingLimits(jerk_max=0.5))
        driver = tight.timed.timing_report.block_durations[0][
            "duration_driver"]
        self.assertEqual(driver["constraint"], "jerk")
        self.assertLessEqual(driver["peak"], 0.5 + TOL)
        peaks = tight.timed.timing_report.vehicle_peaks[0]
        self.assertLessEqual(peaks["jerk"], 0.5 + TOL)
        self.assertLessEqual(peaks["speed"], 0.8 + TOL)

    def test_pure_rotation_pivot_and_nonpivot(self):
        plan = rotation_plan()
        result = retimed(plan)
        self.assertEqual(result.status, TIMED_GEOMETRY_ONLY)
        self.assertTrue(result.audit.success, result.audit.detail)
        timed = result.timed
        # Vehicle 0 has zero offset and must Spin; vehicle 1 rides the arc.
        pivot = timed.trajectories[0]
        self.assertTrue(all(point.mode == 2 for point in pivot))
        self.assertLess(max(abs(point.center_x) for point in pivot), TOL)
        self.assertLess(max(abs(point.center_y) for point in pivot), TOL)
        self.assertLess(max(abs(point.center_speed_mps) for point in pivot),
                        TOL)
        self.assertGreater(max(abs(point.center_yaw_rate_radps)
                               for point in pivot), 0.1)
        follower = timed.trajectories[1]
        self.assertTrue(all(point.mode == 0 for point in follower))
        self.assertGreater(max(abs(point.center_speed_mps)
                               for point in follower), 0.1)
        self.assertGreater(max(abs(point.center_x) for point in follower),
                           1.0)
        intervals = timed.timing_report.mode_intervals
        self.assertEqual([item["mode"] for item in intervals[0]], [2])
        self.assertEqual([item["mode"] for item in intervals[1]], [0])
        for points in timed.trajectories.values():
            self.assertAlmostEqual(points[0].center_speed_mps, 0.0,
                                   places=9)
            self.assertAlmostEqual(points[-1].center_speed_mps, 0.0,
                                   places=9)

    def test_clockwise_rotation_reports_reverse_signed_speeds(self):
        plan = cw_rotation_plan()
        result = retimed(plan)
        self.assertEqual(result.status, TIMED_GEOMETRY_ONLY)
        self.assertTrue(result.audit.success, result.audit.detail)
        timed = result.timed
        follower = timed.trajectories[1]
        self.assertTrue(all(point.mode == 0 for point in follower))
        signed = [point.center_speed_mps for point in follower
                  if abs(point.center_speed_mps) > 1e-6]
        self.assertTrue(signed)
        # The tangent-aligned body is dragged backwards by the clockwise
        # formation rotation: signed speed is negative while the wheel-speed
        # magnitude stays positive.
        self.assertTrue(all(value < 0.0 for value in signed))
        for point in follower:
            self.assertGreaterEqual(point.wheel_speed_mps, 0.0)
        ys = [point.center_y for point in follower]
        self.assertLess(ys[-1], ys[0] - 0.5)
        segment = plan.blocks[0].segment_map[1]
        self.assertEqual(segment.direction, -1)
        self.assertAlmostEqual(segment.control, 1.0 / 1.2, places=12)

    def test_chain_holds_dwell_and_reverse(self):
        plan = chain_plan()
        result = retimed(plan)
        self.assertEqual(result.status, TIMED_GEOMETRY_ONLY)
        self.assertTrue(result.audit.success, result.audit.detail)
        timed = result.timed
        report = timed.timing_report

        kinds = [event["kind"] for event in report.stationary_events]
        self.assertEqual(kinds, ["mode_switch", "wheel_reconfiguration"])
        phase_kinds = [phase["kind"] for phase in timed.phases]
        self.assertEqual(phase_kinds, [
            "motion", "stationary", "motion", "stationary", "motion"])
        for phase in timed.phases[1::2]:
            self.assertGreaterEqual(
                phase["duration_s"],
                FormationTimingLimits().min_mode_dwell_s - TOL)

        for intervals in report.mode_intervals.values():
            for item in intervals:
                self.assertGreaterEqual(
                    item["duration_s"],
                    FormationTimingLimits().min_mode_dwell_s - TOL)
        total_covered = sum(
            item["duration_s"]
            for intervals in report.mode_intervals.values()
            for item in intervals)
        self.assertAlmostEqual(
            total_covered / 2.0, timed.duration, places=6)

        self.assertEqual(len(report.direction_changes), 1)
        change = report.direction_changes[0]
        self.assertEqual(change["vehicle_id"], 1)
        self.assertEqual(change["from_direction"], 1)
        self.assertEqual(change["to_direction"], -1)
        self.assertTrue(change["at_rest"])

        follower = timed.trajectories[1]
        negative = [point.center_speed_mps for point in follower
                    if point.center_speed_mps < -1e-6]
        self.assertTrue(negative)
        for point in follower:
            if point.mode == 2:
                self.assertAlmostEqual(point.center_speed_mps, 0.0,
                                       places=9)
        for point in timed.trajectories[0]:
            self.assertGreaterEqual(point.center_speed_mps, -1e-9)
        self.assertAlmostEqual(
            report.moving_seconds + report.stationary_seconds,
            timed.duration, places=9)

    def test_static_steering_angle_failure(self):
        offsets = (VehicleOffset(0, 0.0, 0.0), VehicleOffset(1, 1.2, 0.0))
        request = snapshot(
            offsets, FormationPose(0.0, 0.0, 0.0),
            FormationPose(1.0, 1.0, 0.0),
            {0: 0.0, 1: 0.0}, {0: 0.0, 1: 0.0},
            formation_id="s8_oblique")
        plan = build_plan(request, (primitive(0, dx=1.0, dy=1.0),))
        result = retime_formation_plan(
            plan, limits=FormationTimingLimits(steer_angle_max=0.1))
        self.assertEqual(result.status, FORMATION_RETIMING_INFEASIBLE)
        self.assertIsNone(result.timed)
        self.assertIsNone(result.audit)
        self.assertIsNotNone(result.failure)
        self.assertEqual(result.failure.constraint, "steer_angle_limit")
        self.assertEqual(result.failure.vehicle_id, 0)
        self.assertEqual(result.failure.block_index, 0)
        self.assertIn("exceeds", result.failure.detail)

    def test_duration_budget_failure(self):
        plan = straight_plan()
        result = retime_formation_plan(
            plan, limits=FormationTimingLimits(
                v_max=0.05, max_block_duration_s=5.0))
        self.assertEqual(result.status, FORMATION_RETIMING_INFEASIBLE)
        self.assertEqual(result.reason, FORMATION_RETIMING_INFEASIBLE)
        self.assertIsNone(result.timed)
        self.assertIsNone(result.audit)
        self.assertEqual(result.failure.constraint, "speed")
        self.assertEqual(result.failure.vehicle_id, 0)
        self.assertEqual(result.failure.block_index, 0)
        self.assertIn("max_block_duration_s", result.failure.detail)

    def test_dwell_limit_extends_block_duration(self):
        plan = straight_plan()
        result = retimed(plan, limits=FormationTimingLimits(
            min_mode_dwell_s=5.0))
        self.assertEqual(result.status, TIMED_GEOMETRY_ONLY)
        self.assertTrue(result.audit.success, result.audit.detail)
        timed = result.timed
        self.assertAlmostEqual(timed.duration, 5.0, places=9)
        driver = timed.timing_report.block_durations[0]["duration_driver"]
        self.assertEqual(driver["constraint"], "mode_dwell")
        for intervals in timed.timing_report.mode_intervals.values():
            for item in intervals:
                self.assertGreaterEqual(item["duration_s"], 5.0 - TOL)

    def test_audit_rejects_tampered_center(self):
        plan = straight_plan()
        timed = retimed(plan).timed
        trajectories = dict(timed.trajectories)
        points = list(trajectories[1])
        point = points[10]
        points[10] = replace(point, center_x=point.center_x + 0.1)
        trajectories[1] = tuple(points)
        audit = audit_formation_timing(
            plan, replace(timed, trajectories=trajectories))
        self.assertFalse(audit.success)
        self.assertEqual(audit.reason, "RIGIDITY_VIOLATION")
        self.assertIn("vehicle_id=1", audit.detail)

    def test_audit_rejects_tampered_heading(self):
        plan = straight_plan()
        timed = retimed(plan).timed
        trajectories = dict(timed.trajectories)
        points = list(trajectories[0])
        point = points[20]
        points[20] = replace(point, theta=point.theta + 0.2)
        trajectories[0] = tuple(points)
        audit = audit_formation_timing(
            plan, replace(timed, trajectories=trajectories))
        self.assertFalse(audit.success)
        self.assertEqual(audit.reason, "HEADING_MISMATCH")
        self.assertIn("vehicle_id=0", audit.detail)

    def test_constructor_rejects_nonmonotonic_times(self):
        plan = straight_plan()
        timed = retimed(plan).timed
        times = list(timed.times_from_start)
        times[5] = times[4] - 0.01
        with self.assertRaises(FormationValidationError) as context:
            replace(timed, times_from_start=tuple(times))
        self.assertIn("TIMES_NOT_STRICTLY_INCREASING", str(context.exception))

    def test_invalid_inputs_fail_closed(self):
        plan = straight_plan()
        with self.assertRaises(FormationValidationError) as context:
            retime_formation_plan(object())
        self.assertIn("INVALID_FORMATION_PLAN", str(context.exception))
        with self.assertRaises(FormationValidationError) as context:
            retime_formation_plan(plan, limits=object())
        self.assertIn("INVALID_FORMATION_TIMING_LIMITS",
                      str(context.exception))
        with self.assertRaises(FormationValidationError) as context:
            retime_formation_plan(plan, dt=0.0)
        self.assertIn("dt must be > 0", str(context.exception))

        invalid_limits = (
            {"v_max": 0.0},
            {"a_max": -1.0},
            {"jerk_max": 0.0},
            {"mode_setup_time": -1.0},
            {"mode_switch_duration": 0.0},
            {"steer_angle_max": math.pi + 0.1},
            {"min_mode_dwell_s": -0.1},
            {"min_mode_dwell_s": 2.0, "max_block_duration_s": 1.0},
        )
        for kwargs in invalid_limits:
            with self.assertRaises(FormationValidationError,
                                   msg=str(kwargs)):
                FormationTimingLimits(**kwargs)

    def test_status_never_claims_feasible_or_executable(self):
        plan = straight_plan()
        result = retimed(plan)
        status = result.as_status()
        self.assertEqual(status["status"], TIMED_GEOMETRY_ONLY)
        self.assertEqual(status["certification"], GEOMETRY_ONLY)
        self.assertTrue(status["timing_audit_passed"])
        for key in ("executable", "trajectory_published", "feasible",
                    "independent_timing_audit", "collision_certificate"):
            self.assertIs(status[key], False, key)
        self.assertNotEqual(status["status"], "FEASIBLE")
        self.assertNotIn("FEASIBLE", status["detail"].upper())

        timed_status = result.timed.as_status()
        self.assertEqual(timed_status["certification"], GEOMETRY_ONLY)
        for key in ("executable", "trajectory_published", "feasible",
                    "independent_timing_audit", "collision_certificate"):
            self.assertIs(timed_status[key], False, key)
        report = result.timed.timing_report
        self.assertEqual(report.certification, GEOMETRY_ONLY)
        self.assertEqual(report.timing_audit, "SELF_CONSISTENCY_ONLY")
        for key in ("executable", "trajectory_published", "feasible",
                    "independent_timing_audit", "collision_certificate"):
            self.assertIs(getattr(report, key), False, key)
        self.assertIn("S9", result.detail)
        self.assertIn("S9 independent evaluation", result.audit.detail)


if __name__ == "__main__":
    unittest.main()