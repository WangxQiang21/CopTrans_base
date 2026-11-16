"""G4 mandatory negative tests for the conservative collision adapter.

Every rejected case here starts from a plan that already passed its geometry
and dynamic layers; none of these tests relaxes the 0.05 m margin, the 0.01 m
sweep step, the sample ceiling or the three-vehicle model.
"""
import dataclasses
import math
import unittest

import numpy as np

from fwis_v3_optimizer.core import Grid, Pose
from fwis_v3_optimizer.formation import FormationPose, VehiclePose
from fwis_v3_optimizer.formation_collision import FormationCollisionChecker
from fwis_v3_optimizer.formation_continuous_a.collision import (
    audit_collision,
    error_envelope,
    make_sweep_segments,
    verify_sweep_segments,
)
from fwis_v3_optimizer.formation_continuous_a.contracts import CollisionConfig
from fwis_v3_optimizer.formation_continuous_a.geometry import VirtualAPath
from fwis_v3_optimizer.formation_continuous_a.trajectory import (
    VirtualATimedPlan,
)

from virtual_a_g4_support import (
    COLLISION,
    CORNER_STATION_M,
    LIMITS,
    MODEL,
    SHAPES,
    TIMING,
    empty_grid,
    hold_path,
    load_synthetic,
    make_request,
    ramp_path,
    retime,
    straight_path,
    straight_profile,
)


class G4CollisionNegativeTests(unittest.TestCase):
    def assert_rejected(self, report, reason, stage=None, bound=None):
        self.assertFalse(report.passed, report.as_dict())
        self.assertEqual(report.reason, reason, report.as_dict())
        location = report.location if isinstance(report.location, dict) else {}
        if stage is not None:
            self.assertEqual(location.get("stage"), stage, report.as_dict())
        if bound is not None:
            self.assertEqual(location.get("bound"), bound, report.as_dict())

    def test_thin_obstacle_between_coarse_samples_is_rejected(self):
        grid, metadata = load_synthetic("thin_barrier")
        request, timing, _context = retime(
            straight_path(10.0), grid)
        self.assertEqual(timing.status, "DYNAMIC_FEASIBLE")

        # The barrier is one 0.05 m cell column at x=5.0..5.05.  Coarse static
        # samples at 4.5 m and 5.5 m are free; the continuous sweep is not.
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        headings = {1: 0.0, 2: 0.0, 3: 0.0}
        for station in (4.5, 5.5):
            report = checker.static_check(
                FormationPose(station, 0.0, 0.0), headings)
            self.assertTrue(report.success, report.reason)
        midpoint = checker.static_check(
            FormationPose(5.0, 0.0, 0.0), headings)
        self.assertFalse(midpoint.success)
        self.assertEqual(midpoint.reason, "MAP_OBB_COLLISION")

        report = audit_collision(request, timing.plan, grid, COLLISION)
        self.assert_rejected(report, "MAP_OBB_COLLISION",
                             stage="analytic_sweep")
        self.assertEqual(report.grid_sha256, metadata["grid_sha256"])
        self.assertGreater(report.check_count, 0)
        trace = report.metrics["sweep_trace"]
        self.assertTrue(any(entry.get("kind") == "sample" for entry in trace))
        self.assertGreater(report.check_count, 0)

    def test_corner_collision_uses_actual_front_heading(self):
        grid, _metadata = load_synthetic("arc_corner")
        path = ramp_path(10.0, 0.2)
        request, timing, _context = retime(path, grid)
        self.assertEqual(timing.status, "DYNAMIC_FEASIBLE")

        state = path.state_at_s(CORNER_STATION_M)
        vehicle = next(
            item for item in state.vehicles
            if item.vehicle_id == 1)
        self.assertGreater(abs(vehicle.theta - state.qf.psi), 0.03)

        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        actual = checker.static_check(
            state.qf, {item.vehicle_id: item.theta for item in state.vehicles})
        psi_front = {
            item.vehicle_id: (state.qf.psi if item.vehicle_id == 1
                              else item.theta)
            for item in state.vehicles}
        psi_only = checker.static_check(state.qf, psi_front)
        self.assertFalse(actual.success, actual.reason)
        self.assertTrue(psi_only.success,
                        "the corner case must not collide when psi replaces "
                        "the real front heading")

        rows, columns = grid.occ.shape
        center_ix = int((vehicle.x - grid.ox) / grid.res)
        center_iy = int((vehicle.y - grid.oy) / grid.res)
        self.assertFalse(grid.occ[center_iy, center_ix])
        corner_x = vehicle.x + math.cos(vehicle.theta) * 0.369 \
            - math.sin(vehicle.theta) * 0.275
        corner_y = vehicle.y + math.sin(vehicle.theta) * 0.369 \
            + math.cos(vehicle.theta) * 0.275
        corner_ix = int((corner_x - grid.ox) / grid.res)
        corner_iy = int((corner_y - grid.oy) / grid.res)
        self.assertTrue(grid.occ[corner_iy, corner_ix])
        self.assertGreater(
            math.hypot(
                grid.ox + (corner_ix + 0.5) * grid.res - vehicle.x,
                grid.oy + (corner_iy + 0.5) * grid.res - vehicle.y),
            0.4)

        report = audit_collision(request, timing.plan, grid, COLLISION)
        self.assert_rejected(report, "MAP_OBB_COLLISION",
                             stage="analytic_sweep")
        self.assertEqual(report.location.get("vehicle_id"), 1)

    def test_same_model_changed_map_limits_and_goal_are_rejected(self):
        grid = load_synthetic("corridor")[0]
        request, timing, _context = retime(straight_path(10.0), grid)
        other_grid = load_synthetic("boundary_goal")[0]
        self.assert_rejected(
            audit_collision(request, timing.plan, other_grid, COLLISION),
            "GRID_DIGEST_MISMATCH")

        changed_collision = CollisionConfig(baseline_margin_m=0.06)
        self.assert_rejected(
            audit_collision(request, timing.plan, grid, changed_collision),
            "COLLISION_CONFIG_MISMATCH")

        changed_limits = make_request(
            request.start_qf, request.goal_qf,
            limits=dataclasses.replace(LIMITS, v_max=0.7))
        self.assert_rejected(
            audit_collision(changed_limits, timing.plan, grid, COLLISION),
            "PLANNING_CONTEXT_MISMATCH")

        changed_goal = make_request(
            request.start_qf,
            FormationPose(request.goal_qf.x + 0.1,
                          request.goal_qf.y, request.goal_qf.psi))
        self.assert_rejected(
            audit_collision(changed_goal, timing.plan, grid, COLLISION),
            "PLANNING_CONTEXT_MISMATCH")

    def test_missing_or_stale_planning_context_digest_is_rejected(self):
        grid = load_synthetic("corridor")[0]
        request, timing, _context = retime(straight_path(10.0), grid)
        plan = timing.plan
        selection = plan.to_model_dict()["timing"]["selection"]
        no_context = VirtualATimedPlan._build_motion(
            request, plan.path, plan.duration_s, selection, None)
        self.assert_rejected(
            audit_collision(request, no_context, grid, COLLISION),
            "PLANNING_CONTEXT_REQUIRED")

        # Deliberate test tampering: a stale model-core context must not be
        # accepted merely because the public convenience digest is unchanged.
        plan._model_core["planning_context"]["timing_config"][
            "max_total_duration_s"] = 999.0
        self.assert_rejected(
            audit_collision(request, plan, grid, COLLISION),
            "PLANNING_CONTEXT_MISMATCH")

    def test_missing_delta_factor_is_caught_by_independent_bounds(self):
        grid = load_synthetic("arc_corner")[0]
        request, timing, _context = retime(ramp_path(10.0, 0.2), grid)
        plan = timing.plan
        segments = make_sweep_segments(plan, COLLISION)
        self.assertTrue(verify_sweep_segments(
            plan, segments, COLLISION).passed)
        segment = segments[0]
        delta = segment.station_end - segment.station_start
        self.assertGreater(delta, 0.0)

        rate_omitted = dataclasses.replace(
            segment,
            formation_rate_bound_rad_per_lambda=(
                segment.formation_rate_bound_rad_per_lambda / delta),
            body_rate_bounds_rad_per_lambda={
                vehicle_id: value / delta
                for vehicle_id, value in
                segment.body_rate_bounds_rad_per_lambda.items()})
        verified = verify_sweep_segments(
            plan, (rate_omitted,), COLLISION)
        self.assertFalse(verified.passed, verified.as_dict())
        self.assertEqual(verified.reason, "SWEEP_BOUND_UNDERESTIMATE")
        self.assertEqual(verified.location.get("bound"), "formation_rate")

        center_omitted = dataclasses.replace(
            segment, center_speed_bound_m_per_lambda=1.0)
        verified = verify_sweep_segments(
            plan, (center_omitted,), COLLISION)
        self.assertFalse(verified.passed, verified.as_dict())
        self.assertEqual(verified.reason, "SWEEP_BOUND_UNDERESTIMATE")
        self.assertEqual(verified.location.get("bound"), "center_speed")

    def test_whole_formation_circumcircle_blocked_but_obb_passes(self):
        grid = load_synthetic("door_circumcircle")[0]
        request, timing, _context = retime(straight_path(10.0), grid)
        report = audit_collision(request, timing.plan, grid, COLLISION)
        self.assertTrue(report.passed, report.as_dict())

        door_rows = [
            iy for iy in range(grid.occ.shape[0])
            if not grid.occ[iy, 140]]
        gap_m = (max(door_rows) - min(door_rows) + 1) * grid.res
        formation_radius = 1.0
        body_radius = math.hypot(0.738 / 2.0, 0.500 / 2.0)
        circumscribed_diameter = 2.0 * (formation_radius + body_radius)
        self.assertGreater(circumscribed_diameter, gap_m)
        self.assertGreater(circumscribed_diameter / 2.0, gap_m / 2.0)
        self.assertGreater(
            report.metrics["checker_statistics"]["obb_pair_queries"], 0)
        self.assertNotIn("circumcircle", report.metrics)

    def test_sample_budget_is_a_failure_not_a_pass(self):
        grid = load_synthetic("corridor")[0]
        config = COLLISION.__class__(max_samples_per_segment=1)
        request, timing, _context = retime(
            straight_path(10.0), grid, collision=config)
        report = audit_collision(request, timing.plan, grid, config)
        self.assert_rejected(
            report, "SWEEP_SAMPLE_BUDGET_EXCEEDED",
            stage="analytic_sweep")
        self.assertGreater(report.check_count, 0)

    def test_map_boundary_goal_is_rejected(self):
        grid = load_synthetic("boundary_goal")[0]
        request, timing, _context = retime(straight_path(10.0), grid)
        report = audit_collision(request, timing.plan, grid, COLLISION)
        self.assert_rejected(report, "MAP_OBB_COLLISION",
                             stage="static_goal")
        stages = {record["stage"]: record
                  for record in report.integration["static"]}
        self.assertTrue(stages["start"]["passed"])
        self.assertFalse(stages["goal"]["passed"])

    def test_unknown_cells_are_occupied_and_rejected(self):
        grid, metadata = load_synthetic("unknown_gray")
        self.assertEqual(metadata["unknown_cells_are"], "occupied")
        self.assertGreater(int(grid.occ.sum()), 0)
        request, timing, _context = retime(straight_path(10.0), grid)
        report = audit_collision(request, timing.plan, grid, COLLISION)
        self.assert_rejected(report, "MAP_OBB_COLLISION",
                             stage="analytic_sweep")

    def test_static_start_collision_is_rejected(self):
        grid = load_synthetic("blocked_start")[0]
        request, timing, _context = retime(straight_path(10.0), grid)
        report = audit_collision(request, timing.plan, grid, COLLISION)
        self.assert_rejected(report, "MAP_OBB_COLLISION",
                             stage="static_start")
        self.assertFalse(report.integration["static"][0]["passed"])

    def test_no_motion_hold_collision_is_rejected(self):
        grid = load_synthetic("blocked_start")[0]
        path = hold_path()
        request, timing, _context = retime(
            path, grid, request=make_request(
                path.start_qf, path.start_qf),
            expected=("NO_MOTION",))
        self.assertEqual(timing.plan.kind, "hold")
        report = audit_collision(request, timing.plan, grid, COLLISION)
        self.assert_rejected(report, "MAP_OBB_COLLISION", stage="hold")
        self.assertGreater(report.check_count, 0)


# Near-contact counterexamples for the factor-2 envelope.  The legacy checker
# expands each body by margin/2 per side, so a numerical envelope e is only
# covered when the margin increase is at least 2*e.  Every geometry below is
# placed at a distance chosen explicitly between the old per-side increment
# e/2 and the corrected per-side increment e; the old margin baseline+e passes,
# the corrected margin rejects.
BASE_MARGIN_M = 0.05
VEHICLE_LENGTH_M = 0.738
VEHICLE_WIDTH_M = 0.500


def _amplified(path, position=1e-7, angle=1e-9):
    """Test-only legal worst-case path bounds, not above the G1 ceiling."""
    path._position_error_bound_m = float(position)
    path._angle_error_bound_rad = float(angle)
    return path


def _grid(origin_x, origin_y, width_cells, height_cells, occupied=(),
          resolution=0.05):
    occ = np.zeros((int(height_cells), int(width_cells)), dtype=bool)
    for row, column in occupied:
        occ[int(row), int(column)] = True
    return Grid(occ, float(resolution), float(origin_x), float(origin_y))


def _half_extents(margin):
    return ((VEHICLE_LENGTH_M + float(margin)) / 2.0,
            (VEHICLE_WIDTH_M + float(margin)) / 2.0)


def _minimum_map_distance(checker, qf, headings):
    distances = checker._distance_objects(0.0)
    poses = checker.model.vehicle_poses(qf, headings)
    return min(distances[pose.vehicle_id].distance(
        Pose(pose.x, pose.y, pose.theta)) for pose in poses)


class G4EnvelopePerSideTests(unittest.TestCase):
    """P0: the full-width margin must cover the envelope on every side.

    These tests never compare only ``margin >= baseline + e``; each one builds
    a real legacy checker, computes its actual binary64 half extent
    ``(shape + margin)/2``, and shows old-pass/corrected-reject on the same
    physical geometry.
    """

    def assert_rejected(self, report, reason, stage):
        self.assertFalse(report.passed, report.as_dict())
        self.assertEqual(report.reason, reason, report.as_dict())
        self.assertEqual(report.location.get("stage"), stage,
                         report.as_dict())

    def _envelope(self, path, grid, *, request=None):
        if request is None:
            request = make_request(path.start_qf, path.endpoint())
        request, timing, _context = retime(path, grid, request=request)
        return request, timing, error_envelope(timing.plan, COLLISION)

    def _static_pair(self, grid, qf, headings, margin_old, margin_new):
        old_checker = FormationCollisionChecker(grid, MODEL, SHAPES,
                                                margin_old)
        new_checker = FormationCollisionChecker(grid, MODEL, SHAPES,
                                                margin_new)
        self.assertEqual(old_checker._base_half_extents[1],
                         _half_extents(margin_old))
        self.assertEqual(new_checker._base_half_extents[1],
                         _half_extents(margin_new))
        old_report = old_checker.static_check(qf, headings)
        new_report = new_checker.static_check(qf, headings)
        self.assertTrue(old_report.success, old_report.reason)
        self.assertFalse(new_report.success)
        self.assertEqual(new_report.reason, "MAP_OBB_COLLISION")
        old_distance = _minimum_map_distance(old_checker, qf, headings)
        new_distance = _minimum_map_distance(new_checker, qf, headings)
        self.assertGreater(old_distance, 0.0)
        self.assertLessEqual(new_distance, 0.0)
        return old_distance, new_distance

    def test_static_start_map_obstacle_old_margin_passes_corrected_rejects(self):
        # Barrier cell column x=5.00..5.05 spanning the vehicle-1 body rows.
        grid = _grid(0.0, -2.0, 200, 100,
                     {(row, 100) for row in range(30, 50)})
        probe = _amplified(VirtualAPath(
            FormationPose(0.0, 0.0, 0.0), straight_profile(0.5), MODEL))
        _request, _timing, probe_env = self._envelope(probe, grid)
        envelope = probe_env["maximum_envelope_m"]
        margin_old = BASE_MARGIN_M + envelope
        margin_new = probe_env["margin_used_m"]
        half_base = _half_extents(BASE_MARGIN_M)[0]
        half_old = _half_extents(margin_old)[0]
        half_new = _half_extents(margin_new)[0]
        # The old per-side increment is e/2; the corrected one is >= e.
        self.assertLess(half_old - half_base, envelope)
        self.assertAlmostEqual(half_old - half_base, envelope / 2.0,
                               delta=1e-15)
        self.assertGreaterEqual(half_new - half_base, envelope)
        delta = 0.75 * envelope
        self.assertGreater(delta, half_old - half_base)
        self.assertLess(delta, half_new - half_base)
        center_x = 5.05 + half_base + delta
        start_x = center_x - 1.0  # vehicle 1 offset is (1, 0) at heading 0

        path = _amplified(VirtualAPath(
            FormationPose(start_x, 0.0, 0.0), straight_profile(0.5), MODEL))
        request, timing, env = self._envelope(path, grid)
        self.assertEqual(env["maximum_envelope_m"], envelope)
        state = timing.plan.path.state_at_s(0.0)
        headings = {vehicle.vehicle_id: vehicle.theta
                    for vehicle in state.vehicles}
        self._static_pair(grid, state.qf, headings, margin_old, margin_new)
        report = audit_collision(request, timing.plan, grid, COLLISION)
        self.assert_rejected(report, "MAP_OBB_COLLISION", "static_start")
        self.assertEqual(report.location.get("vehicle_id"), 1)
        self.assertEqual(report.metrics["margin"]["margin_used_m"], margin_new)

    def test_static_goal_map_boundary_old_margin_passes_corrected_rejects(self):
        # Free map x in [-2, 6], y in [-2, 3]; the goal body reaches y=3.
        grid = _grid(-2.0, -2.0, 160, 100)
        probe = _amplified(VirtualAPath(
            FormationPose(3.0, 0.0, math.pi / 2.0),
            straight_profile(1.2), MODEL))
        probe_headings = {1: math.pi / 2.0, 2: math.pi / 2.0,
                          3: math.pi / 2.0}
        probe_request = make_request(probe.start_qf, probe.endpoint(),
                                     start_headings=probe_headings)
        _request, _timing, probe_env = self._envelope(
            probe, grid, request=probe_request)
        envelope = probe_env["maximum_envelope_m"]
        margin_old = BASE_MARGIN_M + envelope
        margin_new = probe_env["margin_used_m"]
        half_base = _half_extents(BASE_MARGIN_M)[0]
        half_old_increment = _half_extents(margin_old)[0] - half_base
        self.assertLess(half_old_increment, envelope)
        self.assertAlmostEqual(half_old_increment, envelope / 2.0,
                               delta=1e-15)
        self.assertGreaterEqual(
            _half_extents(margin_new)[0] - half_base, envelope)
        delta = 0.75 * envelope
        goal_y = 3.0 - 1.0 - half_base - delta  # vehicle 1 offset (1,0)
        start_y = goal_y - 1.2

        path = _amplified(VirtualAPath(
            FormationPose(3.0, start_y, math.pi / 2.0),
            straight_profile(1.2), MODEL))
        request, timing, env = self._envelope(
            path, grid,
            request=make_request(path.start_qf, path.endpoint(),
                                 start_headings=probe_headings))
        self.assertEqual(env["maximum_envelope_m"], envelope)
        headings = {vehicle.vehicle_id: vehicle.theta
                    for vehicle in timing.plan.path.state_at_s(0.0).vehicles}
        start_state = timing.plan.path.state_at_s(0.0, side="right")
        old_checker = FormationCollisionChecker(grid, MODEL, SHAPES,
                                                margin_old)
        new_checker = FormationCollisionChecker(grid, MODEL, SHAPES,
                                                margin_new)
        self.assertTrue(old_checker.static_check(
            start_state.qf, headings).success)
        self.assertTrue(new_checker.static_check(
            start_state.qf, headings).success)
        goal_state = timing.plan.path.state_at_s(
            timing.plan.path.length_m, side="left")
        goal_headings = {vehicle.vehicle_id: vehicle.theta
                         for vehicle in goal_state.vehicles}
        self._static_pair(grid, goal_state.qf, goal_headings,
                          margin_old, margin_new)
        self.assertGreater(
            _minimum_map_distance(old_checker, goal_state.qf,
                                  goal_headings), 0.0)
        report = audit_collision(request, timing.plan, grid, COLLISION)
        self.assert_rejected(report, "MAP_OBB_COLLISION", "static_goal")
        self.assertEqual(report.location.get("vehicle_id"), 1)

    def test_no_motion_hold_old_margin_passes_corrected_rejects(self):
        grid = _grid(0.0, -2.0, 200, 100,
                     {(row, 100) for row in range(30, 50)})
        probe = _amplified(hold_path(FormationPose(0.0, 0.0, 0.0)))
        probe_request = make_request(probe.start_qf, probe.start_qf)
        probe_request, probe_timing, _context = retime(
            probe, grid, request=probe_request, expected=("NO_MOTION",))
        probe_env = error_envelope(probe_timing.plan, COLLISION)
        envelope = probe_env["maximum_envelope_m"]
        margin_old = BASE_MARGIN_M + envelope
        margin_new = probe_env["margin_used_m"]
        half_base = _half_extents(BASE_MARGIN_M)[0]
        half_old_increment = _half_extents(margin_old)[0] - half_base
        self.assertLess(half_old_increment, envelope)
        self.assertAlmostEqual(half_old_increment, envelope / 2.0,
                               delta=1e-15)
        self.assertGreaterEqual(_half_extents(margin_new)[0] - half_base,
                                envelope)
        center_x = 5.05 + half_base + 0.75 * envelope
        pose = FormationPose(center_x - 1.0, 0.0, 0.0)
        path = _amplified(hold_path(pose))
        request = make_request(pose, pose)
        request, timing, _context = retime(
            path, grid, request=request, expected=("NO_MOTION",))
        self.assertEqual(timing.plan.kind, "hold")
        env = error_envelope(timing.plan, COLLISION)
        self.assertEqual(env["maximum_envelope_m"], envelope)
        state = timing.plan.path.state_at_s(0.0)
        headings = {vehicle.vehicle_id: vehicle.theta
                    for vehicle in state.vehicles}
        self._static_pair(grid, state.qf, headings, margin_old, margin_new)
        report = audit_collision(request, timing.plan, grid, COLLISION)
        self.assert_rejected(report, "MAP_OBB_COLLISION", "hold")
        self.assertEqual(report.location.get("vehicle_id"), 1)

    def test_motion_sweep_padding_is_not_numerical_envelope(self):
        # Full-height barrier at x=5.00..5.05; the path runs parallel in +y.
        grid = _grid(0.0, -2.0, 200, 100,
                     {(row, 100) for row in range(100)})
        heading = math.pi / 2.0
        probe = _amplified(VirtualAPath(
            FormationPose(0.0, 0.0, heading), straight_profile(0.5), MODEL))
        probe_request = make_request(
            probe.start_qf, probe.endpoint(),
            start_headings={1: heading, 2: heading, 3: heading})
        _request, _timing, probe_env = self._envelope(
            probe, grid, request=probe_request)
        envelope = probe_env["maximum_envelope_m"]
        margin_old = BASE_MARGIN_M + envelope
        margin_new = probe_env["margin_used_m"]
        padding = COLLISION.max_corner_step_m
        self.assertEqual(padding, 0.01)
        self.assertEqual(probe_env["sweep_motion_padding_m"], padding)
        half_width_base = _half_extents(BASE_MARGIN_M)[1]
        half_width_old = _half_extents(margin_old)[1]
        half_width_new = _half_extents(margin_new)[1]
        self.assertLess(half_width_old - half_width_base, envelope)
        self.assertAlmostEqual(half_width_old - half_width_base,
                               envelope / 2.0, delta=1e-15)
        self.assertGreaterEqual(half_width_new - half_width_base, envelope)
        delta = 0.75 * envelope
        center_x = 5.0 - half_width_base - padding - delta
        start_x = center_x  # vehicle 1 offset (1,0) rotates to (0,1)

        path = _amplified(VirtualAPath(
            FormationPose(start_x, -1.0, heading), straight_profile(0.5),
            MODEL))
        request, timing, env = self._envelope(
            path, grid,
            request=make_request(path.start_qf, path.endpoint(),
                                 start_headings={1: heading, 2: heading,
                                                 3: heading}))
        self.assertEqual(env["maximum_envelope_m"], envelope)
        self.assertEqual(env["contexts"]["motion_sweep"]["checker_padding_m"],
                         padding)
        self.assertEqual(
            env["contexts"]["static_start_goal"]["checker_padding_m"], 0.0)
        self.assertEqual(
            env["contexts"]["map_boundary"]["checker_padding_m"], 0.0)
        self.assertEqual(
            env["contexts"]["vehicle_vehicle"]["checker_padding_m"], 0.0)
        self.assertIn("sampling-interval motion protection",
                      env["contexts"]["motion_sweep"]["padding_role"])
        state = timing.plan.path.state_at_s(0.0)
        headings = {vehicle.vehicle_id: vehicle.theta
                    for vehicle in state.vehicles}
        old_checker = FormationCollisionChecker(grid, MODEL, SHAPES,
                                                margin_old)
        new_checker = FormationCollisionChecker(grid, MODEL, SHAPES,
                                                margin_new)
        self.assertEqual(old_checker._base_half_extents[1],
                         _half_extents(margin_old))
        self.assertEqual(new_checker._base_half_extents[1],
                         _half_extents(margin_new))
        self.assertTrue(old_checker.static_check(state.qf, headings).success)
        self.assertTrue(new_checker.static_check(state.qf, headings).success)
        old_sweep = old_checker._check_state(state.qf, headings, padding)
        new_sweep = new_checker._check_state(state.qf, headings, padding)
        self.assertTrue(old_sweep.success, old_sweep.reason)
        self.assertFalse(new_sweep.success)
        self.assertEqual(new_sweep.reason, "MAP_OBB_COLLISION")
        report = audit_collision(request, timing.plan, grid, COLLISION)
        self.assert_rejected(report, "MAP_OBB_COLLISION", "analytic_sweep")
        self.assertEqual(report.location.get("vehicle_id"), 1)

    def test_vehicle_pair_envelope_uses_two_body_gap_reduction(self):
        grid = empty_grid()
        path = _amplified(straight_path(10.0))
        _request, timing, env = self._envelope(path, grid)
        envelope = env["maximum_envelope_m"]
        margin_old = BASE_MARGIN_M + envelope
        margin_new = env["margin_used_m"]
        old_checker = FormationCollisionChecker(grid, MODEL, SHAPES,
                                                margin_old)
        new_checker = FormationCollisionChecker(grid, MODEL, SHAPES,
                                                margin_new)
        old_half = old_checker._base_half_extents[1]
        new_half = new_checker._base_half_extents[1]
        self.assertEqual(old_half, _half_extents(margin_old))
        self.assertEqual(new_half, _half_extents(margin_new))
        self.assertGreaterEqual(env["pair_envelope_required_m"],
                                2.0 * envelope)
        self.assertGreaterEqual(env["margin_growth_lower_m"],
                                env["pair_envelope_required_m"])
        # Axis-aligned bodies: overlap iff separation <= length + margin.
        separation = VEHICLE_LENGTH_M + BASE_MARGIN_M + 1.5 * envelope
        self.assertGreater(separation, VEHICLE_LENGTH_M + margin_old)
        self.assertLessEqual(separation, VEHICLE_LENGTH_M + margin_new)
        first = VehiclePose(1, 0.0, 0.0, 0.0)
        second = VehiclePose(2, separation, 0.0, 0.0)
        self.assertFalse(FormationCollisionChecker._obb_overlap(
            first, old_half, second, old_half))
        self.assertTrue(FormationCollisionChecker._obb_overlap(
            first, new_half, second, new_half))

    def test_envelope_closure_radius_and_contexts(self):
        grid = empty_grid()
        for path in (straight_path(10.0),
                     _amplified(straight_path(10.0))):
            _request, timing, env = self._envelope(path, grid)
            envelope = env["maximum_envelope_m"]
            margin = env["margin_used_m"]
            base = env["baseline_margin_m"]
            self.assertGreater(envelope, 0.0)
            self.assertGreaterEqual(margin, env["margin_minimum_required_m"])
            self.assertGreaterEqual(margin, base + 2.0 * envelope)
            self.assertGreaterEqual(env["per_side_margin_m"], envelope)
            self.assertEqual(env["per_side_required_m"], envelope)
            self.assertGreaterEqual(env["closure_gap_m"], 0.0)
            radius = math.hypot((VEHICLE_LENGTH_M + margin) / 2.0,
                                (VEHICLE_WIDTH_M + margin) / 2.0)
            self.assertGreaterEqual(env["radius_upper_m"], radius)
            self.assertGreaterEqual(env["checker_radius_upper_m"], radius)
            self.assertEqual(env["checker_half_length_m"],
                             (VEHICLE_LENGTH_M + margin) / 2.0)
            self.assertEqual(env["checker_half_width_m"],
                             (VEHICLE_WIDTH_M + margin) / 2.0)
            for record in env["vehicles"]:
                self.assertEqual(record["inflated_body_radius_upper_m"],
                                 env["radius_upper_m"])
                direct = (record["eps_position_m"]
                          + (record["offset_hypot_float_m"]
                             + record["inflated_body_radius_upper_m"])
                          * record["eps_formation_heading_rad"]
                          + record["inflated_body_radius_upper_m"]
                          * record["eps_relative_heading_rad"])
                self.assertGreaterEqual(record["envelope_m"], direct)
            self.assertEqual(set(env["contexts"]),
                             {"static_start_goal", "no_motion_hold",
                              "motion_sweep", "map_boundary",
                              "vehicle_vehicle"})
            for name in ("static_start_goal", "no_motion_hold",
                         "motion_sweep", "map_boundary"):
                self.assertTrue(env["contexts"][name]["covered"], name)
                self.assertGreaterEqual(
                    env["contexts"][name]["required_per_side_m"], 0.0)
            self.assertTrue(env["contexts"]["vehicle_vehicle"]["covered"])
            self.assertGreaterEqual(
                env["contexts"]["vehicle_vehicle"][
                    "actual_two_body_gap_reduction_lower_m"],
                env["contexts"]["vehicle_vehicle"][
                    "required_pair_envelope_m"])


if __name__ == "__main__":
    unittest.main()