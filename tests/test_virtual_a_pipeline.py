"""G4 end-to-end autonomous pipeline and route-guidance tests."""
import json
import math
from pathlib import Path
import unittest
from unittest import mock

import numpy as np

from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a import optimizer as optimizer_module
from fwis_v3_optimizer.formation_continuous_a import pipeline as pipeline_module
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    G4_EXECUTABLE_STATUSES,
    RouteGuideConfig,
    request_fingerprint,
)
from fwis_v3_optimizer.formation_continuous_a.pipeline import (
    make_route_guides,
    plan_virtual_a,
)

from virtual_a_g4_support import (
    load_synthetic,
    make_request,
    map_request,
    retime,
    straight_path,
    geometry_result_for,
    scene_map_request,
    empty_grid,
)


SCENES = ("S1", "S1_mirror", "S2", "S2_mirror", "S3", "S3_mirror")
TIMING_KEYS = (
    "route_generation_s", "geometry_solve_s", "retime_s",
    "dynamics_audit_s", "collision_audit_s", "result_assembly_s")


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class G4PipelineTests(unittest.TestCase):
    def assert_three_layer_success(self, result, grid_metadata=None):
        self.assertEqual(result.status, "SUCCESS", result.as_dict())
        self.assertTrue(result.executable, result.as_dict())
        self.assertIn(result.status, G4_EXECUTABLE_STATUSES)
        self.assertIsNotNone(result.plan)
        self.assertIsNotNone(result.path)
        for report in (result.geometry_audit, result.dynamic_audit,
                       result.collision_audit):
            self.assertIsNotNone(report)
            self.assertTrue(report.passed, report.as_dict())
        self.assertEqual(result.plan.planning_context_sha256(),
                         result.planning_context_sha256)
        self.assertEqual(result.plan.request_sha256, result.request_sha256)
        self.assertEqual(result.plan.model_sha256(), result.model_sha256)
        self.assertGreater(result.checks, 0)
        self.assertGreater(result.collision_audit.check_count, 0)
        self.assertGreater(result.metrics["collision_checks"], 0)
        if grid_metadata is not None:
            self.assertEqual(result.grid_sha256,
                             grid_metadata["grid_sha256"])
            self.assertEqual(result.grid_sha256,
                             result.collision_audit.grid_sha256)
        endpoint = result.path.endpoint()
        goal = result.plan._request.goal_qf
        self.assertLessEqual(
            math.hypot(endpoint.x - goal.x, endpoint.y - goal.y), 0.005)
        self.assertLessEqual(abs(_wrap(endpoint.psi - goal.psi)),
                             0.003490658503988659)
        for key in TIMING_KEYS:
            self.assertIn(key, result.timings)
            self.assertGreaterEqual(result.timings[key], 0.0)
        self.assertLessEqual(
            sum(result.timings[key] for key in TIMING_KEYS),
            result.wall_time_s + 1e-9)
        self.assertLessEqual(len(result.routes), 3)
        self.assertEqual(
            result.search["budget"]["total_soft_budget_s"], 60.0)
        self.assertEqual(result.search["budget"]["max_recoveries"], 1)
        self.assertFalse(
            result.search["budget"]["hard_real_time_guarantee"])
        # C3 scheduling caps and evidence fields.  A successful recovery is
        # allowed (S3 needs exactly one endpoint-only recovery), but the
        # scheduling caps and the recorded first-failure layer must hold.
        self.assertLessEqual(result.search["route_count"], 3)
        self.assertLessEqual(result.search["initial_count"], 6)
        self.assertLessEqual(result.search["recovery_count"], 1)
        self.assertEqual(
            len(result.search["recoveries"]),
            result.search["recovery_count"])
        self.assertIn("first_failure_layer", result.search)
        self.assertIn("last_reason", result.search)
        if result.search["recovery_count"]:
            self.assertTrue(result.search["recoveries"])
            self.assertTrue(result.search["first_failure_layer"])
            self.assertTrue(result.search["last_reason"])

    def test_route_guides_are_finite_topology_hints_only(self):
        map_request_value, metadata, _payload = scene_map_request("S2")
        grid = map_request_value.grid
        request = map_request_value.request
        guides = make_route_guides(
            grid, request.start_qf, request.goal_qf,
            RouteGuideConfig(clearance_weight=2.0))
        self.assertGreaterEqual(len(guides), 1)
        self.assertLessEqual(len(guides), 3)
        height, width = grid.occ.shape
        for guide in guides:
            self.assertEqual(guide.grid_sha256, metadata["grid_sha256"])
            self.assertTrue(guide.points)
            self.assertGreaterEqual(len(guide.points), 2)
            self.assertGreaterEqual(guide.length_m, 0.0)
            self.assertLessEqual(
                math.hypot(guide.points[0][0] - request.start_qf.x,
                           guide.points[0][1] - request.start_qf.y), 0.05)
            self.assertLessEqual(
                math.hypot(guide.points[-1][0] - request.goal_qf.x,
                           guide.points[-1][1] - request.goal_qf.y), 0.05)
            previous = None
            path_length = 0.0
            for x, y in guide.points:
                self.assertTrue(math.isfinite(x) and math.isfinite(y))
                ix = int((x - grid.ox) / grid.res)
                iy = int((y - grid.oy) / grid.res)
                self.assertTrue(0 <= ix < width and 0 <= iy < height)
                self.assertFalse(grid.occ[iy, ix])
                if previous is not None:
                    path_length += math.hypot(
                        x - previous[0], y - previous[1])
                previous = (x, y)
            self.assertAlmostEqual(path_length, guide.length_m, places=9)
            self.assertEqual(guide.source,
                             "astar_primary" if guide is guides[0]
                             else guide.source)
            self.assertFalse(hasattr(guide, "profile"))
            self.assertFalse(hasattr(guide, "curvature"))

    def test_six_frozen_scenes_pass_all_three_audits_autonomously(self):
        for name in SCENES:
            with self.subTest(scene=name):
                map_request_value, metadata, payload = scene_map_request(name)
                self.assertEqual(
                    set(payload),
                    {"schema", "stage", "request", "planner_config",
                     "timing_config", "collision_config", "map", "note"})
                # The pipeline must not read any file, particularly not a
                # reference, manifest or old audit.
                with mock.patch.object(
                        Path, "read_text",
                        side_effect=AssertionError("unexpected file read")), \
                        mock.patch.object(
                            Path, "read_bytes",
                            side_effect=AssertionError(
                                "unexpected file read")), \
                        mock.patch.object(
                            json, "load",
                            side_effect=AssertionError("unexpected JSON load")):
                    result = plan_virtual_a(map_request_value)
                self.assert_three_layer_success(result, metadata)
                self.assertEqual(
                    result.request_sha256,
                    request_fingerprint(map_request_value.request))
                self.assertGreater(len(result.search["rounds"]), 0)
                self.assertEqual(
                    result.search["rounds"][-1]["geometry_status"],
                    "GEOMETRY_FEASIBLE")
                self.assertTrue(
                    result.search["rounds"][-1]["collision_passed"])
                # C3 rework 01: the whole request shares one initial pool.
                self.assertLessEqual(
                    result.search["initial_count_total"],
                    min(6, int(map_request_value.planner_config.max_initials)))
                self.assertEqual(
                    len(result.search["initial_keys"]),
                    result.search["initial_count_total"])
                self.assertEqual(
                    result.search["duplicate_initial_launch_count"], 0)
                self.assertLessEqual(result.search["recovery_count"], 1)

    def test_circumcircle_blocked_door_is_solved_by_real_obb_audit(self):
        grid, _metadata = load_synthetic("door_circumcircle")
        request = make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(10.0, 0.0, 0.0))
        result = plan_virtual_a(map_request(request, grid))
        self.assert_three_layer_success(result)
        door_rows = [
            iy for iy in range(grid.occ.shape[0])
            if not grid.occ[iy, 140]]
        gap_m = (max(door_rows) - min(door_rows) + 1) * grid.res
        body_radius = math.hypot(0.738 / 2.0, 0.500 / 2.0)
        self.assertGreater(2.0 * (1.0 + body_radius), gap_m)
        self.assertTrue(result.collision_audit.metrics["map_used"])

    def test_no_motion_free_hold_passes_and_blocked_hold_fails(self):
        grid = empty_grid()
        request = make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(0.0, 0.0, 0.0))
        result = plan_virtual_a(map_request(request, grid))
        self.assertEqual(result.status, "NO_MOTION")
        self.assertTrue(result.executable)
        self.assertEqual(result.plan.kind, "hold")
        self.assertEqual(result.path.length_m, 0.0)
        self.assertTrue(result.collision_audit.passed)
        self.assertEqual(result.collision_audit.metrics["kind"], "hold")

        blocked_grid = load_synthetic("blocked_start")[0]
        result = plan_virtual_a(map_request(request, blocked_grid))
        self.assertEqual(result.status, "COLLISION_FAILED",
                         result.as_dict())
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertIsNone(result.path)
        self.assertIsNone(result.projection)
        self.assertEqual(result.first_failure_layer, "COLLISION")
        self.assertFalse(result.collision_audit.passed)
        self.assertEqual(result.collision_audit.reason,
                         "MAP_OBB_COLLISION")
        self.assertEqual(
            result.collision_audit.location.get("stage"), "hold")
        self.assertGreater(result.checks, 0)

    def test_no_motion_hold_checks_map_when_start_cell_is_occupied(self):
        grid = empty_grid()
        # The formation centre cell blocks the A* start, while the car-1 body
        # cell makes the static hold certificate itself fail.  A route search
        # must not pre-empt that mandatory map/vehicle audit.
        grid.occ[60, 40] = True
        grid.occ[60, 60] = True
        request = make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(0.0, 0.0, 0.0))
        result = plan_virtual_a(map_request(request, grid))
        self.assertEqual(result.status, "COLLISION_FAILED",
                         result.as_dict())
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertIsNone(result.path)
        self.assertEqual(result.first_failure_layer, "COLLISION")
        self.assertEqual(result.collision_audit.reason,
                         "MAP_OBB_COLLISION")
        self.assertEqual(
            result.collision_audit.location.get("stage"), "hold")
        self.assertGreater(result.collision_audit.check_count, 0)
        self.assertEqual(result.routes, ())

    def test_rejected_geometry_candidate_retains_evidence_but_no_plan(self):
        grid = load_synthetic("thin_barrier")[0]
        path = straight_path(10.0)
        request, timing, _context = retime(path, grid)
        geometry = geometry_result_for(path, request)
        with mock.patch.object(
                pipeline_module, "solve_open_path_guided",
                return_value=geometry):
            result = plan_virtual_a(map_request(request, grid))
        self.assertEqual(result.status, "COLLISION_FAILED",
                         result.as_dict())
        self.assertIsNone(result.plan)
        self.assertIsNone(result.path)
        self.assertIsNone(result.projection)
        self.assertEqual(result.first_failure_layer, "COLLISION")
        self.assertFalse(result.collision_audit.passed)
        self.assertEqual(result.collision_audit.reason,
                         "MAP_OBB_COLLISION")
        self.assertGreater(result.checks, 0)
        self.assertGreater(result.metrics["collision_checks"], 0)
        self.assertIsInstance(result.attempts, tuple)
        self.assertIsInstance(result.routes, tuple)
        for audit in (result.geometry_audit, result.dynamic_audit,
                      result.collision_audit):
            self.assertIsNotNone(audit)

    def test_fake_solver_success_never_becomes_an_executable_candidate(self):
        grid = load_synthetic("corridor")[0]
        request = make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(10.0, 0.0, 0.0))

        def fake_solver(_request, _config, _packet, guess,
                        wall_limit_s=None, **kwargs):
            values = np.asarray(guess.vector, dtype=float).copy()
            values[1:] = values[1:] * 1.1
            return {
                "solver": "ipopt",
                "z": values,
                "objective": 1.0,
                "solver_status": "Solve_Succeeded",
                "success": True,
                "iterations": 1,
                "detail": "fake success",
                "wall_time_s": 0.0,
            }

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_solver):
            result = plan_virtual_a(
                map_request(request, grid))
        self.assertNotIn(result.status, G4_EXECUTABLE_STATUSES)
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertIsNone(result.path)
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE",
                         result.as_dict())
        self.assertEqual(result.first_failure_layer, "GEOMETRY")
        self.assertTrue(result.attempts)
        self.assertTrue(all(
            attempt.status == "SOLVER_SUCCESS_AUDIT_REJECTED"
            for attempt in result.attempts))

    def test_total_budget_exhaustion_is_not_executable(self):
        grid = load_synthetic("corridor")[0]
        request = make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(10.0, 0.0, 0.0))
        with mock.patch.object(
                pipeline_module, "G4_TOTAL_SOFT_BUDGET_S", 0.0):
            result = plan_virtual_a(map_request(request, grid))
        self.assertEqual(result.status, "BUDGET_EXHAUSTED",
                         result.as_dict())
        self.assertEqual(result.reason, "SOFT_BUDGET_EXHAUSTED")
        self.assertEqual(result.first_failure_layer, "PLANNING")
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertIsNone(result.path)
        self.assertEqual(
            result.search["budget"]["total_soft_budget_s"], 0.0)


class SDomainSearchEvidenceTests(unittest.TestCase):
    """Step 4: attempt and search round records carry the finite S domain."""

    def _fake_success(self, _request, _config, _packet, guess,
                      wall_limit_s=None, **kwargs):
        return {
            "solver": "ipopt",
            "z": np.asarray(guess.vector, dtype=float).copy(),
            "objective": 1.0,
            "solver_status": "Solve_Succeeded",
            "success": True,
            "iterations": 1,
            "detail": "fake success",
            "wall_time_s": 0.0,
        }

    def test_attempt_and_search_round_share_the_same_finite_domain(self):
        grid = load_synthetic("corridor")[0]
        request = make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(10.0, 0.0, 0.0))
        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=self._fake_success):
            result = plan_virtual_a(map_request(request, grid))
        self.assertTrue(result.search["rounds"])
        for attempt in result.attempts:
            self.assertEqual(attempt.distance_m, 10.0)
            self.assertIsNotNone(attempt.guide_length_m)
            self.assertGreater(attempt.guide_length_m, 0.0)
            self.assertIsNotNone(attempt.s_lower_m)
            self.assertIsNotNone(attempt.s_upper_m)
            self.assertGreater(attempt.s_upper_m, attempt.s_lower_m)
            self.assertLessEqual(attempt.s_lower_m, attempt.s_solution_m)
            self.assertLessEqual(attempt.s_solution_m, attempt.s_upper_m)
            self.assertEqual(
                attempt.s_upper_active,
                abs(attempt.s_solution_m - attempt.s_upper_m) <= 1e-6)
        decisive = result.attempts[-1]
        last_round = result.search["rounds"][-1]
        self.assertEqual(last_round["distance_m"], decisive.distance_m)
        self.assertEqual(
            last_round["guide_length_m"], decisive.guide_length_m)
        self.assertEqual(last_round["S_lower_m"], decisive.s_lower_m)
        self.assertEqual(last_round["S_upper_m"], decisive.s_upper_m)
        self.assertEqual(last_round["S_solution_m"], decisive.s_solution_m)
        self.assertIs(last_round["upper_active"], decisive.s_upper_active)
        for round_record in result.search["rounds"]:
            expected_upper = max(
                round_record["S_lower_m"] + 2.0, 12.0,
                3.0 * round_record["distance_m"] + 10.0)
            if round_record["guide_length_m"] is not None:
                expected_upper = max(
                    expected_upper,
                    1.15 * round_record["guide_length_m"])
            self.assertEqual(round_record["S_upper_m"], expected_upper)
            self.assertEqual(
                round_record["upper_active"],
                abs(round_record["S_solution_m"]
                    - round_record["S_upper_m"]) <= 1e-6)


if __name__ == "__main__":
    unittest.main()