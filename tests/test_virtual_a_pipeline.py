"""G4 end-to-end autonomous pipeline and route-guidance tests."""
import json
import math
from pathlib import Path
import unittest
from unittest import mock

import numpy as np

from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a import optimizer as optimizer_module
from fwis_v3_optimizer.formation_continuous_a import pipeline as pipeline_module
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    FORMATION_CORRIDOR_SCOPE_F_ONLY,
    FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
    G4_EXECUTABLE_STATUSES,
    GuideRoute,
    PlannerConfig,
    RouteGuideConfig,
    VirtualAContractError,
    grid_signature,
    request_fingerprint,
    research_planner_config,
)
from fwis_v3_optimizer.formation_continuous_a.route_processing import (
    build_occupied_index,
    segment_is_free,
    simplify_route,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (
    corridor_chain_from_payload,
)
from fwis_v3_optimizer.formation_continuous_a.formation_certificate import (
    verify_formation_certificate,
)
from fwis_v3_optimizer.formation_continuous_a.formation_corridor import (
    formation_template_from_payload,
)
from fwis_v3_optimizer.formation_continuous_a.pipeline import (
    RouteGuideSearchResult,
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


def _micro_grid(occ, resolution=0.1, origin=(0.0, 0.0)):
    return Grid(np.asarray(occ, dtype=bool), float(resolution),
                float(origin[0]), float(origin[1]))


def _clearance_fields(grid, margin, blocked=()):
    obstacle, boundary = pipeline_module._clearance_components(grid)
    clearance = np.minimum(obstacle, boundary)
    clearance[np.asarray(grid.occ, dtype=bool)] = 0.0
    allowed = pipeline_module._allowed_mask(
        grid, obstacle, boundary, margin, 0.15, blocked)
    return allowed, clearance


def _run_astar(grid, start, goal, config, *, margin=0.10, allowed=None,
               clearance=None, deadline=None, max_seconds=5.0):
    if allowed is None or clearance is None:
        allowed, clearance = _clearance_fields(grid, margin)
    if deadline is None:
        import time
        deadline = time.monotonic() + float(max_seconds)
    return pipeline_module._astar_route(
        grid, allowed, clearance, start, goal, config, deadline,
        route_clearance_m=margin,
        occupied_index=build_occupied_index(grid))


def _dijkstra_cost(grid, allowed, clearance, start, goal, config,
                   margin):
    """Zero-heuristic reference cost with the same edges and exact rule."""
    import heapq
    start_cell = pipeline_module._cell_of(grid, start[0], start[1])
    goal_cell = pipeline_module._cell_of(grid, goal[0], goal[1])
    start_centre = pipeline_module._cell_centre(grid, start_cell)
    heap = [(0.0, 0, start_cell, None)]
    best = {start_cell: 0.0}
    tie = 0
    height, width = allowed.shape
    while heap:
        cost, _, cell, predecessor = heapq.heappop(heap)
        if cost > best.get(cell, math.inf):
            continue
        if cell == goal_cell:
            return cost
        iy, ix = cell
        centre = pipeline_module._cell_centre(grid, cell)
        for dy, dx, step in pipeline_module.ROUTE_NEIGHBOURS:
            jy, jx = iy + dy, ix + dx
            if jy < 0 or jx < 0 or jy >= height or jx >= width:
                continue
            if not allowed[jy, jx]:
                continue
            if dx and dy and (
                    not allowed[iy, jx] or not allowed[jy, ix]):
                continue
            neighbour = (jy, jx)
            neighbour_centre = pipeline_module._cell_centre(
                grid, neighbour)
            if not segment_is_free(
                    grid, centre, neighbour_centre, margin):
                continue
            candidate = cost + pipeline_module._route_edge_cost(
                grid, step, clearance[neighbour], 0.0, config)
            if candidate < best.get(neighbour, math.inf):
                best[neighbour] = candidate
                tie += 1
                heapq.heappush(heap, (candidate, tie, neighbour, cell))
    return None


def _astar_constant_wall(resolution):
    occupancy = np.zeros((15, 15), dtype=bool)
    occupancy[5, 2:10] = True
    occupancy[5:11, 9] = True
    return _micro_grid(occupancy, resolution)


def _channel_grid():
    """Two 1.5 m channels around one slender wall, 6 m x 6 m."""
    occupancy = np.zeros((60, 60), dtype=bool)
    occupancy[15:45, 29:31] = True
    return _micro_grid(occupancy, 0.1)


def _guide(grid, points, source="unit"):
    return GuideRoute(points=tuple(points), source=source,
                      grid_sha256=grid_signature(grid))


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

    def test_corridor_round_records_bounded_diagnostic_assessments(self):
        grid = empty_grid()
        request = make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(10.0, 0.0, 0.0))
        planner = PlannerConfig(guidance_mode="corridor", max_initials=1)
        result = plan_virtual_a(map_request(request, grid, planner=planner))
        self.assertEqual(result.status, "SUCCESS", result.as_dict())
        corridor = result.search["corridor"]
        assessments = corridor["route_assessments"]
        self.assertLessEqual(len(assessments), 3)
        self.assertEqual(len(assessments), result.search["route_count"])
        for assessment in assessments:
            self.assertEqual(
                assessment["schema"], "virtual_a_route_assessment_v1")
            self.assertEqual(assessment["grid_sha256"],
                             result.grid_sha256)
            self.assertEqual(assessment["request_sha256"],
                             result.request_sha256)
            self.assertEqual(assessment["planning_context_sha256"],
                             result.planning_context_sha256)
            self.assertLessEqual(assessment["station_count"], 32)
            self.assertEqual(assessment["candidate_certification"],
                             "DIAGNOSTIC_ONLY")
            self.assertFalse(assessment["continuously_certified"])
            self.assertFalse(
                assessment["formation_obb_hard_constraints"])
        attempt_order = corridor["route_attempt_order"]
        self.assertEqual(sorted(attempt_order),
                         list(range(result.search["route_count"])))
        self.assertEqual(len(corridor["rounds"]), 1)
        round_state = corridor["rounds"][0]
        self.assertEqual(attempt_order[0], round_state["route_index"])
        assessment = round_state["formation_assessment"]
        self.assertEqual(assessment["chain_digest"],
                         round_state["chain_digest"])
        self.assertEqual(assessment["route_index"],
                         round_state["route_index"])
        self.assertEqual(assessment["round_index"],
                         round_state["round_index"])
        self.assertEqual(assessment["legacy_f_allocation"],
                         round_state["allocation"])
        self.assertEqual(len(assessment["candidate_id"]), 64)
        self.assertLessEqual(assessment["station_count"], 64)
        self.assertEqual(assessment["map_sample_status"],
                         "CLEAR_AT_SAMPLES")
        self.assertEqual(assessment["coverage_sample_status"],
                         "SINGLE_REGION")
        self.assertEqual(assessment["kinematic_hint_status"],
                         "CURVATURE_HINT_UNCONFIRMED")
        self.assertEqual(assessment["guide_hint_status"],
                         "VALID_HINT")
        self.assertFalse(assessment["continuously_certified"])
        self.assertFalse(assessment["formation_obb_hard_constraints"])
        self.assertGreaterEqual(result.timings["formation_assessment_s"],
                                0.0)
        selection = round_state["guide_selection"]
        candidates = round_state["guide_candidates"]
        self.assertLessEqual(len(candidates), 5)
        self.assertEqual(selection["candidate_count"], len(candidates))
        self.assertEqual(
            sum(1 for record in candidates if record["selected"]), 1)
        selected = candidates[selection["selected_candidate_index"]]
        self.assertTrue(selected["selected"])
        self.assertEqual(selected["guide_source"],
                         round_state["guide"]["source"])
        self.assertEqual(selected["allocation_count"],
                         len(round_state["allocation"]))
        self.assertEqual(selected["assessment"]["candidate_id"],
                         assessment["candidate_id"])
        self.assertIn(selection["selection_rule"], (
            "SINGLE_BASE_GUIDE", "DIAGNOSTIC_LEXICOGRAPHIC_BEST",
            "ASSESSMENT_UNAVAILABLE_BASE_FALLBACK"))
        self.assertEqual(selection["certification_claim"], "NONE")
        self.assertEqual(selection["candidate_certification"],
                         "DIAGNOSTIC_ONLY")
        self.assertFalse(selection["continuously_certified"])
        self.assertFalse(selection["formation_obb_hard_constraints"])
        for record in candidates:
            self.assertEqual(record["certification_claim"], "NONE")
            self.assertEqual(record["candidate_certification"],
                             "DIAGNOSTIC_ONLY")
            self.assertFalse(record["continuously_certified"])
            self.assertFalse(record["formation_obb_hard_constraints"])

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

    def test_union_scope_certifies_the_f_obb_path_end_to_end(self):
        """TASK03 Step 6: the explicit union scope is the only accepting gate.

        The same empty-grid straight motion is solved through the real
        pipeline with ``three_vehicle_union_v1``: the result must carry the
        sealed template identity, the OBB constraint rows and a VERIFIED
        certificate that re-verifies against the published path.
        """
        grid = empty_grid()
        request = make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(6.0, 0.0, 0.0))
        planner = research_planner_config(
            guidance_mode="corridor", corridor_enforcement="hard",
            formation_corridor_scope=
                FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
            max_initials=3, max_seconds=20.0)
        result = plan_virtual_a(map_request(request, grid, planner=planner))
        self.assertEqual(result.status, "SUCCESS", result.as_dict())
        self.assertTrue(result.executable)
        self.assertEqual(result.formation_corridor_scope,
                         FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1)
        audit = result.formation_corridor_audit
        self.assertEqual(audit["scope"],
                         FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1)
        self.assertEqual(audit["status"], "VERIFIED")
        self.assertEqual(audit["certification_claim"],
                         "CONTINUOUS_F_OBB_CERTIFICATE")
        self.assertEqual(audit["template_id"],
                         result.formation_template_id)
        self.assertGreater(result.formation_obb_constraint_count, 0)
        self.assertTrue(audit["certificate_verification"]["passed"])
        certificate = audit["certificate"]
        self.assertEqual(certificate["status"], "VERIFIED")
        self.assertEqual(certificate["certificate_sha256"],
                         result.formation_certificate["certificate_sha256"])
        self.assertEqual(
            result.metrics["formation_certificate_verified"], True)
        self.assertEqual(
            result.metrics["corridor_containment_certified"], True)
        # The certificate identity is rebuilt from the serialized result
        # evidence, not from process memory.
        chain = corridor_chain_from_payload(
            result.search["corridor"]["rounds"][-1]["chain"])
        template = formation_template_from_payload(audit["template"])
        verification = verify_formation_certificate(
            certificate, result.path, result.plan._request, chain, template,
            planning_context_sha256=result.planning_context_sha256)
        self.assertTrue(verification["passed"], verification)
        payload = result.as_dict()
        self.assertEqual(payload["formation_corridor_scope"],
                         FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1)
        self.assertEqual(payload["formation_template_id"],
                         certificate["template_id"])
        self.assertEqual(
            payload["formation_obb_constraint_count"],
            result.formation_obb_constraint_count)

    def test_f_only_scope_keeps_the_historical_result_key_set(self):
        """TASK03 Step 6: default scope must not change serialized history."""
        grid = empty_grid()
        request = make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(6.0, 0.0, 0.0))
        planner = research_planner_config(
            guidance_mode="corridor", corridor_enforcement="hard",
            formation_corridor_scope=FORMATION_CORRIDOR_SCOPE_F_ONLY,
            max_initials=3, max_seconds=20.0)
        result = plan_virtual_a(map_request(request, grid, planner=planner))
        self.assertEqual(result.status, "SUCCESS", result.as_dict())
        self.assertEqual(result.formation_corridor_scope,
                         FORMATION_CORRIDOR_SCOPE_F_ONLY)
        payload = result.as_dict()
        for key in ("formation_corridor_scope", "formation_corridor_audit",
                    "formation_template_id",
                    "formation_obb_constraint_count",
                    "formation_certificate"):
            self.assertNotIn(key, payload)
        self.assertNotIn("formation_corridor_scope", result.search)
        self.assertNotIn(
            "formation_templates",
            result.search["corridor"]["rounds"][-1])

    def test_f_only_historical_model_and_context_hashes_are_unchanged(self):
        """TASK03 rework Step B: the frozen f_only identity still rebuilds.

        The TASK03 baseline froze the canonical request payload, the planning
        context and the observed digests of the MAP05 straight ``f_only``
        run.  Rebuilding those objects with the reworked source must
        reproduce them exactly: the union-scope additions may not leak into
        the default ``f_only`` serialization or its model/context identity.
        """
        import shutil
        import tempfile

        from fwis_v3_optimizer.core import Grid
        from fwis_v3_optimizer.formation_continuous_a import contracts as contracts_module
        from fwis_v3_optimizer.formation_continuous_a.collision import load_map

        baseline = (Path(__file__).resolve().parents[1]
                    / "work_flow/TASK03_evidence/20261119T133018Z/baseline")
        payload = json.loads(
            (baseline / "MAP05_straight_request_reconstructed.json").read_text(
                encoding="utf-8"))
        frozen_context = json.loads(
            (baseline / "MAP05_straight_planning_context.json").read_text(
                encoding="utf-8"))
        identity = json.loads(
            (baseline / "MAP05_straight_identity.json").read_text(
                encoding="utf-8"))
        observed = identity["observed"]

        request = contracts_module.request_from_payload(payload)
        self.assertEqual(contracts_module.request_payload(request), payload)
        self.assertEqual(contracts_module.request_fingerprint(request),
                         observed["request_sha256"])
        self.assertEqual(
            contracts_module.planning_context_sha256(frozen_context),
            observed["planning_context_sha256"])

        temp = tempfile.mkdtemp(prefix="task03-f-only-identity.")
        try:
            shutil.copy(baseline / "frozen_inputs/MAP05_straight_map.yaml",
                        temp + "/map.yaml")
            shutil.copy(baseline / "frozen_inputs/MAP05_straight_map.pgm",
                        temp + "/map.pgm")
            grid_from_pgm, _metadata = load_map(temp + "/map.yaml")
        finally:
            shutil.rmtree(temp)
        grid = Grid(
            grid_from_pgm.occ,
            np.float32(frozen_context["grid"]["resolution_m"]),
            grid_from_pgm.ox, grid_from_pgm.oy)
        self.assertEqual(grid_signature(grid), observed["grid_sha256"])

        planner = research_planner_config(
            guidance_mode="corridor", corridor_enforcement="hard")
        self.assertEqual(planner.formation_corridor_scope,
                         FORMATION_CORRIDOR_SCOPE_F_ONLY)
        context = contracts_module.planning_context(
            request, grid, planner, contracts_module.TimingConfig(),
            contracts_module.CollisionConfig())
        self.assertEqual(context, frozen_context)
        self.assertEqual(
            contracts_module.planning_context_sha256(context),
            observed["planning_context_sha256"])


class RouteAndGuideSelectionTests(unittest.TestCase):
    """Step D: bounded route ordering and Guide candidate selection."""

    @staticmethod
    def _wall(grid, x0, x1, y0, y1):
        ix0 = int(round((x0 - grid.ox) / grid.res))
        ix1 = int(round((x1 - grid.ox) / grid.res)) + 1
        iy0 = int(round((y0 - grid.oy) / grid.res))
        iy1 = int(round((y1 - grid.oy) / grid.res))
        grid.occ[iy0:iy1, ix0:ix1] = True

    def _two_gate_routes(self):
        # Two free centerlines through one wall line at x~5.  The upper gate
        # is clear for the rigid three-vehicle footprint; the lower gate is
        # not.  Both original A* indices and the requested tuple stay fixed.
        grid = empty_grid()
        self._wall(grid, 4.9, 5.05, -0.6, 0.0)
        self._wall(grid, 4.9, 5.05, 1.1, 1.5)
        request = make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(10.0, 0.0, 0.0))
        digest = grid_signature(grid)
        below = GuideRoute(
            points=((0.0, 0.0), (4.0, -1.9), (6.0, -1.9), (10.0, 0.0)),
            source="route_below", grid_sha256=digest)
        above = GuideRoute(
            points=((0.0, 0.0), (4.0, 0.5), (6.0, 0.5), (10.0, 0.0)),
            source="route_above", grid_sha256=digest)
        planner = PlannerConfig(guidance_mode="corridor", max_initials=1)
        return grid, request, planner, (below, above)

    @staticmethod
    def _failure_result(request, config):
        return optimizer_module._result(
            "NO_ACCEPTED_CANDIDATE", request=request, config=config,
            request_sha256=request_fingerprint(request),
            reason="ALL_ATTEMPTS_NUMERICAL_FAILURE", detail="test spy")

    def test_route_attempt_order_and_guide_selection_reach_the_solver(self):
        grid, request, planner, routes = self._two_gate_routes()
        calls = []

        def solver_spy(solver_request, config, **kwargs):
            calls.append({
                "guide": kwargs.get("route_guides")[0]
                if kwargs.get("route_guides") else None,
                "chain": kwargs.get("corridor_chain"),
                "allocation": kwargs.get("corridor_allocation"),
                "stop_after_route_audit": kwargs.get(
                    "stop_after_route_audit"),
                "config": config,
            })
            return self._failure_result(solver_request, config)

        route_search = RouteGuideSearchResult(
            routes=tuple(routes), attempts=(), stop_reason="MAX_ROUTES",
            route_clearance_m=0.10, direct_los={})
        with mock.patch.object(
                pipeline_module, "make_route_guides",
                return_value=route_search), \
                mock.patch.object(
                    pipeline_module, "solve_open_path_guided",
                    side_effect=solver_spy):
            result = plan_virtual_a(
                map_request(request, grid, planner=planner))
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE",
                         result.as_dict())
        self.assertEqual(result.reason, "ALL_ATTEMPTS_NUMERICAL_FAILURE")
        # The original route tuple and topic IDs are unchanged; only the
        # attempt order is diagnostic.
        self.assertEqual([item["source"] for item in
                          result.search["routes"]],
                         ["route_below", "route_above"])
        corridor = result.search["corridor"]
        self.assertEqual(corridor["route_attempt_order"], [1, 0])
        self.assertEqual([round_state["route_index"]
                          for round_state in corridor["rounds"]], [1, 0])
        assessments = corridor["route_assessments"]
        self.assertEqual([item["route_index"] for item in assessments],
                         [0, 1])
        scores = [
            pipeline_module._route_assessment_score(
                assessments[index], routes[index].length_m, index)
            for index in range(len(routes))]
        self.assertLess(scores[1], scores[0])
        self.assertEqual(len(calls), 2)
        for call, round_state in zip(calls, corridor["rounds"]):
            self.assertIsNotNone(call["guide"])
            self.assertIsNotNone(call["chain"])
            self.assertTrue(call["stop_after_route_audit"])
            self.assertEqual(
                [list(point) for point in call["guide"].points],
                round_state["guide"]["points"])
            self.assertEqual(call["chain"].digest,
                             round_state["chain_digest"])
            self.assertEqual(
                [list(entry) for entry in call["allocation"]],
                round_state["allocation"])
            # The solver input, the selected candidate and the assessment
            # passed to the round are one and the same source.
            selection = round_state["guide_selection"]
            selected = round_state["guide_candidates"][
                selection["selected_candidate_index"]]
            self.assertTrue(selected["selected"])
            assessment = round_state["formation_assessment"]
            self.assertEqual(selected["assessment"]["candidate_id"],
                             assessment["candidate_id"])
            self.assertEqual(assessment["selected_guide_source"],
                             selected["guide_source"])
            self.assertEqual(assessment["selected_guide_source"],
                             str(call["guide"].source))
            self.assertEqual(assessment["chain_digest"],
                             round_state["chain_digest"])
            self.assertEqual(assessment["legacy_f_allocation"],
                             round_state["allocation"])
        # Round 0 attempted original route 1 (upper gate).  The baseline QP
        # assessment reports a Portal-1 risk, so the two canonical guides
        # (base QP + Chebyshev centre) are joined by three one-Portal local
        # variants; the real lexicographic assessment selects the local
        # variant with the fewest sampled collisions.
        upper = corridor["rounds"][0]
        upper_candidates = upper["guide_candidates"]
        self.assertEqual(len(upper_candidates), 5)
        # Every candidate still samples collisions and coverage gaps; none is
        # treated as clear, and the original solver is invoked regardless.
        for candidate in upper_candidates:
            self.assertGreater(
                candidate["selection_counts"]["collision_count"], 0)
            self.assertGreater(candidate["selection_counts"]["gap_count"], 0)
        self.assertEqual(upper_candidates[0]["guide_source"],
                         "corridor_qp_polyline")
        self.assertIsNone(upper_candidates[0]["candidate_origin"])
        self.assertEqual(upper_candidates[1]["candidate_origin"],
                         "CHEBYSHEV_CENTRE_GUIDE")
        for candidate in upper_candidates[2:]:
            self.assertEqual(candidate["candidate_origin"],
                             "ONE_PORTAL_VARIANT_0")
            self.assertEqual(candidate["target_portal_index"], 1)
            self.assertEqual(candidate["target_window_index"], 0)
        upper_selected_index = upper["guide_selection"][
            "selected_candidate_index"]
        self.assertEqual(upper_selected_index, 3)
        self.assertTrue(upper_candidates[upper_selected_index]["selected"])
        self.assertFalse(upper_candidates[0]["selected"])
        # The selection is driven by real FormationCollisionChecker samples:
        # every candidate collides, and the selected local variant samples
        # strictly fewer collisions than the QP base.
        self.assertGreater(
            upper_candidates[0]["selection_counts"]["collision_count"],
            upper_candidates[upper_selected_index]["selection_counts"][
                "collision_count"])
        self.assertEqual(
            upper_candidates[upper_selected_index]["selection_counts"][
                "collision_count"],
            min(candidate["selection_counts"]["collision_count"]
                for candidate in upper_candidates))
        generation = upper["guide_selection"]["candidate_generation"]
        self.assertEqual(generation["selection"], "BASELINE_RISK")
        self.assertEqual(generation["target_portal_index"], 1)
        self.assertEqual(generation["target_window_index"], 0)
        self.assertEqual(generation["target_reason"], "MAP_COLLISION")
        self.assertEqual(generation["local_candidates_kept"], 3)
        # The selected Guide changes only the target Portal point; its start
        # and goal are the fixed request endpoints.
        selected_local = generation["local_candidates"][
            upper_selected_index - 2]
        selected_points = upper["guide"]["points"]
        self.assertEqual(selected_points[0], [0.0, 0.0])
        self.assertEqual(selected_points[-1], [10.0, 0.0])
        self.assertAlmostEqual(selected_points[1][0],
                               selected_local["point"][0], places=12)
        self.assertAlmostEqual(selected_points[1][1],
                               selected_local["point"][1], places=12)
        # Round 1 attempted original route 0 (lower gate); the base QP guide
        # keeps the best real sampled assessment, so candidate 0 wins.
        lower = corridor["rounds"][1]
        lower_candidates = lower["guide_candidates"]
        self.assertEqual(lower["guide_selection"]["selected_candidate_index"],
                         0)
        self.assertTrue(lower_candidates[0]["selected"])
        self.assertEqual(
            lower_candidates[0]["selection_counts"]["collision_count"],
            min(candidate["selection_counts"]["collision_count"]
                for candidate in lower_candidates))
        for call in calls:
            # The original solver path, caps and route audit stay in force
            # even when every generated candidate samples collisions or gaps.
            self.assertEqual(call["config"], planner)
            self.assertEqual(call["config"].max_initials,
                             planner.max_initials)
            self.assertEqual(call["config"].max_iterations,
                             planner.max_iterations)
            self.assertEqual(call["config"].max_seconds,
                             planner.max_seconds)
            self.assertEqual(call["config"].guidance_mode, "corridor")
        for round_state in corridor["rounds"]:
            selection = round_state["guide_selection"]
            self.assertEqual(selection["selection_rule"],
                             "DIAGNOSTIC_LEXICOGRAPHIC_BEST")
            self.assertEqual(selection["certification_claim"], "NONE")
            self.assertFalse(selection["continuously_certified"])
            self.assertFalse(selection["formation_obb_hard_constraints"])
            self.assertEqual(
                round_state["formation_assessment"]["candidate_certification"],
                "DIAGNOSTIC_ONLY")

    def test_unavailable_diagnostics_keep_the_base_guide_and_solver(self):
        grid = empty_grid()
        request = make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(10.0, 0.0, 0.0))
        planner = PlannerConfig(guidance_mode="corridor", max_initials=1)
        with mock.patch.object(
                pipeline_module, "FormationCollisionChecker",
                side_effect=RuntimeError("checker unavailable")):
            result = plan_virtual_a(
                map_request(request, grid, planner=planner))
        self.assertEqual(result.status, "SUCCESS", result.as_dict())
        self.assertTrue(result.executable)
        corridor = result.search["corridor"]
        self.assertTrue(
            corridor["assessment_error"].startswith("CHECKER_UNAVAILABLE"))
        for assessment in corridor["route_assessments"]:
            self.assertEqual(assessment["map_sample_status"], "UNKNOWN")
        round_state = corridor["rounds"][0]
        candidates = round_state["guide_candidates"]
        self.assertEqual(len(candidates), 1)
        self.assertEqual(candidates[0]["status"],
                         "ASSESSMENT_UNAVAILABLE")
        self.assertTrue(candidates[0]["selected"])
        self.assertFalse(candidates[0]["assessment"]["available"])
        self.assertEqual(
            round_state["guide_selection"]["selection_rule"],
            "SINGLE_BASE_GUIDE")
        self.assertEqual(
            round_state["formation_assessment"]["map_sample_status"],
            "UNKNOWN")
        self.assertIn(
            "CHECKER_UNAVAILABLE",
            round_state["formation_assessment"]["incomplete_reasons"])
        self.assertFalse(
            round_state["formation_assessment"]["continuously_certified"])

    def test_incompatible_variant_is_rejected_with_hard_corridor_kept(self):
        grid = empty_grid()
        request = make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(10.0, 0.0, 0.0))
        planner = PlannerConfig(guidance_mode="corridor", max_initials=1)

        def candidates_with_bad_variant(chain, start, goal,
                                         base_guide=None, **_kwargs):
            base = (base_guide if isinstance(base_guide, GuideRoute)
                    else pipeline_module.make_corridor_guide(
                        chain, start, goal))
            bad = GuideRoute(
                points=((0.0, 0.0), (5.0, 5.0), (10.0, 0.0)),
                source="corrupted_variant", grid_sha256=base.grid_sha256)
            return (base, bad), {"selection": "TEST_OVERRIDE"}

        calls = []

        def solver_spy(solver_request, config, **kwargs):
            calls.append({
                "guide": kwargs.get("route_guides")[0],
                "chain": kwargs.get("corridor_chain"),
                "allocation": kwargs.get("corridor_allocation"),
                "stop_after_route_audit": kwargs.get(
                    "stop_after_route_audit"),
            })
            return geometry_result_for(
                straight_path(10.0), request=solver_request, planner=config)

        with mock.patch.object(
                pipeline_module, "plan_guide_candidates",
                side_effect=candidates_with_bad_variant), \
                mock.patch.object(
                    pipeline_module, "solve_open_path_guided",
                    side_effect=solver_spy):
            result = plan_virtual_a(
                map_request(request, grid, planner=planner))
        self.assertEqual(result.status, "SUCCESS", result.as_dict())
        self.assertTrue(result.executable)
        round_state = result.search["corridor"]["rounds"][0]
        candidates = round_state["guide_candidates"]
        self.assertEqual(len(candidates), 2)
        self.assertEqual(candidates[0]["guide_source"],
                         "corridor_endpoint_polyline")
        self.assertEqual(candidates[1]["status"],
                         "ALLOCATION_INCOMPATIBLE")
        self.assertEqual(candidates[1]["reason"],
                         "FORMATION_GUIDE_NOT_SOLVER_COMPATIBLE")
        self.assertFalse(candidates[1]["solver_compatible"])
        self.assertFalse(candidates[1]["selected"])
        self.assertTrue(candidates[0]["selected"])
        selection = round_state["guide_selection"]
        self.assertEqual(selection["solver_compatible_count"], 1)
        self.assertEqual(selection["selected_candidate_index"], 0)
        self.assertTrue(selection["base_guide_solver_compatible"])
        self.assertEqual(selection["certification_claim"], "NONE")
        self.assertEqual(len(calls), 1)
        call = calls[0]
        self.assertTrue(call["stop_after_route_audit"])
        self.assertIsNotNone(call["chain"])
        self.assertEqual(
            [list(point) for point in call["guide"].points],
            round_state["guide"]["points"])
        self.assertEqual(
            [list(entry) for entry in call["allocation"]],
            round_state["allocation"])
        self.assertEqual(call["chain"].digest,
                         round_state["chain_digest"])
        assessment = round_state["formation_assessment"]
        self.assertEqual(
            assessment["candidate_id"],
            candidates[0]["assessment"]["candidate_id"])
        self.assertEqual(assessment["selected_guide_source"],
                         candidates[0]["guide_source"])
        self.assertEqual(assessment["legacy_f_allocation"],
                         round_state["allocation"])
        self.assertEqual(
            assessment["map_sample_status"],
            "CLEAR_AT_SAMPLES")


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


class AAStarContractTests(unittest.TestCase):
    """Targeted A remediation contracts: metric, clearance, status, dedup."""

    def test_metric_g_and_octile_h_agree_with_dijkstra_at_three_resolutions(
            self):
        for resolution in (0.05, 0.10, 0.20):
            grid = _astar_constant_wall(resolution)
            margin = 0.10
            allowed, clearance = _clearance_fields(grid, margin)
            start_cell = (2, 2)
            goal_cell = (12, 12)
            start = pipeline_module._cell_centre(grid, start_cell)
            goal = pipeline_module._cell_centre(grid, goal_cell)
            config = RouteGuideConfig(
                clearance_weight=0.0, clearance_cap_m=1.0,
                max_expansions=100000, max_seconds=5.0,
                route_clearance_m=margin)
            result = _run_astar(
                grid, start, goal, config, margin=margin,
                allowed=allowed, clearance=clearance)
            self.assertEqual(result.status, "FOUND", result)
            reference = _dijkstra_cost(
                grid, allowed, clearance, start, goal, config, margin)
            self.assertIsNotNone(reference)
            self.assertAlmostEqual(
                result.path_length_m, reference, places=9,
                msg="resolution {}".format(resolution))
            travelled = 0.0
            for (iy, ix), (jy, jx) in zip(
                    result.cells, result.cells[1:]):
                step = math.sqrt(2.0) if (
                    iy != jy and ix != jx) else 1.0
                travelled += pipeline_module._route_edge_cost(
                    grid, step, clearance[jy, jx], 0.0, config)
            self.assertAlmostEqual(
                travelled, result.path_length_m, places=9)
            self.assertEqual(
                pipeline_module._octile_heuristic_m(
                    grid, goal_cell, goal), 0.0)
            expected_start_h = resolution * 10.0 * math.sqrt(2.0)
            self.assertAlmostEqual(
                pipeline_module._octile_heuristic_m(
                    grid, start_cell, goal), expected_start_h, places=9)
            height, width = allowed.shape
            for iy in range(height):
                for ix in range(width):
                    if not allowed[iy, ix]:
                        continue
                    cell = (iy, ix)
                    h_value = pipeline_module._octile_heuristic_m(
                        grid, cell, goal)
                    centre = pipeline_module._cell_centre(grid, cell)
                    for dy, dx, step in pipeline_module.ROUTE_NEIGHBOURS:
                        jy, jx = iy + dy, ix + dx
                        if jy < 0 or jx < 0 or jy >= height or jx >= width:
                            continue
                        if not allowed[jy, jx]:
                            continue
                        if dx and dy and (
                                not allowed[iy, jx]
                                or not allowed[jy, ix]):
                            continue
                        neighbour = (jy, jx)
                        if not segment_is_free(
                                grid, centre,
                                pipeline_module._cell_centre(
                                    grid, neighbour), margin):
                            continue
                        edge = pipeline_module._route_edge_cost(
                            grid, step, clearance[jy, jx], 0.0, config)
                        slack = edge + \
                            pipeline_module._octile_heuristic_m(
                                grid, neighbour, goal) - h_value
                        self.assertGreaterEqual(
                            slack, -1e-12,
                            "resolution {} edge {}->{} slack {}".format(
                                resolution, cell, neighbour, slack))

    def test_same_physical_room_at_three_resolutions_reports_legal_metric_routes(
            self):
        margin = 0.20
        lengths = []
        point_counts = []
        for resolution in (0.05, 0.10, 0.20):
            cells = int(round(6.0 / resolution))
            xs = (np.arange(cells) + 0.5) * resolution
            ys = (np.arange(cells) + 0.5) * resolution
            occupancy = (
                (xs[None, :] >= 2.5) & (xs[None, :] <= 3.0)
                & (ys[:, None] >= 1.0) & (ys[:, None] <= 5.0))
            grid = _micro_grid(occupancy, resolution)
            config = RouteGuideConfig(
                clearance_weight=1.0, clearance_cap_m=1.0,
                max_routes=2, max_expansions=100000, max_seconds=8.0,
                alternative_penalty=2.0, route_clearance_m=margin)
            search = make_route_guides(
                grid, FormationPose(0.8, 3.0, 0.0),
                FormationPose(5.2, 3.0, 0.0), config)
            self.assertGreaterEqual(
                len(search), 1,
                "resolution {} stop_reason {}".format(
                    resolution, search.stop_reason))
            route = search.routes[0]
            for first, second in zip(route.points, route.points[1:]):
                self.assertTrue(
                    segment_is_free(grid, first, second, margin),
                    "resolution {} raw segment below margin".format(
                        resolution))
            attempt = search.attempts[0]
            self.assertGreaterEqual(
                attempt["min_clearance_m"], margin - 1e-12)
            lengths.append(float(attempt["path_length_m"]))
            point_counts.append(int(attempt["cell_count"]))
        self.assertLess(
            max(lengths) / min(lengths), 1.25,
            "metric route lengths differ too much: {} cells {}".format(
                lengths, point_counts))

    def test_band_penalty_is_positive_and_opens_the_other_channel(self):
        grid = _channel_grid()
        config = RouteGuideConfig(
            clearance_weight=1.0, clearance_cap_m=0.5,
            alternative_penalty=2.0, max_routes=2,
            max_expansions=100000, max_seconds=8.0,
            route_clearance_m=0.10)
        base = pipeline_module._route_edge_cost(
            grid, 1.0, 1.0, 0.0, config)
        reused = pipeline_module._route_edge_cost(
            grid, 1.0, 1.0, 2.0, config)
        self.assertGreater(reused, base)
        self.assertAlmostEqual(
            reused - base, float(grid.res) * 2.0, places=12)
        search = make_route_guides(
            grid, FormationPose(1.0, 3.0, 0.0),
            FormationPose(5.0, 3.0, 0.0), config)
        self.assertGreaterEqual(
            len(search), 2,
            "stop_reason {} attempts {}".format(
                search.stop_reason, search.attempts))
        simplified = [
            simplify_route(
                grid, route, margin_m=0.10).points
            for route in search.routes]
        side_of_middle = []
        for points in simplified:
            middle = points[len(points) // 2]
            side_of_middle.append(
                "below" if middle[1] < 3.0 else "above")
        self.assertEqual(
            set(side_of_middle[:2]), {"below", "above"},
            "both candidates used the same channel: {}".format(
                side_of_middle))

    def test_shortcut_screen_keeps_channels_and_u_turns_but_merges_doglegs(
            self):
        channel = _channel_grid()
        left = [(1.0, 3.0), (2.0, 3.0)]
        right = [(4.0, 3.0), (5.0, 3.0)]
        self.assertFalse(pipeline_module._routes_shortcut_equivalent(
            channel, left, right, 0.10))

        u_grid = _micro_grid(np.zeros((30, 30), dtype=bool), 0.1)
        u_grid.occ[10:12, 10:20] = True
        u_grid.occ[10:20, 10:12] = True
        u_grid.occ[10:20, 18:20] = True
        inside = [(1.5, 1.5), (1.5, 1.8)]
        outside = [(2.5, 1.5), (2.5, 1.8)]
        self.assertFalse(pipeline_module._routes_shortcut_equivalent(
            u_grid, inside, outside, 0.05))

        empty = _micro_grid(np.zeros((20, 20), dtype=bool), 0.1)
        dogleg = [(0.5, 0.5), (1.0, 0.65), (1.5, 0.5)]
        direct = [(0.5, 0.5), (1.5, 0.5)]
        self.assertTrue(pipeline_module._routes_shortcut_equivalent(
            empty, dogleg, direct, 0.0))

    def test_simplify_keeps_the_intermediate_point_when_direct_los_below_margin(
            self):
        grid = _micro_grid(np.zeros((40, 40), dtype=bool), 0.1)
        grid.occ[16:20, 20:24] = True
        start = (1.0, 1.52)
        middle = (2.2, 1.3)
        goal = (3.0, 1.52)
        self.assertTrue(segment_is_free(grid, start, goal, 0.0))
        self.assertFalse(segment_is_free(grid, start, goal, 0.10))
        self.assertTrue(segment_is_free(grid, start, middle, 0.10))
        self.assertTrue(segment_is_free(grid, middle, goal, 0.10))
        route = _guide(grid, (start, middle, goal))
        zero_margin = simplify_route(grid, route, margin_m=0.0)
        self.assertEqual(zero_margin.points, (start, goal))
        kept = simplify_route(grid, route, margin_m=0.10)
        self.assertEqual(kept.points, (start, middle, goal))
        with self.assertRaises(VirtualAContractError) as raised:
            simplify_route(
                grid, _guide(grid, (start, goal)), margin_m=0.10)
        self.assertEqual(raised.exception.reason, "ROUTE_SEGMENT_BLOCKED")

    def test_boundary_corner_unknown_out_of_map_and_blocked_points(self):
        grid = _micro_grid(np.zeros((5, 5), dtype=bool), 1.0)
        grid.occ[2, 2] = True
        self.assertFalse(segment_is_free(
            grid, (2.0, 2.2), (2.0, 2.8), 0.0))  # box edge touch
        self.assertFalse(segment_is_free(
            grid, (1.0, 3.0), (3.0, 1.0), 0.0))  # box corner touch
        self.assertTrue(segment_is_free(
            grid, (1.9, 2.2), (1.9, 2.8), 0.0))
        self.assertFalse(segment_is_free(
            grid, (1.9, 2.2), (1.9, 2.8), 0.10))  # strict > margin
        self.assertFalse(segment_is_free(
            grid, (-0.01, 0.5), (1.0, 0.5), 0.0))
        self.assertFalse(segment_is_free(
            grid, (2.2, 2.2), (2.8, 2.8), 0.0))  # through unknown/occupied

        blocked_grid = _micro_grid(np.zeros((11, 11), dtype=bool), 0.1)
        obstacle = np.full((11, 11), 10.0)
        boundary = np.full((11, 11), 10.0)
        blocked = ((0.55, 0.55),)
        allowed = pipeline_module._allowed_mask(
            blocked_grid, obstacle, boundary, 0.10, 0.15, blocked)
        self.assertFalse(bool(allowed[5, 5]))
        self.assertTrue(bool(allowed[5, 8]))

        corner_cut = _micro_grid(np.zeros((5, 5), dtype=bool), 1.0)
        allowed = np.ones((5, 5), dtype=bool)
        allowed[0, 1] = False
        allowed[1, 0] = False
        clearance = np.full((5, 5), 10.0)
        config = RouteGuideConfig(
            clearance_weight=0.0, max_expansions=1000, max_seconds=2.0,
            route_clearance_m=0.0)
        result = pipeline_module._astar_route(
            corner_cut, allowed, clearance, (0.5, 0.5), (1.5, 1.5),
            config, __import__("time").monotonic() + 2.0,
            route_clearance_m=0.0,
            occupied_index=build_occupied_index(corner_cut))
        self.assertEqual(result.status, "NO_PATH", result)
        self.assertEqual(result.detail, "OPEN_EXHAUSTED")

    def test_failure_statuses_are_distinct_and_carry_diagnostics(self):
        grid = _micro_grid(np.zeros((7, 7), dtype=bool), 1.0)
        allowed, clearance = _clearance_fields(grid, 0.0)
        start = (0.5, 3.5)
        goal = (6.5, 3.5)
        config = RouteGuideConfig(
            clearance_weight=0.0, max_expansions=1000, max_seconds=2.0,
            route_clearance_m=0.0)
        deadline = __import__("time").monotonic() + 2.0

        out_of_map = pipeline_module._astar_route(
            grid, allowed, clearance, (-0.1, 3.5), goal, config, deadline,
            route_clearance_m=0.0)
        self.assertEqual(out_of_map.status, "INVALID_ENDPOINT")
        self.assertEqual(out_of_map.detail, "START_OUT_OF_MAP")

        denied = np.array(allowed, copy=True)
        denied[3, 0] = False
        blocked_start = pipeline_module._astar_route(
            grid, denied, clearance, start, goal, config, deadline,
            route_clearance_m=0.0)
        self.assertEqual(blocked_start.status, "INVALID_ENDPOINT")
        self.assertEqual(
            blocked_start.detail,
            "START_CELL_BLOCKED_BY_CLEARANCE_OR_BLOCKED_POINT")

        connection_grid = _micro_grid(np.zeros((7, 7), dtype=bool), 1.0)
        connection_grid.occ[3, 3] = True
        connection_allowed = np.ones((7, 7), dtype=bool)
        connection_clearance = np.full((7, 7), 10.0)
        blocked_connection = pipeline_module._astar_route(
            connection_grid, connection_allowed, connection_clearance,
            (3.5, 3.5), (0.5, 0.5), config, deadline,
            route_clearance_m=0.0)
        self.assertEqual(blocked_connection.status, "INVALID_ENDPOINT")
        self.assertEqual(
            blocked_connection.detail, "START_CONNECTION_BLOCKED")

        wall_grid = _micro_grid(np.zeros((7, 7), dtype=bool), 1.0)
        wall_grid.occ[:, 3] = True
        wall_allowed, wall_clearance = _clearance_fields(wall_grid, 0.0)
        no_path = pipeline_module._astar_route(
            wall_grid, wall_allowed, wall_clearance,
            (0.5, 3.5), (6.5, 3.5), config, deadline,
            route_clearance_m=0.0)
        self.assertEqual(no_path.status, "NO_PATH")
        self.assertEqual(no_path.detail, "OPEN_EXHAUSTED")

        before = mock.patch.object(
            pipeline_module.time, "monotonic", return_value=10.0)
        with before:
            timeout_before = pipeline_module._astar_route(
                grid, allowed, clearance, start, goal, config, 5.0,
                route_clearance_m=0.0)
        self.assertEqual(timeout_before.status, "TIMEOUT")
        self.assertEqual(
            timeout_before.detail, "DEADLINE_BEFORE_SEARCH")
        self.assertGreaterEqual(timeout_before.search_time_s, 0.0)

        clock = mock.patch.object(
            pipeline_module.time, "monotonic",
            side_effect=[0.0, 0.0, 10.0, 10.0, 10.0])
        with clock:
            timeout_during = pipeline_module._astar_route(
                grid, allowed, clearance, start, goal, config, 5.0,
                route_clearance_m=0.0)
        self.assertEqual(timeout_during.status, "TIMEOUT")
        self.assertEqual(
            timeout_during.detail, "DEADLINE_DURING_SEARCH")

        big_grid = _micro_grid(np.zeros((60, 60), dtype=bool), 0.1)
        big_grid.occ[5:55, 30] = True
        big_allowed, big_clearance = _clearance_fields(big_grid, 0.10)
        limited = RouteGuideConfig(
            clearance_weight=0.0, max_expansions=100, max_seconds=2.0,
            route_clearance_m=0.10)
        expansion = pipeline_module._astar_route(
            big_grid, big_allowed, big_clearance,
            (0.55, 2.95), (5.45, 2.95), limited,
            __import__("time").monotonic() + 2.0,
            route_clearance_m=0.10)
        self.assertEqual(expansion.status, "EXPANSION_LIMIT")
        self.assertGreaterEqual(expansion.expansions, 100)
        for outcome in (out_of_map, blocked_start, blocked_connection,
                        no_path, timeout_before, timeout_during, expansion):
            self.assertIsInstance(outcome.expansions, int)
            self.assertGreaterEqual(outcome.search_time_s, 0.0)


if __name__ == "__main__":
    unittest.main()