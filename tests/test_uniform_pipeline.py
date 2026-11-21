"""FR04 F01--F19: autonomous budgeted search through the FR02 audit.

F01--F12 are the original eight empty-map requests plus the hold, obstacle,
out-of-bounds and budget cases.  F13--F14 prove that an in-tolerance goal,
default free terminal headings and a zero-motion hold are not falsely
rejected.  F15--F19 inject only the final trajectory-rebuild output or an
alternative input contract and require an explicit non-executable rejection:
wrong start/end, plan-ID mismatch, timing-limit mismatch, same-endpoint
different path, and same-edge-count different controls.

All eight empty-map requests start at (10,10,0) with three zero body headings
on the required 30 m x 30 m, 0.05 m grid.  No reference path, coordinate
special case, goal teleport, backend selection or per-case pivot table is
provided.  Nonempty paths come only from FR03 successors.  The default fixed
budget is 10000 expansions / 30 s; F11 deliberately reduces only the expansion
budget to exercise BUDGET_EXHAUSTED.

For every returned plan this suite calls the FR02 audit again, independently
of the pipeline's terminal screen and final rebuilt-path audit, with an audit
request built from the original request and the authoritative edges -- never
from the plan under test.  Full request fingerprints, primitive paths, models,
samples, timings, counters, rejection histograms and that re-audit are saved
below FR04/ (override with FR04_ARTIFACT_DIR and FR04_RESULTS_PATH).

Run from the V0.2 worktree root::

    OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests \
        -p 'test_uniform_pipeline.py' -v
"""
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import unittest
from unittest import mock

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO_ROOT / "catkin_ws" / "src" / "fwis_v3_optimizer" / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import FormationModel, FormationPose, VehicleOffset
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_quality import FEASIBLE
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
from fwis_v3_optimizer.uniform_motion import (
    PRIMITIVE_A_ARC,
    a_straight_edge,
    o_translation_edge,
)
from fwis_v3_optimizer.uniform_pipeline import (
    PIPELINE_REQUEST_REJECTED,
    UniformPlanningRequest,
    expected_audit_request,
    plan_uniform,
)
from fwis_v3_optimizer.uniform_quality import (
    UNIFORM_DYNAMIC_UNCONFIRMED,
    UniformAuditRequest,
    audit_uniform_timed_plan,
)
from fwis_v3_optimizer.uniform_search import (
    SEARCH_AUDIT_REJECTED,
    SEARCH_BUDGET_EXHAUSTED,
    SEARCH_NO_MOTION,
    SEARCH_NO_SOLUTION,
    SEARCH_SUCCESS,
    GraphSearchConfig,
    SuccessorConfig,
)
from fwis_v3_optimizer.uniform_timing import MotionTiming
from fwis_v3_optimizer.uniform_trajectory import UniformTimedPlan

OFFSETS = ((1, 1.0, 0.0), (2, 0.0, 1.0), (3, 0.0, -1.0))
BODY_SHAPES = {vehicle_id: VehicleShape(0.738, 0.500)
               for vehicle_id, _, _ in OFFSETS}
VEHICLE_IDS = tuple(vehicle_id for vehicle_id, _, _ in OFFSETS)
START = FormationPose(10.0, 10.0, 0.0)
START_HEADINGS = {vehicle_id: 0.0 for vehicle_id in VEHICLE_IDS}
MAP_RES = 0.05
MAP_ORIGIN = -15.0
ARTIFACT_DIR = Path(os.environ.get(
    "FR04_ARTIFACT_DIR",
    str(REPO_ROOT / "evaluation" / "formation_refactor_v02" / "FR04")))
RESULTS_PATH = Path(os.environ.get(
    "FR04_RESULTS_PATH", str(ARTIFACT_DIR / "pipeline_results.json")))
RECORDS = {}


def formation_model():
    return FormationModel(tuple(
        VehicleOffset(vehicle_id, x, y) for vehicle_id, x, y in OFFSETS))


def empty_grid():
    return Grid(np.zeros((600, 600), dtype=bool), MAP_RES, MAP_ORIGIN,
                MAP_ORIGIN)


def enclosed_grid():
    """Statically clear start/goal, but every first motion is blocked."""
    points = []
    offsets = ((0.45, 0.0), (-0.45, 0.0), (0.0, 0.35), (0.0, -0.35),
               (0.45, 0.35), (0.45, -0.35), (-0.45, 0.35), (-0.45, -0.35))
    for cx, cy in ((11.0, 10.0), (10.0, 11.0), (10.0, 9.0)):
        points.extend((cx + dx, cy + dy) for dx, dy in offsets)
    occ = np.zeros((600, 600), dtype=bool)
    for x, y in points:
        row = int(math.floor((y - MAP_ORIGIN) / MAP_RES))
        col = int(math.floor((x - MAP_ORIGIN) / MAP_RES))
        occ[row, col] = True
    return Grid(occ, MAP_RES, MAP_ORIGIN, MAP_ORIGIN), points


def make_request(goal, grid=None, search_config=None, goal_headings=None,
                 plan_id=0):
    return UniformPlanningRequest(
        grid=empty_grid() if grid is None else grid,
        model=formation_model(), vehicle_shapes=BODY_SHAPES,
        start_qf=START, start_headings=START_HEADINGS,
        goal_qf=goal, goal_headings=goal_headings, plan_id=plan_id,
        formation_id="fr04_three_vehicle",
        successor_config=SuccessorConfig(),
        search_config=search_config or GraphSearchConfig())


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def straight_plan(request, start, length, plan_id=None, timing_limits=None):
    """Build a self-contained FR01/FR02 plan for fault injection only."""
    edge = a_straight_edge(
        request.model, start, request.start_headings, (length, 0.0),
        {vehicle_id: 1 for vehicle_id in VEHICLE_IDS}, allow_reverse=True)
    return UniformTimedPlan.from_edges(
        (edge,), BODY_SHAPES,
        plan_id=request.plan_id if plan_id is None else plan_id,
        formation_id=request.formation_id,
        timing_limits=(request.successor_config.timing_limits
                       if timing_limits is None else timing_limits),
        model=request.model)


def self_audit(plan, request):
    """Audit a replacement plan under its own self-described request."""
    return audit_uniform_timed_plan(
        UniformAuditRequest.from_plan(plan), plan, plan.sample(), request.grid)


class UniformPipelineTest(unittest.TestCase):
    maxDiff = None

    def test_continuation_reuses_stopped_oblique_wheels_for_reversal_and_hold(self):
        model = formation_model()
        start, goal = FormationPose(0, 0, 0), FormationPose(1, 2.5, 0)
        request = UniformPlanningRequest(
            empty_grid(), model, BODY_SHAPES, start, START_HEADINGS, goal,
            search_config=GraphSearchConfig(direct_goal_translation=True))
        first = plan_uniform(request)
        self.assertTrue(first.executable, first.detail)
        incoming = first.plan.edges[-1]
        continuation = replace(request, start_qf=goal, goal_qf=start, initial_edge=incoming)
        reverse = plan_uniform(continuation)
        self.assertTrue(reverse.executable, reverse.detail)
        rebuilt = UniformTimedPlan.from_model_json(reverse.plan.to_model_json())
        self.assertEqual(rebuilt.model_sha256(), reverse.plan.model_sha256())
        self.assertEqual(tuple(p.kind for p in rebuilt.phases), ("motion",))
        self.assertTrue(all(d == -1 for _, d in rebuilt.edges[0].directions))
        self.assertTrue(all(d == 1 for _, d in rebuilt.initial_edge.directions))
        stopped = first.plan.state_at(first.plan.duration_s)
        restart = rebuilt.state_at(0.0)
        for old, new in zip(stopped.vehicles, restart.vehicles):
            np.testing.assert_allclose(old.wheel_angles, new.wheel_angles, atol=1e-9)
            self.assertEqual(old.wheel_rolling_speeds, (0.0,) * 4)
            self.assertEqual(new.wheel_rolling_speeds, (0.0,) * 4)
        hold = plan_uniform(replace(continuation, goal_qf=goal))
        self.assertTrue(hold.executable, hold.detail)
        self.assertEqual(hold.status, SEARCH_NO_MOTION)
        latest = UniformTimedPlan.from_model_json(hold.plan.to_model_json()).state_at(0.0)
        self.assertEqual(latest.mode, "O")
        for old, new in zip(stopped.vehicles, latest.vehicles):
            np.testing.assert_allclose(old.wheel_angles, new.wheel_angles, atol=1e-9)
            self.assertEqual(old.direction, new.direction)

        wrong_boundary = plan_uniform(replace(continuation, start_qf=FormationPose(2, 2.5, 0)))
        self.assertFalse(wrong_boundary.executable)
        self.assertEqual(wrong_boundary.reason, "INVALID_CONTINUATION")

    @classmethod
    def tearDownClass(cls):
        ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
        (ARTIFACT_DIR / "cases").mkdir(parents=True, exist_ok=True)
        source_root = PACKAGE_SRC / "fwis_v3_optimizer"
        sources = {}
        for path in (source_root / "uniform_search.py",
                     source_root / "uniform_pipeline.py",
                     Path(__file__)):
            sources[str(path.relative_to(REPO_ROOT))] = sha256_file(path)
        payload = {
            "schema": "fr04.pipeline_results.v1",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "default_budget": asdict(GraphSearchConfig()),
            "required_map": {"width_m": 30.0, "height_m": 30.0,
                             "resolution_m": MAP_RES,
                             "origin": [MAP_ORIGIN, MAP_ORIGIN]},
            "source_sha256": sources,
            "case_count": len(RECORDS),
            "cases": list(RECORDS.values()),
        }
        RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
        RESULTS_PATH.write_text(json.dumps(
            payload, indent=2, sort_keys=True, allow_nan=False) + "\n")

    def _run(self, case_id, description, request, map_metadata=None):
        result = plan_uniform(request)
        independent = None
        if result.plan is not None:
            zero_motion = not result.plan.edges
            authoritative_edges = (() if zero_motion else
                                   tuple(result.search.edges))
            boundary = expected_audit_request(
                request, authoritative_edges, zero_motion=zero_motion)
            projection = result.plan.sample()
            evaluation = audit_uniform_timed_plan(
                boundary, result.plan, projection, request.grid)
            independent = evaluation.as_status()
            independent["audit_request_source"] = (
                "original request + authoritative edges")
            independent["edge_sequence_matches"] = (
                tuple(result.plan.edges) == authoritative_edges)
            independent["goal_within_request_tolerance"] = (
                math.hypot(result.plan.goal_qf.x - request.goal_qf.x,
                           result.plan.goal_qf.y - request.goal_qf.y)
                <= request.successor_config.goal_xy_tol_m
                and abs(wrap(result.plan.goal_qf.psi
                             - request.goal_qf.psi))
                <= request.successor_config.goal_psi_tol_rad)
        record = {
            "case_id": case_id,
            "description": description,
            "assertions_completed": False,
            "request_sha256": result.request_sha256,
            "request": result.request_inputs,
            "map_metadata": map_metadata or {"kind": "empty"},
            "budget": asdict(request.search_config),
            "result": result.as_dict(include_search_path=True),
            "model": None if result.plan is None else result.plan.to_model_dict(),
            "samples": (None if result.projection is None else
                        result.projection.as_dict(include_states=True)),
            "independent_reaudit": independent,
        }
        RECORDS[case_id] = record
        case_path = ARTIFACT_DIR / "cases" / (case_id + ".json")
        case_path.parent.mkdir(parents=True, exist_ok=True)
        case_path.write_text(json.dumps(
            record, indent=2, sort_keys=True, allow_nan=False) + "\n")
        print("FR04 {} status={} reason={} search_wall={:.6f}s "
              "pipeline_wall={:.6f}s T_end={} expansions={} candidates={} "
              "terminal_audits={}".format(
                  case_id, result.status, result.reason,
                  -1.0 if result.search is None else result.search.wall_time_s,
                  result.wall_time_s,
                  None if result.plan is None else result.plan.duration_s,
                  -1 if result.search is None else result.search.expansions,
                  -1 if result.search is None else result.search.candidate_count,
                  0 if result.search is None else len(
                      result.search.terminal_audits)), flush=True)
        return result

    def _success(self, case_id, result, expected_status=SEARCH_SUCCESS):
        self.assertEqual(expected_status, result.status)
        self.assertTrue(result.feasible)
        self.assertTrue(result.executable)
        self.assertIsNotNone(result.plan)
        self.assertEqual(FEASIBLE, result.evaluation.status)
        self.assertTrue(result.evaluation.feasible)
        self.assertEqual(
            FEASIBLE, RECORDS[case_id]["independent_reaudit"]["status"])
        self.assertTrue(RECORDS[case_id]["independent_reaudit"]["feasible"])
        self.assertTrue(RECORDS[case_id]["independent_reaudit"][
            "edge_sequence_matches"])
        self.assertTrue(RECORDS[case_id]["independent_reaudit"][
            "goal_within_request_tolerance"])
        self.assertLessEqual(result.search.expansions,
                             RECORDS[case_id]["budget"]["max_expansions"])
        self.assertLess(result.search.wall_time_s,
                        RECORDS[case_id]["budget"]["max_seconds"])
        goal_state = result.search.goal_state
        self.assertIsNotNone(goal_state)
        self.assertLessEqual(math.hypot(
            goal_state.qf.x - result.search.goal_qf.x,
            goal_state.qf.y - result.search.goal_qf.y),
            0.02 + 1e-12)
        self.assertLessEqual(abs(wrap(
            goal_state.qf.psi - result.search.goal_qf.psi)),
            math.radians(1.0) + 1e-12)
        RECORDS[case_id]["assertions_completed"] = True

    def test_f01_exact_hold(self):
        result = self._run("F01_hold", "exact zero-motion hold",
                           make_request(START, plan_id=1))
        self._success("F01_hold", result, SEARCH_NO_MOTION)
        self.assertEqual(0.0, result.plan.duration_s)
        self.assertEqual(0, len(result.plan.edges))
        self.assertEqual(1, len(result.projection.times))
        self.assertEqual(0, result.search.expansions)

    def test_f02_forward(self):
        result = self._run("F02_forward", "forward (13,10,0)",
                           make_request(FormationPose(13, 10, 0), plan_id=2))
        self._success("F02_forward", result)
        self.assertGreater(result.plan.goal_qf.x, result.plan.start_qf.x)

    def test_f03_reverse(self):
        result = self._run("F03_reverse", "reverse (7,10,0)",
                           make_request(FormationPose(7, 10, 0), plan_id=3))
        self._success("F03_reverse", result)
        self.assertLess(result.plan.goal_qf.x, result.plan.start_qf.x)
        self.assertTrue(any(all(direction < 0 for _, direction in edge.directions)
                            for edge in result.search.edges))

    def test_f04_lateral(self):
        result = self._run("F04_lateral", "lateral (10,13,0)",
                           make_request(FormationPose(10, 13, 0), plan_id=4))
        self._success("F04_lateral", result)
        self.assertGreater(result.plan.goal_qf.y, result.plan.start_qf.y)

    def test_free_goal_uses_local_spin_before_straight_approach(self):
        resolution = 0.05
        y, x = np.indices((240, 360))
        center_y = (y + 0.5) * resolution
        occupied = ((x + 0.5) * resolution >= 8.0) & (
            (center_y <= 4.65) | (center_y >= 7.35))
        grid = Grid(occupied, resolution, 0.0, 0.0)
        request = UniformPlanningRequest(
            grid, formation_model(), BODY_SHAPES,
            FormationPose(3, 6, 0), dict.fromkeys(VEHICLE_IDS, math.pi / 2),
            FormationPose(13, 6, math.pi),
            successor_config=SuccessorConfig(goal_formation_heading_free=True),
            search_config=GraphSearchConfig(direct_goal_translation=True))
        result = plan_uniform(request)
        self.assertTrue(result.executable, result.detail)
        self.assertEqual([edge.mode for edge in result.plan.edges], ["S", "A"])
        self.assertEqual(result.plan.goal_qf, FormationPose(13, 6, 0))
        self.assertTrue(self_audit(result.plan, request).feasible)

    def test_free_goal_turns_the_formation_to_fit_a_narrow_corridor(self):
        resolution = 0.05
        y, x = np.indices((240, 360))
        center_y = (y + 0.5) * resolution
        occupied = ((x + 0.5) * resolution >= 8.0) & (
            (center_y <= 5.45) | (center_y >= 7.40))
        grid = Grid(occupied, resolution, 0.0, 0.0)
        request = UniformPlanningRequest(
            grid, formation_model(), BODY_SHAPES,
            FormationPose(3, 6, 0), START_HEADINGS, FormationPose(13, 6, 0),
            successor_config=SuccessorConfig(goal_formation_heading_free=True),
            search_config=GraphSearchConfig(direct_goal_translation=True))
        result = plan_uniform(request)
        self.assertTrue(result.executable, result.detail)
        self.assertGreater(abs(wrap(result.plan.goal_qf.psi)), math.pi / 4)
        self.assertEqual(result.plan.goal_qf.x, 13.0)
        self.assertEqual(result.plan.goal_qf.y, 6.0)
        self.assertTrue(any(edge.primitive == PRIMITIVE_A_ARC for edge in result.plan.edges))
        self.assertTrue(self_audit(result.plan, request).feasible)

    def test_f05_diagonal(self):
        result = self._run("F05_diagonal", "diagonal (13,12,0)",
                           make_request(FormationPose(13, 12, 0), plan_id=5))
        self._success("F05_diagonal", result)

    def _in_place_rotation(self, case_id, psi, plan_id):
        request = make_request(FormationPose(10, 10, psi), plan_id=plan_id)
        result = self._run(case_id, "cargo in-place rotation", request)
        self._success(case_id, result)
        self.assertEqual(result.search.edges[-1].primitive, PRIMITIVE_A_ARC)
        self.assertEqual(result.plan.start_qf.x, result.plan.goal_qf.x)
        self.assertEqual(result.plan.start_qf.y, result.plan.goal_qf.y)
        nominal = UniformTimedPlan.from_edges(
            result.search.edges, request.vehicle_shapes,
            plan_id=request.plan_id, formation_id=request.formation_id,
            timing_limits=request.successor_config.timing_limits)
        unaudited_clock = self_audit(nominal, request)
        self.assertEqual(unaudited_clock.reason, UNIFORM_DYNAMIC_UNCONFIRMED)
        factors = result.evaluation.metrics["audit_retiming_factors"]
        self.assertEqual(set(factors), {len(result.search.edges) - 1})
        for before, after in zip(nominal.phases, result.plan.phases):
            factor = (factors.get(before.edge_index, 1.0)
                      if isinstance(before, MotionTiming) else 1.0)
            self.assertAlmostEqual(after.duration_s, before.duration_s * factor)
        self.assertGreater(result.plan.duration_s, nominal.duration_s)
        self.assertTrue(result.evaluation.metrics["dynamic_continuous_certificate"])
        self.assertTrue(result.search.terminal_audits[0]["accepted"])
        return result

    def test_f06_in_place_positive_90(self):
        self._in_place_rotation("F06_in_place_positive_90", math.pi / 2.0, 6)

    def test_f07_in_place_negative_90(self):
        self._in_place_rotation("F07_in_place_negative_90", -math.pi / 2.0, 7)

    def test_f08_translate_and_rotate(self):
        result = self._run("F08_translate_and_rotate", "goal (13,10,pi/2)",
                           make_request(FormationPose(13, 10, math.pi / 2), plan_id=8))
        self._success("F08_translate_and_rotate", result)
        self.assertTrue(any(edge.primitive == PRIMITIVE_A_ARC
                            for edge in result.search.edges))

    def test_f09_enclosed_obstacle_no_solution(self):
        grid, points = enclosed_grid()
        result = self._run(
            "F09_enclosed_obstacle", "enclosed start has no accepted successor",
            make_request(FormationPose(13, 10, 0), grid=grid, plan_id=9),
            {"kind": "three_surrounding_obstacle_clusters",
             "occupied_points_m": [list(point) for point in points]})
        self.assertEqual(SEARCH_NO_SOLUTION, result.status)
        self.assertFalse(result.feasible)
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertEqual("search", result.first_failure_layer)
        self.assertEqual(0, result.search.accepted_successors)
        self.assertGreater(result.search.rejection_histogram.get(
            "MAP_OBB_COLLISION", 0), 0)
        RECORDS["F09_enclosed_obstacle"]["assertions_completed"] = True

    def test_f10_out_of_bounds_goal_rejected(self):
        result = self._run(
            "F10_out_of_bounds", "goal outside the 30 m map",
            make_request(FormationPose(16, 10, 0), plan_id=10,
                         goal_headings=START_HEADINGS))
        self.assertEqual(PIPELINE_REQUEST_REJECTED, result.status)
        self.assertEqual("GOAL_CENTER_OUT_OF_BOUNDS", result.reason)
        self.assertEqual("request", result.first_failure_layer)
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertIsNone(result.search)
        RECORDS["F10_out_of_bounds"]["assertions_completed"] = True

    def test_f11_deliberate_expansion_budget_exhaustion(self):
        result = self._run(
            "F11_budget_exhausted", "one-expansion deliberate budget stop",
            make_request(FormationPose(13, 10, 0), plan_id=11,
                         search_config=GraphSearchConfig(max_expansions=1)))
        self.assertEqual(SEARCH_BUDGET_EXHAUSTED, result.status)
        self.assertEqual("EXPANSION_BUDGET_EXHAUSTED", result.reason)
        self.assertEqual("budget", result.first_failure_layer)
        self.assertEqual(1, result.search.expansions)
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        RECORDS["F11_budget_exhausted"]["assertions_completed"] = True

    def _inject_final_rebuild(self, case_id, description, request,
                              replacement, captured=None):
        """Replace only the second trajectory rebuild (the final certificate).

        The real search, terminal FR02 screen, FR02 audit and all request
        checks stay in place; only the reconstruction output is fault-injected.
        """
        import fwis_v3_optimizer.uniform_pipeline as pipeline
        real_from_edges = pipeline.UniformTimedPlan.from_edges
        calls = []

        def build(*args, **kwargs):
            calls.append(1)
            if len(calls) == 2:
                plan = replacement(real_from_edges, args, kwargs)
                if captured is not None:
                    captured.append(plan)
                return plan
            return real_from_edges(*args, **kwargs)

        with mock.patch.object(pipeline.UniformTimedPlan, "from_edges",
                               side_effect=build):
            result = self._run(case_id, description, request)
        return result, len(calls)

    def test_f13_in_tolerance_goal_and_free_terminal_headings(self):
        goal = FormationPose(13.01, 10.005, math.radians(0.5))
        request = make_request(
            goal, plan_id=13,
            goal_headings={vehicle_id: math.pi / 2.0
                           for vehicle_id in VEHICLE_IDS})
        result = self._run(
            "F13_tolerant_goal_free_headings",
            "goal 0.0112 m and 0.5 deg from the reached pose; placeholder "
            "terminal headings must stay free", request)
        self._success("F13_tolerant_goal_free_headings", result)
        distance = math.hypot(result.plan.goal_qf.x - goal.x,
                              result.plan.goal_qf.y - goal.y)
        self.assertLessEqual(distance, 0.02)
        self.assertLessEqual(abs(wrap(result.plan.goal_qf.psi - goal.psi)),
                             math.radians(1.0))
        # The request's placeholder goals are not equality requirements.
        for vehicle_id in VEHICLE_IDS:
            self.assertAlmostEqual(
                0.0, dict(result.plan.goal_headings)[vehicle_id], places=12)
        RECORDS["F13_tolerant_goal_free_headings"][
            "request_goal_to_plan_goal_m"] = distance

    def test_f14_in_tolerance_hold_and_free_terminal_headings(self):
        goal = FormationPose(10.005, 10.005, math.radians(0.5))
        request = make_request(
            goal, plan_id=14,
            goal_headings={vehicle_id: math.pi / 2.0
                           for vehicle_id in VEHICLE_IDS})
        result = self._run(
            "F14_tolerant_hold", "zero-motion hold for a goal inside the "
            "frozen 0.02 m / 1 deg tolerance", request)
        self._success("F14_tolerant_hold", result, SEARCH_NO_MOTION)
        self.assertEqual(0.0, result.plan.duration_s)
        self.assertEqual(0, len(result.plan.edges))
        self.assertEqual((10.0, 10.0, 0.0),
                         (result.plan.goal_qf.x, result.plan.goal_qf.y,
                          result.plan.goal_qf.psi))

    def test_f15_wrong_start_and_goal_rebuild_rejected(self):
        request = make_request(FormationPose(13, 10, 0), plan_id=15)
        wrong = straight_plan(request, FormationPose(0, 0, 0), 1.0)
        self.assertEqual(FEASIBLE, self_audit(wrong, request).status)
        result, calls = self._inject_final_rebuild(
            "F15_wrong_rebuild", "final rebuild replaced by a self-feasible "
            "plan from (0,0,0) to (1,0,0)",
            request, lambda real, args, kwargs: wrong)
        self.assertEqual(2, calls)
        self.assertEqual(SEARCH_AUDIT_REJECTED, result.status)
        self.assertFalse(result.feasible)
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertEqual("PLAN_START_MISMATCH", result.reason)
        self.assertEqual("audit", result.first_failure_layer)
        RECORDS["F15_wrong_rebuild"]["assertions_completed"] = True

    def test_f16_request_id_contract_mismatch_rejected(self):
        request = make_request(FormationPose(13, 10, 0), plan_id=16)
        captured = []

        def replacement(real, args, kwargs):
            changed = dict(kwargs)
            changed["plan_id"] = request.plan_id + 100
            return real(*args, **changed)

        result, calls = self._inject_final_rebuild(
            "F16_plan_id_mismatch", "final rebuild carries plan_id=116 while "
            "the request says 16", request, replacement, captured)
        self.assertEqual(2, calls)
        self.assertEqual(SEARCH_AUDIT_REJECTED, result.status)
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertEqual("PLAN_IDENTITY_MISMATCH", result.reason)
        self.assertEqual(116, captured[0].plan_id)
        self.assertEqual(FEASIBLE, self_audit(captured[0], request).status)
        RECORDS["F16_plan_id_mismatch"]["assertions_completed"] = True

    def test_f17_same_endpoints_but_different_path_rejected(self):
        request = make_request(FormationPose(13, 10, 0), plan_id=17)
        single = straight_plan(request, START, 3.0)
        self.assertEqual((10.0, 10.0, 0.0),
                         (single.start_qf.x, single.start_qf.y,
                          single.start_qf.psi))
        self.assertEqual((13.0, 10.0, 0.0),
                         (single.goal_qf.x, single.goal_qf.y,
                          single.goal_qf.psi))
        self.assertEqual(FEASIBLE, self_audit(single, request).status)
        result, calls = self._inject_final_rebuild(
            "F17_same_endpoints_other_path", "final rebuild is one 3 m "
            "straight with the same endpoints instead of the authoritative "
            "three 1 m edges", request,
            lambda real, args, kwargs: single)
        self.assertEqual(2, calls)
        self.assertEqual(SEARCH_AUDIT_REJECTED, result.status)
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertEqual("PLAN_EDGE_SEQUENCE_MISMATCH", result.reason)
        RECORDS["F17_same_endpoints_other_path"][
            "assertions_completed"] = True

    def test_f18_timing_limit_contract_mismatch_rejected(self):
        from fwis_v3_optimizer.uniform_timing import timing_motion_limits
        request = make_request(FormationPose(13, 10, 0), plan_id=18)
        other_limits = FormationTimingLimits(v_max=0.7)
        other_motion = timing_motion_limits(other_limits)
        replacement_edges = tuple(
            a_straight_edge(
                request.model, FormationPose(10.0 + index, 10.0, 0.0),
                START_HEADINGS, (1.0, 0.0),
                {vehicle_id: 1 for vehicle_id in VEHICLE_IDS},
                limits=other_motion, allow_reverse=True)
            for index in range(3))
        self_consistent = UniformTimedPlan.from_edges(
            replacement_edges, BODY_SHAPES, plan_id=request.plan_id,
            formation_id=request.formation_id,
            timing_limits=other_limits, model=request.model)
        self.assertEqual(FEASIBLE, self_audit(self_consistent, request).status)
        captured = []

        def replacement(real, args, kwargs):
            return self_consistent

        result, calls = self._inject_final_rebuild(
            "F18_timing_limit_mismatch", "final rebuild is timed with "
            "v_max=0.7 while the request uses the default 0.8",
            request, replacement, captured)
        self.assertEqual(2, calls)
        self.assertEqual(SEARCH_AUDIT_REJECTED, result.status)
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertEqual("PLAN_IDENTITY_MISMATCH", result.reason)
        self.assertEqual(other_limits, captured[0].limits)
        self.assertEqual(FEASIBLE, self_audit(captured[0], request).status)
        RECORDS["F18_timing_limit_mismatch"]["assertions_completed"] = True

    def test_f19_same_edge_count_same_endpoints_other_controls_rejected(self):
        request = make_request(FormationPose(13, 10, 0), plan_id=19)
        edges = []
        pose = FormationPose(10.0, 10.0, 0.0)
        for dx, dy in ((2.0, 0.0), (0.0, 1.0), (1.0, -1.0)):
            phi = math.atan2(dy, dx)
            edges.append(o_translation_edge(
                request.model, pose, START_HEADINGS, (dx, dy),
                {vehicle_id: 1 for vehicle_id in VEHICLE_IDS},
                {vehicle_id: phi for vehicle_id in VEHICLE_IDS},
                allow_reverse=True))
            pose = FormationPose(pose.x + dx, pose.y + dy, 0.0)
        alternative = UniformTimedPlan.from_edges(
            tuple(edges), BODY_SHAPES, plan_id=request.plan_id,
            formation_id=request.formation_id,
            timing_limits=request.successor_config.timing_limits,
            model=request.model)
        self.assertEqual(3, len(alternative.edges))
        self.assertEqual((13.0, 10.0, 0.0),
                         (alternative.goal_qf.x, alternative.goal_qf.y,
                          alternative.goal_qf.psi))
        self.assertEqual(FEASIBLE, self_audit(alternative, request).status)
        result, calls = self._inject_final_rebuild(
            "F19_same_count_other_controls", "final rebuild has the same "
            "three-edge count and endpoints but O translations instead of "
            "the authoritative A straights", request,
            lambda real, args, kwargs: alternative)
        self.assertEqual(2, calls)
        self.assertEqual(SEARCH_AUDIT_REJECTED, result.status)
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertEqual("PLAN_EDGE_SEQUENCE_MISMATCH", result.reason)
        RECORDS["F19_same_count_other_controls"][
            "assertions_completed"] = True

    def test_f12_terminal_screen_and_final_rebuilt_path_audit(self):
        import fwis_v3_optimizer.uniform_pipeline as pipeline
        real_audit = pipeline.audit_uniform_timed_plan
        with mock.patch.object(pipeline, "audit_uniform_timed_plan",
                               wraps=real_audit) as wrapped:
            result = self._run(
                "F12_final_reaudit", "terminal FR02 plus final rebuilt-path FR02",
                make_request(FormationPose(13, 10, 0), plan_id=12))
        self._success("F12_final_reaudit", result)
        self.assertEqual(2, wrapped.call_count)
        self.assertEqual(1, len(result.search.terminal_audits))
        self.assertTrue(result.search.terminal_audits[0]["accepted"])


if __name__ == "__main__":
    unittest.main()
