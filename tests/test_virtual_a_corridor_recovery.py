"""C3 candidate scheduling and single-recovery tests.

Every test uses the real G4 request/map/audit types.  Solver-level doubles
count calls and only replace solver iterates; they never bypass the
independent geometry/dynamics/collision certificates.  No ROS process is
started by this module.
"""
from dataclasses import FrozenInstanceError, replace
import math
import time
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np

from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a import optimizer as optimizer_module
from fwis_v3_optimizer.formation_continuous_a import pipeline as pipeline_module
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    AttemptRecord,
    ConvexCorridor,
    CorridorChain,
    GuideRoute,
    HeadingBranch,
    PlannerConfig,
    RouteGuideConfig,
    _wrap,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (
    reallocate_adjacent_portal_once,
    validate_corridor_allocation,
)
from fwis_v3_optimizer.formation_continuous_a.optimizer import (
    initial_guess_key,
    make_initial_guesses,
    solve_open_path_guided,
)
from fwis_v3_optimizer.formation_continuous_a.pipeline import (
    RouteGuideSearchResult,
    _guide_segment_tangents,
    _heading_branch_candidates,
    _winding_retry_allowed,
    make_route_guides,
    plan_virtual_a,
)
from fwis_v3_optimizer.formation_continuous_a.route_processing import (
    segment_is_free,
    simplify_route,
)

from virtual_a_g4_support import (
    PLANNER,
    empty_grid,
    geometry_result_for,
    load_synthetic,
    make_request,
    map_request,
    retime,
    straight_path,
)


def box_grid():
    """One solid box between start and goal; both sides are traversable."""
    grid = empty_grid()
    x0 = int(math.floor((3.5 - grid.ox) / grid.res))
    x1 = int(math.ceil((4.5 - grid.ox) / grid.res))
    y0 = int(math.floor((-1.0 - grid.oy) / grid.res))
    y1 = int(math.ceil((1.0 - grid.oy) / grid.res))
    grid.occ[y0:y1, x0:x1] = True
    return grid


def moving_request():
    return make_request(
        FormationPose(0.0, 0.0, 0.0), FormationPose(8.0, 0.0, 0.0))


def fake_solver_failure(_request, _config, _packet, _guess,
                        wall_limit_s=None, **kwargs):
    return {
        "solver": "ipopt",
        "z": None,
        "objective": None,
        "solver_status": "Maximum_Iterations_Exceeded",
        "success": False,
        "iterations": 120,
        "detail": "fake iteration limit",
        "wall_time_s": 0.0,
    }


def corridor_rectangle(x0, x1, y0, y1, seed_start, seed_end):
    return ConvexCorridor(
        seed_start=seed_start,
        seed_end=seed_end,
        A=((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)),
        b=(float(x1), float(-x0), float(y1), float(-y0)),
        vertices=(
            (float(x0), float(y0)), (float(x1), float(y0)),
            (float(x1), float(y1)), (float(x0), float(y1))))


def portal_chain_and_allocation(second_x0=4.0):
    """Two overlapping corridors whose true overlap is x in [max,5]."""
    chain = CorridorChain(
        regions=(
            corridor_rectangle(
                -1.0, 5.0, -2.0, 2.0, (0.0, 0.0), (4.0, 0.0)),
            corridor_rectangle(
                second_x0, 9.0, -2.0, 2.0, (4.0, 0.0), (8.0, 0.0))),
        route_source="test_corridor",
        grid_sha256="a" * 64)
    allocation = ((0.0, 0.5, 0), (0.5, 1.0, 1))
    return chain, allocation


def width_failure(s_m=4.0, corridor_index=0, width_m=9.68e-6,
                  min_width_m=1e-5):
    return {
        "passed": False,
        "reason": "CORRIDOR_AUDIT_WIDTH_LIMIT",
        "detail": "test fixed-sigma width limit",
        "min_width_m": float(min_width_m),
        "first_failure": {
            "reason": "CORRIDOR_AUDIT_WIDTH_LIMIT",
            "s_m": float(s_m),
            "corridor_index": int(corridor_index),
            "width_m": float(width_m),
            "depth": 17,
            "point_residual_m": -4.89e-6,
            "bound_residual_m": 4.2e-8,
            "detail": "test fixed-sigma width limit",
        },
    }


def geometry_with_attempts(request, path, planner, guesses):
    base = geometry_result_for(path, request)
    attempts = tuple(
        AttemptRecord(
            attempt_index=index,
            initial_source=guess.source,
            initial_label=guess.label,
            solver="ipopt",
            solver_status="Maximum_Iterations_Exceeded",
            status="SOLVER_FAILED",
            initial=guess)
        for index, guess in enumerate(guesses))
    return optimizer_module._result(
        "GEOMETRY_FEASIBLE", request=request, path=path, audit=base.audit,
        attempts=attempts, request_sha256=base.request_sha256,
        config=planner)


class HeadingBranchCandidateTests(unittest.TestCase):
    """Step 3 finite unwrapped-heading candidates; no solver is started."""

    def _request(self, start_psi=0.0, goal_psi=0.0):
        return make_request(
            FormationPose(0.0, 0.0, float(start_psi)),
            FormationPose(8.0, 0.0, float(goal_psi)))

    def test_counter_clockwise_loop_prefers_the_positive_branch(self):
        request = self._request()
        points = (
            (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0))
        branches = _heading_branch_candidates(request, points, "loop_ccw")
        self.assertEqual([b.winding_k for b in branches], [1, 0, 2])
        self.assertEqual([b.candidate_rank for b in branches], [0, 1, 2])
        self.assertAlmostEqual(branches[0].base_heading_rad, 0.0, places=12)
        self.assertAlmostEqual(
            branches[0].guide_turn_rad, 2.0 * math.pi, places=12)
        self.assertAlmostEqual(
            branches[0].unwrapped_heading_rad, 2.0 * math.pi, places=12)
        self.assertEqual(branches[0].source, "loop_ccw")
        for branch in branches:
            self.assertAlmostEqual(
                _wrap(branch.unwrapped_heading_rad
                      - branch.base_heading_rad),
                0.0, places=12)

    def test_clockwise_loop_prefers_the_negative_branch(self):
        request = self._request()
        points = (
            (0.0, 0.0), (1.0, 0.0), (1.0, -1.0), (0.0, -1.0), (0.0, 0.0))
        branches = _heading_branch_candidates(request, points, "loop_cw")
        self.assertEqual([b.winding_k for b in branches], [-1, 0, -2])
        self.assertAlmostEqual(
            branches[0].guide_turn_rad, -2.0 * math.pi, places=12)
        self.assertAlmostEqual(
            branches[0].unwrapped_heading_rad, -2.0 * math.pi, places=12)

    def test_duplicate_and_near_zero_segments_are_removed(self):
        points = (
            (0.0, 0.0), (0.0, 0.0), (1e-9, 0.0), (1.0, 0.0),
            (1.0, 0.0), (1.0, 1.0))
        tangents = _guide_segment_tangents(points)
        self.assertEqual(len(tangents), 2)
        self.assertAlmostEqual(tangents[0], 0.0, places=12)
        self.assertAlmostEqual(tangents[1], math.pi / 2.0, places=12)
        request = self._request()
        branches = _heading_branch_candidates(request, points, "duplicates")
        self.assertEqual([b.winding_k for b in branches], [0, -1, 1])
        self.assertAlmostEqual(
            branches[0].guide_turn_rad, 0.0, places=12)

    def test_near_tie_resolves_to_smaller_absolute_k(self):
        # The 600004 geometry: base=+3.124, guide turn=-3.159..., i.e. the
        # theta=base-2*pi case.  k=0 and k=-2 are both 2*pi from the guide
        # turn mathematically; float accumulation must not let the negative
        # full-turn branch jump ahead of the shorter physical heading.
        goal_psi = 3.124
        # Two principal segments accumulate a negative turn equal to
        # base-2*pi: -pi/2 then a wrapped +pi heading.
        request = self._request(0.0, goal_psi)
        points = ((0.0, 0.0), (0.0, -1.0), (-1.0, -1.0))
        branches = _heading_branch_candidates(request, points, "near_tie")
        self.assertEqual([b.winding_k for b in branches], [-1, 0, -2])
        self.assertEqual([b.candidate_rank for b in branches], [0, 1, 2])
        first = abs(branches[0].unwrapped_heading_rad
                    - branches[0].guide_turn_rad)
        second = abs(branches[1].unwrapped_heading_rad
                     - branches[1].guide_turn_rad)
        third = abs(branches[2].unwrapped_heading_rad
                    - branches[2].guide_turn_rad)
        self.assertLess(first, 1e-9)
        self.assertAlmostEqual(second, third, places=9)

    def test_degenerate_guide_offers_only_the_shortest_wrap_branch(self):
        request = self._request(0.3, -0.4)
        points = ((1.0, 2.0), (1.0, 2.0), (1.0 + 1e-9, 2.0))
        branches = _heading_branch_candidates(request, points, "degenerate")
        self.assertEqual(len(branches), 1)
        branch = branches[0]
        self.assertEqual(branch.winding_k, 0)
        self.assertEqual(branch.source, "no_reliable_guide")
        self.assertIsNone(branch.guide_turn_rad)
        self.assertAlmostEqual(
            branch.unwrapped_heading_rad, _wrap(-0.4 - 0.3), places=12)
        self.assertAlmostEqual(
            _wrap(branch.unwrapped_heading_rad + 0.7), 0.0, places=12)

    def test_physical_goal_pose_is_never_rewritten(self):
        request = self._request(0.0, 0.0)
        before = (
            request.start_qf.x, request.start_qf.y, request.start_qf.psi,
            request.goal_qf.x, request.goal_qf.y, request.goal_qf.psi)
        points = (
            (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0))
        branches = _heading_branch_candidates(request, points, "loop_ccw")
        self.assertNotEqual(branches[0].unwrapped_heading_rad, 0.0)
        after = (
            request.start_qf.x, request.start_qf.y, request.start_qf.psi,
            request.goal_qf.x, request.goal_qf.y, request.goal_qf.psi)
        self.assertEqual(before, after)

    def test_heading_branch_is_immutable_and_serialises_six_fields(self):
        branch = HeadingBranch(
            base_heading_rad=0.1,
            winding_k=1,
            unwrapped_heading_rad=0.1 + 2.0 * math.pi,
            guide_turn_rad=0.1 + 2.0 * math.pi,
            source="unit",
            candidate_rank=0)
        with self.assertRaises(FrozenInstanceError):
            branch.winding_k = 2
        self.assertEqual(
            set(branch.as_dict()),
            {"base_heading_rad", "winding_k", "unwrapped_heading_rad",
             "guide_turn_rad", "source", "candidate_rank"})


class CandidateSchedulingTests(unittest.TestCase):
    def test_open_grid_merges_slight_doglegs_into_one_candidate(self):
        grid = empty_grid()
        request = moving_request()
        guides = make_route_guides(
            grid, request.start_qf, request.goal_qf, RouteGuideConfig())
        self.assertEqual([guide.source for guide in guides],
                         ["astar_primary"])
        for guide in guides:
            self.assertEqual(guide.start,
                             (request.start_qf.x, request.start_qf.y))
            self.assertEqual(guide.goal,
                             (request.goal_qf.x, request.goal_qf.y))
            self.assertLessEqual(len(guides), 3)

    def test_single_box_keeps_both_obstacle_separated_sides(self):
        grid = box_grid()
        request = moving_request()
        guides = make_route_guides(
            grid, request.start_qf, request.goal_qf, RouteGuideConfig())
        self.assertGreaterEqual(len(guides), 2)
        self.assertLessEqual(len(guides), 3)
        self.assertNotIn("open_direct_los",
                         [guide.source for guide in guides])
        above = [guide for guide in guides
                 if max(point[1] for point in guide.points) > 0.9]
        below = [guide for guide in guides
                 if min(point[1] for point in guide.points) < -0.9]
        self.assertTrue(above)
        self.assertTrue(below)
        above_shape = simplify_route(grid, above[0]).points
        below_shape = simplify_route(grid, below[0]).points
        # The two sides are separated by the box: at least one cross pair is
        # blocked, so the shortcut-equivalence screen must retain both.
        self.assertTrue(any(
            not segment_is_free(grid, first, second)
            for first in above_shape for second in below_shape))

    def test_initial_slots_are_one_per_route_plus_endpoint_shapes(self):
        grid = box_grid()
        request = moving_request()
        guides = make_route_guides(
            grid, request.start_qf, request.goal_qf, RouteGuideConfig())
        # A repeated guide must not consume another slot.  Three is the
        # contract maximum, so repeat one of the generated guides.
        duplicated = guides[:2] + guides[:1]
        guesses = make_initial_guesses(
            request, PlannerConfig(max_initials=6),
            route_guides=duplicated)
        self.assertLessEqual(len(guesses), 6)
        route_guesses = [
            guess for guess in guesses
            if guess.source == "map_route_guide"]
        endpoint_guesses = [
            guess for guess in guesses
            if guess.source == "automatic_endpoint_shape"]
        self.assertEqual(
            [guess.metadata["route_index"] for guess in route_guesses],
            [0, 1])
        self.assertGreaterEqual(len(endpoint_guesses), 1)
        shapes = {(guess.metadata["length_multiplier"],
                   guess.metadata["shape_sign"])
                  for guess in endpoint_guesses}
        self.assertEqual(len(shapes), len(endpoint_guesses))
        self.assertLessEqual(
            len({multiplier for multiplier, _ in shapes}), 2)
        self.assertTrue(
            {multiplier for multiplier, _ in shapes} <= {1.2, 1.8})
        self.assertEqual(guesses[0].source, "map_route_guide")
        self.assertEqual(guesses[1].source, "map_route_guide")
        self.assertEqual(guesses[2].source, "automatic_endpoint_shape")
        for guess in endpoint_guesses:
            # A true endpoint-only initial carries no route target at all;
            # the route-guided initial is the only route-attached slot.
            self.assertNotIn("route_index", guess.metadata)
            self.assertNotIn("route_source", guess.metadata)
            self.assertNotIn("route_grid_sha256", guess.metadata)

    def test_corridor_endpoint_initials_are_not_pulled_to_route_zero(self):
        grid = empty_grid()
        request = moving_request()
        guides = make_route_guides(
            grid, request.start_qf, request.goal_qf, RouteGuideConfig())
        calls = []

        def recording_failure(_request, _config, _packet, guess,
                              wall_limit_s=None, **kwargs):
            calls.append({
                "source": guess.source,
                "route_index": guess.metadata.get("route_index"),
                "route_guidance": "route_points" in kwargs,
            })
            return fake_solver_failure(None, None, None, None)

        config = PlannerConfig(guidance_mode="corridor", max_initials=3)
        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=recording_failure), mock.patch.object(
                optimizer_module, "_build_corridor_constraint",
                return_value=object()):
            result = solve_open_path_guided(
                request, config, route_guides=guides,
                corridor_chain=object(), corridor_allocation=object())
        self.assertIsNone(result.path)
        self.assertTrue(calls)
        self.assertEqual(calls[0]["source"], "map_route_guide")
        self.assertTrue(calls[0]["route_guidance"])
        for call in calls[1:]:
            self.assertEqual(call["source"], "automatic_endpoint_shape")
            self.assertFalse(call["route_guidance"])
        self.assertEqual(result.initial_count, len(calls))


class RecoverySchedulingTests(unittest.TestCase):
    def test_fake_solver_success_but_audit_rejection_is_never_certified(self):
        grid = box_grid()
        request = moving_request()

        def fake_success(_request, _config, _packet, guess,
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
                side_effect=fake_success):
            result = plan_virtual_a(map_request(request, grid))
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertIsNone(result.path)
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE")
        self.assertEqual(result.reason, "AUDIT_REJECTED")
        self.assertEqual(result.first_failure_layer, "GEOMETRY")
        # Step 3: an endpoint-constraint rejection may spend the single
        # recovery on the next finite heading branch.  The fake success still
        # never becomes an executable candidate.
        self.assertEqual(result.search["recovery_count"], 1)
        recovery = result.search["recoveries"][0]
        self.assertEqual(recovery["action"], "winding_branch")
        candidates = result.search["winding"]["candidates"]
        self.assertGreaterEqual(len(candidates), 2)
        self.assertEqual(
            recovery["from_winding_k"], candidates[0]["winding_k"])
        self.assertEqual(
            recovery["to_winding_k"], candidates[1]["winding_k"])
        rounds = result.search["rounds"]
        self.assertEqual(len(rounds), 2)
        self.assertEqual(rounds[0]["initial_count"], 5)
        self.assertEqual(rounds[1]["winding_candidate_rank"], 1)
        self.assertEqual(rounds[1]["winding_k"], candidates[1]["winding_k"])
        self.assertEqual(len(result.search["initial_keys"]), 6)
        self.assertEqual(result.search["initial_count_total"], 6)
        self.assertTrue(result.attempts)
        self.assertTrue(all(
            attempt.status == "SOLVER_SUCCESS_AUDIT_REJECTED"
            for attempt in result.attempts))
        self.assertEqual(result.search["route_count"], 3)

    def test_simulated_timeout_never_starts_the_slsqp_fallback(self):
        grid = box_grid()
        request = moving_request()

        def fake_timeout(_request, _config, _packet, _guess,
                         wall_limit_s=None, **kwargs):
            return {
                "solver": "ipopt",
                "z": None,
                "objective": None,
                "solver_status": "Maximum_WallTime_Exceeded",
                "success": False,
                "iterations": 1,
                "detail": "forced timeout",
                "wall_time_s": 0.01,
                "soft_deadline_triggered": True,
                "deadline_trigger_stage": "ipopt_max_wall_time",
                "remaining_budget_s": 1e-9,
            }

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_timeout), mock.patch.object(
                optimizer_module, "_solve_with_scipy",
                side_effect=AssertionError("fallback must not start")):
            result = plan_virtual_a(map_request(request, grid))
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertEqual(result.status, "BUDGET_EXHAUSTED")
        self.assertEqual(result.reason, "SOFT_BUDGET_EXHAUSTED")
        self.assertEqual(result.first_failure_layer, "GEOMETRY")
        self.assertEqual(result.search["recovery_count"], 0)
        self.assertEqual([attempt.status for attempt in result.attempts],
                         ["SOLVER_TIMEOUT"])
        self.assertEqual(result.attempts[0].solver, "ipopt")

    def test_numerical_failure_spends_one_endpoint_only_recovery(self):
        grid = empty_grid()
        request = moving_request()
        calls = []
        # Step 3: with several finite heading branches this failure would spend
        # the recovery on the next branch first.  A degenerate guide offers
        # only k=0, so the historical endpoint-only recovery remains reachable.
        single_branch = (HeadingBranch(
            base_heading_rad=0.0,
            winding_k=0,
            unwrapped_heading_rad=0.0,
            guide_turn_rad=0.0,
            source="test_single_branch",
            candidate_rank=0),)

        def fake_failure(_request, _config, _packet, guess,
                         wall_limit_s=None, **kwargs):
            calls.append(initial_guess_key(guess))
            return fake_solver_failure(None, None, None, None)

        with mock.patch.object(
                pipeline_module, "_heading_branch_candidates",
                return_value=single_branch), mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_failure):
            result = plan_virtual_a(map_request(request, grid))
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertIsNone(result.path)
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE")
        self.assertEqual(result.reason, "SOLVER_NOT_CONVERGED")
        self.assertEqual(result.first_failure_layer, "GEOMETRY")
        rounds = result.search["rounds"]
        self.assertEqual([record["initial_mode"] for record in rounds],
                         ["route_guided", "endpoint_only"])
        # The whole request may launch at most six distinct initials.  The
        # first route-guided round launches five; the endpoint-only recovery
        # may launch only the one still-unused 2.6-length shape.
        self.assertEqual([record["initial_count"] for record in rounds],
                         [5, 1])
        self.assertEqual(
            [record["new_initial_count"] for record in rounds], [5, 1])
        self.assertEqual(len(calls), 6)
        self.assertEqual(len(set(calls)), 6)
        self.assertEqual(result.search["initial_count_total"], 6)
        self.assertEqual(result.search["initial_limit"], 6)
        self.assertEqual(
            result.search["duplicate_initial_launch_count"], 0)
        self.assertEqual(len(result.search["initial_keys"]), 6)
        self.assertEqual(result.search["recovery_count"], 1)
        recovery = result.search["recoveries"][0]
        self.assertEqual(recovery["action"], "endpoint_only_initial")
        self.assertEqual(recovery["reason"], "SOLVER_NOT_CONVERGED")
        self.assertEqual(recovery["first_failure_layer"], "GEOMETRY")
        # The degenerate guide exposes exactly one branch: no winding retry
        # exists, and the endpoint-only round uses the historical shortest wrap.
        self.assertEqual(rounds[0]["winding_candidate_rank"], 0)
        self.assertEqual(rounds[0]["winding_k"], 0)
        self.assertEqual(
            rounds[0]["heading_branch"]["source"], "test_single_branch")
        self.assertEqual(rounds[1]["winding_k"], None)
        self.assertEqual(rounds[1]["heading_branch"], None)

    def test_solver_non_convergence_retries_exactly_one_next_branch(self):
        grid = empty_grid()
        request = moving_request()
        calls = []
        solver_packets = []

        def fake_failure(_request, _config, packet, guess,
                         wall_limit_s=None, **kwargs):
            calls.append(initial_guess_key(guess))
            solver_packets.append(packet)
            return fake_solver_failure(None, None, None, None)

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_failure):
            result = plan_virtual_a(map_request(request, grid))
        self.assertFalse(result.executable)
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE")
        self.assertEqual(result.reason, "SOLVER_NOT_CONVERGED")
        self.assertEqual(result.search["recovery_count"], 1)
        recovery = result.search["recoveries"][0]
        self.assertEqual(recovery["action"], "winding_branch")
        self.assertEqual(recovery["from_route_index"], 0)
        self.assertEqual(recovery["to_route_index"], 0)
        candidates = result.search["winding"]["candidates"]
        self.assertEqual(len(candidates), 3)
        self.assertEqual(
            [candidate["candidate_rank"] for candidate in candidates],
            [0, 1, 2])
        self.assertEqual(candidates[0]["winding_k"], 0)
        self.assertEqual(candidates[1]["winding_k"], -1)
        self.assertEqual(recovery["from_candidate_rank"], 0)
        self.assertEqual(recovery["to_candidate_rank"], 1)
        self.assertEqual(
            recovery["from_winding_k"], candidates[0]["winding_k"])
        self.assertEqual(
            recovery["to_winding_k"], candidates[1]["winding_k"])
        rounds = result.search["rounds"]
        self.assertEqual(len(rounds), 2)
        self.assertEqual(
            [record["initial_mode"] for record in rounds],
            ["route_guided", "route_guided"])
        self.assertEqual(
            [record["winding_k"] for record in rounds], [0, -1])
        self.assertEqual(
            [record["winding_candidate_rank"] for record in rounds], [0, 1])
        self.assertEqual(
            [record["initial_count"] for record in rounds], [5, 1])
        self.assertGreater(rounds[0]["guide_length_m"], 8.0)
        self.assertAlmostEqual(
            rounds[0]["guide_length_m"],
            result.search["routes"][0]["length_m"], places=9)
        self.assertEqual(
            rounds[0]["guide_length_m"], rounds[1]["guide_length_m"])
        # The second round launches only the same route's next-branch initial:
        # its identity carries k=-1 and no already-launched initial repeats.
        self.assertEqual(len(calls), 6)
        self.assertEqual(len(set(calls)), 6)
        self.assertEqual(calls[-1][-1], -1)
        self.assertEqual(result.search["initial_count_total"], 6)
        self.assertEqual(result.search["duplicate_initial_launch_count"], 0)
        self.assertTrue(result.attempts)
        self.assertTrue(all(
            attempt.winding_k is not None for attempt in result.attempts))
        self.assertEqual(
            result.attempts[-1].winding_k, -1)
        self.assertEqual(result.attempts[-1].heading_base_rad, 0.0)
        self.assertEqual(
            result.attempts[-1].guide_turn_rad, 0.0)
        for packet in solver_packets:
            self.assertIn("heading_branch", packet)
            self.assertIn("guide_length_m", packet)
        self.assertEqual(
            solver_packets[-1]["heading_branch"],
            candidates[1])

    def test_non_endpoint_audit_rejection_never_guesses_winding(self):
        grid = empty_grid()
        request = moving_request()

        def fake_success(_request, _config, _packet, guess,
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

        def forced_rejection(*_args, **_kwargs):
            return SimpleNamespace(
                passed=False,
                reason="CERTIFICATE_ERROR",
                detail="forced non-endpoint certification failure",
                scope="geometry",
                check_count=0,
                first_failure=None,
                metrics={})

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_success), mock.patch.object(
                optimizer_module, "audit_geometry",
                side_effect=forced_rejection):
            result = plan_virtual_a(map_request(request, grid))
        self.assertFalse(result.executable)
        self.assertEqual(result.reason, "AUDIT_REJECTED")
        self.assertEqual(result.search["recovery_count"], 0)
        self.assertEqual(result.search["recoveries"], [])
        self.assertEqual(result.search["winding"]["selected_rank"], 0)
        self.assertTrue(result.attempts)
        self.assertTrue(all(
            attempt.status == "SOLVER_SUCCESS_AUDIT_REJECTED"
            for attempt in result.attempts))

    def test_winding_retry_uses_the_concrete_audit_rejection(self):
        def rejection(status, reason):
            return SimpleNamespace(status=status, rejection_reason=reason)

        endpoint = rejection(
            "SOLVER_SUCCESS_AUDIT_REJECTED", "ENDPOINT_HEADING_ERROR")
        certificate = rejection(
            "SOLVER_SUCCESS_AUDIT_REJECTED", "CERTIFICATE_ERROR")
        self.assertTrue(_winding_retry_allowed(SimpleNamespace(
            reason="AUDIT_REJECTED", attempts=(certificate, endpoint))))
        # The optimizer takes the concrete reason from the last rejected
        # attempt: a later certification failure is not an endpoint limit.
        self.assertFalse(_winding_retry_allowed(SimpleNamespace(
            reason="AUDIT_REJECTED", attempts=(endpoint, certificate))))
        self.assertFalse(_winding_retry_allowed(SimpleNamespace(
            reason="AUDIT_REJECTED", attempts=())))
        self.assertFalse(_winding_retry_allowed(SimpleNamespace(
            reason="SOFT_BUDGET_EXHAUSTED", attempts=(endpoint,))))

    def test_dynamics_failure_never_enters_the_winding_retry(self):
        grid = box_grid()
        request = moving_request()
        path = straight_path(8.0)
        request, _timing, _context = retime(path, grid)
        geometry = geometry_result_for(path, request)
        solve_calls = []

        def injected_geometry(*_args, **kwargs):
            solve_calls.append(kwargs.get("heading_branch"))
            return geometry

        def forced_dynamics(*_args, **_kwargs):
            return SimpleNamespace(
                passed=False,
                reason="DYNAMIC_AUDIT_REJECTED",
                detail="forced dynamics failure",
                scope="dynamics",
                check_count=1,
                first_failure=None,
                metrics={})

        with mock.patch.object(
                pipeline_module, "solve_open_path_guided",
                side_effect=injected_geometry), mock.patch.object(
                pipeline_module, "audit_dynamics",
                side_effect=forced_dynamics):
            result = plan_virtual_a(map_request(request, grid))
        self.assertEqual(result.status, "DYNAMICS_FAILED")
        self.assertEqual(result.reason, "DYNAMIC_AUDIT_REJECTED")
        self.assertEqual(result.search["recovery_count"], 0)
        self.assertEqual(result.search["recoveries"], [])
        self.assertEqual(len(solve_calls), 1)
        self.assertIsInstance(solve_calls[0], HeadingBranch)
        self.assertEqual(solve_calls[0].winding_k, 0)

    def test_solver_auto_fallback_is_one_initial_and_no_fake_recovery(self):
        grid = empty_grid()
        request = moving_request()
        ipopt_calls = []
        slsqp_calls = []

        def fake_ipopt(_request, _config, _packet, guess,
                       wall_limit_s=None, **kwargs):
            ipopt_calls.append(initial_guess_key(guess))
            return fake_solver_failure(None, None, None, None)

        def fake_slsqp(_request, _config, _packet, guess,
                       wall_limit_s=None, **kwargs):
            slsqp_calls.append(initial_guess_key(guess))
            return fake_solver_failure(None, None, None, None)

        planner = PlannerConfig(
            max_initials=1, solver="auto", allow_scipy_fallback=True)
        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_ipopt), mock.patch.object(
                optimizer_module, "_solve_with_scipy",
                side_effect=fake_slsqp):
            result = plan_virtual_a(
                map_request(request, grid, planner=planner))
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertIsNone(result.path)
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE")
        self.assertEqual(result.reason, "SOLVER_NOT_CONVERGED")
        # IPOPT -> SLSQP fallback yields two attempts for the same identity,
        # but only one distinct initial and no recovery when the pool is
        # exhausted.
        self.assertEqual(len(result.attempts), 2)
        self.assertEqual(
            len({initial_guess_key(attempt.initial)
                 for attempt in result.attempts}), 1)
        self.assertEqual(len(ipopt_calls), 1)
        self.assertEqual(ipopt_calls, slsqp_calls)
        self.assertEqual(result.search["initial_count_total"], 1)
        self.assertEqual(result.search["recovery_count"], 0)
        self.assertEqual(result.search["recoveries"], [])

    def test_obb_failure_switches_to_one_untried_route_then_stops(self):
        grid = box_grid()
        request = moving_request()
        path = straight_path(8.0)
        request, _timing, _context = retime(path, grid)
        geometry = geometry_result_for(path, request)
        solve_calls = []

        def injected_geometry(*args, **kwargs):
            solve_calls.append(kwargs.get("route_guides"))
            return geometry

        def forced_collision(*args, **kwargs):
            return SimpleNamespace(
                passed=False,
                reason="MAP_OBB_COLLISION",
                first_failure="vehicle 2",
                detail="forced collision at the audited station",
                location={"qf": [4.0, 0.0], "station": 4.0},
                metrics={"margin": -0.2},
                check_count=3)

        with mock.patch.object(
                pipeline_module, "solve_open_path_guided",
                side_effect=injected_geometry), mock.patch.object(
                pipeline_module, "audit_collision",
                side_effect=forced_collision):
            result = plan_virtual_a(map_request(request, grid))
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertIsNone(result.path)
        self.assertEqual(result.status, "COLLISION_FAILED")
        self.assertEqual(result.reason, "MAP_OBB_COLLISION")
        self.assertEqual(result.first_failure_layer, "COLLISION")
        self.assertEqual(len(solve_calls), 2)
        self.assertEqual(result.search["recovery_count"], 1)
        recovery = result.search["recoveries"][0]
        self.assertEqual(recovery["action"], "route_switch")
        self.assertEqual(recovery["reason"], "MAP_OBB_COLLISION")
        self.assertEqual(recovery["from_route_index"], 0)
        self.assertEqual(recovery["to_route_index"], 1)
        self.assertEqual(len(result.search["rounds"]), 2)
        self.assertEqual(result.search["route_count"], 3)

    def test_budget_exhaustion_between_route_generation_and_solve(self):
        grid = empty_grid()
        request = moving_request()
        guides = make_route_guides(
            grid, request.start_qf, request.goal_qf, RouteGuideConfig())
        solver_calls = []

        def slow_generation(*args, **kwargs):
            time.sleep(0.3)
            return RouteGuideSearchResult(
                routes=tuple(guides), attempts=(),
                stop_reason="MAX_ROUTES", route_clearance_m=0.10,
                direct_los={})

        def counting_solver(*args, **kwargs):
            solver_calls.append(1)
            raise AssertionError("the solver must not start")

        with mock.patch.object(
                pipeline_module, "G4_TOTAL_SOFT_BUDGET_S", 0.2), \
                mock.patch.object(
                    pipeline_module, "make_route_guides",
                    side_effect=slow_generation), \
                mock.patch.object(
                    optimizer_module, "_solve_with_ipopt",
                    side_effect=counting_solver), \
                mock.patch.object(
                    optimizer_module, "_solve_with_scipy",
                    side_effect=counting_solver):
            result = plan_virtual_a(map_request(request, grid))
        self.assertEqual(solver_calls, [])
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertEqual(result.status, "BUDGET_EXHAUSTED")
        self.assertEqual(result.reason, "SOFT_BUDGET_EXHAUSTED")
        self.assertEqual(result.first_failure_layer, "PLANNING")


class RequestWideInitialSchedulingTests(unittest.TestCase):
    def test_route_audit_stops_before_spending_endpoint_family(self):
        grid = empty_grid()
        request = moving_request()
        guides = make_route_guides(
            grid, request.start_qf, request.goal_qf, RouteGuideConfig())
        path = straight_path(8.0)
        passing_audit = geometry_result_for(path, request).audit
        calls = []

        def fake_success(_request, _config, _packet, guess,
                         wall_limit_s=None, **kwargs):
            calls.append(initial_guess_key(guess))
            return {
                "solver": "ipopt",
                "z": np.asarray(guess.vector, dtype=float),
                "objective": 1.0,
                "solver_status": "Solve_Succeeded",
                "success": True,
                "iterations": 1,
                "detail": "fake solver success",
                "wall_time_s": 0.0,
            }

        def fake_candidate(_request, _guess, _result):
            return path

        def fake_audit(_request, _candidate, _config):
            return passing_audit

        config = PlannerConfig(max_initials=6)
        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_success), mock.patch.object(
                optimizer_module, "_candidate_path",
                side_effect=fake_candidate), mock.patch.object(
                optimizer_module, "audit_geometry",
                side_effect=fake_audit):
            stopped = solve_open_path_guided(
                request, config, route_guides=guides,
                stop_after_route_audit=True)
            self.assertEqual(stopped.status, "GEOMETRY_FEASIBLE")
            self.assertEqual(len(stopped.attempts), 1)
            self.assertEqual(
                stopped.attempts[0].initial_source, "map_route_guide")
            # The route round constructed five identities, but only the
            # route initial was launched; the endpoint family stayed unused.
            self.assertEqual(stopped.initial_count, 5)
            self.assertEqual(len(calls), 1)
            stopped_keys = set(calls)

            calls.clear()
            ranked = solve_open_path_guided(
                request, config, route_guides=guides,
                stop_after_route_audit=False)
            self.assertEqual(ranked.status, "GEOMETRY_FEASIBLE")
            self.assertEqual(len(ranked.attempts), 5)
            self.assertEqual(len(calls), 5)
            self.assertFalse(stopped_keys & set(calls[1:]))
            self.assertEqual(
                {initial_guess_key(attempt.initial)
                 for attempt in ranked.attempts}, set(calls))


class AdjacentPortalReallocationTests(unittest.TestCase):
    def test_positive_candidate_moves_one_boundary_inside_true_overlap(self):
        path = straight_path(8.0)
        cases = (
            # Right-hand direct segment [0.5, 1.0], assigned corridor 1.
            ("right", portal_chain_and_allocation(),
             width_failure(4.1, 1), 4.1 / 8.0),
            # Left-hand direct segment [0.0, 0.5], assigned corridor 0.
            # The overlap is [3.5, 5.0], so the failure at s=3.7 is already
            # inside both original halfspaces.
            ("left", portal_chain_and_allocation(second_x0=3.5),
             width_failure(3.7, 0), 3.7 / 8.0),
        )
        for side, (chain, allocation), containment, failure_sigma in cases:
            with self.subTest(side=side):
                new = reallocate_adjacent_portal_once(
                    path, chain, allocation, containment)
                self.assertIsNotNone(new)
                self.assertEqual(len(new), len(allocation))
                self.assertEqual(
                    [entry[2] for entry in new],
                    [entry[2] for entry in allocation])
                self.assertAlmostEqual(new[0][0], 0.0)
                self.assertAlmostEqual(new[-1][1], 1.0)
                for index in range(len(new) - 1):
                    self.assertAlmostEqual(
                        new[index][1], new[index + 1][0])
                # The old switch is sigma 0.5 (s=4.0).  The failure sigma
                # lies in the directly adjacent segment on one side, and
                # the old switch is inside the true overlap [4, 5] (right)
                # or [3.5, 5] (left).  The deterministic midpoint moves the
                # single boundary strictly inside that overlap.
                self.assertGreater(new[0][1], 0.5)
                self.assertLess(new[0][1], 5.0 / 8.0)
                self.assertAlmostEqual(
                    containment["first_failure"]["s_m"] / path.length_m,
                    failure_sigma)
                self.assertEqual(
                    validate_corridor_allocation(chain, new), tuple(new))
                qf = path.state_at_s(new[0][1] * path.length_m).qf
                point = (qf.x, qf.y)
                for region in chain.regions:
                    violation = max(
                        row[0] * point[0] + row[1] * point[1] - limit
                        for row, limit in zip(region.A, region.b))
                    self.assertLessEqual(violation, 1e-8)

    def test_failure_far_from_switch_or_degenerate_overlap_returns_none(self):
        path = straight_path(8.0)
        chain, allocation = portal_chain_and_allocation()
        # s=1 m is inside corridor 0, but the failure sigma 0.125 is not
        # inside the local audit-resolution neighbourhood of the adjacent
        # corridor's common region; no recovery candidate exists.
        self.assertIsNone(reallocate_adjacent_portal_once(
            path, chain, allocation, width_failure(1.0, 0)))
        # Touching corridors share only the single point x=5: there is no
        # non-degenerate overlap parameter interval.
        touching_chain, touching_allocation = \
            portal_chain_and_allocation(second_x0=5.0)
        self.assertIsNone(reallocate_adjacent_portal_once(
            path, touching_chain, touching_allocation,
            width_failure(5.0, 0)))
        # C3 rework 02: the failure point is only 5e-6 m from corridor 0's
        # y-boundary (the old boundary-proximity rule used 1e-5 m and would
        # accept it), and the old portal at sigma 0.75 is legal: s=6.0 lies
        # on the x=6 corner shared by both original halfspaces.  But sigma
        # 4.99/8=0.62375 lies in the earlier segment [0.5, 0.625), not in
        # either segment adjacent to the switch at sigma 0.75
        # ([0.625, 0.75] or [0.75, 1.0]), so reallocation must be refused.
        far_chain = CorridorChain(
            regions=(
                corridor_rectangle(
                    -1.0, 6.0, -5e-6, 2.0, (0.0, 0.0), (4.0, 0.0)),
                corridor_rectangle(
                    4.0, 9.0, -2.0, 2.0, (4.0, 0.0), (8.0, 0.0))),
            route_source="test_far_switch",
            grid_sha256="a" * 64)
        far_allocation = (
            (0.0, 0.25, 0), (0.25, 0.5, 0), (0.5, 0.625, 0),
            (0.625, 0.75, 0), (0.75, 1.0, 1))
        self.assertEqual(
            validate_corridor_allocation(far_chain, far_allocation),
            tuple(far_allocation))
        failure_point = path.state_at_s(4.99).qf
        failure_violation = max(
            row[0] * failure_point.x + row[1] * failure_point.y - limit
            for row, limit in zip(
                far_chain.regions[0].A, far_chain.regions[0].b))
        self.assertGreaterEqual(failure_violation, -1e-5)
        switch_point = path.state_at_s(6.0).qf
        for region in far_chain.regions:
            switch_violation = max(
                row[0] * switch_point.x + row[1] * switch_point.y - limit
                for row, limit in zip(region.A, region.b))
            self.assertLessEqual(switch_violation, 1e-8)
        self.assertIsNone(reallocate_adjacent_portal_once(
            path, far_chain, far_allocation, width_failure(4.99, 0)))
        # The failure is in the switch's left allocation segment but is
        # outside the adjacent corridor by 1.0 m; the old switch lies in
        # the overlap, yet the local width-derived neighbourhood has no
        # common-region point, so this is not a fixed-sigma switch failure.
        tight_chain = CorridorChain(
            regions=(
                corridor_rectangle(
                    -1.0, 5.0, -2.0, 5e-6, (0.0, 0.0), (4.0, 0.0)),
                corridor_rectangle(
                    4.0, 9.0, -2.0, 2.0, (4.0, 0.0), (8.0, 0.0))),
            route_source="test_tight_boundary",
            grid_sha256="a" * 64)
        self.assertIsNone(reallocate_adjacent_portal_once(
            path, tight_chain, allocation, width_failure(3.0, 0)))
        malformed = width_failure(4.0, 0)
        malformed["first_failure"]["s_m"] = float("nan")
        self.assertIsNone(reallocate_adjacent_portal_once(
            path, chain, allocation, malformed))

    def _pipeline_fixture(self):
        grid = empty_grid()
        request = moving_request()
        planner = replace(
            PLANNER, guidance_mode="corridor",
            corridor_enforcement="hard")
        chain, allocation = portal_chain_and_allocation()
        guide = GuideRoute(
            points=((0.0, 0.0), (8.0, 0.0)),
            source="test_corridor_guide",
            grid_sha256=chain.grid_sha256)
        chain_audit = {
            "passed": True,
            "reason": "CORRIDOR_CHAIN_VERIFIED",
            "detail": "",
            "grid_sha256": chain.grid_sha256,
            "route_source": chain.route_source,
            "region_count": len(chain.regions),
            "regions": [],
            "portals": [],
            "min_region_area_m2": 1.0,
            "max_seed_violation_m": 0.0,
            "min_portal_radius_m": 0.5,
            "min_portal_area_m2": 1.0,
            "first_collision": None,
            "thresholds": {},
        }
        return (grid, request, planner, chain, allocation, guide,
                chain_audit)

    def test_pipeline_reallocates_once_and_stops_on_second_failure(self):
        grid, request, planner, chain, allocation, guide, chain_audit = \
            self._pipeline_fixture()
        path = straight_path(8.0)
        solve_calls = []
        containment_calls = []

        def fake_solve(_request, _config, *args, **kwargs):
            route_guides = tuple(kwargs.get("route_guides", ()))
            excluded = set(kwargs.get("excluded_initial_keys", ()))
            cap = kwargs.get("max_new_initials")
            guesses = [
                guess for guess in make_initial_guesses(
                    _request, _config, route_guides=route_guides)
                if initial_guess_key(guess) not in excluded]
            if len(solve_calls) == 0:
                # The first round launched four distinct initials; the
                # fifth constructed shape remains for the reallocated round.
                guesses = guesses[:4]
            if cap is not None:
                guesses = guesses[:int(cap)]
            solve_calls.append({
                "keys": [initial_guess_key(guess) for guess in guesses],
                "excluded": excluded,
                "max_new": cap,
                "allocation": kwargs.get("corridor_allocation"),
            })
            return geometry_with_attempts(
                _request, path, planner, guesses)

        def fake_containment(_path, _chain, _allocation, **kwargs):
            containment_calls.append(
                [list(entry) for entry in _allocation])
            # sigma 0.5125 lies in the right-hand allocation segment
            # [0.5, 1.0] directly adjacent to the switch at 0.5.
            return width_failure(4.1, 1)

        with mock.patch.object(
                pipeline_module, "build_corridor_chain",
                return_value=chain), mock.patch.object(
                pipeline_module, "audit_corridor_chain",
                return_value=chain_audit), mock.patch.object(
                pipeline_module, "make_corridor_guide",
                return_value=guide), mock.patch.object(
                pipeline_module, "allocate_corridors",
                return_value=allocation), mock.patch.object(
                pipeline_module, "audit_path_in_corridors",
                side_effect=fake_containment), mock.patch.object(
                pipeline_module, "solve_open_path_guided",
                side_effect=fake_solve):
            result = plan_virtual_a(
                map_request(request, grid, planner=planner))
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertIsNone(result.path)
        self.assertEqual(result.status, "GEOMETRY_FAILED")
        self.assertEqual(result.reason, "CORRIDOR_AUDIT_WIDTH_LIMIT")
        self.assertEqual(len(solve_calls), 2)
        self.assertEqual(len(containment_calls), 2)
        self.assertEqual(result.search["recovery_count"], 1)
        recovery = result.search["recoveries"][0]
        self.assertEqual(
            recovery["action"], "adjacent_portal_reallocation")
        self.assertEqual(recovery["adjacent_corridors"], [0, 1])
        self.assertAlmostEqual(recovery["old_boundary_sigma"], 0.5)
        self.assertGreater(recovery["new_boundary_sigma"], 0.5)
        self.assertLess(recovery["new_boundary_sigma"], 0.625)
        self.assertAlmostEqual(recovery["failure_sigma"], 0.5125)
        self.assertEqual(
            recovery["trigger_reason"], "CORRIDOR_AUDIT_WIDTH_LIMIT")
        reallocation = result.search["reallocation"]
        self.assertTrue(reallocation["available"])
        self.assertEqual(
            reallocation["status"],
            "adjacent_portal_reallocation_applied")
        # The recovery round receives the new allocation and only unused
        # initial identities.
        second = solve_calls[1]
        self.assertEqual(second["max_new"], 2)
        self.assertEqual(second["excluded"], set(solve_calls[0]["keys"]))
        self.assertEqual(
            [list(entry) for entry in second["allocation"]],
            reallocation["new_allocation"])
        self.assertEqual(
            [record["new_initial_count"]
             for record in result.search["rounds"]], [4, 1])
        self.assertEqual(result.search["initial_count_total"], 5)
        self.assertEqual(
            result.search["duplicate_initial_launch_count"], 0)
        corridor_rounds = result.search["corridor"]["rounds"]
        self.assertEqual(len(corridor_rounds), 2)
        self.assertEqual(
            corridor_rounds[-1]["allocation_source"],
            "adjacent_portal_reallocation")
        self.assertEqual(
            corridor_rounds[-1]["allocation"],
            reallocation["new_allocation"])

    def test_pipeline_far_from_switch_failure_does_not_spend_recovery(self):
        grid, request, planner, _chain, _fixture_allocation, guide, \
            chain_audit = self._pipeline_fixture()
        # S1-like far failure: sigma 0.62375 lies in the earlier
        # corridor-0 segment [0.5, 0.625], while the real switch to
        # corridor 1 is at sigma 0.75.  The failure point is only 5e-6 m
        # from corridor 0's boundary and the old portal at sigma 0.75 is
        # legal, so only the switch-adjacency rule can reject it.
        far_chain = CorridorChain(
            regions=(
                corridor_rectangle(
                    -1.0, 6.0, -5e-6, 2.0, (0.0, 0.0), (4.0, 0.0)),
                corridor_rectangle(
                    4.0, 9.0, -2.0, 2.0, (4.0, 0.0), (8.0, 0.0))),
            route_source="test_far_switch",
            grid_sha256="a" * 64)
        far_allocation = (
            (0.0, 0.25, 0), (0.25, 0.5, 0), (0.5, 0.625, 0),
            (0.625, 0.75, 0), (0.75, 1.0, 1))
        path = straight_path(8.0)
        solve_calls = []
        containment_calls = []

        def fake_solve(_request, _config, *args, **kwargs):
            route_guides = tuple(kwargs.get("route_guides", ()))
            excluded = set(kwargs.get("excluded_initial_keys", ()))
            cap = kwargs.get("max_new_initials")
            guesses = [
                guess for guess in make_initial_guesses(
                    _request, _config, route_guides=route_guides)
                if initial_guess_key(guess) not in excluded]
            if len(solve_calls) == 0:
                guesses = guesses[:4]
            if cap is not None:
                guesses = guesses[:int(cap)]
            solve_calls.append([
                initial_guess_key(guess) for guess in guesses])
            return geometry_with_attempts(
                _request, path, planner, guesses)

        def fake_containment(_path, _chain, _allocation, **kwargs):
            containment_calls.append(
                [list(entry) for entry in _allocation])
            return width_failure(4.99, 0)

        with mock.patch.object(
                pipeline_module, "build_corridor_chain",
                return_value=far_chain), mock.patch.object(
                pipeline_module, "audit_corridor_chain",
                return_value=chain_audit), mock.patch.object(
                pipeline_module, "make_corridor_guide",
                return_value=guide), mock.patch.object(
                pipeline_module, "allocate_corridors",
                return_value=far_allocation), mock.patch.object(
                pipeline_module, "audit_path_in_corridors",
                side_effect=fake_containment), mock.patch.object(
                pipeline_module, "solve_open_path_guided",
                side_effect=fake_solve):
            result = plan_virtual_a(
                map_request(request, grid, planner=planner))
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertIsNone(result.path)
        self.assertEqual(result.status, "GEOMETRY_FAILED")
        self.assertEqual(result.reason, "CORRIDOR_AUDIT_WIDTH_LIMIT")
        self.assertEqual(len(solve_calls), 1)
        self.assertEqual(len(containment_calls), 1)
        self.assertEqual(
            containment_calls[0],
            [list(entry) for entry in far_allocation])
        self.assertEqual(len(result.search["rounds"]), 1)
        self.assertEqual(len(result.search["corridor"]["rounds"]), 1)
        self.assertEqual(result.search["recovery_count"], 0)
        self.assertEqual(result.search["recoveries"], [])
        reallocation = result.search["reallocation"]
        self.assertTrue(reallocation["attempted"])
        self.assertFalse(reallocation["available"])
        self.assertFalse(reallocation["consumed_recovery"])
        self.assertEqual(
            reallocation["status"],
            "fixed_sigma_reallocation_unavailable")
        self.assertEqual(
            reallocation["detail"],
            "no eligible adjacent allocation segment with a "
            "non-degenerate local overlap; the fixed-sigma "
            "failure is not at a switch")
        self.assertIsNone(reallocation["new_allocation"])
        self.assertAlmostEqual(reallocation["failure_sigma"], 0.62375)
        self.assertEqual(reallocation["failure_corridor_index"], 0)
        self.assertEqual(
            [list(entry) for entry in far_allocation],
            reallocation["old_allocation"])

    def test_initial_pool_exhaustion_records_no_fake_recovery(self):
        grid, request, planner, chain, allocation, guide, chain_audit = \
            self._pipeline_fixture()
        planner = replace(planner, max_initials=1)
        path = straight_path(8.0)
        solve_calls = []
        containment_calls = []

        def fake_solve(_request, _config, *args, **kwargs):
            route_guides = tuple(kwargs.get("route_guides", ()))
            cap = kwargs.get("max_new_initials")
            guesses = list(make_initial_guesses(
                _request, _config, route_guides=route_guides))
            if cap is not None:
                guesses = guesses[:int(cap)]
            solve_calls.append([initial_guess_key(guess) for guess in guesses])
            return geometry_with_attempts(
                _request, path, planner, guesses)

        def fake_containment(*_args, **_kwargs):
            containment_calls.append(True)
            return width_failure(4.0, 0)

        with mock.patch.object(
                pipeline_module, "build_corridor_chain",
                return_value=chain), mock.patch.object(
                pipeline_module, "audit_corridor_chain",
                return_value=chain_audit), mock.patch.object(
                pipeline_module, "make_corridor_guide",
                return_value=guide), mock.patch.object(
                pipeline_module, "allocate_corridors",
                return_value=allocation), mock.patch.object(
                pipeline_module, "audit_path_in_corridors",
                side_effect=fake_containment), mock.patch.object(
                pipeline_module, "solve_open_path_guided",
                side_effect=fake_solve):
            result = plan_virtual_a(
                map_request(request, grid, planner=planner))
        self.assertEqual(result.status, "GEOMETRY_FAILED")
        self.assertEqual(result.reason, "CORRIDOR_AUDIT_WIDTH_LIMIT")
        self.assertIsNone(result.path)
        self.assertIsNone(result.plan)
        self.assertEqual(len(solve_calls), 1)
        self.assertEqual(len(containment_calls), 1)
        self.assertEqual(result.search["initial_limit"], 1)
        self.assertEqual(result.search["initial_count_total"], 1)
        self.assertEqual(
            result.search["duplicate_initial_launch_count"], 0)
        self.assertEqual(result.search["recovery_count"], 0)
        self.assertEqual(result.search["recoveries"], [])
        reallocation = result.search["reallocation"]
        self.assertFalse(reallocation["attempted"])
        self.assertFalse(reallocation["available"])
        self.assertFalse(reallocation["consumed_recovery"])
        self.assertEqual(
            reallocation["status"],
            "fixed_sigma_reallocation_unavailable")
        self.assertEqual(
            reallocation["detail"],
            "no unused initial slot remains for a reallocated solve")

    def test_pipeline_reallocation_leaves_no_plan_when_obb_fails(self):
        grid, request, planner, chain, allocation, guide, chain_audit = \
            self._pipeline_fixture()
        path = straight_path(8.0)
        solve_calls = []
        containment_calls = []

        def fake_solve(_request, _config, *args, **kwargs):
            route_guides = tuple(kwargs.get("route_guides", ()))
            excluded = set(kwargs.get("excluded_initial_keys", ()))
            cap = kwargs.get("max_new_initials")
            guesses = [
                guess for guess in make_initial_guesses(
                    _request, _config, route_guides=route_guides)
                if initial_guess_key(guess) not in excluded]
            if len(solve_calls) == 0:
                guesses = guesses[:4]
            if cap is not None:
                guesses = guesses[:int(cap)]
            solve_calls.append([
                initial_guess_key(guess) for guess in guesses])
            return geometry_with_attempts(
                _request, path, planner, guesses)

        def fake_containment(_path, _chain, _allocation, **kwargs):
            containment_calls.append(
                [list(entry) for entry in _allocation])
            if len(containment_calls) == 1:
                # sigma 0.5125 is switch-adjacent to the 0.5 boundary.
                return width_failure(4.1, 1)
            return {
                "passed": True,
                "reason": "CORRIDOR_CONTAINMENT_VERIFIED",
                "detail": "",
                "first_failure": None,
                "sample_count": 1,
                "max_bound_residual_m": -0.1,
            }

        def fake_retime(*args, **kwargs):
            return SimpleNamespace(
                status="DYNAMIC_FEASIBLE", reason="", detail="",
                T_s=1.0, plan=SimpleNamespace(kind="motion"),
                geometry_audit=None)

        def fake_dynamics(*args, **kwargs):
            return SimpleNamespace(
                passed=True, reason="", detail="", check_count=1,
                metrics={})

        def fake_collision(*args, **kwargs):
            return SimpleNamespace(
                passed=False,
                reason="MAP_OBB_COLLISION",
                first_failure="vehicle 2",
                detail="forced OBB failure after reallocation",
                location={"qf": [4.0, 0.0], "station": 4.0},
                metrics={"margin": -0.2},
                check_count=3)

        with mock.patch.object(
                pipeline_module, "build_corridor_chain",
                return_value=chain), mock.patch.object(
                pipeline_module, "audit_corridor_chain",
                return_value=chain_audit), mock.patch.object(
                pipeline_module, "make_corridor_guide",
                return_value=guide), mock.patch.object(
                pipeline_module, "allocate_corridors",
                return_value=allocation), mock.patch.object(
                pipeline_module, "audit_path_in_corridors",
                side_effect=fake_containment), mock.patch.object(
                pipeline_module, "retime_path",
                side_effect=fake_retime), mock.patch.object(
                pipeline_module, "audit_dynamics",
                side_effect=fake_dynamics), mock.patch.object(
                pipeline_module, "audit_collision",
                side_effect=fake_collision), mock.patch.object(
                pipeline_module, "solve_open_path_guided",
                side_effect=fake_solve):
            result = plan_virtual_a(
                map_request(request, grid, planner=planner))
        self.assertFalse(result.executable)
        self.assertIsNone(result.plan)
        self.assertIsNone(result.path)
        self.assertEqual(result.status, "COLLISION_FAILED")
        self.assertEqual(result.reason, "MAP_OBB_COLLISION")
        self.assertEqual(len(solve_calls), 2)
        self.assertEqual(len(containment_calls), 2)
        self.assertEqual(result.search["recovery_count"], 1)
        self.assertEqual(
            result.search["recoveries"][0]["action"],
            "adjacent_portal_reallocation")
        self.assertEqual(result.search["initial_count_total"], 5)


class RealRequestTests(unittest.TestCase):
    def test_real_success_and_real_failure_requests(self):
        grid = empty_grid()
        request = moving_request()
        success = plan_virtual_a(map_request(request, grid))
        self.assertEqual(success.status, "SUCCESS", success.as_dict())
        self.assertTrue(success.executable)
        self.assertIsNotNone(success.plan)
        self.assertIsNotNone(success.path)
        for report in (success.geometry_audit, success.dynamic_audit,
                       success.collision_audit):
            self.assertIsNotNone(report)
            self.assertTrue(report.passed, report.as_dict())
        self.assertEqual(success.search["recovery_count"], 0)
        self.assertEqual(success.search["route_count"], 1)
        self.assertLessEqual(success.search["initial_count"], 6)

        blocked_grid, _metadata = load_synthetic("blocked_start")
        hold = make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(0.0, 0.0, 0.0))
        failure = plan_virtual_a(map_request(hold, blocked_grid))
        self.assertFalse(failure.executable)
        self.assertIsNone(failure.plan)
        self.assertIsNone(failure.path)
        self.assertEqual(failure.status, "COLLISION_FAILED")
        self.assertEqual(failure.reason, "MAP_OBB_COLLISION")
        self.assertEqual(failure.first_failure_layer, "COLLISION")
        self.assertFalse(failure.collision_audit.passed)


if __name__ == "__main__":
    unittest.main()