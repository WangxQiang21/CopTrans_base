"""G2 endpoint-driven optimizer tests.

Every acceptance solve uses only the frozen physical request and the frozen
default :class:`PlannerConfig`.  Reference profiles live in
``tests/fixtures/virtual_a/existence_reference.json`` and are never passed to
``solve_open_path``.
"""
import inspect
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    GeometryResult,
    GoalPolicy,
    HeadingBranch,
    InitialAssumption,
    PlannerConfig,
    VirtualAContractError,
    VirtualARequest,
    request_fingerprint,
)
from fwis_v3_optimizer.formation_continuous_a.optimizer import (
    initial_guess_key,
    make_initial_guesses,
    solve_open_path,
    solve_open_path_guided,
)
from fwis_v3_optimizer.formation_continuous_a import optimizer as optimizer_module

ROOT = Path(__file__).resolve().parents[1]
FIXTURE_DIR = ROOT / "tests" / "fixtures" / "virtual_a"
REQUESTS = json.loads((FIXTURE_DIR / "open_requests.json").read_text())
REFERENCES = json.loads((FIXTURE_DIR / "existence_reference.json").read_text())
BASE = REQUESTS["base"]


def model_from_fixture():
    return FormationModel([
        VehicleOffset(row["vehicle_id"], row["x"], row["y"])
        for row in BASE["model"]["offsets"]])


def shapes_from_fixture():
    return {
        row["vehicle_id"]: VehicleShape(row["length_m"], row["width_m"])
        for row in BASE["model"]["vehicle_shapes"]}


def planner_config(**changes):
    values = dict(REQUESTS["planner_config"])
    values.update(changes)
    return PlannerConfig(**values)


def request_from_fixture(name, **changes):
    record = REQUESTS["requests"][name]
    goal = record["goal_qf"]
    headings = {int(k): float(v) for k, v in BASE["start_headings"].items()}
    values = {
        "plan_id": record["plan_id"],
        "formation_id": record["formation_id"],
        "model": model_from_fixture(),
        "vehicle_shapes": shapes_from_fixture(),
        "start_qf": FormationPose(**BASE["start_qf"]),
        "start_headings": headings,
        "goal_qf": FormationPose(goal["x"], goal["y"], goal["psi"]),
        "initial": InitialAssumption(**BASE["initial"]),
        "goal_policy": GoalPolicy(record["goal_policy"]),
        "limits": FormationTimingLimits(**BASE["limits"]),
    }
    values.update(changes)
    return VirtualARequest(**values)


def transformed_request(name, angle, tx, ty):
    request = request_from_fixture(name)
    c, s = math.cos(angle), math.sin(angle)
    start = request.start_qf
    goal = request.goal_qf

    def transform(x, y):
        return (tx + c * x - s * y, ty + s * x + c * y)

    start_x, start_y = transform(start.x, start.y)
    goal_x, goal_y = transform(goal.x, goal.y)
    return request_from_fixture(
        name,
        start_qf=FormationPose(start_x, start_y, start.psi + angle),
        start_headings={k: v + angle
                        for k, v in request.start_headings.items()},
        goal_qf=FormationPose(goal_x, goal_y, goal.psi + angle))


def c2_second_errors(guess):
    n = len(guess.kappas) - 1
    h = guess.S / n
    errors = []
    for j in range(1, n):
        left_c2 = (3.0 * (guess.kappas[j] - guess.kappas[j - 1])
                   - h * (2.0 * guess.etas[j - 1] + guess.etas[j]))
        right_c2 = (3.0 * (guess.kappas[j + 1] - guess.kappas[j])
                    - h * (2.0 * guess.etas[j] + guess.etas[j + 1]))
        errors.append(abs(
            (2.0 * left_c2 + 6.0 * (
                2.0 * (guess.kappas[j - 1] - guess.kappas[j])
                + h * (guess.etas[j - 1] + guess.etas[j]))) / (h * h)
            - 2.0 * right_c2 / (h * h)))
    return errors


class InitialGuessTests(unittest.TestCase):

    def test_default_config_is_the_frozen_fixture(self):
        self.assertEqual(planner_config(), PlannerConfig())

    def test_six_guesses_fixed_order_reconstructible_and_c2(self):
        request = request_from_fixture("E1")
        guesses = make_initial_guesses(request, planner_config())
        self.assertEqual(len(guesses), 6)
        self.assertEqual([g.label for g in guesses], [
            "length_multiplier=1.2;shape=left",
            "length_multiplier=1.2;shape=right",
            "length_multiplier=1.8;shape=left",
            "length_multiplier=1.8;shape=right",
            "length_multiplier=2.6;shape=left",
            "length_multiplier=2.6;shape=right",
        ])
        for guess in guesses:
            self.assertEqual(len(guess.vector), 2 * 8 + 3)
            self.assertEqual(guess.kappas[0], 0.0)
            self.assertEqual(guess.etas[0], 0.0)
            self.assertLessEqual(max(c2_second_errors(guess)), 1e-10)
            self.assertEqual(guess.metadata["projection"],
                             "clamped_cubic_spline_C2_eta0_etaN_zero")
            record = guess.reconstruct()
            self.assertEqual(record["source"], "automatic_endpoint_shape")
            self.assertEqual(record["label"], guess.label)
            self.assertNotIn("alpha", json.dumps(record))
            self.assertNotIn("fixture", json.dumps(record))
            self.assertNotIn("e1", json.dumps(record).lower())
        # The initial construction must not use plan-specific lookup data.
        source = Path(optimizer_module.__file__).read_text()
        for forbidden in ("open_requests", "existence_reference",
                          "tests/fixtures", "evaluation/", "alpha *",
                          "plan_id ==", "formation_id =="):
            self.assertNotIn(forbidden, source)
        self.assertEqual(list(inspect.signature(solve_open_path).parameters),
                         ["request", "config"])

    def test_initial_guess_uses_request_only_automatic_length(self):
        request = request_from_fixture("E1")
        distance = math.hypot(request.goal_qf.x - request.start_qf.x,
                              request.goal_qf.y - request.start_qf.y)
        guesses = make_initial_guesses(request, planner_config())
        for guess, multiplier in zip(guesses, (1.2, 1.2, 1.8, 1.8,
                                               2.6, 2.6)):
            self.assertAlmostEqual(
                guess.S, multiplier * max(distance, 1.0), places=10)

    def test_solve_reads_no_reference_or_fixture_file(self):
        # Import the solver binding before the guard so that plugin loading is
        # not confounded with the production data flow under test.
        import casadi  # noqa: F401
        request = request_from_fixture("E1")
        reads = []
        original_open = Path.open
        forbidden = ("open_requests", "existence_reference",
                     "tests/fixtures", "evaluation/")

        def guarded_open(path, *args, **kwargs):
            text = str(path).lower()
            reads.append(text)
            for token in forbidden:
                if token in text:
                    raise AssertionError(
                        "production solve opened forbidden reference: " + text)
            return original_open(path, *args, **kwargs)

        with mock.patch.object(Path, "open", guarded_open):
            result = solve_open_path(request, planner_config(max_initials=1))
        self.assertEqual(result.status, "GEOMETRY_FEASIBLE", result.as_dict())
        # The production path must not need a fixture, reference, or evidence
        # file at all; endpoint data and the frozen config are the only inputs.
        self.assertEqual(reads, [])


class AcceptanceSolveTests(unittest.TestCase):

    def test_six_endpoints_straight_and_clamped_straight_policy(self):
        config = planner_config()
        names = ("E1", "E2", "E3", "E4", "E5", "E6",
                 "STRAIGHT_LINE", "STRAIGHT_POLICY")
        results = {}
        for name in names:
            with self.subTest(request=name):
                request = request_from_fixture(name)
                result = solve_open_path(request, config)
                results[name] = result
                self.assertIsInstance(result, GeometryResult)
                self.assertEqual(result.status, "GEOMETRY_FEASIBLE",
                                 result.as_dict())
                self.assertIsNotNone(result.path)
                self.assertTrue(result.audit.passed)
                self.assertEqual(result.request_sha256,
                                 request_fingerprint(request))
                self.assertEqual(len(result.attempts), 6)
                for attempt in result.attempts:
                    self.assertEqual(attempt.solver, "ipopt")
                    self.assertEqual(attempt.status, "SOLVER_SUCCESS_AUDITED")
                    self.assertIsNotNone(attempt.audit)
                    self.assertTrue(attempt.audit.passed)
                endpoint = result.path.endpoint()
                goal = request.goal_qf
                self.assertLessEqual(
                    math.hypot(endpoint.x - goal.x, endpoint.y - goal.y),
                    0.005)
                self.assertLessEqual(
                    abs(math.atan2(math.sin(endpoint.psi - goal.psi),
                                   math.cos(endpoint.psi - goal.psi))),
                    math.radians(0.2))
                self.assertLessEqual(
                    result.audit.metrics["sample_audit"][
                        "max_pairwise_rigidity_error_m"], 1e-6)
                self.assertLessEqual(
                    result.audit.integration["position_budget_m"], 1e-7)
                self.assertLessEqual(
                    result.audit.integration["angle_budget_rad"], 1e-9)
                cmax = 1.0 / request.limits.r_min_ackermann
                for segment in result.audit.whole_segment_bounds:
                    for value in segment["max_abs_C_by_vehicle"].values():
                        self.assertLessEqual(value, cmax)
                    self.assertLessEqual(
                        segment["max_static_wheel_angle_rad"],
                        request.limits.steer_angle_max)
                if name == "STRAIGHT_POLICY":
                    terminal = result.path.profile.knots[-1]
                    self.assertLessEqual(abs(terminal.value), 1e-9)
                    self.assertLessEqual(abs(terminal.derivative), 1e-9)
                self.assertEqual(result.goal_heading_branch_rad,
                                 math.atan2(
                                     math.sin(goal.psi - request.start_qf.psi),
                                     math.cos(goal.psi - request.start_qf.psi)))
        # The straight target is genuinely a straight request, not a reference
        # curve lookup; its solution has a very small curvature envelope.
        straight = results["STRAIGHT_LINE"]
        self.assertLessEqual(
            max(abs(segment["max_static_wheel_angle_rad"])
                for segment in straight.audit.whole_segment_bounds), 1e-8)

    def test_rotation_and_translation_equivalence(self):
        original = request_from_fixture("E1")
        transformed = transformed_request("E1", 0.7, 2.0, -3.0)
        original_result = solve_open_path(original, planner_config())
        transformed_result = solve_open_path(transformed, planner_config())
        self.assertEqual(original_result.status, "GEOMETRY_FEASIBLE")
        self.assertEqual(transformed_result.status, "GEOMETRY_FEASIBLE")
        self.assertAlmostEqual(original_result.path.length_m,
                               transformed_result.path.length_m, places=6)
        original_endpoint = original_result.path.endpoint()
        transformed_endpoint = transformed_result.path.endpoint()
        goal = transformed.goal_qf
        self.assertLessEqual(math.hypot(transformed_endpoint.x - goal.x,
                                        transformed_endpoint.y - goal.y),
                             0.005)
        # Compare the solved profiles: the automatic local problem must be
        # invariant under a common rigid transform.
        self.assertLessEqual(
            abs(original_result.path.profile.knots[-1].value
                - transformed_result.path.profile.knots[-1].value), 1e-6)

    def test_exact_hold_is_no_motion_but_near_or_turned_goal_is_not(self):
        exact = request_from_fixture(
            "STRAIGHT_LINE", goal_qf=FormationPose(0.0, 0.0, 0.0))
        result = solve_open_path(exact, planner_config())
        self.assertEqual(result.status, "NO_MOTION")
        self.assertTrue(result.path.is_hold)
        self.assertTrue(result.audit.passed)
        for goal in ((1e-9, 0.0, 0.0), (0.0, 0.0, 0.1)):
            near = request_from_fixture(
                "STRAIGHT_LINE", goal_qf=FormationPose(*goal))
            near_result = solve_open_path(near, planner_config())
            self.assertNotEqual(near_result.status, "NO_MOTION")
            self.assertIn(near_result.status, (
                "GEOMETRY_FEASIBLE", "NO_ACCEPTED_CANDIDATE",
                "BUDGET_EXHAUSTED", "NUMERICAL_FAILURE"))

    def test_small_budget_returns_budget_exhausted_without_path(self):
        request = request_from_fixture("E1")
        result = solve_open_path(request, planner_config(max_seconds=0.0))
        self.assertEqual(result.status, "BUDGET_EXHAUSTED")
        self.assertIsNone(result.path)
        self.assertTrue(result.deadline_exhausted)
        self.assertEqual(len(result.attempts), 0)

    def test_solver_exception_maps_to_numerical_failure(self):
        request = request_from_fixture("E1")
        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=RuntimeError("forced solver failure")):
            result = solve_open_path(
                request, planner_config(max_initials=1))
        self.assertEqual(result.status, "NUMERICAL_FAILURE")
        self.assertIsNone(result.path)
        self.assertTrue(result.attempts)
        self.assertEqual(result.attempts[0].status, "SOLVER_FAILED")
        self.assertEqual(result.attempts[0].solver_status, "IPOPT_EXCEPTION")

    def test_fake_solver_success_wrong_endpoint_is_not_feasible(self):
        request = request_from_fixture("E1")

        def fake_solver(_request, _config, _packet, guess, wall_limit_s=None):
            values = np.asarray(guess.vector, dtype=float).copy()
            # Scale curvature and eta together: the result remains C2 at
            # every node but reaches a different endpoint than the request.
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

        with mock.patch.object(optimizer_module, "_solve_with_ipopt",
                               side_effect=fake_solver):
            result = solve_open_path(
                request, planner_config(max_initials=2))
        self.assertNotEqual(result.status, "GEOMETRY_FEASIBLE")
        self.assertIsNone(result.path)
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE")
        self.assertEqual(result.reason, "AUDIT_REJECTED")
        self.assertTrue(result.attempts)
        for attempt in result.attempts:
            self.assertEqual(attempt.status,
                             "SOLVER_SUCCESS_AUDIT_REJECTED")
            self.assertFalse(attempt.audit.passed)

    def test_fake_solver_success_curvature_violation_is_not_feasible(self):
        request = request_from_fixture("E1")

        def fake_solver(_request, _config, _packet, guess, wall_limit_s=None):
            values = np.asarray(guess.vector, dtype=float).copy()
            values[2] = 0.5  # within the sidecar k bound
            values[1 + 8 + 1] = 20.0  # eta at the first interior node
            return {
                "solver": "ipopt",
                "z": values,
                "objective": 1.0,
                "solver_status": "Solve_Succeeded",
                "success": True,
                "iterations": 1,
                "detail": "fake curvature success",
                "wall_time_s": 0.0,
            }

        with mock.patch.object(optimizer_module, "_solve_with_ipopt",
                               side_effect=fake_solver):
            result = solve_open_path(
                request, planner_config(max_initials=2))
        self.assertNotEqual(result.status, "GEOMETRY_FEASIBLE")
        self.assertIsNone(result.path)

    def test_audit_rejection_survives_later_solver_failure(self):
        """A later iteration limit must not overwrite the audited rejection."""
        request = request_from_fixture("E1")

        def fake_rejection(_request, _config, _packet, guess,
                           wall_limit_s=None):
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

        def fake_failure(_request, _config, _packet, guess,
                         wall_limit_s=None):
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

        calls = []

        def sequence(*args, **kwargs):
            calls.append(1)
            if len(calls) == 1:
                return fake_rejection(*args, **kwargs)
            return fake_failure(*args, **kwargs)

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=sequence):
            result = solve_open_path(
                request, planner_config(max_initials=2))
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE")
        self.assertEqual(result.reason, "AUDIT_REJECTED")
        self.assertIn("ENDPOINT_POSITION_ERROR", result.detail)
        self.assertEqual(result.attempts[0].status,
                         "SOLVER_SUCCESS_AUDIT_REJECTED")
        self.assertEqual(result.attempts[1].status, "SOLVER_FAILED")
        self.assertEqual(result.attempts[1].rejection_reason,
                         "Maximum_Iterations_Exceeded")

    def test_all_nonconverged_attempts_report_solver_not_converged(self):
        request = request_from_fixture("E1")

        def fake_failure(_request, _config, _packet, guess,
                         wall_limit_s=None):
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

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_failure):
            result = solve_open_path(
                request, planner_config(max_initials=2))
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE")
        self.assertEqual(result.reason, "SOLVER_NOT_CONVERGED")
        self.assertTrue(result.attempts)
        self.assertTrue(all(attempt.status == "SOLVER_FAILED"
                            for attempt in result.attempts))

    def test_explicit_ipopt_never_starts_the_slsqp_fallback(self):
        request = request_from_fixture("E1")

        def fake_failure(_request, _config, _packet, guess,
                         wall_limit_s=None):
            return {
                "solver": "ipopt",
                "z": None,
                "objective": None,
                "solver_status": "Solve_Failed",
                "success": False,
                "iterations": 1,
                "detail": "fake failure",
                "wall_time_s": 0.0,
            }

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_failure), mock.patch.object(
                optimizer_module, "_solve_with_scipy") as fallback:
            result = solve_open_path(
                request, planner_config(
                    solver="ipopt", allow_scipy_fallback=True,
                    max_initials=1))
        fallback.assert_not_called()
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE")
        self.assertEqual(result.reason, "SOLVER_NOT_CONVERGED")
        self.assertEqual([attempt.solver for attempt in result.attempts],
                         ["ipopt"])

    def test_scipy_fallback_is_recorded_explicitly(self):
        request = request_from_fixture("E1")
        result = solve_open_path(
            request, planner_config(solver="scipy", max_initials=1))
        self.assertEqual(result.status, "GEOMETRY_FEASIBLE",
                         result.as_dict())
        self.assertTrue(result.attempts)
        self.assertEqual(result.attempts[0].solver, "scipy_slsqp")


class ConstraintResidualTests(unittest.TestCase):
    """Successful attempts must retain a finite, independently checkable
    diagnostic residual; malformed residual diagnostics fail closed."""

    def _expected_residual(self, request, config, z):
        packet = optimizer_module._search_packet(request, config)
        return optimizer_module._constraint_residual(
            z, request, config, packet)

    def test_real_ipopt_success_residual_is_finite_and_recomputable(self):
        request = request_from_fixture("E1")
        config = planner_config(max_initials=1)
        solver_results = []
        real_ipopt = optimizer_module._solve_with_ipopt

        def recording_ipopt(*args, **kwargs):
            result = real_ipopt(*args, **kwargs)
            solver_results.append(result)
            return result

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=recording_ipopt):
            result = solve_open_path(request, config)
        self.assertEqual(result.status, "GEOMETRY_FEASIBLE",
                         result.as_dict())
        self.assertEqual(len(solver_results), 1)
        self.assertEqual(len(result.attempts), 1)
        attempt = result.attempts[0]
        self.assertEqual(attempt.status, "SOLVER_SUCCESS_AUDITED")
        self.assertIsNotNone(attempt.constraint_residual)
        self.assertTrue(math.isfinite(attempt.constraint_residual))
        self.assertGreaterEqual(attempt.constraint_residual, 0.0)
        self.assertEqual(
            attempt.constraint_residual,
            self._expected_residual(
                request, config, solver_results[0]["z"]))
        self.assertEqual(
            attempt.as_dict()["constraint_residual"],
            attempt.constraint_residual)

    def test_real_slsqp_fallback_success_residual_is_finite_and_recomputable(self):
        request = request_from_fixture("E1")
        config = planner_config(solver="auto", max_initials=1)
        solver_results = []
        real_slsqp = optimizer_module._solve_with_scipy

        def failed_ipopt(_request, _config, _packet, _guess,
                         wall_limit_s=None):
            return {
                "solver": "ipopt",
                "z": None,
                "objective": None,
                "solver_status": "Solve_Failed",
                "success": False,
                "iterations": 2,
                "detail": "forced ordinary IPOPT failure for residual test",
                "wall_time_s": 0.01,
                "soft_deadline_triggered": False,
                "deadline_trigger_stage": "",
                "remaining_budget_s": 29.99,
            }

        def recording_slsqp(*args, **kwargs):
            result = real_slsqp(*args, **kwargs)
            solver_results.append(result)
            return result

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=failed_ipopt):
            with mock.patch.object(
                    optimizer_module, "_solve_with_scipy",
                    side_effect=recording_slsqp):
                result = solve_open_path(request, config)
        self.assertEqual(result.status, "GEOMETRY_FEASIBLE",
                         result.as_dict())
        self.assertEqual(len(solver_results), 1)
        self.assertEqual([attempt.status for attempt in result.attempts],
                         ["SOLVER_FAILED", "SOLVER_SUCCESS_AUDITED"])
        fallback_attempt = result.attempts[1]
        self.assertEqual(fallback_attempt.solver, "scipy_slsqp")
        self.assertIsNotNone(fallback_attempt.constraint_residual)
        self.assertTrue(math.isfinite(
            fallback_attempt.constraint_residual))
        self.assertGreaterEqual(fallback_attempt.constraint_residual, 0.0)
        self.assertEqual(
            fallback_attempt.constraint_residual,
            self._expected_residual(
                request, config, solver_results[0]["z"]))
        self.assertIsNone(result.attempts[0].constraint_residual)

    def test_missing_or_nonfinite_success_vector_never_reports_a_residual(self):
        request = request_from_fixture("E1")
        config = planner_config(max_initials=1)
        cases = ("missing", "nan", "inf")
        for case in cases:
            with self.subTest(case=case):
                audit_calls = []

                def fake_solver(_request, _config, _packet, guess,
                                wall_limit_s=None):
                    if case == "missing":
                        values = None
                    else:
                        values = np.asarray(
                            guess.vector, dtype=float).copy()
                        values[1] = (
                            float("nan") if case == "nan" else float("inf"))
                    return {
                        "solver": "ipopt",
                        "z": values,
                        "objective": 0.0,
                        "solver_status": "Solve_Succeeded",
                        "success": True,
                        "iterations": 1,
                        "detail": "forced {}".format(case),
                        "wall_time_s": 0.0,
                        "soft_deadline_triggered": False,
                        "deadline_trigger_stage": "",
                        "remaining_budget_s": 29.99,
                    }

                def recording_audit(*args, **kwargs):
                    audit_calls.append(1)
                    raise AssertionError(
                        "audit must not run for an invalid residual vector")

                with mock.patch.object(
                        optimizer_module, "_solve_with_ipopt",
                        side_effect=fake_solver):
                    with mock.patch.object(
                            optimizer_module, "audit_geometry",
                            side_effect=recording_audit):
                        result = solve_open_path(request, config)
                self.assertEqual(audit_calls, [])
                self.assertIsNone(result.path)
                self.assertNotEqual(result.status, "GEOMETRY_FEASIBLE")
                self.assertEqual(len(result.attempts), 1)
                attempt = result.attempts[0]
                self.assertEqual(attempt.status,
                                 "CONSTRAINT_RESIDUAL_FAILURE")
                self.assertIsNone(attempt.constraint_residual)
                self.assertIsNone(attempt.audit)
                self.assertTrue(attempt.rejection_reason.startswith(
                    "CONSTRAINT_RESIDUAL_FAILURE:"))
                self.assertIn("constraint residual unavailable",
                              attempt.detail)

    def test_residual_calculation_anomaly_fails_closed(self):
        request = request_from_fixture("E1")
        config = planner_config(max_initials=1)
        for mode in ("exception", "nan"):
            with self.subTest(mode=mode):
                audit_calls = []

                def fake_solver(_request, _config, _packet, guess,
                                wall_limit_s=None):
                    return {
                        "solver": "ipopt",
                        "z": np.asarray(guess.vector, dtype=float).copy(),
                        "objective": 0.0,
                        "solver_status": "Solve_Succeeded",
                        "success": True,
                        "iterations": 1,
                        "detail": "forced successful candidate",
                        "wall_time_s": 0.0,
                        "soft_deadline_triggered": False,
                        "deadline_trigger_stage": "",
                        "remaining_budget_s": 29.99,
                    }

                def broken_residual(*args, **kwargs):
                    if mode == "exception":
                        raise RuntimeError("forced residual computation error")
                    return float("nan")

                def recording_audit(*args, **kwargs):
                    audit_calls.append(1)
                    raise AssertionError(
                        "audit must not run after residual failure")

                with mock.patch.object(
                        optimizer_module, "_solve_with_ipopt",
                        side_effect=fake_solver):
                    with mock.patch.object(
                            optimizer_module, "_constraint_residual",
                            side_effect=broken_residual):
                        with mock.patch.object(
                                optimizer_module, "audit_geometry",
                                side_effect=recording_audit):
                            result = solve_open_path(request, config)
                self.assertEqual(audit_calls, [])
                self.assertIsNone(result.path)
                self.assertNotEqual(result.status, "GEOMETRY_FEASIBLE")
                self.assertEqual(len(result.attempts), 1)
                attempt = result.attempts[0]
                self.assertEqual(attempt.status,
                                 "CONSTRAINT_RESIDUAL_FAILURE")
                self.assertIsNone(attempt.constraint_residual)
                self.assertIsNone(attempt.audit)
                self.assertIn("constraint residual unavailable",
                              attempt.detail)
                if mode == "exception":
                    self.assertIn("forced residual computation error",
                                  attempt.detail)
                else:
                    self.assertIn("non-finite", attempt.detail)


class SoftDeadlineTests(unittest.TestCase):

    def _packet_and_guess(self):
        request = request_from_fixture("E1")
        config = planner_config(max_initials=1)
        guess = make_initial_guesses(request, config)[0]
        packet = optimizer_module._search_packet(request, config)
        return request, config, packet, guess

    def test_ipopt_max_wall_time_is_effective_and_truthful(self):
        request, config, packet, guess = self._packet_and_guess()
        result = optimizer_module._solve_with_ipopt(
            request, config, packet, guess, wall_limit_s=1e-9)
        self.assertEqual(result["solver_status"],
                         "Maximum_WallTime_Exceeded")
        self.assertFalse(result["success"])
        self.assertTrue(result["soft_deadline_triggered"])
        self.assertEqual(result["deadline_trigger_stage"],
                         "ipopt_max_wall_time")
        self.assertEqual(result["remaining_budget_s"], 1e-9)
        # The library returns only after its own internal overhead; this is
        # recorded honestly and is not a hard-real-time claim.
        self.assertGreater(result["wall_time_s"], 1e-9)

    def test_slsqp_iteration_callback_stops_and_marks_timeout(self):
        request, config, packet, guess = self._packet_and_guess()
        result = optimizer_module._solve_with_scipy(
            request, config, packet, guess, wall_limit_s=1e-9)
        self.assertEqual(result["solver_status"], "SLSQP_SOFT_DEADLINE")
        self.assertFalse(result["success"])
        self.assertTrue(result["soft_deadline_triggered"])
        self.assertEqual(result["deadline_trigger_stage"],
                         "slsqp_iteration_callback")
        self.assertGreaterEqual(result["iterations"], 1)

    def test_timed_out_success_flag_never_reaches_the_audit(self):
        request = request_from_fixture("E1")

        def timed_out_solver(_request, _config, _packet, guess,
                             wall_limit_s=None):
            return {
                "solver": "ipopt",
                "z": np.asarray(guess.vector, dtype=float).copy(),
                "objective": 0.0,
                # A malicious/stale success flag must not override timeout.
                "solver_status": "Maximum_WallTime_Exceeded",
                "success": True,
                "iterations": 1,
                "detail": "forced timeout",
                "wall_time_s": 0.01,
                "soft_deadline_triggered": True,
                "deadline_trigger_stage": "ipopt_max_wall_time",
                "remaining_budget_s": 1e-9,
            }

        audit_calls = []
        original_audit = optimizer_module.audit_geometry

        def recording_audit(*args, **kwargs):
            audit_calls.append(1)
            return original_audit(*args, **kwargs)

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=timed_out_solver):
            with mock.patch.object(
                    optimizer_module, "audit_geometry",
                    side_effect=recording_audit):
                result = solve_open_path(
                    request, planner_config(max_initials=2))
        self.assertEqual(result.status, "BUDGET_EXHAUSTED",
                         result.as_dict())
        self.assertIsNone(result.path)
        self.assertIsNone(result.audit)
        self.assertTrue(result.deadline_exhausted)
        self.assertEqual(audit_calls, [])
        self.assertTrue(result.attempts)
        self.assertEqual(result.attempts[0].status, "SOLVER_TIMEOUT")
        self.assertTrue(result.attempts[0].soft_deadline_triggered)
        self.assertEqual(result.attempts[0].deadline_trigger_stage,
                         "ipopt_max_wall_time")
        self.assertEqual(result.attempts[0].rejection_reason,
                         "SOFT_BUDGET_EXHAUSTED")
        self.assertEqual(result.deadline_summary["trigger_stage"],
                         "ipopt_max_wall_time")
        self.assertEqual(result.deadline_summary["timeout_solver_status"],
                         "Maximum_WallTime_Exceeded")
        self.assertFalse(
            result.deadline_summary["audited_candidate_preserved"])
        serialized = result.as_dict()
        self.assertTrue(serialized["deadline_exhausted"])
        self.assertTrue(serialized["attempts"][0][
            "soft_deadline_triggered"])
        self.assertEqual(serialized["deadline_summary"]["trigger_stage"],
                         "ipopt_max_wall_time")

    def test_timeout_preserves_only_a_previously_audited_candidate(self):
        request, config, packet, guess = self._packet_and_guess()
        # Obtain one genuine solver result and audit it after the fake
        # timeout on the second attempt.
        real = optimizer_module._solve_with_ipopt(
            request, config, packet, guess)
        self.assertTrue(real["success"], real)
        calls = {"count": 0}

        def first_success_then_timeout(_request, _config, _packet, _guess,
                                       wall_limit_s=None):
            calls["count"] += 1
            if calls["count"] == 1:
                return real
            return {
                "solver": "ipopt",
                "z": np.asarray(guess.vector, dtype=float).copy(),
                "objective": 0.0,
                "solver_status": "Maximum_WallTime_Exceeded",
                "success": False,
                "iterations": 2,
                "detail": "forced timeout after an audited candidate",
                "wall_time_s": 0.01,
                "soft_deadline_triggered": True,
                "deadline_trigger_stage": "ipopt_max_wall_time",
                "remaining_budget_s": 1e-9,
            }

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=first_success_then_timeout):
            result = solve_open_path(
                request, planner_config(max_initials=2))
        self.assertEqual(result.status, "GEOMETRY_FEASIBLE",
                         result.as_dict())
        self.assertIsNotNone(result.path)
        self.assertTrue(result.audit.passed)
        self.assertTrue(result.deadline_exhausted)
        self.assertEqual([attempt.status for attempt in result.attempts],
                         ["SOLVER_SUCCESS_AUDITED", "SOLVER_TIMEOUT"])
        self.assertTrue(
            result.deadline_summary["audited_candidate_preserved"])
        self.assertEqual(len(result.attempts), 2)
        self.assertIsNone(result.attempts[1].audit)
        self.assertEqual(result.attempts[1].status, "SOLVER_TIMEOUT")

    def test_auto_timeout_never_starts_slsqp_or_audits(self):
        request = request_from_fixture("E1")
        config = planner_config(solver="auto", max_initials=1)
        guess = make_initial_guesses(request, config)[0]
        slsqp_calls = []
        audit_calls = []
        original_audit = optimizer_module.audit_geometry

        def timed_out_solver(_request, _config, _packet, guess, wall_limit_s=None):
            return {
                "solver": "ipopt",
                "z": np.asarray(guess.vector, dtype=float).copy(),
                "objective": 0.0,
                "solver_status": "Maximum_WallTime_Exceeded",
                "success": False,
                "iterations": 3,
                "detail": "forced IPOPT timeout",
                "wall_time_s": 0.01,
                "soft_deadline_triggered": True,
                "deadline_trigger_stage": "ipopt_max_wall_time",
                "remaining_budget_s": 1e-9,
            }

        def forbidden_slsqp(*args, **kwargs):
            slsqp_calls.append(1)
            raise AssertionError("SLSQP must not start after an IPOPT timeout")

        def recording_audit(*args, **kwargs):
            audit_calls.append(1)
            return original_audit(*args, **kwargs)

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=timed_out_solver):
            with mock.patch.object(
                    optimizer_module, "_solve_with_scipy",
                    side_effect=forbidden_slsqp):
                with mock.patch.object(
                        optimizer_module, "audit_geometry",
                        side_effect=recording_audit):
                    result = solve_open_path(request, config)
        self.assertEqual(slsqp_calls, [])
        self.assertEqual(audit_calls, [])
        self.assertEqual(result.status, "BUDGET_EXHAUSTED",
                         result.as_dict())
        self.assertIsNone(result.path)
        self.assertIsNone(result.audit)
        self.assertTrue(result.deadline_exhausted)
        self.assertEqual(len(result.attempts), 1)
        self.assertEqual(result.attempts[0].status, "SOLVER_TIMEOUT")
        self.assertEqual(result.attempts[0].solver, "ipopt")
        self.assertTrue(result.attempts[0].soft_deadline_triggered)
        self.assertEqual(result.attempts[0].deadline_trigger_stage,
                         "ipopt_max_wall_time")
        self.assertEqual(result.attempts[0].rejection_reason,
                         "SOFT_BUDGET_EXHAUSTED")
        self.assertEqual(result.solver_used, "ipopt")
        self.assertEqual(result.deadline_summary["trigger_stage"],
                         "ipopt_max_wall_time")
        self.assertEqual(result.deadline_summary["trigger_solver_status"],
                         "Maximum_WallTime_Exceeded")
        self.assertEqual(result.deadline_summary["timeout_solver_status"],
                         "Maximum_WallTime_Exceeded")
        self.assertEqual(result.deadline_summary["timeout_iterations"], 3)

    def test_auto_timeout_after_audited_candidate_never_falls_back(self):
        request = request_from_fixture("E1")
        config = planner_config(solver="auto", max_initials=2)
        guess = make_initial_guesses(request, config)[0]
        packet = optimizer_module._search_packet(request, config)
        real = optimizer_module._solve_with_ipopt(
            request, config, packet, guess)
        self.assertTrue(real["success"], real)
        calls = {"count": 0}
        slsqp_calls = []
        audit_calls = []
        original_audit = optimizer_module.audit_geometry

        def first_success_then_timeout(_request, _config, _packet, _guess,
                                       wall_limit_s=None):
            calls["count"] += 1
            if calls["count"] == 1:
                return real
            return {
                "solver": "ipopt",
                "z": np.asarray(guess.vector, dtype=float).copy(),
                "objective": 0.0,
                "solver_status": "Maximum_WallTime_Exceeded",
                "success": False,
                "iterations": 4,
                "detail": "forced timeout after audited candidate",
                "wall_time_s": 0.01,
                "soft_deadline_triggered": True,
                "deadline_trigger_stage": "ipopt_max_wall_time",
                "remaining_budget_s": 1e-9,
            }

        def forbidden_slsqp(*args, **kwargs):
            slsqp_calls.append(1)
            raise AssertionError("SLSQP must not start after an IPOPT timeout")

        def recording_audit(*args, **kwargs):
            audit_calls.append(1)
            return original_audit(*args, **kwargs)

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=first_success_then_timeout):
            with mock.patch.object(
                    optimizer_module, "_solve_with_scipy",
                    side_effect=forbidden_slsqp):
                with mock.patch.object(
                        optimizer_module, "audit_geometry",
                        side_effect=recording_audit):
                    result = solve_open_path(request, config)
        self.assertEqual(slsqp_calls, [])
        self.assertEqual(len(audit_calls), 1)
        self.assertEqual(result.status, "GEOMETRY_FEASIBLE",
                         result.as_dict())
        self.assertIsNotNone(result.path)
        self.assertTrue(result.audit.passed)
        self.assertTrue(result.deadline_exhausted)
        self.assertEqual([attempt.status for attempt in result.attempts],
                         ["SOLVER_SUCCESS_AUDITED", "SOLVER_TIMEOUT"])
        self.assertTrue(
            result.deadline_summary["audited_candidate_preserved"])
        self.assertEqual(result.deadline_summary["trigger_stage"],
                         "ipopt_max_wall_time")
        self.assertEqual(result.deadline_summary["timeout_solver_status"],
                         "Maximum_WallTime_Exceeded")
        self.assertEqual(result.attempts[1].solver_status,
                         "Maximum_WallTime_Exceeded")

    def test_auto_exception_with_exhausted_budget_never_starts_slsqp(self):
        request = request_from_fixture("E1")
        config = planner_config(
            solver="auto", max_initials=1, max_seconds=10.0)
        slsqp_calls = []
        audit_calls = []
        # The outer solver clock advances past the whole-request budget only
        # after the primary exception has been recorded and before the
        # fallback boundary check.
        fake_time = mock.Mock()
        fake_time.monotonic.side_effect = [
            0.0,    # solve start
            0.0,    # initial boundary
            0.1,    # attempt start
            0.1,    # IPOPT wall limit argument
            0.2,    # exception wall-time/remaining budget
            10.1,   # pre-fallback boundary: no budget remains
            10.2,   # mark_deadline trigger wall
            10.3,   # result wall
            10.4,   # final boundary
            10.5,
        ]

        def failing_ipopt(*args, **kwargs):
            raise RuntimeError("forced non-timeout IPOPT exception")

        def forbidden_slsqp(*args, **kwargs):
            slsqp_calls.append(1)
            raise AssertionError("SLSQP must not start with no budget")

        def recording_audit(*args, **kwargs):
            audit_calls.append(1)
            raise AssertionError("audit must not run without a candidate")

        with mock.patch.object(optimizer_module, "time", fake_time):
            with mock.patch.object(
                    optimizer_module, "_solve_with_ipopt",
                    side_effect=failing_ipopt):
                with mock.patch.object(
                        optimizer_module, "_solve_with_scipy",
                        side_effect=forbidden_slsqp):
                    with mock.patch.object(
                            optimizer_module, "audit_geometry",
                            side_effect=recording_audit):
                        result = solve_open_path(request, config)
        self.assertEqual(slsqp_calls, [])
        self.assertEqual(audit_calls, [])
        self.assertEqual(result.status, "BUDGET_EXHAUSTED",
                         result.as_dict())
        self.assertIsNone(result.path)
        self.assertIsNone(result.audit)
        self.assertTrue(result.deadline_exhausted)
        self.assertEqual(len(result.attempts), 1)
        self.assertEqual(result.attempts[0].status, "SOLVER_FAILED")
        self.assertEqual(result.attempts[0].solver, "ipopt")
        self.assertEqual(result.attempts[0].solver_status,
                         "IPOPT_EXCEPTION")
        self.assertIn("forced non-timeout IPOPT exception",
                      result.attempts[0].detail)
        self.assertEqual(result.deadline_summary["trigger_stage"],
                         "pre_fallback_boundary")
        self.assertEqual(result.deadline_summary["trigger_solver_status"],
                         "IPOPT_EXCEPTION")
        self.assertEqual(
            result.deadline_summary["pre_fallback_solver_status"],
            "IPOPT_EXCEPTION")
        self.assertNotIn("timeout_solver_status", result.deadline_summary)

    def test_auto_ordinary_failure_with_budget_uses_real_slsqp_and_audit(self):
        request = request_from_fixture("E1")
        config = planner_config(solver="auto", max_initials=1)
        audit_calls = []
        original_audit = optimizer_module.audit_geometry

        def failed_ipopt(_request, _config, _packet, _guess,
                         wall_limit_s=None):
            return {
                "solver": "ipopt",
                "z": None,
                "objective": None,
                "solver_status": "Solve_Failed",
                "success": False,
                "iterations": 2,
                "detail": "forced ordinary IPOPT failure",
                "wall_time_s": 0.01,
                "soft_deadline_triggered": False,
                "deadline_trigger_stage": "",
                "remaining_budget_s": 29.99,
            }

        def recording_audit(*args, **kwargs):
            audit_calls.append(1)
            return original_audit(*args, **kwargs)

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=failed_ipopt):
            with mock.patch.object(
                    optimizer_module, "audit_geometry",
                    side_effect=recording_audit):
                result = solve_open_path(request, config)
        self.assertEqual(len(audit_calls), 1)
        self.assertEqual(result.status, "GEOMETRY_FEASIBLE",
                         result.as_dict())
        self.assertTrue(result.audit.passed)
        self.assertEqual(result.solver_used, "scipy_slsqp")
        self.assertEqual(len(result.attempts), 2)
        self.assertEqual(result.attempts[0].status, "SOLVER_FAILED")
        self.assertEqual(result.attempts[0].solver, "ipopt")
        self.assertEqual(result.attempts[0].solver_status, "Solve_Failed")
        self.assertIn("forced ordinary IPOPT failure",
                      result.attempts[0].detail)
        self.assertEqual(result.attempts[1].status,
                         "SOLVER_SUCCESS_AUDITED")
        self.assertEqual(result.attempts[1].solver, "scipy_slsqp")
        self.assertIn("IPOPT Solve_Failed; SLSQP fallback used",
                      result.attempts[1].detail)
        self.assertFalse(result.attempts[1].audit is None)
        self.assertTrue(result.attempts[1].audit.passed)

    def test_auto_slsqp_timeout_is_recorded_without_audit(self):
        request = request_from_fixture("E1")
        config = planner_config(solver="auto", max_initials=1)
        guess = make_initial_guesses(request, config)[0]
        packet = optimizer_module._search_packet(request, config)
        # Obtain a genuine SLSQP callback timeout and feed it through the auto
        # fallback branch; its returned iterate must still never be audited.
        real_timeout = optimizer_module._solve_with_scipy(
            request, config, packet, guess, wall_limit_s=1e-9)
        self.assertEqual(real_timeout["solver_status"],
                         "SLSQP_SOFT_DEADLINE")
        self.assertTrue(real_timeout["soft_deadline_triggered"])
        audit_calls = []
        original_audit = optimizer_module.audit_geometry

        def failed_ipopt(_request, _config, _packet, _guess,
                         wall_limit_s=None):
            return {
                "solver": "ipopt",
                "z": None,
                "objective": None,
                "solver_status": "Solve_Failed",
                "success": False,
                "iterations": 1,
                "detail": "forced ordinary IPOPT failure before timeout",
                "wall_time_s": 0.01,
                "soft_deadline_triggered": False,
                "deadline_trigger_stage": "",
                "remaining_budget_s": 29.99,
            }

        def timeout_fallback(*args, **kwargs):
            return real_timeout

        def recording_audit(*args, **kwargs):
            audit_calls.append(1)
            return original_audit(*args, **kwargs)

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=failed_ipopt):
            with mock.patch.object(
                    optimizer_module, "_solve_with_scipy",
                    side_effect=timeout_fallback):
                with mock.patch.object(
                        optimizer_module, "audit_geometry",
                        side_effect=recording_audit):
                    result = solve_open_path(request, config)
        self.assertEqual(audit_calls, [])
        self.assertEqual(result.status, "BUDGET_EXHAUSTED",
                         result.as_dict())
        self.assertIsNone(result.path)
        self.assertIsNone(result.audit)
        self.assertTrue(result.deadline_exhausted)
        self.assertEqual([attempt.status for attempt in result.attempts],
                         ["SOLVER_FAILED", "SOLVER_TIMEOUT"])
        self.assertEqual(result.attempts[0].solver_status, "Solve_Failed")
        self.assertEqual(result.attempts[1].solver_status,
                         "SLSQP_SOFT_DEADLINE")
        self.assertEqual(result.attempts[1].deadline_trigger_stage,
                         "slsqp_iteration_callback")
        self.assertIsNone(result.attempts[1].audit)
        self.assertTrue(result.attempts[1].soft_deadline_triggered)
        self.assertEqual(result.solver_used, "scipy_slsqp")
        self.assertEqual(result.deadline_summary["trigger_stage"],
                         "slsqp_iteration_callback")
        self.assertEqual(result.deadline_summary["timeout_solver_status"],
                         "SLSQP_SOFT_DEADLINE")
        self.assertGreaterEqual(
            result.deadline_summary["timeout_iterations"], 1)

    def test_nan_planner_tolerance_cannot_enter_the_audit_conversion(self):
        request = request_from_fixture("E1")
        bad = planner_config(position_tolerance_m=float("nan"))
        result = solve_open_path(request, bad)
        self.assertEqual(result.status, "INVALID_REQUEST")
        self.assertEqual(result.reason, "INVALID_CONFIG")
        self.assertIsNone(result.path)
        self.assertIsNone(result.audit)
        # Defence in depth: even if PlannerConfig validation were bypassed,
        # the solver-to-audit conversion rejects the non-finite tolerance.
        bypassed = planner_config()
        object.__setattr__(bypassed, "position_tolerance_m", float("nan"))
        with self.assertRaises(VirtualAContractError) as raised:
            optimizer_module._audit_config(bypassed)
        self.assertEqual(raised.exception.reason, "INVALID_AUDIT_CONFIG")
        self.assertEqual(raised.exception.location["field"],
                         "position_tolerance_m")


class HeadingBranchPacketTests(unittest.TestCase):
    """Step 3: one packet feeds initials, the solver and attempt metadata."""

    def test_one_packet_reaches_initials_solver_and_attempt_metadata(self):
        request = request_from_fixture("STRAIGHT_LINE")
        config = planner_config(max_initials=2, max_seconds=2.0)
        branch = HeadingBranch(
            base_heading_rad=0.0,
            winding_k=1,
            unwrapped_heading_rad=2.0 * math.pi,
            guide_turn_rad=2.0 * math.pi,
            source="test_loop",
            candidate_rank=0)
        packets = []
        solver_packets = []
        real_search_packet = optimizer_module._search_packet

        def recording_search_packet(*args, **kwargs):
            packet = real_search_packet(*args, **kwargs)
            packets.append(packet)
            return packet

        def fake_failure(_request, _config, packet, guess,
                         wall_limit_s=None, **kwargs):
            solver_packets.append((id(packet), initial_guess_key(guess)))
            return {
                "solver": "ipopt",
                "z": None,
                "objective": None,
                "solver_status": "Maximum_Iterations_Exceeded",
                "success": False,
                "iterations": 1,
                "detail": "fake failure",
                "wall_time_s": 0.0,
            }

        with mock.patch.object(
                optimizer_module, "_search_packet",
                side_effect=recording_search_packet), \
                mock.patch.object(
                    optimizer_module, "_solve_with_ipopt",
                    side_effect=fake_failure):
            result = solve_open_path_guided(
                request, config, heading_branch=branch,
                guide_length_m=6.0)
        self.assertEqual(len(packets), 1)
        packet = packets[0]
        self.assertEqual(packet["heading_change_rad"], 2.0 * math.pi)
        self.assertEqual(packet["heading_branch"], branch.as_dict())
        self.assertEqual(packet["guide_length_m"], 6.0)
        self.assertEqual(result.goal_heading_branch_rad, 2.0 * math.pi)
        self.assertTrue(solver_packets)
        for packet_id, key in solver_packets:
            self.assertEqual(packet_id, id(packet))
            self.assertEqual(key[-1], 1)
        self.assertTrue(result.attempts)
        for attempt in result.attempts:
            self.assertEqual(attempt.winding_k, 1)
            self.assertEqual(attempt.heading_base_rad, 0.0)
            self.assertEqual(attempt.guide_turn_rad, 2.0 * math.pi)
        self.assertEqual(
            result.goal_heading_branch_rad,
            result.attempts[0].initial.metadata["heading_branch"][
                "unwrapped_heading_rad"])

    def test_route_initial_identity_changes_with_the_winding_branch(self):
        request = request_from_fixture("STRAIGHT_LINE")
        config = planner_config(max_initials=2)
        guide = optimizer_module.GuideRoute(
            points=((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0),
                    (0.0, 0.0)),
            source="fixture_loop",
            grid_sha256="0" * 64)
        preferred = HeadingBranch(
            base_heading_rad=0.0, winding_k=1,
            unwrapped_heading_rad=2.0 * math.pi,
            guide_turn_rad=2.0 * math.pi,
            source="fixture_loop", candidate_rank=0)
        neighbour = HeadingBranch(
            base_heading_rad=0.0, winding_k=0,
            unwrapped_heading_rad=0.0,
            guide_turn_rad=2.0 * math.pi,
            source="fixture_loop", candidate_rank=1)
        first = make_initial_guesses(
            request, config, route_guides=(guide,),
            heading_branch=preferred, guide_length_m=guide.length_m)
        second = make_initial_guesses(
            request, config, route_guides=(guide,),
            heading_branch=neighbour, guide_length_m=guide.length_m)
        self.assertEqual(first[0].source, "map_route_guide")
        self.assertEqual(second[0].source, "map_route_guide")
        self.assertNotEqual(
            initial_guess_key(first[0]), initial_guess_key(second[0]))
        self.assertEqual(
            first[0].metadata["heading_branch"], preferred.as_dict())
        self.assertEqual(
            second[0].metadata["heading_branch"], neighbour.as_dict())


class SDomainPacketTests(unittest.TestCase):
    """Step 4: one packet records the finite S domain and its solved S."""

    def _packet(self, request, config, **kwargs):
        return optimizer_module._search_packet(request, config, **kwargs)

    def test_finite_s_domain_formula_without_guide(self):
        request = request_from_fixture("STRAIGHT_LINE")
        config = planner_config()
        packet = self._packet(request, config)
        kappa_ref = min(
            float(config.kappa_limit),
            1.0 / float(request.limits.r_min_ackermann))
        lower = max(
            int(config.intervals) * 0.05, 6.0,
            abs(packet["heading_change_rad"]) / kappa_ref)
        upper = max(lower + 2.0, 12.0, 3.0 * 6.0 + 10.0)
        self.assertEqual(packet["kappa_ref_max"], kappa_ref)
        self.assertEqual(packet["distance"], 6.0)
        self.assertIsNone(packet["guide_length_m"])
        self.assertEqual(packet["S_lower"], lower)
        self.assertEqual(packet["S_upper"], upper)
        self.assertEqual(packet["S_upper_active_tol_m"], 1e-6)
        self.assertIsNone(packet["S_solution"])
        self.assertIsNone(packet["upper_active"])

    def test_guide_length_raises_the_finite_upper_bound_with_approved_factor(self):
        request = request_from_fixture("STRAIGHT_LINE")
        config = planner_config()
        plain = self._packet(request, config)
        guided = self._packet(request, config, guide_length_m=100.0)
        self.assertEqual(guided["S_lower"], plain["S_lower"])
        self.assertEqual(guided["guide_length_m"], 100.0)
        self.assertAlmostEqual(guided["S_upper"], 115.0, places=12)
        self.assertGreater(guided["S_upper"], plain["S_upper"])
        # The guide polyline only raises the search upper bound; it is never
        # the lower bound and never certifies feasibility.
        self.assertEqual(guided["S_lower"], guided["distance"])
        for guide_length in (0.0, 1.0, 50.0, 100.0):
            packet = self._packet(
                request, config, guide_length_m=guide_length)
            self.assertEqual(
                packet["S_upper"],
                max(packet["S_lower"] + 2.0, 12.0, 28.0,
                    1.15 * guide_length))

    def test_fifty_and_hundred_metre_straight_lines_are_not_capped_at_sixty(self):
        config = planner_config()
        for distance, expected_upper in ((50.0, 160.0), (100.0, 310.0)):
            request = request_from_fixture(
                "STRAIGHT_LINE",
                goal_qf=FormationPose(distance, 0.0, 0.0))
            packet = self._packet(request, config)
            self.assertEqual(packet["distance"], distance)
            self.assertEqual(packet["S_lower"], distance)
            self.assertEqual(packet["S_upper"], expected_upper)
            self.assertGreater(packet["S_upper"], 60.0)
            guesses = make_initial_guesses(request, config, packet=packet)
            for guess in guesses:
                self.assertGreaterEqual(guess.S, packet["S_lower"])
                self.assertLessEqual(guess.S, packet["S_upper"])

    def test_attempt_records_solution_and_tolerance_based_upper_activity(self):
        metadata = {
            "endpoint_distance_m": 6.0,
            "guide_length_m": 12.0,
            "S_lower_m": 6.0,
            "S_upper_m": 13.8,
            "S_upper_active_tol_m": 1e-6,
        }
        guess = SimpleNamespace(
            metadata=metadata, source="test", label="test")
        tolerance = metadata["S_upper_active_tol_m"]
        for offset, expected in (
                (0.0, True), (-tolerance, True),
                (-2.0 * tolerance, False), (-1.0, False)):
            result = {
                "solver": "ipopt", "z": np.array([13.8 + offset]),
                "objective": 0.0, "solver_status": "Solve_Succeeded",
                "success": True, "iterations": 1, "detail": "",
                "wall_time_s": 0.0,
            }
            attempt = optimizer_module._attempt_record(
                0, guess, result, None, "", "SOLVER_SUCCESS_AUDITED")
            self.assertEqual(attempt.distance_m, 6.0)
            self.assertEqual(attempt.guide_length_m, 12.0)
            self.assertEqual(attempt.s_lower_m, 6.0)
            self.assertEqual(attempt.s_upper_m, 13.8)
            self.assertEqual(attempt.s_solution_m, 13.8 + offset)
            self.assertIs(attempt.s_upper_active, expected)
        failure = {
            "solver": "ipopt", "z": np.array([13.8]),
            "objective": None, "solver_status": "Maximum_Iterations_Exceeded",
            "success": False, "iterations": 1, "detail": "",
            "wall_time_s": 0.0,
        }
        attempt = optimizer_module._attempt_record(
            0, guess, failure, None, "Maximum_Iterations_Exceeded",
            "SOLVER_FAILED")
        self.assertIsNone(attempt.s_solution_m)
        self.assertIsNone(attempt.s_upper_active)

    def test_pathological_domains_fail_closed(self):
        request = request_from_fixture("STRAIGHT_LINE")
        with self.assertRaises(VirtualAContractError) as caught:
            self._packet(request, planner_config(kappa_limit=0.0))
        self.assertEqual(caught.exception.reason, "INVALID_SEARCH_DOMAIN")
        with self.assertRaises(VirtualAContractError) as caught:
            self._packet(
                request, planner_config(), guide_length_m=float("nan"))
        self.assertEqual(caught.exception.reason, "INVALID_GUIDE_LENGTH")
        huge = request_from_fixture(
            "STRAIGHT_LINE",
            goal_qf=FormationPose(1e308, 0.0, 0.0))
        with self.assertRaises(VirtualAContractError) as caught:
            self._packet(huge, planner_config())
        self.assertEqual(caught.exception.reason, "INVALID_SEARCH_DOMAIN")
        for bad_length in (-1.0, float("inf"), float("nan")):
            result = solve_open_path_guided(
                request, planner_config(max_seconds=2.0),
                guide_length_m=bad_length)
            self.assertEqual(result.status, "INVALID_REQUEST")
            self.assertEqual(result.reason, "INVALID_GUIDE_LENGTH")
            self.assertIsNone(result.path)

    def test_packet_and_attempt_agree_on_the_chosen_solution(self):
        request = request_from_fixture("STRAIGHT_LINE")
        config = planner_config(max_initials=1, max_seconds=2.0)
        packets = []
        real_search_packet = optimizer_module._search_packet

        def recording_search_packet(*args, **kwargs):
            packet = real_search_packet(*args, **kwargs)
            packets.append(packet)
            return packet

        def fake_success(_request, _config, packet, guess,
                         wall_limit_s=None, **kwargs):
            return {
                "solver": "ipopt",
                "z": np.asarray(guess.vector, dtype=float).copy(),
                "objective": 0.1,
                "solver_status": "Solve_Succeeded",
                "success": True,
                "iterations": 1,
                "detail": "fake success",
                "wall_time_s": 0.0,
            }

        audit = SimpleNamespace(
            passed=True, reason="", detail="", check_count=0,
            first_failure=None, metrics={}, scope="geometry")
        with mock.patch.object(
                optimizer_module, "_search_packet",
                side_effect=recording_search_packet), \
                mock.patch.object(
                    optimizer_module, "_solve_with_ipopt",
                    side_effect=fake_success), \
                mock.patch.object(
                    optimizer_module, "audit_geometry", return_value=audit):
            result = solve_open_path_guided(request, config)
        self.assertEqual(result.status, "GEOMETRY_FEASIBLE")
        self.assertEqual(len(packets), 1)
        packet = packets[0]
        self.assertEqual(packet["S_solution"], result.path.length_m)
        audited = [
            attempt for attempt in result.attempts
            if attempt.status == "SOLVER_SUCCESS_AUDITED"]
        self.assertEqual(len(audited), 1)
        self.assertEqual(audited[0].s_solution_m, result.path.length_m)
        self.assertIs(
            packet["upper_active"], audited[0].s_upper_active)
        self.assertEqual(
            packet["upper_active"],
            abs(packet["S_solution"] - packet["S_upper"]) <= 1e-6)


if __name__ == "__main__":
    unittest.main()