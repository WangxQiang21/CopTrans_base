"""Independent G2 geometry audit tests.

These tests deliberately build reference paths in the test process.  The
production audit is exercised by ``audit_geometry``; no optimizer residual
function is imported or used.
"""
import json
import math
from pathlib import Path
import unittest
from unittest import mock

from fwis_v3_optimizer.continuous_geometry import ControlKnot
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    DynamicAuditConfig,
    GeometryAuditConfig,
    GoalPolicy,
    InitialAssumption,
    TimingConfig,
    VirtualARequest,
    request_fingerprint,
)
from fwis_v3_optimizer.formation_continuous_a.geometry import (
    C2CurvatureProfile,
    VirtualAPath,
)
from fwis_v3_optimizer.formation_continuous_a.audit import (
    audit_dynamics,
    audit_geometry,
)
from fwis_v3_optimizer.formation_continuous_a import audit as audit_module
from fwis_v3_optimizer.formation_continuous_a import bounds as bounds_module
from fwis_v3_optimizer.formation_continuous_a.timing import retime_path
from fwis_v3_optimizer.formation_continuous_a.trajectory import (
    VirtualATimedPlan,
)

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


def limits_from_fixture(**changes):
    values = dict(BASE["limits"])
    values.update(changes)
    return FormationTimingLimits(**values)


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
        "limits": limits_from_fixture(),
    }
    values.update(changes)
    return VirtualARequest(**values)


def profile_from_knots(records):
    return C2CurvatureProfile.from_knots([
        ControlKnot(float(s), float(k), float(eta)) for s, k, eta in records])


def bind(path, request):
    path.request_sha256 = request_fingerprint(request)
    return path


def path_for_records(records, request=None):
    profile = profile_from_knots(records)
    path = VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile,
                        model_from_fixture())
    if request is None:
        endpoint = path.endpoint()
        request = request_from_fixture(
            "STRAIGHT_LINE",
            goal_qf=FormationPose(endpoint.x, endpoint.y, endpoint.psi))
    bind(path, request)
    return request, path


def quadratic_reference_profile(reference):
    S = float(reference["S"])
    alpha = float(reference["alpha"])
    count = 8
    records = []
    for index in range(count + 1):
        s = S * index / count
        records.append((s, alpha * (s / S) ** 2,
                        2.0 * alpha * s / (S * S)))
    return profile_from_knots(records)


def straight_spline_profile():
    record = REFERENCES["straight_spline"]
    return profile_from_knots(zip(
        record["stations_m"], record["curvature_1_per_m"],
        record["slope_1_per_m2"]))


def nonzero_g_multisegment_profile():
    """A legal C2 cubic ``k(s)=b*s^2+c*s^3`` sampled at four segments.

    Its third curvature derivative is the nonzero constant ``6*c``.  The
    fourth derivative of the position integrand is therefore nonzero, which
    exercises the Simpson ``M4`` remainder instead of a straight line.
    """
    b, c = 0.012, -0.0015
    stations = (0.0, 1.1, 2.4, 3.9, 5.2)
    records = [
        (s, b * s * s + c * s ** 3, 2.0 * b * s + 3.0 * c * s * s)
        for s in stations]
    return profile_from_knots(records)


def high_precision_endpoint(path):
    """Independent adaptive quadrature of the exact phase polynomial."""
    from scipy.integrate import quad
    stations = [0.0] + [piece["s0"] for piece in
                        path.profile.as_dict()["pieces"][1:]]
    stations.append(path.length_m)
    psi = path.profile.psi_at
    kwargs = dict(epsabs=1e-13, epsrel=1e-13, limit=400)
    x = quad(lambda s: math.cos(psi(s)), 0.0, path.length_m,
             points=stations, **kwargs)[0]
    y = quad(lambda s: math.sin(psi(s)), 0.0, path.length_m,
             points=stations, **kwargs)[0]
    return x, y, psi(path.length_m)


class ReferenceAuditTests(unittest.TestCase):

    def test_frozen_reference_endpoints_match_rounded_task_requests(self):
        for name, reference in REFERENCES["quadratic_profiles"].items():
            goal = REQUESTS["requests"][name]["goal_qf"]
            exact = reference["exact_endpoint"]
            self.assertLess(math.hypot(exact["x"] - goal["x"],
                                       exact["y"] - goal["y"]), 1e-9)
            self.assertLess(abs(exact["psi"] - goal["psi"]), 1e-12)

    def test_six_quadratic_existence_profiles_pass_independent_audit(self):
        for name in ("E1", "E2", "E3", "E4", "E5", "E6"):
            with self.subTest(endpoint=name):
                request = request_from_fixture(name)
                reference = REFERENCES["quadratic_profiles"][name]
                profile = quadratic_reference_profile(reference)
                path = bind(VirtualAPath(
                    FormationPose(0.0, 0.0, 0.0), profile,
                    model_from_fixture()), request)
                report = audit_geometry(request, path)
                self.assertTrue(report.passed, report.as_dict())
                self.assertEqual(report.scope, "GEOMETRY")
                self.assertIsNone(report.first_failure)
                self.assertGreater(report.check_count, 10)
                self.assertEqual(len(report.whole_segment_bounds),
                                 profile.segment_count)
                exact = reference["exact_endpoint"]
                endpoint = path.endpoint()
                self.assertLess(math.hypot(endpoint.x - exact["x"],
                                           endpoint.y - exact["y"]), 5e-3)
                self.assertLess(abs(endpoint.psi - exact["psi"]), 5e-3)
                for vehicle in report.whole_segment_bounds:
                    for value in vehicle["max_abs_C_by_vehicle"].values():
                        self.assertLessEqual(
                            value, 1.0 / request.limits.r_min_ackermann)
                self.assertLessEqual(
                    report.integration["position_budget_m"], 1e-7)
                self.assertLessEqual(
                    report.integration["angle_budget_rad"], 1e-9)

    def test_straight_line_goal_passes(self):
        request = request_from_fixture(
            "STRAIGHT_LINE",
            goal_qf=FormationPose(6.0, 0.0, 0.0))
        _, path = path_for_records([(0.0, 0.0, 0.0), (6.0, 0.0, 0.0)],
                                   request)
        report = audit_geometry(request, path)
        self.assertTrue(report.passed, report.as_dict())
        self.assertEqual(report.c2["max_abs_g"], 0.0)

    def test_clamped_cubic_spline_straight_reference_passes(self):
        profile = straight_spline_profile()
        request = request_from_fixture("STRAIGHT_POLICY")
        path = bind(VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile,
                                 model_from_fixture()), request)
        # The independently integrated reference endpoint is the frozen goal.
        exact = REFERENCES["straight_spline"]["exact_endpoint"]
        self.assertLess(math.hypot(path.endpoint().x - exact["x"],
                                   path.endpoint().y - exact["y"]), 5e-3)
        report = audit_geometry(request, path)
        self.assertTrue(report.passed, report.as_dict())
        self.assertEqual(len(report.whole_segment_bounds), 6)

    def test_g1_double_lane_change_endpoint_can_be_a_straight_path(self):
        length = 11.6445099698
        request = request_from_fixture(
            "STRAIGHT_LINE",
            goal_qf=FormationPose(length, 0.0, 0.0))
        _, path = path_for_records(
            [(0.0, 0.0, 0.0), (length, 0.0, 0.0)], request)
        report = audit_geometry(request, path)
        self.assertTrue(report.passed, report.as_dict())
        self.assertLessEqual(report.integration["position_budget_m"], 1e-7)

    def test_hold_only_for_exactly_identical_start_and_goal(self):
        request = request_from_fixture(
            "STRAIGHT_LINE", goal_qf=FormationPose(0.0, 0.0, 0.0))
        path = bind(VirtualAPath.hold(FormationPose(0.0, 0.0, 0.0),
                                      model_from_fixture()), request)
        report = audit_geometry(request, path)
        self.assertTrue(report.passed, report.as_dict())
        self.assertTrue(report.metrics["hold"])
        for goal in ((1e-9, 0.0, 0.0), (0.0, 0.0, 1e-9)):
            near = request_from_fixture(
                "STRAIGHT_LINE", goal_qf=FormationPose(*goal))
            near_path = bind(VirtualAPath.hold(
                FormationPose(0.0, 0.0, 0.0), model_from_fixture()), near)
            rejected = audit_geometry(near, near_path)
            self.assertFalse(rejected.passed)
            self.assertEqual(rejected.reason, "ENDPOINT_MISMATCH")
            self.assertEqual(rejected.location["constraint"], "HOLD_GOAL")

    def test_structural_model_identity_is_not_object_identity(self):
        request = request_from_fixture("E1")
        reference = REFERENCES["quadratic_profiles"]["E1"]
        profile = quadratic_reference_profile(reference)
        independent_model = FormationModel([
            VehicleOffset(1, 1.0, 0.0), VehicleOffset(2, 0.0, 1.0),
            VehicleOffset(3, 0.0, -1.0)])
        self.assertIsNot(independent_model, request.model)
        path = bind(VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile,
                                 independent_model), request)
        report = audit_geometry(request, path)
        self.assertTrue(report.passed, report.as_dict())

    def test_planning_context_keyword_is_accepted_without_map_claim(self):
        request = request_from_fixture("E1")
        reference = REFERENCES["quadratic_profiles"]["E1"]
        path = bind(VirtualAPath(
            FormationPose(0.0, 0.0, 0.0), quadratic_reference_profile(reference),
            model_from_fixture()), request)
        report = audit_geometry(
            request, path, planning_context={"context_id": "G2-test"})
        self.assertTrue(report.passed, report.as_dict())
        self.assertTrue(report.metrics["planning_context_provided"])
        self.assertEqual(report.scope, "GEOMETRY")
        self.assertNotIn("EXECUTABLE", json.dumps(report.as_dict()))


class PolicySemanticsTests(unittest.TestCase):

    def test_terminal_nonzero_curvature_free_passes_straight_rejected(self):
        base_request, base_path = path_for_records(
            [(0.0, 0.0, 0.0), (6.0, 0.1, 0.0)])
        free_report = audit_geometry(base_request, base_path)
        self.assertTrue(free_report.passed, free_report.as_dict())
        straight_request = request_from_fixture(
            "STRAIGHT_LINE", goal_qf=base_request.goal_qf,
            goal_policy=GoalPolicy.STRAIGHT)
        bind(base_path, straight_request)
        rejected = audit_geometry(straight_request, base_path)
        self.assertFalse(rejected.passed)
        self.assertEqual(rejected.reason, "GOAL_POLICY_VIOLATION")
        self.assertEqual(rejected.location["constraint"], "STRAIGHT_K")

    def test_terminal_nonzero_eta_free_passes_straight_rejected(self):
        request, path = path_for_records(
            [(0.0, 0.0, 0.0), (6.0, 0.0, 0.1)])
        free_report = audit_geometry(request, path)
        self.assertTrue(free_report.passed, free_report.as_dict())
        straight = request_from_fixture(
            "STRAIGHT_LINE", goal_qf=request.goal_qf,
            goal_policy=GoalPolicy.STRAIGHT)
        bind(path, straight)
        rejected = audit_geometry(straight, path)
        self.assertFalse(rejected.passed)
        self.assertEqual(rejected.reason, "GOAL_POLICY_VIOLATION")
        self.assertEqual(rejected.location["constraint"], "STRAIGHT_ETA")


class NegativeAuditTests(unittest.TestCase):

    def test_non_c2_profile_rejected(self):
        request, path = path_for_records(
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)])
        profile = path.profile
        object.__setattr__(profile, "_knots", (
            ControlKnot(0.0, 0.0, 0.0),
            ControlKnot(1.0, 0.0, 0.1),
            ControlKnot(2.0, 0.0, 0.0),
        ))
        report = audit_geometry(request, path)
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "NON_C2_PROFILE")
        self.assertEqual(report.location["constraint"], "C2_SECOND")

    def test_car1_wrong_initial_heading_rejected(self):
        request, path = path_for_records(
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
        bad = request_from_fixture(
            "STRAIGHT_LINE", goal_qf=request.goal_qf,
            start_headings={1: 0.1, 2: 0.0, 3: 0.0})
        report = audit_geometry(bad, path)
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "INITIAL_HEADING_MISMATCH")
        self.assertEqual(report.location["vehicle_id"], 1)

    def test_forged_request_digest_rejected(self):
        request, path = path_for_records(
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
        path.request_sha256 = "0" * 64
        report = audit_geometry(request, path)
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "FORGED_REQUEST_DIGEST")
        self.assertEqual(report.location["constraint"], "REQUEST_DIGEST")

    def test_forged_goal_is_checked_against_original_request(self):
        request, path = path_for_records(
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
        forged = request_from_fixture(
            "STRAIGHT_LINE", goal_qf=FormationPose(1.0, 0.2, 0.1))
        path.request_sha256 = None
        report = audit_geometry(forged, path)
        self.assertFalse(report.passed)
        self.assertIn(report.reason, (
            "ENDPOINT_POSITION_ERROR", "ENDPOINT_HEADING_ERROR"))
        self.assertIn(report.location["constraint"],
                      ("ENDPOINT_POSITION", "ENDPOINT_HEADING"))

    def test_wrong_vehicle_shape_rejected(self):
        request, path = path_for_records(
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
        shapes = shapes_from_fixture()
        shapes[1] = VehicleShape(0.74, 0.50)
        bad = request_from_fixture(
            "STRAIGHT_LINE", goal_qf=request.goal_qf,
            vehicle_shapes=shapes)
        report = audit_geometry(bad, path)
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "VEHICLE_SHAPE_MISMATCH")
        self.assertEqual(report.location["vehicle_id"], 1)

    def test_invalid_limit_rejected(self):
        request, path = path_for_records(
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
        limits = limits_from_fixture()
        object.__setattr__(limits, "r_min_ackermann", 0.0)
        bad = request_from_fixture(
            "STRAIGHT_LINE", goal_qf=request.goal_qf, limits=limits)
        report = audit_geometry(bad, path)
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "INVALID_LIMITS")
        self.assertIn("r_min_ackermann", report.detail)

    def test_static_wheel_limit_uses_whole_segment_envelope(self):
        request, path = path_for_records(
            [(0.0, 0.0, 0.0), (6.0, 0.5, 0.0)])
        strict_limits = limits_from_fixture(steer_angle_max=0.1)
        bad = request_from_fixture(
            "STRAIGHT_LINE", goal_qf=request.goal_qf, limits=strict_limits)
        bind(path, bad)
        report = audit_geometry(bad, path)
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "STATIC_WHEEL_LIMIT")
        self.assertEqual(report.location["constraint"], "STATIC_WHEEL_LIMIT")
        # The checked quantity must be the project's ideal-wheel geometry:
        # xw=+/-wheelbase/2, yw=+/-track/2, delta=atan2(xw*C, 1-yw*C).
        self.assertGreater(report.location["value"],
                           strict_limits.steer_angle_max)
        vehicle_id = report.location["vehicle_id"]
        s = report.location["s"]
        curvature = path.state_at_s(s).vehicle(vehicle_id).curvature
        expected = max(
            abs(math.atan2(
                xw * curvature, 1.0 - yw * curvature))
            for xw in (strict_limits.wheelbase / 2.0,
                       -strict_limits.wheelbase / 2.0)
            for yw in (strict_limits.track / 2.0,
                       -strict_limits.track / 2.0))
        self.assertAlmostEqual(report.location["value"], expected, places=9)

    def test_nan_goal_rejected_before_any_geometry_claim(self):
        request, path = path_for_records(
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
        object.__setattr__(request.goal_qf, "x", float("nan"))
        report = audit_geometry(request, path)
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "INVALID_REQUEST")

    def test_segment_interior_curvature_violation_with_legal_nodes(self):
        # One Hermite segment h=0.1, k(0)=k(0.1)=0, eta(0)=0, eta(0.1)=10.
        # Endpoint nodes are legal; the interior curvature is far outside the
        # static limit.  The audit must reject it from the whole segment
        # envelope, not merely from sampled nodes.
        request, path = path_for_records(
            [(0.0, 0.0, 0.0), (0.1, 0.0, 10.0)])
        self.assertEqual(path.profile.knots[0].value, 0.0)
        self.assertEqual(path.profile.knots[1].value, 0.0)
        report = audit_geometry(request, path)
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "CURVATURE_LIMIT")
        self.assertEqual(report.location["constraint"], "CURVATURE_LIMIT")
        self.assertGreater(report.location["s"], 0.0)
        self.assertLess(report.location["s"], 0.1)
        self.assertGreater(abs(report.location["value"]), 1.0 / 0.810)

    def test_unconfirmed_enclosure_is_not_published_as_feasible(self):
        request, path = path_for_records(
            [(0.0, 0.0, 0.0), (0.1, 0.0, 10.0)])
        report = audit_geometry(
            request, path, GeometryAuditConfig(max_depth=0))
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "CURVATURE_LIMIT")
        report = audit_geometry(
            request, path, GeometryAuditConfig(max_depth=20, max_leaves=1))
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "AUDIT_BUDGET_EXHAUSTED")


class AuditConfigFailureClosedTests(unittest.TestCase):

    def _valid_case(self):
        return path_for_records(
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])

    def test_every_integer_field_rejects_bool_float_zero_and_negative(self):
        cases = {
            "max_depth": (True, 2.0, -1),
            "max_leaves": (False, 10.0, 0),
            "sample_points_per_segment": (True, 9.0, 1),
            "audit_max_subintervals_per_segment": (True, 4.0, 1),
            "audit_max_total_subintervals": (False, 8.0, 0),
        }
        request, path = self._valid_case()
        for field, values in cases.items():
            for value in values:
                with self.subTest(field=field, value=value):
                    report = audit_geometry(
                        request, path, GeometryAuditConfig(**{field: value}))
                    self.assertFalse(report.passed)
                    self.assertEqual(report.reason, "INVALID_AUDIT_CONFIG")
                    self.assertEqual(report.location["field"], field)

    def test_every_float_field_rejects_nan_infinity_negative_and_types(self):
        nonnegative = {
            "c2_value_tol", "c2_slope_tol", "c2_second_tol",
            "c2_third_abs_limit",
        }
        positive = {
            "position_tolerance_m", "heading_tolerance_rad",
            "rigidity_tolerance_m", "integration_budget_m",
            "angle_budget_rad", "interval_curvature_tolerance",
            "audit_panel_width_m", "audit_position_budget_m",
        }
        request, path = self._valid_case()
        for field in sorted(nonnegative | positive):
            values = [float("nan"), float("inf"), float("-inf"), -1.0,
                      "0.1", True]
            if field in positive:
                values.append(0.0)
            for value in values:
                with self.subTest(field=field, value=value):
                    report = audit_geometry(
                        request, path, GeometryAuditConfig(**{field: value}))
                    self.assertFalse(report.passed)
                    self.assertEqual(report.reason, "INVALID_AUDIT_CONFIG")
                    self.assertEqual(report.location["field"], field)

    def test_audit_budget_cannot_exceed_g1_contract(self):
        request, path = self._valid_case()
        report = audit_geometry(
            request, path, GeometryAuditConfig(
                integration_budget_m=1e-9,
                audit_position_budget_m=1e-8))
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "INVALID_AUDIT_CONFIG")
        self.assertEqual(report.location["field"], "audit_position_budget_m")

    def test_zero_nonnegative_tolerances_are_a_valid_domain(self):
        request, path = self._valid_case()
        report = audit_geometry(
            request, path, GeometryAuditConfig(
                c2_value_tol=0.0, c2_slope_tol=0.0, c2_second_tol=0.0,
                c2_third_abs_limit=0.0))
        self.assertTrue(report.passed, report.as_dict())

    def test_invalid_config_is_rejected_before_the_hold_branch(self):
        request = request_from_fixture(
            "STRAIGHT_LINE", goal_qf=FormationPose(0.0, 0.0, 0.0))
        path = bind(VirtualAPath.hold(
            FormationPose(0.0, 0.0, 0.0), model_from_fixture()), request)
        report = audit_geometry(
            request, path,
            GeometryAuditConfig(rigidity_tolerance_m=float("nan")))
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "INVALID_AUDIT_CONFIG")
        self.assertFalse(report.metrics.get("hold", False))

    def test_nonfinite_intermediate_audit_value_fails_closed(self):
        request, path = self._valid_case()
        with mock.patch.object(
                audit_module, "_integrate_xy",
                return_value=(float("nan"), 0.0)):
            report = audit_geometry(request, path)
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "NONFINITE_AUDIT_VALUE")
        self.assertEqual(report.location["constraint"], "NONFINITE_AUDIT_VALUE")


class RealCounterexampleTests(unittest.TestCase):

    def test_six_metre_path_to_ten_metre_goal_nan_tolerance_is_rejected(self):
        request = request_from_fixture(
            "STRAIGHT_LINE", goal_qf=FormationPose(10.0, 0.0, 0.0))
        _, path = path_for_records(
            [(0.0, 0.0, 0.0), (6.0, 0.0, 0.0)], request)
        # This is the exact independent-audit bypass reported by Codex: a
        # real legal 6 m path bound to a 10 m request with a NaN tolerance.
        bypass = audit_geometry(
            request, path,
            GeometryAuditConfig(position_tolerance_m=float("nan")))
        self.assertFalse(bypass.passed)
        self.assertEqual(bypass.reason, "INVALID_AUDIT_CONFIG")
        self.assertFalse(bypass.location.get("hold", False))
        normal = audit_geometry(request, path)
        self.assertFalse(normal.passed)
        self.assertEqual(normal.reason, "ENDPOINT_POSITION_ERROR")
        self.assertAlmostEqual(normal.location["error"], 4.0, places=12)
        self.assertEqual(normal.location["constraint"], "ENDPOINT_POSITION")
        # The same path is legal for the request it actually reaches.
        valid = request_from_fixture(
            "STRAIGHT_LINE", goal_qf=FormationPose(6.0, 0.0, 0.0))
        valid_path = path_for_records(
            [(0.0, 0.0, 0.0), (6.0, 0.0, 0.0)], valid)[1]
        accepted = audit_geometry(valid, valid_path)
        self.assertTrue(accepted.passed, accepted.as_dict())


class IndependentIntegrationBudgetTests(unittest.TestCase):

    def _quad_case(self):
        profile = nonzero_g_multisegment_profile()
        path = VirtualAPath(
            FormationPose(0.0, 0.0, 0.0), profile, model_from_fixture())
        x, y, psi = high_precision_endpoint(path)
        request = request_from_fixture(
            "STRAIGHT_LINE", goal_qf=FormationPose(x, y, psi))
        bind(path, request)
        return request, path, (x, y, psi)

    def test_nonzero_g_multisegment_against_independent_quadrature(self):
        request, path, exact = self._quad_case()
        report = audit_geometry(request, path)
        self.assertTrue(report.passed, report.as_dict())
        self.assertGreater(report.c2["max_abs_g"], 0.0)
        self.assertEqual(len(report.whole_segment_bounds), 4)
        integration = report.integration
        audit = integration["audit_integration"]
        self.assertTrue(audit["confirmed"])
        self.assertEqual(audit["fixed_threshold_m"], 5e-8)
        self.assertLess(audit["fixed_threshold_m"], 1e-7)
        self.assertLessEqual(audit["total_budget_m"], 5e-8)
        self.assertLessEqual(
            integration["audit_position_budget_m"], 5e-8)
        self.assertGreater(len(audit["segments"]), 1)
        for segment in audit["segments"]:
            self.assertGreater(segment["m4_upper"], 0.0)
            self.assertGreater(segment["subintervals"], 0)
        independent = integration["independent_endpoint"]
        error = math.hypot(independent["x"] - exact[0],
                           independent["y"] - exact[1])
        self.assertLessEqual(error, audit["total_budget_m"])
        # The returned path must agree with the same request and independent
        # integration, not merely with its own reported budget.
        endpoint = path.endpoint()
        self.assertLessEqual(
            math.hypot(endpoint.x - request.goal_qf.x,
                       endpoint.y - request.goal_qf.y),
            request.limits.r_min_ackermann)
        self.assertLessEqual(
            math.hypot(endpoint.x - independent["x"],
                       endpoint.y - independent["y"]),
            integration["combined_position_budget_m"])

    def test_coarse_grid_refines_until_the_budget_is_confirmed(self):
        request, path, _ = self._quad_case()
        report = audit_geometry(
            request, path,
            GeometryAuditConfig(audit_panel_width_m=1.0))
        self.assertTrue(report.passed, report.as_dict())
        audit = report.integration["audit_integration"]
        self.assertGreater(audit["refinement_steps"], 0)
        self.assertLessEqual(audit["total_budget_m"],
                             audit["fixed_threshold_m"])

    def test_resource_limit_is_bound_unconfirmed_never_feasible(self):
        request, path, _ = self._quad_case()
        report = audit_geometry(
            request, path,
            GeometryAuditConfig(
                audit_panel_width_m=1.0,
                audit_max_total_subintervals=4))
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "BOUND_UNCONFIRMED")
        self.assertEqual(report.location["constraint"],
                         "AUDIT_INTEGRATION_TOTAL_LIMIT")

    def test_unreachable_budget_is_bound_unconfirmed(self):
        request, path, _ = self._quad_case()
        report = audit_geometry(
            request, path,
            GeometryAuditConfig(
                audit_panel_width_m=0.02,
                audit_max_total_subintervals=64,
                audit_position_budget_m=1e-15))
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "BOUND_UNCONFIRMED")
        self.assertIn(report.location["constraint"], (
            "AUDIT_INTEGRATION_TOTAL_LIMIT",
            "AUDIT_INTEGRATION_PANEL_LIMIT",
            "AUDIT_INTEGRATION_REMAINDER"))


def g_jump_request_and_path(**limit_changes):
    """C2 path whose third curvature derivative jumps at two interior nodes."""
    factor = 0.1
    records = (
        (0.0, 0.0, 0.0),
        (1.0, factor * 0.20, factor * 0.50),
        (2.0, factor * 1.05, factor * 1.15),
        (3.0, factor * 2.5166666666666666, factor * 1.85))
    profile = profile_from_knots(records)
    path = VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile,
                        model_from_fixture())
    endpoint = path.endpoint()
    request = request_from_fixture(
        "STRAIGHT_LINE",
        goal_qf=FormationPose(endpoint.x, endpoint.y, endpoint.psi),
        limits=limits_from_fixture(**limit_changes))
    bind(path, request)
    return request, path


def tightened_request_and_path(**tight):
    """High limits everywhere, with only the named metrics left realistic."""
    high = 1e6
    changes = {
        "v_max": high, "a_max": high, "jerk_max": high,
        "yaw_rate_max": high, "yaw_accel_max": high,
        "yaw_jerk_max": high, "lateral_accel_max": high,
        "wheel_speed_max": high, "steer_rate_max": high,
        "steer_accel_max": high,
    }
    changes.update(tight)
    return g_jump_request_and_path(**changes)


def shortened_plan(request, result, factor):
    """A coherent-but-too-short model built from understated T0 bounds.

    The selection table is rescaled so that its arithmetic is internally
    consistent at the shortened duration: each ``order``-metric upper bound is
    multiplied by ``scale ** order`` and its candidate is recomputed from the
    stored values.  The geometry is unchanged, so the audit must still find
    the true overlimit by recomputing from the raw profile rather than trust
    the model's own selection table.
    """
    selection = json.loads(json.dumps(result.bounds_summary))
    duration = result.duration_s * factor
    required = duration / float(selection["safety_factor"])
    scale = required / float(selection["required_duration_s"])
    metrics = selection["metrics_at_T0_s"]
    for record in metrics.values():
        order = int(record["order"])
        record["upper_at_T0_s"] = float(record["upper_at_T0_s"]) * \
            scale ** order
        record["required_duration_s"] = (
            float(record["upper_at_T0_s"]) / float(record["limit"])) ** \
            (1.0 / order)
    selection["required_duration_s"] = max(
        record["required_duration_s"] for record in metrics.values())
    selection["selected_duration_s"] = duration
    return VirtualATimedPlan._build_motion(
        request, result.plan.path, duration, selection)


def point_spatial_derivatives(offset, k, eta, u):
    """Independent closed-form local derivatives for a rigid offset."""
    aoff, boff = offset.x, offset.y
    p_s = (1.0 - boff * k, aoff * k)
    p_ss = (-boff * eta - aoff * k * k,
            k - boff * k * k + aoff * eta)
    p_sss = (-k * k + boff * k ** 3 - 3.0 * aoff * k * eta - boff * u,
             eta - 3.0 * boff * k * eta + aoff * u - aoff * k ** 3)
    return p_s, p_ss, p_sss


def point_jerk_components(offset, k, eta, u, w, a, j):
    """Independent pointwise local jerk from the explicit spatial formulas."""
    p_s, p_ss, p_sss = point_spatial_derivatives(offset, k, eta, u)
    return (p_sss[0] * w ** 3 + 3.0 * p_ss[0] * w * a + p_s[0] * j,
            p_sss[1] * w ** 3 + 3.0 * p_ss[1] * w * a + p_s[1] * j)


def assert_interval_contains(test, interval, value, label):
    test.assertLessEqual(interval[0], value, label)
    test.assertGreaterEqual(interval[1], value, label)


class G3WholeDomainCertificateTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.request, cls.path = g_jump_request_and_path()
        cls.result = retime_path(cls.request, cls.path, TimingConfig())
        cls.config = DynamicAuditConfig()
        cls.report = audit_dynamics(cls.request, cls.result.plan, cls.config)

    def test_curve_is_certified_on_the_complete_domain(self):
        self.assertEqual(self.result.status, "DYNAMIC_FEASIBLE",
                         self.result.as_dict())
        report = self.report
        self.assertTrue(report.passed, report.as_dict())
        self.assertEqual(report.scope, "DYNAMICS")
        self.assertIsNone(report.first_failure)
        self.assertTrue(report.metrics["whole_domain_certificate"])
        self.assertFalse(report.metrics["collision_checked"])
        self.assertFalse(report.metrics["executable"])
        self.assertFalse(report.metrics["map_used"])
        self.assertFalse(report.metrics["ros_used"])
        self.assertFalse(report.timing["retimer_selection_is_certificate"])
        self.assertEqual(report.timing["model_sha256"],
                         self.result.plan.model_sha256())

    def test_every_segment_covers_all_metrics_vehicles_and_wheels(self):
        report = self.report
        self.assertEqual(len(report.whole_segment_bounds), 3)
        metric_names = {
            "speed", "acceleration", "jerk", "yaw_rate", "yaw_acceleration",
            "yaw_jerk", "lateral_acceleration", "wheel_speed", "steer_rate",
            "steer_acceleration"}
        limit_names = {
            "speed": "v_max", "acceleration": "a_max", "jerk": "jerk_max",
            "yaw_rate": "yaw_rate_max",
            "yaw_acceleration": "yaw_accel_max",
            "yaw_jerk": "yaw_jerk_max",
            "lateral_acceleration": "lateral_accel_max",
            "wheel_speed": "wheel_speed_max",
            "steer_rate": "steer_rate_max",
            "steer_acceleration": "steer_accel_max",
        }
        leaves = 0
        for segment in report.whole_segment_bounds:
            self.assertTrue(segment["certified"])
            self.assertGreaterEqual(segment["leaves"], 8)
            self.assertGreaterEqual(segment["max_depth"], 0)
            self.assertEqual(set(segment["metrics"]), metric_names)
            leaves += segment["leaves"]
            for name, record in segment["metrics"].items():
                self.assertGreaterEqual(record["upper_bound"], 0.0)
                self.assertAlmostEqual(
                    record["limit"],
                    getattr(self.request.limits, limit_names[name]))
                self.assertLessEqual(record["upper_bound"], record["limit"],
                                     name)
                self.assertIn(record["vehicle_id"], (1, 2, 3))
                self.assertIn(record["wheel_index"], (0, 1, 2, 3))
                self.assertLessEqual(segment["s0"], record["s0"])
                self.assertLessEqual(record["s1"], segment["s1"])
            angle = segment["wheel_angle"]
            self.assertLessEqual(angle["upper_bound"],
                                 self.request.limits.steer_angle_max)
            self.assertIn(angle["vehicle_id"], (1, 2, 3))
        self.assertEqual(report.metrics["leaves"], leaves)
        utilization = report.metrics["max_utilization"]
        self.assertEqual(set(utilization),
                         metric_names | {"wheel_angle"})
        for name, value in utilization.items():
            self.assertGreaterEqual(value, 0.0)
            self.assertLessEqual(value, 1.0, name)
        self.assertLessEqual(report.metrics["max_wheel_angle_rad"],
                             self.request.limits.steer_angle_max)

    def test_internal_nodes_have_left_and_right_motion_limits(self):
        nodes = self.report.metrics["nodes"]
        self.assertEqual(nodes["internal_node_count"], 2)
        self.assertEqual(len(nodes["records"]), 2)
        self.assertGreater(nodes["max_jump_magnitude"], 1e-8)
        self.assertLessEqual(nodes["max_continuity_error"],
                             self.config.continuity_tolerance)
        self.assertLessEqual(nodes["max_state_error"],
                             self.config.state_tolerance)
        self.assertLessEqual(nodes["max_inverse_error_s"],
                             self.config.time_inverse_tolerance_s)
        for record in nodes["records"]:
            self.assertEqual(record["left_region"], "motion")
            self.assertEqual(record["right_region"], "motion")
            self.assertEqual(record["auto_region"], "motion")
            self.assertGreater(record["w"], 0.0)
            self.assertLessEqual(record["inverse_error_s"],
                                 self.config.time_inverse_tolerance_s)

    def test_pointwise_values_lie_below_every_local_interval_certificate(self):
        report = self.report
        pointwise = report.metrics["pointwise_interval_checks"]
        self.assertEqual(pointwise["leaf_count"], report.metrics["leaves"])
        self.assertEqual(pointwise["vehicle_count"],
                         3 * pointwise["leaf_count"])
        self.assertGreater(pointwise["leaf_count"], 0)
        for name in ("max_jerk_ratio", "max_component_ratio",
                     "max_p_sss_ratio"):
            self.assertGreaterEqual(pointwise[name], 0.0, name)
            self.assertLessEqual(pointwise[name], 1.0, name)
        self.assertIsNotNone(pointwise["worst"])
        worst = pointwise["worst"]
        self.assertIn(worst["vehicle_id"], (1, 2, 3))
        self.assertIn(worst["quantity"],
                      ("jerk", "jerk_x", "jerk_y", "p_sss_x", "p_sss_y"))
        self.assertLessEqual(worst["actual"], worst["bound"])
        self.assertLessEqual(worst["ratio"], 1.0)

    def test_node_left_and_right_values_have_adjacent_cell_certificates(self):
        nodes = self.report.metrics["nodes"]["records"]
        self.assertEqual(len(nodes), 2)
        for record in nodes:
            for side in ("left", "right"):
                coverage = record["pointwise_interval"][side]
                self.assertEqual(set(coverage),
                                 {"jerk_ratio", "component_ratio",
                                  "p_sss_ratio", "worst"})
                for ratio in ("jerk_ratio", "component_ratio",
                              "p_sss_ratio"):
                    self.assertGreaterEqual(coverage[ratio], 0.0)
                    self.assertLessEqual(coverage[ratio], 1.0, ratio)
                worst = coverage["worst"]
                self.assertEqual(set(worst),
                                 {"ratio", "vehicle_id", "quantity",
                                  "actual", "bound", "label"})
                self.assertIn(worst["vehicle_id"], (1, 2, 3))
                self.assertLessEqual(worst["actual"], worst["bound"])
                self.assertLessEqual(worst["ratio"], 1.0)
                self.assertIn(side, worst["label"])

    def test_endpoints_and_one_hundred_nonuniform_inverse_samples(self):
        endpoints = self.report.metrics["endpoints"]
        self.assertEqual(len(endpoints["checks"]), 8)
        self.assertLessEqual(endpoints["max_state_error"],
                             self.config.state_tolerance)
        self.assertEqual(endpoints["initial_region"], "initial_hold")
        self.assertEqual(endpoints["terminal_region"], "terminal_hold")
        inverse = self.report.metrics["time_inverse"]
        self.assertEqual(inverse["sample_count"], 100)
        self.assertLessEqual(inverse["max_error_s"],
                             self.config.time_inverse_tolerance_s)
        # The source-tau error is reported separately because a 1-ulp
        # rounding of the station can move the inverse time more than 1e-9 s
        # near an endpoint; the contract compares time_at_s(s) itself above.
        self.assertGreaterEqual(inverse["max_independent_error_s"], 0.0)
        self.assertEqual(len(inverse["samples"]), 100)

    def test_differential_oracle_recomputes_raw_profile_dynamics(self):
        differential = self.report.metrics["differential"]
        self.assertEqual(len(differential["steps"]), 2)
        self.assertEqual([step["dt_s"] for step in differential["steps"]],
                         [0.01, 0.005])
        self.assertGreater(differential["checked_sample_count"], 0)
        self.assertGreaterEqual(differential["skipped_sample_count"], 0)
        self.assertEqual(
            differential["oracle"]["method"],
            "independent_composite_simpson_positions")
        for name, value in differential["max_errors"].items():
            self.assertLessEqual(
                value, differential["tolerances"][name], name)


class AuditIntervalBoundCoverageTests(unittest.TestCase):
    """Direct coverage tests for the audit's independent interval enclosure."""

    def test_degenerate_interval_contains_codex_counterexample(self):
        offset = VehicleOffset(1, 1.0, 0.0)
        jets = {"k": (0.1, 0.1), "eta": (0.1, 0.1),
                "u": (0.0, 0.0), "g": (0.0, 0.0)}
        record = audit_module._audit_interval_vehicle(
            jets, offset, limits_from_fixture(),
            (1.0, 1.0), (0.0, 0.0), (0.0, 0.0), 0.247, 0.182)
        expected_ps_sss = (-0.04, 0.099)
        expected_norm = math.hypot(*expected_ps_sss)
        self.assertEqual(expected_norm, 0.10677546534667971)
        for axis in (0, 1):
            assert_interval_contains(
                self, record["p_sss_local_interval"][axis],
                expected_ps_sss[axis], "p_sss[{}]".format(axis))
            assert_interval_contains(
                self, record["jerk_local_interval"][axis],
                expected_ps_sss[axis], "jerk[{}]".format(axis))
        self.assertGreaterEqual(record["jerk"], expected_norm)
        self.assertGreaterEqual(
            record["jerk_component_norm_upper"], expected_norm)
        self.assertGreaterEqual(
            record["p_sss_component_upper"][0], abs(expected_ps_sss[0]))
        self.assertGreaterEqual(
            record["p_sss_component_upper"][1], abs(expected_ps_sss[1]))

    def test_narrow_intervals_cover_all_sign_combinations_and_components(self):
        width = 1e-5
        offsets = (
            VehicleOffset(1, 1.0, 0.0),
            VehicleOffset(2, 0.0, 1.0),
            VehicleOffset(3, 0.0, -1.0),
        )
        samples = ((0.12, 0.09, 0.05, 0.3),
                   (0.12, -0.09, -0.05, -0.3),
                   (-0.12, 0.09, -0.07, -0.2),
                   (-0.12, -0.09, 0.07, 0.2))
        for offset in offsets:
            for k, eta, u, g in samples:
                with self.subTest(vehicle_id=offset.vehicle_id,
                                  k=k, eta=eta):
                    jets = {
                        "k": (k - width, k + width),
                        "eta": (eta - width, eta + width),
                        "u": (u - width, u + width),
                        "g": (g - width, g + width),
                    }
                    w_interval = (0.8, 1.2)
                    a_interval = (-0.3, 0.4)
                    j_interval = (-0.5, 0.6)
                    record = audit_module._audit_interval_vehicle(
                        jets, offset, limits_from_fixture(),
                        w_interval, a_interval, j_interval, 0.247, 0.182)
                    actual_k = k + 0.37 * width
                    actual_eta = eta - 0.41 * width
                    actual_u = u + 0.53 * width
                    actual_w = 0.8 + 0.31 * 0.4
                    actual_a = -0.3 + 0.29 * 0.7
                    actual_j = -0.5 + 0.61 * 1.1
                    _p_s, _p_ss, p_sss = point_spatial_derivatives(
                        offset, actual_k, actual_eta, actual_u)
                    actual_jerk = point_jerk_components(
                        offset, actual_k, actual_eta, actual_u,
                        actual_w, actual_a, actual_j)
                    for axis in (0, 1):
                        assert_interval_contains(
                            self,
                            record["p_sss_local_interval"][axis],
                            p_sss[axis], "p_sss[{}]".format(axis))
                        assert_interval_contains(
                            self,
                            record["jerk_local_interval"][axis],
                            actual_jerk[axis], "jerk[{}]".format(axis))
                    self.assertGreaterEqual(
                        record["p_sss_component_upper"][0],
                        abs(p_sss[0]))
                    self.assertGreaterEqual(
                        record["p_sss_component_upper"][1],
                        abs(p_sss[1]))
                    self.assertGreaterEqual(
                        record["jerk_component_upper"][0],
                        abs(actual_jerk[0]))
                    self.assertGreaterEqual(
                        record["jerk_component_upper"][1],
                        abs(actual_jerk[1]))
                    self.assertGreaterEqual(
                        record["jerk_component_norm_upper"],
                        math.hypot(*actual_jerk))
                    self.assertGreaterEqual(
                        record["jerk"], math.hypot(*actual_jerk))


def actual_metric_record(request, angle=0.0):
    """All finite, below-limit actual metrics for every vehicle and wheel."""
    metrics = [name for name, _limit, _order in audit_module._DYNAMIC_METRICS]
    wheel = {name: 0.0 for name in metrics}
    return {
        vehicle_id: {
            "wheels": {
                wheel_index: dict(wheel) for wheel_index in range(4)},
            "wheel_angle_rad": float(angle),
        }
        for vehicle_id in sorted(offset.vehicle_id
                                 for offset in request.model.offsets)
    }


def actual_dynamic_record(**changes):
    """One vehicle's raw actual-dynamics dict with finite inputs."""
    dynamic = {
        "v_i": 0.5,
        "acceleration": (0.0, 0.0),
        "jerk": (0.0, 0.0),
        "omega_i": 0.0,
        "yaw_accel": 0.0,
        "yaw_jerk": 0.0,
        "lateral_accel": 0.0,
        "wheel_speeds": [0.0, 0.0, 0.0, 0.0],
        "steer_rates": [0.0, 0.0, 0.0, 0.0],
        "steer_accels": [0.0, 0.0, 0.0, 0.0],
        "C_i": 0.0,
    }
    dynamic.update(changes)
    return dynamic


class G3NonfiniteActualMetricTests(unittest.TestCase):
    """The actual-witness check must reject every non-finite metric."""

    NONFINITE = (
        ("NaN", float("nan")),
        ("+Infinity", float("inf")),
        ("-Infinity", float("-inf")),
    )

    def setUp(self):
        self.request = request_from_fixture("STRAIGHT_LINE")

    def assert_nonfinite_failure(self, caught, vehicle_id, wheel_index,
                                 name, description):
        error = caught.exception
        self.assertEqual(error.reason, "NONFINITE_AUDIT_VALUE")
        self.assertEqual(error.location["vehicle_id"], vehicle_id)
        self.assertEqual(error.location["wheel_index"], wheel_index)
        self.assertEqual(error.location["quantity"], name)
        self.assertEqual(error.location["constraint"],
                         "NONFINITE_AUDIT_VALUE")
        self.assertIn("vehicle_id={}".format(vehicle_id), error.detail)
        self.assertIn("wheel={}".format(wheel_index), error.detail)
        self.assertIn(name, error.detail)
        self.assertIn(description, error.detail)
        self.assertNotIn("value", error.location)
        json.dumps(error.location, allow_nan=False)

    def test_every_dynamic_metric_rejects_nan_and_both_infinities(self):
        metric_names = [
            name for name, _limit, _order in audit_module._DYNAMIC_METRICS]
        covered_metrics = set()
        covered_vehicles = set()
        covered_wheels = set()
        for metric_index, (name, limit_name, order) in enumerate(
                audit_module._DYNAMIC_METRICS):
            for kind_index, (description, value) in enumerate(
                    self.NONFINITE):
                vehicle_id = 1 + (metric_index + kind_index) % 3
                wheel_index = (3 * metric_index + kind_index) % 4
                actual = actual_metric_record(self.request)
                actual[vehicle_id]["wheels"][wheel_index][name] = value
                with self.assertRaises(
                        audit_module._AuditFailure) as caught:
                    audit_module._audit_check_actual_metrics(
                        actual, self.request)
                self.assert_nonfinite_failure(
                    caught, vehicle_id, wheel_index, name, description)
                covered_metrics.add(name)
                covered_vehicles.add(vehicle_id)
                covered_wheels.add(wheel_index)
        self.assertEqual(covered_metrics, set(metric_names))
        self.assertEqual(covered_vehicles, {1, 2, 3})
        self.assertEqual(covered_wheels, {0, 1, 2, 3})

    def test_nonfinite_wheel_angle_is_rejected_with_location(self):
        for index, (description, value) in enumerate(self.NONFINITE):
            vehicle_id = 1 + index
            actual = actual_metric_record(self.request)
            actual[vehicle_id]["wheel_angle_rad"] = value
            with self.assertRaises(
                    audit_module._AuditFailure) as caught:
                audit_module._audit_check_actual_metrics(
                    actual, self.request)
            self.assert_nonfinite_failure(
                caught, vehicle_id, None, "wheel_angle", description)

    def test_nonfinite_curvature_cannot_be_hidden_by_atan2(self):
        """atan2 maps an infinite C_i to a finite angle; reject the source."""
        for description, value in self.NONFINITE:
            with self.subTest(description=description):
                dynamic = {2: actual_dynamic_record(**{"C_i": value})}
                with self.assertRaises(
                        audit_module._AuditFailure) as caught:
                    audit_module._audit_actual_metrics(
                        dynamic, self.request)
                error = caught.exception
                self.assertEqual(error.reason, "NONFINITE_AUDIT_VALUE")
                self.assertEqual(error.location["vehicle_id"], 2)
                self.assertEqual(error.location["quantity"], "C_i")
                self.assertEqual(error.location["constraint"],
                                 "NONFINITE_AUDIT_VALUE")
                self.assertIn(description, error.detail)
                json.dumps(error.location, allow_nan=False)

    def test_nonfinite_raw_wheel_speed_fails_after_scalar_generation(self):
        dynamic = {
            1: actual_dynamic_record(
                wheel_speeds=[0.0, float("nan"), 0.0, 0.0])}
        actual = audit_module._audit_actual_metrics(dynamic, self.request)
        with self.assertRaises(audit_module._AuditFailure) as caught:
            audit_module._audit_check_actual_metrics(actual, self.request)
        self.assert_nonfinite_failure(
            caught, 1, 1, "wheel_speed", "NaN")

    def test_finite_metrics_below_limits_pass(self):
        actual = actual_metric_record(self.request)
        self.assertTrue(audit_module._audit_check_actual_metrics(
            actual, self.request))

    def test_two_finite_overlimits_report_the_worst_utilization(self):
        """The worst must be found after an earlier lower over-limit."""
        limits = self.request.limits
        actual = actual_metric_record(self.request)
        # speed is visited first (vehicle 1, wheel 0) and already violates;
        # the larger jerk utilization is visited later (vehicle 3, wheel 2).
        # A first-violation implementation would incorrectly report speed.
        actual[1]["wheels"][0]["speed"] = 1.2 * limits.v_max
        actual[3]["wheels"][2]["jerk"] = 2.0 * limits.jerk_max
        with self.assertRaises(audit_module._AuditFailure) as caught:
            audit_module._audit_check_actual_metrics(actual, self.request)
        error = caught.exception
        self.assertEqual(error.reason, "DYNAMIC_LIMIT")
        self.assertEqual(error.location["vehicle_id"], 3)
        self.assertEqual(error.location["wheel_index"], 2)
        self.assertEqual(error.location["quantity"], "jerk")
        self.assertEqual(error.location["value"], 2.0 * limits.jerk_max)
        self.assertEqual(error.location["limit"], limits.jerk_max)
        self.assertNotIn("non-finite", error.detail)

    def test_two_finite_overlimits_report_worst_not_last(self):
        """The worst must beat a later, smaller over-limit too."""
        limits = self.request.limits
        actual = actual_metric_record(self.request)
        # speed is visited first with the larger utilization; the smaller
        # jerk violation is visited last. A last-violation implementation
        # would incorrectly report jerk at vehicle 3/wheel 2.
        actual[1]["wheels"][0]["speed"] = 2.0 * limits.v_max
        actual[3]["wheels"][2]["jerk"] = 1.2 * limits.jerk_max
        with self.assertRaises(audit_module._AuditFailure) as caught:
            audit_module._audit_check_actual_metrics(actual, self.request)
        error = caught.exception
        self.assertEqual(error.reason, "DYNAMIC_LIMIT")
        self.assertEqual(error.location["vehicle_id"], 1)
        self.assertEqual(error.location["wheel_index"], 0)
        self.assertEqual(error.location["quantity"], "speed")
        self.assertEqual(error.location["value"], 2.0 * limits.v_max)
        self.assertEqual(error.location["limit"], limits.v_max)
        self.assertNotIn("non-finite", error.detail)

    def test_nonfinite_dominates_an_earlier_finite_overlimit(self):
        """A later non-finite value must fail immediately, not lose the max."""
        limits = self.request.limits
        for vehicle_id in (1, 3):
            with self.subTest(vehicle_id=vehicle_id):
                actual = actual_metric_record(self.request)
                actual[1]["wheels"][0]["speed"] = 1.2 * limits.v_max
                actual[vehicle_id]["wheels"][2]["jerk"] = float("nan")
                with self.assertRaises(
                        audit_module._AuditFailure) as caught:
                    audit_module._audit_check_actual_metrics(
                        actual, self.request)
                self.assert_nonfinite_failure(
                    caught, vehicle_id, 2, "jerk", "NaN")

    def test_nonfinite_wheel_angle_dominates_finite_metric_overlimit(self):
        """An angle must not be preempted by an earlier finite over-limit."""
        limits = self.request.limits
        actual = actual_metric_record(self.request)
        actual[1]["wheels"][0]["speed"] = 1.2 * limits.v_max
        cases = (
            (1, float("nan"), "NaN"),
            (3, float("inf"), "+Infinity"),
        )
        for vehicle_id, value, description in cases:
            with self.subTest(vehicle_id=vehicle_id):
                candidate = {
                    key: {
                        "wheels": {
                            wheel: dict(record)
                            for wheel, record in data["wheels"].items()},
                        "wheel_angle_rad": data["wheel_angle_rad"],
                    }
                    for key, data in actual.items()
                }
                candidate[vehicle_id]["wheel_angle_rad"] = value
                with self.assertRaises(
                        audit_module._AuditFailure) as caught:
                    audit_module._audit_check_actual_metrics(
                        candidate, self.request)
                self.assert_nonfinite_failure(
                    caught, vehicle_id, None, "wheel_angle", description)

    def test_nonfinite_raw_sources_cannot_be_sanitised(self):
        """Every raw vector/steering source reaches a rejected scalar."""
        cases = (
            ("v_i", float("nan"), "speed", 0),
            ("acceleration", (float("inf"), 0.0), "acceleration", 0),
            ("jerk", (0.0, float("nan")), "jerk", 0),
            ("omega_i", float("inf"), "yaw_rate", 0),
            ("yaw_accel", float("nan"), "yaw_acceleration", 0),
            ("yaw_jerk", float("-inf"), "yaw_jerk", 0),
            ("lateral_accel", float("inf"), "lateral_acceleration", 0),
            ("wheel_speeds", [0.0, float("nan"), 0.0, 0.0],
             "wheel_speed", 1),
            ("steer_rates", [0.0, 0.0, float("inf"), 0.0],
             "steer_rate", 2),
            ("steer_accels", [0.0, 0.0, 0.0, float("-inf")],
             "steer_acceleration", 3),
        )
        for field, value, metric, wheel_index in cases:
            with self.subTest(field=field, metric=metric):
                dynamic = {1: actual_dynamic_record(**{field: value})}
                actual = audit_module._audit_actual_metrics(
                    dynamic, self.request)
                with self.assertRaises(
                        audit_module._AuditFailure) as caught:
                    audit_module._audit_check_actual_metrics(
                        actual, self.request)
                error = caught.exception
                self.assertEqual(error.reason, "NONFINITE_AUDIT_VALUE")
                self.assertEqual(error.location["vehicle_id"], 1)
                self.assertEqual(error.location["wheel_index"], wheel_index)
                self.assertEqual(error.location["quantity"], metric)
                self.assertEqual(error.location["constraint"],
                                 "NONFINITE_AUDIT_VALUE")
                json.dumps(error.location, allow_nan=False)

    def test_finite_wheel_angle_overlimit_keeps_original_reason(self):
        limit = self.request.limits.steer_angle_max
        actual = actual_metric_record(self.request)
        actual[2]["wheel_angle_rad"] = 1.5 * limit
        with self.assertRaises(audit_module._AuditFailure) as caught:
            audit_module._audit_check_actual_metrics(actual, self.request)
        error = caught.exception
        self.assertEqual(error.reason, "WHEEL_ANGLE_LIMIT")
        self.assertEqual(error.location["vehicle_id"], 2)
        self.assertIsNone(error.location["wheel_index"])
        self.assertEqual(error.location["quantity"], "wheel_angle")
        self.assertEqual(error.location["value"], 1.5 * limit)
        self.assertEqual(error.location["limit"], limit)


class G3NonfiniteAuditChainTests(unittest.TestCase):
    """Injected non-finite witnesses fail the complete dynamic audit."""

    def test_coarse_interval_witness_injection_fails_closed(self):
        request, path = g_jump_request_and_path()
        result = retime_path(request, path, TimingConfig())
        self.assertEqual(result.status, "DYNAMIC_FEASIBLE")
        config = DynamicAuditConfig(
            interval_subdivisions_per_segment=1, max_depth=20)
        baseline = audit_dynamics(request, result.plan, config)
        self.assertTrue(baseline.passed, baseline.as_dict())
        self.assertGreater(baseline.metrics["leaves"], 3)

        original = audit_module._audit_actual_metrics
        injections = []

        def injected(dynamic, req):
            record = original(dynamic, req)
            if not injections:
                injections.append(record)
                record[1]["wheels"][0]["wheel_speed"] = float("nan")
            return record

        with mock.patch.object(
                audit_module, "_audit_actual_metrics",
                side_effect=injected):
            report = audit_dynamics(request, result.plan, config)
        # The coarse interval must have required an actual witness/refinement
        # and therefore exercised the injected record.
        self.assertEqual(len(injections), 1)
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "NONFINITE_AUDIT_VALUE",
                         report.as_dict())
        self.assertEqual(report.first_failure["vehicle_id"], 1)
        self.assertEqual(report.first_failure["wheel_index"], 0)
        self.assertEqual(report.first_failure["quantity"], "wheel_speed")
        self.assertEqual(report.first_failure["constraint"],
                         "NONFINITE_AUDIT_VALUE")
        self.assertIn("wheel_speed", report.detail)
        self.assertIn("NaN", report.detail)
        self.assertFalse(report.metrics["dynamic_audit_passed"])
        payload = report.as_dict()
        # A failure report must remain strict-JSON serialisable and must not
        # publish a feasible status anywhere.
        serialised = json.dumps(payload, allow_nan=False)
        self.assertNotIn("DYNAMIC_FEASIBLE", serialised)

    def test_interval_enclosure_generator_nonfinite_fails_closed(self):
        request, path = g_jump_request_and_path()
        result = retime_path(request, path, TimingConfig())
        self.assertEqual(result.status, "DYNAMIC_FEASIBLE")
        config = DynamicAuditConfig(
            interval_subdivisions_per_segment=1, max_depth=20)
        baseline = audit_dynamics(request, result.plan, config)
        self.assertTrue(baseline.passed, baseline.as_dict())
        self.assertGreater(baseline.metrics["leaves"], 3)

        original_dynamic = audit_module._audit_dynamic_at_time
        cases = (("v_i", "speed", 0), ("C_i", "C_i", None))
        for source, quantity, wheel_index in cases:
            with self.subTest(source=source):
                leaf_calls = []
                injections = []

                def failed_interval_leaf(*args, **kwargs):
                    leaf_calls.append(1)
                    raise audit_module._AuditFailure(
                        "NORMAL_BRANCH_VIOLATION",
                        "injected interval-enclosure failure",
                        constraint="NORMAL_BRANCH")

                def injected_dynamic(*args, **kwargs):
                    spatial, dynamic = original_dynamic(*args, **kwargs)
                    # Inject only once the interval leaf has failed, so the
                    # next real generator call is the interval-enclosure
                    # witness at audit.py:2569-2570.
                    if leaf_calls and not injections:
                        injections.append(1)
                        for vehicle in dynamic.values():
                            vehicle[source] = float("nan")
                    return spatial, dynamic

                with mock.patch.object(
                        audit_module, "_audit_interval_leaf",
                        side_effect=failed_interval_leaf), \
                        mock.patch.object(
                            audit_module, "_audit_dynamic_at_time",
                            side_effect=injected_dynamic):
                    report = audit_dynamics(request, result.plan, config)
                # The enclosure fallback, not a subdivision or the
                # bound-exceeded witness branch, must have consumed the
                # non-finite raw source and converted the failure.
                self.assertEqual(len(leaf_calls), 1)
                self.assertEqual(len(injections), 1)
                self.assertFalse(report.passed)
                self.assertEqual(report.reason, "NONFINITE_AUDIT_VALUE",
                                 report.as_dict())
                self.assertEqual(report.first_failure["vehicle_id"], 1)
                self.assertEqual(report.first_failure["wheel_index"],
                                 wheel_index)
                self.assertEqual(report.first_failure["quantity"], quantity)
                self.assertEqual(report.first_failure["constraint"],
                                 "NONFINITE_AUDIT_VALUE")
                self.assertNotIn("value", report.first_failure)
                self.assertIn(quantity, report.detail)
                self.assertIn("NaN", report.detail)
                self.assertFalse(report.metrics["dynamic_audit_passed"])
                payload = report.as_dict()
                serialised = json.dumps(payload, allow_nan=False)
                self.assertNotIn("DYNAMIC_FEASIBLE", serialised)


class G3DurationCounterexampleTests(unittest.TestCase):
    """The audit must reject a coherent-but-too-short model, not trust T."""

    def _assert_short_plan_rejected(self, tight, quantity, vehicle_id,
                                    wheel_index, factor=0.75):
        request, path = tightened_request_and_path(**tight)
        result = retime_path(request, path, TimingConfig())
        self.assertEqual(result.status, "DYNAMIC_FEASIBLE", result.as_dict())
        worst = result.bounds_summary["worst"]
        self.assertEqual(worst["quantity"], quantity)
        accepted = audit_dynamics(request, result.plan, DynamicAuditConfig())
        self.assertTrue(accepted.passed, accepted.as_dict())
        plan = shortened_plan(request, result, factor)
        rejected = audit_dynamics(request, plan, DynamicAuditConfig())
        self.assertFalse(rejected.passed)
        self.assertEqual(rejected.reason, "DYNAMIC_LIMIT",
                         rejected.as_dict())
        location = rejected.first_failure
        self.assertEqual(location["quantity"], quantity)
        self.assertEqual(location["vehicle_id"], vehicle_id)
        if wheel_index is not None:
            self.assertEqual(location["wheel_index"], wheel_index)
        self.assertGreater(location["value"], location["limit"])
        self.assertFalse(rejected.metrics["dynamic_audit_passed"])

    def test_side_vehicle_speed_limit(self):
        self._assert_short_plan_rejected(
            {"v_max": 0.2}, "speed", 3, 0)

    def test_front_wheel_steer_rate_limit(self):
        self._assert_short_plan_rejected(
            {"steer_rate_max": 0.02}, "steer_rate", 1, 0)

    def test_front_wheel_steer_acceleration_limit(self):
        self._assert_short_plan_rejected(
            {"steer_accel_max": 0.02}, "steer_acceleration", 1, 0)

    def test_side_vehicle_front_right_wheel_speed_limit(self):
        self._assert_short_plan_rejected(
            {"wheel_speed_max": 0.05}, "wheel_speed", 3, 1)

    def test_center_vehicle_lateral_acceleration_limit(self):
        self._assert_short_plan_rejected(
            {"lateral_accel_max": 0.05}, "lateral_acceleration", 1, 0)

    def test_side_vehicle_jerk_limit_rejects_a_shortened_plan(self):
        self._assert_short_plan_rejected(
            {"jerk_max": 0.2}, "jerk", 3, 0, factor=0.8)


class G3HoldAndFailClosedTests(unittest.TestCase):

    def test_hold_dynamics_are_audited_without_inverse_division(self):
        request = request_from_fixture(
            "STRAIGHT_LINE", goal_qf=FormationPose(0.0, 0.0, 0.0))
        path = bind(VirtualAPath.hold(
            FormationPose(0.0, 0.0, 0.0), model_from_fixture()), request)
        result = retime_path(request, path, TimingConfig())
        self.assertEqual(result.status, "NO_MOTION")
        report = audit_dynamics(request, result.plan)
        self.assertTrue(report.passed, report.as_dict())
        self.assertEqual(report.scope, "DYNAMICS")
        self.assertEqual(report.metrics["kind"], "hold")
        self.assertFalse(report.metrics["motion"])
        self.assertTrue(report.metrics["hold"]["inverse_division_free"])
        self.assertEqual(report.metrics["hold"]["sample_times"], [0.0])
        self.assertEqual(report.timing["T_s"], 0.0)
        self.assertEqual(report.whole_segment_bounds, ())

    def test_leaf_budget_exhaustion_never_reports_feasible(self):
        request, path = g_jump_request_and_path()
        result = retime_path(request, path, TimingConfig())
        self.assertEqual(result.status, "DYNAMIC_FEASIBLE")
        report = audit_dynamics(
            request, result.plan, DynamicAuditConfig(max_leaves=1))
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "AUDIT_BUDGET_EXCEEDED",
                         report.as_dict())
        self.assertEqual(report.location["constraint"],
                         "BOUND_LEAF_BUDGET")
        self.assertFalse(report.metrics["dynamic_audit_passed"])

    def test_coarse_interval_without_refinement_fails_closed(self):
        request, path = g_jump_request_and_path()
        result = retime_path(request, path, TimingConfig())
        self.assertEqual(result.status, "DYNAMIC_FEASIBLE")
        report = audit_dynamics(
            request, result.plan,
            DynamicAuditConfig(interval_subdivisions_per_segment=1,
                               max_depth=0))
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "BOUND_UNCONFIRMED",
                         report.as_dict())
        self.assertEqual(report.location["constraint"], "BOUND_UNCONFIRMED")
        self.assertGreater(report.location["value"],
                           report.location["limit"])
        self.assertFalse(report.metrics["dynamic_audit_passed"])

    def test_coarse_interval_subdivides_until_every_bound_is_confirmed(self):
        request, path = g_jump_request_and_path()
        result = retime_path(request, path, TimingConfig())
        report = audit_dynamics(
            request, result.plan,
            DynamicAuditConfig(interval_subdivisions_per_segment=1,
                               max_depth=20))
        self.assertTrue(report.passed, report.as_dict())
        self.assertGreater(report.metrics["leaves"], 3)
        self.assertLessEqual(report.metrics["max_utilization"]["jerk"], 1.0)

    def test_invalid_dynamic_audit_config_fails_closed(self):
        request, path = g_jump_request_and_path()
        result = retime_path(request, path, TimingConfig())
        for config, field in (
                (DynamicAuditConfig(max_leaves=0), "max_leaves"),
                (DynamicAuditConfig(max_depth=-1), "max_depth"),
                (DynamicAuditConfig(differential_dt_s=()), "differential_dt_s")):
            with self.subTest(field=field):
                report = audit_dynamics(request, result.plan, config)
                self.assertFalse(report.passed)
                self.assertEqual(report.reason, "INVALID_AUDIT_CONFIG")
                self.assertEqual(report.location["field"], field)


class VersionedAuditBindingTests(unittest.TestCase):
    """Step 5: the audit accepts both supported numerics and rejects swaps."""

    @classmethod
    def setUpClass(cls):
        request, path = g_jump_request_and_path()
        result = retime_path(request, path, TimingConfig())
        cls.request = request
        cls.plan = result.plan
        cls.config = DynamicAuditConfig()

    def _tampered(self, mutate):
        import copy
        plan = copy.deepcopy(self.plan)
        mutate(plan._model_core)
        return audit_dynamics(self.request, plan, self.config)

    def test_v3_plan_passes_with_v3_declared_everywhere(self):
        report = audit_dynamics(self.request, self.plan, self.config)
        self.assertTrue(report.passed, report.as_dict())
        core = self.plan._model_core
        self.assertEqual(core["numerics"]["version"],
                         bounds_module.NUMERICS_V3)
        self.assertEqual(core["path"]["profile"]["numerics_version"],
                         bounds_module.NUMERICS_V3)

    def test_tampered_numerics_version_is_rejected(self):
        cases = (
            ("unknown_numerics",
             lambda core: core["numerics"].__setitem__(
                 "version", "other"),
             "UNKNOWN_NUMERICS_VERSION"),
            ("unknown_profile",
             lambda core: core["path"]["profile"].__setitem__(
                 "numerics_version", "other"),
             "UNSUPPORTED_PROFILE"),
            ("profile_v2_numerics_v3",
             lambda core: core["path"]["profile"].__setitem__(
                 "numerics_version", bounds_module.NUMERICS_V2),
             "MODEL_NUMERICS_MISMATCH"),
            ("numerics_v2_profile_v3",
             lambda core: core["numerics"].__setitem__(
                 "version", bounds_module.NUMERICS_V2),
             "MODEL_NUMERICS_MISMATCH"),
        )
        for name, mutate, expected in cases:
            with self.subTest(case=name):
                report = self._tampered(mutate)
                self.assertFalse(report.passed)
                self.assertEqual(report.reason, expected)

    def test_v3_budget_swapped_for_a_v2_budget_is_rejected(self):
        def mutate(core):
            core["path"]["integration_budget"] = {
                "kind": "motion",
                "position_error_bound_m": 0.0,
                "angle_error_bound_rad": 0.0,
                "total_panels": 0,
            }
        report = self._tampered(mutate)
        self.assertFalse(report.passed)
        self.assertEqual(report.reason, "MODEL_NUMERICS_MISMATCH")


if __name__ == "__main__":
    unittest.main()