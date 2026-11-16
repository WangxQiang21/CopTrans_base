"""G1 geometry contract tests for the virtual rigid formation A backend.

Positive cases are compared with an independent analytic arc or with the
reused ``MotionBlock`` control polynomial.  Negative cases assert the exact
structured rejection reason and location; catching an arbitrary exception is
never accepted as evidence.

Run from the worktree root:

    PYTHONDONTWRITEBYTECODE=1 \
        python3 -B -m unittest discover -s tests \
        -p 'test_virtual_a_geometry.py' -v
"""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO_ROOT / "catkin_ws" / "src" / "fwis_v3_optimizer" / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from fwis_v3_optimizer.continuous_geometry import (  # noqa: E402
    ControlKnot, MotionBlock)
from fwis_v3_optimizer.formation import (  # noqa: E402
    FormationModel, FormationPose, VehicleOffset)
from fwis_v3_optimizer.formation_continuous_a import (  # noqa: E402
    C2CurvatureProfile, CurvatureJet, InitialAssumption, VirtualAContractError,
    VirtualAPath, check_vehicle_curvatures, map_spatial_state,
    validate_initial_state)
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits  # noqa: E402
from fwis_v3_optimizer.uniform_motion import a_arc_edge  # noqa: E402


def _load_benchmark():
    path = REPO_ROOT / "tools" / "virtual_a_benchmark.py"
    spec = importlib.util.spec_from_file_location("virtual_a_benchmark", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BENCHMARK = _load_benchmark()


def research_model():
    return FormationModel((
        VehicleOffset(1, 1.0, 0.0),
        VehicleOffset(2, 0.0, 1.0),
        VehicleOffset(3, 0.0, -1.0)))


def straight_profile(length=2.0):
    return C2CurvatureProfile.from_knots((
        ControlKnot(0.0, 0.0, 0.0),
        ControlKnot(length, 0.0, 0.0)))


def quadratic_profile():
    """k = 0.01 s^2, eta = 0.02 s on [0, 2]; exactly C2."""
    return C2CurvatureProfile.from_knots((
        ControlKnot(0.0, 0.0, 0.0),
        ControlKnot(1.0, 0.01, 0.02),
        ControlKnot(2.0, 0.04, 0.04)))


def g_jump_profile():
    """C2 profile with a curvature-slope jump at s=1 (g left 0.6, right -1.8)."""
    return C2CurvatureProfile.from_knots((
        ControlKnot(0.0, 0.0, 0.0),
        ControlKnot(1.0, 0.0, 0.1),
        ControlKnot(2.0, 0.0, -0.4)))


def constant_curvature_profile(radius=5.0, length=6.0):
    return C2CurvatureProfile.from_knots((
        ControlKnot(0.0, 1.0 / radius, 0.0),
        ControlKnot(length, 1.0 / radius, 0.0)))


class GeometryTestBase(unittest.TestCase):
    def assertRejected(self, expected_reason, call, **expected_location):
        with self.assertRaises(VirtualAContractError) as caught:
            call()
        error = caught.exception
        self.assertEqual(error.reason, expected_reason, msg=str(error))
        for key, value in expected_location.items():
            self.assertIn(key, error.location, msg=str(error))
            self.assertEqual(error.location[key], value,
                             msg="{} expected {!r}".format(key, value))
        return error


class StraightAndArcTests(GeometryTestBase):
    def test_straight_motion_is_exact_and_rigid(self):
        model = research_model()
        path = VirtualAPath(FormationPose(0.0, 0.0, 0.0),
                            straight_profile(), model)
        self.assertLessEqual(path.position_error_bound_m, 1e-7)
        self.assertLessEqual(path.angle_error_bound_rad, 1e-9)
        for s in (0.0, 0.4, 1.9, 2.0):
            state = path.state_at_s(s)
            self.assertAlmostEqual(state.qf.x, s, delta=1e-12)
            self.assertAlmostEqual(state.qf.y, 0.0, delta=1e-12)
            self.assertAlmostEqual(state.qf.psi, 0.0, delta=1e-12)
            for vehicle in state.vehicles:
                offset = {1: (1.0, 0.0), 2: (0.0, 1.0),
                          3: (0.0, -1.0)}[vehicle.vehicle_id]
                self.assertAlmostEqual(vehicle.x, s + offset[0], delta=1e-12)
                self.assertAlmostEqual(vehicle.y, offset[1], delta=1e-12)
                self.assertAlmostEqual(vehicle.theta, 0.0, delta=1e-12)
                self.assertAlmostEqual(vehicle.curvature, 0.0, delta=1e-12)
                self.assertEqual(vehicle.p_s, (1.0, 0.0))
                self.assertEqual(vehicle.p_ss, (0.0, 0.0))
                self.assertEqual(vehicle.p_sss, (0.0, 0.0))
                # The A-only speed ratio is exactly 1 on a straight.
                self.assertAlmostEqual(math.hypot(*vehicle.p_s), 1.0,
                                       delta=1e-15)
            centers = {vehicle.vehicle_id: (vehicle.x, vehicle.y)
                       for vehicle in state.vehicles}
            self.assertLessEqual(model.rigidity_residual(centers), 1e-12)
            self.assertAlmostEqual(
                math.hypot(centers[1][0] - centers[2][0],
                           centers[1][1] - centers[2][1]),
                math.sqrt(2.0), delta=1e-15)
            self.assertAlmostEqual(
                math.hypot(centers[1][0] - centers[3][0],
                           centers[1][1] - centers[3][1]),
                math.sqrt(2.0), delta=1e-15)
            self.assertAlmostEqual(
                math.hypot(centers[2][0] - centers[3][0],
                           centers[2][1] - centers[3][1]), 2.0, delta=1e-15)

    def test_constant_curvature_matches_fixed_pivot_arc(self):
        model = research_model()
        radius = 5.0
        kappa = 1.0 / radius
        path = VirtualAPath(FormationPose(0.0, 0.0, 0.0),
                            constant_curvature_profile(radius), model)
        start_headings = {1: math.atan2(1.0, radius), 2: 0.0, 3: 0.0}
        edge = a_arc_edge(
            model, FormationPose(0.0, 0.0, 0.0), start_headings,
            delta_psi=1.0, pivot=(0.0, radius),
            directions={1: 1, 2: 1, 3: 1})
        curvatures = dict(edge.curvatures)
        for progress in (0.1, 0.35, 0.8):
            edge_state = edge.state_at(progress)
            state = path.state_at_s(progress / kappa)
            self.assertAlmostEqual(state.qf.psi, progress, delta=1e-9)
            self.assertAlmostEqual(
                math.hypot(state.qf.x - edge_state.qf.x,
                           state.qf.y - edge_state.qf.y), 0.0, delta=1e-6)
            for vehicle in state.vehicles:
                centre = edge_state.center(vehicle.vehicle_id)
                self.assertAlmostEqual(
                    math.hypot(vehicle.x - centre[0], vehicle.y - centre[1]),
                    0.0, delta=1e-6)
                self.assertAlmostEqual(
                    math.atan2(
                        math.sin(vehicle.theta
                                 - edge_state.heading(vehicle.vehicle_id)),
                        math.cos(vehicle.theta
                                 - edge_state.heading(vehicle.vehicle_id))),
                    0.0, delta=1e-9)
                self.assertAlmostEqual(
                    vehicle.curvature, curvatures[vehicle.vehicle_id],
                    delta=1e-9)
            # Independent closed form around the cargo pivot at (0, 5).
            for vehicle in state.vehicles:
                offset = {1: (1.0, 0.0), 2: (0.0, 1.0),
                          3: (0.0, -1.0)}[vehicle.vehicle_id]
                # Rotate (offset - pivot) by progress around the pivot.
                relative_x = offset[0]
                relative_y = offset[1] - radius
                cosine, sine = math.cos(progress), math.sin(progress)
                expected_x = (cosine * relative_x - sine * relative_y)
                expected_y = radius + (sine * relative_x
                                       + cosine * relative_y)
                self.assertAlmostEqual(vehicle.x, expected_x, delta=1e-6)
                self.assertAlmostEqual(vehicle.y, expected_y, delta=1e-6)

    def test_constant_curvature_jets_match_analytic_relations(self):
        model = research_model()
        path = VirtualAPath(FormationPose(0.0, 0.0, 0.0),
                            constant_curvature_profile(5.0), model)
        state = path.state_at_s(2.0)
        for vehicle in state.vehicles:
            offset = {1: (1.0, 0.0), 2: (0.0, 1.0),
                      3: (0.0, -1.0)}[vehicle.vehicle_id]
            forward = 1.0 - offset[1] * 0.2
            denominator = forward ** 2 + (offset[0] * 0.2) ** 2
            expected_curvature = 0.2 / math.sqrt(denominator)
            self.assertAlmostEqual(vehicle.curvature, expected_curvature,
                                   delta=1e-12)
            self.assertAlmostEqual(
                math.hypot(*vehicle.p_s), math.sqrt(denominator), delta=1e-12)
            # eta = u = g = 0 for constant curvature.
            self.assertAlmostEqual(vehicle.curvature_s, 0.0, delta=1e-12)
            self.assertAlmostEqual(vehicle.curvature_ss, 0.0, delta=1e-12)
            self.assertAlmostEqual(vehicle.theta_ss, 0.0, delta=1e-12)
            self.assertAlmostEqual(vehicle.theta_sss, 0.0, delta=1e-12)


class ProfileReuseTests(GeometryTestBase):
    def test_c2_profile_matches_motion_block(self):
        profile = quadratic_profile()
        block = MotionBlock(mode=0, direction=1, length=2.0,
                            knots=profile.knots, slope_policy="fixed")
        for s in (0.0, 0.25, 0.5, 0.999, 1.0, 1.5, 2.0):
            jet = profile.jet_at(s)
            value, first, second = block.control(s)
            self.assertAlmostEqual(jet.kappa, value, delta=1e-14)
            self.assertAlmostEqual(jet.eta, first, delta=1e-14)
            self.assertAlmostEqual(jet.u, second, delta=1e-14)
        # Analytic k = 0.01 s^2, eta = 0.02 s, u = 0.02, g = 0.
        for s in (0.4, 1.3, 1.9):
            jet = profile.jet_at(s)
            self.assertAlmostEqual(jet.kappa, 0.01 * s * s, delta=1e-15)
            self.assertAlmostEqual(jet.eta, 0.02 * s, delta=1e-15)
            self.assertAlmostEqual(jet.u, 0.02, delta=1e-14)
            self.assertAlmostEqual(jet.g, 0.0, delta=1e-14)
        # Independent closed form for psi: integral of 0.01 s^2 is s^3 / 300.
        self.assertAlmostEqual(profile.psi_at(1.0), 1.0 / 300.0, delta=1e-15)
        self.assertAlmostEqual(profile.psi_at(2.0), 8.0 / 300.0, delta=1e-15)

    def test_node_side_limits_and_auto_policy(self):
        profile = quadratic_profile()
        left = profile.jet_at(1.0, "left")
        right = profile.jet_at(1.0, "right")
        automatic = profile.jet_at(1.0)
        # Floating-point evaluation of the two pieces may differ in the last
        # bit; values and all derivatives must still agree to tight tolerance.
        self.assertAlmostEqual(left.kappa, right.kappa, delta=1e-15)
        self.assertAlmostEqual(left.eta, right.eta, delta=1e-15)
        self.assertAlmostEqual(left.u, right.u, delta=1e-14)
        self.assertAlmostEqual(left.g, right.g, delta=1e-14)
        self.assertEqual(automatic.as_dict(), right.as_dict())
        self.assertRejected("SIDE_DOMAIN", lambda: profile.jet_at(0.0, "left"),
                            s=0.0)
        self.assertRejected(
            "SIDE_DOMAIN", lambda: profile.jet_at(2.0, "right"), s=2.0)

    def test_g_jump_is_c2_valid_and_side_semantics_hold(self):
        profile = g_jump_profile()
        left = profile.jet_at(1.0, "left")
        right = profile.jet_at(1.0, "right")
        self.assertAlmostEqual(left.kappa, right.kappa, delta=1e-15)
        self.assertAlmostEqual(left.eta, right.eta, delta=1e-15)
        self.assertAlmostEqual(left.u, 0.4, delta=1e-14)
        self.assertAlmostEqual(right.u, 0.4, delta=1e-14)
        self.assertAlmostEqual(left.g, 0.6, delta=1e-14)
        self.assertAlmostEqual(right.g, -1.8, delta=1e-14)
        self.assertEqual(profile.jet_at(1.0).as_dict(), right.as_dict())

    def test_non_c2_profile_is_rejected_at_the_node(self):
        # Mandatory G1 fixture: (s, k, eta) = (0,0,0), (1,.1,0), (2,.3,0).
        mandatory = (ControlKnot(0.0, 0.0, 0.0),
                     ControlKnot(1.0, 0.1, 0.0),
                     ControlKnot(2.0, 0.3, 0.0))
        error = self.assertRejected(
            "NON_C2_PROFILE",
            lambda: C2CurvatureProfile.from_knots(mandatory),
            node=1, s=1.0)
        # Both pieces are cubic Hermite fits of the same C1 data, but their
        # second derivatives at the node are -0.6 and 1.2 1/m^2 (the
        # zero eta at both knots forces the cubic pieces to bend oppositely).
        self.assertAlmostEqual(error.location["u_left"], -0.6, delta=1e-14)
        self.assertAlmostEqual(error.location["u_right"], 1.2, delta=1e-14)
        knots = (ControlKnot(0.0, 0.0, 0.0),
                 ControlKnot(1.0, 0.01, 0.02),
                 ControlKnot(2.0, 0.04, 0.03))
        error = self.assertRejected(
            "NON_C2_PROFILE",
            lambda: C2CurvatureProfile.from_knots(knots),
            node=1, s=1.0)
        self.assertAlmostEqual(error.location["u_left"], 0.02, delta=1e-14)
        self.assertAlmostEqual(error.location["u_right"], 0.04, delta=1e-14)


class RejectionTests(GeometryTestBase):
    def test_station_domain_is_rejected_not_clipped(self):
        profile = quadratic_profile()
        self.assertRejected("S_DOMAIN", lambda: profile.jet_at(-0.1), s=-0.1)
        self.assertRejected("S_DOMAIN", lambda: profile.jet_at(2.1), s=2.1)
        self.assertRejected("S_DOMAIN", lambda: profile.psi_at(2.5), s=2.5)
        self.assertRejected(
            "NONFINITE_VALUE", lambda: profile.jet_at(float("nan")))
        self.assertRejected(
            "NONFINITE_VALUE", lambda: profile.jet_at(float("inf")))
        self.assertRejected(
            "INVALID_SIDE", lambda: profile.jet_at(0.5, "middle"))

    def test_invalid_knots_are_rejected(self):
        self.assertRejected(
            "INVALID_KNOTS", lambda: C2CurvatureProfile.from_knots(()))
        self.assertRejected(
            "INVALID_KNOTS",
            lambda: C2CurvatureProfile.from_knots((ControlKnot(0, 0, 0),)))
        self.assertRejected(
            "INVALID_KNOT",
            lambda: C2CurvatureProfile.from_knots(((0.0, 0.0, 0.0),
                                                   (1.0, 0.0, 0.0))),
            index=0)
        self.assertRejected(
            "NONFINITE_KNOT",
            lambda: C2CurvatureProfile.from_knots((
                ControlKnot(0.0, 0.0, 0.0),
                ControlKnot(1.0, float("nan"), 0.0))),
            index=1)
        self.assertRejected(
            "INVALID_PROFILE_DOMAIN",
            lambda: C2CurvatureProfile.from_knots((
                ControlKnot(0.5, 0.0, 0.0),
                ControlKnot(1.0, 0.0, 0.0))))
        self.assertRejected(
            "NON_POSITIVE_SEGMENT",
            lambda: C2CurvatureProfile.from_knots((
                ControlKnot(0.0, 0.0, 0.0),
                ControlKnot(1.0, 0.0, 0.0),
                ControlKnot(1.0, 0.0, 0.0))),
            index=1, h=0.0, s=1.0)
        self.assertRejected(
            "INVALID_KNOT_ORDER",
            lambda: C2CurvatureProfile.from_knots((
                ControlKnot(0.0, 0.0, 0.0),
                ControlKnot(1.0, 0.0, 0.0),
                ControlKnot(0.5, 0.0, 0.0))),
            index=2)

    def test_wrong_formation_is_rejected(self):
        start = FormationPose(0.0, 0.0, 0.0)
        wrong_offsets = FormationModel((
            VehicleOffset(1, 1.0, 0.0),
            VehicleOffset(2, 0.0, 2.0),
            VehicleOffset(3, 0.0, -1.0)))
        self.assertRejected(
            "FORMATION_MISMATCH",
            lambda: VirtualAPath(start, straight_profile(), wrong_offsets))
        wrong_ids = FormationModel((
            VehicleOffset(1, 1.0, 0.0),
            VehicleOffset(2, 0.0, 1.0),
            VehicleOffset(4, 0.0, -1.0)))
        self.assertRejected(
            "FORMATION_MISMATCH",
            lambda: VirtualAPath(start, straight_profile(), wrong_ids))
        self.assertRejected(
            "FORMATION_MISMATCH",
            lambda: map_spatial_state(
                start, CurvatureJet(0.0, 0.0, 0.0, 0.0), wrong_offsets))
        self.assertRejected(
            "INVALID_FORMATION_MODEL",
            lambda: VirtualAPath(start, straight_profile(), object()))

    def test_normal_branch_violation_names_the_vehicle(self):
        model = research_model()
        error = self.assertRejected(
            "NORMAL_BRANCH_VIOLATION",
            lambda: map_spatial_state(
                FormationPose(0.0, 0.0, 0.0),
                CurvatureJet(1.0, 0.0, 0.0, 0.0), model),
            vehicle_id=2, s=0.0)
        self.assertAlmostEqual(error.location["A"], 0.0, delta=1e-15)
        self.assertRejected(
            "NORMAL_BRANCH_VIOLATION",
            lambda: map_spatial_state(
                FormationPose(0.0, 0.0, 0.0),
                CurvatureJet(-1.0, 0.0, 0.0, 0.0), model),
            vehicle_id=3)
        # kappa = 1/r_min would give A = 1 - 1/0.81 = -0.2345 for car 2 and
        # would pass through A = 0 for kappa = 1.0; both are rejected.
        self.assertRejected(
            "NORMAL_BRANCH_VIOLATION",
            lambda: map_spatial_state(
                FormationPose(0.0, 0.0, 0.0),
                CurvatureJet(1.0 / 0.81, 0.0, 0.0, 0.0), model),
            vehicle_id=2)

    def test_hold_path_accepts_only_zero_station(self):
        model = research_model()
        start = FormationPose(1.0, 2.0, 0.3)
        path = VirtualAPath.hold(start, model)
        state = path.state_at_s(0.0)
        self.assertEqual(state.s, 0.0)
        self.assertEqual(path.endpoint(), start)
        for vehicle in state.vehicles:
            self.assertEqual(vehicle.curvature, 0.0)
            self.assertEqual(vehicle.theta_s, 0.0)
            self.assertAlmostEqual(vehicle.theta, start.psi, delta=1e-15)
        self.assertRejected("S_DOMAIN", lambda: path.state_at_s(0.1), s=0.1)


class InitialStateTests(GeometryTestBase):
    def setUp(self):
        self.model = research_model()
        self.start = FormationPose(0.0, 0.0, 0.0)
        self.assumption = InitialAssumption(True, "A", True, "offline_fixture")

    def test_valid_stationary_state_is_accepted(self):
        result = validate_initial_state(
            self.start, {1: 0.0, 2: 0.0, 3: 0.0},
            self.assumption, model=self.model)
        self.assertTrue(result.passed)
        self.assertEqual(result.reason, None)
        self.assertEqual(result.metrics["vehicle_count"], 3)
        self.assertEqual(result.metrics["source"], "offline_fixture")
        self.assertFalse(result.metrics["node_context_verified"])

    def test_measured_heading_mismatch_is_rejected_per_vehicle(self):
        result = validate_initial_state(
            self.start, {1: 0.1, 2: 0.0, 3: 0.0},
            self.assumption, model=self.model)
        self.assertFalse(result.passed)
        self.assertEqual(result.reason, "INITIAL_HEADING_MISMATCH")
        self.assertEqual(result.location["vehicle_id"], 1)
        self.assertAlmostEqual(result.location["heading_error_rad"], 0.1,
                               delta=1e-12)
        # Headings are compared, not overwritten by psi first.
        result = validate_initial_state(
            self.start, {1: 0.0, 2: 0.2, 3: 0.0},
            self.assumption, model=self.model)
        self.assertEqual(result.reason, "INITIAL_HEADING_MISMATCH")
        self.assertEqual(result.location["vehicle_id"], 2)

    def test_heading_id_set_must_match_the_model(self):
        missing = validate_initial_state(
            self.start, {1: 0.0, 2: 0.0}, self.assumption, model=self.model)
        self.assertEqual(missing.reason, "INVALID_VEHICLE_ID_SET")
        extra = validate_initial_state(
            self.start, {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0},
            self.assumption, model=self.model)
        self.assertEqual(extra.reason, "INVALID_VEHICLE_ID_SET")
        nan = validate_initial_state(
            self.start, {1: 0.0, 2: float("nan"), 3: 0.0},
            self.assumption, model=self.model)
        self.assertEqual(nan.reason, "NONFINITE_VALUE")
        self.assertEqual(nan.location["vehicle_id"], 2)

    def test_nonstationary_or_non_a_declarations_are_rejected(self):
        cases = (
            (InitialAssumption(False, "A", True, "offline_fixture"), {}),
            (InitialAssumption(True, "O", True, "offline_fixture"),
             {"mode": "O"}),
            (InitialAssumption(True, "A", False, "offline_fixture"), {}),
        )
        for assumption, location in cases:
            result = validate_initial_state(
                self.start, {1: 0.0, 2: 0.0, 3: 0.0},
                assumption, model=self.model)
            self.assertEqual(result.reason, "UNSUPPORTED_INITIAL_STATE")
            for key, value in location.items():
                self.assertEqual(result.location[key], value)

    def test_nonzero_initial_curvature_is_rejected(self):
        result = validate_initial_state(
            self.start, {1: 0.0, 2: 0.0, 3: 0.0},
            self.assumption, kappa0=0.01, model=self.model)
        self.assertEqual(result.reason, "UNSUPPORTED_INITIAL_STATE")
        self.assertEqual(result.location["quantity"], "kappa0")
        result = validate_initial_state(
            self.start, {1: 0.0, 2: 0.0, 3: 0.0},
            self.assumption, eta0=1e-6, model=self.model)
        self.assertEqual(result.location["quantity"], "eta0")

    def test_initial_source_contract(self):
        self.assertRejected(
            "MISSING_CONTEXT_ID",
            lambda: InitialAssumption(True, "A", True, "rviz_reset"))
        self.assertRejected(
            "UNKNOWN_INITIAL_SOURCE",
            lambda: InitialAssumption(True, "A", True, "measured"))
        InitialAssumption(True, "A", True, "rviz_reset", "session-1")
        self.assertRejected(
            "INVALID_INITIAL_ASSUMPTION",
            lambda: InitialAssumption(1, "A", True, "offline_fixture"))


class CurvatureLimitTests(GeometryTestBase):
    def test_reference_profile_is_within_the_limit(self):
        model = research_model()
        profile = BENCHMARK.DoubleLaneChangeProfile()
        path = VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile, model)
        limits = FormationTimingLimits()
        for s in (0.0, 0.5, 1.5, 3.0, 4.5, 6.0, 9.0, 11.9, 12.0):
            state = path.state_at_s(s)
            result = check_vehicle_curvatures(state, limits)
            self.assertTrue(result.passed, msg=str(result.as_dict()))
            self.assertLessEqual(result.metrics["max_abs_curvature"],
                                 result.metrics["limit"])

    def test_short_profile_reports_the_front_vehicle_value(self):
        model = research_model()
        short = BENCHMARK.DoubleLaneChangeProfile(L=0.5)
        path = VirtualAPath(FormationPose(0.0, 0.0, 0.0), short, model)
        limits = FormationTimingLimits()
        first_failure = None
        peak = {1: 0.0, 2: 0.0, 3: 0.0}
        for index in range(20001):
            s = short.length_m * index / 20000
            state = path.state_at_s(s)
            for vehicle in state.vehicles:
                peak[vehicle.vehicle_id] = max(peak[vehicle.vehicle_id],
                                               abs(vehicle.curvature))
            if first_failure is None:
                result = check_vehicle_curvatures(state, limits)
                if not result.passed:
                    first_failure = (s, result)
        self.assertIsNotNone(first_failure)
        station, result = first_failure
        self.assertEqual(result.reason, "CURVATURE_LIMIT")
        self.assertEqual(result.location["vehicle_id"], 1)
        self.assertEqual(result.location["s"], station)
        self.assertAlmostEqual(result.location["limit"], 1.0 / 0.810,
                               delta=1e-15)
        self.assertGreater(result.location["curvature"],
                           result.location["limit"])
        self.assertAlmostEqual(peak[1], 4.34822, delta=1e-3)
        self.assertAlmostEqual(peak[2], 0.428571, delta=1e-3)
        self.assertAlmostEqual(peak[3], 0.428571, delta=1e-3)


if __name__ == "__main__":
    unittest.main()