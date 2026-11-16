"""G1 profile, integration-bound and analytic-reference tests.

The analytic sin^3 double-lane-change reference is computed from closed-form
expressions; its position is integrated independently with
``scipy.integrate.quad``.  The production Simpson integral is tested against
that oracle and against five-point derivatives of the oracle, never against
its own derivative fields.

Run from the worktree root:

    PYTHONDONTWRITEBYTECODE=1 \
        python3 -B -m unittest discover -s tests \
        -p 'test_virtual_a_profile.py' -v
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

from fwis_v3_optimizer.continuous_geometry import ControlKnot  # noqa: E402
from fwis_v3_optimizer.formation import (  # noqa: E402
    FormationModel, FormationPose, VehicleOffset)
from fwis_v3_optimizer.formation_continuous_a import (  # noqa: E402
    C2CurvatureProfile, VirtualAContractError, VirtualAPath)
from fwis_v3_optimizer.formation_continuous_a import bounds as bounds_module  # noqa: E402


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


def starting_pose():
    return FormationPose(0.0, 0.0, 0.0)


def quadratic_profile():
    return C2CurvatureProfile.from_knots((
        ControlKnot(0.0, 0.0, 0.0),
        ControlKnot(1.0, 0.01, 0.02),
        ControlKnot(2.0, 0.04, 0.04)))


def g_jump_profile():
    return C2CurvatureProfile.from_knots((
        ControlKnot(0.0, 0.0, 0.0),
        ControlKnot(1.0, 0.0, 0.1),
        ControlKnot(2.0, 0.0, -0.4)))


class AnalyticReferenceTests(unittest.TestCase):
    def setUp(self):
        self.model = research_model()
        self.profile = BENCHMARK.DoubleLaneChangeProfile()
        self.mirror = BENCHMARK.DoubleLaneChangeProfile(mirror=True)
        self.path = VirtualAPath(starting_pose(), self.profile, self.model)
        self.mirror_path = VirtualAPath(starting_pose(), self.mirror,
                                        self.model)

    def test_endpoints_match_the_mandatory_reference(self):
        for profile in (self.profile, self.mirror):
            x, y = BENCHMARK.quadrature_position(profile, profile.L)
            expected_x = (BENCHMARK.EXPECTED_HALF_ENDPOINT[0]
                          if not profile.mirror
                          else BENCHMARK.EXPECTED_HALF_ENDPOINT[0])
            expected_y = (BENCHMARK.EXPECTED_HALF_ENDPOINT[1]
                          if not profile.mirror
                          else -BENCHMARK.EXPECTED_HALF_ENDPOINT[1])
            self.assertAlmostEqual(x, expected_x, delta=1e-6)
            self.assertAlmostEqual(y, expected_y, delta=1e-6)
            self.assertAlmostEqual(profile.psi_at(profile.L), 0.0, delta=1e-12)
            self.assertAlmostEqual(profile.psi_at(2.0 * profile.L), 0.0,
                                   delta=1e-12)
            self.assertAlmostEqual(
                profile.psi_at(2.0 * profile.L), 0.0, delta=1e-12)
        x, y = BENCHMARK.quadrature_position(self.profile, 2.0 * self.profile.L)
        self.assertAlmostEqual(x, BENCHMARK.EXPECTED_FULL_ENDPOINT[0],
                               delta=1e-6)
        self.assertAlmostEqual(y, 0.0, delta=1e-6)

    def test_production_endpoint_is_within_one_micrometre(self):
        x, y = BENCHMARK.quadrature_position(self.profile, self.profile.L)
        endpoint = self.path.state_at_s(self.profile.L)
        self.assertLessEqual(
            math.hypot(endpoint.qf.x - x, endpoint.qf.y - y), 1e-6)
        x, y = BENCHMARK.quadrature_position(self.profile,
                                             self.profile.length_m)
        final = self.path.endpoint()
        self.assertLessEqual(math.hypot(final.x - x, final.y - y), 1e-6)
        self.assertLessEqual(self.path.position_error_bound_m, 1e-7)
        self.assertLessEqual(self.path.angle_error_bound_rad, 1e-9)

    def test_curvature_sign_sequence_both_branches(self):
        stations = (self.profile.L / 4.0, 3.0 * self.profile.L / 4.0,
                    5.0 * self.profile.L / 4.0, 7.0 * self.profile.L / 4.0)
        signs = [1 if self.profile.jet_at(s).kappa > 0 else -1
                 for s in stations]
        self.assertEqual(signs, [1, -1, -1, 1])
        mirror_signs = [1 if self.mirror.jet_at(s).kappa > 0 else -1
                        for s in stations]
        self.assertEqual(mirror_signs, [-1, 1, 1, -1])

    def test_mandatory_node_stations_and_left_right_states(self):
        stations = (self.profile.L / 2.0, self.profile.L,
                    1.5 * self.profile.L)
        for s in stations:
            left = self.path.state_at_s(s, "left")
            right = self.path.state_at_s(s, "right")
            automatic = self.path.state_at_s(s)
            for name in ("kappa", "eta", "u"):
                self.assertAlmostEqual(
                    getattr(left.curvature, name),
                    getattr(right.curvature, name), delta=1e-12)
            if s == self.profile.L:
                # The sin^3 reference changes its third derivative at the
                # middle node: g left = +2.067, g right = -2.067.
                self.assertAlmostEqual(left.curvature.g,
                                       -right.curvature.g, delta=1e-12)
            else:
                self.assertAlmostEqual(left.curvature.g,
                                       right.curvature.g, delta=1e-12)
            self.assertAlmostEqual(left.qf.x, right.qf.x, delta=1e-12)
            self.assertAlmostEqual(left.qf.y, right.qf.y, delta=1e-12)
            self.assertAlmostEqual(left.qf.psi, right.qf.psi, delta=1e-12)
            for vehicle_id in (1, 2, 3):
                fields = ["x", "y", "theta", "curvature", "theta_s",
                          "curvature_s"]
                if s != self.profile.L:
                    fields.append("curvature_ss")
                for name in fields:
                    self.assertAlmostEqual(
                        getattr(left.vehicle(vehicle_id), name),
                        getattr(right.vehicle(vehicle_id), name), delta=1e-12)
                for name in ("x", "y", "theta", "curvature"):
                    self.assertAlmostEqual(
                        getattr(automatic.vehicle(vehicle_id), name),
                        getattr(right.vehicle(vehicle_id), name), delta=0.0)
        # Domain ends have only one side; auto uses the boundary side.
        with self.assertRaises(VirtualAContractError) as caught:
            self.path.state_at_s(0.0, "left")
        self.assertEqual(caught.exception.reason, "SIDE_DOMAIN")
        with self.assertRaises(VirtualAContractError) as caught:
            self.path.state_at_s(self.profile.length_m, "right")
        self.assertEqual(caught.exception.reason, "SIDE_DOMAIN")
        self.assertAlmostEqual(
            self.path.state_at_s(0.0).qf.x,
            self.path.state_at_s(0.0, "right").qf.x, delta=0.0)
        self.assertAlmostEqual(
            self.path.state_at_s(self.profile.length_m).qf.x,
            self.path.state_at_s(self.profile.length_m, "left").qf.x,
            delta=0.0)

    def test_analytic_eta_sup_and_sampled_peaks(self):
        kappa = self.profile.K
        c = self.profile.c
        analytic_sup = 3.0 * kappa * c * 2.0 / (3.0 * math.sqrt(3.0))
        self.assertAlmostEqual(analytic_sup, 0.3627598728, delta=1e-9)
        sampled_eta = max(
            abs(self.profile.jet_at(self.profile.length_m * index / 12000).eta)
            for index in range(12001))
        self.assertLessEqual(sampled_eta, analytic_sup)
        self.assertAlmostEqual(sampled_eta, analytic_sup, delta=1e-6)

        peak = {1: 0.0, 2: 0.0, 3: 0.0}
        for index in range(12001):
            s = self.profile.length_m * index / 12000
            for vehicle in self.path.state_at_s(s).vehicles:
                peak[vehicle.vehicle_id] = max(
                    peak[vehicle.vehicle_id], abs(vehicle.curvature))
        self.assertAlmostEqual(peak[1], 0.526313, delta=1e-3)
        self.assertAlmostEqual(peak[2], 0.428571, delta=1e-3)
        self.assertAlmostEqual(peak[3], 0.428571, delta=1e-3)

    def test_mirror_maps_centres_headings_and_curvature_jets(self):
        swap = {1: 1, 2: 3, 3: 2}
        for s in (0.7, 2.9, 5.5, 6.5, 9.1, 11.4):
            positive = self.path.state_at_s(s)
            mirrored = self.mirror_path.state_at_s(s)
            self.assertAlmostEqual(positive.qf.x, mirrored.qf.x, delta=1e-12)
            self.assertAlmostEqual(positive.qf.y, -mirrored.qf.y, delta=1e-12)
            self.assertAlmostEqual(positive.qf.psi, -mirrored.qf.psi,
                                   delta=1e-12)
            for vehicle_id in (1, 2, 3):
                source = positive.vehicle(vehicle_id)
                target = mirrored.vehicle(swap[vehicle_id])
                self.assertAlmostEqual(source.x, target.x, delta=1e-12)
                self.assertAlmostEqual(source.y, -target.y, delta=1e-12)
                self.assertAlmostEqual(
                    math.atan2(math.sin(source.theta + target.theta),
                               math.cos(source.theta + target.theta)),
                    0.0, delta=1e-12)
                self.assertAlmostEqual(source.curvature, -target.curvature,
                                       delta=1e-12)
                self.assertAlmostEqual(
                    source.curvature_s, -target.curvature_s, delta=1e-12)
                self.assertAlmostEqual(
                    source.curvature_ss, -target.curvature_ss, delta=1e-12)
                for name in ("p_s", "p_ss", "p_sss"):
                    source_vector = getattr(source, name)
                    target_vector = getattr(target, name)
                    self.assertAlmostEqual(
                        source_vector[0], target_vector[0], delta=1e-12)
                    self.assertAlmostEqual(
                        source_vector[1], -target_vector[1], delta=1e-12)

    def test_reference_rigidity_and_pair_distances(self):
        for s in (0.0, 1.2, 3.0, 6.0, 8.4, 11.9, 12.0):
            state = self.path.state_at_s(s)
            centres = {vehicle.vehicle_id: (vehicle.x, vehicle.y)
                       for vehicle in state.vehicles}
            self.assertLessEqual(self.model.rigidity_residual(centres), 1e-6)
            self.assertAlmostEqual(
                math.hypot(centres[1][0] - centres[2][0],
                           centres[1][1] - centres[2][1]),
                math.sqrt(2.0), delta=1e-9)
            self.assertAlmostEqual(
                math.hypot(centres[1][0] - centres[3][0],
                           centres[1][1] - centres[3][1]),
                math.sqrt(2.0), delta=1e-9)
            self.assertAlmostEqual(
                math.hypot(centres[2][0] - centres[3][0],
                           centres[2][1] - centres[3][1]), 2.0, delta=1e-9)

    def test_wheel_speed_and_yaw_rate_diagnostics(self):
        speed = 0.5
        state = self.path.state_at_s(1.3)
        for vehicle in state.vehicles:
            offset = {1: (1.0, 0.0), 2: (0.0, 1.0),
                      3: (0.0, -1.0)}[vehicle.vehicle_id]
            a, b = offset
            kappa = state.curvature.kappa
            eta = state.curvature.eta
            forward = 1.0 - b * kappa
            lateral = a * kappa
            denominator = forward * forward + lateral * lateral
            root = math.sqrt(denominator)
            heading_slope = kappa + a * eta / denominator
            self.assertAlmostEqual(math.hypot(*vehicle.p_s), root,
                                   delta=1e-12)
            self.assertAlmostEqual(vehicle.theta_s, heading_slope, delta=1e-12)
            self.assertAlmostEqual(vehicle.curvature,
                                   heading_slope / root, delta=1e-12)
            center_speed = root * speed
            yaw_rate = heading_slope * speed
            self.assertAlmostEqual(center_speed,
                                   math.hypot(*vehicle.p_s) * speed,
                                   delta=1e-12)
            self.assertAlmostEqual(yaw_rate,
                                   vehicle.theta_s * speed, delta=1e-12)


class NodeAndBoundTests(unittest.TestCase):
    def test_c2_node_values_side_limits_and_auto_policy(self):
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
        # psi and the integral are C0/C1 at the node.
        self.assertAlmostEqual(profile.psi_at(1.0 - 1e-12),
                               profile.psi_at(1.0), delta=1e-9)
        self.assertAlmostEqual(profile.psi_at(1.0 + 1e-12),
                               profile.psi_at(1.0), delta=1e-9)
        # left/right limits of a real interval resolve to the node sides.
        left_interval = profile.interval_bounds(1.0, 1.5)
        right_interval = profile.interval_bounds(0.5, 1.0)
        self.assertAlmostEqual(left_interval["left_limit"]["g"], 0.6,
                               delta=1e-14)
        self.assertAlmostEqual(right_interval["right_limit"]["g"], -1.8,
                               delta=1e-14)

    def test_production_interval_bounds_contain_dense_samples(self):
        for profile in (quadratic_profile(), g_jump_profile()):
            for s0, s1 in profile.domain_segments:
                result = profile.interval_bounds(s0, s1)
                for index, record in enumerate(result["segments"]):
                    self.assertGreaterEqual(record["s0"], s0)
                    self.assertLessEqual(record["s1"], s1)
                    self.assertGreaterEqual(record["s1"], record["s0"])
                    self.assertEqual(len(record["coefficients"]), 4)
                    self.assertGreater(record["h"], 0.0)
                    self.assertIn("bound_source", record)
                for index in range(2001):
                    s = s0 + (s1 - s0) * index / 2000
                    # The interval owns the right limit at its left end and
                    # the left limit at its right end.
                    if s == s0:
                        side = "right"
                    elif s == s1:
                        side = "left"
                    else:
                        side = "auto"
                    jet = profile.jet_at(s, side)
                    for name in ("k", "eta", "u", "g"):
                        value = {"k": jet.kappa, "eta": jet.eta,
                                 "u": jet.u, "g": jet.g}[name]
                        self.assertLessEqual(result[name][0], value + 1e-15)
                        self.assertGreaterEqual(result[name][1], value - 1e-15)
                # The interval is built from widened coefficients, not from
                # the sampled maximum: g=0 still gets a positive-width
                # interval for the quadratic profile.
                if s1 <= 1.0000001:
                    self.assertGreater(result["g"][1], 0.0)

    def test_analytic_envelope_contains_samples_and_is_not_the_sample_max(self):
        profile = BENCHMARK.DoubleLaneChangeProfile()
        result = profile.interval_bounds(0.0, profile.length_m)
        self.assertEqual(result["segments"][0]["bound_source"],
                         "analytic_trig_envelope")
        sampled = {"k": 0.0, "eta": 0.0, "u": 0.0, "g": 0.0}
        for index in range(4001):
            s = profile.length_m * index / 4000
            jet = profile.jet_at(s)
            for name, value in (("k", jet.kappa), ("eta", jet.eta),
                                ("u", jet.u), ("g", jet.g)):
                sampled[name] = max(sampled[name], abs(value))
                self.assertLessEqual(result[name][0], value + 1e-15)
                self.assertGreaterEqual(result[name][1], value - 1e-15)
        # The eta envelope 3 K c is strictly above the true maximum.
        self.assertGreater(result["eta"][1], sampled["eta"])
        self.assertGreater(result["k"][1], sampled["k"] - 1e-12)

    def test_m4_bound_covers_dense_fourth_derivative_expression(self):
        profile = BENCHMARK.DoubleLaneChangeProfile()
        segment_bounds = profile.interval_bounds(0.0, profile.length_m)
        k = segment_bounds["k"][1]
        eta = segment_bounds["eta"][1]
        u = segment_bounds["u"][1]
        g = segment_bounds["g"][1]
        bound = bounds_module.m4_upper(k, eta, u, g)
        worst = 0.0
        for index in range(20001):
            s = profile.length_m * index / 20000
            jet = profile.jet_at(s)
            expression = (abs(jet.g) + 4.0 * abs(jet.kappa) * abs(jet.u)
                          + 3.0 * jet.eta ** 2
                          + 6.0 * jet.kappa ** 2 * abs(jet.eta)
                          + jet.kappa ** 4)
            worst = max(worst, expression)
        self.assertGreaterEqual(bound, worst)
        self.assertGreater(bound, 0.0)

    def test_query_order_does_not_change_the_state(self):
        profile = BENCHMARK.DoubleLaneChangeProfile()
        model = research_model()
        path = VirtualAPath(starting_pose(), profile, model)
        stations = [profile.length_m * index / 50 for index in range(51)]
        forward = [path.state_at_s(s).as_dict() for s in stations]
        backward = [path.state_at_s(s).as_dict()
                    for s in reversed(stations)][::-1]
        shuffled = [path.state_at_s(s).as_dict()
                    for s in stations[1::2] + stations[0::2]]
        self.assertEqual(forward, backward)
        self.assertEqual(sorted(map(repr, forward)), sorted(map(repr, shuffled)))
        fresh = VirtualAPath(starting_pose(), profile, model)
        self.assertEqual(forward, [fresh.state_at_s(s).as_dict()
                                   for s in stations])
        for s in stations[1:-1]:
            automatic = path.state_at_s(s)
            left = path.state_at_s(s, "left")
            right = path.state_at_s(s, "right")
            self.assertEqual(left.position_error_bound_m,
                             automatic.position_error_bound_m)
            self.assertEqual(right.angle_error_bound_rad,
                             automatic.angle_error_bound_rad)
            for vehicle_id in (1, 2, 3):
                for name in ("x", "y", "theta", "curvature"):
                    self.assertAlmostEqual(
                        getattr(left.vehicle(vehicle_id), name),
                        getattr(right.vehicle(vehicle_id), name), delta=1e-12)


class IntegrationBudgetTests(unittest.TestCase):
    def test_quadratic_and_cubic_profile_errors_within_reported_budgets(self):
        cubic = C2CurvatureProfile.from_knots((
            ControlKnot(0.0, 0.0, 0.0),
            ControlKnot(2.0, 0.072, 0.068)))
        for profile in (quadratic_profile(), cubic):
            path = VirtualAPath(starting_pose(), profile, research_model())
            self.assertLessEqual(path.position_error_bound_m, 1e-7)
            self.assertLessEqual(path.angle_error_bound_rad, 1e-9)
            worst = 0.0
            for index in range(201):
                s = profile.length_m * index / 200
                x, y = BENCHMARK.quadrature_position(profile, s)
                state = path.state_at_s(s)
                worst = max(worst,
                            math.hypot(state.qf.x - x, state.qf.y - y))
            self.assertLessEqual(worst, path.position_error_bound_m)
            self.assertGreaterEqual(path.integration_budget["total_panels"], 2)
            self.assertIn("numerics_version", path.integration_budget)

    def test_differential_recovery_against_independent_quadrature(self):
        model = research_model()
        profile = BENCHMARK.DoubleLaneChangeProfile()
        path = VirtualAPath(starting_pose(), profile, model)
        for width in (0.001, 0.0005):
            result = BENCHMARK._differential_evidence(
                profile, path, model, width)
            self.assertEqual(result["points"], 121)
            self.assertLessEqual(result["max_heading_error_rad"], 1e-6)
            self.assertLessEqual(result["max_curvature_error_1_per_m"], 1e-5)
            self.assertLessEqual(result["max_production_position_error_m"],
                                 1e-6)

    def test_analytic_profile_domain_rejections(self):
        profile = BENCHMARK.DoubleLaneChangeProfile()
        for call in (lambda: profile.jet_at(-0.1),
                     lambda: profile.jet_at(profile.length_m + 0.1),
                     lambda: profile.interval_bounds(-1.0, 1.0),
                     lambda: profile.interval_bounds(1.0, 20.0)):
            with self.assertRaises(VirtualAContractError) as caught:
                call()
            self.assertEqual(caught.exception.reason, "S_DOMAIN")
        with self.assertRaises(VirtualAContractError) as caught:
            profile.jet_at(0.0, "left")
        self.assertEqual(caught.exception.reason, "SIDE_DOMAIN")


class SimpsonRemainderReworkTests(unittest.TestCase):
    """Regression tests for the corrected 2*d panel-width remainder."""

    M4 = 5.5

    @staticmethod
    def _panel_remainder(width, m4):
        return width ** 5 * m4 / 2880.0

    @staticmethod
    def _local_panel(profile, path, segment_index, s):
        """Reconstruct the one local Simpson panel used by state_at_s."""
        s0, s1 = profile.domain_segments[segment_index]
        h = s1 - s0
        n = path.integration_budget["panels_per_segment"][segment_index]
        d = h / n
        local = min(max(0.0, s - s0), h)
        whole = int(math.floor(local / d))
        if whole >= n or local >= h - 32.0 * bounds_module.EPSILON * h:
            whole = n
        if whole % 2:
            whole -= 1
        start = s0 + whole * d
        remainder = s - start
        count = 1 if remainder > 32.0 * bounds_module.EPSILON * h else 0
        return {
            "n": n, "d": d, "whole": whole,
            "local_panel_count": count,
            "local_panel_width_m": remainder,
        }

    def test_simpson_segment_uses_2d_full_panels_for_even_n(self):
        for h, n in ((2.0, 2), (1.7, 4), (3.0, 10), (0.8, 200), (6.0, 600)):
            d = h / n
            width = 2.0 * d
            record = bounds_module.simpson_segment_budget(h, n, self.M4)
            expected_full = (n // 2) * self._panel_remainder(width, self.M4)
            expected_local = self._panel_remainder(width, self.M4)
            expected_total = expected_full + expected_local
            self.assertEqual(record["subintervals"], n)
            self.assertEqual(record["full_panel_count"], n // 2)
            self.assertEqual(record["local_panel_count"], 1)
            self.assertGreaterEqual(record["full_panel_width_m"], width)
            self.assertLessEqual(record["full_panel_width_m"],
                                 width * (1.0 + 1e-12))
            self.assertGreaterEqual(record["full_remainder_m"], expected_full)
            self.assertLessEqual(record["full_remainder_m"],
                                 expected_full * (1.0 + 1e-12))
            self.assertGreaterEqual(record["local_remainder_m"],
                                    expected_local)
            self.assertLessEqual(record["local_remainder_m"],
                                 expected_local * (1.0 + 1e-12))
            self.assertGreaterEqual(record["segment_remainder_m"],
                                    expected_total)
            self.assertLessEqual(record["segment_remainder_m"],
                                 expected_total * (1.0 + 1e-12))
            if n == 2:
                # One complete panel of width h plus one local panel of the
                # same width: the old (n+1)*R(d) budget is not this bound.
                self.assertLessEqual(
                    abs(record["segment_remainder_m"]
                        - 2.0 * self._panel_remainder(h, self.M4)),
                    2.0 * self._panel_remainder(h, self.M4) * 1e-12)
            old_underestimate = (n + 1) * self._panel_remainder(d, self.M4)
            self.assertGreater(record["segment_remainder_m"],
                               old_underestimate)

    def test_position_budget_sums_complete_and_local_panels(self):
        records = ((2.0, 2, 7.0), (1.5, 6, 3.0))
        budget = bounds_module.position_budget(records)
        details = [
            bounds_module.simpson_segment_budget(h, n, m4)
            for h, n, m4 in records]
        expected = sum(item["segment_remainder_m"] for item in details)
        self.assertGreaterEqual(budget["per_coordinate_m"], expected)
        self.assertLessEqual(budget["per_coordinate_m"],
                             expected * (1.0 + 1e-12))
        self.assertEqual(budget["total_panels"], 2 + 6)
        self.assertEqual(budget["full_panels"], 1 + 3)
        self.assertEqual(budget["local_panels"], 2)
        self.assertEqual(len(budget["segment_budgets"]), 2)
        self.assertGreaterEqual(
            budget["euclidean_m"],
            math.sqrt(2.0) * budget["per_coordinate_m"])
        self.assertLessEqual(
            budget["euclidean_m"],
            math.sqrt(2.0) * budget["per_coordinate_m"] * (1.0 + 1e-12))

    def test_partial_queries_use_at_most_one_two_d_panel(self):
        model = research_model()
        for profile in (BENCHMARK.DoubleLaneChangeProfile(), g_jump_profile()):
            path = VirtualAPath(starting_pose(), profile, model)
            budget = path.position_error_bound_m
            for index, (s0, s1) in enumerate(profile.domain_segments):
                n = path.integration_budget["panels_per_segment"][index]
                d = (s1 - s0) / n
                stations = [
                    s0,                       # no local panel
                    s0 + 0.37 * d,            # inside a complete panel
                    s0 + 3.0 * d,             # odd grid node
                    s1 - 0.25 * d,            # prefix is shortened to even
                    s1 - 1e-6 * d,            # local width approaches 2*d
                ]
                if index == 0:
                    stations.append(s1)   # curvature-segment boundary
                for s in stations:
                    local = self._local_panel(profile, path, index, s)
                    self.assertIn(local["local_panel_count"], (0, 1))
                    self.assertGreaterEqual(local["local_panel_width_m"], 0.0)
                    self.assertLessEqual(
                        local["local_panel_width_m"],
                        2.0 * d * (1.0 + 1e-12))
                    x, y = BENCHMARK.quadrature_position(profile, s)
                    state = path.state_at_s(s)
                    error = math.hypot(state.qf.x - x, state.qf.y - y)
                    self.assertLessEqual(error, budget)

    def test_nonzero_g_c2_positions_within_corrected_budget(self):
        profile = g_jump_profile()
        path = VirtualAPath(starting_pose(), profile, research_model())
        # The profile has a non-zero third derivative on both pieces, hence a
        # non-zero fourth-derivative term in the cos/sin integrands.
        self.assertAlmostEqual(profile.jet_at(0.25).g, 0.6, delta=1e-12)
        self.assertAlmostEqual(profile.jet_at(1.75).g, -1.8, delta=1e-12)
        self.assertGreater(max(path.integration_budget["m4_per_segment"]), 0.0)
        for s in (0.13, 0.47, 0.999999, 1.0, 1.000001, 1.5, 1.999999):
            x, y = BENCHMARK.quadrature_position(profile, s)
            state = path.state_at_s(s)
            error = math.hypot(state.qf.x - x, state.qf.y - y)
            self.assertLessEqual(error, path.position_error_bound_m)

    def test_refinement_is_recorded_and_uses_the_corrected_remainder(self):
        profile = BENCHMARK.DoubleLaneChangeProfile()
        path = VirtualAPath(starting_pose(), profile, research_model())
        budget = path.integration_budget
        self.assertEqual(len(budget["panels_per_segment"]), 2)
        self.assertEqual(len(budget["initial_panels_per_segment"]), 2)
        self.assertTrue(all(n % 2 == 0 for n in budget["panels_per_segment"]))
        self.assertGreaterEqual(budget["refinement_steps"], 1)
        self.assertTrue(any(
            final > initial
            for final, initial in zip(budget["panels_per_segment"],
                                       budget["initial_panels_per_segment"])))
        self.assertEqual(budget["total_panels"],
                         sum(budget["panels_per_segment"]))
        self.assertEqual(budget["total_full_simpson_panels"],
                         sum(n // 2 for n in budget["panels_per_segment"]))
        self.assertEqual(budget["total_local_simpson_panels"], 2)
        for index, (s0, s1) in enumerate(profile.domain_segments):
            self.assertAlmostEqual(
                budget["simpson_panel_widths_m"][index],
                2.0 * (s1 - s0) / budget["panels_per_segment"][index],
                delta=1e-14)
            self.assertAlmostEqual(
                budget["subinterval_widths_m"][index],
                (s1 - s0) / budget["panels_per_segment"][index],
                delta=1e-14)
        expected_guard = 1e-10 * (
            budget["total_full_simpson_panels"]
            + budget["total_local_simpson_panels"] + 8)
        expected_guard += profile.length_m * budget["angle_error_bound_rad"]
        self.assertGreaterEqual(budget["floating_point_guard_m"],
                                expected_guard * (1.0 - 1e-12))
        self.assertLessEqual(budget["floating_point_guard_m"],
                             expected_guard * (1.0 + 1e-12))
        self.assertLessEqual(path.position_error_bound_m, 1e-7)

    def test_bound_unconfirmed_when_budget_cannot_be_met(self):
        original = bounds_module.POSITION_BUDGET_M
        try:
            bounds_module.POSITION_BUDGET_M = 0.0
            with self.assertRaises(VirtualAContractError) as caught:
                VirtualAPath(starting_pose(),
                             BENCHMARK.DoubleLaneChangeProfile(),
                             research_model())
        finally:
            bounds_module.POSITION_BUDGET_M = original
        self.assertEqual(caught.exception.reason, "BOUND_UNCONFIRMED")
        self.assertIn("total_panels", caught.exception.location)


if __name__ == "__main__":
    unittest.main()