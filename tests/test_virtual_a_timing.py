"""G3-A common-time quintic, inverse map and duration-selection tests.

The tests use one rest-to-rest quintic for the complete path.  They check the
time-law identities and inverse map independently, then exercise the public
retimer on a straight path and the exact NO_MOTION hold.  Dynamic whole-domain
counterexamples and the independent dynamic audit belong to G3-B.
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
from fwis_v3_optimizer.formation_continuous_a import timing as timing_module
from fwis_v3_optimizer.formation_continuous_a.timing import (
    progress_is_strictly_increasing,
    quintic_derivatives,
    quintic_progress,
    retime_path,
    tau_at_fraction,
)

ROOT = Path(__file__).resolve().parents[1]
FIXTURE_DIR = ROOT / "tests" / "fixtures" / "virtual_a"
REQUESTS = json.loads((FIXTURE_DIR / "open_requests.json").read_text())
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


def request_for(path, goal=None, policy="FREE", **changes):
    record = REQUESTS["requests"]["STRAIGHT_LINE"]
    goal = path.endpoint() if goal is None else goal
    values = {
        "plan_id": record["plan_id"],
        "formation_id": record["formation_id"],
        "model": model_from_fixture(),
        "vehicle_shapes": shapes_from_fixture(),
        "start_qf": FormationPose(**BASE["start_qf"]),
        "start_headings": {
            int(key): float(value)
            for key, value in BASE["start_headings"].items()},
        "goal_qf": FormationPose(goal.x, goal.y, goal.psi),
        "initial": InitialAssumption(**BASE["initial"]),
        "goal_policy": GoalPolicy(policy),
        "limits": limits_from_fixture(),
    }
    values.update(changes)
    return VirtualARequest(**values)


def bind(path, request):
    path.request_sha256 = request_fingerprint(request)
    return path


def straight_path(length=5.0):
    profile = C2CurvatureProfile.from_knots([
        ControlKnot(0.0, 0.0, 0.0), ControlKnot(length, 0.0, 0.0)])
    return VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile,
                        model_from_fixture())


def unit_quintic_derivatives(tau):
    """Independent textbook expressions for the common time law."""
    h = 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5
    h1 = 30.0 * tau ** 2 * (1.0 - tau) ** 2
    h2 = 60.0 * tau - 180.0 * tau ** 2 + 120.0 * tau ** 3
    h3 = 60.0 - 360.0 * tau + 360.0 * tau ** 2
    return h, h1, h2, h3


class QuinticLawTests(unittest.TestCase):

    def test_quintic_progress_and_derivative_identities(self):
        for tau in (0.0, 1e-6, 0.01, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99,
                    1.0 - 1e-6, 1.0):
            with self.subTest(tau=tau):
                h, h1, h2, h3 = unit_quintic_derivatives(tau)
                actual = quintic_progress(tau)
                derivatives = quintic_derivatives(tau)
                for expected, value in zip((h, h1, h2, h3), derivatives):
                    self.assertLessEqual(abs(expected - value),
                                         1e-13 * max(1.0, abs(expected)))
                self.assertLessEqual(abs(actual - h), 1e-13)
                self.assertLessEqual(abs((1.0 - h) - quintic_progress(
                    1.0 - tau)), 5e-15)
        self.assertEqual(quintic_progress(0.0), 0.0)
        self.assertEqual(quintic_progress(1.0), 1.0)

    def test_h1_is_strictly_positive_inside_and_zero_at_both_ends(self):
        for tau in (0.0, 1.0):
            self.assertEqual(quintic_derivatives(tau)[1], 0.0)
            self.assertFalse(progress_is_strictly_increasing(tau))
        samples = (1e-9, 1e-4, 0.01, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99,
                   1.0 - 1e-9)
        for tau in samples:
            self.assertGreater(quintic_derivatives(tau)[1], 0.0)
            self.assertTrue(progress_is_strictly_increasing(tau))

    def test_derivatives_match_centered_differences(self):
        step = 1e-6
        for tau in (0.01, 0.1, 0.3, 0.5, 0.7, 0.9, 0.99):
            with self.subTest(tau=tau):
                _, h1, h2, _ = unit_quintic_derivatives(tau)
                first = (quintic_progress(tau + step)
                         - quintic_progress(tau - step)) / (2.0 * step)
                self.assertLessEqual(abs(first - h1), 2e-9)
                # A second difference needs a larger step: at 1e-6 the
                # cancellation of ~unit progress values costs ~1e-4.
                coarse = 1e-4
                second = (quintic_progress(tau + coarse)
                          - 2.0 * quintic_progress(tau)
                          + quintic_progress(tau - coarse)) / (coarse * coarse)
                self.assertLessEqual(abs(second - h2), 2e-5)

    def test_inverse_map_at_one_hundred_nonuniform_fractions(self):
        fractions = [1e-9] + [0.5 - 0.5 * math.cos(
            math.pi * (index + 0.5) / 100.0) for index in range(100)]
        fractions.append(1.0 - 1e-9)
        previous = -1.0
        for fraction in fractions:
            tau = tau_at_fraction(fraction)
            residual = abs(quintic_progress(tau) - fraction)
            self.assertLessEqual(residual, 1e-14, (fraction, tau, residual))
            self.assertGreaterEqual(tau, previous)
            previous = tau
        self.assertEqual(tau_at_fraction(0.0), 0.0)
        self.assertEqual(tau_at_fraction(1.0), 1.0)

    def test_inverse_rejects_out_of_range_fraction(self):
        from fwis_v3_optimizer.formation_continuous_a.contracts import (
            VirtualAContractError)
        for fraction in (-1e-12, 1.0 + 1e-12):
            with self.assertRaises(VirtualAContractError):
                tau_at_fraction(fraction)


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


class IntervalBoundCoverageTests(unittest.TestCase):
    """The planner's interval enclosure must contain independent point values.

    The production interval implementation is compared with handwritten
    closed-form spatial and jerk formulas, never with the audit's copy of the
    same interval code.
    """

    def test_degenerate_interval_contains_codex_counterexample(self):
        offset = VehicleOffset(1, 1.0, 0.0)
        jets = {"k": (0.1, 0.1), "eta": (0.1, 0.1),
                "u": (0.0, 0.0), "g": (0.0, 0.0)}
        result = timing_module._vehicle_bounds(
            jets, offset, limits_from_fixture(),
            (1.0, 1.0), (0.0, 0.0), (0.0, 0.0), 0.247, 0.182)
        expected_ps_sss = (-0.04, 0.099)
        expected_norm = math.hypot(*expected_ps_sss)
        self.assertEqual(expected_norm, 0.10677546534667971)
        assert_interval_contains(
            self, result["p_sss_local_interval"][0],
            expected_ps_sss[0], "p_sss.x")
        assert_interval_contains(
            self, result["p_sss_local_interval"][1],
            expected_ps_sss[1], "p_sss.y")
        self.assertGreaterEqual(
            result["p_sss_norm_upper"], expected_norm)
        self.assertGreaterEqual(result["jerk"], expected_norm)
        self.assertGreaterEqual(
            result["jerk_component_norm_upper"], expected_norm)
        for axis in (0, 1):
            assert_interval_contains(
                self, result["jerk_local_interval"][axis],
                expected_ps_sss[axis], "jerk[{}]".format(axis))

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
                    bounds_record = timing_module._vehicle_bounds(
                        jets, offset, limits_from_fixture(),
                        w_interval, a_interval, j_interval,
                        0.247, 0.182)
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
                            bounds_record["p_sss_local_interval"][axis],
                            p_sss[axis], "p_sss[{}]".format(axis))
                        assert_interval_contains(
                            self,
                            bounds_record["jerk_local_interval"][axis],
                            actual_jerk[axis], "jerk[{}]".format(axis))
                    self.assertGreaterEqual(
                        bounds_record["p_sss_norm_upper"],
                        math.hypot(*p_sss))
                    self.assertGreaterEqual(
                        bounds_record["p_sss_component_upper"][0],
                        abs(p_sss[0]))
                    self.assertGreaterEqual(
                        bounds_record["p_sss_component_upper"][1],
                        abs(p_sss[1]))
                    self.assertGreaterEqual(
                        bounds_record["jerk_component_upper"][0],
                        abs(actual_jerk[0]))
                    self.assertGreaterEqual(
                        bounds_record["jerk_component_upper"][1],
                        abs(actual_jerk[1]))
                    self.assertGreaterEqual(
                        bounds_record["jerk_component_norm_upper"],
                        math.hypot(*actual_jerk))
                    self.assertGreaterEqual(
                        bounds_record["jerk"],
                        math.hypot(*actual_jerk))


class StraightRetimingTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.path = straight_path(5.0)
        cls.request = request_for(cls.path)
        bind(cls.path, cls.request)
        cls.result = retime_path(cls.request, cls.path, TimingConfig())

    def test_straight_line_selects_speed_limited_duration(self):
        result = self.result
        self.assertEqual(result.status, "DYNAMIC_FEASIBLE", result.as_dict())
        self.assertTrue(result.feasible)
        self.assertEqual(result.rescalings, 0)
        self.assertIsNotNone(result.plan)
        expected_required = 5.0 * 1.875 / 0.8
        self.assertAlmostEqual(
            result.bounds_summary["required_duration_s"],
            expected_required, delta=1e-12)
        self.assertAlmostEqual(
            result.bounds_summary["selected_duration_s"],
            expected_required * 1.01, delta=1e-12)
        self.assertAlmostEqual(result.duration_s,
                               expected_required * 1.01, delta=1e-12)
        worst = result.bounds_summary["worst"]
        self.assertEqual(worst["quantity"], "speed")
        self.assertEqual(worst["order"], 1)
        self.assertIn(worst["vehicle_id"], (1, 2, 3))
        self.assertEqual(worst["wheel_index"], 0)
        self.assertEqual(result.plan.total_S_m, 5.0)
        self.assertEqual(result.plan.duration_s, result.duration_s)
        self.assertEqual(result.plan.kind, "motion")

    def test_straight_scaling_records_all_metric_orders(self):
        metrics = self.result.bounds_summary["metrics_at_T0_s"]
        expected_orders = {
            "speed": 1, "acceleration": 2, "jerk": 3,
            "yaw_rate": 1, "yaw_acceleration": 2, "yaw_jerk": 3,
            "lateral_acceleration": 2, "wheel_speed": 1,
            "steer_rate": 1, "steer_acceleration": 2}
        self.assertEqual(set(metrics), set(expected_orders))
        for name, order in expected_orders.items():
            record = metrics[name]
            self.assertEqual(record["order"], order)
            self.assertGreaterEqual(record["upper_at_T0_s"], 0.0)
            self.assertGreater(record["limit"], 0.0)
            expected = (record["upper_at_T0_s"]
                        / record["limit"]) ** (1.0 / order)
            self.assertAlmostEqual(record["required_duration_s"],
                                   expected, delta=1e-15)
        # At T0 the largest scaled speed candidate is the binding one.
        self.assertEqual(
            self.result.bounds_summary["worst"]["quantity"], "speed")

    def test_two_second_selection_changes_only_the_common_time(self):
        fast = retime_path(self.request, self.path,
                           TimingConfig(safety_factor=2.0))
        self.assertEqual(fast.status, "DYNAMIC_FEASIBLE")
        self.assertAlmostEqual(
            fast.T_s, 2.0 * self.result.bounds_summary["required_duration_s"],
            delta=1e-9)
        # Arc-length parameterisation and path digest do not depend on T.
        self.assertEqual(fast.plan.total_S_m, self.result.plan.total_S_m)
        self.assertEqual(fast.plan.request_sha256,
                         self.result.plan.request_sha256)
        self.assertAlmostEqual(
            fast.bounds_summary["required_duration_s"],
            self.result.bounds_summary["required_duration_s"], delta=1e-12)


class HoldRetimingTests(unittest.TestCase):

    def test_no_motion_is_exact_hold(self):
        model = model_from_fixture()
        path = VirtualAPath.hold(FormationPose(0.0, 0.0, 0.0), model)
        request = request_for(path, goal=FormationPose(0.0, 0.0, 0.0))
        bind(path, request)
        result = retime_path(request, path, TimingConfig())
        self.assertEqual(result.status, "NO_MOTION", result.as_dict())
        self.assertTrue(result.feasible)
        self.assertEqual(result.T_s, 0.0)
        self.assertEqual(result.total_S_m, 0.0)
        plan = result.plan
        self.assertEqual(plan.duration_s, 0.0)
        self.assertEqual(plan.total_S_m, 0.0)
        state = plan.state_at(0.0)
        self.assertEqual(state.t, 0.0)
        self.assertEqual(state.s, 0.0)
        self.assertEqual(state.w, 0.0)
        self.assertEqual(state.a, 0.0)
        self.assertEqual(state.j, 0.0)
        self.assertEqual(state.region, "initial_hold")
        for vehicle in state.vehicles:
            self.assertEqual(vehicle.v_i, 0.0)
            self.assertEqual(vehicle.omega_i, 0.0)
            self.assertEqual(vehicle.velocity, (0.0, 0.0))
            self.assertEqual(vehicle.acceleration, (0.0, 0.0))
            self.assertEqual(vehicle.jerk, (0.0, 0.0))
            self.assertEqual(vehicle.wheel_speeds, (0.0, 0.0, 0.0, 0.0))
            self.assertEqual(vehicle.steer_rates, (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(plan.time_at_s(0.0), 0.0)
        projection = plan.sample([0.0])
        self.assertEqual(projection.times, (0.0,))
        self.assertEqual(len(projection.states), 1)
        self.assertFalse(projection.authoritative)

    def test_hold_rejects_a_different_goal(self):
        path = VirtualAPath.hold(FormationPose(0.0, 0.0, 0.0),
                                 model_from_fixture())
        request = request_for(path, goal=FormationPose(1e-12, 0.0, 0.0))
        bind(path, request)
        result = retime_path(request, path, TimingConfig())
        self.assertIn(result.status, ("INVALID_PATH", "GEOMETRY_REJECTED"))
        self.assertFalse(result.feasible)
        self.assertIsNone(result.plan)


class DurationSelectionTests(unittest.TestCase):

    def test_longer_straight_path_requires_proportional_common_time(self):
        requests = []
        for length in (5.0, 10.0):
            path = straight_path(length)
            request = request_for(path)
            bind(path, request)
            result = retime_path(request, path, TimingConfig())
            self.assertEqual(result.status, "DYNAMIC_FEASIBLE")
            self.assertEqual(result.rescalings, 0)
            requests.append((path, request, result))
        short, long = requests[0][2], requests[1][2]
        self.assertAlmostEqual(long.T_s, 2.0 * short.T_s, delta=1e-9)
        self.assertAlmostEqual(
            long.bounds_summary["required_duration_s"],
            2.0 * short.bounds_summary["required_duration_s"], delta=1e-9)
        self.assertEqual(short.bounds_summary["worst"]["quantity"], "speed")
        self.assertEqual(long.bounds_summary["worst"]["quantity"], "speed")

    def test_duration_budget_is_a_hard_gate(self):
        path = straight_path(5.0)
        request = request_for(path)
        bind(path, request)
        result = retime_path(
            request, path, TimingConfig(max_total_duration_s=1.0))
        self.assertEqual(result.status, "TIMING_BUDGET_EXCEEDED",
                         result.as_dict())
        self.assertFalse(result.feasible)
        self.assertIsNone(result.plan)
        self.assertGreater(result.duration_s, 1.0)

    def test_rescale_configuration_cap_is_enforced(self):
        path = straight_path(5.0)
        request = request_for(path)
        bind(path, request)
        result = retime_path(
            request, path, TimingConfig(max_rescalings=9))
        self.assertEqual(result.status, "INVALID_TIMING_CONFIG",
                         result.as_dict())
        self.assertFalse(result.feasible)
        self.assertIsNone(result.plan)

    def test_rescale_loop_is_recorded_in_the_plan(self):
        path = straight_path(5.0)
        request = request_for(path)
        bind(path, request)
        baseline = retime_path(request, path, TimingConfig())
        self.assertEqual(baseline.status, "DYNAMIC_FEASIBLE")
        real_certify = timing_module._certify_bounds
        calls = {"count": 0}

        def forced_failure(*args, **kwargs):
            calls["count"] += 1
            if calls["count"] <= 2:
                raise timing_module._BoundFailure(
                    "DYNAMIC_LIMIT", "test-forced rescale",
                    constraint="DYNAMIC_LIMIT")
            return real_certify(*args, **kwargs)

        with mock.patch.object(timing_module, "_certify_bounds",
                               side_effect=forced_failure):
            result = retime_path(
                request, path, TimingConfig(max_rescalings=8))
        self.assertEqual(result.status, "DYNAMIC_FEASIBLE")
        self.assertEqual(result.rescalings, 2)
        expected = (baseline.bounds_summary["required_duration_s"] * 1.01
                    * 1.05 * 1.05)
        self.assertAlmostEqual(result.duration_s, expected, delta=1e-9)
        selection = result.plan.to_model_dict()["timing"]["selection"]
        self.assertEqual(selection["rescalings"], 2)
        self.assertAlmostEqual(selection["selected_duration_s"],
                               expected, delta=1e-9)

    def test_exhausted_rescales_never_publish_a_feasible_plan(self):
        path = straight_path(5.0)
        request = request_for(path)
        bind(path, request)

        def always_fail(*args, **kwargs):
            raise timing_module._BoundFailure(
                "DYNAMIC_LIMIT", "test-forced rescale",
                constraint="DYNAMIC_LIMIT")

        with mock.patch.object(timing_module, "_certify_bounds",
                               side_effect=always_fail):
            result = retime_path(
                request, path, TimingConfig(max_rescalings=1))
        self.assertEqual(result.status, "BOUND_UNCONFIRMED", result.as_dict())
        self.assertFalse(result.feasible)
        self.assertIsNone(result.plan)


class GeometryGateTests(unittest.TestCase):

    def test_rejected_geometry_never_yields_a_timed_plan(self):
        # Curvature 2 1/m violates the requested 0.81 m Ackermann radius.
        profile = C2CurvatureProfile.from_knots([
            ControlKnot(0.0, 2.0, 0.0),
            ControlKnot(4.0, 2.0, 0.0)])
        path = VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile,
                            model_from_fixture())
        request = request_for(path)
        bind(path, request)
        result = retime_path(request, path, TimingConfig())
        self.assertEqual(result.status, "GEOMETRY_REJECTED",
                         result.as_dict())
        self.assertIsNone(result.plan)
        self.assertFalse(result.feasible)
        self.assertIsNotNone(result.geometry_audit)
        self.assertFalse(result.geometry_audit.passed)


if __name__ == "__main__":
    unittest.main()