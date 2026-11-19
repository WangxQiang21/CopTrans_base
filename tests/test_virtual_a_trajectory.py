"""G3-A pointwise common-time trajectory tests.

The common time law is checked with the textbook quintic written independently
in this file.  A curved C2 profile with genuine ``g`` jumps is used for
one-sided node checks and ideal-wheel crosschecks.  These tests do not run the
independent whole-domain dynamic audit (G3-B).
"""
import json
import math
from pathlib import Path
import unittest

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
    VirtualAContractError,
    VirtualARequest,
    request_fingerprint,
)
from fwis_v3_optimizer.formation_continuous_a.geometry import (
    C2CurvatureProfile,
    VirtualAPath,
)
from fwis_v3_optimizer.formation_continuous_a.timing import retime_path
from fwis_v3_optimizer.formation_continuous_a.trajectory import (
    VirtualATimedPlan,
)
from fwis_v3_optimizer.formation_continuous_a import (
    bounds as bounds_module,
)

ROOT = Path(__file__).resolve().parents[1]
FIXTURE_DIR = ROOT / "tests" / "fixtures" / "virtual_a"
REQUESTS = json.loads((FIXTURE_DIR / "open_requests.json").read_text())
BASE = REQUESTS["base"]
WHEEL_COORDINATES = ((0.5, 0.5), (0.5, -0.5), (-0.5, 0.5), (-0.5, -0.5))


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


def request_for(path, goal=None, policy="FREE"):
    record = REQUESTS["requests"]["STRAIGHT_LINE"]
    goal = path.endpoint() if goal is None else goal
    return VirtualARequest(
        plan_id=record["plan_id"],
        formation_id=record["formation_id"],
        model=model_from_fixture(),
        vehicle_shapes=shapes_from_fixture(),
        start_qf=FormationPose(**BASE["start_qf"]),
        start_headings={
            int(key): float(value)
            for key, value in BASE["start_headings"].items()},
        goal_qf=FormationPose(goal.x, goal.y, goal.psi),
        initial=InitialAssumption(**BASE["initial"]),
        goal_policy=GoalPolicy(policy),
        limits=limits_from_fixture())


def bind(path, request):
    path.request_sha256 = request_fingerprint(request)
    return path


def straight_request_and_path(length=5.0):
    profile = C2CurvatureProfile.from_knots([
        ControlKnot(0.0, 0.0, 0.0), ControlKnot(length, 0.0, 0.0)])
    path = VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile,
                        model_from_fixture())
    request = request_for(path)
    bind(path, request)
    return request, path


def curved_request_and_path():
    """C2 knots whose ``g`` jumps at both internal nodes.

    The profile is ``0.1 * k(s)`` for the analytically integrated piecewise
    quadratic curvature: ``g`` is 0.06, -0.03 and 0.04, while ``k``, ``eta``
    and ``u`` are continuous.  The goal is FREE, so the nonzero terminal
    curvature/heading/wheel angle must be preserved by the terminal hold.
    """
    factor = 0.1
    records = (
        (0.0, 0.0, 0.0),
        (1.0, factor * 0.20, factor * 0.50),
        (2.0, factor * 1.05, factor * 1.15),
        (3.0, factor * 2.5166666666666666, factor * 1.85))
    profile = C2CurvatureProfile.from_knots([
        ControlKnot(s, k, eta) for s, k, eta in records])
    path = VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile,
                        model_from_fixture())
    request = request_for(path)
    bind(path, request)
    return request, path


def independent_quintic(tau):
    h = 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5
    h1 = 30.0 * tau ** 2 * (1.0 - tau) ** 2
    h2 = 60.0 * tau - 180.0 * tau ** 2 + 120.0 * tau ** 3
    h3 = 60.0 - 360.0 * tau + 360.0 * tau ** 2
    return h, h1, h2, h3


def independent_inverse(fraction, iterations=160):
    low, high = 0.0, 1.0
    for _ in range(iterations):
        middle = 0.5 * (low + high)
        h = 10.0 * middle ** 3 - 15.0 * middle ** 4 \
            + 6.0 * middle ** 5
        if h < fraction:
            low = middle
        else:
            high = middle
    return 0.5 * (low + high)


def nonuniform_taus(count=100):
    return [0.5 - 0.5 * math.cos(math.pi * (index + 0.5) / count)
            for index in range(count)]


def independent_wheels(vehicle, limits, v, w, a):
    angles, speeds, rates, accels = [], [], [], []
    C = vehicle.C_i if hasattr(vehicle, "C_i") else vehicle.curvature
    C_s = getattr(vehicle, "curvature_s")
    C_ss = getattr(vehicle, "curvature_ss")
    for xw_factor, yw_factor in WHEEL_COORDINATES:
        xw = xw_factor * limits.wheelbase
        yw = yw_factor * limits.track
        m = 1.0 - yw * C
        n = xw * C
        Ew = m * m + n * n
        Ew_C = -2.0 * yw + 2.0 * (xw * xw + yw * yw) * C
        delta_s = xw / Ew * C_s
        delta_ss = xw / Ew * C_ss - xw * Ew_C / (Ew * Ew) * C_s * C_s
        angles.append(math.atan2(n, m))
        speeds.append(math.sqrt(Ew) * v)
        rates.append(delta_s * w)
        accels.append(delta_ss * w * w + delta_s * a)
    return tuple(angles), tuple(speeds), tuple(rates), tuple(accels)


def assert_vector_close(test, actual, expected, tolerance, message):
    test.assertEqual(len(actual), len(expected), message)
    for index, (actual_value, expected_value) in enumerate(
            zip(actual, expected)):
        test.assertLessEqual(
            abs(actual_value - expected_value),
            tolerance * max(1.0, abs(expected_value)),
            "{}[{}]: {!r} vs {!r}".format(
                message, index, actual_value, expected_value))


class PointwiseStraightTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.request, cls.path = straight_request_and_path(5.0)
        cls.result = retime_path(cls.request, cls.path, TimingConfig())
        cls.plan = cls.result.plan

    def test_straight_pointwise_time_law_and_translation(self):
        self.assertEqual(self.result.status, "DYNAMIC_FEASIBLE")
        plan = self.plan
        S, T = plan.total_S_m, plan.duration_s
        for tau in nonuniform_taus():
            t = T * tau
            h, h1, h2, h3 = independent_quintic(tau)
            state = plan.state_at(t)
            self.assertEqual(state.t, t)
            self.assertLessEqual(abs(state.s - S * h),
                                 1e-14 * max(1.0, S))
            self.assertLessEqual(abs(state.w - S * h1 / T),
                                 1e-14 * max(1.0, S / T))
            self.assertLessEqual(abs(state.a - S * h2 / (T * T)),
                                 1e-13 * max(1.0, S / (T * T)))
            self.assertLessEqual(abs(state.j - S * h3 / (T ** 3)),
                                 1e-12 * max(1.0, S / (T ** 3)))
            self.assertLessEqual(abs(state.qf.x - state.s), 1e-14)
            self.assertLessEqual(abs(state.qf.y), 1e-14)
            self.assertLessEqual(abs(state.qf.psi), 1e-14)
            for vehicle in state.vehicles:
                self.assertEqual(vehicle.theta, 0.0)
                self.assertEqual(vehicle.C_i, 0.0)
                self.assertLessEqual(abs(vehicle.v_i - state.w), 1e-14)
                self.assertEqual(vehicle.omega_i, 0.0)
                self.assertEqual(vehicle.wheel_angles, (0.0, 0.0, 0.0, 0.0))
                assert_vector_close(
                    self, vehicle.wheel_speeds,
                    (state.w, state.w, state.w, state.w), 1e-14,
                    "straight wheel speeds")
                self.assertEqual(vehicle.steer_rates,
                                 (0.0, 0.0, 0.0, 0.0))
                self.assertEqual(vehicle.steer_accels,
                                 (0.0, 0.0, 0.0, 0.0))
                self.assertEqual(vehicle.lateral_accel, 0.0)
                assert_vector_close(
                    self, vehicle.velocity, (state.w, 0.0), 1e-14,
                    "straight velocity")
                assert_vector_close(
                    self, vehicle.acceleration, (state.a, 0.0), 1e-14,
                    "straight acceleration")
                assert_vector_close(
                    self, vehicle.jerk, (state.j, 0.0), 1e-12,
                    "straight jerk")

    def test_straight_vehicle_centres_keep_fixed_offsets(self):
        state = self.plan.state_at(1.234)
        by_id = {vehicle.vehicle_id: vehicle for vehicle in state.vehicles}
        for offset in self.request.model.offsets:
            vehicle = by_id[offset.vehicle_id]
            self.assertLessEqual(
                abs(vehicle.x - (state.qf.x + offset.x)), 1e-13)
            self.assertLessEqual(
                abs(vehicle.y - (state.qf.y + offset.y)), 1e-13)

    def test_endpoints_are_stationary_rest_to_rest(self):
        start = self.plan.state_at(0.0, "right")
        end = self.plan.state_at(self.plan.duration_s, "left")
        self.assertEqual(start.w, 0.0)
        self.assertEqual(start.a, 0.0)
        self.assertNotEqual(start.j, 0.0)
        self.assertEqual(end.w, 0.0)
        self.assertEqual(end.a, 0.0)
        self.assertNotEqual(end.j, 0.0)
        for vehicle in start.vehicles + end.vehicles:
            self.assertEqual(vehicle.velocity, (0.0, 0.0))
            self.assertEqual(vehicle.acceleration, (0.0, 0.0))
            self.assertEqual(vehicle.omega_i, 0.0)


class CurvedPointwiseTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.request, cls.path = curved_request_and_path()
        cls.result = retime_path(cls.request, cls.path, TimingConfig())
        cls.plan = cls.result.plan
        cls.stations = [piece[1] for piece in
                        cls.path.profile.domain_segments[:-1]]

    def test_time_law_composition_matches_spatial_jets(self):
        self.assertEqual(self.result.status, "DYNAMIC_FEASIBLE")
        plan = self.plan
        S, T = plan.total_S_m, plan.duration_s
        limits = self.request.limits
        checked = 0
        for tau in nonuniform_taus():
            h, h1, h2, h3 = independent_quintic(tau)
            station = S * h
            if any(abs(station - node) < 1e-9 for node in self.stations):
                continue
            t = T * tau
            state = plan.state_at(t)
            spatial = self.path.state_at_s(station)
            self.assertLessEqual(abs(state.s - station), 1e-13)
            self.assertLessEqual(abs(state.qf.x - spatial.qf.x), 1e-12)
            self.assertLessEqual(abs(state.qf.y - spatial.qf.y), 1e-12)
            self.assertLessEqual(abs(state.qf.psi - spatial.qf.psi), 1e-12)
            self.assertLessEqual(abs(state.w - S * h1 / T),
                                 1e-14 * max(1.0, S / T))
            self.assertLessEqual(abs(state.a - S * h2 / (T * T)),
                                 1e-13 * max(1.0, S / (T * T)))
            self.assertLessEqual(abs(state.j - S * h3 / (T ** 3)),
                                 1e-12 * max(1.0, S / (T ** 3)))
            for spatial_vehicle, vehicle in zip(
                    spatial.vehicles, state.vehicles):
                self.assertEqual(vehicle.vehicle_id,
                                 spatial_vehicle.vehicle_id)
                C = spatial_vehicle.curvature
                eta = spatial.curvature.eta
                aoff = self.request.model.offsets[
                    vehicle.vehicle_id - 1].x
                boff = self.request.model.offsets[
                    vehicle.vehicle_id - 1].y
                kappa = spatial.curvature.kappa
                A = 1.0 - boff * kappa
                B = aoff * kappa
                D = A * A + B * B
                Q = math.sqrt(D)
                H = kappa + aoff * eta / D
                self.assertLessEqual(abs(vehicle.C_i - C), 1e-14)
                self.assertLessEqual(abs(vehicle.theta
                                         - spatial_vehicle.theta), 1e-14)
                self.assertLessEqual(abs(vehicle.v_i - Q * state.w),
                                     1e-13 * max(1.0, Q * state.w))
                self.assertLessEqual(
                    abs(vehicle.omega_i - H * state.w),
                    1e-12 * max(1.0, abs(H * state.w)))
                p_s, p_ss, p_sss = (spatial_vehicle.p_s,
                                    spatial_vehicle.p_ss,
                                    spatial_vehicle.p_sss)
                w, a, j = state.w, state.a, state.j
                expected_velocity = (p_s[0] * w, p_s[1] * w)
                expected_acceleration = (
                    p_ss[0] * w * w + p_s[0] * a,
                    p_ss[1] * w * w + p_s[1] * a)
                expected_jerk = (
                    p_sss[0] * w ** 3 + 3.0 * p_ss[0] * w * a
                    + p_s[0] * j,
                    p_sss[1] * w ** 3 + 3.0 * p_ss[1] * w * a
                    + p_s[1] * j)
                expected_yaw_accel = (spatial_vehicle.theta_s * a
                                      + spatial_vehicle.theta_ss * w * w)
                expected_yaw_jerk = (
                    spatial_vehicle.theta_s * j
                    + 3.0 * spatial_vehicle.theta_ss * w * a
                    + spatial_vehicle.theta_sss * w ** 3)
                assert_vector_close(
                    self, vehicle.velocity, expected_velocity, 1e-13,
                    "curved velocity")
                assert_vector_close(
                    self, vehicle.acceleration, expected_acceleration,
                    1e-12, "curved acceleration")
                assert_vector_close(
                    self, vehicle.jerk, expected_jerk, 1e-11,
                    "curved jerk")
                self.assertLessEqual(
                    abs(vehicle.yaw_accel - expected_yaw_accel),
                    1e-12 * max(1.0, abs(expected_yaw_accel)))
                self.assertLessEqual(
                    abs(vehicle.yaw_jerk - expected_yaw_jerk),
                    1e-10 * max(1.0, abs(expected_yaw_jerk)))
                self.assertLessEqual(
                    abs(vehicle.lateral_accel
                        - abs(C) * (Q * w) ** 2),
                    1e-12 * max(1.0, abs(C) * (Q * w) ** 2))
                angles, speeds, rates, accels = independent_wheels(
                    spatial_vehicle, limits, Q * w, w, a)
                assert_vector_close(
                    self, vehicle.wheel_angles, angles, 1e-13,
                    "wheel angles")
                assert_vector_close(
                    self, vehicle.wheel_speeds, speeds, 1e-13,
                    "wheel speeds")
                assert_vector_close(
                    self, vehicle.steer_rates, rates, 1e-13,
                    "steer rates")
                assert_vector_close(
                    self, vehicle.steer_accels, accels, 1e-12,
                    "steer accels")
            checked += 1
        self.assertGreaterEqual(checked, 95)

    def test_internal_nodes_are_motion_with_one_sided_limits(self):
        plan = self.plan
        S, T = plan.total_S_m, plan.duration_s
        self.assertEqual(len(self.stations), 2)
        for index, station in enumerate(self.stations):
            with self.subTest(node=index):
                tau = independent_inverse(station / S)
                t = T * tau
                left = plan.state_at(t, "left")
                right = plan.state_at(t, "right")
                auto = plan.state_at(t)
                self.assertEqual(left.region, "motion")
                self.assertEqual(right.region, "motion")
                self.assertEqual(auto.region, "motion")
                self.assertLessEqual(abs(left.s - station), 1e-12)
                self.assertLessEqual(abs(right.s - station), 1e-12)
                self.assertGreater(left.w, 0.0)
                self.assertGreater(right.w, 0.0)
                self.assertEqual(auto.as_dict(), right.as_dict())
                left_spatial = self.path.state_at_s(station, "left")
                right_spatial = self.path.state_at_s(station, "right")
                for actual, spatial in (
                        (left, left_spatial), (right, right_spatial)):
                    self.assertLessEqual(
                        abs(actual.qf.x - spatial.qf.x), 1e-12)
                    self.assertLessEqual(
                        abs(actual.qf.y - spatial.qf.y), 1e-12)
                    self.assertLessEqual(
                        abs(actual.qf.psi - spatial.qf.psi), 1e-12)
                    for actual_vehicle, spatial_vehicle in zip(
                            actual.vehicles, spatial.vehicles):
                        self.assertLessEqual(
                            abs(actual_vehicle.C_i
                                - spatial_vehicle.curvature), 1e-14)
                        self.assertLessEqual(
                            abs(actual_vehicle.theta
                                - spatial_vehicle.theta), 1e-13)
                # Position, heading, curvature and velocity are continuous;
                # only third-derivative quantities may jump at a g node.
                node_jump = 0.0
                for left_vehicle, right_vehicle in zip(
                        left.vehicles, right.vehicles):
                    for name in ("x", "y", "theta", "C_i", "v_i", "omega_i",
                                 "yaw_accel", "lateral_accel"):
                        self.assertLessEqual(
                            abs(getattr(left_vehicle, name)
                                - getattr(right_vehicle, name)), 1e-12)
                    assert_vector_close(
                        self, left_vehicle.velocity,
                        right_vehicle.velocity, 1e-12,
                        "node velocity continuity")
                    assert_vector_close(
                        self, left_vehicle.acceleration,
                        right_vehicle.acceleration, 1e-12,
                        "node acceleration continuity")
                    assert_vector_close(
                        self, left_vehicle.wheel_angles,
                        right_vehicle.wheel_angles, 1e-12,
                        "node wheel-angle continuity")
                    assert_vector_close(
                        self, left_vehicle.wheel_speeds,
                        right_vehicle.wheel_speeds, 1e-12,
                        "node wheel-speed continuity")
                    assert_vector_close(
                        self, left_vehicle.steer_rates,
                        right_vehicle.steer_rates, 1e-12,
                        "node steer-rate continuity")
                    node_jump = max(
                        node_jump,
                        abs(left_vehicle.yaw_jerk - right_vehicle.yaw_jerk),
                        math.hypot(
                            left_vehicle.jerk[0] - right_vehicle.jerk[0],
                            left_vehicle.jerk[1] - right_vehicle.jerk[1]))
                # With aoff=0 vehicles 2/3 have H_ss independent of g, so the
                # g jump appears in the ID1 (aoff=1) yaw jerk.
                self.assertGreater(node_jump, 1e-8)

    def test_inverse_map_on_one_hundred_nonuniform_stations(self):
        plan = self.plan
        S, T = plan.total_S_m, plan.duration_s
        previous = -1.0
        for fraction in ([1e-6]
                         + [0.5 - 0.5 * math.cos(
                             math.pi * (index + 0.5) / 100.0)
                            for index in range(100)]
                         + [1.0 - 1e-6]):
            tau = independent_inverse(fraction)
            expected = T * tau
            actual = plan.time_at_s(S * fraction)
            self.assertLessEqual(abs(actual - expected), 1e-9,
                                 (fraction, actual, expected))
            self.assertGreater(actual, previous)
            previous = actual
        self.assertEqual(plan.time_at_s(0.0), 0.0)
        self.assertEqual(plan.time_at_s(S), T)

    def test_sample_is_the_same_pointwise_states(self):
        plan = self.plan
        T = plan.duration_s
        times = (0.0, T / 3.0, T / 2.0, T)
        projection = plan.sample(times)
        self.assertEqual(projection.times, times)
        self.assertFalse(projection.authoritative)
        self.assertEqual(projection.model_summary, plan.model_summary())
        for t, state in zip(times, projection.states):
            self.assertEqual(state.as_dict(), plan.state_at(t).as_dict())

    def test_terminal_hold_preserves_free_endpoint_state(self):
        plan = self.plan
        S, T = plan.total_S_m, plan.duration_s
        terminal_hold = plan.state_at(T, "right")
        terminal_motion = plan.state_at(T, "left")
        endpoint = self.path.state_at_s(S, "left")
        self.assertEqual(terminal_hold.region, "terminal_hold")
        self.assertEqual(terminal_motion.region, "motion")
        self.assertEqual(terminal_hold.qf, terminal_motion.qf)
        self.assertLessEqual(abs(terminal_hold.qf.x - endpoint.qf.x), 1e-12)
        self.assertLessEqual(abs(terminal_hold.qf.y - endpoint.qf.y), 1e-12)
        self.assertLessEqual(abs(terminal_hold.qf.psi
                                 - endpoint.qf.psi), 1e-12)
        for actual, spatial in zip(terminal_hold.vehicles,
                                   endpoint.vehicles):
            self.assertLessEqual(abs(actual.C_i - spatial.curvature), 1e-14)
            angles, speeds, rates, accels = independent_wheels(
                spatial, self.request.limits, 0.0, 0.0, 0.0)
            assert_vector_close(
                self, actual.wheel_angles, angles, 1e-14,
                "terminal free wheel angle")
            self.assertEqual(actual.v_i, 0.0)
            self.assertEqual(actual.omega_i, 0.0)
            self.assertEqual(actual.velocity, (0.0, 0.0))
            self.assertEqual(actual.acceleration, (0.0, 0.0))
            self.assertEqual(actual.jerk, (0.0, 0.0))
        self.assertGreater(
            max(abs(vehicle.C_i) for vehicle in terminal_hold.vehicles),
            1e-3)
        self.assertGreater(
            max(abs(angle) for vehicle in terminal_hold.vehicles
                for angle in vehicle.wheel_angles), 1e-3)

    def test_initial_hold_is_exactly_the_straight_start(self):
        plan = self.plan
        for t, side in ((-1.0, "left"), (-1e-9, "auto"),
                        (0.0, "left"), (0.0, "left")):
            state = plan.state_at(t, side)
            self.assertEqual(state.region, "initial_hold")
            self.assertEqual(state.t, t)
            self.assertEqual(state.s, 0.0)
            self.assertEqual(state.w, 0.0)
            self.assertEqual(state.a, 0.0)
            self.assertEqual(state.j, 0.0)
            self.assertEqual(state.qf, self.request.start_qf)
            for vehicle in state.vehicles:
                self.assertEqual(vehicle.C_i, 0.0)
                self.assertEqual(vehicle.theta, 0.0)
                self.assertEqual(vehicle.v_i, 0.0)
                self.assertEqual(vehicle.omega_i, 0.0)
                self.assertEqual(vehicle.velocity, (0.0, 0.0))
                self.assertEqual(vehicle.acceleration, (0.0, 0.0))
                self.assertEqual(vehicle.wheel_angles,
                                 (0.0, 0.0, 0.0, 0.0))
        motion_start = plan.state_at(0.0, "right")
        self.assertEqual(motion_start.region, "motion")
        self.assertEqual(motion_start.w, 0.0)
        self.assertEqual(motion_start.a, 0.0)
        self.assertNotEqual(motion_start.j, 0.0)
        self.assertEqual(plan.state_at(0.0).as_dict(),
                         motion_start.as_dict())

    def test_time_beyond_the_terminal_hold_does_not_rezero_geometry(self):
        plan = self.plan
        T = plan.duration_s
        state = plan.state_at(T + 5.0, "right")
        self.assertEqual(state.region, "terminal_hold")
        self.assertEqual(state.t, T + 5.0)
        self.assertEqual(state.s, plan.total_S_m)
        self.assertEqual(state.w, 0.0)
        self.assertEqual(state.a, 0.0)
        self.assertEqual(state.j, 0.0)
        for vehicle in state.vehicles:
            self.assertEqual(vehicle.velocity, (0.0, 0.0))
            self.assertGreater(abs(vehicle.C_i), 1e-3)

    def test_wheel_vectors_have_four_diagnostic_wheels(self):
        state = self.plan.state_at(self.plan.duration_s / 2.0)
        for vehicle in state.vehicles:
            for name in ("wheel_angles", "wheel_speeds", "steer_rates",
                         "steer_accels"):
                self.assertEqual(len(getattr(vehicle, name)), 4)
        payload = state.vehicles[0].as_dict()
        self.assertTrue(
            payload["ideal_wheel_values_are_diagnostics_not_commands"])


class NoMotionTrajectoryTests(unittest.TestCase):

    def test_hold_state_and_projection_are_exact(self):
        model = model_from_fixture()
        path = VirtualAPath.hold(FormationPose(0.0, 0.0, 0.0), model)
        request = request_for(path, goal=FormationPose(0.0, 0.0, 0.0))
        bind(path, request)
        result = retime_path(request, path, TimingConfig())
        plan = result.plan
        self.assertEqual(result.status, "NO_MOTION")
        state = plan.state_at(0.0)
        self.assertEqual(state.t, 0.0)
        self.assertEqual(state.region, "initial_hold")
        for t in (-2.0, 0.0, 3.0):
            state = plan.state_at(t)
            self.assertEqual(state.t, 0.0)
            self.assertEqual(state.s, 0.0)
            self.assertEqual(state.w, 0.0)
            self.assertEqual(state.a, 0.0)
            self.assertEqual(state.j, 0.0)
        self.assertEqual(plan.time_at_s(0.0), 0.0)
        projection = plan.sample([0.0])
        self.assertEqual(projection.times, (0.0,))
        self.assertFalse(projection.authoritative)
        for side in ("auto", "left", "right"):
            self.assertEqual(plan.state_at(0.0, side).as_dict(),
                             plan.state_at(0.0).as_dict())


def canonical_json(document):
    return json.dumps(document, sort_keys=True, separators=(",", ":"))


class SerializationIdentityTests(unittest.TestCase):
    """G3-C: canonical model identity and non-authoritative samples."""

    @classmethod
    def setUpClass(cls):
        cls.request, cls.path = curved_request_and_path()
        cls.result = retime_path(cls.request, cls.path, TimingConfig())
        cls.plan = cls.result.plan
        cls.payload = cls.plan.to_model_json()

    def _mutated(self, mutate):
        document = json.loads(self.payload)
        mutate(document)
        return canonical_json(document)

    def test_roundtrip_restores_the_exact_authoritative_model(self):
        plan = self.plan
        self.assertEqual(plan.to_model_json(), canonical_json(
            json.loads(plan.to_model_json())))
        self.assertEqual(len(plan.model_sha256()), 64)
        rebuilt = VirtualATimedPlan.from_model_json(
            self.payload, expected_request=self.request)
        self.assertEqual(rebuilt.to_model_json(), self.payload)
        self.assertEqual(rebuilt.model_sha256(), plan.model_sha256())
        self.assertEqual(rebuilt.model_summary(), plan.model_summary())
        T = plan.duration_s
        for t in (0.0, T / 7.0, T / 2.0, 0.999 * T, T):
            self.assertEqual(rebuilt.state_at(t).as_dict(),
                             plan.state_at(t).as_dict())
            self.assertEqual(rebuilt.state_at(t, "left").as_dict(),
                             plan.state_at(t, "left").as_dict())
            self.assertEqual(rebuilt.state_at(t, "right").as_dict(),
                             plan.state_at(t, "right").as_dict())
        for fraction in [index / 20.0 for index in range(21)]:
            station = plan.total_S_m * fraction
            self.assertEqual(rebuilt.time_at_s(station),
                             plan.time_at_s(station))
        self.assertEqual(rebuilt.to_model_json(), self.payload)

    def test_model_has_no_self_digest_audit_or_execution_claim(self):
        text = self.plan.to_model_json()
        for forbidden in ('"model_sha256"', '"dynamic_audit_passed"',
                          '"executable"', '"collision_checked"', '"audit"'):
            self.assertNotIn(forbidden, text)
        document = json.loads(text)
        self.assertNotIn("model_sha256", document)
        self.assertNotIn("executable", document)
        self.assertNotIn("collision_checked", document)
        self.assertNotIn("dynamic_audit_passed", document)

    def test_timing_and_selection_tamper_is_rejected(self):
        cases = (
            ("T", lambda d: d["timing"].__setitem__(
                "T_s", d["timing"]["T_s"] * 2.0),
             "MODEL_TIMING_INCONSISTENT"),
            ("selected", lambda d: d["timing"]["selection"].__setitem__(
                "selected_duration_s",
                d["timing"]["selection"]["selected_duration_s"] * 2.0),
             "MODEL_TIMING_INCONSISTENT"),
            ("required", lambda d: d["timing"]["selection"].__setitem__(
                "required_duration_s",
                d["timing"]["selection"]["required_duration_s"] * 2.0),
             "MODEL_TIMING_INCONSISTENT"),
            ("safety", lambda d: d["timing"]["selection"].__setitem__(
                "safety_factor", 0.5),
             "INVALID_MODEL_JSON"),
            ("rescalings", lambda d: d["timing"]["selection"].__setitem__(
                "rescalings", 9),
             "INVALID_MODEL_JSON"),
        )
        for name, mutate, expected in cases:
            with self.subTest(case=name):
                with self.assertRaises(VirtualAContractError) as caught:
                    VirtualATimedPlan.from_model_json(self._mutated(mutate))
                self.assertEqual(caught.exception.reason, expected)

    def test_coordinated_duration_tamper_is_rejected(self):
        # T, selected and required are edited together, so the scalar
        # consistency rules alone would accept them.  The stored metric table
        # is not scaled and therefore proves the selection inconsistent.
        def mutate(document):
            selection = document["timing"]["selection"]
            factor = 1.1
            document["timing"]["T_s"] *= factor
            selection["selected_duration_s"] *= factor
            selection["required_duration_s"] *= factor
        with self.assertRaises(VirtualAContractError) as caught:
            VirtualATimedPlan.from_model_json(self._mutated(mutate))
        self.assertEqual(caught.exception.reason,
                         "MODEL_TIMING_INCONSISTENT")

    def test_selection_metric_table_tamper_is_rejected(self):
        document = json.loads(self.payload)
        metrics = document["timing"]["selection"]["metrics_at_T0_s"]
        worst_name = document["timing"]["selection"]["worst"]["quantity"]
        other_name = min(
            (name for name in metrics if name != worst_name),
            key=lambda name: metrics[name]["required_duration_s"])

        def metric(name, key, value):
            def mutate(document):
                document["timing"]["selection"]["metrics_at_T0_s"][
                    name][key] = value
            return mutate

        cases = (
            ("coordinated_metric_order",
             metric("speed", "order", 2), "MODEL_TIMING_INCONSISTENT"),
            ("metric_upper",
             metric("speed", "upper_at_T0_s",
                    metrics["speed"]["upper_at_T0_s"] + 0.01),
             "MODEL_TIMING_INCONSISTENT"),
            ("metric_limit",
             metric("speed", "limit", metrics["speed"]["limit"] * 1.01),
             "MODEL_TIMING_INCONSISTENT"),
            ("metric_candidate",
             metric("speed", "required_duration_s",
                    metrics["speed"]["required_duration_s"] * 1.02),
             "MODEL_TIMING_INCONSISTENT"),
            ("metric_witness",
             metric("speed", "vehicle_id", 999),
             "INVALID_MODEL_JSON"),
            ("worst_quantity",
             lambda d: d["timing"]["selection"]["worst"].__setitem__(
                 "quantity", other_name), "MODEL_TIMING_INCONSISTENT"),
            ("worst_vehicle",
             lambda d: d["timing"]["selection"]["worst"].__setitem__(
                 "vehicle_id",
                 d["timing"]["selection"]["worst"]["vehicle_id"] + 1),
             "MODEL_TIMING_INCONSISTENT"),
            ("interval_subdivisions",
             lambda d: d["timing"]["selection"].__setitem__(
                 "interval_subdivisions_per_segment", 0),
             "INVALID_MODEL_JSON"),
            ("T0",
             lambda d: d["timing"]["selection"].__setitem__("T0_s", 2.0),
             "MODEL_TIMING_INCONSISTENT"),
            ("wheel_angle_limit",
             lambda d: d["timing"]["selection"].__setitem__(
                 "wheel_angle_limit_rad",
                 d["timing"]["selection"]["wheel_angle_limit_rad"] * 1.01),
             "MODEL_TIMING_INCONSISTENT"),
            ("metric_extra", metric("speed", "extra", 1),
             "UNSUPPORTED_MODEL_FIELD"),
        )
        for name, mutate, expected in cases:
            with self.subTest(case=name):
                with self.assertRaises(VirtualAContractError) as caught:
                    VirtualATimedPlan.from_model_json(self._mutated(mutate))
                self.assertEqual(caught.exception.reason, expected)

    def test_request_identity_tamper_is_rejected(self):
        cases = (
            ("heading", lambda d: d["request_inputs"]["start_headings"][0]
             .__setitem__(1, d["request_inputs"]["start_headings"][0][1]
                          + 0.1), "INITIAL_HEADING_MISMATCH"),
            ("goal", lambda d: d["request_inputs"]["goal_qf"].__setitem__(
                0, d["request_inputs"]["goal_qf"][0] + 0.1),
             "REQUEST_DIGEST_MISMATCH"),
            ("policy", lambda d: d["request_inputs"].__setitem__(
                "goal_policy", "STRAIGHT"), "REQUEST_DIGEST_MISMATCH"),
            ("wheelbase", lambda d: d["request_inputs"]["limits"].__setitem__(
                "wheelbase",
                d["request_inputs"]["limits"]["wheelbase"] + 0.01),
             "REQUEST_DIGEST_MISMATCH"),
            ("v_max", lambda d: d["request_inputs"]["limits"].__setitem__(
                "v_max", 2.0), "REQUEST_DIGEST_MISMATCH"),
            ("request_sha", lambda d: d.__setitem__(
                "request_sha256", "0" * 64), "REQUEST_DIGEST_MISMATCH"),
        )
        for name, mutate, expected in cases:
            with self.subTest(case=name):
                with self.assertRaises(VirtualAContractError) as caught:
                    VirtualATimedPlan.from_model_json(self._mutated(mutate))
                self.assertEqual(caught.exception.reason, expected)

    def test_version_numerics_and_profile_tamper_is_rejected(self):
        cases = (
            ("backend", lambda d: d.__setitem__("backend", "other"),
             "UNKNOWN_BACKEND"),
            ("model_version", lambda d: d.__setitem__(
                "model_version", "other"), "UNKNOWN_MODEL_VERSION"),
            ("path_version", lambda d: d.__setitem__(
                "path_model_version", "other"), "UNKNOWN_PATH_VERSION"),
            ("timing_law", lambda d: d["timing"].__setitem__(
                "law", "other"), "UNKNOWN_TIMING_LAW"),
            ("numerics_version", lambda d: d["numerics"].__setitem__(
                "version", "other"), "UNKNOWN_NUMERICS_VERSION"),
            ("numerics_method", lambda d: d["numerics"].__setitem__(
                "integration_method", "hold"), "MODEL_NUMERICS_MISMATCH"),
            ("path_length", lambda d: d["path"].__setitem__(
                "length_m", d["path"]["length_m"] + 1.0),
             "MODEL_PATH_LENGTH_MISMATCH"),
            ("position_budget", lambda d: d["path"].__setitem__(
                "position_error_bound_m", 1e-3),
             "MODEL_NUMERICS_MISMATCH"),
            ("knot_value", lambda d: d["path"]["profile"]["knots"][1]
             .__setitem__("value",
                          d["path"]["profile"]["knots"][1]["value"] + 0.05),
             "NON_C2_PROFILE"),
            ("knot_derivative", lambda d: d["path"]["profile"]["knots"][1]
             .__setitem__("derivative",
                          d["path"]["profile"]["knots"][1]["derivative"]
                          + 0.05), "NON_C2_PROFILE"),
        )
        for name, mutate, expected in cases:
            with self.subTest(case=name):
                with self.assertRaises(VirtualAContractError) as caught:
                    VirtualATimedPlan.from_model_json(self._mutated(mutate))
                self.assertEqual(caught.exception.reason, expected)

    def test_unknown_extra_and_self_referential_fields_are_rejected(self):
        cases = (
            ("extra_top", lambda d: d.__setitem__("extra", 1)),
            ("self_digest", lambda d: d.__setitem__("model_sha256", "0" * 64)),
            ("audit_report", lambda d: d.__setitem__("dynamic_audit", {})),
        )
        for name, mutate in cases:
            with self.subTest(case=name):
                with self.assertRaises(VirtualAContractError) as caught:
                    VirtualATimedPlan.from_model_json(self._mutated(mutate))
                self.assertEqual(caught.exception.reason,
                                 "UNSUPPORTED_MODEL_FIELD")

    def test_sample_is_pointwise_non_authoritative_and_cannot_alter_model(self):
        plan = self.plan
        T = plan.duration_s
        before = plan.to_model_json()
        times = (0.0, 0.1 * T, 0.3 * T, 0.7 * T, T)
        projection = plan.sample(times)
        self.assertEqual(projection.times, times)
        self.assertEqual(len(projection.states), len(times))
        self.assertFalse(projection.authoritative)
        self.assertEqual(projection.model_summary, plan.model_summary())
        for t, state in zip(times, projection.states):
            self.assertEqual(state.as_dict(), plan.state_at(t).as_dict())
        # Sampling is not interpolation: an unsampled time is absent from the
        # bundle, and querying it later still returns the exact pointwise law.
        unsampled = 0.2 * T
        self.assertNotIn(unsampled, projection.times)
        self.assertNotEqual(plan.state_at(unsampled).s,
                            plan.state_at(0.1 * T).s)
        self.assertEqual(plan.sample([unsampled]).states[0].as_dict(),
                         plan.state_at(unsampled).as_dict())
        # Mutating the non-authoritative copy does not change the model.
        payload = projection.as_dict()
        payload["authoritative"] = True
        payload["times"][0] = 12345.0
        payload["states"][0]["qf"]["x"] = 12345.0
        payload["model_summary"]["model_sha256"] = "0" * 64
        self.assertFalse(projection.authoritative)
        self.assertEqual(plan.to_model_json(), before)
        self.assertEqual(plan.state_at(0.0).qf.x, self.request.start_qf.x)
        self.assertEqual(plan.model_summary()["model_sha256"],
                         plan.model_sha256())
        # A second sample is identical and still non-authoritative.
        again = plan.sample(times)
        self.assertEqual(again.states, projection.states)
        self.assertFalse(again.authoritative)

    def test_sample_rejects_nonfinite_or_noniterable_times(self):
        for times in ([float("nan")], [float("inf")], ["x"], [None], 1.0):
            with self.subTest(times=times):
                with self.assertRaises(VirtualAContractError) as caught:
                    self.plan.sample(times)
                self.assertIn(caught.exception.reason,
                              ("NONFINITE_VALUE", "INVALID_SAMPLE_TIMES"))

    def test_hold_roundtrip_and_one_sample_contract(self):
        model = model_from_fixture()
        path = VirtualAPath.hold(FormationPose(0.0, 0.0, 0.0), model)
        request = request_for(path, goal=FormationPose(0.0, 0.0, 0.0))
        bind(path, request)
        result = retime_path(request, path, TimingConfig())
        plan = result.plan
        payload = plan.to_model_json()
        rebuilt = VirtualATimedPlan.from_model_json(
            payload, expected_request=request)
        self.assertEqual(rebuilt.to_model_json(), payload)
        self.assertEqual(rebuilt.model_sha256(), plan.model_sha256())
        self.assertEqual(rebuilt.total_S_m, 0.0)
        self.assertEqual(rebuilt.duration_s, 0.0)
        self.assertEqual(rebuilt.state_at(0.0).as_dict(),
                         plan.state_at(0.0).as_dict())
        projection = rebuilt.sample([0.0])
        self.assertEqual(projection.times, (0.0,))
        self.assertFalse(projection.authoritative)
        for invalid in ((), (0.0, 1.0), (1.0,), (-1.0,)):
            with self.subTest(times=invalid):
                with self.assertRaises(VirtualAContractError):
                    rebuilt.sample(invalid)
        before = rebuilt.to_model_json()
        payload_dict = projection.as_dict()
        payload_dict["states"][0]["vehicles"][0]["wheel_speeds"][0] = 1.0
        self.assertEqual(rebuilt.to_model_json(), before)
        self.assertEqual(rebuilt.state_at(0.0).vehicles[0].wheel_speeds,
                         (0.0, 0.0, 0.0, 0.0))


class VersionedNumericsModelTests(unittest.TestCase):
    """Step 5: old v2 model replay and rejection of version tampering."""

    V2_SHA256 = ("df146b9bb4710132adeacd4d718d5a84"
                 "0c0f0e3bce353d06331a84142c2eaaf8")

    @classmethod
    def setUpClass(cls):
        cls.v2_payload = (
            FIXTURE_DIR / "v2_model_600000.json").read_text()
        cls.v2_document = json.loads(cls.v2_payload)
        request, path = curved_request_and_path()
        cls.v3_plan = retime_path(request, path, TimingConfig()).plan
        cls.v3_payload = cls.v3_plan.to_model_json()
        cls.v3_document = json.loads(cls.v3_payload)

    @staticmethod
    def _mutate(document, mutate):
        mutate(document)
        return canonical_json(document)

    def test_historical_v2_model_replays_byte_identically(self):
        self.assertEqual(bounds_module.NUMERICS_V2,
                         "virtual_rigid_a.numerics.g1.v2")
        plan = VirtualATimedPlan.from_model_json(self.v2_payload)
        self.assertEqual(plan.path.numerics_version, bounds_module.NUMERICS_V2)
        self.assertEqual(plan.path.profile.numerics_version,
                         bounds_module.NUMERICS_V2)
        budget = plan.path.integration_budget
        self.assertEqual(budget["numerics_version"], bounds_module.NUMERICS_V2)
        self.assertNotIn("segments", budget)
        self.assertEqual(plan.model_sha256(), self.V2_SHA256)
        self.assertEqual(plan.to_model_json(), self.v2_payload)
        self.assertLessEqual(plan.path.position_error_bound_m, 1e-7)

    def test_new_v3_model_declares_v3_in_every_numerics_field(self):
        document = self.v3_document
        self.assertEqual(document["numerics"]["version"],
                         bounds_module.NUMERICS_V3)
        self.assertEqual(document["path"]["profile"]["numerics_version"],
                         bounds_module.NUMERICS_V3)
        self.assertEqual(
            document["path"]["integration_budget"]["numerics_version"],
            bounds_module.NUMERICS_V3)
        self.assertIn("segments",
                      document["path"]["integration_budget"])
        rebuilt = VirtualATimedPlan.from_model_json(self.v3_payload)
        self.assertEqual(rebuilt.to_model_json(), self.v3_payload)
        self.assertEqual(rebuilt.model_sha256(), self.v3_plan.model_sha256())

    def test_cross_version_tampering_is_rejected(self):
        def set_version(document, block, value):
            if block == "profile":
                document["path"]["profile"]["numerics_version"] = value
            else:
                document[block]["version"] = value

        cases = (
            ("v2_numerics_to_v3_only",
             lambda d: set_version(d, "numerics",
                                   bounds_module.NUMERICS_V3),
             "MODEL_NUMERICS_MISMATCH"),
            ("v2_profile_to_v3_only",
             lambda d: set_version(d, "profile", bounds_module.NUMERICS_V3),
             "MODEL_NUMERICS_MISMATCH"),
            ("v2_document_claims_v3",
             lambda d: (set_version(d, "numerics",
                                    bounds_module.NUMERICS_V3),
                        set_version(d, "profile",
                                    bounds_module.NUMERICS_V3)),
             "MODEL_NUMERICS_MISMATCH"),
            ("v2_numerics_unknown",
             lambda d: set_version(d, "numerics", "other"),
             "UNKNOWN_NUMERICS_VERSION"),
            ("v2_profile_unknown",
             lambda d: set_version(d, "profile", "other"),
             "UNKNOWN_NUMERICS_VERSION"),
        )
        for name, mutate, expected in cases:
            with self.subTest(case=name):
                with self.assertRaises(VirtualAContractError) as caught:
                    VirtualATimedPlan.from_model_json(
                        self._mutate(json.loads(self.v2_payload), mutate))
                self.assertEqual(caught.exception.reason, expected)

        def v3_as_v2(document):
            set_version(document, "numerics", bounds_module.NUMERICS_V2)
            set_version(document, "profile", bounds_module.NUMERICS_V2)

        def v3_profile_as_v2(document):
            set_version(document, "profile", bounds_module.NUMERICS_V2)

        def v3_budget_replaced_by_v2(document):
            document["path"]["integration_budget"] = self.v2_document[
                "path"]["integration_budget"]

        for name, mutate, expected in (
                ("v3_document_claims_v2", v3_as_v2,
                 "MODEL_NUMERICS_MISMATCH"),
                ("v3_profile_claims_v2", v3_profile_as_v2,
                 "MODEL_NUMERICS_MISMATCH"),
                ("v3_budget_replaced_by_v2", v3_budget_replaced_by_v2,
                 "MODEL_NUMERICS_MISMATCH")):
            with self.subTest(case=name):
                with self.assertRaises(VirtualAContractError) as caught:
                    VirtualATimedPlan.from_model_json(
                        self._mutate(json.loads(self.v3_payload), mutate))
                self.assertEqual(caught.exception.reason, expected)


if __name__ == "__main__":
    unittest.main()