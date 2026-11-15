"""FR01 targeted acceptance tests for analytic uniform-mode edges.

The expected geometry in this module is recomputed from the contract formulas
and the fixed cargo offsets.  The tests do not accept an edge merely because
its constructor returned one: every positive case is compared with an
independent analytic state, an independent finite difference or an
independently computed canonical wheel configuration.  Negative cases assert
the explicit rejection reason instead of treating any exception as success.

Run from the V0.2 worktree root:

    OPENBLAS_NUM_THREADS=1 \
        python3 -m unittest discover -s tests -p 'test_uniform_motion.py' -v
"""
from contextlib import contextmanager
from dataclasses import replace
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO_ROOT / "catkin_ws" / "src" / "fwis_v3_optimizer" / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from fwis_v3_optimizer.core import Action
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_kinematics import (
    ACKERMANN,
    OBLIQUE,
    SPIN,
)
from fwis_v3_optimizer.retiming import MotionLimits, wheel_configuration
from fwis_v3_optimizer.uniform_motion import (
    MODE_A,
    MODE_O,
    MODE_S,
    PRIMITIVE_A_ARC,
    PRIMITIVE_A_STRAIGHT,
    PRIMITIVE_O_TRANSLATION,
    PRIMITIVE_S_SPIN,
    UniformMotionError,
    a_arc_edge,
    a_arc_required_headings,
    a_straight_edge,
    o_translation_edge,
    s_spin_edge,
    validate_uniform_edge,
)

DEFAULT_OFFSETS = (
    (1, 1.0, 0.0),
    (2, 0.0, 1.0),
    (3, 0.0, -1.0),
)
BODY_LENGTH_M = 0.738
BODY_WIDTH_M = 0.500
REFERENCE_DISTANCES = {
    (1, 2): math.sqrt(2.0),
    (1, 3): math.sqrt(2.0),
    (2, 3): 2.0,
}
NONUNIFORM_FRACTIONS = (0.0, 0.113, 0.371, 0.618, 0.887, 1.0)

DEFAULT_RESULTS_PATH = (
    REPO_ROOT / "evaluation" / "formation_refactor_v02" / "FR01"
    / "primitive_results.json")
RESULTS_PATH = Path(
    os.environ.get("FR01_RESULTS_PATH", str(DEFAULT_RESULTS_PATH)))
RESULTS = []


def _json_default(value):
    if hasattr(value, "item"):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (set, frozenset)):
        return sorted(value)
    raise TypeError("not JSON serializable: {!r}".format(value))


@contextmanager
def record_case(case_id, title, inputs):
    """Record a case as FAILED until its body completes successfully."""
    record = {
        "case": case_id,
        "title": title,
        "status": "FAILED",
        "inputs": inputs,
        "subcases": [],
    }
    RESULTS.append(record)
    try:
        yield record
    except Exception as exc:
        record["status"] = (
            "FAILED" if isinstance(exc, AssertionError) else "ERROR")
        record["error_type"] = type(exc).__name__
        record["error"] = str(exc)
        raise
    else:
        record["status"] = "PASSED"


def _model():
    return FormationModel(tuple(
        VehicleOffset(vehicle_id, x, y)
        for vehicle_id, x, y in DEFAULT_OFFSETS))


def _rot(x, y, angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return cosine * x - sine * y, sine * x + cosine * y


def _centers_from_pose(qf):
    result = {}
    for vehicle_id, x, y in DEFAULT_OFFSETS:
        rx, ry = _rot(x, y, qf.psi)
        result[vehicle_id] = (qf.x + rx, qf.y + ry)
    return result


def _pair_map(pairs):
    return {int(vehicle_id): float(value) for vehicle_id, value in pairs}


def _heading_error(first, second):
    return abs(math.atan2(
        math.sin(float(first) - float(second)),
        math.cos(float(first) - float(second))))


def _line_qf(qf_start, displacement, progress):
    dx, dy = displacement
    length = math.hypot(dx, dy)
    return FormationPose(
        qf_start.x + progress * dx / length,
        qf_start.y + progress * dy / length,
        qf_start.psi)


def _arc_qf(qf_start, pivot, delta_psi, progress):
    sign = 1.0 if delta_psi > 0.0 else -1.0
    pivot_x, pivot_y = pivot
    wx, wy = _rot(pivot_x, pivot_y, qf_start.psi)
    center_x = qf_start.x + wx
    center_y = qf_start.y + wy
    psi = qf_start.psi + sign * progress
    rx, ry = _rot(pivot_x, pivot_y, psi)
    return FormationPose(center_x - rx, center_y - ry, psi)


def _independent_wheel(mode, control, limits):
    """Wheel angles/factors from the wheel positions, not from edge code."""
    angles = []
    factors = []
    for x in (limits.wheelbase / 2.0, -limits.wheelbase / 2.0):
        for y in (limits.track / 2.0, -limits.track / 2.0):
            if mode == ACKERMANN:
                vx, vy = 1.0 - control * y, control * x
            elif mode == OBLIQUE:
                vx, vy = math.cos(control), math.sin(control)
            elif mode == SPIN:
                vx, vy = -y, x
            else:
                raise AssertionError("unknown wheel mode {}".format(mode))
            angle = math.atan2(vy, vx)
            if mode == SPIN:
                if angle > math.pi / 2.0:
                    angle -= math.pi
                if angle < -math.pi / 2.0:
                    angle += math.pi
            angles.append(angle)
            factors.append(math.hypot(vx, vy))
    return tuple(angles), tuple(factors)


class UniformMotionAcceptanceTest(unittest.TestCase):
    maxDiff = None

    # ---- assertions -------------------------------------------------
    def _expect_reason(self, reason, function, vehicle_id=None,
                       detail_contains=()):
        with self.assertRaises(UniformMotionError) as context:
            function()
        error = context.exception
        self.assertEqual(error.reason, reason, str(error))
        if vehicle_id is not None:
            self.assertEqual(error.vehicle_id, vehicle_id, str(error))
        for token in detail_contains:
            self.assertIn(token, error.detail, str(error))
        return error

    def _assert_contract(self, edge, length, unit, position_unit,
                         angle_unit):
        self.assertGreater(length, 0.0)
        self.assertEqual(edge.progress_interval[0], 0.0)
        self.assertAlmostEqual(
            edge.progress_interval[1], length, delta=1e-12)
        self.assertEqual(edge.progress_unit, unit)
        self.assertEqual(edge.position_derivative_unit, position_unit)
        self.assertEqual(edge.angle_derivative_unit, angle_unit)
        self.assertFalse(edge.as_dict()["real_time_domain"])
        self.assertIsNone(edge.as_dict()["time_seconds"])

    def _assert_pair_values(self, actual_pairs, expected, tolerance=1e-9):
        actual = _pair_map(actual_pairs)
        self.assertEqual(sorted(actual), sorted(expected))
        for vehicle_id, value in expected.items():
            self.assertAlmostEqual(
                actual[vehicle_id], value, delta=tolerance,
                msg="vehicle_id={}".format(vehicle_id))

    def _assert_heading_values(self, actual_pairs, expected, tolerance=1e-9):
        actual = _pair_map(actual_pairs)
        self.assertEqual(sorted(actual), sorted(expected))
        for vehicle_id, value in expected.items():
            self.assertLessEqual(
                _heading_error(actual[vehicle_id], value), tolerance,
                "vehicle_id={} actual={!r} expected={!r}".format(
                    vehicle_id, actual[vehicle_id], value))

    def _assert_directions(self, edge, expected):
        self.assertEqual(_pair_map(edge.directions), expected)

    def _assert_rigidity(self, centers):
        for (first, second), reference in REFERENCE_DISTANCES.items():
            actual = math.hypot(
                centers[second][0] - centers[first][0],
                centers[second][1] - centers[first][1])
            self.assertLessEqual(
                abs(actual - reference), 1e-6,
                "pair=({}, {})".format(first, second))

    def _assert_geometry_samples(self, edge, qf_function, heading_function,
                                 fractions=NONUNIFORM_FRACTIONS):
        length = edge.progress_interval[1]
        for fraction in fractions:
            progress = length * fraction
            state = edge.state_at(progress)
            self.assertAlmostEqual(state.progress, progress, delta=0.0)
            expected_qf = qf_function(progress)
            self.assertAlmostEqual(
                state.qf.x, expected_qf.x, delta=1e-7)
            self.assertAlmostEqual(
                state.qf.y, expected_qf.y, delta=1e-7)
            self.assertLessEqual(
                _heading_error(state.qf.psi, expected_qf.psi), 1e-7)
            expected_centers = _centers_from_pose(expected_qf)
            centers = {
                vehicle_id: (x, y)
                for vehicle_id, x, y in state.centers}
            self.assertEqual(sorted(centers), [1, 2, 3])
            for vehicle_id in (1, 2, 3):
                self.assertAlmostEqual(
                    centers[vehicle_id][0],
                    expected_centers[vehicle_id][0], delta=1e-7)
                self.assertAlmostEqual(
                    centers[vehicle_id][1],
                    expected_centers[vehicle_id][1], delta=1e-7)
            self._assert_rigidity(centers)
            headings = _pair_map(state.headings)
            for vehicle_id in (1, 2, 3):
                self.assertLessEqual(
                    _heading_error(
                        headings[vehicle_id],
                        heading_function(vehicle_id, progress)),
                    1e-9,
                    "vehicle_id={} progress={}".format(
                        vehicle_id, progress))

    def _assert_wheels(self, edge, mode, controls):
        for vehicle_id in (1, 2, 3):
            expected_angles, expected_factors = _independent_wheel(
                mode, controls[vehicle_id], edge.limits)
            for at_end in (False, True):
                configuration = edge.wheel_configuration(
                    vehicle_id, at_end=at_end)
                for actual, expected in zip(
                        configuration.angles, expected_angles):
                    self.assertAlmostEqual(actual, expected, delta=1e-9)
                for actual, expected in zip(
                        configuration.factors, expected_factors):
                    self.assertAlmostEqual(actual, expected, delta=1e-9)
                self.assertLessEqual(
                    max(abs(value) for value in configuration.angles),
                    edge.limits.steer_angle_max + 1e-9)

    def _assert_metrics(self, edge):
        metrics = validate_uniform_edge(edge)
        self.assertLessEqual(metrics.max_endpoint_qf_error_m, 1e-7)
        self.assertLessEqual(metrics.max_endpoint_heading_error_rad, 1e-7)
        self.assertLessEqual(metrics.max_rigidity_residual_m, 1e-6)
        self.assertLessEqual(metrics.max_mode_equation_residual, 1e-6)
        self.assertLessEqual(metrics.max_derivative_error, 1e-4)
        self.assertLessEqual(
            metrics.max_wheel_angle_rad,
            edge.limits.steer_angle_max + 1e-9)
        for name, value in metrics.as_dict().items():
            if name not in ("primitive", "sample_count"):
                self.assertTrue(
                    math.isfinite(value),
                    "non-finite metric {}={!r}".format(name, value))
        return metrics

    def _assert_arc_finite_difference(
            self, edge, qf_start, pivot, delta_psi, headings, directions,
            samples=(0.31, 0.73)):
        sign = 1.0 if delta_psi > 0.0 else -1.0
        step = 1e-5
        pivot_x, pivot_y = pivot
        for progress in samples:
            qf_plus = _arc_qf(
                qf_start, pivot, delta_psi, progress + step)
            qf_minus = _arc_qf(
                qf_start, pivot, delta_psi, progress - step)
            derivative = edge.derivative_at(progress)
            # d/dlambda[-R(psi)c] = -a R(psi)Jc, with Jc=(-cy,cx).
            jc_x, jc_y = -pivot_y, pivot_x
            rotated_jc = _rot(
                jc_x, jc_y, qf_start.psi + sign * progress)
            expected_dqf = (
                -sign * rotated_jc[0], -sign * rotated_jc[1], sign)
            finite_dqf = (
                (qf_plus.x - qf_minus.x) / (2.0 * step),
                (qf_plus.y - qf_minus.y) / (2.0 * step),
                (qf_plus.psi - qf_minus.psi) / (2.0 * step))
            for actual, finite, analytic in zip(
                    derivative.dqf, finite_dqf, expected_dqf):
                self.assertLessEqual(abs(actual - finite), 1e-4)
                self.assertLessEqual(abs(actual - analytic), 1e-6)
            centers_plus = _centers_from_pose(qf_plus)
            centers_minus = _centers_from_pose(qf_minus)
            dp_map = {
                vehicle_id: (x, y)
                for vehicle_id, x, y in derivative.dp}
            dtheta_map = _pair_map(derivative.dtheta)
            psi = qf_start.psi + sign * progress
            for vehicle_id, offset_x, offset_y in DEFAULT_OFFSETS:
                finite_dp = (
                    (centers_plus[vehicle_id][0]
                     - centers_minus[vehicle_id][0]) / (2.0 * step),
                    (centers_plus[vehicle_id][1]
                     - centers_minus[vehicle_id][1]) / (2.0 * step))
                rx = offset_x - pivot_x
                ry = offset_y - pivot_y
                jx, jy = -ry, rx
                analytic_dp = (
                    sign * (math.cos(psi) * jx - math.sin(psi) * jy),
                    sign * (math.sin(psi) * jx + math.cos(psi) * jy))
                for actual, finite, analytic in zip(
                        dp_map[vehicle_id], finite_dp, analytic_dp):
                    self.assertLessEqual(abs(actual - finite), 1e-4)
                    self.assertLessEqual(abs(actual - analytic), 1e-9)
                theta = headings[vehicle_id] + sign * progress
                ex, ey = math.cos(theta), math.sin(theta)
                projection = finite_dp[0] * ex + finite_dp[1] * ey
                expected_projection = (
                    directions[vehicle_id] * math.hypot(rx, ry))
                self.assertLessEqual(
                    abs(projection - expected_projection), 1e-4,
                    "vehicle_id={}".format(vehicle_id))
                lateral = (
                    finite_dp[0] - projection * ex,
                    finite_dp[1] - projection * ey)
                self.assertLessEqual(
                    math.hypot(*lateral), 1e-4,
                    "vehicle_id={}".format(vehicle_id))
                radius = math.hypot(rx, ry)
                curvature = (
                    sign / (directions[vehicle_id] * radius))
                self.assertAlmostEqual(
                    _pair_map(edge.curvatures)[vehicle_id],
                    curvature, delta=1e-9)
                self.assertAlmostEqual(
                    dtheta_map[vehicle_id], sign, delta=1e-12)
                self.assertAlmostEqual(
                    dtheta_map[vehicle_id] / projection,
                    curvature, delta=1e-6)

    # ---- P01 --------------------------------------------------------
    def test_p01_a_straight_forward_and_reverse(self):
        inputs = {
            "start_qf": [0.5, -0.25, 0.1],
            "headings": {"1": 0.0, "2": math.pi, "3": 0.0},
            "displacement": [3.0, 0.0],
            "directions": {"1": 1, "2": -1, "3": 1},
            "allow_reverse": True,
        }
        with record_case(
                "P01", "A straight with mixed gear branches", inputs) as case:
            model = _model()
            qf_start = FormationPose(0.5, -0.25, 0.1)
            headings = {1: 0.0, 2: math.pi, 3: 0.0}
            directions = {1: 1, 2: -1, 3: 1}
            displacement = (3.0, 0.0)
            edge = a_straight_edge(
                model, qf_start, headings, displacement, directions,
                allow_reverse=True)
            case["edge"] = edge.as_dict()
            self.assertEqual(edge.mode, MODE_A)
            self.assertEqual(edge.primitive, PRIMITIVE_A_STRAIGHT)
            self.assertIsNone(edge.pivot_in_cargo_frame)
            self._assert_contract(edge, 3.0, "m", "m/m", "rad/m")
            self._assert_directions(edge, directions)
            self._assert_pair_values(
                edge.curvatures, {1: 0.0, 2: 0.0, 3: 0.0})
            self._assert_pair_values(
                edge.signed_progress_speeds,
                {1: 1.0, 2: -1.0, 3: 1.0})
            self._assert_heading_values(
                edge.start_headings, headings)
            self._assert_heading_values(
                edge.end_headings, headings)
            for segment in edge.segments:
                self.assertEqual(segment.mode, ACKERMANN)
                self.assertEqual(
                    segment.direction, directions[segment.vehicle_id])
                self.assertAlmostEqual(
                    segment.travel_distance, 3.0, delta=1e-12)
            self._assert_geometry_samples(
                edge,
                lambda progress: _line_qf(
                    qf_start, displacement, progress),
                lambda vehicle_id, progress: headings[vehicle_id])
            start_state = edge.state_at(0.0)
            end_state = edge.state_at(3.0)
            self.assertAlmostEqual(
                end_state.qf.x, qf_start.x + 3.0, delta=1e-12)
            self.assertAlmostEqual(
                end_state.qf.y, qf_start.y, delta=1e-12)
            self.assertLessEqual(
                _heading_error(end_state.qf.psi, qf_start.psi), 1e-12)
            self.assertEqual(_pair_map(start_state.headings), headings)
            self.assertEqual(_pair_map(end_state.headings), headings)
            self._assert_wheels(edge, ACKERMANN, {1: 0.0, 2: 0.0, 3: 0.0})

            step = 1e-5
            progress = 1.13
            qf_plus = _line_qf(
                qf_start, displacement, progress + step)
            qf_minus = _line_qf(
                qf_start, displacement, progress - step)
            derivative = edge.derivative_at(progress)
            self.assertAlmostEqual(derivative.dqf[0], 1.0, delta=1e-12)
            self.assertAlmostEqual(derivative.dqf[1], 0.0, delta=1e-12)
            self.assertAlmostEqual(derivative.dqf[2], 0.0, delta=1e-12)
            centers_plus = _centers_from_pose(qf_plus)
            centers_minus = _centers_from_pose(qf_minus)
            dp_map = {
                vehicle_id: (x, y) for vehicle_id, x, y in derivative.dp}
            for vehicle_id in (1, 2, 3):
                finite_dp = (
                    (centers_plus[vehicle_id][0]
                     - centers_minus[vehicle_id][0]) / (2.0 * step),
                    (centers_plus[vehicle_id][1]
                     - centers_minus[vehicle_id][1]) / (2.0 * step))
                self.assertLessEqual(
                    abs(finite_dp[0] - dp_map[vehicle_id][0]), 1e-4)
                self.assertLessEqual(
                    abs(finite_dp[1] - dp_map[vehicle_id][1]), 1e-4)
                theta = headings[vehicle_id]
                ex, ey = math.cos(theta), math.sin(theta)
                projection = finite_dp[0] * ex + finite_dp[1] * ey
                lateral = finite_dp[0] * (-ey) + finite_dp[1] * ex
                self.assertLessEqual(
                    abs(projection - directions[vehicle_id]), 1e-4)
                self.assertLessEqual(abs(lateral), 1e-4)
            case["validation"] = self._assert_metrics(edge).as_dict()

    # ---- P02 --------------------------------------------------------
    def test_p02_a_arc_positive_around_zero_pivot(self):
        inputs = {
            "start_qf": [0.2, -0.1, 0.0],
            "pivot": [0.0, 0.0],
            "delta_psi": math.pi / 2.0,
            "headings": {
                "1": math.pi / 2.0, "2": math.pi, "3": 0.0},
            "directions": {"1": 1, "2": 1, "3": 1},
        }
        with record_case(
                "P02", "A arc, c=0, positive rotation", inputs) as case:
            model = _model()
            qf_start = FormationPose(0.2, -0.1, 0.0)
            delta_psi = math.pi / 2.0
            pivot = (0.0, 0.0)
            directions = {1: 1, 2: 1, 3: 1}
            headings = {1: math.pi / 2.0, 2: math.pi, 3: 0.0}
            expected_headings = a_arc_required_headings(
                model, qf_start, delta_psi, pivot, directions)
            self._assert_heading_values(
                expected_headings.items(), headings, tolerance=1e-12)
            edge = a_arc_edge(
                model, qf_start, headings, delta_psi, pivot, directions)
            case["edge"] = edge.as_dict()
            self.assertEqual(edge.mode, MODE_A)
            self.assertEqual(edge.primitive, PRIMITIVE_A_ARC)
            self.assertEqual(edge.pivot_in_cargo_frame, pivot)
            self._assert_contract(
                edge, math.pi / 2.0, "rad", "m/rad", "rad/rad")
            self._assert_directions(edge, directions)
            self._assert_pair_values(
                edge.curvatures, {1: 1.0, 2: 1.0, 3: 1.0})
            self._assert_pair_values(
                edge.signed_progress_speeds,
                {1: 1.0, 2: 1.0, 3: 1.0})
            self._assert_heading_values(
                edge.end_headings,
                {1: math.pi / 2.0 + delta_psi,
                 2: math.pi + delta_psi,
                 3: delta_psi})
            self._assert_geometry_samples(
                edge,
                lambda progress: _arc_qf(
                    qf_start, pivot, delta_psi, progress),
                lambda vehicle_id, progress: headings[vehicle_id] + progress)
            self._assert_wheels(edge, ACKERMANN, {1: 1.0, 2: 1.0, 3: 1.0})
            self._assert_arc_finite_difference(
                edge, qf_start, pivot, delta_psi, headings, directions)
            case["validation"] = self._assert_metrics(edge).as_dict()

    # ---- P03 --------------------------------------------------------
    def test_p03_a_arc_clockwise_uses_reverse_branch(self):
        inputs = {
            "start_qf": [0.2, -0.1, 0.0],
            "pivot": [0.0, 0.0],
            "delta_psi": -math.pi / 2.0,
            "headings": {
                "1": math.pi / 2.0, "2": math.pi, "3": 0.0},
            "directions": {"1": -1, "2": -1, "3": -1},
            "allow_reverse": True,
        }
        with record_case(
                "P03", "A arc clockwise with all reverse branches",
                inputs) as case:
            model = _model()
            qf_start = FormationPose(0.2, -0.1, 0.0)
            delta_psi = -math.pi / 2.0
            pivot = (0.0, 0.0)
            headings = {1: math.pi / 2.0, 2: math.pi, 3: 0.0}
            directions = {1: -1, 2: -1, 3: -1}
            edge = a_arc_edge(
                model, qf_start, headings, delta_psi, pivot, directions,
                allow_reverse=True)
            case["edge"] = edge.as_dict()
            self.assertEqual(edge.mode, MODE_A)
            self.assertEqual(edge.primitive, PRIMITIVE_A_ARC)
            self._assert_contract(
                edge, math.pi / 2.0, "rad", "m/rad", "rad/rad")
            self._assert_directions(edge, directions)
            self._assert_pair_values(
                edge.curvatures, {1: 1.0, 2: 1.0, 3: 1.0})
            self._assert_pair_values(
                edge.signed_progress_speeds,
                {1: -1.0, 2: -1.0, 3: -1.0})
            self._assert_heading_values(
                edge.end_headings,
                {1: math.pi / 2.0 + delta_psi,
                 2: math.pi + delta_psi,
                 3: delta_psi})
            self._assert_geometry_samples(
                edge,
                lambda progress: _arc_qf(
                    qf_start, pivot, delta_psi, progress),
                lambda vehicle_id, progress: headings[vehicle_id] - progress)
            self._assert_wheels(edge, ACKERMANN, {1: 1.0, 2: 1.0, 3: 1.0})
            self._assert_arc_finite_difference(
                edge, qf_start, pivot, delta_psi, headings, directions)
            case["validation"] = self._assert_metrics(edge).as_dict()

    # ---- P04 --------------------------------------------------------
    def test_p04_a_arc_offset_pivot_radii_and_curvatures(self):
        relative = math.atan2(1.0, 2.0)
        inputs = {
            "start_qf": [0.3, -0.2, 0.0],
            "pivot": [0.0, 2.0],
            "delta_psi": math.pi / 2.0,
            "headings": {"1": relative, "2": 0.0, "3": 0.0},
            "directions": {"1": 1, "2": 1, "3": 1},
        }
        with record_case(
                "P04", "A arc with c=(0,2)", inputs) as case:
            model = _model()
            qf_start = FormationPose(0.3, -0.2, 0.0)
            delta_psi = math.pi / 2.0
            pivot = (0.0, 2.0)
            directions = {1: 1, 2: 1, 3: 1}
            headings = {1: relative, 2: 0.0, 3: 0.0}
            expected = a_arc_required_headings(
                model, qf_start, delta_psi, pivot, directions)
            self._assert_heading_values(
                expected.items(), headings, tolerance=1e-12)
            edge = a_arc_edge(
                model, qf_start, headings, delta_psi, pivot, directions)
            case["edge"] = edge.as_dict()
            self.assertEqual(edge.mode, MODE_A)
            self._assert_contract(
                edge, math.pi / 2.0, "rad", "m/rad", "rad/rad")
            radii = {1: math.sqrt(5.0), 2: 1.0, 3: 3.0}
            self._assert_pair_values(
                edge.signed_progress_speeds, radii)
            self._assert_pair_values(
                edge.curvatures,
                {1: 1.0 / math.sqrt(5.0), 2: 1.0, 3: 1.0 / 3.0})
            self._assert_heading_values(
                edge.start_headings,
                {1: relative, 2: 0.0, 3: 0.0})
            relative_actual = {
                vehicle_id: _pair_map(edge.start_headings)[vehicle_id]
                - qf_start.psi for vehicle_id in (1, 2, 3)}
            self.assertLessEqual(
                _heading_error(relative_actual[1], math.atan2(1.0, 2.0)),
                1e-12)
            self.assertLessEqual(
                _heading_error(relative_actual[2], 0.0), 1e-12)
            self.assertLessEqual(
                _heading_error(relative_actual[3], 0.0), 1e-12)
            self._assert_geometry_samples(
                edge,
                lambda progress: _arc_qf(
                    qf_start, pivot, delta_psi, progress),
                lambda vehicle_id, progress: headings[vehicle_id] + progress)
            self._assert_wheels(
                edge, ACKERMANN,
                {1: 1.0 / math.sqrt(5.0), 2: 1.0, 3: 1.0 / 3.0})
            self._assert_arc_finite_difference(
                edge, qf_start, pivot, delta_psi, headings, directions)
            case["validation"] = self._assert_metrics(edge).as_dict()

    # ---- P05 --------------------------------------------------------
    def test_p05_a_arc_nonzero_cargo_heading(self):
        relative = math.atan2(1.0, 2.0)
        psi_start = 0.37
        inputs = {
            "start_qf": [1.3, -0.4, psi_start],
            "pivot": [0.0, 2.0],
            "delta_psi": math.pi / 2.0,
            "headings": {
                "1": psi_start + relative,
                "2": psi_start,
                "3": psi_start},
            "directions": {"1": 1, "2": 1, "3": 1},
        }
        with record_case(
                "P05", "A arc with non-zero cargo heading", inputs) as case:
            model = _model()
            qf_start = FormationPose(1.3, -0.4, psi_start)
            delta_psi = math.pi / 2.0
            pivot = (0.0, 2.0)
            directions = {1: 1, 2: 1, 3: 1}
            headings = {
                1: psi_start + relative,
                2: psi_start,
                3: psi_start}
            expected = a_arc_required_headings(
                model, qf_start, delta_psi, pivot, directions)
            self._assert_heading_values(
                expected.items(), headings, tolerance=1e-12)
            edge = a_arc_edge(
                model, qf_start, headings, delta_psi, pivot, directions)
            case["edge"] = edge.as_dict()
            self._assert_contract(
                edge, math.pi / 2.0, "rad", "m/rad", "rad/rad")
            pivot_world = _rot(0.0, 2.0, psi_start)
            pivot_world = (
                qf_start.x + pivot_world[0],
                qf_start.y + pivot_world[1])
            for fraction in NONUNIFORM_FRACTIONS:
                progress = edge.progress_interval[1] * fraction
                state = edge.state_at(progress)
                restored_x, restored_y = _rot(0.0, 2.0, state.qf.psi)
                self.assertAlmostEqual(
                    state.qf.x + restored_x, pivot_world[0], delta=1e-12)
                self.assertAlmostEqual(
                    state.qf.y + restored_y, pivot_world[1], delta=1e-12)
            self._assert_geometry_samples(
                edge,
                lambda progress: _arc_qf(
                    qf_start, pivot, delta_psi, progress),
                lambda vehicle_id, progress: headings[vehicle_id] + progress)
            self._assert_wheels(
                edge, ACKERMANN,
                {1: 1.0 / math.sqrt(5.0), 2: 1.0, 3: 1.0 / 3.0})
            self._assert_arc_finite_difference(
                edge, qf_start, pivot, delta_psi, headings, directions)
            case["validation"] = self._assert_metrics(edge).as_dict()

    # ---- P06 --------------------------------------------------------
    def test_p06_a_arc_rejects_vehicle_radius_below_limit(self):
        inputs = {
            "start_qf": [0.0, 0.0, 0.0],
            "pivot": [0.0, 1.5],
            "delta_psi": math.pi / 2.0,
            "headings": {
                "1": math.atan2(1.0, 1.5), "2": 0.0, "3": 0.0},
            "directions": {"1": 1, "2": 1, "3": 1},
        }
        with record_case(
                "P06", "c=(0,1.5) vehicle radius 0.5 is rejected",
                inputs) as case:
            model = _model()
            qf_start = FormationPose(0.0, 0.0, 0.0)
            pivot = (0.0, 1.5)
            headings = {
                1: math.atan2(1.0, 1.5), 2: 0.0, 3: 0.0}
            directions = {1: 1, 2: 1, 3: 1}
            error = self._expect_reason(
                "INSUFFICIENT_TURN_RADIUS",
                lambda: a_arc_edge(
                    model, qf_start, headings, math.pi / 2.0, pivot,
                    directions),
                vehicle_id=2,
                detail_contains=("0.5", "0.81"))
            case["rejection"] = {
                "reason": error.reason,
                "vehicle_id": error.vehicle_id,
                "detail": error.detail,
            }
            helper_error = self._expect_reason(
                "INSUFFICIENT_TURN_RADIUS",
                lambda: a_arc_required_headings(
                    model, qf_start, math.pi / 2.0, pivot, directions),
                vehicle_id=2)
            case["helper_rejection"] = {
                "reason": helper_error.reason,
                "vehicle_id": helper_error.vehicle_id,
                "detail": helper_error.detail,
            }
            # Cargo radius 1.5 is above 0.81; it must not replace the
            # per-vehicle check and let vehicle 2 through.
            case["cargo_radius_m"] = 1.5
            case["vehicle_radii_m"] = {
                "1": math.hypot(1.0, 1.5),
                "2": 0.5,
                "3": math.hypot(1.0, 1.5),
            }
            self.assertGreater(
                math.hypot(1.0, 1.5), 0.81,
                "cars 1 and 3 are individually above the minimum")
            self.assertLess(
                case["vehicle_radii_m"]["2"], 0.81,
                "car 2 is the actual per-vehicle rejection trigger")

    # ---- P07 --------------------------------------------------------
    def test_p07_a_arc_rejects_pivot_at_vehicle_centre(self):
        inputs = {
            "start_qf": [0.0, 0.0, 0.0],
            "pivot": [1.0, 0.0],
            "delta_psi": math.pi / 2.0,
            "headings": {"1": 0.0, "2": 0.0, "3": 0.0},
            "directions": {"1": 1, "2": 1, "3": 1},
        }
        with record_case(
                "P07", "pivot exactly on car1 centre", inputs) as case:
            model = _model()
            qf_start = FormationPose(0.0, 0.0, 0.0)
            pivot = (1.0, 0.0)
            headings = {1: 0.0, 2: 0.0, 3: 0.0}
            directions = {1: 1, 2: 1, 3: 1}
            error = self._expect_reason(
                "PIVOT_AT_VEHICLE_CENTER_UNSUPPORTED",
                lambda: a_arc_edge(
                    model, qf_start, headings, math.pi / 2.0, pivot,
                    directions),
                vehicle_id=1,
                detail_contains=(
                    "does not implement the degenerate parked-pivot",
                    "no general physical impossibility"))
            case["rejection"] = {
                "reason": error.reason,
                "vehicle_id": error.vehicle_id,
                "detail": error.detail,
            }
            helper_error = self._expect_reason(
                "PIVOT_AT_VEHICLE_CENTER_UNSUPPORTED",
                lambda: a_arc_required_headings(
                    model, qf_start, math.pi / 2.0, pivot, directions),
                vehicle_id=1)
            case["helper_rejection"] = {
                "reason": helper_error.reason,
                "vehicle_id": helper_error.vehicle_id,
                "detail": helper_error.detail,
            }
            self.assertTrue(helper_error.detail)

    # ---- P08 --------------------------------------------------------
    def test_p08_a_arc_requires_explicit_s_alignment(self):
        inputs = {
            "start_qf": [0.0, 0.0, 0.0],
            "pivot": [0.0, 0.0],
            "delta_psi": math.pi / 2.0,
            "headings": {"1": 0.0, "2": 0.0, "3": 0.0},
            "directions": {"1": 1, "2": 1, "3": 1},
        }
        with record_case(
                "P08", "A arc with non-tangent headings is rejected",
                inputs) as case:
            model = _model()
            qf_start = FormationPose(0.0, 0.0, 0.0)
            pivot = (0.0, 0.0)
            headings = {1: 0.0, 2: 0.0, 3: 0.0}
            directions = {1: 1, 2: 1, 3: 1}
            error = self._expect_reason(
                "A_HEADING_REQUIRES_S_ALIGNMENT",
                lambda: a_arc_edge(
                    model, qf_start, headings, math.pi / 2.0, pivot,
                    directions),
                vehicle_id=1,
                detail_contains=("insert an explicit S alignment edge",
                                 "will not rotate the body without time"))
            self._assert_heading_values(
                error.required_headings.items(),
                {1: math.pi / 2.0, 2: math.pi, 3: 0.0},
                tolerance=1e-12)
            required = a_arc_required_headings(
                model, qf_start, math.pi / 2.0, pivot, directions)
            self._assert_heading_values(
                required.items(),
                {1: math.pi / 2.0, 2: math.pi, 3: 0.0},
                tolerance=1e-12)
            case["rejection"] = {
                "reason": error.reason,
                "vehicle_id": error.vehicle_id,
                "detail": error.detail,
                "required_headings": {
                    str(key): value
                    for key, value in error.required_headings.items()},
            }
            # The rejected call returned no edge and did not mutate the
            # caller's heading mapping.
            self.assertEqual(headings, {1: 0.0, 2: 0.0, 3: 0.0})
            self.assertEqual(len(required), 3)

    # ---- P09 --------------------------------------------------------
    def test_p09_o_translation_and_oblique(self):
        horizontal_1 = {
            "start_qf": [0.6, -0.3, 0.2],
            "headings": {"1": 0.0, "2": math.pi, "3": math.pi / 2.0},
            "displacement": [2.0, 0.0],
            "directions": {"1": 1, "2": -1, "3": 1},
            "oblique_angles": {
                "1": 0.0, "2": 0.0, "3": -math.pi / 2.0},
            "allow_reverse": True,
        }
        horizontal_2 = {
            "start_qf": [0.6, -0.3, 0.2],
            "headings": {"1": 0.0, "2": math.pi, "3": math.pi / 2.0},
            "displacement": [2.0, 0.0],
            "directions": {"1": 1, "2": -1, "3": -1},
            "oblique_angles": {
                "1": 0.0, "2": 0.0, "3": math.pi / 2.0},
            "allow_reverse": True,
        }
        diagonal = {
            "start_qf": [0.0, 0.0, 0.4],
            "headings": {"1": 0.0, "2": math.pi, "3": 0.0},
            "displacement": [1.5, 1.5],
            "directions": {"1": 1, "2": -1, "3": 1},
            "oblique_angles": {
                "1": math.pi / 4.0, "2": math.pi / 4.0,
                "3": math.pi / 4.0},
            "allow_reverse": True,
        }
        with record_case(
                "P09", "O translation including +/-90 degree branches",
                {"horizontal_minus_90": horizontal_1,
                 "horizontal_plus_90": horizontal_2,
                 "diagonal": diagonal}) as case:
            model = _model()
            edge_records = []
            for name, inputs in (
                    ("horizontal_minus_90", horizontal_1),
                    ("horizontal_plus_90", horizontal_2),
                    ("diagonal", diagonal)):
                qf_start = FormationPose(*inputs["start_qf"])
                headings = {
                    int(key): value
                    for key, value in inputs["headings"].items()}
                directions = {
                    int(key): value
                    for key, value in inputs["directions"].items()}
                phis = {
                    int(key): value
                    for key, value in inputs["oblique_angles"].items()}
                displacement = tuple(inputs["displacement"])
                unit_x = displacement[0] / math.hypot(*displacement)
                unit_y = displacement[1] / math.hypot(*displacement)
                edge = o_translation_edge(
                    model, qf_start, headings, displacement, directions,
                    phis, allow_reverse=True)
                edge_records.append({
                    "name": name,
                    "edge": edge.as_dict(),
                    "validation": validate_uniform_edge(edge).as_dict(),
                })
                self.assertEqual(edge.mode, MODE_O)
                self.assertEqual(edge.primitive, PRIMITIVE_O_TRANSLATION)
                length = math.hypot(*displacement)
                self._assert_contract(edge, length, "m", "m/m", "rad/m")
                self._assert_directions(edge, directions)
                self._assert_pair_values(
                    edge.oblique_angles, phis)
                self._assert_pair_values(
                    edge.signed_progress_speeds,
                    {key: float(value) for key, value in directions.items()})
                self._assert_heading_values(
                    edge.start_headings, headings)
                self._assert_heading_values(edge.end_headings, headings)
                self._assert_geometry_samples(
                    edge,
                    lambda progress: _line_qf(
                        qf_start, displacement, progress),
                    lambda vehicle_id, progress: headings[vehicle_id])
                for vehicle_id in (1, 2, 3):
                    sigma = directions[vehicle_id]
                    phi = phis[vehicle_id]
                    expected_x = sigma * math.cos(
                        headings[vehicle_id] + phi)
                    expected_y = sigma * math.sin(
                        headings[vehicle_id] + phi)
                    self.assertLessEqual(
                        abs(expected_x - unit_x), 1e-12,
                        "{} vehicle_id={}".format(name, vehicle_id))
                    self.assertLessEqual(
                        abs(expected_y - unit_y), 1e-12,
                        "{} vehicle_id={}".format(name, vehicle_id))
                self._assert_wheels(edge, OBLIQUE, phis)
                # The lateral branch is only accepted by the full vector
                # equation; a longitudinal projection would be zero here.
                if name == "horizontal_minus_90":
                    self.assertLessEqual(
                        abs(math.cos(headings[3])), 1e-12)
                    self.assertAlmostEqual(phis[3], -math.pi / 2.0,
                                           delta=0.0)
                if name == "horizontal_plus_90":
                    self.assertLessEqual(
                        abs(math.cos(headings[3])), 1e-12)
                    self.assertAlmostEqual(phis[3], math.pi / 2.0,
                                           delta=0.0)
                step = 1e-5
                progress = 0.43 * length
                qf_plus = _line_qf(
                    qf_start, displacement, progress + step)
                qf_minus = _line_qf(
                    qf_start, displacement, progress - step)
                centers_plus = _centers_from_pose(qf_plus)
                centers_minus = _centers_from_pose(qf_minus)
                derivative = edge.derivative_at(progress)
                dp_map = {
                    vehicle_id: (x, y)
                    for vehicle_id, x, y in derivative.dp}
                for vehicle_id in (1, 2, 3):
                    finite_dp = (
                        (centers_plus[vehicle_id][0]
                         - centers_minus[vehicle_id][0]) / (2.0 * step),
                        (centers_plus[vehicle_id][1]
                         - centers_minus[vehicle_id][1]) / (2.0 * step))
                    self.assertLessEqual(
                        abs(finite_dp[0] - dp_map[vehicle_id][0]), 1e-4)
                    self.assertLessEqual(
                        abs(finite_dp[1] - dp_map[vehicle_id][1]), 1e-4)
                    theta = headings[vehicle_id]
                    ex, ey = math.cos(theta), math.sin(theta)
                    projection = finite_dp[0] * ex + finite_dp[1] * ey
                    lateral = finite_dp[0] * (-ey) + finite_dp[1] * ex
                    self.assertLessEqual(
                        abs(projection
                            - directions[vehicle_id]
                            * math.cos(phis[vehicle_id])), 1e-4)
                    self.assertLessEqual(
                        abs(lateral
                            - directions[vehicle_id]
                            * math.sin(phis[vehicle_id])), 1e-4)
                case["validation_" + name] = self._assert_metrics(
                    edge).as_dict()
            case["edges"] = edge_records

    # ---- P10 --------------------------------------------------------
    def test_p10_s_spin_with_zero_angle_hold(self):
        deltas = {1: math.pi / 2.0, 2: 0.0, 3: -math.pi / 4.0}
        inputs = {
            "start_qf": [0.4, -0.2, 0.3],
            "headings": {"1": 0.1, "2": -0.4, "3": 1.2},
            "delta_thetas": {
                "1": math.pi / 2.0, "2": 0.0, "3": -math.pi / 4.0},
        }
        with record_case(
                "P10", "S spin with zero-angle hold and canonical S wheels",
                inputs) as case:
            model = _model()
            qf_start = FormationPose(0.4, -0.2, 0.3)
            headings = {1: 0.1, 2: -0.4, 3: 1.2}
            edge = s_spin_edge(model, qf_start, headings, deltas)
            case["edge"] = edge.as_dict()
            self.assertEqual(edge.mode, MODE_S)
            self.assertEqual(edge.primitive, PRIMITIVE_S_SPIN)
            length = math.pi / 2.0
            self._assert_contract(edge, length, "rad", "m/rad", "rad/rad")
            self._assert_directions(edge, {1: 1, 2: 1, 3: -1})
            self._assert_pair_values(
                edge.signed_progress_speeds,
                {1: 0.0, 2: 0.0, 3: 0.0})
            self.assertEqual(len(edge.curvatures), 0)
            self.assertEqual(len(edge.oblique_angles), 0)
            for segment in edge.segments:
                self.assertEqual(segment.mode, SPIN)
                self.assertEqual(segment.travel_distance, 0.0)
                self.assertEqual(segment.start_station, 0.0)
                self.assertAlmostEqual(
                    segment.end_station, length, delta=1e-12)
            self._assert_geometry_samples(
                edge,
                lambda progress: qf_start,
                lambda vehicle_id, progress:
                    headings[vehicle_id]
                    + deltas[vehicle_id] * progress / length)
            # Endpoint signs are preserved, including a zero-delta hold.
            self._assert_heading_values(
                edge.end_headings,
                {1: headings[1] + deltas[1],
                 2: headings[2],
                 3: headings[3] + deltas[3]})
            derivative = edge.derivative_at(0.61)
            self.assertEqual(
                {vehicle_id: (x, y)
                 for vehicle_id, x, y in derivative.dp},
                {1: (0.0, 0.0), 2: (0.0, 0.0), 3: (0.0, 0.0)})
            self._assert_pair_values(
                derivative.dtheta,
                {1: 1.0, 2: 0.0, 3: -0.5})
            self.assertEqual(derivative.dqf, (0.0, 0.0, 0.0))

            limits = edge.limits
            expected_angles, expected_factors = _independent_wheel(
                SPIN, 0.0, limits)
            helper_angles, helper_factors = wheel_configuration(
                Action(SPIN, 0.0, 0.0), limits)
            for actual, expected in zip(
                    helper_angles, expected_angles):
                self.assertAlmostEqual(float(actual), expected, delta=1e-12)
            for actual, expected in zip(
                    helper_factors, expected_factors):
                self.assertAlmostEqual(float(actual), expected, delta=1e-12)
            self.assertAlmostEqual(
                expected_factors[0],
                math.hypot(limits.wheelbase / 2.0, limits.track / 2.0),
                delta=1e-12)
            for vehicle_id in (1, 2, 3):
                for at_end in (False, True):
                    configuration = edge.wheel_configuration(
                        vehicle_id, at_end=at_end)
                    for actual, expected in zip(
                            configuration.angles, expected_angles):
                        self.assertAlmostEqual(
                            actual, expected, delta=1e-9)
                    for actual, expected in zip(
                            configuration.factors, expected_factors):
                        self.assertAlmostEqual(
                            actual, expected, delta=1e-9)
                self.assertLessEqual(
                    max(abs(value)
                        for value in edge.wheel_configuration(
                            vehicle_id).angles),
                    limits.steer_angle_max + 1e-9)
            # The zero-angle vehicle keeps the same S wheels as the moving
            # vehicles; it is not silently given the A zero-curvature set.
            a_angles, a_factors = _independent_wheel(
                ACKERMANN, 0.0, limits)
            differences = edge.angular_delta_from_zero_curvature_a()
            difference_map = {
                item.vehicle_id: item for item in differences}
            self.assertAlmostEqual(
                difference_map[2].max_abs_angle_rad,
                math.atan2(
                    limits.wheelbase / 2.0, limits.track / 2.0),
                delta=1e-9)
            self.assertGreater(
                difference_map[2].max_abs_angle_rad, 0.9)
            for actual, expected in zip(
                    difference_map[2].angles,
                    tuple(math.atan2(
                        math.sin(a - b), math.cos(a - b))
                        for a, b in zip(expected_angles, a_angles))):
                self.assertAlmostEqual(actual, expected, delta=1e-9)
            metrics = self._assert_metrics(edge)
            case["validation"] = metrics.as_dict()
            case["canonical_s_wheel_angles"] = list(expected_angles)
            case["canonical_s_wheel_factors"] = list(expected_factors)
            case["zero_angle_delta_from_a_zero_curvature_rad"] = list(
                difference_map[2].angles)
            self._assert_heading_values(
                edge.start_headings, headings)

            # Tamper: S mode label with A wheel configuration must be
            # rejected by the constructor.
            a_zero_pairs = tuple(
                (vehicle_id, a_angles) for vehicle_id in (1, 2, 3))
            a_factor_pairs = tuple(
                (vehicle_id, a_factors) for vehicle_id in (1, 2, 3))
            error = self._expect_reason(
                "INTERNAL_EDGE_FIELD_MISMATCH",
                lambda: replace(edge, wheel_angles_start=a_zero_pairs))
            case["subcases"].append({
                "name": "constructor_rejects_A_wheels_on_S",
                "status": "REJECTED",
                "reason": error.reason})
            error = self._expect_reason(
                "INTERNAL_EDGE_FIELD_MISMATCH",
                lambda: replace(edge, wheel_factors_start=a_factor_pairs))
            case["subcases"].append({
                "name": "constructor_rejects_A_factors_on_S",
                "status": "REJECTED",
                "reason": error.reason})
            # The independent auditor must also catch a post-construction
            # wheel tamper that bypassed the frozen dataclass constructor.
            tampered = s_spin_edge(model, qf_start, headings, deltas)
            object.__setattr__(
                tampered, "wheel_angles_start", a_zero_pairs)
            error = self._expect_reason(
                "EDGE_VALIDATION_FAILED",
                lambda: validate_uniform_edge(tampered))
            case["subcases"].append({
                "name": "auditor_rejects_A_wheels_on_S",
                "status": "REJECTED",
                "reason": error.reason})

    # ---- P11 --------------------------------------------------------
    def test_p11_s_spin_zero_and_exact_pi(self):
        inputs = {
            "zero_deltas": {"1": 0.0, "2": 0.0, "3": 0.0},
            "pi_deltas": {
                "1": math.pi, "2": -math.pi, "3": 0.0},
            "start_qf": [0.1, 0.2, -0.3],
            "headings": {"1": 0.2, "2": -0.1, "3": 0.4},
        }
        with record_case(
                "P11", "S all-zero and exact +/-pi", inputs) as case:
            model = _model()
            qf_start = FormationPose(0.1, 0.2, -0.3)
            headings = {1: 0.2, 2: -0.1, 3: 0.4}
            zero_error = self._expect_reason(
                "ZERO_SPIN_NO_EDGE",
                lambda: s_spin_edge(
                    model, qf_start, headings,
                    {1: 0.0, 2: 0.0, 3: 0.0}))
            case["rejection"] = {
                "reason": zero_error.reason,
                "detail": zero_error.detail,
                "vehicle_id": zero_error.vehicle_id,
            }
            pi_deltas = {1: math.pi, 2: -math.pi, 3: 0.0}
            edge = s_spin_edge(model, qf_start, headings, pi_deltas)
            case["edge"] = edge.as_dict()
            self._assert_contract(
                edge, math.pi, "rad", "m/rad", "rad/rad")
            self._assert_directions(edge, {1: 1, 2: -1, 3: 1})
            end_map = _pair_map(edge.end_headings)
            self.assertAlmostEqual(
                end_map[1] - headings[1], math.pi, delta=1e-12)
            self.assertAlmostEqual(
                end_map[2] - headings[2], -math.pi, delta=1e-12)
            self.assertAlmostEqual(
                end_map[3] - headings[3], 0.0, delta=1e-12)
            self.assertGreater(end_map[1] - headings[1], 3.0)
            self.assertLess(end_map[2] - headings[2], -3.0)
            self._assert_geometry_samples(
                edge,
                lambda progress: qf_start,
                lambda vehicle_id, progress:
                    headings[vehicle_id]
                    + pi_deltas[vehicle_id] * progress / math.pi)
            halfway = edge.state_at(math.pi / 2.0)
            halfway_map = _pair_map(halfway.headings)
            self.assertAlmostEqual(
                halfway_map[1] - headings[1], math.pi / 2.0, delta=1e-12)
            self.assertAlmostEqual(
                halfway_map[2] - headings[2], -math.pi / 2.0, delta=1e-12)
            self.assertAlmostEqual(
                halfway_map[3], headings[3], delta=1e-12)
            metrics = self._assert_metrics(edge)
            case["validation"] = metrics.as_dict()
            case["end_theta_deltas"] = {
                "1": end_map[1] - headings[1],
                "2": end_map[2] - headings[2],
                "3": end_map[3] - headings[3],
            }
            # Outside the declared domain.
            self._expect_reason(
                "SPIN_INCREMENT_OUT_OF_RANGE",
                lambda: s_spin_edge(
                    model, qf_start, headings,
                    {1: math.pi + 1e-6, 2: 0.0, 3: 0.0}),
                vehicle_id=1)
            self._expect_reason(
                "SPIN_INCREMENT_OUT_OF_RANGE",
                lambda: s_spin_edge(
                    model, qf_start, headings,
                    {1: -math.pi - 1e-6, 2: 0.0, 3: 0.0}),
                vehicle_id=1)
            case["subcases"].append({
                "name": "reject_delta_outside_pi",
                "status": "REJECTED",
                "reason": "SPIN_INCREMENT_OUT_OF_RANGE"})

    # ---- P12 --------------------------------------------------------
    def test_p12_rejections_and_tampering(self):
        inputs = {
            "note": (
                "direction/curvature/segment tampering, illegal mode and "
                "ids, non-finite values, invalid phi and independent auditor")
        }
        with record_case(
                "P12", "Explicit rejections and tamper negatives",
                inputs) as case:
            model = _model()
            straight = a_straight_edge(
                model, FormationPose(0.0, 0.0, 0.0),
                {1: 0.0, 2: math.pi, 3: 0.0}, (2.0, 0.0),
                {1: 1, 2: -1, 3: 1}, allow_reverse=True)
            arc = a_arc_edge(
                model, FormationPose(0.0, 0.0, 0.0),
                {1: math.pi / 2.0, 2: math.pi, 3: 0.0},
                math.pi / 2.0, (0.0, 0.0), {1: 1, 2: 1, 3: 1})
            o_edge = o_translation_edge(
                model, FormationPose(0.0, 0.0, 0.0),
                {1: 0.0, 2: math.pi, 3: 0.0}, (2.0, 0.0),
                {1: 1, 2: -1, 3: 1}, {1: 0.0, 2: 0.0, 3: 0.0},
                allow_reverse=True)
            s_edge = s_spin_edge(
                model, FormationPose(0.0, 0.0, 0.0),
                {1: 0.0, 2: 0.0, 3: 0.0},
                {1: math.pi / 2.0, 2: 0.0, 3: -math.pi / 4.0})
            case["baseline_edges"] = {
                "straight": straight.as_dict(),
                "arc": arc.as_dict(),
                "o": o_edge.as_dict(),
                "s": s_edge.as_dict(),
            }

            def subcase(name, reason, function, **kwargs):
                try:
                    error = self._expect_reason(reason, function, **kwargs)
                except Exception as exc:
                    case["subcases"].append({
                        "name": name,
                        "status": "FAILED",
                        "error": str(exc)})
                    raise
                case["subcases"].append({
                    "name": name,
                    "status": "REJECTED",
                    "reason": error.reason,
                    "vehicle_id": error.vehicle_id,
                    "detail": error.detail,
                })
                return error

            subcase(
                "tamper_s_direction",
                "DIRECTION_MISMATCH",
                lambda: replace(
                    s_edge,
                    directions=((1, -1), (2, 1), (3, 1))),
                vehicle_id=1)
            subcase(
                "tamper_a_direction",
                "A_HEADING_REQUIRES_S_ALIGNMENT",
                lambda: replace(
                    straight,
                    directions=((1, 1), (2, 1), (3, 1))),
                vehicle_id=2)
            subcase(
                "tamper_a_curvature",
                "INTERNAL_EDGE_FIELD_MISMATCH",
                lambda: replace(
                    straight,
                    curvatures=((1, 0.0), (2, 0.5), (3, 0.0))))
            subcase(
                "tamper_arc_curvature",
                "INTERNAL_EDGE_FIELD_MISMATCH",
                lambda: replace(
                    arc,
                    curvatures=((1, -1.0), (2, 1.0), (3, 1.0))))
            subcase(
                "tamper_segment_center",
                "INTERNAL_EDGE_FIELD_MISMATCH",
                lambda: replace(
                    straight,
                    segments=(
                        replace(
                            straight.segments[0],
                            start_center_x=(
                                straight.segments[0].start_center_x
                                + 0.01)),
                    ) + straight.segments[1:]))
            subcase(
                "illegal_mode",
                "INVALID_MODE",
                lambda: replace(straight, mode="X"))
            subcase(
                "mode_primitive_mismatch",
                "MODE_PRIMITIVE_MISMATCH",
                lambda: replace(straight, mode=MODE_S))
            subcase(
                "primitive_mode_mismatch",
                "MODE_PRIMITIVE_MISMATCH",
                lambda: replace(
                    straight, primitive=PRIMITIVE_S_SPIN))
            subcase(
                "missing_vehicle_id",
                "INVALID_VEHICLE_ID_SET",
                lambda: a_straight_edge(
                    model, FormationPose(0.0, 0.0, 0.0),
                    {1: 0.0, 2: math.pi}, (2.0, 0.0),
                    {1: 1, 2: -1, 3: 1}, allow_reverse=True))
            subcase(
                "extra_vehicle_id",
                "INVALID_VEHICLE_ID_SET",
                lambda: a_straight_edge(
                    model, FormationPose(0.0, 0.0, 0.0),
                    {1: 0.0, 2: math.pi, 3: 0.0, 4: 0.0},
                    (2.0, 0.0), {1: 1, 2: -1, 3: 1},
                    allow_reverse=True))
            subcase(
                "non_integer_vehicle_id",
                "INVALID_VEHICLE_ID_SET",
                lambda: a_straight_edge(
                    model, FormationPose(0.0, 0.0, 0.0),
                    {1.0: 0.0, 2: math.pi, 3: 0.0}, (2.0, 0.0),
                    {1: 1, 2: -1, 3: 1}, allow_reverse=True))
            subcase(
                "bad_pair_id_in_constructor",
                "UNSORTED_PAIR_FIELD",
                lambda: replace(
                    straight,
                    start_headings=((1, 0.0), (2, math.pi), (4, 0.0))))
            subcase(
                "nonfinite_displacement",
                "NONFINITE_VALUE",
                lambda: a_straight_edge(
                    model, FormationPose(0.0, 0.0, 0.0),
                    {1: 0.0, 2: math.pi, 3: 0.0},
                    (math.nan, 0.0), {1: 1, 2: -1, 3: 1},
                    allow_reverse=True))
            subcase(
                "nonfinite_heading",
                "NONFINITE_VALUE",
                lambda: a_straight_edge(
                    model, FormationPose(0.0, 0.0, 0.0),
                    {1: math.inf, 2: math.pi, 3: 0.0},
                    (2.0, 0.0), {1: 1, 2: -1, 3: 1},
                    allow_reverse=True))
            subcase(
                "nonfinite_spin_delta",
                "NONFINITE_VALUE",
                lambda: s_spin_edge(
                    model, FormationPose(0.0, 0.0, 0.0),
                    {1: 0.0, 2: 0.0, 3: 0.0},
                    {1: math.nan, 2: 0.0, 3: 0.0}))
            subcase(
                "nonfinite_stored_heading",
                "NONFINITE_VALUE",
                lambda: replace(
                    straight,
                    start_headings=(
                        (1, 0.0), (2, math.nan), (3, 0.0))))
            subcase(
                "oblique_angle_outside_limit",
                "OBLIQUE_ANGLE_LIMIT",
                lambda: o_translation_edge(
                    model, FormationPose(0.0, 0.0, 0.0),
                    {1: 0.0, 2: math.pi, 3: 0.0}, (2.0, 0.0),
                    {1: 1, 2: -1, 3: 1},
                    {1: 0.0, 2: 0.0, 3: math.pi / 2.0 + 0.05},
                    allow_reverse=True),
                vehicle_id=3)
            subcase(
                "oblique_vector_mismatch",
                "OBLIQUE_VECTOR_MISMATCH",
                lambda: o_translation_edge(
                    model, FormationPose(0.0, 0.0, 0.0),
                    {1: 0.0, 2: math.pi, 3: 0.0}, (2.0, 0.0),
                    {1: 1, 2: -1, 3: 1},
                    {1: 0.2, 2: 0.0, 3: 0.0},
                    allow_reverse=True),
                vehicle_id=1)
            subcase(
                "reverse_not_allowed",
                "REVERSE_NOT_ALLOWED",
                lambda: a_straight_edge(
                    model, FormationPose(0.0, 0.0, 0.0),
                    {1: 0.0, 2: math.pi, 3: 0.0}, (2.0, 0.0),
                    {1: 1, 2: -1, 3: 1}, allow_reverse=False),
                vehicle_id=2)
            subcase(
                "invalid_wheel_vehicle_id",
                "INVALID_INTEGER",
                lambda: straight.wheel_configuration(1.0))
            subcase(
                "unknown_wheel_vehicle_id",
                "INVALID_VEHICLE_ID_SET",
                lambda: straight.wheel_configuration(99))
            for name, progress in (
                    ("progress_below_zero", -1e-9),
                    ("progress_above_length", 2.0 + 1e-9),
                    ("nonfinite_progress", math.nan)):
                subcase(
                    name,
                    ("NONFINITE_VALUE" if not math.isfinite(progress)
                     else "PROGRESS_OUT_OF_RANGE"),
                    lambda progress=progress:
                        straight.state_at(progress))

            # Independent auditor catches post-construction tampering that
            # bypassed the frozen dataclass constructor.
            def auditor_case(name, mutate, reason="EDGE_VALIDATION_FAILED"):
                tampered = a_straight_edge(
                    model, FormationPose(0.0, 0.0, 0.0),
                    {1: 0.0, 2: math.pi, 3: 0.0}, (2.0, 0.0),
                    {1: 1, 2: -1, 3: 1}, allow_reverse=True)
                mutate(tampered)
                subcase(
                    name, reason,
                    lambda: validate_uniform_edge(tampered))

            auditor_case(
                "auditor_rejects_curvature_tamper",
                lambda edge: object.__setattr__(
                    edge, "curvatures",
                    ((1, 0.5), (2, 0.0), (3, 0.0))))
            auditor_case(
                "auditor_rejects_direction_tamper",
                lambda edge: object.__setattr__(
                    edge, "directions",
                    ((1, 1), (2, 1), (3, 1))))
            auditor_case(
                "auditor_rejects_start_heading_tamper",
                lambda edge: object.__setattr__(
                    edge, "start_headings",
                    ((1, 0.0), (2, 0.0), (3, 0.0))))

    # ---- result file ------------------------------------------------
    @classmethod
    def tearDownClass(cls):
        payload = {
            "task": "FR01",
            "generated_at_utc": datetime.now(
                timezone.utc).isoformat(),
            "python_version": sys.version,
            "python_executable": sys.executable,
            "package_source": str(
                PACKAGE_SRC / "fwis_v3_optimizer" / "uniform_motion.py"),
            "test_source": str(Path(__file__).resolve()),
            "thresholds": {
                "analytic_geometry_m": 1e-7,
                "rigidity_m": 1e-6,
                "normalized_mode_equation": 1e-6,
                "finite_difference": 1e-4,
            },
            "cases": RESULTS,
        }
        RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
        RESULTS_PATH.write_text(
            json.dumps(
                payload, indent=2, sort_keys=True,
                default=_json_default) + "\n",
            encoding="utf-8")


if __name__ == "__main__":
    unittest.main(verbosity=2)