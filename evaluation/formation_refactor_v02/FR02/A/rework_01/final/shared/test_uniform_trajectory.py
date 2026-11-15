"""FR02-A A06--A10: analytic evaluation, roundtrip, hold and rejection.

Every projected sample is checked against the same exact evaluator.  Finite
differences use only neighbouring analytic states, not the implementation's
reported derivative chain.  Model corruption cases assert stable reasons and
retain the offending input details in the machine-readable result.
"""
from contextlib import contextmanager
import copy
from dataclasses import replace
from datetime import datetime, timezone
from fractions import Fraction
import json
import math
import os
from pathlib import Path
import platform
import struct
import sys
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO_ROOT / "catkin_ws" / "src" / "fwis_v3_optimizer" / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from fwis_v3_optimizer.core import Action
from fwis_v3_optimizer.formation import FormationModel, FormationPose, VehicleOffset
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_kinematics import ACKERMANN, OBLIQUE, SPIN
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
from fwis_v3_optimizer.retiming import MotionLimits, wheel_configuration
from fwis_v3_optimizer.uniform_motion import (
    MODE_A, MODE_O, MODE_S, PRIMITIVE_A_ARC, PRIMITIVE_A_STRAIGHT,
    PRIMITIVE_S_SPIN, UniformMotionError,
    a_arc_edge, a_arc_required_headings, a_straight_edge,
    o_translation_edge, s_spin_edge,
)
from fwis_v3_optimizer.uniform_timing import (
    MotionTiming, SetupTiming, motion_derivatives, schedule_uniform_edges,
)
from fwis_v3_optimizer.uniform_trajectory import (
    CERTIFICATION, MODEL_VERSION, PROJECTION_SEMANTICS,
    UniformTimedPlan, UniformTrajectoryError,
)

OFFSETS = ((1, 1.0, 0.0), (2, 0.0, 1.0), (3, 0.0, -1.0))
BODY_SHAPES = {vehicle_id: VehicleShape(0.738, 0.500)
               for vehicle_id, _, _ in OFFSETS}
NONUNIFORM_FRACTIONS = (
    0.013, 0.071, 0.113, 0.227, 0.319, 0.421,
    0.537, 0.618, 0.733, 0.827, 0.919, 0.987,
)
RESULTS_PATH = Path(os.environ.get(
    "FR02_TRAJECTORY_RESULTS_PATH",
    str(REPO_ROOT / "evaluation" / "formation_refactor_v02" / "FR02" / "A"
        / "trajectory_results.json")))
ARTIFACT_DIR = Path(os.environ.get(
    "FR02_ARTIFACT_DIR",
    str(REPO_ROOT / "evaluation" / "formation_refactor_v02" / "FR02" / "A"
        / "artifacts")))
RESULTS = []


def model(ids_offsets=OFFSETS):
    return FormationModel(tuple(
        VehicleOffset(vehicle_id, x, y) for vehicle_id, x, y in ids_offsets))


def headings(value=0.0):
    return {vehicle_id: float(value) for vehicle_id, _, _ in OFFSETS}


def directions(value=1):
    return {vehicle_id: int(value) for vehicle_id, _, _ in OFFSETS}


def plan_for(edges, limits=None):
    return UniformTimedPlan.from_edges(
        tuple(edges), BODY_SHAPES, timing_limits=limits)


def spin_plan():
    edge = s_spin_edge(
        model(), FormationPose(0.0, 0.0, 0.0), headings(),
        {1: math.pi / 2.0, 2: 0.0, 3: -math.pi / 4.0})
    return plan_for((edge,))


def negative_spin_plan():
    edge = s_spin_edge(
        model(), FormationPose(0.0, 0.0, 0.0), headings(),
        {1: -math.pi / 2.0, 2: -math.pi / 3.0, 3: 0.0})
    return plan_for((edge,))


def straight_plan(displacement=(2.0, 0.0)):
    edge = a_straight_edge(
        model(), FormationPose(0.0, 0.0, 0.0), headings(), displacement,
        directions())
    return plan_for((edge,))


def complex_edges():
    qf = FormationPose(2.0, -1.0, 0.35)
    start_headings = {1: 0.10, 2: -0.20, 3: 0.30}
    delta_psi = 0.60
    pivot = (0.0, 2.0)
    arc_headings = a_arc_required_headings(
        model(), qf, delta_psi, pivot, directions())
    alignment = s_spin_edge(
        model(), qf, start_headings,
        {vehicle_id: arc_headings[vehicle_id] - start_headings[vehicle_id]
         for vehicle_id in start_headings})
    arc = a_arc_edge(
        model(), qf, dict(alignment.end_headings), delta_psi, pivot,
        directions())
    second_spin = s_spin_edge(
        model(), arc.end_qf, dict(arc.end_headings),
        {1: 0.13, 2: -0.17, 3: 0.11})
    translation_headings = dict(second_spin.end_headings)
    translation_angle = math.atan2(0.70, 0.45)
    translation = o_translation_edge(
        model(), arc.end_qf, translation_headings, (0.45, 0.70), directions(),
        {vehicle_id: translation_angle - translation_headings[vehicle_id]
         for vehicle_id in translation_headings})
    return (alignment, arc, second_spin, translation)


@contextmanager
def record_case(case_id, title, inputs):
    record = {"case": case_id, "title": title, "status": "FAILED",
              "inputs": inputs, "subcases": []}
    RESULTS.append(record)
    try:
        yield record
    except Exception as exc:
        record["status"] = "FAILED" if isinstance(exc, AssertionError) else "ERROR"
        record["error_type"] = type(exc).__name__
        record["error"] = str(exc)
        raise
    else:
        record["status"] = "PASSED"


def passed(record, name, **values):
    record["subcases"].append(dict(name=name, status="PASSED", **values))


def expect_rejection(test, record, name, expected_reason, callback, **inputs):
    try:
        callback()
    except UniformMotionError as exc:
        test.assertEqual(
            exc.reason, expected_reason,
            "{}: expected {}, got {} ({})".format(
                name, expected_reason, exc.reason, exc.detail))
        detail = {
            "name": name, "status": "PASSED", "inputs": inputs,
            "rejection": {
                "exception": type(exc).__name__, "reason": exc.reason,
                "detail": exc.detail,
                "vehicle_id": getattr(exc, "vehicle_id", None),
                "edge_index": getattr(exc, "edge_index", None),
                "phase_id": getattr(exc, "phase_id", None),
                "constraint": getattr(exc, "constraint", None),
            },
        }
        record["subcases"].append(detail)
        return exc
    except Exception as exc:
        test.fail("{} raised unexpected {}: {}".format(
            name, type(exc).__name__, exc))
    test.fail("{} was not rejected; expected {}".format(name, expected_reason))


def motion_at(plan, edge_index):
    return next(phase for phase in plan.phases
                if isinstance(phase, MotionTiming) and phase.edge_index == edge_index)


def numeric_difference(actual, expected, path="state"):
    """Return (max absolute numeric error, path, compared numeric count)."""
    if isinstance(expected, bool) or expected is None or isinstance(expected, str):
        if actual != expected or type(actual) is not type(expected):
            raise AssertionError("{}: {!r} != {!r}".format(path, actual, expected))
        return 0.0, path, 0
    if isinstance(expected, (int, float)):
        if isinstance(actual, bool) or not isinstance(actual, (int, float)):
            raise AssertionError("{} is not numeric".format(path))
        return abs(float(actual) - float(expected)), path, 1
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or set(actual) != set(expected):
            raise AssertionError("{} keys differ".format(path))
        best = (0.0, path, 0)
        for key in expected:
            error, error_path, count = numeric_difference(
                actual[key], expected[key], "{}.{}".format(path, key))
            if error >= best[0]:
                best = (error, error_path, best[2] + count)
            else:
                best = (best[0], best[1], best[2] + count)
        return best
    if isinstance(expected, (list, tuple)):
        if not isinstance(actual, (list, tuple)) or len(actual) != len(expected):
            raise AssertionError("{} array differs".format(path))
        best = (0.0, path, 0)
        for index, (a, e) in enumerate(zip(actual, expected)):
            error, error_path, count = numeric_difference(
                a, e, "{}[{}]".format(path, index))
            best = (max(best[0], error),
                    error_path if error >= best[0] else best[1],
                    best[2] + count)
        return best
    raise AssertionError("{} unsupported type {}".format(path, type(expected)))


def assert_state_equal(test, first, second, tolerance=1e-10):
    error, path, count = numeric_difference(first.as_dict(), second.as_dict())
    test.assertLessEqual(error, tolerance, "state mismatch at {}: {}".format(path, error))
    return {"max_abs_error": error, "path": path, "numeric_fields": count}


def independent_arc_qf(edge, progress):
    qf = edge.start_qf
    pivot_x = edge.control.pivot_x
    pivot_y = edge.control.pivot_y
    sign = 1.0 if edge.control.delta_psi > 0.0 else -1.0
    cosine = math.cos(qf.psi)
    sine = math.sin(qf.psi)
    center_x = qf.x + cosine * pivot_x - sine * pivot_y
    center_y = qf.y + sine * pivot_x + cosine * pivot_y
    psi = qf.psi + sign * progress
    cosine_end = math.cos(psi)
    sine_end = math.sin(psi)
    return FormationPose(
        center_x - cosine_end * pivot_x + sine_end * pivot_y,
        center_y - sine_end * pivot_x - cosine_end * pivot_y,
        psi)


def mode_residual(plan, state):
    """Maximum wrapped wheel-angle residual from the declared A/O/S mode."""
    expected_mode = {MODE_A: ACKERMANN, MODE_O: OBLIQUE, MODE_S: SPIN}[state.mode]
    residual = 0.0
    for vehicle in state.vehicles:
        angles, _ = wheel_configuration(
            Action(expected_mode, 0.0, vehicle.mode_control),
            plan.motion_limits)
        for actual, expected in zip(vehicle.wheel_angles, angles):
            residual = max(residual, abs(math.atan2(
                math.sin(actual - expected), math.cos(actual - expected))))
    return residual


def plan_mode_residual(plan):
    worst = 0.0
    for phase in plan.phases:
        if isinstance(phase, MotionTiming):
            for fraction in (0.25, 0.5, 0.75):
                state = plan.state_at(phase.t0_s + fraction * phase.duration_s)
                worst = max(worst, mode_residual(plan, state))
    return worst


def save_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(
        payload, indent=2, sort_keys=True, allow_nan=False) + "\n")


def previous_float(value):
    """Adjacent double below ``value`` without relying on math.nextafter."""
    bits = struct.unpack("Q", struct.pack("d", float(value)))[0]
    if bits & 0x8000000000000000:
        return struct.unpack("d", struct.pack("Q", bits + 1))[0]
    if bits == 0:
        return struct.unpack("d", struct.pack("Q", 0x8000000000000001))[0]
    return struct.unpack("d", struct.pack("Q", bits - 1))[0]


def next_float(value):
    """Adjacent double above ``value`` without relying on math.nextafter."""
    bits = struct.unpack("Q", struct.pack("d", float(value)))[0]
    if bits & 0x8000000000000000:
        if bits == 0x8000000000000000:
            return 0.0
        return struct.unpack("d", struct.pack("Q", bits - 1))[0]
    return struct.unpack("d", struct.pack("Q", bits + 1))[0]


def exact_unit_quintic(tau):
    """Exact rational unit quintic for a rational time fraction."""
    return 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5


def exact_motion_progress(time_s, phase):
    """Independent rational progress for a legal motion-phase time."""
    tau = ((Fraction.from_float(float(time_s))
            - Fraction.from_float(phase.t0_s))
           / Fraction.from_float(phase.duration_s))
    return float(Fraction.from_float(phase.edge.control.parameter_length)
                 * exact_unit_quintic(tau))


def exact_motion_progress_rate(time_s, phase):
    """Independent rational progress rate for a legal motion-phase time."""
    tau = ((Fraction.from_float(float(time_s))
            - Fraction.from_float(phase.t0_s))
           / Fraction.from_float(phase.duration_s))
    rate = 30 * tau ** 2 * (1 - tau) ** 2
    length = Fraction.from_float(phase.edge.control.parameter_length)
    return float(length * rate / Fraction.from_float(phase.duration_s))


def exact_setup_progress(time_s, phase):
    """Independent rational steering progress for a setup-phase time."""
    elapsed = (Fraction.from_float(float(time_s))
               - Fraction.from_float(phase.t0_s)
               - Fraction.from_float(phase.setup_overhead_s))
    tau = elapsed / Fraction.from_float(phase.steering_duration_s)
    return exact_unit_quintic(tau)


def nonfinite_paths(value, path="state"):
    """Return all non-finite leaf paths in a JSON-shaped state."""
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return []
    if isinstance(value, (int, float)):
        return [] if math.isfinite(float(value)) else [path]
    if isinstance(value, dict):
        result = []
        for key, item in value.items():
            result.extend(nonfinite_paths(item, "{}.{}".format(path, key)))
        return result
    if isinstance(value, (list, tuple)):
        result = []
        for index, item in enumerate(value):
            result.extend(nonfinite_paths(item, "{}[{}]".format(path, index)))
        return result
    return [path]


def spin_canonical_wheels(plan):
    """FR01 canonical S angles and magnitude factors for this plan."""
    angles, factors = wheel_configuration(
        Action(SPIN, 0.0, 0.0), plan.limits)
    return (tuple(float(value) for value in angles),
            tuple(float(value) for value in factors))


def spin_reference_yaw_rates(plan, motion, time_s):
    """Independent per-vehicle omega from the S segment headings."""
    tau = (time_s - motion.t0_s) / motion.duration_s
    progress_rate = (
        motion.edge.control.parameter_length
        * 30.0 * tau * tau * (1.0 - tau) * (1.0 - tau)
        / motion.duration_s)
    rates = {}
    for segment in motion.edge.segments:
        station_span = segment.end_station - segment.start_station
        if station_span <= 0.0:
            raise AssertionError("nonpositive S segment station span")
        rates[segment.vehicle_id] = (
            (segment.theta_end - segment.theta_start)
            / station_span * progress_rate)
    return rates


def spin_wheel_vector_rows(plan, state, motion, time_s):
    """Independent wheel-vector projection against canonical folded angles."""
    positions = tuple(
        (x, y)
        for x in (plan.limits.wheelbase / 2.0, -plan.limits.wheelbase / 2.0)
        for y in (plan.limits.track / 2.0, -plan.limits.track / 2.0))
    canonical_angles, canonical_factors = spin_canonical_wheels(plan)
    reference_omega = spin_reference_yaw_rates(plan, motion, time_s)
    rows = []
    for vehicle in state.vehicles:
        if len(vehicle.wheel_angles) != 4:
            raise AssertionError("vehicle {} has {} wheel angles".format(
                vehicle.vehicle_id, len(vehicle.wheel_angles)))
        if len(vehicle.wheel_rolling_speeds) != 4:
            raise AssertionError(
                "vehicle {} has {} rolling speeds".format(
                    vehicle.vehicle_id,
                    len(vehicle.wheel_rolling_speeds)))
        omega = reference_omega[vehicle.vehicle_id]
        for index, ((wheel_x, wheel_y), angle, rolling) in enumerate(
                zip(positions, vehicle.wheel_angles,
                    vehicle.wheel_rolling_speeds)):
            canonical_angle = canonical_angles[index]
            expected_vector = (-omega * wheel_y, omega * wheel_x)
            represented_vector = (rolling * math.cos(canonical_angle),
                                  rolling * math.sin(canonical_angle))
            expected_signed = (
                expected_vector[0] * math.cos(canonical_angle)
                + expected_vector[1] * math.sin(canonical_angle))
            raw_angle = math.atan2(wheel_x, -wheel_y)
            rows.append({
                "vehicle_id": vehicle.vehicle_id,
                "wheel_index": index,
                "wheel_x_m": wheel_x,
                "wheel_y_m": wheel_y,
                "reference_yaw_rate_radps": omega,
                "canonical_wheel_angle_rad": canonical_angle,
                "canonical_wheel_factor_m": canonical_factors[index],
                "reported_yaw_rate_radps": float(vehicle.yaw_rate_radps),
                "wheel_angle_rad": float(angle),
                "raw_spin_direction_rad": raw_angle,
                "folded": (abs(raw_angle) > math.pi / 2.0),
                "rolling_speed_mps": float(rolling),
                "expected_signed_mps": expected_signed,
                "represented_vector": list(represented_vector),
                "expected_vector": list(expected_vector),
                "vector_error_mps": math.hypot(
                    represented_vector[0] - expected_vector[0],
                    represented_vector[1] - expected_vector[1]),
            })
    if len(rows) != 4 * len(state.vehicles):
        raise AssertionError("expected four wheel rows per vehicle")
    return rows


class UniformTrajectoryAcceptance(unittest.TestCase):
    maxDiff = None

    def check_time_derivatives(self, plan, motion):
        state = plan.state_at(motion.t0_s + 0.43 * motion.duration_s)
        h = motion.duration_s * 1e-4
        before = plan.state_at(state.time_s - h)
        after = plan.state_at(state.time_s + h)
        errors = []

        def check(name, actual, numerical):
            error = abs(actual - numerical)
            errors.append(error)
            self.assertLessEqual(
                error, 1e-4, "{} {} analytic={} numerical={} error={}".format(
                    motion.phase_id, name, actual, numerical, error))

        for index, name in enumerate(("x", "y", "psi")):
            p0 = (before.qf.x, before.qf.y, before.qf.psi)[index]
            p1 = (after.qf.x, after.qf.y, after.qf.psi)[index]
            v0 = before.qf_velocity[index]
            v1 = after.qf_velocity[index]
            a0 = before.qf_acceleration[index]
            a1 = after.qf_acceleration[index]
            check("qf_{}_velocity".format(name), state.qf_velocity[index],
                  (p1 - p0) / (2.0 * h))
            check("qf_{}_acceleration".format(name), state.qf_acceleration[index],
                  (v1 - v0) / (2.0 * h))
            check("qf_{}_jerk".format(name), state.qf_jerk[index],
                  (a1 - a0) / (2.0 * h))
        for vehicle_id in plan.vehicle_ids:
            center = state.vehicle(vehicle_id)
            previous = before.vehicle(vehicle_id)
            following = after.vehicle(vehicle_id)
            for index, axis in enumerate((0, 1)):
                check("car{}_position_{}".format(vehicle_id, axis),
                      center.velocity[axis],
                      ((after.vehicle(vehicle_id).x if axis == 0
                        else after.vehicle(vehicle_id).y) -
                       (before.vehicle(vehicle_id).x if axis == 0
                        else before.vehicle(vehicle_id).y)) / (2.0 * h))
                check("car{}_velocity_{}".format(vehicle_id, axis),
                      center.acceleration[axis],
                      (following.velocity[axis] - previous.velocity[axis]) / (2.0 * h))
                check("car{}_acceleration_{}".format(vehicle_id, axis),
                      center.jerk[axis],
                      (following.acceleration[axis] - previous.acceleration[axis])
                      / (2.0 * h))
            check("car{}_yaw_rate".format(vehicle_id), center.yaw_rate_radps,
                  (following.theta - previous.theta) / (2.0 * h))
            check("car{}_yaw_acceleration".format(vehicle_id),
                  center.yaw_acceleration_radps2,
                  (following.yaw_rate_radps - previous.yaw_rate_radps) / (2.0 * h))
            check("car{}_yaw_jerk".format(vehicle_id), center.yaw_jerk_radps3,
                  (following.yaw_acceleration_radps2
                   - previous.yaw_acceleration_radps2) / (2.0 * h))
        for time_s in (motion.t0_s, motion.end_time):
            endpoint = plan.state_at(time_s, side="left")
            self.assertLessEqual(abs(endpoint.progress_acceleration), 1e-9)
            for vehicle in endpoint.vehicles:
                self.assertLessEqual(math.hypot(*vehicle.velocity), 1e-9)
                self.assertLessEqual(abs(vehicle.yaw_rate_radps), 1e-9)
        return max(errors, default=0.0)

    def test_A06_chain_rule_units_and_independent_duration_drivers(self):
        with record_case("A06", "Quintic chain rule and speed/acceleration/yaw/wheel drivers", {
                "finite_difference_relative_h": 1e-4,
                "finite_difference_tolerance": 1e-4}) as record:
            line_plan = straight_plan()
            line_motion = motion_at(line_plan, 0)
            self.assertEqual(line_motion.edge.progress_unit, "m")
            line_error = self.check_time_derivatives(line_plan, line_motion)
            line_residual = plan_mode_residual(line_plan)
            self.assertLessEqual(line_residual, 1e-6)
            passed(record, "A_straight_metre_progress",
                   max_difference_error=line_error,
                   max_mode_residual=line_residual,
                   duration_driver=line_motion.driver)

            complex_plan = plan_for(complex_edges())
            arc_motion = motion_at(complex_plan, 1)
            self.assertEqual(arc_motion.edge.progress_unit, "rad")
            self.assertEqual(arc_motion.edge.primitive, PRIMITIVE_A_ARC)
            arc_error = self.check_time_derivatives(complex_plan, arc_motion)
            complex_residual = plan_mode_residual(complex_plan)
            self.assertLessEqual(complex_residual, 1e-6)
            passed(record, "A_arc_radian_progress",
                   max_difference_error=arc_error,
                   max_mode_residual=complex_residual,
                   duration_driver=arc_motion.driver)

            spin = spin_plan()
            spin_motion = motion_at(spin, 0)
            self.assertEqual(spin_motion.edge.progress_unit, "rad")
            spin_error = self.check_time_derivatives(spin, spin_motion)
            spin_residual = plan_mode_residual(spin)
            self.assertLessEqual(spin_residual, 1e-6)
            self.assertEqual(spin_motion.driver["constraint"], "yaw_rate")
            passed(record, "S_radian_progress_and_yaw_driver",
                   max_difference_error=spin_error,
                   max_mode_residual=spin_residual,
                   duration_driver=spin_motion.driver)

            long_edge = o_translation_edge(
                model(), FormationPose(0.0, 0.0, 0.0), headings(), (10.0, 0.0),
                directions(), headings(0.0))
            speed_plan = plan_for((long_edge,))
            speed_motion = motion_at(speed_plan, 0)
            self.assertEqual(speed_motion.driver["constraint"], "speed")
            speed_residual = plan_mode_residual(speed_plan)
            self.assertLessEqual(speed_residual, 1e-6)
            passed(record, "speed_driver", duration_driver=speed_motion.driver,
                   duration_s=speed_motion.duration_s,
                   max_mode_residual=speed_residual)

            acceleration_limits = replace(
                FormationTimingLimits(), v_max=2.0, jerk_max=1000.0)
            unit_edge = o_translation_edge(
                model(), FormationPose(0.0, 0.0, 0.0), headings(), (1.0, 0.0),
                directions(), headings(0.0))
            acceleration_plan = plan_for((unit_edge,), acceleration_limits)
            acceleration_motion = motion_at(acceleration_plan, 0)
            self.assertEqual(acceleration_motion.driver["constraint"], "acceleration")
            acceleration_residual = plan_mode_residual(acceleration_plan)
            self.assertLessEqual(acceleration_residual, 1e-6)
            passed(record, "acceleration_driver",
                   duration_driver=acceleration_motion.driver,
                   max_mode_residual=acceleration_residual,
                   limits=acceleration_limits.as_dict())

            wheel_limits = replace(
                FormationTimingLimits(), v_max=10.0, a_max=100.0, jerk_max=10000.0)
            wheel_plan = plan_for((unit_edge,), wheel_limits)
            wheel_motion = motion_at(wheel_plan, 0)
            self.assertEqual(wheel_motion.driver["constraint"], "wheel_speed")
            self.assertEqual(wheel_motion.driver["limit"], wheel_limits.wheel_speed_max)
            wheel_residual = plan_mode_residual(wheel_plan)
            self.assertLessEqual(wheel_residual, 1e-6)
            passed(record, "wheel_speed_driver", duration_driver=wheel_motion.driver,
                   max_mode_residual=wheel_residual,
                   limits=wheel_limits.as_dict())

    def test_A07_setup_interior_and_both_boundary_sides(self):
        with record_case("A07", "Initial setup and S motion are separate bounded phases", {
                "interior_samples_per_phase": 12,
                "boundary_policy": "left-closed/right-open; final endpoint defined"}) as record:
            plan = spin_plan()
            setup = plan.phases[0]
            motion = plan.phases[1]
            self.assertIsInstance(setup, SetupTiming)
            self.assertIsInstance(motion, MotionTiming)
            self.assertEqual(setup.kind, "initial_setup")
            self.assertEqual(setup.to_mode, MODE_S)
            max_rate = 0.0
            max_accel = 0.0
            position_error = 0.0
            heading_error = 0.0
            expected_centers = plan.model.centers(setup.qf)
            expected_headings = dict(setup.headings)
            times = [setup.t0_s + fraction * setup.duration_s
                     for fraction in NONUNIFORM_FRACTIONS]
            for time_s in times:
                state = plan.state_at(time_s)
                self.assertEqual(state.phase_id, setup.phase_id)
                self.assertEqual(state.mode, MODE_A)
                for vehicle in state.vehicles:
                    position_error = max(
                        position_error,
                        abs(vehicle.x - expected_centers[vehicle.vehicle_id][0]),
                        abs(vehicle.y - expected_centers[vehicle.vehicle_id][1]))
                    heading_error = max(
                        heading_error, abs(math.atan2(
                            math.sin(vehicle.theta - expected_headings[vehicle.vehicle_id]),
                            math.cos(vehicle.theta - expected_headings[vehicle.vehicle_id]))))
                    self.assertEqual(vehicle.velocity, (0.0, 0.0))
                    self.assertEqual(vehicle.acceleration, (0.0, 0.0))
                    self.assertEqual(vehicle.yaw_rate_radps, 0.0)
                    self.assertEqual(vehicle.yaw_acceleration_radps2, 0.0)
                    demand = setup.demand(vehicle.vehicle_id)
                    local = time_s - setup.t0_s
                    tau = max(0.0, min(1.0,
                        (local - setup.setup_overhead_s) / setup.steering_duration_s))
                    unit = tau ** 3 * (10.0 - 15.0 * tau + 6.0 * tau ** 2)
                    unit_rate = 30.0 * tau ** 2 * (1.0 - tau) ** 2
                    unit_accel = 60.0 * tau * (1.0 - tau) * (1.0 - 2.0 * tau)
                    for index, (start, delta) in enumerate(zip(
                            demand.wheel_angles_start, demand.wheel_angle_deltas)):
                        self.assertAlmostEqual(vehicle.wheel_angles[index],
                                               start + delta * unit, places=11)
                        self.assertAlmostEqual(
                            vehicle.wheel_angular_rates[index],
                            delta * unit_rate / setup.steering_duration_s, places=11)
                        self.assertAlmostEqual(
                            vehicle.wheel_angular_accelerations[index],
                            delta * unit_accel / setup.steering_duration_s ** 2,
                            places=11)
                    max_rate = max(max_rate,
                                   *(abs(value) for value in vehicle.wheel_angular_rates))
                    max_accel = max(max_accel,
                                    *(abs(value) for value in
                                      vehicle.wheel_angular_accelerations))
            self.assertLessEqual(max_rate, plan.limits.steer_rate_max * (1.0 + 1e-9))
            self.assertLessEqual(max_accel, plan.limits.steer_accel_max * (1.0 + 1e-9))
            self.assertLessEqual(position_error, 1e-9)
            self.assertLessEqual(heading_error, 1e-9)
            left = plan.state_at(setup.end_time, side="left")
            right = plan.state_at(setup.end_time, side="right")
            automatic = plan.state_at(setup.end_time)
            self.assertEqual(left.mode, MODE_A)
            self.assertEqual(right.mode, MODE_S)
            self.assertEqual(automatic.as_dict(), right.as_dict())
            self.assertEqual(left.centers, right.centers)
            self.assertEqual(left.headings, right.headings)
            self.assertTrue(all(state.mode == MODE_S for state in (right, automatic)))
            for vehicle in right.vehicles:
                demand = setup.demand(vehicle.vehicle_id)
                for actual, expected in zip(vehicle.wheel_angles,
                                            demand.wheel_angles_end):
                    self.assertAlmostEqual(actual, expected, places=12)
            totals = plan.phase_totals()
            self.assertEqual(totals["initial_setup_s"], setup.duration_s)
            self.assertEqual(totals["motion_s"], motion.duration_s)
            self.assertAlmostEqual(plan.duration_s,
                                   setup.duration_s + motion.duration_s, places=12)
            passed(record, "initial_setup_boundary_and_quintic_wheels",
                   setup=setup.as_dict(), motion=motion.as_dict(),
                   sampled_max_steer_rate=max_rate,
                   sampled_max_steer_acceleration=max_accel,
                   setup_position_error=position_error,
                   setup_heading_error=heading_error, totals=totals)

    def test_A08_versioned_roundtrip_all_phases_and_exact_arc_projection(self):
        with record_case("A08", "All phases >=11 nonuniform interior samples and exact nonzero-pivot arc", {
                "interior_samples_per_phase": len(NONUNIFORM_FRACTIONS),
                "model_version": MODEL_VERSION,
                "projection_semantics": PROJECTION_SEMANTICS}) as record:
            plan = plan_for(complex_edges())
            payload = plan.to_model_json()
            restored = UniformTimedPlan.from_model_json(payload)
            model_error, model_error_path, model_fields = numeric_difference(
                restored.to_model_dict(), plan.to_model_dict(), "model")
            self.assertLessEqual(model_error, 1e-10, model_error_path)
            self.assertEqual(restored.model_sha256(), plan.model_sha256())
            comparisons = []
            sample_times = {0.0, plan.duration_s}
            arc_samples = []
            arc_error = 0.0
            max_rigidity_error = 0.0
            chord_error = 0.0
            for phase_index, phase in enumerate(plan.phases):
                self.assertIs(type(restored.phases[phase_index]), type(phase))
                local_times = [phase.t0_s, phase.end_time] + [
                    phase.t0_s + fraction * phase.duration_s
                    for fraction in NONUNIFORM_FRACTIONS]
                for time_s in local_times:
                    sample_times.add(time_s)
                    comparison = assert_state_equal(
                        self, plan.state_at(time_s), restored.state_at(time_s))
                    comparison.update(phase_id=phase.phase_id, time_s=time_s)
                    comparisons.append(comparison)
                if phase_index > 0:
                    for side in ("left", "right"):
                        comparison = assert_state_equal(
                            self, plan.state_at(phase.t0_s, side=side),
                            restored.state_at(phase.t0_s, side=side))
                        comparison.update(phase_id=phase.phase_id,
                                          time_s=phase.t0_s, side=side)
                        comparisons.append(comparison)
                if isinstance(phase, MotionTiming) and phase.edge_index == 1:
                    for fraction in NONUNIFORM_FRACTIONS:
                        time_s = phase.t0_s + fraction * phase.duration_s
                        state = plan.state_at(time_s)
                        expected = independent_arc_qf(phase.edge, state.progress)
                        error = max(abs(state.qf.x - expected.x),
                                    abs(state.qf.y - expected.y),
                                    abs(state.qf.psi - expected.psi))
                        arc_error = max(arc_error, error)
                        chord_fraction = state.progress / phase.edge.control.parameter_length
                        chord_x = (phase.edge.start_qf.x + chord_fraction *
                                   (phase.edge.end_qf.x - phase.edge.start_qf.x))
                        chord_y = (phase.edge.start_qf.y + chord_fraction *
                                   (phase.edge.end_qf.y - phase.edge.start_qf.y))
                        deviation = math.hypot(state.qf.x - chord_x, state.qf.y - chord_y)
                        chord_error = max(chord_error, deviation)
                        centers = dict((item[0], (item[1], item[2])) for item in state.centers)
                        for first, second in ((1, 2), (1, 3), (2, 3)):
                            distance = math.hypot(centers[first][0] - centers[second][0],
                                                  centers[first][1] - centers[second][1])
                            expected_distance = math.sqrt(2.0) if 1 in (first, second) else 2.0
                            max_rigidity_error = max(
                                max_rigidity_error, abs(distance - expected_distance))
                        arc_samples.append({
                            "time_s": time_s, "progress_rad": state.progress,
                            "qf": [state.qf.x, state.qf.y, state.qf.psi],
                            "independent_qf": [expected.x, expected.y, expected.psi],
                            "abs_error": error, "chord_deviation_m": deviation,
                        })
            self.assertGreaterEqual(len(arc_samples), 11)
            self.assertLessEqual(arc_error, 1e-9)
            self.assertLessEqual(max_rigidity_error, 1e-6)
            self.assertGreater(chord_error, 0.01)
            times = tuple(sorted(sample_times))
            projection = plan.sample(times)
            restored_projection = restored.sample(times)
            self.assertEqual(projection.semantic_version, PROJECTION_SEMANTICS)
            self.assertEqual(projection.certification, CERTIFICATION)
            self.assertEqual(projection.model_sha256, plan.model_sha256())
            self.assertEqual(len(projection.states), len(times))
            projection_error = 0.0
            for index, (state, restored_state) in enumerate(zip(
                    projection.states, restored_projection.states)):
                expected = plan.state_at(times[index])
                result = assert_state_equal(self, state, expected)
                restored_result = assert_state_equal(self, restored_state, expected)
                projection_error = max(projection_error, result["max_abs_error"],
                                       restored_result["max_abs_error"])
                for vehicle in state.vehicles:
                    point = projection.timed_trajectory.trajectories[
                        vehicle.vehicle_id][index]
                    self.assertAlmostEqual(point.center_x, vehicle.x, places=12)
                    self.assertAlmostEqual(point.center_y, vehicle.y, places=12)
                    self.assertAlmostEqual(point.theta, vehicle.theta, places=12)
                    self.assertAlmostEqual(point.center_speed_mps,
                                           vehicle.center_speed_mps, places=12)
                    self.assertAlmostEqual(point.center_yaw_rate_radps,
                                           vehicle.yaw_rate_radps, places=12)

            shortened = copy.deepcopy(plan.to_model_dict())
            last_motion = next(item for item in reversed(shortened["phases"])
                               if item["kind"] == "motion")
            old_duration = last_motion["duration_s"]
            last_motion["duration_s"] = old_duration * 0.97
            last_motion["end_time_s"] = last_motion["t0_s"] + last_motion["duration_s"]
            shortened["duration_s"] = last_motion["end_time_s"]
            restored_shortened = UniformTimedPlan.from_model_dict(shortened)
            self.assertAlmostEqual(restored_shortened.duration_s,
                                   shortened["duration_s"], places=12)
            self.assertNotAlmostEqual(restored_shortened.duration_s,
                                      plan.duration_s, places=12)

            model_dict = plan.to_model_dict()
            save_json(ARTIFACT_DIR / "model.json", model_dict)
            save_json(ARTIFACT_DIR / "projection.json", projection.as_dict())
            save_json(ARTIFACT_DIR / "initial_state_limits.json", {
                "initial_state": model_dict["initial_state"],
                "requested_goal": model_dict["requested_goal"],
                "vehicle_shapes": model_dict["vehicle_shapes"],
                "timing_limits": model_dict["timing_limits"],
                "s8_motion_limits": model_dict["s8_motion_limits"],
            })
            save_json(ARTIFACT_DIR / "phase_table.json", {
                "phases": plan.phase_table(), "totals": plan.phase_totals()})
            save_json(ARTIFACT_DIR / "setup_demands.json", [
                phase.as_dict() for phase in plan.phases
                if isinstance(phase, SetupTiming)])
            save_json(ARTIFACT_DIR / "motion_peaks.json", [
                phase.as_dict() for phase in plan.phases
                if isinstance(phase, MotionTiming)])
            save_json(ARTIFACT_DIR / "arc_internal_samples.json", arc_samples)
            max_roundtrip = max(item["max_abs_error"] for item in comparisons)
            self.assertLessEqual(max_roundtrip, 1e-10)
            save_json(ARTIFACT_DIR / "roundtrip_errors.json", {
                "state_comparisons": len(comparisons),
                "model_numeric_fields_compared": model_fields,
                "max_model_roundtrip_abs_error": model_error,
                "model_roundtrip_error_path": model_error_path,
                "original_model_sha256": plan.model_sha256(),
                "restored_model_sha256": restored.model_sha256(),
                "model_sha256_equal": (restored.model_sha256()
                                       == plan.model_sha256()),
                "model_sha256_semantics": "12 significant digits; digest only",
                "max_roundtrip_abs_error": max_roundtrip,
                "max_projection_abs_error": projection_error,
                "arc_interior_samples": len(arc_samples),
                "max_arc_qf_abs_error": arc_error,
                "max_rigidity_error_m": max_rigidity_error,
                "max_arc_vs_chord_deviation_m": chord_error,
                "stored_duration_decode_probe": {
                    "original_duration_s": plan.duration_s,
                    "stored_positive_duration_s": shortened["duration_s"],
                    "restored_duration_s": restored_shortened.duration_s,
                    "retimed": False,
                },
                "comparison_location_of_max": max(
                    comparisons, key=lambda item: item["max_abs_error"]),
            })
            passed(record, "roundtrip_and_projection",
                   state_comparisons=len(comparisons),
                   projection_samples=len(times), arc_samples=len(arc_samples),
                   max_roundtrip_abs_error=max_roundtrip,
                   max_projection_abs_error=projection_error,
                   max_arc_qf_abs_error=arc_error,
                   max_rigidity_error_m=max_rigidity_error,
                   max_arc_vs_chord_deviation_m=chord_error,
                   artifacts=str(ARTIFACT_DIR))

    def test_A09_explicit_zero_motion_and_unmet_requests(self):
        with record_case("A09", "Only an explicit identical start/goal is a zero-duration hold", {
                "hold_duration_s": 0.0, "hold_sample_count": 1}) as record:
            qf = FormationPose(1.0, -2.0, 0.4)
            start_headings = {1: 0.2, 2: -0.3, 3: 0.7}
            hold = UniformTimedPlan.from_edges(
                (), BODY_SHAPES, model=model(), start_qf=qf,
                start_headings=start_headings, expected_end_qf=qf,
                expected_end_headings=start_headings)
            self.assertEqual(hold.duration_s, 0.0)
            self.assertEqual(hold.phases, ())
            projection = hold.sample()
            self.assertEqual(projection.times, (0.0,))
            state = projection.states[0]
            self.assertEqual(state.phase_kind, "hold")
            self.assertEqual(state.mode, MODE_A)
            self.assertEqual(state.qf, qf)
            self.assertEqual(dict(state.headings), start_headings)
            self.assertTrue(state.stationary)
            for vehicle in state.vehicles:
                self.assertEqual(vehicle.velocity, (0.0, 0.0))
                self.assertEqual(vehicle.yaw_rate_radps, 0.0)
                self.assertTrue(all(value == 0.0 for value in
                                    vehicle.wheel_angular_rates))
            passed(record, "true_zero_motion_single_sample", state=state.as_dict(),
                   projection=projection.as_dict(include_states=False))

            changed_headings = dict(start_headings)
            changed_headings[2] += 0.4
            expect_rejection(
                self, record, "empty_sequence_heading_request_not_hold",
                "REQUESTED_HEADING_MISMATCH",
                lambda: UniformTimedPlan.from_edges(
                    (), BODY_SHAPES, model=model(), start_qf=qf,
                    start_headings=start_headings, expected_end_qf=qf,
                    expected_end_headings=changed_headings),
                start_headings=start_headings, goal_headings=changed_headings)
            spin = s_spin_edge(model(), qf, start_headings, {1: 0.0, 2: 0.4, 3: 0.0})
            spin = plan_for((spin,))
            self.assertGreater(spin.duration_s, 0.0)
            self.assertEqual(spin.edges[0].primitive, PRIMITIVE_S_SPIN)
            passed(record, "same_position_heading_change_is_real_S",
                   duration_s=spin.duration_s, phases=spin.phase_table())

            expect_rejection(
                self, record, "empty_sequence_position_request_not_hold",
                "REQUESTED_POSE_MISMATCH",
                lambda: UniformTimedPlan.from_edges(
                    (), BODY_SHAPES, model=model(), start_qf=qf,
                    start_headings=start_headings,
                    expected_end_qf=FormationPose(1.1, -2.0, 0.4),
                    expected_end_headings=start_headings),
                requested_goal=[1.1, -2.0, 0.4])
            straight = a_straight_edge(
                model(), FormationPose(0.0, 0.0, 0.0), headings(), (1.0, 0.0),
                directions())
            expect_rejection(
                self, record, "nonempty_plan_unmet_goal_rejected",
                "REQUESTED_POSE_MISMATCH",
                lambda: UniformTimedPlan.from_edges(
                    (straight,), BODY_SHAPES,
                    expected_end_qf=FormationPose(2.0, 0.0, 0.0),
                    expected_end_headings=straight.end_headings),
                factory_goal=[1.0, 0.0, 0.0], requested_goal=[2.0, 0.0, 0.0])

    def test_A10_explicit_rejections_and_real_duration_limit(self):
        with record_case("A10", "Continuity, identity, finite values, progress/time and duration rejection", {
                "time_continuity_tolerance_s": 1e-9,
                "max_block_duration_s": 120.0}) as record:
            first = o_translation_edge(
                model(), FormationPose(0.0, 0.0, 0.0), headings(), (0.5, 0.0),
                directions(), headings(0.0))
            shifted = o_translation_edge(
                model(), FormationPose(0.51, 0.0, 0.0), headings(), (0.2, 0.0),
                directions(), headings(0.0))
            expect_rejection(self, record, "qf_position_jump", "EDGE_QF_DISCONTINUITY",
                             lambda: schedule_uniform_edges((first, shifted)),
                             jump_m=0.01)
            heading_jump = o_translation_edge(
                model(), first.end_qf, headings(0.2), (0.2, 0.0), directions(),
                headings(-0.2))
            expect_rejection(self, record, "unwrapped_heading_jump",
                             "EDGE_HEADING_DISCONTINUITY",
                             lambda: schedule_uniform_edges((first, heading_jump)),
                             jump_rad=0.2)

            other_ids = ((1, 1.0, 0.0), (2, 0.0, 1.0), (4, 0.0, -1.0))
            id_headings = {1: 0.0, 2: 0.0, 4: 0.0}
            id_directions = {1: 1, 2: 1, 4: 1}
            wrong_ids = o_translation_edge(
                model(other_ids), first.end_qf, id_headings, (0.2, 0.0),
                id_directions, id_headings)
            expect_rejection(self, record, "vehicle_ids_mismatch",
                             "EDGE_VEHICLE_IDS_MISMATCH",
                             lambda: schedule_uniform_edges((first, wrong_ids)),
                             first_ids=[1, 2, 3], second_ids=[1, 2, 4])
            other_offsets = ((1, 1.1, 0.0), (2, 0.0, 1.0), (3, 0.0, -1.0))
            wrong_offsets = o_translation_edge(
                model(other_offsets), first.end_qf, headings(), (0.2, 0.0),
                directions(), headings(0.0))
            expect_rejection(self, record, "offsets_mismatch", "EDGE_OFFSETS_MISMATCH",
                             lambda: schedule_uniform_edges((first, wrong_offsets)),
                             first_offset_x=1.0, second_offset_x=1.1)
            wrong_geometry = o_translation_edge(
                model(), FormationPose(0.0, 0.0, 0.0), headings(), (0.5, 0.0),
                directions(), headings(0.0),
                limits=replace(MotionLimits(), wheelbase=0.55))
            expect_rejection(self, record, "geometry_limits_mismatch",
                             "TIMING_GEOMETRY_LIMIT_MISMATCH",
                             lambda: schedule_uniform_edges((wrong_geometry,)),
                             edge_wheelbase=0.55, timing_wheelbase=0.494)
            expect_rejection(self, record, "zero_min_mode_dwell",
                             "MIN_MODE_DWELL_REQUIRED",
                             lambda: schedule_uniform_edges(
                                 (first,), replace(FormationTimingLimits(),
                                                   min_mode_dwell_s=0.0)),
                             min_mode_dwell_s=0.0)

            plan = spin_plan()
            base_model = plan.to_model_dict()
            unknown_version = copy.deepcopy(base_model)
            unknown_version["model_version"] = "uniform_timed_plan.v999"
            expect_rejection(self, record, "unknown_model_version",
                             "UNKNOWN_MODEL_VERSION",
                             lambda: UniformTimedPlan.from_model_dict(unknown_version),
                             model_version="uniform_timed_plan.v999")
            nonfinite = copy.deepcopy(base_model)
            nonfinite["phases"][-1]["duration_s"] = float("nan")
            expect_rejection(self, record, "nonfinite_model_input", "NONFINITE_VALUE",
                             lambda: UniformTimedPlan.from_model_dict(nonfinite),
                             field="phases[-1].duration_s", value="NaN")
            missing_id = copy.deepcopy(base_model)
            missing_id["vehicle_shapes"] = missing_id["vehicle_shapes"][:-1]
            expect_rejection(self, record, "missing_vehicle_id", "INVALID_VEHICLE_ID_SET",
                             lambda: UniformTimedPlan.from_model_dict(missing_id),
                             removed_vehicle_id=3)
            duplicate_id = copy.deepcopy(base_model)
            duplicate_id["vehicle_shapes"].append(
                copy.deepcopy(duplicate_id["vehicle_shapes"][0]))
            expect_rejection(self, record, "duplicate_vehicle_id", "DUPLICATE_VEHICLE_ID",
                             lambda: UniformTimedPlan.from_model_dict(duplicate_id),
                             duplicated_vehicle_id=1)
            compact = json.dumps(base_model, separators=(",", ":"), allow_nan=False)
            duplicate_json = compact.replace('"plan_id":0',
                                             '"plan_id":0,"plan_id":0', 1)
            expect_rejection(self, record, "duplicate_json_key", "DUPLICATE_JSON_KEY",
                             lambda: UniformTimedPlan.from_model_json(duplicate_json),
                             key="plan_id")
            illegal_enum = copy.deepcopy(base_model)
            illegal_enum["edges"][0]["factory"]["primitive"] = "not_a_primitive"
            expect_rejection(self, record, "illegal_primitive", "INVALID_PRIMITIVE",
                             lambda: UniformTimedPlan.from_model_dict(illegal_enum),
                             primitive="not_a_primitive")
            redundant = copy.deepcopy(base_model)
            redundant["phases"][-1]["end_time_s"] += 0.1
            expect_rejection(self, record, "redundant_value_conflict",
                             "REDUNDANT_VALUE_MISMATCH",
                             lambda: UniformTimedPlan.from_model_dict(redundant),
                             field="phases[-1].end_time_s", increment_s=0.1)

            motion = motion_at(plan, 0)
            for name, progress, reason in (
                    ("negative_progress", -1e-6, "PROGRESS_OUT_OF_RANGE"),
                    ("progress_above_L", motion.edge.control.parameter_length + 1e-6,
                     "PROGRESS_OUT_OF_RANGE"),
                    ("nan_progress", float("nan"), "NONFINITE_VALUE")):
                expect_rejection(self, record, name, reason,
                                 lambda progress=progress: motion_derivatives(motion, progress),
                                 progress=repr(progress), L=motion.edge.control.parameter_length)
            for name, time_s, reason in (
                    ("negative_time", -1e-6, "TIME_OUT_OF_RANGE"),
                    ("time_above_end", plan.duration_s + 1e-6, "TIME_OUT_OF_RANGE"),
                    ("nan_time", float("nan"), "NONFINITE_VALUE")):
                expect_rejection(self, record, name, reason,
                                 lambda time_s=time_s: plan.state_at(time_s),
                                 time_s=repr(time_s), duration_s=plan.duration_s)
            for name, duration in (("zero_phase_duration", 0.0),
                                   ("negative_phase_duration", -1.0)):
                corrupted = copy.deepcopy(base_model)
                corrupted["phases"][-1]["duration_s"] = duration
                expect_rejection(self, record, name, "NONPOSITIVE_PHASE_DURATION",
                                 lambda corrupted=corrupted:
                                     UniformTimedPlan.from_model_dict(corrupted),
                                 duration_s=duration)
            for name, shift in (("phase_time_gap", 0.05),
                                ("phase_time_overlap", -0.05)):
                corrupted = copy.deepcopy(base_model)
                corrupted["phases"][-1]["t0_s"] += shift
                expect_rejection(self, record, name, "PHASE_TIME_DISCONTINUITY",
                                 lambda corrupted=corrupted:
                                     UniformTimedPlan.from_model_dict(corrupted),
                                 motion_t0_shift_s=shift)

            long_edge = o_translation_edge(
                model(), FormationPose(0.0, 0.0, 0.0), headings(), (1000.0, 0.0),
                directions(), headings(0.0))
            error = expect_rejection(self, record, "max_block_duration_exceeded",
                                     "MOTION_DURATION_LIMIT",
                                     lambda: schedule_uniform_edges((long_edge,)),
                                     displacement_m=1000.0, max_block_duration_s=120.0)
            self.assertIn("max_block_duration_s", error.detail)
            self.assertEqual(error.constraint, "speed")
            self.assertEqual(error.vehicle_id, 1)
            save_json(ARTIFACT_DIR / "negative_cases.json", record)

    def test_R1_stable_near_endpoint_progress_and_roundtrip(self):
        with record_case("R1", "Legal near-endpoint times never overflow progress", {
                "plans": ["straight", "mixed_sign_spin", "negative_spin",
                          "S_arc_S_O"],
                "relative_distances": ["1e-5", "1e-6", "1e-7", "1e-8",
                                       "1e-9", "1e-10", "1e-11", "1e-12",
                                       "1e-13", "1e-14", "1e-15"],
                "includes_predecessor_floats": True,
                "includes_serialized_models": True}) as record:
            scenarios = (
                ("straight", straight_plan()),
                ("mixed_sign_spin", spin_plan()),
                ("negative_spin", negative_spin_plan()),
                ("S_arc_S_O", plan_for(complex_edges())),
            )
            max_progress_overshoot = 0.0
            min_progress = math.inf
            max_monotonic_drop = 0.0
            max_roundtrip_error = 0.0
            max_center_error = 0.0
            max_rigidity_error = 0.0
            max_arc_error = 0.0
            max_progress_reference_error = 0.0
            max_progress_rate_reference_error = 0.0
            max_setup_angle_overshoot = 0.0
            max_setup_angle_reference_error = 0.0
            max_sample_error = 0.0
            near_count = 0
            boundary_side_checks = 0
            rounded_boundary_count = 0
            skipped_boundary_requests = []
            phase_records = []
            for scenario_name, plan in scenarios:
                restored = UniformTimedPlan.from_model_dict(plan.to_model_dict())
                expected_distances = {}
                offsets = {item.vehicle_id: (item.x, item.y)
                           for item in plan.model.offsets}
                ids = sorted(offsets)
                for first_index, first in enumerate(ids):
                    for second in ids[first_index + 1:]:
                        expected_distances[(first, second)] = math.hypot(
                            offsets[first][0] - offsets[second][0],
                            offsets[first][1] - offsets[second][1])
                for phase_index, phase in enumerate(plan.phases):
                    if not isinstance(phase, MotionTiming):
                        continue
                    length = phase.edge.control.parameter_length
                    near = {}

                    def add_near(time_s, label):
                        if phase.t0_s <= time_s < phase.end_time:
                            near.setdefault(time_s, []).append(label)
                        else:
                            nonlocal rounded_boundary_count
                            rounded_boundary_count += 1
                            skipped_boundary_requests.append({
                                "scenario": scenario_name,
                                "phase_id": phase.phase_id,
                                "label": label,
                                "requested_time_s": time_s,
                                "t0_s": phase.t0_s,
                                "end_time_s": phase.end_time,
                            })

                    for exponent in range(5, 16):
                        fraction = 10.0 ** (-exponent)
                        add_near(phase.end_time - phase.duration_s * fraction,
                                 "end_relative_1e-{}".format(exponent))
                        add_near(phase.t0_s + phase.duration_s * fraction,
                                 "start_relative_1e-{}".format(exponent))
                    add_near(previous_float(phase.end_time), "predecessor_end_float")
                    add_near(next_float(phase.t0_s), "successor_start_float")

                    start_state = plan.state_at(phase.t0_s, side="right")
                    end_state = plan.state_at(phase.end_time, side="left")
                    self.assertEqual(start_state.phase_id, phase.phase_id)
                    self.assertEqual(end_state.phase_id, phase.phase_id)
                    self.assertGreaterEqual(start_state.progress, 0.0)
                    self.assertEqual(start_state.progress, 0.0)
                    self.assertLessEqual(end_state.progress, length)
                    self.assertEqual(end_state.progress, length)

                    for boundary_time, expected in (
                            (phase.t0_s, "start"),
                            (phase.end_time, "end")):
                        for side in ("left", "right"):
                            boundary_state = plan.state_at(
                                boundary_time, side=side)
                            restored_boundary = restored.state_at(
                                boundary_time, side=side)
                            comparison = assert_state_equal(
                                self, boundary_state, restored_boundary)
                            max_roundtrip_error = max(
                                max_roundtrip_error,
                                comparison["max_abs_error"])
                            self.assertEqual(
                                nonfinite_paths(boundary_state.as_dict()), [])
                            if expected == "start" and side == "right":
                                self.assertEqual(
                                    boundary_state.phase_id, phase.phase_id)
                                self.assertEqual(boundary_state.progress, 0.0)
                            if expected == "end" and side == "left":
                                self.assertEqual(
                                    boundary_state.phase_id, phase.phase_id)
                                self.assertEqual(boundary_state.progress, length)
                            boundary_side_checks += 1

                    checkpoint_states = [(phase.t0_s, 0.0, start_state),
                                         (phase.end_time, length, end_state)]
                    for time_s, labels in sorted(near.items()):
                        state = plan.state_at(time_s)
                        self.assertEqual(state.phase_id, phase.phase_id)
                        restored_state = restored.state_at(time_s)
                        self.assertEqual(restored_state.phase_id, phase.phase_id)
                        comparison = assert_state_equal(
                            self, state, restored_state)
                        max_roundtrip_error = max(
                            max_roundtrip_error, comparison["max_abs_error"])
                        self.assertEqual(nonfinite_paths(state.as_dict()), [])
                        overshoot = state.progress - length
                        max_progress_overshoot = max(
                            max_progress_overshoot, overshoot)
                        min_progress = min(min_progress, state.progress)
                        self.assertGreaterEqual(state.progress, 0.0)
                        self.assertLessEqual(state.progress, length)

                        expected_progress = exact_motion_progress(
                            time_s, phase)
                        expected_rate = exact_motion_progress_rate(
                            time_s, phase)
                        progress_error = abs(
                            state.progress - expected_progress)
                        rate_error = abs(
                            state.progress_rate - expected_rate)
                        max_progress_reference_error = max(
                            max_progress_reference_error, progress_error)
                        max_progress_rate_reference_error = max(
                            max_progress_rate_reference_error, rate_error)
                        self.assertLessEqual(
                            progress_error, 1e-12 * max(1.0, length),
                            "{} progress disagrees with exact time->quintic"
                            .format(phase.phase_id))
                        self.assertLessEqual(
                            rate_error,
                            1e-12 * max(1.0, abs(expected_rate)),
                            "{} progress_rate disagrees with exact quintic"
                            .format(phase.phase_id))

                        expected_geometric = phase.edge.state_at(
                            expected_progress)
                        centers = plan.model.centers(expected_geometric.qf)
                        center_error = 0.0
                        for vehicle in state.vehicles:
                            center_error = max(
                                center_error,
                                abs(vehicle.x - centers[vehicle.vehicle_id][0]),
                                abs(vehicle.y - centers[vehicle.vehicle_id][1]))
                        max_center_error = max(max_center_error, center_error)
                        self.assertLessEqual(center_error, 1e-7)
                        centers_by_id = {
                            vehicle_id: (x, y)
                            for vehicle_id, x, y in state.centers}
                        for (first, second), expected_distance in (
                                expected_distances.items()):
                            actual_distance = math.hypot(
                                centers_by_id[first][0] - centers_by_id[second][0],
                                centers_by_id[first][1] - centers_by_id[second][1])
                            error = abs(actual_distance - expected_distance)
                            max_rigidity_error = max(max_rigidity_error, error)
                            self.assertLessEqual(error, 1e-7)
                        if phase.edge.primitive == PRIMITIVE_A_ARC:
                            expected_qf = independent_arc_qf(
                                phase.edge, expected_progress)
                            error = max(
                                abs(state.qf.x - expected_qf.x),
                                abs(state.qf.y - expected_qf.y),
                                abs(state.qf.psi - expected_qf.psi))
                            max_arc_error = max(max_arc_error, error)
                            self.assertLessEqual(error, 1e-9)
                        checkpoint_states.append(
                            (time_s, state.progress, state))
                        near_count += 1
                        phase_records.append({
                            "scenario": scenario_name,
                            "phase_id": phase.phase_id,
                            "time_s": time_s,
                            "labels": labels,
                            "progress": state.progress,
                            "expected_progress": expected_progress,
                            "progress_reference_error": progress_error,
                            "progress_rate_reference_error": rate_error,
                            "length": length,
                            "progress_overshoot": overshoot,
                            "roundtrip_max_abs_error":
                                comparison["max_abs_error"],
                            "center_error_m": center_error,
                            "step_from_end_s": phase.end_time - time_s,
                        })

                    checkpoints = [(time_s, progress)
                                   for time_s, progress, _ in checkpoint_states]
                    checkpoints.sort(key=lambda item: item[0])
                    for (_, before), (_, after) in zip(
                            checkpoints, checkpoints[1:]):
                        drop = before - after
                        max_monotonic_drop = max(max_monotonic_drop, drop)
                        self.assertLessEqual(
                            drop, 1e-12 * max(1.0, length),
                            "{} nonmonotone near endpoint".format(
                                phase.phase_id))
                    self.assertLessEqual(max_progress_overshoot, 0.0)
                    self.assertGreaterEqual(min_progress, 0.0)
                    self.assertLess(min_progress, math.inf)
                    phase_records.append({
                        "scenario": scenario_name,
                        "phase_id": phase.phase_id,
                        "near_time_count": len(near),
                        "start_progress": start_state.progress,
                        "end_progress": end_state.progress,
                        "length": length,
                    })

                # Setup phases share the stable normalized quintic as well.
                for phase in plan.phases:
                    if not isinstance(phase, SetupTiming):
                        continue
                    for exponent in range(5, 16):
                        time_s = (
                            phase.end_time
                            - phase.duration_s * 10.0 ** (-exponent))
                        if not phase.t0_s <= time_s < phase.end_time:
                            rounded_boundary_count += 1
                            skipped_boundary_requests.append({
                                "scenario": scenario_name,
                                "phase_id": phase.phase_id,
                                "label": "setup_end_relative_1e-{}".format(
                                    exponent),
                                "requested_time_s": time_s,
                                "t0_s": phase.t0_s,
                                "end_time_s": phase.end_time,
                            })
                            continue
                        state = plan.state_at(time_s)
                        restored_state = restored.state_at(time_s)
                        self.assertEqual(state.phase_id, phase.phase_id)
                        comparison = assert_state_equal(
                            self, state, restored_state)
                        max_roundtrip_error = max(
                            max_roundtrip_error, comparison["max_abs_error"])
                        self.assertEqual(nonfinite_paths(state.as_dict()), [])
                        expected_setup_level = exact_setup_progress(
                            time_s, phase)
                        sample_overshoot = 0.0
                        sample_reference_error = 0.0
                        for vehicle in state.vehicles:
                            demand = phase.demand(vehicle.vehicle_id)
                            for actual, start, delta in zip(
                                    vehicle.wheel_angles,
                                    demand.wheel_angles_start,
                                    demand.wheel_angle_deltas):
                                low = min(start, start + delta)
                                high = max(start, start + delta)
                                overshoot = max(low - actual, actual - high, 0.0)
                                sample_overshoot = max(
                                    sample_overshoot, overshoot)
                                expected_angle = float(
                                    Fraction.from_float(start)
                                    + Fraction.from_float(delta)
                                    * expected_setup_level)
                                reference_error = abs(
                                    actual - expected_angle)
                                sample_reference_error = max(
                                    sample_reference_error, reference_error)
                                self.assertLessEqual(overshoot, 1e-12)
                                self.assertLessEqual(
                                    reference_error, 1e-12,
                                    "{} setup wheel angle disagrees with "
                                    "exact time->quintic".format(
                                        phase.phase_id))
                        max_setup_angle_overshoot = max(
                            max_setup_angle_overshoot, sample_overshoot)
                        max_setup_angle_reference_error = max(
                            max_setup_angle_reference_error,
                            sample_reference_error)
                        phase_records.append({
                            "scenario": scenario_name,
                            "phase_id": phase.phase_id,
                            "time_s": time_s,
                            "setup_step_from_end_s":
                                phase.end_time - time_s,
                            "wheel_angle_overshoot_rad": sample_overshoot,
                            "wheel_angle_reference_error_rad":
                                sample_reference_error,
                        })

                # Projection near the final endpoint uses the same evaluator.
                last_motion = next(
                    item for item in reversed(plan.phases)
                    if isinstance(item, MotionTiming))
                near_sample = max(
                    last_motion.t0_s,
                    plan.duration_s - plan.duration_s * 1e-5)
                times = (0.0, near_sample, plan.duration_s)
                self.assertGreater(times[1] - times[0], 1e-9)
                self.assertGreater(times[2] - times[1], 1e-9)
                projection = plan.sample(times)
                for index, time_s in enumerate(times):
                    comparison = assert_state_equal(
                        self, projection.states[index], plan.state_at(time_s))
                    max_sample_error = max(
                        max_sample_error, comparison["max_abs_error"])
                passed(record, "projection_near_final_endpoint",
                       scenario=scenario_name, times=list(times),
                       max_projection_state_error=max_sample_error)

            for scenario_name, plan in scenarios:
                above = next_float(plan.duration_s)
                expect_rejection(
                    self, record, "{}_one_ulp_above_plan_end".format(
                        scenario_name), "TIME_OUT_OF_RANGE",
                    lambda plan=plan, above=above: plan.state_at(above),
                    plan=scenario_name, time=repr(above),
                    duration_s=plan.duration_s)
            expect_rejection(
                self, record, "negative_one_ulp_time", "TIME_OUT_OF_RANGE",
                lambda: scenarios[0][1].state_at(previous_float(0.0)),
                time=repr(previous_float(0.0)))

            save_json(ARTIFACT_DIR / "R1_near_endpoint.json", {
                "scenarios": [name for name, _ in scenarios],
                "near_time_count": near_count,
                "boundary_side_checks": boundary_side_checks,
                "rounded_boundary_skipped_count": rounded_boundary_count,
                "skipped_boundary_requests": skipped_boundary_requests,
                "max_progress_overshoot": max_progress_overshoot,
                "min_progress": min_progress,
                "max_monotonic_drop": max_monotonic_drop,
                "monotonic_tolerance": 1e-12,
                "max_roundtrip_abs_error": max_roundtrip_error,
                "max_center_error_m": max_center_error,
                "max_rigidity_error_m": max_rigidity_error,
                "max_arc_qf_error": max_arc_error,
                "max_progress_reference_error":
                    max_progress_reference_error,
                "max_progress_rate_reference_error":
                    max_progress_rate_reference_error,
                "max_setup_wheel_angle_overshoot_rad":
                    max_setup_angle_overshoot,
                "max_setup_wheel_angle_reference_error_rad":
                    max_setup_angle_reference_error,
                "max_projection_state_error": max_sample_error,
                "records": phase_records,
            })
            passed(record, "near_endpoint_evaluations",
                   near_time_count=near_count,
                   boundary_side_checks=boundary_side_checks,
                   rounded_boundary_skipped_count=rounded_boundary_count,
                   max_progress_overshoot=max_progress_overshoot,
                   max_monotonic_drop=max_monotonic_drop,
                   max_roundtrip_abs_error=max_roundtrip_error,
                   max_center_error_m=max_center_error,
                   max_rigidity_error_m=max_rigidity_error,
                   max_arc_qf_error=max_arc_error,
                   max_progress_reference_error=
                       max_progress_reference_error,
                   max_progress_rate_reference_error=
                       max_progress_rate_reference_error,
                   max_setup_wheel_angle_overshoot_rad=
                       max_setup_angle_overshoot,
                   max_setup_wheel_angle_reference_error_rad=
                       max_setup_angle_reference_error,
                   max_projection_state_error=max_sample_error)

    def test_R2_signed_spin_wheel_rolling_vectors(self):
        with record_case("R2", "S rolling speeds follow folded wheel directions", {
                "plans": ["mixed_sign_spin", "negative_spin"],
                "samples_per_motion": [0.05, 0.25, 0.5, 0.75, 0.95],
                "vector_tolerance_mps": 1e-7}) as record:
            scenarios = (
                ("mixed_sign_spin", spin_plan()),
                ("negative_spin", negative_spin_plan()),
            )
            max_vector_error = 0.0
            max_peak_error = 0.0
            max_roundtrip_error = 0.0
            max_yaw_reference_error = 0.0
            max_angle_error = 0.0
            max_speed_reference_error = 0.0
            row_records = []
            zero_vehicle_records = []
            folded_count = 0
            unfolded_count = 0
            for scenario_name, plan in scenarios:
                restored = UniformTimedPlan.from_model_dict(plan.to_model_dict())
                motion = motion_at(plan, 0)
                canonical_angles, canonical_factors = spin_canonical_wheels(plan)
                for fraction in (0.05, 0.25, 0.5, 0.75, 0.95):
                    time_s = motion.t0_s + fraction * motion.duration_s
                    state = plan.state_at(time_s)
                    restored_state = restored.state_at(time_s)
                    comparison = assert_state_equal(self, state, restored_state)
                    max_roundtrip_error = max(
                        max_roundtrip_error, comparison["max_abs_error"])
                    reference_omega = spin_reference_yaw_rates(
                        plan, motion, time_s)
                    rows = spin_wheel_vector_rows(plan, state, motion, time_s)
                    self.assertEqual(len(rows), 4 * len(state.vehicles))
                    folded_count += sum(row["folded"] for row in rows)
                    unfolded_count += sum(not row["folded"] for row in rows)
                    state_wheel_peak_error = 0.0
                    for row in rows:
                        self.assertLessEqual(
                            row["vector_error_mps"], 1e-7,
                            "{} vehicle {} wheel {} vector mismatch".format(
                                scenario_name, row["vehicle_id"],
                                row["wheel_index"]))
                        max_vector_error = max(
                            max_vector_error, row["vector_error_mps"])
                        rolling_error = abs(
                            row["rolling_speed_mps"]
                            - row["expected_signed_mps"])
                        self.assertLessEqual(
                            rolling_error,
                            1e-12 * max(1.0,
                                        abs(row["expected_signed_mps"])))
                        yaw_error = abs(
                            row["reported_yaw_rate_radps"]
                            - row["reference_yaw_rate_radps"])
                        max_yaw_reference_error = max(
                            max_yaw_reference_error, yaw_error)
                        self.assertLessEqual(
                            yaw_error,
                            1e-12 * max(1.0,
                                        abs(row["reference_yaw_rate_radps"])))
                        row["scenario"] = scenario_name
                        row["time_s"] = time_s
                        row_records.append(row)
                    for vehicle in state.vehicles:
                        self.assertEqual(len(vehicle.wheel_angles), 4)
                        self.assertEqual(
                            len(vehicle.wheel_rolling_speeds), 4)
                        angle_error = max(
                            abs(actual - canonical)
                            for actual, canonical in zip(
                                vehicle.wheel_angles, canonical_angles))
                        max_angle_error = max(max_angle_error, angle_error)
                        self.assertLessEqual(angle_error, 1e-15)
                        omega_ref = reference_omega[vehicle.vehicle_id]
                        expected_peak_now = max(canonical_factors) * abs(omega_ref)
                        speed_error = abs(
                            vehicle.wheel_speed_mps - expected_peak_now)
                        max_speed_reference_error = max(
                            max_speed_reference_error, speed_error)
                        self.assertLessEqual(
                            speed_error,
                            1e-12 * max(1.0, expected_peak_now))
                        expected_peak = motion.peaks[
                            vehicle.vehicle_id]["wheel_speed"]
                        self.assertLessEqual(
                            vehicle.wheel_speed_mps, expected_peak + 1e-12)
                        error = 0.0
                        if fraction == 0.5:
                            error = abs(
                                vehicle.wheel_speed_mps - expected_peak)
                            state_wheel_peak_error = max(
                                state_wheel_peak_error, error)
                            max_peak_error = max(max_peak_error, error)
                            self.assertLessEqual(error, 1e-12)
                        if omega_ref == 0.0:
                            self.assertEqual(vehicle.yaw_rate_radps, 0.0)
                            self.assertTrue(all(
                                value == 0.0 for value in
                                vehicle.wheel_rolling_speeds))
                            self.assertGreater(max(abs(value) for value in
                                                   vehicle.wheel_angles), 0.9)
                            zero_vehicle_records.append({
                                "scenario": scenario_name,
                                "time_s": time_s,
                                "vehicle_id": vehicle.vehicle_id,
                                "wheel_angles_rad": list(vehicle.wheel_angles),
                                "rolling_speeds_mps": list(
                                    vehicle.wheel_rolling_speeds),
                            })
                    signs = {1 if row["rolling_speed_mps"] > 0.0
                             else -1 if row["rolling_speed_mps"] < 0.0 else 0
                             for row in rows
                             if row["reference_yaw_rate_radps"] != 0.0}
                    self.assertEqual(signs, {-1, 1})
                    passed(record, "{}_fraction_{}".format(
                        scenario_name, fraction),
                        time_s=time_s,
                        max_vector_error_mps=max(row["vector_error_mps"]
                                                 for row in rows),
                        max_yaw_reference_error_radps=max_yaw_reference_error,
                        max_canonical_angle_error_rad=max_angle_error,
                        max_wheel_speed_reference_error_mps=
                            max_speed_reference_error,
                        max_peak_error_mps=state_wheel_peak_error,
                        roundtrip_max_abs_error=comparison["max_abs_error"])

            self.assertGreater(folded_count, 0)
            self.assertGreater(unfolded_count, 0)
            self.assertGreater(len(zero_vehicle_records), 0)
            save_json(ARTIFACT_DIR / "R2_spin_wheel_vectors.json", {
                "scenarios": [name for name, _ in scenarios],
                "folded_wheel_samples": folded_count,
                "unfolded_wheel_samples": unfolded_count,
                "zero_yaw_vehicle_samples": zero_vehicle_records,
                "max_vector_error_mps": max_vector_error,
                "max_yaw_rate_reference_error_radps":
                    max_yaw_reference_error,
                "max_canonical_wheel_angle_error_rad": max_angle_error,
                "max_wheel_speed_reference_error_mps":
                    max_speed_reference_error,
                "max_peak_wheel_speed_error_mps": max_peak_error,
                "max_roundtrip_abs_error": max_roundtrip_error,
                "rows": row_records,
            })
            passed(record, "folded_unfolded_and_zero_yaw_wheels",
                   folded_wheel_samples=folded_count,
                   unfolded_wheel_samples=unfolded_count,
                   zero_yaw_vehicle_samples=len(zero_vehicle_records),
                   max_vector_error_mps=max_vector_error,
                   max_yaw_rate_reference_error_radps=
                       max_yaw_reference_error,
                   max_canonical_wheel_angle_error_rad=max_angle_error,
                   max_wheel_speed_reference_error_mps=
                       max_speed_reference_error,
                   max_peak_wheel_speed_error_mps=max_peak_error,
                   max_roundtrip_abs_error=max_roundtrip_error)

    @classmethod
    def tearDownClass(cls):
        RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema": "fr02a.trajectory_results.v1",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "status": "PASSED" if RESULTS and all(
                item["status"] == "PASSED" for item in RESULTS) else "FAILED",
            "python": sys.version,
            "platform": platform.platform(),
            "repo_root": str(REPO_ROOT),
            "package_src": str(PACKAGE_SRC),
            "import_paths": {
                "uniform_motion": sys.modules["fwis_v3_optimizer.uniform_motion"].__file__,
                "uniform_timing": sys.modules["fwis_v3_optimizer.uniform_timing"].__file__,
                "uniform_trajectory": sys.modules["fwis_v3_optimizer.uniform_trajectory"].__file__,
            },
            "cases": RESULTS,
        }
        save_json(RESULTS_PATH, payload)


if __name__ == "__main__":
    unittest.main(verbosity=2)
