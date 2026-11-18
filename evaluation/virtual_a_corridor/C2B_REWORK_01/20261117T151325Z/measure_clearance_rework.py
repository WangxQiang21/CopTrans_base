"""Measure the C2-B rework clearance world-frame contract.

All expectations are handwritten from the task contract; the script never
calls the rework helper to derive an expectation.  The legacy query formula is
kept only as the negative/baseline control.
"""
import argparse
import json
import math

import numpy as np
from unittest import mock

import casadi

from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    CachedClearanceField,
    PlannerConfig,
)
from fwis_v3_optimizer.formation_continuous_a.optimizer import (
    _ClearanceProxy,
    _build_casadi_nlp,
    _clearance_proxy_angles,
    _clearance_world_body,
    _numeric_objective,
    _proxy_numeric_penalty,
    _search_packet,
)
from virtual_a_g4_support import make_request


class RecordingField:
    def __init__(self, direction=0.0):
        self.direction = direction
        self.queries = []

    def direction_at(self, x, y):
        self.queries.append((float(x), float(y)))
        return 0.5, self.direction, 1.0

    def value_at(self, x, y):
        self.queries.append((float(x), float(y)))
        return 0.5


def asymmetric_field():
    values = tuple(
        tuple(0.01 + 0.002 * column + 0.003 * row for column in range(36))
        for row in range(32))
    return CachedClearanceField(
        origin_x=-2.0, origin_y=-8.0, resolution_m=0.5,
        grid_sha256="1" * 64, values=values)


def proxy_for_field(field, sample_count=72):
    angles = tuple(
        tuple(0.1 * sample + 0.03 * vehicle for vehicle in range(3))
        for sample in range(sample_count))
    return _ClearanceProxy(
        field=field, angles=angles, target_m=0.05, weight=5.0e4)


def legacy_queries(request, x, y, psi, kappa):
    """Pre-rework local-frame body centres; expectation/negative control only."""
    queries = []
    for offset in request.model.offsets:
        a, b = offset.x, offset.y
        center_x = x + math.cos(psi) * a - math.sin(psi) * b
        center_y = y + math.sin(psi) * a + math.cos(psi) * b
        theta = psi + math.atan2(a * kappa, 1.0 - b * kappa)
        queries.append((center_x, center_y, theta))
    return tuple(queries)


def hand_penalty(request, proxy, sample, queries):
    row = proxy.angles[sample]
    total = 0.0
    for index, offset in enumerate(request.model.offsets):
        shape = request.vehicle_shapes[offset.vehicle_id]
        center_x, center_y, theta = queries[index]
        clearance = proxy.field.value_at(center_x, center_y)
        support = (
            0.5 * shape.length_m * abs(math.cos(theta - row[index]))
            + 0.5 * shape.width_m * abs(math.sin(theta - row[index])))
        hinge = proxy.target_m + support - clearance
        if hinge > 0.0:
            total += hinge * hinge
    return total


def build_off_origin_request():
    angle, tx, ty = 0.7, 3.0, -2.0
    goal = FormationPose(
        tx + math.cos(angle) * 8.0 - math.sin(angle) * 2.0,
        ty + math.sin(angle) * 8.0 + math.cos(angle) * 2.0,
        angle)
    return make_request(
        FormationPose(tx, ty, angle), goal,
        start_headings={1: angle, 2: angle, 3: angle})


def build_origin_request():
    return make_request(
        FormationPose(0.0, 0.0, 0.0), FormationPose(8.0, 0.0, 0.0),
        start_headings={1: 0.0, 2: 0.0, 3: 0.0})


def world_center_section():
    request = build_off_origin_request()
    angle, tx, ty = 0.7, 3.0, -2.0
    field = RecordingField(direction=0.25)
    angles = _clearance_proxy_angles(
        field, request, ((0.0, 0.0, 0.0, 0.0),))
    cosine, sine = math.cos(angle), math.sin(angle)
    expected = (
        (tx + cosine, ty + sine),
        (tx - sine, ty + cosine),
        (tx + sine, ty - cosine))
    errors = [math.hypot(actual[0] - want[0], actual[1] - want[1])
              for actual, want in zip(field.queries, expected)]
    fallback_field = RecordingField(direction=None)
    fallback = _clearance_proxy_angles(
        fallback_field, request, ((0.0, 0.0, 0.0, 0.2),))
    expected_theta = (angle + math.atan2(0.2, 1.0), angle, angle)
    theta_errors = [abs(actual - want)
                    for actual, want in zip(fallback[0], expected_theta)]
    direction_errors = [abs(actual - 0.25) for actual in angles[0]]
    return {
        "start": [tx, ty, angle],
        "field_queries": field.queries,
        "hand_world_centres": expected,
        "body_centre_errors_m": errors,
        "max_body_centre_error_m": max(errors),
        "stored_directions": angles[0],
        "direction_semantics_errors": direction_errors,
        "fallback_angles": fallback[0],
        "hand_fallback_angles": expected_theta,
        "fallback_theta_errors_rad": theta_errors,
        "max_fallback_theta_error_rad": max(theta_errors),
    }


def objective_section():
    request = build_off_origin_request()
    config = PlannerConfig(
        guidance_mode="corridor", solver="scipy", max_initials=1)
    packet = _search_packet(request, config)
    n = int(config.intervals)
    collocation = int(config.collocation_points)
    proxy = proxy_for_field(asymmetric_field(), sample_count=n * collocation)
    z = np.zeros(2 * n + 3, dtype=float)
    z[0] = 8.0
    z[1:1 + n + 1] = 0.04 * np.sin(0.3 * np.arange(n + 1))
    z[1 + n + 1:] = 0.03 * np.cos(0.2 * np.arange(n + 1))
    numeric = _numeric_objective(
        z, request, config, packet, collocation, clearance_proxy=proxy)
    numeric_without = _numeric_objective(
        z, request, config, packet, collocation)
    captured = {}

    def fake_nlpsol(name, plugin, nlp, options=None):
        captured["nlp"] = nlp
        return object()

    with mock.patch.object(casadi, "nlpsol", side_effect=fake_nlpsol):
        _build_casadi_nlp(request, config, packet, clearance_proxy=proxy)
    function = casadi.Function(
        "rework_objective", [captured["nlp"]["x"]], [captured["nlp"]["f"]])
    casadi_value = float(function(z))
    numeric_value = float(numeric)
    difference = abs(casadi_value - numeric_value)
    relative = difference / max(1.0, abs(numeric_value))
    return {
        "start": [3.0, -2.0, 0.7],
        "kappa_vector": [float(value) for value in z[1:1 + n + 1]],
        "eta_vector": [float(value) for value in z[1 + n + 1:]],
        "S": float(z[0]),
        "field_origin": [-2.0, -8.0],
        "field_shape": [32, 36],
        "field_resolution_m": 0.5,
        "numeric_objective": numeric_value,
        "casadi_objective": casadi_value,
        "absolute_difference": float(difference),
        "relative_difference": float(relative),
        "numeric_without_proxy": float(numeric_without),
        "clearance_contribution": abs(numeric_value - float(numeric_without)),
        "tolerance": 1e-9 * max(1.0, abs(numeric_value)),
    }


def negative_section():
    request = build_off_origin_request()
    proxy = proxy_for_field(asymmetric_field())
    x, y, psi, kappa, sample = 4.0, 1.0, 0.6, 0.15, 0
    correct = _proxy_numeric_penalty(
        proxy, request, x, y, psi, kappa, sample)
    queries = legacy_queries(request, x, y, psi, kappa)
    wrong = hand_penalty(request, proxy, sample, queries)
    world = [
        _clearance_world_body(request, x, y, psi, kappa, offset)
        for offset in request.model.offsets]
    separation = max(
        math.hypot(legacy[0] - world_pose[0], legacy[1] - world_pose[1])
        for legacy, world_pose in zip(queries, world))
    return {
        "state": [x, y, psi, kappa, sample],
        "correct_world_penalty": correct,
        "wrong_local_penalty": wrong,
        "penalty_difference": abs(correct - wrong),
        "legacy_queries": queries,
        "world_queries": world,
        "max_query_separation_m": separation,
        "difference_gate": 1e-3,
        "separation_gate_m": 1.0,
    }


def origin_section():
    request = build_origin_request()
    proxy = proxy_for_field(asymmetric_field())
    x, y, psi, kappa, sample = 1.5, -0.7, 0.35, 0.12, 3
    computed = [
        _clearance_world_body(request, x, y, psi, kappa, offset)
        for offset in request.model.offsets]
    expected = legacy_queries(request, x, y, psi, kappa)
    errors = [math.hypot(actual[0] - want[0], actual[1] - want[1])
              for actual, want in zip(computed, expected)]
    theta_errors = [abs(actual[2] - want[2])
                    for actual, want in zip(computed, expected)]
    penalty = _proxy_numeric_penalty(
        proxy, request, x, y, psi, kappa, sample)
    expected_penalty = hand_penalty(request, proxy, sample, expected)
    return {
        "state": [x, y, psi, kappa, sample],
        "max_centre_error_m": max(errors),
        "max_theta_error_rad": max(theta_errors),
        "penalty": float(penalty),
        "hand_penalty": float(expected_penalty),
        "penalty_error": abs(float(penalty) - float(expected_penalty)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("out")
    args = parser.parse_args()
    body = world_center_section()
    objective = objective_section()
    negative = negative_section()
    origin = origin_section()
    gates = {
        "body_centre_error_within_1e12": bool(
            body["max_body_centre_error_m"] <= 1e-12),
        "fallback_theta_within_1e12": bool(
            body["max_fallback_theta_error_rad"] <= 1e-12),
        "direction_semantics_exact": bool(
            max(body["direction_semantics_errors"]) == 0.0),
        "numeric_casadi_within_1e9": bool(
            objective["absolute_difference"] <= objective["tolerance"]),
        "clearance_term_active": bool(
            objective["clearance_contribution"] > 1e-6),
        "negative_difference_detected": bool(
            negative["penalty_difference"] > negative["difference_gate"]),
        "negative_separation_detected": bool(
            negative["max_query_separation_m"]
            > negative["separation_gate_m"]),
        "origin_centre_regression_within_1e12": bool(
            origin["max_centre_error_m"] <= 1e-12),
        "origin_penalty_regression_within_1e12": bool(
            origin["penalty_error"] <= 1e-12),
    }
    summary = {
        "world_centre": body,
        "objective": objective,
        "negative_control": negative,
        "origin_regression": origin,
        "gates": gates,
        "all_gates_passed": all(gates.values()),
    }
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=1, sort_keys=True, default=str)
        handle.write("\n")
    print(json.dumps(summary, indent=1, sort_keys=True, default=str))


if __name__ == "__main__":
    main()