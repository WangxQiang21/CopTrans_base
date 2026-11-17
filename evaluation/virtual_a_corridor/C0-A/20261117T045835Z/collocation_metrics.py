#!/usr/bin/env python3
"""Measure the C0-A collocation numbers asserted by the targeted tests.

Read-only: writes only ``collocation_metrics.json`` next to this script.
"""
import json
import math
from pathlib import Path

import numpy as np

from fwis_v3_optimizer.continuous_geometry import ControlKnot
from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a import optimizer as optimizer_module
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    CachedClearanceField,
    PlannerConfig,
)
from fwis_v3_optimizer.formation_continuous_a.geometry import (
    C2CurvatureProfile,
    VirtualAPath,
)
from test_virtual_a_collocation import (
    GAUSS_POINTS,
    MODEL,
    _casadi_nlp_functions,
    _legacy_cumulative_states,
    _linear_curvature_profile,
    _packet,
)
from virtual_a_g4_support import make_request


def straight_metrics():
    request, config, packet = _packet(FormationPose(4.0, 0.0, 0.0))
    n = packet["intervals"]
    vector = np.zeros(2 * n + 3, dtype=float)
    vector[0] = 4.0
    states = optimizer_module._collocation_states(
        vector, packet, GAUSS_POINTS)
    nodes = optimizer_module._gauss(GAUSS_POINTS)[0]
    h = 4.0 / n
    worst = 0.0
    for j in range(n):
        for q, (x, y, psi, kappa) in zip(
                nodes, states[j * GAUSS_POINTS:(j + 1) * GAUSS_POINTS]):
            worst = max(worst, abs(x - (j * h + h * q)), abs(y),
                        abs(psi), abs(kappa))
    legacy = _legacy_cumulative_states(vector, packet, GAUSS_POINTS)
    legacy_error = max(
        abs(legacy[j][0] - h * nodes[j % GAUSS_POINTS])
        for j in range(GAUSS_POINTS))
    corrected_error = max(
        abs(states[j][0] - h * nodes[j % GAUSS_POINTS])
        for j in range(GAUSS_POINTS))
    return {
        "worst_all_gauss_errors_m_rad": worst,
        "legacy_first_piece_x_error_m": legacy_error,
        "corrected_first_piece_x_error_m": corrected_error,
    }


def symmetry_metrics():
    h = 0.75
    coeffs = optimizer_module._coeffs(h, 0.11, 0.27, -0.4, 0.35)
    nodes = optimizer_module._gauss(GAUSS_POINTS)[0]
    psi0 = 0.3
    angle = 0.7
    base, base_heads = optimizer_module._piece_position_samples(
        h, coeffs, psi0, nodes, math.sin, math.cos)
    rotated, rotated_heads = optimizer_module._piece_position_samples(
        h, coeffs, psi0 + angle, nodes, math.sin, math.cos)
    mirrored, mirrored_heads = optimizer_module._piece_position_samples(
        h, tuple(-value for value in coeffs), -psi0, nodes, math.sin,
        math.cos)
    worst_rotation = 0.0
    worst_mirror = 0.0
    for (x, y), (rx, ry), heading, rot_heading in zip(
            base, rotated, base_heads, rotated_heads):
        worst_rotation = max(
            worst_rotation,
            abs(rx - (math.cos(angle) * x - math.sin(angle) * y)),
            abs(ry - (math.sin(angle) * x + math.cos(angle) * y)),
            abs(rot_heading - (heading + angle)))
    for (x, y), (mx, my), heading, mirror_heading in zip(
            base, mirrored, base_heads, mirrored_heads):
        worst_mirror = max(
            worst_mirror, abs(mx - x), abs(my + y),
            abs(mirror_heading + heading))
    return {
        "rotation_translation_worst_m_rad": worst_rotation,
        "mirror_worst_m_rad": worst_mirror,
    }


def path_metrics():
    length = 6.0
    profile, kappas, etas = _linear_curvature_profile(length)
    start = FormationPose(2.5, -1.25, 0.4)
    path = VirtualAPath(start, profile, MODEL)
    request = make_request(
        start, path.endpoint(),
        start_headings={1: start.psi, 2: start.psi, 3: start.psi})
    config = PlannerConfig()
    packet = optimizer_module._search_packet(request, config)
    n = packet["intervals"]
    vector = np.concatenate(([length], kappas, etas))
    states = optimizer_module._collocation_states(
        vector, packet, GAUSS_POINTS)
    values = optimizer_module._numeric_constraint_vector(
        vector, request, config, packet, GAUSS_POINTS)
    equal_count = (n - 1) + 3
    cosine, sine = math.cos(start.psi), math.sin(start.psi)
    worst = 0.0
    nodes = optimizer_module._gauss(GAUSS_POINTS)[0]
    for j in range(n):
        for q, (x, y, psi, kappa) in zip(
                nodes, states[j * GAUSS_POINTS:(j + 1) * GAUSS_POINTS]):
            station = j * (length / n) + q * (length / n)
            reference = path.state_at_s(station)
            world_x = start.x + cosine * x - sine * y
            world_y = start.y + sine * x + cosine * y
            worst = max(worst, math.hypot(
                world_x - reference.qf.x, world_y - reference.qf.y))
    curvature_limit = packet["cmax"] * (1.0 - config.curvature_margin)
    return {
        "length_m": length,
        "intervals": n,
        "collocation_points": GAUSS_POINTS,
        "sample_count": len(states),
        "worst_non_grid_error_vs_path_m": worst,
        "path_position_error_bound_m": float(path.position_error_bound_m),
        "worst_plus_certificate_m": worst + float(path.position_error_bound_m),
        "endpoint_equality_worst": float(
            np.max(np.abs(values[:equal_count]))),
        "wheel_constraint_worst": float(
            np.max(np.abs(values[equal_count:]))),
        "wheel_constraint_limit": curvature_limit,
    }


def fg_metrics():
    request = make_request(
        FormationPose(0.0, 0.0, 0.0), FormationPose(5.98, 0.30, 0.2))
    config = PlannerConfig(max_initials=1)
    packet = optimizer_module._search_packet(request, config)
    guess = optimizer_module.make_initial_guesses(request, config)[0]
    z = np.asarray(guess.vector, dtype=float)
    result = {}

    def compare(label, **kwargs):
        objective, constraints = _casadi_nlp_functions(
            request, config, packet, **kwargs)
        numeric_f = optimizer_module._numeric_objective(
            z, request, config, packet, config.collocation_points, **kwargs)
        casadi_f = float(objective(z))
        numeric_g = np.sort(optimizer_module._numeric_constraint_vector(
            z, request, config, packet, config.collocation_points))
        casadi_g = np.sort(np.asarray(constraints(z)).reshape(-1))
        result[label] = {
            "objective_numpy": numeric_f,
            "objective_casadi": casadi_f,
            "objective_relative_diff": abs(numeric_f - casadi_f) / max(
                1.0, abs(numeric_f)),
            "constraint_count": int(numeric_g.size),
            "constraint_max_abs_diff": float(
                np.max(np.abs(numeric_g - casadi_g))),
        }

    compare("base")
    nodes = optimizer_module._gauss(config.collocation_points)[0]
    route_points = np.column_stack((
        np.linspace(0.0, 3.0, config.intervals + 1),
        np.linspace(0.0, 1.2, config.intervals + 1)))
    route_kappas = np.full(config.intervals + 1, 0.03)
    route_collocation = np.column_stack((
        np.linspace(0.0, 3.0, config.intervals * len(nodes)),
        np.linspace(0.0, 1.2, config.intervals * len(nodes))))
    compare(
        "route",
        route_points=route_points,
        route_weight=5e5,
        route_kappas=route_kappas,
        route_curvature_weight=100.0,
        route_collocation_points=route_collocation,
        route_collocation_weight=5e5)
    field = CachedClearanceField(
        origin_x=0.0, origin_y=0.0, resolution_m=1.0,
        grid_sha256="0" * 64,
        values=tuple(tuple(
            2.0 + 0.1 * i + 0.05 * j for i in range(5))
            for j in range(5)))
    states = optimizer_module._collocation_states(
        z, packet, config.collocation_points)
    proxy = optimizer_module._ClearanceProxy(
        field=field,
        angles=optimizer_module._clearance_proxy_angles(
            field, request, states),
        target_m=5.0, weight=1.0)
    compare("clearance", clearance_proxy=proxy)
    return result


def route_penalty_metrics():
    request, config, packet = _packet(FormationPose(4.0, 0.0, 0.0))
    n = packet["intervals"]
    vector = np.zeros(2 * n + 3, dtype=float)
    vector[0] = 4.0
    nodes = optimizer_module._gauss(GAUSS_POINTS)[0]
    route_points = np.column_stack((
        np.linspace(0.0, 4.0, n + 1), np.zeros(n + 1)))
    route_collocation = np.column_stack((
        np.concatenate([
            (float(j) + nodes) / n * 4.0 for j in range(n)]),
        np.zeros(n * len(nodes))))
    kwargs = {
        "route_points": route_points,
        "route_weight": 1.0,
        "route_kappas": np.zeros(n + 1),
        "route_curvature_weight": 1.0,
        "route_collocation_points": route_collocation,
        "route_collocation_weight": 1.0,
    }
    base = optimizer_module._numeric_objective(
        vector, request, config, packet, GAUSS_POINTS)
    guided = optimizer_module._numeric_objective(
        vector, request, config, packet, GAUSS_POINTS, **kwargs)
    legacy = _legacy_cumulative_states(vector, packet, GAUSS_POINTS)
    legacy_penalty = 0.0
    for (x, y, psi, kappa), point in zip(legacy, route_collocation):
        legacy_penalty += (x - float(point[0])) ** 2 + (
            y - float(point[1])) ** 2
    count = len(route_collocation)
    return {
        "base_objective": base,
        "guided_objective": guided,
        "guided_minus_base": guided - base,
        "collocation_sample_count": count,
        "legacy_route_collocation_penalty_weight1_sum": legacy_penalty,
        "legacy_route_collocation_penalty_weight1_mean": (
            legacy_penalty / count),
        "legacy_route_collocation_penalty_weight_5e5_mean": (
            5e5 * legacy_penalty / count),
    }


def main():
    output = {
        "schema": "virtual_a_corridor.c0a.collocation_metrics.v1",
        "gauss_points": GAUSS_POINTS,
        "thresholds": {
            "straight_all_gauss_m_rad": 1e-12,
            "legacy_reproduction_min_m": 1e-2,
            "symmetry_m_rad": 1e-9,
            "path_non_grid_plus_certificate_m": 1e-7,
            "objective_relative": 1e-9,
            "constraint_absolute": 1e-9,
        },
        "straight": straight_metrics(),
        "route_collocation": route_penalty_metrics(),
        "symmetry": symmetry_metrics(),
        "path": path_metrics(),
        "numpy_vs_casadi": fg_metrics(),
    }
    destination = Path(__file__).resolve().parent / "collocation_metrics.json"
    destination.write_text(
        json.dumps(output, indent=1, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps(output, indent=1, sort_keys=True))


if __name__ == "__main__":
    main()