"""C0 collocation-position regression tests.

The G4 collocation samples must be prefix integrals inside each C2 curvature
piece.  They are not the cumulative full-weight Gauss sum of the whole piece.
The tests compare the corrected helper with the independent ``VirtualAPath``
reference, and the numpy objective/constraints with the CasADi NLP at the
same ``z``.
"""
import math
import unittest
from unittest import mock

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
from virtual_a_g4_support import MODEL, make_request

try:
    import casadi
except ImportError:  # reported as NOT_RUN in evidence, never silently faked
    casadi = None

GAUSS_POINTS = 9


def _packet(goal):
    request = make_request(FormationPose(0.0, 0.0, 0.0), goal)
    config = PlannerConfig()
    return request, config, optimizer_module._search_packet(request, config)


def _linear_curvature_profile(length=6.0, slope=0.05, intervals=8):
    h = float(length) / intervals
    kappas = [slope * j * h for j in range(intervals + 1)]
    etas = [slope] * (intervals + 1)
    profile = C2CurvatureProfile.from_knots([
        ControlKnot(j * h, kappas[j], etas[j])
        for j in range(intervals + 1)])
    return profile, kappas, etas


def _casadi_nlp_functions(request, config, packet, **kwargs):
    captured = {}

    def fake_nlpsol(name, plugin, nlp, options=None):
        captured["nlp"] = nlp
        return object()

    with mock.patch.object(casadi, "nlpsol", side_effect=fake_nlpsol):
        optimizer_module._build_casadi_nlp(request, config, packet, **kwargs)
    nlp = captured["nlp"]
    objective = casadi.Function(
        "c0_objective", [nlp["x"]], [nlp["f"]])
    constraints = casadi.Function(
        "c0_constraints", [nlp["x"]], [nlp["g"]])
    return objective, constraints


def _legacy_cumulative_states(vector, packet, collocation):
    """The pre-C0 cumulative-position code, retained only to prove the bug."""
    n = int(packet["intervals"])
    z = np.asarray(vector, dtype=float).reshape(-1)
    S = float(z[0])
    kappas = np.asarray(z[1:1 + n + 1], dtype=float)
    etas = np.asarray(z[1 + n + 1:], dtype=float)
    h = S / n
    nodes, weights = optimizer_module._gauss(int(collocation))
    x = 0.0
    y = 0.0
    psi = 0.0
    states = []
    for j in range(n):
        c0, c1, c2, c3 = optimizer_module._coeffs(
            h, kappas[j], kappas[j + 1], etas[j], etas[j + 1])
        for q, weight in zip(nodes, weights):
            local_psi = psi + h * q * (
                c0 + q * (c1 / 2.0 + q * (c2 / 3.0 + q * c3 / 4.0)))
            x += h * weight * math.cos(local_psi)
            y += h * weight * math.sin(local_psi)
            kappa = c0 + q * (c1 + q * (c2 + q * c3))
            states.append((x, y, local_psi, kappa))
        psi += h * (c0 + c1 / 2.0 + c2 / 3.0 + c3 / 4.0)
    return states


class CollocationPositionTests(unittest.TestCase):

    def test_straight_line_prefix_positions_are_exact(self):
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
        self.assertLessEqual(worst, 1e-12)

    def test_straight_route_collocation_penalty_is_zero(self):
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
        self.assertLessEqual(abs(guided - base), 1e-9)
        if casadi is None:
            return
        base_function, _ = _casadi_nlp_functions(request, config, packet)
        guided_function, _ = _casadi_nlp_functions(
            request, config, packet, **kwargs)
        self.assertLessEqual(
            abs(float(guided_function(vector)) - float(base_function(vector))),
            1e-9)

    def test_legacy_cumulative_positions_reproduce_the_old_error(self):
        request, config, packet = _packet(FormationPose(4.0, 0.0, 0.0))
        n = packet["intervals"]
        vector = np.zeros(2 * n + 3, dtype=float)
        vector[0] = 4.0
        corrected = optimizer_module._collocation_states(
            vector, packet, GAUSS_POINTS)
        legacy = _legacy_cumulative_states(vector, packet, GAUSS_POINTS)
        nodes = optimizer_module._gauss(GAUSS_POINTS)[0]
        h = 4.0 / n
        legacy_error = max(
            abs(legacy[j][0] - h * nodes[j % GAUSS_POINTS])
            for j in range(GAUSS_POINTS))
        corrected_error = max(
            abs(corrected[j][0] - h * nodes[j % GAUSS_POINTS])
            for j in range(GAUSS_POINTS))
        self.assertGreater(legacy_error, 1e-2)
        self.assertLessEqual(corrected_error, 1e-12)

    def test_piece_samples_are_rotation_and_mirror_equivariant(self):
        h = 0.75
        coeffs = optimizer_module._coeffs(h, 0.11, 0.27, -0.4, 0.35)
        nodes = optimizer_module._gauss(GAUSS_POINTS)[0]
        psi0 = 0.3
        angle = 0.7
        base, base_heads = optimizer_module._piece_position_samples(
            h, coeffs, psi0, nodes, math.sin, math.cos)
        rotated, rotated_heads = optimizer_module._piece_position_samples(
            h, coeffs, psi0 + angle, nodes, math.sin, math.cos)
        mirrored_coeffs = tuple(-value for value in coeffs)
        mirrored, mirrored_heads = optimizer_module._piece_position_samples(
            h, mirrored_coeffs, -psi0, nodes, math.sin, math.cos)
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
        self.assertLessEqual(worst_rotation, 1e-9)
        self.assertLessEqual(worst_mirror, 1e-9)

    def test_legal_c2_curve_matches_the_independent_path(self):
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
        self.assertEqual(len(states), n * GAUSS_POINTS)
        self.assertGreaterEqual(len(states), 20)
        # The endpoint and the wheel constraints make "legal C2" concrete.
        values = optimizer_module._numeric_constraint_vector(
            vector, request, config, packet, GAUSS_POINTS)
        equal_count = (n - 1) + 3
        self.assertLessEqual(
            float(np.max(np.abs(values[:equal_count]))), 1e-9)
        curvature_limit = packet["cmax"] * (1.0 - config.curvature_margin)
        self.assertLessEqual(
            float(np.max(np.abs(values[equal_count:]))), curvature_limit)
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
                error = math.hypot(
                    world_x - reference.qf.x, world_y - reference.qf.y)
                worst = max(worst, error)
        self.assertLessEqual(
            worst + path.position_error_bound_m, 1e-7,
            "worst={!r} bound={!r}".format(
                worst, path.position_error_bound_m))

    @unittest.skipIf(casadi is None, "CasADi is not installed")
    def test_numpy_and_casadi_objective_and_constraints_match(self):
        request = make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(5.98, 0.30, 0.2))
        config = PlannerConfig(max_initials=1)
        packet = optimizer_module._search_packet(request, config)
        guess = optimizer_module.make_initial_guesses(request, config)[0]
        z = np.asarray(guess.vector, dtype=float)
        objective, constraints = _casadi_nlp_functions(
            request, config, packet)
        numeric_f = optimizer_module._numeric_objective(
            z, request, config, packet, config.collocation_points)
        casadi_f = float(objective(z))
        self.assertLessEqual(
            abs(numeric_f - casadi_f) / max(1.0, abs(numeric_f)), 1e-9)
        numeric_g = np.sort(optimizer_module._numeric_constraint_vector(
            z, request, config, packet, config.collocation_points))
        casadi_g = np.sort(np.asarray(constraints(z)).reshape(-1))
        self.assertEqual(numeric_g.size, casadi_g.size)
        self.assertLessEqual(
            float(np.max(np.abs(numeric_g - casadi_g))), 1e-9)

        # Route-position, route-collocation and route-curvature terms.
        nodes = optimizer_module._gauss(config.collocation_points)[0]
        route_points = np.column_stack((
            np.linspace(0.0, 3.0, config.intervals + 1),
            np.linspace(0.0, 1.2, config.intervals + 1)))
        route_kappas = np.full(config.intervals + 1, 0.03)
        route_collocation = np.column_stack((
            np.linspace(0.0, 3.0, config.intervals * len(nodes)),
            np.linspace(0.0, 1.2, config.intervals * len(nodes))))
        kwargs = {
            "route_points": route_points,
            "route_weight": 5e5,
            "route_kappas": route_kappas,
            "route_curvature_weight": 100.0,
            "route_collocation_points": route_collocation,
            "route_collocation_weight": 5e5,
        }
        objective, _ = _casadi_nlp_functions(
            request, config, packet, **kwargs)
        numeric_f = optimizer_module._numeric_objective(
            z, request, config, packet, config.collocation_points, **kwargs)
        casadi_f = float(objective(z))
        self.assertLessEqual(
            abs(numeric_f - casadi_f) / max(1.0, abs(numeric_f)), 1e-9)

        # Clearance proxy, including centres outside the field (clamped).
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
        outside = False
        for x, y, psi, kappa in states:
            for offset in request.model.offsets:
                a, b = offset.x, offset.y
                center_x = x + math.cos(psi) * a - math.sin(psi) * b
                center_y = y + math.sin(psi) * a + math.cos(psi) * b
                if not (0.0 <= center_x <= 4.0
                        and 0.0 <= center_y <= 4.0):
                    outside = True
        self.assertTrue(outside)
        objective, _ = _casadi_nlp_functions(
            request, config, packet, clearance_proxy=proxy)
        numeric_f = optimizer_module._numeric_objective(
            z, request, config, packet, config.collocation_points,
            clearance_proxy=proxy)
        casadi_f = float(objective(z))
        self.assertLessEqual(
            abs(numeric_f - casadi_f) / max(1.0, abs(numeric_f)), 1e-9)


if __name__ == "__main__":
    unittest.main()