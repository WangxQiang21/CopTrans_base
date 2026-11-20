"""C2 tests for corridor guidance: QP/guide/allocation and optimizer wiring.

The hand-built corridors here are exact axis-aligned or rotated boxes so every
expected halfspace, portal centre and normalized cut can be checked directly.
The solver tests use the frozen G2 straight-line request and a single box that
contains it; they exercise the same hard halfspaces, numeric residual and
independent continuous audit that production uses.  This is reference
geometry only: no test claims a three-vehicle executable path.
"""
import inspect
import json
import math
import unittest
import xml.etree.ElementTree as ElementTree
from dataclasses import replace
from pathlib import Path
from unittest import mock

import numpy as np

try:
    import casadi
except ImportError:  # reported as NOT_RUN in evidence, never silently faked
    casadi = None

from fwis_v3_optimizer.continuous_geometry import ControlKnot
from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    CachedClearanceField,
    ConvexCorridor,
    CorridorChain,
    GuideRoute,
    HeadingBranch,
    PlannerConfig,
    VirtualAContractError,
    planner_config_from_dict,
    research_planner_config,
    validate_planner_config,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (
    _edge_constraints,
    _guide_qp_terms,
    allocate_corridors,
    audit_path_in_corridors,
    make_corridor_guide,
)
from fwis_v3_optimizer.formation_continuous_a.geometry import (
    C2CurvatureProfile,
    VirtualAPath,
)
from fwis_v3_optimizer.formation_continuous_a.optimizer import (
    _ClearanceProxy,
    _build_casadi_nlp,
    _build_corridor_constraint,
    _clearance_proxy_angles,
    _clearance_world_body,
    _constraint_value_offsets,
    _corridor_residual_diagnostic,
    _gauss,
    _guidance_mode,
    _numeric_constraint_vector,
    _numeric_objective,
    _proxy_numeric_penalty,
    _search_packet,
    solve_open_path_guided,
)
from fwis_v3_optimizer.formation_continuous_a.pipeline import (
    _cached_clearance_field,
)
from tests.test_virtual_a_optimizer import (
    model_from_fixture,
    planner_config,
    request_from_fixture,
)
from virtual_a_g4_support import empty_grid

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch"


def _box(x0, y0, x1, y1, seed_start, seed_end):
    return ConvexCorridor(
        seed_start=seed_start,
        seed_end=seed_end,
        A=((-1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0)),
        b=(-x0, x1, -y0, y1),
        vertices=((x0, y0), (x1, y0), (x1, y1), (x0, y1)))


def _segments_chain(seeds, padding=0.6):
    regions = []
    for start, end in zip(seeds, seeds[1:]):
        low_x = min(start[0], end[0]) - padding
        high_x = max(start[0], end[0]) + padding
        low_y = min(start[1], end[1]) - padding
        high_y = max(start[1], end[1]) + padding
        regions.append(_box(
            low_x, low_y, high_x, high_y, start, end))
    return CorridorChain(
        regions=tuple(regions), route_source="fixture",
        grid_sha256="0" * 64)


def _parallel_chain(count, padding=0.6):
    return _segments_chain(
        [(float(index), 0.0) for index in range(count + 1)], padding)


def _row_violation(point, region):
    return max(
        0.0,
        max(row[0] * point[0] + row[1] * point[1] - limit
            for row, limit in zip(region.A, region.b)))


def _assert_guide_safe(test, chain, guide):
    points = guide.points
    test.assertEqual(len(points), len(chain.regions) + 1)
    test.assertEqual(guide.grid_sha256, chain.grid_sha256)
    test.assertIsInstance(guide.quality, dict)
    test.assertLessEqual(
        float(guide.quality["primal_violation_m"]), 1e-8)
    test.assertEqual(
        guide.quality["segment_corridor_indices"],
        list(range(len(chain.regions))))
    for index, region in enumerate(chain.regions):
        first = points[index]
        second = points[index + 1]
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            sample = (
                first[0] + fraction * (second[0] - first[0]),
                first[1] + fraction * (second[1] - first[1]))
            test.assertLessEqual(
                _row_violation(sample, region), 1e-8,
                (index, fraction, sample))
    json.dumps(guide.as_dict(), sort_keys=True)


def _assert_allocation(test, chain, guide, intervals=8):
    allocation = allocate_corridors(chain, guide, intervals)
    test.assertIsInstance(allocation, tuple)
    test.assertGreater(len(allocation), 0)
    test.assertAlmostEqual(allocation[0][0], 0.0)
    test.assertAlmostEqual(allocation[-1][1], 1.0)
    seen = set()
    previous = None
    for left, right, corridor_index in allocation:
        test.assertGreater(right, left)
        test.assertLessEqual(right, 1.0)
        test.assertGreaterEqual(left, 0.0)
        test.assertIsInstance(corridor_index, int)
        seen.add(corridor_index)
        if previous is not None:
            test.assertAlmostEqual(previous[1], left, places=15)
            test.assertGreaterEqual(corridor_index, previous[2])
        previous = (left, right, corridor_index)
    test.assertEqual(seen, set(range(len(chain.regions))))

    lengths = [
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(guide.points, guide.points[1:])]
    total = math.fsum(lengths)
    tau = [0.0]
    running = 0.0
    for length in lengths:
        running += length
        tau.append(running / total)
    tau[-1] = 1.0
    for index in range(1, len(chain.regions)):
        boundary = tau[index]
        matching = [
            item for item in allocation
            if abs(item[1] - boundary) <= 1e-12 or
            abs(item[0] - boundary) <= 1e-12]
        test.assertTrue(matching, (index, boundary, allocation))
        left = [item for item in allocation
                if item[1] <= boundary + 1e-12][-1]
        right = [item for item in allocation
                 if item[0] >= boundary - 1e-12][0]
        test.assertEqual(left[2], index - 1)
        test.assertEqual(right[2], index)
        vertex = guide.points[index]
        test.assertLessEqual(
            _row_violation(vertex, chain.regions[index - 1]), 1e-8)
        test.assertLessEqual(
            _row_violation(vertex, chain.regions[index]), 1e-8)
    for left, right, corridor_index in allocation:
        middle = 0.5 * (left + right)
        test.assertGreaterEqual(middle, tau[corridor_index])
        test.assertLessEqual(middle, tau[corridor_index + 1])
    return allocation


class GuideQpTests(unittest.TestCase):
    def test_single_region_is_endpoint_polyline_and_eight_intervals(self):
        chain = _parallel_chain(1)
        guide = make_corridor_guide(chain, (0.0, 0.0), (1.0, 0.0))
        self.assertEqual(guide.source, "corridor_endpoint_polyline")
        self.assertFalse(guide.quality["qp_attempted"])
        self.assertFalse(guide.quality["qp_used"])
        self.assertEqual(guide.points[0], (0.0, 0.0))
        self.assertEqual(guide.points[-1], (1.0, 0.0))
        _assert_guide_safe(self, chain, guide)
        allocation = _assert_allocation(self, chain, guide)
        self.assertEqual(len(allocation), 8)
        self.assertEqual({item[2] for item in allocation}, {0})

    def test_three_unequal_regions_use_the_slsqp_portal_qp(self):
        seeds = [(0.0, 0.0), (2.0, 0.0), (3.0, 1.0), (6.0, 0.0)]
        chain = _segments_chain(seeds, padding=0.4)
        guide = make_corridor_guide(chain, seeds[0], seeds[-1])
        self.assertEqual(guide.source, "corridor_qp_polyline")
        self.assertTrue(guide.quality["qp_attempted"])
        self.assertTrue(guide.quality["qp_used"])
        self.assertEqual(guide.quality["qp_status"], "SLSQP_SUCCESS")
        self.assertLessEqual(
            guide.quality["primal_violation_m"], 1e-8)
        self.assertGreater(guide.quality["qp_iterations"], 0)
        self.assertGreater(guide.quality["qp_objective"], 0.0)
        terms = guide.quality["qp_terms"]
        self.assertAlmostEqual(
            guide.quality["qp_objective"],
            terms["length"] + terms["smoothness"] + terms["centre"],
            places=12)
        seed_lengths = guide.quality["seed_lengths_m"]
        self.assertGreater(max(seed_lengths) - min(seed_lengths), 1.0)
        self.assertNotEqual(guide.points[1], (2.0, 0.0))
        self.assertNotEqual(guide.points[2], (3.0, 0.5))
        _assert_guide_safe(self, chain, guide)
        _assert_allocation(self, chain, guide, 8)

    def test_qp_analytic_gradient_matches_central_difference(self):
        start = (0.0, 0.0)
        goal = (4.0, 1.0)
        centres = [[1.0, 0.2], [2.0, 0.8], [3.0, 0.1]]
        seed_lengths = [2.0, 1.4, 3.2, 1.0]
        l0 = math.hypot(goal[0] - start[0], goal[1] - start[1])
        vector = [0.7, 0.2, 1.9, 0.7, 3.1, 0.4]
        analytic = _guide_qp_terms(
            vector, start, goal, centres, seed_lengths, l0)["gradient"]
        step = 1e-6
        error = 0.0
        for index in range(len(vector)):
            plus = list(vector)
            minus = list(vector)
            plus[index] += step
            minus[index] -= step
            numeric = (
                _guide_qp_terms(
                    plus, start, goal, centres, seed_lengths, l0)["value"]
                - _guide_qp_terms(
                    minus, start, goal, centres, seed_lengths, l0)["value"]
            ) / (2.0 * step)
            error = max(error, abs(numeric - analytic[index]))
        self.assertLess(error, 1e-7)

    def test_twelve_regions_allocate_all_corridors_against_eight(self):
        chain = _parallel_chain(12)
        guide = make_corridor_guide(chain, (0.0, 0.0), (12.0, 0.0))
        self.assertEqual(guide.source, "corridor_qp_polyline")
        self.assertTrue(guide.quality["qp_used"])
        self.assertEqual(len(guide.points), 13)
        _assert_guide_safe(self, chain, guide)
        allocation = _assert_allocation(self, chain, guide, 8)
        self.assertGreater(len(allocation), 8)
        self.assertEqual(
            [item[2] for item in allocation],
            sorted(item[2] for item in allocation))

    def test_qp_exception_keeps_the_checked_chebyshev_polyline(self):
        chain = _parallel_chain(3)
        with mock.patch(
                "fwis_v3_optimizer.formation_continuous_a.corridor.minimize",
                side_effect=RuntimeError("forced failure")):
            guide = make_corridor_guide(
                chain, (0.0, 0.0), (3.0, 0.0))
        self.assertEqual(guide.source, "corridor_chebyshev_polyline")
        self.assertFalse(guide.quality["qp_used"])
        self.assertIn(
            "RuntimeError: forced failure", guide.quality["fallback_reason"])
        _assert_guide_safe(self, chain, guide)
        _assert_allocation(self, chain, guide)

    def test_endpoint_outside_first_corridor_is_rejected(self):
        chain = _parallel_chain(2)
        with self.assertRaises(VirtualAContractError) as raised:
            make_corridor_guide(chain, (-1.0, 0.0), (2.0, 0.0))
        self.assertEqual(
            raised.exception.reason, "CORRIDOR_GUIDE_OUTSIDE")


class AllocationTests(unittest.TestCase):
    def test_guide_without_quality_keeps_the_old_serialization(self):
        guide = GuideRoute(
            points=((0.0, 0.0), (1.0, 0.0)), source="astar_primary",
            grid_sha256="0" * 64)
        payload = guide.as_dict()
        self.assertEqual(
            set(payload),
            {"source", "grid_sha256", "points", "length_m",
             "is_executable_path"})
        self.assertNotIn("quality", payload)
        same = GuideRoute(
            points=((0.0, 0.0), (1.0, 0.0)), source="astar_primary",
            grid_sha256="0" * 64)
        self.assertEqual(guide, same)
        self.assertEqual(hash(guide), hash(same))

    def test_mismatched_guide_count_is_rejected(self):
        chain = _parallel_chain(3)
        guide = GuideRoute(
            points=((0.0, 0.0), (3.0, 0.0)), source="short",
            grid_sha256=chain.grid_sha256)
        with self.assertRaises(VirtualAContractError) as raised:
            allocate_corridors(chain, guide)
        self.assertEqual(
            raised.exception.reason, "CORRIDOR_ALLOCATION_MISMATCH")

    def test_coincident_tau_and_hermite_cuts_do_not_skip_a_corridor(self):
        chain = _parallel_chain(8)
        guide = GuideRoute(
            points=tuple((float(index), 0.0) for index in range(9)),
            source="coincident_fixture", grid_sha256=chain.grid_sha256)
        allocation = _assert_allocation(self, chain, guide, 8)
        self.assertEqual(len(allocation), 8)
        self.assertEqual(
            [item[2] for item in allocation], list(range(8)))
        for index, item in enumerate(allocation):
            self.assertAlmostEqual(item[0], float(index) / 8.0, places=15)
            self.assertAlmostEqual(item[1], float(index + 1) / 8.0, places=15)

    def test_switch_points_are_at_guide_cumulative_boundaries(self):
        seeds = [(0.0, 0.0), (1.0, 0.0), (3.0, 1.0), (4.0, 1.0)]
        chain = _segments_chain(seeds, padding=0.4)
        guide = make_corridor_guide(chain, seeds[0], seeds[-1])
        allocation = _assert_allocation(self, chain, guide, 8)
        transitions = [
            (previous, current)
            for previous, current in zip(allocation, allocation[1:])
            if previous[2] != current[2]]
        self.assertEqual(len(transitions), 2)
        for previous, current in transitions:
            self.assertAlmostEqual(previous[1], current[0], places=15)


class TopologyTests(unittest.TestCase):
    def test_negative_turn_is_warned_but_the_hairpin_vertex_survives(self):
        seeds = [(0.0, 0.0), (2.0, 0.0), (2.1, 0.3), (1.7, 0.1)]
        chain = _segments_chain(seeds, padding=0.4)
        with mock.patch(
                "fwis_v3_optimizer.formation_continuous_a.corridor.minimize",
                side_effect=RuntimeError("forced fallback")):
            guide = make_corridor_guide(chain, seeds[0], seeds[-1])
        self.assertEqual(guide.source, "corridor_chebyshev_polyline")
        self.assertEqual(len(guide.points), 4)
        self.assertEqual(guide.quality["negative_turn_indices"], [2])
        self.assertEqual(
            guide.quality["warnings"],
            ["NEGATIVE_DOT_PRODUCT_PRESERVED"])
        self.assertEqual(
            guide.quality["rounding"], "PL_ONLY_NO_BEZIER")

    def _u_chain(self):
        return CorridorChain(
            regions=(
                _box(-0.5, -0.5, 2.5, 0.5, (0.0, 0.0), (2.0, 0.0)),
                _box(1.5, -0.5, 2.5, 2.5, (2.0, 0.0), (2.0, 2.0)),
                _box(-0.5, 1.5, 2.5, 2.5, (2.0, 2.0), (0.0, 2.0)),
            ),
            route_source="u_fixture", grid_sha256="0" * 64)

    def test_u_turn_vertices_are_not_deleted_or_shortcut(self):
        chain = self._u_chain()
        guide = make_corridor_guide(chain, (0.0, 0.0), (0.0, 2.0))
        self.assertEqual(len(guide.points), 4)
        self.assertGreater(guide.points[1][0], 0.5)
        self.assertGreater(guide.points[2][0], 0.5)
        self.assertAlmostEqual(guide.points[1][0], guide.points[2][0])
        self.assertLess(guide.points[1][1], guide.points[2][1])
        self.assertAlmostEqual(guide.points[-1][0], 0.0)
        _assert_guide_safe(self, chain, guide)
        allocation = _assert_allocation(self, chain, guide, 8)
        self.assertEqual({item[2] for item in allocation}, {0, 1, 2})
        self.assertTrue(guide.quality["negative_turn_indices"] in ([], [2]))

    def test_sharp_corner_is_not_silently_rounded_or_called_smooth(self):
        chain = self._u_chain()
        guide = make_corridor_guide(chain, (0.0, 0.0), (0.0, 2.0))
        self.assertEqual(
            guide.quality["rounding"], "PL_ONLY_NO_BEZIER")
        self.assertEqual(guide.quality["smoothness_claim"], "NONE")
        self.assertEqual(
            len(guide.points), len(chain.regions) + 1)
        for first, second in zip(guide.points, guide.points[1:]):
            self.assertGreater(
                math.hypot(second[0] - first[0],
                           second[1] - first[1]), 1e-8)


LEGACY_PLANNER_KEYS = {
    "intervals", "max_initials", "max_seconds", "max_iterations", "solver",
    "allow_scipy_fallback", "collocation_points", "curvature_margin",
    "eta_limit", "kappa_limit", "position_tolerance_m",
    "heading_tolerance_rad", "rigidity_tolerance_m", "ipopt_tolerance",
    "scipy_tolerance", "scipy_maxiter",
}

CORRIDOR_PLANNER_KEYS = {
    "guidance_mode", "corridor_enforcement", "corridor_margin_m",
    "corridor_margin_portal_fraction", "corridor_kappa_weight",
    "corridor_guide_weight", "corridor_clearance_weight",
    "corridor_clearance_target_m", "corridor_quadrature_points",
    "corridor_audit_position_margin_m", "corridor_audit_max_depth",
    "corridor_audit_max_intervals", "corridor_audit_min_width_m",
    "corridor_box_padding_m",
}


def _straight_vector(config, length=6.0):
    n = int(config.intervals)
    return np.asarray(
        [length] + [0.0] * (n + 1) + [0.0] * (n + 1), dtype=float)


def _corridor_chain(regions, source="fixture"):
    return CorridorChain(
        regions=tuple(regions), route_source=source,
        grid_sha256="0" * 64)


def _eight_entries(index=0):
    return tuple(
        (float(i) / 8.0, float(i + 1) / 8.0, index) for i in range(8))


class _RecordingClearanceField:
    """Record every world query; optionally return no obstacle direction."""

    def __init__(self, direction=0.0):
        self.direction = direction
        self.queries = []

    def direction_at(self, x, y):
        self.queries.append((float(x), float(y)))
        return 0.5, self.direction, 1.0

    def value_at(self, x, y):
        self.queries.append((float(x), float(y)))
        return 0.5


def _asymmetric_clearance_field():
    values = tuple(
        tuple(0.01 + 0.002 * column + 0.003 * row for column in range(36))
        for row in range(32))
    return CachedClearanceField(
        origin_x=-2.0, origin_y=-8.0, resolution_m=0.5,
        grid_sha256="1" * 64, values=values)


def _proxy_for_field(field, sample_count=72):
    angles = tuple(
        tuple(0.1 * sample + 0.03 * vehicle for vehicle in range(3))
        for sample in range(sample_count))
    return _ClearanceProxy(
        field=field, angles=angles, target_m=0.05, weight=5.0e4)


def _legacy_local_clearance_queries(proxy, request, x, y, psi, kappa):
    """The pre-rework local-frame formula, kept only to prove the bug."""
    queries = []
    for offset in request.model.offsets:
        a, b = offset.x, offset.y
        center_x = x + math.cos(psi) * a - math.sin(psi) * b
        center_y = y + math.sin(psi) * a + math.cos(psi) * b
        theta = psi + math.atan2(a * kappa, 1.0 - b * kappa)
        queries.append((center_x, center_y, theta))
    return tuple(queries)


def _rotated_box(angle, tx, ty, x0, y0, x1, y1):
    cosine, sine = math.cos(angle), math.sin(angle)

    def world(x, y):
        return (tx + cosine * x - sine * y, ty + sine * x + cosine * y)

    vertices = tuple(world(x, y) for x, y in (
        (x0, y0), (x1, y0), (x1, y1), (x0, y1)))
    edges = _edge_constraints(vertices)
    return ConvexCorridor(
        seed_start=world(0.0, 0.0),
        seed_end=world(6.0, 0.0),
        A=tuple(axis for axis, _limit in edges),
        b=tuple(limit for _axis, limit in edges),
        vertices=vertices)


class CorridorOptimizerC2BTests(unittest.TestCase):
    def test_legacy_config_encoding_and_corridor_opt_in(self):
        default = PlannerConfig()
        payload = default.as_dict()
        self.assertEqual(set(payload), LEGACY_PLANNER_KEYS)
        restored = planner_config_from_dict(json.loads(json.dumps(payload)))
        self.assertEqual(restored, default)
        self.assertEqual(restored.as_dict(), payload)

        encoded = PlannerConfig(guidance_mode="corridor").as_dict()
        self.assertEqual(
            set(encoded) - LEGACY_PLANNER_KEYS, CORRIDOR_PLANNER_KEYS)
        self.assertTrue(
            LEGACY_PLANNER_KEYS.issubset(set(encoded)))

        partial = dict(payload)
        partial["guidance_mode"] = "corridor"
        partial_config = planner_config_from_dict(partial)
        self.assertEqual(partial_config.guidance_mode, "corridor")
        self.assertEqual(partial_config.corridor_margin_m, 1e-5)
        self.assertEqual(partial_config.corridor_quadrature_points, 9)
        self.assertIn("corridor_enforcement", partial_config.as_dict())

        unknown = dict(payload)
        unknown["corridor_margins_m"] = 1e-3
        with self.assertRaises(VirtualAContractError) as raised:
            planner_config_from_dict(unknown)
        self.assertEqual(
            raised.exception.reason, "INVALID_PLANNER_CONFIG")
        missing = dict(payload)
        missing.pop("solver")
        with self.assertRaises(VirtualAContractError):
            planner_config_from_dict(missing)

        self.assertEqual(
            validate_planner_config(
                PlannerConfig(guidance_mode="sideways")).reason,
            "INVALID_CONFIG")
        self.assertEqual(
            validate_planner_config(
                PlannerConfig(corridor_quadrature_points=65)).reason,
            "INVALID_CONFIG")

    def test_research_entry_factory_changes_only_the_two_choices(self):
        config = research_planner_config()
        self.assertEqual(config.guidance_mode, "corridor")
        self.assertEqual(config.corridor_enforcement, "hard")
        payload = config.as_dict()
        self.assertEqual(payload["guidance_mode"], "corridor")
        self.assertEqual(payload["corridor_enforcement"], "hard")
        # Historical direct construction keeps its legacy default.
        self.assertEqual(PlannerConfig().guidance_mode, "legacy_route")
        self.assertEqual(
            PlannerConfig().as_dict().get("guidance_mode"), None)

        explicit = research_planner_config(
            guidance_mode=" Corridor ", corridor_enforcement="SOFT")
        self.assertEqual(explicit.guidance_mode, "corridor")
        self.assertEqual(explicit.corridor_enforcement, "soft")
        self.assertEqual(
            planner_config_from_dict(
                json.loads(json.dumps(explicit.as_dict()))), explicit)

        for guidance, enforcement in (("sideways", "hard"),
                                      ("corridor", "loose")):
            with self.assertRaises(VirtualAContractError) as raised:
                research_planner_config(
                    guidance_mode=guidance,
                    corridor_enforcement=enforcement)
            self.assertEqual(raised.exception.reason, "INVALID_CONFIG")

    def test_launch_research_defaults_reach_the_casadi_halfspaces(self):
        root = ElementTree.parse(str(LAUNCH)).getroot()
        args = {arg.get("name"): arg.get("default")
                for arg in root.findall("arg")}
        self.assertEqual(args["backend"], "virtual_rigid_a")
        config = research_planner_config(
            guidance_mode=args["virtual_a_guidance_mode"],
            corridor_enforcement=args["virtual_a_corridor_enforcement"])
        # This is the same normalized config the node logs and hands to the
        # pipeline; the optimizer selects corridor constraints from it.
        self.assertEqual(_guidance_mode(config), "corridor")
        self.assertEqual(config.corridor_enforcement, "hard")

        request = request_from_fixture("STRAIGHT_LINE")
        chain = _corridor_chain((
            _box(-1.0, -1.0, 7.0, 1.0, (0.0, 0.0), (6.0, 0.0)),))
        allocation = _eight_entries()
        packet = _search_packet(request, config)
        constraint = _build_corridor_constraint(
            chain, allocation, request, config)
        solver, _lower_x, _upper_x, lower_g, upper_g = _build_casadi_nlp(
            request, config, packet, corridor_constraint=constraint)
        _equality_end, signed_end, corridor_end = _constraint_value_offsets(
            request, config, packet, constraint)
        self.assertEqual(constraint.halfspace_constraint_count, 352)
        self.assertEqual(corridor_end - signed_end, 352)
        self.assertEqual(int(np.count_nonzero(
            np.isneginf(lower_g[signed_end:corridor_end]))), 352)
        self.assertEqual(int(np.count_nonzero(
            upper_g[signed_end:corridor_end] == 0.0)), 352)
        self.assertEqual(
            np.asarray(solver.get_function("nlp_g")(
                _straight_vector(config), np.zeros(0)),
                dtype=float).ravel()[signed_end:corridor_end].size, 352)

    def test_halfspace_residual_is_identical_in_numeric_and_casadi_nlp(self):
        request = request_from_fixture("STRAIGHT_LINE")
        config = planner_config(
            guidance_mode="corridor", solver="ipopt", max_initials=1)
        chain = _corridor_chain((
            _box(-1.0, -1.0, 7.0, 1.0, (0.0, 0.0), (6.0, 0.0)),))
        allocation = _eight_entries()
        packet = _search_packet(request, config)
        constraint = _build_corridor_constraint(
            chain, allocation, request, config)
        self.assertEqual(constraint.sample_count, 88)
        self.assertEqual(constraint.quadrature_points, 9)
        self.assertEqual(constraint.margin_m, 1e-5)
        self.assertEqual(constraint.halfspace_constraint_count, 352)
        self.assertEqual(
            constraint.as_dict()["halfspace_constraint_count"], 352)
        self.assertFalse(
            constraint.as_dict()["is_three_vehicle_certificate"])

        z = _straight_vector(config)
        residual, error = _corridor_residual_diagnostic(
            z, request, config, packet, constraint)
        self.assertEqual(error, "")
        self.assertLessEqual(residual, 1e-8)

        equality_end, signed_end, corridor_end = _constraint_value_offsets(
            request, config, packet, constraint)
        values = _numeric_constraint_vector(
            z, request, config, packet, int(config.collocation_points),
            corridor_constraint=constraint)
        self.assertEqual(values.shape, (corridor_end,))
        numeric = np.asarray(values[signed_end:corridor_end], dtype=float)

        solver, _lower_x, _upper_x, lower_g, upper_g = _build_casadi_nlp(
            request, config, packet, corridor_constraint=constraint)
        casadi_values = np.asarray(
            solver.get_function("nlp_g")(z, np.zeros(0)),
            dtype=float).ravel()
        self.assertEqual(casadi_values.size, corridor_end)
        np.testing.assert_allclose(
            casadi_values[signed_end:], numeric, atol=1e-12, rtol=0.0)
        self.assertLessEqual(
            float(np.max(casadi_values[signed_end:])), 1e-8)
        self.assertTrue(np.all(np.isneginf(lower_g[signed_end:])))
        self.assertTrue(np.all(upper_g[signed_end:] == 0.0))
        self.assertEqual(
            corridor_end - signed_end,
            constraint.halfspace_constraint_count)
        self.assertEqual(equality_end + 264, signed_end)

        shifted = _corridor_chain((
            _box(-1.0, 0.5, 7.0, 1.5, (0.0, 0.0), (6.0, 0.0)),),
            source="shifted")
        shifted_constraint = _build_corridor_constraint(
            shifted, allocation, request, config)
        shifted_residual, error = _corridor_residual_diagnostic(
            z, request, config, packet, shifted_constraint)
        self.assertEqual(error, "")
        self.assertGreater(shifted_residual, 0.4)

    def test_branch_heading_reaches_numeric_and_casadi_equalities(self):
        request = request_from_fixture("STRAIGHT_LINE")
        config = planner_config(
            guidance_mode="corridor", solver="ipopt", max_initials=1)
        chain = _corridor_chain((
            _box(-1.0, -1.0, 7.0, 1.0, (0.0, 0.0), (6.0, 0.0)),))
        allocation = _eight_entries()
        branch = HeadingBranch(
            base_heading_rad=0.0,
            winding_k=1,
            unwrapped_heading_rad=2.0 * math.pi,
            guide_turn_rad=2.0 * math.pi,
            source="fixture_loop",
            candidate_rank=0)
        packet = _search_packet(
            request, config, heading_branch=branch, guide_length_m=6.0)
        self.assertEqual(packet["heading_change_rad"], 2.0 * math.pi)
        self.assertEqual(packet["heading_branch"], branch.as_dict())
        self.assertEqual(packet["guide_length_m"], 6.0)
        constraint = _build_corridor_constraint(
            chain, allocation, request, config)
        z = _straight_vector(config)
        equality_end, _signed_end, _corridor_end = _constraint_value_offsets(
            request, config, packet, constraint)
        values = _numeric_constraint_vector(
            z, request, config, packet, int(config.collocation_points),
            corridor_constraint=constraint)
        self.assertAlmostEqual(
            float(values[equality_end - 1]), -2.0 * math.pi, places=6)
        solver, _lower_x, _upper_x, _lower_g, _upper_g = _build_casadi_nlp(
            request, config, packet, corridor_constraint=constraint)
        casadi_values = np.asarray(
            solver.get_function("nlp_g")(z, np.zeros(0)),
            dtype=float).ravel()
        np.testing.assert_allclose(
            casadi_values[:equality_end], values[:equality_end],
            atol=1e-12, rtol=0.0)
        self.assertAlmostEqual(
            float(casadi_values[equality_end - 1]), -2.0 * math.pi, places=6)

    def test_corridor_solution_passes_solver_and_independent_audit(self):
        request = request_from_fixture("STRAIGHT_LINE")
        config = planner_config(
            guidance_mode="corridor", solver="ipopt", max_initials=1,
            max_seconds=20.0)
        chain = _corridor_chain((
            _box(-1.0, -1.0, 7.0, 1.0, (0.0, 0.0), (6.0, 0.0)),))
        guide = make_corridor_guide(chain, (0.0, 0.0), (6.0, 0.0))
        allocation = allocate_corridors(
            chain, guide, int(config.intervals))
        result = solve_open_path_guided(
            request, config, route_guides=(guide,),
            corridor_chain=chain, corridor_allocation=allocation)
        self.assertEqual(result.status, "GEOMETRY_FEASIBLE", result.detail)
        self.assertIsNotNone(result.path)
        self.assertTrue(result.audit.passed)
        audited = [
            attempt for attempt in result.attempts
            if attempt.status == "SOLVER_SUCCESS_AUDITED"]
        self.assertEqual(len(audited), 1)
        self.assertIsNotNone(audited[0].corridor_residual_m)
        self.assertLessEqual(audited[0].corridor_residual_m, 1e-8)
        self.assertGreaterEqual(audited[0].corridor_residual_m, 0.0)
        # Result evidence comes from the actual constraint rows, not from the
        # display polygon: 88 samples x 4 box halfspaces.
        self.assertEqual(result.corridor_halfspace_constraint_count, 352)
        self.assertEqual(
            result.as_dict()["corridor_halfspace_constraint_count"], 352)
        self.assertEqual(
            result.corridor_max_residual_m, audited[0].corridor_residual_m)
        self.assertLessEqual(result.corridor_max_residual_m, 1e-8)
        start = result.path.state_at_s(0.0).qf
        end = result.path.state_at_s(result.path.length_m).qf
        self.assertAlmostEqual(start.x, 0.0, places=9)
        self.assertAlmostEqual(start.y, 0.0, places=9)
        self.assertAlmostEqual(end.x, 6.0, places=6)
        self.assertAlmostEqual(end.y, 0.0, places=6)
        self.assertAlmostEqual(end.psi, 0.0, places=6)

        containment = audit_path_in_corridors(
            result.path, chain, allocation,
            position_budget_m=config.corridor_audit_position_margin_m,
            max_depth=int(config.corridor_audit_max_depth),
            max_intervals=int(config.corridor_audit_max_intervals),
            min_width_m=float(config.corridor_audit_min_width_m))
        self.assertTrue(containment["passed"])
        self.assertEqual(
            containment["reason"], "CORRIDOR_CONTAINMENT_VERIFIED")
        self.assertLess(containment["max_point_residual_m"], 0.0)
        self.assertFalse(containment["is_three_vehicle_certificate"])

    def test_world_rotation_and_translation_reach_every_halfspace(self):
        angle, tx, ty = 0.7, 3.5, -2.0
        cosine, sine = math.cos(angle), math.sin(angle)

        def world(x, y):
            return (tx + cosine * x - sine * y, ty + sine * x + cosine * y)

        base = request_from_fixture("STRAIGHT_LINE")
        request = request_from_fixture(
            "STRAIGHT_LINE",
            start_qf=FormationPose(*world(0.0, 0.0), angle),
            start_headings={
                key: value + angle
                for key, value in base.start_headings.items()},
            goal_qf=FormationPose(*world(6.0, 0.0), angle))
        config = planner_config(
            guidance_mode="corridor", solver="ipopt", max_initials=1)
        packet = _search_packet(request, config)
        allocation = _eight_entries()
        z = _straight_vector(config)

        chain = _corridor_chain((
            _rotated_box(angle, tx, ty, -1.0, -1.0, 7.0, 1.0),),
            source="rotated")
        residual, error = _corridor_residual_diagnostic(
            z, request, config, packet,
            _build_corridor_constraint(chain, allocation, request, config))
        self.assertEqual(error, "")
        self.assertLessEqual(residual, 1e-9)

        axis_chain = _corridor_chain((
            _box(-1.0, -0.05, 7.0, 0.05,
                 world(0.0, 0.0), world(6.0, 0.0)),),
            source="axis_aligned")
        axis_residual, error = _corridor_residual_diagnostic(
            z, request, config, packet,
            _build_corridor_constraint(
                axis_chain, allocation, request, config))
        self.assertEqual(error, "")
        self.assertGreater(axis_residual, 1.0)

    def test_allocation_switch_uses_the_assigned_region(self):
        request = request_from_fixture("STRAIGHT_LINE")
        config = planner_config(
            guidance_mode="corridor", solver="ipopt", max_initials=1)
        packet = _search_packet(request, config)
        chain = _corridor_chain((
            _box(-1.0, -1.0, 3.5, 1.0, (0.0, 0.0), (3.0, 0.0)),
            _box(2.5, -1.0, 7.0, 1.0, (3.0, 0.0), (6.0, 0.0))),
            source="switch")
        allocation = ((0.0, 0.5, 0), (0.5, 1.0, 1))
        constraint = _build_corridor_constraint(
            chain, allocation, request, config)
        self.assertEqual(constraint.sample_count, 22)
        self.assertEqual(constraint.region_count, 2)
        self.assertEqual(constraint.allocation, allocation)
        self.assertEqual(
            sum(len(group) for group in constraint.groups), 22)
        _equality_end, signed_end, corridor_end = _constraint_value_offsets(
            request, config, packet, constraint)
        self.assertEqual(corridor_end - signed_end, 88)
        switch_group = constraint.groups[4]
        self.assertGreaterEqual(len(switch_group), 2)
        limits = {
            round(float(limit), 9)
            for _q, _rows, group_limits in switch_group
            for limit in group_limits}
        self.assertIn(round(3.5 - 1e-5, 9), limits)
        self.assertIn(round(7.0 - 1e-5, 9), limits)
        residual, error = _corridor_residual_diagnostic(
            _straight_vector(config), request, config, packet, constraint)
        self.assertEqual(error, "")
        self.assertLessEqual(residual, 1e-8)

    def test_continuous_audit_rejects_an_exit_between_corridor_samples(self):
        model = model_from_fixture()
        profile = C2CurvatureProfile.from_knots((
            ControlKnot(0.0, 0.5, 0.0), ControlKnot(8.0, 0.5, 0.0)))
        path = VirtualAPath(
            FormationPose(0.0, 0.0, 0.0), profile, model)
        length = path.length_m
        dense = [
            path.state_at_s(length * index / 8000.0).qf
            for index in range(8001)]
        xs = [state.x for state in dense]
        ys = [state.y for state in dense]
        allocation = _eight_entries()
        low_x, high_x = min(xs) - 0.2, max(xs) + 0.2
        low_y, high_y = min(ys) - 0.2, max(ys) + 0.2

        accepted = audit_path_in_corridors(
            path,
            _corridor_chain((
                _box(low_x, low_y, high_x, high_y, (0.0, 0.0), (6.0, 0.0)),),
                source="accepted"),
            allocation,
            position_budget_m=5e-8)
        self.assertTrue(accepted["passed"], accepted["first_failure"])
        self.assertEqual(accepted["reason"], "CORRIDOR_CONTAINMENT_VERIFIED")
        self.assertLess(accepted["max_point_residual_m"], 0.0)
        self.assertGreater(accepted["epsilon_total_m"], 0.0)
        self.assertFalse(accepted["is_three_vehicle_certificate"])

        nodes, _weights = _gauss(9)
        sigmas = (0.0,) + tuple(float(node) for node in nodes) + (1.0,)
        samples = [path.state_at_s(sigma * length).qf for sigma in sigmas]
        boundary = max(state.y for state in samples)
        self.assertLessEqual(
            max(state.y - boundary for state in samples), 0.0)
        rejected = audit_path_in_corridors(
            path,
            _corridor_chain((
                _box(low_x, low_y, high_x, boundary + 1e-9,
                     (0.0, 0.0), (6.0, 0.0)),),
                source="between_samples"),
            allocation,
            position_budget_m=5e-8)
        self.assertFalse(rejected["passed"])
        self.assertIn(
            rejected["reason"],
            ("CORRIDOR_CONTAINMENT_VIOLATION",
             "CORRIDOR_AUDIT_WIDTH_LIMIT"))
        self.assertIsNotNone(rejected["first_failure"])
        self.assertGreater(rejected["max_point_residual_m"], 0.0)
        self.assertLess(
            rejected["first_failure"]["width_m"], 1e-5 + 1e-12)

    def test_corridor_inputs_are_rejected_outside_corridor_mode(self):
        request = request_from_fixture("STRAIGHT_LINE")
        chain = _corridor_chain((
            _box(-1.0, -1.0, 7.0, 1.0, (0.0, 0.0), (6.0, 0.0)),))
        allocation = ((0.0, 1.0, 0),)
        guide = GuideRoute(
            points=((0.0, 0.0), (6.0, 0.0)), source="fixture",
            grid_sha256="0" * 64)

        legacy = planner_config(max_initials=1)
        result = solve_open_path_guided(
            request, legacy, corridor_chain=chain,
            corridor_allocation=allocation)
        self.assertEqual(result.status, "INVALID_REQUEST")
        self.assertEqual(result.reason, "CORRIDOR_INPUTS_IN_LEGACY_MODE")

        corridor = planner_config(guidance_mode="corridor", max_initials=1)
        result = solve_open_path_guided(request, corridor)
        self.assertEqual(result.status, "INVALID_REQUEST")
        self.assertEqual(result.reason, "CORRIDOR_INPUTS_REQUIRED")

        result = solve_open_path_guided(
            request, corridor, route_guides=(guide, guide),
            corridor_chain=chain, corridor_allocation=allocation)
        self.assertEqual(result.status, "INVALID_REQUEST")
        self.assertEqual(result.reason, "CORRIDOR_GUIDE_AMBIGUOUS")

    def test_clearance_body_centres_use_world_frame(self):
        angle, tx, ty = 0.7, 3.0, -2.0
        base = request_from_fixture("STRAIGHT_LINE")
        request = request_from_fixture(
            "STRAIGHT_LINE",
            start_qf=FormationPose(tx, ty, angle),
            start_headings={
                key: value + angle
                for key, value in base.start_headings.items()},
            goal_qf=FormationPose(
                tx + 8.0 * math.cos(angle),
                ty + 8.0 * math.sin(angle), angle))
        field = _RecordingClearanceField()
        angles = _clearance_proxy_angles(
            field, request, ((0.0, 0.0, 0.0, 0.0),))
        cosine, sine = math.cos(angle), math.sin(angle)
        expected = (
            (tx + cosine, ty + sine),
            (tx - sine, ty + cosine),
            (tx + sine, ty - cosine))
        self.assertEqual(len(field.queries), 3)
        np.testing.assert_allclose(
            np.asarray(field.queries), np.asarray(expected),
            atol=1e-12, rtol=0.0)
        self.assertEqual(angles, ((0.0, 0.0, 0.0),))

        missing = _RecordingClearanceField(direction=None)
        fallback = _clearance_proxy_angles(
            missing, request, ((0.0, 0.0, 0.0, 0.2),))
        expected_theta = (
            angle + math.atan2(0.2, 1.0),
            angle,
            angle)
        np.testing.assert_allclose(
            np.asarray(fallback[0]), np.asarray(expected_theta),
            atol=1e-12, rtol=0.0)

    def test_numeric_and_casadi_clearance_objectives_agree_off_origin(self):
        if casadi is None:
            self.skipTest("casadi unavailable")
        angle, tx, ty = 0.7, 3.0, -2.0
        base = request_from_fixture("STRAIGHT_LINE")
        goal_x = tx + math.cos(angle) * 8.0 - math.sin(angle) * 2.0
        goal_y = ty + math.sin(angle) * 8.0 + math.cos(angle) * 2.0
        request = request_from_fixture(
            "STRAIGHT_LINE",
            start_qf=FormationPose(tx, ty, angle),
            start_headings={
                key: value + angle
                for key, value in base.start_headings.items()},
            goal_qf=FormationPose(goal_x, goal_y, angle))
        config = planner_config(
            guidance_mode="corridor", solver="scipy", max_initials=1)
        packet = _search_packet(request, config)
        n = int(config.intervals)
        collocation = int(config.collocation_points)
        proxy = _proxy_for_field(
            _asymmetric_clearance_field(), sample_count=n * collocation)
        z = np.zeros(2 * n + 3, dtype=float)
        z[0] = 8.0
        z[1:1 + n + 1] = 0.04 * np.sin(0.3 * np.arange(n + 1))
        z[1 + n + 1:] = 0.03 * np.cos(0.2 * np.arange(n + 1))

        numeric = _numeric_objective(
            z, request, config, packet, collocation,
            clearance_proxy=proxy)
        captured = {}

        def fake_nlpsol(name, plugin, nlp, options=None):
            captured["nlp"] = nlp
            return object()

        with mock.patch.object(casadi, "nlpsol", side_effect=fake_nlpsol):
            _build_casadi_nlp(
                request, config, packet, clearance_proxy=proxy)
        objective = casadi.Function(
            "rework_clearance_objective",
            [captured["nlp"]["x"]], [captured["nlp"]["f"]])
        casadi_value = float(objective(z))
        tolerance = 1e-9 * max(1.0, abs(numeric))
        self.assertLessEqual(abs(casadi_value - numeric), tolerance)

        without_proxy = _numeric_objective(
            z, request, config, packet, collocation)
        self.assertGreater(abs(numeric - without_proxy), 1e-6)

    def test_local_clearance_query_is_detectably_wrong(self):
        angle, tx, ty = 0.7, 3.0, -2.0
        base = request_from_fixture("STRAIGHT_LINE")
        request = request_from_fixture(
            "STRAIGHT_LINE",
            start_qf=FormationPose(tx, ty, angle),
            start_headings={
                key: value + angle
                for key, value in base.start_headings.items()},
            goal_qf=FormationPose(
                tx + 8.0 * math.cos(angle),
                ty + 8.0 * math.sin(angle), angle))
        proxy = _proxy_for_field(_asymmetric_clearance_field())
        x, y, psi, kappa, sample = 4.0, 1.0, 0.6, 0.15, 0
        correct = _proxy_numeric_penalty(
            proxy, request, x, y, psi, kappa, sample)
        legacy_queries = _legacy_local_clearance_queries(
            proxy, request, x, y, psi, kappa)
        row = proxy.angles[sample]
        wrong = 0.0
        for index, offset in enumerate(request.model.offsets):
            shape = request.vehicle_shapes[offset.vehicle_id]
            center_x, center_y, theta = legacy_queries[index]
            clearance = proxy.field.value_at(center_x, center_y)
            support = (
                0.5 * shape.length_m * abs(math.cos(theta - row[index]))
                + 0.5 * shape.width_m * abs(math.sin(theta - row[index])))
            hinge = proxy.target_m + support - clearance
            if hinge > 0.0:
                wrong += hinge * hinge
        self.assertGreater(abs(correct - wrong), 1e-3)

        world = [
            _clearance_world_body(request, x, y, psi, kappa, offset)
            for offset in request.model.offsets]
        separation = max(
            math.hypot(legacy[0] - correct_body[0],
                       legacy[1] - correct_body[1])
            for legacy, correct_body in zip(legacy_queries, world))
        self.assertGreater(separation, 1.0)

    def test_origin_start_clearance_semantics_are_unchanged(self):
        request = request_from_fixture("STRAIGHT_LINE")
        proxy = _proxy_for_field(_asymmetric_clearance_field())
        x, y, psi, kappa, sample = 1.5, -0.7, 0.35, 0.12, 3
        legacy_queries = _legacy_local_clearance_queries(
            proxy, request, x, y, psi, kappa)
        for offset, expected in zip(request.model.offsets, legacy_queries):
            np.testing.assert_allclose(
                _clearance_world_body(
                    request, x, y, psi, kappa, offset),
                expected, atol=1e-12, rtol=0.0)
        row = proxy.angles[sample]
        expected_penalty = 0.0
        for index, offset in enumerate(request.model.offsets):
            shape = request.vehicle_shapes[offset.vehicle_id]
            center_x, center_y, theta = legacy_queries[index]
            clearance = proxy.field.value_at(center_x, center_y)
            support = (
                0.5 * shape.length_m * abs(math.cos(theta - row[index]))
                + 0.5 * shape.width_m * abs(math.sin(theta - row[index])))
            hinge = proxy.target_m + support - clearance
            if hinge > 0.0:
                expected_penalty += hinge * hinge
        self.assertAlmostEqual(
            _proxy_numeric_penalty(proxy, request, x, y, psi, kappa, sample),
            expected_penalty, places=12)


class SoftClearanceAblationTests(unittest.TestCase):
    """Step 5: the EDT proxy is a preference, never a certificate input."""

    def test_zero_weight_is_the_exact_no_proxy_objective(self):
        request = request_from_fixture("STRAIGHT_LINE")
        config = planner_config(
            guidance_mode="corridor", solver="ipopt", max_initials=1)
        packet = _search_packet(request, config)
        n = int(config.intervals)
        collocation = int(config.collocation_points)
        proxy = _proxy_for_field(
            _asymmetric_clearance_field(), sample_count=n * collocation)
        zero = _ClearanceProxy(
            field=proxy.field, angles=proxy.angles,
            target_m=proxy.target_m, weight=0.0)
        z = _straight_vector(config)
        without = _numeric_objective(
            z, request, config, packet, collocation)
        exact = _numeric_objective(
            z, request, config, packet, collocation,
            clearance_proxy=zero)
        active = _numeric_objective(
            z, request, config, packet, collocation,
            clearance_proxy=proxy)
        # No term is added at all for weight 0, so this is exact equality,
        # not a tolerance: the soft preference cannot even perturb rounding.
        self.assertEqual(exact, without)
        self.assertGreater(active, without)

        if casadi is None:
            return
        captured = {}

        def build(proxy_value, tag):
            def fake_nlpsol(name, plugin, nlp, options=None):
                captured[tag] = nlp
                return object()
            with mock.patch.object(
                    casadi, "nlpsol", side_effect=fake_nlpsol):
                _build_casadi_nlp(
                    request, config, packet,
                    clearance_proxy=proxy_value)

        build(None, "without")
        build(zero, "zero")
        build(proxy, "active")
        second = z.copy()
        second[1:1 + n + 1] = 0.03 * np.sin(0.4 * np.arange(n + 1))
        second[1 + n + 1:] = 0.02 * np.cos(0.3 * np.arange(n + 1))
        for vector in (z, second):
            values = {}
            for tag, nlp in captured.items():
                function = casadi.Function(
                    "clearance_ablation_" + tag,
                    [nlp["x"]], [nlp["f"], nlp["g"]])
                values[tag] = function(vector)
            self.assertEqual(float(values["zero"][0]),
                             float(values["without"][0]))
            np.testing.assert_array_equal(
                np.asarray(values["zero"][1], dtype=float),
                np.asarray(values["without"][1], dtype=float))
            self.assertGreater(
                float(values["active"][0]),
                float(values["without"][0]))

    def test_clearance_field_has_no_corridor_wall_input(self):
        grid = empty_grid()
        ix0 = int(round((2.0 - grid.ox) / grid.res))
        ix1 = int(round((4.0 - grid.ox) / grid.res))
        iy0 = int(round((1.6 - grid.oy) / grid.res))
        iy1 = int(round((2.2 - grid.oy) / grid.res))
        grid.occ[iy0:iy1, ix0:ix1] = True
        field = _cached_clearance_field(grid)
        # Occupancy is the only wall source: an occupied coarse node is zero
        # up to the bilinear interpolation round-off of the cached lattice.
        self.assertLessEqual(field.value_at(2.025, 1.625), 1e-12)
        # Free-space points on would-be artificial corridor edges keep their
        # real distance to occupancy/map boundary instead of becoming walls.
        for x, y in ((0.0, 1.0), (3.0, 1.0), (3.0, -1.0)):
            self.assertGreater(field.value_at(x, y), 0.05)
        parameters = list(inspect.signature(
            _cached_clearance_field).parameters)
        self.assertEqual(parameters, ["grid", "downsample"])
        self.assertFalse(any("corridor" in name for name in parameters))

    def test_soft_config_never_changes_the_certificate(self):
        from fwis_v3_optimizer.formation_continuous_a.formation_certificate \
            import certify_formation_path
        from tests.test_virtual_a_formation_corridor import (
            CONTEXT_SHA as FORMATION_CONTEXT_SHA,
            _scene,
            _slanted_wall_scene,
        )

        def certificate(scene, config):
            return certify_formation_path(
                scene["path"], scene["request"], scene["chain"],
                scene["template"],
                allocation=scene["allocation"], guide=scene["guide"],
                planning_context_sha256=FORMATION_CONTEXT_SHA,
                config=config)

        arms = (
            ("zero", dict(
                corridor_clearance_weight=0.0,
                corridor_clearance_target_m=0.0)),
            ("default", dict(
                corridor_clearance_weight=1.0,
                corridor_clearance_target_m=0.05)),
            ("test_high", dict(
                corridor_clearance_weight=5.0,
                corridor_clearance_target_m=0.6)),
        )
        for scene, expected in (
                (_scene(), "VERIFIED"),
                (_slanted_wall_scene(), "VIOLATED")):
            certificates = [
                certificate(scene, replace(scene["config"], **changes))
                for _name, changes in arms]
            self.assertEqual(
                {item["status"] for item in certificates}, {expected})
            # The payload has no soft-weight field, so the canonical hash is
            # identical: soft guidance cannot verify or violate anything.
            self.assertEqual(
                len({item["certificate_sha256"]
                     for item in certificates}), 1)


if __name__ == "__main__":
    unittest.main()