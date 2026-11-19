"""TASK 02 geometry and finite formation-assessment tests.

Step B fixtures are analytic.  The union predicate must inspect the whole
polygon, not the four corners or the centre, and must not build the convex
hull of ``C_left union C_right``.  Every ambiguous, boundary or degenerate
case returns ``UNCONFIRMED_NUMERICS`` rather than passing by tolerance.

Step C fixtures use the real ``FormationCollisionChecker`` on a real grid.
Map, corridor-coverage and kinematic statuses are asserted independently; a
diagnostic assessment never becomes a certificate.
"""
import math
import types
import unittest

import numpy as np

from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import FormationModel, VehicleOffset
from fwis_v3_optimizer.formation_collision import (
    FormationCollisionChecker,
    VehicleShape,
)
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    ConvexCorridor,
    CorridorChain,
    GuideRoute,
    VirtualAContractError,
    grid_signature,
)
from fwis_v3_optimizer.formation_continuous_a import (
    pipeline as pipeline_module,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (
    CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY,
    COVERAGE_ADJACENT_UNION,
    COVERAGE_GAP,
    COVERAGE_SINGLE_REGION,
    COVERAGE_UNCONFIRMED,
    CORRIDOR_GUIDE_CANDIDATE_LIMIT,
    CORRIDOR_GUIDE_PORTAL_VARIANT_LIMIT,
    GUIDE_CANDIDATE_EVIDENCE_SCHEMA,
    GUIDE_HINT_DEGENERATE,
    GUIDE_HINT_VALID,
    KINEMATIC_CURVATURE_UNCONFIRMED,
    KINEMATIC_NORMAL_BRANCH_INVALID,
    KINEMATIC_NORMAL_BRANCH_OK,
    MAP_CLEAR_AT_SAMPLES,
    MAP_COLLISION_OBSERVED,
    MAP_SAMPLE_UNCONFIRMED,
    PORTAL_LOCAL_CANDIDATE_POINT_LIMIT,
    allocate_corridors,
    assess_formation_guide,
    assess_route_guide,
    formation_transition_windows,
    obb_coverage_in_adjacent_union,
    obb_coverage_in_corridor,
    obb_polygon,
    obb_support_halfwidth,
    plan_guide_candidates,
    select_guide_target_portal,
)
from fwis_v3_optimizer.formation_continuous_a.optimizer import (
    _node_route_curvature,
)

from virtual_a_g4_support import MODEL, SHAPES, empty_grid


def _rectangle(x0, y0, x1, y1):
    """Axis-aligned CCW ConvexCorridor with outward unit halfspaces."""
    return ConvexCorridor(
        seed_start=((x0 + x1) / 2.0, y0),
        seed_end=((x0 + x1) / 2.0, y1),
        A=((-1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0)),
        b=(-x0, x1, -y0, y1),
        vertices=((x0, y0), (x1, y0), (x1, y1), (x0, y1)))


def _rect_points(x0, y0, x1, y1):
    return ((x0, y0), (x1, y0), (x1, y1), (x0, y1))


def _signed_twice_area(vertices):
    return sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(
            vertices, vertices[1:] + vertices[:1]))


def _rotate_start(vertices, offset):
    return tuple(vertices[offset:] + vertices[:offset])


class SupportFunctionTests(unittest.TestCase):
    """O1: the support half-width equals the explicit corner projection."""

    def test_support_matches_explicit_corner_projection(self):
        shapes = (
            VehicleShape(0.738, 0.500),
            VehicleShape(4.000, 1.600),
            VehicleShape(0.200, 0.100),
        )
        centres = ((0.0, 0.0), (3.5, -2.25), (-7.75, 4.5))
        headings = (0.0, 0.37, math.pi / 2.0, -2.1, math.pi, 1e-4)
        normals = (
            (1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0),
            (math.cos(0.61), math.sin(0.61)),
            (math.cos(-1.93), math.sin(-1.93)),
            (0.6, -0.8),
        )
        for shape in shapes:
            for center_x, center_y in centres:
                for heading in headings:
                    corners = obb_polygon(
                        (center_x, center_y), heading, shape)
                    for normal_x, normal_y in normals:
                        support = max(
                            normal_x * x + normal_y * y
                            for x, y in corners)
                        halfwidth = obb_support_halfwidth(
                            (normal_x, normal_y), heading, shape)
                        centre_projection = (
                            normal_x * center_x + normal_y * center_y)
                        self.assertLessEqual(
                            abs(support - centre_projection - halfwidth),
                            1e-12,
                            msg="shape={} heading={} normal={}".format(
                                (shape.length_m, shape.width_m), heading,
                                (normal_x, normal_y)))

    def test_obb_polygon_is_ccw_and_uses_the_actual_heading(self):
        shape = VehicleShape(2.0, 1.0)
        corners = obb_polygon((1.0, -2.0), 0.0, shape)
        self.assertEqual(corners, (
            (0.0, -2.5), (2.0, -2.5), (2.0, -1.5), (0.0, -1.5)))
        self.assertGreater(_signed_twice_area(corners), 0.0)
        rotated = obb_polygon((0.0, 0.0), math.pi / 2.0, shape)
        for x, y in rotated:
            self.assertAlmostEqual(abs(x), 0.5, places=12)
            self.assertAlmostEqual(abs(y), 1.0, places=12)
        self.assertGreater(_signed_twice_area(rotated), 0.0)
        self.assertAlmostEqual(
            obb_support_halfwidth((1.0, 0.0), math.pi / 2.0, shape),
            0.5, places=12)
        self.assertAlmostEqual(
            obb_support_halfwidth((0.0, 1.0), math.pi / 2.0, shape),
            1.0, places=12)

    def test_invalid_obb_inputs_fail_closed(self):
        good = VehicleShape(0.738, 0.500)
        for shape in (
                types.SimpleNamespace(length_m=0.0, width_m=0.5),
                types.SimpleNamespace(length_m=-0.7, width_m=0.5),
                types.SimpleNamespace(length_m=float("nan"), width_m=0.5),
                types.SimpleNamespace(length_m=0.738, width_m=0.0),
                types.SimpleNamespace(length_m=None, width_m=0.5)):
            with self.assertRaises(VirtualAContractError):
                obb_polygon((0.0, 0.0), 0.0, shape)
            with self.assertRaises(VirtualAContractError):
                obb_support_halfwidth((1.0, 0.0), 0.0, shape)
        with self.assertRaises(VirtualAContractError):
            obb_polygon((0.0, 0.0), float("inf"), good)
        with self.assertRaises(VirtualAContractError):
            obb_support_halfwidth((0.6, 0.7), 0.0, good)  # not unit length


class SingleCorridorCoverageTests(unittest.TestCase):
    """U3: one corridor, complete polygon, explicit gap and band statuses."""

    def setUp(self):
        self.corridor = _rectangle(-2.0, -1.0, 0.3, 1.0)

    def test_polygon_entirely_inside_returns_single_region(self):
        polygon = _rect_points(-1.0, -0.5, -0.5, 0.5)
        result = obb_coverage_in_corridor(
            polygon, self.corridor, region_index=7)
        self.assertEqual(result["status"], COVERAGE_SINGLE_REGION)
        self.assertEqual(result["region_index"], 7)
        self.assertEqual(result["reason"], "ROBUSTLY_INSIDE")
        self.assertLess(result["max_violation_m"], 0.0)
        self.assertEqual(
            result["witness"]["violation_m"],
            result["max_violation_m"])
        for key in ("halfspace_index", "vertex_index", "vertex",
                    "violation_m"):
            self.assertIn(key, result["witness"])

    def test_corner_outside_is_a_strict_gap(self):
        polygon = _rect_points(-0.5, -0.2, 0.5, 0.2)
        result = obb_coverage_in_corridor(polygon, self.corridor)
        self.assertEqual(result["status"], COVERAGE_GAP)
        self.assertAlmostEqual(result["max_violation_m"], 0.2, places=12)
        self.assertGreater(result["witness"]["violation_m"], 0.0)
        self.assertEqual(result["witness"]["vertex"], [0.5, -0.2])

    def test_exact_boundary_contact_is_unconfirmed_not_inside(self):
        polygon = _rect_points(0.5, -0.5, 1.0, 0.5)
        touch = _rectangle(-1.0, -1.0, 1.0, 1.0)
        result = obb_coverage_in_corridor(polygon, touch)
        self.assertEqual(result["status"], COVERAGE_UNCONFIRMED)
        self.assertEqual(result["reason"], "TOLERANCE_BAND")
        self.assertEqual(result["max_violation_m"], 0.0)

    def test_invalid_polygons_fail_closed(self):
        clockwise = _rect_points(-0.5, -0.2, 0.5, 0.2)[::-1]
        cases = {
            "clockwise": clockwise,
            "nan": ((0.0, 0.0), (1.0, 0.0), (float("nan"), 1.0)),
            "inf": ((0.0, 0.0), (1.0, 0.0), (0.0, float("inf"))),
            "two_points": ((0.0, 0.0), (1.0, 0.0)),
            "collinear": ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)),
            "duplicate": ((-0.5, -0.2), (-0.5, -0.2), (0.5, 0.2)),
            "non_convex": ((0.0, 0.0), (1.0, 0.0), (0.5, 0.2),
                           (1.0, 1.0), (0.0, 1.0)),
        }
        for name, polygon in cases.items():
            with self.subTest(name=name):
                result = obb_coverage_in_corridor(
                    polygon, self.corridor)
                self.assertEqual(result["status"], COVERAGE_UNCONFIRMED)
                self.assertIsNone(result["max_violation_m"])

    def test_invalid_tolerance_raises(self):
        polygon = _rect_points(-1.0, -0.5, -0.5, 0.5)
        for value in (-1e-9, float("nan"), float("inf")):
            with self.assertRaises(VirtualAContractError):
                obb_coverage_in_corridor(
                    polygon, self.corridor, tol_m=value)

    def test_non_corridor_argument_raises(self):
        polygon = _rect_points(-1.0, -0.5, -0.5, 0.5)
        with self.assertRaises(VirtualAContractError):
            obb_coverage_in_corridor(polygon, object())


class AdjacentUnionCoverageTests(unittest.TestCase):
    """U1/U2/U3 and the order/degeneracy contract for ``C_L union C_R``."""

    def test_u1_cross_region_polygon_is_covered_by_both_orders(self):
        left = _rectangle(-2.0, -1.0, 0.3, 1.0)
        right = _rectangle(-0.3, -1.0, 2.0, 1.0)
        polygon = _rect_points(-0.5, -0.2, 0.5, 0.2)
        self.assertEqual(
            obb_coverage_in_corridor(polygon, left)["status"],
            COVERAGE_GAP)
        self.assertEqual(
            obb_coverage_in_corridor(polygon, right)["status"],
            COVERAGE_GAP)

        result = obb_coverage_in_adjacent_union(polygon, left, right)
        self.assertEqual(result["status"], COVERAGE_ADJACENT_UNION)
        self.assertIsNone(result["region_index"])
        self.assertEqual(
            result["order_results"],
            {"forward": "COVERED", "reverse": "COVERED"})
        self.assertEqual(result["reason"], "BOTH_ORDERS_COVERED")
        self.assertIn("covered_pieces_forward", result["witness"])
        self.assertIn("covered_pieces_reverse", result["witness"])
        self.assertLess(result["max_violation_m"], 0.0)

        swapped = obb_coverage_in_adjacent_union(polygon, right, left)
        self.assertEqual(swapped["status"], COVERAGE_ADJACENT_UNION)
        self.assertEqual(
            swapped["order_results"],
            {"forward": "COVERED", "reverse": "COVERED"})

        rotated = obb_coverage_in_adjacent_union(
            _rotate_start(polygon, 2), left, right)
        self.assertEqual(rotated["status"], COVERAGE_ADJACENT_UNION)

    def test_u2_diamond_corner_trap_is_never_covered(self):
        left = _rectangle(-2.0, -2.0, 0.0, 2.0)
        right = _rectangle(-2.0, -2.0, 2.0, 0.0)
        # CCW diamond: top -> left -> bottom -> right.
        polygon = ((0.0, 1.0), (-1.0, 0.0), (0.0, -1.0), (1.0, 0.0))
        # Every original corner lies on the boundary of at least one
        # region, so a four-corner test would wrongly accept this body.
        self.assertEqual(
            obb_coverage_in_corridor(polygon, left)["status"],
            COVERAGE_GAP)
        self.assertEqual(
            obb_coverage_in_corridor(polygon, right)["status"],
            COVERAGE_GAP)
        result = obb_coverage_in_adjacent_union(polygon, left, right)
        self.assertIn(result["status"],
                      (COVERAGE_GAP, COVERAGE_UNCONFIRMED))
        self.assertNotEqual(result["status"], COVERAGE_ADJACENT_UNION)
        self.assertEqual(
            result["order_results"],
            {"forward": "GAP", "reverse": "GAP"})
        self.assertEqual(result["status"], COVERAGE_GAP)
        self.assertGreater(result["witness"]["violation_m"], 0.0)

        clockwise = ((0.0, 1.0), (1.0, 0.0), (0.0, -1.0), (-1.0, 0.0))
        rejected = obb_coverage_in_adjacent_union(clockwise, left, right)
        self.assertEqual(rejected["status"], COVERAGE_UNCONFIRMED)

    def test_u3_single_region_membership_is_reported_per_side(self):
        left = _rectangle(-2.0, -1.0, 0.3, 1.0)
        right = _rectangle(-0.3, -1.0, 2.0, 1.0)
        inside_left = _rect_points(-1.0, -0.5, -0.5, 0.5)
        result = obb_coverage_in_adjacent_union(inside_left, left, right)
        self.assertEqual(result["status"], COVERAGE_SINGLE_REGION)
        self.assertEqual(result["region_index"], 0)
        self.assertEqual(result["reason"], "ROBUSTLY_INSIDE_LEFT_REGION")
        swapped = obb_coverage_in_adjacent_union(inside_left, right, left)
        self.assertEqual(swapped["status"], COVERAGE_SINGLE_REGION)
        self.assertEqual(swapped["region_index"], 1)
        self.assertEqual(
            swapped["reason"], "ROBUSTLY_INSIDE_RIGHT_REGION")

        inside_right = _rect_points(1.0, -0.5, 1.5, 0.5)
        result = obb_coverage_in_adjacent_union(inside_right, left, right)
        self.assertEqual(result["status"], COVERAGE_SINGLE_REGION)
        self.assertEqual(result["region_index"], 1)

    def test_overlap_returns_first_region_deterministically(self):
        left = _rectangle(-2.0, -1.0, 0.3, 1.0)
        right = _rectangle(-0.3, -1.0, 2.0, 1.0)
        overlap = _rect_points(-0.2, -0.2, 0.2, 0.2)
        result = obb_coverage_in_adjacent_union(overlap, left, right)
        self.assertEqual(result["status"], COVERAGE_SINGLE_REGION)
        self.assertEqual(result["region_index"], 0)
        swapped = obb_coverage_in_adjacent_union(overlap, right, left)
        self.assertEqual(swapped["status"], COVERAGE_SINGLE_REGION)
        # ``region_index`` is relative to the call arguments: the first
        # argument wins when the body lies in the overlap of both regions.
        self.assertEqual(swapped["region_index"], 0)
        self.assertEqual(
            swapped["reason"], "ROBUSTLY_INSIDE_LEFT_REGION")

    def test_strict_gap_between_regions_cannot_be_averaged_away(self):
        left = _rectangle(-2.0, -1.0, 0.3, 1.0)
        right = _rectangle(0.4, -1.0, 2.0, 1.0)
        polygon = _rect_points(-0.5, -0.2, 0.5, 0.2)
        result = obb_coverage_in_adjacent_union(
            polygon, left, right, tol_m=1e-9)
        self.assertEqual(result["status"], COVERAGE_GAP)
        self.assertAlmostEqual(result["witness"]["violation_m"], 0.1,
                               places=12)
        # A tolerance wider than the real gap may only become ambiguous;
        # it must never turn the gap into a covered union.
        band = obb_coverage_in_adjacent_union(
            polygon, left, right, tol_m=0.2)
        self.assertEqual(band["status"], COVERAGE_UNCONFIRMED)
        self.assertEqual(band["reason"], "ORDER_RESULTS_DISAGREE")
        # A tolerance smaller than the gap still proves the strict gap.
        strict = obb_coverage_in_adjacent_union(
            polygon, left, right, tol_m=1e-6)
        self.assertEqual(strict["status"], COVERAGE_GAP)

    def test_narrow_sliver_is_unconfirmed(self):
        left = _rectangle(0.0, 0.0, 1.0, 1.0)
        right = _rectangle(1.0, 0.0, 2.0, 1.0)
        sliver = _rect_points(
            1.0 - 5e-10, 0.2, 1.0 + 5e-10, 0.8)
        result = obb_coverage_in_adjacent_union(sliver, left, right)
        self.assertEqual(result["status"], COVERAGE_UNCONFIRMED)
        self.assertNotEqual(result["status"], COVERAGE_ADJACENT_UNION)

    def test_exact_tangent_union_is_unconfirmed(self):
        left = _rectangle(-1.0, -1.0, 1.0, 1.0)
        right = _rectangle(1.0, -1.0, 2.0, 1.0)
        polygon = _rect_points(0.5, -0.5, 1.0, 0.5)
        result = obb_coverage_in_adjacent_union(polygon, left, right)
        self.assertEqual(result["status"], COVERAGE_UNCONFIRMED)
        self.assertNotEqual(result["status"], COVERAGE_ADJACENT_UNION)

    def test_invalid_polygon_and_non_corridor_fail_closed(self):
        left = _rectangle(-1.0, -1.0, 1.0, 1.0)
        right = _rectangle(-1.0, -1.0, 1.0, 1.0)
        result = obb_coverage_in_adjacent_union(
            ((0.0, 0.0), (1.0, 0.0), (float("nan"), 1.0)), left, right)
        self.assertEqual(result["status"], COVERAGE_UNCONFIRMED)
        self.assertEqual(result["reason"], "NON_FINITE_POLYGON")
        with self.assertRaises(VirtualAContractError):
            obb_coverage_in_adjacent_union(
                _rect_points(-0.5, -0.5, 0.5, 0.5), left, object())


def _chain(regions, grid):
    return CorridorChain(
        regions=tuple(regions), route_source="formation_assessment_test",
        grid_sha256=grid_signature(grid))


def _test_grid(width=400, height=200, origin_x=-5.0, origin_y=-5.0):
    return Grid(np.zeros((height, width), dtype=bool), 0.05,
                origin_x, origin_y)


def _wall_grid():
    grid = _test_grid()
    # 0.2 x 0.4 m wall around x=[4.9, 5.1], y=[1.2, 1.6].
    grid.occ[84:92, 138:142] = True
    return grid


def _portal_gap_inputs():
    """Two-region diamond trap with one vehicle OBB strictly in the gap."""
    grid = _test_grid()
    left = _rectangle(-4.0, -4.0, 0.0, 4.0)
    right = _rectangle(-4.0, -4.0, 4.0, 0.0)
    points = (
        (-1.3 + 1.2 * math.cos(math.radians(230.0)),
         -0.4 + 1.2 * math.sin(math.radians(230.0))),
        (-1.3, -0.4),
        (-0.14088900845311803, -0.7105828541230249))
    start_heading = math.atan2(points[1][1] - points[0][1],
                               points[1][0] - points[0][0])
    goal_heading = math.atan2(points[2][1] - points[1][1],
                              points[2][0] - points[1][0])
    return grid, (left, right), points, start_heading, goal_heading


def _assess(chain, points, checker, model, shapes, start_heading, goal_heading,
            grid, **kwargs):
    curvature = tuple(float(value)
                      for value in _node_route_curvature(points))
    options = {
        "route_index": 0,
        "round_index": 0,
        "guide_source": "unit_test_guide",
        "allocation": ((0.0, 0.5, 0), (0.5, 1.0, 1)),
        "node_curvature": curvature,
        "grid_sha256": grid_signature(grid),
        "request_sha256": "a" * 64,
        "planning_context_sha256": "b" * 64,
    }
    options.update(kwargs)
    return assess_formation_guide(
        chain, points, checker, model, shapes, start_heading, goal_heading,
        **options)


class TransitionWindowTests(unittest.TestCase):
    """Portal windows are finite reach hints with explicit clamping."""

    def test_window_formula_and_sigma(self):
        grid = _test_grid()
        chain = _chain((_rectangle(-4.0, -4.0, 0.0, 4.0),
                        _rectangle(-4.0, -4.0, 4.0, 0.0)), grid)
        points = ((-4.0, -1.0), (-1.0, -1.0), (3.0, -1.0))
        windows = formation_transition_windows(
            chain, points, MODEL, SHAPES)
        self.assertEqual(len(windows), 1)
        window = windows[0]
        rho = math.hypot(1.0, 0.0) + 0.5 * math.hypot(0.738, 0.500)
        self.assertAlmostEqual(window["rho_m"], rho, places=12)
        self.assertAlmostEqual(window["sigma_hint"], 3.0 / 7.0, places=12)
        self.assertAlmostEqual(window["s_window_m"][0], 3.0 - rho, places=12)
        self.assertAlmostEqual(window["s_window_m"][1], 3.0 + rho, places=12)
        self.assertAlmostEqual(window["sigma_window"][0],
                               (3.0 - rho) / 7.0, places=12)
        self.assertAlmostEqual(window["sigma_window"][1],
                               (3.0 + rho) / 7.0, places=12)
        self.assertEqual(window["source_region_index"], 0)
        self.assertEqual(window["target_region_index"], 1)
        self.assertEqual(window["portal_seed_point"], [-1.0, -1.0])

    def test_window_is_clamped_to_the_guide(self):
        grid = _test_grid()
        chain = _chain((_rectangle(-4.0, -4.0, 0.0, 4.0),
                        _rectangle(-4.0, -4.0, 4.0, 0.0)), grid)
        points = ((-2.0713451316238474, -1.3192533317427735),
                  (-1.3, -0.4),
                  (-0.14088900845311803, -0.7105828541230249))
        window = formation_transition_windows(
            chain, points, MODEL, SHAPES)[0]
        self.assertGreaterEqual(window["s_window_m"][0], 0.0)
        self.assertLessEqual(window["s_window_m"][1],
                             sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                 for a, b in zip(points, points[1:])))
        self.assertGreaterEqual(window["sigma_window"][0], 0.0)
        self.assertLessEqual(window["sigma_window"][1], 1.0)

    def test_missing_shape_makes_the_window_unconfirmed(self):
        grid = _test_grid()
        chain = _chain((_rectangle(-4.0, -4.0, 0.0, 4.0),
                        _rectangle(-4.0, -4.0, 4.0, 0.0)), grid)
        points = ((-4.0, -1.0), (-1.0, -1.0), (3.0, -1.0))
        windows = formation_transition_windows(
            chain, points, MODEL, {1: SHAPES[1], 2: SHAPES[2]})
        self.assertIsNone(windows[0]["rho_m"])
        self.assertIsNone(windows[0]["s_window_m"])
        self.assertEqual(windows[0]["reason"], "MISSING_VEHICLE_SHAPE")


class FormationAssessmentTests(unittest.TestCase):
    """Real checker samples keep map, coverage and kinematic states apart."""

    def test_clear_guide_reports_all_three_states_independently(self):
        grid = empty_grid(width=400, height=200, origin_x=-5.0,
                          origin_y=-5.0)
        chain = _chain((_rectangle(-2.0, -3.0, 12.0, 3.0),), grid)
        points = ((0.0, 0.0), (10.0, 0.0))
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        result = _assess(chain, points, checker, MODEL, SHAPES, 0.0, 0.0,
                         grid)
        self.assertEqual(result["map_sample_status"],
                         MAP_CLEAR_AT_SAMPLES)
        self.assertEqual(result["map_sample_counts"]["unconfirmed"], 0)
        self.assertEqual(result["coverage_sample_status"],
                         COVERAGE_SINGLE_REGION)
        self.assertEqual(result["kinematic_hint_status"],
                         KINEMATIC_CURVATURE_UNCONFIRMED)
        self.assertEqual(result["normal_branch_status"],
                         KINEMATIC_NORMAL_BRANCH_OK)
        self.assertEqual(result["guide_hint_status"], GUIDE_HINT_VALID)
        self.assertFalse(result["sample_cap"])
        self.assertGreater(result["station_count"], 2)
        for vehicle_id in (1, 2, 3):
            vehicle = result["kinematic_vehicles"][vehicle_id]
            self.assertEqual(vehicle["vehicle_curvature_status"],
                             KINEMATIC_CURVATURE_UNCONFIRMED)
        self.assertEqual(result["candidate_certification"],
                         CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY)
        self.assertFalse(result["formation_obb_hard_constraints"])
        self.assertFalse(result["continuously_certified"])
        self.assertEqual(result["legacy_f_allocation"],
                         [[0.0, 0.5, 0], [0.5, 1.0, 1]])
        self.assertEqual(result["transition_windows"], [])

    def test_map_obstacle_collision_is_observed_not_clear(self):
        grid = _wall_grid()
        chain = _chain((_rectangle(-2.0, -3.0, 12.0, 3.0),), grid)
        points = ((0.0, 0.0), (10.0, 0.0))
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        result = _assess(chain, points, checker, MODEL, SHAPES, 0.0, 0.0,
                         grid)
        self.assertEqual(result["map_sample_status"],
                         MAP_COLLISION_OBSERVED)
        self.assertGreater(result["map_sample_counts"]["map_collision"], 0)
        collisions = result["vehicle_map_collisions"]
        self.assertTrue(collisions)
        self.assertEqual(collisions[0]["reason"], "MAP_OBB_COLLISION")
        # The wall sits on the right-side vehicle (offset b=-1.0 m).
        self.assertEqual(collisions[0]["vehicle_id"], 3)
        self.assertIsNotNone(collisions[0]["station_index"])
        self.assertNotEqual(result["map_sample_status"],
                            MAP_CLEAR_AT_SAMPLES)
        # Corridor containment is a separate field and stays successful.
        self.assertEqual(result["coverage_sample_status"],
                         COVERAGE_SINGLE_REGION)

    def test_vehicle_sat_collision_keeps_its_own_reason(self):
        grid = empty_grid(width=400, height=200, origin_x=-5.0,
                          origin_y=-5.0)
        large = {1: VehicleShape(2.0, 2.0), 2: VehicleShape(2.0, 2.0),
                 3: VehicleShape(2.0, 2.0)}
        chain = _chain((_rectangle(-3.0, -4.0, 13.0, 4.0),), grid)
        points = ((0.0, 0.0), (10.0, 0.0))
        checker = FormationCollisionChecker(grid, MODEL, large, 0.05)
        result = _assess(chain, points, checker, MODEL, large, 0.0, 0.0,
                         grid)
        self.assertEqual(result["map_sample_status"],
                         MAP_COLLISION_OBSERVED)
        self.assertGreater(
            result["map_sample_counts"]["vehicle_collision"], 0)
        self.assertEqual(result["map_sample_counts"]["map_collision"], 0)
        self.assertEqual(result["vehicle_map_collisions"][0]["reason"],
                         "VEHICLE_OBB_COLLISION")
        self.assertIsNotNone(
            result["vehicle_map_collisions"][0]["other_vehicle_id"])

    def test_union_gap_is_coverage_insufficient_not_map_collision(self):
        grid, regions, points, start_heading, goal_heading = \
            _portal_gap_inputs()
        chain = _chain(regions, grid)
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        result = _assess(chain, points, checker, MODEL, SHAPES,
                         start_heading, goal_heading, grid)
        # The authoritative map/vehicle SAT is clear at every sample.
        self.assertEqual(result["map_sample_status"],
                         MAP_CLEAR_AT_SAMPLES)
        self.assertEqual(result["vehicle_map_collisions"], [])
        # The whole-OBB adjacent-union test still proves a strict gap.
        self.assertEqual(result["coverage_sample_status"], COVERAGE_GAP)
        self.assertEqual(result["transition_windows"][0]["status"],
                         "PORTAL_TRANSITION_GAP")
        self.assertEqual(
            result["transition_windows"][0]["map_sample_status"],
            MAP_CLEAR_AT_SAMPLES)
        vehicle_one = result["coverage_vehicles"][1]
        self.assertEqual(vehicle_one["status"], COVERAGE_GAP)
        self.assertGreater(vehicle_one["counts"]["gap"], 0)
        self.assertGreater(vehicle_one["worst_gap_violation_m"], 0.0)
        witness = vehicle_one["witnesses"][0]
        self.assertEqual(witness["status"], COVERAGE_GAP)
        self.assertIn("vertex", witness["witness"])
        # A strict gap is not a physical infeasibility certificate.
        self.assertFalse(result["continuously_certified"])
        self.assertFalse(result["formation_obb_hard_constraints"])

    def test_normal_branch_invalid_does_not_fabricate_an_obb(self):
        grid = _test_grid()
        chain = _chain((_rectangle(-1.0, -1.0, 1.2, 1.0),
                        _rectangle(0.9, -1.0, 2.0, 1.0)), grid)
        points = ((0.0, 0.0), (1.0, 0.0), (1.05, 0.05))
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        result = _assess(chain, points, checker, MODEL, SHAPES, 0.0,
                         math.atan2(0.05, 0.05), grid)
        self.assertEqual(result["kinematic_hint_status"],
                         KINEMATIC_NORMAL_BRANCH_INVALID)
        self.assertEqual(result["normal_branch_status"],
                         KINEMATIC_NORMAL_BRANCH_INVALID)
        vehicle_two = result["kinematic_vehicles"][2]
        self.assertEqual(vehicle_two["normal_branch_status"],
                         KINEMATIC_NORMAL_BRANCH_INVALID)
        self.assertEqual(vehicle_two["vehicle_curvature_status"],
                         KINEMATIC_CURVATURE_UNCONFIRMED)
        self.assertEqual(vehicle_two["witnesses"][0]["reason"],
                         "NORMAL_BRANCH_VIOLATION")
        # The failing station is UNKNOWN, never a fabricated clear OBB.
        self.assertEqual(result["map_sample_status"],
                         MAP_SAMPLE_UNCONFIRMED)
        self.assertEqual(result["map_sample_counts"]["clear"],
                         result["station_count"] - 1)
        self.assertFalse(result["continuously_certified"])

    def test_sample_cap_marks_the_window_unconfirmed(self):
        grid, regions, points, start_heading, goal_heading = \
            _portal_gap_inputs()
        chain = _chain(regions, grid)
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        result = _assess(chain, points, checker, MODEL, SHAPES,
                         start_heading, goal_heading, grid, max_stations=8)
        self.assertTrue(result["sample_cap"])
        self.assertGreater(result["dropped_required_stations"], 0)
        self.assertEqual(result["coverage_sample_status"],
                         COVERAGE_UNCONFIRMED)
        self.assertEqual(result["transition_windows"][0]["status"],
                         "PORTAL_TRANSITION_UNCONFIRMED")
        self.assertIn("SAMPLE_CAP",
                      result["transition_windows"][0]["reason"])
        self.assertLessEqual(result["station_count"], 8)
        self.assertNotEqual(result["coverage_sample_status"],
                            COVERAGE_ADJACENT_UNION)

    def test_missing_shape_is_unknown_not_fabricated_success(self):
        grid, regions, points, start_heading, goal_heading = \
            _portal_gap_inputs()
        chain = _chain(regions, grid)
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        result = assess_formation_guide(
            chain, points, checker, MODEL, {1: SHAPES[1], 2: SHAPES[2]},
            start_heading, goal_heading,
            grid_sha256=grid_signature(grid),
            allocation=((0.0, 0.5, 0), (0.5, 1.0, 1)))
        self.assertIn("MISSING_VEHICLE_SHAPE",
                      result["incomplete_reasons"])
        self.assertEqual(result["map_sample_status"],
                         MAP_SAMPLE_UNCONFIRMED)
        self.assertNotEqual(result["map_sample_status"],
                            MAP_CLEAR_AT_SAMPLES)
        self.assertEqual(result["coverage_sample_status"],
                         COVERAGE_UNCONFIRMED)
        self.assertEqual(result["coverage_vehicles"][3]["status"],
                         COVERAGE_UNCONFIRMED)
        self.assertEqual(result["transition_windows"][0]["status"],
                         "PORTAL_TRANSITION_UNCONFIRMED")
        self.assertIn("MISSING_VEHICLE_SHAPE",
                      result["transition_windows"][0]["reason"])
        self.assertFalse(result["continuously_certified"])

    def test_identity_mismatch_and_limits_fail_closed(self):
        grid = _test_grid()
        chain = _chain((_rectangle(-1.0, -4.0, 12.0, 4.0),), grid)
        points = ((0.0, 0.0), (10.0, 0.0))
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        with self.assertRaises(VirtualAContractError) as wrong_grid:
            assess_formation_guide(
                chain, points, checker, MODEL, SHAPES, 0.0, 0.0,
                grid_sha256="0" * 64)
        self.assertEqual(wrong_grid.exception.reason,
                         "INVALID_ASSESSMENT_CHECKER")
        with self.assertRaises(VirtualAContractError) as limit:
            assess_formation_guide(
                chain, points, checker, MODEL, SHAPES, 0.0, 0.0,
                grid_sha256=grid_signature(grid), max_stations=0)
        self.assertEqual(limit.exception.reason,
                         "INVALID_ASSESSMENT_LIMIT")

    def test_candidate_identity_is_deterministic_and_bound(self):
        grid = empty_grid(width=400, height=200, origin_x=-5.0,
                          origin_y=-5.0)
        chain = _chain((_rectangle(-2.0, -3.0, 12.0, 3.0),), grid)
        points = ((0.0, 0.0), (10.0, 0.0))
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        first = _assess(chain, points, checker, MODEL, SHAPES, 0.0, 0.0,
                        grid)
        second = _assess(chain, points, checker, MODEL, SHAPES, 0.0, 0.0,
                         grid)
        self.assertEqual(len(first["candidate_id"]), 64)
        self.assertEqual(first["candidate_id"], second["candidate_id"])
        self.assertEqual(first["selected_guide_sha256"],
                         second["selected_guide_sha256"])
        self.assertEqual(first["request_sha256"], "a" * 64)
        self.assertEqual(first["grid_sha256"], grid_signature(grid))
        self.assertEqual(first["planning_context_sha256"], "b" * 64)
        self.assertEqual(first["chain_digest"], chain.digest)
        changed_round = _assess(chain, points, checker, MODEL, SHAPES,
                                0.0, 0.0, grid, round_index=1)
        changed_route = _assess(chain, points, checker, MODEL, SHAPES,
                                0.0, 0.0, grid, route_index=2)
        self.assertNotEqual(first["candidate_id"],
                            changed_round["candidate_id"])
        self.assertNotEqual(first["candidate_id"],
                            changed_route["candidate_id"])

    def test_zero_length_guide_is_degenerate(self):
        grid = _test_grid()
        chain = _chain((_rectangle(-3.0, -3.0, 3.0, 3.0),), grid)
        points = ((0.0, 0.0), (0.0, 0.0))
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        result = _assess(chain, points, checker, MODEL, SHAPES, 0.0, 0.0,
                         grid)
        self.assertEqual(result["guide_hint_status"],
                         GUIDE_HINT_DEGENERATE)
        self.assertEqual(result["coverage_sample_status"],
                         COVERAGE_UNCONFIRMED)
        self.assertFalse(result["continuously_certified"])


class RouteAssessmentTests(unittest.TestCase):
    """A* route coarse sampling stays bounded and claims no corridor state."""

    def test_route_assessment_uses_real_checker_and_stays_diagnostic(self):
        grid = _wall_grid()
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        points = ((0.0, 0.0), (5.0, 0.0), (10.0, 0.0))
        curvature = tuple(float(value)
                          for value in _node_route_curvature(points))
        result = assess_route_guide(
            1, points, checker, MODEL, SHAPES, 0.0, 0.0,
            node_curvature=curvature,
            grid_sha256=grid_signature(grid),
            request_sha256="c" * 64,
            planning_context_sha256="d" * 64)
        self.assertEqual(result["schema"], "virtual_a_route_assessment_v1")
        self.assertEqual(result["route_index"], 1)
        self.assertEqual(result["grid_sha256"], grid_signature(grid))
        self.assertEqual(result["map_sample_status"],
                         MAP_COLLISION_OBSERVED)
        self.assertGreater(result["map_sample_counts"]["map_collision"], 0)
        self.assertEqual(result["coverage_sample_status"],
                         COVERAGE_UNCONFIRMED)
        self.assertIsNone(result["chain_digest"])
        self.assertLessEqual(result["station_count"], 32)
        self.assertEqual(result["candidate_certification"],
                         CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY)
        self.assertFalse(result["continuously_certified"])

    def test_route_sample_cap_is_unconfirmed(self):
        grid = empty_grid(width=400, height=200, origin_x=-5.0,
                          origin_y=-5.0)
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        points = tuple((float(index) * 0.75, 0.0) for index in range(21))
        curvature = tuple(float(value)
                          for value in _node_route_curvature(points))
        result = assess_route_guide(
            0, points, checker, MODEL, SHAPES, 0.0, 0.0,
            node_curvature=curvature,
            grid_sha256=grid_signature(grid), max_stations=6)
        self.assertTrue(result["sample_cap"])
        self.assertLessEqual(result["station_count"], 6)
        self.assertNotEqual(result["map_sample_status"],
                            MAP_CLEAR_AT_SAMPLES)


def _point_violation(point, region):
    """Max outward halfspace violation of one point in one corridor."""
    return max(row[0] * point[0] + row[1] * point[1] - limit
               for row, limit in zip(region.A, region.b))


def _transition_window(portal_index, status="PORTAL_TRANSITION_SAMPLED",
                       s_window_m=None):
    return {
        "portal_index": int(portal_index),
        "status": str(status),
        "s_window_m": (None if s_window_m is None else
                       (float(s_window_m[0]), float(s_window_m[1]))),
        "vehicles": [],
        "collisions": [],
    }


def _baseline_assessment(windows):
    """Minimal real-shaped baseline with only Portal risk fields."""
    return {
        "transition_windows": tuple(windows),
        "vehicle_map_collisions": (),
        "coverage_vehicles": {},
    }


class GuideCandidateGenerationTests(unittest.TestCase):
    """Rework A: one baseline-driven Portal repair stays bounded.

    The local variants must follow the risk Portal that the baseline
    assessment actually reports, change only that Portal point, remain inside
    both adjacent corridor halfspaces, and re-allocate without relaxing any
    F-only constraint.  A real checker improvement is selected by the same
    lexicographic assessment used by the pipeline.
    """

    @staticmethod
    def _multi_portal_fixture():
        grid = _test_grid()
        regions = (
            _rectangle(-1.0, -2.0, 3.0, 2.0),
            _rectangle(2.0, -2.0, 8.0, 2.0),
            _rectangle(7.0, -2.0, 13.0, 2.0),
            _rectangle(12.0, -2.0, 18.0, 2.0),
        )
        chain = _chain(regions, grid)
        base = GuideRoute(
            points=((-0.5, 0.0), (2.5, 0.0), (7.5, 0.0),
                    (12.5, 0.0), (17.0, 0.0)),
            source="unit_base_guide", grid_sha256=grid_signature(grid))
        baseline = _baseline_assessment((
            _transition_window(0, s_window_m=(1.5, 3.5)),
            _transition_window(1, s_window_m=(6.5, 8.5)),
            _transition_window(2, "PORTAL_TRANSITION_GAP",
                               s_window_m=(11.5, 13.5)),
        ))
        return grid, chain, base, baseline

    def test_later_portal_risk_changes_only_that_portal_and_reallocates(self):
        grid, chain, base, baseline = self._multi_portal_fixture()
        target, selection = select_guide_target_portal(
            chain, base, baseline)
        self.assertEqual(selection["schema"], GUIDE_CANDIDATE_EVIDENCE_SCHEMA)
        self.assertEqual(target, 3)
        self.assertEqual(selection["selection"], "BASELINE_RISK")
        self.assertEqual(selection["target_portal_index"], 3)
        self.assertEqual(selection["target_window_index"], 2)
        self.assertEqual(selection["target_reason"], "UNION_GAP")
        self.assertEqual(
            [entry["portal_index"] for entry in selection["risk_portals"]],
            [3])
        self.assertEqual(selection["certification_claim"], "NONE")
        self.assertEqual(selection["candidate_certification"],
                         CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY)
        self.assertFalse(selection["continuously_certified"])
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        candidates, generation = plan_guide_candidates(
            chain, (-0.5, 0.0), (17.0, 0.0), base_guide=base,
            baseline_assessment=baseline, checker=checker, model=MODEL,
            vehicle_shapes=SHAPES)
        self.assertLessEqual(len(candidates),
                             CORRIDOR_GUIDE_CANDIDATE_LIMIT)
        self.assertEqual(candidates[0].points, base.points)
        self.assertEqual(generation["target_portal_index"], 3)
        self.assertEqual(generation["local_candidates_kept"],
                         CORRIDOR_GUIDE_PORTAL_VARIANT_LIMIT)
        self.assertLessEqual(
            generation["local_candidate_point_count"],
            PORTAL_LOCAL_CANDIDATE_POINT_LIMIT)
        local = [candidate for candidate in candidates
                 if str((candidate.quality or {}).get(
                     "candidate_origin") or "").startswith(
                         "ONE_PORTAL_VARIANT_")]
        self.assertEqual(len(local), CORRIDOR_GUIDE_PORTAL_VARIANT_LIMIT)
        for candidate in local:
            # Start, goal and the first two Portal points are byte-identical
            # to the base; only the target Portal point moves.
            self.assertEqual(candidate.points[:3], base.points[:3])
            self.assertEqual(candidate.points[4], base.points[4])
            self.assertNotEqual(candidate.points[3], base.points[3])
            self.assertLessEqual(
                _point_violation(
                    candidate.points[3], chain.regions[2]), 1e-9)
            self.assertLessEqual(
                _point_violation(
                    candidate.points[3], chain.regions[3]), 1e-9)
            quality = candidate.quality or {}
            self.assertEqual(quality["target_portal_index"], 3)
            self.assertEqual(quality["target_window_index"], 2)
            self.assertEqual(quality["candidate_certification"],
                             CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY)
        # Every complete Guide is re-allocated through the same F-only
        # corridor assignment; none needs a relaxed halfspace.
        for candidate in candidates:
            allocation = allocate_corridors(chain, candidate)
            self.assertTrue(allocation)

    def test_earliest_baseline_risk_wins_over_a_later_risk_portal(self):
        _grid, chain, base, baseline = self._multi_portal_fixture()
        windows = [dict(window) for window in baseline["transition_windows"]]
        windows[0]["status"] = "PORTAL_TRANSITION_UNCONFIRMED"
        baseline = dict(baseline)
        baseline["transition_windows"] = tuple(windows)
        target, selection = select_guide_target_portal(
            chain, base, baseline)
        self.assertEqual(target, 1)
        self.assertEqual(selection["target_portal_index"], 1)
        self.assertEqual(selection["target_window_index"], 0)
        self.assertEqual(selection["target_reason"], "TRANSITION_UNKNOWN")
        self.assertEqual(
            [entry["portal_index"] for entry in selection["risk_portals"]],
            [1, 3])

    def _no_risk_fixture(self, later_left):
        grid = _test_grid()
        regions = (
            _rectangle(-1.0, -2.0, 3.0, 2.0),
            _rectangle(2.0, -2.0, 12.0, 2.0),
            _rectangle(later_left, -2.0, 20.0, 2.0),
        )
        chain = _chain(regions, grid)
        portal_point = 0.5 * (later_left + 12.0)
        base = GuideRoute(
            points=((-0.5, 0.0), (2.5, 0.0), (portal_point, 0.0),
                    (17.0, 0.0)),
            source="unit_base_guide", grid_sha256=grid_signature(grid))
        baseline = _baseline_assessment((
            _transition_window(0, s_window_m=(1.5, 3.5)),
            _transition_window(1, s_window_m=(later_left - 1.0,
                                              later_left + 1.0)),
        ))
        return chain, base, baseline

    def test_no_risk_portal_uses_the_smallest_chebyshev_radius(self):
        chain, base, baseline = self._no_risk_fixture(11.5)
        target, selection = select_guide_target_portal(
            chain, base, baseline)
        self.assertEqual(target, 2)
        self.assertEqual(selection["selection"],
                         "SMALLEST_CHEBYSHEV_RADIUS")
        self.assertEqual(selection["target_reason"],
                         "SMALLEST_CHEBYSHEV_RADIUS")
        radii = dict(selection["portal_radii_m"])
        self.assertAlmostEqual(radii[1], 0.5, places=12)
        self.assertAlmostEqual(radii[2], 0.25, places=12)
        self.assertAlmostEqual(selection["target_radius_m"], 0.25,
                               places=12)
        self.assertEqual(selection["risk_portals"], [])

    def test_no_risk_radius_tie_uses_the_lowest_portal_index(self):
        chain, base, baseline = self._no_risk_fixture(11.0)
        target, selection = select_guide_target_portal(
            chain, base, baseline)
        self.assertEqual(target, 1)
        radii = dict(selection["portal_radii_m"])
        self.assertAlmostEqual(radii[1], radii[2], places=12)
        self.assertEqual(selection["selection"],
                         "SMALLEST_CHEBYSHEV_RADIUS")

    def _real_collision_fixture(self):
        grid = _test_grid()
        # Two small map obstacles straddle the Portal approach at y=+-1.
        grid.occ[118:121, 198:201] = True
        grid.occ[78:81, 198:201] = True
        regions = (
            _rectangle(-1.0, -2.0, 6.0, 2.0),
            _rectangle(4.0, -2.0, 11.0, 2.0),
        )
        chain = _chain(regions, grid)
        base = GuideRoute(
            points=((0.0, 0.0), (5.0, 0.0), (10.0, 0.0)),
            source="unit_base_guide", grid_sha256=grid_signature(grid))
        return grid, chain, base

    def test_real_static_check_improvement_is_selected_by_assessment(self):
        grid, chain, base = self._real_collision_fixture()
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.0)
        baseline = _assess(
            chain, base.points, checker, MODEL, SHAPES, 0.0, 0.0, grid,
            guide_source=base.source)
        self.assertEqual(baseline["map_sample_status"],
                         MAP_COLLISION_OBSERVED)
        self.assertGreater(
            baseline["map_sample_counts"]["map_collision"], 0)
        target, selection = select_guide_target_portal(
            chain, base, baseline)
        self.assertEqual(target, 1)
        self.assertEqual(selection["target_reason"], "MAP_COLLISION")
        candidates, generation = plan_guide_candidates(
            chain, (0.0, 0.0), (10.0, 0.0), base_guide=base,
            baseline_assessment=baseline, checker=checker, model=MODEL,
            vehicle_shapes=SHAPES)
        self.assertEqual(generation["target_portal_index"], 1)
        records = []
        assessments = []
        for index, candidate in enumerate(candidates):
            allocation = allocate_corridors(chain, candidate)
            assessment = _assess(
                chain, candidate.points, checker, MODEL, SHAPES, 0.0, 0.0,
                grid, guide_source=candidate.source,
                allocation=allocation)
            assessments.append(assessment)
            records.append(pipeline_module._guide_candidate_record(
                index, candidate, allocation, assessment))
        scores = [pipeline_module._guide_candidate_score(record)
                  for record in records]
        selected = min(range(len(records)),
                       key=lambda index: scores[index])
        self.assertNotEqual(selected, 0)
        self.assertTrue(records[selected]["candidate_origin"].startswith(
            "ONE_PORTAL_VARIANT_"))
        self.assertGreater(
            records[0]["selection_counts"]["collision_count"], 0)
        self.assertEqual(
            records[selected]["selection_counts"]["collision_count"], 0)
        self.assertLess(
            records[selected]["selection_counts"]["collision_count"],
            records[0]["selection_counts"]["collision_count"])
        # The record, the assessment and the re-allocation are one source:
        # the candidate identity is the digest of exactly this guide.
        self.assertEqual(records[selected]["assessment"]["candidate_id"],
                         assessments[selected]["candidate_id"])
        self.assertEqual(assessments[selected]["selected_guide_source"],
                         str(candidates[selected].source))
        self.assertEqual(
            assessments[selected]["legacy_f_allocation"],
            [[float(entry[0]), float(entry[1]), int(entry[2])]
             for entry in allocate_corridors(chain, candidates[selected])])
        self.assertGreater(checker._stats["static_checks"], 0)

    def test_generation_is_capped_at_five_with_three_local_variants(self):
        grid, chain, base, baseline = self._multi_portal_fixture()
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        candidates, generation = plan_guide_candidates(
            chain, (-0.5, 0.0), (17.0, 0.0), base_guide=base,
            baseline_assessment=baseline, checker=checker, model=MODEL,
            vehicle_shapes=SHAPES)
        self.assertEqual(len(candidates),
                         CORRIDOR_GUIDE_CANDIDATE_LIMIT)
        origins = [(candidate.quality or {}).get("candidate_origin")
                   for candidate in candidates]
        self.assertEqual(origins[0], None)
        self.assertEqual(origins[1], "CHEBYSHEV_CENTRE_GUIDE")
        self.assertTrue(all(
            str(origin).startswith("ONE_PORTAL_VARIANT_")
            for origin in origins[2:]))
        self.assertEqual(generation["local_candidates_kept"],
                         CORRIDOR_GUIDE_PORTAL_VARIANT_LIMIT)
        self.assertEqual(
            len(generation["local_candidates"]),
            CORRIDOR_GUIDE_PORTAL_VARIANT_LIMIT)
        self.assertLessEqual(
            generation["local_candidate_point_count"],
            PORTAL_LOCAL_CANDIDATE_POINT_LIMIT)
        # Any fourth local point is dropped with an explicit reason; the
        # canonical base/Chebyshev guides are never dropped.
        self.assertTrue(
            any(entry["reason"] == "LOCAL_CANDIDATE_LIMIT"
                for entry in generation["dropped_local_candidates"]) or
            generation["local_candidate_point_count"] <=
            CORRIDOR_GUIDE_PORTAL_VARIANT_LIMIT)

    def test_budget_exhaustion_keeps_computed_results_and_marks_unknown(self):
        grid, chain, base, baseline = self._multi_portal_fixture()
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        candidates, generation = plan_guide_candidates(
            chain, (-0.5, 0.0), (17.0, 0.0), base_guide=base,
            baseline_assessment=baseline, checker=checker, model=MODEL,
            vehicle_shapes=SHAPES, deadline=-1.0)
        # No budget was extended and no local station was sampled.
        self.assertEqual(checker._stats["static_checks"], 0)
        self.assertLessEqual(len(candidates),
                             CORRIDOR_GUIDE_CANDIDATE_LIMIT)
        self.assertEqual(candidates[0].points, base.points)
        local = [candidate for candidate in candidates
                 if str((candidate.quality or {}).get(
                     "candidate_origin") or "").startswith(
                         "ONE_PORTAL_VARIANT_")]
        self.assertEqual(len(local), CORRIDOR_GUIDE_PORTAL_VARIANT_LIMIT)
        for candidate in local:
            local_assessment = (candidate.quality or {})[
                "local_portal_assessment"]
            self.assertEqual(local_assessment["status"], "UNKNOWN")
            self.assertEqual(local_assessment["reason"], "SOFT_BUDGET")
            self.assertEqual(local_assessment["counts"]["sampled"], 0)
            # UNKNOWN is never converted into a clear claim.
            self.assertEqual(local_assessment["counts"]["map_collision"], 0)
        self.assertEqual(generation["local_candidates_kept"],
                         CORRIDOR_GUIDE_PORTAL_VARIANT_LIMIT)

    def test_without_a_baseline_only_the_canonical_guides_are_generated(
            self):
        grid, chain, base, _baseline = self._multi_portal_fixture()
        checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
        candidates, generation = plan_guide_candidates(
            chain, (-0.5, 0.0), (17.0, 0.0), base_guide=base,
            baseline_assessment=None, checker=checker, model=MODEL,
            vehicle_shapes=SHAPES)
        self.assertEqual(generation["selection"],
                         "NO_BASELINE_ASSESSMENT")
        self.assertIsNone(generation["target_portal_index"])
        self.assertEqual(generation["local_candidate_count"], 0)
        self.assertEqual(checker._stats["static_checks"], 0)
        self.assertTrue(candidates)
        self.assertTrue(all(
            not str((candidate.quality or {}).get("candidate_origin")
                    or "").startswith("ONE_PORTAL_VARIANT_")
            for candidate in candidates))


if __name__ == "__main__":
    unittest.main()