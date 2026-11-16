"""P1 tests for the exact per-panel fourth-derivative audit bound.

The independent audit must confirm the G2/G3-accepted 5e-8 m position budget
without reducing the G1 1e-7 m production contract, the floating/angle guard
or the Simpson panel accounting.  These tests exercise the exact interval
bound, the budget arithmetic and the frozen scene configurations.
"""
import importlib.util
import math
import unittest

from fwis_v3_optimizer.continuous_geometry import ControlKnot
from fwis_v3_optimizer.formation_continuous_a import (
    audit,
    bounds,
    integration_bounds,
)
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    GeometryAuditConfig,
)
from fwis_v3_optimizer.formation_continuous_a.geometry import (
    C2CurvatureProfile,
)

from virtual_a_g4_support import (
    REFERENCE_GEOMETRY_AUDIT,
    ROOT,
    scene_inputs,
)

SCENE_IDS = ("S1", "S1_mirror", "S2", "S2_mirror", "S3", "S3_mirror")
SAMPLES_PER_PANEL = 257
V2_METHOD = "independent_composite_simpson_v2_exact_panel"


def _load_scene_tool():
    path = ROOT / "tools" / "virtual_a_scenes.py"
    spec = importlib.util.spec_from_file_location(
        "virtual_a_scenes_under_test", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SCENES = _load_scene_tool()


def _independent_pieces(profile):
    pieces, _stations, _values, _derivatives = audit._independent_pieces(
        profile._knots)
    return pieces


def _actual_m4(piece, q):
    k = piece.value(q)
    curvature = piece.slope(q)
    second = piece.second(q)
    third = piece.third()
    real = k ** 4 - 4.0 * k * second - 3.0 * curvature * curvature
    imaginary = third - 6.0 * k * k * curvature
    return math.hypot(real, imaginary)


class G4ExactPanelBoundTests(unittest.TestCase):
    def test_s2_panel_bounds_enclose_dense_actual_fourth_derivative(self):
        profile, _independent, name = SCENES.s2_profile()
        self.assertEqual(name, "clamped_cubic_double_lane_change")
        self.assertAlmostEqual(profile.length_m, 12.0)
        pieces = _independent_pieces(profile)
        self.assertEqual(len(pieces), 8)
        for piece in pieces:
            for subintervals in (2, 8, 32, 76):
                values = integration_bounds.panel_m4_bounds(
                    piece.c0, piece.c1, piece.c2, piece.c3, piece.h,
                    subintervals)
                self.assertEqual(len(values), subintervals // 2)
                for index, bound in enumerate(values):
                    self.assertGreaterEqual(bound, 0.0)
                    q0 = 2.0 * index / subintervals
                    q1 = 2.0 * (index + 1) / subintervals
                    actual = max(
                        _actual_m4(
                            piece, q0 + (q1 - q0) * sample
                            / float(SAMPLES_PER_PANEL - 1))
                        for sample in range(SAMPLES_PER_PANEL))
                    self.assertGreaterEqual(
                        bound, actual,
                        "piece s0={} sub={} panel={} bound={} actual={}"
                        .format(piece.s0, subintervals, index, bound, actual))

    def test_structured_single_cubic_panel_bounds_enclose_samples(self):
        profile = C2CurvatureProfile.from_knots([
            ControlKnot(0.0, 0.0, 0.0),
            ControlKnot(1.5, 0.3, 0.2),
        ])
        pieces = _independent_pieces(profile)
        self.assertEqual(len(pieces), 1)
        piece = pieces[0]
        self.assertNotEqual(piece.c3, 0.0)
        for subintervals in (2, 8, 32):
            values = integration_bounds.panel_m4_bounds(
                piece.c0, piece.c1, piece.c2, piece.c3, piece.h,
                subintervals)
            for index, bound in enumerate(values):
                q0 = 2.0 * index / subintervals
                q1 = 2.0 * (index + 1) / subintervals
                actual = max(
                    _actual_m4(
                        piece, q0 + (q1 - q0) * sample
                        / float(SAMPLES_PER_PANEL - 1))
                    for sample in range(SAMPLES_PER_PANEL))
                self.assertGreaterEqual(bound, actual)

    def test_straight_profile_has_zero_panel_m4(self):
        profile = C2CurvatureProfile.from_knots([
            ControlKnot(0.0, 0.0, 0.0),
            ControlKnot(1.5, 0.0, 0.0),
        ])
        pieces = _independent_pieces(profile)
        self.assertEqual(len(pieces), 1)
        piece = pieces[0]
        values = integration_bounds.panel_m4_bounds(
            piece.c0, piece.c1, piece.c2, piece.c3, piece.h, 8)
        # Coefficient widening keeps a zero polynomial at roundoff, not exact
        # zero; the straight-path remainder stays far below any G1 budget.
        for value in values:
            self.assertGreaterEqual(value, 0.0)
            self.assertLess(value, 1e-12)
        budget = integration_bounds.position_budget(
            [(piece.h, 8, values)])
        self.assertGreaterEqual(budget["per_coordinate_m"], 0.0)
        self.assertLess(budget["euclidean_m"], 1e-18)

    def test_s2_audit_confirms_5e8_with_exact_panel_arithmetic(self):
        profile, _independent, _name = SCENES.s2_profile()
        pieces = _independent_pieces(profile)
        config = GeometryAuditConfig(audit_position_budget_m=5e-8)
        plan = audit._integration_plan(pieces, 0.0, config)
        budget = plan["budget"]
        self.assertEqual(budget["method"], V2_METHOD)
        self.assertEqual(budget["m4_method"],
                         "exact_interval_fourth_derivative_per_panel")
        self.assertEqual(budget["fixed_threshold_m"], 5e-8)
        self.assertLessEqual(budget["total_budget_m"], 5e-8)
        self.assertEqual(budget["total_subintervals"],
                         sum(plan["subintervals"]))
        self.assertEqual(budget["full_simpson_panels"],
                         sum(count // 2 for count in plan["subintervals"]))
        self.assertEqual(budget["local_simpson_panels"], len(pieces))
        for segment, subintervals in zip(budget["segments"],
                                         plan["subintervals"]):
            panels = segment["panel_m4_upper_m"]
            self.assertEqual(len(panels), subintervals // 2)
            self.assertEqual(segment["maximum_panel_m4_upper_m"],
                             max(panels))
            self.assertGreaterEqual(
                segment["coarse_triangle_m4_upper_m"],
                segment["maximum_panel_m4_upper_m"])
            width = 2.0 * segment["h"] / subintervals
            direct_full = sum(
                width ** 5 * m4 / 2880.0 for m4 in panels)
            direct_local = width ** 5 * max(panels) / 2880.0
            self.assertGreaterEqual(segment["full_remainder_m"],
                                    direct_full)
            self.assertGreaterEqual(segment["local_remainder_m"],
                                    direct_local)
            self.assertGreaterEqual(segment["remainder_m"],
                                    direct_full + direct_local)
        self.assertGreaterEqual(
            budget["total_budget_m"],
            budget["euclidean_remainder_m"]
            + budget["roundoff_and_angle_guard_m"])

    def test_infeasible_threshold_fails_closed(self):
        profile, _independent, _name = SCENES.s2_profile()
        pieces = _independent_pieces(profile)
        config = GeometryAuditConfig(audit_position_budget_m=1e-12)
        with self.assertRaises(audit._AuditFailure) as raised:
            audit._integration_plan(pieces, 0.0, config)
        self.assertEqual(raised.exception.reason, "BOUND_UNCONFIRMED")
        constraint = str(raised.exception.location.get("constraint", ""))
        self.assertTrue(constraint.startswith("AUDIT_INTEGRATION"),
                        constraint)

    def test_frozen_scene_inputs_and_defaults_bind_5e8_audit(self):
        for name in SCENE_IDS:
            _grid, _metadata, payload = scene_inputs(name)
            budget = payload["timing_config"]["geometry_audit_config"][
                "audit_position_budget_m"]
            self.assertEqual(budget, 5e-8, name)
            self.assertLessEqual(budget, 5e-8, name)
        self.assertEqual(REFERENCE_GEOMETRY_AUDIT.audit_position_budget_m,
                         5e-8)
        self.assertEqual(SCENES.REFERENCE_GEOMETRY_AUDIT
                         .audit_position_budget_m, 5e-8)
        self.assertEqual(GeometryAuditConfig().audit_position_budget_m,
                         5e-8)
        self.assertEqual(GeometryAuditConfig().integration_budget_m, 1e-7)
        self.assertEqual(bounds.POSITION_BUDGET_M, 1e-7)
        self.assertEqual(bounds.ANGLE_BUDGET_RAD, 1e-9)


if __name__ == "__main__":
    unittest.main()