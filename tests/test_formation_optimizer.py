"""S7 fixed-mode-structure formation NLP tests.

These tests exercise the ROS-free optimizer library only.  They verify that
the NLP deforms block boundary parameters while every vehicle center remains
derived from ``q_F + R(psi_F) r_i``, that the S5 mode-direction word is held
fixed, and that a solver success is never accepted without a fresh S5/S6
independent audit.  No test may claim ``FEASIBLE`` or executable timing.
"""
from pathlib import Path
import math
from types import SimpleNamespace
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    FormationValidationError,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_geometry import (
    KINEMATIC_VALID,
    audit_continuous_plan,
    build_continuous_plans,
)
from fwis_v3_optimizer.formation_kinematics import (
    ACKERMANN,
    OBLIQUE,
    SPIN,
    TRANSLATION,
    KinematicLimits,
)
from fwis_v3_optimizer.formation_optimizer import (
    INDEPENDENT_AUDIT_FAILED,
    NLP_INFEASIBLE,
    NO_IMPROVEMENT,
    OPTIMIZED,
    REJECTED,
    SEED_RETAINED,
    UNSUPPORTED_STRUCTURE,
    FormationOptimizerConfig,
    _BlockVariable,
    _check_seed_structure,
    optimize_continuous_plan,
)

try:
    import casadi  # noqa: F401
    CASADI_AVAILABLE = True
except ImportError:  # pragma: no cover - environment guard only
    CASADI_AVAILABLE = False


def primitive(geometry_type, dx=0.0, dy=0.0, dpsi=0.0,
              pivot_x=0.0, pivot_y=0.0):
    return SimpleNamespace(
        geometry_type=geometry_type, dx=dx, dy=dy, dpsi=dpsi,
        pivot_x=pivot_x, pivot_y=pivot_y)


def header():
    return SimpleNamespace(
        frame_id="map", stamp=SimpleNamespace(secs=123, nsecs=456))


def snapshot(offsets, start, goal, start_headings, goal_headings,
             free=None, allow_reverse=False, plan_id=17,
             formation_id="s7_test"):
    model = FormationModel(offsets)
    if free is None:
        free = {offset.vehicle_id: False for offset in offsets}
    return SimpleNamespace(
        plan_id=plan_id, formation_id=formation_id,
        start=start, goal=goal, model=model,
        vehicle_shapes={offset.vehicle_id: VehicleShape(0.738, 0.500)
                        for offset in offsets},
        start_headings=dict(start_headings),
        goal_headings=dict(goal_headings),
        goal_heading_free=dict(free),
        allow_reverse=allow_reverse,
        vehicle_ids=model.vehicle_ids)


def seed_for(request, primitives):
    return SimpleNamespace(
        header=header(),
        plan_id=request.plan_id,
        formation_id=request.formation_id,
        formation_start=request.start,
        formation_goal=request.goal,
        primitives=tuple(primitives))


def build_plan(request, primitives):
    build = build_continuous_plans(
        request, seed_for(request, primitives), max_candidates=4)
    if not build.plans:
        raise AssertionError(
            "seed build failed: {} {}".format(build.reason, build.detail))
    plan = build.plans[0]
    audit = audit_continuous_plan(plan, request)
    if not audit.success:
        raise AssertionError(
            "seed audit failed: {} {}".format(audit.reason, audit.detail))
    return plan


def grid_with_box(x0, x1, y0, y1, resolution=0.05,
                  origin_x=-2.0, origin_y=-2.0, cells=200):
    occupied = [[False] * cells for _ in range(cells)]
    ix0 = int(round((x0 - origin_x) / resolution))
    ix1 = int(round((x1 - origin_x) / resolution))
    iy0 = int(round((y0 - origin_y) / resolution))
    iy1 = int(round((y1 - origin_y) / resolution))
    for ix in range(ix0, ix1):
        for iy in range(iy0, iy1):
            occupied[iy][ix] = True
    return Grid(occupied=occupied, resolution=resolution,
                origin_x=origin_x, origin_y=origin_y)


def mode_word(plan):
    return tuple(
        tuple((segment.vehicle_id, segment.mode, segment.direction)
              for segment in block.vehicle_mode_segments)
        for block in plan.blocks)


class FormationOptimizerTests(unittest.TestCase):
    def setUp(self):
        self.offsets = (
            VehicleOffset(0, 0.0, 0.0),
            VehicleOffset(1, 0.0, 1.2),
        )

    def detour_request(self, headings=(0.3, 0.3), allow_reverse=False):
        start = FormationPose(0.0, 0.0, 0.0)
        goal = FormationPose(2.0, 0.0, 0.0)
        return snapshot(
            self.offsets, start, goal,
            {0: headings[0], 1: headings[1]},
            {0: headings[0], 1: headings[1]},
            allow_reverse=allow_reverse)

    def detour_plan(self):
        request = self.detour_request()
        return request, build_plan(request, (
            primitive(0, dx=1.0, dy=1.2),
            primitive(0, dx=1.0, dy=-1.2)))

    @unittest.skipUnless(CASADI_AVAILABLE, "CasADi is not installed")
    def test_nlp_improves_objective_and_returns_kinematic_only(self):
        request, plan = self.detour_plan()
        seed_length = plan.total_parameter_length
        result = optimize_continuous_plan(plan, request)
        report = result.report.as_dict()

        self.assertEqual(result.status, KINEMATIC_VALID)
        self.assertEqual(result.reason, OPTIMIZED)
        self.assertEqual(result.certification, KINEMATIC_VALID)
        self.assertTrue(result.accepted)
        self.assertEqual(report["solver_name"], "ipopt")
        self.assertEqual(report["solver_status"], "Solve_Succeeded")
        self.assertTrue(report["converged"])
        self.assertGreater(report["iterations"], 0)
        self.assertEqual(report["variable_count"], 4)
        self.assertLessEqual(report["hard_residual"], 1e-6)
        self.assertGreater(report["improvement"], 0.5)
        self.assertLess(result.plan.total_parameter_length, seed_length)
        self.assertTrue(report["optimized_candidate_accepted"])
        self.assertFalse(report["seed_retained"])
        self.assertEqual(report["audit_reason"], "OK")
        self.assertTrue(report["centers_eliminated"])
        self.assertTrue(report["mode_structure_fixed"])
        self.assertFalse(report["time_parameterization"])
        self.assertFalse(report["collision_certificate"])

        status = result.as_status()
        for key in ("executable", "trajectory_published",
                    "independent_timing_audit", "feasible"):
            self.assertFalse(status[key])
        self.assertNotIn("FEASIBLE", repr(status))
        self.assertNotIn("FEASIBLE", result.detail)

        audit = audit_continuous_plan(result.plan, request)
        self.assertTrue(audit.success, audit.reason + " " + audit.detail)
        self.assertEqual(result.plan.certification, KINEMATIC_VALID)
        # Every center is still derived from q_F; the plan stores no second
        # center curve and each block keeps the seed mode-direction word.
        self.assertEqual(mode_word(result.plan), mode_word(plan))
        for block in result.plan.blocks:
            centers = request.model.centers(block.qf_at(0.0))
            self.assertEqual(set(centers), set(request.vehicle_ids))

    def test_ackermann_translation_angle_is_eliminated_not_overconstrained(self):
        request = snapshot(
            self.offsets,
            FormationPose(0.0, 0.0, 0.0),
            FormationPose(2.0, 0.0, 0.0),
            {0: 0.0, 1: 0.0}, {0: 0.0, 1: 0.0})
        plan = build_plan(request, (primitive(0, dx=2.0),))
        for block in plan.blocks:
            for segment in block.vehicle_mode_segments:
                self.assertEqual(segment.mode, ACKERMANN)

        if not CASADI_AVAILABLE:
            self.skipTest("CasADi is not installed")
        result = optimize_continuous_plan(plan, request)
        report = result.report.as_dict()
        # Only the length variable remains; the A-fixed angle is an exact
        # expression, so IPOPT does not see duplicated equalities.
        self.assertEqual(report["variable_count"], 1)
        self.assertEqual(report["solver_status"], "Solve_Succeeded")
        self.assertTrue(report["converged"])
        self.assertEqual(report["improvement"], 0.0)
        self.assertEqual(result.status, KINEMATIC_VALID)
        self.assertEqual(result.reason, SEED_RETAINED)
        self.assertTrue(report["seed_retained"])
        self.assertFalse(report["optimized_candidate_accepted"])
        self.assertEqual(report["audit_reason"], NO_IMPROVEMENT)
        self.assertEqual(result.plan.total_parameter_length,
                         plan.total_parameter_length)
        self.assertFalse(result.as_status()["feasible"])

        rejected = optimize_continuous_plan(
            plan, request,
            config=FormationOptimizerConfig(allow_seed_fallback=False))
        self.assertEqual(rejected.status, REJECTED)
        self.assertEqual(rejected.reason, NO_IMPROVEMENT)
        self.assertIsNone(rejected.plan)
        self.assertEqual(rejected.certification, "NONE")

    def test_pure_rotation_structure_is_supported_and_reaudited(self):
        # Vehicle 1 sits on +x, so its forward tangent is +pi/2.
        offsets = (VehicleOffset(0, 0.0, 0.0),
                   VehicleOffset(1, 1.2, 0.0))
        dpsi = 0.5
        request = snapshot(
            offsets,
            FormationPose(0.0, 0.0, 0.0),
            FormationPose(0.0, 0.0, dpsi),
            {0: 0.0, 1: math.pi / 2.0},
            {0: dpsi, 1: math.pi / 2.0 + dpsi})
        plan = build_plan(request, (primitive(1, dpsi=dpsi),))
        block = plan.blocks[0]
        self.assertEqual(block.geometry_type, "rotation")
        modes = {segment.vehicle_id: segment.mode
                 for segment in block.vehicle_mode_segments}
        self.assertEqual(modes, {0: SPIN, 1: ACKERMANN})

        if not CASADI_AVAILABLE:
            self.skipTest("CasADi is not installed")
        result = optimize_continuous_plan(plan, request)
        report = result.report.as_dict()
        self.assertEqual(report["variable_count"], 1)
        self.assertEqual(report["solver_status"], "Solve_Succeeded")
        self.assertEqual(result.status, KINEMATIC_VALID)
        self.assertEqual(result.reason, SEED_RETAINED)
        self.assertEqual(report["audit_reason"], NO_IMPROVEMENT)
        self.assertEqual(result.plan.blocks[0].parameter_length, dpsi)
        self.assertEqual(result.plan.certification, KINEMATIC_VALID)

    def test_rotation_translation_chain_keeps_fixed_mode_word(self):
        dpsi = 0.3
        offsets = (VehicleOffset(0, 0.0, 0.0),
                   VehicleOffset(1, 1.2, 0.0))
        start = FormationPose(0.0, 0.0, 0.0)
        goal = FormationPose(
            2.0 * math.cos(dpsi), 2.0 * math.sin(dpsi), dpsi)
        request = snapshot(
            offsets, start, goal,
            {0: 0.0, 1: math.pi / 2.0},
            {0: dpsi, 1: math.pi / 2.0 + dpsi},
            allow_reverse=True)
        plan = build_plan(request, (
            primitive(1, dpsi=dpsi),
            primitive(0, dx=1.0, dy=0.3),
            primitive(0, dx=1.0, dy=-0.3)))
        word = mode_word(plan)
        directions = {
            (segment.vehicle_id, segment.direction)
            for block in plan.blocks
            for segment in block.vehicle_mode_segments}
        self.assertIn((1, -1), directions)

        if not CASADI_AVAILABLE:
            self.skipTest("CasADi is not installed")
        result = optimize_continuous_plan(plan, request)
        report = result.report.as_dict()
        self.assertEqual(result.status, KINEMATIC_VALID)
        self.assertEqual(result.reason, OPTIMIZED)
        self.assertEqual(report["solver_status"], "Solve_Succeeded")
        self.assertGreater(report["improvement"], 0.0)
        self.assertGreaterEqual(report["variable_count"], 4)
        self.assertEqual(mode_word(result.plan), word)
        self.assertIn((1, -1), {
            (segment.vehicle_id, segment.direction)
            for block in result.plan.blocks
            for segment in block.vehicle_mode_segments})
        audit = audit_continuous_plan(result.plan, request)
        self.assertTrue(audit.success, audit.reason + " " + audit.detail)
        self.assertNotIn("FEASIBLE", repr(result.as_status()))

    def test_post_solve_collision_audit_rejects_solver_candidate(self):
        request, plan = self.detour_plan()
        # The seed detours to y=1.2 and clears this box; the near-straight
        # optimized path crosses it.  The obstacle is outside both endpoints.
        grid = grid_with_box(0.8, 0.9, 0.1, 0.2)
        seed_audit = audit_continuous_plan(
            plan, request, grid=grid, margin_m=0.05)
        self.assertTrue(seed_audit.success,
                        seed_audit.reason + " " + seed_audit.detail)

        if not CASADI_AVAILABLE:
            self.skipTest("CasADi is not installed")
        retained = optimize_continuous_plan(
            plan, request, grid=grid, margin_m=0.05)
        report = retained.report.as_dict()
        self.assertEqual(retained.status, KINEMATIC_VALID)
        self.assertEqual(retained.reason, SEED_RETAINED)
        self.assertTrue(report["seed_retained"])
        self.assertFalse(report["optimized_candidate_accepted"])
        self.assertEqual(report["solver_status"], "Solve_Succeeded")
        self.assertTrue(report["converged"])
        self.assertEqual(report["audit_reason"], "COLLISION_PRECHECK_FAILED")
        self.assertEqual(retained.certification, KINEMATIC_VALID)
        self.assertEqual(retained.plan.plan_id, plan.plan_id)
        self.assertEqual(retained.plan.total_parameter_length,
                         plan.total_parameter_length)
        again = audit_continuous_plan(
            retained.plan, request, grid=grid, margin_m=0.05)
        self.assertTrue(again.success, again.reason + " " + again.detail)

        rejected = optimize_continuous_plan(
            plan, request, grid=grid, margin_m=0.05,
            config=FormationOptimizerConfig(allow_seed_fallback=False))
        rejected_report = rejected.report.as_dict()
        self.assertEqual(rejected.status, REJECTED)
        self.assertEqual(rejected.reason, "COLLISION_PRECHECK_FAILED")
        self.assertIsNone(rejected.plan)
        self.assertEqual(rejected.certification, "NONE")
        self.assertFalse(rejected_report["optimized_candidate_accepted"])
        self.assertFalse(rejected_report["seed_retained"])
        # The solver did succeed; rejecting its iterate is the audit's job.
        self.assertEqual(rejected_report["solver_status"], "Solve_Succeeded")
        self.assertTrue(rejected_report["converged"])
        self.assertNotIn("FEASIBLE", repr(rejected.as_status()))

    def test_seed_audit_failure_never_runs_the_solver(self):
        request, plan = self.detour_plan()
        grid = grid_with_box(0.9, 1.1, 1.1, 1.3)
        seed_audit = audit_continuous_plan(
            plan, request, grid=grid, margin_m=0.05)
        self.assertFalse(seed_audit.success)
        self.assertEqual(seed_audit.reason, "COLLISION_PRECHECK_FAILED")

        if not CASADI_AVAILABLE:
            self.skipTest("CasADi is not installed")
        result = optimize_continuous_plan(
            plan, request, grid=grid, margin_m=0.05)
        report = result.report.as_dict()
        self.assertEqual(result.status, REJECTED)
        self.assertEqual(result.reason, "COLLISION_PRECHECK_FAILED")
        self.assertIsNone(result.plan)
        self.assertFalse(report["attempted"])
        self.assertEqual(report["solver_status"], "not_run")
        self.assertEqual(report["iterations"], 0)
        self.assertFalse(report["optimized_candidate_accepted"])

    def test_unsupported_fixed_mode_structure_fails_closed(self):
        plan = SimpleNamespace(
            blocks=(SimpleNamespace(
                geometry_type=TRANSLATION,
                vehicle_mode_segments=(SimpleNamespace(
                    vehicle_id=0, mode=SPIN, direction=1,
                    theta_start=0.0, theta_end=0.0),)),),
            start_heading_map={0: 0.0},
            offsets=(VehicleOffset(0, 0.0, 0.0),))
        parameters = (_BlockVariable(
            TRANSLATION, 0, seed_alpha=0.0, seed_length=1.0),)
        with self.assertRaises(FormationValidationError) as context:
            _check_seed_structure(plan, parameters, KinematicLimits())
        self.assertIn("SPIN_TRANSLATION_UNSUPPORTED", str(context.exception))

        plan = SimpleNamespace(
            blocks=(SimpleNamespace(
                geometry_type=TRANSLATION,
                vehicle_mode_segments=(SimpleNamespace(
                    vehicle_id=0, mode=OBLIQUE, direction=1,
                    theta_start=0.0, theta_end=0.5),)),),
            start_heading_map={0: 0.0},
            offsets=(VehicleOffset(0, 0.0, 0.0),))
        with self.assertRaises(FormationValidationError) as context:
            _check_seed_structure(plan, parameters, KinematicLimits())
        self.assertIn(
            "TRANSLATION_HEADING_CHANGE_UNSUPPORTED",
            str(context.exception))

    def test_invalid_inputs_and_configs_fail_closed(self):
        request, plan = self.detour_plan()
        with self.assertRaises(FormationValidationError):
            optimize_continuous_plan(object(), request)
        with self.assertRaises(FormationValidationError):
            optimize_continuous_plan(plan, request, config=object())
        with self.assertRaises(FormationValidationError):
            optimize_continuous_plan(plan, request, limits=object())
        with self.assertRaises(FormationValidationError):
            FormationOptimizerConfig(translation_angle_bound_rad=math.pi + 1)
        with self.assertRaises(FormationValidationError):
            FormationOptimizerConfig(length_lower_factor=2.0,
                                     length_upper_factor=1.0)
        with self.assertRaises(FormationValidationError):
            FormationOptimizerConfig(allow_seed_fallback=1)
        with self.assertRaises(FormationValidationError):
            FormationOptimizerConfig(time_limit_s=0.0)
        with self.assertRaises(FormationValidationError):
            FormationOptimizerConfig(max_iterations=0)
        with self.assertRaises(FormationValidationError):
            FormationOptimizerConfig(objective_length_weight=-1.0)

    def test_status_names_are_explicit_and_never_feasible(self):
        # ``FORMATION_NLP_INFEASIBLE`` contains the substring "FEASIBLE" but
        # is a rejection reason; no status value equals the feasibility claim.
        codes = (OPTIMIZED, SEED_RETAINED, REJECTED, INDEPENDENT_AUDIT_FAILED,
                 NLP_INFEASIBLE, UNSUPPORTED_STRUCTURE, NO_IMPROVEMENT,
                 "COLLISION_PRECHECK_FAILED")
        self.assertNotIn("FEASIBLE", codes)
        for code in codes:
            self.assertNotEqual(code, "FEASIBLE")


if __name__ == "__main__":
    unittest.main()