"""S7 fixed-mode-structure joint continuous optimization.

The optimizer keeps the S6 block sequence and every vehicle's A/O/S
mode-direction word fixed.  It optimizes only low-dimensional, mode-consistent
block parameters:

* a translation block has a formation motion angle and a positive length;
* a rotation block has a signed formation yaw increment about its explicit
  formation-frame pivot.

Vehicle centers are never free variables.  For every block they remain
``p_i = p_F + R(psi_F) r_i`` and body headings are propagated from the fixed
S6 mode word.  The reconstruction calls the S5 enumerator and audit for each
block, then the S6 plan audit.  A solver iterate is therefore a candidate only:
it is discarded unless both independent audits accept it.  This module never
publishes a trajectory and never returns ``FEASIBLE``.
"""
from dataclasses import dataclass, replace
import math
import time
from typing import Optional, Tuple

from .formation import FormationPose, FormationValidationError
from .formation_geometry import (
    GEOMETRY_VALID,
    KINEMATIC_VALID,
    FormationBlock,
    FormationContinuousPlan,
    FormationPathControl,
    audit_continuous_plan,
)
from .formation_kinematics import (
    ACKERMANN,
    OBLIQUE,
    SPIN,
    ROTATION,
    TRANSLATION,
    FormationKinematicBlock,
    KinematicLimits,
    audit_mode_assignment,
    enumerate_mode_assignments,
    wrap_angle,
)

OPTIMIZED = "OPTIMIZED"
SEED_RETAINED = "SEED_RETAINED"
REJECTED = "REJECTED"
CASADI_UNAVAILABLE = "CASADI_UNAVAILABLE"
NLP_INFEASIBLE = "FORMATION_NLP_INFEASIBLE"
NLP_RESIDUAL_EXCEEDED = "NLP_HARD_RESIDUAL_EXCEEDED"
INDEPENDENT_AUDIT_FAILED = "INDEPENDENT_AUDIT_FAILED"
NO_IMPROVEMENT = "NO_IMPROVEMENT"
UNSUPPORTED_STRUCTURE = "UNSUPPORTED_FIXED_MODE_STRUCTURE"


def _finite(value, context, positive=False, nonnegative=False):
    if isinstance(value, bool) or isinstance(value, (str, bytes)):
        raise FormationValidationError(
            "NONFINITE_VALUE context={} value={!r}".format(context, value))
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise FormationValidationError(
            "NONFINITE_VALUE context={} value={!r}".format(context, value)) from exc
    if not math.isfinite(result):
        raise FormationValidationError(
            "NONFINITE_VALUE context={} value={!r}".format(context, value))
    if positive and result <= 0.0:
        raise FormationValidationError(
            "NONPOSITIVE_VALUE context={}".format(context))
    if nonnegative and result < 0.0:
        raise FormationValidationError(
            "NEGATIVE_VALUE context={}".format(context))
    return result


def _positive_integer(value, context):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise FormationValidationError(
            "INVALID_POSITIVE_INTEGER context={} value={!r}".format(
                context, value))
    return value


def _branch(value, target):
    """Return the 2*pi branch that makes ``value-target`` continuous."""
    return 2.0 * math.pi * round((value - target) / (2.0 * math.pi))


@dataclass(frozen=True)
class FormationOptimizerConfig:
    """Small fixed-structure NLP budget and deformation bounds."""

    time_limit_s: float = 1.0
    max_iterations: int = 120
    hard_residual_tolerance: float = 1e-6
    min_block_length_m: float = 0.05
    translation_angle_bound_rad: float = math.pi / 2.0
    rotation_angle_bound_rad: float = math.pi / 3.0
    length_lower_factor: float = 0.25
    length_upper_factor: float = 4.0
    objective_length_weight: float = 1.0
    objective_adjustment_weight: float = 0.10
    allow_seed_fallback: bool = True
    require_improvement: bool = False
    max_schedules_per_block: int = 64

    def __post_init__(self):
        object.__setattr__(self, "time_limit_s", _finite(
            self.time_limit_s, "time_limit_s", positive=True))
        _positive_integer(self.max_iterations, "max_iterations")
        object.__setattr__(self, "hard_residual_tolerance", _finite(
            self.hard_residual_tolerance, "hard_residual_tolerance",
            positive=True))
        object.__setattr__(self, "min_block_length_m", _finite(
            self.min_block_length_m, "min_block_length_m", positive=True))
        for name in (
                "translation_angle_bound_rad", "rotation_angle_bound_rad"):
            value = _finite(getattr(self, name), name, nonnegative=True)
            if value > math.pi:
                raise FormationValidationError(
                    "INVALID_OPTIMIZER_BOUND {} must be <= pi".format(name))
            object.__setattr__(self, name, value)
        low = _finite(self.length_lower_factor, "length_lower_factor",
                      positive=True)
        high = _finite(self.length_upper_factor, "length_upper_factor",
                       positive=True)
        if low >= high:
            raise FormationValidationError(
                "INVALID_LENGTH_FACTORS lower must be below upper")
        object.__setattr__(self, "length_lower_factor", low)
        object.__setattr__(self, "length_upper_factor", high)
        for name in ("objective_length_weight",
                     "objective_adjustment_weight"):
            object.__setattr__(self, name, _finite(
                getattr(self, name), name, nonnegative=True))
        for name in ("allow_seed_fallback", "require_improvement"):
            if not isinstance(getattr(self, name), bool):
                raise FormationValidationError(
                    "INVALID_OPTIMIZER_FLAG {} expected bool".format(name))
        _positive_integer(self.max_schedules_per_block,
                          "max_schedules_per_block")


@dataclass(frozen=True)
class _BlockVariable:
    geometry_type: str
    block_index: int
    alpha_index: Optional[int] = None
    length_index: Optional[int] = None
    dpsi_index: Optional[int] = None
    seed_alpha: float = 0.0
    seed_length: float = 0.0
    seed_dpsi: float = 0.0


@dataclass(frozen=True)
class FormationOptimizationReport:
    """Numerical solver and independent-audit evidence for one seed plan."""

    attempted: bool
    solver_name: str
    solver_status: str
    converged: bool
    iterations: int
    variable_count: int
    elapsed_s: float
    hard_residual: float
    hard_residuals: Tuple[Tuple[str, float], ...]
    initial_objective: Optional[float]
    final_objective: Optional[float]
    improvement: Optional[float]
    optimized_candidate_accepted: bool
    seed_retained: bool
    audit_reason: str
    audit_detail: str
    note: str = ""
    model: str = "fixed_mode_block_boundary_nlp"

    def as_dict(self):
        return {
            "schema": "formation_optimization_report/v1",
            "attempted": self.attempted,
            "solver_name": self.solver_name,
            "solver_status": self.solver_status,
            "converged": self.converged,
            "iterations": self.iterations,
            "variable_count": self.variable_count,
            "elapsed_s": self.elapsed_s,
            "hard_residual": self.hard_residual,
            "hard_residuals": dict(self.hard_residuals),
            "initial_objective": self.initial_objective,
            "final_objective": self.final_objective,
            "improvement": self.improvement,
            "optimized_candidate_accepted": (
                self.optimized_candidate_accepted),
            "seed_retained": self.seed_retained,
            "audit_reason": self.audit_reason,
            "audit_detail": self.audit_detail,
            "note": self.note,
            "model": self.model,
            "centers_eliminated": True,
            "mode_structure_fixed": True,
            "time_parameterization": False,
            "collision_certificate": False,
            "final_swept_audit_required": True,
        }


@dataclass(frozen=True)
class FormationOptimizationResult:
    status: str
    reason: str
    detail: str
    plan: Optional[FormationContinuousPlan]
    report: FormationOptimizationReport
    certification: str = "NONE"

    @property
    def accepted(self):
        return self.plan is not None and self.certification == KINEMATIC_VALID

    def as_status(self):
        return {
            "schema": "formation_optimization_result/v1",
            "status": self.status,
            "reason": self.reason,
            "detail": self.detail,
            "certification": self.certification,
            "plan": None if self.plan is None else self.plan.as_status(),
            "report": self.report.as_dict(),
            "executable": False,
            "trajectory_published": False,
            "independent_timing_audit": False,
            "feasible": False,
        }


def _block_parameters(plan):
    parameters = []
    for index, block in enumerate(plan.blocks):
        control = block.control
        if block.geometry_type == TRANSLATION:
            dx = control.end_qf.x - control.start_qf.x
            dy = control.end_qf.y - control.start_qf.y
            length = math.hypot(dx, dy)
            alpha = math.atan2(dy, dx)
            if length <= 0.0:
                raise FormationValidationError(
                    "ZERO_TRANSLATION_BLOCK index={}".format(index))
            parameters.append(_BlockVariable(
                TRANSLATION, index, seed_alpha=alpha,
                seed_length=length))
        elif block.geometry_type == ROTATION:
            dpsi = control.end_qf.psi - control.start_qf.psi
            if abs(dpsi) <= 0.0:
                raise FormationValidationError(
                    "ZERO_ROTATION_BLOCK index={}".format(index))
            parameters.append(_BlockVariable(
                ROTATION, index, seed_dpsi=dpsi,
                seed_length=block.parameter_length))
        else:
            raise FormationValidationError(
                "UNSUPPORTED_BLOCK_GEOMETRY index={}".format(index))
    return tuple(parameters)


def _check_seed_structure(plan, parameters, limits):
    """Ensure the fixed-mode S6 plan is in the optimizer's supported class."""
    if not plan.blocks:
        raise FormationValidationError("EMPTY_FORMATION_PLAN")
    offsets = {offset.vehicle_id: offset for offset in plan.offsets}
    current_headings = dict(plan.start_heading_map)
    # Reconstruct seed block headers and verify the heading recurrence that
    # the optimizer preserves exactly.
    for index, (block, parameter) in enumerate(zip(plan.blocks, parameters)):
        for segment in block.vehicle_mode_segments:
            vehicle_id = segment.vehicle_id
            if vehicle_id not in offsets:
                raise FormationValidationError(
                    "UNKNOWN_VEHICLE_ID index={} vehicle_id={}".format(
                        index, vehicle_id))
            if block.geometry_type == TRANSLATION:
                if segment.mode == SPIN:
                    raise FormationValidationError(
                        "SPIN_TRANSLATION_UNSUPPORTED index={} vehicle={}".format(
                            index, vehicle_id))
                if segment.mode not in (ACKERMANN, OBLIQUE):
                    raise FormationValidationError(
                        "INVALID_MODE index={} vehicle={}".format(
                            index, vehicle_id))
                if abs(wrap_angle(
                        segment.theta_start - current_headings[vehicle_id])) > (
                            limits.heading_tolerance_rad * 10.0):
                    raise FormationValidationError(
                        "SEED_HEADING_RECURRENCE_MISMATCH index={} vehicle={}".format(
                            index, vehicle_id))
                if abs(wrap_angle(
                        segment.theta_end - segment.theta_start)) > (
                            limits.heading_tolerance_rad * 10.0):
                    raise FormationValidationError(
                        "TRANSLATION_HEADING_CHANGE_UNSUPPORTED index={} vehicle={}".format(
                            index, vehicle_id))
            else:
                if segment.mode == OBLIQUE:
                    raise FormationValidationError(
                        "OBLIQUE_ROTATION_UNSUPPORTED index={} vehicle={}".format(
                            index, vehicle_id))
                if segment.mode not in (ACKERMANN, SPIN):
                    raise FormationValidationError(
                        "INVALID_MODE index={} vehicle={}".format(
                            index, vehicle_id))
                expected = current_headings[vehicle_id] + (
                    parameter.seed_dpsi)
                if abs(wrap_angle(segment.theta_start - current_headings[
                        vehicle_id])) > limits.heading_tolerance_rad * 10.0:
                    raise FormationValidationError(
                        "SEED_HEADING_RECURRENCE_MISMATCH index={} vehicle={}".format(
                            index, vehicle_id))
                if abs(wrap_angle(segment.theta_end - expected)) > (
                        limits.heading_tolerance_rad * 10.0):
                    raise FormationValidationError(
                        "ROTATION_HEADING_RECURRENCE_MISMATCH index={} vehicle={}".format(
                            index, vehicle_id))
        if block.geometry_type == ROTATION:
            for vehicle_id in current_headings:
                current_headings[vehicle_id] += parameter.seed_dpsi
    return current_headings


def _build_nlp(plan, parameters, limits, config, seed_final_headings):
    """Build the fixed-structure NLP and return its variable values."""
    import casadi as ca

    opt = ca.Opti()
    alpha = {}
    length = {}
    dpsi = {}
    x0 = []
    bounds = []
    qf_x = plan.start.x
    qf_y = plan.start.y
    qf_psi = plan.start.psi
    seed_psi = plan.start.psi
    headings = {vehicle_id: heading for vehicle_id, heading in
                plan.vehicle_start_headings}
    seed_headings = dict(headings)
    constraints = []

    def add_equality(name, expression):
        checked = ca.MX(expression)
        if checked.is_constant():
            value = float(ca.evalf(checked))
            if abs(value) > config.hard_residual_tolerance:
                raise FormationValidationError(
                    "NLP_CONSTANT_CONSTRAINT_VIOLATION {} value={:.12g}".format(
                        name, value))
            return
        constraints.append(name)
        opt.subject_to(checked == 0.0)

    def add_nonnegative(name, expression):
        checked = ca.MX(expression)
        if checked.is_constant():
            value = float(ca.evalf(checked))
            if value < -config.hard_residual_tolerance:
                raise FormationValidationError(
                    "NLP_CONSTANT_CONSTRAINT_VIOLATION {} value={:.12g}".format(
                        name, value))
            return
        constraints.append(name)
        opt.subject_to(checked >= 0.0)

    for block, parameter in zip(plan.blocks, parameters):
        index = parameter.block_index
        if block.geometry_type == TRANSLATION:
            seed_alpha = parameter.seed_alpha
            ackermann = [
                segment for segment in block.vehicle_mode_segments
                if segment.mode == ACKERMANN]
            if ackermann:
                # A fixes alpha exactly.  Keeping alpha as a free variable
                # would duplicate this equality once per vehicle and make the
                # NLP overconstrained, so alpha becomes an expression of the
                # prior rotation variables (a float for a pure translation
                # plan).  Every A segment must agree at the seed.
                expressions = []
                seed_values = []
                for segment in ackermann:
                    offset = 0.0 if segment.direction > 0 else math.pi
                    branch = _branch(
                        seed_alpha, segment.theta_start + offset)
                    expressions.append(
                        headings[segment.vehicle_id] + offset + branch)
                    seed_values.append(
                        seed_headings[segment.vehicle_id] + offset + branch)
                for value in seed_values[1:]:
                    if abs(wrap_angle(value - seed_values[0])) > (
                            config.hard_residual_tolerance):
                        raise FormationValidationError(
                            "INCONSISTENT_A_TRANSLATION_ANGLES block={} "
                            "values={!r}".format(index, tuple(seed_values)))
                a = expressions[0]
                alpha[index] = a
                add_nonnegative(
                    "alpha_lower_bound_block_{}".format(index),
                    a - (seed_alpha - config.translation_angle_bound_rad))
                add_nonnegative(
                    "alpha_upper_bound_block_{}".format(index),
                    (seed_alpha + config.translation_angle_bound_rad) - a)
            else:
                a = opt.variable(1)
                opt.set_initial(a, seed_alpha)
                opt.subject_to(opt.bounded(
                    seed_alpha - config.translation_angle_bound_rad,
                    a, seed_alpha + config.translation_angle_bound_rad))
                alpha[index] = a
                x0.append(seed_alpha)
                bounds.append(
                    (seed_alpha - config.translation_angle_bound_rad,
                     seed_alpha + config.translation_angle_bound_rad))
            lower = max(config.min_block_length_m,
                        parameter.seed_length * config.length_lower_factor)
            upper = max(lower + 1e-9,
                        parameter.seed_length * config.length_upper_factor)
            l = opt.variable(1)
            opt.set_initial(l, parameter.seed_length)
            opt.subject_to(opt.bounded(lower, l, upper))
            length[index] = l
            x0.append(parameter.seed_length)
            bounds.append((lower, upper))
            qf_x_next = qf_x + l * ca.cos(a)
            qf_y_next = qf_y + l * ca.sin(a)
            qf_psi_next = qf_psi
            phi_limit = limits.oblique_angle_max
            cosine_limit = math.cos(phi_limit) - 1e-10
            for segment in block.vehicle_mode_segments:
                if segment.mode != OBLIQUE:
                    continue
                vehicle_id = segment.vehicle_id
                h = headings[vehicle_id]
                name = "O_translation_angle_block_{}_vehicle_{}".format(
                    index, vehicle_id)
                if segment.direction > 0:
                    # cos(alpha - h) >= cos(phi_max)
                    add_nonnegative(name, ca.cos(a - h) - cosine_limit)
                else:
                    # cos(alpha - h + pi) >= cos(phi_max)
                    add_nonnegative(name, -ca.cos(a - h) - cosine_limit)
            qf_x, qf_y, qf_psi = (
                qf_x_next, qf_y_next, qf_psi_next)
            continue

        delta = opt.variable(1)
        seed_dpsi = parameter.seed_dpsi
        opt.set_initial(delta, seed_dpsi)
        if seed_dpsi > 0.0:
            delta_lower = max(
                1e-6, seed_dpsi - config.rotation_angle_bound_rad)
            delta_upper = seed_dpsi + config.rotation_angle_bound_rad
        else:
            delta_lower = seed_dpsi - config.rotation_angle_bound_rad
            delta_upper = min(
                -1e-6, seed_dpsi + config.rotation_angle_bound_rad)
        opt.subject_to(opt.bounded(delta_lower, delta, delta_upper))
        dpsi[index] = delta
        x0.append(seed_dpsi)
        bounds.append((delta_lower, delta_upper))
        pivot_x = block.control.pivot_x
        pivot_y = block.control.pivot_y
        cosine, sine = ca.cos(qf_psi), ca.sin(qf_psi)
        pivot_world_x = qf_x + cosine * pivot_x - sine * pivot_y
        pivot_world_y = qf_y + sine * pivot_x + cosine * pivot_y
        psi_next = qf_psi + delta
        cn, sn = ca.cos(psi_next), ca.sin(psi_next)
        qf_x_next = pivot_world_x - cn * pivot_x + sn * pivot_y
        qf_y_next = pivot_world_y - sn * pivot_x - cn * pivot_y
        tangent_added = False
        for segment in block.vehicle_mode_segments:
            vehicle_id = segment.vehicle_id
            offset = next(
                item for item in plan.offsets
                if item.vehicle_id == vehicle_id)
            rx = offset.x - pivot_x
            ry = offset.y - pivot_y
            radius = math.hypot(rx, ry)
            if segment.mode == SPIN:
                if radius > limits.pivot_tolerance_m:
                    raise FormationValidationError(
                        "NON_PIVOT_SPIN_UNSUPPORTED block={} vehicle={}".format(
                            index, vehicle_id))
                if segment.direction != (1 if parameter.seed_dpsi >= 0.0
                                         else -1):
                    raise FormationValidationError(
                        "SPIN_DIRECTION_MISMATCH block={} vehicle={}".format(
                            index, vehicle_id))
                headings[vehicle_id] = headings[vehicle_id] + delta
                continue
            if radius <= limits.pivot_tolerance_m:
                raise FormationValidationError(
                    "PIVOT_ACKERMANN_UNSUPPORTED block={} vehicle={}".format(
                        index, vehicle_id))
            curvature = (1.0 if parameter.seed_dpsi >= 0.0 else -1.0) / radius
            if abs(curvature) > 1.0 / limits.r_min_ackermann + (
                    limits.heading_tolerance_rad):
                raise FormationValidationError(
                    "CURVATURE_LIMIT block={} vehicle={}".format(
                        index, vehicle_id))
            tangent_base = math.atan2(rx, -ry)
            offset_angle = 0.0 if segment.direction > 0 else math.pi
            branch = _branch(
                segment.theta_start,
                tangent_base + seed_psi + offset_angle)
            # Every vehicle's heading follows the same dpsi sums, so all A
            # tangent equalities in this block are the same constraint up to
            # a seed-verified constant.  Add it once to keep the equality
            # system square.
            seed_residual = (
                seed_headings[vehicle_id] - tangent_base - seed_psi -
                offset_angle - branch)
            if abs(wrap_angle(seed_residual)) > (
                    config.hard_residual_tolerance):
                raise FormationValidationError(
                    "A_ROTATION_TANGENT_SEED_MISMATCH block={} vehicle={} "
                    "residual={:.12g}".format(
                        index, vehicle_id, seed_residual))
            if not tangent_added:
                add_equality(
                    "A_rotation_tangent_block_{}_vehicle_{}".format(
                        index, vehicle_id),
                    headings[vehicle_id] - tangent_base - qf_psi -
                    offset_angle - branch)
                tangent_added = True
            headings[vehicle_id] = headings[vehicle_id] + delta
        for vehicle_id in seed_headings:
            seed_headings[vehicle_id] += parameter.seed_dpsi
        qf_x, qf_y, qf_psi = qf_x_next, qf_y_next, psi_next
        seed_psi += parameter.seed_dpsi

    goal_psi = plan.goal.psi + 2.0 * math.pi * round(
        (seed_psi - plan.goal.psi) / (2.0 * math.pi))
    add_equality("final_x", qf_x - plan.goal.x)
    add_equality("final_y", qf_y - plan.goal.y)
    add_equality("final_psi", qf_psi - goal_psi)
    for vehicle_id, free in plan.goal_heading_free:
        if free:
            continue
        target = plan.goal_heading_map[vehicle_id]
        # Every S5 rotation segment advances theta_i by exactly dpsi_F, so a
        # vehicle's final heading differs from final psi_F by a constant.
        # ``final_psi`` plus this seed-verified relation already fixes every
        # non-free final heading; adding one equality per vehicle would only
        # duplicate that constraint.
        residual = wrap_angle(seed_final_headings[vehicle_id] - target)
        if abs(residual) > config.hard_residual_tolerance:
            raise FormationValidationError(
                "FINAL_HEADING_GOAL_INCONSISTENT vehicle={} "
                "residual={:.12g}".format(vehicle_id, residual))

    path_measure = 0.0
    adjustment = 0.0
    for parameter in parameters:
        if parameter.geometry_type == TRANSLATION:
            path_measure = path_measure + length[parameter.block_index]
            adjustment = adjustment + (
                alpha[parameter.block_index] - parameter.seed_alpha) ** 2
            adjustment = adjustment + (
                (length[parameter.block_index] - parameter.seed_length) /
                max(parameter.seed_length, 1e-9)) ** 2
        else:
            sign = 1.0 if parameter.seed_dpsi >= 0.0 else -1.0
            path_measure = path_measure + sign * dpsi[parameter.block_index]
            adjustment = adjustment + (
                dpsi[parameter.block_index] - parameter.seed_dpsi) ** 2
    objective = (
        config.objective_length_weight * path_measure +
        config.objective_adjustment_weight * adjustment)
    opt.minimize(objective)
    return opt, objective, alpha, length, dpsi, x0, bounds


def _values_from_solution(opt, solution, parameters, alpha, length, dpsi):
    def evaluated(item):
        if isinstance(item, (int, float)):
            return float(item)
        return float(solution.value(item))

    values = {}
    for parameter in parameters:
        index = parameter.block_index
        if parameter.geometry_type == TRANSLATION:
            values[index] = (
                evaluated(alpha[index]),
                evaluated(length[index]),
                0.0)
        else:
            values[index] = (0.0, 0.0, evaluated(dpsi[index]))
    return values


def _hard_residuals(plan, parameters, values, limits):
    """Recompute mode, heading and terminal residuals from solved values."""
    residuals = []
    qf = plan.start
    headings = dict(plan.start_heading_map)
    for block, parameter in zip(plan.blocks, parameters):
        index = parameter.block_index
        if block.geometry_type == TRANSLATION:
            a, l, _ = values[index]
            alpha = a
            qf_next = FormationPose(
                qf.x + l * math.cos(alpha), qf.y + l * math.sin(alpha),
                qf.psi)
            for segment in block.vehicle_mode_segments:
                h = headings[segment.vehicle_id]
                if segment.mode == ACKERMANN:
                    offset = 0.0 if segment.direction > 0 else math.pi
                    residuals.append((
                        "A_translation_alignment_block_{}_vehicle_{}".format(
                            index, segment.vehicle_id),
                        abs(wrap_angle(alpha - h - offset))))
                elif segment.mode == OBLIQUE:
                    offset = 0.0 if segment.direction > 0 else math.pi
                    phi = wrap_angle(alpha - h - offset)
                    residuals.append((
                        "O_translation_angle_block_{}_vehicle_{}".format(
                            index, segment.vehicle_id),
                        max(0.0, abs(phi) - limits.oblique_angle_max)))
                else:
                    residuals.append((
                        "translation_spin_block_{}_vehicle_{}".format(
                            index, segment.vehicle_id), 1e9))
            qf = qf_next
            continue
        delta = values[index][2]
        pivot_x = block.control.pivot_x
        pivot_y = block.control.pivot_y
        cosine, sine = math.cos(qf.psi), math.sin(qf.psi)
        pivot_world_x = qf.x + cosine * pivot_x - sine * pivot_y
        pivot_world_y = qf.y + sine * pivot_x + cosine * pivot_y
        next_psi = qf.psi + delta
        cn, sn = math.cos(next_psi), math.sin(next_psi)
        qf_next = FormationPose(
            pivot_world_x - cn * pivot_x + sn * pivot_y,
            pivot_world_y - sn * pivot_x - cn * pivot_y,
            next_psi)
        for segment in block.vehicle_mode_segments:
            offset = next(
                item for item in plan.offsets
                if item.vehicle_id == segment.vehicle_id)
            rx = offset.x - pivot_x
            ry = offset.y - pivot_y
            radius = math.hypot(rx, ry)
            h = headings[segment.vehicle_id]
            if segment.mode == SPIN:
                residuals.append((
                    "S_pivot_radius_block_{}_vehicle_{}".format(
                        index, segment.vehicle_id),
                    max(0.0, radius - limits.pivot_tolerance_m)))
                headings[segment.vehicle_id] = h + delta
                continue
            if segment.mode == ACKERMANN:
                tangent = math.atan2(rx, -ry) + qf.psi
                turn = 0.0 if segment.direction > 0 else math.pi
                residuals.append((
                    "A_rotation_tangent_block_{}_vehicle_{}".format(
                        index, segment.vehicle_id),
                    abs(wrap_angle(h - tangent - turn))))
                curvature = abs(delta) / max(radius, 1e-9)
                residuals.append((
                    "A_rotation_curvature_block_{}_vehicle_{}".format(
                        index, segment.vehicle_id),
                    max(0.0, curvature - 1.0 / limits.r_min_ackermann)))
                headings[segment.vehicle_id] = h + delta
            else:
                residuals.append((
                    "rotation_oblique_block_{}_vehicle_{}".format(
                        index, segment.vehicle_id), 1e9))
        qf = qf_next
    residuals.append(("final_xy", math.hypot(
        qf.x - plan.goal.x, qf.y - plan.goal.y)))
    residuals.append(("final_psi", abs(wrap_angle(qf.psi - plan.goal.psi))))
    for vehicle_id, free in plan.goal_heading_free:
        if not free:
            residuals.append((
                "final_heading_vehicle_{}".format(vehicle_id),
                abs(wrap_angle(
                    headings[vehicle_id] - plan.goal_heading_map[vehicle_id]))))
    return tuple(residuals)


def _reconstruct_plan(plan, snapshot, parameter_values, limits,
                      config, grid, margin_m, sweep_policy):
    qf = plan.start
    blocks = []
    current_headings = dict(plan.start_heading_map)
    parameters = _block_parameters(plan)
    for index, (block, parameter) in enumerate(zip(plan.blocks, parameters)):
        if block.geometry_type == TRANSLATION:
            alpha, length_value, _ = parameter_values[index]
            end = FormationPose(
                qf.x + length_value * math.cos(alpha),
                qf.y + length_value * math.sin(alpha), qf.psi)
            control = FormationPathControl(
                TRANSLATION, length_value, qf, end)
            kinematic = FormationKinematicBlock(
                TRANSLATION, length_value, qf, end, 0.0, 0.0,
                plan.offsets)
        else:
            delta = parameter_values[index][2]
            pivot_x = block.control.pivot_x
            pivot_y = block.control.pivot_y
            cosine, sine = math.cos(qf.psi), math.sin(qf.psi)
            pivot_world_x = qf.x + cosine * pivot_x - sine * pivot_y
            pivot_world_y = qf.y + sine * pivot_x + cosine * pivot_y
            next_psi = qf.psi + delta
            cn, sn = math.cos(next_psi), math.sin(next_psi)
            end = FormationPose(
                pivot_world_x - cn * pivot_x + sn * pivot_y,
                pivot_world_y - sn * pivot_x - cn * pivot_y, next_psi)
            control = FormationPathControl(
                ROTATION, abs(delta), qf, end, pivot_x, pivot_y)
            kinematic = FormationKinematicBlock(
                ROTATION, abs(delta), qf, end, pivot_x, pivot_y,
                plan.offsets)
        is_last = index == len(plan.blocks) - 1
        assignments = enumerate_mode_assignments(
            kinematic, current_headings, limits=limits,
            allow_reverse=plan.allow_reverse,
            expected_end_headings=(snapshot.goal_headings if is_last else None),
            goal_heading_free=(snapshot.goal_heading_free if is_last else None),
            max_schedules=config.max_schedules_per_block,
            max_combinations=4096)
        if not assignments:
            return None, "MODE_ASSIGNMENT_FAILED", "no mode schedule returned"
        if not assignments[0].success:
            return None, assignments[0].reason, assignments[0].detail
        fixed = None
        for assignment in assignments:
            if not assignment.success:
                continue
            if all(
                    segment.mode == block.segment_map[
                        segment.vehicle_id].mode and
                    segment.direction == block.segment_map[
                        segment.vehicle_id].direction
                    for segment in assignment.segments):
                fixed = assignment
                break
        if fixed is None:
            return None, "FIXED_MODE_SCHEDULE_MISSING", (
                "the solved geometry has no schedule matching the fixed "
                "S6 mode-direction word")
        audited = audit_mode_assignment(
            fixed, limits=limits,
            expected_end_headings=(snapshot.goal_headings if is_last else None),
            goal_heading_free=(snapshot.goal_heading_free if is_last else None),
            allow_reverse=plan.allow_reverse)
        if not audited.success:
            return None, "MODE_AUDIT_FAILED", audited.detail
        blocks.append(FormationBlock(
            kinematic.geometry_type, kinematic.parameter_length,
            (control,), fixed.segments, block_index=index))
        current_headings = dict(fixed.end_headings)
        qf = end
    rebuilt = FormationContinuousPlan(
        plan.plan_id, plan.formation_id, plan.start, plan.goal,
        plan.offsets, plan.vehicle_start_headings,
        plan.vehicle_goal_headings, tuple(blocks),
        allow_reverse=plan.allow_reverse,
        goal_heading_free=plan.goal_heading_free,
        certification=GEOMETRY_VALID,
        dynamic_profile="fixed_structure_nlp",
        source_signature=("s7", plan.plan_id))
    audit = audit_continuous_plan(
        rebuilt, snapshot, limits=limits, grid=grid, margin_m=margin_m,
        sweep_policy=sweep_policy)
    if not audit.success:
        return None, audit.reason, audit.detail
    return replace(rebuilt, certification=KINEMATIC_VALID), "OK", audit.detail


def _seed_retained_result(plan, report, reason, detail):
    certified = replace(
        plan, certification=KINEMATIC_VALID,
        dynamic_profile=plan.dynamic_profile)
    report = replace(
        report, seed_retained=True, optimized_candidate_accepted=False,
        audit_reason=reason, audit_detail=detail)
    return FormationOptimizationResult(
        status=KINEMATIC_VALID, reason=SEED_RETAINED,
        detail="re-audited seed retained; no optimized candidate accepted",
        plan=certified, report=report, certification=KINEMATIC_VALID)


def _rejected_result(report, reason, detail):
    report = replace(
        report, optimized_candidate_accepted=False, seed_retained=False,
        audit_reason=reason, audit_detail=detail)
    return FormationOptimizationResult(
        status=REJECTED, reason=reason, detail=detail,
        plan=None, report=report, certification="NONE")


def optimize_continuous_plan(plan, snapshot, limits=None, grid=None,
                             margin_m=0.05, sweep_policy=None,
                             config=None):
    """Optimize one audited S6 plan and require independent re-audit.

    The function returns at most a ``KINEMATIC_VALID`` geometry candidate.  It
    never writes a timed trajectory and never returns ``FEASIBLE``.  If the
    solver fails, or its iterate fails either independent audit, the original
    seed may be retained only after that seed is itself independently audited
    again (``allow_seed_fallback``).
    """
    if not isinstance(plan, FormationContinuousPlan):
        raise FormationValidationError(
            "INVALID_CONTINUOUS_PLAN expected FormationContinuousPlan")
    config = config or FormationOptimizerConfig()
    if not isinstance(config, FormationOptimizerConfig):
        raise FormationValidationError(
            "INVALID_OPTIMIZER_CONFIG expected FormationOptimizerConfig")
    limits = limits or KinematicLimits()
    if not isinstance(limits, KinematicLimits):
        raise FormationValidationError(
            "INVALID_KINEMATIC_LIMITS expected KinematicLimits")
    begin = time.perf_counter()
    seed_audit = audit_continuous_plan(
        plan, snapshot, limits=limits, grid=grid, margin_m=margin_m,
        sweep_policy=sweep_policy)
    if not seed_audit.success:
        empty = FormationOptimizationReport(
            attempted=False, solver_name="ipopt", solver_status="not_run",
            converged=False, iterations=0, variable_count=0,
            elapsed_s=time.perf_counter() - begin, hard_residual=float("inf"),
            hard_residuals=(), initial_objective=None, final_objective=None,
            improvement=None, optimized_candidate_accepted=False,
            seed_retained=False, audit_reason=seed_audit.reason,
            audit_detail=seed_audit.detail, note="seed audit failed")
        return _rejected_result(empty, seed_audit.reason, seed_audit.detail)

    try:
        parameters = _block_parameters(plan)
        seed_final_headings = _check_seed_structure(plan, parameters, limits)
    except (FormationValidationError, ValueError) as exc:
        empty = FormationOptimizationReport(
            attempted=False, solver_name="ipopt", solver_status="not_run",
            converged=False, iterations=0, variable_count=0,
            elapsed_s=time.perf_counter() - begin, hard_residual=float("inf"),
            hard_residuals=(), initial_objective=None, final_objective=None,
            improvement=None, optimized_candidate_accepted=False,
            seed_retained=False, audit_reason=UNSUPPORTED_STRUCTURE,
            audit_detail=str(exc), note="fixed mode structure unsupported")
        if config.allow_seed_fallback:
            return _seed_retained_result(
                plan, empty, UNSUPPORTED_STRUCTURE, str(exc))
        return _rejected_result(empty, UNSUPPORTED_STRUCTURE, str(exc))

    try:
        opt, objective, alpha, length, dpsi, x0, bounds = (
            _build_nlp(plan, parameters, limits, config,
                       seed_final_headings))
    except ImportError:
        empty = FormationOptimizationReport(
            attempted=False, solver_name="ipopt",
            solver_status="casadi_not_available", converged=False,
            iterations=0, variable_count=0,
            elapsed_s=time.perf_counter() - begin, hard_residual=float("inf"),
            hard_residuals=(), initial_objective=None, final_objective=None,
            improvement=None, optimized_candidate_accepted=False,
            seed_retained=False, audit_reason=CASADI_UNAVAILABLE,
            audit_detail="CasADi is not installed", note="solver unavailable")
        if config.allow_seed_fallback:
            return _seed_retained_result(
                plan, empty, CASADI_UNAVAILABLE, "CasADi is not installed")
        return _rejected_result(empty, CASADI_UNAVAILABLE,
                               "CasADi is not installed")
    except (FormationValidationError, ValueError, RuntimeError) as exc:
        empty = FormationOptimizationReport(
            attempted=False, solver_name="ipopt", solver_status="build_failed",
            converged=False, iterations=0, variable_count=0,
            elapsed_s=time.perf_counter() - begin, hard_residual=float("inf"),
            hard_residuals=(), initial_objective=None, final_objective=None,
            improvement=None, optimized_candidate_accepted=False,
            seed_retained=False, audit_reason=UNSUPPORTED_STRUCTURE,
            audit_detail=str(exc), note="NLP build failed")
        if config.allow_seed_fallback:
            return _seed_retained_result(
                plan, empty, UNSUPPORTED_STRUCTURE, str(exc))
        return _rejected_result(empty, UNSUPPORTED_STRUCTURE, str(exc))

    variable_count = len(x0)
    try:
        opt.solver("ipopt", {"print_time": False, "expand": True}, {
            "print_level": 0,
            "max_iter": config.max_iterations,
            "max_wall_time": config.time_limit_s,
            "tol": 1e-9,
            "constr_viol_tol": 1e-9,
            "dual_inf_tol": 1e-9,
            "hessian_approximation": "limited-memory",
        })
        solution = opt.solve_limited()
        stats = opt.stats()
        solver_status = stats.get("return_status", "unknown")
        converged = bool(stats.get("success", False))
        iterations = int(stats.get("iter_count", 0))
        values = _values_from_solution(
            opt, solution, parameters, alpha, length, dpsi)
        final_objective = float(solution.value(objective))
    except Exception as exc:  # solver failures must remain structured
        empty = FormationOptimizationReport(
            attempted=True, solver_name="ipopt",
            solver_status=getattr(exc, "args", ["solver_failure"])[0]
            if getattr(exc, "args", None) else "solver_failure",
            converged=False, iterations=0, variable_count=variable_count,
            elapsed_s=time.perf_counter() - begin, hard_residual=float("inf"),
            hard_residuals=(), initial_objective=None, final_objective=None,
            improvement=None, optimized_candidate_accepted=False,
            seed_retained=False, audit_reason=NLP_INFEASIBLE,
            audit_detail=str(exc), note="IPOPT raised or did not return")
        if config.allow_seed_fallback:
            return _seed_retained_result(plan, empty, NLP_INFEASIBLE, str(exc))
        return _rejected_result(empty, NLP_INFEASIBLE, str(exc))

    # The seed parameters reproduce the seed objective.  The objective is a
    # proxy only; an equal or worse iterate is not worth reconstructing.
    seed_objective = 0.0
    adjustment_seed = 0.0
    for parameter in parameters:
        if parameter.geometry_type == TRANSLATION:
            seed_objective += parameter.seed_length
        else:
            seed_objective += abs(parameter.seed_dpsi)
    improvement = seed_objective - final_objective
    hard = _hard_residuals(plan, parameters, values, limits)
    hard_residual = max((value for _, value in hard), default=float("inf"))
    report_base = FormationOptimizationReport(
        attempted=True, solver_name="ipopt", solver_status=solver_status,
        converged=converged, iterations=iterations,
        variable_count=variable_count,
        elapsed_s=time.perf_counter() - begin,
        hard_residual=hard_residual, hard_residuals=hard,
        initial_objective=seed_objective, final_objective=final_objective,
        improvement=improvement, optimized_candidate_accepted=False,
        seed_retained=False, audit_reason="unknown", audit_detail="")

    if not converged or hard_residual > config.hard_residual_tolerance:
        reason = (NLP_INFEASIBLE if not converged
                  else NLP_RESIDUAL_EXCEEDED)
        detail = "solver_success={} hard_residual={:.3g} tolerance={:.3g}".format(
            converged, hard_residual, config.hard_residual_tolerance)
        if config.allow_seed_fallback:
            return _seed_retained_result(plan, report_base, reason, detail)
        return _rejected_result(report_base, reason, detail)

    if improvement <= 1e-9 or config.require_improvement and improvement <= 1e-9:
        reason = NO_IMPROVEMENT
        detail = "objective improvement={:.3g} is not material".format(
            improvement)
        if config.allow_seed_fallback:
            return _seed_retained_result(plan, report_base, reason, detail)
        return _rejected_result(report_base, reason, detail)

    try:
        candidate, reason, detail = _reconstruct_plan(
            plan, snapshot, values, limits, config, grid, margin_m,
            sweep_policy)
    except (FormationValidationError, ValueError, TypeError) as exc:
        candidate, reason, detail = None, INDEPENDENT_AUDIT_FAILED, str(exc)
    if candidate is None:
        if config.allow_seed_fallback:
            return _seed_retained_result(plan, report_base, reason, detail)
        return _rejected_result(report_base, reason, detail)
    report = replace(
        report_base, optimized_candidate_accepted=True, seed_retained=False,
        audit_reason="OK", audit_detail=detail,
        elapsed_s=time.perf_counter() - begin)
    return FormationOptimizationResult(
        status=KINEMATIC_VALID, reason=OPTIMIZED,
        detail="fixed-structure NLP candidate passed S5/S6 independent audit",
        plan=candidate, report=report, certification=KINEMATIC_VALID)