"""Independent FR02-B audit of a V0.2 uniform timed plan.

The audit does not trust FR01 derived fields, the FR02-A projection, or any
S8 timing report as evidence.  It re-derives every edge field from the
primary geometry, re-analyses the common motion duration with the S9/S8
kernel, re-evaluates the analytic state at every phase boundary and at
interior times, re-checks the dynamic candidates and conservative bounds,
compares the projection item by item, and re-runs collision checking over
exact analytic states with a conservative per-segment displacement bound.
Failure never produces a success certificate.
"""
from collections.abc import Mapping as MappingABC
from dataclasses import asdict, dataclass, field
import math
from typing import Mapping, Optional, Tuple

from .core import Action
from .formation import (
    FormationModel,
    FormationPose,
    FormationValidationError,
    VehicleOffset,
)
from .formation_collision import (
    AnalyticSweepSegment,
    FormationCollisionChecker,
    FormationState,
    SweepPolicy,
    VehicleShape,
)
from .formation_geometry import (
    BODY_SPIN,
    ROTATION,
    TRANSLATION,
    FormationBlock,
)
from .formation_kinematics import (
    ACKERMANN,
    OBLIQUE,
    SPIN,
    VehicleModeSegment,
)
from .formation_quality import (
    FEASIBLE,
    FORMATION_RETIMING_INFEASIBLE,
    GEOMETRY_ONLY_NOT_EXECUTABLE,
    INVALID,
    INVALID_REQUEST,
    INVALID_TIMED_TRAJECTORY,
    REJECTED,
    FormationEvaluation,
    FormationEvaluationFailure,
    FormationQualityConfig,
    START_FORMATION_COLLISION,
    GOAL_FORMATION_COLLISION,
    FORMATION_SWEEP_COLLISION,
    _DYNAMIC_LIMITS,
    _EvalError,
    _IndependentEvaluator,
    _TAUS,
    _dynamic_values,
    grid_signature,
)
from .formation_retiming import (
    GEOMETRY_ONLY,
    FormationTimedTrajectory,
    FormationTimingLimits,
    FormationTimingReport,
    _TimingInfeasible,
    _analyze_block,
    _formation_derivatives,
    _motion_limits,
    _vehicle_derivatives,
)
from .mode_switch_timing import mode_switch_event
from .retiming import MotionLimits, wheel_configuration
from .uniform_motion import (
    MODE_A,
    MODE_O,
    MODE_S,
    PRIMITIVE_A_ARC,
    PRIMITIVE_A_STRAIGHT,
    PRIMITIVE_O_TRANSLATION,
    PRIMITIVE_S_SPIN,
    UNIFORM_PRIMITIVES,
    UniformMotionEdge,
)
from .uniform_timing import (
    SetupTiming,
    MotionTiming,
    legacy_mode_word,
    timing_motion_limits,
)
from .uniform_trajectory import (
    CERTIFICATION,
    MODEL_VERSION,
    PROJECTION_SEMANTICS,
    TIME_CONTINUITY_TOL_S,
    UniformSampleProjection,
    UniformTimedPlan,
    UniformTimeState,
    UniformVehicleTimeState,
)

# Audit failure codes that are specific to the V0.2 uniform model.  Other
# failures reuse the existing S9 codes so callers see one vocabulary.
UNIFORM_MODEL_MISMATCH = "UNIFORM_MODEL_MISMATCH"
UNIFORM_PROJECTION_MISMATCH = "UNIFORM_PROJECTION_MISMATCH"
UNIFORM_PHASE_SCHEDULE_INVALID = "UNIFORM_PHASE_SCHEDULE_INVALID"
UNIFORM_SWEEP_BUDGET_EXCEEDED = "UNIFORM_SWEEP_BUDGET_EXCEEDED"

_GEOMETRY_TOL = 1e-7
_TIME_TOL = 1e-9
_ENDPOINT_TOL = 1e-9
_STATE_TOL = 1e-7
_WHEEL_TOL = 1e-9
_MODE_CONTROL_TOL = 1e-9
_SETUP_TOL = 1e-9
_RIGIDITY_TOL = 1e-6
_FIELD_TOL = 1e-9

_PRIMITIVE_MODE = {
    PRIMITIVE_A_STRAIGHT: MODE_A,
    PRIMITIVE_A_ARC: MODE_A,
    PRIMITIVE_O_TRANSLATION: MODE_O,
    PRIMITIVE_S_SPIN: MODE_S,
}
_PRIMITIVE_GEOMETRY = {
    PRIMITIVE_A_STRAIGHT: TRANSLATION,
    PRIMITIVE_A_ARC: ROTATION,
    PRIMITIVE_O_TRANSLATION: TRANSLATION,
    PRIMITIVE_S_SPIN: BODY_SPIN,
}
_MODE_TO_LEGACY = {MODE_A: ACKERMANN, MODE_O: OBLIQUE, MODE_S: SPIN}

# Interior phase times: 16 uniform fractions plus the quintic extrema.  The
# count is deliberately above the eleven required by the task.
_INTERIOR_TAUS = tuple(sorted(set(
    [index / 16.0 for index in range(1, 16)] + [
        0.5 - math.sqrt(3.0) / 6.0,
        0.5,
        0.5 + math.sqrt(3.0) / 6.0,
    ])))
_SETUP_INTERIOR_FRACTIONS = tuple(index / 12.0 for index in range(1, 12))


def _pair_map(pairs):
    return {int(vehicle_id): float(theta) for vehicle_id, theta in pairs}


def _centers(qf, offsets):
    cosine, sine = math.cos(qf.psi), math.sin(qf.psi)
    result = {}
    for offset in offsets:
        result[offset.vehicle_id] = (
            qf.x + cosine * offset.x - sine * offset.y,
            qf.y + sine * offset.x + cosine * offset.y)
    return result


def _pose_record(qf):
    return (float(qf.x), float(qf.y), float(qf.psi))


def _headings_record(pairs):
    return {int(vehicle_id): float(theta)
            for vehicle_id, theta in pairs}


class _OffsetContext:
    """Minimal offset view accepted by the shared geometry derivative kernel."""

    __slots__ = ("offset_x", "offset_y")

    def __init__(self, x, y):
        self.offset_x = float(x)
        self.offset_y = float(y)


def _wheel_positions(motion_limits):
    values = []
    for x in (motion_limits.wheelbase / 2.0,
              -motion_limits.wheelbase / 2.0):
        for y in (motion_limits.track / 2.0, -motion_limits.track / 2.0):
            values.append((float(x), float(y)))
    return tuple(values)


def _uniform_status(code):
    if code in (INVALID_REQUEST, INVALID_TIMED_TRAJECTORY,
                UNIFORM_SWEEP_BUDGET_EXCEEDED):
        return INVALID
    if code == FORMATION_RETIMING_INFEASIBLE:
        return GEOMETRY_ONLY_NOT_EXECUTABLE
    return REJECTED


def _uniform_failure(plan_id, formation_id, code, detail, metrics=None,
                     checks=0, **fields):
    return FormationEvaluation(
        plan_id=plan_id,
        formation_id=formation_id,
        status=_uniform_status(code),
        feasible=False,
        reason=code,
        detail=detail,
        failure=FormationEvaluationFailure(
            code=code, detail=detail, **fields),
        metrics=dict(metrics or {}),
        checks=int(checks))


@dataclass(frozen=True)
class UniformAuditRequest:
    """The request fields a V0.2 timed plan must reproduce exactly."""

    plan_id: int
    formation_id: str
    model: FormationModel
    vehicle_shapes: Mapping[int, VehicleShape]
    start_qf: FormationPose
    start_headings: Mapping[int, float]
    goal_qf: FormationPose
    goal_headings: Mapping[int, float]
    timing_limits: FormationTimingLimits = field(
        default_factory=FormationTimingLimits)
    allow_reverse: bool = False

    def __post_init__(self):
        if isinstance(self.plan_id, bool) or not isinstance(
                self.plan_id, int) or not 0 <= self.plan_id <= 2 ** 32 - 1:
            raise FormationValidationError(
                "INVALID_UNIFORM_REQUEST plan_id")
        if not isinstance(self.formation_id, str) or not self.formation_id:
            raise FormationValidationError(
                "INVALID_UNIFORM_REQUEST formation_id")
        if not isinstance(self.model, FormationModel):
            raise FormationValidationError(
                "INVALID_UNIFORM_REQUEST model")
        if not isinstance(self.start_qf, FormationPose) or not isinstance(
                self.goal_qf, FormationPose):
            raise FormationValidationError(
                "INVALID_UNIFORM_REQUEST start_qf/goal_qf")
        if not isinstance(self.timing_limits, FormationTimingLimits):
            raise FormationValidationError(
                "INVALID_UNIFORM_REQUEST timing_limits")
        if not isinstance(self.allow_reverse, bool):
            raise FormationValidationError(
                "INVALID_UNIFORM_REQUEST allow_reverse")
        vehicle_ids = self.model.vehicle_ids
        if not isinstance(self.vehicle_shapes, MappingABC):
            raise FormationValidationError(
                "INVALID_UNIFORM_REQUEST vehicle_shapes")
        shapes = dict(self.vehicle_shapes)
        if set(shapes) != set(vehicle_ids):
            raise FormationValidationError(
                "INVALID_UNIFORM_REQUEST vehicle_shapes ids")
        for vehicle_id in vehicle_ids:
            if not isinstance(shapes[vehicle_id], VehicleShape):
                raise FormationValidationError(
                    "INVALID_UNIFORM_REQUEST vehicle_shapes")
        start = self._heading_input(
            self.start_headings, vehicle_ids, "start_headings")
        goal = self._heading_input(
            self.goal_headings, vehicle_ids, "goal_headings")
        object.__setattr__(self, "vehicle_shapes", shapes)
        object.__setattr__(self, "start_headings", start)
        object.__setattr__(self, "goal_headings", goal)

    @staticmethod
    def _heading_input(values, vehicle_ids, context):
        if not isinstance(values, MappingABC):
            raise FormationValidationError(
                "INVALID_UNIFORM_REQUEST {}".format(context))
        if set(values) != set(vehicle_ids):
            raise FormationValidationError(
                "INVALID_UNIFORM_REQUEST {} ids".format(context))
        result = []
        for vehicle_id in vehicle_ids:
            value = values[vehicle_id]
            if isinstance(value, bool) or isinstance(value, (str, bytes)):
                raise FormationValidationError(
                    "INVALID_UNIFORM_REQUEST {}[{}]".format(
                        context, vehicle_id))
            theta = float(value)
            if not math.isfinite(theta):
                raise FormationValidationError(
                    "INVALID_UNIFORM_REQUEST {}[{}]".format(
                        context, vehicle_id))
            result.append((vehicle_id, theta))
        return tuple(result)

    @classmethod
    def from_plan(cls, plan, allow_reverse=None):
        if not isinstance(plan, UniformTimedPlan):
            raise FormationValidationError(
                "INVALID_UNIFORM_REQUEST expected UniformTimedPlan")
        if allow_reverse is None:
            allow_reverse = any(
                bool(edge.allow_reverse) for edge in plan.edges)
        if not isinstance(allow_reverse, bool):
            raise FormationValidationError(
                "INVALID_UNIFORM_REQUEST allow_reverse")
        return cls(
            plan_id=plan.plan_id,
            formation_id=plan.formation_id,
            model=plan.model,
            vehicle_shapes=plan.vehicle_shapes,
            start_qf=plan.start_qf,
            start_headings=_headings_record(plan.start_headings),
            goal_qf=plan.goal_qf,
            goal_headings=_headings_record(plan.goal_headings),
            timing_limits=plan.limits,
            allow_reverse=allow_reverse)


class _UniformAuditor(_IndependentEvaluator):
    def __init__(self, request, plan, projection, grid, config):
        super().__init__(request, plan, projection.timed_trajectory, grid,
                         config)
        self.request = request
        self.plan = plan
        self.projection = projection
        self.model = plan.model
        self.vehicle_ids = tuple(plan.model.vehicle_ids)
        self.offset_map = {
            offset.vehicle_id: offset for offset in plan.model.offsets}
        self.shapes = dict(plan.vehicle_shapes)
        self.motion_limits = None
        self.edge_objects = ()
        self.edge_data = []
        self.setup_data = []
        self.motion_data = []
        self._checker = None
        self._projection_times = ()
        self._max_state_error = 0.0
        self._max_projection_error = 0.0
        self._max_rigidity_error = 0.0
        self._max_finite_difference_error = 0.0
        self._max_interior_geometry_error = 0.0
        self._max_wheel_start_error = 0.0
        self._max_wheel_end_error = 0.0
        self._max_normalized_mode_residual = 0.0
        self._max_mode_residual = 0.0
        self._setup_evidence = []
        self._interior_counts = {}
        self._boundary_errors = []
        self._dynamic_evidence = {}
        self._dynamic_peak_ratio = 0.0
        self._dynamic_continuous_certificate = True
        self._dynamic_limitations = []
        self._sweep_evidence = {}
        self._first_collision = None

    # -- entry ---------------------------------------------------------
    def run(self):
        try:
            self._check_grid_and_signature()
            self._check_request_uniform()
            self._check_plan_identity()
            self._check_edges()
            self._check_phase_axis()
            self._check_model_nodes()
            self._check_interior_states()
            self._check_dynamic_peaks()
            self._check_projection_contract()
            self._check_static_collisions()
            self._check_analytic_sweeps()
            return self._uniform_success()
        except _EvalError as exc:
            return _uniform_failure(
                self.plan_id, self.formation_id, exc.code, exc.detail,
                metrics=self.metrics, checks=self.checks, **exc.fields)
        except FormationValidationError as exc:
            return _uniform_failure(
                self.plan_id, self.formation_id, INVALID_TIMED_TRAJECTORY,
                "uniform audit input became invalid: {}".format(exc),
                metrics=self.metrics, checks=self.checks)

    # -- request and identity -----------------------------------------
    def _check_request_uniform(self):
        request = self.request
        if not isinstance(request, UniformAuditRequest):
            self._fail(INVALID_REQUEST, "expected UniformAuditRequest",
                       field="request")
        self.plan_id = request.plan_id
        self.formation_id = request.formation_id
        self.model = request.model
        self.vehicle_ids = tuple(request.model.vehicle_ids)
        self.offset_map = {
            offset.vehicle_id: offset for offset in request.model.offsets}
        self.shapes = dict(request.vehicle_shapes)
        self.motion_limits = _motion_limits(request.timing_limits)
        self._count(len(self.vehicle_ids) * 2 + 10)

    def _require_finite(self, value, context, code=UNIFORM_MODEL_MISMATCH):
        if isinstance(value, bool) or isinstance(value, (str, bytes)):
            self._fail(code, "{}={!r} is not a number".format(
                context, value), field=context)
        try:
            result = float(value)
        except (TypeError, ValueError, OverflowError):
            self._fail(code, "{}={!r} is not a number".format(
                context, value), field=context)
        if not math.isfinite(result):
            self._fail(code, "{}={!r} is not finite".format(
                context, value), field=context)
        return result

    def _compare_number(self, actual, expected, tolerance, context, code,
                        category="model", **fields):
        actual_value = self._require_finite(actual, context + ".actual", code)
        expected_value = self._require_finite(
            expected, context + ".expected", code)
        error = abs(actual_value - expected_value)
        if category == "projection":
            self._max_projection_error = max(
                self._max_projection_error, error)
        else:
            self._max_state_error = max(self._max_state_error, error)
        if error > tolerance:
            self._fail(
                code, "{} actual={:.17g} expected={:.17g} error={:.3g} > "
                "{:.3g}".format(context, actual_value, expected_value,
                                error, tolerance),
                field=context, max_error=error, **fields)
        return error

    def _compare_exact(self, actual, expected, context, code, category="model",
                       **fields):
        if actual != expected:
            self._fail(
                code, "{} actual={!r} expected={!r}".format(
                    context, actual, expected),
                field=context, **fields)
        if category == "projection":
            self._count()

    def _compare_pose(self, actual, expected, context, code,
                      category="model", tolerance=_GEOMETRY_TOL, **fields):
        if not isinstance(actual, FormationPose):
            self._fail(code, "{} is not FormationPose".format(context),
                       field=context, **fields)
        self._compare_number(actual.x, expected.x, tolerance,
                             context + ".x", code, category, **fields)
        self._compare_number(actual.y, expected.y, tolerance,
                             context + ".y", code, category, **fields)
        # psi is unwrapped: compare directly, not modulo 2*pi.
        self._compare_number(actual.psi, expected.psi, tolerance,
                             context + ".psi", code, category, **fields)

    def _compare_heading_map(self, actual, expected, context, code,
                             category="model", tolerance=_GEOMETRY_TOL,
                             **fields):
        if not isinstance(actual, MappingABC):
            self._fail(code, "{} is not a heading mapping".format(context),
                       field=context, **fields)
        if set(actual) != set(expected):
            self._fail(code, "{} ids differ: actual={} expected={}".format(
                context, sorted(actual), sorted(expected)),
                field=context, **fields)
        for vehicle_id in sorted(expected):
            self._compare_number(
                actual[vehicle_id], expected[vehicle_id], tolerance,
                "{}.{}".format(context, vehicle_id), code, category,
                vehicle_id=vehicle_id, **fields)

    def _compare_record(self, actual, expected, context, code,
                        category="projection", tolerance=1e-9, **fields):
        if isinstance(expected, MappingABC):
            if not isinstance(actual, MappingABC):
                self._fail(code, "{} is not a mapping".format(context),
                           field=context, **fields)
            if set(actual) != set(expected):
                self._fail(
                    code, "{} keys differ: missing={} extra={}".format(
                        context, sorted(set(expected) - set(actual)),
                        sorted(set(actual) - set(expected))),
                    field=context, **fields)
            for key in sorted(expected):
                self._compare_record(
                    actual[key], expected[key],
                    "{}.{}".format(context, key), code, category,
                    tolerance, **fields)
            return
        if isinstance(expected, (list, tuple)):
            if not isinstance(actual, (list, tuple)):
                self._fail(code, "{} is not a sequence".format(context),
                           field=context, **fields)
            if len(actual) != len(expected):
                self._fail(
                    code, "{} length {} != {}".format(
                        context, len(actual), len(expected)),
                    field=context, **fields)
            for index, (first, second) in enumerate(zip(actual, expected)):
                self._compare_record(
                    first, second, "{}[{}]".format(context, index), code,
                    category, tolerance, **fields)
            return
        if expected is None or isinstance(expected, (str, bool)):
            self._compare_exact(actual, expected, context, code, category,
                                **fields)
            return
        if isinstance(expected, int) and not isinstance(expected, bool):
            self._compare_exact(actual, expected, context, code, category,
                                **fields)
            return
        self._compare_number(actual, expected, tolerance, context, code,
                             category, **fields)

    def _limits_equal(self, actual, expected, context, code,
                      tolerance=1e-12):
        self._compare_record(
            asdict(actual), asdict(expected), context, code,
            category="model", tolerance=tolerance)

    def _check_plan_identity(self):
        plan = self.plan
        request = self.request
        if not isinstance(plan, UniformTimedPlan):
            self._fail(INVALID_REQUEST,
                       "plan must be UniformTimedPlan", field="plan")
        if plan.plan_id != request.plan_id:
            self._fail(UNIFORM_MODEL_MISMATCH,
                       "plan_id differs from the request", field="plan_id")
        if plan.formation_id != request.formation_id:
            self._fail(UNIFORM_MODEL_MISMATCH,
                       "formation_id differs from the request",
                       field="formation_id")
        if len(plan.model.offsets) != len(request.model.offsets):
            self._fail(UNIFORM_MODEL_MISMATCH,
                       "offset count differs from the request",
                       field="offsets")
        for actual, expected in zip(plan.model.offsets, request.model.offsets):
            if actual.vehicle_id != expected.vehicle_id:
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "offset vehicle_id order differs", field="offsets")
            self._compare_number(actual.x, expected.x, _FIELD_TOL,
                                 "offsets.{}.x".format(actual.vehicle_id),
                                 UNIFORM_MODEL_MISMATCH)
            self._compare_number(actual.y, expected.y, _FIELD_TOL,
                                 "offsets.{}.y".format(actual.vehicle_id),
                                 UNIFORM_MODEL_MISMATCH)
        if set(plan.vehicle_shapes) != set(self.vehicle_ids):
            self._fail(UNIFORM_MODEL_MISMATCH,
                       "plan vehicle_shapes ids differ", field="vehicle_shapes")
        for vehicle_id in self.vehicle_ids:
            actual = plan.vehicle_shapes[vehicle_id]
            expected = request.vehicle_shapes[vehicle_id]
            if not isinstance(actual, VehicleShape):
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "plan shape is not VehicleShape",
                           field="vehicle_shapes", vehicle_id=vehicle_id)
            self._compare_number(actual.length_m, expected.length_m,
                                 _FIELD_TOL,
                                 "vehicle_shapes.{}.length_m".format(
                                     vehicle_id), UNIFORM_MODEL_MISMATCH,
                                 vehicle_id=vehicle_id)
            self._compare_number(actual.width_m, expected.width_m,
                                 _FIELD_TOL,
                                 "vehicle_shapes.{}.width_m".format(
                                     vehicle_id), UNIFORM_MODEL_MISMATCH,
                                 vehicle_id=vehicle_id)
        self._limits_equal(plan.limits, request.timing_limits, "plan.limits",
                           UNIFORM_MODEL_MISMATCH)
        self._compare_record(
            asdict(plan.motion_limits), asdict(self.motion_limits),
            "plan.motion_limits", UNIFORM_MODEL_MISMATCH,
            category="model", tolerance=1e-12)
        self._compare_pose(plan.start_qf, request.start_qf, "plan.start_qf",
                           UNIFORM_MODEL_MISMATCH)
        self._compare_pose(plan.goal_qf, request.goal_qf, "plan.goal_qf",
                           UNIFORM_MODEL_MISMATCH)
        self._compare_heading_map(
            _headings_record(plan.start_headings),
            _headings_record(request.start_headings), "plan.start_headings",
            UNIFORM_MODEL_MISMATCH)
        self._compare_heading_map(
            _headings_record(plan.goal_headings),
            _headings_record(request.goal_headings), "plan.goal_headings",
            UNIFORM_MODEL_MISMATCH)
        duration = self._require_finite(
            plan.duration_s, "plan.duration_s", UNIFORM_MODEL_MISMATCH)
        if duration < 0.0:
            self._fail(UNIFORM_MODEL_MISMATCH,
                       "plan duration must be nonnegative",
                       field="plan.duration_s")
        if not plan.edges:
            if plan.phases:
                self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                           "zero-motion hold must not have phases",
                           field="phases")
            if plan.duration_s != 0.0:
                self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                           "zero-motion hold must have duration 0",
                           field="duration_s")
            self._compare_pose(plan.start_qf, plan.goal_qf,
                               "hold.start_goal_qf", UNIFORM_MODEL_MISMATCH)
            self._compare_heading_map(
                _headings_record(plan.start_headings),
                _headings_record(plan.goal_headings),
                "hold.start_goal_headings", UNIFORM_MODEL_MISMATCH)
        else:
            if not plan.phases:
                self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                           "a moving plan must have phases", field="phases")
            if not plan.duration_s > 0.0:
                self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                           "a moving plan needs a positive duration",
                           field="duration_s")
            self._compare_pose(
                plan.start_qf, plan.edges[0].start_qf,
                "plan.start_qf.first_edge", UNIFORM_MODEL_MISMATCH)
            self._compare_pose(
                plan.goal_qf, plan.edges[-1].end_qf,
                "plan.goal_qf.last_edge", UNIFORM_MODEL_MISMATCH)
            self._compare_heading_map(
                _headings_record(plan.start_headings),
                _headings_record(plan.edges[0].start_headings),
                "plan.start_headings.first_edge", UNIFORM_MODEL_MISMATCH)
            self._compare_heading_map(
                _headings_record(plan.goal_headings),
                _headings_record(plan.edges[-1].end_headings),
                "plan.goal_headings.last_edge", UNIFORM_MODEL_MISMATCH)
        self._count(12)

    # -- edge re-derivation -------------------------------------------
    def _edge_controls(self, edge, primitive):
        """Derive per-vehicle control values and geometric scalars."""
        controls = {}
        speeds = {}
        radii = {}
        sign = 0
        if primitive == PRIMITIVE_A_ARC:
            delta_psi = edge.control.end_qf.psi - edge.control.start_qf.psi
            sign = 1 if delta_psi > 0.0 else -1
            length = edge.control.parameter_length
            self._compare_number(
                abs(delta_psi), length, _FIELD_TOL,
                "edge{}.control.delta_psi_length".format(edge.index
                                                         if hasattr(edge, "index")
                                                         else ""),
                UNIFORM_MODEL_MISMATCH)
            pivot_x = edge.control.pivot_x
            pivot_y = edge.control.pivot_y
            direction_map = _pair_map(edge.directions)
            for offset in self.model.offsets:
                vehicle_id = offset.vehicle_id
                rx = offset.x - pivot_x
                ry = offset.y - pivot_y
                radius = math.hypot(rx, ry)
                if not math.isfinite(radius):
                    self._fail(UNIFORM_MODEL_MISMATCH,
                               "A arc radius is not finite",
                               vehicle_id=vehicle_id)
                if radius <= 1e-12:
                    self._fail(
                        UNIFORM_MODEL_MISMATCH,
                        "A arc pivot coincides with vehicle centre",
                        vehicle_id=vehicle_id)
                if radius < self.motion_limits.r_min_ack - _FIELD_TOL:
                    self._fail(
                        UNIFORM_MODEL_MISMATCH,
                        "A arc radius {:.12g} is below r_min {:.12g}".format(
                            radius, self.motion_limits.r_min_ack),
                        vehicle_id=vehicle_id, constraint="r_min_ackermann")
                sigma = direction_map[vehicle_id]
                controls[vehicle_id] = sign / (sigma * radius)
                speeds[vehicle_id] = float(sigma * radius)
                radii[vehicle_id] = radius
            return controls, speeds, radii, sign
        if primitive == PRIMITIVE_A_STRAIGHT:
            direction_map = _pair_map(edge.directions)
            for vehicle_id in self.vehicle_ids:
                controls[vehicle_id] = 0.0
                speeds[vehicle_id] = float(direction_map[vehicle_id])
            return controls, speeds, radii, sign
        if primitive == PRIMITIVE_O_TRANSLATION:
            direction_map = _pair_map(edge.directions)
            oblique_map = _pair_map(edge.oblique_angles)
            for vehicle_id in self.vehicle_ids:
                controls[vehicle_id] = float(oblique_map[vehicle_id])
                speeds[vehicle_id] = float(direction_map[vehicle_id])
            return controls, speeds, radii, sign
        for vehicle_id in self.vehicle_ids:
            controls[vehicle_id] = 0.0
            speeds[vehicle_id] = 0.0
        return controls, speeds, radii, sign

    def _check_edges(self):
        self.edge_objects = tuple(self.plan.edges)
        if not self.edge_objects:
            return
        previous = None
        for index, edge in enumerate(self.edge_objects):
            if not isinstance(edge, UniformMotionEdge):
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "edge {} is not UniformMotionEdge".format(index),
                           block_index=index)
            if edge.mode not in _MODE_TO_LEGACY:
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "edge {} mode {!r}".format(index, edge.mode),
                           block_index=index)
            if edge.primitive not in UNIFORM_PRIMITIVES:
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "edge {} primitive {!r}".format(
                               index, edge.primitive),
                           block_index=index)
            if _PRIMITIVE_MODE[edge.primitive] != edge.mode:
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "edge {} mode/primitive mismatch".format(index),
                           block_index=index, field="mode")
            if edge.control.geometry_type != _PRIMITIVE_GEOMETRY[edge.primitive]:
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "edge {} control geometry mismatch".format(index),
                           block_index=index, field="control.geometry_type")
            if len(edge.model.offsets) != len(self.model.offsets):
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "edge {} model offsets differ".format(index),
                           block_index=index, field="edges.model")
            for actual, expected in zip(edge.model.offsets,
                                        self.model.offsets):
                if actual.vehicle_id != expected.vehicle_id or (
                        abs(actual.x - expected.x) > _FIELD_TOL or
                        abs(actual.y - expected.y) > _FIELD_TOL):
                    self._fail(UNIFORM_MODEL_MISMATCH,
                               "edge {} model offsets differ".format(index),
                               block_index=index, field="edges.model")
            if not isinstance(edge.allow_reverse, bool):
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "edge {} allow_reverse is not bool".format(index),
                           block_index=index)
            self._limits_equal(
                edge.limits, self.motion_limits,
                "edge{}.limits".format(index), UNIFORM_MODEL_MISMATCH)
            control = edge.control
            primitive = edge.primitive
            length = self._require_finite(
                control.parameter_length,
                "edge{}.parameter_length".format(index))
            if length <= 0.0:
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "edge {} parameter_length is not positive".format(
                               index), block_index=index)
            if primitive in (PRIMITIVE_A_STRAIGHT, PRIMITIVE_O_TRANSLATION):
                dx = control.end_qf.x - control.start_qf.x
                dy = control.end_qf.y - control.start_qf.y
                displacement = math.hypot(dx, dy)
                self._compare_number(
                    displacement, length, _FIELD_TOL,
                    "edge{}.control.length".format(index),
                    UNIFORM_MODEL_MISMATCH, block_index=index)
                if displacement <= 0.0:
                    self._fail(UNIFORM_MODEL_MISMATCH,
                               "edge {} has zero translation".format(index),
                               block_index=index)
                if abs(control.end_qf.psi - control.start_qf.psi) > (
                        _GEOMETRY_TOL):
                    self._fail(UNIFORM_MODEL_MISMATCH,
                               "edge {} translation changes psi".format(index),
                               block_index=index, field="control.psi")
            elif primitive == PRIMITIVE_A_ARC:
                delta_psi = control.end_qf.psi - control.start_qf.psi
                self._compare_number(
                    abs(delta_psi), length, _FIELD_TOL,
                    "edge{}.control.length".format(index),
                    UNIFORM_MODEL_MISMATCH, block_index=index)
            else:
                if abs(control.end_qf.psi - control.start_qf.psi) > (
                        _GEOMETRY_TOL):
                    self._fail(UNIFORM_MODEL_MISMATCH,
                               "edge {} spin changes psi_F".format(index),
                               block_index=index, field="control.psi")
                if math.hypot(control.end_qf.x - control.start_qf.x,
                              control.end_qf.y - control.start_qf.y) > (
                        _GEOMETRY_TOL):
                    self._fail(UNIFORM_MODEL_MISMATCH,
                               "edge {} spin moves q_F".format(index),
                               block_index=index, field="control.position")

            start_map = _pair_map(edge.start_headings)
            end_map = _pair_map(edge.end_headings)
            direction_map = _pair_map(edge.directions)
            if set(start_map) != set(self.vehicle_ids) or set(
                    end_map) != set(self.vehicle_ids) or set(
                    direction_map) != set(self.vehicle_ids):
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "edge {} heading/direction ids differ".format(index),
                           block_index=index)
            for vehicle_id in self.vehicle_ids:
                if direction_map[vehicle_id] not in (-1, 1):
                    self._fail(UNIFORM_MODEL_MISMATCH,
                               "edge {} direction must be +-1".format(index),
                               block_index=index, vehicle_id=vehicle_id)
            controls, speeds, radii, sign = self._edge_controls(edge, primitive)

            expected_end = {}
            expected_curvatures = {}
            expected_oblique = {}
            expected_speeds = {}
            expected_controls = {}
            if primitive == PRIMITIVE_A_STRAIGHT:
                dx = control.end_qf.x - control.start_qf.x
                dy = control.end_qf.y - control.start_qf.y
                tangent = math.atan2(dy, dx)
                for offset in self.model.offsets:
                    vehicle_id = offset.vehicle_id
                    sigma = direction_map[vehicle_id]
                    required = self._wrap_angle(
                        tangent if sigma > 0 else tangent + math.pi)
                    self._compare_number(
                        self._wrap_angle(start_map[vehicle_id]), required,
                        _GEOMETRY_TOL,
                        "edge{}.start_heading[{}]".format(index, vehicle_id),
                        UNIFORM_MODEL_MISMATCH, block_index=index,
                        vehicle_id=vehicle_id)
                    if sigma < 0 and not edge.allow_reverse:
                        self._fail(UNIFORM_MODEL_MISMATCH,
                                   "edge {} reverse not allowed".format(index),
                                   block_index=index, vehicle_id=vehicle_id,
                                   constraint="allow_reverse")
                    if sigma < 0 and not self.request.allow_reverse:
                        self._fail(UNIFORM_MODEL_MISMATCH,
                                   "edge {} uses reverse but the request "
                                   "forbids it".format(index),
                                   block_index=index, vehicle_id=vehicle_id,
                                   constraint="allow_reverse")
                    expected_end[vehicle_id] = start_map[vehicle_id]
                    expected_curvatures[vehicle_id] = 0.0
                    expected_speeds[vehicle_id] = float(sigma)
                    expected_controls[vehicle_id] = 0.0
            elif primitive == PRIMITIVE_A_ARC:
                pivot_x = control.pivot_x
                pivot_y = control.pivot_y
                for offset in self.model.offsets:
                    vehicle_id = offset.vehicle_id
                    rx = offset.x - pivot_x
                    ry = offset.y - pivot_y
                    sigma = direction_map[vehicle_id]
                    tangent = control.start_qf.psi + math.atan2(rx, -ry)
                    motion = tangent if sign > 0 else tangent + math.pi
                    required = self._wrap_angle(
                        motion if sigma > 0 else motion + math.pi)
                    self._compare_number(
                        self._wrap_angle(start_map[vehicle_id]), required,
                        _GEOMETRY_TOL,
                        "edge{}.start_heading[{}]".format(index, vehicle_id),
                        UNIFORM_MODEL_MISMATCH, block_index=index,
                        vehicle_id=vehicle_id)
                    if sigma < 0 and (not edge.allow_reverse or
                                      not self.request.allow_reverse):
                        self._fail(UNIFORM_MODEL_MISMATCH,
                                   "edge {} reverse not allowed".format(index),
                                   block_index=index, vehicle_id=vehicle_id,
                                   constraint="allow_reverse")
                    curvature = controls[vehicle_id]
                    if abs(curvature) > 1.0 / self.motion_limits.r_min_ack + (
                            _FIELD_TOL):
                        self._fail(
                            UNIFORM_MODEL_MISMATCH,
                            "edge {} curvature exceeds 1/r_min".format(index),
                            block_index=index, vehicle_id=vehicle_id,
                            constraint="curvature", max_error=abs(curvature))
                    expected_end[vehicle_id] = (
                        start_map[vehicle_id] + sign * length)
                    expected_curvatures[vehicle_id] = curvature
                    expected_speeds[vehicle_id] = speeds[vehicle_id]
                    expected_controls[vehicle_id] = curvature
            elif primitive == PRIMITIVE_O_TRANSLATION:
                dx = control.end_qf.x - control.start_qf.x
                dy = control.end_qf.y - control.start_qf.y
                unit_x, unit_y = dx / length, dy / length
                oblique_map = _pair_map(edge.oblique_angles)
                if set(oblique_map) != set(self.vehicle_ids):
                    self._fail(UNIFORM_MODEL_MISMATCH,
                               "edge {} oblique ids differ".format(index),
                               block_index=index)
                for offset in self.model.offsets:
                    vehicle_id = offset.vehicle_id
                    sigma = direction_map[vehicle_id]
                    phi = oblique_map[vehicle_id]
                    if abs(phi) > math.pi / 2.0 + _FIELD_TOL:
                        self._fail(UNIFORM_MODEL_MISMATCH,
                                   "edge {} oblique angle out of range".format(
                                       index), block_index=index,
                                   vehicle_id=vehicle_id,
                                   constraint="oblique_angle")
                    expected_x = sigma * math.cos(start_map[vehicle_id] + phi)
                    expected_y = sigma * math.sin(start_map[vehicle_id] + phi)
                    residual = max(abs(expected_x - unit_x),
                                   abs(expected_y - unit_y))
                    if residual > _GEOMETRY_TOL:
                        self._fail(
                            UNIFORM_MODEL_MISMATCH,
                            "edge {} oblique vector residual {:.3g}".format(
                                index, residual), block_index=index,
                            vehicle_id=vehicle_id,
                            constraint="oblique_vector", max_error=residual)
                    if sigma < 0 and (not edge.allow_reverse or
                                      not self.request.allow_reverse):
                        self._fail(UNIFORM_MODEL_MISMATCH,
                                   "edge {} reverse not allowed".format(index),
                                   block_index=index, vehicle_id=vehicle_id,
                                   constraint="allow_reverse")
                    expected_end[vehicle_id] = start_map[vehicle_id]
                    expected_oblique[vehicle_id] = phi
                    expected_speeds[vehicle_id] = float(sigma)
                    expected_controls[vehicle_id] = phi
            else:
                delta_map = {
                    vehicle_id: end_map[vehicle_id] - start_map[vehicle_id]
                    for vehicle_id in self.vehicle_ids}
                largest = max(abs(value) for value in delta_map.values())
                if largest <= 1e-12:
                    self._fail(UNIFORM_MODEL_MISMATCH,
                               "edge {} has no spin".format(index),
                               block_index=index)
                self._compare_number(
                    largest, length, _FIELD_TOL,
                    "edge{}.spin_length".format(index),
                    UNIFORM_MODEL_MISMATCH, block_index=index)
                for offset in self.model.offsets:
                    vehicle_id = offset.vehicle_id
                    delta = delta_map[vehicle_id]
                    expected_direction = -1 if delta < -1e-12 else 1
                    if direction_map[vehicle_id] != expected_direction:
                        self._fail(UNIFORM_MODEL_MISMATCH,
                                   "edge {} spin direction mismatch".format(
                                       index), block_index=index,
                                   vehicle_id=vehicle_id,
                                   constraint="direction")
                    expected_end[vehicle_id] = end_map[vehicle_id]
                    expected_speeds[vehicle_id] = 0.0
                    expected_controls[vehicle_id] = 0.0
            self._compare_heading_map(
                end_map, expected_end, "edge{}.end_headings".format(index),
                UNIFORM_MODEL_MISMATCH, block_index=index)
            self._check_edge_derived_maps(
                index, edge, expected_curvatures, expected_oblique,
                expected_speeds, expected_end, start_map, controls,
                primitive)
            self._check_edge_segments(
                index, edge, controls, speeds, radii, expected_end,
                start_map, sign)
            # FR01's own validator is an additional consistency gate, not the
            # source of the checks above.
            try:
                from .uniform_motion import validate_uniform_edge
                validate_uniform_edge(edge)
            except FormationValidationError as exc:
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "edge {} failed FR01 validation: {}".format(
                               index, exc), block_index=index)
            if previous is not None:
                self._compare_pose(
                    edge.control.start_qf, previous.control.end_qf,
                    "edges[{}-{}].qf_continuity".format(index - 1, index),
                    UNIFORM_MODEL_MISMATCH)
                self._compare_heading_map(
                    _pair_map(edge.start_headings),
                    _pair_map(previous.end_headings),
                    "edges[{}-{}].heading_continuity".format(
                        index - 1, index),
                    UNIFORM_MODEL_MISMATCH)
            previous = edge
            self._count(len(self.vehicle_ids) * 4 + 8)

    @staticmethod
    def _wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _check_edge_derived_maps(self, index, edge, expected_curvatures,
                                 expected_oblique, expected_speeds,
                                 expected_end, start_map, controls,
                                 primitive):
        for name, expected in (
                ("curvatures", expected_curvatures),
                ("oblique_angles", expected_oblique),
                ("signed_progress_speeds", expected_speeds)):
            actual = _pair_map(getattr(edge, name))
            if set(actual) != set(expected):
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "edge {} {} ids differ".format(index, name),
                           block_index=index)
            for vehicle_id in actual:
                error = self._compare_number(
                    actual[vehicle_id], expected[vehicle_id], _FIELD_TOL,
                    "edge{}.{}.{}".format(index, name, vehicle_id),
                    UNIFORM_MODEL_MISMATCH, block_index=index,
                    vehicle_id=vehicle_id)
                self._max_mode_residual = max(self._max_mode_residual, error)
        expected_wheels = {}
        legacy = _MODE_TO_LEGACY[edge.mode]
        for vehicle_id in self.vehicle_ids:
            angles, factors = wheel_configuration(
                Action(legacy, 0.0, controls[vehicle_id]),
                self.motion_limits)
            expected_wheels[vehicle_id] = (
                tuple(float(value) for value in angles),
                tuple(float(value) for value in factors))
        for name, at_end in (("start", False), ("end", True)):
            actual_angles = {
                int(vehicle_id): tuple(float(value) for value in values)
                for vehicle_id, values in getattr(
                    edge, "wheel_angles_{}".format(name))}
            actual_factors = {
                int(vehicle_id): tuple(float(value) for value in values)
                for vehicle_id, values in getattr(
                    edge, "wheel_factors_{}".format(name))}
            if set(actual_angles) != set(self.vehicle_ids) or set(
                    actual_factors) != set(self.vehicle_ids):
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "edge {} wheel ids differ".format(index),
                           block_index=index)
            for vehicle_id in self.vehicle_ids:
                angles, factors = expected_wheels[vehicle_id]
                actual_a = actual_angles[vehicle_id]
                actual_f = actual_factors[vehicle_id]
                if len(actual_a) != 4 or len(actual_f) != 4:
                    self._fail(UNIFORM_MODEL_MISMATCH,
                               "edge {} wheel tuple length".format(index),
                               block_index=index, vehicle_id=vehicle_id)
                for wheel in range(4):
                    angle_error = self._compare_number(
                        actual_a[wheel], angles[wheel], _WHEEL_TOL,
                        "edge{}.wheel_angles_{}[{}][{}]".format(
                            index, name, vehicle_id, wheel),
                        UNIFORM_MODEL_MISMATCH, block_index=index,
                        vehicle_id=vehicle_id)
                    factor_error = self._compare_number(
                        actual_f[wheel], factors[wheel], _WHEEL_TOL,
                        "edge{}.wheel_factors_{}[{}][{}]".format(
                            index, name, vehicle_id, wheel),
                        UNIFORM_MODEL_MISMATCH, block_index=index,
                        vehicle_id=vehicle_id)
                    if name == "start":
                        self._max_wheel_start_error = max(
                            self._max_wheel_start_error,
                            angle_error, factor_error)
                    else:
                        self._max_wheel_end_error = max(
                            self._max_wheel_end_error,
                            angle_error, factor_error)
                    self._max_normalized_mode_residual = max(
                        self._max_normalized_mode_residual,
                        angle_error, factor_error)

    def _check_edge_segments(self, index, edge, controls, speeds, radii,
                             expected_end, start_map, sign):
        if not isinstance(edge.segments, tuple):
            self._fail(UNIFORM_MODEL_MISMATCH,
                       "edge {} segments is not a tuple".format(index),
                       block_index=index)
        expected_ids = tuple(self.vehicle_ids)
        actual_ids = tuple(segment.vehicle_id for segment in edge.segments)
        if actual_ids != expected_ids:
            self._fail(UNIFORM_MODEL_MISMATCH,
                       "edge {} segment ids/order differ".format(index),
                       block_index=index, field="segments")
        start_centers = _centers(edge.control.start_qf, self.model.offsets)
        end_centers = _centers(edge.control.end_qf, self.model.offsets)
        length = edge.control.parameter_length
        for segment, vehicle_id in zip(edge.segments, expected_ids):
            if not isinstance(segment, VehicleModeSegment):
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "edge {} segment type".format(index),
                           block_index=index, vehicle_id=vehicle_id)
            if segment.mode != _MODE_TO_LEGACY[edge.mode]:
                self._fail(UNIFORM_MODEL_MISMATCH,
                           "edge {} segment mode differs".format(index),
                           block_index=index, vehicle_id=vehicle_id,
                           field="segments.mode")
            expected_control = controls[vehicle_id]
            if edge.primitive == PRIMITIVE_A_ARC:
                travel = abs(speeds[vehicle_id]) * length
                pivot_x = edge.control.pivot_x
                pivot_y = edge.control.pivot_y
            elif edge.primitive == PRIMITIVE_S_SPIN:
                travel = 0.0
                pivot_x = 0.0
                pivot_y = 0.0
            else:
                travel = length
                pivot_x = 0.0
                pivot_y = 0.0
            fields = (
                ("start_station", 0.0, _FIELD_TOL),
                ("end_station", length, _FIELD_TOL),
                ("direction", _pair_map(edge.directions)[vehicle_id], 0.0),
                ("theta_start", start_map[vehicle_id], _GEOMETRY_TOL),
                ("theta_end", expected_end[vehicle_id], _GEOMETRY_TOL),
                ("control", expected_control, _WHEEL_TOL),
                ("travel_distance", travel, _FIELD_TOL),
                ("start_center_x", start_centers[vehicle_id][0], _GEOMETRY_TOL),
                ("start_center_y", start_centers[vehicle_id][1], _GEOMETRY_TOL),
                ("end_center_x", end_centers[vehicle_id][0], _GEOMETRY_TOL),
                ("end_center_y", end_centers[vehicle_id][1], _GEOMETRY_TOL),
                ("pivot_x", pivot_x, _FIELD_TOL),
                ("pivot_y", pivot_y, _FIELD_TOL),
            )
            for name, value, tolerance in fields:
                if name == "direction":
                    if getattr(segment, name) != value:
                        self._fail(UNIFORM_MODEL_MISMATCH,
                                   "edge {} segment {}".format(index, name),
                                   block_index=index, vehicle_id=vehicle_id)
                else:
                    error = self._compare_number(
                        getattr(segment, name), value, tolerance,
                        "edge{}.segments[{}].{}".format(
                            index, vehicle_id, name),
                        UNIFORM_MODEL_MISMATCH, block_index=index,
                        vehicle_id=vehicle_id)
                    if name == "control":
                        self._max_mode_residual = max(
                            self._max_mode_residual, error)

    # -- phase axis ----------------------------------------------------
    def _initial_descriptors(self):
        angles, factors = wheel_configuration(
            Action(ACKERMANN, 0.0, 0.0), self.motion_limits)
        return {
            vehicle_id: {
                "mode": MODE_A,
                "direction": 1,
                "control": 0.0,
                "angles": tuple(float(value) for value in angles),
            } for vehicle_id in self.vehicle_ids}

    def _edge_descriptors(self, edge, at_end):
        values = {
            configuration.vehicle_id: configuration
            for configuration in edge.wheel_configurations(at_end=at_end)}
        direction_map = _pair_map(edge.directions)
        result = {}
        for vehicle_id in self.vehicle_ids:
            result[vehicle_id] = {
                "mode": edge.mode,
                "direction": direction_map[vehicle_id],
                "control": self._edge_control_value(edge, vehicle_id),
                "angles": tuple(float(value)
                                for value in values[vehicle_id].angles),
            }
        return result

    def _edge_control_value(self, edge, vehicle_id):
        if edge.primitive == PRIMITIVE_A_ARC:
            segment = {item.vehicle_id: item for item in edge.segments}[
                vehicle_id]
            return float(segment.control)
        if edge.primitive == PRIMITIVE_O_TRANSLATION:
            return _pair_map(edge.oblique_angles)[vehicle_id]
        return 0.0

    def _expected_setup(self, previous, edge, index, t0_s, initial):
        current = self._edge_descriptors(edge, at_end=False)
        demands = []
        mode_changed = False
        wheel_changed = False
        for vehicle_id in self.vehicle_ids:
            before = previous[vehicle_id]
            after = current[vehicle_id]
            signed_deltas = tuple(
                self._wrap_angle(end - start) for start, end in zip(
                    before["angles"], after["angles"]))
            delta = max(abs(value) for value in signed_deltas)
            event = mode_switch_event(delta, self.motion_limits)
            if before["mode"] != after["mode"]:
                mode_changed = True
            if delta > 1e-9:
                wheel_changed = True
            demands.append({
                "vehicle_id": vehicle_id,
                "from_mode": before["mode"], "to_mode": after["mode"],
                "from_direction": before["direction"],
                "to_direction": after["direction"],
                "from_control": before["control"],
                "to_control": after["control"],
                "wheel_angles_start": before["angles"],
                "wheel_angles_end": after["angles"],
                "wheel_angle_deltas": signed_deltas,
                "max_wheel_angle_delta_rad": delta,
                "event_duration_s": float(event["duration"]),
                "setup_s": float(event["setup_s"]),
                "steering_s": float(event["steering_s"]),
                "steering_profile": str(event["steering_profile"]),
                "event_steer_rate_peak_radps": float(
                    event["steer_rate_peak"]),
                "event_steer_accel_peak_radps2": float(
                    event["steer_accel_peak"]),
            })
        if not mode_changed and not wheel_changed:
            return None
        duration = max(item["event_duration_s"] for item in demands)
        overhead = max(item["setup_s"] for item in demands)
        steering = duration - overhead
        if steering < -1e-12:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "edge {} common steering duration is negative".format(
                           index), block_index=index, constraint="setup")
        steering = max(0.0, steering)
        rate_peak = 0.0
        accel_peak = 0.0
        for demand in demands:
            for delta in demand["wheel_angle_deltas"]:
                rate_peak = max(
                    rate_peak, 1.875 * abs(delta) / max(steering, 1e-12))
                accel_peak = max(
                    accel_peak,
                    (10.0 / math.sqrt(3.0)) * abs(delta)
                    / max(steering ** 2, 1e-12))
        if rate_peak > self.motion_limits.steer_rate_max * (
                1.0 + _SETUP_TOL):
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "edge {} setup rate {:.12g} exceeds {:.12g}".format(
                           index, rate_peak,
                           self.motion_limits.steer_rate_max),
                       block_index=index, constraint="steer_rate",
                       max_error=rate_peak)
        if accel_peak > self.motion_limits.steer_accel_max * (
                1.0 + _SETUP_TOL):
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "edge {} setup acceleration {:.12g} exceeds "
                       "{:.12g}".format(
                           index, accel_peak,
                           self.motion_limits.steer_accel_max),
                       block_index=index, constraint="steer_acceleration",
                       max_error=accel_peak)
        from_modes = {item["from_mode"] for item in demands}
        to_modes = {item["to_mode"] for item in demands}
        if len(from_modes) != 1 or len(to_modes) != 1:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "edge {} setup has per-vehicle global modes".format(
                           index), block_index=index, constraint="mode")
        return {
            "phase_id": ("initial_setup_0000" if initial
                         else "setup_{:04d}".format(index)),
            "edge_index": index, "t0_s": t0_s,
            "duration_s": duration, "setup_overhead_s": overhead,
            "steering_duration_s": steering, "initial": initial,
            "from_mode": next(iter(from_modes)),
            "to_mode": next(iter(to_modes)),
            "qf": edge.start_qf,
            "headings": _headings_record(edge.start_headings),
            "demands": demands,
            "common_steer_rate_peak_radps": rate_peak,
            "common_steer_accel_peak_radps2": accel_peak,
        }

    def _check_phase_axis(self):
        if not self.edge_objects:
            self._count()
            return
        expected_phases = []
        previous = self._initial_descriptors()
        cursor = 0.0
        for index, edge in enumerate(self.edge_objects):
            block = FormationBlock(
                geometry_type=edge.control.geometry_type,
                parameter_length=edge.control.parameter_length,
                formation_controls=(edge.control,),
                vehicle_mode_segments=tuple(edge.segments),
                block_index=index)
            try:
                item = _analyze_block(
                    block, self.model.offsets, self.request.timing_limits,
                    self.motion_limits)
            except _TimingInfeasible as exc:
                self._fail(
                    FORMATION_RETIMING_INFEASIBLE,
                    "edge {} S8 timing failed: {}".format(index, exc),
                    block_index=index, vehicle_id=exc.vehicle_id,
                    constraint=exc.constraint)
            except FormationValidationError as exc:
                self._fail(
                    UNIFORM_PHASE_SCHEDULE_INVALID,
                    "edge {} S8 analysis rejected: {}".format(index, exc),
                    block_index=index)
            setup = self._expected_setup(
                previous, edge, index, cursor, index == 0)
            if setup is not None:
                expected_phases.append(("setup", setup))
                cursor = setup["t0_s"] + setup["duration_s"]
            motion = {
                "phase_id": "motion_{:04d}".format(index),
                "edge_index": index,
                "edge": edge,
                "block": block,
                "item": item,
                "t0_s": cursor,
                "duration_s": float(item.duration),
                "required_duration_s": float(item.required_duration),
                "dynamic_duration_s": float(
                    item.report["dynamic_duration_s"]),
                "driver": dict(item.driver),
                "limiting": tuple(dict(record) for record in item.limiting),
                "peaks": {int(vehicle_id): dict(values)
                          for vehicle_id, values in item.peaks.items()},
            }
            expected_phases.append(("motion", motion))
            cursor = motion["t0_s"] + motion["duration_s"]
            previous = self._edge_descriptors(edge, at_end=True)
        phases = tuple(self.plan.phases)
        if len(phases) != len(expected_phases):
            self._fail(
                UNIFORM_PHASE_SCHEDULE_INVALID,
                "phase count {} != expected {}".format(
                    len(phases), len(expected_phases)),
                field="phases", block_index=min(
                    len(phases), len(expected_phases)))
        setup_cursor = 0.0
        for position, (kind, expected) in enumerate(expected_phases):
            phase = phases[position]
            if kind == "setup":
                if not isinstance(phase, SetupTiming):
                    self._fail(
                        UNIFORM_PHASE_SCHEDULE_INVALID,
                        "phase {} must be SetupTiming".format(position),
                        field="phases", block_index=position)
                self._check_setup_phase(phase, expected, setup_cursor,
                                        position)
                setup_cursor = phase.end_time
                self.setup_data.append({
                    "phase": phase, "expected": expected})
            else:
                if not isinstance(phase, MotionTiming):
                    self._fail(
                        UNIFORM_PHASE_SCHEDULE_INVALID,
                        "phase {} must be MotionTiming".format(position),
                        field="phases", block_index=position)
                self._check_motion_phase(phase, expected, setup_cursor,
                                         position)
                setup_cursor = phase.end_time
                self.motion_data.append({
                    "phase": phase, "edge_index": expected["edge_index"],
                    "edge": expected["edge"], "block": expected["block"],
                    "item": expected["item"]})
        if setup_cursor != self.plan.duration_s:
            self._fail(
                UNIFORM_PHASE_SCHEDULE_INVALID,
                "phase chain end {:.17g} != duration {:.17g}".format(
                    setup_cursor, self.plan.duration_s),
                field="duration_s", time_s=setup_cursor)
        final = self.plan.phases[-1]
        self._compare_pose(
            final.edge.end_qf if isinstance(final, MotionTiming) else
            final.qf, self.request.goal_qf,
            "phase_axis.goal_qf", UNIFORM_PHASE_SCHEDULE_INVALID)
        self._count(len(expected_phases) * 2)

    def _check_setup_phase(self, phase, expected, cursor, position):
        if phase.phase_id != expected["phase_id"]:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "setup phase_id {!r} != {!r}".format(
                           phase.phase_id, expected["phase_id"]),
                       field="phase_id", block_index=expected["edge_index"])
        if phase.edge_index != expected["edge_index"]:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "setup edge_index differs", field="edge_index",
                       block_index=expected["edge_index"])
        if phase.initial != expected["initial"]:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "setup initial flag differs", field="initial",
                       block_index=expected["edge_index"])
        if phase.t0_s != cursor:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "setup t0 {:.17g} != cursor {:.17g}".format(
                           phase.t0_s, cursor), field="t0_s",
                       time_s=phase.t0_s, block_index=expected["edge_index"])
        if not phase.duration_s > 0.0:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "setup duration must be positive", field="duration_s",
                       block_index=expected["edge_index"])
        if phase.duration_s < expected["duration_s"] - _SETUP_TOL:
            self._fail(
                UNIFORM_PHASE_SCHEDULE_INVALID,
                "setup duration {:.17g} is shorter than the event minimum "
                "{:.17g}".format(phase.duration_s, expected["duration_s"]),
                field="duration_s", block_index=expected["edge_index"],
                max_error=expected["duration_s"] - phase.duration_s)
        self._compare_number(
            phase.setup_overhead_s, expected["setup_overhead_s"],
            _SETUP_TOL, "setup.setup_overhead_s", UNIFORM_PHASE_SCHEDULE_INVALID,
            block_index=expected["edge_index"])
        self._compare_number(
            phase.steering_duration_s,
            phase.duration_s - phase.setup_overhead_s, _SETUP_TOL,
            "setup.steering_duration_s", UNIFORM_PHASE_SCHEDULE_INVALID,
            block_index=expected["edge_index"])
        if phase.steering_duration_s < expected["steering_duration_s"] - (
                _SETUP_TOL):
            self._fail(
                UNIFORM_PHASE_SCHEDULE_INVALID,
                "setup steering duration {:.17g} is shorter than the event "
                "minimum {:.17g}".format(
                    phase.steering_duration_s,
                    expected["steering_duration_s"]),
                field="steering_duration_s",
                block_index=expected["edge_index"],
                max_error=(expected["steering_duration_s"]
                           - phase.steering_duration_s))
        if phase.from_mode != expected["from_mode"]:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "setup from_mode differs", field="from_mode",
                       block_index=expected["edge_index"])
        if phase.to_mode != expected["to_mode"]:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "setup to_mode differs", field="to_mode",
                       block_index=expected["edge_index"])
        if phase.mode != phase.from_mode:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "setup phase mode must be from_mode", field="mode",
                       block_index=expected["edge_index"])
        self._compare_pose(phase.qf, expected["qf"], "setup.qf",
                           UNIFORM_PHASE_SCHEDULE_INVALID,
                           block_index=expected["edge_index"])
        self._compare_heading_map(
            _headings_record(phase.headings), expected["headings"],
            "setup.headings", UNIFORM_PHASE_SCHEDULE_INVALID,
            block_index=expected["edge_index"])
        actual_demands = {demand.vehicle_id: demand
                          for demand in phase.demands}
        expected_demands = {demand["vehicle_id"]: demand
                            for demand in expected["demands"]}
        if set(actual_demands) != set(expected_demands):
            missing = sorted(set(expected_demands) - set(actual_demands))
            extra = sorted(set(actual_demands) - set(expected_demands))
            self._fail(
                UNIFORM_PHASE_SCHEDULE_INVALID,
                "setup demand ids differ: missing={} extra={}".format(
                    missing, extra),
                field="demands",
                vehicle_id=(missing + extra)[0] if (missing or extra) else None,
                block_index=expected["edge_index"])
        for vehicle_id in self.vehicle_ids:
            actual = actual_demands[vehicle_id]
            wanted = expected_demands[vehicle_id]
            for name in (
                    "from_mode", "to_mode", "from_direction", "to_direction",
                    "from_control", "to_control", "steering_profile"):
                if getattr(actual, name) != wanted[name]:
                    self._fail(
                        UNIFORM_PHASE_SCHEDULE_INVALID,
                        "setup demand {} {} differs".format(vehicle_id, name),
                        field="demands." + name,
                        vehicle_id=vehicle_id,
                        block_index=expected["edge_index"])
            for name in (
                    "wheel_angles_start", "wheel_angles_end",
                    "wheel_angle_deltas"):
                actual_values = getattr(actual, name)
                wanted_values = wanted[name]
                if len(actual_values) != len(wanted_values):
                    self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                               "setup demand wheel length differs",
                               field="demands." + name,
                               vehicle_id=vehicle_id,
                               block_index=expected["edge_index"])
                for wheel, (actual_value, wanted_value) in enumerate(
                        zip(actual_values, wanted_values)):
                    self._compare_number(
                        actual_value, wanted_value, _WHEEL_TOL,
                        "setup.demand[{}].{}[{}]".format(
                            vehicle_id, name, wheel),
                        UNIFORM_PHASE_SCHEDULE_INVALID,
                        vehicle_id=vehicle_id,
                        block_index=expected["edge_index"])
            for name in (
                    "max_wheel_angle_delta_rad", "event_duration_s",
                    "setup_s", "steering_s", "event_steer_rate_peak_radps",
                    "event_steer_accel_peak_radps2"):
                self._compare_number(
                    getattr(actual, name), wanted[name], _WHEEL_TOL,
                    "setup.demand[{}].{}".format(vehicle_id, name),
                    UNIFORM_PHASE_SCHEDULE_INVALID, vehicle_id=vehicle_id,
                    block_index=expected["edge_index"])
        # Common peaks are re-derived from the stored steering duration and
        # the stored wheel deltas; longer legal setups reduce the peaks.
        rate_peak = 0.0
        accel_peak = 0.0
        steering = phase.steering_duration_s
        for demand in phase.demands:
            for delta in demand.wheel_angle_deltas:
                rate_peak = max(
                    rate_peak, 1.875 * abs(delta) / max(steering, 1e-12))
                accel_peak = max(
                    accel_peak,
                    (10.0 / math.sqrt(3.0)) * abs(delta)
                    / max(steering ** 2, 1e-12))
        self._compare_number(
            phase.common_steer_rate_peak_radps, rate_peak, _WHEEL_TOL,
            "setup.common_steer_rate_peak_radps",
            UNIFORM_PHASE_SCHEDULE_INVALID,
            block_index=expected["edge_index"])
        self._compare_number(
            phase.common_steer_accel_peak_radps2, accel_peak, _WHEEL_TOL,
            "setup.common_steer_accel_peak_radps2",
            UNIFORM_PHASE_SCHEDULE_INVALID,
            block_index=expected["edge_index"])
        if rate_peak > self.motion_limits.steer_rate_max * (
                1.0 + _SETUP_TOL):
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "stored setup steering rate exceeds limit",
                       constraint="steer_rate",
                       block_index=expected["edge_index"],
                       max_error=rate_peak)
        if accel_peak > self.motion_limits.steer_accel_max * (
                1.0 + _SETUP_TOL):
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "stored setup steering acceleration exceeds limit",
                       constraint="steer_acceleration",
                       block_index=expected["edge_index"],
                       max_error=accel_peak)
        self._setup_evidence.append({
            "phase_id": str(phase.phase_id),
            "edge_index": int(phase.edge_index),
            "initial": bool(phase.initial),
            "actual_duration_s": float(phase.duration_s),
            "minimum_duration_s": float(expected["duration_s"]),
            "duration_margin_s": float(
                phase.duration_s - expected["duration_s"]),
            "actual_steering_duration_s": float(
                phase.steering_duration_s),
            "minimum_steering_duration_s": float(
                expected["steering_duration_s"]),
            "setup_overhead_s": float(phase.setup_overhead_s),
            "steer_rate_peak_radps": float(rate_peak),
            "steer_rate_limit_radps": float(
                self.motion_limits.steer_rate_max),
            "steer_accel_peak_radps2": float(accel_peak),
            "steer_accel_limit_radps2": float(
                self.motion_limits.steer_accel_max),
        })

    def _check_motion_phase(self, phase, expected, cursor, position):
        edge_index = expected["edge_index"]
        driver = expected.get("driver") or {}
        if phase.phase_id != expected["phase_id"]:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "motion phase_id differs", field="phase_id",
                       block_index=edge_index)
        if phase.edge_index != edge_index:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "motion edge_index differs", field="edge_index",
                       block_index=edge_index)
        if not isinstance(phase.edge, UniformMotionEdge) or (
                phase.edge is not expected["edge"]):
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "motion edge is not the plan edge object",
                       field="edge", block_index=edge_index)
        if phase.t0_s != cursor:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "motion t0 {:.17g} != cursor {:.17g}".format(
                           phase.t0_s, cursor), field="t0_s",
                       time_s=phase.t0_s, block_index=edge_index)
        if not phase.duration_s > 0.0:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "motion duration must be positive",
                       field="duration_s", block_index=edge_index)
        if phase.duration_s > self.request.timing_limits.max_block_duration_s + (
                _TIME_TOL):
            self._fail(
                UNIFORM_PHASE_SCHEDULE_INVALID,
                "motion duration exceeds max_block_duration_s",
                field="duration_s", block_index=edge_index,
                constraint="max_block_duration_s",
                vehicle_id=driver.get("vehicle_id"),
                max_error=phase.duration_s)
        if phase.duration_s < expected["duration_s"] - _TIME_TOL:
            self._fail(
                UNIFORM_PHASE_SCHEDULE_INVALID,
                "motion duration {:.17g} is shorter than the S8 minimum "
                "{:.17g} (constraint={}, vehicle_id={})".format(
                    phase.duration_s, expected["duration_s"],
                    driver.get("constraint", ""),
                    driver.get("vehicle_id")),
                field="duration_s", block_index=edge_index,
                constraint=driver.get("constraint", ""),
                vehicle_id=driver.get("vehicle_id"),
                max_error=expected["duration_s"] - phase.duration_s)
        self._compare_number(
            phase.required_duration_s, expected["required_duration_s"],
            _TIME_TOL, "motion.required_duration_s",
            UNIFORM_PHASE_SCHEDULE_INVALID, block_index=edge_index)
        self._compare_number(
            phase.dynamic_duration_s, expected["dynamic_duration_s"],
            _TIME_TOL, "motion.dynamic_duration_s",
            UNIFORM_PHASE_SCHEDULE_INVALID, block_index=edge_index)
        if not isinstance(phase.block, FormationBlock):
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "motion block is not FormationBlock",
                       field="block", block_index=edge_index)
        if phase.block.geometry_type != expected["block"].geometry_type:
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "motion block geometry differs", field="block",
                       block_index=edge_index)
        self._compare_number(
            phase.block.parameter_length,
            expected["block"].parameter_length, _FIELD_TOL,
            "motion.block.parameter_length", UNIFORM_PHASE_SCHEDULE_INVALID,
            block_index=edge_index)
        if len(phase.block.formation_controls) != 1 or (
                phase.block.formation_controls[0] is not
                expected["block"].formation_controls[0]):
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "motion block controls differ", field="block",
                       block_index=edge_index)
        if tuple(phase.block.vehicle_mode_segments) != tuple(
                expected["block"].vehicle_mode_segments):
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "motion block segments differ", field="block",
                       block_index=edge_index)
        self._compare_record(
            phase.driver, expected["driver"], "motion.driver",
            UNIFORM_PHASE_SCHEDULE_INVALID, category="model",
            tolerance=_TIME_TOL, block_index=edge_index)
        self._compare_record(
            list(phase.limiting), list(expected["limiting"]),
            "motion.limiting", UNIFORM_PHASE_SCHEDULE_INVALID,
            category="model", tolerance=_TIME_TOL, block_index=edge_index)
        if set(phase.peaks) != set(expected["peaks"]):
            self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                       "motion peak ids differ", field="peaks",
                       block_index=edge_index)
        for vehicle_id in self.vehicle_ids:
            self._compare_record(
                dict(phase.peaks[vehicle_id]),
                expected["peaks"][vehicle_id],
                "motion.peaks[{}]".format(vehicle_id),
                UNIFORM_PHASE_SCHEDULE_INVALID, category="model",
                tolerance=_TIME_TOL, block_index=edge_index,
                vehicle_id=vehicle_id)

    # -- independent state evaluation ---------------------------------
    def _pose_at(self, control, progress):
        length = control.parameter_length
        if control.geometry_type == TRANSLATION:
            return FormationPose(
                control.start_qf.x + (progress / length) * (
                    control.end_qf.x - control.start_qf.x),
                control.start_qf.y + (progress / length) * (
                    control.end_qf.y - control.start_qf.y),
                control.start_qf.psi)
        if control.geometry_type == BODY_SPIN:
            return FormationPose(
                control.start_qf.x, control.start_qf.y,
                control.start_qf.psi)
        pivot_x, pivot_y = control.pivot_x, control.pivot_y
        cosine, sine = math.cos(control.start_qf.psi), math.sin(
            control.start_qf.psi)
        pivot_world_x = (
            control.start_qf.x + cosine * pivot_x - sine * pivot_y)
        pivot_world_y = (
            control.start_qf.y + sine * pivot_x + cosine * pivot_y)
        psi = control.start_qf.psi + (progress / length) * (
            control.end_qf.psi - control.start_qf.psi)
        cn, sn = math.cos(psi), math.sin(psi)
        return FormationPose(
            pivot_world_x - cn * pivot_x + sn * pivot_y,
            pivot_world_y - sn * pivot_x - cn * pivot_y,
            psi)

    def _headings_at(self, edge, progress):
        start = _pair_map(edge.start_headings)
        length = edge.control.parameter_length
        if edge.primitive in (PRIMITIVE_A_STRAIGHT,
                              PRIMITIVE_O_TRANSLATION):
            return start
        deltas = {
            vehicle_id: _pair_map(edge.end_headings)[vehicle_id] -
            start[vehicle_id] for vehicle_id in self.vehicle_ids}
        return {
            vehicle_id: start[vehicle_id] + deltas[vehicle_id] * progress
            / length for vehicle_id in self.vehicle_ids}

    def _derivative_context(self, edge, vehicle_id):
        if edge.primitive == PRIMITIVE_A_ARC:
            delta = edge.control.end_qf.psi - edge.control.start_qf.psi
            return float(1 if delta > 0.0 else -1), 0.0
        if edge.primitive == PRIMITIVE_S_SPIN:
            deltas = {
                item.vehicle_id: item.theta_end - item.theta_start
                for item in edge.segments}
            return (deltas[vehicle_id] / edge.control.parameter_length,
                    deltas[vehicle_id])
        return 0.0, 0.0

    def _expected_motion_state(self, time_s, phase, tau_override=None):
        edge = phase.edge
        duration = phase.duration_s
        if tau_override is not None:
            tau = min(1.0, max(0.0, float(tau_override)))
        elif time_s == phase.t0_s:
            tau = 0.0
        elif time_s == phase.end_time:
            tau = 1.0
        else:
            tau = (time_s - phase.t0_s) / duration
            tau = min(1.0, max(0.0, tau))
        progress_unit, first, second, third = self._quintic(tau)
        length = edge.control.parameter_length
        progress = length * progress_unit
        progress_rate = length * first / duration
        progress_acceleration = length * second / (duration * duration)
        progress_jerk = length * third / (duration ** 3)
        qf = self._pose_at(edge.control, progress)
        centers = _centers(qf, self.model.offsets)
        headings = self._headings_at(edge, progress)
        q_first, q_second, q_third, _, _ = _formation_derivatives(
            edge.control, progress)

        def chain(first_vector, second_vector, third_vector):
            velocity = tuple(item * progress_rate for item in first_vector)
            acceleration = tuple(
                second_item * progress_rate ** 2
                + first_item * progress_acceleration
                for first_item, second_item in zip(first_vector, second_vector))
            jerk = tuple(
                third_item * progress_rate ** 3
                + 3.0 * second_item * progress_rate * progress_acceleration
                + first_item * progress_jerk
                for first_item, second_item, third_item in zip(
                    first_vector, second_vector, third_vector))
            return velocity, acceleration, jerk

        qf_velocity, qf_acceleration, qf_jerk = chain(
            q_first, q_second, q_third)
        wheel_positions = _wheel_positions(self.motion_limits)
        angle_map = {
            int(vehicle_id): tuple(float(value) for value in values)
            for vehicle_id, values in edge.wheel_angles_start}
        factor_map = {
            int(vehicle_id): tuple(float(value) for value in values)
            for vehicle_id, values in edge.wheel_factors_start}
        direction_map = _pair_map(edge.directions)
        vehicles = []
        for vehicle_id in self.vehicle_ids:
            offset = self.offset_map[vehicle_id]
            first_v, second_v, third_v = _vehicle_derivatives(
                edge.control, _OffsetContext(offset.x, offset.y),
                progress)
            velocity, acceleration, jerk = chain(
                first_v, second_v, third_v)
            speed = math.hypot(velocity[0], velocity[1])
            theta = headings[vehicle_id]
            theta_rate, _ = self._derivative_context(edge, vehicle_id)
            direction = direction_map[vehicle_id]
            control = self._edge_control_value(edge, vehicle_id)
            yaw_rate = theta_rate * progress_rate
            yaw_acceleration = theta_rate * progress_acceleration
            yaw_jerk = theta_rate * progress_jerk
            if edge.mode == MODE_S:
                center_speed = 0.0
                longitudinal = 0.0
                lateral_speed = 0.0
                rolling = tuple(
                    (-yaw_rate * position_y) * math.cos(angle)
                    + (yaw_rate * position_x) * math.sin(angle)
                    for (position_x, position_y), angle in zip(
                        wheel_positions, angle_map[vehicle_id]))
                lateral_acceleration = 0.0
            else:
                center_speed = direction * speed
                longitudinal = (velocity[0] * math.cos(theta)
                                + velocity[1] * math.sin(theta))
                lateral_speed = (-velocity[0] * math.sin(theta)
                                 + velocity[1] * math.cos(theta))
                rolling = tuple(
                    factor * center_speed
                    for factor in factor_map[vehicle_id])
                lateral_acceleration = (
                    abs(control) * speed ** 2
                    if edge.mode == MODE_A else 0.0)
            vehicles.append(UniformVehicleTimeState(
                vehicle_id=vehicle_id,
                x=centers[vehicle_id][0], y=centers[vehicle_id][1],
                theta=theta, velocity=velocity, acceleration=acceleration,
                jerk=jerk, yaw_rate_radps=yaw_rate,
                yaw_acceleration_radps2=yaw_acceleration,
                yaw_jerk_radps3=yaw_jerk, center_speed_mps=center_speed,
                longitudinal_speed_mps=longitudinal,
                lateral_speed_mps=lateral_speed,
                center_acceleration_mps2=math.hypot(
                    acceleration[0], acceleration[1]),
                center_jerk_mps3=math.hypot(jerk[0], jerk[1]),
                wheel_speed_mps=max(abs(value) for value in rolling),
                lateral_acceleration_mps2=lateral_acceleration,
                direction=int(direction), mode_control=float(control),
                wheel_angles=tuple(angle_map[vehicle_id]),
                wheel_angular_rates=(0.0, 0.0, 0.0, 0.0),
                wheel_angular_accelerations=(0.0, 0.0, 0.0, 0.0),
                wheel_rolling_speeds=rolling))
        return UniformTimeState(
            time_s=time_s, phase_id=phase.phase_id, phase_kind="motion",
            phase_index=self._phase_position(phase),
            edge_index=phase.edge_index, mode=edge.mode, qf=qf,
            vehicles=tuple(vehicles), progress=progress,
            progress_unit=edge.progress_unit, progress_rate=progress_rate,
            progress_acceleration=progress_acceleration,
            progress_jerk=progress_jerk, qf_velocity=qf_velocity,
            qf_acceleration=qf_acceleration, qf_jerk=qf_jerk,
            from_mode=edge.mode, to_mode=edge.mode)

    def _expected_setup_state(self, time_s, phase, expected):
        local = time_s - phase.t0_s
        if phase.steering_duration_s > 0.0:
            steering_tau = (local - phase.setup_overhead_s) / (
                phase.steering_duration_s)
        else:
            steering_tau = 1.0
        if steering_tau <= 0.0:
            tau = 0.0
            rate_factor = 0.0
            accel_factor = 0.0
        elif steering_tau >= 1.0:
            tau = 1.0
            rate_factor = 0.0
            accel_factor = 0.0
        else:
            tau = steering_tau
            _, first, second, _ = self._quintic(tau)
            rate_factor = first / phase.steering_duration_s
            accel_factor = second / (phase.steering_duration_s ** 2)
        progress, _, _, _ = self._quintic(tau)
        centers = _centers(expected["qf"], self.model.offsets)
        headings = expected["headings"]
        demands = {demand["vehicle_id"]: demand
                   for demand in expected["demands"]}
        vehicles = []
        for vehicle_id in self.vehicle_ids:
            demand = demands[vehicle_id]
            angles = tuple(
                start + delta * progress for start, delta in zip(
                    demand["wheel_angles_start"],
                    demand["wheel_angle_deltas"]))
            rates = tuple(delta * rate_factor
                          for delta in demand["wheel_angle_deltas"])
            accelerations = tuple(delta * accel_factor
                                  for delta in demand["wheel_angle_deltas"])
            vehicles.append(self._zero_vehicle(
                vehicle_id, centers[vehicle_id], headings[vehicle_id],
                demand["from_direction"], demand["from_control"], angles,
                wheel_angular_rates=rates,
                wheel_angular_accelerations=accelerations))
        return UniformTimeState(
            time_s=time_s, phase_id=phase.phase_id, phase_kind=phase.kind,
            phase_index=self._phase_position(phase), edge_index=phase.edge_index,
            mode=phase.from_mode, qf=expected["qf"], vehicles=tuple(vehicles),
            progress=0.0, progress_unit=(
                self.edge_objects[phase.edge_index].progress_unit
                if self.edge_objects else "m"),
            progress_rate=0.0, progress_acceleration=0.0, progress_jerk=0.0,
            qf_velocity=(0.0, 0.0, 0.0),
            qf_acceleration=(0.0, 0.0, 0.0), qf_jerk=(0.0, 0.0, 0.0),
            from_mode=phase.from_mode, to_mode=phase.to_mode)

    def _zero_vehicle(self, vehicle_id, center, theta, direction, control,
                      angles, wheel_angular_rates=(0.0, 0.0, 0.0, 0.0),
                      wheel_angular_accelerations=(0.0, 0.0, 0.0, 0.0)):
        return UniformVehicleTimeState(
            vehicle_id=vehicle_id, x=center[0], y=center[1], theta=theta,
            velocity=(0.0, 0.0), acceleration=(0.0, 0.0), jerk=(0.0, 0.0),
            yaw_rate_radps=0.0, yaw_acceleration_radps2=0.0,
            yaw_jerk_radps3=0.0, center_speed_mps=0.0,
            longitudinal_speed_mps=0.0, lateral_speed_mps=0.0,
            center_acceleration_mps2=0.0, center_jerk_mps3=0.0,
            wheel_speed_mps=0.0, lateral_acceleration_mps2=0.0,
            direction=int(direction), mode_control=float(control),
            wheel_angles=tuple(angles),
            wheel_angular_rates=tuple(wheel_angular_rates),
            wheel_angular_accelerations=tuple(wheel_angular_accelerations),
            wheel_rolling_speeds=(0.0, 0.0, 0.0, 0.0))

    @staticmethod
    def _quintic(tau):
        value = tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))
        velocity = 30.0 * tau * tau * (1.0 + tau * (-2.0 + tau))
        acceleration = 60.0 * tau * (1.0 + tau * (-3.0 + 2.0 * tau))
        jerk = 60.0 - 360.0 * tau + 360.0 * tau * tau
        return value, velocity, acceleration, jerk

    def _expected_hold_state(self, time_s):
        centers = _centers(self.plan.start_qf, self.model.offsets)
        headings = _headings_record(self.plan.start_headings)
        angles, _ = wheel_configuration(
            Action(ACKERMANN, 0.0, 0.0), self.motion_limits)
        vehicles = tuple(self._zero_vehicle(
            vehicle_id, centers[vehicle_id], headings[vehicle_id], 1, 0.0,
            tuple(float(value) for value in angles))
            for vehicle_id in self.vehicle_ids)
        return UniformTimeState(
            time_s=time_s, phase_id="hold", phase_kind="hold",
            phase_index=-1, edge_index=None, mode=MODE_A,
            qf=self.plan.start_qf, vehicles=vehicles, progress=0.0,
            progress_unit="m", progress_rate=0.0,
            progress_acceleration=0.0, progress_jerk=0.0,
            qf_velocity=(0.0, 0.0, 0.0),
            qf_acceleration=(0.0, 0.0, 0.0), qf_jerk=(0.0, 0.0, 0.0),
            from_mode=MODE_A, to_mode=MODE_A)

    def _phase_position(self, phase):
        for index, item in enumerate(self.plan.phases):
            if item is phase:
                return index
        return -1

    def _phase_at(self, time_s, side="right"):
        if not self.plan.phases:
            return None
        phases = self.plan.phases
        if side == "left" and time_s > 0.0:
            for phase in phases:
                if time_s == phase.end_time:
                    return phase
        if time_s == self.plan.duration_s:
            return phases[-1]
        for phase in phases:
            if phase.t0_s <= time_s < phase.end_time:
                return phase
        self._fail(
            UNIFORM_PHASE_SCHEDULE_INVALID,
            "no phase contains t={:.17g}".format(time_s),
            field="phase_time", time_s=time_s)

    def _expected_state(self, time_s, side="right"):
        phase = self._phase_at(time_s, side)
        if phase is None:
            return self._expected_hold_state(time_s)
        if isinstance(phase, MotionTiming):
            return self._expected_motion_state(time_s, phase)
        if isinstance(phase, SetupTiming):
            expected = None
            for record in self.setup_data:
                if record["phase"] is phase:
                    expected = record["expected"]
                    break
            if expected is None:
                self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                           "setup phase was not validated",
                           field="setup", time_s=time_s)
            return self._expected_setup_state(time_s, phase, expected)
        self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                   "unsupported phase type", field="phase", time_s=time_s)

    # -- node and interior comparisons --------------------------------
    def _compare_state(self, actual, expected, context, code,
                       category="model", **fields):
        if not isinstance(actual, UniformTimeState):
            self._fail(code, "{} is not UniformTimeState".format(context),
                       field=context, **fields)
        self._compare_number(
            actual.time_s, expected.time_s, _TIME_TOL,
            context + ".time_s", code, category, **fields)
        self._compare_exact(actual.phase_id, expected.phase_id,
                            context + ".phase_id", code, category, **fields)
        self._compare_exact(actual.phase_kind, expected.phase_kind,
                            context + ".phase_kind", code, category,
                            **fields)
        if actual.phase_index != expected.phase_index:
            self._fail(code, "{}.phase_index {} != {}".format(
                context, actual.phase_index, expected.phase_index),
                field=context + ".phase_index", **fields)
        self._compare_exact(actual.edge_index, expected.edge_index,
                            context + ".edge_index", code, category, **fields)
        self._compare_exact(actual.mode, expected.mode,
                            context + ".mode", code, category, **fields)
        self._compare_pose(actual.qf, expected.qf, context + ".qf", code,
                           category, **fields)
        self._compare_number(
            actual.progress, expected.progress, _STATE_TOL,
            context + ".progress", code, category, **fields)
        self._compare_number(
            actual.progress_rate, expected.progress_rate, _STATE_TOL,
            context + ".progress_rate", code, category, **fields)
        self._compare_number(
            actual.progress_acceleration, expected.progress_acceleration,
            _STATE_TOL, context + ".progress_acceleration", code, category,
            **fields)
        self._compare_number(
            actual.progress_jerk, expected.progress_jerk, _STATE_TOL,
            context + ".progress_jerk", code, category, **fields)
        self._compare_exact(actual.progress_unit, expected.progress_unit,
                            context + ".progress_unit", code, category,
                            **fields)
        for name in ("qf_velocity", "qf_acceleration", "qf_jerk"):
            actual_values = getattr(actual, name)
            expected_values = getattr(expected, name)
            for index, (first, second) in enumerate(zip(
                    actual_values, expected_values)):
                self._compare_number(
                    first, second, _STATE_TOL,
                    "{}.{}[{}]".format(context, name, index), code,
                    category, **fields)
        self._compare_exact(actual.stationary, expected.stationary,
                            context + ".stationary", code, category, **fields)
        self._compare_exact(actual.from_mode, expected.from_mode,
                            context + ".from_mode", code, category, **fields)
        self._compare_exact(actual.to_mode, expected.to_mode,
                            context + ".to_mode", code, category, **fields)
        actual_vehicles = {vehicle.vehicle_id: vehicle
                           for vehicle in actual.vehicles}
        expected_vehicles = {vehicle.vehicle_id: vehicle
                             for vehicle in expected.vehicles}
        if set(actual_vehicles) != set(expected_vehicles):
            self._fail(code, "{} vehicle ids differ".format(context),
                       field=context + ".vehicles", **fields)
        for vehicle_id in sorted(expected_vehicles):
            self._compare_vehicle_state(
                actual_vehicles[vehicle_id], expected_vehicles[vehicle_id],
                "{}.vehicle[{}]".format(context, vehicle_id), code,
                category, vehicle_id=vehicle_id, **fields)
        # Rigidity is checked from the actual centers, not from the expected
        # state, so a coherent center tamper cannot hide in q_F.
        actual_centers = {
            vehicle_id: (vehicle.x, vehicle.y)
            for vehicle_id, vehicle in actual_vehicles.items()}
        for offset in self.model.offsets:
            for other in self.model.offsets:
                if other.vehicle_id <= offset.vehicle_id:
                    continue
                reference = math.hypot(
                    other.x - offset.x, other.y - offset.y)
                actual_distance = math.hypot(
                    actual_centers[other.vehicle_id][0]
                    - actual_centers[offset.vehicle_id][0],
                    actual_centers[other.vehicle_id][1]
                    - actual_centers[offset.vehicle_id][1])
                error = abs(actual_distance - reference)
                self._max_rigidity_error = max(
                    self._max_rigidity_error, error)
                if error > _RIGIDITY_TOL:
                    self._fail(
                        code, "{} rigidity pair ({}, {}) error {:.3g}".format(
                            context, offset.vehicle_id, other.vehicle_id,
                            error),
                        field=context + ".rigidity",
                        vehicle_id=offset.vehicle_id,
                        other_vehicle_id=other.vehicle_id,
                        pair_id=self._pair_text(
                            offset.vehicle_id, other.vehicle_id),
                        max_error=error, **fields)
        self._count(1 + len(expected_vehicles))

    def _compare_vehicle_state(self, actual, expected, context, code,
                               category="model", **fields):
        for name in ("x", "y", "theta"):
            self._compare_number(
                getattr(actual, name), getattr(expected, name),
                _GEOMETRY_TOL, context + "." + name, code, category,
                **fields)
        for name in ("velocity", "acceleration", "jerk"):
            actual_values = getattr(actual, name)
            expected_values = getattr(expected, name)
            for index in range(2):
                self._compare_number(
                    actual_values[index], expected_values[index],
                    _STATE_TOL,
                    "{}.{}[{}]".format(context, name, index), code,
                    category, **fields)
        for name in (
                "yaw_rate_radps", "yaw_acceleration_radps2",
                "yaw_jerk_radps3", "center_speed_mps",
                "longitudinal_speed_mps", "lateral_speed_mps",
                "center_acceleration_mps2", "center_jerk_mps3",
                "wheel_speed_mps", "lateral_acceleration_mps2"):
            self._compare_number(
                getattr(actual, name), getattr(expected, name),
                _STATE_TOL, context + "." + name, code, category, **fields)
        if actual.direction != expected.direction:
            self._fail(code, "{}.direction {} != {}".format(
                context, actual.direction, expected.direction),
                field=context + ".direction", **fields)
        self._compare_number(
            actual.mode_control, expected.mode_control, _MODE_CONTROL_TOL,
            context + ".mode_control", code, category, **fields)
        for name in (
                "wheel_angles", "wheel_angular_rates",
                "wheel_angular_accelerations", "wheel_rolling_speeds"):
            actual_values = getattr(actual, name)
            expected_values = getattr(expected, name)
            if len(actual_values) != len(expected_values):
                self._fail(code, "{}.{} length differs".format(
                    context, name), field=context + "." + name, **fields)
            for index, (first, second) in enumerate(zip(
                    actual_values, expected_values)):
                self._compare_number(
                    first, second, _WHEEL_TOL,
                    "{}.{}[{}]".format(context, name, index), code,
                    category, **fields)

    def _check_model_nodes(self):
        if not self.plan.phases:
            actual = self.plan.state_at(0.0)
            expected = self._expected_hold_state(0.0)
            self._compare_state(
                actual, expected, "hold.node", UNIFORM_MODEL_MISMATCH,
                time_s=0.0)
            self._count()
            return
        self._check_node(0.0, "right")
        for index, phase in enumerate(self.plan.phases):
            self._check_node(phase.t0_s, "right")
            if index < len(self.plan.phases) - 1:
                self._check_node(phase.end_time, "left")
                self._check_node(phase.end_time, "right")
                self._check_boundary_continuity(
                    phase, self.plan.phases[index + 1])
            else:
                self._check_node(phase.end_time, "left")
                self._check_node(phase.end_time, "right")
        final = self.plan.phases[-1]
        if isinstance(final, MotionTiming):
            self._compare_pose(
                self.plan.state_at(self.plan.duration_s).qf,
                final.edge.control.end_qf,
                "final.goal_qf", UNIFORM_MODEL_MISMATCH,
                time_s=self.plan.duration_s)
        self._count(len(self.plan.phases) * 3)

    def _check_node(self, time_s, side):
        actual = self.plan.state_at(time_s, side=side)
        expected = self._expected_state(time_s, side)
        context = "node[{:.12g}].{}".format(time_s, side)
        self._compare_state(
            actual, expected, context, UNIFORM_MODEL_MISMATCH,
            time_s=time_s)
        self._count()

    def _check_boundary_continuity(self, first, second):
        left = self.plan.state_at(first.end_time, side="left")
        right = self.plan.state_at(first.end_time, side="right")
        context = "boundary[{:.12g}]".format(first.end_time)
        self._compare_number(
            right.qf.x, left.qf.x, _GEOMETRY_TOL,
            context + ".qf.x", UNIFORM_MODEL_MISMATCH,
            time_s=first.end_time)
        self._compare_number(
            right.qf.y, left.qf.y, _GEOMETRY_TOL,
            context + ".qf.y", UNIFORM_MODEL_MISMATCH,
            time_s=first.end_time)
        self._compare_number(
            right.qf.psi, left.qf.psi, _GEOMETRY_TOL,
            context + ".qf.psi", UNIFORM_MODEL_MISMATCH,
            time_s=first.end_time)
        left_headings = {vehicle.vehicle_id: vehicle.theta
                         for vehicle in left.vehicles}
        right_headings = {vehicle.vehicle_id: vehicle.theta
                          for vehicle in right.vehicles}
        left_centers = {vehicle.vehicle_id: (vehicle.x, vehicle.y)
                        for vehicle in left.vehicles}
        right_centers = {vehicle.vehicle_id: (vehicle.x, vehicle.y)
                         for vehicle in right.vehicles}
        maximum = 0.0
        for vehicle_id in self.vehicle_ids:
            heading_error = abs(
                right_headings[vehicle_id] - left_headings[vehicle_id])
            position_error = math.hypot(
                right_centers[vehicle_id][0] - left_centers[vehicle_id][0],
                right_centers[vehicle_id][1] - left_centers[vehicle_id][1])
            maximum = max(maximum, heading_error, position_error)
            if heading_error > _GEOMETRY_TOL:
                self._fail(
                    UNIFORM_MODEL_MISMATCH,
                    "{} heading jump {:.3g} for vehicle {}".format(
                        context, heading_error, vehicle_id),
                    field=context + ".heading", vehicle_id=vehicle_id,
                    time_s=first.end_time, max_error=heading_error)
            if position_error > _GEOMETRY_TOL:
                self._fail(
                    UNIFORM_MODEL_MISMATCH,
                    "{} center jump {:.3g} for vehicle {}".format(
                        context, position_error, vehicle_id),
                    field=context + ".center", vehicle_id=vehicle_id,
                    time_s=first.end_time, max_error=position_error)
        self._boundary_errors.append({
            "time_s": first.end_time,
            "between": [first.phase_id, second.phase_id],
            "max_heading_or_center_jump_m_or_rad": maximum,
        })
        self._count(len(self.vehicle_ids) * 2)

    def _geometry_state_error(self, actual, expected):
        maximum = max(
            abs(actual.qf.x - expected.qf.x),
            abs(actual.qf.y - expected.qf.y),
            abs(actual.qf.psi - expected.qf.psi))
        actual_vehicles = {
            vehicle.vehicle_id: vehicle for vehicle in actual.vehicles}
        for expected_vehicle in expected.vehicles:
            vehicle = actual_vehicles[expected_vehicle.vehicle_id]
            maximum = max(
                maximum,
                abs(vehicle.x - expected_vehicle.x),
                abs(vehicle.y - expected_vehicle.y),
                abs(vehicle.theta - expected_vehicle.theta))
        return maximum

    def _check_interior_states(self):
        if not self.plan.phases:
            self._interior_counts["hold"] = 0
            self._count()
            return
        phase_counts = []
        for phase in self.plan.phases:
            if isinstance(phase, MotionTiming):
                values = []
                for tau in _INTERIOR_TAUS:
                    time_s = phase.t0_s + tau * phase.duration_s
                    actual = self.plan.state_at(time_s)
                    expected = self._expected_motion_state(time_s, phase)
                    self._compare_state(
                        actual, expected,
                        "interior[{}][{:.6g}]".format(
                            phase.phase_id, tau),
                        UNIFORM_MODEL_MISMATCH, time_s=time_s)
                    self._max_interior_geometry_error = max(
                        self._max_interior_geometry_error,
                        self._geometry_state_error(actual, expected))
                    values.append(time_s)
                self._interior_counts[phase.phase_id] = len(values)
                phase_counts.append(len(values))
                self._check_finite_differences(phase)
            elif isinstance(phase, SetupTiming):
                values = []
                for fraction in _SETUP_INTERIOR_FRACTIONS:
                    time_s = phase.t0_s + fraction * phase.duration_s
                    actual = self.plan.state_at(time_s)
                    expected = self._expected_state(time_s)
                    self._compare_state(
                        actual, expected,
                        "interior[{}][{:.6g}]".format(
                            phase.phase_id, fraction),
                        UNIFORM_MODEL_MISMATCH, time_s=time_s)
                    self._max_interior_geometry_error = max(
                        self._max_interior_geometry_error,
                        self._geometry_state_error(actual, expected))
                    values.append(time_s)
                self._interior_counts[phase.phase_id] = len(values)
                phase_counts.append(len(values))
            else:
                self._fail(UNIFORM_PHASE_SCHEDULE_INVALID,
                           "unsupported interior phase", field="phase")
        # The task requires at least eleven interior times for every phase.
        # Motion phases use seventeen fractions; setup phases use eleven.
        minimum = min(phase_counts) if phase_counts else 0
        if minimum < 11:
            self._fail(
                UNIFORM_MODEL_MISMATCH,
                "only {} interior states per phase; at least 11 are "
                "required".format(minimum),
                field="interior_states")
        self._count(sum(self._interior_counts.values()))

    def _check_finite_differences(self, phase):
        duration = phase.duration_s
        if duration <= 0.0:
            return
        for tau in (0.25, 0.5, 0.75):
            h = 2e-4 * duration
            center = phase.t0_s + tau * duration
            times = (center - 2.0 * h, center - h, center,
                     center + h, center + 2.0 * h)
            if times[0] < phase.t0_s or times[-1] > phase.end_time:
                continue
            positions = {vehicle_id: [] for vehicle_id in self.vehicle_ids}
            headings = {vehicle_id: [] for vehicle_id in self.vehicle_ids}
            for time_s in times:
                state = self._expected_motion_state(time_s, phase)
                for vehicle in state.vehicles:
                    positions[vehicle.vehicle_id].append(
                        (vehicle.x, vehicle.y))
                    headings[vehicle.vehicle_id].append(vehicle.theta)
            expected = self._expected_motion_state(center, phase)
            for vehicle in expected.vehicles:
                vehicle_id = vehicle.vehicle_id
                points = positions[vehicle_id]
                velocity = tuple(
                    (points[0][index] - 8.0 * points[1][index]
                     + 8.0 * points[3][index] - points[4][index])
                    / (12.0 * h) for index in range(2))
                acceleration = tuple(
                    (-points[0][index] + 16.0 * points[1][index]
                     - 30.0 * points[2][index] + 16.0 * points[3][index]
                     - points[4][index]) / (12.0 * h * h)
                    for index in range(2))
                jerk = tuple(
                    (-points[0][index] + 2.0 * points[1][index]
                     - 2.0 * points[3][index] + points[4][index])
                    / (2.0 * h ** 3) for index in range(2))
                theta_values = headings[vehicle_id]
                yaw_rate = (
                    theta_values[0] - 8.0 * theta_values[1]
                    + 8.0 * theta_values[3] - theta_values[4]) / (12.0 * h)
                yaw_acceleration = (
                    -theta_values[0] + 16.0 * theta_values[1]
                    - 30.0 * theta_values[2] + 16.0 * theta_values[3]
                    - theta_values[4]) / (12.0 * h * h)
                yaw_jerk = (
                    -theta_values[0] + 2.0 * theta_values[1]
                    - 2.0 * theta_values[3] + theta_values[4]) / (
                        2.0 * h ** 3)
                for name, actual_values, expected_values in (
                        ("velocity", velocity, vehicle.velocity),
                        ("acceleration", acceleration,
                         vehicle.acceleration),
                        ("jerk", jerk, vehicle.jerk)):
                    for index in range(2):
                        error = abs(actual_values[index]
                                    - expected_values[index])
                        self._max_finite_difference_error = max(
                            self._max_finite_difference_error, error)
                        if error > 1e-4 + 1e-4 * abs(expected_values[index]):
                            self._fail(
                                UNIFORM_MODEL_MISMATCH,
                                "finite difference {}[{}] error {:.3g} at "
                                "t={:.12g} vehicle={}".format(
                                    name, index, error, center, vehicle_id),
                                field="finite_difference." + name,
                                vehicle_id=vehicle_id, time_s=center,
                                max_error=error)
                for name, actual_value, expected_value in (
                        ("yaw_rate", yaw_rate, vehicle.yaw_rate_radps),
                        ("yaw_acceleration", yaw_acceleration,
                         vehicle.yaw_acceleration_radps2),
                        ("yaw_jerk", yaw_jerk, vehicle.yaw_jerk_radps3)):
                    error = abs(actual_value - expected_value)
                    self._max_finite_difference_error = max(
                        self._max_finite_difference_error, error)
                    if error > 1e-4 + 1e-4 * abs(expected_value):
                        self._fail(
                            UNIFORM_MODEL_MISMATCH,
                            "finite difference {} error {:.3g} at "
                            "t={:.12g} vehicle={}".format(
                                name, error, center, vehicle_id),
                            field="finite_difference." + name,
                            vehicle_id=vehicle_id, time_s=center,
                            max_error=error)
            self._count(len(self.vehicle_ids) * 3)

    # -- dynamics ------------------------------------------------------
    def _check_dynamic_peaks(self):
        limits = self.request.timing_limits
        maximum_ratio = 0.0
        for record in self.motion_data:
            phase = record["phase"]
            edge_index = record["edge_index"]
            block = record["block"]
            duration = phase.duration_s
            for vehicle_id in self.vehicle_ids:
                segment = block.segment_map[vehicle_id]
                factors = tuple(float(value) for value in wheel_configuration(
                    Action(segment.mode, 0.0, float(segment.control)),
                    self.motion_limits)[1])
                peaks = {name: 0.0 for name, _ in _DYNAMIC_LIMITS}
                taus = {name: 0.0 for name, _ in _DYNAMIC_LIMITS}
                for tau in _TAUS:
                    values = _dynamic_values(
                        block, segment, self.offset_map[vehicle_id], duration,
                        tau, self.motion_limits, factors)
                    for name, _ in _DYNAMIC_LIMITS:
                        if not math.isfinite(values[name]):
                            self._fail(
                                UNIFORM_MODEL_MISMATCH,
                                "dynamic {} is not finite at tau={}".format(
                                    name, tau),
                                block_index=edge_index,
                                vehicle_id=vehicle_id, constraint=name)
                        if values[name] > peaks[name]:
                            peaks[name] = values[name]
                            taus[name] = tau
                bounds = self._dynamic_bounds(
                    record["edge"], vehicle_id, duration, factors)
                bound_ratios = {}
                for name, limit_name in _DYNAMIC_LIMITS:
                    limit = getattr(limits, limit_name)
                    ratio = peaks[name] / limit
                    maximum_ratio = max(maximum_ratio, ratio)
                    if ratio > 1.0 + self.config.dynamics_tolerance_ratio:
                        self._fail(
                            FORMATION_RETIMING_INFEASIBLE,
                            "{} peak {:.12g} exceeds limit {:.12g}".format(
                                name, peaks[name], limit),
                            constraint=name, vehicle_id=vehicle_id,
                            block_index=edge_index,
                            time_s=phase.t0_s + taus[name] * duration,
                            station=taus[name], max_error=peaks[name])
                    bound_ratio = bounds[name] / limit
                    bound_ratios[name] = bound_ratio
                    if bound_ratio > 1.0 + self.config.dynamics_tolerance_ratio:
                        self._dynamic_continuous_certificate = False
                        limitation = {
                            "block_index": edge_index,
                            "vehicle_id": vehicle_id,
                            "constraint": name,
                            "conservative_bound": bounds[name],
                            "limit": limit,
                            "bound_ratio": bound_ratio,
                            "candidate_peak_ratio": ratio,
                        }
                        self._dynamic_limitations.append(limitation)
                self._dynamic_evidence.setdefault(
                    edge_index, {})[vehicle_id] = {
                        "peaks": {name: peaks[name]
                                  for name, _ in _DYNAMIC_LIMITS},
                        "peak_taus": {name: taus[name]
                                      for name, _ in _DYNAMIC_LIMITS},
                        "conservative_bounds": {
                            name: bounds[name]
                            for name, _ in _DYNAMIC_LIMITS},
                        "bound_ratios": bound_ratios,
                        "candidate_count": len(_TAUS),
                        "duration_s": duration,
                        "continuous_certificate": all(
                            value <= 1.0 + self.config.dynamics_tolerance_ratio
                            for value in bound_ratios.values()),
                    }
                self._count(len(_DYNAMIC_LIMITS) + len(_TAUS))
        self._dynamic_peak_ratio = maximum_ratio
        self.metrics["dynamic_peak_ratio"] = float(maximum_ratio)
        self.metrics["dynamic_continuous_certificate"] = bool(
            self._dynamic_continuous_certificate)
        self.metrics["dynamic_limitations"] = [
            dict(item) for item in self._dynamic_limitations]
        self.metrics["dynamic_evidence"] = {
            int(edge_index): {
                int(vehicle_id): {
                    "peaks": dict(values["peaks"]),
                    "peak_taus": dict(values["peak_taus"]),
                    "conservative_bounds": dict(
                        values["conservative_bounds"]),
                    "bound_ratios": dict(values["bound_ratios"]),
                    "candidate_count": int(values["candidate_count"]),
                    "duration_s": float(values["duration_s"]),
                    "continuous_certificate": bool(
                        values["continuous_certificate"]),
                } for vehicle_id, values in vehicles.items()}
            for edge_index, vehicles in self._dynamic_evidence.items()}

    def _dynamic_bounds(self, edge, vehicle_id, duration, factors):
        """Conservative per-unit-time suprema for every dynamic quantity."""
        length = edge.control.parameter_length
        delta_psi = edge.control.end_qf.psi - edge.control.start_qf.psi
        lambda_dot = 1.875 * length / duration
        lambda_ddot = (10.0 / math.sqrt(3.0)) * length / (duration * duration)
        lambda_dddot = 60.0 * length / (duration ** 3)
        if edge.primitive in (PRIMITIVE_A_STRAIGHT,
                              PRIMITIVE_O_TRANSLATION):
            center_first = 1.0
            center_second = 0.0
            center_third = 0.0
            position_first = 1.0
            speed = position_first * lambda_dot
        elif edge.primitive == PRIMITIVE_A_ARC:
            offset = self.offset_map[vehicle_id]
            radius = math.hypot(
                offset.x - edge.control.pivot_x,
                offset.y - edge.control.pivot_y)
            center_first = radius
            center_second = radius
            center_third = radius
            speed = radius * lambda_dot
        else:
            # A body spin keeps every centre and q_F exactly fixed.
            center_first = 0.0
            center_second = 0.0
            center_third = 0.0
            speed = 0.0
        acceleration = (
            center_second * lambda_dot * lambda_dot
            + center_first * lambda_ddot)
        jerk = (
            center_third * lambda_dot ** 3
            + 3.0 * center_second * lambda_dot * lambda_ddot
            + center_first * lambda_dddot)
        # theta' per unit progress is the heading change divided by the edge
        # length; an A arc has |theta'| = 1 and a straight/O edge has 0.
        if edge.primitive == PRIMITIVE_S_SPIN:
            theta_rate = abs(_pair_map(edge.end_headings)[vehicle_id]
                             - _pair_map(edge.start_headings)[vehicle_id]
                             ) / length
        elif edge.primitive == PRIMITIVE_A_ARC:
            theta_rate = abs(delta_psi) / length
        else:
            theta_rate = 0.0
        yaw_rate = theta_rate * lambda_dot
        yaw_acceleration = theta_rate * lambda_ddot
        yaw_jerk = theta_rate * lambda_dddot
        if edge.mode == MODE_S:
            wheel_speed = max(factors) * yaw_rate
            lateral = 0.0
        else:
            wheel_speed = max(factors) * speed
            lateral = (
                abs(self._edge_control_value(edge, vehicle_id)) * speed ** 2
                if edge.mode == MODE_A else 0.0)
        return {
            "speed": speed,
            "acceleration": acceleration,
            "jerk": jerk,
            "yaw_rate": yaw_rate,
            "yaw_acceleration": yaw_acceleration,
            "yaw_jerk": yaw_jerk,
            "wheel_speed": wheel_speed,
            "lateral_acceleration": lateral,
        }

    # -- projection contract ------------------------------------------
    def _projection_binding(self):
        projection = self.projection
        if not isinstance(projection, UniformSampleProjection):
            self._fail(INVALID_REQUEST,
                       "projection must be UniformSampleProjection",
                       field="projection")
        if projection.semantic_version != PROJECTION_SEMANTICS:
            self._fail(
                UNIFORM_PROJECTION_MISMATCH,
                "projection semantics {!r} != {!r}".format(
                    projection.semantic_version, PROJECTION_SEMANTICS),
                field="projection.semantic_version")
        if projection.model_version != MODEL_VERSION:
            self._fail(
                UNIFORM_PROJECTION_MISMATCH,
                "projection model version {!r} != {!r}".format(
                    projection.model_version, MODEL_VERSION),
                field="projection.model_version")
        if projection.plan_id != self.plan.plan_id:
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "projection plan_id differs",
                       field="projection.plan_id")
        try:
            plan_hash = self.plan.model_sha256()
        except (TypeError, ValueError, OverflowError) as exc:
            self._fail(
                UNIFORM_MODEL_MISMATCH,
                "plan model digest cannot be computed: {}".format(exc),
                field="plan.model")
        if projection.model_sha256 != plan_hash:
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "projection model hash differs",
                       field="projection.model_sha256")
        if projection.certification != CERTIFICATION:
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "projection certification differs",
                       field="projection.certification")
        timed = projection.timed_trajectory
        if not isinstance(timed, FormationTimedTrajectory):
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "projection timed_trajectory type",
                       field="projection.timed_trajectory")
        if timed.plan_id != self.plan.plan_id:
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "timed plan_id differs", field="timed.plan_id")
        if timed.formation_id != self.plan.formation_id:
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "timed formation_id differs",
                       field="timed.formation_id")
        if len(timed.offsets) != len(self.model.offsets):
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "timed offsets count differs", field="timed.offsets")
        for actual, expected in zip(timed.offsets, self.model.offsets):
            if actual.vehicle_id != expected.vehicle_id:
                self._fail(UNIFORM_PROJECTION_MISMATCH,
                           "timed offset ids differ", field="timed.offsets")
            self._compare_number(
                actual.x, expected.x, _FIELD_TOL, "timed.offsets.x",
                UNIFORM_PROJECTION_MISMATCH, category="projection")
            self._compare_number(
                actual.y, expected.y, _FIELD_TOL, "timed.offsets.y",
                UNIFORM_PROJECTION_MISMATCH, category="projection")

    def _check_projection_contract(self):
        self._projection_binding()
        projection = self.projection
        timed = projection.timed_trajectory
        times = tuple(projection.times)
        states = tuple(projection.states)
        self._projection_times = times
        if not times:
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "projection times are empty",
                       field="projection.times")
        if len(states) != len(times):
            self._fail(
                UNIFORM_PROJECTION_MISMATCH,
                "projection has {} states for {} times".format(
                    len(states), len(times)),
                field="projection.states")
        if abs(times[0]) > _TIME_TOL:
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "projection must start at t=0",
                       field="projection.times")
        if abs(times[-1] - self.plan.duration_s) > _TIME_TOL:
            self._fail(
                UNIFORM_PROJECTION_MISMATCH,
                "projection end {:.17g} != plan duration {:.17g}".format(
                    times[-1], self.plan.duration_s),
                field="projection.times")
        for index, value in enumerate(times):
            self._require_finite(value, "projection.times[{}]".format(index),
                                 UNIFORM_PROJECTION_MISMATCH)
            if value < 0.0 or value > self.plan.duration_s + _TIME_TOL:
                self._fail(UNIFORM_PROJECTION_MISMATCH,
                           "projection time outside the plan",
                           field="projection.times", sample_index=index)
            if index and value - times[index - 1] <= _TIME_TOL:
                self._fail(UNIFORM_PROJECTION_MISMATCH,
                           "projection times are not strictly increasing",
                           field="projection.times", sample_index=index)
        for index, (state, time_s) in enumerate(zip(states, times)):
            if not isinstance(state, UniformTimeState):
                self._fail(UNIFORM_PROJECTION_MISMATCH,
                           "projection state type",
                           field="projection.states", sample_index=index)
            if abs(state.time_s - time_s) > _TIME_TOL:
                self._fail(
                    UNIFORM_PROJECTION_MISMATCH,
                    "state time {:.17g} != projection time {:.17g}".format(
                        state.time_s, time_s),
                    field="projection.states.time_s", sample_index=index,
                    time_s=time_s)
            expected = self._expected_state(time_s, "right")
            self._compare_state(
                state, expected,
                "projection.state[{}]".format(index),
                UNIFORM_PROJECTION_MISMATCH, category="projection",
                sample_index=index, time_s=time_s)
        if len(timed.formation_poses) != len(times):
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "formation_poses count differs",
                       field="timed.formation_poses")
        for index, (actual, time_s, state) in enumerate(zip(
                timed.formation_poses, times, states)):
            self._compare_pose(
                actual, state.qf,
                "timed.formation_poses[{}]".format(index),
                UNIFORM_PROJECTION_MISMATCH, category="projection",
                sample_index=index, time_s=time_s)
            self._compare_pose(
                actual, self._expected_state(time_s, "right").qf,
                "timed.formation_poses_expected[{}]".format(index),
                UNIFORM_PROJECTION_MISMATCH, category="projection",
                sample_index=index, time_s=time_s)
        if set(timed.trajectories) != set(self.vehicle_ids):
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "timed trajectory ids differ",
                       field="timed.trajectories")
        for vehicle_id in self.vehicle_ids:
            points = tuple(timed.trajectories[vehicle_id])
            if len(points) != len(times):
                self._fail(UNIFORM_PROJECTION_MISMATCH,
                           "timed point count differs",
                           field="timed.trajectories",
                           vehicle_id=vehicle_id)
            for index, (point, time_s, state) in enumerate(zip(
                    points, times, states)):
                vehicle = state.vehicle(vehicle_id)
                self._compare_number(
                    point.time_from_start, time_s, _TIME_TOL,
                    "timed.point[{}].time".format(index),
                    UNIFORM_PROJECTION_MISMATCH, category="projection",
                    vehicle_id=vehicle_id, sample_index=index)
                for name, actual, expected in (
                        ("center_x", point.center_x, vehicle.x),
                        ("center_y", point.center_y, vehicle.y),
                        ("theta", point.theta, vehicle.theta),
                        ("center_speed_mps", point.center_speed_mps,
                         vehicle.center_speed_mps),
                        ("center_yaw_rate_radps", point.center_yaw_rate_radps,
                         vehicle.yaw_rate_radps),
                        ("mode_control", point.mode_control,
                         vehicle.mode_control),
                        ("station", point.station, state.progress),
                        ("center_acceleration_mps2",
                         point.center_acceleration_mps2,
                         vehicle.center_acceleration_mps2),
                        ("center_jerk_mps3", point.center_jerk_mps3,
                         vehicle.center_jerk_mps3),
                        ("wheel_speed_mps", point.wheel_speed_mps,
                         vehicle.wheel_speed_mps),
                        ("lateral_acceleration_mps2",
                         point.lateral_acceleration_mps2,
                         vehicle.lateral_acceleration_mps2)):
                    tolerance = (_GEOMETRY_TOL if name in (
                        "center_x", "center_y", "theta") else _STATE_TOL)
                    self._compare_number(
                        actual, expected, tolerance,
                        "timed.point[{}].{}".format(index, name),
                        UNIFORM_PROJECTION_MISMATCH, category="projection",
                        vehicle_id=vehicle_id, sample_index=index)
                expected_mode = legacy_mode_word(state.mode)
                if point.mode != expected_mode:
                    self._fail(
                        UNIFORM_PROJECTION_MISMATCH,
                        "timed point mode {} != {}".format(
                            point.mode, expected_mode),
                        field="timed.mode", vehicle_id=vehicle_id,
                        sample_index=index)
                expected_block = (
                    0 if state.edge_index is None else int(state.edge_index))
                if point.block_index != expected_block:
                    self._fail(
                        UNIFORM_PROJECTION_MISMATCH,
                        "timed point block {} != {}".format(
                            point.block_index, expected_block),
                        field="timed.block_index", vehicle_id=vehicle_id,
                        sample_index=index)
                if point.stationary != state.stationary:
                    self._fail(
                        UNIFORM_PROJECTION_MISMATCH,
                        "timed point stationary {} != {}".format(
                            point.stationary, state.stationary),
                        field="timed.stationary", vehicle_id=vehicle_id,
                        sample_index=index)
            self._count(len(times) * 3)
        self._check_timed_phase_table(timed)
        self._check_timing_report(timed)

    def _check_timed_phase_table(self, timed):
        expected = self.plan.phase_table()
        if not isinstance(timed.phases, tuple):
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "timed phases is not a tuple", field="timed.phases")
        if len(timed.phases) != len(expected):
            self._fail(
                UNIFORM_PROJECTION_MISMATCH,
                "timed phase count {} != {}".format(
                    len(timed.phases), len(expected)),
                field="timed.phases")
        self._compare_record(
            list(timed.phases), expected, "timed.phases",
            UNIFORM_PROJECTION_MISMATCH, category="projection",
            tolerance=_TIME_TOL)
        self._compare_exact(
            timed.certification, GEOMETRY_ONLY, "timed.certification",
            UNIFORM_PROJECTION_MISMATCH, category="projection")
        self._compare_exact(
            timed.dynamic_profile, "formation_piecewise_quintic_v1",
            "timed.dynamic_profile", UNIFORM_PROJECTION_MISMATCH,
            category="projection")

    def _check_timing_report(self, timed):
        report = timed.timing_report
        if not isinstance(report, FormationTimingReport):
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "timing_report type", field="timed.timing_report")
        times = self._projection_times
        dt = min((second - first for first, second in zip(times, times[1:])),
                 default=0.0)
        self._compare_number(
            report.dt, dt, _TIME_TOL, "report.dt",
            UNIFORM_PROJECTION_MISMATCH, category="projection")
        self._compare_number(
            report.duration_s, self.plan.duration_s, _TIME_TOL,
            "report.duration_s", UNIFORM_PROJECTION_MISMATCH,
            category="projection")
        motion_seconds = sum(
            phase.duration_s for phase in self.plan.phases
            if isinstance(phase, MotionTiming))
        setup_seconds = sum(
            phase.duration_s for phase in self.plan.phases
            if isinstance(phase, SetupTiming))
        self._compare_number(
            report.moving_seconds, motion_seconds, _TIME_TOL,
            "report.moving_seconds", UNIFORM_PROJECTION_MISMATCH,
            category="projection")
        self._compare_number(
            report.stationary_seconds, setup_seconds, _TIME_TOL,
            "report.stationary_seconds", UNIFORM_PROJECTION_MISMATCH,
            category="projection")
        expected_blocks = [
            {
                "block_index": phase.edge_index,
                "geometry_type": phase.edge.control.geometry_type,
                "parameter_length": phase.edge.control.parameter_length,
                "duration_s": phase.duration_s,
                "dynamic_duration_s": phase.dynamic_duration_s,
                "duration_driver": dict(phase.driver),
                "limiting_constraints": [
                    dict(item) for item in phase.limiting],
                "vehicle_peaks": {
                    int(vehicle_id): dict(values)
                    for vehicle_id, values in phase.peaks.items()},
            } for phase in self.plan.phases
            if isinstance(phase, MotionTiming)]
        self._compare_record(
            [dict(item) for item in report.block_durations],
            expected_blocks, "report.block_durations",
            UNIFORM_PROJECTION_MISMATCH, category="projection",
            tolerance=_TIME_TOL)
        self._compare_record(
            [dict(item) for item in report.phases], self.plan.phase_table(),
            "report.phases", UNIFORM_PROJECTION_MISMATCH,
            category="projection", tolerance=_TIME_TOL)
        expected_events = [
            phase.as_dict() for phase in self.plan.phases
            if isinstance(phase, SetupTiming)]
        if not self.plan.phases:
            expected_events = [{
                "kind": "NO_MOTION", "t0_s": 0.0, "duration_s": 0.0,
                "qf": list(_pose_record(self.plan.start_qf)),
                "headings": [
                    {"vehicle_id": int(vehicle_id),
                     "theta": float(theta)}
                    for vehicle_id, theta in self.plan.start_headings],
            }]
        self._compare_record(
            [dict(item) for item in report.stationary_events],
            expected_events, "report.stationary_events",
            UNIFORM_PROJECTION_MISMATCH, category="projection",
            tolerance=_TIME_TOL)
        if tuple(report.direction_changes):
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "unexpected direction changes",
                       field="report.direction_changes")
        self._compare_record(
            [dict(item) for item in report.active_constraints],
            [dict(phase.driver) for phase in self.plan.phases
             if isinstance(phase, MotionTiming)],
            "report.active_constraints", UNIFORM_PROJECTION_MISMATCH,
            category="projection", tolerance=_TIME_TOL)
        expected_intervals = {}
        for vehicle_id in self.vehicle_ids:
            intervals = []
            for phase in self.plan.phases:
                mode = legacy_mode_word(phase.mode)
                if intervals and intervals[-1]["mode"] == mode and (
                        abs(intervals[-1]["end_time_s"] - phase.t0_s)
                        <= TIME_CONTINUITY_TOL_S):
                    intervals[-1]["end_time_s"] = phase.end_time
                    intervals[-1]["duration_s"] = (
                        intervals[-1]["end_time_s"] - intervals[-1]["t0_s"])
                else:
                    intervals.append({
                        "mode": mode, "t0_s": phase.t0_s,
                        "end_time_s": phase.end_time,
                        "duration_s": phase.duration_s})
            expected_intervals[int(vehicle_id)] = intervals
        actual_intervals = {
            int(vehicle_id): [dict(item) for item in values]
            for vehicle_id, values in report.mode_intervals.items()}
        if set(actual_intervals) != set(expected_intervals):
            self._fail(UNIFORM_PROJECTION_MISMATCH,
                       "report.mode_intervals ids differ",
                       field="report.mode_intervals")
        self._compare_record(
            actual_intervals, expected_intervals, "report.mode_intervals",
            UNIFORM_PROJECTION_MISMATCH, category="projection",
            tolerance=_TIME_TOL)
        self._compare_record(
            {int(vehicle_id): dict(values)
             for vehicle_id, values in report.vehicle_peaks.items()},
            self._expected_vehicle_peaks(), "report.vehicle_peaks",
            UNIFORM_PROJECTION_MISMATCH, category="projection",
            tolerance=_TIME_TOL)
        expected_limits = dict(self.plan.limits.as_dict())
        expected_limits["uniform_semantics"] = PROJECTION_SEMANTICS
        self._compare_record(
            dict(report.limits), expected_limits, "report.limits",
            UNIFORM_PROJECTION_MISMATCH, category="projection",
            tolerance=1e-12)
        for name, expected in (
                ("certification", GEOMETRY_ONLY),
                ("timing_audit", "SELF_CONSISTENCY_ONLY"),
                ("executable", False),
                ("trajectory_published", False),
                ("feasible", False),
                ("independent_timing_audit", False),
                ("collision_certificate", False)):
            self._compare_exact(
                getattr(report, name), expected, "report." + name,
                UNIFORM_PROJECTION_MISMATCH, category="projection")
        self._count(14)

    def _expected_vehicle_peaks(self):
        names = (
            "speed", "acceleration", "jerk", "yaw_rate",
            "yaw_acceleration", "yaw_jerk", "wheel_speed",
            "lateral_acceleration", "steer_rate", "steer_acceleration",
            "wheel_angle")
        result = {vehicle_id: {name: 0.0 for name in names}
                  for vehicle_id in self.vehicle_ids}
        for phase in self.plan.phases:
            if isinstance(phase, MotionTiming):
                for vehicle_id, values in phase.peaks.items():
                    for name, value in values.items():
                        if name in result[vehicle_id]:
                            result[vehicle_id][name] = max(
                                result[vehicle_id][name], float(value))
            elif isinstance(phase, SetupTiming):
                for demand in phase.demands:
                    rate = 0.0
                    accel = 0.0
                    for delta in demand.wheel_angle_deltas:
                        rate = max(rate, 1.875 * abs(delta) / max(
                            phase.steering_duration_s, 1e-12))
                        accel = max(
                            accel,
                            (10.0 / math.sqrt(3.0)) * abs(delta) / max(
                                phase.steering_duration_s ** 2, 1e-12))
                    values = result[demand.vehicle_id]
                    values["steer_rate"] = max(values["steer_rate"], rate)
                    values["steer_acceleration"] = max(
                        values["steer_acceleration"], accel)
                    values["wheel_angle"] = max(
                        values["wheel_angle"],
                        *(abs(value) for value in demand.wheel_angles_start),
                        *(abs(value) for value in demand.wheel_angles_end))
        return result

    # -- collision -----------------------------------------------------
    def _static_nodes(self):
        if not self.plan.phases:
            return [("start", 0.0, self._expected_hold_state(0.0))]
        nodes = [("start", 0.0, self._expected_state(0.0, "right"))]
        for phase in self.plan.phases:
            nodes.append((
                "phase[{}].start".format(phase.phase_id), phase.t0_s,
                self._expected_state(phase.t0_s, "right")))
            nodes.append((
                "phase[{}].end".format(phase.phase_id), phase.end_time,
                self._expected_state(phase.end_time, "left")))
            nodes.append((
                "phase[{}].next".format(phase.phase_id), phase.end_time,
                self._expected_state(phase.end_time, "right")))
        nodes.append((
            "goal", self.plan.duration_s,
            self._expected_state(self.plan.duration_s, "left")))
        return nodes

    def _collision_failure(self, report, code, context, time_s=None,
                           station=None):
        pair_id = None
        if report.vehicle_id is not None and (
                report.other_vehicle_id is not None):
            pair_id = self._pair_text(
                report.vehicle_id, report.other_vehicle_id)
        detail = "{} {}: {}".format(context, report.reason, report.detail)
        self._fail(
            code, detail, field="collision",
            vehicle_id=report.vehicle_id,
            other_vehicle_id=report.other_vehicle_id,
            pair_id=pair_id, segment_index=report.segment_index,
            sample_index=report.sample_index,
            time_s=time_s if time_s is not None else report.time_s,
            station=station if station is not None else report.station)

    def _check_static_collisions(self):
        self._checker = FormationCollisionChecker(
            self.grid, self.model, self.shapes,
            self.config.collision_margin_m)
        nodes = self._static_nodes()
        for label, time_s, state in nodes:
            headings = {vehicle.vehicle_id: vehicle.theta
                        for vehicle in state.vehicles}
            report = self._checker.static_check(state.qf, headings)
            self._count()
            if report.success:
                continue
            if label == "start":
                code = START_FORMATION_COLLISION
            elif label == "goal":
                code = GOAL_FORMATION_COLLISION
            else:
                code = FORMATION_SWEEP_COLLISION
            self._collision_failure(
                report, code, "static {}".format(label), time_s=time_s,
                station=float(state.progress))
        self.metrics["static_collision_checks"] = len(nodes)
        self.metrics["collision_margin_m"] = float(
            self.config.collision_margin_m)

    def _motion_sweep_segment(self, record):
        phase = record["phase"]
        edge = record["edge"]
        length = edge.control.parameter_length
        start_map = _pair_map(edge.start_headings)
        end_map = _pair_map(edge.end_headings)
        if edge.primitive == PRIMITIVE_A_ARC:
            delta_psi = edge.control.end_qf.psi - edge.control.start_qf.psi
            center_bound = 0.0
            for offset in self.model.offsets:
                radius = math.hypot(
                    offset.x - edge.control.pivot_x,
                    offset.y - edge.control.pivot_y)
                center_bound = max(
                    center_bound, 1.875 * abs(delta_psi) * radius)
            formation_bound = 1.875 * abs(delta_psi)
            body_bounds = {
                vehicle_id: 1.875 * abs(
                    end_map[vehicle_id] - start_map[vehicle_id])
                for vehicle_id in self.vehicle_ids}
        elif edge.primitive == PRIMITIVE_S_SPIN:
            center_bound = 0.0
            formation_bound = 0.0
            body_bounds = {
                vehicle_id: 1.875 * abs(
                    end_map[vehicle_id] - start_map[vehicle_id])
                for vehicle_id in self.vehicle_ids}
        else:
            center_bound = 1.875 * length
            formation_bound = 0.0
            body_bounds = {vehicle_id: 0.0 for vehicle_id in self.vehicle_ids}

        def state_at(fraction, phase=phase):
            station = float(fraction)
            time_s = phase.t0_s + station * phase.duration_s
            state = self._expected_motion_state(
                time_s, phase, tau_override=station)
            return FormationState(
                station=station, qf=state.qf,
                body_headings={
                    vehicle.vehicle_id: vehicle.theta
                    for vehicle in state.vehicles})

        def time_at(fraction, phase=phase):
            return phase.t0_s + float(fraction) * phase.duration_s

        return AnalyticSweepSegment(
            station_start=0.0, station_end=1.0, state_at=state_at,
            time_at=time_at,
            center_speed_bound_m_per_lambda=center_bound,
            formation_rate_bound_rad_per_lambda=formation_bound,
            body_rate_bounds_rad_per_lambda=body_bounds,
            label="motion_{:04d}".format(record["edge_index"]))

    def _check_analytic_sweeps(self):
        if not self.motion_data:
            self._count()
            return
        if self._checker is None:
            self._checker = FormationCollisionChecker(
                self.grid, self.model, self.shapes,
                self.config.collision_margin_m)
        policy = SweepPolicy(
            max_corner_step_m=self.config.sweep_spacing_m,
            max_samples_per_segment=self.config.max_samples_per_segment)
        self.metrics["sweep_policy"] = {
            "max_corner_step_m": float(policy.max_corner_step_m),
            "max_samples_per_segment": int(policy.max_samples_per_segment),
            "motion_padding_m": float(policy.motion_padding_m),
            "collision_margin_m": float(self.config.collision_margin_m),
        }
        coverage = {}
        for record in self.motion_data:
            edge_index = int(record["edge_index"])
            segment = self._motion_sweep_segment(record)
            trace = []
            report = self._checker.swept_check_analytic(
                [segment], policy, trace=trace)
            if not report.success:
                if report.reason == "SWEEP_SAMPLE_BUDGET_EXCEEDED":
                    self._fail(
                        UNIFORM_SWEEP_BUDGET_EXCEEDED,
                        "edge {} analytic sweep exceeds the sample budget: "
                        "{}".format(edge_index, report.detail or ""),
                        field="sweep_budget", block_index=edge_index,
                        time_s=report.time_s, station=report.station,
                        segment_index=report.segment_index,
                        sample_index=report.sample_index)
                if report.time_s is None or report.station is None:
                    self._fail(
                        INVALID_TIMED_TRAJECTORY,
                        "analytic sweep failed without an exact state: "
                        "{}".format(report.detail or report.reason),
                        field="sweep", block_index=edge_index,
                        segment_index=report.segment_index,
                        sample_index=report.sample_index)
                self._first_collision = {
                    "edge_index": edge_index,
                    "reason": report.reason,
                    "vehicle_id": report.vehicle_id,
                    "other_vehicle_id": report.other_vehicle_id,
                    "time_s": float(report.time_s),
                    "station": float(report.station),
                    "qf": (list(_pose_record(report.qf))
                           if report.qf is not None else None),
                    "segment_index": int(report.segment_index),
                    "sample_index": int(report.sample_index),
                    "label": segment.label,
                }
                pair_id = None
                if (report.vehicle_id is not None
                        and report.other_vehicle_id is not None):
                    pair_id = self._pair_text(
                        report.vehicle_id, report.other_vehicle_id)
                self.metrics["first_collision"] = dict(self._first_collision)
                self._fail(
                    FORMATION_SWEEP_COLLISION,
                    "edge {} analytic sweep {} at t={:.12g} station={:.12g}"
                    .format(edge_index, report.reason, report.time_s,
                            report.station),
                    field="collision", block_index=edge_index,
                    vehicle_id=report.vehicle_id,
                    other_vehicle_id=report.other_vehicle_id,
                    pair_id=pair_id,
                    segment_index=report.segment_index,
                    sample_index=report.sample_index,
                    time_s=report.time_s, station=report.station)
            budget = [dict(item) for item in trace
                      if item.get("kind") == "budget"]
            checked = [item for item in trace
                       if item.get("fraction") is not None]
            coverage[edge_index] = {
                "label": segment.label,
                "station_start": float(segment.station_start),
                "station_end": float(segment.station_end),
                "center_speed_bound_m_per_lambda": float(
                    segment.center_speed_bound_m_per_lambda),
                "formation_rate_bound_rad_per_lambda": float(
                    segment.formation_rate_bound_rad_per_lambda),
                "body_rate_bounds_rad_per_lambda": {
                    int(vehicle_id): float(value)
                    for vehicle_id, value in
                    segment.body_rate_bounds_rad_per_lambda.items()},
                "bound_m": float(
                    self._checker.analytic_segment_bound(segment)),
                "budget": budget,
                "checked_samples": int(len(checked)),
                "first_time_s": (float(checked[0]["time_s"])
                                 if checked else None),
                "last_time_s": (float(checked[-1]["time_s"])
                                if checked else None),
            }
            self._count(2 + len(checked))
        self.metrics["sweep_coverage"] = dict(coverage)
        self.metrics["first_collision"] = None

    # -- success -------------------------------------------------------
    def _uniform_success(self):
        self.metrics.update({
            "checks": int(self.checks),
            "certification": FEASIBLE,
            "independent_audit": True,
            "collision_certificate": True,
            "map_version_checked": True,
            "model_version": MODEL_VERSION,
            "model_sha256": self.plan.model_sha256(),
            "projection_semantics": PROJECTION_SEMANTICS,
            "duration_s": float(self.plan.duration_s),
            "max_state_error_m_or_rad": float(self._max_state_error),
            "max_projection_error_m_or_rad": float(
                self._max_projection_error),
            "max_rigidity_error_m": float(self._max_rigidity_error),
            "max_finite_difference_error": float(
                self._max_finite_difference_error),
            "max_interior_geometry_error_m_or_rad": float(
                self._max_interior_geometry_error),
            "max_wheel_start_error_rad": float(
                self._max_wheel_start_error),
            "max_wheel_end_error_rad": float(self._max_wheel_end_error),
            "max_normalized_mode_residual": float(
                self._max_normalized_mode_residual),
            "max_mode_residual_rad": float(self._max_mode_residual),
            "interior_counts": {
                str(phase_id): int(count)
                for phase_id, count in self._interior_counts.items()},
            "boundary_errors": [dict(item) for item in self._boundary_errors],
            "setup_evidence": [
                dict(item) for item in self._setup_evidence],
            "dynamic_candidate_count": int(len(_TAUS)),
            "dynamic_candidate_taus": [float(tau) for tau in _TAUS],
            "dynamic_method": {
                "candidate_grid": (
                    "801 uniform samples plus the 0.5, 0.5+/-sqrt(3)/6 "
                    "quintic critical points, deduplicated"),
                "candidate_count": int(len(_TAUS)),
                "quintic_supremum": {
                    "lambda_dot_over_length_per_duration": 1.875,
                    "lambda_ddot_over_length_per_duration_squared":
                        10.0 / math.sqrt(3.0),
                    "lambda_dddot_over_length_per_duration_cubed": 60.0,
                },
                "candidate_peak_meaning": (
                    "maximum over the shared S9 candidate grid; not a "
                    "continuous-time proof by itself"),
                "conservative_bound_meaning": (
                    "per-unit-time supremum from the quintic derivative "
                    "suprema; a bound above the limit means the candidate "
                    "grid alone cannot certify the continuous interval"),
                "strict_continuous_certificate":
                    bool(self._dynamic_continuous_certificate),
            },
        })
        if not all(math.isfinite(float(value)) for value in (
                self.metrics.get("duration_s", 0.0),
                self.metrics.get("max_rigidity_error_m", 0.0),
                self.metrics.get("dynamic_peak_ratio", 0.0),
                self.metrics.get("max_interior_geometry_error_m_or_rad", 0.0),
                self.metrics.get("max_wheel_start_error_rad", 0.0),
                self.metrics.get("max_wheel_end_error_rad", 0.0))):
            self._fail(FORMATION_RETIMING_INFEASIBLE,
                       "uniform audit metrics are not finite",
                       field="metrics")
        return FormationEvaluation(
            plan_id=self.plan_id,
            formation_id=self.formation_id,
            status=FEASIBLE,
            feasible=True,
            reason="OK",
            detail=(
                "independent V0.2 uniform audit passed: edge re-derivation, "
                "common-time schedule, analytic states, dynamics candidates "
                "and conservative bounds, projection contract, static and "
                "analytic swept collision"),
            failure=None,
            metrics=dict(self.metrics),
            checks=self.checks)


def audit_uniform_timed_plan(request, plan, projection, grid, config=None):
    """Audit one V0.2 uniform plan/projection pair; failures are structured."""
    plan_id = getattr(plan, "plan_id", -1)
    formation_id = getattr(plan, "formation_id", "")
    if not isinstance(request, UniformAuditRequest):
        return _uniform_failure(
            plan_id if isinstance(plan_id, int) else -1,
            formation_id if isinstance(formation_id, str) else "",
            INVALID_REQUEST,
            "request must be UniformAuditRequest", field="request")
    if not isinstance(plan, UniformTimedPlan):
        return _uniform_failure(
            request.plan_id, request.formation_id, INVALID_REQUEST,
            "plan must be UniformTimedPlan", field="plan")
    if not isinstance(projection, UniformSampleProjection):
        return _uniform_failure(
            request.plan_id, request.formation_id,
            UNIFORM_PROJECTION_MISMATCH,
            "projection must be UniformSampleProjection",
            field="projection")
    if projection.model_version != MODEL_VERSION:
        return _uniform_failure(
            request.plan_id, request.formation_id,
            UNIFORM_PROJECTION_MISMATCH,
            "projection model version {!r} is not {!r}".format(
                projection.model_version, MODEL_VERSION),
            field="projection.model_version")
    try:
        if config is None:
            config = FormationQualityConfig(
                timing_limits=request.timing_limits)
        if not isinstance(config, FormationQualityConfig):
            raise FormationValidationError(
                "INVALID_CONFIG expected FormationQualityConfig")
        auditor = _UniformAuditor(request, plan, projection, grid, config)
    except FormationValidationError as exc:
        return _uniform_failure(
            request.plan_id, request.formation_id,
            INVALID_REQUEST, "invalid uniform audit input: {}".format(exc))
    return auditor.run()