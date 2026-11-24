"""FR04 thin autonomous pipeline: request -> search -> FR02 timing -> audit.

There is no legacy geometric seed, no goal-nearby pose teleport, no ROS and no
NLP here.  Every nonempty path consists only of FR03 successors derived from
FR01 edges.  The search's terminal validator is the real FR02 audit, and a
returned SUCCESS is rebuilt and independently audited once more before the
pipeline marks it feasible.  A geometric goal path alone is never executable.
"""
from dataclasses import asdict, dataclass, field, replace
import hashlib
import json
import math
import time
from typing import Callable, Mapping, Optional

from .core import Grid
from .formation import FormationModel, FormationPose, FormationValidationError
from .formation_collision import CargoFootprint, FormationCollisionChecker, VehicleShape
from .formation_quality import (
    FormationEvaluation,
    FormationEvaluationFailure,
    FormationQualityConfig,
    REJECTED,
)
from .formation_retiming import _DYNAMIC_LIMITS
from .uniform_motion import UniformMotionEdge
from .uniform_quality import (
    UNIFORM_DYNAMIC_UNCONFIRMED, UniformAuditRequest, audit_uniform_timed_plan,
)
from .uniform_search import (
    GraphSearchConfig,
    SEARCH_AUDIT_REJECTED,
    SEARCH_BUDGET_EXHAUSTED,
    SEARCH_NO_MOTION,
    SEARCH_NO_SOLUTION,
    SEARCH_SUCCESS,
    SuccessorConfig,
    UniformGraphSearch,
    UniformGraphSearchResult,
    initial_state,
)
from .uniform_trajectory import UniformTimedPlan, _edge_record, _validate_initial_edge
from .uniform_timing import MotionTiming

PIPELINE_REQUEST_REJECTED = "REQUEST_REJECTED"


def _canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      allow_nan=False)


def _sha256_text(value):
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def _pose_list(pose):
    return [float(pose.x), float(pose.y), float(pose.psi)]


def _heading_records(headings):
    return None if headings is None else [
        {"vehicle_id": int(vehicle_id), "theta": float(theta)}
        for vehicle_id, theta in sorted(headings.items())]


@dataclass(frozen=True)
class UniformPlanningRequest:
    """One complete offline FR04 planning request."""

    grid: Grid
    model: FormationModel
    vehicle_shapes: Mapping[int, VehicleShape]
    start_qf: FormationPose
    start_headings: Mapping[int, float]
    goal_qf: FormationPose
    goal_headings: Optional[Mapping[int, float]] = None
    plan_id: int = 0
    formation_id: str = "uniform_v02"
    successor_config: SuccessorConfig = field(default_factory=SuccessorConfig)
    search_config: GraphSearchConfig = field(default_factory=GraphSearchConfig)
    initial_edge: Optional[UniformMotionEdge] = None
    cancelled: Optional[Callable[[], bool]] = None
    cargo_footprint: Optional[CargoFootprint] = None


@dataclass
class UniformPlanningResult:
    """Executable plan only for SUCCESS/NO_MOTION with a FEASIBLE FR02 audit."""

    status: str
    feasible: bool
    reason: str
    detail: str
    first_failure_layer: str
    request_sha256: str
    wall_time_s: float
    search: Optional[UniformGraphSearchResult] = None
    plan: Optional[UniformTimedPlan] = None
    projection: object = None
    evaluation: object = None
    request_inputs: dict = field(default_factory=dict)

    @property
    def executable(self):
        return bool(self.feasible and self.plan is not None
                    and self.evaluation is not None
                    and self.evaluation.feasible)

    def as_dict(self, include_search_path=True):
        search = None if self.search is None else self.search.as_dict(
            include_path=include_search_path)
        evaluation = None
        if self.evaluation is not None:
            evaluation = self.evaluation.as_status()
            evaluation["metrics"] = dict(self.evaluation.metrics)
            evaluation["checks"] = self.evaluation.checks
        return {
            "schema": "fr04.uniform_planning_result.v1",
            "status": self.status,
            "feasible": bool(self.feasible),
            "executable": bool(self.executable),
            "reason": self.reason,
            "detail": self.detail,
            "first_failure_layer": self.first_failure_layer,
            "request_sha256": self.request_sha256,
            "wall_time_s": self.wall_time_s,
            "duration_s": None if self.plan is None else self.plan.duration_s,
            "search": search,
            "evaluation": evaluation,
            "plan_model": None if self.plan is None else self.plan.to_model_dict(),
            "projection_summary": self.projection_summary(),
        }

    def projection_summary(self):
        if self.projection is None:
            return None
        return {
            "semantic_version": self.projection.semantic_version,
            "model_version": self.projection.model_version,
            "model_sha256": self.projection.model_sha256,
            "sample_count": len(self.projection.times),
            "times_from_start": list(self.projection.times),
            "formation_poses": [
                _pose_list(pose) for pose in
                self.projection.timed_trajectory.formation_poses],
        }


def request_fingerprint(request):
    """Stable hashes for map, model, request, successor and budget configs."""
    grid = request.grid
    grid_payload = {
        "shape": list(grid.occ.shape),
        "resolution": grid.res,
        "origin": [grid.ox, grid.oy],
        "occupied_sha256": hashlib.sha256(
            grid.occ.tobytes(order="C")).hexdigest(),
    }
    model_payload = [
        {"vehicle_id": offset.vehicle_id, "x": offset.x, "y": offset.y}
        for offset in request.model.offsets]
    shape_payload = [
        {"vehicle_id": int(vehicle_id),
         "length_m": request.vehicle_shapes[vehicle_id].length_m,
         "width_m": request.vehicle_shapes[vehicle_id].width_m}
        for vehicle_id in sorted(request.vehicle_shapes)]
    search_config = asdict(request.search_config)
    if not search_config["continuous_a_refinement"]:
        del search_config["continuous_a_refinement"]
    if not search_config["direct_goal_translation"]:
        # Preserve the fingerprint of requests using the original search.
        del search_config["direct_goal_translation"]
        del search_config["global_translation_route"]
        del search_config["preview_checks"]
    if not search_config.get("global_translation_route", False):
        del search_config["heading_aligned_route"]
        del search_config["heading_transition_route"]
        del search_config["formation_raster"]
    successor_config = asdict(request.successor_config)
    if not successor_config["goal_formation_heading_free"]:
        del successor_config["goal_formation_heading_free"]
    inputs = {
        "plan_id": request.plan_id,
        "formation_id": request.formation_id,
        "start_qf": _pose_list(request.start_qf),
        "start_headings": _heading_records(request.start_headings),
        "goal_qf": _pose_list(request.goal_qf),
        "goal_headings": _heading_records(request.goal_headings),
        "grid": grid_payload,
        "model": model_payload,
        "vehicle_shapes": shape_payload,
        "successor_config": successor_config,
        "search_config": search_config,
    }
    if request.initial_edge is not None:
        inputs["previous_terminal_edge"] = _edge_record(request.initial_edge)
    if request.cargo_footprint is not None:
        inputs["cargo_footprint"] = asdict(request.cargo_footprint)
    return _sha256_text(_canonical_json(inputs)), inputs


def evaluation_detail(evaluation):
    failure = evaluation.failure
    return {
        "status": evaluation.status,
        "feasible": bool(evaluation.feasible),
        "reason": evaluation.reason,
        "detail": evaluation.detail,
        "failure_code": None if failure is None else failure.code,
        "failure_constraint": None if failure is None else failure.constraint,
        "failure_vehicle_id": None if failure is None else failure.vehicle_id,
        "failure_detail": None if failure is None else failure.detail,
        "duration_s": evaluation.metrics.get("duration_s"),
    }


_BINDING_POSITION_TOL_M = 1e-9
_BINDING_ANGLE_TOL_RAD = 1e-9


class _ContractError(Exception):
    """A request/expected-path contract mismatch detected before auditing."""

    def __init__(self, code, field_name, detail, vehicle_id=None):
        super().__init__(detail)
        self.code = code
        self.field = field_name
        self.detail = detail
        self.vehicle_id = vehicle_id


@dataclass(frozen=True)
class _ExpectedContract:
    """The request-bound identity a plan must reproduce exactly."""

    audit_request: UniformAuditRequest
    edges: tuple
    zero_motion: bool
    goal_qf: FormationPose
    goal_headings: dict


def _wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _offset_key(model):
    return tuple((int(offset.vehicle_id), float(offset.x), float(offset.y))
                 for offset in model.offsets)


def _headings_dict(values):
    return {int(vehicle_id): float(theta)
            for vehicle_id, theta in dict(values).items()}


def _require_pose(actual, expected, context, code):
    if not isinstance(actual, FormationPose):
        raise _ContractError(code, context,
                             "{} is not a FormationPose".format(context))
    if not isinstance(expected, FormationPose):
        raise _ContractError(code, context,
                             "{} expected is not a FormationPose".format(context))
    for name, actual_value, expected_value in (
            ("x", actual.x, expected.x),
            ("y", actual.y, expected.y),
            ("psi", actual.psi, expected.psi)):
        tolerance = _BINDING_POSITION_TOL_M
        if name == "psi":
            error = abs(_wrap_angle(
                float(actual_value) - float(expected_value)))
            tolerance = _BINDING_ANGLE_TOL_RAD
        else:
            error = abs(float(actual_value) - float(expected_value))
        if error > tolerance:
            raise _ContractError(
                code, "{}.{}".format(context, name),
                "{} {}={:.17g} differs from expected {:.17g} by {:.3g} > "
                "{:.3g}".format(context, name, actual_value, expected_value,
                                error, tolerance))


def _require_headings(actual, expected, context, code):
    try:
        actual_map = _headings_dict(actual)
    except (TypeError, ValueError):
        raise _ContractError(code, context,
                             "{} is not a heading mapping".format(context))
    expected_map = _headings_dict(expected)
    if set(actual_map) != set(expected_map):
        raise _ContractError(
            code, context, "{} vehicle IDs differ: actual={} expected={}"
            .format(context, sorted(actual_map), sorted(expected_map)))
    for vehicle_id in sorted(expected_map):
        error = abs(_wrap_angle(
            actual_map[vehicle_id] - expected_map[vehicle_id]))
        if error > _BINDING_ANGLE_TOL_RAD:
            raise _ContractError(
                code, "{}.{}".format(context, vehicle_id),
                "{} vehicle_id={} theta={:.17g} differs from expected "
                "{:.17g} by {:.3g} > {:.3g}".format(
                    context, vehicle_id, actual_map[vehicle_id],
                    expected_map[vehicle_id], error,
                    _BINDING_ANGLE_TOL_RAD), vehicle_id=vehicle_id)


def _check_goal_tolerance(request, expected_goal_qf):
    config = request.successor_config
    distance = math.hypot(expected_goal_qf.x - request.goal_qf.x,
                          expected_goal_qf.y - request.goal_qf.y)
    if distance > config.goal_xy_tol_m:
        raise _ContractError(
            "REQUEST_GOAL_TOLERANCE_EXCEEDED", "goal_qf.xy",
            "expected endpoint x={:.17g} y={:.17g} is {:.6g} m from the "
            "requested x={:.17g} y={:.17g}; frozen goal_xy_tol_m={:.6g}"
            .format(expected_goal_qf.x, expected_goal_qf.y, distance,
                    request.goal_qf.x, request.goal_qf.y,
                    config.goal_xy_tol_m))
    psi_error = abs(_wrap_angle(expected_goal_qf.psi - request.goal_qf.psi))
    if not config.goal_formation_heading_free and psi_error > config.goal_psi_tol_rad:
        raise _ContractError(
            "REQUEST_GOAL_TOLERANCE_EXCEEDED", "goal_qf.psi",
            "expected endpoint psi={:.17g} is {:.6g} rad from the requested "
            "{:.17g}; frozen goal_psi_tol_rad={:.6g}".format(
                expected_goal_qf.psi, psi_error, request.goal_qf.psi,
                config.goal_psi_tol_rad))


def _check_terminal_headings(request, expected_headings):
    """Only enforce terminal body headings when the request enables them.

    Default free terminal headings add no equality requirement.  The exact
    edge headings still bind the plan to the authoritative path.
    """
    config = request.successor_config
    if config.goal_heading_free:
        return
    if request.goal_headings is None:
        return
    actual = _headings_dict(expected_headings)
    requested = _headings_dict(request.goal_headings)
    for vehicle_id in request.model.vehicle_ids:
        error = abs(_wrap_angle(actual[vehicle_id]
                                - requested[vehicle_id]))
        if error > config.goal_heading_tol_rad:
            raise _ContractError(
                "REQUEST_TERMINAL_HEADINGS_MISMATCH",
                "goal_headings.{}".format(vehicle_id),
                "expected terminal heading vehicle_id={} theta={:.17g} is "
                "{:.6g} rad from the requested {:.17g}; frozen "
                "goal_heading_tol_rad={:.6g}".format(
                    vehicle_id, actual[vehicle_id], error,
                    requested[vehicle_id],
                    config.goal_heading_tol_rad), vehicle_id=vehicle_id)


def _expected_contract(request, edges, zero_motion):
    """Bind request identity and the authoritative edges before construction."""
    edges = tuple(edges)
    config = request.successor_config
    if zero_motion:
        if edges:
            raise _ContractError(
                "ZERO_MOTION_EDGES_PRESENT", "edges",
                "a zero-motion hold cannot carry {} authoritative edges"
                .format(len(edges)))
        expected_goal_qf = request.start_qf
        expected_goal_headings = _headings_dict(request.start_headings)
    else:
        if not edges:
            raise _ContractError(
                "EMPTY_AUTHORITATIVE_EDGES", "edges",
                "a moving plan needs the authoritative edge sequence")
        for index, edge in enumerate(edges):
            if not isinstance(edge, UniformMotionEdge):
                raise _ContractError(
                    "INVALID_AUTHORITATIVE_EDGE", "edges[{}]".format(index),
                    "authoritative edge {} has type {}".format(
                        index, type(edge).__name__))
            if _offset_key(edge.model) != _offset_key(request.model):
                raise _ContractError(
                    "REQUEST_EDGE_MODEL_MISMATCH",
                    "edges[{}].model".format(index),
                    "authoritative edge {} offsets differ from the request "
                    "model".format(index))
        _require_pose(edges[0].start_qf, request.start_qf,
                      "edges[0].start_qf", "REQUEST_START_EDGE_MISMATCH")
        _require_headings(edges[0].start_headings, request.start_headings,
                          "edges[0].start_headings",
                          "REQUEST_START_EDGE_MISMATCH")
        for index in range(1, len(edges)):
            _require_pose(edges[index].start_qf, edges[index - 1].end_qf,
                          "edges[{}].start_qf".format(index),
                          "REQUEST_EDGE_SEQUENCE_DISCONTINUITY")
            _require_headings(edges[index].start_headings,
                              edges[index - 1].end_headings,
                              "edges[{}].start_headings".format(index),
                              "REQUEST_EDGE_SEQUENCE_DISCONTINUITY")
        expected_goal_qf = edges[-1].end_qf
        expected_goal_headings = _headings_dict(edges[-1].end_headings)
    _check_goal_tolerance(request, expected_goal_qf)
    _check_terminal_headings(request, expected_goal_headings)
    audit_request = UniformAuditRequest(
        plan_id=request.plan_id,
        formation_id=request.formation_id,
        model=request.model,
        vehicle_shapes=dict(request.vehicle_shapes),
        start_qf=request.start_qf,
        start_headings=_headings_dict(request.start_headings),
        goal_qf=expected_goal_qf,
        goal_headings=dict(expected_goal_headings),
        timing_limits=config.timing_limits,
        allow_reverse=config.allow_reverse, initial_edge=request.initial_edge,
        cargo_footprint=request.cargo_footprint)
    return _ExpectedContract(
        audit_request=audit_request, edges=edges,
        zero_motion=bool(zero_motion), goal_qf=expected_goal_qf,
        goal_headings=dict(expected_goal_headings))


def expected_audit_request(request, edges, zero_motion=False):
    """Publicly build the independently request-bound FR02 audit request."""
    return _expected_contract(request, edges, zero_motion).audit_request


def _verify_plan(plan, contract, request):
    """Check the constructed plan reproduces the request/edge contract."""
    if not isinstance(plan, UniformTimedPlan):
        raise _ContractError(
            "PLAN_IDENTITY_MISMATCH", "plan",
            "plan is not a UniformTimedPlan")
    if plan.plan_id != request.plan_id:
        raise _ContractError(
            "PLAN_IDENTITY_MISMATCH", "plan_id",
            "plan_id={!r} differs from the request {!r}".format(
                plan.plan_id, request.plan_id))
    if plan.formation_id != request.formation_id:
        raise _ContractError(
            "PLAN_IDENTITY_MISMATCH", "formation_id",
            "formation_id={!r} differs from the request {!r}".format(
                plan.formation_id, request.formation_id))
    if _offset_key(plan.model) != _offset_key(request.model):
        raise _ContractError(
            "PLAN_IDENTITY_MISMATCH", "offsets",
            "plan offsets differ from the request model")
    if set(plan.vehicle_shapes) != set(request.vehicle_shapes):
        raise _ContractError(
            "PLAN_IDENTITY_MISMATCH", "vehicle_shapes",
            "plan vehicle shape IDs differ from the request")
    for vehicle_id in request.model.vehicle_ids:
        actual = plan.vehicle_shapes[vehicle_id]
        expected = request.vehicle_shapes[vehicle_id]
        if not isinstance(actual, VehicleShape):
            raise _ContractError(
                "PLAN_IDENTITY_MISMATCH",
                "vehicle_shapes.{}".format(vehicle_id),
                "plan shape is not a VehicleShape", vehicle_id)
        for name in ("length_m", "width_m"):
            if abs(float(getattr(actual, name))
                   - float(getattr(expected, name))) > (
                       _BINDING_POSITION_TOL_M):
                raise _ContractError(
                    "PLAN_IDENTITY_MISMATCH",
                    "vehicle_shapes.{}.{}".format(vehicle_id, name),
                    "plan vehicle_id={} {} differs from the request".format(
                        vehicle_id, name), vehicle_id)
    if plan.limits != request.successor_config.timing_limits:
        raise _ContractError(
            "PLAN_IDENTITY_MISMATCH", "timing_limits",
            "plan timing limits differ from the request")
    _require_pose(plan.start_qf, request.start_qf, "plan.start_qf",
                  "PLAN_START_MISMATCH")
    _require_headings(plan.start_headings, request.start_headings,
                      "plan.start_headings", "PLAN_START_MISMATCH")
    if plan.initial_edge != request.initial_edge:
        raise _ContractError("PLAN_START_MISMATCH", "plan.initial_edge",
                             "plan changed the confirmed initial wheel state")
    _require_pose(plan.goal_qf, contract.goal_qf, "plan.goal_qf",
                  "PLAN_GOAL_MISMATCH")
    _require_headings(plan.goal_headings, contract.goal_headings,
                      "plan.goal_headings", "PLAN_GOAL_MISMATCH")
    if contract.zero_motion:
        if plan.edges:
            raise _ContractError(
                "PLAN_EDGE_SEQUENCE_MISMATCH", "plan.edges",
                "zero-motion contract but the plan carries {} edges".format(
                    len(plan.edges)))
        if plan.duration_s != 0.0:
            raise _ContractError(
                "PLAN_EDGE_SEQUENCE_MISMATCH", "plan.duration_s",
                "zero-motion plan duration is {:.17g}, not 0".format(
                    plan.duration_s))
    else:
        if len(plan.edges) != len(contract.edges):
            raise _ContractError(
                "PLAN_EDGE_SEQUENCE_MISMATCH", "plan.edges",
                "plan carries {} edges but the authoritative sequence carries "
                "{}; equal endpoints alone do not bind the path".format(
                    len(plan.edges), len(contract.edges)))
        for index, (actual, expected) in enumerate(
                zip(plan.edges, contract.edges)):
            if actual != expected:
                raise _ContractError(
                    "PLAN_EDGE_SEQUENCE_MISMATCH",
                    "plan.edges[{}]".format(index),
                    "plan edge {} (mode={}, primitive={}) is not the "
                    "authoritative input edge (mode={}, primitive={})".format(
                        index, actual.mode, actual.primitive, expected.mode,
                        expected.primitive))


def _contract_failure(request, exc):
    return FormationEvaluation(
        plan_id=request.plan_id,
        formation_id=request.formation_id,
        status=REJECTED,
        feasible=False,
        reason=exc.code,
        detail=exc.detail,
        failure=FormationEvaluationFailure(
            code=exc.code, field=exc.field, vehicle_id=exc.vehicle_id,
            detail=exc.detail),
        metrics={},
        checks=0)


def _retime_unconfirmed_phases(plan, evaluation):
    """Scale only affected motion clocks using the independent audit bounds."""
    if evaluation.reason != UNIFORM_DYNAMIC_UNCONFIRMED:
        return None, {}
    orders = {name: order for name, _, order in _DYNAMIC_LIMITS}
    factors = {}
    for row in evaluation.metrics.get("dynamic_confirmation", {}).get("unconfirmed", ()):
        index = row["block_index"]
        ratio = row["adopted_bound"] / row["limit"]
        order = orders.get(row["constraint"])
        if order is None or not math.isfinite(ratio) or ratio <= 1.0:
            return None, {}
        scale = 1.0001 * ratio ** (1.0 / order)
        factors[index] = max(scale, factors.get(index, 1.0))
    if not factors:
        return None, {}

    phases = []
    time_s = 0.0
    for phase in plan.phases:
        duration = phase.duration_s
        if isinstance(phase, MotionTiming) and phase.edge_index in factors:
            duration *= factors[phase.edge_index]
            if duration > plan.limits.max_block_duration_s:
                return None, {}
        shifted = replace(phase, t0_s=time_s, duration_s=duration)
        phases.append(shifted)
        time_s = shifted.end_time
    return replace(plan, phases=tuple(phases), duration_s=time_s), factors


def _build_and_audit(edges, request, zero_motion=False):
    """Construct and audit under a contract fixed before construction.

    IDs, formation/model, shapes, limits, allow_reverse, exact start and
    initial headings come from the original request.  The exact goal and
    terminal headings come from the pre-fixed authoritative edge sequence
    (or the request start for a hold), after a separate frozen-tolerance goal
    check against the original request.  The resulting plan must reproduce
    the authoritative sequence, not merely the same endpoints.
    """
    try:
        contract = _expected_contract(request, edges, zero_motion)
    except _ContractError as exc:
        return None, None, _contract_failure(request, exc)
    except FormationValidationError as exc:
        return None, None, _contract_failure(request, _ContractError(
            "REQUEST_BINDING_INVALID", "request",
            "request binding input is invalid: {}".format(exc)))

    try:
        if zero_motion:
            plan = UniformTimedPlan.from_edges(
                (), request.vehicle_shapes, plan_id=request.plan_id,
                formation_id=request.formation_id,
                timing_limits=request.successor_config.timing_limits,
                model=request.model, start_qf=request.start_qf,
                start_headings=_headings_dict(request.start_headings),
                expected_end_qf=contract.goal_qf,
                expected_end_headings=dict(contract.goal_headings), initial_edge=request.initial_edge)
        else:
            plan = UniformTimedPlan.from_edges(
                contract.edges, request.vehicle_shapes,
                plan_id=request.plan_id,
                formation_id=request.formation_id,
                timing_limits=request.successor_config.timing_limits,
                model=request.model, start_qf=request.start_qf,
                start_headings=_headings_dict(request.start_headings),
                expected_end_qf=contract.goal_qf,
                expected_end_headings=dict(contract.goal_headings), initial_edge=request.initial_edge)
    except _ContractError as exc:
        return None, None, _contract_failure(request, exc)
    except FormationValidationError as exc:
        return None, None, _contract_failure(request, _ContractError(
            "PLAN_CONSTRUCTION_INVALID", "plan",
            "plan construction was rejected: {}".format(exc)))

    try:
        _verify_plan(plan, contract, request)
        projection = plan.sample()
        audit_config = FormationQualityConfig(
            timing_limits=request.successor_config.timing_limits,
            collision_margin_m=max(FormationQualityConfig.collision_margin_m,
                                   request.successor_config.collision_margin_m))
        evaluation = audit_uniform_timed_plan(
            contract.audit_request, plan, projection, request.grid, audit_config)
        retimed, factors = _retime_unconfirmed_phases(plan, evaluation)
        if retimed is not None:
            _verify_plan(retimed, contract, request)
            new_projection = retimed.sample()
            recertification = audit_uniform_timed_plan(
                contract.audit_request, retimed, new_projection, request.grid, audit_config)
            if recertification.feasible:
                evaluation = replace(
                    recertification,
                    metrics=dict(recertification.metrics, audit_retiming_factors=factors))
            else:
                evaluation = recertification
            plan, projection = retimed, new_projection
    except _ContractError as exc:
        return None, None, _contract_failure(request, exc)
    except FormationValidationError as exc:
        return None, None, _contract_failure(request, _ContractError(
            "PLAN_CONSTRUCTION_INVALID", "plan",
            "plan/sample construction was rejected: {}".format(exc)))
    return plan, projection, evaluation


def _goal_center_outside(grid, model, goal_qf):
    max_x = grid.ox + grid.width * grid.res
    max_y = grid.oy + grid.height * grid.res
    for x, y in model.centers(goal_qf).values():
        if x < grid.ox or x > max_x or y < grid.oy or y > max_y:
            return True
    return False


def plan_uniform(request):
    """Run one autonomous, budgeted FR04 request without any reference seed."""
    started = time.perf_counter()
    request_sha256, inputs = request_fingerprint(request)
    checker = FormationCollisionChecker(
        request.grid, request.model, request.vehicle_shapes,
        request.successor_config.collision_margin_m, cargo_footprint=request.cargo_footprint)

    def result(status, feasible, reason, layer, detail="", search=None,
               plan=None, projection=None, evaluation=None):
        return UniformPlanningResult(
            status=status, feasible=bool(feasible), reason=reason,
            detail=detail, first_failure_layer=layer,
            request_sha256=request_sha256,
            wall_time_s=time.perf_counter() - started, search=search,
            plan=plan, projection=projection, evaluation=evaluation,
            request_inputs=inputs)

    start_report = checker.static_check(request.start_qf, request.start_headings)
    if not start_report.success:
        return result(
            PIPELINE_REQUEST_REJECTED, False,
            start_report.reason or "START_STATIC_COLLISION", "request",
            "start static check failed: vehicle_id={} detail={}".format(
                start_report.vehicle_id, start_report.detail))

    fixed_goal_heading = not request.successor_config.goal_formation_heading_free
    if fixed_goal_heading and _goal_center_outside(request.grid, request.model, request.goal_qf):
        return result(
            PIPELINE_REQUEST_REJECTED, False, "GOAL_CENTER_OUT_OF_BOUNDS",
            "request", "at least one rigid goal vehicle centre is outside the map")

    if fixed_goal_heading and not checker.cargo_map_clear(request.goal_qf):
        return result(PIPELINE_REQUEST_REJECTED, False, "CARGO_OBB_COLLISION", "request",
                      "goal cargo envelope intersects occupied map or boundary")

    if fixed_goal_heading and request.goal_headings is not None:
        goal_report = checker.static_check(
            request.goal_qf, request.goal_headings)
        if not goal_report.success:
            return result(
                PIPELINE_REQUEST_REJECTED, False,
                goal_report.reason or "GOAL_STATIC_COLLISION", "request",
                "goal static check failed: vehicle_id={} detail={}".format(
                    goal_report.vehicle_id, goal_report.detail))

    if request.initial_edge is not None:
        try:
            _validate_initial_edge(request.initial_edge, request.model, request.start_qf,
                                   tuple(sorted(request.start_headings.items())),
                                   request.successor_config.timing_limits)
        except FormationValidationError as exc:
            return result(PIPELINE_REQUEST_REJECTED, False, "INVALID_CONTINUATION", "request", str(exc))
    initial = initial_state(
        request.model, request.start_qf, request.start_headings,
        config=request.successor_config, goal_qf=request.goal_qf, initial_edge=request.initial_edge)
    search = UniformGraphSearch(
        request.model, checker, request.successor_config,
        request.search_config)

    def terminal_validator(edges):
        _, _, evaluation = _build_and_audit(edges, request)
        return evaluation.feasible, evaluation_detail(evaluation)

    search_result = search.search(
        initial, request.goal_qf, terminal_validator=terminal_validator,
        goal_headings=request.goal_headings, cancelled=request.cancelled)

    if search_result.status == SEARCH_SUCCESS:
        # Rebuild and independently re-audit the returned path.  The search
        # terminal audit was a candidate screen; this final audit is the
        # pipeline's executable certificate.
        plan, projection, evaluation = _build_and_audit(
            search_result.edges, request)
        if not evaluation.feasible:
            return result(
                SEARCH_AUDIT_REJECTED, False,
                evaluation.failure.code if evaluation.failure else
                evaluation.reason,
                "audit", "final rebuilt-path audit rejected a path that the "
                "terminal screen had accepted", search_result,
                plan, projection, evaluation)
        if request.search_config.continuous_a_refinement:
            from .uniform_curve_refinement import refine_turn

            refinement_request = request
            if not fixed_goal_heading:
                refinement_request = replace(request, goal_qf=FormationPose(
                    request.goal_qf.x, request.goal_qf.y, plan.goal_qf.psi))
            improved = refine_turn(refinement_request, plan, _build_and_audit,
                                   started + request.search_config.max_seconds)
            if improved is not None:
                plan, projection, evaluation = improved
                return result(SEARCH_SUCCESS, True, evaluation.reason, "none",
                              "continuous A refined and audited on the common A/O/S timeline",
                              search_result, plan, projection, evaluation)
        return result(
            SEARCH_SUCCESS, True, evaluation.reason, "none",
            "autonomous path rebuilt and independently audited FEASIBLE",
            search_result, plan, projection, evaluation)

    if search_result.status == SEARCH_NO_MOTION:
        plan, projection, evaluation = _build_and_audit(
            (), request, zero_motion=True)
        return result(
            SEARCH_NO_MOTION, evaluation.feasible, evaluation.reason,
            "none" if evaluation.feasible else "audit",
            "exact zero-motion hold audited by FR02", search_result,
            plan, projection, evaluation)

    layer = search_result.first_failure_layer
    if search_result.status == SEARCH_BUDGET_EXHAUSTED:
        layer = "budget"
    elif search_result.status == SEARCH_AUDIT_REJECTED:
        layer = "audit"
    elif search_result.status == SEARCH_NO_SOLUTION:
        layer = "search"
    return result(
        search_result.status, False, search_result.reason, layer,
        search_result.detail, search_result)
