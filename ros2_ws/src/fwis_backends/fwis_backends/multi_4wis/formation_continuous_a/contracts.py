"""Finite data contracts for the virtual rigid formation A-only model (G1).

These objects are new.  Formation geometry, poses and the research offsets are
reused from :mod:`fwis_v3_optimizer.formation`; no legacy type is replaced.
Every value is immutable and finite; rejections carry a stable reason code and
a structured location instead of an anonymous exception.
"""
from collections.abc import Mapping as MappingABC
from dataclasses import dataclass, field
import enum
import hashlib
import json
import math
import numbers
import struct

from ..formation import FormationModel, FormationPose, VehicleOffset
from ..formation_collision import VehicleShape
from ..formation_retiming import FormationTimingLimits


SCHEMA = "virtual_rigid_a.g1.contracts.v1"
G2_SCHEMA = "virtual_rigid_a.g2.contracts.v1"
G3_SCHEMA = "virtual_rigid_a.g3.contracts.v1"
G4_SCHEMA = "virtual_rigid_a.g4.contracts.v1"
G4_PLANNING_SCHEMA = "virtual_rigid_a.g4.planning_context.v1"
REQUEST_SCHEMA = "virtual_rigid_a.g2.request.v1"
# TASK 01 Step 1 display contract: the pipeline emits read-only stage events,
# the planner node adds admission identity, and only the node may report
# PUBLISHED after the atomic playback envelope is actually on the wire.
PROGRESS_SCHEMA = "virtual_a_progress/v1"
PROGRESS_STAGES = (
    "REQUEST_ADMITTED",
    "ASTAR_SEARCH",
    "CORRIDOR_BUILD",
    "PORTAL_GUIDE",
    "NLP_SOLVE",
    "GEOMETRY_AUDIT",
    "DYNAMICS_AUDIT",
    "OBB_COLLISION_AUDIT",
    "PUBLISHED",
)
PROGRESS_STAGE_INDEX = {
    stage: index for index, stage in enumerate(PROGRESS_STAGES)}
PROGRESS_STATES = frozenset(("PLANNING", "FAILED", "PUBLISHED"))
PROGRESS_FAILED_STAGE = "FAILED"
GEOMETRY_SCOPE = "GEOMETRY"
DYNAMICS_SCOPE = "DYNAMICS"
COLLISION_SCOPE = "COLLISION"
BACKEND = "virtual_rigid_a"
FOOTPRINT_GUIDANCE = "footprint_seed"
TIMED_MODEL_VERSION = "virtual_rigid_a.timed.v1"
PATH_MODEL_VERSION = "virtual_rigid_a.path.v1"
PLAYBACK_SEMANTIC_VERSION = "virtual_rigid_a.playback.v1"
PLAYBACK_CERTIFICATION = "virtual_a_geometry_dynamics_collision/v1"
TIMING_LAW_VERSION = "virtual_rigid_a.timing.quintic_smoothstep.v1"
TIMING_STATUSES = (
    "DYNAMIC_FEASIBLE", "NO_MOTION", "INVALID_REQUEST", "INVALID_PATH",
    "INVALID_TIMING_CONFIG", "GEOMETRY_REJECTED", "TIMING_BUDGET_EXCEEDED",
    "BOUND_UNCONFIRMED", "NUMERICAL_FAILURE",
)
GEOMETRY_STATUSES = (
    "GEOMETRY_FEASIBLE", "NO_MOTION", "INVALID_REQUEST",
    "NO_ACCEPTED_CANDIDATE", "BUDGET_EXHAUSTED", "NUMERICAL_FAILURE",
)
G4_RESULT_STATUSES = (
    "SUCCESS", "NO_MOTION", "INVALID_REQUEST", "GEOMETRY_FAILED",
    "DYNAMICS_FAILED", "COLLISION_FAILED", "NO_ACCEPTED_CANDIDATE",
    "BUDGET_EXHAUSTED", "NUMERICAL_FAILURE",
)
G4_EXECUTABLE_STATUSES = ("SUCCESS", "NO_MOTION")
RESEARCH_VEHICLE_SHAPE_M = (0.738, 0.500)
SHAPE_TOLERANCE_M = 1e-12
DEFAULT_GEOMETRY_POSITION_TOL_M = 5e-3
DEFAULT_GEOMETRY_HEADING_TOL_RAD = math.radians(0.2)
DEFAULT_GEOMETRY_RIGIDITY_TOL_M = 1e-6
DEFAULT_INITIAL_LENGTH_MULTIPLIERS = (1.2, 1.8, 2.6)
MAX_INITIALS = 6
INITIAL_SOURCES = ("offline_fixture", "rviz_reset", "verified_terminal")
ROS_INITIAL_SOURCES = ("rviz_reset", "verified_terminal")
INITIAL_HEADING_TOLERANCE_RAD = 1e-7
INITIAL_CURVATURE_TOLERANCE = 1e-12
RESEARCH_VEHICLE_IDS = (1, 2, 3)
RESEARCH_OFFSETS = ((1, 1.0, 0.0), (2, 0.0, 1.0), (3, 0.0, -1.0))
FORMATION_TOLERANCE_M = 1e-12

# TASK03: versioned hard-corridor scope and finite OBB coverage templates.
FORMATION_CORRIDOR_SCOPE_F_ONLY = "f_only"
FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1 = "three_vehicle_union_v1"
FORMATION_CORRIDOR_SCOPES = (
    FORMATION_CORRIDOR_SCOPE_F_ONLY,
    FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
)
# Template schema v2 adds the per-segment F reference-point region assignment.
# The schema/compiler bump makes every v1 payload fail closed instead of being
# silently reinterpreted as if it carried an F assignment.  The certificate
# payload itself does not change; it binds the template id, which now covers
# the F assignment as well.
FORMATION_CORRIDOR_TEMPLATE_SCHEMA = "formation_corridor_template.v2"
FORMATION_CORRIDOR_COMPILER_VERSION = "formation_corridor_rows.v2"
FORMATION_CORRIDOR_CERTIFICATE_VERSION = "formation_corridor_certificate.v1"
FORMATION_CORRIDOR_DECOMPOSITIONS = ("whole", "diagonal_0", "diagonal_1")
FORMATION_CORRIDOR_STATUS_VERIFIED = "FORMATION_CORRIDOR_VERIFIED"
FORMATION_CORRIDOR_STATUS_VIOLATED = "FORMATION_CORRIDOR_VIOLATED"
FORMATION_CORRIDOR_STATUS_UNCONFIRMED = "FORMATION_CORRIDOR_UNCONFIRMED"
FORMATION_CORRIDOR_STATUS_TEMPLATE_INSUFFICIENT = (
    "FORMATION_CORRIDOR_TEMPLATE_INSUFFICIENT")
FORMATION_CORRIDOR_STATUS_F_ONLY_GATE_BLOCKED = (
    "FORMATION_CORRIDOR_F_ONLY_GATE_BLOCKED")
# The OBB map requires the normal branch A>0 and D>0.  CasADi must not
# silently evaluate a corner through atan2(0, 0) or a negative A.  Every OBB
# scalar row carries this smooth guard (identical numeric/symbolic form), so a
# degenerate vehicle state cannot satisfy the rows by accident.  The tiny
# positive epsilon keeps the branch open; the penalty makes a violation
# unambiguously infeasible without changing the row count.
FORMATION_NORMAL_BRANCH_EPS = 1e-9
FORMATION_NORMAL_BRANCH_PENALTY = 1.0e12
FORMATION_CORRIDOR_STATUSES = (
    FORMATION_CORRIDOR_STATUS_VERIFIED,
    FORMATION_CORRIDOR_STATUS_VIOLATED,
    FORMATION_CORRIDOR_STATUS_UNCONFIRMED,
    FORMATION_CORRIDOR_STATUS_TEMPLATE_INSUFFICIENT,
    FORMATION_CORRIDOR_STATUS_F_ONLY_GATE_BLOCKED,
)


class VirtualAContractError(ValueError):
    """Structured rejection of an invalid A-only contract value."""

    def __init__(self, reason, detail="", **location):
        self.reason = str(reason)
        self.detail = str(detail)
        self.location = dict(location)
        message = self.reason
        if self.detail:
            message += ": " + self.detail
        if self.location:
            message += " " + repr(sorted(self.location.items()))
        super().__init__(message)

    def as_dict(self):
        return {
            "reason": self.reason,
            "detail": self.detail,
            "location": dict(sorted(self.location.items())),
        }


def _wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def _finite(value, context):
    if isinstance(value, (bool, str, bytes)):
        raise VirtualAContractError(
            "NONFINITE_VALUE", "{} value={!r}".format(context, value))
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise VirtualAContractError(
            "NONFINITE_VALUE", "{} value={!r}".format(context, value)) from exc
    if not math.isfinite(result):
        raise VirtualAContractError(
            "NONFINITE_VALUE", "{} value={!r}".format(context, value))
    return result


class GoalPolicy(str, enum.Enum):
    """Terminal curvature policy; the first edition has exactly two values."""

    FREE = "FREE"
    STRAIGHT = "STRAIGHT"


def coerce_goal_policy(value):
    if isinstance(value, GoalPolicy):
        return value
    if isinstance(value, str) and value in GoalPolicy.__members__:
        return GoalPolicy[value]
    raise VirtualAContractError(
        "UNSUPPORTED_GOAL_POLICY", "value={!r}".format(value))


@dataclass(frozen=True)
class InitialAssumption:
    """Explicit experiment initial-state declaration; never a measurement.

    ``offline_fixture`` is the only source a G1 offline test may use.
    ``rviz_reset``/``verified_terminal`` describe ROS experiments and require
    the non-empty session ``context_id`` that only the G5 node can verify.
    """

    stationary: bool
    mode: str
    wheels_straight: bool
    source: str
    context_id: str = ""

    def __post_init__(self):
        if not isinstance(self.stationary, bool):
            raise VirtualAContractError(
                "INVALID_INITIAL_ASSUMPTION", "stationary must be bool")
        if not isinstance(self.wheels_straight, bool):
            raise VirtualAContractError(
                "INVALID_INITIAL_ASSUMPTION", "wheels_straight must be bool")
        if not isinstance(self.mode, str) or not self.mode.strip():
            raise VirtualAContractError(
                "INVALID_INITIAL_ASSUMPTION", "mode must be a non-empty string")
        if not isinstance(self.source, str):
            raise VirtualAContractError(
                "INVALID_INITIAL_ASSUMPTION", "source must be a string")
        if self.source not in INITIAL_SOURCES:
            raise VirtualAContractError(
                "UNKNOWN_INITIAL_SOURCE", "source={!r} allowed={}".format(
                    self.source, list(INITIAL_SOURCES)))
        if not isinstance(self.context_id, str):
            raise VirtualAContractError(
                "INVALID_INITIAL_ASSUMPTION", "context_id must be a string")
        if self.source in ROS_INITIAL_SOURCES and not self.context_id.strip():
            raise VirtualAContractError(
                "MISSING_CONTEXT_ID",
                "source={} requires a non-empty context_id".format(self.source))

    def as_dict(self):
        return {
            "stationary": self.stationary,
            "mode": self.mode,
            "wheels_straight": self.wheels_straight,
            "source": self.source,
            "context_id": self.context_id,
            "node_context_verified": False,
        }


@dataclass(frozen=True)
class CurvatureJet:
    """Formation-reference curvature and its arc-length derivatives."""

    kappa: float
    eta: float
    u: float
    g: float

    def __post_init__(self):
        for name in ("kappa", "eta", "u", "g"):
            object.__setattr__(
                self, name, _finite(getattr(self, name), "CurvatureJet." + name))

    # Formula aliases used by the spatial-state record.
    @property
    def curvature(self):
        return self.kappa

    @property
    def curvature_s(self):
        return self.eta

    @property
    def curvature_ss(self):
        return self.u

    @property
    def curvature_sss(self):
        return self.g

    def as_dict(self):
        return {"kappa": self.kappa, "eta": self.eta, "u": self.u,
                "g": self.g}


@dataclass(frozen=True)
class VehicleSpatialJet:
    """One vehicle centre at formation station ``s`` with world derivatives."""

    vehicle_id: int
    x: float
    y: float
    theta: float
    curvature: float
    p_s: tuple
    p_ss: tuple
    p_sss: tuple
    theta_s: float
    theta_ss: float
    theta_sss: float
    curvature_s: float
    curvature_ss: float

    def __post_init__(self):
        if isinstance(self.vehicle_id, bool) or not isinstance(
                self.vehicle_id, int):
            raise VirtualAContractError(
                "INVALID_VEHICLE_ID", "vehicle_id={!r}".format(self.vehicle_id))
        for name in ("x", "y", "theta", "curvature", "theta_s", "theta_ss",
                     "theta_sss", "curvature_s", "curvature_ss"):
            object.__setattr__(
                self, name, _finite(getattr(self, name),
                                    "VehicleSpatialJet." + name))
        for name in ("p_s", "p_ss", "p_sss"):
            vector = tuple(getattr(self, name))
            if len(vector) != 2:
                raise VirtualAContractError(
                    "INVALID_VECTOR", "{} must have two components".format(name))
            object.__setattr__(
                self, name, tuple(
                    _finite(component, "{}.{}".format(name, index))
                    for index, component in enumerate(vector)))

    # Formula aliases: C_i is the body curvature, H_i the heading slope.
    @property
    def C_i(self):
        return self.curvature

    @property
    def H_i(self):
        return self.theta_s

    @property
    def H_i_s(self):
        return self.theta_ss

    @property
    def H_i_ss(self):
        return self.theta_sss

    @property
    def C_i_s(self):
        return self.curvature_s

    @property
    def C_i_ss(self):
        return self.curvature_ss

    def as_dict(self):
        return {
            "vehicle_id": self.vehicle_id,
            "x": self.x, "y": self.y, "theta": self.theta,
            "curvature": self.curvature,
            "p_s": list(self.p_s), "p_ss": list(self.p_ss),
            "p_sss": list(self.p_sss),
            "theta_s": self.theta_s, "theta_ss": self.theta_ss,
            "theta_sss": self.theta_sss,
            "curvature_s": self.curvature_s,
            "curvature_ss": self.curvature_ss,
            "C_i": self.C_i, "H_i": self.H_i,
            "H_i_s": self.H_i_s, "H_i_ss": self.H_i_ss,
            "C_i_s": self.C_i_s, "C_i_ss": self.C_i_ss,
        }


@dataclass(frozen=True)
class FormationSpatialState:
    """Formation pose plus the ID-ordered three vehicle spatial jets."""

    s: float
    qf: FormationPose
    curvature: CurvatureJet
    vehicles: tuple
    position_error_bound_m: float = 0.0
    angle_error_bound_rad: float = 0.0

    def __post_init__(self):
        object.__setattr__(self, "s", _finite(self.s, "FormationSpatialState.s"))
        if not isinstance(self.qf, FormationPose):
            raise VirtualAContractError(
                "INVALID_FORMATION_POSE", "qf must be a FormationPose")
        if not isinstance(self.curvature, CurvatureJet):
            raise VirtualAContractError(
                "INVALID_CURVATURE_JET", "curvature must be a CurvatureJet")
        vehicles = tuple(self.vehicles)
        if not vehicles:
            raise VirtualAContractError(
                "EMPTY_STATE", "at least one vehicle is required")
        for vehicle in vehicles:
            if not isinstance(vehicle, VehicleSpatialJet):
                raise VirtualAContractError(
                    "INVALID_VEHICLE_JET",
                    "vehicles must contain VehicleSpatialJet records")
        ids = tuple(vehicle.vehicle_id for vehicle in vehicles)
        if ids != tuple(sorted(ids)) or len(set(ids)) != len(ids):
            raise VirtualAContractError(
                "INVALID_VEHICLE_ID_SET",
                "vehicles must be unique and sorted by vehicle_id")
        for name in ("position_error_bound_m", "angle_error_bound_rad"):
            value = _finite(getattr(self, name), "FormationSpatialState." + name)
            if value < 0.0:
                raise VirtualAContractError(
                    "INVALID_ERROR_BOUND", "{} must be >= 0".format(name))
            object.__setattr__(self, name, value)
        object.__setattr__(self, "vehicles", vehicles)

    @property
    def vehicle_ids(self):
        return tuple(vehicle.vehicle_id for vehicle in self.vehicles)

    def vehicle(self, vehicle_id):
        if isinstance(vehicle_id, bool) or not isinstance(vehicle_id, int):
            raise VirtualAContractError(
                "INVALID_VEHICLE_ID", "vehicle_id={!r}".format(vehicle_id))
        for vehicle in self.vehicles:
            if vehicle.vehicle_id == vehicle_id:
                return vehicle
        raise VirtualAContractError(
            "VEHICLE_NOT_IN_STATE", "vehicle_id={}".format(vehicle_id))

    def as_dict(self):
        return {
            "s": self.s,
            "qf": {"x": self.qf.x, "y": self.qf.y, "psi": self.qf.psi},
            "curvature": self.curvature.as_dict(),
            "vehicles": [vehicle.as_dict() for vehicle in self.vehicles],
            "position_error_bound_m": self.position_error_bound_m,
            "angle_error_bound_rad": self.angle_error_bound_rad,
        }


@dataclass(frozen=True)
class ValidationResult:
    """Structured pass/fail with a stable reason and a failure location."""

    passed: bool
    reason: str = None
    detail: str = ""
    location: dict = field(default_factory=dict)
    metrics: dict = field(default_factory=dict)

    def __post_init__(self):
        if not isinstance(self.passed, bool):
            raise VirtualAContractError(
                "INVALID_VALIDATION_RESULT", "passed must be bool")
        if self.passed and self.reason is not None:
            raise VirtualAContractError(
                "INVALID_VALIDATION_RESULT",
                "a passing result carries no failure reason")
        if self.reason is not None and not isinstance(self.reason, str):
            raise VirtualAContractError(
                "INVALID_VALIDATION_RESULT", "reason must be a string or None")
        object.__setattr__(self, "location", dict(sorted(self.location.items())))
        object.__setattr__(self, "metrics", dict(sorted(self.metrics.items())))

    @classmethod
    def ok(cls, **metrics):
        return cls(True, metrics=metrics)

    @classmethod
    def fail(cls, reason, detail="", **location):
        metrics = location.pop("metrics", {})
        return cls(False, reason=reason, detail=detail, location=location,
                   metrics=metrics)

    def as_dict(self):
        return {
            "passed": self.passed,
            "reason": self.reason,
            "detail": self.detail,
            "location": self.location,
            "metrics": self.metrics,
        }


def validate_formation_model(model):
    """Require the authoritative three-vehicle research configuration."""
    if not isinstance(model, FormationModel):
        return ValidationResult.fail(
            "INVALID_FORMATION_MODEL", "expected FormationModel")
    expected = tuple(
        VehicleOffset(vehicle_id, x, y)
        for vehicle_id, x, y in RESEARCH_OFFSETS)
    offsets = model.offsets
    if len(offsets) != len(expected):
        return ValidationResult.fail(
            "FORMATION_MISMATCH",
            "vehicle_count={} expected={}".format(len(offsets), len(expected)))
    for actual, wanted in zip(offsets, expected):
        if (actual.vehicle_id != wanted.vehicle_id
                or abs(actual.x - wanted.x) > FORMATION_TOLERANCE_M
                or abs(actual.y - wanted.y) > FORMATION_TOLERANCE_M):
            return ValidationResult.fail(
                "FORMATION_MISMATCH",
                "offset vehicle_id={} actual=({}, {}) expected=({}, {})".format(
                    actual.vehicle_id, actual.x, actual.y,
                    wanted.x, wanted.y))
    return ValidationResult.ok(
        formation_ids=model.vehicle_ids,
        offsets=[[offset.vehicle_id, offset.x, offset.y] for offset in offsets])


def require_research_formation(model):
    result = validate_formation_model(model)
    if not result.passed:
        raise VirtualAContractError(
            result.reason, result.detail, **result.location)
    return result

@dataclass(frozen=True)
class VirtualARequest:
    """One G2 physical request.  Construction is deliberately permissive.

    Structural validity is checked by :func:`validate_virtual_a_request` and by
    the optimiser/auditor so that an invalid request produces a structured
    ``INVALID_REQUEST`` result instead of an opaque constructor exception.
    """

    plan_id: int
    formation_id: str
    model: FormationModel
    vehicle_shapes: MappingABC
    start_qf: FormationPose
    start_headings: MappingABC
    goal_qf: FormationPose
    initial: InitialAssumption
    goal_policy: GoalPolicy
    limits: FormationTimingLimits

    def __post_init__(self):
        if isinstance(self.vehicle_shapes, MappingABC):
            object.__setattr__(self, "vehicle_shapes", dict(self.vehicle_shapes))
        if isinstance(self.start_headings, MappingABC):
            object.__setattr__(self, "start_headings", dict(self.start_headings))
        if isinstance(self.goal_policy, str):
            try:
                object.__setattr__(self, "goal_policy", coerce_goal_policy(
                    self.goal_policy))
            except VirtualAContractError:
                pass

    @property
    def vehicle_ids(self):
        try:
            return tuple(self.model.vehicle_ids)
        except (AttributeError, TypeError):
            return ()

    @property
    def planning_distance_m(self):
        if not (isinstance(self.start_qf, FormationPose)
                and isinstance(self.goal_qf, FormationPose)):
            return float("nan")
        dx = self.goal_qf.x - self.start_qf.x
        dy = self.goal_qf.y - self.start_qf.y
        return math.hypot(dx, dy)

    def request_summary(self):
        return request_summary(self)

    def as_dict(self):
        return request_summary(self)


def _shape_summary(shapes):
    if not isinstance(shapes, MappingABC):
        return {"type": type(shapes).__name__, "valid": False}
    rows = []
    for vehicle_id in sorted(
            shapes, key=lambda value: (not isinstance(value, int), str(value))):
        shape = shapes[vehicle_id]
        rows.append({
            "vehicle_id": vehicle_id,
            "length_m": getattr(shape, "length_m", None),
            "width_m": getattr(shape, "width_m", None),
        })
    return rows


def _headings_summary(headings):
    if not isinstance(headings, MappingABC):
        return {"type": type(headings).__name__, "valid": False}
    return [
        {"vehicle_id": vehicle_id, "heading_rad": headings[vehicle_id]}
        for vehicle_id in sorted(
            headings, key=lambda value: (not isinstance(value, int), str(value)))]


def _offsets_summary(model):
    try:
        return [
            {"vehicle_id": offset.vehicle_id, "x": offset.x, "y": offset.y}
            for offset in model.offsets]
    except (AttributeError, TypeError):
        return []


def _pose_summary(pose):
    try:
        return {"x": pose.x, "y": pose.y, "psi": pose.psi}
    except AttributeError:
        return {"type": type(pose).__name__, "valid": False}


def request_summary(request):
    """JSON-safe physical-request summary; no planner settings are included."""
    if not isinstance(request, VirtualARequest):
        return {"type": type(request).__name__, "valid": False}
    initial = request.initial
    try:
        initial_summary = initial.as_dict()
    except AttributeError:
        initial_summary = {"type": type(initial).__name__, "valid": False}
    try:
        limits_summary = request.limits.as_dict()
    except AttributeError:
        limits_summary = {"type": type(request.limits).__name__, "valid": False}
    policy = request.goal_policy
    return {
        "schema": REQUEST_SCHEMA,
        "plan_id": request.plan_id,
        "formation_id": request.formation_id,
        "model": {
            "vehicle_ids": list(request.vehicle_ids),
            "offsets": _offsets_summary(request.model),
        },
        "vehicle_shapes": _shape_summary(request.vehicle_shapes),
        "start_qf": _pose_summary(request.start_qf),
        "start_headings": _headings_summary(request.start_headings),
        "goal_qf": _pose_summary(request.goal_qf),
        "initial": initial_summary,
        "goal_policy": getattr(policy, "value", policy),
        "limits": limits_summary,
    }


def _canonical_float(value, context):
    result = _finite(value, context)
    # JSON representations of finite floats are deterministic for a fixed
    # platform.  The explicit finite check is the important contract.
    return result


def request_payload(request):
    """Canonical material hashed by :func:`request_fingerprint`.

    This is the physical request only.  G4 planning-context material is hashed
    separately and must never be folded into this digest.
    """
    if not isinstance(request, VirtualARequest):
        raise VirtualAContractError(
            "INVALID_REQUEST", "expected VirtualARequest")
    if not isinstance(request.model, FormationModel):
        raise VirtualAContractError(
            "INVALID_FORMATION_MODEL", "model must be FormationModel")
    if not isinstance(request.vehicle_shapes, MappingABC):
        raise VirtualAContractError(
            "INVALID_VEHICLE_SHAPES", "vehicle_shapes must be a mapping")
    if not isinstance(request.start_headings, MappingABC):
        raise VirtualAContractError(
            "INVALID_START_HEADINGS", "start_headings must be a mapping")
    if not isinstance(request.start_qf, FormationPose):
        raise VirtualAContractError(
            "INVALID_FORMATION_POSE", "start_qf must be FormationPose")
    if not isinstance(request.goal_qf, FormationPose):
        raise VirtualAContractError(
            "INVALID_FORMATION_POSE", "goal_qf must be FormationPose")
    shapes = []
    for vehicle_id in sorted(request.vehicle_shapes):
        shape = request.vehicle_shapes[vehicle_id]
        if isinstance(vehicle_id, bool) or not isinstance(vehicle_id, int):
            raise VirtualAContractError(
                "INVALID_VEHICLE_ID_SET",
                "vehicle_shapes key={!r}".format(vehicle_id))
        if not isinstance(shape, VehicleShape):
            raise VirtualAContractError(
                "INVALID_VEHICLE_SHAPES",
                "vehicle_id={} expected VehicleShape".format(vehicle_id))
        shapes.append([
            vehicle_id,
            _canonical_float(shape.length_m, "shape.length_m"),
            _canonical_float(shape.width_m, "shape.width_m"),
        ])
    headings = []
    for vehicle_id in sorted(request.start_headings):
        if isinstance(vehicle_id, bool) or not isinstance(vehicle_id, int):
            raise VirtualAContractError(
                "INVALID_VEHICLE_ID_SET",
                "start_headings key={!r}".format(vehicle_id))
        headings.append([
            vehicle_id,
            _canonical_float(request.start_headings[vehicle_id],
                             "start_heading"),
        ])
    if not isinstance(request.initial, InitialAssumption):
        raise VirtualAContractError(
            "INVALID_INITIAL_ASSUMPTION", "initial must be InitialAssumption")
    try:
        policy = coerce_goal_policy(request.goal_policy)
    except VirtualAContractError:
        raise
    if not isinstance(request.limits, FormationTimingLimits):
        raise VirtualAContractError(
            "INVALID_LIMITS", "limits must be FormationTimingLimits")
    initial = request.initial.as_dict()
    initial.pop("node_context_verified", None)
    limits = request.limits.as_dict()
    return {
        "schema": REQUEST_SCHEMA,
        "plan_id": request.plan_id,
        "formation_id": request.formation_id,
        "model": [[offset.vehicle_id, _canonical_float(offset.x, "offset.x"),
                   _canonical_float(offset.y, "offset.y")]
                  for offset in request.model.offsets],
        "vehicle_shapes": shapes,
        "start_qf": [_canonical_float(request.start_qf.x, "start_qf.x"),
                     _canonical_float(request.start_qf.y, "start_qf.y"),
                     _canonical_float(request.start_qf.psi, "start_qf.psi")],
        "start_headings": headings,
        "goal_qf": [_canonical_float(request.goal_qf.x, "goal_qf.x"),
                    _canonical_float(request.goal_qf.y, "goal_qf.y"),
                    _canonical_float(request.goal_qf.psi, "goal_qf.psi")],
        "initial": initial,
        "goal_policy": policy.value,
        "limits": {key: _canonical_float(value, "limits." + key)
                   for key, value in sorted(limits.items())},
    }


def request_fingerprint(request):
    encoded = json.dumps(
        request_payload(request), sort_keys=True, separators=(",", ":"),
        ensure_ascii=False, allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def validate_virtual_a_request(request):
    """Validate the complete physical request before any search starts."""
    if not isinstance(request, VirtualARequest):
        return ValidationResult.fail(
            "INVALID_REQUEST", "expected VirtualARequest")
    if (isinstance(request.plan_id, bool)
            or not isinstance(request.plan_id, int)
            or request.plan_id < 0 or request.plan_id > 2 ** 32 - 1):
        return ValidationResult.fail(
            "INVALID_PLAN_ID", "plan_id={!r}".format(request.plan_id))
    if not isinstance(request.formation_id, str) or not \
            request.formation_id.strip():
        return ValidationResult.fail(
            "INVALID_FORMATION_ID",
            "formation_id={!r}".format(request.formation_id))
    formation = validate_formation_model(request.model)
    if not formation.passed:
        return formation
    if not isinstance(request.vehicle_shapes, MappingABC):
        return ValidationResult.fail(
            "INVALID_VEHICLE_SHAPES", "vehicle_shapes must be a mapping")
    if set(request.vehicle_shapes) != set(RESEARCH_VEHICLE_IDS):
        return ValidationResult.fail(
            "INVALID_VEHICLE_ID_SET",
            "shape ids={} expected={}".format(
                sorted(request.vehicle_shapes), list(RESEARCH_VEHICLE_IDS)))
    for vehicle_id in RESEARCH_VEHICLE_IDS:
        shape = request.vehicle_shapes[vehicle_id]
        if not isinstance(shape, VehicleShape):
            return ValidationResult.fail(
                "INVALID_VEHICLE_SHAPES",
                "vehicle_id={} expected VehicleShape".format(vehicle_id),
                vehicle_id=vehicle_id)
        if (abs(shape.length_m - RESEARCH_VEHICLE_SHAPE_M[0])
                > SHAPE_TOLERANCE_M
                or abs(shape.width_m - RESEARCH_VEHICLE_SHAPE_M[1])
                > SHAPE_TOLERANCE_M):
            return ValidationResult.fail(
                "VEHICLE_SHAPE_MISMATCH",
                "vehicle_id={} actual=({}, {}) expected={}".format(
                    vehicle_id, shape.length_m, shape.width_m,
                    RESEARCH_VEHICLE_SHAPE_M), vehicle_id=vehicle_id)
    if not isinstance(request.start_qf, FormationPose):
        return ValidationResult.fail(
            "INVALID_FORMATION_POSE", "start_qf must be FormationPose")
    if not isinstance(request.goal_qf, FormationPose):
        return ValidationResult.fail(
            "INVALID_FORMATION_POSE", "goal_qf must be FormationPose")
    # Local import avoids a module cycle: geometry imports these contracts.
    from .geometry import validate_initial_state
    initial = validate_initial_state(
        request.start_qf, request.start_headings, request.initial,
        kappa0=0.0, eta0=0.0, model=request.model)
    if not initial.passed:
        return initial
    try:
        policy = coerce_goal_policy(request.goal_policy)
    except VirtualAContractError as exc:
        return ValidationResult.fail(
            exc.reason, exc.detail, **exc.location)
    if not isinstance(request.limits, FormationTimingLimits):
        return ValidationResult.fail(
            "INVALID_LIMITS", "limits must be FormationTimingLimits")
    limits_summary = request.limits.as_dict()
    for name in ("r_min_ackermann", "wheelbase", "track", "steer_angle_max"):
        value = _finite(limits_summary[name], "limits." + name)
        if value <= 0.0:
            return ValidationResult.fail(
                "INVALID_LIMITS", "{} must be positive".format(name))
    if limits_summary["steer_angle_max"] > math.pi:
        return ValidationResult.fail(
            "INVALID_LIMITS", "steer_angle_max must be <= pi")
    if abs(_wrap(request.goal_qf.psi - request.start_qf.psi)) > math.pi:
        # _wrap always returns in [-pi, pi]; retained as an explicit finite
        # branch check and documentation of the G2 shortest-wrap policy.
        return ValidationResult.fail(
            "UNSUPPORTED_GOAL_HEADING_BRANCH",
            "goal heading branch could not be represented by shortest wrap")
    try:
        fingerprint = request_fingerprint(request)
    except (VirtualAContractError, ValueError, TypeError) as exc:
        return ValidationResult.fail(
            "INVALID_REQUEST", "request fingerprint failed: {}".format(exc))
    return ValidationResult.ok(
        request_sha256=fingerprint,
        goal_heading_branch_rad=_wrap(
            request.goal_qf.psi - request.start_qf.psi),
        goal_policy=policy.value,
        planning_distance_m=request.planning_distance_m,
        limits=limits_summary,
        initial=initial.metrics)


@dataclass(frozen=True)
class PlannerConfig:
    """Engineering planner settings; not a physical-success declaration."""

    intervals: int = 8
    max_initials: int = 6
    max_seconds: float = 30.0
    max_iterations: int = 120
    solver: str = "ipopt"
    allow_scipy_fallback: bool = True
    collocation_points: int = 9
    curvature_margin: float = 0.02
    eta_limit: float = 2.0
    kappa_limit: float = 1.0 / (1.0 + 0.810)
    position_tolerance_m: float = DEFAULT_GEOMETRY_POSITION_TOL_M
    heading_tolerance_rad: float = DEFAULT_GEOMETRY_HEADING_TOL_RAD
    rigidity_tolerance_m: float = DEFAULT_GEOMETRY_RIGIDITY_TOL_M
    ipopt_tolerance: float = 1e-8
    scipy_tolerance: float = 1e-8
    scipy_maxiter: int = 120
    # C2-B corridor guidance.  The default legacy_route encoding below is the
    # unchanged G4/G5 planner config; these fields are appended to as_dict()
    # only when at least one differs from its default, so historical payloads
    # and planning-context digests remain reproducible.
    guidance_mode: str = "legacy_route"
    corridor_enforcement: str = "hard"
    corridor_margin_m: float = 1e-5
    corridor_margin_portal_fraction: float = 0.25
    corridor_kappa_weight: float = 0.01
    corridor_guide_weight: float = 0.1
    corridor_clearance_weight: float = 1.0
    corridor_clearance_target_m: float = 0.05
    corridor_quadrature_points: int = 9
    corridor_audit_position_margin_m: float = 5e-8
    corridor_audit_max_depth: int = 20
    corridor_audit_max_intervals: int = 20000
    corridor_audit_min_width_m: float = 1e-5
    corridor_box_padding_m: float = 2.0
    # TASK03 versioned hard-constraint scope.  ``f_only`` is the historical
    # reference-point containment contract.  ``three_vehicle_union_v1`` adds
    # complete three-vehicle OBB rows and the independent continuous
    # certificate.  f_only is omitted from ``as_dict`` so every historical
    # payload and planning-context digest is byte-identical; unknown values
    # fail closed in validation.
    formation_corridor_scope: str = "f_only"

    def as_dict(self):
        encoded = {
            "intervals": self.intervals,
            "max_initials": self.max_initials,
            "max_seconds": self.max_seconds,
            "max_iterations": self.max_iterations,
            "solver": self.solver,
            "allow_scipy_fallback": self.allow_scipy_fallback,
            "collocation_points": self.collocation_points,
            "curvature_margin": self.curvature_margin,
            "eta_limit": self.eta_limit,
            "kappa_limit": self.kappa_limit,
            "position_tolerance_m": self.position_tolerance_m,
            "heading_tolerance_rad": self.heading_tolerance_rad,
            "rigidity_tolerance_m": self.rigidity_tolerance_m,
            "ipopt_tolerance": self.ipopt_tolerance,
            "scipy_tolerance": self.scipy_tolerance,
            "scipy_maxiter": self.scipy_maxiter,
        }
        corridor = _planner_corridor_dict(self)
        if corridor != _PLANNER_CORRIDOR_DEFAULTS:
            encoded.update(corridor)
        scope = str(self.formation_corridor_scope).strip().lower()
        if scope != FORMATION_CORRIDOR_SCOPE_F_ONLY:
            # Union scope is an explicit, versioned planning-context input.
            # f_only is the omitted historical default and keeps every old
            # config/context SHA reproducible.
            encoded["formation_corridor_scope"] = scope
        return encoded


_PLANNER_CORRIDOR_DEFAULTS = {
    "guidance_mode": "legacy_route",
    "corridor_enforcement": "hard",
    "corridor_margin_m": 1e-5,
    "corridor_margin_portal_fraction": 0.25,
    "corridor_kappa_weight": 0.01,
    "corridor_guide_weight": 0.1,
    "corridor_clearance_weight": 1.0,
    "corridor_clearance_target_m": 0.05,
    "corridor_quadrature_points": 9,
    "corridor_audit_position_margin_m": 5e-8,
    "corridor_audit_max_depth": 20,
    "corridor_audit_max_intervals": 20000,
    "corridor_audit_min_width_m": 1e-5,
    "corridor_box_padding_m": 2.0,
}


def _planner_corridor_dict(config):
    return {
        "guidance_mode": config.guidance_mode,
        "corridor_enforcement": config.corridor_enforcement,
        "corridor_margin_m": config.corridor_margin_m,
        "corridor_margin_portal_fraction":
            config.corridor_margin_portal_fraction,
        "corridor_kappa_weight": config.corridor_kappa_weight,
        "corridor_guide_weight": config.corridor_guide_weight,
        "corridor_clearance_weight": config.corridor_clearance_weight,
        "corridor_clearance_target_m": config.corridor_clearance_target_m,
        "corridor_quadrature_points": config.corridor_quadrature_points,
        "corridor_audit_position_margin_m":
            config.corridor_audit_position_margin_m,
        "corridor_audit_max_depth": config.corridor_audit_max_depth,
        "corridor_audit_max_intervals": config.corridor_audit_max_intervals,
        "corridor_audit_min_width_m": config.corridor_audit_min_width_m,
        "corridor_box_padding_m": config.corridor_box_padding_m,
    }


def validate_planner_config(config):
    if not isinstance(config, PlannerConfig):
        return ValidationResult.fail(
            "INVALID_CONFIG", "expected PlannerConfig")
    try:
        intervals = int(config.intervals)
        max_initials = int(config.max_initials)
        max_iterations = int(config.max_iterations)
        collocation = int(config.collocation_points)
        scipy_iterations = int(config.scipy_maxiter)
        max_seconds = _finite(config.max_seconds, "config.max_seconds")
        margin = _finite(config.curvature_margin, "config.curvature_margin")
        eta_limit = _finite(config.eta_limit, "config.eta_limit")
        kappa_limit = _finite(config.kappa_limit, "config.kappa_limit")
        position_tol = _finite(
            config.position_tolerance_m, "config.position_tolerance_m")
        heading_tol = _finite(
            config.heading_tolerance_rad, "config.heading_tolerance_rad")
        rigidity_tol = _finite(
            config.rigidity_tolerance_m, "config.rigidity_tolerance_m")
        ipopt_tol = _finite(config.ipopt_tolerance, "config.ipopt_tolerance")
        scipy_tol = _finite(config.scipy_tolerance, "config.scipy_tolerance")
        corridor_margin = _finite(
            config.corridor_margin_m, "config.corridor_margin_m")
        portal_fraction = _finite(
            config.corridor_margin_portal_fraction,
            "config.corridor_margin_portal_fraction")
        kappa_weight = _finite(
            config.corridor_kappa_weight, "config.corridor_kappa_weight")
        guide_weight = _finite(
            config.corridor_guide_weight, "config.corridor_guide_weight")
        clearance_weight = _finite(
            config.corridor_clearance_weight,
            "config.corridor_clearance_weight")
        clearance_target = _finite(
            config.corridor_clearance_target_m,
            "config.corridor_clearance_target_m")
        audit_position_margin = _finite(
            config.corridor_audit_position_margin_m,
            "config.corridor_audit_position_margin_m")
        audit_min_width = _finite(
            config.corridor_audit_min_width_m,
            "config.corridor_audit_min_width_m")
        box_padding = _finite(
            config.corridor_box_padding_m, "config.corridor_box_padding_m")
        quadrature = config.corridor_quadrature_points
        audit_depth = config.corridor_audit_max_depth
        audit_intervals = config.corridor_audit_max_intervals
        if isinstance(quadrature, bool) or not isinstance(
                quadrature, numbers.Integral):
            raise VirtualAContractError(
                "INVALID_CONFIG",
                "corridor_quadrature_points must be an integer")
        if isinstance(audit_depth, bool) or not isinstance(
                audit_depth, numbers.Integral):
            raise VirtualAContractError(
                "INVALID_CONFIG", "corridor_audit_max_depth must be an integer")
        if isinstance(audit_intervals, bool) or not isinstance(
                audit_intervals, numbers.Integral):
            raise VirtualAContractError(
                "INVALID_CONFIG",
                "corridor_audit_max_intervals must be an integer")
        quadrature = int(quadrature)
        audit_depth = int(audit_depth)
        audit_intervals = int(audit_intervals)
    except (TypeError, ValueError, VirtualAContractError) as exc:
        return ValidationResult.fail("INVALID_CONFIG", str(exc))
    if intervals < 2 or intervals > 64:
        return ValidationResult.fail(
            "INVALID_CONFIG", "intervals must be in [2, 64]")
    if max_initials < 1 or max_initials > MAX_INITIALS:
        return ValidationResult.fail(
            "INVALID_CONFIG",
            "max_initials must be in [1, {}]".format(MAX_INITIALS))
    if max_seconds < 0.0 or not math.isfinite(max_seconds):
        return ValidationResult.fail(
            "INVALID_CONFIG", "max_seconds must be finite and >= 0")
    if max_iterations < 1 or max_iterations > 10000:
        return ValidationResult.fail(
            "INVALID_CONFIG", "max_iterations must be in [1, 10000]")
    if collocation < 9:
        return ValidationResult.fail(
            "INVALID_CONFIG", "collocation_points must be >= 9")
    if not (0.0 <= margin < 0.5):
        return ValidationResult.fail(
            "INVALID_CONFIG", "curvature_margin must be in [0, 0.5)")
    if eta_limit <= 0.0 or kappa_limit <= 0.0 or kappa_limit >= 1.0:
        return ValidationResult.fail(
            "INVALID_CONFIG", "eta/kappa limits must be positive and kappa<1")
    if not (0.0 < position_tol < 0.1):
        return ValidationResult.fail(
            "INVALID_CONFIG", "position_tolerance_m must be in (0, 0.1)")
    if not (0.0 < heading_tol < 0.1):
        return ValidationResult.fail(
            "INVALID_CONFIG", "heading_tolerance_rad must be in (0, 0.1)")
    if not (0.0 < rigidity_tol < 0.01):
        return ValidationResult.fail(
            "INVALID_CONFIG", "rigidity_tolerance_m must be in (0, 0.01)")
    if ipopt_tol <= 0.0 or scipy_tol <= 0.0:
        return ValidationResult.fail(
            "INVALID_CONFIG", "solver tolerances must be positive")
    if scipy_iterations < 1:
        return ValidationResult.fail(
            "INVALID_CONFIG", "scipy_maxiter must be >= 1")
    guidance_mode = str(config.guidance_mode).strip().lower()
    if guidance_mode not in ("legacy_route", "corridor", FOOTPRINT_GUIDANCE):
        return ValidationResult.fail(
            "INVALID_CONFIG",
            "guidance_mode={!r} allowed=('legacy_route','corridor','footprint_seed')".format(
                config.guidance_mode))
    enforcement = str(config.corridor_enforcement).strip().lower()
    if enforcement not in ("hard", "soft"):
        return ValidationResult.fail(
            "INVALID_CONFIG",
            "corridor_enforcement={!r} allowed=('hard','soft')".format(
                config.corridor_enforcement))
    if guidance_mode == FOOTPRINT_GUIDANCE and enforcement != "hard":
        return ValidationResult.fail(
            "INVALID_CONFIG", "footprint_seed requires hard corridor enforcement")
    formation_scope = str(config.formation_corridor_scope).strip().lower()
    if formation_scope not in FORMATION_CORRIDOR_SCOPES:
        return ValidationResult.fail(
            "INVALID_CONFIG",
            "formation_corridor_scope={!r} allowed={}".format(
                config.formation_corridor_scope, FORMATION_CORRIDOR_SCOPES))
    if formation_scope == FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1:
        # The union scope is only meaningful with the corridor guidance and
        # its hard rows.  Silently falling back to soft/legacy would make the
        # certificate gate meaningless.
        if guidance_mode != "corridor" or enforcement != "hard":
            return ValidationResult.fail(
                "INVALID_CONFIG",
                "formation_corridor_scope={!r} requires "
                "guidance_mode='corridor' and corridor_enforcement='hard'".format(
                    formation_scope))
    if not (0.0 <= corridor_margin < 0.5):
        return ValidationResult.fail(
            "INVALID_CONFIG", "corridor_margin_m must be in [0, 0.5)")
    if not (0.0 < portal_fraction <= 0.25):
        return ValidationResult.fail(
            "INVALID_CONFIG",
            "corridor_margin_portal_fraction must be in (0, 0.25]")
    for name, value in (
            ("corridor_kappa_weight", kappa_weight),
            ("corridor_guide_weight", guide_weight),
            ("corridor_clearance_weight", clearance_weight),
            ("corridor_clearance_target_m", clearance_target)):
        if value < 0.0:
            return ValidationResult.fail(
                "INVALID_CONFIG", "{} must be nonnegative".format(name))
    if quadrature < 3 or quadrature > 64:
        return ValidationResult.fail(
            "INVALID_CONFIG", "corridor_quadrature_points must be in [3, 64]")
    if not (0.0 <= audit_position_margin < 0.1):
        return ValidationResult.fail(
            "INVALID_CONFIG",
            "corridor_audit_position_margin_m must be in [0, 0.1)")
    if audit_depth < 0 or audit_depth > 64:
        return ValidationResult.fail(
            "INVALID_CONFIG", "corridor_audit_max_depth must be in [0, 64]")
    if audit_intervals < 1 or audit_intervals > 10_000_000:
        return ValidationResult.fail(
            "INVALID_CONFIG",
            "corridor_audit_max_intervals must be in [1, 10000000]")
    if not (0.0 < audit_min_width < 0.1):
        return ValidationResult.fail(
            "INVALID_CONFIG", "corridor_audit_min_width_m must be in (0, 0.1)")
    if not (0.0 <= box_padding < 10.0):
        return ValidationResult.fail(
            "INVALID_CONFIG", "corridor_box_padding_m must be in [0, 10)")
    solver = str(config.solver).lower()
    if solver not in ("ipopt", "scipy", "slsqp", "auto"):
        return ValidationResult.fail(
            "INVALID_CONFIG",
            "solver={!r} allowed=('ipopt','scipy','slsqp','auto')".format(
                config.solver))
    return ValidationResult.ok(
        intervals=intervals, max_initials=max_initials,
        max_seconds=max_seconds, max_iterations=max_iterations,
        collocation_points=collocation, solver=solver,
        curvature_margin=margin, eta_limit=eta_limit,
        kappa_limit=kappa_limit, position_tolerance_m=position_tol,
        heading_tolerance_rad=heading_tol,
        rigidity_tolerance_m=rigidity_tol,
        ipopt_tolerance=ipopt_tol, scipy_tolerance=scipy_tol,
        scipy_maxiter=scipy_iterations, guidance_mode=guidance_mode,
        corridor_enforcement=enforcement,
        formation_corridor_scope=formation_scope,
        corridor_margin_m=corridor_margin,
        corridor_margin_portal_fraction=portal_fraction,
        corridor_kappa_weight=kappa_weight,
        corridor_guide_weight=guide_weight,
        corridor_clearance_weight=clearance_weight,
        corridor_clearance_target_m=clearance_target,
        corridor_quadrature_points=quadrature,
        corridor_audit_position_margin_m=audit_position_margin,
        corridor_audit_max_depth=audit_depth,
        corridor_audit_max_intervals=audit_intervals,
        corridor_audit_min_width_m=audit_min_width,
        corridor_box_padding_m=box_padding)


def research_planner_config(guidance_mode="corridor",
                            corridor_enforcement="hard",
                            formation_corridor_scope="f_only", **overrides):
    """Build the default virtual-A research planner configuration.

    The two historical entry-point choices are preset.  ``f_only`` remains the
    scientific reference by default; ``three_vehicle_union_v1`` must be
    requested explicitly so no existing run silently changes scope.  Every
    historical ``PlannerConfig`` default, ``as_dict`` echo and ``from_dict``
    round trip is unchanged: direct construction still receives
    ``legacy_route``.  Guidance, enforcement and scope are normalized before
    validation, so the returned frozen config is exactly what the planner
    consumes.  Unknown values fail here rather than at solve time.
    """
    if not isinstance(guidance_mode, str):
        raise VirtualAContractError(
            "INVALID_CONFIG",
            "guidance_mode must be a string, got {!r}".format(guidance_mode))
    if not isinstance(corridor_enforcement, str):
        raise VirtualAContractError(
            "INVALID_CONFIG",
            "corridor_enforcement must be a string, got {!r}".format(
                corridor_enforcement))
    if not isinstance(formation_corridor_scope, str):
        raise VirtualAContractError(
            "INVALID_CONFIG",
            "formation_corridor_scope must be a string, got {!r}".format(
                formation_corridor_scope))
    guidance = guidance_mode.strip().lower()
    enforcement = corridor_enforcement.strip().lower()
    scope = formation_corridor_scope.strip().lower()
    config = PlannerConfig(
        guidance_mode=guidance,
        corridor_enforcement=enforcement,
        formation_corridor_scope=scope,
        **overrides)
    validation = validate_planner_config(config)
    if not validation.passed:
        raise VirtualAContractError(
            validation.reason, validation.detail, **validation.location)
    return config


def validate_geometry_audit_config(config):
    """Validate every independent-audit limit before it is used.

    NaN or infinity must never reach a comparison in the audit: Python's
    ordered comparisons all return ``False`` for NaN, which would turn a
    rejection test into a silent pass.  Integer fields are checked as real
    integers, not truncated floats or booleans.
    """
    if not isinstance(config, GeometryAuditConfig):
        return ValidationResult.fail(
            "INVALID_AUDIT_CONFIG", "expected GeometryAuditConfig")
    integer_domains = {
        "max_depth": (0, 64),
        "max_leaves": (1, 10_000_000),
        "sample_points_per_segment": (2, 1_000_000),
        "audit_max_subintervals_per_segment": (2, 1_000_000),
        "audit_max_total_subintervals": (1, 10_000_000),
    }
    values = {}
    for name, (minimum, maximum) in integer_domains.items():
        value = getattr(config, name)
        if isinstance(value, bool) or not isinstance(value, numbers.Integral):
            return ValidationResult.fail(
                "INVALID_AUDIT_CONFIG",
                "{} must be an integer, got {!r}".format(name, value),
                field=name)
        value = int(value)
        if value < minimum or value > maximum:
            return ValidationResult.fail(
                "INVALID_AUDIT_CONFIG",
                "{}={} outside [{}, {}]".format(
                    name, value, minimum, maximum),
                field=name)
        values[name] = value
    float_domains = {
        # Non-negative limits: zero is meaningful for an exactly zero
        # discontinuity/third derivative policy.
        "c2_value_tol": (0.0, True),
        "c2_slope_tol": (0.0, True),
        "c2_second_tol": (0.0, True),
        "c2_third_abs_limit": (0.0, True),
        # Strictly positive tolerances and budgets.
        "position_tolerance_m": (0.0, False),
        "heading_tolerance_rad": (0.0, False),
        "rigidity_tolerance_m": (0.0, False),
        "integration_budget_m": (0.0, False),
        "angle_budget_rad": (0.0, False),
        "interval_curvature_tolerance": (0.0, False),
        "audit_panel_width_m": (0.0, False),
        "audit_position_budget_m": (0.0, False),
    }
    for name, (minimum, allow_equal) in float_domains.items():
        value = getattr(config, name)
        if isinstance(value, bool) or not isinstance(value, numbers.Real):
            return ValidationResult.fail(
                "INVALID_AUDIT_CONFIG",
                "{} must be a finite real number, got {!r}".format(
                    name, value),
                field=name)
        try:
            value = float(value)
        except (TypeError, ValueError, OverflowError):
            return ValidationResult.fail(
                "INVALID_AUDIT_CONFIG",
                "{} cannot be converted to float".format(name),
                field=name)
        if not math.isfinite(value):
            return ValidationResult.fail(
                "INVALID_AUDIT_CONFIG",
                "{} must be finite, got {!r}".format(name, value),
                field=name)
        if value < minimum or (value == minimum and not allow_equal):
            return ValidationResult.fail(
                "INVALID_AUDIT_CONFIG",
                "{}={!r} outside its {} domain".format(
                    name, value,
                    "non-negative" if allow_equal else "positive"),
                field=name)
        values[name] = value
    if values["audit_position_budget_m"] > values["integration_budget_m"]:
        return ValidationResult.fail(
            "INVALID_AUDIT_CONFIG",
            "audit_position_budget_m={!r} exceeds the G1 position contract "
            "{!r}".format(values["audit_position_budget_m"],
                          values["integration_budget_m"]),
            field="audit_position_budget_m")
    return ValidationResult.ok(**values)


@dataclass(frozen=True)
class InitialGuess:
    """One reconstructible automatic initial value for the endpoint NLP."""

    source: str
    label: str
    S: float
    kappas: tuple
    etas: tuple
    length_multiplier: float
    shape_sign: int
    node_s: tuple = ()
    metadata: dict = field(default_factory=dict)

    def __post_init__(self):
        object.__setattr__(self, "kappas", tuple(float(v) for v in self.kappas))
        object.__setattr__(self, "etas", tuple(float(v) for v in self.etas))
        object.__setattr__(self, "node_s", tuple(float(v) for v in self.node_s))
        object.__setattr__(self, "metadata", dict(sorted(self.metadata.items())))

    @property
    def vector(self):
        return (float(self.S),) + self.kappas + self.etas

    def reconstruct(self):
        return {
            "source": self.source, "label": self.label,
            "S": self.S, "kappas": list(self.kappas),
            "etas": list(self.etas),
            "length_multiplier": self.length_multiplier,
            "shape_sign": self.shape_sign,
            "node_s": list(self.node_s),
            "metadata": dict(self.metadata),
        }

    def as_dict(self):
        return self.reconstruct()


@dataclass(frozen=True)
class HeadingBranch:
    """One finite unwrapped endpoint-heading candidate for a guide.

    TASK 01 Step 3: the curvature path's endpoint heading is
    ``base + 2*pi*k``.  ``base_heading_rad`` is the shortest physical wrap,
    ``winding_k`` is the integer branch, and ``unwrapped_heading_rad`` is the
    value the NLP endpoint equality and the initial packet actually use.  The
    physical request goal pose is never rewritten.  ``guide_turn_rad`` is the
    accumulated guide-tangent turn used to rank the branch (``None`` when no
    reliable guide exists); ``candidate_rank`` is the 0-based position in the
    deterministic finite candidate ordering.
    """

    base_heading_rad: float
    winding_k: int
    unwrapped_heading_rad: float
    guide_turn_rad: float = None
    source: str = "no_reliable_guide"
    candidate_rank: int = 0

    def __post_init__(self):
        object.__setattr__(self, "base_heading_rad",
                           float(self.base_heading_rad))
        object.__setattr__(self, "winding_k", int(self.winding_k))
        object.__setattr__(
            self, "unwrapped_heading_rad",
            float(self.unwrapped_heading_rad))
        if self.guide_turn_rad is not None:
            object.__setattr__(self, "guide_turn_rad",
                               float(self.guide_turn_rad))
        object.__setattr__(self, "candidate_rank", int(self.candidate_rank))
        object.__setattr__(self, "source", str(self.source))

    def as_dict(self):
        return {
            "base_heading_rad": self.base_heading_rad,
            "winding_k": self.winding_k,
            "unwrapped_heading_rad": self.unwrapped_heading_rad,
            "guide_turn_rad": self.guide_turn_rad,
            "source": self.source,
            "candidate_rank": self.candidate_rank,
        }


@dataclass(frozen=True)
class GeometryAuditConfig:
    """Independent geometry-audit limits; no solver residual is trusted."""

    max_depth: int = 20
    max_leaves: int = 200000
    sample_points_per_segment: int = 9
    c2_value_tol: float = 1e-9
    c2_slope_tol: float = 1e-9
    c2_second_tol: float = 1e-8
    c2_third_abs_limit: float = 1.0e6
    position_tolerance_m: float = DEFAULT_GEOMETRY_POSITION_TOL_M
    heading_tolerance_rad: float = DEFAULT_GEOMETRY_HEADING_TOL_RAD
    rigidity_tolerance_m: float = DEFAULT_GEOMETRY_RIGIDITY_TOL_M
    integration_budget_m: float = 1e-7
    angle_budget_rad: float = 1e-9
    interval_curvature_tolerance: float = 1e-9
    audit_panel_width_m: float = 0.02
    audit_max_subintervals_per_segment: int = 4096
    audit_max_total_subintervals: int = 20000
    audit_position_budget_m: float = 5e-8

    @property
    def interval_max_depth(self):
        return self.max_depth

    @property
    def interval_leaf_limit(self):
        return self.max_leaves

    def as_dict(self):
        return {
            "max_depth": self.max_depth,
            "max_leaves": self.max_leaves,
            "sample_points_per_segment": self.sample_points_per_segment,
            "c2_value_tol": self.c2_value_tol,
            "c2_slope_tol": self.c2_slope_tol,
            "c2_second_tol": self.c2_second_tol,
            "c2_third_abs_limit": self.c2_third_abs_limit,
            "position_tolerance_m": self.position_tolerance_m,
            "heading_tolerance_rad": self.heading_tolerance_rad,
            "rigidity_tolerance_m": self.rigidity_tolerance_m,
            "integration_budget_m": self.integration_budget_m,
            "angle_budget_rad": self.angle_budget_rad,
            "interval_curvature_tolerance": self.interval_curvature_tolerance,
            "audit_panel_width_m": self.audit_panel_width_m,
            "audit_max_subintervals_per_segment":
                self.audit_max_subintervals_per_segment,
            "audit_max_total_subintervals": self.audit_max_total_subintervals,
            "audit_position_budget_m": self.audit_position_budget_m,
        }


@dataclass(frozen=True)
class AuditReport:
    """Structured independent-audit outcome with a first-failure location."""

    scope: str
    passed: bool
    reason: str = None
    detail: str = ""
    location: dict = field(default_factory=dict)
    request_summary: dict = field(default_factory=dict)
    path_summary: dict = field(default_factory=dict)
    limits_summary: dict = field(default_factory=dict)
    whole_segment_bounds: tuple = ()
    integration: dict = field(default_factory=dict)
    c2: dict = field(default_factory=dict)
    check_count: int = 0
    metrics: dict = field(default_factory=dict)
    request_sha256: str = ""
    timing: dict = field(default_factory=dict)
    path_sha256: str = ""
    model_sha256: str = ""
    limits_sha256: str = ""
    grid_sha256: str = ""
    planning_context_sha256: str = ""
    assumptions: tuple = ()

    def __post_init__(self):
        if self.scope not in (GEOMETRY_SCOPE, DYNAMICS_SCOPE, COLLISION_SCOPE):
            raise VirtualAContractError(
                "INVALID_AUDIT_SCOPE", "scope={!r}".format(self.scope))
        if not isinstance(self.passed, bool):
            raise VirtualAContractError(
                "INVALID_AUDIT_REPORT", "passed must be bool")
        if self.passed and self.reason is not None:
            raise VirtualAContractError(
                "INVALID_AUDIT_REPORT",
                "a passing audit carries no failure reason")
        object.__setattr__(self, "location", dict(sorted(self.location.items())))
        object.__setattr__(self, "whole_segment_bounds",
                           tuple(self.whole_segment_bounds))
        object.__setattr__(self, "metrics", dict(sorted(self.metrics.items())))
        object.__setattr__(self, "timing", dict(sorted(self.timing.items())))
        object.__setattr__(self, "assumptions", tuple(self.assumptions))

    @classmethod
    def ok(cls, **kwargs):
        kwargs.setdefault("scope", GEOMETRY_SCOPE)
        kwargs.setdefault("passed", True)
        return cls(**kwargs)

    @classmethod
    def fail(cls, reason, detail="", **kwargs):
        location = kwargs.pop("location", {})
        scope = kwargs.pop("scope", GEOMETRY_SCOPE)
        kwargs.update(scope=scope, passed=False, reason=reason,
                      detail=detail, location=location)
        return cls(**kwargs)

    @property
    def first_failure(self):
        return dict(self.location) if not self.passed else None

    @property
    def failure_location(self):
        return self.first_failure

    @property
    def checks(self):
        return int(self.check_count)

    @property
    def request_fingerprint(self):
        return self.request_sha256

    def as_dict(self):
        return {
            "scope": self.scope,
            "passed": self.passed,
            "reason": self.reason,
            "detail": self.detail,
            "first_failure": self.first_failure,
            "request_sha256": self.request_sha256,
            "path_sha256": self.path_sha256,
            "model_sha256": self.model_sha256,
            "limits_sha256": self.limits_sha256,
            "grid_sha256": self.grid_sha256,
            "planning_context_sha256": self.planning_context_sha256,
            "assumptions": list(self.assumptions),
            "request_summary": self.request_summary,
            "path_summary": self.path_summary,
            "limits_summary": self.limits_summary,
            "whole_segment_bounds": [
                dict(record) for record in self.whole_segment_bounds],
            "integration": self.integration,
            "c2": self.c2,
            "timing": self.timing,
            "check_count": self.check_count,
            "checks": self.check_count,
            "metrics": self.metrics,
        }


@dataclass(frozen=True)
class AttemptRecord:
    """One solver attempt; the candidate path is retained only in memory."""

    attempt_index: int
    initial_source: str
    initial_label: str
    solver: str
    solver_status: str
    status: str
    iterations: int = 0
    objective: float = None
    constraint_residual: float = None
    corridor_residual_m: float = None
    endpoint_error_m: float = None
    heading_error_rad: float = None
    wall_time_s: float = 0.0
    audit: object = None
    rejection_reason: str = ""
    initial: object = None
    candidate_path: object = None
    detail: str = ""
    soft_deadline_triggered: bool = False
    deadline_trigger_stage: str = ""
    remaining_budget_s: float = None
    # Step 3 branch evidence: the same HeadingBranch object that produced the
    # initial packet and the NLP endpoint equality.
    heading_base_rad: float = None
    winding_k: int = None
    guide_turn_rad: float = None
    # Step 4 S-domain evidence: the finite domain this attempt was solved in,
    # the solved S and whether that S sits on the upper bound.  ``upper_active``
    # is decided by the packet tolerance, never by a solver status string.
    distance_m: float = None
    guide_length_m: float = None
    s_lower_m: float = None
    s_upper_m: float = None
    s_solution_m: float = None
    s_upper_active: bool = None

    @property
    def candidate_audit(self):
        return self.audit

    def as_dict(self):
        payload = {
            "attempt_index": self.attempt_index,
            "initial_source": self.initial_source,
            "initial_label": self.initial_label,
            "initial": self.initial.as_dict()
            if hasattr(self.initial, "as_dict") else None,
            "solver": self.solver,
            "solver_status": self.solver_status,
            "status": self.status,
            "iterations": self.iterations,
            "objective": self.objective,
            "constraint_residual": self.constraint_residual,
            "endpoint_error_m": self.endpoint_error_m,
            "heading_error_rad": self.heading_error_rad,
            "wall_time_s": self.wall_time_s,
            "candidate_audit": self.audit.as_dict()
            if hasattr(self.audit, "as_dict") else None,
            "rejection_reason": self.rejection_reason,
            "detail": self.detail,
            "soft_deadline_triggered": self.soft_deadline_triggered,
            "deadline_trigger_stage": self.deadline_trigger_stage,
            "remaining_budget_s": self.remaining_budget_s,
        }
        if self.heading_base_rad is not None:
            payload["heading_base_rad"] = self.heading_base_rad
        if self.winding_k is not None:
            payload["winding_k"] = self.winding_k
        if self.guide_turn_rad is not None:
            payload["guide_turn_rad"] = self.guide_turn_rad
        if self.distance_m is not None:
            payload["distance_m"] = self.distance_m
        if self.guide_length_m is not None:
            payload["guide_length_m"] = self.guide_length_m
        if self.s_lower_m is not None:
            payload["S_lower_m"] = self.s_lower_m
        if self.s_upper_m is not None:
            payload["S_upper_m"] = self.s_upper_m
        if self.s_solution_m is not None:
            payload["S_solution_m"] = self.s_solution_m
        if self.s_upper_active is not None:
            payload["upper_active"] = self.s_upper_active
        if self.corridor_residual_m is not None:
            payload["corridor_residual_m"] = self.corridor_residual_m
        return payload


@dataclass(frozen=True)
class GeometryResult:
    """G2 result; only the allowed geometry statuses are permitted."""

    status: str
    path: object = None
    request_summary: dict = field(default_factory=dict)
    audit: object = None
    attempts: tuple = ()
    wall_time_s: float = 0.0
    request_sha256: str = ""
    reason: str = ""
    detail: str = ""
    config_summary: dict = field(default_factory=dict)
    solver_used: str = ""
    goal_heading_branch_rad: float = None
    deadline_exhausted: bool = False
    deadline_summary: dict = field(default_factory=dict)
    # C3 record only: number of initial guesses this solve was scheduled with.
    initial_count: int = 0
    # C2-B evidence: actual scalar halfspaces built for the NLP and the
    # maximum halfspace excess of the returned candidate (or of the recorded
    # attempts on failure).  ``None`` means no corridor was constructed.
    corridor_halfspace_constraint_count: int = None
    corridor_max_residual_m: float = None
    # TASK03 Step 1: the finite complete-OBB templates are additional scalar
    # rows in the same NLP.  These fields report what was actually compiled;
    # they are not a continuous certificate.
    formation_corridor_scope: str = "f_only"
    formation_template_id: str = ""
    # Template F reference-point rows and complete-OBB rows are reported
    # separately; the OBB count never includes F rows.
    formation_f_constraint_count: int = None
    formation_obb_constraint_count: int = None
    # TASK03 Step 3: independent finite sample audit and the at-most-one
    # audit-directed cut.  These are evidence records, never a certificate.
    formation_sample_audit: dict = field(default_factory=dict)
    formation_refinement: dict = field(default_factory=dict)
    formation_f_max_residual_m: float = None
    formation_obb_max_residual_m: float = None
    # TASK03 Step 4: an independent continuous F+OBB interval certificate.
    # Only a VERIFIED certificate with a matching identity may pass the
    # optimizer gate; the finite audit and row counts are never a certificate.
    formation_certificate: dict = field(default_factory=dict)

    def __post_init__(self):
        if self.status not in GEOMETRY_STATUSES:
            raise VirtualAContractError(
                "INVALID_GEOMETRY_STATUS", "status={!r}".format(self.status))
        object.__setattr__(self, "attempts", tuple(self.attempts))

    @property
    def feasible(self):
        return self.status == "GEOMETRY_FEASIBLE"

    @property
    def request_fingerprint(self):
        return self.request_sha256

    def as_dict(self):
        path_summary = None
        if self.path is not None:
            try:
                path_summary = geometry_path_summary(self.path)
            except Exception as exc:  # report never hides path construction
                path_summary = {"summary_error": "{}: {}".format(
                    type(exc).__name__, exc)}
        payload = {
            "status": self.status,
            "reason": self.reason,
            "detail": self.detail,
            "request_sha256": self.request_sha256,
            "request_summary": self.request_summary,
            "config_summary": self.config_summary,
            "solver_used": self.solver_used,
            "goal_heading_branch_rad": self.goal_heading_branch_rad,
            "wall_time_s": self.wall_time_s,
            "deadline_exhausted": self.deadline_exhausted,
            "deadline_summary": self.deadline_summary,
            "initial_count": self.initial_count,
            "corridor_halfspace_constraint_count":
                self.corridor_halfspace_constraint_count,
            "corridor_max_residual_m": self.corridor_max_residual_m,
            "path": path_summary,
            "audit": self.audit.as_dict() if hasattr(
                self.audit, "as_dict") else None,
            "attempts": [attempt.as_dict() for attempt in self.attempts],
        }
        # f_only results keep their historical key set byte for byte.
        if self.formation_corridor_scope != FORMATION_CORRIDOR_SCOPE_F_ONLY:
            payload["formation_corridor_scope"] = self.formation_corridor_scope
            payload["formation_template_id"] = self.formation_template_id
            payload["formation_f_constraint_count"] = (
                self.formation_f_constraint_count)
            payload["formation_obb_constraint_count"] = (
                self.formation_obb_constraint_count)
            payload["formation_sample_audit"] = dict(
                self.formation_sample_audit)
            payload["formation_refinement"] = dict(
                self.formation_refinement)
            payload["formation_f_max_residual_m"] = (
                self.formation_f_max_residual_m)
            payload["formation_obb_max_residual_m"] = (
                self.formation_obb_max_residual_m)
            payload["formation_certificate"] = dict(
                self.formation_certificate)
        return payload


def geometry_path_summary(path):
    if path is None:
        return None
    summary = {
        "kind": getattr(path, "kind", None),
        "length_m": getattr(path, "length_m", None),
        "start_qf": _pose_summary(getattr(path, "start_qf", None)),
        "position_error_bound_m": getattr(
            path, "position_error_bound_m", None),
        "angle_error_bound_rad": getattr(
            path, "angle_error_bound_rad", None),
        "integration_budget": getattr(path, "integration_budget", None),
    }
    try:
        endpoint = path.endpoint()
        summary["endpoint"] = _pose_summary(endpoint)
    except Exception as exc:
        summary["endpoint"] = {"summary_error": "{}: {}".format(
            type(exc).__name__, exc)}
    profile = getattr(path, "profile", None)
    if profile is not None:
        try:
            summary["profile"] = profile.as_dict()
        except Exception as exc:
            summary["profile"] = {"summary_error": "{}: {}".format(
                type(exc).__name__, exc)}
        summary["segment_count"] = getattr(profile, "segment_count", None)
        summary["domain_segments"] = list(
            getattr(profile, "domain_segments", ()) or ())
    else:
        summary["profile"] = None
        summary["segment_count"] = 0
        summary["domain_segments"] = []
    return summary


@dataclass(frozen=True)
class TimingConfig:
    """Engineering settings for the common-time quintic retimer.

    The 600 s value is a G3 engineering budget, not the legacy S8
    ``max_block_duration_s``.  The dynamic limits are the original
    ``FormationTimingLimits`` carried by the request; none is relaxed here.
    """

    max_total_duration_s: float = 600.0
    safety_factor: float = 1.01
    max_rescalings: int = 8
    rescale_factor: float = 1.05
    interval_subdivisions_per_segment: int = 8
    max_subdivisions_per_segment: int = 4096
    max_total_subdivisions: int = 200000
    geometry_audit_config: GeometryAuditConfig = field(
        default_factory=GeometryAuditConfig)

    def as_dict(self):
        return {
            "max_total_duration_s": self.max_total_duration_s,
            "safety_factor": self.safety_factor,
            "max_rescalings": self.max_rescalings,
            "rescale_factor": self.rescale_factor,
            "interval_subdivisions_per_segment":
                self.interval_subdivisions_per_segment,
            "max_subdivisions_per_segment": self.max_subdivisions_per_segment,
            "max_total_subdivisions": self.max_total_subdivisions,
            "geometry_audit_config": self.geometry_audit_config.as_dict(),
            "timing_law": TIMING_LAW_VERSION,
        }


def validate_timing_config(config):
    if not isinstance(config, TimingConfig):
        return ValidationResult.fail(
            "INVALID_TIMING_CONFIG", "expected TimingConfig")
    for name in ("max_total_duration_s", "safety_factor", "rescale_factor"):
        value = getattr(config, name)
        if isinstance(value, bool) or not isinstance(value, numbers.Real):
            return ValidationResult.fail(
                "INVALID_TIMING_CONFIG",
                "{} must be a finite real, got {!r}".format(name, value),
                field=name)
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            return ValidationResult.fail(
                "INVALID_TIMING_CONFIG",
                "{} must be finite and positive".format(name), field=name)
    if float(config.safety_factor) < 1.0:
        return ValidationResult.fail(
            "INVALID_TIMING_CONFIG", "safety_factor must be >= 1.0",
            field="safety_factor")
    for name, minimum, maximum in (
            ("max_rescalings", 0, 8),
            ("interval_subdivisions_per_segment", 1, 1_000_000),
            ("max_subdivisions_per_segment", 1, 1_000_000),
            ("max_total_subdivisions", 1, 10_000_000)):
        value = getattr(config, name)
        if isinstance(value, bool) or not isinstance(value, numbers.Integral):
            return ValidationResult.fail(
                "INVALID_TIMING_CONFIG",
                "{} must be an integer, got {!r}".format(name, value),
                field=name)
        value = int(value)
        if value < minimum or value > maximum:
            return ValidationResult.fail(
                "INVALID_TIMING_CONFIG",
                "{}={} outside [{}, {}]".format(name, value, minimum, maximum),
                field=name)
    if float(config.rescale_factor) < 1.0:
        return ValidationResult.fail(
            "INVALID_TIMING_CONFIG", "rescale_factor must be >= 1.0",
            field="rescale_factor")
    geometry = validate_geometry_audit_config(config.geometry_audit_config)
    if not geometry.passed:
        return ValidationResult.fail(
            "INVALID_TIMING_CONFIG",
            "geometry_audit_config: {}".format(geometry.detail),
            field="geometry_audit_config",
            metrics=geometry.metrics)
    return ValidationResult.ok(
        max_total_duration_s=float(config.max_total_duration_s),
        safety_factor=float(config.safety_factor),
        max_rescalings=int(config.max_rescalings),
        rescale_factor=float(config.rescale_factor),
        interval_subdivisions_per_segment=int(
            config.interval_subdivisions_per_segment),
        max_subdivisions_per_segment=int(
            config.max_subdivisions_per_segment),
        max_total_subdivisions=int(config.max_total_subdivisions),
        timing_law=TIMING_LAW_VERSION)


@dataclass(frozen=True)
class TimingResult:
    """G3 retimer result; a plan is present only for a successful status."""

    status: str
    plan: object = None
    reason: str = ""
    detail: str = ""
    request_summary: dict = field(default_factory=dict)
    request_sha256: str = ""
    config_summary: dict = field(default_factory=dict)
    geometry_audit: object = None
    duration_s: float = None
    total_S_m: float = None
    T_s: float = None
    rescalings: int = 0
    bounds_summary: dict = field(default_factory=dict)
    wall_time_s: float = 0.0
    metrics: dict = field(default_factory=dict)

    def __post_init__(self):
        if self.status not in TIMING_STATUSES:
            raise VirtualAContractError(
                "INVALID_TIMING_STATUS", "status={!r}".format(self.status))
        object.__setattr__(self, "bounds_summary",
                           dict(sorted(self.bounds_summary.items())))
        object.__setattr__(self, "metrics", dict(sorted(self.metrics.items())))

    @property
    def feasible(self):
        return self.status in ("DYNAMIC_FEASIBLE", "NO_MOTION")

    @property
    def request_fingerprint(self):
        return self.request_sha256

    def as_dict(self):
        return {
            "status": self.status,
            "reason": self.reason,
            "detail": self.detail,
            "request_sha256": self.request_sha256,
            "request_summary": self.request_summary,
            "config_summary": self.config_summary,
            "wall_time_s": self.wall_time_s,
            "duration_s": self.duration_s,
            "total_S_m": self.total_S_m,
            "T_s": self.T_s,
            "rescalings": self.rescalings,
            "bounds_summary": self.bounds_summary,
            "metrics": self.metrics,
            "geometry_audit": self.geometry_audit.as_dict()
            if hasattr(self.geometry_audit, "as_dict") else None,
            "plan": self.plan.model_summary()
            if hasattr(self.plan, "model_summary") else None,
            "model_sha256": self.plan.model_sha256()
            if hasattr(self.plan, "model_sha256") else None,
        }


@dataclass(frozen=True)
class DynamicAuditConfig:
    """Independent dynamic-audit limits; the retimer's peaks are not trusted."""

    max_depth: int = 20
    max_leaves: int = 200000
    interval_subdivisions_per_segment: int = 8
    samples_per_segment: int = 5
    differential_dt_s: tuple = (0.01, 0.005)
    differential_samples_per_segment: int = 3
    velocity_tolerance: float = 1e-5
    angular_rate_tolerance: float = 1e-5
    acceleration_tolerance: float = 1e-4
    angular_accel_tolerance: float = 1e-4
    jerk_tolerance: float = 1e-2
    angular_jerk_tolerance: float = 1e-2
    time_inverse_tolerance_s: float = 1e-9
    state_tolerance: float = 1e-7
    endpoint_tolerance: float = 1e-9
    continuity_tolerance: float = 1e-6
    oracle_step_m: float = 0.002
    geometry_audit_config: GeometryAuditConfig = field(
        default_factory=GeometryAuditConfig)

    def as_dict(self):
        return {
            "max_depth": self.max_depth,
            "max_leaves": self.max_leaves,
            "interval_subdivisions_per_segment":
                self.interval_subdivisions_per_segment,
            "samples_per_segment": self.samples_per_segment,
            "differential_dt_s": list(self.differential_dt_s),
            "differential_samples_per_segment":
                self.differential_samples_per_segment,
            "velocity_tolerance": self.velocity_tolerance,
            "angular_rate_tolerance": self.angular_rate_tolerance,
            "acceleration_tolerance": self.acceleration_tolerance,
            "angular_accel_tolerance": self.angular_accel_tolerance,
            "jerk_tolerance": self.jerk_tolerance,
            "angular_jerk_tolerance": self.angular_jerk_tolerance,
            "time_inverse_tolerance_s": self.time_inverse_tolerance_s,
            "state_tolerance": self.state_tolerance,
            "endpoint_tolerance": self.endpoint_tolerance,
            "continuity_tolerance": self.continuity_tolerance,
            "oracle_step_m": self.oracle_step_m,
            "geometry_audit_config": self.geometry_audit_config.as_dict(),
        }


def validate_dynamic_audit_config(config):
    if not isinstance(config, DynamicAuditConfig):
        return ValidationResult.fail(
            "INVALID_AUDIT_CONFIG", "expected DynamicAuditConfig")
    for name, minimum, maximum in (
            ("max_depth", 0, 64),
            ("max_leaves", 1, 10_000_000),
            ("interval_subdivisions_per_segment", 1, 1_000_000),
            ("samples_per_segment", 1, 1_000_000),
            ("differential_samples_per_segment", 0, 1_000_000)):
        value = getattr(config, name)
        if isinstance(value, bool) or not isinstance(value, numbers.Integral):
            return ValidationResult.fail(
                "INVALID_AUDIT_CONFIG",
                "{} must be an integer, got {!r}".format(name, value),
                field=name)
        value = int(value)
        if value < minimum or value > maximum:
            return ValidationResult.fail(
                "INVALID_AUDIT_CONFIG",
                "{}={} outside [{}, {}]".format(name, value, minimum, maximum),
                field=name)
    for name in ("velocity_tolerance", "angular_rate_tolerance",
                 "acceleration_tolerance", "angular_accel_tolerance",
                 "jerk_tolerance", "angular_jerk_tolerance",
                 "time_inverse_tolerance_s", "state_tolerance",
                 "endpoint_tolerance", "continuity_tolerance",
                 "oracle_step_m"):
        value = getattr(config, name)
        if isinstance(value, bool) or not isinstance(value, numbers.Real):
            return ValidationResult.fail(
                "INVALID_AUDIT_CONFIG",
                "{} must be a finite real, got {!r}".format(name, value),
                field=name)
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            return ValidationResult.fail(
                "INVALID_AUDIT_CONFIG",
                "{} must be finite and positive".format(name), field=name)
    try:
        dt_values = tuple(float(value) for value in config.differential_dt_s)
    except (TypeError, ValueError, OverflowError) as exc:
        return ValidationResult.fail(
            "INVALID_AUDIT_CONFIG",
            "differential_dt_s is not a float tuple: {}".format(exc),
            field="differential_dt_s")
    if not dt_values:
        return ValidationResult.fail(
            "INVALID_AUDIT_CONFIG", "differential_dt_s is empty",
            field="differential_dt_s")
    for value in dt_values:
        if not math.isfinite(value) or value <= 0.0 or value > 0.1:
            return ValidationResult.fail(
                "INVALID_AUDIT_CONFIG",
                "differential dt={!r} outside (0, 0.1]".format(value),
                field="differential_dt_s")
    geometry = validate_geometry_audit_config(config.geometry_audit_config)
    if not geometry.passed:
        return ValidationResult.fail(
            "INVALID_AUDIT_CONFIG",
            "geometry_audit_config: {}".format(geometry.detail),
            field="geometry_audit_config",
            metrics=geometry.metrics)
    if float(config.oracle_step_m) > 0.01:
        return ValidationResult.fail(
            "INVALID_AUDIT_CONFIG",
            "oracle_step_m must be <= 0.01", field="oracle_step_m")
    return ValidationResult.ok(
        max_depth=int(config.max_depth),
        max_leaves=int(config.max_leaves),
        interval_subdivisions_per_segment=int(
            config.interval_subdivisions_per_segment),
        samples_per_segment=int(config.samples_per_segment),
        differential_dt_s=dt_values,
        differential_samples_per_segment=int(
            config.differential_samples_per_segment),
        oracle_step_m=float(config.oracle_step_m))


# ---------------------------------------------------------------------------
# G4 map/collision planning contracts
# ---------------------------------------------------------------------------

def _g4_canonical_json(value):
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False)


def _g4_normalize_json(value):
    return json.loads(_g4_canonical_json(value))


def _g4_sha256_bytes(payload):
    return hashlib.sha256(payload).hexdigest()


def _g4_sha256_json(value):
    return _g4_sha256_bytes(_g4_canonical_json(value).encode("utf-8"))


def grid_occupancy_sha256(grid):
    """SHA-256 of the exact row-major (iy, ix) occupancy bytes.

    The hash is over the binary occupancy array used by the collision checker,
    not over a rendered image or YAML text.  Two maps with the same cells,
    resolution and origin therefore have the same summary even if their PGM
    files use different metadata formatting.
    """
    try:
        occupancy = grid.occ
        data = occupancy.astype("uint8", copy=False).tobytes(order="C")
    except Exception as exc:
        raise VirtualAContractError(
            "INVALID_GRID", "cannot hash Grid.occupied: {}: {}".format(
                type(exc).__name__, exc)) from exc
    return _g4_sha256_bytes(data)


def grid_summary(grid):
    """Canonical, JSON-safe identity of a ``core.Grid`` instance."""
    try:
        height, width = (int(value) for value in grid.occ.shape)
        resolution = _finite(grid.res, "grid.res")
        origin_x = _finite(grid.ox, "grid.ox")
        origin_y = _finite(grid.oy, "grid.oy")
    except VirtualAContractError:
        raise
    except Exception as exc:
        raise VirtualAContractError(
            "INVALID_GRID", "unusable Grid: {}: {}".format(
                type(exc).__name__, exc)) from exc
    if height <= 0 or width <= 0:
        raise VirtualAContractError(
            "INVALID_GRID", "empty grid {}x{}".format(height, width))
    if resolution <= 0.0:
        raise VirtualAContractError(
            "INVALID_GRID", "resolution must be positive")
    return {
        "height": height,
        "width": width,
        "resolution_m": resolution,
        "origin_x": origin_x,
        "origin_y": origin_y,
        "occupancy_sha256": grid_occupancy_sha256(grid),
        "cell_count": height * width,
        "occupied_cells": int(sum(
            1 for value in grid.occ.ravel(order="C") if bool(value))),
    }


def grid_summary_sha256(summary):
    """SHA-256 of an already canonical grid summary."""
    return _g4_sha256_json(summary)


def grid_signature(grid):
    """SHA-256 of the canonical grid summary (used by :class:`GuideRoute`)."""
    return grid_summary_sha256(grid_summary(grid))


def _require_grid_signature(grid, reported, field="grid_sha256"):
    actual = grid_signature(grid)
    if reported != actual:
        raise VirtualAContractError(
            "GRID_DIGEST_MISMATCH",
            "{} differs from the supplied grid".format(field),
            expected=actual, reported=reported)


@dataclass(frozen=True)
class CollisionConfig:
    """Conservative sweep settings; none may be relaxed below the baseline.

    ``baseline_margin_m`` is the accepted 0.05 m safety margin.  The legacy
    checker halves the full-width margin into per-side half extents, so the
    numerical error envelope is reserved by adding twice its maximum to the
    full-width margin; it is never subtracted.  A caller cannot trade the
    sampling step for a smaller margin.
    """

    baseline_margin_m: float = 0.05
    max_corner_step_m: float = 0.01
    max_samples_per_segment: int = 200000

    def as_dict(self):
        return {
            "baseline_margin_m": float(self.baseline_margin_m),
            "max_corner_step_m": float(self.max_corner_step_m),
            "max_samples_per_segment": int(self.max_samples_per_segment),
            "baseline_margin_floor_m": 0.05,
            "max_corner_step_ceiling_m": 0.01,
            "sample_budget_ceiling": 200000,
        }


def validate_collision_config(config):
    if not isinstance(config, CollisionConfig):
        return ValidationResult.fail(
            "INVALID_COLLISION_CONFIG", "expected CollisionConfig")
    try:
        margin = _finite(config.baseline_margin_m, "baseline_margin_m")
        step = _finite(config.max_corner_step_m, "max_corner_step_m")
    except (TypeError, ValueError, VirtualAContractError) as exc:
        return ValidationResult.fail("INVALID_COLLISION_CONFIG", str(exc))
    budget = config.max_samples_per_segment
    if isinstance(budget, bool) or not isinstance(budget, numbers.Integral):
        return ValidationResult.fail(
            "INVALID_COLLISION_CONFIG",
            "max_samples_per_segment must be an integer",
            field="max_samples_per_segment")
    budget = int(budget)
    if margin < 0.05:
        return ValidationResult.fail(
            "COLLISION_MARGIN_RELAXED",
            "baseline_margin_m={} is below the accepted 0.05 m".format(margin),
            field="baseline_margin_m")
    if not math.isfinite(margin) or margin > 10.0:
        return ValidationResult.fail(
            "INVALID_COLLISION_CONFIG",
            "baseline_margin_m={} outside [0.05, 10]".format(margin),
            field="baseline_margin_m")
    if step <= 0.0 or step > 0.01:
        return ValidationResult.fail(
            "COLLISION_STEP_RELAXED",
            "max_corner_step_m={} outside (0, 0.01]".format(step),
            field="max_corner_step_m")
    if budget < 1 or budget > 200000:
        return ValidationResult.fail(
            "INVALID_COLLISION_CONFIG",
            "max_samples_per_segment={} outside [1, 200000]".format(budget),
            field="max_samples_per_segment")
    return ValidationResult.ok(
        baseline_margin_m=margin, max_corner_step_m=step,
        max_samples_per_segment=budget)


@dataclass(frozen=True)
class RouteGuideConfig:
    """Finite A* route-search settings; a guide is not an executable path."""

    max_routes: int = 3
    max_expansions: int = 200000
    max_seconds: float = 8.0
    clearance_weight: float = 0.5
    clearance_cap_m: float = 2.0
    alternative_penalty: float = 2.0
    blocked_radius_m: float = 0.15
    # F-point guide geometry only.  This is never a vehicle/cargo collision
    # margin and is deliberately independent of ``blocked_radius_m``.
    route_clearance_m: float = 0.10

    def as_dict(self):
        return {
            "max_routes": int(self.max_routes),
            "max_expansions": int(self.max_expansions),
            "max_seconds": float(self.max_seconds),
            "clearance_weight": float(self.clearance_weight),
            "clearance_cap_m": float(self.clearance_cap_m),
            "alternative_penalty": float(self.alternative_penalty),
            "alternative_penalty_definition": (
                "dimensionless V increment per previous-guide cell band; "
                "edge factor = 1 + clearance_weight*P(D) + V"),
            "blocked_radius_m": float(self.blocked_radius_m),
            "route_clearance_m": float(self.route_clearance_m),
            "route_clearance_definition": (
                "strict F-point guide clearance to occupied cells and the "
                "map boundary; not a vehicle collision margin"),
            "algorithm": "eight_neighbour_astar_distance_field",
            "guide_is_executable_path": False,
        }


def validate_route_guide_config(config):
    if not isinstance(config, RouteGuideConfig):
        return ValidationResult.fail(
            "INVALID_ROUTE_CONFIG", "expected RouteGuideConfig")
    for name, minimum, maximum in (
            ("max_routes", 1, 3), ("max_expansions", 100, 5000000)):
        value = getattr(config, name)
        if isinstance(value, bool) or not isinstance(value, numbers.Integral):
            return ValidationResult.fail(
                "INVALID_ROUTE_CONFIG",
                "{} must be an integer".format(name), field=name)
        if int(value) < minimum or int(value) > maximum:
            return ValidationResult.fail(
                "INVALID_ROUTE_CONFIG",
                "{}={} outside [{}, {}]".format(name, value, minimum, maximum),
                field=name)
    for name in ("max_seconds", "clearance_weight", "clearance_cap_m",
                 "alternative_penalty", "blocked_radius_m",
                 "route_clearance_m"):
        value = getattr(config, name)
        if isinstance(value, bool) or not isinstance(value, numbers.Real):
            return ValidationResult.fail(
                "INVALID_ROUTE_CONFIG",
                "{} must be a finite real".format(name), field=name)
        value = float(value)
        if not math.isfinite(value) or value < 0.0:
            return ValidationResult.fail(
                "INVALID_ROUTE_CONFIG",
                "{} must be finite and nonnegative".format(name), field=name)
    if float(config.max_seconds) <= 0.0 or float(config.max_seconds) > 60.0:
        return ValidationResult.fail(
            "INVALID_ROUTE_CONFIG", "max_seconds outside (0, 60]",
            field="max_seconds")
    if float(config.clearance_weight) > 1e6:
        return ValidationResult.fail(
            "INVALID_ROUTE_CONFIG", "clearance_weight is unreasonable",
            field="clearance_weight")
    if float(config.clearance_cap_m) <= 0.0:
        return ValidationResult.fail(
            "INVALID_ROUTE_CONFIG", "clearance_cap_m must be positive",
            field="clearance_cap_m")
    if float(config.blocked_radius_m) > 5.0:
        return ValidationResult.fail(
            "INVALID_ROUTE_CONFIG", "blocked_radius_m too large",
            field="blocked_radius_m")
    if float(config.route_clearance_m) > 5.0:
        return ValidationResult.fail(
            "INVALID_ROUTE_CONFIG", "route_clearance_m too large",
            field="route_clearance_m")
    return ValidationResult.ok(**{
        "max_routes": int(config.max_routes),
        "max_expansions": int(config.max_expansions),
        "max_seconds": float(config.max_seconds),
        "clearance_weight": float(config.clearance_weight),
        "clearance_cap_m": float(config.clearance_cap_m),
        "alternative_penalty": float(config.alternative_penalty),
        "blocked_radius_m": float(config.blocked_radius_m),
        "route_clearance_m": float(config.route_clearance_m),
    })


@dataclass(frozen=True)
class GuideRoute:
    """A finite F-point guide polyline; it carries no vehicle reference path.

    ``quality`` is optional construction evidence for a corridor centreline
    generated by C2.  A* and simplification guides leave it ``None`` and keep
    their original serialisation.  It never makes a guide executable.
    """

    points: tuple
    source: str
    grid_sha256: str
    quality: MappingABC = field(default=None, compare=False)

    def __post_init__(self):
        try:
            points = tuple((float(point[0]), float(point[1]))
                           for point in self.points)
        except (TypeError, ValueError, IndexError) as exc:
            raise VirtualAContractError(
                "INVALID_GUIDE_ROUTE",
                "points must be (x, y) pairs") from exc
        if len(points) < 2:
            raise VirtualAContractError(
                "INVALID_GUIDE_ROUTE", "a guide needs at least two points")
        for x, y in points:
            if not (math.isfinite(x) and math.isfinite(y)):
                raise VirtualAContractError(
                    "INVALID_GUIDE_ROUTE", "guide point is not finite")
        if not isinstance(self.source, str) or not self.source:
            raise VirtualAContractError(
                "INVALID_GUIDE_ROUTE", "source must be a non-empty string")
        if not isinstance(self.grid_sha256, str) or len(self.grid_sha256) != 64:
            raise VirtualAContractError(
                "INVALID_GUIDE_ROUTE", "grid_sha256 must be 64 hex characters")
        if self.quality is not None:
            if not isinstance(self.quality, MappingABC):
                raise VirtualAContractError(
                    "INVALID_GUIDE_ROUTE", "quality must be a mapping")
            object.__setattr__(self, "quality", dict(self.quality))
        object.__setattr__(self, "points", points)

    @property
    def length_m(self):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(self.points, self.points[1:]))

    @property
    def start(self):
        return self.points[0]

    @property
    def goal(self):
        return self.points[-1]

    def as_dict(self):
        payload = {
            "source": self.source,
            "grid_sha256": self.grid_sha256,
            "points": [list(point) for point in self.points],
            "length_m": self.length_m,
            "is_executable_path": False,
        }
        if self.quality is not None:
            payload["quality"] = dict(self.quality)
        return payload


@dataclass(frozen=True)
class ConvexCorridor:
    """One closed convex separating-hyperplane corridor region.

    ``A`` rows are outward unit normals and ``b`` the matching halfspace
    limits, so the region is ``{p : A p <= b}``.  ``vertices`` are the closed
    polygon in canonical counter-clockwise order.  The seed is the guide
    segment the region was built for; a corridor is reference-point geometry
    only and carries no three-vehicle safety claim.
    """

    seed_start: tuple
    seed_end: tuple
    A: tuple
    b: tuple
    vertices: tuple

    def __post_init__(self):
        try:
            seed_start = (float(self.seed_start[0]), float(self.seed_start[1]))
            seed_end = (float(self.seed_end[0]), float(self.seed_end[1]))
            rows = tuple((float(row[0]), float(row[1])) for row in self.A)
            limits = tuple(float(value) for value in self.b)
            vertices = tuple(
                (float(vertex[0]), float(vertex[1]))
                for vertex in self.vertices)
        except (TypeError, ValueError, IndexError) as exc:
            raise VirtualAContractError(
                "INVALID_CONVEX_CORRIDOR",
                "corridor fields must be numeric pairs") from exc
        for x, y in (seed_start, seed_end):
            if not (math.isfinite(x) and math.isfinite(y)):
                raise VirtualAContractError(
                    "INVALID_CONVEX_CORRIDOR", "seed is not finite")
        if len(rows) != len(limits) or len(rows) < 3:
            raise VirtualAContractError(
                "INVALID_CONVEX_CORRIDOR",
                "A and b must have equal length >= 3")
        for x, y in rows:
            if not (math.isfinite(x) and math.isfinite(y)):
                raise VirtualAContractError(
                    "INVALID_CONVEX_CORRIDOR", "normal is not finite")
            if abs(math.hypot(x, y) - 1.0) > 1e-9:
                raise VirtualAContractError(
                    "INVALID_CONVEX_CORRIDOR", "normal is not unit length")
        for value in limits:
            if not math.isfinite(value):
                raise VirtualAContractError(
                    "INVALID_CONVEX_CORRIDOR", "halfspace limit is not finite")
        if len(vertices) < 3:
            raise VirtualAContractError(
                "INVALID_CONVEX_CORRIDOR",
                "a corridor polygon needs at least three vertices")
        for x, y in vertices:
            if not (math.isfinite(x) and math.isfinite(y)):
                raise VirtualAContractError(
                    "INVALID_CONVEX_CORRIDOR", "vertex is not finite")
        twice_area = sum(
            x0 * y1 - x1 * y0
            for (x0, y0), (x1, y1) in zip(
                vertices, vertices[1:] + vertices[:1]))
        if not twice_area > 0.0:
            raise VirtualAContractError(
                "INVALID_CONVEX_CORRIDOR",
                "vertices must be counter-clockwise with positive area")
        object.__setattr__(self, "seed_start", seed_start)
        object.__setattr__(self, "seed_end", seed_end)
        object.__setattr__(self, "A", rows)
        object.__setattr__(self, "b", limits)
        object.__setattr__(self, "vertices", vertices)

    @property
    def area_m2(self):
        return 0.5 * sum(
            x0 * y1 - x1 * y0
            for (x0, y0), (x1, y1) in zip(
                self.vertices, self.vertices[1:] + self.vertices[:1]))

    def as_dict(self):
        return {
            "seed_start": [self.seed_start[0], self.seed_start[1]],
            "seed_end": [self.seed_end[0], self.seed_end[1]],
            "A": [[row[0], row[1]] for row in self.A],
            "b": [value for value in self.b],
            "vertices": [[x, y] for x, y in self.vertices],
            "area_m2": self.area_m2,
        }


@dataclass(frozen=True)
class CorridorChain:
    """An ordered chain of convex corridors built from one guide route."""

    regions: tuple
    route_source: str
    grid_sha256: str

    def __post_init__(self):
        regions = tuple(self.regions)
        if not regions or any(
                not isinstance(region, ConvexCorridor)
                for region in regions):
            raise VirtualAContractError(
                "INVALID_CORRIDOR_CHAIN",
                "regions must be a non-empty sequence of ConvexCorridor")
        if not isinstance(self.route_source, str) or not self.route_source:
            raise VirtualAContractError(
                "INVALID_CORRIDOR_CHAIN",
                "route_source must be a non-empty string")
        if not isinstance(self.grid_sha256, str) or len(self.grid_sha256) != 64:
            raise VirtualAContractError(
                "INVALID_CORRIDOR_CHAIN",
                "grid_sha256 must be 64 hex characters")
        object.__setattr__(self, "regions", regions)

    def _payload(self):
        return {
            "regions": [region.as_dict() for region in self.regions],
            "route_source": self.route_source,
            "grid_sha256": self.grid_sha256,
            "is_executable_path": False,
        }

    @property
    def digest(self):
        payload = json.dumps(
            self._payload(), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()

    def as_dict(self):
        payload = self._payload()
        payload["digest"] = self.digest
        return payload


def _formation_sha256_hex(value):
    return isinstance(value, str) and len(value) == 64 and all(
        character in "0123456789abcdef" for character in value)


def _formation_finite(value, context):
    return _finite(value, context)


@dataclass(frozen=True)
class FormationCorridorVehicleFrame:
    """One vehicle's fixed offset and body rectangle for a corridor template.

    The frame is copied from the request so a template can validate its own
    local convex pieces without trusting a caller-supplied shape.  It is still
    bound to the request by ``model_sha256``.
    """

    vehicle_id: int
    offset_x: float
    offset_y: float
    length_m: float
    width_m: float

    def __post_init__(self):
        if isinstance(self.vehicle_id, bool) or not isinstance(
                self.vehicle_id, int):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "vehicle_id must be an integer, got {!r}".format(
                    self.vehicle_id))
        object.__setattr__(self, "offset_x", _formation_finite(
            self.offset_x, "frame.offset_x"))
        object.__setattr__(self, "offset_y", _formation_finite(
            self.offset_y, "frame.offset_y"))
        length = _formation_finite(self.length_m, "frame.length_m")
        width = _formation_finite(self.width_m, "frame.width_m")
        if length <= 0.0 or width <= 0.0:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "frame length/width must be positive")
        object.__setattr__(self, "length_m", length)
        object.__setattr__(self, "width_m", width)

    @property
    def local_corners(self):
        half_length = 0.5 * self.length_m
        half_width = 0.5 * self.width_m
        return (
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width),
        )

    def as_dict(self):
        return {
            "vehicle_id": int(self.vehicle_id),
            "offset_x": float(self.offset_x),
            "offset_y": float(self.offset_y),
            "length_m": float(self.length_m),
            "width_m": float(self.width_m),
        }


@dataclass(frozen=True)
class FormationCorridorPiece:
    """One closed convex body-local piece assigned to one convex region."""

    region_index: int
    local_vertices: tuple

    def __post_init__(self):
        if isinstance(self.region_index, bool) or not isinstance(
                self.region_index, int) or self.region_index < 0:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "piece region_index must be a nonnegative integer")
        try:
            vertices = tuple(
                (_formation_finite(vertex[0], "piece.vertex.x"),
                 _formation_finite(vertex[1], "piece.vertex.y"))
                for vertex in self.local_vertices)
        except (TypeError, ValueError, IndexError) as exc:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "piece vertices must be finite (x, y) pairs") from exc
        if len(vertices) < 3:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "a convex piece needs at least three vertices")
        area = 0.5 * sum(
            x0 * y1 - x1 * y0
            for (x0, y0), (x1, y1) in zip(
                vertices, vertices[1:] + vertices[:1]))
        if not area > 0.0:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "piece vertices must be counter-clockwise with positive area")
        object.__setattr__(self, "local_vertices", vertices)

    @property
    def area_m2(self):
        vertices = self.local_vertices
        return 0.5 * sum(
            x0 * y1 - x1 * y0
            for (x0, y0), (x1, y1) in zip(
                vertices, vertices[1:] + vertices[:1]))

    def as_dict(self):
        return {
            "region_index": int(self.region_index),
            "local_vertices": [
                [float(x), float(y)] for x, y in self.local_vertices],
            "area_m2": self.area_m2,
        }


@dataclass(frozen=True)
class FormationCorridorVehicleCover:
    """A finite decomposition of one vehicle rectangle into convex pieces."""

    vehicle_id: int
    decomposition: str
    pieces: tuple

    def __post_init__(self):
        if isinstance(self.vehicle_id, bool) or not isinstance(
                self.vehicle_id, int):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "cover vehicle_id must be an integer")
        if self.decomposition not in FORMATION_CORRIDOR_DECOMPOSITIONS:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "cover decomposition={!r} allowed={}".format(
                    self.decomposition, FORMATION_CORRIDOR_DECOMPOSITIONS))
        pieces = tuple(self.pieces)
        expected = 1 if self.decomposition == "whole" else 2
        if len(pieces) != expected or any(
                not isinstance(piece, FormationCorridorPiece)
                for piece in pieces):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "decomposition {!r} requires {} FormationCorridorPiece "
                "entries".format(self.decomposition, expected))
        object.__setattr__(self, "pieces", pieces)

    def as_dict(self):
        return {
            "vehicle_id": int(self.vehicle_id),
            "decomposition": str(self.decomposition),
            "pieces": [piece.as_dict() for piece in self.pieces],
        }


def _formation_cover_canonical(cover, frame):
    """Check exact finite-template coverage against one frame's rectangle."""
    corners = frame.local_corners
    if cover.decomposition == "whole":
        expected = (corners,)
    elif cover.decomposition == "diagonal_0":
        expected = (
            (corners[0], corners[1], corners[2]),
            (corners[0], corners[2], corners[3]),
        )
    else:
        expected = (
            (corners[1], corners[2], corners[3]),
            (corners[1], corners[3], corners[0]),
        )
    for piece, expected_vertices in zip(cover.pieces, expected):
        if len(piece.local_vertices) != len(expected_vertices):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "vehicle {} decomposition {!r} piece vertex count is not the "
                "canonical one".format(cover.vehicle_id, cover.decomposition))
        for actual, expected_vertex in zip(
                piece.local_vertices, expected_vertices):
            if abs(actual[0] - expected_vertex[0]) > FORMATION_TOLERANCE_M \
                    or abs(actual[1] - expected_vertex[1]) > \
                    FORMATION_TOLERANCE_M:
                raise VirtualAContractError(
                    "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                    "vehicle {} decomposition {!r} does not use the exact "
                    "canonical corner decomposition".format(
                        cover.vehicle_id, cover.decomposition))
        # Every convex piece must be a strict subset of the rectangle: its
        # vertices are canonical corners, so this also rules out invented
        # pieces with the right area but the wrong shape.
    region_indices = tuple(piece.region_index for piece in cover.pieces)
    unique = sorted(set(region_indices))
    if len(unique) > 2:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "one vehicle cover references more than two regions")
    if len(unique) == 2 and unique[1] - unique[0] != 1:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "two-region cover must name adjacent regions, got {}".format(
                unique))


@dataclass(frozen=True)
class FormationCorridorTemplateSegment:
    """One contiguous sigma interval with fixed per-vehicle assignments.

    ``f_region_index`` is the single convex Chain region assigned to the F
    reference point for this whole open interval.  It is a finite template
    decision exactly like the per-vehicle covers, not a certificate: the
    continuous certificate re-checks the F point against this same region and
    the two-sided cut checks require both neighbouring assignments to hold at
    a shared boundary.  The contract validates the index range and adjacency
    against the audited Chain in :func:`validate_formation_template`.
    """

    sigma_start: float
    sigma_end: float
    covers: tuple
    f_region_index: int

    def __post_init__(self):
        start = _formation_finite(self.sigma_start, "segment.sigma_start")
        end = _formation_finite(self.sigma_end, "segment.sigma_end")
        if not (0.0 <= start < end <= 1.0):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "template segment requires 0 <= sigma_start < sigma_end <= 1")
        if isinstance(self.f_region_index, bool) \
                or not isinstance(self.f_region_index, int) \
                or self.f_region_index < 0:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "segment f_region_index must be a nonnegative integer")
        covers = tuple(self.covers)
        if not covers or any(
                not isinstance(cover, FormationCorridorVehicleCover)
                for cover in covers):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "segment covers must be a non-empty FormationCorridorVehicleCover "
                "sequence")
        ids = [cover.vehicle_id for cover in covers]
        if len(ids) != len(set(ids)):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "segment has duplicate vehicle covers")
        object.__setattr__(self, "sigma_start", start)
        object.__setattr__(self, "sigma_end", end)
        object.__setattr__(self, "covers", covers)
        object.__setattr__(self, "f_region_index", int(self.f_region_index))

    def as_dict(self):
        return {
            "sigma_start": float(self.sigma_start),
            "sigma_end": float(self.sigma_end),
            "f_region_index": int(self.f_region_index),
            "covers": [cover.as_dict() for cover in self.covers],
        }


@dataclass(frozen=True)
class FormationCorridorTemplate:
    """Immutable finite OBB coverage template and its complete identity.

    The template is a conservative sufficient condition for an OBB to lie in
    one convex region or the union of an adjacent pair.  It is never a claim
    that the union containment problem has been solved generally.  Every
    piece is a canonical part of the exact vehicle rectangle; the sigma
    segments cover ``[0, 1]`` without gaps or overlaps.
    """

    request_sha256: str
    grid_sha256: str
    planning_context_sha256: str
    model_sha256: str
    chain_digest: str
    route_index: int
    candidate_index: int
    guide_source: str
    guide_sha256: str
    allocation_digest: str
    margin_m: float
    margin_source: str
    vehicle_frames: tuple
    segments: tuple
    schema_version: str = FORMATION_CORRIDOR_TEMPLATE_SCHEMA
    compiler_version: str = FORMATION_CORRIDOR_COMPILER_VERSION
    certificate_version: str = FORMATION_CORRIDOR_CERTIFICATE_VERSION

    def __post_init__(self):
        if self.schema_version != FORMATION_CORRIDOR_TEMPLATE_SCHEMA:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "unknown schema_version={!r}".format(self.schema_version))
        if self.compiler_version != FORMATION_CORRIDOR_COMPILER_VERSION:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "unknown compiler_version={!r}".format(
                    self.compiler_version))
        if self.certificate_version != FORMATION_CORRIDOR_CERTIFICATE_VERSION:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "unknown certificate_version={!r}".format(
                    self.certificate_version))
        for name, value in (
                ("request_sha256", self.request_sha256),
                ("grid_sha256", self.grid_sha256),
                ("planning_context_sha256", self.planning_context_sha256),
                ("model_sha256", self.model_sha256),
                ("chain_digest", self.chain_digest),
                ("guide_sha256", self.guide_sha256),
                ("allocation_digest", self.allocation_digest)):
            if not _formation_sha256_hex(value):
                raise VirtualAContractError(
                    "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                    "{} must be 64 lowercase hex characters".format(name))
        for name, value in (
                ("route_index", self.route_index),
                ("candidate_index", self.candidate_index)):
            if isinstance(value, bool) or not isinstance(value, int) \
                    or value < 0:
                raise VirtualAContractError(
                    "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                    "{} must be a nonnegative integer".format(name))
        if not isinstance(self.guide_source, str) or not self.guide_source:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "guide_source must be a non-empty string")
        margin = _formation_finite(self.margin_m, "template.margin_m")
        if margin < 0.0:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "margin_m must be nonnegative")
        if not isinstance(self.margin_source, str) or not self.margin_source:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "margin_source must be a non-empty string")
        object.__setattr__(self, "margin_m", margin)
        frames = tuple(self.vehicle_frames)
        if not frames or any(
                not isinstance(frame, FormationCorridorVehicleFrame)
                for frame in frames):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "vehicle_frames must be a non-empty frame sequence")
        frame_ids = [frame.vehicle_id for frame in frames]
        if len(frame_ids) != len(set(frame_ids)):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "vehicle_frames contains duplicate vehicle ids")
        object.__setattr__(self, "vehicle_frames", frames)
        segments = tuple(self.segments)
        if not segments or any(
                not isinstance(segment, FormationCorridorTemplateSegment)
                for segment in segments):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "segments must be a non-empty template-segment sequence")
        if abs(segments[0].sigma_start) > FORMATION_TOLERANCE_M:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "template segments must start at sigma=0")
        if abs(segments[-1].sigma_end - 1.0) > FORMATION_TOLERANCE_M:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "template segments must end at sigma=1")
        for previous, current in zip(segments, segments[1:]):
            if abs(previous.sigma_end - current.sigma_start) > \
                    FORMATION_TOLERANCE_M:
                raise VirtualAContractError(
                    "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                    "template sigma segments have a gap or overlap: "
                    "{!r} -> {!r}".format(
                        previous.sigma_end, current.sigma_start))
        frame_by_id = {frame.vehicle_id: frame for frame in frames}
        expected_ids = set(frame_by_id)
        for segment in segments:
            cover_ids = [cover.vehicle_id for cover in segment.covers]
            if set(cover_ids) != expected_ids:
                raise VirtualAContractError(
                    "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                    "segment vehicle covers {} do not match frames {}".format(
                        sorted(cover_ids), sorted(expected_ids)))
            for cover in segment.covers:
                _formation_cover_canonical(cover, frame_by_id[
                    cover.vehicle_id])
        object.__setattr__(self, "segments", segments)

    def _payload(self):
        return {
            "schema_version": self.schema_version,
            "compiler_version": self.compiler_version,
            "certificate_version": self.certificate_version,
            "request_sha256": self.request_sha256,
            "grid_sha256": self.grid_sha256,
            "planning_context_sha256": self.planning_context_sha256,
            "model_sha256": self.model_sha256,
            "chain_digest": self.chain_digest,
            "route_index": int(self.route_index),
            "candidate_index": int(self.candidate_index),
            "guide_source": self.guide_source,
            "guide_sha256": self.guide_sha256,
            "allocation_digest": self.allocation_digest,
            "margin_m": float(self.margin_m),
            "margin_source": self.margin_source,
            "vehicle_frames": [
                frame.as_dict() for frame in self.vehicle_frames],
            "segments": [segment.as_dict() for segment in self.segments],
        }

    @property
    def template_id(self):
        return _g4_sha256_json(self._payload())

    def as_dict(self):
        payload = self._payload()
        payload["template_id"] = self.template_id
        return payload


@dataclass(frozen=True)
class CachedClearanceField:
    """Coarse immutable clearance cache for the NLP guidance proxy.

    ``values[j][i]`` is a distance-to-nearest-occupied-cell sample at the cell
    centre ``(origin_x + i * resolution_m, origin_y + j * resolution_m)``.
    Rows run bottom-to-top.  The field is built from the authoritative grid
    once and is only a smooth objective proxy; every final decision is still
    made by the unchanged real three-vehicle analytic sweep.
    """

    origin_x: float
    origin_y: float
    resolution_m: float
    grid_sha256: str
    values: tuple
    downsample: int = 1

    def __post_init__(self):
        origin_x = _finite(self.origin_x, "origin_x")
        origin_y = _finite(self.origin_y, "origin_y")
        resolution = _finite(self.resolution_m, "resolution_m")
        if resolution <= 0.0:
            raise VirtualAContractError(
                "INVALID_CLEARANCE_FIELD",
                "resolution_m must be positive")
        if not isinstance(self.grid_sha256, str) \
                or len(self.grid_sha256) != 64:
            raise VirtualAContractError(
                "INVALID_CLEARANCE_FIELD",
                "grid_sha256 must be 64 hex characters")
        downsample = self.downsample
        if isinstance(downsample, bool) or not isinstance(
                downsample, numbers.Integral) or int(downsample) <= 0:
            raise VirtualAContractError(
                "INVALID_CLEARANCE_FIELD",
                "downsample must be a positive integer")
        if not isinstance(self.values, (tuple, list)) or not self.values:
            raise VirtualAContractError(
                "INVALID_CLEARANCE_FIELD", "values must be a non-empty table")
        rows = []
        width = None
        for row in self.values:
            if not isinstance(row, (tuple, list)) or not row:
                raise VirtualAContractError(
                    "INVALID_CLEARANCE_FIELD",
                    "every values row must be a non-empty sequence")
            if width is None:
                width = len(row)
            elif len(row) != width:
                raise VirtualAContractError(
                    "INVALID_CLEARANCE_FIELD",
                    "values rows must have equal length")
            converted = []
            for entry in row:
                value = _finite(entry, "clearance value")
                if value < 0.0:
                    raise VirtualAContractError(
                        "INVALID_CLEARANCE_FIELD",
                        "clearance values must be nonnegative")
                converted.append(value)
            rows.append(tuple(converted))
        object.__setattr__(self, "origin_x", origin_x)
        object.__setattr__(self, "origin_y", origin_y)
        object.__setattr__(self, "resolution_m", resolution)
        object.__setattr__(self, "downsample", int(downsample))
        object.__setattr__(self, "values", tuple(rows))

    @property
    def height(self):
        return len(self.values)

    @property
    def width(self):
        return len(self.values[0])

    def value_at(self, x, y):
        """Bilinearly sample the cached field; edge-clamped, never raises."""
        fx = (float(x) - self.origin_x) / self.resolution_m
        fy = (float(y) - self.origin_y) / self.resolution_m
        fx = min(max(fx, 0.0), self.width - 1.0)
        fy = min(max(fy, 0.0), self.height - 1.0)
        i0 = min(int(math.floor(fx)), self.width - 1)
        j0 = min(int(math.floor(fy)), self.height - 1)
        i1 = min(i0 + 1, self.width - 1)
        j1 = min(j0 + 1, self.height - 1)
        tx = fx - i0
        ty = fy - j0
        bottom = self.values[j0][i0] * (1.0 - tx) + self.values[j0][i1] * tx
        top = self.values[j1][i0] * (1.0 - tx) + self.values[j1][i1] * tx
        return float(bottom * (1.0 - ty) + top * ty)

    def direction_at(self, x, y, step=None):
        """Return ``(clearance, direction_to_obstacle_rad, gradient_norm)``."""
        clearance = self.value_at(x, y)
        if step is None:
            step = 0.25 * self.resolution_m
        step = float(step)
        if not math.isfinite(step) or step <= 0.0:
            step = 0.25 * self.resolution_m
        gx = (self.value_at(float(x) + step, y)
              - self.value_at(float(x) - step, y)) / (2.0 * step)
        gy = (self.value_at(x, float(y) + step)
              - self.value_at(x, float(y) - step)) / (2.0 * step)
        norm = math.hypot(gx, gy)
        if norm <= 1e-9:
            return clearance, None, norm
        return clearance, math.atan2(-gy, -gx), norm

    def sha256(self):
        digest = hashlib.sha256()
        digest.update(repr((
            self.origin_x, self.origin_y, self.resolution_m,
            self.grid_sha256, self.downsample, self.height, self.width
        )).encode("utf-8"))
        for row in self.values:
            for value in row:
                digest.update(struct.pack("<d", value))
        return digest.hexdigest()

    def as_dict(self):
        return {
            "origin_x": float(self.origin_x),
            "origin_y": float(self.origin_y),
            "resolution_m": float(self.resolution_m),
            "grid_sha256": self.grid_sha256,
            "downsample": int(self.downsample),
            "height": int(self.height),
            "width": int(self.width),
            "field_sha256": self.sha256(),
            "role": "smooth_nlp_collision_guidance_only",
            "is_collision_certificate": False,
        }


@dataclass(frozen=True)
class ObstacleContext:
    """Optional collision-driven re-solve targets; never a collision pass.

    A collision audit may fail and the pipeline may ask the optimiser to
    re-solve with nodes pushed away from the reported location.  These are
    soft guidance targets; the final authority remains a complete real sweep.
    """

    grid_sha256: str
    node_targets: tuple = ()
    source: str = ""
    collision_station: float = None
    collision_reason: str = ""
    blocked_points: tuple = ()
    # A collision retry may relax the soft route-position objective when the
    # strong guide itself would demand an infeasible terminal turn.  This is a
    # guidance scale only; the endpoint, C2, curvature and all audits remain
    # unchanged and apply to the resulting candidate.
    route_weight_scale: float = 1.0
    # Cached distance samples let the NLP penalise a body footprint that the
    # centreline guide alone would allow to overlap an obstacle.  Optional so
    # the G2 call path keeps working without a map.
    clearance_field: object = None

    def __post_init__(self):
        if not isinstance(self.grid_sha256, str) or len(self.grid_sha256) != 64:
            raise VirtualAContractError(
                "INVALID_OBSTACLE_CONTEXT",
                "grid_sha256 must be 64 hex characters")
        targets = []
        for entry in self.node_targets:
            if not isinstance(entry, MappingABC):
                raise VirtualAContractError(
                    "INVALID_OBSTACLE_CONTEXT", "node target must be a mapping")
            index = entry.get("index")
            x = entry.get("x")
            y = entry.get("y")
            radius = entry.get("radius_m")
            if isinstance(index, bool) or not isinstance(index, numbers.Integral):
                raise VirtualAContractError(
                    "INVALID_OBSTACLE_CONTEXT", "node index must be an integer")
            for value in (x, y, radius):
                if isinstance(value, bool) or not isinstance(value, numbers.Real) \
                        or not math.isfinite(float(value)):
                    raise VirtualAContractError(
                        "INVALID_OBSTACLE_CONTEXT",
                        "node target coordinates/radius must be finite")
            if float(radius) < 0.0:
                raise VirtualAContractError(
                    "INVALID_OBSTACLE_CONTEXT", "radius must be nonnegative")
            targets.append(dict(sorted({
                "index": int(index), "x": float(x), "y": float(y),
                "radius_m": float(radius)}.items())))
        points = tuple((float(point[0]), float(point[1]))
                       for point in self.blocked_points)
        for x, y in points:
            if not (math.isfinite(x) and math.isfinite(y)):
                raise VirtualAContractError(
                    "INVALID_OBSTACLE_CONTEXT", "blocked point is not finite")
        if self.collision_station is not None:
            value = _finite(self.collision_station, "collision_station")
        else:
            value = None
        scale = _finite(self.route_weight_scale, "route_weight_scale")
        if scale <= 0.0 or scale > 1.0:
            raise VirtualAContractError(
                "INVALID_OBSTACLE_CONTEXT",
                "route_weight_scale must be in (0, 1]")
        field = self.clearance_field
        if field is not None:
            if not isinstance(field, CachedClearanceField):
                raise VirtualAContractError(
                    "INVALID_OBSTACLE_CONTEXT",
                    "clearance_field must be a CachedClearanceField")
            if field.grid_sha256 != self.grid_sha256:
                raise VirtualAContractError(
                    "INVALID_OBSTACLE_CONTEXT",
                    "clearance field is not bound to the context grid")
        object.__setattr__(self, "node_targets", tuple(targets))
        object.__setattr__(self, "blocked_points", points)
        object.__setattr__(self, "collision_station", value)
        object.__setattr__(self, "route_weight_scale", scale)
        object.__setattr__(self, "clearance_field", field)

    def as_dict(self):
        return {
            "grid_sha256": self.grid_sha256,
            "node_targets": [dict(entry) for entry in self.node_targets],
            "source": self.source,
            "collision_station": self.collision_station,
            "collision_reason": self.collision_reason,
            "blocked_points": [list(point) for point in self.blocked_points],
            "route_weight_scale": float(self.route_weight_scale),
            "clearance_field": self.clearance_field.as_dict()
            if self.clearance_field is not None else None,
        }


@dataclass(frozen=True)
class VirtualAMapRequest:
    """G4 wrapper: physical request, authoritative grid and three configs."""

    request: object
    grid: object
    planner_config: object
    timing_config: object
    collision_config: object

    def __post_init__(self):
        if not isinstance(self.planner_config, PlannerConfig):
            raise VirtualAContractError(
                "INVALID_MAP_REQUEST", "planner_config must be PlannerConfig")
        if not isinstance(self.timing_config, TimingConfig):
            raise VirtualAContractError(
                "INVALID_MAP_REQUEST", "timing_config must be TimingConfig")
        if not isinstance(self.collision_config, CollisionConfig):
            raise VirtualAContractError(
                "INVALID_MAP_REQUEST",
                "collision_config must be CollisionConfig")

    @property
    def request_sha256(self):
        try:
            return request_fingerprint(self.request)
        except Exception:
            return ""

    @property
    def grid_summary(self):
        return grid_summary(self.grid)

    @property
    def grid_sha256(self):
        return grid_signature(self.grid)

    def context(self):
        return planning_context(
            self.request, self.grid, self.planner_config,
            self.timing_config, self.collision_config)

    def as_dict(self):
        return {
            "grid": self.grid_summary,
            "request": request_summary(self.request),
            "planner_config": self.planner_config.as_dict(),
            "timing_config": self.timing_config.as_dict(),
            "collision_config": self.collision_config.as_dict(),
        }


def planning_context(request, grid, planner_config, timing_config,
                     collision_config):
    """Canonical G4 context bound into every timed model and audit."""
    request_value = request_fingerprint(request)
    return _g4_normalize_json({
        "schema": G4_PLANNING_SCHEMA,
        "stage": "G4",
        "request_sha256": request_value,
        "grid": grid_summary(grid),
        "planner_config": planner_config.as_dict(),
        "timing_config": timing_config.as_dict(),
        "collision_config": collision_config.as_dict(),
    })


def planning_context_sha256(context):
    if context is None:
        return ""
    if not isinstance(context, MappingABC):
        raise VirtualAContractError(
            "INVALID_PLANNING_CONTEXT", "context must be a mapping")
    return _g4_sha256_json(context)


@dataclass(frozen=True)
class VirtualAResult:
    """G4 end-to-end result; executable only with all three audits passed."""

    status: str
    reason: str = ""
    detail: str = ""
    first_failure_layer: str = ""
    request_sha256: str = ""
    planning_context_sha256: str = ""
    grid_sha256: str = ""
    grid_summary: dict = field(default_factory=dict)
    model_sha256: str = ""
    path: object = None
    plan: object = None
    projection: object = None
    geometry_audit: object = None
    dynamic_audit: object = None
    collision_audit: object = None
    attempts: tuple = ()
    routes: tuple = ()
    search: dict = field(default_factory=dict)
    wall_time_s: float = 0.0
    timings: dict = field(default_factory=dict)
    metrics: dict = field(default_factory=dict)
    checks: int = 0
    # TASK03 Step 6: versioned formation corridor scope.  ``f_only`` keeps the
    # historical serialized key set; a union result carries the finite
    # template identity, the sealed continuous certificate that authorised it
    # and the audit block that binds both to the accepted path.
    formation_corridor_scope: str = FORMATION_CORRIDOR_SCOPE_F_ONLY
    formation_corridor_audit: dict = field(default_factory=dict)
    formation_template_id: str = ""
    formation_f_constraint_count: int = None
    formation_obb_constraint_count: int = None
    formation_certificate: dict = field(default_factory=dict)

    def __post_init__(self):
        if self.status not in G4_RESULT_STATUSES:
            raise VirtualAContractError(
                "INVALID_RESULT_STATUS", "status={!r}".format(self.status))
        if self.formation_corridor_scope not in FORMATION_CORRIDOR_SCOPES:
            raise VirtualAContractError(
                "INVALID_RESULT",
                "formation_corridor_scope={!r} allowed={}".format(
                    self.formation_corridor_scope,
                    FORMATION_CORRIDOR_SCOPES))
        object.__setattr__(self, "attempts", tuple(self.attempts))
        object.__setattr__(self, "routes", tuple(self.routes))
        object.__setattr__(self, "grid_summary",
                           dict(sorted(self.grid_summary.items())))
        object.__setattr__(self, "search", dict(sorted(self.search.items())))
        object.__setattr__(self, "timings", dict(sorted(self.timings.items())))
        object.__setattr__(self, "metrics", dict(sorted(self.metrics.items())))
        success = self.status in G4_EXECUTABLE_STATUSES
        if success:
            if self.plan is None or self.path is None:
                raise VirtualAContractError(
                    "INVALID_RESULT", "executable status requires a plan/path")
            for report in (self.geometry_audit, self.dynamic_audit,
                           self.collision_audit):
                if report is None or not bool(getattr(report, "passed", False)):
                    raise VirtualAContractError(
                        "INVALID_RESULT",
                        "executable status requires three passing audits")
            if self.geometry_audit.scope != GEOMETRY_SCOPE or \
                    self.dynamic_audit.scope != DYNAMICS_SCOPE or \
                    self.collision_audit.scope != COLLISION_SCOPE:
                raise VirtualAContractError(
                    "INVALID_RESULT", "audit scopes do not match layers")
            if not self.planning_context_sha256:
                raise VirtualAContractError(
                    "INVALID_RESULT", "executable status requires a context")
            plan_context_sha = getattr(
                self.plan, "planning_context_sha256", None)
            if callable(plan_context_sha):
                plan_context_sha = plan_context_sha()
            if plan_context_sha != self.planning_context_sha256:
                raise VirtualAContractError(
                    "INVALID_RESULT", "plan and result context digests differ")
            for report in (self.geometry_audit, self.dynamic_audit,
                           self.collision_audit):
                value = getattr(report, "planning_context_sha256", "")
                if value and value != self.planning_context_sha256:
                    raise VirtualAContractError(
                        "INVALID_RESULT",
                        "audit and result context digests differ")
            if self.request_sha256 != getattr(
                    self.plan, "request_sha256", None):
                raise VirtualAContractError(
                    "INVALID_RESULT", "plan and result request digests differ")
            if self.model_sha256 != getattr(
                    self.plan, "model_sha256", lambda: "")():
                raise VirtualAContractError(
                    "INVALID_RESULT", "result model digest differs from plan")
            if self.status == "NO_MOTION" and getattr(
                    self.plan, "kind", None) != "hold":
                raise VirtualAContractError(
                    "INVALID_RESULT", "NO_MOTION requires a hold plan")
            if self.formation_corridor_scope != \
                    FORMATION_CORRIDOR_SCOPE_F_ONLY:
                self._check_union_certificate()
        else:
            if self.plan is not None or self.path is not None:
                raise VirtualAContractError(
                    "INVALID_RESULT",
                    "a failed result must not publish a plan/path")
            if self.projection is not None:
                raise VirtualAContractError(
                    "INVALID_RESULT",
                    "a failed result must not publish a projection")

    def _check_union_certificate(self):
        """Fail closed unless a VERIFIED identity-matched certificate seals it.

        This mirrors the identity gate of
        ``formation_certificate.verify_formation_certificate`` for the fields
        that the contract itself owns (path, request, grid, context, model and
        template), and recomputes the sealed body hash.  A non-``VERIFIED``,
        stale or edited certificate can therefore never make a union result
        executable.
        """
        certificate = self.formation_certificate
        if not isinstance(certificate, dict) or not certificate:
            raise VirtualAContractError(
                "INVALID_RESULT",
                "union scope requires a sealed continuous certificate")
        if certificate.get(
                "schema") != "virtual_a_formation_continuous_certificate_v1":
            raise VirtualAContractError(
                "INVALID_RESULT",
                "union certificate schema={!r}".format(
                    certificate.get("schema")))
        if certificate.get("status") != "VERIFIED":
            raise VirtualAContractError(
                "INVALID_RESULT",
                "union certificate status={!r} is not VERIFIED".format(
                    certificate.get("status")))
        if certificate.get("is_continuous_certificate") is not True:
            raise VirtualAContractError(
                "INVALID_RESULT",
                "union certificate is not a continuous certificate")
        maximum = certificate.get("max_outward_residual_m")
        try:
            maximum = float(maximum)
        except (TypeError, ValueError) as exc:
            raise VirtualAContractError(
                "INVALID_RESULT",
                "union certificate residual is unavailable") from exc
        if not math.isfinite(maximum) or maximum > 0.0:
            raise VirtualAContractError(
                "INVALID_RESULT",
                "union certificate max_outward_residual_m={!r}".format(
                    maximum))
        if int(certificate.get("unclosed_interval_count", -1)) != 0:
            raise VirtualAContractError(
                "INVALID_RESULT",
                "union certificate has unclosed intervals")
        if certificate.get("witness") is not None:
            raise VirtualAContractError(
                "INVALID_RESULT", "union certificate carries a witness")
        template_id = str(certificate.get("template_id", ""))
        if not template_id or template_id != self.formation_template_id:
            raise VirtualAContractError(
                "INVALID_RESULT",
                "union certificate/template identity differs from the result")
        stored_hash = certificate.get("certificate_sha256")
        body = {key: value for key, value in certificate.items()
                if key != "certificate_sha256" and key != "elapsed_s"}
        if stored_hash != _g4_sha256_json(body):
            raise VirtualAContractError(
                "INVALID_RESULT",
                "union certificate sealed hash does not match its body")
        if certificate.get("request_sha256") != self.request_sha256:
            raise VirtualAContractError(
                "INVALID_RESULT",
                "union certificate request identity differs")
        if certificate.get("grid_sha256") != self.grid_sha256:
            raise VirtualAContractError(
                "INVALID_RESULT", "union certificate grid identity differs")
        if certificate.get(
                "planning_context_sha256") != self.planning_context_sha256:
            raise VirtualAContractError(
                "INVALID_RESULT",
                "union certificate planning-context identity differs")
        if certificate.get("identity", {}).get(
                "model_sha256") != certificate.get("model_sha256"):
            raise VirtualAContractError(
                "INVALID_RESULT",
                "union certificate model identity record differs")
        path = self.path
        path_payload = {
            "kind": str(getattr(path, "kind", "")),
            "numerics_version": str(getattr(path, "numerics_version", "")),
            # The certificate hashes the path against the request model
            # digest, not the timed-plan digest.  Use the sealed identity
            # value and never trust a caller-supplied model sha.
            "model_sha256": str(certificate.get("model_sha256", "")),
            "length_m": float(path.length_m),
            "start_qf": {
                "x": float(path.start_qf.x),
                "y": float(path.start_qf.y),
                "psi": float(path.start_qf.psi),
            },
            "profile": path.profile.as_dict(),
        }
        if certificate.get("path_sha256") != _g4_sha256_json(path_payload):
            raise VirtualAContractError(
                "INVALID_RESULT",
                "union certificate path identity differs from the result")
        audit = self.formation_corridor_audit
        if not isinstance(audit, dict) or \
                audit.get("template_id") != template_id:
            raise VirtualAContractError(
                "INVALID_RESULT",
                "union formation_corridor_audit is missing or stale")

    @property
    def feasible(self):
        return self.status in G4_EXECUTABLE_STATUSES

    @property
    def executable(self):
        if not self.feasible:
            return False
        return all(bool(getattr(report, "passed", False)) for report in (
            self.geometry_audit, self.dynamic_audit, self.collision_audit))

    @property
    def certification(self):
        return self.status if self.executable else "NONE"

    @property
    def collision_certificate(self):
        return bool(self.executable)

    @property
    def independent_audit(self):
        return True

    @property
    def trajectory_published(self):
        return False

    @property
    def request_fingerprint(self):
        return self.request_sha256

    def as_status(self):
        failure = {
            "status": self.status,
            "reason": self.reason,
            "detail": self.detail,
            "first_failure_layer": self.first_failure_layer,
            "geometry": self.geometry_audit.as_dict()
            if hasattr(self.geometry_audit, "as_dict") else None,
            "dynamics": self.dynamic_audit.as_dict()
            if hasattr(self.dynamic_audit, "as_dict") else None,
            "collision": self.collision_audit.as_dict()
            if hasattr(self.collision_audit, "as_dict") else None,
        }
        return {
            "schema": "virtual_a_result/v1",
            "status": self.status,
            "feasible": bool(self.feasible),
            "reason": self.reason,
            "detail": self.detail,
            "first_failure_layer": self.first_failure_layer,
            "certification": self.certification,
            "executable": bool(self.executable),
            "trajectory_published": False,
            "independent_audit": True,
            "collision_certificate": self.collision_certificate,
            "request_sha256": self.request_sha256,
            "planning_context_sha256": self.planning_context_sha256,
            "grid_sha256": self.grid_sha256,
            "model_sha256": self.model_sha256,
            "failure": None if self.executable else failure,
            "metrics": dict(self.metrics),
            "checks": int(self.checks),
            "timings": dict(self.timings),
            "grid": dict(self.grid_summary),
        }

    def as_dict(self):
        model_summary = None
        if self.plan is not None and hasattr(self.plan, "model_summary"):
            model_summary = self.plan.model_summary()
        payload = {
            "status": self.status,
            "reason": self.reason,
            "detail": self.detail,
            "first_failure_layer": self.first_failure_layer,
            "executable": bool(self.executable),
            "request_sha256": self.request_sha256,
            "planning_context_sha256": self.planning_context_sha256,
            "grid_sha256": self.grid_sha256,
            "grid_summary": self.grid_summary,
            "model_sha256": self.model_sha256,
            "model_summary": model_summary,
            "checks": int(self.checks),
            "metrics": self.metrics,
            "timings": self.timings,
            "search": self.search,
            "routes": [route.as_dict() if hasattr(route, "as_dict") else route
                       for route in self.routes],
            "attempts": [attempt.as_dict() if hasattr(attempt, "as_dict")
                         else attempt for attempt in self.attempts],
            "geometry_audit": self.geometry_audit.as_dict()
            if hasattr(self.geometry_audit, "as_dict") else None,
            "dynamic_audit": self.dynamic_audit.as_dict()
            if hasattr(self.dynamic_audit, "as_dict") else None,
            "collision_audit": self.collision_audit.as_dict()
            if hasattr(self.collision_audit, "as_dict") else None,
            "projection": self.projection,
        }
        # f_only results keep their historical key set byte for byte.
        if self.formation_corridor_scope != FORMATION_CORRIDOR_SCOPE_F_ONLY:
            payload["formation_corridor_scope"] = self.formation_corridor_scope
            payload["formation_corridor_audit"] = dict(
                self.formation_corridor_audit)
            payload["formation_template_id"] = self.formation_template_id
            payload["formation_f_constraint_count"] = (
                self.formation_f_constraint_count)
            payload["formation_obb_constraint_count"] = (
                self.formation_obb_constraint_count)
            payload["formation_certificate"] = dict(self.formation_certificate)
        return payload


# ---------------------------------------------------------------------------
# G4 payload/config reconstruction (used by the scene manifest and benchmark)
# ---------------------------------------------------------------------------

def _check_payload_keys(payload, cls, name, echoes=()):
    if not isinstance(payload, MappingABC):
        raise VirtualAContractError(
            "INVALID_" + name.upper(), "{} must be a mapping".format(name))
    known = set(cls.__dataclass_fields__)
    required = known - set(echoes)
    missing = sorted(required - set(payload))
    extra = sorted(set(payload) - known - set(echoes))
    if missing or extra:
        raise VirtualAContractError(
            "INVALID_" + name.upper(),
            "{} fields missing={} extra={}".format(name, missing, extra))
    return dict(payload)


def _exact_fields(payload, cls, name):
    return _check_payload_keys(payload, cls, name)


def planner_config_from_dict(payload):
    values = _check_payload_keys(
        payload, PlannerConfig, "planner_config",
        echoes=tuple(_PLANNER_CORRIDOR_DEFAULTS)
        + ("formation_corridor_scope",))
    # Historical payloads omit every C2-B field; partial C2-B payloads fall
    # back to the dataclass default for any omitted field.  Unknown keys still
    # fail in _check_payload_keys.
    values = {
        key: value for key, value in values.items()
        if key in PlannerConfig.__dataclass_fields__}
    return PlannerConfig(**values)


def geometry_audit_config_from_dict(payload):
    return GeometryAuditConfig(**_exact_fields(
        payload, GeometryAuditConfig, "geometry_audit_config"))


def timing_config_from_dict(payload):
    values = _check_payload_keys(
        payload, TimingConfig, "timing_config", echoes=("timing_law",))
    if "timing_law" in values:
        if values.pop("timing_law") != TIMING_LAW_VERSION:
            raise VirtualAContractError(
                "INVALID_TIMING_CONFIG", "unknown timing_law echo")
    values["geometry_audit_config"] = geometry_audit_config_from_dict(
        values["geometry_audit_config"])
    return TimingConfig(**values)


def collision_config_from_dict(payload):
    values = _check_payload_keys(
        payload, CollisionConfig, "collision_config",
        echoes=("baseline_margin_floor_m", "max_corner_step_ceiling_m",
                "sample_budget_ceiling"))
    values = {key: value for key, value in values.items()
              if key in CollisionConfig.__dataclass_fields__}
    return CollisionConfig(**values)


def route_guide_config_from_dict(payload):
    values = _check_payload_keys(
        payload, RouteGuideConfig, "route_guide_config",
        echoes=("algorithm", "guide_is_executable_path"))
    values = {key: value for key, value in values.items()
              if key in RouteGuideConfig.__dataclass_fields__}
    return RouteGuideConfig(**values)


def formation_timing_limits_from_dict(payload):
    return FormationTimingLimits(**_exact_fields(
        payload, FormationTimingLimits, "limits"))


def request_from_payload(payload):
    """Reconstruct the exact physical request hashed by ``request_payload``."""
    if not isinstance(payload, MappingABC):
        raise VirtualAContractError(
            "INVALID_REQUEST", "request payload must be a mapping")
    expected = {
        "schema", "plan_id", "formation_id", "model", "vehicle_shapes",
        "start_qf", "start_headings", "goal_qf", "initial", "goal_policy",
        "limits",
    }
    if set(payload) != expected:
        raise VirtualAContractError(
            "INVALID_REQUEST",
            "payload keys missing={} extra={}".format(
                sorted(expected - set(payload)),
                sorted(set(payload) - expected)))
    if payload["schema"] != REQUEST_SCHEMA:
        raise VirtualAContractError(
            "INVALID_REQUEST", "unknown request schema")
    model = FormationModel([
        VehicleOffset(int(row[0]), float(row[1]), float(row[2]))
        for row in payload["model"]])
    shapes = {
        int(row[0]): VehicleShape(float(row[1]), float(row[2]))
        for row in payload["vehicle_shapes"]}
    start_headings = {
        int(row[0]): float(row[1]) for row in payload["start_headings"]}
    start_qf = FormationPose(*[float(value) for value in payload["start_qf"]])
    goal_qf = FormationPose(*[float(value) for value in payload["goal_qf"]])
    initial_values = dict(payload["initial"])
    initial_values.pop("node_context_verified", None)
    initial = InitialAssumption(**initial_values)
    limits = formation_timing_limits_from_dict(payload["limits"])
    return VirtualARequest(
        plan_id=payload["plan_id"],
        formation_id=payload["formation_id"],
        model=model,
        vehicle_shapes=shapes,
        start_qf=start_qf,
        start_headings=start_headings,
        goal_qf=goal_qf,
        initial=initial,
        goal_policy=coerce_goal_policy(payload["goal_policy"]),
        limits=limits)


__all__ = [
    "G4_SCHEMA", "G4_PLANNING_SCHEMA", "COLLISION_SCOPE",
    "PROGRESS_SCHEMA", "PROGRESS_STAGES", "PROGRESS_STAGE_INDEX",
    "PROGRESS_STATES", "PROGRESS_FAILED_STAGE",
    "PlannerConfig", "research_planner_config", "HeadingBranch",
    "G4_RESULT_STATUSES", "G4_EXECUTABLE_STATUSES",
    "FORMATION_CORRIDOR_SCOPE_F_ONLY",
    "FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1",
    "FORMATION_CORRIDOR_SCOPES", "FORMATION_CORRIDOR_TEMPLATE_SCHEMA",
    "FORMATION_CORRIDOR_COMPILER_VERSION",
    "FORMATION_CORRIDOR_CERTIFICATE_VERSION",
    "FORMATION_CORRIDOR_DECOMPOSITIONS", "FORMATION_CORRIDOR_STATUSES",
    "FORMATION_CORRIDOR_STATUS_VERIFIED", "FORMATION_CORRIDOR_STATUS_VIOLATED",
    "FORMATION_CORRIDOR_STATUS_UNCONFIRMED",
    "FORMATION_CORRIDOR_STATUS_TEMPLATE_INSUFFICIENT",
    "FORMATION_CORRIDOR_STATUS_F_ONLY_GATE_BLOCKED",
    "FormationCorridorVehicleFrame", "FormationCorridorPiece",
    "FormationCorridorVehicleCover", "FormationCorridorTemplateSegment",
    "FormationCorridorTemplate",
    "PLAYBACK_SEMANTIC_VERSION", "PLAYBACK_CERTIFICATION",
    "CollisionConfig", "RouteGuideConfig", "GuideRoute", "ObstacleContext",
    "ConvexCorridor", "CorridorChain", "CachedClearanceField",
    "VirtualAMapRequest", "VirtualAResult",
    "grid_occupancy_sha256", "grid_summary", "grid_signature",
    "grid_summary_sha256", "planning_context", "planning_context_sha256",
    "validate_collision_config", "validate_route_guide_config",
    "request_from_payload", "planner_config_from_dict",
    "timing_config_from_dict", "collision_config_from_dict",
    "route_guide_config_from_dict", "geometry_audit_config_from_dict",
    "formation_timing_limits_from_dict",
]
