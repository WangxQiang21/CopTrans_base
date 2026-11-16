"""Authoritative G3 timed model and pointwise common-time evaluation.

``VirtualATimedPlan`` evaluates the production C2 Hermite geometry along the
single rest-to-rest quintic progress law.  The authoritative model is a finite
canonical JSON core: original request inputs, raw profile knots, unified
duration and the numerics declaration.  A sampled projection is never an
authoritative path and is never used to reconstruct the model.

A JSON payload is restored only structurally/mathematically.  It is not
"audited" merely because it was loadable; :func:`audit.audit_dynamics` must be
run separately.
"""
from dataclasses import dataclass, field
import copy
import hashlib
import json
import math

from ..continuous_geometry import ControlKnot
from ..formation import FormationModel, FormationPose, VehicleOffset
from ..formation_collision import VehicleShape
from ..formation_retiming import FormationTimingLimits
from . import bounds
from .contracts import (
    BACKEND,
    PATH_MODEL_VERSION,
    REQUEST_SCHEMA,
    TIMED_MODEL_VERSION,
    TIMING_LAW_VERSION,
    DYNAMICS_SCOPE,
    DynamicAuditConfig,
    GoalPolicy,
    InitialAssumption,
    TimingConfig,
    VirtualAContractError,
    VirtualARequest,
    _finite,
    planning_context_sha256 as _context_sha256,
    request_fingerprint,
    request_payload,
    validate_virtual_a_request,
)
from .geometry import C2CurvatureProfile, VirtualAPath
from .timing import _SCALED_METRICS, quintic_derivatives, time_at_s_seconds

_MODEL_TOP_KEYS = frozenset((
    "backend", "model_version", "path_model_version", "request_inputs",
    "request_sha256", "path", "timing", "numerics", "planning_context"))
_PATH_KEYS = frozenset((
    "kind", "model_version", "start_qf", "length_m",
    "position_error_bound_m", "angle_error_bound_rad", "integration_budget",
    "profile"))
_PROFILE_KEYS = frozenset((
    "kind", "numerics_version", "length_m", "knots"))
_KNOT_KEYS = frozenset(("s", "value", "derivative"))
_TIMING_KEYS = frozenset(("law", "kind", "S", "T_s", "selection"))
_SELECTION_KEYS = frozenset((
    "required_duration_s", "selected_duration_s", "safety_factor", "worst",
    "metrics_at_T0_s", "wheel_angle_bound_rad", "wheel_angle_limit_rad",
    "interval_subdivisions_per_segment", "T0_s", "rescalings"))
_WORST_KEYS = frozenset((
    "quantity", "vehicle_id", "wheel_index", "order", "segment_index",
    "s0", "s1"))
_METRIC_KEYS = frozenset((
    "upper_at_T0_s", "limit", "order", "required_duration_s",
    "upper_bound_interval", "vehicle_id", "wheel_index", "segment_index"))
_NUMERICS_KEYS = frozenset((
    "version", "position_error_bound_m", "angle_error_bound_rad",
    "integration_method", "floating_point_assumption"))
_REQUEST_KEYS = frozenset((
    "schema", "plan_id", "formation_id", "model", "vehicle_shapes",
    "start_qf", "start_headings", "goal_qf", "initial", "goal_policy",
    "limits"))
_INITIAL_KEYS = frozenset((
    "stationary", "mode", "wheels_straight", "source", "context_id"))
_WHEEL_COORDINATES = (
    (0.5, 0.5), (0.5, -0.5), (-0.5, 0.5), (-0.5, -0.5))
# A station reconstructed from a floating time can differ from the exact knot
# by a few ulps.  Explicit one-sided evaluation is only well defined exactly at
# a knot, so a station this close to one is resolved to that knot.
_NODE_SNAP_TOLERANCE_M = 1e-12
_FLOATING_POINT_ASSUMPTION = (
    "path positions use the G1 composite-Simpson budget; time-law and "
    "dynamic values are analytic in the C2 Hermite profile; libm exp/trig "
    "values are assumed correctly rounded within the documented guards")


def _canonical_json(value):
    try:
        return json.dumps(
            value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
            allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise VirtualAContractError(
            "NONFINITE_MODEL_VALUE",
            "model core is not finite canonical JSON: {}".format(exc)) from exc


def _normalize_json(value):
    return json.loads(_canonical_json(value))


def _reject_constant(value):
    raise ValueError("non-finite JSON constant {!r}".format(value))


def _require_exact_keys(record, expected, context):
    if not isinstance(record, dict):
        raise VirtualAContractError(
            "INVALID_MODEL_JSON", "{} must be an object".format(context))
    keys = set(record)
    if keys != set(expected):
        missing = sorted(set(expected) - keys)
        extra = sorted(keys - set(expected))
        raise VirtualAContractError(
            "UNSUPPORTED_MODEL_FIELD",
            "{} missing={} extra={}".format(context, missing, extra),
            context=context, missing=missing, extra=extra)
    return record


def _json_float(value, context):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise VirtualAContractError(
            "NONFINITE_MODEL_VALUE",
            "{} must be a JSON number, got {!r}".format(context, value))
    return _finite(value, context)


def _json_int(value, context, minimum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise VirtualAContractError(
            "INVALID_MODEL_JSON",
            "{} must be an integer, got {!r}".format(context, value))
    if minimum is not None and value < minimum:
        raise VirtualAContractError(
            "INVALID_MODEL_JSON",
            "{}={} must be >= {}".format(context, value, minimum))
    return int(value)


@dataclass(frozen=True)
class VehicleTimeState:
    """One vehicle's common-time dynamic state.

    Ideal wheel angle, wheel speed, steering rate and steering acceleration
    are diagnostic values; they are not CAN commands.
    """

    vehicle_id: int
    x: float
    y: float
    theta: float
    C_i: float
    v_i: float
    omega_i: float
    velocity: tuple
    acceleration: tuple
    jerk: tuple
    yaw_accel: float
    yaw_jerk: float
    lateral_accel: float
    wheel_angles: tuple
    wheel_speeds: tuple
    steer_rates: tuple
    steer_accels: tuple

    def __post_init__(self):
        if isinstance(self.vehicle_id, bool) or not isinstance(
                self.vehicle_id, int):
            raise VirtualAContractError(
                "INVALID_VEHICLE_ID", "vehicle_id={!r}".format(self.vehicle_id))
        for name in ("x", "y", "theta", "C_i", "v_i", "omega_i", "yaw_accel",
                     "yaw_jerk", "lateral_accel"):
            object.__setattr__(
                self, name, _finite(getattr(self, name),
                                    "VehicleTimeState." + name))
        for name in ("velocity", "acceleration", "jerk"):
            vector = tuple(getattr(self, name))
            if len(vector) != 2:
                raise VirtualAContractError(
                    "INVALID_VECTOR", "{} must have two components".format(name))
            object.__setattr__(
                self, name, tuple(
                    _finite(component, "{}.{}".format(name, index))
                    for index, component in enumerate(vector)))
        for name in ("wheel_angles", "wheel_speeds", "steer_rates",
                     "steer_accels"):
            vector = tuple(getattr(self, name))
            if len(vector) != 4:
                raise VirtualAContractError(
                    "INVALID_WHEEL_VECTOR",
                    "{} must have four components".format(name))
            object.__setattr__(
                self, name, tuple(
                    _finite(component, "{}.{}".format(name, index))
                    for index, component in enumerate(vector)))

    def as_dict(self):
        return {
            "vehicle_id": self.vehicle_id,
            "x": self.x, "y": self.y, "theta": self.theta,
            "C_i": self.C_i, "v_i": self.v_i, "omega_i": self.omega_i,
            "velocity": list(self.velocity),
            "acceleration": list(self.acceleration),
            "jerk": list(self.jerk),
            "yaw_accel": self.yaw_accel, "yaw_jerk": self.yaw_jerk,
            "lateral_accel": self.lateral_accel,
            "wheel_angles": list(self.wheel_angles),
            "wheel_speeds": list(self.wheel_speeds),
            "steer_rates": list(self.steer_rates),
            "steer_accels": list(self.steer_accels),
            "ideal_wheel_values_are_diagnostics_not_commands": True,
        }


@dataclass(frozen=True)
class VirtualATimeState:
    """Complete common-time state; mode is always the unified A mode."""

    t: float
    s: float
    w: float
    a: float
    j: float
    qf: FormationPose
    vehicles: tuple
    mode: str = "A"
    region: str = "motion"

    def __post_init__(self):
        for name in ("t", "s", "w", "a", "j"):
            object.__setattr__(
                self, name, _finite(getattr(self, name),
                                    "VirtualATimeState." + name))
        if not isinstance(self.qf, FormationPose):
            raise VirtualAContractError(
                "INVALID_FORMATION_POSE", "qf must be a FormationPose")
        if self.mode != "A":
            raise VirtualAContractError(
                "UNSUPPORTED_MODE", "mode={!r}; only A is supported".format(
                    self.mode))
        if self.region not in ("motion", "initial_hold", "terminal_hold"):
            raise VirtualAContractError(
                "INVALID_TIME_REGION", "region={!r}".format(self.region))
        vehicles = tuple(self.vehicles)
        if not vehicles:
            raise VirtualAContractError(
                "EMPTY_STATE", "at least one vehicle is required")
        for vehicle in vehicles:
            if not isinstance(vehicle, VehicleTimeState):
                raise VirtualAContractError(
                    "INVALID_VEHICLE_STATE",
                    "vehicles must contain VehicleTimeState records")
        ids = tuple(vehicle.vehicle_id for vehicle in vehicles)
        if ids != tuple(sorted(ids)) or len(set(ids)) != len(ids):
            raise VirtualAContractError(
                "INVALID_VEHICLE_ID_SET",
                "vehicles must be unique and sorted by vehicle_id")
        object.__setattr__(self, "vehicles", vehicles)

    def vehicle(self, vehicle_id):
        for vehicle in self.vehicles:
            if vehicle.vehicle_id == vehicle_id:
                return vehicle
        raise VirtualAContractError(
            "VEHICLE_NOT_IN_STATE", "vehicle_id={}".format(vehicle_id))

    def as_dict(self):
        return {
            "t": self.t, "s": self.s, "w": self.w, "a": self.a, "j": self.j,
            "qf": {"x": self.qf.x, "y": self.qf.y, "psi": self.qf.psi},
            "mode": self.mode,
            "region": self.region,
            "vehicles": [vehicle.as_dict() for vehicle in self.vehicles],
        }


@dataclass(frozen=True)
class VirtualAProjection:
    """Non-authoritative sample bundle: times, complete states, model summary."""

    times: tuple
    states: tuple
    model_summary: dict = field(default_factory=dict)

    def __post_init__(self):
        times = tuple(_finite(value, "projection.t") for value in self.times)
        states = tuple(self.states)
        if len(times) != len(states):
            raise VirtualAContractError(
                "INVALID_PROJECTION",
                "times={} states={}".format(len(times), len(states)))
        for t, state in zip(times, states):
            if not isinstance(state, VirtualATimeState):
                raise VirtualAContractError(
                    "INVALID_PROJECTION",
                    "states must contain VirtualATimeState records")
            if state.t != t:
                raise VirtualAContractError(
                    "INVALID_PROJECTION",
                    "state.t={!r} does not equal projection time {!r}".format(
                        state.t, t))
        object.__setattr__(self, "times", times)
        object.__setattr__(self, "states", states)
        object.__setattr__(self, "model_summary",
                           dict(sorted(dict(self.model_summary).items())))

    @property
    def authoritative(self):
        return False

    def as_dict(self):
        return {
            "authoritative": False,
            "times": list(self.times),
            "states": [state.as_dict() for state in self.states],
            "model_summary": dict(self.model_summary),
        }


class VirtualATimedPlan:
    """Common-time timed model with strict canonical JSON reconstruction."""

    def __init__(self, request, path, total_s, duration_s, kind, selection,
                 planning_context=None):
        self._request = request
        self._path = path
        self._S = float(total_s)
        self._T = float(duration_s)
        self._kind = str(kind)
        self._selection = copy.deepcopy(selection)
        self._planning_context = copy.deepcopy(planning_context)
        self._node_stations = self._node_stations_for(path)
        self._model_core = self._build_core()

    @staticmethod
    def _node_stations_for(path):
        if path.is_hold:
            return (0.0,)
        return tuple([0.0] + [piece[1]
                              for piece in path.profile.domain_segments])

    @classmethod
    def _build_motion(cls, request, path, duration_s, selection,
                      planning_context=None):
        if not isinstance(path, VirtualAPath) or path.is_hold:
            raise VirtualAContractError(
                "INVALID_PATH", "motion plan requires a motion VirtualAPath")
        duration = _finite(duration_s, "duration_s")
        if duration <= 0.0:
            raise VirtualAContractError(
                "INVALID_DURATION", "duration_s={!r}".format(duration))
        return cls(request, path, path.length_m, duration, "motion",
                   selection, planning_context)

    @classmethod
    def _build_hold(cls, request, path, planning_context=None):
        if not isinstance(path, VirtualAPath) or not path.is_hold:
            raise VirtualAContractError(
                "INVALID_PATH", "hold plan requires a hold VirtualAPath")
        selection = {
            "required_duration_s": 0.0,
            "selected_duration_s": 0.0,
            "safety_factor": 1.0,
            "worst": {
                "quantity": None, "vehicle_id": None, "wheel_index": None,
                "order": None, "segment_index": None, "s0": None, "s1": None},
            "metrics_at_T0_s": {},
            "wheel_angle_bound_rad": 0.0,
            "wheel_angle_limit_rad": float(request.limits.steer_angle_max),
            "interval_subdivisions_per_segment": 0,
            "T0_s": 0.0,
            "rescalings": 0,
        }
        return cls(request, path, 0.0, 0.0, "hold", selection,
                   planning_context)

    def _build_core(self):
        if not isinstance(self._request, VirtualARequest):
            raise VirtualAContractError(
                "INVALID_REQUEST", "plan requires a VirtualARequest")
        if not isinstance(self._path, VirtualAPath):
            raise VirtualAContractError(
                "INVALID_PATH", "plan requires a VirtualAPath")
        profile = self._path.profile
        if self._kind == "motion":
            if profile is None or self._path.is_hold:
                raise VirtualAContractError(
                    "INVALID_PATH", "motion plan needs a curvature profile")
            profile_block = {
                "kind": profile.kind,
                "numerics_version": profile.numerics_version,
                "length_m": float(profile.length_m),
                "knots": [
                    {"s": float(knot.s), "value": float(knot.value),
                     "derivative": float(knot.derivative)}
                    for knot in profile.knots],
            }
        elif self._kind == "hold":
            if profile is not None or not self._path.is_hold:
                raise VirtualAContractError(
                    "INVALID_PATH", "hold plan must have profile=None")
            if self._S != 0.0 or self._T != 0.0:
                raise VirtualAContractError(
                    "INVALID_HOLD",
                    "S={!r} T={!r} must both be zero".format(
                        self._S, self._T))
            profile_block = None
        else:
            raise VirtualAContractError(
                "INVALID_PLAN_KIND", "kind={!r}".format(self._kind))
        path_block = {
            "kind": self._kind,
            "model_version": PATH_MODEL_VERSION,
            "start_qf": [self._path.start_qf.x, self._path.start_qf.y,
                         self._path.start_qf.psi],
            "length_m": float(self._path.length_m),
            "position_error_bound_m": float(
                self._path.position_error_bound_m),
            "angle_error_bound_rad": float(
                self._path.angle_error_bound_rad),
            "integration_budget": _normalize_json(
                self._path.integration_budget),
            "profile": profile_block,
        }
        timing_block = {
            "law": TIMING_LAW_VERSION,
            "kind": self._kind,
            "S": self._S,
            "T_s": self._T,
            "selection": _normalize_json(self._selection),
        }
        numerics_block = {
            "version": bounds.NUMERICS_VERSION,
            "position_error_bound_m": float(
                self._path.position_error_bound_m),
            "angle_error_bound_rad": float(
                self._path.angle_error_bound_rad),
            "integration_method": "motion" if self._kind == "motion"
            else "hold",
            "floating_point_assumption": _FLOATING_POINT_ASSUMPTION,
        }
        core = {
            "backend": BACKEND,
            "model_version": TIMED_MODEL_VERSION,
            "path_model_version": PATH_MODEL_VERSION,
            "request_inputs": _normalize_json(request_payload(self._request)),
            "request_sha256": request_fingerprint(self._request),
            "path": path_block,
            "timing": timing_block,
            "numerics": numerics_block,
            "planning_context": _normalize_json(self._planning_context),
        }
        # Validate the selection's duration consistency before the core is
        # exposed: a single-field duration edit is not a valid model.
        self._check_selection_consistency(timing_block, self._S, self._request)
        return _normalize_json(core)

    @staticmethod
    def _check_selection_consistency(timing_block, total_s, request=None):
        selection = timing_block.get("selection")
        _require_exact_keys(selection, _SELECTION_KEYS, "timing.selection")
        required = _json_float(
            selection["required_duration_s"],
            "selection.required_duration_s")
        selected = _json_float(
            selection["selected_duration_s"],
            "selection.selected_duration_s")
        safety = _json_float(
            selection["safety_factor"], "selection.safety_factor")
        duration = _json_float(timing_block["T_s"], "timing.T_s")
        if safety < 1.0 or required < 0.0 or selected < 0.0:
            raise VirtualAContractError(
                "INVALID_MODEL_JSON", "negative duration/selection value")
        if timing_block["kind"] == "hold":
            if total_s != 0.0 or duration != 0.0 or required != 0.0 or \
                    selected != 0.0:
                raise VirtualAContractError(
                    "INVALID_MODEL_JSON",
                    "hold timing must have S=T=required=selected=0",
                    S=total_s, T=duration)
            return
        if total_s <= 0.0 or duration <= 0.0:
            raise VirtualAContractError(
                "INVALID_MODEL_JSON", "motion model needs S>0 and T>0")
        lower = required * safety
        tolerance = 1e-12 * max(1.0, duration)
        if selected + tolerance < lower:
            raise VirtualAContractError(
                "MODEL_TIMING_INCONSISTENT",
                "selected duration {!r} is below required*safety {!r}".format(
                    selected, lower))
        if abs(selected - duration) > tolerance:
            raise VirtualAContractError(
                "MODEL_TIMING_INCONSISTENT",
                "timing.T_s={!r} differs from selection.selected={!r}".format(
                    duration, selected))
        rescalings = _json_int(
            selection["rescalings"], "selection.rescalings", minimum=0)
        if rescalings == 0 and abs(selected - lower) > 1e-9 * max(1.0, lower):
            raise VirtualAContractError(
                "MODEL_TIMING_INCONSISTENT",
                "unrescaled selection must equal required*safety")
        if rescalings > 8:
            raise VirtualAContractError(
                "INVALID_MODEL_JSON", "rescalings={} exceeds 8".format(
                    rescalings))
        # The scalar duration is not self-certifying.  Recompute every
        # per-metric candidate from the stored outward bounds and bind the
        # stored limits to the original request, so a coordinated edit of T,
        # selected and required is still rejected unless the whole selection
        # table is internally consistent.
        metrics = selection["metrics_at_T0_s"]
        expected_names = {name for name, _limit, _order in _SCALED_METRICS}
        if not isinstance(metrics, dict) or set(metrics) != expected_names:
            raise VirtualAContractError(
                "INVALID_MODEL_JSON",
                "selection metric names differ from the G3 metric set")
        recomputed = {}
        for name, limit_name, expected_order in _SCALED_METRICS:
            record = _require_exact_keys(
                metrics[name], _METRIC_KEYS,
                "timing.selection.metrics_at_T0_s." + name)
            order = _json_int(record["order"], "metric.order", minimum=1)
            if order != expected_order:
                raise VirtualAContractError(
                    "MODEL_TIMING_INCONSISTENT",
                    "{} order={} expected {}".format(
                        name, order, expected_order))
            upper = _json_float(
                record["upper_at_T0_s"], "metric.upper_at_T0_s")
            limit = _json_float(record["limit"], "metric.limit")
            candidate = _json_float(
                record["required_duration_s"], "metric.required_duration_s")
            if upper < 0.0 or limit <= 0.0 or candidate < 0.0:
                raise VirtualAContractError(
                    "INVALID_MODEL_JSON",
                    "{} has a negative/nonpositive metric value".format(name))
            if request is not None and limit != float(
                    getattr(request.limits, limit_name)):
                raise VirtualAContractError(
                    "MODEL_TIMING_INCONSISTENT",
                    "{} stored limit differs from the request".format(name))
            expected_candidate = (upper / limit) ** (1.0 / order)
            candidate_tolerance = 1e-12 * max(1.0, expected_candidate)
            if abs(candidate - expected_candidate) > candidate_tolerance:
                raise VirtualAContractError(
                    "MODEL_TIMING_INCONSISTENT",
                    "{} required duration does not match its own bound".format(
                        name))
            interval = record["upper_bound_interval"]
            if interval is None:
                if not (upper == 0.0 and candidate == 0.0):
                    raise VirtualAContractError(
                        "INVALID_MODEL_JSON",
                        "{} has no interval for a positive bound".format(name))
            elif not isinstance(interval, list) or len(interval) != 2:
                raise VirtualAContractError(
                    "INVALID_MODEL_JSON",
                    "{} upper_bound_interval must have two values".format(name))
            elif interval[0] is None and interval[1] is None:
                if not (upper == 0.0 and candidate == 0.0):
                    raise VirtualAContractError(
                        "INVALID_MODEL_JSON",
                        "{} has no interval for a positive bound".format(name))
            else:
                s0 = _json_float(interval[0], "metric.interval[0]")
                s1 = _json_float(interval[1], "metric.interval[1]")
                if not (0.0 <= s0 <= s1 <= total_s):
                    raise VirtualAContractError(
                        "INVALID_MODEL_JSON",
                        "{} interval is outside the path".format(name))
            if record["vehicle_id"] is None:
                if not (upper == 0.0 and candidate == 0.0):
                    raise VirtualAContractError(
                        "INVALID_MODEL_JSON",
                        "{} has no witness for a positive bound".format(name))
            else:
                vehicle_id = _json_int(
                    record["vehicle_id"], "metric.vehicle_id")
                if request is not None:
                    known_ids = {
                        offset.vehicle_id for offset in request.model.offsets}
                    if vehicle_id not in known_ids:
                        raise VirtualAContractError(
                            "INVALID_MODEL_JSON",
                            "{} witness vehicle_id={} is unknown".format(
                                name, vehicle_id))
                wheel_index = _json_int(
                    record["wheel_index"], "metric.wheel_index", minimum=0)
                if wheel_index > 3:
                    raise VirtualAContractError(
                        "INVALID_MODEL_JSON", "{} wheel_index={}".format(
                            name, wheel_index))
                _json_int(record["segment_index"], "metric.segment_index",
                          minimum=0)
            recomputed[name] = candidate
        maximum_required = max(recomputed.values())
        required_tolerance = 1e-12 * max(1.0, maximum_required)
        if abs(required - maximum_required) > required_tolerance:
            raise VirtualAContractError(
                "MODEL_TIMING_INCONSISTENT",
                "required duration does not equal the maximum metric "
                "candidate")
        worst = _require_exact_keys(
            selection["worst"], _WORST_KEYS, "timing.selection.worst")
        quantity = worst["quantity"]
        if not isinstance(quantity, str) or quantity not in metrics:
            raise VirtualAContractError(
                "MODEL_TIMING_INCONSISTENT",
                "worst quantity is not a selection metric")
        worst_metric = metrics[quantity]
        if abs(_json_float(
                worst_metric["required_duration_s"],
                "worst metric required_duration_s") - required) > \
                required_tolerance:
            raise VirtualAContractError(
                "MODEL_TIMING_INCONSISTENT",
                "worst metric does not attain the required duration")
        for key in ("vehicle_id", "wheel_index", "segment_index", "order"):
            if worst[key] != worst_metric[key]:
                raise VirtualAContractError(
                    "MODEL_TIMING_INCONSISTENT",
                    "worst {} differs from its metric record".format(key))
        interval = worst_metric["upper_bound_interval"]
        if not isinstance(interval, list) or len(interval) != 2 or \
                worst["s0"] != interval[0] or worst["s1"] != interval[1]:
            raise VirtualAContractError(
                "MODEL_TIMING_INCONSISTENT",
                "worst interval differs from its metric record")
        angle_bound = _json_float(
            selection["wheel_angle_bound_rad"],
            "selection.wheel_angle_bound_rad")
        angle_limit = _json_float(
            selection["wheel_angle_limit_rad"],
            "selection.wheel_angle_limit_rad")
        if angle_bound < 0.0 or angle_limit <= 0.0 or \
                angle_bound > angle_limit:
            raise VirtualAContractError(
                "INVALID_MODEL_JSON",
                "wheel-angle selection is outside its limit")
        if request is not None and angle_limit != float(
                request.limits.steer_angle_max):
            raise VirtualAContractError(
                "MODEL_TIMING_INCONSISTENT",
                "wheel-angle limit differs from the request")
        if _json_float(selection["T0_s"], "selection.T0_s") != 1.0:
            raise VirtualAContractError(
                "MODEL_TIMING_INCONSISTENT",
                "selection T0 must be exactly 1 s")
        _json_int(selection["interval_subdivisions_per_segment"],
                  "selection.interval_subdivisions_per_segment", minimum=1)

    @property
    def duration_s(self):
        return self._T

    @property
    def total_S_m(self):
        return self._S

    @property
    def kind(self):
        return self._kind

    @property
    def path(self):
        return self._path

    @property
    def request_sha256(self):
        return self._model_core["request_sha256"]

    @property
    def planning_context(self):
        return copy.deepcopy(self._planning_context)

    def planning_context_sha256(self):
        return _context_sha256(self._planning_context)

    def model_summary(self):
        profile = self._path.profile
        context_sha = self.planning_context_sha256()
        context = self._planning_context if isinstance(
            self._planning_context, dict) else {}
        return {
            "backend": BACKEND,
            "model_version": TIMED_MODEL_VERSION,
            "path_model_version": PATH_MODEL_VERSION,
            "request_sha256": self.request_sha256,
            "planning_context_sha256": context_sha or None,
            "grid": copy.deepcopy(context.get("grid")),
            "kind": self._kind,
            "S_m": self._S,
            "T_s": self._T,
            "segment_count": getattr(profile, "segment_count", 0)
            if profile is not None else 0,
            "knot_count": len(profile.knots) if profile is not None else 0,
            "model_sha256": self.model_sha256(),
            "authoritative_model": True,
        }

    def to_model_dict(self):
        return copy.deepcopy(self._model_core)

    def to_model_json(self):
        return _canonical_json(self._model_core)

    def model_sha256(self):
        return hashlib.sha256(
            _canonical_json(self._model_core).encode("utf-8")).hexdigest()

    def _require_time(self, t):
        return _finite(t, "time")

    def time_at_s(self, s):
        return time_at_s_seconds(s, self._S, self._T)

    def _wheel_values(self, curvature, curvature_s, curvature_ss, v, w, a):
        angles, speeds, rates, accels = [], [], [], []
        for xw_factor, yw_factor in _WHEEL_COORDINATES:
            xw = xw_factor * self._request.limits.wheelbase
            yw = yw_factor * self._request.limits.track
            m = 1.0 - yw * curvature
            n = xw * curvature
            Ew = m * m + n * n
            if not (Ew > 0.0 and math.isfinite(Ew)):
                raise VirtualAContractError(
                    "WHEEL_GEOMETRY_VIOLATION",
                    "Ew={!r} at curvature={!r}".format(Ew, curvature))
            factor = math.sqrt(Ew)
            Ew_C = -2.0 * yw + 2.0 * (xw * xw + yw * yw) * curvature
            delta_s = xw / Ew * curvature_s
            delta_ss = (xw / Ew * curvature_ss
                        - xw * Ew_C / (Ew * Ew) * curvature_s ** 2)
            angles.append(math.atan2(n, m))
            speeds.append(factor * v)
            rates.append(delta_s * w)
            accels.append(delta_ss * w * w + delta_s * a)
        return tuple(angles), tuple(speeds), tuple(rates), tuple(accels)

    def _vehicles_from_spatial(self, spatial, w, a, j):
        vehicles = []
        for offset, vehicle in zip(
                self._request.model.offsets, spatial.vehicles):
            kappa = spatial.curvature.kappa
            A = 1.0 - offset.y * kappa
            B = offset.x * kappa
            D = A * A + B * B
            if not (A > 0.0 and D > 0.0):
                raise VirtualAContractError(
                    "NORMAL_BRANCH_VIOLATION",
                    "vehicle_id={} A={!r} D={!r}".format(
                        offset.vehicle_id, A, D))
            Q = math.sqrt(D)
            C = vehicle.curvature
            C_s = vehicle.curvature_s
            C_ss = vehicle.curvature_ss
            H = vehicle.theta_s
            H_s = vehicle.theta_ss
            H_ss = vehicle.theta_sss
            v = Q * w
            omega = H * w
            w2 = w * w
            w3 = w2 * w
            p_s = vehicle.p_s
            p_ss = vehicle.p_ss
            p_sss = vehicle.p_sss
            velocity = (p_s[0] * w, p_s[1] * w)
            acceleration = (
                p_ss[0] * w2 + p_s[0] * a,
                p_ss[1] * w2 + p_s[1] * a)
            jerk = (
                p_sss[0] * w3 + 3.0 * p_ss[0] * w * a + p_s[0] * j,
                p_sss[1] * w3 + 3.0 * p_ss[1] * w * a + p_s[1] * j)
            yaw_accel = H * a + H_s * w2
            yaw_jerk = H * j + 3.0 * H_s * w * a + H_ss * w3
            lateral = abs(C) * v * v
            angles, speeds, rates, accels = self._wheel_values(
                C, C_s, C_ss, v, w, a)
            vehicles.append(VehicleTimeState(
                vehicle_id=offset.vehicle_id,
                x=vehicle.x, y=vehicle.y, theta=vehicle.theta,
                C_i=C, v_i=v, omega_i=omega,
                velocity=velocity, acceleration=acceleration, jerk=jerk,
                yaw_accel=yaw_accel, yaw_jerk=yaw_jerk,
                lateral_accel=lateral,
                wheel_angles=angles, wheel_speeds=speeds,
                steer_rates=rates, steer_accels=accels))
        return tuple(vehicles)

    def _state_from_spatial(self, t, s, w, a, j, spatial, region):
        return VirtualATimeState(
            t=t, s=s, w=w, a=a, j=j, qf=spatial.qf,
            vehicles=self._vehicles_from_spatial(spatial, w, a, j),
            mode="A", region=region)

    def _snap_station(self, s, side):
        """Resolve a floating station that is numerically at a profile knot.

        The returned ``side`` is the one-sided limit to use at that knot.
        Explicit ``left``/``right`` requests at the domain ends are left to
        the path, which reports the missing one-sided limit rather than
        silently choosing the other side.
        """
        if self._kind == "hold":
            return s, side
        nearest = None
        for station in self._node_stations:
            if abs(s - station) <= _NODE_SNAP_TOLERANCE_M and (
                    nearest is None or
                    abs(s - station) < abs(s - nearest)):
                nearest = station
        if nearest is None:
            return s, side
        if side == "auto":
            return nearest, ("left" if nearest == self._S else "right")
        if side == "left" and nearest == 0.0:
            return s, side
        if side == "right" and nearest == self._S:
            return s, side
        return nearest, side

    def _hold_state(self, t, side="auto", initial=None):
        if self._kind == "hold":
            station = 0.0
            spatial = self._path.state_at_s(0.0)
            t_value = 0.0
            initial = True
        elif initial is True:
            station = 0.0
            spatial = self._path.state_at_s(0.0)
            t_value = t
        elif initial is False:
            station = self._S
            spatial = self._path.state_at_s(self._S, "left")
            t_value = t
        elif t < 0.0:
            station = 0.0
            spatial = self._path.state_at_s(0.0)
            t_value = t
            initial = True
        else:
            station = self._S
            spatial = self._path.state_at_s(self._S, "left")
            t_value = t
            initial = False
        return self._state_from_spatial(
            t_value, station, 0.0, 0.0, 0.0, spatial,
            "initial_hold" if initial else "terminal_hold")

    def state_at(self, t, side="auto"):
        """Pointwise state; never linear interpolation between vehicle samples."""
        t = self._require_time(t)
        if side not in ("auto", "left", "right"):
            raise VirtualAContractError(
                "INVALID_SIDE", "side={!r}".format(side))
        if self._kind == "hold":
            return self._hold_state(t)
        if t < 0.0:
            return self._hold_state(t, initial=True)
        if t > self._T:
            return self._hold_state(t, initial=False)
        if t == 0.0:
            if side == "left":
                return self._hold_state(t, initial=True)
            tau = 0.0
            h, h1, h2, h3 = quintic_derivatives(tau)
            s = 0.0
            w = 0.0
            a = 0.0
            j = self._S * h3 / (self._T ** 3)
            spatial = self._path.state_at_s(0.0, "right")
            return self._state_from_spatial(t, s, w, a, j, spatial, "motion")
        if t == self._T:
            if side == "right":
                return self._hold_state(t, initial=False)
            tau = 1.0
            h, h1, h2, h3 = quintic_derivatives(tau)
            s = self._S
            w = 0.0
            a = 0.0
            j = self._S * h3 / (self._T ** 3)
            spatial = self._path.state_at_s(self._S, "left")
            return self._state_from_spatial(t, s, w, a, j, spatial, "motion")
        tau = t / self._T
        if tau <= 0.0:
            return self._hold_state(t, initial=True)
        if tau >= 1.0:
            return self._hold_state(t, initial=False)
        h, h1, h2, h3 = quintic_derivatives(tau)
        s = self._S * h
        if s <= 0.0:
            s = 0.0
            spatial = self._path.state_at_s(0.0, "right")
        elif s >= self._S:
            s = self._S
            spatial = self._path.state_at_s(self._S, "left")
        else:
            s, resolved_side = self._snap_station(s, side)
            spatial = self._path.state_at_s(s, resolved_side)
        w = self._S * h1 / self._T
        a = self._S * h2 / (self._T * self._T)
        j = self._S * h3 / (self._T ** 3)
        return self._state_from_spatial(t, s, w, a, j, spatial, "motion")

    def sample(self, times):
        try:
            supplied = tuple(times)
        except TypeError as exc:
            raise VirtualAContractError(
                "INVALID_SAMPLE_TIMES", "times must be iterable") from exc
        if self._kind == "hold":
            if len(supplied) != 1 or _finite(
                    supplied[0], "sample time") != 0.0:
                raise VirtualAContractError(
                    "INVALID_SAMPLE_TIMES",
                    "NO_MOTION model has exactly one t=0 sample")
            return VirtualAProjection(
                times=(0.0,), states=(self.state_at(0.0),),
                model_summary=self.model_summary())
        checked = tuple(_finite(value, "sample time") for value in supplied)
        states = tuple(self.state_at(value) for value in checked)
        return VirtualAProjection(
            times=checked, states=states, model_summary=self.model_summary())

    @classmethod
    def from_model_json(cls, payload, *, expected_request=None):
        if not isinstance(payload, str):
            raise VirtualAContractError(
                "INVALID_MODEL_JSON", "payload must be a JSON string")
        try:
            document = json.loads(payload, parse_constant=_reject_constant)
        except (TypeError, ValueError) as exc:
            raise VirtualAContractError(
                "INVALID_MODEL_JSON",
                "payload is not valid finite JSON: {}".format(exc)) from exc
        _require_exact_keys(document, _MODEL_TOP_KEYS, "model")
        if document["backend"] != BACKEND:
            raise VirtualAContractError(
                "UNKNOWN_BACKEND", "backend={!r}".format(document["backend"]))
        if document["model_version"] != TIMED_MODEL_VERSION:
            raise VirtualAContractError(
                "UNKNOWN_MODEL_VERSION",
                "model_version={!r}".format(document["model_version"]))
        if document["path_model_version"] != PATH_MODEL_VERSION:
            raise VirtualAContractError(
                "UNKNOWN_PATH_VERSION",
                "path_model_version={!r}".format(
                    document["path_model_version"]))
        request = _request_from_inputs(document["request_inputs"])
        request_validation = validate_virtual_a_request(request)
        if not request_validation.passed:
            raise VirtualAContractError(
                request_validation.reason, request_validation.detail,
                **request_validation.location)
        request_sha = request_fingerprint(request)
        if document["request_sha256"] != request_sha:
            raise VirtualAContractError(
                "REQUEST_DIGEST_MISMATCH",
                "model request_sha256 does not match its request_inputs")
        if expected_request is not None:
            if not isinstance(expected_request, VirtualARequest):
                raise VirtualAContractError(
                    "INVALID_REQUEST", "expected_request must be a request")
            expected_validation = validate_virtual_a_request(expected_request)
            if not expected_validation.passed:
                raise VirtualAContractError(
                    expected_validation.reason, expected_validation.detail,
                    **expected_validation.location)
            if request_payload(expected_request) != document["request_inputs"]:
                raise VirtualAContractError(
                    "REQUEST_MISMATCH",
                    "model request_inputs differ from expected_request")
            if request_fingerprint(expected_request) != request_sha:
                raise VirtualAContractError(
                    "REQUEST_DIGEST_MISMATCH",
                    "expected_request digest differs from model digest")
        path_block = _require_exact_keys(
            document["path"], _PATH_KEYS, "path")
        if path_block["model_version"] != PATH_MODEL_VERSION:
            raise VirtualAContractError(
                "UNKNOWN_PATH_VERSION",
                "path.model_version={!r}".format(path_block["model_version"]))
        timing = _require_exact_keys(
            document["timing"], _TIMING_KEYS, "timing")
        if timing["law"] != TIMING_LAW_VERSION:
            raise VirtualAContractError(
                "UNKNOWN_TIMING_LAW", "law={!r}".format(timing["law"]))
        if timing["kind"] not in ("motion", "hold"):
            raise VirtualAContractError(
                "INVALID_MODEL_JSON", "timing.kind={!r}".format(
                    timing["kind"]))
        if timing["kind"] != path_block["kind"]:
            raise VirtualAContractError(
                "MODEL_TIMING_INCONSISTENT",
                "path.kind={!r} timing.kind={!r}".format(
                    path_block["kind"], timing["kind"]))
        total_s = _json_float(timing["S"], "timing.S")
        duration = _json_float(timing["T_s"], "timing.T_s")
        start_qf_values = path_block["start_qf"]
        if not isinstance(start_qf_values, list) or len(start_qf_values) != 3:
            raise VirtualAContractError(
                "INVALID_MODEL_JSON", "path.start_qf must have three values")
        start_qf = FormationPose(*[
            _json_float(value, "path.start_qf[{}]".format(index))
            for index, value in enumerate(start_qf_values)])
        if request.start_qf != start_qf:
            raise VirtualAContractError(
                "START_MISMATCH",
                "path.start_qf differs from request start_qf")
        if timing["kind"] == "motion":
            profile_block = _require_exact_keys(
                path_block["profile"], _PROFILE_KEYS, "path.profile")
            if profile_block["kind"] != "hermite_c2":
                raise VirtualAContractError(
                    "UNSUPPORTED_PROFILE",
                    "profile kind={!r}; only production hermite_c2 is "
                    "accepted".format(profile_block["kind"]))
            if profile_block["numerics_version"] != bounds.NUMERICS_VERSION:
                raise VirtualAContractError(
                    "UNKNOWN_NUMERICS_VERSION",
                    "profile numerics_version={!r}".format(
                        profile_block["numerics_version"]))
            profile_length = _json_float(
                profile_block["length_m"], "profile.length_m")
            if profile_length <= 0.0:
                raise VirtualAContractError(
                    "INVALID_MODEL_JSON", "profile length must be > 0")
            knots = profile_block["knots"]
            if not isinstance(knots, list) or len(knots) < 2:
                raise VirtualAContractError(
                    "INVALID_MODEL_JSON",
                    "profile.knots must contain at least two knots")
            control_knots = []
            for index, knot in enumerate(knots):
                _require_exact_keys(
                    knot, _KNOT_KEYS, "profile.knots[{}]".format(index))
                control_knots.append(ControlKnot(
                    _json_float(knot["s"], "knot.s"),
                    _json_float(knot["value"], "knot.value"),
                    _json_float(knot["derivative"], "knot.derivative")))
            profile = C2CurvatureProfile.from_knots(control_knots)
            if abs(profile.length_m - profile_length) > 0.0:
                raise VirtualAContractError(
                    "MODEL_PROFILE_LENGTH_MISMATCH",
                    "profile.length_m={!r} knots end={!r}".format(
                        profile_length, profile.length_m))
            path = VirtualAPath(start_qf, profile, request.model)
        else:
            if path_block["profile"] is not None:
                raise VirtualAContractError(
                    "INVALID_MODEL_JSON",
                    "hold path must have profile=None")
            profile = None
            profile_length = 0.0
            path = VirtualAPath.hold(start_qf, request.model)
        declared_length = _json_float(
            path_block["length_m"], "path.length_m")
        if declared_length != path.length_m or declared_length != total_s:
            raise VirtualAContractError(
                "MODEL_PATH_LENGTH_MISMATCH",
                "path.length_m={!r} timing.S={!r} rebuilt={!r}".format(
                    declared_length, total_s, path.length_m))
        position_budget = _json_float(
            path_block["position_error_bound_m"],
            "path.position_error_bound_m")
        angle_budget = _json_float(
            path_block["angle_error_bound_rad"],
            "path.angle_error_bound_rad")
        if position_budget != path.position_error_bound_m or \
                angle_budget != path.angle_error_bound_rad:
            raise VirtualAContractError(
                "MODEL_NUMERICS_MISMATCH",
                "model path budgets do not match the rebuilt production path")
        integration_budget = _normalize_json(path_block["integration_budget"])
        if integration_budget != _normalize_json(path.integration_budget):
            raise VirtualAContractError(
                "MODEL_NUMERICS_MISMATCH",
                "model integration budget does not match the rebuilt path")
        numerics = _require_exact_keys(
            document["numerics"], _NUMERICS_KEYS, "numerics")
        if numerics["version"] != bounds.NUMERICS_VERSION:
            raise VirtualAContractError(
                "UNKNOWN_NUMERICS_VERSION",
                "numerics.version={!r}".format(numerics["version"]))
        expected_method = "motion" if timing["kind"] == "motion" else "hold"
        if numerics["integration_method"] != expected_method:
            raise VirtualAContractError(
                "MODEL_NUMERICS_MISMATCH",
                "numerics.integration_method={!r} expected {!r}".format(
                    numerics["integration_method"], expected_method))
        if numerics["floating_point_assumption"] != \
                _FLOATING_POINT_ASSUMPTION:
            raise VirtualAContractError(
                "MODEL_NUMERICS_MISMATCH",
                "floating_point_assumption changed")
        if _json_float(numerics["position_error_bound_m"],
                       "numerics.position_error_bound_m") != position_budget:
            raise VirtualAContractError(
                "MODEL_NUMERICS_MISMATCH",
                "numerics position budget differs from path budget")
        if _json_float(numerics["angle_error_bound_rad"],
                       "numerics.angle_error_bound_rad") != angle_budget:
            raise VirtualAContractError(
                "MODEL_NUMERICS_MISMATCH",
                "numerics angle budget differs from path budget")
        cls._check_selection_consistency(timing, total_s, request)
        if timing["kind"] == "motion":
            if duration <= 0.0:
                raise VirtualAContractError(
                    "INVALID_MODEL_JSON", "motion duration must be > 0")
            if profile_length <= 0.0:
                raise VirtualAContractError(
                    "INVALID_MODEL_JSON", "motion length must be > 0")
        else:
            if duration != 0.0 or total_s != 0.0:
                raise VirtualAContractError(
                    "INVALID_MODEL_JSON", "hold timing must have S=T=0")
            if request.start_qf != request.goal_qf:
                raise VirtualAContractError(
                    "HOLD_GOAL_MISMATCH",
                    "NO_MOTION requires an exactly identical goal")
        planning_context = document["planning_context"]
        if planning_context is not None:
            planning_context = _normalize_json(planning_context)
        plan = cls.__new__(cls)
        plan._request = request
        plan._path = path
        plan._S = total_s
        plan._T = duration
        plan._kind = timing["kind"]
        plan._selection = copy.deepcopy(timing["selection"])
        plan._planning_context = plan._request_planning_context(
            planning_context)
        plan._node_stations = cls._node_stations_for(path)
        plan._model_core = _normalize_json(document)
        if plan.to_model_json() != _canonical_json(document):
            raise VirtualAContractError(
                "INVALID_MODEL_JSON", "payload was not canonical JSON")
        return plan

    @staticmethod
    def _request_planning_context(value):
        if value is None:
            return None
        return _normalize_json(value)


def _request_from_inputs(inputs):
    _require_exact_keys(inputs, _REQUEST_KEYS, "request_inputs")
    if inputs["schema"] != REQUEST_SCHEMA:
        raise VirtualAContractError(
            "UNKNOWN_REQUEST_SCHEMA",
            "request schema={!r}".format(inputs["schema"]))
    plan_id = _json_int(inputs["plan_id"], "request.plan_id", minimum=0)
    if not isinstance(inputs["formation_id"], str) or not \
            inputs["formation_id"].strip():
        raise VirtualAContractError(
            "INVALID_REQUEST", "formation_id must be a non-empty string")
    model_rows = inputs["model"]
    if not isinstance(model_rows, list) or not model_rows:
        raise VirtualAContractError(
            "INVALID_MODEL_JSON", "request.model must be a non-empty list")
    offsets = []
    for index, row in enumerate(model_rows):
        if not isinstance(row, list) or len(row) != 3:
            raise VirtualAContractError(
                "INVALID_MODEL_JSON",
                "request.model[{}] must be [id, x, y]".format(index))
        offsets.append(VehicleOffset(
            _json_int(row[0], "offset.vehicle_id"),
            _json_float(row[1], "offset.x"),
            _json_float(row[2], "offset.y")))
    model = FormationModel(offsets)
    shape_rows = inputs["vehicle_shapes"]
    if not isinstance(shape_rows, list):
        raise VirtualAContractError(
            "INVALID_MODEL_JSON", "vehicle_shapes must be a list")
    shapes = {}
    for index, row in enumerate(shape_rows):
        if not isinstance(row, list) or len(row) != 3:
            raise VirtualAContractError(
                "INVALID_MODEL_JSON",
                "vehicle_shapes[{}] must be [id, length, width]".format(index))
        vehicle_id = _json_int(row[0], "shape.vehicle_id")
        if vehicle_id in shapes:
            raise VirtualAContractError(
                "DUPLICATE_VEHICLE_ID",
                "duplicate vehicle_id={}".format(vehicle_id))
        shapes[vehicle_id] = VehicleShape(
            _json_float(row[1], "shape.length_m"),
            _json_float(row[2], "shape.width_m"))
    start_values = inputs["start_qf"]
    goal_values = inputs["goal_qf"]
    for name, values in (("start_qf", start_values), ("goal_qf", goal_values)):
        if not isinstance(values, list) or len(values) != 3:
            raise VirtualAContractError(
                "INVALID_MODEL_JSON", "{} must have three values".format(name))
    start_qf = FormationPose(*[
        _json_float(value, "start_qf[{}]".format(index))
        for index, value in enumerate(start_values)])
    goal_qf = FormationPose(*[
        _json_float(value, "goal_qf[{}]".format(index))
        for index, value in enumerate(goal_values)])
    heading_rows = inputs["start_headings"]
    if not isinstance(heading_rows, list):
        raise VirtualAContractError(
            "INVALID_MODEL_JSON", "start_headings must be a list")
    headings = {}
    for index, row in enumerate(heading_rows):
        if not isinstance(row, list) or len(row) != 2:
            raise VirtualAContractError(
                "INVALID_MODEL_JSON",
                "start_headings[{}] must be [id, heading]".format(index))
        vehicle_id = _json_int(row[0], "heading.vehicle_id")
        if vehicle_id in headings:
            raise VirtualAContractError(
                "DUPLICATE_VEHICLE_ID",
                "duplicate heading vehicle_id={}".format(vehicle_id))
        headings[vehicle_id] = _json_float(
            row[1], "start_headings[{}]".format(index))
    initial_inputs = _require_exact_keys(
        inputs["initial"], _INITIAL_KEYS, "request.initial")
    if not isinstance(initial_inputs["stationary"], bool) or \
            not isinstance(initial_inputs["wheels_straight"], bool):
        raise VirtualAContractError(
            "INVALID_MODEL_JSON", "initial booleans must be JSON booleans")
    initial = InitialAssumption(
        stationary=initial_inputs["stationary"],
        mode=initial_inputs["mode"],
        wheels_straight=initial_inputs["wheels_straight"],
        source=initial_inputs["source"],
        context_id=initial_inputs["context_id"])
    try:
        policy = GoalPolicy[inputs["goal_policy"]]
    except (KeyError, TypeError) as exc:
        raise VirtualAContractError(
            "UNSUPPORTED_GOAL_POLICY",
            "goal_policy={!r}".format(inputs["goal_policy"])) from exc
    limit_values = inputs["limits"]
    expected_limit_keys = set(FormationTimingLimits.__dataclass_fields__)
    if not isinstance(limit_values, dict) or set(limit_values) != \
            expected_limit_keys:
        raise VirtualAContractError(
            "INVALID_MODEL_JSON", "limits keys do not match the model limits")
    limits = FormationTimingLimits(**{
        key: _json_float(value, "limits." + key)
        for key, value in limit_values.items()})
    return VirtualARequest(
        plan_id=plan_id,
        formation_id=inputs["formation_id"],
        model=model,
        vehicle_shapes=shapes,
        start_qf=start_qf,
        start_headings=headings,
        goal_qf=goal_qf,
        initial=initial,
        goal_policy=policy,
        limits=limits)