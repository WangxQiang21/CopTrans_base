"""FR01 authoritative uniform-mode analytic motion edges (no time domain).

One :class:`UniformMotionEdge` is a rigid formation motion primitive whose
global mode is A (Ackermann: straight or fixed-pivot arc), O (oblique
translation) or S (all vehicles spin about their own centres).  Every edge
stores a block-local geometric progress ``lambda`` in ``[0, L]`` and the
primitive kind; ``lambda`` is *not* time.  The derivative query returns
``dq_F/dlambda``, ``dp_i/dlambda`` and ``dtheta_i/dlambda`` with the units
published by the edge (m/m, m/rad, rad/m or rad/rad).  It never returns m/s
or rad/s.

The constructor factories accept the actual initial body headings and the
explicit direction/control branch.  They reject a heading that is not the
required tangent instead of rotating a vehicle without time; the caller must
insert an S edge to align it.  The canonical start/end wheel configurations
are stored for FR02, but no wheel speed, setup duration or timing is
implemented here.

Geometry reuses :class:`~fwis_v3_optimizer.formation_geometry.FormationPathControl`
for ``q_F(lambda)`` and :class:`~fwis_v3_optimizer.formation_kinematics.VehicleModeSegment`
only as a numeric carrier.  The global mode of an edge is authoritative; the
legacy integer mode is derived from it and is never used to mix modes.
"""
from collections.abc import Mapping as MappingABC
from dataclasses import dataclass
import math
from typing import Mapping, Optional, Tuple

from .core import Action
from .formation import (
    FormationModel,
    FormationPose,
    FormationValidationError,
    VehicleOffset,
)
from .formation_geometry import FormationPathControl
from .formation_kinematics import (
    ACKERMANN,
    BODY_SPIN,
    OBLIQUE,
    ROTATION,
    SPIN,
    TRANSLATION,
    VehicleModeSegment,
)
from .retiming import MotionLimits, wheel_configuration
from .transport_contract import TransportContract

MODE_A = "A"
MODE_O = "O"
MODE_S = "S"
UNIFORM_MODES = (MODE_A, MODE_O, MODE_S)

PRIMITIVE_A_STRAIGHT = "A_STRAIGHT"
PRIMITIVE_A_ARC = "A_ARC"
PRIMITIVE_O_TRANSLATION = "O_TRANSLATION"
PRIMITIVE_S_SPIN = "S_SPIN"
UNIFORM_PRIMITIVES = (
    PRIMITIVE_A_STRAIGHT,
    PRIMITIVE_A_ARC,
    PRIMITIVE_O_TRANSLATION,
    PRIMITIVE_S_SPIN,
)

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
_LEGACY_MODE = {
    MODE_A: ACKERMANN,
    MODE_O: OBLIQUE,
    MODE_S: SPIN,
}
_RADIAN_PRIMITIVES = (PRIMITIVE_A_ARC, PRIMITIVE_S_SPIN)

# Matching an initial body heading to a required tangent is a geometric
# identity check, not a search tolerance.  Ten nano-radians is far below the
# 1e-7 m / 1e-6 rad acceptance scale but well above atan2 round-off.
HEADING_MATCH_TOL_RAD = 1e-9
# The cargo-frame pivot must not coincide with a vehicle centre in FR01.
# This is an explicit first-version primitive-domain restriction, not a claim
# that every parked-pivot A motion is physically impossible.
PIVOT_CENTER_TOL_M = 1e-9
# Residual below which a spin increment is treated as the all-zero S case.
# A wrapped heading difference of a few 1e-16 rad is not a real spin.
SPIN_ZERO_TOL_RAD = 1e-12
# Stored derived fields are recomputed by the constructor; this tolerance
# only absorbs deterministic floating-point re-evaluation.
EDGE_FIELD_TOL = 1e-9
# Vector form of the O equation is checked component-wise.
OBLIQUE_VECTOR_TOL = 1e-9


class UniformMotionError(FormationValidationError):
    """Explicit FR01 rejection with a machine-readable reason.

    A rejected construction never returns a partially valid edge.  The
    ``reason`` is stable; ``detail`` is diagnostic.  For heading rejections,
    ``required_headings`` carries the tangent candidate that a preceding S
    edge would have to produce (principal angles in ``(-pi, pi]``).
    """

    def __init__(self, reason, detail, vehicle_id=None,
                 required_headings=None):
        self.reason = str(reason)
        self.detail = str(detail)
        self.vehicle_id = vehicle_id
        self.required_headings = dict(required_headings or {})
        super().__init__("{}: {}".format(self.reason, self.detail))


# ---------------------------------------------------------------------------
# Small numeric and mapping validators
# ---------------------------------------------------------------------------

def _finite(value, context):
    if isinstance(value, (bool, str, bytes)):
        raise UniformMotionError(
            "NONFINITE_VALUE",
            "{} value={!r} is not a finite number".format(context, value))
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise UniformMotionError(
            "NONFINITE_VALUE",
            "{} value={!r} is not a finite number".format(
                context, value)) from exc
    if not math.isfinite(result):
        raise UniformMotionError(
            "NONFINITE_VALUE",
            "{} value={!r} is not finite".format(context, value))
    return result


def _integer(value, context, allowed=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise UniformMotionError(
            "INVALID_INTEGER",
            "{} value={!r} must be an integer".format(context, value))
    if allowed is not None and value not in allowed:
        raise UniformMotionError(
            "INVALID_INTEGER",
            "{} value={} must be one of {}".format(
                context, value, allowed))
    return value


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _angle_error(first, second):
    return abs(_wrap(float(first) - float(second)))


def _require_mapping(values, vehicle_ids, context):
    if not isinstance(values, MappingABC):
        raise UniformMotionError(
            "INVALID_MAPPING",
            "{} must be a vehicle_id -> value mapping".format(context))
    for key in values.keys():
        if isinstance(key, bool) or not isinstance(key, int):
            raise UniformMotionError(
                "INVALID_VEHICLE_ID_SET",
                "{} vehicle_id={!r} must be an int".format(context, key))
    expected = set(vehicle_ids)
    actual = set(values.keys())
    missing = sorted(expected - actual)
    extra = sorted(actual - expected)
    if missing or extra:
        raise UniformMotionError(
            "INVALID_VEHICLE_ID_SET",
            "{} missing={} extra={}".format(context, missing, extra))
    return values


def _heading_pairs(values, vehicle_ids, context):
    values = _require_mapping(values, vehicle_ids, context)
    return tuple(sorted(
        (vehicle_id, _finite(values[vehicle_id],
                             "{} vehicle_id={}".format(context, vehicle_id)))
        for vehicle_id in vehicle_ids))


def _direction_pairs(values, vehicle_ids, context):
    values = _require_mapping(values, vehicle_ids, context)
    return tuple(sorted(
        (vehicle_id, _integer(
            values[vehicle_id],
            "{} vehicle_id={}".format(context, vehicle_id),
            allowed=(-1, 1)))
        for vehicle_id in vehicle_ids))


def _numeric_pairs(values, vehicle_ids, context):
    values = _require_mapping(values, vehicle_ids, context)
    return tuple(sorted(
        (vehicle_id, _finite(values[vehicle_id],
                             "{} vehicle_id={}".format(context, vehicle_id)))
        for vehicle_id in vehicle_ids))


def _pairs_dict(pairs):
    return {vehicle_id: value for vehicle_id, value in pairs}


def _pair_vehicle_id(vehicle_id, context):
    if isinstance(vehicle_id, bool) or not isinstance(vehicle_id, int):
        raise UniformMotionError(
            "INVALID_PAIR_FIELD",
            "{} vehicle_id={!r} must be an int".format(context, vehicle_id))
    return vehicle_id


def _validate_sorted_pair_field(pairs, vehicle_ids, context,
                                allow_empty=False):
    try:
        supplied = tuple(
            (_pair_vehicle_id(vehicle_id, context), value)
            for vehicle_id, value in pairs)
    except UniformMotionError:
        raise
    except (TypeError, ValueError) as exc:
        raise UniformMotionError(
            "INVALID_PAIR_FIELD",
            "{} must be an iterable of (vehicle_id, value) pairs".format(
                context)) from exc
    if allow_empty and not supplied:
        return ()
    expected_order = tuple(sorted(vehicle_ids))
    actual_order = tuple(vehicle_id for vehicle_id, _ in supplied)
    if actual_order != expected_order:
        raise UniformMotionError(
            "UNSORTED_PAIR_FIELD",
            "{} ids {} must be sorted exactly as {}".format(
                context, actual_order, expected_order))
    return tuple(
        (vehicle_id, _finite(value,
                             "{} vehicle_id={}".format(context, vehicle_id)))
        for vehicle_id, value in supplied)


def _validate_direction_field(pairs, vehicle_ids, context):
    try:
        supplied = tuple(
            (_pair_vehicle_id(vehicle_id, context), value)
            for vehicle_id, value in pairs)
    except UniformMotionError:
        raise
    except (TypeError, ValueError) as exc:
        raise UniformMotionError(
            "INVALID_PAIR_FIELD",
            "{} must be an iterable of (vehicle_id, direction) pairs".format(
                context)) from exc
    expected_order = tuple(sorted(vehicle_ids))
    if tuple(vehicle_id for vehicle_id, _ in supplied) != expected_order:
        raise UniformMotionError(
            "UNSORTED_PAIR_FIELD",
            "{} ids must be sorted exactly as {}".format(
                context, expected_order))
    return tuple(
        (vehicle_id, _integer(
            value, "{} vehicle_id={}".format(context, vehicle_id),
            allowed=(-1, 1)))
        for vehicle_id, value in supplied)


def _validate_wheel_field(pairs, vehicle_ids, context):
    try:
        supplied = tuple(
            (_pair_vehicle_id(vehicle_id, context), tuple(values))
            for vehicle_id, values in pairs)
    except UniformMotionError:
        raise
    except (TypeError, ValueError) as exc:
        raise UniformMotionError(
            "INVALID_PAIR_FIELD",
            "{} must be an iterable of (vehicle_id, four values) pairs".format(
                context)) from exc
    expected_order = tuple(sorted(vehicle_ids))
    if tuple(vehicle_id for vehicle_id, _ in supplied) != expected_order:
        raise UniformMotionError(
            "UNSORTED_PAIR_FIELD",
            "{} ids must be sorted exactly as {}".format(
                context, expected_order))
    result = []
    for vehicle_id, values in supplied:
        if len(values) != 4:
            raise UniformMotionError(
                "INVALID_PAIR_FIELD",
                "{} vehicle_id={} must contain four wheel values".format(
                    context, vehicle_id))
        result.append((vehicle_id, tuple(
            _finite(value, "{} vehicle_id={}".format(context, vehicle_id))
            for value in values)))
    return tuple(result)


def _compare_pairs(name, actual, expected, tolerance=EDGE_FIELD_TOL):
    if len(actual) != len(expected):
        raise UniformMotionError(
            "INTERNAL_EDGE_FIELD_MISMATCH",
            "{} length {} != expected {}".format(
                name, len(actual), len(expected)))
    for (actual_id, actual_value), (expected_id, expected_value) in zip(
            actual, expected):
        if actual_id != expected_id:
            raise UniformMotionError(
                "INTERNAL_EDGE_FIELD_MISMATCH",
                "{} vehicle_id {} != expected {}".format(
                    name, actual_id, expected_id))
        if abs(actual_value - expected_value) > tolerance:
            raise UniformMotionError(
                "INTERNAL_EDGE_FIELD_MISMATCH",
                "{} vehicle_id={} value={:.17g} != recomputed {:.17g}".format(
                    name, actual_id, actual_value, expected_value))


def _compare_wheel_pairs(name, actual, expected,
                         tolerance=EDGE_FIELD_TOL):
    if len(actual) != len(expected):
        raise UniformMotionError(
            "INTERNAL_EDGE_FIELD_MISMATCH",
            "{} length {} != expected {}".format(
                name, len(actual), len(expected)))
    for (actual_id, actual_values), (expected_id, expected_values) in zip(
            actual, expected):
        if actual_id != expected_id:
            raise UniformMotionError(
                "INTERNAL_EDGE_FIELD_MISMATCH",
                "{} vehicle_id {} != expected {}".format(
                    name, actual_id, expected_id))
        for index, (actual_value, expected_value) in enumerate(
                zip(actual_values, expected_values)):
            if abs(actual_value - expected_value) > tolerance:
                raise UniformMotionError(
                    "INTERNAL_EDGE_FIELD_MISMATCH",
                    "{} vehicle_id={} wheel={} value={:.17g} != "
                    "recomputed {:.17g}".format(
                        name, actual_id, index, actual_value,
                        expected_value))


def _wheel_values(legacy_mode, control, limits, vehicle_id, context):
    angles, factors = wheel_configuration(
        Action(legacy_mode, 0.0, float(control)), limits)
    angles = tuple(float(value) for value in angles)
    factors = tuple(float(value) for value in factors)
    if any(not math.isfinite(value) for value in angles + factors):
        raise UniformMotionError(
            "NONFINITE_WHEEL_CONFIGURATION",
            "{} vehicle_id={} produced a non-finite wheel configuration"
            .format(context, vehicle_id), vehicle_id=vehicle_id)
    if max(abs(value) for value in angles) > limits.steer_angle_max + 1e-9:
        raise UniformMotionError(
            "WHEEL_ANGLE_LIMIT",
            "{} vehicle_id={} wheel angle {:.12g} exceeds limit {:.12g} rad"
            .format(context, vehicle_id,
                    max(abs(value) for value in angles),
                    limits.steer_angle_max), vehicle_id=vehicle_id)
    return angles, factors


# ---------------------------------------------------------------------------
# Public convenience constructors
# ---------------------------------------------------------------------------

def formation_model_from_contract(contract):
    """Build the rigid model from the authoritative transport contract."""
    if not isinstance(contract, TransportContract):
        raise UniformMotionError(
            "INVALID_CONTRACT",
            "expected a TransportContract, got {!r}".format(type(contract)))
    offsets = tuple(
        VehicleOffset(vehicle_id, float(offset[0]), float(offset[1]))
        for vehicle_id, offset in zip(contract.vehicle_ids, contract.offsets))
    return FormationModel(offsets)


def a_straight_required_headings(model, displacement, directions):
    """Principal body headings needed by an A straight edge.

    This is the pure candidate generator that a search layer may use before
    inserting an explicit S alignment edge.  It does not modify any state.
    """
    if not isinstance(model, FormationModel):
        raise UniformMotionError(
            "INVALID_MODEL", "expected a FormationModel")
    dx = _finite(displacement[0], "a_straight displacement.x")
    dy = _finite(displacement[1], "a_straight displacement.y")
    length = math.hypot(dx, dy)
    if length <= 0.0:
        raise UniformMotionError(
            "ZERO_TRANSLATION",
            "A straight needs a non-zero displacement")
    direction_pairs = _direction_pairs(
        directions, model.vehicle_ids, "a_straight directions")
    direction_map = _pairs_dict(direction_pairs)
    tangent = math.atan2(dy, dx)
    result = {}
    for vehicle_id in model.vehicle_ids:
        sigma = direction_map[vehicle_id]
        result[vehicle_id] = _wrap(
            tangent if sigma > 0 else tangent + math.pi)
    return result


def a_arc_required_headings(model, start_qf, delta_psi, pivot, directions,
                            limits=None):
    """Principal body headings needed by an A fixed-pivot arc edge.

    The pivot is in the cargo frame.  A vehicle whose pivot radius is below
    the first-version domain raises the same explicit rejection as the edge
    factory; the helper never silently substitutes an S vehicle.
    """
    if not isinstance(model, FormationModel):
        raise UniformMotionError(
            "INVALID_MODEL", "expected a FormationModel")
    if not isinstance(start_qf, FormationPose):
        raise UniformMotionError(
            "INVALID_START_POSE", "expected a FormationPose")
    if limits is None:
        limits = MotionLimits()
    if not isinstance(limits, MotionLimits):
        raise UniformMotionError(
            "INVALID_LIMITS", "limits must be a MotionLimits or None")
    delta_psi = _finite(delta_psi, "a_arc delta_psi")
    if delta_psi == 0.0:
        raise UniformMotionError(
            "ZERO_ROTATION", "A arc needs a non-zero delta_psi")
    pivot_x = _finite(pivot[0], "a_arc pivot.x")
    pivot_y = _finite(pivot[1], "a_arc pivot.y")
    direction_pairs = _direction_pairs(
        directions, model.vehicle_ids, "a_arc directions")
    direction_map = _pairs_dict(direction_pairs)
    sign = 1 if delta_psi > 0.0 else -1
    result = {}
    for offset in model.offsets:
        vehicle_id = offset.vehicle_id
        rx = offset.x - pivot_x
        ry = offset.y - pivot_y
        radius = math.hypot(rx, ry)
        if radius <= PIVOT_CENTER_TOL_M:
            raise UniformMotionError(
                "PIVOT_AT_VEHICLE_CENTER_UNSUPPORTED",
                "vehicle_id={} pivot coincides with the vehicle centre; "
                "FR01 does not implement the degenerate parked-pivot A "
                "branch (no general physical impossibility is claimed)"
                .format(vehicle_id), vehicle_id=vehicle_id)
        if radius < limits.r_min_ack - EDGE_FIELD_TOL:
            raise UniformMotionError(
                "INSUFFICIENT_TURN_RADIUS",
                "vehicle_id={} pivot radius {:.12g} m is below the "
                "Ackermann minimum {:.12g} m".format(
                    vehicle_id, radius, limits.r_min_ack),
                vehicle_id=vehicle_id)
        tangent = start_qf.psi + math.atan2(rx, -ry)
        motion = tangent if sign > 0 else tangent + math.pi
        sigma = direction_map[vehicle_id]
        result[vehicle_id] = _wrap(
            motion if sigma > 0 else motion + math.pi)
    return result


# ---------------------------------------------------------------------------
# Public sample/value types
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class UniformMotionState:
    """Geometric state at one progress value; progress is not time."""

    progress: float
    qf: FormationPose
    centers: Tuple[Tuple[int, float, float], ...]
    headings: Tuple[Tuple[int, float], ...]

    def center(self, vehicle_id):
        return _pairs_dict(
            ((vid, (x, y)) for vid, x, y in self.centers))[vehicle_id]

    def heading(self, vehicle_id):
        return _pairs_dict(self.headings)[vehicle_id]

    def as_dict(self):
        return {
            "progress": self.progress,
            "qf": [self.qf.x, self.qf.y, self.qf.psi],
            "centers": [
                {"vehicle_id": vehicle_id, "x": x, "y": y}
                for vehicle_id, x, y in self.centers],
            "headings": [
                {"vehicle_id": vehicle_id, "theta": theta}
                for vehicle_id, theta in self.headings],
        }


@dataclass(frozen=True)
class UniformMotionDerivative:
    """Geometric first derivative with explicit, non-time units.

    ``dqf`` is ``dq_F/dlambda``.  ``dp`` and ``dtheta`` are ``dp_i/dlambda``
    and ``dtheta_i/dlambda``.  For a metre-progress edge the position unit is
    m/m and the angle unit rad/m; for a radian-progress edge they are m/rad
    and rad/rad.  These are never m/s or rad/s.
    """

    progress: float
    position_unit: str
    angle_unit: str
    dqf: Tuple[float, float, float]
    dp: Tuple[Tuple[int, float, float], ...]
    dtheta: Tuple[Tuple[int, float], ...]

    def as_dict(self):
        return {
            "progress": self.progress,
            "position_unit": self.position_unit,
            "angle_unit": self.angle_unit,
            "dqf": list(self.dqf),
            "dp": [
                {"vehicle_id": vehicle_id, "x": x, "y": y}
                for vehicle_id, x, y in self.dp],
            "dtheta": [
                {"vehicle_id": vehicle_id, "theta_rate": rate}
                for vehicle_id, rate in self.dtheta],
        }


@dataclass(frozen=True)
class VehicleWheelConfiguration:
    """Canonical four-wheel angles and geometric rate factors for one body."""

    vehicle_id: int
    angles: Tuple[float, float, float, float]
    factors: Tuple[float, float, float, float]

    def as_dict(self):
        return {
            "vehicle_id": self.vehicle_id,
            "wheel_angles": list(self.angles),
            "wheel_factors": list(self.factors),
        }


@dataclass(frozen=True)
class VehicleZeroCurvatureDelta:
    """Per-wheel angle difference from the A zero-curvature configuration.

    The value is a geometric angle difference in radians, not a setup time.
    """

    vehicle_id: int
    angles: Tuple[float, float, float, float]
    max_abs_angle_rad: float

    def as_dict(self):
        return {
            "vehicle_id": self.vehicle_id,
            "angles_from_a_zero_curvature_rad": list(self.angles),
            "max_abs_angle_rad": self.max_abs_angle_rad,
        }


@dataclass(frozen=True)
class UniformEdgeValidation:
    """Independent audit metrics for one edge (geometry only, no timing)."""

    primitive: str
    sample_count: int
    max_endpoint_qf_error_m: float
    max_endpoint_heading_error_rad: float
    max_rigidity_residual_m: float
    max_mode_equation_residual: float
    max_derivative_error: float
    max_wheel_angle_rad: float

    def as_dict(self):
        return {
            "primitive": self.primitive,
            "sample_count": self.sample_count,
            "max_endpoint_qf_error_m": self.max_endpoint_qf_error_m,
            "max_endpoint_heading_error_rad":
                self.max_endpoint_heading_error_rad,
            "max_rigidity_residual_m": self.max_rigidity_residual_m,
            "max_mode_equation_residual": self.max_mode_equation_residual,
            "max_derivative_error": self.max_derivative_error,
            "max_wheel_angle_rad": self.max_wheel_angle_rad,
        }


# ---------------------------------------------------------------------------
# Derivation shared by factories and constructor re-validation
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class _DerivedFields:
    end_headings: Tuple[Tuple[int, float], ...]
    curvatures: Tuple[Tuple[int, float], ...]
    oblique_angles: Tuple[Tuple[int, float], ...]
    signed_progress_speeds: Tuple[Tuple[int, float], ...]
    wheel_angles_start: Tuple[Tuple[int, Tuple[float, ...]], ...]
    wheel_factors_start: Tuple[Tuple[int, Tuple[float, ...]], ...]
    wheel_angles_end: Tuple[Tuple[int, Tuple[float, ...]], ...]
    wheel_factors_end: Tuple[Tuple[int, Tuple[float, ...]], ...]


def _require_heading_match(vehicle_id, theta, expected, required):
    if _angle_error(theta, expected) > HEADING_MATCH_TOL_RAD:
        raise UniformMotionError(
            "A_HEADING_REQUIRES_S_ALIGNMENT",
            "vehicle_id={} body heading {:.12g} is not the required tangent "
            "{:.12g} (mod 2*pi); insert an explicit S alignment edge; the "
            "factory will not rotate the body without time".format(
                vehicle_id, theta, _wrap(expected)),
            vehicle_id=vehicle_id, required_headings=required)


def _derive_fields(mode, primitive, model, control, start_headings,
                   end_headings, directions, oblique_angles, limits,
                   allow_reverse):
    vehicle_ids = model.vehicle_ids
    start_map = _pairs_dict(start_headings)
    end_map = _pairs_dict(end_headings)
    direction_map = _pairs_dict(directions)
    oblique_map = _pairs_dict(oblique_angles)
    length = control.parameter_length
    sign = None
    if primitive == PRIMITIVE_A_ARC:
        delta_psi = control.delta_psi
        if abs(abs(delta_psi) - length) > EDGE_FIELD_TOL:
            raise UniformMotionError(
                "DELTA_PSI_LENGTH_MISMATCH",
                "A arc parameter_length={:.17g} must equal |dpsi|={:.17g}"
                .format(length, abs(delta_psi)))
        sign = 1 if delta_psi > 0.0 else -1

    end_result = {}
    curvature_result = {}
    oblique_result = {}
    speed_result = {}
    wheel_start = {}
    wheel_end = {}

    if primitive == PRIMITIVE_A_STRAIGHT:
        if control.geometry_type != TRANSLATION:
            raise UniformMotionError(
                "INVALID_CONTROL_FOR_PRIMITIVE",
                "A straight requires a translation FormationPathControl")
        dx = control.end_qf.x - control.start_qf.x
        dy = control.end_qf.y - control.start_qf.y
        displacement = math.hypot(dx, dy)
        if abs(displacement - length) > EDGE_FIELD_TOL:
            raise UniformMotionError(
                "CONTROL_LENGTH_MISMATCH",
                "A straight parameter_length={:.17g} does not equal "
                "|displacement|={:.17g}".format(length, displacement))
        tangent = math.atan2(dy, dx)
        required = {
            vehicle_id: _wrap(
                tangent if direction_map[vehicle_id] > 0
                else tangent + math.pi)
            for vehicle_id in vehicle_ids}
        for offset in model.offsets:
            vehicle_id = offset.vehicle_id
            theta = start_map[vehicle_id]
            sigma = direction_map[vehicle_id]
            _require_heading_match(
                vehicle_id, theta, required[vehicle_id], required)
            if sigma < 0 and not allow_reverse:
                raise UniformMotionError(
                    "REVERSE_NOT_ALLOWED",
                    "vehicle_id={} A straight would travel backward but "
                    "allow_reverse is false".format(vehicle_id),
                    vehicle_id=vehicle_id)
            if abs(end_map[vehicle_id] - theta) > EDGE_FIELD_TOL:
                raise UniformMotionError(
                    "INTERNAL_EDGE_FIELD_MISMATCH",
                    "A straight vehicle_id={} must keep its body heading"
                    .format(vehicle_id), vehicle_id=vehicle_id)
            end_result[vehicle_id] = theta
            curvature_result[vehicle_id] = 0.0
            speed_result[vehicle_id] = float(sigma)
            wheel_start[vehicle_id] = _wheel_values(
                ACKERMANN, 0.0, limits, vehicle_id, "A straight")
            wheel_end[vehicle_id] = wheel_start[vehicle_id]
        return _DerivedFields(
            end_headings=tuple(sorted(end_result.items())),
            curvatures=tuple(sorted(curvature_result.items())),
            oblique_angles=(),
            signed_progress_speeds=tuple(sorted(speed_result.items())),
            wheel_angles_start=tuple(sorted(
                (vehicle_id, values[0])
                for vehicle_id, values in wheel_start.items())),
            wheel_factors_start=tuple(sorted(
                (vehicle_id, values[1])
                for vehicle_id, values in wheel_start.items())),
            wheel_angles_end=tuple(sorted(
                (vehicle_id, values[0])
                for vehicle_id, values in wheel_end.items())),
            wheel_factors_end=tuple(sorted(
                (vehicle_id, values[1])
                for vehicle_id, values in wheel_end.items())),
        )

    if primitive == PRIMITIVE_A_ARC:
        if control.geometry_type != ROTATION:
            raise UniformMotionError(
                "INVALID_CONTROL_FOR_PRIMITIVE",
                "A arc requires a rotation FormationPathControl")
        pivot_x = control.pivot_x
        pivot_y = control.pivot_y
        required = {}
        radii = {}
        for offset in model.offsets:
            vehicle_id = offset.vehicle_id
            rx = offset.x - pivot_x
            ry = offset.y - pivot_y
            radius = math.hypot(rx, ry)
            radii[vehicle_id] = radius
            if radius <= PIVOT_CENTER_TOL_M:
                raise UniformMotionError(
                    "PIVOT_AT_VEHICLE_CENTER_UNSUPPORTED",
                    "vehicle_id={} pivot coincides with the vehicle centre; "
                    "FR01 does not implement the degenerate parked-pivot A "
                    "branch (no general physical impossibility is claimed)"
                    .format(vehicle_id), vehicle_id=vehicle_id)
            if radius < limits.r_min_ack - EDGE_FIELD_TOL:
                raise UniformMotionError(
                    "INSUFFICIENT_TURN_RADIUS",
                    "vehicle_id={} pivot radius {:.12g} m is below the "
                    "Ackermann minimum {:.12g} m; the cargo radius alone does "
                    "not certify this primitive".format(
                        vehicle_id, radius, limits.r_min_ack),
                    vehicle_id=vehicle_id)
            tangent = control.start_qf.psi + math.atan2(rx, -ry)
            motion = tangent if sign > 0 else tangent + math.pi
            required[vehicle_id] = _wrap(
                motion if direction_map[vehicle_id] > 0
                else motion + math.pi)
        for offset in model.offsets:
            vehicle_id = offset.vehicle_id
            theta = start_map[vehicle_id]
            sigma = direction_map[vehicle_id]
            _require_heading_match(
                vehicle_id, theta, required[vehicle_id], required)
            if sigma < 0 and not allow_reverse:
                raise UniformMotionError(
                    "REVERSE_NOT_ALLOWED",
                    "vehicle_id={} A arc would travel backward but "
                    "allow_reverse is false".format(vehicle_id),
                    vehicle_id=vehicle_id)
            radius = radii[vehicle_id]
            curvature = sign / (sigma * radius)
            if abs(curvature) > 1.0 / limits.r_min_ack + EDGE_FIELD_TOL:
                raise UniformMotionError(
                    "CURVATURE_LIMIT",
                    "vehicle_id={} |kappa|={:.12g} exceeds 1/Rmin={:.12g}"
                    .format(vehicle_id, abs(curvature),
                            1.0 / limits.r_min_ack),
                    vehicle_id=vehicle_id)
            expected_end = theta + control.delta_psi
            if abs(end_map[vehicle_id] - expected_end) > EDGE_FIELD_TOL:
                raise UniformMotionError(
                    "INTERNAL_EDGE_FIELD_MISMATCH",
                    "A arc vehicle_id={} end heading {:.12g} != start+dpsi "
                    "{:.12g}".format(
                        vehicle_id, end_map[vehicle_id], expected_end),
                    vehicle_id=vehicle_id)
            end_result[vehicle_id] = expected_end
            curvature_result[vehicle_id] = curvature
            speed_result[vehicle_id] = sigma * radius
            wheel_start[vehicle_id] = _wheel_values(
                ACKERMANN, curvature, limits, vehicle_id, "A arc")
            wheel_end[vehicle_id] = wheel_start[vehicle_id]
        return _DerivedFields(
            end_headings=tuple(sorted(end_result.items())),
            curvatures=tuple(sorted(curvature_result.items())),
            oblique_angles=(),
            signed_progress_speeds=tuple(sorted(speed_result.items())),
            wheel_angles_start=tuple(sorted(
                (vehicle_id, values[0])
                for vehicle_id, values in wheel_start.items())),
            wheel_factors_start=tuple(sorted(
                (vehicle_id, values[1])
                for vehicle_id, values in wheel_start.items())),
            wheel_angles_end=tuple(sorted(
                (vehicle_id, values[0])
                for vehicle_id, values in wheel_end.items())),
            wheel_factors_end=tuple(sorted(
                (vehicle_id, values[1])
                for vehicle_id, values in wheel_end.items())),
        )

    if primitive == PRIMITIVE_O_TRANSLATION:
        if control.geometry_type != TRANSLATION:
            raise UniformMotionError(
                "INVALID_CONTROL_FOR_PRIMITIVE",
                "O translation requires a translation FormationPathControl")
        dx = control.end_qf.x - control.start_qf.x
        dy = control.end_qf.y - control.start_qf.y
        displacement = math.hypot(dx, dy)
        if displacement <= 0.0:
            raise UniformMotionError(
                "ZERO_TRANSLATION",
                "O translation needs a non-zero displacement")
        if abs(displacement - length) > EDGE_FIELD_TOL:
            raise UniformMotionError(
                "CONTROL_LENGTH_MISMATCH",
                "O parameter_length={:.17g} does not equal "
                "|displacement|={:.17g}".format(length, displacement))
        direction_x = dx / displacement
        direction_y = dy / displacement
        for offset in model.offsets:
            vehicle_id = offset.vehicle_id
            theta = start_map[vehicle_id]
            sigma = direction_map[vehicle_id]
            phi = oblique_map[vehicle_id]
            if abs(phi) > math.pi / 2.0 + EDGE_FIELD_TOL:
                raise UniformMotionError(
                    "OBLIQUE_ANGLE_LIMIT",
                    "vehicle_id={} |phi|={:.12g} exceeds pi/2".format(
                        vehicle_id, abs(phi)), vehicle_id=vehicle_id)
            expected_x = sigma * math.cos(theta + phi)
            expected_y = sigma * math.sin(theta + phi)
            residual = max(
                abs(expected_x - direction_x),
                abs(expected_y - direction_y))
            if residual > OBLIQUE_VECTOR_TOL:
                raise UniformMotionError(
                    "OBLIQUE_VECTOR_MISMATCH",
                    "vehicle_id={} sigma*e(theta+phi)=({:.17g}, {:.17g}) "
                    "does not equal the full displacement direction "
                    "({:.17g}, {:.17g}); residual={:.3g}".format(
                        vehicle_id, expected_x, expected_y,
                        direction_x, direction_y, residual),
                    vehicle_id=vehicle_id)
            if sigma < 0 and not allow_reverse:
                raise UniformMotionError(
                    "REVERSE_NOT_ALLOWED",
                    "vehicle_id={} O translation would travel backward but "
                    "allow_reverse is false".format(vehicle_id),
                    vehicle_id=vehicle_id)
            if abs(end_map[vehicle_id] - theta) > EDGE_FIELD_TOL:
                raise UniformMotionError(
                    "INTERNAL_EDGE_FIELD_MISMATCH",
                    "O translation vehicle_id={} must keep its heading"
                    .format(vehicle_id), vehicle_id=vehicle_id)
            end_result[vehicle_id] = theta
            oblique_result[vehicle_id] = phi
            speed_result[vehicle_id] = float(sigma)
            wheel_start[vehicle_id] = _wheel_values(
                OBLIQUE, phi, limits, vehicle_id, "O translation")
            wheel_end[vehicle_id] = wheel_start[vehicle_id]
        return _DerivedFields(
            end_headings=tuple(sorted(end_result.items())),
            curvatures=(),
            oblique_angles=tuple(sorted(oblique_result.items())),
            signed_progress_speeds=tuple(sorted(speed_result.items())),
            wheel_angles_start=tuple(sorted(
                (vehicle_id, values[0])
                for vehicle_id, values in wheel_start.items())),
            wheel_factors_start=tuple(sorted(
                (vehicle_id, values[1])
                for vehicle_id, values in wheel_start.items())),
            wheel_angles_end=tuple(sorted(
                (vehicle_id, values[0])
                for vehicle_id, values in wheel_end.items())),
            wheel_factors_end=tuple(sorted(
                (vehicle_id, values[1])
                for vehicle_id, values in wheel_end.items())),
        )

    if primitive == PRIMITIVE_S_SPIN:
        if control.geometry_type != BODY_SPIN:
            raise UniformMotionError(
                "INVALID_CONTROL_FOR_PRIMITIVE",
                "S spin requires a body_spin FormationPathControl")
        deltas = {}
        for vehicle_id in vehicle_ids:
            delta = end_map[vehicle_id] - start_map[vehicle_id]
            if abs(delta) > math.pi + 1e-12:
                raise UniformMotionError(
                    "SPIN_INCREMENT_OUT_OF_RANGE",
                    "vehicle_id={} delta_theta={:.12g} is outside [-pi, pi]"
                    .format(vehicle_id, delta), vehicle_id=vehicle_id)
            deltas[vehicle_id] = delta
        largest = max(abs(value) for value in deltas.values())
        if largest <= SPIN_ZERO_TOL_RAD:
            raise UniformMotionError(
                "ZERO_SPIN_NO_EDGE",
                "every delta_theta is zero; no S edge is generated and no "
                "positive L is fabricated")
        if abs(largest - length) > EDGE_FIELD_TOL:
            raise UniformMotionError(
                "SPIN_LENGTH_MISMATCH",
                "S parameter_length={:.17g} must equal max|delta_theta|="
                "{:.17g}".format(length, largest))
        for offset in model.offsets:
            vehicle_id = offset.vehicle_id
            delta = deltas[vehicle_id]
            expected_direction = (
                -1 if delta < -SPIN_ZERO_TOL_RAD else 1)
            if direction_map[vehicle_id] != expected_direction:
                raise UniformMotionError(
                    "DIRECTION_MISMATCH",
                    "vehicle_id={} direction={} does not match sign(delta_"
                    "theta)={}; a zero-angle vehicle uses the +1 placeholder"
                    .format(vehicle_id, direction_map[vehicle_id],
                            expected_direction),
                    vehicle_id=vehicle_id)
            end_result[vehicle_id] = end_map[vehicle_id]
            speed_result[vehicle_id] = 0.0
            wheel_start[vehicle_id] = _wheel_values(
                SPIN, 0.0, limits, vehicle_id, "S spin")
            wheel_end[vehicle_id] = wheel_start[vehicle_id]
        return _DerivedFields(
            end_headings=tuple(sorted(end_result.items())),
            curvatures=(),
            oblique_angles=(),
            signed_progress_speeds=tuple(sorted(speed_result.items())),
            wheel_angles_start=tuple(sorted(
                (vehicle_id, values[0])
                for vehicle_id, values in wheel_start.items())),
            wheel_factors_start=tuple(sorted(
                (vehicle_id, values[1])
                for vehicle_id, values in wheel_start.items())),
            wheel_angles_end=tuple(sorted(
                (vehicle_id, values[0])
                for vehicle_id, values in wheel_end.items())),
            wheel_factors_end=tuple(sorted(
                (vehicle_id, values[1])
                for vehicle_id, values in wheel_end.items())),
        )

    raise UniformMotionError(
        "INVALID_PRIMITIVE", "unknown primitive {!r}".format(primitive))


def _build_segments(mode, primitive, model, control, start_headings,
                    directions, derived):
    start_centers = model.centers(control.start_qf)
    end_centers = model.centers(control.end_qf)
    start_map = _pairs_dict(start_headings)
    end_map = _pairs_dict(derived.end_headings)
    direction_map = _pairs_dict(directions)
    curvature_map = _pairs_dict(derived.curvatures)
    oblique_map = _pairs_dict(derived.oblique_angles)
    speed_map = _pairs_dict(derived.signed_progress_speeds)
    length = control.parameter_length
    pivot_x = control.pivot_x if primitive == PRIMITIVE_A_ARC else 0.0
    pivot_y = control.pivot_y if primitive == PRIMITIVE_A_ARC else 0.0
    segments = []
    for offset in model.offsets:
        vehicle_id = offset.vehicle_id
        if primitive == PRIMITIVE_A_STRAIGHT:
            control_value = 0.0
            travel = length
        elif primitive == PRIMITIVE_A_ARC:
            control_value = curvature_map[vehicle_id]
            travel = abs(speed_map[vehicle_id]) * length
        elif primitive == PRIMITIVE_O_TRANSLATION:
            control_value = oblique_map[vehicle_id]
            travel = length
        else:
            control_value = 0.0
            travel = 0.0
        start_center = start_centers[vehicle_id]
        end_center = end_centers[vehicle_id]
        segments.append(VehicleModeSegment(
            vehicle_id=vehicle_id,
            start_station=0.0,
            end_station=length,
            mode=_LEGACY_MODE[mode],
            direction=direction_map[vehicle_id],
            theta_start=start_map[vehicle_id],
            theta_end=end_map[vehicle_id],
            control=control_value,
            travel_distance=travel,
            start_center_x=start_center[0],
            start_center_y=start_center[1],
            end_center_x=end_center[0],
            end_center_y=end_center[1],
            pivot_x=pivot_x,
            pivot_y=pivot_y))
    return tuple(segments)


def _compare_segments(actual, expected):
    if len(actual) != len(expected):
        raise UniformMotionError(
            "INTERNAL_EDGE_FIELD_MISMATCH",
            "segments length {} != expected {}".format(
                len(actual), len(expected)))
    for actual_segment, expected_segment in zip(actual, expected):
        fields = (
            ("vehicle_id", actual_segment.vehicle_id,
             expected_segment.vehicle_id, 0.0),
            ("start_station", actual_segment.start_station,
             expected_segment.start_station, EDGE_FIELD_TOL),
            ("end_station", actual_segment.end_station,
             expected_segment.end_station, EDGE_FIELD_TOL),
            ("mode", actual_segment.mode, expected_segment.mode, 0.0),
            ("direction", actual_segment.direction,
             expected_segment.direction, 0.0),
            ("theta_start", actual_segment.theta_start,
             expected_segment.theta_start, EDGE_FIELD_TOL),
            ("theta_end", actual_segment.theta_end,
             expected_segment.theta_end, EDGE_FIELD_TOL),
            ("control", actual_segment.control,
             expected_segment.control, EDGE_FIELD_TOL),
            ("travel_distance", actual_segment.travel_distance,
             expected_segment.travel_distance, EDGE_FIELD_TOL),
            ("start_center_x", actual_segment.start_center_x,
             expected_segment.start_center_x, EDGE_FIELD_TOL),
            ("start_center_y", actual_segment.start_center_y,
             expected_segment.start_center_y, EDGE_FIELD_TOL),
            ("end_center_x", actual_segment.end_center_x,
             expected_segment.end_center_x, EDGE_FIELD_TOL),
            ("end_center_y", actual_segment.end_center_y,
             expected_segment.end_center_y, EDGE_FIELD_TOL),
            ("pivot_x", actual_segment.pivot_x,
             expected_segment.pivot_x, EDGE_FIELD_TOL),
            ("pivot_y", actual_segment.pivot_y,
             expected_segment.pivot_y, EDGE_FIELD_TOL),
        )
        for name, actual_value, expected_value, tolerance in fields:
            if abs(actual_value - expected_value) > tolerance:
                raise UniformMotionError(
                    "INTERNAL_EDGE_FIELD_MISMATCH",
                    "segments vehicle_id={} {}={!r} != recomputed {!r}"
                    .format(actual_segment.vehicle_id, name,
                            actual_value, expected_value),
                    vehicle_id=actual_segment.vehicle_id)


# ---------------------------------------------------------------------------
# Authoritative edge
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class UniformMotionEdge:
    """One immutable uniform-mode analytic motion edge.

    Primary decision fields are ``mode``, ``primitive``, ``control`` and the
    per-vehicle headings/directions/controls.  The remaining numeric fields
    are derived and are re-derived by ``__post_init__``; a tampered derived
    field is rejected instead of yielding a half-valid edge.
    """

    mode: str
    primitive: str
    model: FormationModel
    control: FormationPathControl
    start_headings: Tuple[Tuple[int, float], ...]
    end_headings: Tuple[Tuple[int, float], ...]
    directions: Tuple[Tuple[int, int], ...]
    curvatures: Tuple[Tuple[int, float], ...]
    oblique_angles: Tuple[Tuple[int, float], ...]
    signed_progress_speeds: Tuple[Tuple[int, float], ...]
    wheel_angles_start: Tuple[Tuple[int, Tuple[float, ...]], ...]
    wheel_factors_start: Tuple[Tuple[int, Tuple[float, ...]], ...]
    wheel_angles_end: Tuple[Tuple[int, Tuple[float, ...]], ...]
    wheel_factors_end: Tuple[Tuple[int, Tuple[float, ...]], ...]
    segments: Tuple[VehicleModeSegment, ...]
    limits: MotionLimits
    allow_reverse: bool

    def __post_init__(self):
        if self.mode not in UNIFORM_MODES:
            raise UniformMotionError(
                "INVALID_MODE", "mode={!r} is not one of {}".format(
                    self.mode, UNIFORM_MODES))
        if self.primitive not in UNIFORM_PRIMITIVES:
            raise UniformMotionError(
                "INVALID_PRIMITIVE",
                "primitive={!r} is not one of {}".format(
                    self.primitive, UNIFORM_PRIMITIVES))
        if _PRIMITIVE_MODE[self.primitive] != self.mode:
            raise UniformMotionError(
                "MODE_PRIMITIVE_MISMATCH",
                "mode={} is inconsistent with primitive={}".format(
                    self.mode, self.primitive))
        if not isinstance(self.model, FormationModel):
            raise UniformMotionError(
                "INVALID_MODEL", "model must be a FormationModel")
        if not isinstance(self.control, FormationPathControl):
            raise UniformMotionError(
                "INVALID_CONTROL",
                "control must be a FormationPathControl")
        if self.control.geometry_type != _PRIMITIVE_GEOMETRY[self.primitive]:
            raise UniformMotionError(
                "INVALID_CONTROL_FOR_PRIMITIVE",
                "primitive={} requires geometry_type={}, got {}".format(
                    self.primitive,
                    _PRIMITIVE_GEOMETRY[self.primitive],
                    self.control.geometry_type))
        if not isinstance(self.limits, MotionLimits):
            raise UniformMotionError(
                "INVALID_LIMITS", "limits must be a MotionLimits")
        if not isinstance(self.allow_reverse, bool):
            raise UniformMotionError(
                "INVALID_ALLOW_REVERSE", "allow_reverse must be a bool")
        vehicle_ids = self.model.vehicle_ids
        start_pairs = _validate_sorted_pair_field(
            self.start_headings, vehicle_ids, "start_headings")
        end_pairs = _validate_sorted_pair_field(
            self.end_headings, vehicle_ids, "end_headings")
        direction_pairs = _validate_direction_field(
            self.directions, vehicle_ids, "directions")
        curvature_pairs = _validate_sorted_pair_field(
            self.curvatures, vehicle_ids, "curvatures", allow_empty=True)
        oblique_pairs = _validate_sorted_pair_field(
            self.oblique_angles, vehicle_ids, "oblique_angles",
            allow_empty=True)
        speed_pairs = _validate_sorted_pair_field(
            self.signed_progress_speeds, vehicle_ids,
            "signed_progress_speeds")
        wheel_angles_start = _validate_wheel_field(
            self.wheel_angles_start, vehicle_ids, "wheel_angles_start")
        wheel_factors_start = _validate_wheel_field(
            self.wheel_factors_start, vehicle_ids, "wheel_factors_start")
        wheel_angles_end = _validate_wheel_field(
            self.wheel_angles_end, vehicle_ids, "wheel_angles_end")
        wheel_factors_end = _validate_wheel_field(
            self.wheel_factors_end, vehicle_ids, "wheel_factors_end")

        if self.primitive in (PRIMITIVE_A_STRAIGHT, PRIMITIVE_A_ARC):
            if not curvature_pairs:
                raise UniformMotionError(
                    "INTERNAL_EDGE_FIELD_MISMATCH",
                    "{} requires per-vehicle curvatures".format(
                        self.primitive))
            if oblique_pairs:
                raise UniformMotionError(
                    "INTERNAL_EDGE_FIELD_MISMATCH",
                    "{} must not carry oblique angles".format(
                        self.primitive))
        elif self.primitive == PRIMITIVE_O_TRANSLATION:
            if curvature_pairs:
                raise UniformMotionError(
                    "INTERNAL_EDGE_FIELD_MISMATCH",
                    "O translation must not carry curvatures")
            if not oblique_pairs:
                raise UniformMotionError(
                    "INTERNAL_EDGE_FIELD_MISMATCH",
                    "O translation requires per-vehicle oblique angles")
        else:
            if curvature_pairs or oblique_pairs:
                raise UniformMotionError(
                    "INTERNAL_EDGE_FIELD_MISMATCH",
                    "S spin carries neither curvatures nor oblique angles")

        derived = _derive_fields(
            self.mode, self.primitive, self.model, self.control,
            start_pairs, end_pairs, direction_pairs, oblique_pairs,
            self.limits, self.allow_reverse)
        _compare_pairs("end_headings", end_pairs, derived.end_headings)
        _compare_pairs("curvatures", curvature_pairs, derived.curvatures)
        _compare_pairs("oblique_angles", oblique_pairs,
                       derived.oblique_angles)
        _compare_pairs("signed_progress_speeds", speed_pairs,
                       derived.signed_progress_speeds)
        _compare_wheel_pairs(
            "wheel_angles_start", wheel_angles_start,
            derived.wheel_angles_start)
        _compare_wheel_pairs(
            "wheel_factors_start", wheel_factors_start,
            derived.wheel_factors_start)
        _compare_wheel_pairs(
            "wheel_angles_end", wheel_angles_end,
            derived.wheel_angles_end)
        _compare_wheel_pairs(
            "wheel_factors_end", wheel_factors_end,
            derived.wheel_factors_end)
        expected_segments = _build_segments(
            self.mode, self.primitive, self.model, self.control,
            start_pairs, direction_pairs, derived)
        _compare_segments(tuple(self.segments), expected_segments)

        # Store canonical sorted tuples so later consumers see one order.
        object.__setattr__(self, "start_headings", start_pairs)
        object.__setattr__(self, "end_headings", end_pairs)
        object.__setattr__(self, "directions", direction_pairs)
        object.__setattr__(self, "curvatures", curvature_pairs)
        object.__setattr__(self, "oblique_angles", oblique_pairs)
        object.__setattr__(self, "signed_progress_speeds", speed_pairs)
        object.__setattr__(self, "wheel_angles_start", wheel_angles_start)
        object.__setattr__(self, "wheel_factors_start", wheel_factors_start)
        object.__setattr__(self, "wheel_angles_end", wheel_angles_end)
        object.__setattr__(self, "wheel_factors_end", wheel_factors_end)
        object.__setattr__(self, "segments", tuple(self.segments))

    # ---- geometry -----------------------------------------------------

    @property
    def start_qf(self):
        return self.control.start_qf

    @property
    def end_qf(self):
        return self.control.end_qf

    @property
    def progress_interval(self):
        """Closed geometric progress interval ``[0, L]``; not a time span."""
        return (0.0, self.control.parameter_length)

    @property
    def progress_unit(self):
        """``m`` for straight/O edges, ``rad`` for arc/S edges."""
        return "rad" if self.primitive in _RADIAN_PRIMITIVES else "m"

    @property
    def position_derivative_unit(self):
        return ("m/rad" if self.progress_unit == "rad" else "m/m")

    @property
    def angle_derivative_unit(self):
        return ("rad/rad" if self.progress_unit == "rad" else "rad/m")

    @property
    def pivot_in_cargo_frame(self):
        if self.primitive != PRIMITIVE_A_ARC:
            return None
        return (self.control.pivot_x, self.control.pivot_y)

    def _progress(self, value, context):
        progress = _finite(value, context)
        length = self.control.parameter_length
        if progress < 0.0 or progress > length:
            raise UniformMotionError(
                "PROGRESS_OUT_OF_RANGE",
                "{} progress={:.17g} is outside the closed interval "
                "[0, {:.17g}]; no clamping or path change is performed"
                .format(context, progress, length))
        return progress

    def _delta_by_vehicle(self):
        deltas = {}
        start_map = _pairs_dict(self.start_headings)
        end_map = _pairs_dict(self.end_headings)
        for vehicle_id in self.model.vehicle_ids:
            deltas[vehicle_id] = end_map[vehicle_id] - start_map[vehicle_id]
        return deltas

    def _headings_at(self, progress):
        start_map = _pairs_dict(self.start_headings)
        if self.primitive in (PRIMITIVE_A_STRAIGHT,
                              PRIMITIVE_O_TRANSLATION):
            return dict(start_map)
        if self.primitive == PRIMITIVE_A_ARC:
            sign = 1 if self.control.delta_psi > 0.0 else -1
            return {
                vehicle_id: theta + sign * progress
                for vehicle_id, theta in start_map.items()}
        length = self.control.parameter_length
        deltas = self._delta_by_vehicle()
        return {
            vehicle_id: theta + deltas[vehicle_id] * progress / length
            for vehicle_id, theta in start_map.items()}

    def state_at(self, progress):
        """Return the geometric state at ``progress``; progress is not time."""
        value = self._progress(progress, "state_at")
        qf = self.control.qf_at(value)
        centers = self.model.centers(qf)
        headings = self._headings_at(value)
        return UniformMotionState(
            progress=value,
            qf=qf,
            centers=tuple(sorted(
                (vehicle_id, center[0], center[1])
                for vehicle_id, center in centers.items())),
            headings=tuple(sorted(headings.items())))

    def derivative_at(self, progress):
        """Return geometric first derivatives in the edge's published units."""
        value = self._progress(progress, "derivative_at")
        dqf = self.control.dqf_dlambda(value)
        vehicle_ids = self.model.vehicle_ids
        if self.primitive == PRIMITIVE_A_ARC:
            sign = 1 if self.control.delta_psi > 0.0 else -1
            psi = self.start_qf.psi + sign * value
            cosine, sine = math.cos(psi), math.sin(psi)
            pivot_x = self.control.pivot_x
            pivot_y = self.control.pivot_y
            dp = {}
            for offset in self.model.offsets:
                jx = -(offset.y - pivot_y)
                jy = offset.x - pivot_x
                dp[offset.vehicle_id] = (
                    sign * (cosine * jx - sine * jy),
                    sign * (sine * jx + cosine * jy))
            dtheta = {
                vehicle_id: float(sign) for vehicle_id in vehicle_ids}
        elif self.primitive == PRIMITIVE_S_SPIN:
            dp = {vehicle_id: (0.0, 0.0) for vehicle_id in vehicle_ids}
            length = self.control.parameter_length
            deltas = self._delta_by_vehicle()
            dtheta = {
                vehicle_id: deltas[vehicle_id] / length
                for vehicle_id in vehicle_ids}
        else:
            length = self.control.parameter_length
            dx = (
                self.control.end_qf.x - self.control.start_qf.x) / length
            dy = (
                self.control.end_qf.y - self.control.start_qf.y) / length
            dp = {vehicle_id: (dx, dy) for vehicle_id in vehicle_ids}
            dtheta = {vehicle_id: 0.0 for vehicle_id in vehicle_ids}
        return UniformMotionDerivative(
            progress=value,
            position_unit=self.position_derivative_unit,
            angle_unit=self.angle_derivative_unit,
            dqf=(float(dqf[0]), float(dqf[1]), float(dqf[2])),
            dp=tuple(sorted(
                (vehicle_id, x, y)
                for vehicle_id, (x, y) in dp.items())),
            dtheta=tuple(sorted(dtheta.items())))

    # ---- canonical wheel boundary ------------------------------------

    def wheel_configurations(self, at_end=False):
        """Canonical start (or end) four-wheel configuration per vehicle."""
        angles_pairs = (
            self.wheel_angles_end if at_end else self.wheel_angles_start)
        factors_pairs = (
            self.wheel_factors_end if at_end else self.wheel_factors_start)
        angle_map = _pairs_dict(angles_pairs)
        factor_map = _pairs_dict(factors_pairs)
        return tuple(
            VehicleWheelConfiguration(
                vehicle_id, tuple(angle_map[vehicle_id]),
                tuple(factor_map[vehicle_id]))
            for vehicle_id in self.model.vehicle_ids)

    def wheel_configuration(self, vehicle_id, at_end=False):
        vehicle_id = _integer(vehicle_id, "wheel_configuration vehicle_id")
        for configuration in self.wheel_configurations(at_end=at_end):
            if configuration.vehicle_id == vehicle_id:
                return configuration
        raise UniformMotionError(
            "INVALID_VEHICLE_ID_SET",
            "vehicle_id={} is not part of this edge".format(vehicle_id))

    def wheel_configuration_at(self, progress):
        """Validate progress and return the constant canonical boundary.

        Every FR01 primitive uses one constant control value, so the start
        and end wheel configurations are identical.  This query exists so a
        future layer cannot sample a wheel configuration outside the edge.
        """
        self._progress(progress, "wheel_configuration_at")
        return self.wheel_configurations(at_end=False)

    def angular_delta_from_zero_curvature_a(self):
        """Per-wheel angular difference from A zero-curvature, in radians.

        This is a geometric setup demand only; it is not a setup duration or
        a wheel speed.  It is what FR02 must use even for a zero-angle S
        vehicle, whose canonical wheel configuration is still S.
        """
        result = []
        for configuration in self.wheel_configurations(at_end=False):
            zero_angles, _ = _wheel_values(
                ACKERMANN, 0.0, self.limits, configuration.vehicle_id,
                "A zero-curvature reference")
            deltas = tuple(
                _wrap(actual - reference)
                for actual, reference in zip(
                    configuration.angles, zero_angles))
            result.append(VehicleZeroCurvatureDelta(
                configuration.vehicle_id, deltas,
                max(abs(value) for value in deltas)))
        return tuple(result)

    # ---- serialization ------------------------------------------------

    def as_dict(self):
        start_map = _pairs_dict(self.start_headings)
        end_map = _pairs_dict(self.end_headings)
        direction_map = _pairs_dict(self.directions)
        curvature_map = _pairs_dict(self.curvatures)
        oblique_map = _pairs_dict(self.oblique_angles)
        speed_map = _pairs_dict(self.signed_progress_speeds)
        wheel_start_angles = _pairs_dict(self.wheel_angles_start)
        wheel_start_factors = _pairs_dict(self.wheel_factors_start)
        wheel_end_angles = _pairs_dict(self.wheel_angles_end)
        wheel_end_factors = _pairs_dict(self.wheel_factors_end)
        segment_map = {
            segment.vehicle_id: segment for segment in self.segments}
        offset_map = {offset.vehicle_id: offset for offset in self.model.offsets}
        pivot = self.pivot_in_cargo_frame
        if self.primitive in (
                PRIMITIVE_A_STRAIGHT, PRIMITIVE_O_TRANSLATION):
            length = self.control.parameter_length
            unit_displacement = [
                (self.control.end_qf.x - self.control.start_qf.x) / length,
                (self.control.end_qf.y - self.control.start_qf.y) / length]
        else:
            unit_displacement = None
        vehicle_records = []
        for vehicle_id in self.model.vehicle_ids:
            segment = segment_map[vehicle_id]
            if self.primitive == PRIMITIVE_A_ARC:
                offset = offset_map[vehicle_id]
                radius = math.hypot(offset.x - pivot[0], offset.y - pivot[1])
            else:
                radius = None
            if self.mode == MODE_S:
                longitudinal = None
                lateral = None
            elif self.primitive == PRIMITIVE_O_TRANSLATION:
                phi = oblique_map[vehicle_id]
                longitudinal = direction_map[vehicle_id] * math.cos(phi)
                lateral = direction_map[vehicle_id] * math.sin(phi)
            else:
                # A straight/arc: body heading is parallel to the actual
                # center velocity, so the body-frame motion is longitudinal.
                longitudinal = float(direction_map[vehicle_id])
                lateral = 0.0
            vehicle_records.append({
                "vehicle_id": vehicle_id,
                "direction": direction_map[vehicle_id],
                "curvature": curvature_map.get(vehicle_id),
                "oblique_angle": oblique_map.get(vehicle_id),
                "signed_progress_speed": speed_map[vehicle_id],
                "radius_from_cargo_pivot": radius,
                "body_longitudinal_projection": longitudinal,
                "body_lateral_projection": lateral,
                "world_unit_displacement": (
                    None if unit_displacement is None
                    else list(unit_displacement)),
                "theta_start": start_map[vehicle_id],
                "theta_end": end_map[vehicle_id],
                "travel_distance": segment.travel_distance,
                "wheel_angles_start":
                    list(wheel_start_angles[vehicle_id]),
                "wheel_factors_start":
                    list(wheel_start_factors[vehicle_id]),
                "wheel_angles_end": list(wheel_end_angles[vehicle_id]),
                "wheel_factors_end": list(wheel_end_factors[vehicle_id]),
            })
        return {
            "mode": self.mode,
            "primitive": self.primitive,
            "progress_interval": list(self.progress_interval),
            "progress_unit": self.progress_unit,
            "position_derivative_unit": self.position_derivative_unit,
            "angle_derivative_unit": self.angle_derivative_unit,
            "real_time_domain": False,
            "time_seconds": None,
            "start_qf": [self.start_qf.x, self.start_qf.y,
                         self.start_qf.psi],
            "end_qf": [self.end_qf.x, self.end_qf.y, self.end_qf.psi],
            "pivot_in_cargo_frame": (
                None if self.pivot_in_cargo_frame is None
                else list(self.pivot_in_cargo_frame)),
            "allow_reverse": self.allow_reverse,
            "limits": {
                "wheelbase": self.limits.wheelbase,
                "track": self.limits.track,
                "r_min_ack": self.limits.r_min_ack,
                "steer_angle_max": self.limits.steer_angle_max,
            },
            "vehicles": vehicle_records,
            "wheel_delta_from_zero_curvature_a": [
                item.as_dict()
                for item in self.angular_delta_from_zero_curvature_a()],
            "segments": [{
                "vehicle_id": segment.vehicle_id,
                "start_station": segment.start_station,
                "end_station": segment.end_station,
                "legacy_mode": segment.mode,
                "direction": segment.direction,
                "theta_start": segment.theta_start,
                "theta_end": segment.theta_end,
                "control": segment.control,
                "travel_distance": segment.travel_distance,
                "start_center": [segment.start_center_x,
                                 segment.start_center_y],
                "end_center": [segment.end_center_x,
                               segment.end_center_y],
                "pivot": [segment.pivot_x, segment.pivot_y],
            } for segment in self.segments],
        }


# ---------------------------------------------------------------------------
# Factories
# ---------------------------------------------------------------------------

def _factory_inputs(model, start_qf, headings, limits):
    if not isinstance(model, FormationModel):
        raise UniformMotionError(
            "INVALID_MODEL", "model must be a FormationModel")
    if not isinstance(start_qf, FormationPose):
        raise UniformMotionError(
            "INVALID_START_POSE", "start_qf must be a FormationPose")
    if limits is None:
        limits = MotionLimits()
    if not isinstance(limits, MotionLimits):
        raise UniformMotionError(
            "INVALID_LIMITS",
            "limits must be a MotionLimits or None")
    start_pairs = _heading_pairs(
        headings, model.vehicle_ids, "start_headings")
    return start_pairs, limits


def _assemble(mode, primitive, model, control, start_pairs, end_pairs,
              direction_pairs, oblique_pairs, limits, allow_reverse):
    derived = _derive_fields(
        mode, primitive, model, control, start_pairs, end_pairs,
        direction_pairs, oblique_pairs, limits, allow_reverse)
    segments = _build_segments(
        mode, primitive, model, control, start_pairs, direction_pairs,
        derived)
    return UniformMotionEdge(
        mode=mode,
        primitive=primitive,
        model=model,
        control=control,
        start_headings=start_pairs,
        end_headings=derived.end_headings,
        directions=direction_pairs,
        curvatures=derived.curvatures,
        oblique_angles=derived.oblique_angles,
        signed_progress_speeds=derived.signed_progress_speeds,
        wheel_angles_start=derived.wheel_angles_start,
        wheel_factors_start=derived.wheel_factors_start,
        wheel_angles_end=derived.wheel_angles_end,
        wheel_factors_end=derived.wheel_factors_end,
        segments=segments,
        limits=limits,
        allow_reverse=allow_reverse)


def a_straight_edge(model, start_qf, headings, displacement, directions,
                    limits=None, allow_reverse=False):
    """Build an A straight edge with an explicit per-vehicle gear branch.

    ``displacement`` is a world-frame vector in metres; the progress length
    is its Euclidean norm.  Every body heading must already be parallel or
    anti-parallel to the displacement.  A mismatching heading is rejected as
    ``A_HEADING_REQUIRES_S_ALIGNMENT``; this factory never rotates a body.
    """
    start_pairs, limits = _factory_inputs(
        model, start_qf, headings, limits)
    dx = _finite(displacement[0], "a_straight displacement.x")
    dy = _finite(displacement[1], "a_straight displacement.y")
    length = math.hypot(dx, dy)
    if length <= 0.0:
        raise UniformMotionError(
            "ZERO_TRANSLATION",
            "A straight needs a non-zero displacement")
    direction_pairs = _direction_pairs(
        directions, model.vehicle_ids, "a_straight directions")
    end_qf = FormationPose(
        start_qf.x + dx, start_qf.y + dy, start_qf.psi)
    control = FormationPathControl(
        TRANSLATION, length, start_qf, end_qf, 0.0, 0.0)
    return _assemble(
        MODE_A, PRIMITIVE_A_STRAIGHT, model, control, start_pairs,
        start_pairs, direction_pairs, (), limits, allow_reverse)


def a_arc_edge(model, start_qf, headings, delta_psi, pivot, directions,
               limits=None, allow_reverse=False):
    """Build an A fixed-cargo-pivot arc edge.

    ``pivot`` is ``(x, y)`` in the cargo frame, not the world frame.  The
    progress length is ``|delta_psi|`` in radians.  Body headings must already
    equal the required tangent or its opposite for the supplied direction
    branch; a pivot at a vehicle centre or a radius below
    ``limits.r_min_ack`` is rejected explicitly.
    """
    start_pairs, limits = _factory_inputs(
        model, start_qf, headings, limits)
    delta_psi = _finite(delta_psi, "a_arc delta_psi")
    if delta_psi == 0.0:
        raise UniformMotionError(
            "ZERO_ROTATION", "A arc needs a non-zero delta_psi")
    pivot_x = _finite(pivot[0], "a_arc pivot.x")
    pivot_y = _finite(pivot[1], "a_arc pivot.y")
    length = abs(delta_psi)
    direction_pairs = _direction_pairs(
        directions, model.vehicle_ids, "a_arc directions")
    cosine = math.cos(start_qf.psi)
    sine = math.sin(start_qf.psi)
    pivot_world_x = start_qf.x + cosine * pivot_x - sine * pivot_y
    pivot_world_y = start_qf.y + sine * pivot_x + cosine * pivot_y
    end_psi = start_qf.psi + delta_psi
    cosine_end = math.cos(end_psi)
    sine_end = math.sin(end_psi)
    end_qf = FormationPose(
        pivot_world_x - cosine_end * pivot_x + sine_end * pivot_y,
        pivot_world_y - sine_end * pivot_x - cosine_end * pivot_y,
        end_psi)
    control = FormationPathControl(
        ROTATION, length, start_qf, end_qf, pivot_x, pivot_y)
    start_map = _pairs_dict(start_pairs)
    end_pairs = tuple(sorted(
        (vehicle_id, start_map[vehicle_id] + delta_psi)
        for vehicle_id in model.vehicle_ids))
    return _assemble(
        MODE_A, PRIMITIVE_A_ARC, model, control, start_pairs, end_pairs,
        direction_pairs, (), limits, allow_reverse)


def o_translation_edge(model, start_qf, headings, displacement, directions,
                       oblique_angles, limits=None, allow_reverse=False):
    """Build an O fixed-heading whole-formation translation edge.

    ``oblique_angles`` and ``directions`` are explicit per-vehicle controls.
    The factory checks the full vector equation
    ``d/|d| = sigma_i e(theta_i + phi_i)``; it never infers the +/-90 degree
    branch from a longitudinal projection.  Cargo ``psi_F`` and all body
    headings remain fixed.
    """
    start_pairs, limits = _factory_inputs(
        model, start_qf, headings, limits)
    dx = _finite(displacement[0], "o_translation displacement.x")
    dy = _finite(displacement[1], "o_translation displacement.y")
    length = math.hypot(dx, dy)
    if length <= 0.0:
        raise UniformMotionError(
            "ZERO_TRANSLATION",
            "O translation needs a non-zero displacement")
    direction_pairs = _direction_pairs(
        directions, model.vehicle_ids, "o_translation directions")
    oblique_pairs = _numeric_pairs(
        oblique_angles, model.vehicle_ids, "o_translation oblique_angles")
    end_qf = FormationPose(
        start_qf.x + dx, start_qf.y + dy, start_qf.psi)
    control = FormationPathControl(
        TRANSLATION, length, start_qf, end_qf, 0.0, 0.0)
    return _assemble(
        MODE_O, PRIMITIVE_O_TRANSLATION, model, control, start_pairs,
        start_pairs, direction_pairs, oblique_pairs, limits, allow_reverse)


def s_spin_edge(model, start_qf, headings, delta_thetas, limits=None):
    """Build an all-vehicle S edge with explicit signed per-body increments.

    Every vehicle, including one with ``delta_theta_i == 0``, has the
    canonical S wheel configuration.  The progress length is
    ``max_i |delta_theta_i|`` in radians.  All-zero increments return the
    explicit ``ZERO_SPIN_NO_EDGE`` rejection; no positive length is
    fabricated.  Exactly ``+pi`` and ``-pi`` are stored with their supplied
    sign and are never silently swapped.
    """
    start_pairs, limits = _factory_inputs(
        model, start_qf, headings, limits)
    delta_pairs = _numeric_pairs(
        delta_thetas, model.vehicle_ids, "s_spin delta_thetas")
    delta_map = _pairs_dict(delta_pairs)
    length = max(abs(value) for _, value in delta_pairs)
    if length <= SPIN_ZERO_TOL_RAD:
        raise UniformMotionError(
            "ZERO_SPIN_NO_EDGE",
            "every delta_theta is zero; no S edge is generated and no "
            "positive L is fabricated")
    start_map = _pairs_dict(start_pairs)
    end_pairs = tuple(sorted(
        (vehicle_id, start_map[vehicle_id] + delta_map[vehicle_id])
        for vehicle_id in model.vehicle_ids))
    direction_pairs = tuple(sorted(
        (vehicle_id,
         -1 if delta_map[vehicle_id] < -SPIN_ZERO_TOL_RAD else 1)
        for vehicle_id in model.vehicle_ids))
    control = FormationPathControl(
        BODY_SPIN, length, start_qf, start_qf, 0.0, 0.0)
    return _assemble(
        MODE_S, PRIMITIVE_S_SPIN, model, control, start_pairs, end_pairs,
        direction_pairs, (), limits, False)


# ---------------------------------------------------------------------------
# Independent geometric audit
# ---------------------------------------------------------------------------

def _independent_wheel_values(legacy_mode, control, limits):
    """Four wheel angles/factors from wheel positions, without the helper."""
    angles = []
    factors = []
    for x in (limits.wheelbase / 2.0, -limits.wheelbase / 2.0):
        for y in (limits.track / 2.0, -limits.track / 2.0):
            if legacy_mode == ACKERMANN:
                vx, vy = 1.0 - control * y, control * x
            elif legacy_mode == OBLIQUE:
                vx = math.cos(control)
                vy = math.sin(control)
            elif legacy_mode == SPIN:
                vx, vy = -y, x
            else:
                raise UniformMotionError(
                    "INVALID_MODE",
                    "unknown legacy mode {}".format(legacy_mode))
            angle = math.atan2(vy, vx)
            if legacy_mode == SPIN:
                if angle > math.pi / 2.0:
                    angle -= math.pi
                if angle < -math.pi / 2.0:
                    angle += math.pi
            angles.append(angle)
            factors.append(math.hypot(vx, vy))
    return tuple(angles), tuple(factors)


def _independent_qf(edge, progress):
    if edge.primitive in (PRIMITIVE_A_STRAIGHT,
                          PRIMITIVE_O_TRANSLATION):
        length = edge.control.parameter_length
        ux = (edge.end_qf.x - edge.start_qf.x) / length
        uy = (edge.end_qf.y - edge.start_qf.y) / length
        return FormationPose(
            edge.start_qf.x + progress * ux,
            edge.start_qf.y + progress * uy,
            edge.start_qf.psi)
    if edge.primitive == PRIMITIVE_A_ARC:
        sign = 1 if edge.control.delta_psi > 0.0 else -1
        pivot_x = edge.control.pivot_x
        pivot_y = edge.control.pivot_y
        cosine = math.cos(edge.start_qf.psi)
        sine = math.sin(edge.start_qf.psi)
        pivot_world_x = (
            edge.start_qf.x + cosine * pivot_x - sine * pivot_y)
        pivot_world_y = (
            edge.start_qf.y + sine * pivot_x + cosine * pivot_y)
        psi = edge.start_qf.psi + sign * progress
        cosine = math.cos(psi)
        sine = math.sin(psi)
        return FormationPose(
            pivot_world_x - cosine * pivot_x + sine * pivot_y,
            pivot_world_y - sine * pivot_x - cosine * pivot_y,
            psi)
    return FormationPose(
        edge.start_qf.x, edge.start_qf.y, edge.start_qf.psi)


def _independent_headings(edge, progress):
    start_map = _pairs_dict(edge.start_headings)
    if edge.primitive in (PRIMITIVE_A_STRAIGHT,
                          PRIMITIVE_O_TRANSLATION):
        return dict(start_map)
    if edge.primitive == PRIMITIVE_A_ARC:
        sign = 1 if edge.control.delta_psi > 0.0 else -1
        return {
            vehicle_id: theta + sign * progress
            for vehicle_id, theta in start_map.items()}
    length = edge.control.parameter_length
    end_map = _pairs_dict(edge.end_headings)
    return {
        vehicle_id: start_map[vehicle_id] + (
            end_map[vehicle_id] - start_map[vehicle_id]) * progress / length
        for vehicle_id in start_map}


def _independent_derivative(edge, progress):
    vehicle_ids = edge.model.vehicle_ids
    if edge.primitive in (PRIMITIVE_A_STRAIGHT,
                          PRIMITIVE_O_TRANSLATION):
        length = edge.control.parameter_length
        ux = (edge.end_qf.x - edge.start_qf.x) / length
        uy = (edge.end_qf.y - edge.start_qf.y) / length
        return ((ux, uy, 0.0),
                {vehicle_id: (ux, uy) for vehicle_id in vehicle_ids},
                {vehicle_id: 0.0 for vehicle_id in vehicle_ids})
    if edge.primitive == PRIMITIVE_A_ARC:
        sign = 1 if edge.control.delta_psi > 0.0 else -1
        pivot_x = edge.control.pivot_x
        pivot_y = edge.control.pivot_y
        psi = edge.start_qf.psi + sign * progress
        cosine, sine = math.cos(psi), math.sin(psi)
        jx = -pivot_y
        jy = pivot_x
        dqf = (
            -sign * (cosine * jx - sine * jy),
            -sign * (sine * jx + cosine * jy),
            float(sign))
        dp = {}
        for offset in edge.model.offsets:
            vx = offset.x - pivot_x
            vy = offset.y - pivot_y
            jx, jy = -vy, vx
            dp[offset.vehicle_id] = (
                sign * (cosine * jx - sine * jy),
                sign * (sine * jx + cosine * jy))
        return dqf, dp, {
            vehicle_id: float(sign) for vehicle_id in vehicle_ids}
    length = edge.control.parameter_length
    start_map = _pairs_dict(edge.start_headings)
    end_map = _pairs_dict(edge.end_headings)
    return ((0.0, 0.0, 0.0),
            {vehicle_id: (0.0, 0.0) for vehicle_id in vehicle_ids},
            {vehicle_id:
             (end_map[vehicle_id] - start_map[vehicle_id]) / length
             for vehicle_id in vehicle_ids})


def validate_uniform_edge(edge, sample_fractions=(0.0, 0.137, 0.413,
                                                  0.769, 1.0)):
    """Independently audit one edge's geometry, derivatives and wheels.

    The audit recomputes ``q_F``, headings, centres, derivatives and wheel
    configurations from the primitive definition and the fixed offsets.  It
    does not trust the edge's stored derived fields.  Any mismatch raises
    ``EDGE_VALIDATION_FAILED``; the returned metrics are geometry-only and
    carry no timing or collision certification.
    """
    if not isinstance(edge, UniformMotionEdge):
        raise UniformMotionError(
            "INVALID_EDGE", "expected a UniformMotionEdge")
    if not sample_fractions:
        raise UniformMotionError(
            "INVALID_SAMPLES", "sample_fractions must be non-empty")
    length = edge.control.parameter_length
    reference = edge.model.pairwise_reference_distances()
    vehicle_ids = edge.model.vehicle_ids
    max_qf_error = 0.0
    max_heading_error = 0.0
    max_rigidity = 0.0
    max_mode_residual = 0.0
    max_wheel_angle = 0.0
    problems = []

    for fraction in sample_fractions:
        fraction = _finite(fraction, "sample fraction")
        if fraction < 0.0 or fraction > 1.0:
            raise UniformMotionError(
                "INVALID_SAMPLES",
                "sample fraction {} is outside [0, 1]".format(fraction))
        progress = length * fraction
        state = edge.state_at(progress)
        expected_qf = _independent_qf(edge, progress)
        qf_error = max(
            abs(state.qf.x - expected_qf.x),
            abs(state.qf.y - expected_qf.y),
            abs(_wrap(state.qf.psi - expected_qf.psi)))
        max_qf_error = max(max_qf_error, qf_error)
        expected_headings = _independent_headings(edge, progress)
        actual_headings = _pairs_dict(state.headings)
        for vehicle_id in vehicle_ids:
            max_heading_error = max(
                max_heading_error,
                _angle_error(actual_headings[vehicle_id],
                             expected_headings[vehicle_id]))
        centre_map = {
            vehicle_id: (x, y) for vehicle_id, x, y in state.centers}
        for (first, second), distance in reference.items():
            actual = math.hypot(
                centre_map[second][0] - centre_map[first][0],
                centre_map[second][1] - centre_map[first][1])
            max_rigidity = max(max_rigidity, abs(actual - distance))
        derivative = edge.derivative_at(progress)
        dp_map = {
            vehicle_id: (x, y) for vehicle_id, x, y in derivative.dp}
        dtheta_map = _pairs_dict(derivative.dtheta)
        direction_map = _pairs_dict(edge.directions)
        oblique_map = _pairs_dict(edge.oblique_angles)
        start_map = _pairs_dict(edge.start_headings)
        if edge.primitive == PRIMITIVE_A_STRAIGHT:
            for vehicle_id in vehicle_ids:
                sigma = direction_map[vehicle_id]
                theta = start_map[vehicle_id]
                residual = max(
                    abs(dp_map[vehicle_id][0] - sigma * math.cos(theta)),
                    abs(dp_map[vehicle_id][1] - sigma * math.sin(theta)))
                max_mode_residual = max(max_mode_residual, residual)
        elif edge.primitive == PRIMITIVE_A_ARC:
            sign = 1 if edge.control.delta_psi > 0.0 else -1
            psi = edge.start_qf.psi + sign * progress
            cosine, sine = math.cos(psi), math.sin(psi)
            pivot_x = edge.control.pivot_x
            pivot_y = edge.control.pivot_y
            for offset in edge.model.offsets:
                vehicle_id = offset.vehicle_id
                vx = offset.x - pivot_x
                vy = offset.y - pivot_y
                expected = (
                    sign * (cosine * (-vy) - sine * vx),
                    sign * (sine * (-vy) + cosine * vx))
                residual = max(
                    abs(dp_map[vehicle_id][0] - expected[0]),
                    abs(dp_map[vehicle_id][1] - expected[1]))
                max_mode_residual = max(max_mode_residual, residual)
        elif edge.primitive == PRIMITIVE_O_TRANSLATION:
            length_unit = edge.control.parameter_length
            ux = (edge.end_qf.x - edge.start_qf.x) / length_unit
            uy = (edge.end_qf.y - edge.start_qf.y) / length_unit
            for vehicle_id in vehicle_ids:
                sigma = direction_map[vehicle_id]
                phi = oblique_map[vehicle_id]
                residual = max(
                    abs(ux - sigma * math.cos(
                        start_map[vehicle_id] + phi)),
                    abs(uy - sigma * math.sin(
                        start_map[vehicle_id] + phi)))
                max_mode_residual = max(max_mode_residual, residual)
        else:
            for vehicle_id in vehicle_ids:
                residual = max(
                    abs(dp_map[vehicle_id][0]),
                    abs(dp_map[vehicle_id][1]))
                max_mode_residual = max(max_mode_residual, residual)
        for configuration in edge.wheel_configurations():
            max_wheel_angle = max(
                max_wheel_angle,
                max(abs(value) for value in configuration.angles))

    max_derivative_error = 0.0
    for fraction in (0.23, 0.57, 0.83):
        progress = length * fraction
        derivative = edge.derivative_at(progress)
        dqf, dp, dtheta = _independent_derivative(edge, progress)
        max_derivative_error = max(
            max_derivative_error,
            abs(derivative.dqf[0] - dqf[0]),
            abs(derivative.dqf[1] - dqf[1]),
            abs(derivative.dqf[2] - dqf[2]))
        actual_dp = {
            vehicle_id: (x, y) for vehicle_id, x, y in derivative.dp}
        actual_dtheta = _pairs_dict(derivative.dtheta)
        for vehicle_id in vehicle_ids:
            max_derivative_error = max(
                max_derivative_error,
                abs(actual_dp[vehicle_id][0] - dp[vehicle_id][0]),
                abs(actual_dp[vehicle_id][1] - dp[vehicle_id][1]),
                abs(actual_dtheta[vehicle_id] - dtheta[vehicle_id]))

    curvature_map = _pairs_dict(edge.curvatures)
    direction_map = _pairs_dict(edge.directions)
    offset_map = {
        offset.vehicle_id: (offset.x, offset.y)
        for offset in edge.model.offsets}
    if edge.primitive == PRIMITIVE_A_ARC:
        arc_sign = 1 if edge.control.delta_psi > 0.0 else -1
        arc_pivot = (edge.control.pivot_x, edge.control.pivot_y)
    else:
        arc_sign = 0
        arc_pivot = (0.0, 0.0)
    for configuration in edge.wheel_configurations():
        vehicle_id = configuration.vehicle_id
        if edge.primitive in (PRIMITIVE_A_STRAIGHT, PRIMITIVE_A_ARC):
            if edge.primitive == PRIMITIVE_A_STRAIGHT:
                expected_curvature = 0.0
            else:
                radius = math.hypot(
                    offset_map[vehicle_id][0] - arc_pivot[0],
                    offset_map[vehicle_id][1] - arc_pivot[1])
                if radius <= PIVOT_CENTER_TOL_M:
                    problems.append(
                        "pivot coincides with vehicle_id={} centre".format(
                            vehicle_id))
                    continue
                expected_curvature = (
                    arc_sign / (direction_map[vehicle_id] * radius))
            if abs(curvature_map[vehicle_id]
                   - expected_curvature) > 1e-9:
                problems.append(
                    "curvature vehicle_id={} actual={:.17g} "
                    "expected={:.17g}".format(
                        vehicle_id, curvature_map[vehicle_id],
                        expected_curvature))
            control_value = expected_curvature
            legacy_mode = ACKERMANN
        elif edge.primitive == PRIMITIVE_O_TRANSLATION:
            control_value = _pairs_dict(edge.oblique_angles)[vehicle_id]
            legacy_mode = OBLIQUE
        else:
            control_value = 0.0
            legacy_mode = SPIN
        expected_angles, expected_factors = _independent_wheel_values(
            legacy_mode, control_value, edge.limits)
        for actual, expected in zip(
                configuration.angles, expected_angles):
            if abs(actual - expected) > 1e-9:
                problems.append(
                    "wheel_angles vehicle_id={} actual={:.17g} "
                    "expected={:.17g}".format(vehicle_id, actual, expected))
        for actual, expected in zip(
                configuration.factors, expected_factors):
            if abs(actual - expected) > 1e-9:
                problems.append(
                    "wheel_factors vehicle_id={} actual={:.17g} "
                    "expected={:.17g}".format(vehicle_id, actual, expected))

    if edge.primitive in (PRIMITIVE_A_STRAIGHT, PRIMITIVE_A_ARC,
                          PRIMITIVE_O_TRANSLATION):
        actual_end = _pairs_dict(edge.end_headings)
        expected_end = _independent_headings(edge, length)
        for vehicle_id in vehicle_ids:
            if _angle_error(actual_end[vehicle_id],
                            expected_end[vehicle_id]) > 1e-9:
                problems.append(
                    "end_heading vehicle_id={} actual={:.17g} "
                    "expected={:.17g}".format(
                        vehicle_id, actual_end[vehicle_id],
                        expected_end[vehicle_id]))
    elif edge.primitive == PRIMITIVE_S_SPIN:
        deltas = edge._delta_by_vehicle()
        if abs(max(abs(value) for value in deltas.values()) - length) > 1e-9:
            problems.append("spin length does not equal max|delta_theta|")
        for vehicle_id, delta in deltas.items():
            if abs(delta) > math.pi + 1e-12:
                problems.append(
                    "spin delta vehicle_id={} outside [-pi, pi]: {:.17g}"
                    .format(vehicle_id, delta))

    if max_qf_error > 1e-7:
        problems.append(
            "endpoint q_F error {:.3g} exceeds 1e-7 m".format(
                max_qf_error))
    if max_heading_error > 1e-7:
        problems.append(
            "endpoint heading error {:.3g} exceeds 1e-7 rad".format(
                max_heading_error))
    if max_rigidity > 1e-6:
        problems.append(
            "rigidity residual {:.3g} exceeds 1e-6 m".format(
                max_rigidity))
    if max_mode_residual > 1e-6:
        problems.append(
            "normalized mode equation residual {:.3g} exceeds 1e-6".format(
                max_mode_residual))
    if max_derivative_error > 1e-4:
        problems.append(
            "derivative error {:.3g} exceeds 1e-4".format(
                max_derivative_error))

    if problems:
        raise UniformMotionError(
            "EDGE_VALIDATION_FAILED", "; ".join(problems[:8]))

    return UniformEdgeValidation(
        primitive=edge.primitive,
        sample_count=len(sample_fractions) + 3,
        max_endpoint_qf_error_m=max_qf_error,
        max_endpoint_heading_error_rad=max_heading_error,
        max_rigidity_residual_m=max_rigidity,
        max_mode_equation_residual=max_mode_residual,
        max_derivative_error=max_derivative_error,
        max_wheel_angle_rad=max_wheel_angle)