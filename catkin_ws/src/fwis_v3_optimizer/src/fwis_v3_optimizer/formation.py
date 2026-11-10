"""Rigid multi-vehicle geometry without ROS, collision, or timing dependencies.

A formation pose ``q_F = (x, y, psi)`` places vehicle centers through
``p_i = p_F + R(psi) r_i``.  Body headings are independent inputs: neither
``centers`` nor ``fit_formation_pose`` chooses a vehicle's body heading.
"""
from collections.abc import Mapping as MappingABC
from dataclasses import dataclass
import math
from typing import Dict, Iterable, Mapping, Optional, Tuple, Union


_UINT32_MAX = 2**32 - 1


class FormationValidationError(ValueError):
    """Invalid formation input or a failed rigid-geometry contract."""


def _vehicle_id(value, context):
    if isinstance(value, bool) or not isinstance(value, int):
        raise FormationValidationError(
            "INVALID_VEHICLE_ID context={} vehicle_id={!r}: expected uint32".format(
                context, value))
    if not 0 <= value <= _UINT32_MAX:
        raise FormationValidationError(
            "INVALID_VEHICLE_ID context={} vehicle_id={}: outside uint32".format(
                context, value))
    return value


def _finite(value, context):
    if isinstance(value, (bool, str, bytes)):
        raise FormationValidationError(
            "NONFINITE_VALUE context={} value={!r}: expected a finite number".format(
                context, value))
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise FormationValidationError(
            "NONFINITE_VALUE context={} value={!r}: expected a finite number".format(
                context, value)) from exc
    if not math.isfinite(result):
        raise FormationValidationError(
            "NONFINITE_VALUE context={} value={!r}".format(context, value))
    return result


def _tolerance(value):
    result = _finite(value, "tolerance_m")
    if result < 0.0:
        raise FormationValidationError(
            "INVALID_TOLERANCE tolerance_m={}: expected >= 0".format(result))
    return result


@dataclass(frozen=True)
class VehicleOffset:
    """Vehicle-center offset in the formation frame, in metres."""

    vehicle_id: int
    x: float
    y: float

    def __post_init__(self):
        _vehicle_id(self.vehicle_id, "VehicleOffset")
        object.__setattr__(self, "x", _finite(
            self.x, "VehicleOffset.x vehicle_id={}".format(self.vehicle_id)))
        object.__setattr__(self, "y", _finite(
            self.y, "VehicleOffset.y vehicle_id={}".format(self.vehicle_id)))


@dataclass(frozen=True)
class FormationPose:
    """Formation reference pose; psi may be unwrapped and is not a body yaw."""

    x: float
    y: float
    psi: float

    def __post_init__(self):
        object.__setattr__(self, "x", _finite(self.x, "FormationPose.x"))
        object.__setattr__(self, "y", _finite(self.y, "FormationPose.y"))
        object.__setattr__(self, "psi", _finite(self.psi, "FormationPose.psi"))

    @classmethod
    def from_core(cls, pose):
        """Convert the existing ``core.Pose(x, y, th)`` without modifying it."""
        try:
            return cls(pose.x, pose.y, pose.th)
        except AttributeError as exc:
            raise FormationValidationError(
                "INVALID_CORE_POSE expected fields x/y/th") from exc


@dataclass(frozen=True)
class VehiclePose:
    """A vehicle center and its independently specified body heading."""

    vehicle_id: int
    x: float
    y: float
    theta: float

    def __post_init__(self):
        _vehicle_id(self.vehicle_id, "VehiclePose")
        object.__setattr__(self, "x", _finite(
            self.x, "VehiclePose.x vehicle_id={}".format(self.vehicle_id)))
        object.__setattr__(self, "y", _finite(
            self.y, "VehiclePose.y vehicle_id={}".format(self.vehicle_id)))
        object.__setattr__(self, "theta", _finite(
            self.theta, "VehiclePose.theta vehicle_id={}".format(self.vehicle_id)))


@dataclass(frozen=True)
class FormationValidation:
    """Successful validation metrics; invalid inputs raise instead of returning."""

    vehicle_count: int
    max_pairwise_error_m: float
    max_reconstruction_error_m: float
    tolerance_m: float


CenterInput = Union[Mapping[int, object], Iterable[object]]


class FormationModel:
    """Immutable, ID-ordered offsets and their exact rigid-center mapping."""

    def __init__(self, offsets: Iterable[VehicleOffset]):
        try:
            supplied = tuple(offsets)
        except TypeError as exc:
            raise FormationValidationError(
                "INVALID_OFFSETS expected an iterable of VehicleOffset") from exc
        if not supplied:
            raise FormationValidationError("EMPTY_FORMATION vehicle_count=0")
        for index, offset in enumerate(supplied):
            if not isinstance(offset, VehicleOffset):
                raise FormationValidationError(
                    "INVALID_OFFSET index={} value={!r}: expected VehicleOffset".format(
                        index, offset))
        ordered = tuple(sorted(supplied, key=lambda offset: offset.vehicle_id))
        for previous, current in zip(ordered, ordered[1:]):
            if previous.vehicle_id == current.vehicle_id:
                raise FormationValidationError(
                    "DUPLICATE_VEHICLE_ID context=offsets vehicle_id={}".format(
                        current.vehicle_id))
        for index, first in enumerate(ordered):
            for second in ordered[index + 1:]:
                if first.x == second.x and first.y == second.y:
                    raise FormationValidationError(
                        "COINCIDENT_OFFSETS pair_id=({}, {})".format(
                            first.vehicle_id, second.vehicle_id))
        self._offsets = ordered
        self._reference_distances = {}
        for first, second in self._pairs():
            distance = math.hypot(second.x - first.x, second.y - first.y)
            if not math.isfinite(distance):
                raise FormationValidationError(
                    "NONFINITE_REFERENCE_DISTANCE pair_id=({}, {})".format(
                        first.vehicle_id, second.vehicle_id))
            self._reference_distances[(first.vehicle_id, second.vehicle_id)] = distance

    @property
    def offsets(self) -> Tuple[VehicleOffset, ...]:
        return self._offsets

    @property
    def vehicle_ids(self) -> Tuple[int, ...]:
        return tuple(offset.vehicle_id for offset in self._offsets)

    def _pairs(self):
        for index, first in enumerate(self._offsets):
            for second in self._offsets[index + 1:]:
                yield first, second

    @staticmethod
    def _require_pose(qf):
        if not isinstance(qf, FormationPose):
            raise FormationValidationError(
                "INVALID_FORMATION_POSE expected FormationPose, got {!r}".format(qf))

    def _require_exact_ids(self, actual_ids, context):
        expected = set(self.vehicle_ids)
        actual = set(actual_ids)
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        if missing or extra:
            raise FormationValidationError(
                "VEHICLE_ID_SET_MISMATCH context={} missing={} extra={}".format(
                    context, missing, extra))

    def _normalise_centers(self, given_centers: CenterInput) -> Dict[int, Tuple[float, float]]:
        if isinstance(given_centers, MappingABC):
            items = list(given_centers.items())
        else:
            if isinstance(given_centers, (str, bytes)):
                raise FormationValidationError(
                    "INVALID_CENTERS expected mapping or vehicle-pose iterable")
            try:
                poses = list(given_centers)
            except TypeError as exc:
                raise FormationValidationError(
                    "INVALID_CENTERS expected mapping or vehicle-pose iterable") from exc
            items = []
            for index, pose in enumerate(poses):
                if not hasattr(pose, "vehicle_id"):
                    raise FormationValidationError(
                        "INVALID_CENTER index={}: vehicle_id is required".format(index))
                items.append((pose.vehicle_id, pose))
        result = {}
        for vehicle_id, value in items:
            checked_id = _vehicle_id(vehicle_id, "given_centers")
            if checked_id in result:
                raise FormationValidationError(
                    "DUPLICATE_VEHICLE_ID context=given_centers vehicle_id={}".format(
                        checked_id))
            if hasattr(value, "x") and hasattr(value, "y"):
                raw_x, raw_y = value.x, value.y
            else:
                try:
                    if len(value) != 2:
                        raise FormationValidationError(
                            "INVALID_CENTER vehicle_id={}: expected two coordinates".format(
                                checked_id))
                    raw_x, raw_y = value
                except (TypeError, ValueError) as exc:
                    raise FormationValidationError(
                        "INVALID_CENTER vehicle_id={}: expected x/y or a coordinate pair".format(
                            checked_id)) from exc
            result[checked_id] = (
                _finite(raw_x, "given_centers.x vehicle_id={}".format(checked_id)),
                _finite(raw_y, "given_centers.y vehicle_id={}".format(checked_id)),
            )
        self._require_exact_ids(result, "given_centers")
        return result

    def centers(self, qf: FormationPose) -> Dict[int, Tuple[float, float]]:
        """Return vehicle centers only; no body heading is inferred here."""
        self._require_pose(qf)
        cosine, sine = math.cos(qf.psi), math.sin(qf.psi)
        result = {}
        for offset in self._offsets:
            x = qf.x + cosine * offset.x - sine * offset.y
            y = qf.y + sine * offset.x + cosine * offset.y
            if not math.isfinite(x) or not math.isfinite(y):
                raise FormationValidationError(
                    "NONFINITE_CENTER context=centers vehicle_id={}".format(
                        offset.vehicle_id))
            result[offset.vehicle_id] = (x, y)
        return result

    def vehicle_poses(self, qf: FormationPose,
                      body_headings: Mapping[int, float]) -> Tuple[VehiclePose, ...]:
        """Combine rigid centers with an exact-ID mapping of independent yaws."""
        if not isinstance(body_headings, MappingABC):
            raise FormationValidationError(
                "INVALID_BODY_HEADINGS expected a vehicle_id -> theta mapping")
        checked_headings = {}
        for vehicle_id, theta in body_headings.items():
            checked_id = _vehicle_id(vehicle_id, "body_headings")
            checked_headings[checked_id] = _finite(
                theta, "body_headings vehicle_id={}".format(checked_id))
        self._require_exact_ids(checked_headings, "body_headings")
        return tuple(
            VehiclePose(offset.vehicle_id, center[0], center[1],
                        checked_headings[offset.vehicle_id])
            for offset, center in zip(self._offsets, self.centers(qf).values())
        )

    def pairwise_reference_distances(self) -> Dict[Tuple[int, int], float]:
        """Return a copy so callers cannot mutate the reference geometry."""
        return dict(self._reference_distances)

    def _pairwise_errors(self, centers):
        maximum = 0.0
        worst_pair = None
        for first, second in self._pairs():
            pair = (first.vehicle_id, second.vehicle_id)
            actual = math.hypot(
                centers[second.vehicle_id][0] - centers[first.vehicle_id][0],
                centers[second.vehicle_id][1] - centers[first.vehicle_id][1])
            error = abs(actual - self._reference_distances[pair])
            if error > maximum or worst_pair is None:
                maximum, worst_pair = error, pair
        return maximum, worst_pair

    def rigidity_residual(self, given_centers: CenterInput) -> float:
        """Maximum absolute pair-distance error after strict ID validation."""
        centers = self._normalise_centers(given_centers)
        return self._pairwise_errors(centers)[0]

    def validate_vehicle_centers(self, qf: FormationPose,
                                 given_centers: CenterInput,
                                 tolerance_m: float = 1e-6) -> FormationValidation:
        """Check exact IDs, pair distances, and reconstruction from this q_F.

        Pair distances alone cannot detect reflection or swapped labels.  The
        per-ID reconstruction check against q_F is therefore mandatory.
        """
        self._require_pose(qf)
        tolerance = _tolerance(tolerance_m)
        centers = self._normalise_centers(given_centers)
        pairwise_error, pair = self._pairwise_errors(centers)
        if pairwise_error > tolerance:
            raise FormationValidationError(
                "FORMATION_RIGIDITY_VIOLATION pair_id={} error_m={:.17g} "
                "tolerance_m={:.17g}".format(pair, pairwise_error, tolerance))
        expected = self.centers(qf)
        reconstruction_error = 0.0
        worst_id = self.vehicle_ids[0]
        for vehicle_id in self.vehicle_ids:
            error = math.hypot(
                centers[vehicle_id][0] - expected[vehicle_id][0],
                centers[vehicle_id][1] - expected[vehicle_id][1])
            if error > reconstruction_error:
                reconstruction_error, worst_id = error, vehicle_id
        if reconstruction_error > tolerance:
            raise FormationValidationError(
                "FORMATION_CENTER_MISMATCH vehicle_id={} error_m={:.17g} "
                "tolerance_m={:.17g}".format(
                    worst_id, reconstruction_error, tolerance))
        return FormationValidation(
            len(self._offsets), pairwise_error, reconstruction_error, tolerance)

    def validate_centers(self, qf: FormationPose, given_centers: CenterInput,
                         tol: float = 1e-6) -> FormationValidation:
        """Compatibility name from the design skeleton; same strict checks."""
        return self.validate_vehicle_centers(qf, given_centers, tol)

    def fit_formation_pose(self, given_centers: CenterInput,
                           psi: Optional[float] = None,
                           tolerance_m: float = 1e-6) -> FormationPose:
        """Fit translation and a proper rotation; reflection is never allowed.

        With no explicit ``psi``, a least-squares Procrustes rotation is used.
        If orientation is unobservable (including N=1), callers must supply
        ``psi``.  The returned pose is accepted only after the same pair and
        per-ID reconstruction checks used by ``validate_vehicle_centers``.
        """
        tolerance = _tolerance(tolerance_m)
        centers = self._normalise_centers(given_centers)
        ids = self.vehicle_ids
        reference_x = math.fsum(offset.x for offset in self._offsets) / len(ids)
        reference_y = math.fsum(offset.y for offset in self._offsets) / len(ids)
        center_x = math.fsum(centers[vehicle_id][0] for vehicle_id in ids) / len(ids)
        center_y = math.fsum(centers[vehicle_id][1] for vehicle_id in ids) / len(ids)
        cross_terms = []
        dot_terms = []
        orientation_scale_terms = []
        for offset in self._offsets:
            ref_x, ref_y = offset.x - reference_x, offset.y - reference_y
            given_x, given_y = (centers[offset.vehicle_id][0] - center_x,
                                centers[offset.vehicle_id][1] - center_y)
            dot_terms.append(ref_x * given_x + ref_y * given_y)
            cross_terms.append(ref_x * given_y - ref_y * given_x)
            orientation_scale_terms.append(math.hypot(ref_x, ref_y) *
                                           math.hypot(given_x, given_y))
        dot = math.fsum(dot_terms)
        cross = math.fsum(cross_terms)
        orientation_scale = math.fsum(orientation_scale_terms)
        degenerate = (orientation_scale == 0.0 or
                      math.hypot(dot, cross) <= 1e-12 * orientation_scale)
        if psi is None:
            if degenerate:
                raise FormationValidationError(
                    "FORMATION_ORIENTATION_UNOBSERVABLE explicit psi is required")
            fitted_psi = math.atan2(cross, dot)
        else:
            fitted_psi = _finite(psi, "fit_formation_pose.psi")
        cosine, sine = math.cos(fitted_psi), math.sin(fitted_psi)
        qf = FormationPose(
            center_x - cosine * reference_x + sine * reference_y,
            center_y - sine * reference_x - cosine * reference_y,
            fitted_psi,
        )
        self.validate_vehicle_centers(qf, centers, tolerance)
        return qf
