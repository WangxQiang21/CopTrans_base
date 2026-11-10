"""Multi-vehicle OBB and conservative joint-state swept checks.

This module composes the S1 rigid-center model with the existing exact
``FootprintDistance`` grid query and an exact pair-of-OBB separating-axis test.
It does not search for a path, assign vehicle modes, retime a trajectory, or
publish a feasibility result.
"""
from collections.abc import Mapping as MappingABC
from dataclasses import dataclass
import math
from numbers import Real
from types import MappingProxyType
from typing import Mapping, Optional

from .core import Grid, Pose
from .formation import (FormationModel, FormationPose,
                        FormationValidationError)
from .quality import FootprintDistance


UINT32_MAX = 2**32 - 1


def _finite_number(value, context, positive=False, nonnegative=False):
    if isinstance(value, (bool, str, bytes)):
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
            "NONPOSITIVE_VALUE context={} value={!r}".format(context, value))
    if nonnegative and result < 0.0:
        raise FormationValidationError(
            "NEGATIVE_VALUE context={} value={!r}".format(context, value))
    return result


def _checked_vehicle_id(value, context):
    if isinstance(value, bool) or not isinstance(value, int):
        raise FormationValidationError(
            "INVALID_VEHICLE_ID context={} vehicle_id={!r}".format(context, value))
    if not 0 <= value <= UINT32_MAX:
        raise FormationValidationError(
            "INVALID_VEHICLE_ID context={} vehicle_id={} outside uint32".format(
                context, value))
    return value


@dataclass(frozen=True)
class VehicleShape:
    """Physical vehicle dimensions; safety margin is owned by the checker."""

    length_m: float
    width_m: float

    def __post_init__(self):
        object.__setattr__(self, "length_m", _finite_number(
            self.length_m, "VehicleShape.length_m", positive=True))
        object.__setattr__(self, "width_m", _finite_number(
            self.width_m, "VehicleShape.width_m", positive=True))


@dataclass(frozen=True)
class CollisionReport:
    """Structured pass/fail; every collision names vehicle and sweep segment."""

    success: bool
    reason: Optional[str] = None
    vehicle_id: Optional[int] = None
    other_vehicle_id: Optional[int] = None
    segment_index: int = 0
    sample_index: int = 0
    detail: Optional[str] = None
    qf: Optional[FormationPose] = None


@dataclass(frozen=True)
class FormationState:
    """One synchronized full-formation sample ordered by ``station``.

    ``psi`` and body headings are expected to preserve continuous unwrapped
    values.  The checker interpolates raw angle differences; it does not collapse
    a multi-turn motion with ``wrap``.
    """

    station: float
    qf: FormationPose
    body_headings: Mapping[int, float]

    def __post_init__(self):
        if not isinstance(self.qf, FormationPose):
            raise FormationValidationError(
                "INVALID_FORMATION_STATE expected FormationPose qf")
        if not isinstance(self.body_headings, MappingABC):
            raise FormationValidationError(
                "INVALID_FORMATION_STATE expected body_headings mapping")
        object.__setattr__(self, "body_headings",
                           MappingProxyType(dict(self.body_headings)))


@dataclass(frozen=True)
class SweepPolicy:
    """Sampling step and per-segment guard for conservative swept checks."""

    max_corner_step_m: float = 0.01
    max_samples_per_segment: int = 100000

    def __post_init__(self):
        object.__setattr__(self, "max_corner_step_m", _finite_number(
            self.max_corner_step_m, "SweepPolicy.max_corner_step_m", positive=True))
        value = self.max_samples_per_segment
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise FormationValidationError(
                "INVALID_SWEEP_POLICY max_samples_per_segment={!r}".format(value))

    @property
    def motion_padding_m(self):
        """Expansion at every sweep sample, equal to the discrete motion bound."""
        return self.max_corner_step_m


class _HeadingValidationError(FormationValidationError):
    def __init__(self, reason, detail, vehicle_id=None):
        self.reason = reason
        self.vehicle_id = vehicle_id
        super().__init__(detail)


class FormationCollisionChecker:
    """Exact static OBB checks plus conservative joint-state sweep sampling."""

    def __init__(self, grid, formation_model, vehicle_shapes, margin_m):
        if not isinstance(grid, Grid):
            raise FormationValidationError(
                "INVALID_GRID expected core.Grid")
        if not isinstance(formation_model, FormationModel):
            raise FormationValidationError(
                "INVALID_FORMATION_MODEL expected FormationModel")
        self.grid = grid
        self.model = formation_model
        self.margin_m = _finite_number(
            margin_m, "FormationCollisionChecker.margin_m", nonnegative=True)
        self._vehicle_ids = tuple(self.model.vehicle_ids)
        self._shapes = self._normalise_shapes(vehicle_shapes)
        self._base_half_extents = {
            vehicle_id: ((shape.length_m + self.margin_m) / 2.0,
                         (shape.width_m + self.margin_m) / 2.0)
            for vehicle_id, shape in self._shapes.items()
        }
        self._corner_radii = {
            vehicle_id: math.hypot(half_length, half_width)
            for vehicle_id, (half_length, half_width)
            in self._base_half_extents.items()
        }
        self._reference_radius = max(
            math.hypot(offset.x, offset.y) for offset in self.model.offsets)

        # The first distance object builds the obstacle boundary index once;
        # all per-shape queries reuse it, including sweep-padded variants.
        self._obstacle_index = FootprintDistance(self.grid)
        self._base_distances = {
            vehicle_id: FootprintDistance(
                self.grid,
                shape.length_m + self.margin_m,
                shape.width_m + self.margin_m,
                obstacle_index=self._obstacle_index)
            for vehicle_id, shape in self._shapes.items()
        }
        self._padding_distance_cache = {0.0: self._base_distances}
        self._stats = {
            "static_checks": 0,
            "swept_checks": 0,
            "segments_checked": 0,
            "states_checked": 0,
            "swept_samples_checked": 0,
            "map_queries": 0,
            "obb_pair_queries": 0,
        }

    def _normalise_shapes(self, vehicle_shapes):
        if not isinstance(vehicle_shapes, MappingABC):
            raise FormationValidationError(
                "INVALID_VEHICLE_SHAPES expected vehicle_id -> VehicleShape mapping")
        result = {}
        for vehicle_id, value in vehicle_shapes.items():
            checked_id = _checked_vehicle_id(vehicle_id, "vehicle_shapes")
            if checked_id in result:
                raise FormationValidationError(
                    "DUPLICATE_VEHICLE_ID context=vehicle_shapes vehicle_id={}".format(
                        checked_id))
            if isinstance(value, VehicleShape):
                shape = value
            elif isinstance(value, (tuple, list)) and len(value) == 2:
                shape = VehicleShape(value[0], value[1])
            elif hasattr(value, "length_m") and hasattr(value, "width_m"):
                shape = VehicleShape(value.length_m, value.width_m)
            elif hasattr(value, "length") and hasattr(value, "width"):
                shape = VehicleShape(value.length, value.width)
            else:
                raise FormationValidationError(
                    "INVALID_VEHICLE_SHAPE vehicle_id={} value={!r}".format(
                        checked_id, value))
            result[checked_id] = shape
        expected = set(self._vehicle_ids)
        actual = set(result)
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        if missing or extra:
            raise FormationValidationError(
                "VEHICLE_ID_SET_MISMATCH context=vehicle_shapes missing={} extra={}".format(
                    missing, extra))
        return {vehicle_id: result[vehicle_id] for vehicle_id in self._vehicle_ids}

    def _checked_headings(self, headings):
        if not isinstance(headings, MappingABC):
            raise _HeadingValidationError(
                "INVALID_BODY_HEADINGS",
                "expected vehicle_id -> theta mapping")
        checked = {}
        for vehicle_id, theta in headings.items():
            try:
                checked_id = _checked_vehicle_id(vehicle_id, "body_headings")
            except FormationValidationError as exc:
                raise _HeadingValidationError(
                    "INVALID_VEHICLE_ID", str(exc)) from exc
            if checked_id in checked:
                raise _HeadingValidationError(
                    "DUPLICATE_VEHICLE_ID",
                    "context=body_headings vehicle_id={}".format(checked_id),
                    checked_id)
            checked[checked_id] = theta
        expected = set(self._vehicle_ids)
        actual = set(checked)
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        if missing or extra:
            located = missing[0] if missing else extra[0]
            raise _HeadingValidationError(
                "VEHICLE_ID_SET_MISMATCH",
                "context=body_headings missing={} extra={}".format(missing, extra),
                located)
        for vehicle_id in self._vehicle_ids:
            try:
                checked[vehicle_id] = _finite_number(
                    checked[vehicle_id],
                    "body_headings.theta vehicle_id={}".format(vehicle_id))
            except FormationValidationError as exc:
                raise _HeadingValidationError(
                    "NONFINITE_VALUE", str(exc), vehicle_id) from exc
        return checked

    def _distance_objects(self, padding_m):
        padding = _finite_number(padding_m, "sweep_padding_m", nonnegative=True)
        if padding not in self._padding_distance_cache:
            self._padding_distance_cache[padding] = {
                vehicle_id: FootprintDistance(
                    self.grid,
                    shape.length_m + self.margin_m + 2.0 * padding,
                    shape.width_m + self.margin_m + 2.0 * padding,
                    obstacle_index=self._obstacle_index)
                for vehicle_id, shape in self._shapes.items()
            }
        return self._padding_distance_cache[padding]

    @staticmethod
    def _obb_overlap(first, first_half, second, second_half):
        """Exact 2-D OBB SAT; touching expanded bodies counts as collision."""
        c1, s1 = math.cos(first.theta), math.sin(first.theta)
        c2, s2 = math.cos(second.theta), math.sin(second.theta)
        axes = ((c1, s1), (-s1, c1), (c2, s2), (-s2, c2))
        dx, dy = second.x - first.x, second.y - first.y
        hl1, hw1 = first_half
        hl2, hw2 = second_half
        for ax, ay in axes:
            r1 = hl1 * abs(c1 * ax + s1 * ay) + hw1 * abs(-s1 * ax + c1 * ay)
            r2 = hl2 * abs(c2 * ax + s2 * ay) + hw2 * abs(-s2 * ax + c2 * ay)
            if abs(dx * ax + dy * ay) > r1 + r2:
                return False
        return True

    @staticmethod
    def _failure(reason, segment_index, sample_index, qf=None,
                 vehicle_id=None, other_vehicle_id=None, detail=None):
        return CollisionReport(
            False, reason, vehicle_id, other_vehicle_id,
            segment_index, sample_index, detail, qf)

    @staticmethod
    def _error_code(exc):
        text = str(exc)
        return text.split(None, 1)[0] if text else "INVALID_FORMATION_INPUT"

    def _check_poses(self, qf, vehicle_poses, padding_m, segment_index,
                     sample_index):
        distances = self._distance_objects(padding_m)
        for vehicle_pose in vehicle_poses:
            self._stats["map_queries"] += 1
            distance = distances[vehicle_pose.vehicle_id].distance(
                Pose(vehicle_pose.x, vehicle_pose.y, vehicle_pose.theta))
            # Existing Grid also rejects clear poses exactly on its boundary.
            # Strict positivity makes touching map/cell boundaries conservative.
            if not distance > 0.0:
                return self._failure(
                    "MAP_OBB_COLLISION", segment_index, sample_index, qf,
                    vehicle_id=vehicle_pose.vehicle_id)
        shape_half = {
            vehicle_id: (half_length + padding_m, half_width + padding_m)
            for vehicle_id, (half_length, half_width)
            in self._base_half_extents.items()
        }
        poses_by_id = {pose.vehicle_id: pose for pose in vehicle_poses}
        for index, first_id in enumerate(self._vehicle_ids):
            for second_id in self._vehicle_ids[index + 1:]:
                self._stats["obb_pair_queries"] += 1
                if self._obb_overlap(
                        poses_by_id[first_id], shape_half[first_id],
                        poses_by_id[second_id], shape_half[second_id]):
                    return self._failure(
                        "VEHICLE_OBB_COLLISION", segment_index, sample_index, qf,
                        vehicle_id=first_id, other_vehicle_id=second_id)
        return CollisionReport(True, segment_index=segment_index,
                               sample_index=sample_index, qf=qf)

    def _check_state(self, qf, headings, padding_m, segment_index=0,
                     sample_index=0):
        self._stats["states_checked"] += 1
        try:
            vehicle_poses = self.model.vehicle_poses(qf, headings)
        except FormationValidationError as exc:
            return self._failure(
                self._error_code(exc), segment_index, sample_index, qf,
                detail=str(exc))
        return self._check_poses(qf, vehicle_poses, padding_m,
                                 segment_index, sample_index)

    def static_check(self, qf, body_headings):
        """Check all map OBBs first, then every vehicle pair at one instant."""
        self._stats["static_checks"] += 1
        if not isinstance(qf, FormationPose):
            return self._failure(
                "INVALID_FORMATION_POSE", 0, 0,
                detail="expected FormationPose")
        try:
            headings = self._checked_headings(body_headings)
        except FormationValidationError as exc:
            reason = getattr(exc, "reason", self._error_code(exc))
            return self._failure(
                reason, 0, 0, qf,
                vehicle_id=getattr(exc, "vehicle_id", None),
                detail=str(exc))
        return self._check_state(qf, headings, 0.0)

    @staticmethod
    def _normalise_policy(policy):
        if isinstance(policy, SweepPolicy):
            return policy
        try:
            if isinstance(policy, Real) and not isinstance(policy, bool):
                return SweepPolicy(policy)
            if isinstance(policy, MappingABC):
                return SweepPolicy(**dict(policy))
            fields = {}
            for name in ("max_corner_step_m", "max_samples_per_segment"):
                if hasattr(policy, name):
                    fields[name] = getattr(policy, name)
            if fields:
                return SweepPolicy(**fields)
        except (FormationValidationError, TypeError, ValueError) as exc:
            raise FormationValidationError(
                "INVALID_SWEEP_POLICY {}".format(exc)) from exc
        raise FormationValidationError(
            "INVALID_SWEEP_POLICY expected SweepPolicy, number, or policy fields")

    @staticmethod
    def _state_parts(state):
        if isinstance(state, FormationState):
            return state.station, state.qf, state.body_headings
        if isinstance(state, MappingABC):
            try:
                return state["station"], state["qf"], state["body_headings"]
            except KeyError as exc:
                raise FormationValidationError(
                    "INVALID_FORMATION_STATE missing field={}".format(exc)) from exc
        try:
            return state.station, state.qf, state.body_headings
        except AttributeError as exc:
            raise FormationValidationError(
                "INVALID_FORMATION_STATE expected station/qf/body_headings") from exc

    def _normalise_states(self, states):
        try:
            supplied = list(states)
        except TypeError as exc:
            raise FormationValidationError(
                "INVALID_STATES expected an ordered iterable") from exc
        if not supplied:
            raise FormationValidationError("EMPTY_STATES")
        result = []
        previous_station = None
        for index, state in enumerate(supplied):
            segment_index = max(0, index - 1)
            try:
                station, qf, headings = self._state_parts(state)
                station = _finite_number(
                    station, "states[{}].station".format(index))
                if not isinstance(qf, FormationPose):
                    raise FormationValidationError(
                        "INVALID_FORMATION_POSE states[{}].qf".format(index))
                headings = self._checked_headings(headings)
            except FormationValidationError as exc:
                raise _StateNormalisationError(
                    getattr(exc, "reason", self._error_code(exc)),
                    segment_index, str(exc),
                    getattr(exc, "vehicle_id", None)) from exc
            if previous_station is not None and not station > previous_station:
                raise _StateNormalisationError(
                    "NONMONOTONIC_STATIONS", segment_index,
                    "states[{}].station={} must exceed {}".format(
                        index, station, previous_station))
            previous_station = station
            result.append((station, qf, headings))
        return result

    def _displacement_bound(self, first, second):
        first_station, first_qf, first_headings = first
        second_station, second_qf, second_headings = second
        translation = math.hypot(second_qf.x - first_qf.x,
                                 second_qf.y - first_qf.y)
        # Raw differences preserve an explicitly stored multi-turn rotation.
        formation_rotation = abs(second_qf.psi - first_qf.psi)
        body_rotation = max(
            self._corner_radii[vehicle_id] *
            abs(second_headings[vehicle_id] - first_headings[vehicle_id])
            for vehicle_id in self._vehicle_ids)
        return translation + self._reference_radius * formation_rotation + body_rotation

    def _interpolate(self, first, second, fraction):
        _, first_qf, first_headings = first
        _, second_qf, second_headings = second
        qf = FormationPose(
            first_qf.x + fraction * (second_qf.x - first_qf.x),
            first_qf.y + fraction * (second_qf.y - first_qf.y),
            first_qf.psi + fraction * (second_qf.psi - first_qf.psi))
        headings = {}
        for vehicle_id in self._vehicle_ids:
            headings[vehicle_id] = (
                first_headings[vehicle_id] +
                fraction * (second_headings[vehicle_id] -
                            first_headings[vehicle_id]))
        headings = self._checked_headings(headings)
        return qf, headings

    def swept_check(self, states, policy):
        """Check ordered joint states and conservative samples between them."""
        self._stats["swept_checks"] += 1
        try:
            normalized = self._normalise_states(states)
        except _StateNormalisationError as exc:
            return self._failure(exc.reason, exc.segment_index, 0,
                                 vehicle_id=exc.vehicle_id,
                                 detail=exc.detail)
        except FormationValidationError as exc:
            return self._failure(
                self._error_code(exc), 0, 0, detail=str(exc))
        try:
            checked_policy = self._normalise_policy(policy)
        except FormationValidationError as exc:
            return self._failure(
                self._error_code(exc), 0, 0, detail=str(exc))
        padding = checked_policy.motion_padding_m

        self._stats["swept_samples_checked"] += 1
        report = self._check_state(normalized[0][1], normalized[0][2],
                                   padding, 0, 0)
        if not report.success:
            return report

        for segment_index, (first, second) in enumerate(
                zip(normalized, normalized[1:])):
            self._stats["segments_checked"] += 1
            try:
                bound = self._displacement_bound(first, second)
            except (ArithmeticError, ValueError) as exc:
                return self._failure(
                    "INVALID_SWEEP_DISPLACEMENT", segment_index, 0,
                    second[1], detail=str(exc))
            if not math.isfinite(bound):
                return self._failure(
                    "NONFINITE_SWEEP_DISPLACEMENT", segment_index, 0,
                    second[1], detail="displacement bound is not finite")
            sample_count = max(
                1, int(math.ceil(bound / checked_policy.max_corner_step_m)))
            if sample_count > checked_policy.max_samples_per_segment:
                return self._failure(
                    "SWEEP_SAMPLE_BUDGET_EXCEEDED", segment_index, 0,
                    second[1],
                    detail="segment needs {} samples > {}".format(
                        sample_count, checked_policy.max_samples_per_segment))
            for sample_index in range(1, sample_count + 1):
                self._stats["swept_samples_checked"] += 1
                fraction = sample_index / sample_count
                try:
                    qf, headings = self._interpolate(first, second, fraction)
                except FormationValidationError as exc:
                    return self._failure(
                        getattr(exc, "reason", self._error_code(exc)),
                        segment_index, sample_index,
                        vehicle_id=getattr(exc, "vehicle_id", None),
                        detail=str(exc))
                report = self._check_state(
                    qf, headings, padding, segment_index, sample_index)
                if not report.success:
                    return report
        return CollisionReport(True)

    def statistics(self):
        """Return a copy of cumulative query and sample counts."""
        return dict(self._stats)


class _StateNormalisationError(ValueError):
    def __init__(self, reason, segment_index, detail, vehicle_id=None):
        self.reason = reason
        self.segment_index = segment_index
        self.detail = detail
        self.vehicle_id = vehicle_id
        super().__init__(detail)
