"""Multi-vehicle OBB and conservative joint-state swept checks.

This module composes the S1 rigid-center model with the existing exact
``FootprintDistance`` grid query and an exact pair-of-OBB separating-axis test.
It does not search for a path, assign vehicle modes, retime a trajectory, or
publish a feasibility result.
"""
from collections.abc import Mapping as MappingABC
from dataclasses import dataclass, replace
import math
from numbers import Real
from types import MappingProxyType
from typing import Callable, Mapping, Optional

import numpy as np

from .core import Grid, Pose
from .formation import (FormationModel, FormationPose,
                        FormationValidationError, VehiclePose)
from .quality import FootprintDistance


UINT32_MAX = 2**32 - 1


def grid_from_occupancy_grid(message):
    """Convert ``nav_msgs/OccupancyGrid`` into the shared collision grid."""
    try:
        info = message.info
        frame_id = getattr(message.header, "frame_id", "")
        width = int(info.width)
        height = int(info.height)
        resolution = float(info.resolution)
        origin = info.origin
        ox = float(origin.position.x)
        oy = float(origin.position.y)
        qx = float(origin.orientation.x)
        qy = float(origin.orientation.y)
        qz = float(origin.orientation.z)
        qw = float(origin.orientation.w)
        if frame_id != "map":
            return None, "map frame_id must be 'map', got {!r}".format(frame_id)
        if width <= 0 or height <= 0:
            return None, "map dimensions must be positive"
        if not math.isfinite(resolution) or resolution <= 0.0:
            return None, "map resolution must be positive and finite"
        if not (math.isfinite(ox) and math.isfinite(oy)):
            return None, "map origin must be finite"
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not math.isfinite(norm) or norm <= 0.0:
            return None, "map origin quaternion is invalid"
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        if not math.isfinite(yaw) or abs(yaw) > 1e-7:
            return None, "rotated map origins are not supported"
        data = np.asarray(message.data, dtype=np.int8)
        if data.size != width * height:
            return None, "map data length does not match its dimensions"
        # Unobserved cells are occupied in each formation collision check.
        occupied = data.reshape(height, width) != 0
        return Grid(occupied, resolution, ox, oy), None
    except Exception as exc:
        return None, "invalid map message: {!r}".format(exc)


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
    # Analytic sweeps report the exact model time/station that produced the
    # failing state.  Legacy swept_check leaves both fields unset.
    time_s: Optional[float] = None
    station: Optional[float] = None


@dataclass(frozen=True)
class AnalyticSweepSegment:
    """One exactly-evaluated V0.2 segment for conservative swept coverage.

    ``fraction`` runs from 0 to 1 over the segment and must be mapped by the
    caller to the authoritative analytic model, never to a chord interpolation.
    The three bounds are suprema per unit fraction over the closed segment:

        sup ||d q_F / d lambda||
        sup |d psi_F / d lambda|
        sup |d theta_i / d lambda| for every vehicle i

    The checker turns them into the task bound
    ``sup ||p_i'(lambda)|| + body_half_diagonal_i * sup |theta_i'(lambda)|``
    using ``||p_i'|| <= ||q_F'|| + |r_i| |psi_F'|`` and the margin-inflated
    body diagonal.  A caller that cannot justify a finite nonnegative bound
    must not use this API.
    """

    station_start: float
    station_end: float
    state_at: Callable[[float], "FormationState"]
    time_at: Callable[[float], float]
    center_speed_bound_m_per_lambda: float
    formation_rate_bound_rad_per_lambda: float
    body_rate_bounds_rad_per_lambda: Mapping[int, float]
    label: str = ""

    def __post_init__(self):
        object.__setattr__(self, "station_start", _finite_number(
            self.station_start, "AnalyticSweepSegment.station_start"))
        object.__setattr__(self, "station_end", _finite_number(
            self.station_end, "AnalyticSweepSegment.station_end"))
        if not self.station_end > self.station_start:
            raise FormationValidationError(
                "NONMONOTONIC_ANALYTIC_SEGMENT station_end={} must exceed "
                "station_start={}".format(self.station_end, self.station_start))
        if not callable(self.state_at) or not callable(self.time_at):
            raise FormationValidationError(
                "INVALID_ANALYTIC_SEGMENT state_at/time_at must be callable")
        for name in ("center_speed_bound_m_per_lambda",
                     "formation_rate_bound_rad_per_lambda"):
            object.__setattr__(self, name, _finite_number(
                getattr(self, name),
                "AnalyticSweepSegment." + name, nonnegative=True))
        if not isinstance(self.body_rate_bounds_rad_per_lambda, MappingABC):
            raise FormationValidationError(
                "INVALID_ANALYTIC_SEGMENT expected body-rate bound mapping")
        checked = {}
        for vehicle_id, value in self.body_rate_bounds_rad_per_lambda.items():
            checked_id = _checked_vehicle_id(
                vehicle_id, "AnalyticSweepSegment.body_rate_bounds")
            checked[checked_id] = _finite_number(
                value, "body_rate_bounds[{}]".format(checked_id),
                nonnegative=True)
        object.__setattr__(self, "body_rate_bounds_rad_per_lambda",
                           MappingProxyType(checked))


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
                 vehicle_id=None, other_vehicle_id=None, detail=None,
                 time_s=None, station=None):
        return CollisionReport(
            False, reason, vehicle_id, other_vehicle_id,
            segment_index, sample_index, detail, qf,
            time_s=time_s, station=station)

    @staticmethod
    def _error_code(exc):
        text = str(exc)
        return text.split(None, 1)[0] if text else "INVALID_FORMATION_INPUT"

    def _check_poses(self, qf, vehicle_poses, padding_m, segment_index,
                     sample_index):
        distances = self._distance_objects(padding_m)
        for vehicle_pose in vehicle_poses:
            self._stats["map_queries"] += 1
            if not distances[vehicle_pose.vehicle_id].pose_clear(
                    Pose(vehicle_pose.x, vehicle_pose.y, vehicle_pose.theta)):
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

    def sample_static_headings(self, qf, body_headings, free_ids):
        """Find a static fit at fixed centres, sampling body rectangles every 5°."""
        centers = self.model.centers(qf)
        headings = self._checked_headings(body_headings)
        free_ids = set(free_ids)
        # A rectangle repeats after pi. Try the closest heading first.
        offsets = [0.0]
        for angle in range(5, 90, 5):
            offsets.extend((math.radians(angle), math.radians(-angle)))
        offsets.append(math.pi / 2.0)
        options = {}
        for vehicle_id, (x, y) in centers.items():
            allowed = offsets if vehicle_id in free_ids else (0.0,)
            yaws = (headings[vehicle_id] + offset for offset in allowed)
            options[vehicle_id] = [
                VehiclePose(vehicle_id, x, y, yaw) for yaw in yaws
                if self._base_distances[vehicle_id].pose_clear(Pose(x, y, yaw))]
            if not options[vehicle_id]:
                return None

        order = sorted(options, key=lambda vehicle_id: len(options[vehicle_id]))
        chosen = {}
        halves = self._base_half_extents

        def place(index):
            if index == len(order):
                return True
            vehicle_id = order[index]
            for pose in options[vehicle_id]:
                if any(self._obb_overlap(pose, halves[vehicle_id], other, halves[other_id])
                       for other_id, other in chosen.items()):
                    continue
                chosen[vehicle_id] = pose
                if place(index + 1):
                    return True
                del chosen[vehicle_id]
            return False

        if place(0):
            return {vehicle_id: pose.theta for vehicle_id, pose in chosen.items()}
        return None

    def fixed_heading_blocked_cells(self, formation_yaw, body_headings, padding_m=0.0):
        """Conservatively mark map collisions for translating fixed OBBs.

        Each free cell permits any F position inside it at these headings.
        The two half-cell allowances cover both the motion of F within the
        cell and the occupied cell's extent. Pairwise vehicle separation is
        independent of XY and is checked at the start and by the final sweep.
        """
        padding = _finite_number(padding_m, "configuration padding", nonnegative=True)
        headings = self._checked_headings(body_headings)
        centers = self.model.centers(FormationPose(0.0, 0.0, formation_yaw))
        grid = self.grid
        blocked = np.zeros_like(grid.occ, dtype=bool)
        occupied_prefix = np.zeros((grid.height, grid.width + 1), dtype=np.int32)
        np.cumsum(grid.occ, axis=1, dtype=np.int32, out=occupied_prefix[:, 1:])
        for vehicle_id in self._vehicle_ids:
            center_x, center_y = centers[vehicle_id]
            c = math.cos(headings[vehicle_id])
            s = math.sin(headings[vehicle_id])
            half_l, half_w = self._base_half_extents[vehicle_id]
            half_l += padding
            half_w += padding
            half_x = abs(c) * half_l + abs(s) * half_w + grid.res
            half_y = abs(s) * half_l + abs(c) * half_w + grid.res
            cols = math.ceil((abs(center_x) + half_x) / grid.res)
            rows = math.ceil((abs(center_y) + half_y) / grid.res)
            if cols >= grid.width or rows >= grid.height:
                # Skip the conservative raster route when its padding
                # reaches across the map. The exact checker still runs.
                blocked.fill(True)
                return blocked
            x = grid.res * np.arange(-cols, cols + 1)[None, :] - center_x
            y = grid.res * np.arange(-rows, rows + 1)[:, None] - center_y
            obb_pad = grid.res * (abs(c) + abs(s)) + 1e-12
            kernel = ((np.abs(x) <= half_x + 1e-12) & (np.abs(y) <= half_y + 1e-12)
                      & (np.abs(c * x + s * y) <= half_l + obb_pad)
                      & (np.abs(-s * x + c * y) <= half_w + obb_pad))
            # The intersection of the SAT strips is convex, so each
            # stencil row touches one interval. Query those intervals with
            # row prefix sums, keeping memory proportional to the map area.
            support_y, support_x = np.nonzero(kernel)
            first_row = max(0, rows - int(support_y.min()))
            last_row = min(grid.height, grid.height + rows - int(support_y.max()))
            first_col = max(0, cols - int(support_x.min()))
            last_col = min(grid.width, grid.width + cols - int(support_x.max()))
            if first_row >= last_row or first_col >= last_col:
                blocked.fill(True)
                return blocked
            blocked[:first_row] = blocked[last_row:] = True
            blocked[:, :first_col] = blocked[:, last_col:] = True
            interior = blocked[first_row:last_row, first_col:last_col]
            for kernel_row in np.unique(support_y):
                positions = support_x[support_y == kernel_row]
                left = int(positions[0]) - cols
                right = int(positions[-1]) - cols + 1
                shift = int(kernel_row) - rows
                prefix = occupied_prefix[first_row + shift:last_row + shift]
                interior |= (prefix[:, first_col + left:last_col + left]
                             != prefix[:, first_col + right:last_col + right])
        return blocked

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

    def inflated_body_half_diagonals(self):
        """Read-only copy of margin-inflated body half-diagonals."""
        return dict(self._corner_radii)

    def _normalise_analytic_segments(self, segments):
        try:
            supplied = list(segments)
        except TypeError as exc:
            raise FormationValidationError(
                "INVALID_ANALYTIC_SEGMENTS expected an ordered iterable") from exc
        if not supplied:
            raise FormationValidationError("EMPTY_ANALYTIC_SEGMENTS")
        result = []
        expected_ids = set(self._vehicle_ids)
        for index, segment in enumerate(supplied):
            if not isinstance(segment, AnalyticSweepSegment):
                raise FormationValidationError(
                    "INVALID_ANALYTIC_SEGMENTS segment {} is not "
                    "AnalyticSweepSegment".format(index))
            if set(segment.body_rate_bounds_rad_per_lambda) != expected_ids:
                raise FormationValidationError(
                    "INVALID_ANALYTIC_SEGMENTS segment {} body-rate ids "
                    "differ".format(index))
            result.append(segment)
        return tuple(result)

    def _analytic_state(self, segment, fraction, segment_index):
        try:
            state = segment.state_at(float(fraction))
        except (ArithmeticError, TypeError, ValueError) as exc:
            raise FormationValidationError(
                "INVALID_ANALYTIC_SWEEP_STATE segment {} fraction {}: {}"
                .format(segment_index, fraction, exc)) from exc
        if not isinstance(state, FormationState):
            raise FormationValidationError(
                "INVALID_ANALYTIC_SWEEP_STATE segment {} did not return "
                "FormationState".format(segment_index))
        expected_station = (segment.station_start +
                            float(fraction) * (segment.station_end -
                                               segment.station_start))
        if abs(state.station - expected_station) > 1e-12:
            raise FormationValidationError(
                "INVALID_ANALYTIC_SWEEP_STATE segment {} station {} does not "
                "match lambda-derived {}".format(
                    segment_index, state.station, expected_station))
        headings = self._checked_headings(state.body_headings)
        return expected_station, state.qf, headings

    def _analytic_time(self, segment, fraction, segment_index):
        try:
            value = segment.time_at(float(fraction))
        except (ArithmeticError, TypeError, ValueError) as exc:
            raise FormationValidationError(
                "INVALID_ANALYTIC_SWEEP_TIME segment {} fraction {}: {}"
                .format(segment_index, fraction, exc)) from exc
        return _finite_number(
            value, "analytic_segments[{}].time_at".format(segment_index))

    def analytic_segment_bound(self, segment):
        """Conservative per-vehicle displacement bound over this segment."""
        if not isinstance(segment, AnalyticSweepSegment):
            raise FormationValidationError(
                "INVALID_ANALYTIC_SEGMENT expected AnalyticSweepSegment")
        maximum = 0.0
        for offset in self.model.offsets:
            vehicle_id = offset.vehicle_id
            centre_bound = (
                segment.center_speed_bound_m_per_lambda
                + math.hypot(offset.x, offset.y)
                * segment.formation_rate_bound_rad_per_lambda)
            body_bound = (
                self._corner_radii[vehicle_id]
                * segment.body_rate_bounds_rad_per_lambda[vehicle_id])
            maximum = max(maximum, centre_bound + body_bound)
        return maximum

    def swept_check_analytic(self, segments, policy, trace=None):
        """Sample exact V0.2 states with conservative per-segment bounds.

        Unlike :meth:`swept_check`, no chord interpolation is used.  Every
        checked pose is produced by the caller's analytic callback at the
        exact fraction.  The per-segment sample budget is enforced before any
        state is evaluated and is never increased to force a pass.
        """
        self._stats["swept_checks"] += 1
        try:
            checked_segments = self._normalise_analytic_segments(segments)
        except FormationValidationError as exc:
            return self._failure(
                self._error_code(exc), 0, 0, detail=str(exc))
        try:
            checked_policy = self._normalise_policy(policy)
        except FormationValidationError as exc:
            return self._failure(
                self._error_code(exc), 0, 0, detail=str(exc))
        if trace is not None and not hasattr(trace, "append"):
            raise FormationValidationError(
                "INVALID_SWEEP_TRACE expected a list-like trace")
        padding = checked_policy.motion_padding_m
        for segment_index, segment in enumerate(checked_segments):
            try:
                station, qf, headings = self._analytic_state(
                    segment, 0.0, segment_index)
                time_s = self._analytic_time(segment, 0.0, segment_index)
            except FormationValidationError as exc:
                return self._failure(
                    getattr(exc, "reason", self._error_code(exc)),
                    segment_index, 0, detail=str(exc))
            if trace is not None:
                trace.append({
                    "segment_index": segment_index,
                    "label": segment.label,
                    "fraction": 0.0,
                    "time_s": time_s,
                    "station": station,
                    "qf": [qf.x, qf.y, qf.psi],
                    "kind": "segment_start",
                })
            self._stats["swept_samples_checked"] += 1
            report = self._check_state(
                qf, headings, padding, segment_index, 0)
            if not report.success:
                return replace(report, time_s=time_s, station=station)
            try:
                bound = self.analytic_segment_bound(segment)
            except (ArithmeticError, OverflowError) as exc:
                return self._failure(
                    "INVALID_SWEEP_DISPLACEMENT", segment_index, 0,
                    qf, detail=str(exc), time_s=time_s, station=station)
            if not math.isfinite(bound):
                return self._failure(
                    "NONFINITE_SWEEP_DISPLACEMENT", segment_index, 0,
                    qf, detail="analytic displacement bound is not finite",
                    time_s=time_s, station=station)
            sample_count = max(
                1, int(math.ceil(bound / checked_policy.max_corner_step_m)))
            if sample_count > checked_policy.max_samples_per_segment:
                return self._failure(
                    "SWEEP_SAMPLE_BUDGET_EXCEEDED", segment_index, 0,
                    qf,
                    detail="analytic segment needs {} samples > {}".format(
                        sample_count,
                        checked_policy.max_samples_per_segment),
                    time_s=time_s, station=station)
            if trace is not None:
                trace.append({
                    "segment_index": segment_index,
                    "label": segment.label,
                    "kind": "budget",
                    "bound_m": bound,
                    "sample_count": sample_count,
                    "max_corner_step_m": checked_policy.max_corner_step_m,
                    "max_samples_per_segment":
                        checked_policy.max_samples_per_segment,
                    "station_start": segment.station_start,
                    "station_end": segment.station_end,
                })
            self._stats["segments_checked"] += 1
            for sample_index in range(1, sample_count + 1):
                fraction = sample_index / float(sample_count)
                try:
                    station, qf, headings = self._analytic_state(
                        segment, fraction, segment_index)
                    time_s = self._analytic_time(
                        segment, fraction, segment_index)
                except FormationValidationError as exc:
                    return self._failure(
                        getattr(exc, "reason", self._error_code(exc)),
                        segment_index, sample_index, detail=str(exc))
                self._stats["swept_samples_checked"] += 1
                if trace is not None:
                    trace.append({
                        "segment_index": segment_index,
                        "label": segment.label,
                        "fraction": fraction,
                        "time_s": time_s,
                        "station": station,
                        "qf": [qf.x, qf.y, qf.psi],
                        "kind": "sample",
                    })
                report = self._check_state(
                    qf, headings, padding, segment_index, sample_index)
                if not report.success:
                    if trace is not None:
                        trace[-1]["collision"] = True
                        trace[-1]["reason"] = report.reason
                    return replace(report, time_s=time_s, station=station)
        return CollisionReport(True)

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
