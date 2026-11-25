"""FR03 finite-label search states, keys, uniform-mode successors and cost.

Scope (FORMATION_REFACTOR_IMPLEMENTATION_PLAN.md section 7 and
FORMATION_REFACTOR_DECISIONS.md section 7):

* An exact formation state carries ``q_F``, the exact (unwrapped) body
  headings, the current mode and the previous canonical wheel/primitive
  signature.  Nothing is snapped to a grid centre.
* A finite-label key is used only for deduplication.  It keeps the XY bin
  (0.20 m), the cargo-heading bin (10 deg), the mode, a per-vehicle wheel
  signature label, a per-vehicle heading label (10 deg bin plus the gear /
  direction label) and the finite A-scheme capability labels.  It is *not* a
  high-resolution joint heading grid and it never merges states that differ in
  mode, heading label, wheel label or exact A-scheme capability.
* ``a_labels`` is the bounded compatibility distinction required by
  DECISIONS section 7 (D3-A): the set of labels of the A candidates that the
  exact headings actually satisfy under the current search goal.  The domain
  is finite: the goal-directed straight scheme plus every configured
  fixed/cropped arc scheme, times every allowed gear combination (at most
  8 + 15 schemes x 8 gears before deduplication).  A state that is not yet
  tangent-compatible therefore cannot swallow a generated S alignment child
  that unlocks an A candidate, even when both fall in the same 10 deg bin.
  The label is computed by the same FR01 required-heading helpers that the
  generator uses to emit the candidate, so a label is present exactly when
  that candidate can be generated.  Labels are goal-dependent; a
  representative table must be used for one goal at a time.
* Successors are single authoritative FR01 edges.  A/O/S candidates are
  generated from a fixed, centralised library; every candidate is checked with
  the real analytic-pose OBB sweep before it is accepted.  Full S8/S9 retiming
  is never run per expansion.
* The edge cost is a seconds-dimension *proxy*: the actual common wheel-setup
  demand (FR02 semantics) plus a first-order motion duration estimate.  It is
  explicitly not a complete dynamic minimum time, not admissible and not a
  global-optimality certificate.  The real ``T_end`` comes from S8 later.

The FR04 graph search uses these same successors for every connector and
lattice expansion. Fixed-heading raster routes only propose sequences of
waypoints; the analytic sweeps and terminal audit still gate the result.
"""
from dataclasses import dataclass, field, replace
import heapq
import itertools
import math
import time
from typing import Dict, List, Mapping, Optional, Tuple

from .core import Action
from .formation import FormationModel, FormationPose, FormationValidationError
from .formation_collision import (
    AnalyticSweepSegment,
    FormationCollisionChecker,
    FormationState,
    SweepPolicy,
    VehicleShape,
)
from .formation_kinematics import ACKERMANN
from .formation_retiming import FormationTimingLimits, translation_block_length_limit
from .mode_switch_timing import mode_switch_event
from .retiming import wheel_configuration
from .uniform_motion import (
    MODE_A,
    MODE_O,
    MODE_S,
    PRIMITIVE_A_ARC,
    PRIMITIVE_A_STRAIGHT,
    PRIMITIVE_O_TRANSLATION,
    PRIMITIVE_S_SPIN,
    UniformMotionEdge,
    UniformMotionError,
    a_arc_edge,
    a_arc_required_headings,
    a_straight_edge,
    a_straight_required_headings,
    o_translation_edge,
    s_spin_edge,
)
from .uniform_timing import timing_motion_limits
from .uniform_convex_corridor import convex_corridor, shortcut_corridor
from .uniform_translation_route import fixed_heading_route

# Reuse FR02's exact wheel-change threshold so a "no setup" decision in the
# search matches the timing auditor.
WHEEL_CHANGE_TOL_RAD = 1e-9
HEADING_MATCH_TOL_RAD = 1e-9
GEOMETRY_CONTINUITY_TOL_M = 1e-7


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _finite(value, context):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise FormationValidationError(
            "NONFINITE_VALUE context={} value={!r}".format(context, value)) from exc
    if not math.isfinite(result):
        raise FormationValidationError(
            "NONFINITE_VALUE context={} value={!r}".format(context, value))
    return result


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SuccessorConfig:
    """Centralised, visible successor/key/cost configuration.

    The pivot and angle library is exactly the first-version library fixed in
    DECISIONS section 5: ``c=(0,0),(0,±2),(±2,0)`` cargo-frame metres and a
    ±10 deg arc step.  Nothing here may be tuned per test case to force a pass.
    """

    xy_bin_m: float = 0.20
    psi_bin_rad: float = math.radians(10.0)
    heading_bin_rad: float = math.radians(10.0)
    control_bin: float = 0.01
    arc_pivots: Tuple[Tuple[float, float], ...] = (
        (0.0, 0.0), (0.0, 2.0), (0.0, -2.0), (2.0, 0.0), (-2.0, 0.0))
    arc_step_rad: float = math.radians(10.0)
    translation_step_m: float = 1.0
    allow_reverse: bool = True
    goal_heading_free: bool = True
    goal_formation_heading_free: bool = False
    goal_xy_tol_m: float = 0.02
    goal_psi_tol_rad: float = math.radians(1.0)
    goal_heading_tol_rad: float = math.radians(1.0)
    sweep_spacing_m: float = 0.01
    max_samples_per_segment: int = 200000
    collision_margin_m: float = 0.05
    timing_limits: FormationTimingLimits = field(
        default_factory=FormationTimingLimits)

    def __post_init__(self):
        for name in ("xy_bin_m", "psi_bin_rad", "heading_bin_rad",
                     "control_bin", "arc_step_rad", "translation_step_m",
                     "goal_xy_tol_m", "goal_psi_tol_rad",
                     "goal_heading_tol_rad", "sweep_spacing_m"):
            object.__setattr__(self, name, _finite(
                getattr(self, name), "SuccessorConfig." + name))
            if getattr(self, name) <= 0.0:
                raise FormationValidationError(
                    "INVALID_SUCCESSOR_CONFIG {} must be positive".format(name))
        if not isinstance(self.timing_limits, FormationTimingLimits):
            raise FormationValidationError(
                "INVALID_SUCCESSOR_CONFIG timing_limits")
        for name in ("allow_reverse", "goal_heading_free", "goal_formation_heading_free"):
            if not isinstance(getattr(self, name), bool):
                raise FormationValidationError(
                    "INVALID_SUCCESSOR_CONFIG {} must be bool".format(name))
        if isinstance(self.max_samples_per_segment, bool) or not isinstance(
                self.max_samples_per_segment, int) or self.max_samples_per_segment <= 0:
            raise FormationValidationError(
                "INVALID_SUCCESSOR_CONFIG max_samples_per_segment")
        object.__setattr__(self, "collision_margin_m", _finite(
            self.collision_margin_m, "SuccessorConfig.collision_margin_m"))
        if self.collision_margin_m < 0.0:
            raise FormationValidationError(
                "INVALID_SUCCESSOR_CONFIG collision_margin_m")
        object.__setattr__(self, "arc_pivots", tuple(
            (_finite(px, "arc_pivot.x"), _finite(py, "arc_pivot.y"))
            for px, py in self.arc_pivots))


# ---------------------------------------------------------------------------
# Exact state and wheel signature
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class VehicleWheelState:
    """One vehicle's canonical wheel/primitive signature at a state."""

    vehicle_id: int
    mode: str
    direction: int
    control: float
    angles: Tuple[float, float, float, float]

    def label(self, control_bin):
        return (self.vehicle_id, self.mode,
                int(round(self.control / control_bin)))

    def as_dict(self):
        return {
            "vehicle_id": self.vehicle_id,
            "mode": self.mode,
            "direction": self.direction,
            "control": self.control,
            "wheel_angles_rad": list(self.angles),
        }


@dataclass(frozen=True)
class UniformSearchState:
    """An exact formation state; poses and headings are never grid-snapped."""

    qf: FormationPose
    headings: Tuple[Tuple[int, float], ...]
    mode: str
    wheel: Tuple[VehicleWheelState, ...]
    g: float = 0.0
    parent: Optional["UniformSearchState"] = None
    edge: Optional[UniformMotionEdge] = None
    # Finite A-scheme capabilities of these exact headings under the search
    # goal; part of the key so a necessary S alignment is never swallowed by
    # a same-bin state that cannot generate the A candidate.  Goal-dependent:
    # a table is valid for one goal at a time.
    a_labels: Tuple[str, ...] = ()

    def heading_map(self):
        return {vehicle_id: theta for vehicle_id, theta in self.headings}

    def as_dict(self):
        return {
            "qf": [self.qf.x, self.qf.y, self.qf.psi],
            "headings": [
                {"vehicle_id": vehicle_id, "theta": theta}
                for vehicle_id, theta in self.headings],
            "mode": self.mode,
            "g": self.g,
            "wheel": [item.as_dict() for item in self.wheel],
            "a_labels": list(self.a_labels),
            "has_parent": self.parent is not None,
            "edge_primitive": (None if self.edge is None else self.edge.primitive),
        }


@dataclass(frozen=True)
class UniformStateKey:
    """Finite-label deduplication key; never a strict-dominance certificate."""

    xy_bin: Tuple[int, int]
    psi_bin: int
    mode: str
    wheel_label: Tuple[Tuple[int, str, int], ...]
    heading_label: Tuple[Tuple[int, int, int], ...]
    a_labels: Tuple[str, ...]

    def as_dict(self):
        return {
            "xy_bin": list(self.xy_bin),
            "psi_bin": self.psi_bin,
            "mode": self.mode,
            "wheel_label": [list(item) for item in self.wheel_label],
            "heading_label": [list(item) for item in self.heading_label],
            "a_labels": list(self.a_labels),
        }


def initial_wheel_signature(model, motion_limits):
    """The contractual initial configuration: all A, kappa=0, stopped."""
    angles, _ = wheel_configuration(
        Action(ACKERMANN, 0.0, 0.0), motion_limits)
    angles = tuple(float(value) for value in angles)
    return tuple(
        VehicleWheelState(
            vehicle_id=vehicle_id, mode=MODE_A, direction=1, control=0.0,
            angles=angles)
        for vehicle_id in model.vehicle_ids)


def initial_state(model, qf, headings, motion_limits=None, config=None,
                  goal_qf=None, *, initial_edge=None):
    """Build the exact start state, retaining a supplied terminal wheel configuration.

    Pass ``goal_qf`` to label the state with its finite A capabilities for
    that search goal.  Without a goal the goal-directed straight and cropped
    arc labels are unknown and omitted (splitting keys, never merging two
    different capabilities); fixed arc labels are still exact.
    """
    if config is not None:
        motion_limits = timing_motion_limits(config.timing_limits)
    if motion_limits is None:
        motion_limits = timing_motion_limits(FormationTimingLimits())
    heading_pairs = tuple(sorted(
        (int(vehicle_id), _finite(theta, "initial heading"))
        for vehicle_id, theta in dict(headings).items()))
    return UniformSearchState(
        qf=qf, headings=heading_pairs, mode=MODE_A if initial_edge is None else initial_edge.mode,
        wheel=(initial_wheel_signature(model, motion_limits) if initial_edge is None
               else wheel_signature_from_edge(initial_edge)), g=0.0,
        a_labels=a_capability_labels(
            model, qf, heading_pairs, goal_qf, config, motion_limits))


def wheel_signature_from_edge(edge):
    """The canonical end-of-edge wheel signature (constant within an edge)."""
    directions = dict(edge.directions)
    controls = {segment.vehicle_id: float(segment.control)
                for segment in edge.segments}
    configurations = {
        configuration.vehicle_id: configuration
        for configuration in edge.wheel_configurations(at_end=True)}
    return tuple(
        VehicleWheelState(
            vehicle_id=vehicle_id, mode=edge.mode,
            direction=int(directions[vehicle_id]),
            control=controls[vehicle_id],
            angles=tuple(float(value)
                         for value in configurations[vehicle_id].angles))
        for vehicle_id in edge.model.vehicle_ids)


def make_key(state, config):
    """Finite-label key.  Binning is a key only, never a state snap."""
    xy_bin = (int(round(state.qf.x / config.xy_bin_m)),
              int(round(state.qf.y / config.xy_bin_m)))
    psi_bin = int(round(_wrap(state.qf.psi) / config.psi_bin_rad))
    wheel_label = tuple(
        item.label(config.control_bin) for item in state.wheel)
    directions = {item.vehicle_id: item.direction for item in state.wheel}
    heading_label = tuple(
        (vehicle_id,
         int(round(_wrap(theta) / config.heading_bin_rad)),
         int(directions[vehicle_id]))
        for vehicle_id, theta in state.headings)
    return UniformStateKey(
        xy_bin=xy_bin, psi_bin=psi_bin, mode=state.mode,
        wheel_label=wheel_label, heading_label=heading_label,
        a_labels=tuple(state.a_labels))


class RepresentativeTable:
    """Same-complete-key representative / lower-g selection.

    This is a bounded discrete approximation, not a strict dominance theorem
    over arbitrary exact states inside one bin.  Exact poses/headings stay in
    the node and are used for edge integration.
    """

    def __init__(self, config):
        self.config = config
        self._best: Dict[UniformStateKey, UniformSearchState] = {}

    def offer(self, state):
        """Return ``(accepted_as_representative, key, incumbent)``."""
        key = make_key(state, self.config)
        incumbent = self._best.get(key)
        if incumbent is None or state.g < incumbent.g:
            self._best[key] = state
            return True, key, incumbent
        return False, key, incumbent

    def representative(self, key):
        return self._best.get(key)

    def __len__(self):
        return len(self._best)


class _PythonFrontier:
    def __init__(self, config, rank):
        self.config = config
        self.rank = rank
        self.table = RepresentativeTable(config)
        self.heap = []
        self.serial = itertools.count()

    def offer(self, state, initial=False):
        accepted, _, _ = self.table.offer(state)
        if accepted:
            heapq.heappush(self.heap, (
                self.rank(state, initial), state.g, next(self.serial), state))
        return accepted

    def pop(self):
        _, _, _, state = heapq.heappop(self.heap)
        key = make_key(state, self.config)
        return state if self.table.representative(key) is state else None

    def __len__(self):
        return len(self.heap)


class _NativeFrontier:
    def __init__(self, config, rank):
        try:
            from fwis_multi_frontier_native import Frontier
        except ImportError as exc:
            raise FormationValidationError(
                "NATIVE_SEARCH_FRONTIER_UNAVAILABLE: build fwis_multi_planner_cpp with pybind11") from exc
        self.frontier = Frontier(config)
        self.rank = rank
        self.pending = {}

    def offer(self, state, initial=False):
        serial = self.frontier.offer(state, state.g, self.rank(state, initial))
        if serial is None:
            return False
        self.pending[serial] = state
        return True

    def pop(self):
        serial, stale = self.frontier.pop()
        state = self.pending.pop(serial)
        return None if stale else state

    def __len__(self):
        return len(self.frontier)


# ---------------------------------------------------------------------------
# Cost: actual setup demand + first-order motion proxy (seconds)
# ---------------------------------------------------------------------------


def setup_demand_s(previous_wheel, edge, motion_limits):
    """Common stationary setup seconds between a state and an edge start.

    Mirrors FR02 ``_make_setup``: per vehicle the signed wheel-angle deltas and
    the mode decide whether a stationary event exists; the common duration is
    the slowest vehicle's ``mode_switch_event`` duration.  A mode change forces
    an event even when the canonical wheel angles coincide.  No fixed 0.3 s is
    used to hide a longer steering demand.
    """
    previous = {item.vehicle_id: item for item in previous_wheel}
    current = wheel_signature_from_edge(edge)
    # wheel_signature_from_edge uses the end configuration, which equals the
    # start configuration for every FR01 primitive (constant control).  FR02's
    # ``_make_setup`` reads the same constant start descriptor.
    duration = 0.0
    mode_changed = False
    wheel_changed = False
    for item in current:
        before = previous[item.vehicle_id]
        signed_deltas = tuple(
            _wrap(end - start)
            for start, end in zip(before.angles, item.angles))
        delta = max(abs(value) for value in signed_deltas)
        if before.mode != item.mode:
            mode_changed = True
        if delta > WHEEL_CHANGE_TOL_RAD:
            wheel_changed = True
        event = mode_switch_event(delta, motion_limits)
        duration = max(duration, float(event["duration"]))
    if not mode_changed and not wheel_changed:
        return 0.0
    return duration


def motion_proxy_s(edge, timing_limits):
    """First-order seconds estimate: max(center travel / v, yaw travel / omega).

    Center travel is the largest per-vehicle centre path length; yaw travel is
    the largest body-heading change.  This is a constant-speed first-order
    proxy only -- it ignores acceleration, jerk, wheel-speed limits and the
    common quintic law, so it is neither the S8 duration nor an admissible
    lower bound.
    """
    center_distance = max(
        (float(segment.travel_distance) for segment in edge.segments),
        default=0.0)
    start = dict(edge.start_headings)
    end = dict(edge.end_headings)
    yaw_distance = max(
        (abs(end[vehicle_id] - start[vehicle_id])
         for vehicle_id in start), default=0.0)
    t_center = center_distance / float(timing_limits.v_max)
    t_yaw = yaw_distance / float(timing_limits.yaw_rate_max)
    return max(t_center, t_yaw)


def _translation_pieces(edge, length_limit):
    """Subdivide an exact A/O translation without altering its route or gears."""
    if (edge.primitive not in (PRIMITIVE_A_STRAIGHT, PRIMITIVE_O_TRANSLATION)
            or edge.control.parameter_length <= length_limit):
        return (edge,)
    count = math.ceil(edge.control.parameter_length / length_limit)
    directions = dict(edge.directions)
    headings = dict(edge.start_headings)
    obliques = dict(edge.oblique_angles)
    motion_options = dict(limits=edge.limits, allow_reverse=edge.allow_reverse)
    start, end = edge.start_qf, edge.end_qf
    pieces = []
    for index in range(1, count + 1):
        target = (edge.state_at(edge.control.parameter_length * index / count).qf
                  if index < count else end)
        displacement = (target.x - start.x, target.y - start.y)
        if edge.primitive == PRIMITIVE_A_STRAIGHT:
            part = a_straight_edge(edge.model, start, headings, displacement,
                                   directions, **motion_options)
        else:
            part = o_translation_edge(edge.model, start, headings, displacement,
                                       directions, obliques, **motion_options)
        pieces.append(part)
        start = part.end_qf
    return tuple(pieces)


# ---------------------------------------------------------------------------
# Analytic geometric sweep (no S8/S9 retiming)
# ---------------------------------------------------------------------------


def analytic_sweep_segment(edge, proxy_duration_s):
    """Build a geometric AnalyticSweepSegment for one authoritative edge.

    The segment parameter is the normalised fraction in [0, 1]; the state is
    the exact ``edge.state_at(fraction * L)``.  Bounds are geometric
    per-fraction suprema (no 1.875 quintic-time factor, because the search
    does not retime the edge).  ``proxy_duration_s`` is only the synthetic
    time axis used by the existing checker trace; it is not an S8 duration.
    """
    length = edge.control.parameter_length
    primitive = edge.primitive
    start = dict(edge.start_headings)
    end = dict(edge.end_headings)
    if primitive == PRIMITIVE_A_ARC:
        pivot = (edge.control.pivot_x, edge.control.pivot_y)
        # ||dq_F/dpsi|| = |pivot|; |dpsi_F/dfraction| = |delta_psi| = L.
        center_bound = length * math.hypot(pivot[0], pivot[1])
        formation_bound = length
        body_bounds = {
            vehicle_id: abs(end[vehicle_id] - start[vehicle_id])
            for vehicle_id in start}
    elif primitive == PRIMITIVE_S_SPIN:
        center_bound = 0.0
        formation_bound = 0.0
        body_bounds = {
            vehicle_id: abs(end[vehicle_id] - start[vehicle_id])
            for vehicle_id in start}
    else:
        # Straight / O: unit-direction translation, psi and headings fixed.
        center_bound = length
        formation_bound = 0.0
        body_bounds = {vehicle_id: 0.0 for vehicle_id in start}

    def state_at(fraction):
        state = edge.state_at(float(fraction) * length)
        return FormationState(
            station=float(fraction), qf=state.qf,
            body_headings={vehicle_id: theta
                           for vehicle_id, theta in state.headings})

    def time_at(fraction):
        return float(fraction) * proxy_duration_s

    return AnalyticSweepSegment(
        station_start=0.0, station_end=1.0, state_at=state_at,
        time_at=time_at,
        center_speed_bound_m_per_lambda=center_bound,
        formation_rate_bound_rad_per_lambda=formation_bound,
        body_rate_bounds_rad_per_lambda=body_bounds,
        label="{}_L{:g}".format(primitive, length))


# ---------------------------------------------------------------------------
# Successor containers
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Successor:
    """One accepted successor edge, its exact next state and proxy cost."""

    edge: UniformMotionEdge
    next_state: UniformSearchState
    setup_s: float
    motion_proxy_s: float
    cost_s: float
    label: str

    def as_dict(self):
        return {
            "label": self.label,
            "primitive": self.edge.primitive,
            "mode": self.edge.mode,
            "setup_s": self.setup_s,
            "motion_proxy_s": self.motion_proxy_s,
            "cost_s": self.cost_s,
            "start_qf": [self.edge.start_qf.x, self.edge.start_qf.y,
                         self.edge.start_qf.psi],
            "end_qf": [self.edge.end_qf.x, self.edge.end_qf.y,
                       self.edge.end_qf.psi],
            "end_headings": [
                {"vehicle_id": vehicle_id, "theta": theta}
                for vehicle_id, theta in self.edge.end_headings],
        }


@dataclass
class SuccessorResult:
    """Accepted successors plus the full rejection distribution and timings."""

    successors: List[Successor] = field(default_factory=list)
    rejections: List[dict] = field(default_factory=list)
    candidate_count: int = 0
    generation_s: float = 0.0
    collision_s: float = 0.0

    def reject(self, label, reason, detail=""):
        self.rejections.append(
            {"label": label, "reason": reason, "detail": detail})

    def rejection_histogram(self):
        histogram: Dict[str, int] = {}
        for item in self.rejections:
            histogram[item["reason"]] = histogram.get(item["reason"], 0) + 1
        return histogram

    def as_dict(self):
        return {
            "candidate_count": self.candidate_count,
            "accepted_count": len(self.successors),
            "rejected_count": len(self.rejections),
            "rejection_histogram": self.rejection_histogram(),
            "generation_s": self.generation_s,
            "collision_s": self.collision_s,
            "successors": [item.as_dict() for item in self.successors],
            "rejections": list(self.rejections),
        }


# ---------------------------------------------------------------------------
# Candidate helpers
# ---------------------------------------------------------------------------


def _gear_combinations(vehicle_ids, allow_reverse):
    if allow_reverse:
        return list(itertools.product((1, -1), repeat=len(vehicle_ids)))
    return [tuple(1 for _ in vehicle_ids)]


def _headings_match(current, required, tol=HEADING_MATCH_TOL_RAD):
    return all(
        abs(_wrap(current[vehicle_id] - required[vehicle_id])) <= tol
        for vehicle_id in required)


def _o_branch(theta, direction_angle, allow_reverse):
    """Unique (sigma, phi) solving ``u = sigma * e(theta + phi)``, |phi|<=pi/2.

    Returns ``None`` when the only solution needs a reverse branch that the
    configuration forbids.  The full vector equation is still re-validated by
    the FR01 factory; this helper never infers the branch from a longitudinal
    projection.
    """
    phi_forward = _wrap(direction_angle - theta)
    if abs(phi_forward) <= math.pi / 2.0 + 1e-12:
        return 1, _clamp_oblique(phi_forward)
    phi_reverse = _wrap(direction_angle - theta + math.pi)
    if allow_reverse and abs(phi_reverse) <= math.pi / 2.0 + 1e-12:
        return -1, _clamp_oblique(phi_reverse)
    return None


def _clamp_oblique(phi):
    # Numerical guard so |phi| never exceeds pi/2 by a few ulp.
    limit = math.pi / 2.0
    return max(-limit, min(limit, phi))


def goal_direction(qf, goal_qf, config):
    """World displacement to the goal, or ``None`` inside the XY tolerance."""
    if goal_qf is None:
        return None
    dx = goal_qf.x - qf.x
    dy = goal_qf.y - qf.y
    distance = math.hypot(dx, dy)
    if distance <= config.goal_xy_tol_m:
        return None
    return dx, dy, distance


def gear_label(gears):
    return "".join("+" if value > 0 else "-" for value in gears)


def _aligned_gears(current, tangents, vehicle_ids, allow_reverse):
    """At most one gear per vehicle can match a forward tangent."""
    gears = []
    for vehicle_id in vehicle_ids:
        error = abs(_wrap(current[vehicle_id] - tangents[vehicle_id]))
        if error <= HEADING_MATCH_TOL_RAD:
            gears.append(1)
        elif allow_reverse and abs(math.pi - error) <= HEADING_MATCH_TOL_RAD:
            gears.append(-1)
        else:
            return None
    return tuple(gears)


def a_arc_schemes(qf, goal_qf, config):
    """The finite fixed/cropped arc scheme library for this state and goal.

    Fixed +/-10 deg schemes are always offered; the goal-cropped scheme is
    offered only when the remaining cargo-heading difference is non-zero
    (both signs at exactly pi, like the generator).  Geometrically identical
    (pivot, delta) duplicates keep their first label, matching the generator.
    """
    schemes = []
    seen = set()

    def add(label, delta, pivot):
        key = (pivot[0], pivot[1], round(delta, 9))
        if key in seen:
            return
        seen.add(key)
        schemes.append((label, delta, pivot))

    for pivot in config.arc_pivots:
        for sign in (1, -1):
            label = "A_arc_p{:g}_{:g}_{}".format(
                pivot[0], pivot[1], "pos" if sign > 0 else "neg")
            add(label, sign * config.arc_step_rad, pivot)
    if goal_qf is not None and not config.goal_formation_heading_free:
        cropped = _wrap(goal_qf.psi - qf.psi)
        if abs(cropped) > 1e-9:
            deltas = ((math.pi, -math.pi)
                      if abs(abs(cropped) - math.pi) <= 1e-9
                      else (cropped,))
            for pivot in config.arc_pivots:
                for delta in deltas:
                    label = "A_arc_crop_p{:g}_{:g}_{}".format(
                        pivot[0], pivot[1], "pos" if delta > 0 else "neg")
                    add(label, delta, pivot)
    return schemes


def a_capability_labels(model, qf, headings, goal_qf, config=None,
                        motion_limits=None):
    """Finite labels of the A candidates these exact headings unlock.

    A label is present exactly when the exact headings satisfy the required
    heading set that the successor generator uses to emit that candidate
    (same FR01 helpers, same 1e-9 tolerance, same gear-combination and
    allow_reverse rules).  The label domain is the finite scheme library:
    at most 8 goal-directed straight schemes plus a bounded number of
    fixed/cropped arc schemes, each times the allowed gear combinations.
    No continuous heading or pose is hashed, and no per-theta key is made.

    ``goal_qf=None`` omits the goal-directed straight and cropped-arc labels
    (they are unknown without a goal); fixed arc labels remain exact.
    """
    config = config or SuccessorConfig()
    if motion_limits is None:
        motion_limits = timing_motion_limits(config.timing_limits)
    current = dict(headings)
    labels = []
    ids = model.vehicle_ids
    forward = dict.fromkeys(ids, 1)
    direction = goal_direction(qf, goal_qf, config)
    if direction is not None:
        tangents = a_straight_required_headings(model, direction[:2], forward)
        gears = _aligned_gears(current, tangents, ids, config.allow_reverse)
        if gears is not None:
            labels.append("A_straight_goal_gear{}".format(gear_label(gears)))
    for base_label, delta_psi, pivot in a_arc_schemes(qf, goal_qf, config):
        try:
            tangents = a_arc_required_headings(
                model, qf, delta_psi, pivot, forward, limits=motion_limits)
        except UniformMotionError:
            continue
        gears = _aligned_gears(current, tangents, ids, config.allow_reverse)
        if gears is not None:
            labels.append("{}_gear{}".format(base_label, gear_label(gears)))
    return tuple(sorted(set(labels)))


# ---------------------------------------------------------------------------
# Successor generator
# ---------------------------------------------------------------------------


class SuccessorGenerator:
    """Generate finite, mode-consistent, swept, costed successors for a state."""

    def __init__(self, model, checker, config=None, *, native_spin_sweep=False,
                 native_candidates=False):
        if not isinstance(model, FormationModel):
            raise FormationValidationError(
                "INVALID_MODEL expected FormationModel")
        if not isinstance(checker, FormationCollisionChecker):
            raise FormationValidationError(
                "INVALID_COLLISION_CHECKER expected FormationCollisionChecker")
        self.model = model
        self.checker = checker
        self.config = config or SuccessorConfig()
        self._native_spin_sweep = native_spin_sweep
        self._native_candidates = None
        if native_candidates and len(model.vehicle_ids) <= 16:
            try:
                import fwis_multi_frontier_native as native
                native.generate_lattice
            except (ImportError, AttributeError) as exc:
                raise FormationValidationError(
                    "NATIVE_SUCCESSOR_GENERATOR_UNAVAILABLE: rebuild fwis_multi_planner_cpp") from exc
            self._native_candidates = native
        self._offsets = tuple((offset.x, offset.y) for offset in model.offsets)
        self.motion_limits = timing_motion_limits(self.config.timing_limits)
        self._translation_length_limit = translation_block_length_limit(
            self.config.timing_limits)
        self.vehicle_ids = tuple(model.vehicle_ids)
        self._sweep_policy = SweepPolicy(
            max_corner_step_m=self.config.sweep_spacing_m,
            max_samples_per_segment=self.config.max_samples_per_segment)

    # -- public entry ----------------------------------------------------

    def generate(self, state, goal_qf, *, direct_goal_translation=False,
                 translation_mode=None):
        """Return a :class:`SuccessorResult` for one exact state and goal pose."""
        result = SuccessorResult()
        current = state.heading_map()

        def consider(label, edge):
            self._consider(state, goal_qf, label, edge, result)

        generation_start = time.perf_counter()
        if direct_goal_translation:
            goal = self._goal_direction(state, goal_qf)
            # Both translations preserve F heading. Individual connector
            # legs may be shorter than the lattice step.
            if (goal is not None
                    and (translation_mode is not None or goal[2] > self.config.translation_step_m)
                    and abs(_wrap(state.qf.psi - goal_qf.psi))
                    <= self.config.goal_psi_tol_rad):
                displacement = goal[:2]
                if self._native_candidates is not None:
                    branches = self._native_candidates.generate_candidates(
                        state.qf.psi, self._offsets, tuple(current[i] for i in self.vehicle_ids),
                        (("O_direct_goal", displacement),) if translation_mode in (None, MODE_O) else (),
                        (("A_direct_goal", displacement),) if translation_mode in (None, MODE_A) else (),
                        (), self.motion_limits.r_min_ack, self.config.allow_reverse, False)
                    self._add_native_branches(state, current, branches, consider, result)
                elif translation_mode in (None, MODE_A):
                    self._add_a_straight(state, current, "A_direct_goal", displacement,
                                         consider, result)
                if self._native_candidates is None and translation_mode in (None, MODE_O):
                    self._add_o(state, current, "O_direct_goal", displacement,
                                consider, result)
            result.generation_s = time.perf_counter() - generation_start - result.collision_s
            return result

        if self._native_candidates is not None:
            target = self._a_straight_displacements(state, goal_qf)
            target_yaw = (None if goal_qf is None or self.config.goal_formation_heading_free
                          else goal_qf.psi)
            branches = self._native_candidates.generate_lattice(
                state.qf.psi, self._offsets, tuple(current[i] for i in self.vehicle_ids),
                self.config.translation_step_m, target[0][1] if target else None,
                self.config.arc_pivots, self.config.arc_step_rad, target_yaw,
                self.motion_limits.r_min_ack, self.config.allow_reverse)
            self._add_native_branches(state, current, branches, consider, result)
            result.generation_s = time.perf_counter() - generation_start - result.collision_s
            return result

        # ---- O translation candidates: 8 cargo-frame directions + goal ----
        for label, displacement in self._o_displacements(state, goal_qf):
            self._add_o(state, current, label, displacement, consider, result)

        # ---- A straight candidates (only when already tangent-aligned) ----
        for label, displacement in self._a_straight_displacements(state, goal_qf):
            self._add_a_straight(
                state, current, label, displacement, consider, result)

        # ---- A fixed-pivot arc candidates (only when already aligned) ----
        for label, delta_psi, pivot in self._a_arc_schemes(state, goal_qf):
            self._add_a_arc(state, current, label, delta_psi, pivot,
                            consider, result)

        # ---- S alignment candidates towards A-scheme required headings ----
        for label, target in self._s_alignment_targets(state, goal_qf):
            self._add_s(state, current, label, target, consider, result)

        result.generation_s = time.perf_counter() - generation_start - result.collision_s
        return result

    def _add_native_branches(self, state, current, branches, consider, result):
        """Materialize the ordered C++ branch descriptors as authoritative FR01 edges."""
        native = self._native_candidates
        options = {"limits": self.motion_limits, "allow_reverse": self.config.allow_reverse}
        for kind, label, control, delta, gears, values, refusal, index, bearing in branches:
            if refusal == native.Refusal.REVERSE_REQUIRED:
                result.candidate_count += 1
                result.reject(label, "O_BRANCH_REQUIRES_REVERSE", "vehicle_id={} direction={:.6g}".format(
                    self.vehicle_ids[index], bearing))
                continue

            directions = dict(zip(self.vehicle_ids, gears))
            diagnostic_label = (label.rsplit("_sig", 1)[0]
                                if kind == native.Primitive.OBLIQUE else label)
            try:
                aligned = refusal != native.Refusal.ALIGNMENT_REQUIRED
                if refusal == native.Refusal.ARC_DOMAIN:
                    # Keep the detailed authoritative geometry diagnostic at a rejected
                    # radius and resolve libm differences on the radius boundary.
                    target = a_arc_required_headings(
                        self.model, state.qf, delta, control, directions, limits=self.motion_limits)
                    aligned = _headings_match(current, target)
                if not aligned:
                    result.candidate_count += 1
                    detail = ("body headings are not the required arc tangent"
                              if kind == native.Primitive.ARC else
                              "body headings are not the required tangent/anti-tangent")
                    result.reject(label, "A_HEADING_REQUIRES_S_ALIGNMENT", detail)
                    continue
                if kind == native.Primitive.OBLIQUE:
                    edge = o_translation_edge(self.model, state.qf, current, control, directions,
                                              dict(zip(self.vehicle_ids, values)), **options)
                elif kind == native.Primitive.STRAIGHT:
                    edge = a_straight_edge(self.model, state.qf, current, control, directions, **options)
                elif kind == native.Primitive.ARC:
                    edge = a_arc_edge(self.model, state.qf, current, delta, control, directions, **options)
                else:
                    edge = s_spin_edge(self.model, state.qf, current,
                                       dict(zip(self.vehicle_ids, values)), limits=self.motion_limits)
            except UniformMotionError as exc:
                result.candidate_count += 1
                result.reject(diagnostic_label, exc.reason, exc.detail)
                continue
            consider(label, edge)

    def _consider(self, state, goal_qf, label, edge, result):
        """Apply the same cost, sweep and exact state update to every edge."""
        result.candidate_count += 1
        if (edge.primitive in (PRIMITIVE_A_STRAIGHT, PRIMITIVE_O_TRANSLATION)
                and edge.control.parameter_length / self.config.max_samples_per_segment
                > self._translation_length_limit):
            result.reject(label, "MOTION_DURATION_LIMIT",
                          "required subdivision exceeds the per-edge work limit")
            return
        if self._native_candidates is None:
            setup_s = setup_demand_s(state.wheel, edge, self.motion_limits)
            proxy_s = motion_proxy_s(edge, self.config.timing_limits)
        else:
            previous = {item.vehicle_id: item for item in state.wheel}
            new_wheel = wheel_signature_from_edge(edge)
            end = dict(edge.end_headings)
            limits = self.motion_limits
            timing = self.config.timing_limits
            setup_s, proxy_s = self._native_candidates.edge_cost(
                [(previous[item.vehicle_id].mode, previous[item.vehicle_id].angles) for item in new_wheel],
                [(item.mode, item.angles) for item in new_wheel],
                [float(segment.travel_distance) for segment in edge.segments],
                [abs(end[i] - angle) for i, angle in edge.start_headings],
                (limits.mode_setup_time, limits.mode_switch_duration or 0., limits.steer_rate_max,
                 limits.steer_accel_max, timing.v_max, timing.yaw_rate_max,
                 limits.translation_speed_policy == "mode_switch_only"))
        segment = analytic_sweep_segment(edge, proxy_s)
        start = time.perf_counter()
        spin = None
        native_edge = edge if self._native_candidates is not None else None
        if native_edge is None and self._native_spin_sweep and edge.primitive == PRIMITIVE_S_SPIN:
            spin = (edge.control.parameter_length, dict(edge.start_headings), dict(edge.end_headings))
        report = self.checker.swept_check_analytic(
            [segment], self._sweep_policy, _stationary_spin=spin, _exact_edge=native_edge)
        result.collision_s += time.perf_counter() - start
        if not report.success:
            result.reject(
                label, report.reason or "SWEEP_REJECTED",
                "vehicle_id={} station={} detail={}".format(
                    report.vehicle_id, report.station, report.detail))
            return
        # One sweep certifies the whole straight. Store bounded physical
        # motion blocks along it so the duration cap cannot reject the
        # connector after search. Subdivision keeps the same gear and steering.
        parts = _translation_pieces(edge, self._translation_length_limit)
        next_state = state
        for index, part in enumerate(parts, 1):
            next_state = UniformSearchState(
                qf=part.end_qf, headings=tuple(part.end_headings), mode=part.mode,
                wheel=wheel_signature_from_edge(part),
                g=state.g + setup_s + proxy_s * index / len(parts),
                parent=next_state, edge=part,
                a_labels=() if index < len(parts) else a_capability_labels(
                    self.model, part.end_qf, part.end_headings, goal_qf,
                    self.config, self.motion_limits))
        result.successors.append(Successor(
            edge=edge, next_state=next_state, setup_s=setup_s,
            motion_proxy_s=proxy_s, cost_s=setup_s + proxy_s, label=label))

    def translation_junctions(self, state, goal_qf):
        """Two exact translations along and across the initial body tangent."""
        direction = self._goal_direction(state, goal_qf)
        if (direction is None or direction[2] <= self.config.translation_step_m
                or abs(_wrap(state.qf.psi - goal_qf.psi)) > self.config.goal_psi_tol_rad):
            return ()
        lead_heading = state.heading_map()[self.vehicle_ids[0]]
        c, s = math.cos(lead_heading), math.sin(lead_heading)
        dx, dy = direction[:2]
        along, across = c * dx + s * dy, -s * dx + c * dy
        if min(abs(along), abs(across)) <= self.config.goal_xy_tol_m:
            return ()
        start = state.qf
        return (
            (MODE_A, FormationPose(start.x + along * c, start.y + along * s, start.psi), MODE_O),
            (MODE_O, FormationPose(start.x - across * s, start.y + across * c, start.psi), MODE_A),
        )

    def offset_translation_junctions(self, state, corners):
        """Try an interior transverse crossing between the two projected ends."""
        if not corners:
            return
        start = state.qf
        along = (corners[0][1].x - start.x, corners[0][1].y - start.y)
        current = state.heading_map()
        if not any(_headings_match(current, a_straight_required_headings(
                self.model, along, dict(zip(self.vehicle_ids, gears))))
                for gears in _gear_combinations(self.vehicle_ids, self.config.allow_reverse)):
            return
        spacing = max(0.5 * self.config.translation_step_m, self.config.goal_xy_tol_m)
        for order, (outer, corner, middle) in enumerate(corners):
            across = corners[1 - order][1]
            dx, dy = corner.x - start.x, corner.y - start.y
            cross_x, cross_y = across.x - start.x, across.y - start.y
            length = math.hypot(dx, dy)
            midpoint = length / 2.0
            for index in range(math.ceil(midpoint / spacing)):
                distances = ((midpoint,) if index == 0 else
                             (midpoint - index * spacing, midpoint + index * spacing))
                for distance in distances:
                    if min(distance, length - distance) <= self.config.goal_xy_tol_m:
                        continue
                    fraction = distance / length
                    x, y = start.x + fraction * dx, start.y + fraction * dy
                    yield (outer, FormationPose(x, y, start.psi),
                           middle, FormationPose(x + cross_x, y + cross_y, start.psi))

    def crossing_samples_clear(self, start, end, headings):
        """Reject a blocked sampled placement before paying for a full sweep."""
        dx, dy = end.x - start.x, end.y - start.y
        count = max(1, math.ceil(math.hypot(dx, dy) / self.config.translation_step_m))
        for index in range(count + 1):
            fraction = index / count
            pose = FormationPose(start.x + fraction * dx, start.y + fraction * dy, start.psi)
            if not self.checker.static_check(pose, headings).success:
                return False
        return True

    def straight_preparations(self, state, goal, goal_headings):
        """Try stopped A alignment along approach rays on the map grid axes."""
        start = state.qf
        if abs(_wrap(start.psi - goal.psi)) > self.config.goal_psi_tol_rad:
            return
        current = state.heading_map()
        preferred = (dict(goal_headings)
                     if goal_headings and not self.config.goal_heading_free else current)
        axes = ((goal.x - start.x, 0.0), (0.0, goal.y - start.y))
        spacing = max(0.5 * self.config.translation_step_m, self.config.goal_xy_tol_m)
        for dx, dy in sorted(axes, key=lambda axis: -math.hypot(*axis)):
            length = math.hypot(dx, dy)
            if length <= self.config.goal_xy_tol_m:
                continue
            tangent = math.atan2(dy, dx)
            directions = {
                i: -1 if self.config.allow_reverse and math.cos(preferred[i] - tangent) < 0.0
                else 1 for i in self.vehicle_ids}
            target = a_straight_required_headings(self.model, (dx, dy), directions)
            if _headings_match(current, target):
                continue
            endpoint = FormationPose(goal.x, goal.y, start.psi)
            if not self.is_goal(replace(state, qf=endpoint, headings=tuple(target.items())),
                                goal, goal_headings):
                continue
            if not self.checker.static_check(endpoint, target).success:
                continue
            for index in range(math.ceil((length - self.config.goal_xy_tol_m) / spacing)):
                remaining = 1.0 - index * spacing / length
                yield FormationPose(goal.x - remaining * dx, goal.y - remaining * dy,
                                    start.psi), target

    def generate_spin_preparation(self, state, goal, target):
        started = time.perf_counter()
        result = SuccessorResult()

        def consider(label, edge):
            self._consider(state, goal, label, edge, result)

        self._add_s(state, state.heading_map(), "S_prepare_A_goal", target, consider, result)
        result.generation_s = time.perf_counter() - started - result.collision_s
        return result

    def arc_goal_connectors(self, state, goal_qf, *, delta_psi=None):
        """Yield one swept exact-pose arc candidate at a time, with S if needed."""
        start = state.qf
        turn = _wrap(goal_qf.psi - start.psi)
        if abs(turn) <= self.config.goal_psi_tol_rad:
            return
        cosine, sine = math.cos(start.psi), math.sin(start.psi)
        dx, dy = goal_qf.x - start.x, goal_qf.y - start.y
        local_x, local_y = cosine * dx + sine * dy, -sine * dx + cosine * dy
        if delta_psi is not None:
            turns = (delta_psi,)
        else:
            turns = ((turn, turn - math.copysign(math.tau, turn))
                     if abs(abs(turn) - math.pi) <= 1e-9 else (turn,))
        current = state.heading_map()
        for delta in turns:
            generation_start = time.perf_counter()
            # local displacement = (I - R(delta)) * pivot, solved using
            # the half angle to avoid cancellation for shallow turns.
            half_cot = 0.5 / math.tan(delta / 2.0)
            pivot = (0.5 * local_x - half_cot * local_y,
                     half_cot * local_x + 0.5 * local_y)
            native_branches = None
            if self._native_candidates is not None:
                native_branches = self._native_candidates.goal_arc_branches(
                    start.psi, self._offsets, tuple(current[i] for i in self.vehicle_ids),
                    pivot, delta, self.motion_limits.r_min_ack, self.config.allow_reverse)
            if native_branches is not None:
                gear_branches = [(gears, dict(zip(self.vehicle_ids, values)))
                                 for gears, values in native_branches]
            else:
                forward = dict.fromkeys(self.vehicle_ids, 1)
                try:
                    tangents = a_arc_required_headings(
                        self.model, start, delta, pivot, forward, limits=self.motion_limits)
                except UniformMotionError as exc:
                    result = SuccessorResult(candidate_count=1,
                                             generation_s=time.perf_counter() - generation_start)
                    result.reject("A_arc_direct_goal", exc.reason, exc.detail)
                    yield result
                    continue

                scored = []
                for gears in _gear_combinations(self.vehicle_ids, self.config.allow_reverse):
                    target = {i: tangents[i] + (math.pi if gear < 0 else 0.0)
                              for i, gear in zip(self.vehicle_ids, gears)}
                    spins = [abs(_wrap(target[i] - current[i])) for i in self.vehicle_ids]
                    scored.append(((max(spins), sum(spins)), gears, target))
                gear_branches = [(gears, target)
                                 for _, gears, target in sorted(scored, key=lambda branch: branch[0])]

            for gears, target in gear_branches:
                result = SuccessorResult()
                arc_start = state
                branch = gear_label(gears)
                if not _headings_match(current, target):
                    def consider_spin(label, edge):
                        self._consider(state, goal_qf, label, edge, result)

                    self._add_s(state, current, "S_to_A_arc_direct_goal_gear" + branch,
                                target, consider_spin, result)
                    arc_start = (result.successors.pop().next_state
                                 if result.successors else None)
                if arc_start is not None:
                    label = "A_arc_direct_goal" if arc_start is state else "S_then_A_arc_direct_goal"
                    try:
                        edge = a_arc_edge(
                            self.model, arc_start.qf, arc_start.heading_map(), delta, pivot,
                            dict(zip(self.vehicle_ids, gears)), limits=self.motion_limits,
                            allow_reverse=self.config.allow_reverse)
                    except UniformMotionError as exc:
                        result.candidate_count += 1
                        result.reject(label + "_gear" + branch, exc.reason, exc.detail)
                    else:
                        self._consider(arc_start, goal_qf, label, edge, result)
                result.generation_s = time.perf_counter() - generation_start - result.collision_s
                yield result
                if any(rejection["label"] in ("A_arc_direct_goal", "S_then_A_arc_direct_goal")
                       and rejection["reason"] in (
                           "MAP_OBB_COLLISION", "VEHICLE_OBB_COLLISION", "CARGO_OBB_COLLISION")
                       for rejection in result.rejections):
                    # Gear branches change the body heading by pi. The arc's
                    # centres and rectangle sweep are identical for all gears.
                    break
                generation_start = time.perf_counter()

    # -- goal test --------------------------------------------------------

    def is_goal(self, state, goal_qf, goal_headings=None):
        """Test position and the requested terminal heading policies."""
        if math.hypot(state.qf.x - goal_qf.x,
                      state.qf.y - goal_qf.y) > self.config.goal_xy_tol_m:
            return False
        if (not self.config.goal_formation_heading_free and
                abs(_wrap(state.qf.psi - goal_qf.psi)) > self.config.goal_psi_tol_rad):
            return False
        if self.config.goal_heading_free:
            return True
        if goal_headings is None:
            return True
        current = state.heading_map()
        goals = dict(goal_headings)
        return all(
            abs(_wrap(current[vehicle_id] - goals[vehicle_id]))
            <= self.config.goal_heading_tol_rad
            for vehicle_id in current)

    # -- candidate libraries ---------------------------------------------

    def _goal_direction(self, state, goal_qf):
        return goal_direction(state.qf, goal_qf, self.config)

    def _o_displacements(self, state, goal_qf):
        """Eight cargo-frame directions plus the goal direction."""
        step = self.config.translation_step_m
        cosine = math.cos(state.qf.psi)
        sine = math.sin(state.qf.psi)
        seen = set()
        result = []
        for index in range(8):
            angle = index * math.pi / 4.0
            cargo_dx = math.cos(angle)
            cargo_dy = math.sin(angle)
            dx = step * (cosine * cargo_dx - sine * cargo_dy)
            dy = step * (sine * cargo_dx + cosine * cargo_dy)
            key = (round(dx, 9), round(dy, 9))
            if key in seen:
                continue
            seen.add(key)
            result.append(("O_cargo_{:02d}".format(index), (dx, dy)))
        goal = self._goal_direction(state, goal_qf)
        if goal is not None:
            dx, dy, distance = goal
            length = min(step, distance)
            scale = length / distance
            gdx, gdy = dx * scale, dy * scale
            key = (round(gdx, 9), round(gdy, 9))
            if key not in seen:
                seen.add(key)
                result.append(("O_goal", (gdx, gdy)))
        return result

    def _a_straight_displacements(self, state, goal_qf):
        goal = self._goal_direction(state, goal_qf)
        if goal is None:
            return []
        dx, dy, distance = goal
        step = self.config.translation_step_m
        length = min(step, distance)
        scale = length / distance
        return [("A_straight_goal", (dx * scale, dy * scale))]

    def _a_arc_schemes(self, state, goal_qf):
        """Fixed +/-10 deg library plus the goal-cropped angle.

        DECISIONS section 5 fixes the pivots and the 10 deg step and permits a
        cropped angle near the goal heading.  The crop is the exact remaining
        cargo-heading difference, so the goal heading is reachable without
        silently changing the exploration step.  At exactly pi both rotation
        directions are offered because their sweeps can differ.
        """
        return a_arc_schemes(state.qf, goal_qf, self.config)

    # -- O / A / S construction ------------------------------------------

    def _add_o(self, state, current, label, displacement, consider, result):
        dx, dy = displacement
        if math.hypot(dx, dy) <= 0.0:
            return
        direction_angle = math.atan2(dy, dx)
        directions = {}
        obliques = {}
        for vehicle_id in self.vehicle_ids:
            branch = _o_branch(
                current[vehicle_id], direction_angle, self.config.allow_reverse)
            if branch is None:
                result.candidate_count += 1
                result.reject(
                    label, "O_BRANCH_REQUIRES_REVERSE",
                    "vehicle_id={} direction={:.6g}".format(
                        vehicle_id, direction_angle))
                return
            sigma, phi = branch
            directions[vehicle_id] = sigma
            obliques[vehicle_id] = phi
        try:
            edge = o_translation_edge(
                self.model, state.qf, current, (dx, dy), directions, obliques,
                limits=self.motion_limits, allow_reverse=self.config.allow_reverse)
        except UniformMotionError as exc:
            result.candidate_count += 1
            result.reject(label, exc.reason, exc.detail)
            return
        consider("{}_sig{}".format(
            label, "".join("+" if directions[v] > 0 else "-"
                           for v in self.vehicle_ids)), edge)

    def _add_a_straight(self, state, current, label, displacement,
                        consider, result):
        dx, dy = displacement
        if math.hypot(dx, dy) <= 0.0:
            return
        for gears in _gear_combinations(self.vehicle_ids,
                                        self.config.allow_reverse):
            directions = dict(zip(self.vehicle_ids, gears))
            gear_label = "".join("+" if g > 0 else "-" for g in gears)
            full_label = "{}_gear{}".format(label, gear_label)
            required = a_straight_required_headings(
                self.model, (dx, dy), directions)
            if not _headings_match(current, required):
                # Not tangent-aligned: this scheme is realised through an S
                # alignment successor instead of an illegal direct A edge.
                result.candidate_count += 1
                result.reject(
                    full_label, "A_HEADING_REQUIRES_S_ALIGNMENT",
                    "body headings are not the required tangent/anti-tangent")
                continue
            try:
                edge = a_straight_edge(
                    self.model, state.qf, current, (dx, dy), directions,
                    limits=self.motion_limits,
                    allow_reverse=self.config.allow_reverse)
            except UniformMotionError as exc:
                result.candidate_count += 1
                result.reject(full_label, exc.reason, exc.detail)
                continue
            consider(full_label, edge)

    def _add_a_arc(self, state, current, label, delta_psi, pivot,
                   consider, result):
        for gears in _gear_combinations(self.vehicle_ids,
                                        self.config.allow_reverse):
            directions = dict(zip(self.vehicle_ids, gears))
            gear_label = "".join("+" if g > 0 else "-" for g in gears)
            full_label = "{}_gear{}".format(label, gear_label)
            try:
                required = a_arc_required_headings(
                    self.model, state.qf, delta_psi, pivot, directions,
                    limits=self.motion_limits)
            except UniformMotionError as exc:
                result.candidate_count += 1
                result.reject(full_label, exc.reason, exc.detail)
                continue
            if not _headings_match(current, required):
                result.candidate_count += 1
                result.reject(
                    full_label, "A_HEADING_REQUIRES_S_ALIGNMENT",
                    "body headings are not the required arc tangent")
                continue
            try:
                edge = a_arc_edge(
                    self.model, state.qf, current, delta_psi, pivot, directions,
                    limits=self.motion_limits,
                    allow_reverse=self.config.allow_reverse)
            except UniformMotionError as exc:
                result.candidate_count += 1
                result.reject(full_label, exc.reason, exc.detail)
                continue
            consider(full_label, edge)

    def _s_alignment_targets(self, state, goal_qf):
        """Deduplicated required-heading sets demanded by successor A schemes."""
        targets = {}
        current = state.heading_map()

        def register(label, required):
            key = tuple(sorted(
                (vehicle_id, round(_wrap(theta), 9))
                for vehicle_id, theta in required.items()))
            if key not in targets:
                targets[key] = (label, required)

        for label, displacement in self._a_straight_displacements(state, goal_qf):
            dx, dy = displacement
            if math.hypot(dx, dy) <= 0.0:
                continue
            for gears in _gear_combinations(self.vehicle_ids,
                                            self.config.allow_reverse):
                directions = dict(zip(self.vehicle_ids, gears))
                required = a_straight_required_headings(
                    self.model, (dx, dy), directions)
                gear_label = "".join("+" if g > 0 else "-" for g in gears)
                register("S_to_{}_gear{}".format(label, gear_label), required)

        for label, delta_psi, pivot in self._a_arc_schemes(state, goal_qf):
            for gears in _gear_combinations(self.vehicle_ids,
                                            self.config.allow_reverse):
                directions = dict(zip(self.vehicle_ids, gears))
                try:
                    required = a_arc_required_headings(
                        self.model, state.qf, delta_psi, pivot, directions,
                        limits=self.motion_limits)
                except UniformMotionError:
                    continue
                gear_label = "".join("+" if g > 0 else "-" for g in gears)
                register("S_to_{}_gear{}".format(label, gear_label), required)

        ordered = []
        for key in sorted(targets):
            label, required = targets[key]
            ordered.append((label, required))
        return ordered

    def _add_s(self, state, current, label, target, consider, result):
        if self._native_candidates is None:
            deltas = {i: _wrap(target[i] - current[i]) for i in self.vehicle_ids}
            if not any(abs(delta) > 1e-12 for delta in deltas.values()):
                return
        else:
            values = self._native_candidates.spin_increments(
                tuple(current[i] for i in self.vehicle_ids), tuple(target[i] for i in self.vehicle_ids))
            if values is None:
                return
            deltas = dict(zip(self.vehicle_ids, values))
        try:
            edge = s_spin_edge(
                self.model, state.qf, current, deltas, limits=self.motion_limits)
        except UniformMotionError as exc:
            result.candidate_count += 1
            result.reject(label, exc.reason, exc.detail)
            return
        consider(label, edge)


# ---------------------------------------------------------------------------
# Continuity helper (acceptance threshold 1e-7 m / rad)
# ---------------------------------------------------------------------------


def continuity_error(state, successor):
    """Max |edge.start - state| over q_F (m) and body headings (rad)."""
    edge = successor.edge
    position = math.hypot(edge.start_qf.x - state.qf.x,
                          edge.start_qf.y - state.qf.y)
    psi = abs(_wrap(edge.start_qf.psi - state.qf.psi))
    current = state.heading_map()
    start = dict(edge.start_headings)
    heading = max(
        (abs(_wrap(start[vehicle_id] - current[vehicle_id]))
         for vehicle_id in current), default=0.0)
    return {"position_m": position, "psi_rad": psi, "heading_rad": heading}


# ---------------------------------------------------------------------------
# FR04: budgeted autonomous graph search (no ROS, no NLP, no legacy seed)
# ---------------------------------------------------------------------------

SEARCH_SUCCESS = "SUCCESS"
SEARCH_NO_MOTION = "NO_MOTION"
SEARCH_NO_SOLUTION = "NO_SOLUTION"
SEARCH_BUDGET_EXHAUSTED = "BUDGET_EXHAUSTED"
SEARCH_AUDIT_REJECTED = "AUDIT_REJECTED"
SEARCH_STATUSES = (
    SEARCH_SUCCESS, SEARCH_NO_MOTION, SEARCH_NO_SOLUTION,
    SEARCH_BUDGET_EXHAUSTED, SEARCH_AUDIT_REJECTED)


@dataclass(frozen=True)
class GraphSearchConfig:
    """Central FR04 budget and heuristic configuration.

    The first-round fixed budget is 10000 expansions / 30 seconds.  The
    heuristic is deliberately only a search guide: the FR03 proxy cost is not
    admissible, so neither weighted A* nor the rotation weight is an
    optimality claim.  ``rotation_weight`` prevents a necessary nonzero-pivot
    goal rotation from being postponed behind unlimited zero-translation
    heading/translation alternatives merely because its arc temporarily
    increases cargo XY distance.
    """

    max_expansions: int = 10000
    max_seconds: float = 30.0
    heuristic_weight: float = 3.0
    rotation_weight: float = 3.0
    direct_goal_translation: bool = False
    global_translation_route: bool = True
    heading_aligned_route: bool = True
    heading_transition_route: bool = True
    continuous_a_refinement: bool = False
    formation_raster: bool = True
    preview_checks: bool = True
    native_frontier: bool = False
    convex_corridor: bool = False
    prefer_ackermann: bool = False

    def __post_init__(self):
        if not isinstance(self.native_frontier, bool):
            raise FormationValidationError("INVALID_SEARCH_CONFIG native_frontier must be bool")
        if not isinstance(self.direct_goal_translation, bool):
            raise FormationValidationError(
                "INVALID_SEARCH_CONFIG direct_goal_translation must be bool")
        if not isinstance(self.global_translation_route, bool):
            raise FormationValidationError(
                "INVALID_SEARCH_CONFIG global_translation_route must be bool")
        if not isinstance(self.heading_aligned_route, bool):
            raise FormationValidationError(
                "INVALID_SEARCH_CONFIG heading_aligned_route must be bool")
        if not isinstance(self.heading_transition_route, bool):
            raise FormationValidationError(
                "INVALID_SEARCH_CONFIG heading_transition_route must be bool")
        if not isinstance(self.continuous_a_refinement, bool):
            raise FormationValidationError(
                "INVALID_SEARCH_CONFIG continuous_a_refinement must be bool")
        for name in ("formation_raster", "preview_checks", "prefer_ackermann"):
            if not isinstance(getattr(self, name), bool):
                raise FormationValidationError(
                    "INVALID_SEARCH_CONFIG {} must be bool".format(name))
        if isinstance(self.max_expansions, bool) or not isinstance(
                self.max_expansions, int) or self.max_expansions < 0:
            raise FormationValidationError(
                "INVALID_SEARCH_BUDGET max_expansions must be a nonnegative int")
        for name in ("max_seconds", "heuristic_weight", "rotation_weight"):
            value = _finite(getattr(self, name), "GraphSearchConfig." + name)
            if value <= 0.0:
                raise FormationValidationError(
                    "INVALID_SEARCH_CONFIG {} must be positive".format(name))


@dataclass
class UniformGraphSearchResult:
    """Search outcome; ``SUCCESS`` requires the terminal validator to accept."""

    status: str
    reason: str
    start_state: UniformSearchState
    goal_qf: FormationPose
    goal_state: Optional[UniformSearchState] = None
    expansions: int = 0
    candidate_count: int = 0
    accepted_successors: int = 0
    duplicate_key_rejections: int = 0
    redundant_s_pruned: int = 0
    stale_heap_entries: int = 0
    open_size: int = 0
    wall_time_s: float = 0.0
    generation_s: float = 0.0
    collision_s: float = 0.0
    rejection_histogram: Dict[str, int] = field(default_factory=dict)
    terminal_rejection_histogram: Dict[str, int] = field(default_factory=dict)
    terminal_audits: List[dict] = field(default_factory=list)
    direct_goal_translation: Optional[str] = None
    offset_preview_probes: int = 0
    offset_preview_rejections: int = 0
    straight_preparation_probes: int = 0
    global_route: Optional[dict] = None
    corridor_guide: Optional[dict] = None
    heading_transition_probes: int = 0
    first_failure_layer: str = "none"
    detail: str = ""
    frontier_backend: str = "python"

    @property
    def edges(self):
        return () if self.goal_state is None else reconstruct_path(
            self.goal_state)

    def as_dict(self, include_path=True):
        result = {
            "status": self.status,
            "reason": self.reason,
            "detail": self.detail,
            "first_failure_layer": self.first_failure_layer,
            "frontier_backend": self.frontier_backend,
            "goal_qf": [self.goal_qf.x, self.goal_qf.y, self.goal_qf.psi],
            "expansions": self.expansions,
            "candidate_count": self.candidate_count,
            "accepted_successors": self.accepted_successors,
            "duplicate_key_rejections": self.duplicate_key_rejections,
            "redundant_s_pruned": self.redundant_s_pruned,
            "stale_heap_entries": self.stale_heap_entries,
            "open_size": self.open_size,
            "wall_time_s": self.wall_time_s,
            "generation_s": self.generation_s,
            "collision_s": self.collision_s,
            "rejection_histogram": dict(self.rejection_histogram),
            "terminal_rejection_histogram": dict(
                self.terminal_rejection_histogram),
            "terminal_audits": list(self.terminal_audits),
            "direct_goal_translation": self.direct_goal_translation,
            "offset_preview_probes": self.offset_preview_probes,
            "offset_preview_rejections": self.offset_preview_rejections,
            "straight_preparation_probes": self.straight_preparation_probes,
            "global_route": self.global_route,
            "heading_transition_probes": self.heading_transition_probes,
            "goal_state": (None if self.goal_state is None
                           else self.goal_state.as_dict()),
        }
        if include_path:
            result["path"] = path_records(self.goal_state)
        if self.corridor_guide is not None:
            result["corridor_guide"] = self.corridor_guide
        return result


def reconstruct_path(state):
    """Return the exact authoritative edges from the root to ``state``."""
    if state is None:
        return ()
    edges = []
    current = state
    while current is not None and current.edge is not None:
        edges.append(current.edge)
        current = current.parent
    return tuple(reversed(edges))


def path_records(state):
    """Primitive signatures and cumulative proxy cost along a parent chain."""
    if state is None:
        return []
    states = []
    current = state
    while current is not None:
        states.append(current)
        current = current.parent
    states.reverse()
    records = []
    for index, node in enumerate(states[1:], 1):
        edge = node.edge
        records.append({
            "index": index,
            "primitive": edge.primitive,
            "mode": edge.mode,
            "delta_psi": edge.control.delta_psi,
            "pivot": [edge.control.pivot_x, edge.control.pivot_y],
            "parameter_length": edge.control.parameter_length,
            "directions": {str(vehicle_id): int(value)
                           for vehicle_id, value in edge.directions},
            "start_qf": [edge.start_qf.x, edge.start_qf.y, edge.start_qf.psi],
            "end_qf": [edge.end_qf.x, edge.end_qf.y, edge.end_qf.psi],
            "start_headings": {str(vehicle_id): theta
                               for vehicle_id, theta in edge.start_headings},
            "end_headings": {str(vehicle_id): theta
                             for vehicle_id, theta in edge.end_headings},
            "cumulative_proxy_g_s": node.g,
            "a_labels": list(node.a_labels),
        })
    return records


def goal_heuristic_s(state, goal_qf, successor_config, search_config):
    """Weighted first-order goal distance in seconds (not a lower bound)."""
    limits = successor_config.timing_limits
    distance_s = math.hypot(
        goal_qf.x - state.qf.x, goal_qf.y - state.qf.y) / limits.v_max
    if successor_config.goal_formation_heading_free:
        return distance_s
    rotation_s = abs(_wrap(goal_qf.psi - state.qf.psi)) / limits.yaw_rate_max
    return distance_s + search_config.rotation_weight * rotation_s


class UniformGraphSearch:
    """FR04 open/representative/parent search over FR03 exact successors.

    Consecutive S alignment edges are skipped.  This is a redundancy
    elimination, not a new primitive restriction: every S target depends only
    on ``q_F`` and the goal, and FR01 can construct one S edge from the first
    edge's headings directly to the second edge's target (including zero-angle
    vehicles).  Thus an S->S chain never unlocks an A scheme that the first
    state could not unlock with one S edge.

    ``terminal_validator(edges)`` returns ``(accepted, serialisable_detail)``.
    The FR04 pipeline supplies the real FR02 timing+audit there.  A rejected
    terminal path is recorded and search continues finitely; an unvalidated
    path is never returned as SUCCESS.
    """

    def __init__(self, model, checker, successor_config=None,
                 search_config=None):
        self.model = model
        self.checker = checker
        self.successor_config = successor_config or SuccessorConfig()
        self.search_config = search_config or GraphSearchConfig()
        self.generator = SuccessorGenerator(
            model, checker, self.successor_config,
            native_spin_sweep=self.search_config.native_frontier,
            native_candidates=self.search_config.native_frontier)

    def search(self, start_state, goal_qf, terminal_validator=None,
               goal_headings=None, cancelled=None, progress=None):
        started = time.perf_counter()
        config = self.search_config
        result = UniformGraphSearchResult(
            status=SEARCH_NO_SOLUTION, reason=SEARCH_NO_SOLUTION,
            start_state=start_state, goal_qf=goal_qf,
            frontier_backend="cpp" if config.native_frontier else "python")
        frontier = None
        is_cancelled = cancelled or (lambda: False)
        native = self.generator._native_candidates
        budget = (native.SearchBudget(config.max_expansions, config.max_seconds)
                  if native is not None else None)

        def finish(status, reason, layer, detail="", goal_state=None):
            result.status = status
            result.reason = reason
            result.first_failure_layer = layer
            result.detail = detail
            result.goal_state = goal_state
            result.open_size = 0 if frontier is None else len(frontier)
            result.wall_time_s = time.perf_counter() - started
            return result

        if self.generator.is_goal(start_state, goal_qf, goal_headings):
            return finish(SEARCH_NO_MOTION, SEARCH_NO_MOTION, "none",
                          "the exact start already satisfies the goal test",
                          start_state)

        def record_generation(generated):
            result.candidate_count += generated.candidate_count
            result.generation_s += generated.generation_s
            result.collision_s += generated.collision_s
            for reason, count in generated.rejection_histogram().items():
                result.rejection_histogram[reason] = (
                    result.rejection_histogram.get(reason, 0) + count)
            if progress is not None:
                progress(result.expansions, result.candidate_count)

        deadline = started + config.max_seconds

        def translate(state, mode, target, try_until):
            if time.perf_counter() >= try_until:
                return None
            generated = self.generator.generate(
                state, target, direct_goal_translation=True, translation_mode=mode)
            record_generation(generated)
            if not generated.successors:
                return None
            # Fixed A/O displacements and headings determine a unique branch.
            last, = generated.successors
            return last

        def follow_translation(legs, try_until, origin=start_state):
            state = origin
            for mode, target in legs:
                last = translate(state, mode, target, try_until)
                if last is None:
                    return None
                state = last.next_state
            label = "_then_".join(mode for mode, _ in legs) + "_direct_goal"
            return replace(last, label=label)

        raster_cache = {}

        def global_translations(origin, target, try_until, *, record_route):
            start_qf = origin.qf
            headings = origin.heading_map()
            opts = self.successor_config
            if (is_cancelled() or time.perf_counter() >= try_until
                    or math.hypot(target.x - start_qf.x, target.y - start_qf.y) <= opts.translation_step_m
                    or not self.checker.static_check(target, headings).success):
                return
            layer = (start_qf.psi, origin.headings)
            if not config.formation_raster:
                blocked = self.checker.grid.occ
            else:
                if layer not in raster_cache:
                    raster_cache[layer] = self.checker.fixed_heading_blocked_cells(
                        start_qf.psi, headings, padding_m=opts.sweep_spacing_m)
                blocked = raster_cache[layer]
            angle = math.remainder(headings[self.generator.vehicle_ids[0]], math.pi / 2)
            frames = (0.0, angle) if config.heading_aligned_route and abs(angle) > 1e-9 else (0.0,)
            candidates = []
            diagnostics = []

            def direction_steps(frame):
                return tuple((dy, dx) for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0))
                             if all(_o_branch(theta, frame + math.atan2(dy, dx),
                                              opts.allow_reverse) is not None
                                    for theta in headings.values()))

            def direction_allowed(first, second):
                dx, dy = second[0] - first[0], second[1] - first[1]
                if math.hypot(dx, dy) <= opts.goal_xy_tol_m:
                    return False
                bearing = math.atan2(dy, dx)
                return all(_o_branch(theta, bearing, opts.allow_reverse) is not None
                           for theta in headings.values())

            def legs_for_points(points):
                legs = []
                length = a_distance = 0.0
                for a, b in zip(points, points[1:]):
                    dx, dy = b[0] - a[0], b[1] - a[1]
                    bearing = math.atan2(dy, dx)
                    tangent = all(abs(_wrap(theta - bearing)) <= HEADING_MATCH_TOL_RAD
                                  or (opts.allow_reverse and abs(_wrap(theta - bearing + math.pi))
                                      <= HEADING_MATCH_TOL_RAD) for theta in headings.values())
                    travel = math.hypot(dx, dy)
                    length += travel
                    if tangent:
                        a_distance += travel
                    legs.append((MODE_A if tangent else MODE_O, FormationPose(*b, start_qf.psi)))
                return legs, length, a_distance

            if config.convex_corridor and config.formation_raster:
                began = time.perf_counter()
                corridor_deadline = began + min(.5, .20 * max(0.0, try_until - began))
                grid = self.checker.grid
                corridor = convex_corridor(
                    grid, blocked, (start_qf.x, start_qf.y), (target.x, target.y), direction_steps(0.0),
                    deadline=corridor_deadline, max_expansions=config.max_expansions, cancelled=cancelled)
                record = {
                    "status": corridor.status, "expansions": corridor.expansions, "frame_angle_rad": 0.0,
                    "wall_time_s": time.perf_counter() - began,
                    "waypoints": [list(p) for p in corridor.points],
                    "convex_regions_m": [[grid.ox + r.left * grid.res, grid.oy + r.bottom * grid.res,
                                          grid.ox + r.right * grid.res, grid.oy + r.top * grid.res]
                                         for r in corridor.rectangles],
                }
                diagnostics.append(record)
                if corridor.points:
                    def stop_shortcuts():
                        return is_cancelled() or time.perf_counter() >= corridor_deadline

                    shorter = shortcut_corridor(grid, corridor, direction_allowed, stop_shortcuts)
                    paths = [(False, corridor.points)]
                    if shorter != corridor.points:
                        paths.append((True, shorter))
                    routes = []
                    for shortcut, points in paths:
                        legs, length, a_distance = legs_for_points(points)
                        details = dict(record, length_m=length, a_distance_m=a_distance,
                                       uses_shortcuts=shortcut, waypoints=[list(point) for point in points])
                        last = follow_translation(legs, try_until, origin)
                        if last is not None:
                            routes.append((last.next_state.g, details, last))
                    for _, details, last in sorted(routes, key=lambda candidate: candidate[0]):
                        if record_route:
                            result.global_route = details
                        result.corridor_guide = details
                        yield replace(last, label="convex_corridor_global_route")

            def probe(frame, raster, alternative=False):
                began = time.perf_counter()
                if began >= try_until:
                    return
                steps = direction_steps(frame)
                expansions = 0
                for spacing in (2 * opts.xy_bin_m, opts.xy_bin_m):
                    route = fixed_heading_route(
                        self.checker.grid, raster, (start_qf.x, start_qf.y), (target.x, target.y), steps,
                        grid_step_m=spacing, turn_cost_m=opts.translation_step_m,
                        deadline=try_until, max_expansions=config.max_expansions - expansions,
                        frame_angle_rad=frame, cancelled=cancelled)
                    expansions += route.expansions
                    if route.status != "NO_ROUTE_IN_RASTER":
                        break
                record = {"status": route.status, "expansions": expansions, "frame_angle_rad": frame,
                          "grid_step_target_m": spacing, "wall_time_s": time.perf_counter() - began,
                          "waypoints": [list(p) for p in route.points]}
                if alternative:
                    record["avoids_first_route"] = True
                diagnostics.append(record)
                if not route.points:
                    return
                legs, length, a_distance = legs_for_points(route.points)
                score = length + opts.translation_step_m * (len(legs) - 1)
                record.update(length_m=length, a_distance_m=a_distance, score_m=score)
                return (round(score, 9), -a_distance), record, legs

            for frame in frames:
                candidate = probe(frame, blocked)
                if candidate is not None:
                    candidates.append(candidate)
            if record_route and not candidates and diagnostics:
                result.global_route = dict(diagnostics[-1], alternatives=diagnostics)
            candidates.sort(key=lambda item: item[0])
            for _, record, legs in candidates:
                if record_route:
                    result.global_route = dict(record, alternatives=diagnostics)
                last = follow_translation(legs, try_until, origin)
                if last is not None:
                    yield replace(last, label="fixed_heading_global_route")
            if record_route and candidates and time.perf_counter() < try_until:
                # If the first topology reached the raster goal but failed
                # primitive reconstruction or certification, propose a route
                # that avoids a patch in its interior. The real map and the
                # final sweep remain untouched.
                _, first, _ = candidates[0]
                points = first["waypoints"]
                span = max(zip(points, points[1:]), key=lambda pair: math.dist(*pair))
                grid = self.checker.grid
                x, y = ((span[0][index] + span[1][index]) / 2 for index in (0, 1))
                radius = opts.translation_step_m / 2
                if min(math.hypot(x - p[0], y - p[1]) for p in (points[0], points[-1])) > 2 * radius:
                    masked = blocked.copy()
                    c0, c1 = (math.floor((xx - grid.ox) / grid.res) for xx in (x - radius, x + radius))
                    r0, r1 = (math.floor((yy - grid.oy) / grid.res) for yy in (y - radius, y + radius))
                    masked[max(0, r0):r1 + 1, max(0, c0):c1 + 1] = True
                    alternative = probe(first["frame_angle_rad"], masked, alternative=True)
                    if alternative is not None:
                        _, record, legs = alternative
                        result.global_route = dict(record, alternatives=diagnostics)
                        last = follow_translation(legs, try_until, origin)
                        if last is not None:
                            yield replace(last, label="fixed_heading_alternate_topology")

        def complete_headings(state, try_until):
            if self.generator.is_goal(state, goal_qf, goal_headings):
                return state
            if (goal_headings is None or time.perf_counter() >= try_until
                    or not self.generator.is_goal(
                        replace(state, headings=tuple(goal_headings.items())), goal_qf, goal_headings)):
                return None
            spin = self.generator.generate_spin_preparation(state, goal_qf, goal_headings)
            record_generation(spin)
            return spin.successors[0].next_state if spin.successors else None

        def candidate_from_state(state, label):
            setup_s = setup_demand_s(state.parent.wheel, state.edge, self.generator.motion_limits)
            proxy_s = state.g - state.parent.g - setup_s
            return Successor(state.edge, state, setup_s, proxy_s, setup_s + proxy_s, label)

        def translation_to(origin, target, try_until):
            if time.perf_counter() >= try_until:
                return None
            dx, dy = target.x - origin.qf.x, target.y - origin.qf.y
            opts = self.successor_config
            if math.hypot(dx, dy) <= opts.goal_xy_tol_m:
                return origin
            headings = origin.heading_map()
            if not config.preview_checks or self.generator.crossing_samples_clear(origin.qf, target, headings):
                tangents = a_straight_required_headings(self.model, (dx, dy),
                                                        dict.fromkeys(headings, 1))
                gears = _aligned_gears(headings, tangents, self.generator.vehicle_ids,
                                       opts.allow_reverse)
                leg = translate(origin, MODE_A if gears is not None else MODE_O, target, try_until)
                if leg is not None:
                    return leg.next_state
            route = next(global_translations(origin, target, try_until, record_route=False), None)
            return None if route is None else route.next_state

        def turn_preview(entry, delta, pivot, target, allow_realign):
            # All gear branches occupy the same rectangles during the turn.
            try:
                headings = a_arc_required_headings(
                    self.model, entry, delta, pivot, dict.fromkeys(self.generator.vehicle_ids, 1),
                    limits=self.generator.motion_limits)
            except UniformMotionError:
                return False
            c, s = math.cos(entry.psi), math.sin(entry.psi)
            px, py = c * pivot[0] - s * pivot[1], s * pivot[0] + c * pivot[1]
            for index in range(5):
                offset = delta * index / 4
                co, si = math.cos(offset), math.sin(offset)
                pose = FormationPose(entry.x + px - co * px + si * py,
                                     entry.y + py - si * px - co * py, entry.psi + offset)
                if not self.checker.static_check(
                        pose, {i: theta + offset for i, theta in headings.items()}).success:
                    return False
            if allow_realign:
                return True
            final_headings = {i: theta + delta for i, theta in headings.items()}
            return self.checker.static_check(target, final_headings).success

        def heading_routes(target, try_until, *, compact_turns=False):
            start = start_state.qf
            base_delta = _wrap(target.psi - start.psi)
            opts = self.successor_config
            if abs(base_delta) <= opts.goal_psi_tol_rad:
                return
            deltas = (base_delta, base_delta - math.copysign(math.tau, base_delta))
            if abs(base_delta) < math.radians(150):
                deltas = deltas[:1]
            min_radius = (self.generator.motion_limits.r_min_ack
                          + max(math.hypot(o.x, o.y) for o in self.model.offsets))
            step = opts.translation_step_m
            dx, dy = target.x - start.x, target.y - start.y
            direction = (math.cos(start.psi), math.sin(start.psi))
            # Candidate entry points lie near both corridor ends and on
            # approach rays. Each arc connects two exact cargo heading layers.
            positions = [(start.x, start.y), (target.x, target.y)]
            positions += [(start.x + f * dx, start.y + f * dy) for f in (0.25, 0.5, 0.75)]
            positions += [(x + offset * step * direction[0], y + offset * step * direction[1])
                          for x, y in positions[:2] for offset in (-8, -4, -2, -1, 1, 2, 4, 8, 12, 16)]
            proposals = []
            seen = set()
            for delta in deltas:
                pivots = list(opts.arc_pivots) if compact_turns else []
                for gear in ((1, -1) if opts.allow_reverse else (1,)):
                    lateral = math.copysign(1.0, delta) * gear
                    unit_x = gear * math.sin(abs(delta))
                    unit_y = lateral * (1.0 - math.cos(delta))
                    ux = direction[0] * unit_x - direction[1] * unit_y
                    uy = direction[1] * unit_x + direction[0] * unit_y
                    fit_radius = (ux * dx + uy * dy) / (ux * ux + uy * uy)
                    radii = [min_radius, min_radius + step, min_radius + 2 * step]
                    if fit_radius > min_radius and fit_radius <= (math.hypot(dx, dy) + 2 * step) / math.hypot(ux, uy):
                        radii.insert(0, fit_radius)
                    if not compact_turns:
                        pivots.extend((0.0, lateral * radius) for radius in radii)
                cd, sd = math.cos(delta), math.sin(delta)
                for px, py in pivots:
                    local_dx, local_dy = (1.0 - cd) * px + sd * py, -sd * px + (1.0 - cd) * py
                    exit_dx = direction[0] * local_dx - direction[1] * local_dy
                    exit_dy = direction[1] * local_dx + direction[0] * local_dy
                    anchors = positions + [(target.x - exit_dx, target.y - exit_dy)]
                    radius = math.hypot(px, py)
                    for x, y in anchors:
                        key = (round(x, 8), round(y, 8), round(px, 8), round(py, 8), delta)
                        if key in seen:
                            continue
                        seen.add(key)
                        entry = FormationPose(x, y, start.psi)
                        exit_pose = FormationPose(x + exit_dx, y + exit_dy, start.psi + delta)
                        score = (math.hypot(x - start.x, y - start.y) + radius * abs(delta)
                                 + math.hypot(target.x - exit_pose.x, target.y - exit_pose.y))
                        if abs(px) > 1e-9 or radius < min_radius:
                            # Compact pivots generally require more steering
                            # and longer body alignment before translating.
                            score += 2 * step
                        proposals.append((score, entry, exit_pose, delta, (px, py)))

            for _, entry, exit_pose, delta, pivot in sorted(proposals, key=lambda candidate: candidate[0]):
                if is_cancelled() or time.perf_counter() >= try_until:
                    return
                result.heading_transition_probes += 1
                if not self.checker.static_check(entry, start_state.heading_map()).success:
                    continue
                now = time.perf_counter()
                clear = not config.preview_checks or turn_preview(
                    entry, delta, pivot, target, compact_turns)
                result.collision_s += time.perf_counter() - now
                if not clear:
                    continue
                arrival = translation_to(start_state, entry, try_until)
                if arrival is None:
                    continue
                if arrival is start_state:
                    # Preserve the arc's displacement if the short approach
                    # was already within the endpoint tolerance.
                    exit_pose = FormationPose(exit_pose.x + start.x - entry.x,
                                              exit_pose.y + start.y - entry.y, exit_pose.psi)
                for attempt in self.generator.arc_goal_connectors(arrival, exit_pose, delta_psi=delta):
                    record_generation(attempt)
                    if time.perf_counter() >= try_until:
                        return
                    if not attempt.successors:
                        continue
                    rotated = attempt.successors[0].next_state
                    end = translation_to(rotated, FormationPose(target.x, target.y, rotated.qf.psi),
                                         try_until)
                    if end is None and compact_turns and time.perf_counter() < try_until:
                        aligned = dict.fromkeys(self.generator.vehicle_ids, rotated.qf.psi)
                        if self.checker.static_check(target, aligned).success:
                            correction = self.generator.generate_spin_preparation(rotated, target, aligned)
                            record_generation(correction)
                            if correction.successors:
                                ready = correction.successors[0].next_state
                                end = translation_to(ready, FormationPose(target.x, target.y, ready.qf.psi),
                                                     try_until)
                    if end is None:
                        continue
                    end = complete_headings(end, try_until)
                    if end is not None:
                        yield candidate_from_state(end, "heading_transition_global_route")

        def prepared_straights(target_pose, try_until, current_headings):
            for junction, target in self.generator.straight_preparations(
                    start_state, target_pose, goal_headings):
                now = time.perf_counter()
                if is_cancelled() or now >= try_until:
                    return
                result.straight_preparation_probes += 1
                state = start_state
                approach = math.hypot(junction.x - state.qf.x, junction.y - state.qf.y) > 1e-9
                clear = (not config.preview_checks or
                         self.generator.crossing_samples_clear(junction, target_pose, target)
                         and (not approach or self.generator.crossing_samples_clear(
                             state.qf, junction, current_headings)))
                result.collision_s += time.perf_counter() - now
                if not clear:
                    continue
                if approach:
                    incoming = translate(state, MODE_O, junction, try_until)
                    if incoming is None:
                        continue
                    state = incoming.next_state
                if time.perf_counter() >= try_until:
                    return
                spin = self.generator.generate_spin_preparation(state, target_pose, target)
                record_generation(spin)
                if not spin.successors:
                    continue
                last = translate(spin.successors[0].next_state, MODE_A, target_pose, try_until)
                if last is not None and self.generator.is_goal(last.next_state, goal_qf, goal_headings):
                    label = "O_then_S_then_A_direct_goal" if approach else "S_then_A_straight_direct_goal"
                    yield replace(last, label=label)

        def align_start(bearing, target):
            headings = start_state.heading_map()
            aligned = {}
            for vehicle_id, theta in headings.items():
                branch = _o_branch(theta, bearing, self.successor_config.allow_reverse)
                aligned[vehicle_id] = bearing if branch is None or branch[0] == 1 else bearing + math.pi
            spin = self.generator.generate_spin_preparation(start_state, target, aligned)
            record_generation(spin)
            return spin.successors[0].next_state if spin.successors else None

        def goal_connectors():
            free = self.successor_config.goal_formation_heading_free
            start = start_state.qf
            target = FormationPose(goal_qf.x, goal_qf.y, start.psi) if free else goal_qf
            end = complete_headings(start_state, deadline)
            if end is not None:
                yield candidate_from_state(end, "S_terminal_headings")
            generated = self.generator.generate(
                start_state, target, direct_goal_translation=True)
            record_generation(generated)
            direct_candidates = list(generated.successors)
            direction = self.generator._goal_direction(start_state, target)
            if free and direction is not None and not any(
                    candidate.edge.mode == MODE_A for candidate in direct_candidates):
                ready = align_start(math.atan2(direction[1], direction[0]), target)
                if ready is not None:
                    straight = translate(ready, MODE_A, target, deadline)
                    if straight is not None:
                        direct_candidates.append(replace(straight, label="S_then_A_straight_direct_goal"))
                direct_candidates.sort(key=lambda item: item.next_state.g)
            yield from (item for item in direct_candidates
                        if self.generator.is_goal(item.next_state, goal_qf, goal_headings))
            corners = self.generator.translation_junctions(start_state, target)
            headings = start_state.heading_map()
            if corners and not self.checker.static_check(
                    FormationPose(goal_qf.x, goal_qf.y, start_state.qf.psi), headings).success:
                # Translating at the incoming body angles cannot occupy the goal.
                corners = ()
            for first_mode, junction, second_mode in corners:
                last = follow_translation(((first_mode, junction), (second_mode, target)),
                                          deadline)
                if last is not None and self.generator.is_goal(last.next_state, goal_qf, goal_headings):
                    yield last
            # Bound additional probing so the lattice retains time when the
            # fixed-heading connector cannot get through.
            preview_deadline = 0.5 * (deadline + time.perf_counter())
            for outer, first, middle, second in self.generator.offset_translation_junctions(
                    start_state, corners):
                now = time.perf_counter()
                if is_cancelled() or now >= preview_deadline:
                    break
                result.offset_preview_probes += 1
                clear = not config.preview_checks or self.generator.crossing_samples_clear(first, second, headings)
                result.collision_s += time.perf_counter() - now
                if not clear:
                    result.offset_preview_rejections += 1
                    continue
                last = follow_translation(((outer, first), (middle, second), (outer, target)),
                                          preview_deadline)
                if last is not None and self.generator.is_goal(last.next_state, goal_qf, goal_headings):
                    yield last
            if config.global_translation_route:
                layer_goal = FormationPose(goal_qf.x, goal_qf.y, start.psi)
                if self.generator.is_goal(replace(start_state, qf=layer_goal), goal_qf, goal_headings):
                    yield from global_translations(start_state, layer_goal, preview_deadline,
                                                   record_route=True)
            yield from prepared_straights(target, preview_deadline, headings)
            if free:
                if config.global_translation_route:
                    # Test different body and cargo headings in sequence.
                    # A free goal never prescribes a turn toward the RViz arrow.
                    now = time.perf_counter()
                    aligned_deadline = now + 0.15 * (deadline - now)
                    bearing = math.atan2(target.y - start.y, target.x - start.x)
                    axis = round(bearing / (math.pi / 2)) * (math.pi / 2)
                    if time.perf_counter() < aligned_deadline:
                        ready = align_start(axis, target)
                        if ready is not None:
                            for item in global_translations(ready, target, aligned_deadline, record_route=True):
                                if self.generator.is_goal(item.next_state, goal_qf, goal_headings):
                                    yield replace(item, label="S_then_global_translation")
                    if config.heading_transition_route:
                        angles = sorted((index * math.pi / 4 for index in range(8)),
                                        key=lambda psi: abs(_wrap(psi - start.psi)))
                        angles = [psi for psi in angles if abs(_wrap(psi - start.psi))
                                  > self.successor_config.goal_psi_tol_rad]
                        turn_deadline = time.perf_counter() + 0.65 * (deadline - time.perf_counter())
                        for index, psi in enumerate(angles):
                            now = time.perf_counter()
                            if is_cancelled() or now >= turn_deadline:
                                break
                            stop = now + (turn_deadline - now) / (len(angles) - index)
                            layer_goal = FormationPose(target.x, target.y, psi)
                            yield from heading_routes(layer_goal, stop, compact_turns=True)
                return
            arcs = self.generator.arc_goal_connectors(start_state, goal_qf)
            while not is_cancelled() and time.perf_counter() < deadline:
                arc = next(arcs, None)
                if arc is None:
                    break
                record_generation(arc)
                yield from (item for item in arc.successors
                            if self.generator.is_goal(item.next_state, goal_qf, goal_headings))
            if config.global_translation_route and config.heading_transition_route:
                now = time.perf_counter()
                # A failed connector must leave room for ordinary expansion
                # and independent terminal certification.
                yield from heading_routes(goal_qf, now + 0.55 * (deadline - now))
                if abs(_wrap(goal_qf.psi - start_state.qf.psi)) >= math.pi / 6:
                    now = time.perf_counter()
                    yield from heading_routes(goal_qf, now + 0.25 * (deadline - now), compact_turns=True)

        direct = goal_connectors() if config.direct_goal_translation else iter(())

        def rank(state, initial):
            heuristic = goal_heuristic_s(state, goal_qf, self.successor_config, config)
            weighted = config.heuristic_weight * heuristic
            return weighted if initial else state.g + weighted

        frontier_type = _NativeFrontier if config.native_frontier else _PythonFrontier
        frontier = frontier_type(self.successor_config, rank)
        frontier.offer(start_state, initial=True)

        while True:
            cancelled_now = bool(is_cancelled())
            elapsed = 0.0 if cancelled_now else time.perf_counter() - started
            status = (budget.probe(cancelled_now, result.expansions, elapsed)
                      if budget is not None else None)
            if (cancelled_now if budget is None else status == native.BudgetStatus.CANCELLED):
                return finish(SEARCH_NO_SOLUTION, "REQUEST_CANCELLED",
                              "request", "manual reset superseded the pending search")
            if (result.expansions >= config.max_expansions if budget is None
                    else status == native.BudgetStatus.EXPANSIONS):
                return finish(
                    SEARCH_BUDGET_EXHAUSTED, "EXPANSION_BUDGET_EXHAUSTED",
                    "budget", "expansions {} >= {}".format(
                        result.expansions, config.max_expansions))
            if (elapsed >= config.max_seconds if budget is None
                    else status == native.BudgetStatus.DEADLINE):
                return finish(
                    SEARCH_BUDGET_EXHAUSTED, "TIME_BUDGET_EXHAUSTED",
                    "budget", "wall time {:.9g} s >= {:.9g} s".format(
                        elapsed, config.max_seconds))
            if not frontier:
                if result.terminal_audits:
                    last = result.terminal_audits[-1]
                    return finish(
                        SEARCH_AUDIT_REJECTED,
                        last.get("failure_code") or last.get(
                            "reason", SEARCH_AUDIT_REJECTED),
                        "audit", "open set exhausted after {} rejected "
                        "terminal audit(s)".format(len(result.terminal_audits)))
                return finish(
                    SEARCH_NO_SOLUTION, SEARCH_NO_SOLUTION, "search",
                    "open set exhausted without reaching the goal")

            connector = next(direct, None)
            if is_cancelled():
                continue
            if config.direct_goal_translation and time.perf_counter() - started >= config.max_seconds:
                continue
            if connector is not None:
                state = connector.next_state
            else:
                state = frontier.pop()
                if state is None:
                    # A lower-g exact state superseded this heap entry.
                    result.stale_heap_entries += 1
                    continue

            if (connector is not None
                    or self.generator.is_goal(state, goal_qf, goal_headings)):
                edges = reconstruct_path(state)
                if terminal_validator is None:
                    return finish(
                        SEARCH_AUDIT_REJECTED, "TERMINAL_VALIDATOR_REQUIRED",
                        "audit", "no terminal validator was supplied; an "
                        "unaudited geometric goal is not an FR04 success",
                        state)
                if budget is None:
                    audit_started = time.perf_counter()
                    accepted, audit_detail = terminal_validator(edges)
                    audit_finished = time.perf_counter()
                    audit_late = audit_finished - started >= config.max_seconds
                else:
                    accepted, audit_detail, audit_started, audit_finished, audit_late = (
                        budget.audit_terminal(started, time.perf_counter, terminal_validator, edges))
                record = dict(audit_detail or {})
                record.update({
                    "accepted": bool(accepted),
                    "edge_count": len(edges),
                    "path_proxy_g_s": state.g,
                    "path": path_records(state),
                    "audit_wall_time_s": audit_finished - audit_started,
                    "search_elapsed_at_audit_s": audit_finished - started,
                })
                if connector is not None:
                    record["direct_goal_translation"] = connector.label
                result.terminal_audits.append(record)
                if accepted:
                    if audit_late:
                        return finish(
                            SEARCH_BUDGET_EXHAUSTED, "TIME_BUDGET_EXHAUSTED",
                            "budget", "terminal audit completed at {:.9g} s, "
                            "at or beyond the {:.9g} s budget; an accepted "
                            "path cannot bypass the time budget".format(
                                audit_finished - started, config.max_seconds),
                            state)
                    if connector is not None:
                        result.direct_goal_translation = connector.label
                    return finish(
                        SEARCH_SUCCESS, "TERMINAL_AUDIT_ACCEPTED", "none",
                        "goal path accepted by the terminal validator", state)
                rejection = record.get("failure_code") or record.get(
                    "reason", SEARCH_AUDIT_REJECTED)
                result.terminal_rejection_histogram[rejection] = (
                    result.terminal_rejection_histogram.get(rejection, 0) + 1)
                continue

            result.expansions += 1
            generated = self.generator.generate(state, goal_qf)
            record_generation(generated)

            for successor in generated.successors:
                if (state.edge is not None
                        and state.edge.primitive == PRIMITIVE_S_SPIN
                        and successor.edge.primitive == PRIMITIVE_S_SPIN):
                    result.redundant_s_pruned += 1
                    continue
                child = successor.next_state
                if not frontier.offer(child):
                    result.duplicate_key_rejections += 1
                    continue
                result.accepted_successors += 1
