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

This module does not implement the whole-map graph search (FR04), ROS, or any
NLP.  ``RepresentativeTable`` is only the same-key representative/lower-g
selection rule required here; open/closed/priority-queue handling is FR04.
"""
from dataclasses import dataclass, field
import itertools
import math
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
from .formation_retiming import FormationTimingLimits
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
        for name in ("allow_reverse", "goal_heading_free"):
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
                  goal_qf=None):
    """Build the exact start state in the canonical initial A configuration.

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
        qf=qf, headings=heading_pairs, mode=MODE_A,
        wheel=initial_wheel_signature(model, motion_limits), g=0.0,
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
    if goal_qf is not None:
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
    direction = goal_direction(qf, goal_qf, config)
    if direction is not None:
        displacement = (direction[0], direction[1])
        for gears in _gear_combinations(model.vehicle_ids,
                                        config.allow_reverse):
            required = a_straight_required_headings(
                model, displacement,
                dict(zip(model.vehicle_ids, gears)))
            if _headings_match(current, required):
                labels.append(
                    "A_straight_goal_gear{}".format(gear_label(gears)))
    for base_label, delta_psi, pivot in a_arc_schemes(qf, goal_qf, config):
        for gears in _gear_combinations(model.vehicle_ids,
                                        config.allow_reverse):
            try:
                required = a_arc_required_headings(
                    model, qf, delta_psi, pivot,
                    dict(zip(model.vehicle_ids, gears)),
                    limits=motion_limits)
            except UniformMotionError:
                continue
            if _headings_match(current, required):
                labels.append("{}_gear{}".format(
                    base_label, gear_label(gears)))
    return tuple(sorted(set(labels)))


# ---------------------------------------------------------------------------
# Successor generator
# ---------------------------------------------------------------------------


class SuccessorGenerator:
    """Generate finite, mode-consistent, swept, costed successors for a state."""

    def __init__(self, model, checker, config=None):
        if not isinstance(model, FormationModel):
            raise FormationValidationError(
                "INVALID_MODEL expected FormationModel")
        if not isinstance(checker, FormationCollisionChecker):
            raise FormationValidationError(
                "INVALID_COLLISION_CHECKER expected FormationCollisionChecker")
        self.model = model
        self.checker = checker
        self.config = config or SuccessorConfig()
        self.motion_limits = timing_motion_limits(self.config.timing_limits)
        self.vehicle_ids = tuple(model.vehicle_ids)
        self._sweep_policy = SweepPolicy(
            max_corner_step_m=self.config.sweep_spacing_m,
            max_samples_per_segment=self.config.max_samples_per_segment)

    # -- public entry ----------------------------------------------------

    def generate(self, state, goal_qf):
        """Return a :class:`SuccessorResult` for one exact state and goal pose."""
        import time

        result = SuccessorResult()
        current = state.heading_map()
        collision_time = 0.0

        def consider(label, edge):
            """Cost + sweep + accept/reject one constructed edge."""
            nonlocal collision_time
            result.candidate_count += 1
            setup_s = setup_demand_s(state.wheel, edge, self.motion_limits)
            proxy_s = motion_proxy_s(edge, self.config.timing_limits)
            segment = analytic_sweep_segment(edge, proxy_s)
            start = time.perf_counter()
            report = self.checker.swept_check_analytic(
                [segment], self._sweep_policy)
            collision_time += time.perf_counter() - start
            if not report.success:
                result.reject(
                    label, report.reason or "SWEEP_REJECTED",
                    "vehicle_id={} station={} detail={}".format(
                        report.vehicle_id, report.station, report.detail))
                return
            next_state = UniformSearchState(
                qf=edge.end_qf, headings=tuple(edge.end_headings),
                mode=edge.mode, wheel=wheel_signature_from_edge(edge),
                g=state.g + setup_s + proxy_s, parent=state, edge=edge,
                a_labels=a_capability_labels(
                    self.model, edge.end_qf, edge.end_headings, goal_qf,
                    self.config, self.motion_limits))
            result.successors.append(Successor(
                edge=edge, next_state=next_state, setup_s=setup_s,
                motion_proxy_s=proxy_s, cost_s=setup_s + proxy_s, label=label))

        generation_start = time.perf_counter()

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

        result.generation_s = time.perf_counter() - generation_start - collision_time
        result.collision_s = collision_time
        return result

    # -- goal test --------------------------------------------------------

    def is_goal(self, state, goal_qf, goal_headings=None):
        """Cargo position/heading are hard; body headings free by default."""
        if math.hypot(state.qf.x - goal_qf.x,
                      state.qf.y - goal_qf.y) > self.config.goal_xy_tol_m:
            return False
        if abs(_wrap(state.qf.psi - goal_qf.psi)) > self.config.goal_psi_tol_rad:
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
        deltas = {}
        any_change = False
        for vehicle_id in self.vehicle_ids:
            delta = _wrap(target[vehicle_id] - current[vehicle_id])
            deltas[vehicle_id] = delta
            if abs(delta) > 1e-12:
                any_change = True
        if not any_change:
            # Already aligned with this scheme; no zero-cost S self-loop.
            return
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
