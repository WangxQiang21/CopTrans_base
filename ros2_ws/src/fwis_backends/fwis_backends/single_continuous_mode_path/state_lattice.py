"""Experimental A/O/S lattice with exact snapped primitives and certified incumbents."""

from dataclasses import dataclass
from functools import lru_cache
import heapq
from itertools import count
import math
from time import perf_counter
from typing import Callable

from .continuous_geometry import fit_continuous_endpoint, plan_from_actions
from .continuous_retiming import retime_continuous
from .core import Action, Grid, Pose, advance, collapse_exact_runs, fit_action_to_pose, wrap
from .quality import TrajectoryEvaluator, action_shunting_violation
from .retiming import MotionLimits, retime


_ACKERMANN = 0
_OBLIQUE = 1
_SPIN = 2
_HEADING_STEP = math.pi / 2


@dataclass(frozen=True)
class Primitive:
    dx: int
    dy: int
    heading: int
    action: Action


@dataclass(frozen=True)
class LatticeConfig:
    resolution_m: float = .30
    terminal_radius_m: float = 2.0
    max_expansions: int = 5000
    time_budget_s: float = 2.0
    allow_reverse: bool = True
    refine_continuous: bool = False
    analytic_ackermann: bool = True

    def __post_init__(self):
        sizes = self.resolution_m, self.terminal_radius_m, self.time_budget_s
        if not all(math.isfinite(s) and s > 0.0 for s in sizes) or self.max_expansions < 1:
            raise ValueError("invalid lattice resolution or budget")


@dataclass(frozen=True)
class _State:
    x: int
    y: int
    heading: int
    mode: int
    gear: int
    control: float
    reversals: int
    spin_steps: int


@dataclass(frozen=True)
class Candidate:
    actions: tuple[Action, ...]
    timed: object
    quality: dict
    continuous_geometry: object | None = None


@dataclass(frozen=True)
class LatticeResult:
    incumbent: Candidate | None
    reason: str
    expanded: int
    tested_goals: int
    rejected_goals: int
    first_certified_s: float | None
    wall_s: float


def _primitive(heading, resolution, action):
    peak = advance(Pose(0.0, 0.0, heading * _HEADING_STEP), action)
    x, y = round(peak.x / resolution), round(peak.y / resolution)
    end_heading = round(peak.th / _HEADING_STEP) % 4
    if (math.hypot(peak.x - x * resolution, peak.y - y * resolution) > 1e-8
            or abs(wrap(peak.th - end_heading * _HEADING_STEP)) > 1e-8):
        raise ValueError("primitive failed lattice closure")
    return Primitive(x, y, end_heading, action)


@lru_cache(maxsize=32)
def generate_primitives(resolution_m, min_radius_m, allow_reverse=True):
    """Precompute one translation-invariant action library for all four headings."""
    if not all(math.isfinite(v) and v > 0 for v in (resolution_m, min_radius_m)):
        raise ValueError("lattice geometry must be finite and positive")
    radius_cells = math.ceil(min_radius_m / resolution_m)
    radii = tuple((radius_cells + extra) * resolution_m for extra in (0, 2))
    result = []
    for heading in range(4):
        actions = []
        for gear in (1, -1) if allow_reverse else (1,):
            actions.append(Action(_ACKERMANN, gear * resolution_m, 0.0))
            actions.extend(Action(_ACKERMANN, gear * radius * _HEADING_STEP, turn / radius)
                           for radius in radii for turn in (-1, 1))
        for x, y in ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)):
            angle = wrap(math.atan2(y, x) - heading * _HEADING_STEP)
            gear = 1
            if abs(angle) > _HEADING_STEP + 1e-12:
                gear = -1
                angle = wrap(angle + math.pi)
            if abs(angle) > 1e-10 and ((gear == 1) or allow_reverse):
                actions.append(Action(_OBLIQUE, gear * resolution_m * math.hypot(x, y), angle))
        actions.extend(Action(_SPIN, turn * _HEADING_STEP, 0.0) for turn in (-1, 1))
        result.append(tuple(_primitive(heading, resolution_m, action) for action in actions))
    return tuple(result)


def _next_state(state, primitive):
    action = primitive.action
    if action.mode == _SPIN:
        spin = (state.spin_steps + round(action.amount / _HEADING_STEP)) % 4
        return _State(state.x, state.y, primitive.heading, _SPIN, state.gear,
                      0.0, state.reversals, spin)
    gear = 1 if action.amount > 0.0 else -1
    changed = state.gear != 0 and gear != state.gear
    if changed and (state.reversals or not state.spin_steps):
        return None
    return _State(state.x + primitive.dx, state.y + primitive.dy, primitive.heading,
                  action.mode, gear, round(action.control, 10), state.reversals + int(changed), 0)


def _cost(action, previous_mode):
    base = (0.45 * abs(action.amount) if action.mode == _SPIN else abs(action.amount))
    return base + (0.35 if action.mode != previous_mode else 0.0) + (
        0.12 * base if action.mode != _SPIN and action.amount < 0.0 else 0.0)


def _connect(start, goal, limits, allow_reverse):
    """Return exact terminal connections with at most one translation and one Spin."""
    tolerance = 1e-7
    rotated = Pose(start.x, start.y, goal.th)
    pre_goal = Pose(goal.x, goal.y, start.th)
    spin = fit_action_to_pose(_SPIN, start, rotated, tolerance)
    proposals = [(start, goal, None, None)]
    if spin is not None and abs(spin.amount) > tolerance:
        proposals.extend(((rotated, goal, spin, None), (start, pre_goal, None, spin)))
    if math.hypot(start.x - goal.x, start.y - goal.y) <= tolerance:
        yield (spin,) if spin is not None and abs(spin.amount) > tolerance else ()
    for fore, aft, before, after in proposals:
        for mode in (_ACKERMANN, _OBLIQUE):
            action = fit_action_to_pose(mode, fore, aft, tolerance)
            if (action is None or (action.amount < 0.0 and not allow_reverse)
                    or (mode == _ACKERMANN and abs(action.control) > 1.0 / limits.r_min_ack + 1e-10)
                    or (mode == _OBLIQUE and abs(action.control) < 1e-10)):
                continue
            yield tuple(step for step in (before, action, after) if step is not None)


def audited_continuous_candidate(geometry, actions, evaluator, *, analytic_ackermann,
                                 existing=None):
    """Prefer fast constant A timing only after an independent audit."""
    if existing is None:
        timed = retime_continuous(geometry, evaluator.limits)
        audit = evaluator.evaluate_continuous_geometry(geometry, timed)
        if audit["status"] == "FEASIBLE":
            existing = Candidate(actions, timed, audit, geometry)
    if not analytic_ackermann or not any(
            block.mode == _ACKERMANN and all(knot.value == block.knots[0].value and knot.derivative == 0.
                                            for knot in block.knots)
            for block in geometry.blocks):
        return existing
    try:
        timed = retime_continuous(geometry, evaluator.limits, analytic_ackermann=True)
        if existing is not None and timed.duration >= existing.timed.duration - 1e-9:
            return existing
        quality = evaluator.evaluate_continuous_geometry(geometry, timed)
        if quality["status"] == "FEASIBLE":
            return Candidate(actions, timed, quality, geometry)
    except (ValueError, ArithmeticError, RuntimeError):
        pass
    return existing


def _maybe_refine(candidate, evaluator, cutoff):
    """Keep mode order and direction, then ask the existing audit to judge the new curve."""
    remaining = cutoff - perf_counter()
    if remaining <= .08 or not candidate.actions:
        return candidate
    try:
        plan = plan_from_actions(evaluator.start, evaluator.goal, candidate.actions)
        fitted, _ = fit_continuous_endpoint(plan, r_min_ack=evaluator.limits.r_min_ack, deadline=cutoff)
        if fitted is None or perf_counter() >= cutoff:
            return candidate
        timed = retime_continuous(fitted, evaluator.limits)
        quality = evaluator.evaluate_continuous_geometry(fitted, timed)
    except (ValueError, ArithmeticError, RuntimeError):
        return candidate
    if quality["status"] == "FEASIBLE" and timed.duration < candidate.timed.duration:
        return Candidate(candidate.actions, timed, quality, fitted)
    return candidate


def search_lattice(start: Pose, goal: Pose, grid: Grid, limits: MotionLimits,
                   config: LatticeConfig | None = None,
                   on_certified: Callable[[Candidate], None] | None = None, *,
                   footprint_m: tuple[float, float] = (.738, .5),
                   cancelled: Callable[[], bool] | None = None) -> LatticeResult:
    """Search bounded mode/direction paths, returning the best independently certified one."""
    config = config or LatticeConfig()
    started = perf_counter()
    cutoff = started + config.time_budget_s
    is_cancelled = cancelled if cancelled is not None else lambda: False
    if is_cancelled():
        return LatticeResult(None, "CANCELLED", 0, 0, 0, None, perf_counter() - started)
    if not all(math.isfinite(v) for pose in (start, goal) for v in (pose.x, pose.y, pose.th)):
        return LatticeResult(None, "INVALID_ENDPOINT", 0, 0, 0, None, perf_counter() - started)
    if not grid.pose_clear(start) or not grid.pose_clear(goal):
        return LatticeResult(None, "ENDPOINT_COLLISION", 0, 0, 0, None, perf_counter() - started)
    library = generate_primitives(config.resolution_m, limits.r_min_ack, config.allow_reverse)
    length, width = footprint_m
    margin = 2.0 * grid.half_l - length
    if (not all(math.isfinite(v) and v > 0 for v in footprint_m)
            or margin < 0.0 or abs(margin - (2.0 * grid.half_w - width)) > 1e-8):
        raise ValueError("grid footprint and independent lattice audit disagree")
    evaluator = TrajectoryEvaluator(grid, start, goal, limits, length=length, width=width,
                                    xy_tol=.02, yaw_tol=math.radians(1.0),
                                    allow_reverse=config.allow_reverse, allow_shunting=False)
    base = _State(0, 0, 0, _ACKERMANN, 0, 0.0, 0, 0)
    rotation = math.cos(start.th), math.sin(start.th)
    expanded = tested = rejected = 0
    incumbent, first_time = None, None
    checked_signatures = set()

    def pose(state):
        c, s = rotation
        h = config.resolution_m
        return Pose(start.x + h * (c * state.x - s * state.y),
                    start.y + h * (s * state.x + c * state.y),
                    wrap(start.th + state.heading * _HEADING_STEP))

    def offer(actions):
        nonlocal tested, rejected, incumbent, first_time
        if is_cancelled() or perf_counter() >= cutoff:
            return
        actions = tuple(collapse_exact_runs(actions))
        sig = evaluator.signature(actions)
        if sig in checked_signatures:
            return
        checked_signatures.add(sig)
        tested += 1
        if action_shunting_violation(actions) is not None:
            rejected += 1
            return
        candidate = None
        try:
            # A single constant-control translation has the same geometry
            # in both models. Use cruise-capable continuous timing.
            if len(actions) == 1 and actions[0].mode in (_ACKERMANN, _OBLIQUE):
                geometry = plan_from_actions(start, goal, actions)
                candidate = audited_continuous_candidate(
                    geometry, actions, evaluator, analytic_ackermann=config.analytic_ackermann)
            if candidate is None:
                timed = retime(start, actions, limits, allow_reverse=config.allow_reverse)
                if incumbent is not None and timed.duration >= incumbent.timed.duration - 1e-9:
                    return
                quality = evaluator.evaluate(actions, timed)
                if quality["status"] == "FEASIBLE":
                    candidate = Candidate(actions, timed, quality)
        except (ValueError, ArithmeticError):
            rejected += 1
            return
        if candidate is None:
            rejected += 1
            return
        if incumbent is not None and candidate.timed.duration >= incumbent.timed.duration - 1e-9:
            return
        elapsed = perf_counter() - started
        if is_cancelled() or elapsed >= config.time_budget_s:
            return
        incumbent = candidate
        if first_time is None:
            first_time = elapsed
        if on_certified is not None:
            on_certified(incumbent)

    def clear_terminal(current, suffix):
        for action in suffix:
            if not grid.action_clear(current, action, .04):
                return False
            current = advance(current, action)
        return True

    # Keep an incumbent across a quick weighted pass and a lower-weight pass.
    # Duplicate dominance uses the discrete proxy score, not certified duration.
    serial = count()
    for weight, expansion_limit, until_first in (
        (2.2, max(1, config.max_expansions // 2), True),
        (1.0, config.max_expansions, False),
    ):
        best = {base: 0.0}
        pending = [(weight * math.hypot(start.x - goal.x, start.y - goal.y),
                    next(serial), 0.0, base, ())]
        local_cutoff = started + config.time_budget_s * (.55 if until_first else
                                                         .88 if config.refine_continuous else 1.0)
        while (pending and expanded < expansion_limit and not is_cancelled()
               and perf_counter() < local_cutoff):
            _, _, cost, state, path = heapq.heappop(pending)
            if cost > best[state]:
                continue
            current = pose(state)
            expanded += 1
            if (math.hypot(current.x - goal.x, current.y - goal.y) <= config.terminal_radius_m
                    or state == base):
                for suffix in _connect(current, goal, limits, config.allow_reverse):
                    if is_cancelled() or perf_counter() >= local_cutoff:
                        break
                    if clear_terminal(current, suffix):
                        offer(path + suffix)
                if until_first and incumbent is not None:
                    break
            for primitive in library[state.heading]:
                if is_cancelled() or perf_counter() >= local_cutoff:
                    break
                destination = _next_state(state, primitive)
                if destination is None:
                    continue
                trial = cost + _cost(primitive.action, state.mode)
                if trial >= best.get(destination, math.inf):
                    continue
                if not grid.action_clear(current, primitive.action, .04):
                    continue
                target = pose(destination)
                best[destination] = trial
                score = trial + weight * math.hypot(target.x - goal.x, target.y - goal.y)
                heapq.heappush(pending, (score, next(serial), trial, destination, path + (primitive.action,)))

    if incumbent is not None and config.refine_continuous and not is_cancelled():
        refined = _maybe_refine(incumbent, evaluator, cutoff)
        if refined is not incumbent and perf_counter() < cutoff:
            incumbent = refined
            if on_certified is not None:
                on_certified(incumbent)
    reason = ("CANCELLED" if is_cancelled() else "CERTIFIED" if incumbent is not None else "BUDGET_EXHAUSTED"
              if perf_counter() >= cutoff or expanded >= config.max_expansions else "NO_LATTICE_SOLUTION")
    return LatticeResult(incumbent, reason, expanded, tested, rejected, first_time, perf_counter() - started)
