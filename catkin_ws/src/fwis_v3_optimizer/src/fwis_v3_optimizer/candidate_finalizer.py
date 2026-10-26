"""Search-independent completion of learned or classical primitive candidates.

All nonempty published candidates pass the existing continuous geometry,
timing, dynamics, direction, and display-interpolation audits.
"""
from dataclasses import asdict, dataclass, field, replace
import hashlib
import json
import math
import numbers
import time

from .core import Pose, replay, wrap, collapse_exact_runs
from .continuous_geometry import (
    plan_from_actions, fit_continuous_endpoint, fit_continuous_suffix)
from .continuous_retiming import (
    ContinuousTimedPlan, SegmentTimingCache, retime_continuous)
from .quality import QualityProfile, TrajectoryEvaluator, action_shunting_violation


@dataclass(frozen=True)
class PlanningProblem:
    grid: object
    start: Pose
    goal: Pose
    limits: object
    length: float = .738
    width: float = .500
    allow_reverse: bool = True
    allow_shunting: bool = False
    initial_mode: int = 0
    xy_tol: float = .02
    yaw_tol: float = math.pi/180.
    quality_profile: object = field(default_factory=QualityProfile)
    task_id: str = ""


@dataclass(frozen=True)
class FinalizationConfig:
    sampling_dt: float = .05
    sweep_spacing: float = .005
    fit_max_iterations: int = 40
    max_projection_xy: float = .16
    max_projection_yaw: float = .14
    suffix_repair: bool = True
    include_metrics: bool = True

    def __post_init__(self):
        for name in ("sampling_dt", "sweep_spacing", "max_projection_xy",
                     "max_projection_yaw"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError("invalid_finalization_config:"+name)
        if self.sweep_spacing > .005:
            raise ValueError("final_sweep_must_not_exceed_5mm")
        if (isinstance(self.fit_max_iterations, bool) or
                not isinstance(self.fit_max_iterations, numbers.Integral) or
                self.fit_max_iterations < 1):
            raise ValueError("invalid_fit_iterations")


@dataclass
class FinalizationResult:
    status: str
    source: str
    actions: tuple
    reason: str = None
    failure_stage: str = None
    plan: object = None
    timed: object = None
    audit: dict = field(default_factory=dict)
    stages_s: dict = field(default_factory=dict)
    detail: dict = field(default_factory=dict)

    @property
    def feasible(self):
        return self.status == "FEASIBLE"

    def report(self, include_trace=False):
        def finite(value):
            value = float(value)
            return value if math.isfinite(value) else None
        rows = [[finite(a.mode), finite(a.amount), finite(a.control)]
                for a in self.actions]
        out = {"status": self.status, "source": self.source,
               "reason": self.reason, "failure_stage": self.failure_stage,
               "actions": rows,
               "actions_sha256": hashlib.sha256(json.dumps(
                   rows, separators=(",", ":"), allow_nan=False).encode()).hexdigest(),
               "audit": self.audit, "stages_s": self.stages_s,
               "detail": self.detail}
        if self.timed is not None:
            out["timing"] = self.timed.report()
            if include_trace:
                out["timed_trace"] = [
                    [float(t), p.x, p.y, p.th, float(v), int(m)]
                    for t, p, v, m in self.timed.samples]
        if self.plan is not None:
            out["continuous_plan"] = asdict(self.plan)
        return out


def _identity(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


def _expired(deadline):
    if deadline is not None and time.perf_counter() >= deadline:
        raise ValueError("planning_deadline_exceeded")


def finalize_candidate(problem, actions, config=None, deadline=None,
                       source="candidate", segment_cache=None):
    """Return a certificate or a stage-specific rejection; never run search."""
    config = config or FinalizationConfig()
    actions = tuple(actions)
    result = FinalizationResult("REJECTED", source, actions)
    stage = "input"
    begun = time.perf_counter()
    stage_begin = begun

    def enter(name):
        nonlocal stage, stage_begin
        result.stages_s[stage] = time.perf_counter()-stage_begin
        _expired(deadline)
        stage, stage_begin = name, time.perf_counter()

    try:
        if deadline is not None and not math.isfinite(deadline):
            raise ValueError("invalid_planning_deadline")
        _expired(deadline)
        if problem.initial_mode != 0:
            raise ValueError("unsupported_initial_mode")
        for p in (problem.start, problem.goal):
            if not all(math.isfinite(x) for x in (p.x, p.y, p.th)):
                raise ValueError("nonfinite_problem_pose")
            if not problem.grid.pose_clear(p):
                raise ValueError("invalid_start_or_goal_footprint")
        for tol in (problem.xy_tol, problem.yaw_tol):
            if not math.isfinite(tol) or tol <= 0:
                raise ValueError("invalid_goal_tolerance")
        for a in actions:
            if a.mode not in (0, 1, 2):
                raise ValueError("unknown_mode")
            if not all(math.isfinite(z) for z in (a.amount, a.control)):
                raise ValueError("nonfinite_action")
            if abs(a.amount) <= 1e-10:
                raise ValueError("zero_amount_action")
            if not problem.allow_reverse and a.mode != 2 and a.amount < 0:
                raise ValueError("reverse_motion_forbidden")
            if a.mode == 0 and abs(a.control) > 1./problem.limits.r_min_ack+1e-9:
                raise ValueError("ackermann_curvature_limit")
            if a.mode == 1 and abs(a.control) > math.pi/2+1e-9:
                raise ValueError("oblique_angle_limit")
            if a.mode == 2 and abs(a.control) > 1e-10:
                raise ValueError("spin_control_must_be_zero")
        if not problem.allow_shunting:
            violation = action_shunting_violation(actions)
            if violation:
                raise ValueError("shunting_forbidden:"+violation["reason"])
        if not actions:
            if (math.hypot(problem.start.x-problem.goal.x,
                           problem.start.y-problem.goal.y) > 1e-10 or
                    abs(wrap(problem.start.th-problem.goal.th)) > 1e-10):
                raise ValueError("empty_path_does_not_reach_goal")
            # The continuous motion auditor requires a moving block; certify
            # this explicitly static task separately, without inventing motion.
            result.timed = ContinuousTimedPlan(
                [(0., problem.start, 0., 0)], [], 0., 0., 0., 0., 0,
                0., 0., problem.limits, [], [])
            result.audit = {"status": "FEASIBLE", "geometry_status": "PASS",
                            "dynamic_status": "PASS", "timing_status": "PASS",
                            "certificate": "zero_motion_exact_same_pose",
                            "score": 0.}
            result.status = "FEASIBLE"
            return result
        actions = tuple(collapse_exact_runs(actions))
        result.actions = actions
        enter("primitive_sweep")
        poses = replay(problem.start, actions)
        if not problem.grid.trajectory_clear(
                poses, actions, max_corner_step=config.sweep_spacing):
            raise ValueError("primitive_swept_collision")
        if (math.hypot(poses[-1].x-problem.goal.x,
                       poses[-1].y-problem.goal.y) > config.max_projection_xy or
                abs(wrap(poses[-1].th-problem.goal.th)) > config.max_projection_yaw):
            raise ValueError("candidate_outside_terminal_projection_budget")
        audit_profile = replace(problem.quality_profile, sweep_spacing=min(
            problem.quality_profile.sweep_spacing, config.sweep_spacing))
        evaluator = TrajectoryEvaluator(
            problem.grid, problem.start, problem.goal, problem.limits,
            problem.length, problem.width, problem.xy_tol, problem.yaw_tol,
            audit_profile, allow_reverse=problem.allow_reverse,
            allow_shunting=problem.allow_shunting)
        enter("continuous_build")
        original = plan_from_actions(problem.start, problem.goal, actions)
        result.detail["primitive_geometry_sha256"] = _identity({
            "start": asdict(problem.start), "actions": [asdict(a) for a in actions]})
        result.detail["initial_continuous_sha256"] = _identity(asdict(original))
        enter("endpoint_fit")
        plan, fit = fit_continuous_endpoint(
            original, problem.limits.r_min_ack,
            max_iterations=config.fit_max_iterations,
            terminal_xy_tol=min(.015, problem.xy_tol),
            terminal_yaw_tol=min(math.radians(.75), problem.yaw_tol),
            stop_on_feasible=True, deadline=deadline)
        result.detail["endpoint_fit"] = fit
        enter("continuous_sweep")
        collision = evaluator.continuous_collision(plan) if fit["success"] else None
        if collision is not None or not fit["success"]:
            original_collision = evaluator.continuous_collision(original)
            attempts = []
            if config.suffix_repair and original_collision is None:
                for tail in (2., 4.):
                    _expired(deadline)
                    repaired, repair = fit_continuous_suffix(
                        original, tail_length_m=tail,
                        r_min_ack=problem.limits.r_min_ack,
                        max_iterations=config.fit_max_iterations,
                        terminal_xy_tol=1e-7, terminal_yaw_tol=1e-7,
                        deadline=deadline)
                    issue = (evaluator.continuous_collision(repaired)
                             if repair["success"] else None)
                    attempts.append(dict(repair, first_collision=issue))
                    if repair["success"] and issue is None:
                        plan, fit, collision = repaired, repair, None
                        break
            result.detail["suffix_attempts"] = attempts
            if collision is not None or not fit["success"]:
                result.detail["first_collision"] = collision
                raise ValueError("continuous_projection_or_sweep_failed")
        result.plan = plan
        result.detail["final_continuous_sha256"] = _identity(asdict(plan))
        result.detail["audit_sweep_spacing_m"] = audit_profile.sweep_spacing
        enter("retime")
        result.timed = retime_continuous(
            plan, problem.limits, dt=config.sampling_dt,
            spatial_dt=config.sweep_spacing, segment_cache=segment_cache)
        enter("independent_audit")
        result.audit = evaluator.evaluate_continuous_geometry(
            plan, result.timed, xy_tol=problem.xy_tol, yaw_tol=problem.yaw_tol,
            allow_reverse=problem.allow_reverse,
            include_metrics=config.include_metrics,
            defer_score=not config.include_metrics)
        if result.audit["status"] != "FEASIBLE":
            raise ValueError(result.audit.get("reason") or "independent_audit_failed")
        enter("display_sweep")
        samples = result.timed.samples
        if any(b[0] <= a[0] for a, b in zip(samples, samples[1:])):
            raise ValueError("nonincreasing_published_timestamps")
        for left, right in zip(samples, samples[1:]):
            _expired(deadline)
            if not problem.grid.interpolated_clear(
                    left[1], right[1], config.sweep_spacing):
                raise ValueError("display_interpolated_swept_collision")
        _expired(deadline)
        result.detail["published_samples_sha256"] = _identity([
            [t, p.x, p.y, p.th, v, mode] for t, p, v, mode in samples])
        result.status = "FEASIBLE"
    except (ValueError, ArithmeticError, RuntimeError) as exc:
        result.reason, result.failure_stage = str(exc), stage
    finally:
        result.stages_s[stage] = time.perf_counter()-stage_begin
        result.stages_s["total"] = time.perf_counter()-begun
    return result


def finalize_candidate_set(problem, candidates, config=None, deadline=None):
    """Complete source/action pairs and choose the shortest certified duration."""
    cache = SegmentTimingCache()
    results = []
    signatures = set()
    for source, actions in candidates:
        if deadline is not None and time.perf_counter() >= deadline:
            break
        actions = tuple(actions)
        key = tuple((a.mode, a.amount, a.control) for a in actions)
        if key in signatures:
            continue
        signatures.add(key)
        results.append(finalize_candidate(problem, actions, config, deadline,
                                          source, cache))
    eligible = [r for r in results if r.feasible]
    winner = min(eligible, key=lambda r: (
        r.timed.duration, r.audit.get("score") or 0.)) if eligible else None
    return winner, results
