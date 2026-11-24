"""S6 fixed-budget candidate master for continuous formation geometry.

This module only admits candidates whose S5 mode assignments and S6 continuous
plan pass an independent audit.  It publishes nothing and never returns
``FEASIBLE``: a selected candidate is ``KINEMATIC_VALID`` geometry, not a
timed, collision-certified or executable trajectory.
"""
from dataclasses import dataclass, replace
import math
from typing import Optional, Tuple

from .formation import FormationValidationError
from .formation_collision import SweepPolicy
from .formation_geometry import (
    KINEMATIC_VALID,
    FormationContinuousPlan,
    FormationPlanAudit,
    FormationPlanBuild,
    audit_continuous_plan,
    plan_candidate_score,
)
from .formation_kinematics import KinematicLimits


def _positive_integer(value, context):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise FormationValidationError(
            "INVALID_POSITIVE_INTEGER context={} value={!r}".format(
                context, value))
    return value


def _nonnegative(value, context):
    if isinstance(value, bool):
        raise FormationValidationError(
            "INVALID_NONNEGATIVE context={} value={!r}".format(
                context, value))
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise FormationValidationError(
            "INVALID_NONNEGATIVE context={} value={!r}".format(
                context, value)) from exc
    if not math.isfinite(result) or result < 0.0:
        raise FormationValidationError(
            "INVALID_NONNEGATIVE context={} value={!r}".format(
                context, value))
    return result


@dataclass(frozen=True)
class FormationMasterConfig:
    """Fixed K, sampling and admission policy for the S6 candidate master."""

    max_candidates: int = 4
    max_schedule_combinations: int = 64
    max_schedules_per_block: int = 4
    collision_margin_m: float = 0.05
    sweep_spacing_m: float = 0.02
    max_samples_per_block: int = 20000
    require_collision_precheck: bool = False

    def __post_init__(self):
        for name in (
                "max_candidates", "max_schedule_combinations",
                "max_schedules_per_block", "max_samples_per_block"):
            _positive_integer(getattr(self, name), name)
        object.__setattr__(self, "collision_margin_m", _nonnegative(
            self.collision_margin_m, "collision_margin_m"))
        object.__setattr__(self, "sweep_spacing_m", _nonnegative(
            self.sweep_spacing_m, "sweep_spacing_m"))
        if self.sweep_spacing_m <= 0.0:
            raise FormationValidationError(
                "INVALID_SWEEP_SPACING sweep_spacing_m must be positive")
        if not isinstance(self.require_collision_precheck, bool):
            raise FormationValidationError(
                "INVALID_REQUIRE_COLLISION_PRECHECK expected bool")


@dataclass(frozen=True)
class FormationCandidate:
    """An admitted continuous geometry candidate with its independent audit."""

    candidate_id: int
    source: str
    plan: FormationContinuousPlan
    audit: FormationPlanAudit
    score: float
    key: tuple

    def __post_init__(self):
        _positive_integer(self.candidate_id, "FormationCandidate.candidate_id")
        if not isinstance(self.source, str) or not self.source:
            raise FormationValidationError(
                "INVALID_CANDIDATE_SOURCE")
        if not isinstance(self.plan, FormationContinuousPlan):
            raise FormationValidationError(
                "INVALID_CANDIDATE_PLAN")
        if not isinstance(self.audit, FormationPlanAudit):
            raise FormationValidationError(
                "INVALID_CANDIDATE_AUDIT")
        if not self.audit.success:
            raise FormationValidationError(
                "UNAUDITED_CANDIDATE cannot be admitted")
        if self.plan.certification != KINEMATIC_VALID:
            raise FormationValidationError(
                "INVALID_CANDIDATE_CERTIFICATION expected KINEMATIC_VALID")
        _nonnegative(self.score, "FormationCandidate.score")

    def as_status(self):
        return {
            "schema": "formation_candidate/v1",
            "candidate_id": self.candidate_id,
            "source": self.source,
            "plan_id": self.plan.plan_id,
            "formation_id": self.plan.formation_id,
            "certification": self.plan.certification,
            "score": self.score,
            "blocks": len(self.plan.blocks),
            "mode_switches": self.plan.mode_switch_count,
            "total_parameter_length": self.plan.total_parameter_length,
            "audit_reason": self.audit.reason,
            "audit_detail": self.audit.detail,
            "collision_prechecked": self.audit.collision_prechecked,
            "collision_success": self.audit.collision_success,
            "executable": False,
            "trajectory_published": False,
        }


@dataclass(frozen=True)
class FormationCandidateRejection:
    source: str
    reason: str
    detail: str
    candidate_key: Optional[tuple] = None
    candidate_id: Optional[int] = None

    def __post_init__(self):
        if not isinstance(self.source, str) or not self.source:
            raise FormationValidationError(
                "INVALID_REJECTION_SOURCE")
        if not isinstance(self.reason, str) or not self.reason:
            raise FormationValidationError("INVALID_REJECTION_REASON")
        if not isinstance(self.detail, str):
            raise FormationValidationError("INVALID_REJECTION_DETAIL")

    def as_dict(self):
        return {
            "source": self.source,
            "reason": self.reason,
            "detail": self.detail,
            "candidate_id": self.candidate_id,
            "has_candidate_key": self.candidate_key is not None,
        }


@dataclass(frozen=True)
class FormationMasterResult:
    status: str
    reason: str
    detail: str
    selected: Optional[FormationCandidate]
    candidates: Tuple[FormationCandidate, ...]
    rejections: Tuple[FormationCandidateRejection, ...]
    considered: int
    duplicates: int
    certification: str

    def as_status(self):
        return {
            "schema": "formation_master_result/v1",
            "status": self.status,
            "reason": self.reason,
            "detail": self.detail,
            "certification": self.certification,
            "selected_candidate_id": (
                None if self.selected is None
                else self.selected.candidate_id),
            "candidates": [item.as_status() for item in self.candidates],
            "rejections": [item.as_dict() for item in self.rejections],
            "considered": self.considered,
            "duplicates": self.duplicates,
            "executable": False,
            "trajectory_published": False,
            "independent_timing_audit": False,
        }


class FormationModeMaster:
    """Retain at most K independently audited continuous geometry candidates."""

    def __init__(self, config=None, limits=None):
        self.config = config or FormationMasterConfig()
        if not isinstance(self.config, FormationMasterConfig):
            raise FormationValidationError(
                "INVALID_MASTER_CONFIG expected FormationMasterConfig")
        self.limits = limits or KinematicLimits()
        if not isinstance(self.limits, KinematicLimits):
            raise FormationValidationError(
                "INVALID_KINEMATIC_LIMITS expected KinematicLimits")
        self._candidates = []
        self._rejections = []
        self._seen = set()
        self._next_candidate_id = 1
        self._considered = 0
        self._duplicates = 0

    def _reject(self, source, reason, detail, key=None, candidate_id=None):
        self._rejections.append(FormationCandidateRejection(
            source, reason, detail, key, candidate_id))

    def _record_build_rejections(self, build, source):
        for rejection in build.rejections:
            detail = "block={} primitive={} vehicle={} {}".format(
                rejection.block_index, rejection.primitive_index,
                rejection.vehicle_id, rejection.detail)
            self._reject(source, rejection.reason, detail)

    def reject_build(self, build, source):
        if not isinstance(build, FormationPlanBuild):
            raise FormationValidationError(
                "INVALID_PLAN_BUILD expected FormationPlanBuild")
        if build.success:
            raise FormationValidationError(
                "INVALID_REJECTION a successful build is not a rejection")
        detail = "{} {}".format(build.reason, build.detail).strip()
        self._reject(source, "SEED_REJECTED", detail)
        self._record_build_rejections(build, source)

    def consider(self, plan, snapshot, source="continuous",
                 grid=None, sweep_policy=None, score=None):
        """Audit and admit one plan.  Returns (candidate, rejection_reason)."""
        if not isinstance(plan, FormationContinuousPlan):
            raise FormationValidationError(
                "INVALID_CONTINUOUS_PLAN expected FormationContinuousPlan")
        if not isinstance(source, str) or not source:
            raise FormationValidationError("INVALID_CANDIDATE_SOURCE")
        key = plan.candidate_key()
        if key in self._seen:
            self._duplicates += 1
            self._reject(source, "DUPLICATE_CANDIDATE",
                         "identical geometry/mode/offset/profile candidate",
                         key)
            return None
        self._seen.add(key)
        self._considered += 1
        if self.config.require_collision_precheck and grid is None:
            self._reject(source, "COLLISION_PRECHECK_REQUIRED",
                         "configuration requires a map-backed precheck", key)
            return None
        policy = sweep_policy or SweepPolicy(
            max_corner_step_m=self.config.sweep_spacing_m)
        if not isinstance(policy, SweepPolicy):
            raise FormationValidationError(
                "INVALID_SWEEP_POLICY expected SweepPolicy")
        audit = audit_continuous_plan(
            plan, snapshot, limits=self.limits, grid=grid,
            margin_m=self.config.collision_margin_m,
            sweep_policy=policy)
        if not audit.success:
            self._reject(source, audit.reason, audit.detail, key)
            return None
        if score is None:
            score = plan_candidate_score(plan)
        score = _nonnegative(score, "candidate score")
        certified = replace(plan, certification=KINEMATIC_VALID)
        candidate = FormationCandidate(
            self._next_candidate_id, source, certified, audit, score, key)
        self._next_candidate_id += 1

        if len(self._candidates) >= self.config.max_candidates:
            worst = max(
                self._candidates,
                key=lambda item: (item.score, item.candidate_id))
            if (score, candidate.candidate_id) >= (
                    worst.score, worst.candidate_id):
                self._reject(
                    source, "CANDIDATE_LIMIT_REACHED",
                    "fixed K={} retained; new score={:.12g} is not better "
                    "than worst={:.12g}".format(
                        self.config.max_candidates, score, worst.score),
                    key)
                return None
            self._candidates.remove(worst)
            self._reject(
                worst.source, "EVICTED_BY_BETTER_CANDIDATE",
                "candidate {} score={:.12g} replaced by score={:.12g}".format(
                    worst.candidate_id, worst.score, score),
                worst.key, worst.candidate_id)
        self._candidates.append(candidate)
        self._candidates.sort(key=lambda item: (
            item.score, item.candidate_id))
        return candidate

    def consider_build(self, build, snapshot, source="continuous",
                       grid=None, sweep_policy=None):
        if not isinstance(build, FormationPlanBuild):
            raise FormationValidationError(
                "INVALID_PLAN_BUILD expected FormationPlanBuild")
        if not build.success:
            self.reject_build(build, source)
            return ()
        self._record_build_rejections(build, source)
        admitted = []
        for plan in build.plans:
            candidate = self.consider(
                plan, snapshot, source=source, grid=grid,
                sweep_policy=sweep_policy)
            if candidate is not None:
                admitted.append(candidate)
        return tuple(admitted)

    @property
    def candidates(self):
        return tuple(self._candidates)

    @property
    def rejections(self):
        return tuple(self._rejections)

    def result(self):
        if not self._candidates:
            reason = ("NO_CANDIDATE" if self._considered == 0
                      else "ALL_CANDIDATES_REJECTED")
            detail = ("no plan was considered" if self._considered == 0
                      else "{} considered, 0 admitted".format(
                          self._considered))
            return FormationMasterResult(
                status="REJECTED", reason=reason, detail=detail,
                selected=None, candidates=(),
                rejections=tuple(self._rejections),
                considered=self._considered, duplicates=self._duplicates,
                certification="NONE")
        selected = min(self._candidates, key=lambda item: (
            item.score, item.candidate_id))
        return FormationMasterResult(
            status=KINEMATIC_VALID,
            reason="OK",
            detail="KINEMATIC_ONLY_UNCERTIFIED_SAFETY",
            selected=selected,
            candidates=tuple(self._candidates),
            rejections=tuple(self._rejections),
            considered=self._considered,
            duplicates=self._duplicates,
            certification=KINEMATIC_VALID,
        )

    def as_status(self):
        return self.result().as_status()


def evaluate_seed(snapshot, seed, master, grid=None,
                  expected_map_signature=None):
    """Convenience pipeline: build S6 plans from one seed, then admit them."""
    if not isinstance(master, FormationModeMaster):
        raise FormationValidationError(
            "INVALID_MASTER expected FormationModeMaster")
    from .formation_geometry import build_continuous_plans
    config = master.config
    build = build_continuous_plans(
        snapshot, seed, limits=master.limits,
        max_candidates=config.max_candidates,
        max_schedule_combinations=config.max_schedule_combinations,
        max_schedules_per_block=config.max_schedules_per_block,
        expected_map_signature=expected_map_signature)
    master.consider_build(
        build, snapshot, source="s6_seed", grid=grid,
        sweep_policy=SweepPolicy(
            max_corner_step_m=config.sweep_spacing_m))
    return build