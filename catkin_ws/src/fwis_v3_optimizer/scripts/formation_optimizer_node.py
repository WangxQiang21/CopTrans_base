#!/usr/bin/env python3
"""S9 formation optimizer: S6 build -> optional S7 -> S8 retime -> S9 gate.

The ROS-independent part of this file owns the complete planning pipeline and
the transactional publication state.  The ROS adapter only converts messages
and publishes the events produced by :class:`FormationNodeCore`.

No trajectory is published merely because a geometry or timed candidate was
built.  ``FormationTimedTrajectory`` is emitted only after
``select_formation_candidate`` returns ``FEASIBLE`` and the publication gate
accepts the result for the current ``plan_id``.  A new request, a failed seed,
a changed map, or an invalidated publication clears the previous trajectory.
"""
from collections.abc import Mapping as MappingABC
from dataclasses import dataclass, field, replace
import json
import math
import time as _time
from typing import Mapping, Optional, Tuple

import numpy as np

from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationValidationError,
)
from fwis_v3_optimizer.formation_collision import SweepPolicy
from fwis_v3_optimizer.formation_geometry import (
    FormationContinuousPlan,
    FormationPlanBuild,
    build_continuous_plans,
)
from fwis_v3_optimizer.formation_hold import (
    HOLD_CONTRACT,
    NO_MOTION,
    build_hold_timed_trajectory,
    evaluate_formation_hold,
    hold_failure,
    zero_motion_seed,
)
from fwis_v3_optimizer.formation_kinematics import (
    KinematicLimits,
    normalize_allowed_modes,
)
from fwis_v3_optimizer.formation_master import (
    FormationMasterConfig,
    FormationMasterResult,
    FormationModeMaster,
)
from fwis_v3_optimizer.formation_optimizer import (
    FormationOptimizerConfig,
    optimize_continuous_plan,
)
from fwis_v3_optimizer.formation_quality import (
    FEASIBLE,
    FORMATION_RETIMING_INFEASIBLE,
    GEOMETRY_ONLY_NOT_EXECUTABLE,
    INVALID,
    INVALID_REQUEST,
    REJECTED,
    FormationPublicationGate,
    FormationQualityConfig,
    FormationSelection,
    FormationTimedCandidate,
    grid_signature,
    select_formation_candidate,
)
from fwis_v3_optimizer.formation_request import (
    INVALID_TIMED_TRAJECTORY,
    validate_formation_request,
    validate_formation_timed_trajectory,
)
from fwis_v3_optimizer.formation_retiming import (
    FORMATION_RETIMING_INFEASIBLE as RETIMING_INFEASIBLE,
    FormationTimedTrajectory,
    FormationTimingLimits,
    retime_formation_plan,
)


STATUS_SCHEMA = "formation_optimizer_status/v1"
QUALITY_SCHEMA = "formation_quality_report/v1"
PIPELINE_SCHEMA = "formation_pipeline_result/v1"

_ALLOWED_PIPELINE_STATUS = (
    FEASIBLE, NO_MOTION, REJECTED, GEOMETRY_ONLY_NOT_EXECUTABLE, INVALID)
_ALLOWED_REJECTION_STAGES = (
    "build", "master", "optimization", "retiming", "hold")
_NANOSECONDS = 1_000_000_000


def _positive_integer(value, context):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise FormationValidationError(
            "INVALID_POSITIVE_INTEGER context={} value={!r}".format(
                context, value))
    return value


def _positive_finite(value, context):
    if isinstance(value, bool):
        raise FormationValidationError(
            "INVALID_POSITIVE_VALUE context={} value={!r}".format(
                context, value))
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise FormationValidationError(
            "INVALID_POSITIVE_VALUE context={} value={!r}".format(
                context, value)) from exc
    if not math.isfinite(result) or result <= 0.0:
        raise FormationValidationError(
            "INVALID_POSITIVE_VALUE context={} value={!r}".format(
                context, value))
    return result


def _stamp_tuple(value):
    """Return a non-zero ``(secs, nsecs)`` tuple or ``None``."""
    if value is None:
        return None
    if (isinstance(value, (str, bytes)) or isinstance(value, MappingABC) or
            not isinstance(value, (tuple, list)) or len(value) != 2):
        return None
    secs, nsecs = value
    if isinstance(secs, bool) or not isinstance(secs, int) or secs < 0:
        return None
    if (isinstance(nsecs, bool) or not isinstance(nsecs, int) or
            not 0 <= nsecs < _NANOSECONDS):
        return None
    if secs == 0 and nsecs == 0:
        return None
    return secs, nsecs


def _message_stamp(message):
    header = getattr(message, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return _stamp_tuple((
        getattr(stamp, "secs", None), getattr(stamp, "nsecs", None)))


def _seed_primitive_key(seed):
    primitives = getattr(seed, "primitives", ())
    try:
        items = []
        for primitive in primitives:
            values = []
            for name in ("geometry_type", "dx", "dy", "dpsi",
                         "pivot_x", "pivot_y"):
                value = getattr(primitive, name, None)
                if isinstance(value, bool):
                    values.append(("bool", value))
                elif isinstance(value, (int, float)):
                    values.append(round(float(value), 12))
                else:
                    values.append(("other", repr(value)))
            items.append(tuple(values))
        return tuple(items)
    except Exception:
        return (("unreadable", repr(primitives)),)


def _identity(message):
    plan_id = getattr(message, "plan_id", None)
    formation_id = getattr(message, "formation_id", None)
    return (plan_id if isinstance(plan_id, int) and
            not isinstance(plan_id, bool) else None,
            formation_id if isinstance(formation_id, str) else None)


@dataclass(frozen=True)
class FormationPipelineConfig:
    """Limits and candidate policy for one S9 planning attempt.

    ``quality_config`` is the single source of truth for S6/S7 kinematic
    limits, S8 timing limits and S9 collision/publication sampling.  No second
    set of safety limits is introduced here.
    """

    quality_config: FormationQualityConfig = field(
        default_factory=FormationQualityConfig)
    max_candidates: int = 4
    max_schedule_combinations: int = 64
    max_schedules_per_block: int = 4
    planning_dt_s: float = 0.02
    enable_nlp: bool = True
    require_collision_precheck: bool = True
    optimizer_config: FormationOptimizerConfig = field(
        default_factory=FormationOptimizerConfig)
    allowed_modes: Optional[Tuple[int, ...]] = None

    def __post_init__(self):
        if not isinstance(self.quality_config, FormationQualityConfig):
            raise FormationValidationError(
                "INVALID_FORMATION_QUALITY_CONFIG")
        for name in ("max_candidates", "max_schedule_combinations",
                     "max_schedules_per_block"):
            _positive_integer(getattr(self, name), name)
        object.__setattr__(self, "planning_dt_s", _positive_finite(
            self.planning_dt_s, "planning_dt_s"))
        if not isinstance(self.enable_nlp, bool):
            raise FormationValidationError(
                "INVALID_ENABLE_NLP expected bool")
        if not isinstance(self.require_collision_precheck, bool):
            raise FormationValidationError(
                "INVALID_REQUIRE_COLLISION_PRECHECK expected bool")
        if not isinstance(self.optimizer_config, FormationOptimizerConfig):
            raise FormationValidationError(
                "INVALID_OPTIMIZER_CONFIG")
        object.__setattr__(
            self, "allowed_modes",
            normalize_allowed_modes(self.allowed_modes))

    @property
    def kinematic_limits(self):
        return self.quality_config.kinematic_limits

    @property
    def timing_limits(self):
        return self.quality_config.timing_limits


@dataclass(frozen=True)
class FormationPipelineRejection:
    """One candidate rejected before or during independent S9 evaluation."""

    stage: str
    reason: str
    detail: str = ""
    candidate_id: Optional[int] = None
    source_candidate_id: Optional[int] = None
    failure_code: Optional[str] = None
    constraint: str = ""
    vehicle_id: Optional[int] = None
    block_index: Optional[int] = None

    def __post_init__(self):
        if self.stage not in _ALLOWED_REJECTION_STAGES:
            raise FormationValidationError(
                "INVALID_REJECTION_STAGE {}".format(self.stage))
        if not isinstance(self.reason, str) or not self.reason:
            raise FormationValidationError("INVALID_REJECTION_REASON")
        if not isinstance(self.detail, str):
            raise FormationValidationError("INVALID_REJECTION_DETAIL")
        if self.failure_code is not None and not isinstance(
                self.failure_code, str):
            raise FormationValidationError("INVALID_REJECTION_FAILURE_CODE")

    def as_dict(self):
        return {
            "stage": self.stage,
            "candidate_id": self.candidate_id,
            "source_candidate_id": self.source_candidate_id,
            "reason": self.reason,
            "failure_code": self.failure_code,
            "constraint": self.constraint,
            "vehicle_id": self.vehicle_id,
            "block_index": self.block_index,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class FormationPipelineCandidate:
    """One S8 timed candidate that reached independent S9 evaluation."""

    candidate_id: int
    source_candidate_id: int
    origin: str
    plan: FormationContinuousPlan
    timed: FormationTimedTrajectory
    optimization: Optional[Mapping] = None

    def __post_init__(self):
        _positive_integer(self.candidate_id, "candidate_id")
        _positive_integer(self.source_candidate_id, "source_candidate_id")
        if self.origin not in ("s6_seed", "s7_optimized",
                               "s7_seed_retained"):
            raise FormationValidationError(
                "INVALID_CANDIDATE_ORIGIN {}".format(self.origin))
        if not isinstance(self.plan, FormationContinuousPlan):
            raise FormationValidationError("INVALID_CANDIDATE_PLAN")
        if not isinstance(self.timed, FormationTimedTrajectory):
            raise FormationValidationError("INVALID_CANDIDATE_TRAJECTORY")
        if self.optimization is not None and not isinstance(
                self.optimization, MappingABC):
            raise FormationValidationError(
                "INVALID_CANDIDATE_OPTIMIZATION")

    def as_dict(self):
        return {
            "candidate_id": self.candidate_id,
            "source_candidate_id": self.source_candidate_id,
            "origin": self.origin,
            "duration": float(self.timed.duration),
            "certification": self.plan.certification,
            "timing_audit_passed": True,
            "optimization": (None if self.optimization is None
                             else dict(self.optimization)),
        }


@dataclass(frozen=True)
class FormationPipelineResult:
    """Result of one S6/S7/S8/S9 attempt for a matching seed."""

    status: str
    reason: str
    detail: str
    build: Optional[FormationPlanBuild]
    master: Optional[FormationMasterResult]
    candidates: Tuple[FormationPipelineCandidate, ...]
    rejections: Tuple[FormationPipelineRejection, ...]
    selection: Optional[FormationSelection]
    timings_s: Tuple[Tuple[str, float], ...] = ()

    def __post_init__(self):
        if self.status not in _ALLOWED_PIPELINE_STATUS:
            raise FormationValidationError(
                "INVALID_PIPELINE_STATUS {}".format(self.status))
        if not isinstance(self.reason, str) or not self.reason:
            raise FormationValidationError("INVALID_PIPELINE_REASON")
        if not isinstance(self.detail, str):
            raise FormationValidationError("INVALID_PIPELINE_DETAIL")
        if self.build is not None and not isinstance(
                self.build, FormationPlanBuild):
            raise FormationValidationError("INVALID_PIPELINE_BUILD")
        if self.master is not None and not isinstance(
                self.master, FormationMasterResult):
            raise FormationValidationError("INVALID_PIPELINE_MASTER")
        if self.selection is not None and not isinstance(
                self.selection, FormationSelection):
            raise FormationValidationError("INVALID_PIPELINE_SELECTION")
        seen = set()
        for candidate in self.candidates:
            if not isinstance(candidate, FormationPipelineCandidate):
                raise FormationValidationError(
                    "INVALID_PIPELINE_CANDIDATE")
            if candidate.candidate_id in seen:
                raise FormationValidationError(
                    "DUPLICATE_PIPELINE_CANDIDATE_ID")
            seen.add(candidate.candidate_id)
        if self.status in (FEASIBLE, NO_MOTION):
            if (self.selection is None or
                    self.selection.status != FEASIBLE or
                    self.selection.selected_timed is None):
                raise FormationValidationError(
                    "PIPELINE_FEASIBLE_WITHOUT_SELECTION")
        elif self.selection is not None and self.selection.status == FEASIBLE:
            raise FormationValidationError(
                "PIPELINE_NONFEASIBLE_WITH_FEASIBLE_SELECTION")

    @property
    def selected_timed(self):
        return (None if self.selection is None
                else self.selection.selected_timed)

    @property
    def selected_evaluation(self):
        return (None if self.selection is None
                else self.selection.selected_evaluation)

    def as_status(self):
        selection = (None if self.selection is None
                     else self.selection.as_status())
        return {
            "schema": PIPELINE_SCHEMA,
            "status": self.status,
            "reason": self.reason,
            "detail": self.detail,
            "build": (None if self.build is None
                      else self.build.as_status()),
            "master": (None if self.master is None
                       else self.master.as_status()),
            "candidates": [item.as_dict() for item in self.candidates],
            "rejections": [item.as_dict() for item in self.rejections],
            "selection": selection,
            "selected_candidate_id": (
                None if self.selection is None
                else self.selection.selected_candidate_id),
            "T_end": (None if self.selected_timed is None
                      else float(self.selected_timed.duration)),
            "candidate_count": len(self.candidates),
            "executable": self.status in (FEASIBLE, NO_MOTION),
            "trajectory_published": False,
            "independent_audit": self.selection is not None,
            "collision_certificate": self.status in (FEASIBLE, NO_MOTION),
            "no_motion": self.status == NO_MOTION,
            "hold_contract": (
                HOLD_CONTRACT if self.status == NO_MOTION else None),
            "timings_s": {name: float(value)
                          for name, value in self.timings_s},
        }


def _build_rejection(item, stage="build"):
    return FormationPipelineRejection(
        stage=stage,
        reason=item.reason,
        detail=item.detail,
        failure_code=item.reason,
        vehicle_id=item.vehicle_id,
        block_index=item.block_index)


def _master_rejection(item):
    return FormationPipelineRejection(
        stage="master",
        reason=item.reason,
        detail=item.detail,
        source_candidate_id=item.candidate_id,
        failure_code=item.reason)


def _no_motion_result(snapshot, seed, grid, build, quality):
    """Return an audited hold result, or ``None`` for a non-hold seed."""
    report = zero_motion_seed(
        snapshot, seed, expected_map_signature=quality.expected_map_signature)
    if report is None:
        return None
    try:
        timed = build_hold_timed_trajectory(
            snapshot, dt_s=quality.publication_dt_s)
        evaluation = evaluate_formation_hold(
            snapshot, timed, grid, quality)
    except FormationValidationError as exc:
        evaluation = hold_failure(
            snapshot.plan_id, snapshot.formation_id, "INVALID_HOLD",
            "could not build the audited hold: {}".format(exc))
    feasible = bool(evaluation.feasible)
    selection = FormationSelection(
        status=evaluation.status,
        selected_candidate_id=None,
        selected_plan=None,
        selected_timed=(timed if feasible else None),
        selected_evaluation=evaluation,
        evaluations=(evaluation,),
        rejections=(),
        candidate_count=0,
        feasible_count=(1 if feasible else 0))
    rejections = ()
    if not feasible:
        failure = evaluation.failure
        rejections = (FormationPipelineRejection(
            stage="hold",
            reason=evaluation.reason,
            detail=evaluation.detail,
            failure_code=evaluation.reason,
            vehicle_id=(None if failure is None else failure.vehicle_id)),)
    return FormationPipelineResult(
        status=(NO_MOTION if feasible else evaluation.status),
        reason=evaluation.reason,
        detail=evaluation.detail,
        build=build, master=None, candidates=(), rejections=rejections,
        selection=selection)


class FormationPlanningPipeline:
    """Build, optionally optimize, retime and independently select candidates."""

    def __init__(self, config=None):
        self.config = config or FormationPipelineConfig()
        if not isinstance(self.config, FormationPipelineConfig):
            raise FormationValidationError(
                "INVALID_FORMATION_PIPELINE_CONFIG")

    def _master_config(self):
        quality = self.config.quality_config
        return FormationMasterConfig(
            max_candidates=self.config.max_candidates,
            max_schedule_combinations=self.config.max_schedule_combinations,
            max_schedules_per_block=self.config.max_schedules_per_block,
            collision_margin_m=quality.collision_margin_m,
            sweep_spacing_m=quality.sweep_spacing_m,
            max_samples_per_block=quality.max_samples_per_segment,
            require_collision_precheck=self.config.require_collision_precheck)

    def _sweep_policy(self):
        quality = self.config.quality_config
        return SweepPolicy(
            max_corner_step_m=quality.sweep_spacing_m,
            max_samples_per_segment=quality.max_samples_per_segment)

    def run(self, snapshot, seed, grid):
        if not isinstance(grid, Grid):
            raise FormationValidationError(
                "INVALID_GRID an exact occupancy map is required")
        quality = self.config.quality_config
        started = _time.perf_counter()
        timings = {}

        def finish(result):
            timings.setdefault("total_s", _time.perf_counter() - started)
            return replace(
                result, timings_s=tuple(sorted(timings.items())))

        build = build_continuous_plans(
            snapshot, seed, limits=quality.kinematic_limits,
            max_candidates=self.config.max_candidates,
            max_schedule_combinations=self.config.max_schedule_combinations,
            max_schedules_per_block=self.config.max_schedules_per_block,
            allowed_modes=self.config.allowed_modes)
        timings["build_s"] = _time.perf_counter() - started
        if not build.success or not build.plans:
            no_motion = _no_motion_result(
                snapshot, seed, grid, build, quality)
            if no_motion is not None:
                return finish(no_motion)
            rejections = tuple(
                _build_rejection(item) for item in build.rejections)
            status = (INVALID if build.reason == "INVALID_SEED"
                      else REJECTED)
            reason = build.reason if build.reason else "NO_CONTINUOUS_CANDIDATE"
            return finish(FormationPipelineResult(
                status=status, reason=reason,
                detail=build.detail, build=build, master=None,
                candidates=(), rejections=rejections, selection=None))

        master = FormationModeMaster(
            self._master_config(), limits=quality.kinematic_limits)
        master_started = _time.perf_counter()
        admitted = master.consider_build(
            build, snapshot, source="s6_seed", grid=grid,
            sweep_policy=self._sweep_policy())
        timings["master_s"] = _time.perf_counter() - master_started
        if not admitted:
            result = master.result()
            return finish(FormationPipelineResult(
                status=REJECTED, reason=result.reason,
                detail=result.detail, build=build, master=result,
                candidates=(), rejections=tuple(
                    _master_rejection(item)
                    for item in result.rejections),
                selection=None))

        timed_candidates = []
        pipeline_candidates = []
        rejections = []
        nlp_s = 0.0
        retiming_s = 0.0
        for master_candidate in admitted:
            source_id = master_candidate.candidate_id
            plan = master_candidate.plan
            optimization = None
            origin = "s6_seed"
            if self.config.enable_nlp:
                nlp_started = _time.perf_counter()
                try:
                    optimized = optimize_continuous_plan(
                        plan, snapshot,
                        limits=quality.kinematic_limits, grid=grid,
                        margin_m=quality.collision_margin_m,
                        sweep_policy=self._sweep_policy(),
                        config=self.config.optimizer_config,
                        allowed_modes=self.config.allowed_modes)
                except Exception as exc:
                    rejections.append(FormationPipelineRejection(
                        stage="optimization",
                        reason="FORMATION_NLP_INFEASIBLE",
                        detail="optimizer raised: {!r}".format(exc),
                        source_candidate_id=source_id,
                        failure_code="FORMATION_NLP_INFEASIBLE"))
                    nlp_s += _time.perf_counter() - nlp_started
                    continue
                nlp_s += _time.perf_counter() - nlp_started
                optimization = optimized.report.as_dict()
                if optimized.plan is None or not optimized.accepted:
                    rejections.append(FormationPipelineRejection(
                        stage="optimization",
                        reason=optimized.reason,
                        detail=optimized.detail,
                        source_candidate_id=source_id,
                        failure_code=optimized.reason))
                    continue
                plan = optimized.plan
                origin = ("s7_optimized" if optimized.reason == "OPTIMIZED"
                          else "s7_seed_retained")

            retime_started = _time.perf_counter()
            try:
                timing = retime_formation_plan(
                    plan, limits=quality.timing_limits,
                    dt=self.config.planning_dt_s)
            except FormationValidationError as exc:
                rejections.append(FormationPipelineRejection(
                    stage="retiming",
                    reason="INVALID_FORMATION_PLAN",
                    detail=str(exc),
                    source_candidate_id=source_id,
                    failure_code="INVALID_FORMATION_PLAN"))
                retiming_s += _time.perf_counter() - retime_started
                continue
            retiming_s += _time.perf_counter() - retime_started
            if timing.timed is None:
                failure = timing.failure
                rejections.append(FormationPipelineRejection(
                    stage="retiming",
                    reason=timing.reason,
                    detail=timing.detail,
                    source_candidate_id=source_id,
                    failure_code=timing.reason,
                    constraint=("" if failure is None else failure.constraint),
                    vehicle_id=(None if failure is None
                                else failure.vehicle_id),
                    block_index=(None if failure is None
                                 else failure.block_index)))
                continue

            candidate_id = len(timed_candidates) + 1
            timed_candidates.append(FormationTimedCandidate(
                candidate_id, plan, timing.timed))
            pipeline_candidates.append(FormationPipelineCandidate(
                candidate_id=candidate_id,
                source_candidate_id=source_id,
                origin=origin,
                plan=plan,
                timed=timing.timed,
                optimization=optimization))

        timings["nlp_s"] = nlp_s
        timings["retiming_s"] = retiming_s
        if not timed_candidates:
            retiming_only = (bool(rejections) and all(
                item.stage == "retiming" and
                item.failure_code == RETIMING_INFEASIBLE
                for item in rejections))
            if retiming_only:
                return finish(FormationPipelineResult(
                    status=GEOMETRY_ONLY_NOT_EXECUTABLE,
                    reason=RETIMING_INFEASIBLE,
                    detail=("all admitted geometry candidates failed "
                            "common-time retiming; no executable trajectory"),
                    build=build, master=master.result(),
                    candidates=(), rejections=tuple(rejections),
                    selection=None))
            first = rejections[0] if rejections else None
            return finish(FormationPipelineResult(
                status=REJECTED,
                reason=("ALL_CANDIDATES_REJECTED" if first is None
                        else first.reason),
                detail=("" if first is None else first.detail),
                build=build, master=master.result(),
                candidates=(), rejections=tuple(rejections),
                selection=None))

        selection_started = _time.perf_counter()
        selection = select_formation_candidate(
            snapshot, tuple(timed_candidates), grid, config=quality)
        timings["selection_s"] = _time.perf_counter() - selection_started
        if selection.status == FEASIBLE:
            timings["audit_s"] = timings["selection_s"]
            return finish(FormationPipelineResult(
                status=FEASIBLE, reason="OK",
                detail=("independent S9 evaluation selected the feasible "
                        "candidate with the least common T_end; publication "
                        "still requires the node transaction gate"),
                build=build, master=master.result(),
                candidates=tuple(pipeline_candidates),
                rejections=tuple(rejections), selection=selection))

        all_evaluations_timing = (
            bool(selection.evaluations) and all(
                evaluation.status == GEOMETRY_ONLY_NOT_EXECUTABLE
                for evaluation in selection.evaluations))
        all_pre_timing = all(
            item.failure_code == RETIMING_INFEASIBLE
            for item in rejections)
        timings["audit_s"] = timings["selection_s"]
        if all_evaluations_timing and all_pre_timing:
            return finish(FormationPipelineResult(
                status=GEOMETRY_ONLY_NOT_EXECUTABLE,
                reason=RETIMING_INFEASIBLE,
                detail=("geometry candidates were independently rejected "
                        "for timing/dynamic infeasibility"),
                build=build, master=master.result(),
                candidates=tuple(pipeline_candidates),
                rejections=tuple(rejections), selection=selection))
        first = selection.rejections[0] if selection.rejections else None
        return finish(FormationPipelineResult(
            status=REJECTED,
            reason=("ALL_CANDIDATES_REJECTED" if first is None
                    else first["rejection_reason"]),
            detail=("" if first is None else first.get("detail", "")),
            build=build, master=master.result(),
            candidates=tuple(pipeline_candidates),
            rejections=tuple(rejections), selection=selection))


def timed_trajectory_fields(snapshot, timed, stamp):
    """Build ROS-message field values, failing closed on malformed input."""
    try:
        return _timed_trajectory_fields(snapshot, timed, stamp)
    except FormationValidationError:
        raise
    except (AttributeError, KeyError, IndexError, TypeError,
            ValueError, OverflowError) as exc:
        raise FormationValidationError(
            "INVALID_TIMED_TRAJECTORY conversion failed: {!r}".format(
                exc)) from exc


def _timed_trajectory_fields(snapshot, timed, stamp):
    """Implementation of :func:`timed_trajectory_fields`."""
    if not isinstance(timed, FormationTimedTrajectory):
        raise FormationValidationError(
            "INVALID_FORMATION_TIMED_TRAJECTORY")
    if (getattr(snapshot, "plan_id", None) != timed.plan_id or
            getattr(snapshot, "formation_id", None) != timed.formation_id):
        raise FormationValidationError(
            "TIMED_IDENTITY_MISMATCH snapshot plan/formation differs")
    stamp = _stamp_tuple(stamp)
    if stamp is None:
        raise FormationValidationError(
            "INVALID_PUBLICATION_STAMP expected non-zero (secs, nsecs)")
    try:
        model = snapshot.model
        if not isinstance(model, FormationModel):
            raise FormationValidationError("INVALID_REQUEST_MODEL")
        offsets = tuple(model.offsets)
        if offsets != tuple(timed.offsets):
            raise FormationValidationError(
                "TIMED_OFFSETS_MISMATCH request offsets")
        start_headings = dict(snapshot.start_headings)
        goal_headings = dict(snapshot.goal_headings)
        free_flags = dict(snapshot.goal_heading_free)
        shapes = dict(snapshot.vehicle_shapes)
        start_centers = model.centers(snapshot.start)
        goal_centers = model.centers(snapshot.goal)
    except (AttributeError, TypeError, ValueError) as exc:
        raise FormationValidationError(
            "INVALID_REQUEST for trajectory conversion") from exc
    if set(start_headings) != set(timed.vehicle_ids):
        raise FormationValidationError("TIMED_VEHICLE_IDS_MISMATCH")

    times = tuple(float(value) for value in timed.times_from_start)
    if (not times or times[0] != 0.0 or any(
            second <= first for first, second in zip(times, times[1:]))):
        raise FormationValidationError(
            "TIMES_NOT_STRICTLY_INCREASING")
    if len(timed.formation_poses) != len(times):
        raise FormationValidationError(
            "FORMATION_POSE_COUNT_MISMATCH")
    if set(timed.trajectories) != set(timed.vehicle_ids):
        raise FormationValidationError("TIMED_TRAJECTORY_SET_MISMATCH")
    vehicles = []
    trajectories = []
    for offset in offsets:
        vehicle_id = offset.vehicle_id
        shape = shapes.get(vehicle_id)
        if shape is None:
            raise FormationValidationError(
                "MISSING_VEHICLE_SHAPE vehicle_id={}".format(vehicle_id))
        vehicles.append({
            "vehicle_id": vehicle_id,
            "offset": (float(offset.x), float(offset.y), 0.0),
            "start_center": (float(start_centers[vehicle_id][0]),
                             float(start_centers[vehicle_id][1]),
                             float(start_headings[vehicle_id])),
            "goal_center": (float(goal_centers[vehicle_id][0]),
                            float(goal_centers[vehicle_id][1]),
                            float(goal_headings[vehicle_id])),
            "goal_heading_free": bool(free_flags.get(vehicle_id, False)),
            "body_length_m": float(shape.length_m),
            "body_width_m": float(shape.width_m),
        })
        points = tuple(timed.trajectories[vehicle_id])
        if len(points) != len(times):
            raise FormationValidationError(
                "TIMED_POINT_COUNT_MISMATCH vehicle_id={}".format(vehicle_id))
        converted = []
        for index, point in enumerate(points):
            if abs(float(point.time_from_start) - times[index]) > 1e-9:
                raise FormationValidationError(
                    "TIMED_POINT_TIME_MISMATCH vehicle_id={} index={}".format(
                        vehicle_id, index))
            converted.append((
                float(point.time_from_start),
                float(point.center_x), float(point.center_y),
                float(point.theta),
                float(point.center_speed_mps),
                float(point.center_yaw_rate_radps),
                int(point.mode), float(point.mode_control)))
        trajectories.append({
            "vehicle_id": vehicle_id,
            "points": converted,
        })
    return {
        "frame_id": "map",
        "stamp": stamp,
        "plan_id": int(timed.plan_id),
        "formation_id": timed.formation_id,
        "vehicles": vehicles,
        "formation_poses": [
            (float(pose.x), float(pose.y), float(pose.psi))
            for pose in timed.formation_poses],
        "times_from_start": list(times),
        "trajectories": trajectories,
    }


@dataclass(frozen=True)
class FormationNodeEvent:
    """One transactional state transition produced by the node core."""

    status: Mapping
    quality: Mapping
    clear: bool = False
    clear_stamp: Optional[Tuple[int, int]] = None
    clear_plan_id: int = 0
    clear_formation_id: str = ""
    trajectory_fields: Optional[Mapping] = None


class FormationNodeCore:
    """ROS-independent transactional request/seed/map state machine."""

    def __init__(self, pipeline_config=None, gate=None):
        self.pipeline_config = (pipeline_config or
                                FormationPipelineConfig())
        if not isinstance(self.pipeline_config, FormationPipelineConfig):
            raise FormationValidationError(
                "INVALID_FORMATION_PIPELINE_CONFIG")
        self.gate = gate or FormationPublicationGate()
        if not isinstance(self.gate, FormationPublicationGate):
            raise FormationValidationError(
                "INVALID_PUBLICATION_GATE expected FormationPublicationGate")
        self.requests_seen = 0
        self.seeds_seen = 0
        self.snapshot = None
        self.grid = None
        self.map_signature = None
        self.map_error = None
        self.map_frame_id = None
        self.request_map_signature = None
        self.pending_seed = None
        self.pending_seed_stamp = None
        self._seed_attempts = set()
        self._last_request_status = None
        self._last_seed_status = None
        self._last_pipeline = None
        self._last_publication = None

    # ---- status / quality helpers -------------------------------------

    @property
    def plan_id(self):
        return None if self.snapshot is None else self.snapshot.plan_id

    @property
    def formation_id(self):
        return (None if self.snapshot is None
                else self.snapshot.formation_id)

    def _base_status(self, node_state, status, reason="", detail="",
                     feasible=False, published=False, extra=None):
        data = {
            "schema": STATUS_SCHEMA,
            "node_state": node_state,
            "plan_id": self.plan_id,
            "formation_id": self.formation_id,
            "status": status,
            "feasible": bool(feasible),
            "executable": bool(
                published and status in (FEASIBLE, NO_MOTION)),
            "trajectory_published": bool(published),
            "reason": reason,
            "detail": detail,
            "requests_seen": int(self.requests_seen),
            "seeds_seen": int(self.seeds_seen),
            "map_ready": bool(self.grid is not None and
                              self.map_error is None),
            "map_frame_id": self.map_frame_id,
            "map_signature": self.map_signature,
            "request_map_signature": self.request_map_signature,
            "publication_version": int(self.gate.version),
            "published_plan_id": self.gate.published_plan_id,
        }
        if extra:
            data.update(extra)
        return data

    def _quality(self, event_kind, status, reason, detail, *,
                 feasible=False, published=False, failure=None,
                 seed=None, publication=None, extra=None):
        pipeline = (None if self._last_pipeline is None
                    else self._last_pipeline.as_status())
        data = {
            "schema": QUALITY_SCHEMA,
            "event": event_kind,
            "plan_id": self.plan_id,
            "formation_id": self.formation_id,
            "status": status,
            "reason": reason,
            "detail": detail,
            "feasible": bool(feasible),
            "executable": bool(
                published and status in (FEASIBLE, NO_MOTION)),
            "trajectory_published": bool(published),
            "independent_audit": bool(
                pipeline is not None and
                pipeline.get("independent_audit", False)),
            "collision_certificate": bool(
                feasible and pipeline is not None and
                pipeline.get("collision_certificate", False)),
            "requests_seen": int(self.requests_seen),
            "seeds_seen": int(self.seeds_seen),
            "map_ready": bool(self.grid is not None and
                              self.map_error is None),
            "map_frame_id": self.map_frame_id,
            "map_signature": self.map_signature,
            "request_map_signature": self.request_map_signature,
            "publication_version": int(self.gate.version),
            "published_plan_id": self.gate.published_plan_id,
            "request": self._last_request_status,
            "seed": seed if seed is not None else self._last_seed_status,
            "pipeline": pipeline,
            "selection": (None if pipeline is None
                          else pipeline.get("selection")),
            "publication": (publication if publication is not None
                            else self._last_publication),
            "failure": failure,
        }
        if extra:
            data.update(extra)
        return data

    def _event(self, node_state, status, reason="", detail="", *,
               feasible=False, published=False, failure=None, seed=None,
               publication=None, clear=False, stamp=None,
               trajectory_fields=None, extra=None):
        status_doc = self._base_status(
            node_state, status, reason, detail, feasible, published, extra)
        if node_state in ("REQUEST_VALIDATED", "REQUEST_REJECTED"):
            event_kind = "request"
        elif node_state in ("MAP_UNAVAILABLE", "MAP_VERSION_MISMATCH",
                            "MAP_CHANGED_CLEARED"):
            event_kind = "map"
        else:
            event_kind = "plan_seed"
        quality = self._quality(
            event_kind, status, reason, detail, feasible=feasible,
            published=published, failure=failure, seed=seed,
            publication=publication, extra=extra)
        stamp = _stamp_tuple(stamp)
        if stamp is None and self.snapshot is not None:
            stamp = _stamp_tuple(self.snapshot.header_stamp)
        if stamp is None:
            stamp = (1, 0)
        return FormationNodeEvent(
            status=status_doc, quality=quality, clear=bool(clear),
            clear_stamp=stamp,
            clear_plan_id=(0 if self.plan_id is None else int(self.plan_id)),
            clear_formation_id=(self.formation_id or ""),
            trajectory_fields=trajectory_fields)

    # ---- request -------------------------------------------------------

    def on_request(self, message, publication_stamp=None):
        self.requests_seen += 1
        try:
            validation = validate_formation_request(message)
        except Exception as exc:  # the validator is fail-closed already
            validation = None
            validation_error = "node-level validation failure: {!r}".format(
                exc)
        else:
            validation_error = ""
        request_status = (None if validation is None
                          else validation.as_status())
        self._last_request_status = request_status
        request_plan_id, request_formation_id = _identity(message)
        if validation is None or not validation.valid:
            pending_seed = self.pending_seed
            pending_stamp = self.pending_seed_stamp
            decision = self.gate.invalidate()
            self.snapshot = None
            self.request_map_signature = None
            keep_pending = (
                pending_seed is not None and
                isinstance(getattr(pending_seed, "plan_id", None), int) and
                not isinstance(getattr(pending_seed, "plan_id", None), bool)
                and (request_plan_id is None or
                     pending_seed.plan_id >= request_plan_id))
            self.pending_seed = pending_seed if keep_pending else None
            self.pending_seed_stamp = (pending_stamp if keep_pending else None)
            self._seed_attempts.clear()
            self._last_seed_status = None
            self._last_pipeline = None
            reason = (INVALID_REQUEST if validation is None
                      else validation.reason)
            detail = (validation_error if validation is None
                      else validation.detail)
            failure = {
                "code": INVALID_REQUEST,
                "field": ("" if validation is None else validation.field),
                "vehicle_id": (None if validation is None
                               else validation.vehicle_id),
                "detail": detail,
            }
            self._last_publication = decision.as_status()
            return self._event(
                "REQUEST_REJECTED", INVALID, reason, detail,
                failure=failure, publication=decision.as_status(),
                clear=True, stamp=publication_stamp,
                extra={"valid": False,
                       "request": request_status,
                       "request_plan_id": request_plan_id,
                       "request_formation_id": request_formation_id})

        snapshot = validation.snapshot
        pending_seed = self.pending_seed
        pending_stamp = self.pending_seed_stamp
        self.snapshot = snapshot
        self.pending_seed = None
        self.pending_seed_stamp = None
        decision = self.gate.begin_request(snapshot.plan_id)
        self.request_map_signature = self.map_signature
        self._seed_attempts = {
            key for key in self._seed_attempts
            if key[0] == snapshot.plan_id}
        self._last_pipeline = None
        self._last_publication = decision.as_status()
        self._last_seed_status = None

        if self._pending_seed_matches_snapshot(snapshot, pending_seed):
            seed_key = self._seed_key(pending_seed, pending_stamp)
            return self._run_seed(snapshot, pending_seed, pending_stamp,
                                  seed_key, publication_stamp, pre_clear=True)
        if (pending_seed is not None and
                isinstance(getattr(pending_seed, "plan_id", None), int) and
                not isinstance(getattr(pending_seed, "plan_id", None), bool)
                and pending_seed.plan_id > snapshot.plan_id):
            # Keep a newer latched seed for the request that has not arrived
            # yet.  It cannot be used for the current plan_id.
            self.pending_seed = pending_seed
            self.pending_seed_stamp = pending_stamp

        status = self._base_status(
            "REQUEST_VALIDATED", "VALIDATED", "OK",
            "request validated; waiting for a matching formation seed",
            feasible=False, published=False,
            extra={"valid": True,
                   "request": request_status,
                   "publication": decision.as_status(),
                   "vehicle_ids": list(snapshot.vehicle_ids)})
        quality = self._quality(
            "request", "VALIDATED", "OK",
            "request validated; no trajectory is published before S9",
            publication=decision.as_status(),
            extra={"valid": True, "request": request_status})
        return FormationNodeEvent(
            status=status, quality=quality, clear=True,
            clear_stamp=_stamp_tuple(publication_stamp) or
            _stamp_tuple(snapshot.header_stamp) or (1, 0),
            clear_plan_id=int(snapshot.plan_id),
            clear_formation_id=snapshot.formation_id)

    @staticmethod
    def _pending_seed_matches_snapshot(snapshot, seed):
        if seed is None:
            return False
        return (getattr(seed, "plan_id", None) == snapshot.plan_id and
                getattr(seed, "formation_id", None) == snapshot.formation_id)

    # ---- seed ----------------------------------------------------------

    def _seed_key(self, seed, seed_stamp):
        return (getattr(seed, "plan_id", None),
                seed_stamp,
                self.map_signature,
                _seed_primitive_key(seed))

    def on_seed(self, seed, publication_stamp=None, seed_stamp=None):
        self.seeds_seen += 1
        if seed_stamp is None:
            seed_stamp = _message_stamp(seed)
        seed_status = {
            "header_stamp": (None if seed_stamp is None
                             else list(seed_stamp)),
            "plan_id": getattr(seed, "plan_id", None),
            "formation_id": getattr(seed, "formation_id", None),
            "primitive_count": len(getattr(seed, "primitives", ()) or ()),
            "map_signature": self.map_signature,
        }
        self._last_seed_status = seed_status

        if self.snapshot is None:
            if (isinstance(getattr(seed, "plan_id", None), int) and
                    not isinstance(getattr(seed, "plan_id", None), bool) and
                    isinstance(getattr(seed, "formation_id", None), str) and
                    seed_stamp is not None):
                self.pending_seed = seed
                self.pending_seed_stamp = seed_stamp
                return self._event(
                    "SEED_CACHED", "PENDING", "NO_ACTIVE_REQUEST",
                    "seed cached until a matching request arrives",
                    seed=seed_status, clear=False,
                    stamp=publication_stamp,
                    extra={"cached_plan_id": seed.plan_id})
            return self._event(
                "SEED_REJECTED", INVALID, INVALID_REQUEST,
                "seed arrived without a matching active request",
                failure={"code": INVALID_REQUEST, "field": "seed.plan_id",
                         "vehicle_id": None,
                         "detail": "no active request"},
                seed=seed_status, clear=False, stamp=publication_stamp)

        plan_id = getattr(seed, "plan_id", None)
        if not isinstance(plan_id, int) or isinstance(plan_id, bool):
            decision = self.gate.fail(
                self.snapshot.plan_id, INVALID_REQUEST,
                "seed plan_id is not an integer")
            self._last_publication = decision.as_status()
            return self._event(
                "SEED_REJECTED", INVALID, INVALID_REQUEST,
                "seed plan_id is missing or invalid",
                failure={"code": INVALID_REQUEST, "field": "seed.plan_id",
                         "vehicle_id": None,
                         "detail": "missing or non-integer plan_id"},
                seed=seed_status, publication=decision.as_status(),
                clear=decision.clear, stamp=publication_stamp)

        if plan_id < self.snapshot.plan_id:
            return self._event(
                "SEED_STALE", REJECTED, "STALE_PLAN",
                "seed plan_id={} is older than request plan_id={}".format(
                    plan_id, self.snapshot.plan_id),
                seed=seed_status, clear=False, stamp=publication_stamp)

        if plan_id > self.snapshot.plan_id:
            if seed_stamp is not None:
                self.pending_seed = seed
                self.pending_seed_stamp = seed_stamp
            return self._event(
                "SEED_CACHED", "PENDING", "REQUEST_NOT_RECEIVED",
                "seed plan_id={} is newer than request plan_id={}".format(
                    plan_id, self.snapshot.plan_id),
                seed=seed_status, clear=False, stamp=publication_stamp)

        if getattr(seed, "formation_id", None) != self.snapshot.formation_id:
            decision = self.gate.fail(
                self.snapshot.plan_id, INVALID_REQUEST,
                "seed formation_id differs from the active request")
            self._last_publication = decision.as_status()
            return self._event(
                "SEED_REJECTED", INVALID, INVALID_REQUEST,
                "seed formation_id differs from the active request",
                failure={"code": INVALID_REQUEST,
                         "field": "seed.formation_id", "vehicle_id": None,
                         "detail": "formation_id mismatch"},
                seed=seed_status, publication=decision.as_status(),
                clear=decision.clear, stamp=publication_stamp)

        key = self._seed_key(seed, seed_stamp)
        if key in self._seed_attempts:
            return self._event(
                "DUPLICATE_SEED", REJECTED, "DUPLICATE_SEED",
                "the same seed and map were already processed",
                seed=seed_status, clear=False, stamp=publication_stamp)

        return self._run_seed(
            self.snapshot, seed, seed_stamp, key, publication_stamp,
            pre_clear=False)

    def _run_seed(self, snapshot, seed, seed_stamp, seed_key,
                  publication_stamp, pre_clear):
        plan_id = snapshot.plan_id
        seed_status = {
            "header_stamp": (None if seed_stamp is None
                             else list(seed_stamp)),
            "plan_id": getattr(seed, "plan_id", None),
            "formation_id": getattr(seed, "formation_id", None),
            "primitive_count": len(getattr(seed, "primitives", ()) or ()),
            "map_signature": self.map_signature,
        }
        self._last_seed_status = seed_status

        if self.grid is None:
            # Keep the seed so a late /map can trigger the same attempt
            # without requiring the caller to republish it.
            if self._pending_seed_matches_snapshot(snapshot, seed):
                self.pending_seed = seed
                self.pending_seed_stamp = seed_stamp
            decision = self.gate.fail(
                plan_id, INVALID_REQUEST,
                "an exact occupancy map is required")
            self._last_publication = decision.as_status()
            return self._event(
                "MAP_UNAVAILABLE", INVALID, INVALID_REQUEST,
                self.map_error or "no /map has been received",
                failure={"code": INVALID_REQUEST, "field": "map",
                         "vehicle_id": None,
                         "detail": self.map_error or "no map"},
                seed=seed_status, publication=decision.as_status(),
                clear=decision.clear or pre_clear,
                stamp=publication_stamp,
                extra={"planning_started": False, "map_ready": False})

        if (self.request_map_signature is not None and
                self.map_signature != self.request_map_signature):
            decision = self.gate.fail(
                plan_id, INVALID_REQUEST,
                "map signature differs from the request map")
            self._last_publication = decision.as_status()
            return self._event(
                "MAP_VERSION_MISMATCH", INVALID, INVALID_REQUEST,
                "map signature changed between request and seed",
                failure={"code": INVALID_REQUEST,
                         "field": "map_signature", "vehicle_id": None,
                         "detail": "request map signature differs from the "
                                   "current exact occupancy grid"},
                seed=seed_status, publication=decision.as_status(),
                clear=decision.clear or pre_clear,
                stamp=publication_stamp,
                extra={"planning_started": False})

        # A new seed for the same current plan supersedes any old publication
        # before planning starts; it is republished only if it is feasible.
        if pre_clear or self.gate.published_plan_id == plan_id:
            pre_decision = self.gate.begin_request(plan_id)
            pre_clear = True
            self._last_publication = pre_decision.as_status()

        quality_config = replace(
            self.pipeline_config.quality_config,
            expected_map_signature=self.request_map_signature)
        pipeline_config = replace(
            self.pipeline_config, quality_config=quality_config)
        self._last_pipeline = None
        try:
            result = FormationPlanningPipeline(pipeline_config).run(
                snapshot, seed, self.grid)
        except Exception as exc:
            decision = self.gate.invalidate()
            self._last_publication = decision.as_status()
            self._seed_attempts.add(seed_key)
            return self._event(
                "INTERNAL_ERROR", REJECTED, "INTERNAL_ERROR",
                "planning pipeline raised: {!r}".format(exc),
                failure={"code": "INTERNAL_ERROR", "field": "pipeline",
                         "vehicle_id": None,
                         "detail": "pipeline raised: {!r}".format(exc)},
                seed=seed_status, publication=decision.as_status(),
                clear=True, stamp=publication_stamp,
                extra={"planning_started": True})
        self._last_pipeline = result
        self._seed_attempts.add(seed_key)

        if result.status in (FEASIBLE, NO_MOTION):
            decision = self.gate.offer(
                plan_id, result.selected_timed,
                result.selected_evaluation)
            self._last_publication = decision.as_status()
            if decision.publish:
                try:
                    fields = timed_trajectory_fields(
                        snapshot, result.selected_timed,
                        publication_stamp or snapshot.header_stamp)
                except FormationValidationError as exc:
                    failure_decision = self.gate.invalidate()
                    self._last_publication = failure_decision.as_status()
                    return self._event(
                        "PUBLICATION_REJECTED", INVALID,
                        INVALID_TIMED_TRAJECTORY, str(exc),
                        failure={"code": INVALID_TIMED_TRAJECTORY,
                                 "field": "trajectory",
                                 "vehicle_id": None,
                                 "detail": str(exc)},
                        seed=seed_status, feasible=True,
                        publication=failure_decision.as_status(),
                        clear=True, stamp=publication_stamp,
                        extra={"planning_started": True})
                is_hold = result.status == NO_MOTION
                return self._event(
                    ("NO_MOTION_PUBLISHED" if is_hold
                     else "FEASIBLE_PUBLISHED"),
                    result.status, ("NO_MOTION" if is_hold else "OK"),
                    ("zero-motion request: independent hold audit passed "
                     "and the static hold was published through the "
                     "transaction gate" if is_hold else
                     "independent S9 evaluation passed and the current "
                     "plan_id was published through the transaction gate"),
                    feasible=True, published=True,
                    seed=seed_status, publication=decision.as_status(),
                    clear=pre_clear,
                    stamp=(publication_stamp or snapshot.header_stamp),
                    trajectory_fields=fields,
                    extra={"selected_candidate_id": (
                               None if result.selection is None else
                               result.selection.selected_candidate_id),
                           "duration": float(result.selected_timed.duration),
                           "no_motion": is_hold,
                           "hold_contract": (HOLD_CONTRACT if is_hold
                                             else None),
                           "planning_started": True})
            return self._event(
                "STALE_PUBLICATION", result.status, "STALE_PLAN",
                decision.detail, feasible=True, published=False,
                seed=seed_status, publication=decision.as_status(),
                clear=decision.clear or pre_clear,
                stamp=publication_stamp,
                extra={"planning_started": True})

        decision = self.gate.fail(
            plan_id, result.reason, result.detail)
        self._last_publication = decision.as_status()
        node_state = {
            GEOMETRY_ONLY_NOT_EXECUTABLE: "GEOMETRY_ONLY_NOT_EXECUTABLE",
            INVALID: "REJECTED",
            REJECTED: "REJECTED",
        }.get(result.status, "REJECTED")
        return self._event(
            node_state, result.status, result.reason, result.detail,
            feasible=False, published=False,
            seed=seed_status, publication=decision.as_status(),
            clear=decision.clear or pre_clear,
            stamp=publication_stamp,
            extra={"planning_started": True})

    # ---- map -----------------------------------------------------------

    def on_map(self, grid, error=None, frame_id=None, stamp=None):
        previous_signature = self.map_signature
        if grid is None or error:
            self.grid = None
            self.map_signature = None
            self.map_error = error or "invalid map"
            self.map_frame_id = frame_id
            published = self.gate.published_plan_id is not None
            decision = self.gate.invalidate() if published else None
            if decision is not None:
                self._last_publication = decision.as_status()
            return self._event(
                "MAP_UNAVAILABLE", INVALID, INVALID_REQUEST,
                self.map_error,
                failure={"code": INVALID_REQUEST, "field": "map",
                         "vehicle_id": None, "detail": self.map_error},
                publication=(None if decision is None
                             else decision.as_status()),
                clear=published, stamp=stamp,
                extra={"map_ready": False})

        try:
            signature = grid_signature(grid)
        except FormationValidationError as exc:
            return self.on_map(
                None, "invalid map: {}".format(exc), frame_id, stamp)
        self.grid = grid
        self.map_signature = signature
        self.map_error = None
        self.map_frame_id = frame_id
        if (self.snapshot is not None and
                self.request_map_signature is None and
                self.gate.published_plan_id is None):
            # If a request arrived before /map, pin the first exact map that
            # becomes available.  A later change then rejects the seed.
            self.request_map_signature = signature
        if (self.snapshot is not None and
                self.gate.published_plan_id is None and
                self.pending_seed is not None and
                self._pending_seed_matches_snapshot(
                    self.snapshot, self.pending_seed)):
            seed = self.pending_seed
            seed_stamp = self.pending_seed_stamp
            self.pending_seed = None
            self.pending_seed_stamp = None
            return self._run_seed(
                self.snapshot, seed, seed_stamp,
                self._seed_key(seed, seed_stamp), stamp, pre_clear=False)
        changed = (previous_signature is not None and
                   previous_signature != signature)
        if changed and self.gate.published_plan_id is not None:
            decision = self.gate.invalidate()
            self._last_publication = decision.as_status()
            return self._event(
                "MAP_CHANGED_CLEARED", INVALID, "MAP_VERSION_MISMATCH",
                "map signature changed; the published trajectory was cleared",
                failure={"code": INVALID_REQUEST,
                         "field": "map_signature", "vehicle_id": None,
                         "detail": "the map used for publication changed"},
                publication=decision.as_status(), clear=True,
                stamp=stamp,
                extra={"previous_map_signature": previous_signature,
                       "map_signature": signature})
        return None

    # ---- adapter error path -------------------------------------------

    def publication_rejected(self, failure_code, detail,
                             field="trajectory", vehicle_id=None,
                             stamp=None):
        decision = self.gate.invalidate()
        self._last_publication = decision.as_status()
        return self._event(
            "PUBLICATION_REJECTED", INVALID, failure_code, detail,
            feasible=False, published=False,
            failure={"code": failure_code, "field": field,
                     "vehicle_id": vehicle_id, "detail": detail},
            publication=decision.as_status(), clear=True, stamp=stamp,
            extra={"planning_started": True})


# ---- ROS adapter --------------------------------------------------------


def grid_from_occupancy_grid(message):
    """Convert an ``OccupancyGrid`` into the exact ``core.Grid`` used by S9."""
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
            return None, "map frame_id must be 'map', got {!r}".format(
                frame_id)
        if width <= 0 or height <= 0:
            return None, "map dimensions must be positive"
        if not math.isfinite(resolution) or resolution <= 0.0:
            return None, "map resolution must be positive and finite"
        if not (math.isfinite(ox) and math.isfinite(oy)):
            return None, "map origin must be finite"
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not math.isfinite(norm) or norm <= 0.0:
            return None, "map origin quaternion is invalid"
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz))
        if not math.isfinite(yaw) or abs(yaw) > 1e-7:
            return None, "rotated map origins are not supported"
        data = np.asarray(message.data, dtype=np.int8)
        if data.size != width * height:
            return None, "map data length does not match its dimensions"
        # Unknown (-1) is conservatively treated as occupied, matching the
        # exact-grid contract: no unchecked cell is ever assumed free.
        occupied = data.reshape(height, width) != 0
        return Grid(occupied, resolution, ox, oy), None
    except Exception as exc:
        return None, "invalid map message: {!r}".format(exc)


try:  # ROS is optional for the ROS-independent pipeline tests.
    import rospy
    from geometry_msgs.msg import Point, Pose2D
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from fwis_v3_msgs.msg import (FormationPlanRequest, FormationPlanSeed,
                                  FormationTimedPoint,
                                  FormationTimedTrajectory as
                                  FormationTimedTrajectoryMsg,
                                  FormationVehicleSpec,
                                  FormationVehicleTrajectory)
    ROS_AVAILABLE = True
    ROS_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover - environment guard
    rospy = None
    Point = Pose2D = OccupancyGrid = String = None
    (FormationPlanRequest, FormationPlanSeed, FormationTimedPoint,
     FormationTimedTrajectoryMsg, FormationVehicleSpec,
     FormationVehicleTrajectory) = (None, None, None, None, None, None)
    ROS_AVAILABLE = False
    ROS_IMPORT_ERROR = exc


class FormationOptimizerNode:
    """Thin ROS boundary around :class:`FormationNodeCore`."""

    REQUEST_TOPIC = "/formation/plan_request"
    SEED_TOPIC = "/formation/plan_seed"
    MAP_TOPIC = "/map"
    STATUS_TOPIC = "/formation/optimizer_status"
    QUALITY_TOPIC = "/formation/quality_report"
    TRAJECTORY_TOPIC = "/formation/timed_trajectory"

    def __init__(self, core=None):
        if not ROS_AVAILABLE:
            raise RuntimeError(
                "ROS message imports unavailable: {!r}".format(
                    ROS_IMPORT_ERROR))
        self.core = core or FormationNodeCore(
            pipeline_config=_load_pipeline_config())
        self.start_on_publication = bool(rospy.get_param("~start_on_publication", False))
        self.status_pub = rospy.Publisher(
            self.STATUS_TOPIC, String, queue_size=1, latch=True)
        self.quality_pub = rospy.Publisher(
            self.QUALITY_TOPIC, String, queue_size=1, latch=True)
        self.trajectory_pub = rospy.Publisher(
            self.TRAJECTORY_TOPIC, FormationTimedTrajectoryMsg,
            queue_size=1, latch=True)
        rospy.Subscriber(self.MAP_TOPIC, OccupancyGrid, self.on_map,
                         queue_size=1)
        rospy.Subscriber(self.REQUEST_TOPIC, FormationPlanRequest,
                         self.on_request, queue_size=1)
        rospy.Subscriber(self.SEED_TOPIC, FormationPlanSeed, self.on_seed,
                         queue_size=1)

    def _now(self):
        try:
            now = rospy.Time.now()
            if now.secs or now.nsecs:
                return int(now.secs), int(now.nsecs)
        except Exception:
            pass
        snapshot = self.core.snapshot
        if snapshot is not None:
            return tuple(snapshot.header_stamp)
        return 1, 0

    def on_map(self, message):
        grid, error = grid_from_occupancy_grid(message)
        stamp = self._now()
        event = self.core.on_map(
            grid, error=error,
            frame_id=getattr(message.header, "frame_id", None),
            stamp=stamp)
        self._publish_event(event)

    def on_request(self, message):
        self._publish_event(self.core.on_request(
            message, publication_stamp=self._now()))

    def on_seed(self, message):
        self._publish_event(self.core.on_seed(
            message, publication_stamp=self._now(),
            seed_stamp=_message_stamp(message)))

    def _clear_message(self, event):
        message = FormationTimedTrajectoryMsg()
        message.header.frame_id = "map"
        secs, nsecs = event.clear_stamp or (1, 0)
        message.header.stamp = rospy.Time(secs, nsecs)
        message.plan_id = int(event.clear_plan_id)
        message.formation_id = event.clear_formation_id
        return message

    def _timed_message(self, fields):
        message = FormationTimedTrajectoryMsg()
        message.header.frame_id = fields["frame_id"]
        secs, nsecs = fields["stamp"]
        message.header.stamp = rospy.Time(secs, nsecs)
        message.plan_id = int(fields["plan_id"])
        message.formation_id = fields["formation_id"]
        for values in fields["vehicles"]:
            spec = FormationVehicleSpec()
            spec.vehicle_id = int(values["vehicle_id"])
            spec.offset_in_formation = Point(
                x=values["offset"][0], y=values["offset"][1], z=0.0)
            spec.start_center = Pose2D(
                x=values["start_center"][0], y=values["start_center"][1],
                theta=values["start_center"][2])
            spec.goal_center = Pose2D(
                x=values["goal_center"][0], y=values["goal_center"][1],
                theta=values["goal_center"][2])
            spec.goal_heading_free = bool(values["goal_heading_free"])
            spec.body_length_m = float(values["body_length_m"])
            spec.body_width_m = float(values["body_width_m"])
            message.vehicles.append(spec)
        for x, y, theta in fields["formation_poses"]:
            message.formation_poses.append(Pose2D(x=x, y=y, theta=theta))
        for time in fields["times_from_start"]:
            message.times_from_start.append(
                rospy.Duration.from_sec(float(time)))
        for values in fields["trajectories"]:
            trajectory = FormationVehicleTrajectory()
            trajectory.vehicle_id = int(values["vehicle_id"])
            for (time, x, y, theta, speed, yaw_rate, mode,
                 control) in values["points"]:
                point = FormationTimedPoint()
                point.time_from_start = rospy.Duration.from_sec(float(time))
                point.center_pose = Pose2D(
                    x=float(x), y=float(y), theta=float(theta))
                point.center_speed_mps = float(speed)
                point.center_yaw_rate_radps = float(yaw_rate)
                point.mode = int(mode)
                point.mode_control = float(control)
                trajectory.points.append(point)
            message.trajectories.append(trajectory)
        return message

    @staticmethod
    def _json(data):
        return json.dumps(data, sort_keys=True, allow_nan=False)

    def _publish_event(self, event):
        if event is None:
            return
        if event.clear:
            self.trajectory_pub.publish(self._clear_message(event))
        if event.trajectory_fields is not None:
            try:
                fields = event.trajectory_fields
                if self.start_on_publication:
                    # Shift the shared time origin only. Relative times, every
                    # vehicle's samples and all independently audited dynamics
                    # stay identical. Planning time is not playback time.
                    fields = dict(fields, stamp=self._now())
                message = self._timed_message(fields)
            except Exception as exc:
                event = self.core.publication_rejected(
                    INVALID_TIMED_TRAJECTORY,
                    "message conversion failed: {!r}".format(exc),
                    field="trajectory", stamp=self._now())
                self.trajectory_pub.publish(self._clear_message(event))
                self._publish_documents(event)
                return
            validation = validate_formation_timed_trajectory(message)
            if not validation.valid:
                event = self.core.publication_rejected(
                    INVALID_TIMED_TRAJECTORY,
                    "converted trajectory violates the ROS contract: "
                    "{} {}".format(validation.field, validation.detail),
                    field=validation.field,
                    vehicle_id=validation.vehicle_id,
                    stamp=self._now())
                self.trajectory_pub.publish(self._clear_message(event))
                self._publish_documents(event)
                return
            self.trajectory_pub.publish(message)
        self._publish_documents(event)

    def _publish_documents(self, event):
        self.status_pub.publish(String(data=self._json(event.status)))
        self.quality_pub.publish(String(data=self._json(event.quality)))
        state = event.status.get("node_state")
        status = event.status.get("status")
        reason = event.status.get("reason")
        rospy.loginfo(
            "[formation optimizer] state=%s status=%s plan_id=%s "
            "published=%s reason=%s",
            state, status, event.status.get("plan_id"),
            event.status.get("trajectory_published"), reason)


def _load_limits(parameter, factory):
    values = rospy.get_param(parameter, {})
    if not isinstance(values, dict):
        raise FormationValidationError(
            "INVALID_PARAMETER {} must be a dictionary".format(parameter))
    return factory(**values)


def _load_pipeline_config():
    kinematic_limits = _load_limits(
        "~kinematic_limits", KinematicLimits)
    timing_limits = _load_limits(
        "~timing_limits", FormationTimingLimits)
    quality = FormationQualityConfig(
        timing_limits=timing_limits,
        kinematic_limits=kinematic_limits,
        collision_margin_m=rospy.get_param("~collision_margin", 0.05),
        sweep_spacing_m=rospy.get_param("~sweep_spacing", 0.01),
        publication_dt_s=rospy.get_param("~publication_dt", 0.02),
        max_samples_per_segment=rospy.get_param(
            "~max_samples_per_segment", 200000))
    optimizer = FormationOptimizerConfig(
        time_limit_s=rospy.get_param("~optimizer_time_limit", 1.0),
        max_iterations=rospy.get_param("~optimizer_max_iterations", 120))
    return FormationPipelineConfig(
        quality_config=quality,
        max_candidates=rospy.get_param("~max_candidates", 4),
        max_schedule_combinations=rospy.get_param(
            "~max_schedule_combinations", 64),
        max_schedules_per_block=rospy.get_param(
            "~max_schedules_per_block", 4),
        planning_dt_s=rospy.get_param("~planning_dt", 0.02),
        enable_nlp=rospy.get_param("~enable_nlp", True),
        require_collision_precheck=rospy.get_param(
            "~require_collision_precheck", True),
        optimizer_config=optimizer,
        allowed_modes=rospy.get_param("~allowed_modes", None))


def main():
    rospy.init_node("fwis_v3_formation_optimizer")
    try:
        FormationOptimizerNode()
    except Exception as exc:
        rospy.logfatal(
            "[formation optimizer] failed to start: %s", exc)
        raise
    rospy.loginfo(
        "[formation optimizer] S9 independent evaluator ready; only "
        "FEASIBLE trajectories are published on %s",
        FormationOptimizerNode.TRAJECTORY_TOPIC)
    rospy.spin()


if __name__ == "__main__":
    main()
