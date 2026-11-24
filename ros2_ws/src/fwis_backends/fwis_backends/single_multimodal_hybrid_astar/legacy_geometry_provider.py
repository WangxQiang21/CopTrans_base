"""Bounded legacy geometry sources for the strict mode/path pipeline.

This module deliberately stops at geometry.  It does not publish ROS messages,
create a demo time trace, or certify the restricted execution profile.  Every
candidate is rechecked by the shared geometry evaluator and must still pass
the mode/path retimer and independent execution audit before publication.
"""
from dataclasses import dataclass
import hashlib
import json
import math
import time

from .core import (Action, anchor_mode_sequences, mode_schedule_neighbor_candidates,
                   refine, replay)
from .continuous_geometry import fit_continuous_suffix, plan_from_actions
from .quality import TrajectoryEvaluator


LEGACY_GEOMETRY_VERSION = "legacy_geometry_provider_v1"


@dataclass(frozen=True)
class GeometryCandidate:
    """Immutable geometry snapshot with provenance and strict geometry audit."""
    source: str
    parent_source: str
    actions: tuple
    topology_id: str
    mode_direction_word: tuple
    geometry_version: str
    map_hash: str
    audit: dict
    details: dict
    continuous_plan: object = None

    def summary(self):
        return {
            "source": self.source,
            "parent_source": self.parent_source,
            "topology_id": self.topology_id,
            "mode_direction_word": [list(item) for item in self.mode_direction_word],
            "geometry_version": self.geometry_version,
            "map_hash": self.map_hash,
            "geometry_audit": self.audit,
            "details": self.details,
            "has_certified_continuous_seed": self.continuous_plan is not None,
        }


class LegacyGeometryCandidateProvider:
    """Expose anchor, exact-local-repair, and fixed-corridor legacy geometry.

    The provider keeps its inputs immutable and only admits candidates that
    meet the same terminal, swept-margin, direction, and shunting checks used
    by the new backend. Dynamic feasibility remains the caller's responsibility.
    """

    def __init__(self, evaluator, max_candidates=4, max_window=4,
                 refine_iterations=12, refine_wall_time_s=0.35):
        self.evaluator = evaluator
        self.max_candidates = int(max_candidates)
        self.max_window = int(max_window)
        self.refine_iterations = int(refine_iterations)
        self.refine_wall_time_s = float(refine_wall_time_s)
        if (self.max_candidates < 1 or self.max_window < 1 or
                self.refine_iterations < 1 or
                not math.isfinite(self.refine_wall_time_s) or
                self.refine_wall_time_s <= 0):
            raise ValueError("invalid_legacy_geometry_provider_budget")

    def _map_hash(self):
        grid = self.evaluator.grid
        signature = {
            "occupied_sha256": hashlib.sha256(grid.occ.tobytes()).hexdigest(),
            "width": int(grid.width), "height": int(grid.height),
            "resolution_m": float(grid.res),
            "origin_m": [float(grid.ox), float(grid.oy)],
            "margin_half_dimensions_m": [float(grid.half_l), float(grid.half_w)],
            "sweep_spacing_m": float(self.evaluator.profile.sweep_spacing),
        }
        return hashlib.sha256(json.dumps(
            signature, sort_keys=True, separators=(",", ":")).encode()).hexdigest()

    @staticmethod
    def _topology(actions):
        return tuple((int(action.mode),
                      0 if action.mode == 2 else
                      (1 if action.amount >= 0 else -1))
                     for action in actions)

    def generate(self, seed_candidates, budget_s=1.5):
        """Return strictly geometry-audited alternatives and a full funnel report."""
        if not math.isfinite(float(budget_s)) or float(budget_s) <= 0:
            raise ValueError("invalid_legacy_geometry_provider_wall_time")
        begin = time.perf_counter()
        deadline = begin + float(budget_s)
        report = {
            "schema": "legacy_geometry_provider_report_v1",
            "geometry_version": LEGACY_GEOMETRY_VERSION,
            "budget_s": float(budget_s), "map_hash": self._map_hash(),
            "seed_audits": [], "generated": [], "rejected": [],
            "stage_timings_s": {"seed_audit": 0.0, "anchor_connections": 0.0,
                                "local_exact_repair": 0.0,
                                "fixed_corridor_refine": 0.0,
                                "terminal_suffix_repair": 0.0},
            "status": "IN_PROGRESS",
        }
        accepted = []
        safe_seeds = []
        terminal_repair_seeds = []
        seed_candidates = list(seed_candidates or [])
        seen = {self.evaluator.signature(actions)
                for actions, _ in seed_candidates}

        def continuous_signature(plan):
            return ("continuous_motion_v2", tuple(
                (int(block.mode), int(block.direction), round(float(block.length), 9),
                 round(float(block.signed_angle), 9),
                 tuple((round(float(knot.s), 9), round(float(knot.value), 9),
                        round(float(knot.derivative), 9)) for knot in block.knots))
                for block in plan.blocks))

        def audit(actions, source, parent_source, details, continuous_plan=None):
            actions = tuple(actions)
            key = self.evaluator.signature(actions)
            if continuous_plan is not None:
                key = (key, continuous_signature(continuous_plan))
            if key in seen:
                report["rejected"].append({"source": source,
                                            "parent_source": parent_source,
                                            "reason": "duplicate_geometry"})
                return None
            seen.add(key)
            if continuous_plan is None:
                result = self.evaluator.evaluate(
                    actions, timed=None, allow_reverse=self.evaluator.allow_reverse)
            else:
                result = self.evaluator.evaluate_continuous_geometry(
                    continuous_plan, timed=None,
                    xy_tol=self.evaluator.xy_tol,
                    yaw_tol=self.evaluator.yaw_tol,
                    allow_reverse=self.evaluator.allow_reverse,
                    include_metrics=False)
            if result.get("geometry_status") != "PASS":
                report["rejected"].append({"source": source,
                    "parent_source": parent_source,
                    "reason": result.get("reason") or "strict_geometry_rejected",
                    "geometry_audit": result})
                return None
            candidate = GeometryCandidate(
                source=str(source), parent_source=str(parent_source),
                actions=actions,
                topology_id=hashlib.sha256(repr(key).encode()).hexdigest()[:20],
                mode_direction_word=self._topology(actions),
                geometry_version=LEGACY_GEOMETRY_VERSION,
                map_hash=report["map_hash"], audit=result,
                details=dict(details or {}), continuous_plan=continuous_plan)
            report["generated"].append(candidate.summary())
            accepted.append(candidate)
            return candidate

        for seed_index, (actions, source) in enumerate(seed_candidates):
            if time.perf_counter() >= deadline:
                report["rejected"].append({"source": str(source),
                    "reason": "provider_budget_before_seed_audit"})
                continue
            stage_begin = time.perf_counter()
            actions = tuple(actions)
            coarse = self.evaluator.evaluate(
                actions, timed=None, allow_reverse=self.evaluator.allow_reverse)
            report["seed_audits"].append({
                "source": str(source), "seed_index": seed_index,
                "geometry_status": coarse.get("geometry_status"),
                "reason": coarse.get("reason"),
                "terminal": coarse.get("terminal"),
                "metrics": coarse.get("metrics"),
                "action_count": len(actions),
            })
            report["stage_timings_s"]["seed_audit"] += (
                time.perf_counter()-stage_begin)
            if coarse.get("geometry_status") == "PASS":
                safe_seeds.append((actions, str(source)))
            elif coarse.get("reason") == "terminal":
                # evaluate() checks action values, direction, and the shared
                # shunting policy before it reports terminal error. The raw
                # action sweep is screened separately because terminal failure
                # is raised before the evaluator reaches its sweep check.
                try:
                    action_poses = replay(self.evaluator.start, actions)
                    if self.evaluator.sweep_grid.trajectory_clear(
                            action_poses, actions,
                            self.evaluator.profile.sweep_spacing):
                        terminal_repair_seeds.append(
                            (actions, str(source), coarse))
                        report["seed_audits"][-1][
                            "terminal_repair_eligible"] = True
                    else:
                        report["seed_audits"][-1][
                            "terminal_repair_eligible"] = False
                        report["seed_audits"][-1][
                            "terminal_repair_rejection"] = "action_swept_collision"
                except (ValueError, RuntimeError) as exc:
                    report["seed_audits"][-1][
                        "terminal_repair_eligible"] = False
                    report["seed_audits"][-1][
                        "terminal_repair_rejection"] = str(exc).splitlines()[-1]

        # An old action route can be safe along its body sweep but miss the
        # strict goal pose. Keep its topology as a seed and repair only a
        # bounded suffix of the continuous curve. The unmodified prefix must
        # already pass the 5 mm sweep; every repaired result gets the complete
        # strict geometry audit before entering the shared retimer.
        for actions, source, seed_audit in terminal_repair_seeds:
            if len(accepted) >= self.max_candidates or time.perf_counter() >= deadline:
                break
            stage_begin = time.perf_counter()
            try:
                plan = plan_from_actions(self.evaluator.start,
                                         self.evaluator.goal, list(actions))
                initial_collision = self.evaluator.continuous_collision(plan)
                if initial_collision is not None:
                    report["rejected"].append({
                        "source": "legacy_terminal_suffix_repair",
                        "parent_source": source,
                        "reason": "continuous_seed_swept_collision",
                        "first_collision": initial_collision})
                    continue
                repair_attempts = []
                for tail_length in (2.0, 4.0):
                    remaining = deadline-time.perf_counter()
                    if remaining <= .05:
                        break
                    try:
                        repaired, fit = fit_continuous_suffix(
                            plan, tail_length_m=tail_length,
                            r_min_ack=self.evaluator.limits.r_min_ack,
                            max_iterations=max(40, self.refine_iterations),
                            terminal_xy_tol=min(1e-7, self.evaluator.xy_tol/1000.),
                            terminal_yaw_tol=min(1e-7, self.evaluator.yaw_tol/1000.),
                            deadline=deadline)
                        repair_attempts.append(dict(fit))
                        if not fit.get("success"):
                            continue
                        # audit() performs the complete strict geometry check,
                        # including this same 5 mm swept-collision gate. Do not
                        # pre-scan the repaired plan here: that repeated every
                        # footprint query and consumed most of Figure3's
                        # bounded provider budget without adding an acceptance
                        # condition.
                        candidate = audit(
                            actions, "legacy_terminal_suffix_repair", source,
                            {"operation": "fit_continuous_suffix",
                             "tail_length_m": tail_length,
                             "fixed_prefix_length_m": fit.get(
                                 "fixed_prefix_length_m"),
                             "seed_terminal": seed_audit.get("terminal"),
                             "action_schedule_is_topology_seed": True,
                             "repair_attempts": repair_attempts},
                            continuous_plan=repaired)
                        if candidate is not None:
                            break
                    except (ValueError, RuntimeError) as exc:
                        repair_attempts.append({"tail_length_m": tail_length,
                            "reason": str(exc).splitlines()[-1]})
                if not any(item.source == "legacy_terminal_suffix_repair" and
                           item.parent_source == source for item in accepted):
                    report["rejected"].append({
                        "source": "legacy_terminal_suffix_repair",
                        "parent_source": source,
                        "reason": "no_strictly_safe_suffix_repair",
                        "attempts": repair_attempts})
            except (ValueError, RuntimeError) as exc:
                report["rejected"].append({
                    "source": "legacy_terminal_suffix_repair",
                    "parent_source": source,
                    "reason": str(exc).splitlines()[-1]})
            finally:
                report["stage_timings_s"]["terminal_suffix_repair"] += (
                    time.perf_counter()-stage_begin)

        if safe_seeds and time.perf_counter() < deadline:
            actions, source = safe_seeds[0]
            remaining = deadline-time.perf_counter()
            # `refine()` also builds a corridor NLP and strictly audits its
            # returned action schedule. Reserve measured time for that work
            # before starting the solver, whose wall-time stop can return
            # after its requested limit under load.
            refine_audit_reserve_s = .75
            if (len(accepted) < self.max_candidates and
                    remaining > self.refine_wall_time_s+refine_audit_reserve_s):
                stage_begin = time.perf_counter()
                wall_time = self.refine_wall_time_s
                try:
                    poses, refined, _, meta = refine(
                        self.evaluator.start, self.evaluator.goal,
                        list(actions), self.evaluator.grid,
                        xy_tol=self.evaluator.xy_tol,
                        yaw_tol=self.evaluator.yaw_tol,
                        max_iter=self.refine_iterations,
                        max_wall_time_s=wall_time)
                    audit(refined, "legacy_fixed_corridor_refine", source,
                          {"solver": meta.get("solver", "fixed_corridor_ipopt"),
                           "max_iterations": self.refine_iterations,
                           "profile": meta.get("profile", {}),
                           "endpoint": [poses[-1].x, poses[-1].y, poses[-1].th],
                           "wall_time_limit_s": wall_time})
                except Exception as exc:
                    report["rejected"].append({"source": "legacy_fixed_corridor_refine",
                        "parent_source": source,
                        "reason": str(exc).splitlines()[-1],
                        "wall_time_limit_s": wall_time})
                report["stage_timings_s"]["fixed_corridor_refine"] += (
                    time.perf_counter()-stage_begin)
            elif len(accepted) < self.max_candidates:
                report["rejected"].append({
                    "source": "legacy_fixed_corridor_refine",
                    "parent_source": source,
                    "reason": "fixed_corridor_budget_reserve",
                    "remaining_budget_s": max(0., remaining),
                    "required_reserve_s": (self.refine_wall_time_s+
                                           refine_audit_reserve_s),
                })

            # Prefer the fixed-corridor recovery early when it is the only
            # bounded source likely to change a strict-safe seed. Anchor and
            # local schedule searches are optional diversity work; run them
            # only when there is measured time left after a usable candidate
            # may already have been established.
            anchor_reserve_s = min(.40, .25*float(budget_s))
            remaining = deadline-time.perf_counter()
            if (len(accepted) < self.max_candidates and
                    remaining > anchor_reserve_s):
                stage_begin = time.perf_counter()
                try:
                    anchors = anchor_mode_sequences(
                        self.evaluator.start, actions, self.evaluator.grid,
                        max_candidates=min(2, self.max_candidates), beam_width=12)
                    for schedule, score in anchors:
                        if (len(accepted) >= self.max_candidates or
                                time.perf_counter() >= deadline):
                            break
                        audit(schedule, "legacy_anchor_connection", source,
                              {"common_score": float(score),
                               "operation": "anchor_mode_sequences"})
                except Exception as exc:
                    report["rejected"].append({"source": "legacy_anchor_connection",
                        "parent_source": source,
                        "reason": str(exc).splitlines()[-1]})
                report["stage_timings_s"]["anchor_connections"] += (
                    time.perf_counter()-stage_begin)
            elif len(accepted) < self.max_candidates:
                report["rejected"].append({
                    "source": "legacy_anchor_connection",
                    "parent_source": source,
                    "reason": "anchor_connection_budget_reserve",
                    "remaining_budget_s": max(0., remaining),
                    "required_reserve_s": anchor_reserve_s,
                })

            local_repair_reserve_s = min(.60, .40*float(budget_s))
            remaining = deadline-time.perf_counter()
            if (accepted and len(accepted) < self.max_candidates and
                    remaining > local_repair_reserve_s):
                parent = accepted[0]
                stage_begin = time.perf_counter()
                try:
                    neighbors = mode_schedule_neighbor_candidates(
                        self.evaluator.start, list(parent.actions),
                        self.evaluator.grid,
                        max_candidates=min(2, self.max_candidates-len(accepted)),
                        max_window=self.max_window, sweep_step=.005)
                    for score, proposal, _, details in neighbors:
                        if (len(accepted) >= self.max_candidates or
                                time.perf_counter() >= deadline):
                            break
                        audit(proposal, "legacy_local_exact_repair",
                              parent.source,
                              dict(details, common_score=float(score)))
                except Exception as exc:
                    report["rejected"].append({"source": "legacy_local_exact_repair",
                        "parent_source": parent.source,
                        "reason": str(exc).splitlines()[-1]})
                report["stage_timings_s"]["local_exact_repair"] += (
                    time.perf_counter()-stage_begin)
            elif accepted and len(accepted) < self.max_candidates:
                report["rejected"].append({
                    "source": "legacy_local_exact_repair",
                    "parent_source": accepted[0].source,
                    "reason": "local_repair_budget_reserve",
                    "remaining_budget_s": max(0., remaining),
                    "required_reserve_s": local_repair_reserve_s,
                })

        report["candidate_count"] = len(accepted)
        report["elapsed_s"] = time.perf_counter()-begin
        report["budget_exceeded"] = report["elapsed_s"] > float(budget_s)
        report["status"] = "GEOMETRY_CANDIDATES" if accepted else "NO_GEOMETRY_CANDIDATE"
        return accepted, report
