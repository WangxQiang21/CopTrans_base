#!/usr/bin/env python3
"""Replay the bounded legacy terminal-repair provider on archived probe seeds.

This is an offline audit. It reads a frozen probe run and its map/config, then
checks every provider candidate through the restricted-profile continuous
retimer and the full geometry/dynamics/terminal evaluator. It never starts ROS.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))

from diagnose_continuous_seed import load_run
from fwis_v3_optimizer.continuous_retiming import retime_continuous
from fwis_v3_optimizer.continuous_geometry import (
    fit_continuous_endpoint, fit_continuous_suffix, plan_from_actions,
)
from fwis_v3_optimizer.legacy_geometry_provider import (
    LegacyGeometryCandidateProvider,
)


def audit_run(run_dir: Path, budget_s: float, max_candidates: int) -> dict:
    run_dir = run_dir.resolve()
    probe = json.loads((run_dir / "probe_result.json").read_text())
    case = json.loads((run_dir / "case.json").read_text())
    evaluator, actions = load_run(run_dir)
    encoded_actions = json.dumps(
        [{"mode": a.mode, "amount": a.amount, "control": a.control}
         for a in actions], sort_keys=True, separators=(",", ":")).encode()
    seed_audit = evaluator.evaluate(
        actions, timed=None, allow_reverse=evaluator.allow_reverse)
    provider = LegacyGeometryCandidateProvider(
        evaluator, max_candidates=max_candidates)
    candidates, report = provider.generate(
        [(actions, "archived_probe_seed")], budget_s=budget_s)

    candidate_results = []
    for candidate in candidates:
        item = {
            "candidate": candidate.summary(),
            "continuous_seed_preserved": candidate.continuous_plan is not None,
            "shared_execution_audit": None,
        }
        try:
            stage_begin = time.perf_counter()
            continuous_plan = candidate.continuous_plan
            initialization = {
                "strategy": ("preserved_provider_certified_continuous_seed"
                             if continuous_plan is not None else
                             "shared_action_to_continuous_projection"),
                "endpoint_fit": None,
                "projected_sweep_collision": None,
                "unprojected_sweep_collision": None,
                "suffix_repairs": [],
            }
            if continuous_plan is None:
                unprojected_plan = plan_from_actions(
                    evaluator.start, evaluator.goal, list(candidate.actions))
                fit_deadline = stage_begin+8.0
                try:
                    continuous_plan, fit = fit_continuous_endpoint(
                        unprojected_plan, evaluator.limits.r_min_ack,
                        max_iterations=40, terminal_xy_tol=.015,
                        terminal_yaw_tol=0.01308996938995747,
                        deadline=fit_deadline)
                    initialization["endpoint_fit"] = fit
                except (ValueError, RuntimeError) as exc:
                    continuous_plan, fit = unprojected_plan, {
                        "success": False,
                        "reason": str(exc).splitlines()[-1],
                    }
                    initialization["endpoint_fit"] = fit
                projected_collision = (
                    evaluator.continuous_collision(continuous_plan)
                    if fit.get("success") else None)
                initialization["projected_sweep_collision"] = projected_collision
                geometry_ready = bool(fit.get("success") and
                                      projected_collision is None)
                if not fit.get("success") or projected_collision is not None:
                    original_collision = evaluator.continuous_collision(
                        unprojected_plan)
                    initialization["unprojected_sweep_collision"] = original_collision
                    if original_collision is None:
                        for tail_length in (2.0, 4.0):
                            if time.perf_counter() >= fit_deadline:
                                break
                            try:
                                repaired, repair_fit = fit_continuous_suffix(
                                    unprojected_plan, tail_length_m=tail_length,
                                    r_min_ack=evaluator.limits.r_min_ack,
                                    max_iterations=40,
                                    terminal_xy_tol=1e-7,
                                    terminal_yaw_tol=1e-7,
                                    deadline=fit_deadline)
                                collision = (evaluator.continuous_collision(repaired)
                                             if repair_fit.get("success") else None)
                                initialization["suffix_repairs"].append({
                                    "fit": repair_fit,
                                    "first_collision": collision,
                                })
                                if repair_fit.get("success") and collision is None:
                                    continuous_plan, fit = repaired, repair_fit
                                    geometry_ready = True
                                    break
                            except (ValueError, RuntimeError) as exc:
                                initialization["suffix_repairs"].append({
                                    "tail_length_m": tail_length,
                                    "reason": str(exc).splitlines()[-1],
                                })
                    if not geometry_ready:
                        item["shared_execution_audit"] = {
                            "status": "REJECTED",
                            "reason": "shared_continuous_geometry_initialization_failed",
                            "geometry_initialization": initialization,
                            "audit_wall_s": time.perf_counter()-stage_begin,
                        }
                        candidate_results.append(item)
                        continue
            timed = retime_continuous(continuous_plan, evaluator.limits)
            audit = evaluator.evaluate_continuous_geometry(
                continuous_plan, timed,
                xy_tol=evaluator.xy_tol, yaw_tol=evaluator.yaw_tol,
                allow_reverse=evaluator.allow_reverse)
            item["shared_execution_audit"] = {
                "status": audit.get("status"),
                "geometry_status": audit.get("geometry_status"),
                "dynamic_status": audit.get("dynamic_status"),
                "timing_status": audit.get("timing_status"),
                "reason": audit.get("reason"),
                "duration_s": timed.duration,
                "terminal": audit.get("terminal"),
                "metrics": audit.get("metrics"),
                "score": audit.get("score"),
                "geometry_initialization": initialization,
                "audit_wall_s": time.perf_counter()-stage_begin,
            }
        except (ValueError, RuntimeError, ArithmeticError) as exc:
            item["shared_execution_audit"] = {
                "status": "ERROR", "reason": str(exc).splitlines()[-1],
            }
        candidate_results.append(item)

    return {
        "case": case.get("case", run_dir.name),
        "source_run_dir": str(run_dir.relative_to(ROOT)),
        "source_probe_plan_id": probe.get("plan_id"),
        "input": {
            "map": probe.get("map"),
            "start": case.get("start"),
            "goal": case.get("goal"),
            "map_scale": case.get("map_scale"),
            "terminal_tolerance": {
                "xy_m": evaluator.xy_tol, "yaw_rad": evaluator.yaw_tol,
            },
            "sweep_spacing_m": evaluator.profile.sweep_spacing,
            "action_seed_sha256": hashlib.sha256(encoded_actions).hexdigest(),
            "action_count": len(actions),
        },
        "seed_geometry_audit": {
            "status": seed_audit.get("geometry_status"),
            "reason": seed_audit.get("reason"),
            "terminal": seed_audit.get("terminal"),
        },
        "provider": report,
        "candidate_results": candidate_results,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dirs", nargs="+", type=Path,
                        help="frozen probe run directories")
    parser.add_argument("--budget-s", type=float, default=1.5)
    parser.add_argument("--max-candidates", type=int, default=4)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    cases = [audit_run(path, args.budget_s, args.max_candidates)
             for path in args.run_dirs]
    payload = {
        "schema": "r12_legacy_terminal_suffix_repair_audit_v1",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "execution": "offline archived seed replay; no ROS process started",
        "provider_budget_s": args.budget_s,
        "provider_max_candidates": args.max_candidates,
        "case_count": len(cases),
        "provider_candidate_count": sum(
            case["provider"].get("candidate_count", 0) for case in cases),
        "shared_execution_pass_count": sum(
            result.get("shared_execution_audit", {}).get("status") == "FEASIBLE"
            for case in cases for result in case["candidate_results"]),
        "cases": cases,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({k: payload[k] for k in (
        "case_count", "provider_candidate_count", "shared_execution_pass_count")},
        ensure_ascii=False))
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
