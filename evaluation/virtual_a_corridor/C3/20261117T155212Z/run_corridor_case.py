"""Run one C3 corridor case with the frozen C2-B scene input.

Evidence harness only: it calls the unmodified public ``plan_virtual_a`` with
``guidance_mode=corridor`` and writes the scheduling/audit evidence as JSON.
The input is the same frozen scene request/planner config used by the C2-B
corridor cases, so the C2 result can be compared one-to-one.
"""
import argparse
import json
import time
from dataclasses import replace

from fwis_v3_optimizer.formation_continuous_a.pipeline import plan_virtual_a
from virtual_a_g4_support import scene_map_request


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scene")
    parser.add_argument("out")
    parser.add_argument("--solver", default="ipopt")
    parser.add_argument(
        "--guidance-mode", default="corridor",
        choices=("legacy_route", "corridor"))
    args = parser.parse_args()
    base, metadata, _payload = scene_map_request(args.scene)
    planner = replace(
        base.planner_config, guidance_mode=args.guidance_mode,
        solver=args.solver)
    request = replace(base, planner_config=planner)
    started = time.monotonic()
    result = plan_virtual_a(request)
    measured = time.monotonic() - started

    rounds = []
    for record in result.search.get("rounds", []):
        rounds.append({
            key: record.get(key) for key in (
                "round_index", "geometry_status", "geometry_reason",
                "geometry_attempts", "route_index", "route_count",
                "initial_count", "initial_mode",
                "corridor_containment_passed",
                "corridor_containment_reason", "timing_status",
                "timing_reason", "dynamics_passed", "dynamics_reason",
                "collision_passed", "collision_reason",
                "collision_first_failure")})
    corridor_rounds = []
    for entry in result.search.get("corridor", {}).get("rounds", []):
        corridor_rounds.append({
            "round_index": entry.get("round_index"),
            "route_index": entry.get("route_index"),
            "route_source": entry.get("route_source"),
            "chain_digest": entry.get("chain_digest"),
            "region_count": len(entry.get("chain", {}).get("regions", [])),
            "allocation": entry.get("allocation"),
            "effective_margin_m": entry.get("effective_margin_m"),
            "chain_audit_passed": entry.get("chain_audit", {}).get("passed"),
            "chain_audit_reason": entry.get("chain_audit", {}).get("reason"),
            "guide_source": entry.get("guide", {}).get("source"),
            "guide_points": entry.get("guide", {}).get("points"),
            "containment": entry.get("containment"),
        })
    attempts = []
    for attempt in result.attempts:
        attempts.append({
            "attempt_index": attempt.attempt_index,
            "initial_source": attempt.initial_source,
            "initial_label": attempt.initial_label,
            "status": attempt.status,
            "solver": attempt.solver,
            "solver_status": attempt.solver_status,
            "iterations": attempt.iterations,
            "objective": attempt.objective,
            "constraint_residual": attempt.constraint_residual,
            "corridor_residual_m": attempt.corridor_residual_m,
            "rejection_reason": attempt.rejection_reason,
            "detail": attempt.detail,
            "wall_time_s": attempt.wall_time_s,
        })
    summary = {
        "scene": args.scene,
        "guidance_mode": args.guidance_mode,
        "solver": args.solver,
        "planner_config": planner.as_dict(),
        "grid_metadata_sha256": metadata.get("grid_sha256"),
        "status": result.status,
        "reason": result.reason,
        "detail": result.detail,
        "feasible": bool(result.feasible),
        "executable": bool(result.executable),
        "path_present": result.path is not None,
        "plan_present": result.plan is not None,
        "path_length_m": (
            float(result.path.length_m) if result.path is not None else None),
        "request_sha256": result.request_sha256,
        "planning_context_sha256": result.planning_context_sha256,
        "grid_sha256": result.grid_sha256,
        "model_sha256": result.model_sha256,
        "plan_request_sha256": (
            result.plan.request_sha256 if result.plan is not None else None),
        "plan_planning_context_sha256": (
            result.plan.planning_context_sha256()
            if result.plan is not None else None),
        "plan_model_sha256": (
            result.plan.model_sha256()
            if result.plan is not None else None),
        "model_binding_ok": bool(
            result.plan is not None
            and result.plan.request_sha256 == result.request_sha256
            and result.plan.planning_context_sha256()
            == result.planning_context_sha256
            and result.plan.model_sha256() == result.model_sha256),
        "wall_time_s": result.wall_time_s,
        "measured_wall_s": measured,
        "metrics": result.metrics,
        "timings": result.timings,
        "budget": result.search.get("budget"),
        "route_count": result.search.get("route_count"),
        "initial_count": result.search.get("initial_count"),
        "recovery_count": result.search.get("recovery_count"),
        "first_failure_layer": result.search.get("first_failure_layer"),
        "last_reason": result.search.get("last_reason"),
        "recoveries": result.search.get("recoveries"),
        "routes": list(result.search.get("routes", ())),
        "rounds": rounds,
        "corridor_rounds": corridor_rounds,
        "attempts": attempts,
        "geometry_audit_passed": (
            None if result.geometry_audit is None
            else bool(result.geometry_audit.passed)),
        "dynamic_audit_passed": (
            None if result.dynamic_audit is None
            else bool(result.dynamic_audit.passed)),
        "collision_audit_passed": (
            None if result.collision_audit is None
            else bool(result.collision_audit.passed)),
        "checks": int(result.checks),
    }
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=1, sort_keys=True, default=str)
        handle.write("\n")
    print(json.dumps({
        "scene": args.scene,
        "status": result.status,
        "reason": result.reason,
        "route_count": result.search.get("route_count"),
        "initial_count": result.search.get("initial_count"),
        "recovery_count": result.search.get("recovery_count"),
        "first_failure_layer": result.search.get("first_failure_layer"),
        "last_reason": result.search.get("last_reason"),
        "path_present": result.path is not None,
        "wall_s": round(measured, 3),
    }, sort_keys=True))


if __name__ == "__main__":
    main()