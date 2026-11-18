"""Replay one frozen scene through the current C3-rework public pipeline.

Evidence harness only.  It calls the unmodified ``plan_virtual_a`` and writes
scheduling/audit evidence as JSON; it does not publish, retime a new law or
touch ROS.  ``--max-initials`` is only used for the configured stricter-limit
exhaustion negative replay.
"""
import argparse
import json
import time
from dataclasses import replace

from fwis_v3_optimizer.formation_continuous_a.pipeline import plan_virtual_a
from virtual_a_g4_support import scene_map_request


def _attempt(record):
    return {
        "attempt_index": record.attempt_index,
        "initial_source": record.initial_source,
        "initial_label": record.initial_label,
        "status": record.status,
        "solver": record.solver,
        "solver_status": record.solver_status,
        "iterations": record.iterations,
        "objective": record.objective,
        "constraint_residual": record.constraint_residual,
        "corridor_residual_m": record.corridor_residual_m,
        "rejection_reason": record.rejection_reason,
        "detail": record.detail,
        "wall_time_s": record.wall_time_s,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scene")
    parser.add_argument("out")
    parser.add_argument("--solver", default="ipopt")
    parser.add_argument(
        "--guidance-mode", default="corridor",
        choices=("legacy_route", "corridor"))
    parser.add_argument("--max-initials", type=int, default=None)
    args = parser.parse_args()
    base, metadata, payload = scene_map_request(args.scene)
    planner_kwargs = {
        "guidance_mode": args.guidance_mode, "solver": args.solver}
    if args.max_initials is not None:
        planner_kwargs["max_initials"] = int(args.max_initials)
    planner = replace(base.planner_config, **planner_kwargs)
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
                "initial_count", "new_initial_count", "initial_mode",
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
            "allocation": entry.get("allocation"),
            "allocation_source": entry.get("allocation_source"),
            "previous_allocation": entry.get("previous_allocation"),
            "reallocation": entry.get("reallocation"),
            "effective_margin_m": entry.get("effective_margin_m"),
            "chain_audit_passed": entry.get("chain_audit", {}).get("passed"),
            "chain_audit_reason": entry.get("chain_audit", {}).get("reason"),
            "guide_source": entry.get("guide", {}).get("source"),
            "guide_points": entry.get("guide", {}).get("points"),
            "containment": entry.get("containment"),
        })
    summary = {
        "scene": args.scene,
        "guidance_mode": args.guidance_mode,
        "solver": args.solver,
        "configured_max_initials": planner.max_initials,
        "planner_config": planner.as_dict(),
        "input_schema": payload.get("schema"),
        "input_stage": payload.get("stage"),
        "grid_metadata_sha256": metadata.get("grid_sha256"),
        "status": result.status,
        "reason": result.reason,
        "detail": result.detail,
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
            result.plan.model_sha256() if result.plan is not None else None),
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
        "initial_count_total": result.search.get("initial_count_total"),
        "initial_limit": result.search.get("initial_limit"),
        "duplicate_initial_launch_count": result.search.get(
            "duplicate_initial_launch_count"),
        "initial_keys": result.search.get("initial_keys"),
        "recovery_count": result.search.get("recovery_count"),
        "first_failure_layer": result.search.get("first_failure_layer"),
        "last_reason": result.search.get("last_reason"),
        "recoveries": result.search.get("recoveries"),
        "reallocation": result.search.get("reallocation"),
        "routes": list(result.search.get("routes", ())),
        "rounds": rounds,
        "corridor_rounds": corridor_rounds,
        "attempts": [_attempt(record) for record in result.attempts],
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
        "false_publications": 0,
        "publication": "NOT_RUN",
        "ros": "NOT_RUN",
    }
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=1, sort_keys=True, default=str)
        handle.write("\n")
    print(json.dumps({
        "scene": args.scene,
        "guidance_mode": args.guidance_mode,
        "max_initials": planner.max_initials,
        "status": result.status,
        "reason": result.reason,
        "route_count": result.search.get("route_count"),
        "initial_count_total": result.search.get("initial_count_total"),
        "initial_limit": result.search.get("initial_limit"),
        "duplicate_initial_launch_count": result.search.get(
            "duplicate_initial_launch_count"),
        "recovery_count": result.search.get("recovery_count"),
        "reallocation_available": (
            result.search.get("reallocation") or {}).get("available"),
        "path_present": result.path is not None,
        "plan_present": result.plan is not None,
        "wall_s": round(measured, 3),
    }, sort_keys=True))


if __name__ == "__main__":
    main()