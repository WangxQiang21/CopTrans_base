"""Run one C2-B pipeline case and dump the evidence fields as JSON.

This is an evidence harness, not production code.  It calls the unchanged
``plan_virtual_a`` pipeline with an explicit guidance mode and stores the
result plus the corridor rounds, search records and attempt diagnostics.
"""
import argparse
import json
import time
from dataclasses import replace

from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a.pipeline import plan_virtual_a
from virtual_a_g4_support import (
    load_synthetic, make_request, map_request, scene_map_request)


def _case_request(name):
    if name == "door_circumcircle":
        grid, metadata = load_synthetic(name)
        request = make_request(
            FormationPose(-1.5, 0.0, 0.0), FormationPose(9.0, 0.0, 0.0))
        return map_request(request, grid), metadata, {}
    return scene_map_request(name)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scene")
    parser.add_argument("mode", choices=("legacy_route", "corridor"))
    parser.add_argument("out")
    parser.add_argument("--solver", default="ipopt")
    parser.add_argument("--max-seconds", type=float, default=None)
    parser.add_argument("--max-initials", type=int, default=None)
    args = parser.parse_args()

    base, _metadata, _payload = _case_request(args.scene)
    changes = {"guidance_mode": args.mode, "solver": args.solver}
    if args.max_seconds is not None:
        changes["max_seconds"] = float(args.max_seconds)
    if args.max_initials is not None:
        changes["max_initials"] = int(args.max_initials)
    planner = replace(base.planner_config, **changes)
    request = replace(base, planner_config=planner)
    started = time.monotonic()
    result = plan_virtual_a(request)
    measured = time.monotonic() - started

    corridor_rounds = []
    for entry in result.search.get("corridor", {}).get("rounds", []):
        containment = entry.get("containment")
        corridor_rounds.append({
            "round_index": entry.get("round_index"),
            "route_source": entry.get("route_source"),
            "chain_digest": entry.get("chain_digest"),
            "region_count": len(entry.get("chain", {}).get("regions", [])),
            "region_rows": [
                len(region["A"])
                for region in entry.get("chain", {}).get("regions", [])],
            "allocation": entry.get("allocation"),
            "effective_margin_m": entry.get("effective_margin_m"),
            "chain_audit_passed": entry.get(
                "chain_audit", {}).get("passed"),
            "chain_audit_reason": entry.get(
                "chain_audit", {}).get("reason"),
            "guide_source": entry.get("guide", {}).get("source"),
            "guide_points": entry.get("guide", {}).get("points"),
            "guide_quality": entry.get("guide", {}).get("quality"),
            "containment": containment,
        })
    rounds = []
    for record in result.search.get("rounds", []):
        rounds.append({
            key: record.get(key) for key in (
                "round_index", "geometry_status", "geometry_reason",
                "corridor_containment_passed",
                "corridor_containment_reason", "timing_status",
                "timing_reason", "dynamics_passed", "dynamics_reason",
                "collision_passed", "collision_reason",
                "collision_first_failure")})
    attempts = []
    for attempt in result.attempts:
        attempts.append({
            "attempt_index": attempt.attempt_index,
            "initial_source": attempt.initial_source,
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
    kappa_peaks = None
    kappa_reference_peak = None
    if result.path is not None:
        path = result.path
        station_count = 400
        stations = [
            min(index * path.length_m / station_count, path.length_m)
            for index in range(station_count + 1)]
        per_vehicle = {}
        reference_peak = 0.0
        for state in (path.state_at_s(station) for station in stations):
            reference_peak = max(reference_peak, abs(state.curvature.kappa))
            for vehicle in state.vehicles:
                previous = per_vehicle.get(vehicle.vehicle_id, 0.0)
                per_vehicle[vehicle.vehicle_id] = max(
                    previous, abs(vehicle.curvature))
        kappa_peaks = per_vehicle
        kappa_reference_peak = reference_peak
    summary = {
        "scene": args.scene,
        "guidance_mode": args.mode,
        "planner_config": planner.as_dict(),
        "kappa_peaks_per_vehicle": kappa_peaks,
        "kappa_reference_peak": kappa_reference_peak,
        "status": result.status,
        "reason": result.reason,
        "detail": result.detail,
        "feasible": bool(result.feasible),
        "executable": bool(result.executable),
        "path_present": result.path is not None,
        "plan_present": result.plan is not None,
        "path_length_m": (
            float(result.path.length_m) if result.path is not None else None),
        "plan_kind": getattr(result.plan, "kind", None),
        "request_sha256": result.request_sha256,
        "planning_context_sha256": result.planning_context_sha256,
        "grid_sha256": result.grid_sha256,
        "wall_time_s": result.wall_time_s,
        "measured_wall_s": measured,
        "metrics": result.metrics,
        "timings": result.timings,
        "corridor_rounds": corridor_rounds,
        "search_rounds": rounds,
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
        "mode": args.mode,
        "status": result.status,
        "reason": result.reason,
        "path_present": result.path is not None,
        "corridor_containment": [
            (entry.get("containment") or {}).get("reason")
            for entry in result.search.get("corridor", {}).get(
                "rounds", [])],
        "wall_s": round(measured, 3),
    }, sort_keys=True))


if __name__ == "__main__":
    main()