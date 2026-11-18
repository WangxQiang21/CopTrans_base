"""Run the C2-B rework curved positive case through the real pipeline.

The request is autonomous: it enters ``plan_virtual_a`` unchanged, builds its
own corridor chain/guide/allocation, solves, retimes and passes or fails the
three production audits.  This harness only records evidence; it never builds
a path or injects a solver success.
"""
import argparse
import hashlib
import json
import math
import time
from dataclasses import replace

from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    request_payload,
)
from fwis_v3_optimizer.formation_continuous_a.pipeline import plan_virtual_a
from virtual_a_g4_support import empty_grid, make_request, map_request, PLANNER


START_WORLD = (2.0, 0.0, 0.6)
LOCAL_GOAL = (8.0, 2.0, 0.0)


def world_goal():
    x0, y0, psi0 = START_WORLD
    dx, dy, dpsi = LOCAL_GOAL
    cosine, sine = math.cos(psi0), math.sin(psi0)
    return (x0 + cosine * dx - sine * dy,
            y0 + sine * dx + cosine * dy,
            psi0 + dpsi)


def build_case(solver):
    goal = world_goal()
    request = make_request(
        FormationPose(*START_WORLD),
        FormationPose(*goal),
        start_headings={1: START_WORLD[2], 2: START_WORLD[2],
                        3: START_WORLD[2]})
    grid = empty_grid(width=400, height=300, resolution=0.05,
                      origin_x=-2.0, origin_y=-5.0)
    planner = replace(PLANNER, guidance_mode="corridor", solver=solver)
    return map_request(request, grid, planner=planner)


def _round_records(result):
    records = []
    for entry in result.search.get("corridor", {}).get("rounds", []):
        chain = entry.get("chain", {})
        records.append({
            "round_index": entry.get("round_index"),
            "route_source": entry.get("route_source"),
            "chain_digest": entry.get("chain_digest"),
            "region_count": len(chain.get("regions", [])),
            "regions": chain.get("regions", []),
            "chain_audit": entry.get("chain_audit"),
            "allocation": entry.get("allocation"),
            "effective_margin_m": entry.get("effective_margin_m"),
            "guide": entry.get("guide"),
            "containment": entry.get("containment"),
        })
    return records


def _search_rounds(result):
    keys = (
        "round_index", "geometry_status", "geometry_reason",
        "corridor_containment_passed", "corridor_containment_reason",
        "timing_status", "timing_reason", "dynamics_passed",
        "dynamics_reason", "collision_passed", "collision_reason",
        "collision_first_failure")
    return [{key: record.get(key) for key in keys}
            for record in result.search.get("rounds", [])]


def _attempts(result):
    keys = (
        "attempt_index", "initial_source", "status", "solver",
        "solver_status", "iterations", "objective", "constraint_residual",
        "corridor_residual_m", "rejection_reason", "detail", "wall_time_s")
    return [{key: getattr(attempt, key, None) for key in keys}
            for attempt in result.attempts]


def _audit_record(report):
    if report is None:
        return {"present": False}
    record = {
        "present": True,
        "scope": report.scope,
        "passed": bool(report.passed),
        "reason": getattr(report, "reason", None),
        "detail": getattr(report, "detail", ""),
        "check_count": getattr(report, "check_count", 0),
        "metrics": getattr(report, "metrics", {}),
        "integration": getattr(report, "integration", {}),
        "location": getattr(report, "location", {}),
        "request_sha256": getattr(report, "request_sha256", ""),
        "planning_context_sha256": getattr(
            report, "planning_context_sha256", ""),
        "grid_sha256": getattr(report, "grid_sha256", ""),
    }
    return record


def _path_metrics(result):
    if result.path is None:
        return None
    path = result.path
    count = 400
    stations = [min(index * path.length_m / count, path.length_m)
                for index in range(count + 1)]
    per_vehicle = {}
    reference_peak = 0.0
    for state in (path.state_at_s(station) for station in stations):
        reference_peak = max(reference_peak, abs(state.curvature.kappa))
        for vehicle in state.vehicles:
            per_vehicle[str(vehicle.vehicle_id)] = max(
                per_vehicle.get(str(vehicle.vehicle_id), 0.0),
                abs(vehicle.curvature))
    endpoint = path.endpoint()
    goal = result_goal
    position_error = math.hypot(endpoint.x - goal.x, endpoint.y - goal.y)
    heading_error = abs((endpoint.psi - goal.psi + math.pi) % (2.0 * math.pi)
                        - math.pi)
    return {
        "path_length_m": float(path.length_m),
        "reference_curvature_peak_inv_m": reference_peak,
        "curvature_peak_per_vehicle_inv_m": per_vehicle,
        "endpoint": {"x": endpoint.x, "y": endpoint.y, "psi": endpoint.psi},
        "endpoint_position_error_m": position_error,
        "endpoint_heading_error_rad": heading_error,
        "endpoint_heading_error_deg": math.degrees(heading_error),
        "position_error_bound_m": getattr(
            path, "position_error_bound_m", None),
        "angle_error_bound_rad": getattr(
            path, "angle_error_bound_rad", None),
    }


result_goal = FormationPose(*world_goal())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--solver", default="scipy")
    parser.add_argument("out")
    args = parser.parse_args()
    straight = build_case(args.solver)
    payload = request_payload(straight.request)
    started = time.monotonic()
    result = plan_virtual_a(straight)
    measured = time.monotonic() - started

    geometry = _audit_record(result.geometry_audit)
    dynamics = _audit_record(result.dynamic_audit)
    collision = _audit_record(result.collision_audit)
    sample = (geometry.get("metrics") or {}).get("sample_audit") or {}
    integration = geometry.get("integration") or {}
    path_metrics = _path_metrics(result)
    containment = [
        entry.get("containment")
        for entry in result.search.get("corridor", {}).get("rounds", [])]
    context = result.planning_context_sha256
    publishable = (
        result.status in ("SUCCESS", "NO_MOTION")
        and bool(result.feasible) and bool(result.executable)
        and result.path is not None and result.plan is not None
        and geometry.get("passed") is True
        and dynamics.get("passed") is True
        and collision.get("passed") is True
        and all((entry or {}).get("reason") == "CORRIDOR_CONTAINMENT_VERIFIED"
                for entry in containment))
    summary = {
        "case": "curved_lateral_translation",
        "solver": args.solver,
        "start_world": list(START_WORLD),
        "local_goal": list(LOCAL_GOAL),
        "goal_world": list(world_goal()),
        "start_headings": straight.request.start_headings,
        "request_payload": payload,
        "request_payload_sha256": hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":"),
                       ensure_ascii=False, allow_nan=False).encode("utf-8")
        ).hexdigest(),
        "planner_config": straight.planner_config.as_dict(),
        "timing_config": straight.timing_config.as_dict(),
        "collision_config": straight.collision_config.as_dict(),
        "grid_summary": straight.grid_summary,
        "grid_sha256": result.grid_sha256,
        "model_sha256": result.model_sha256,
        "request_sha256": result.request_sha256,
        "planning_context_sha256": context,
        "status": result.status,
        "reason": result.reason,
        "detail": result.detail,
        "first_failure_layer": result.first_failure_layer,
        "feasible": bool(result.feasible),
        "executable": bool(result.executable),
        "path_present": result.path is not None,
        "plan_present": result.plan is not None,
        "plan_kind": getattr(result.plan, "kind", None),
        "path_metrics": path_metrics,
        "max_pairwise_rigidity_error_m": sample.get(
            "max_pairwise_rigidity_error_m"),
        "sample_audit": sample,
        "endpoint_position_error_m_audited": integration.get(
            "reported_position_error_m"),
        "endpoint_heading_error_rad_audited": integration.get(
            "reported_heading_error_rad"),
        "geometry_audit": geometry,
        "dynamic_audit": dynamics,
        "collision_audit": collision,
        "corridor_rounds": _round_records(result),
        "corridor_containment_reasons": [
            (entry or {}).get("reason") for entry in containment],
        "search_rounds": _search_rounds(result),
        "attempts": _attempts(result),
        "checks": int(result.checks),
        "result_wall_time_s": result.wall_time_s,
        "measured_wall_time_s": measured,
        "timings": result.timings,
        "metrics": result.metrics,
        "publication_eligible_by_evidence_gate": publishable,
        "false_publication": bool(
            result.path is not None or result.plan is not None)
        and not publishable,
    }
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=1, sort_keys=True, default=str)
        handle.write("\n")
    print(json.dumps({
        "solver": args.solver,
        "status": result.status,
        "reason": result.reason,
        "feasible": bool(result.feasible),
        "executable": bool(result.executable),
        "path": result.path is not None,
        "plan": result.plan is not None,
        "containment": summary["corridor_containment_reasons"],
        "kappa_peak": (
            None if path_metrics is None
            else path_metrics["reference_curvature_peak_inv_m"]),
        "endpoint_position_error_m": (
            None if path_metrics is None
            else path_metrics["endpoint_position_error_m"]),
        "endpoint_heading_error_deg": (
            None if path_metrics is None
            else path_metrics["endpoint_heading_error_deg"]),
        "rigidity_error_m": summary["max_pairwise_rigidity_error_m"],
        "audits": [geometry.get("passed"), dynamics.get("passed"),
                   collision.get("passed")],
        "publishable": publishable,
        "wall_s": round(measured, 3),
    }, sort_keys=True))


if __name__ == "__main__":
    main()