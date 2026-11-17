#!/usr/bin/env python3
"""Run the three frozen C0-B pipeline cases once each after the guard fix.

Inputs come only from tests/fixtures/virtual_a_corridor/cases.json.  The map
grid is rebuilt with the float32 resolution that ROS publishes, so B/C match
the historical grid SHA 33b3... instead of the direct-YAML 1fa8... digest.
No Git, ROS, branch or file outside evaluation/ is written by this runner.

Compared with the C0-A runner, the audit record also carries the independent
geometry audit's integration budget (``report.integration["audit_integration"]``)
so the same-candidate budget can be checked in the pipeline result.
"""
import json
import math
import platform
import sys
from pathlib import Path

import numpy as np

from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_continuous_a.collision import Grid, load_map
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    GoalPolicy,
    InitialAssumption,
    VirtualARequest,
    VirtualAMapRequest,
    collision_config_from_dict,
    grid_signature,
    planner_config_from_dict,
    planning_context_sha256,
    request_fingerprint,
    timing_config_from_dict,
)
from fwis_v3_optimizer.formation_continuous_a.ros_adapter import solve
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits

ROOT = Path(__file__).resolve().parents[4]
CASES = ROOT / "tests" / "fixtures" / "virtual_a_corridor" / "cases.json"


def load_spec():
    return json.loads(CASES.read_text(encoding="utf-8"))


def model_and_shapes(spec):
    model = FormationModel([
        VehicleOffset(row["vehicle_id"], row["x"], row["y"])
        for row in spec["model"]["offsets"]])
    shapes = {
        int(vehicle_id): VehicleShape(row["length_m"], row["width_m"])
        for vehicle_id, row in spec["model"]["vehicle_shapes"].items()}
    return model, shapes


def build_case(spec, name):
    case = spec["cases"][name]
    map_spec = spec["maps"][name]
    model, shapes = model_and_shapes(spec)
    start = FormationPose(**case["start_qf"])
    goal = FormationPose(**case["goal_qf"])
    request = VirtualARequest(
        plan_id=int(case["plan_id"]),
        formation_id=case["formation_id"],
        model=model,
        vehicle_shapes=shapes,
        start_qf=start,
        start_headings={
            int(vehicle_id): float(value)
            for vehicle_id, value in case["start_headings"].items()},
        goal_qf=goal,
        initial=InitialAssumption(**case["initial"]),
        goal_policy=GoalPolicy(case["goal_policy"]),
        limits=FormationTimingLimits(**spec["limits"]))
    grid, metadata = load_map(ROOT / map_spec["yaml"])
    grid = Grid(
        grid.occ.copy(), float(np.float32(grid.res)), grid.ox, grid.oy)
    planner = planner_config_from_dict(spec["planner_config"])
    timing = timing_config_from_dict(spec["timing_config"])
    collision = collision_config_from_dict(spec["collision_config"])
    map_request = VirtualAMapRequest(request, grid, planner, timing, collision)
    identity = {
        "request_sha256": request_fingerprint(request),
        "grid_sha256": grid_signature(grid),
        "planning_context_sha256": planning_context_sha256(
            map_request.context()),
        "expected_request_sha256": case["request_sha256"],
        "expected_grid_sha256": map_spec["grid_sha256"],
        "map_yaml_sha256": metadata.get("yaml_sha256"),
        "map_image_sha256": metadata.get("image_sha256"),
        "grid_resolution_m": float(grid.res),
        "grid_origin": [float(grid.ox), float(grid.oy)],
        "grid_shape": [int(grid.occ.shape[0]), int(grid.occ.shape[1])],
    }
    return map_request, identity


def audit_integration_record(report):
    integration = getattr(report, "integration", None)
    if not isinstance(integration, dict):
        return None
    budget = integration.get("audit_integration")
    if not isinstance(budget, dict):
        return None
    return {
        "method": budget.get("method"),
        "fixed_threshold_m": budget.get("fixed_threshold_m"),
        "total_budget_m": budget.get("total_budget_m"),
        "euclidean_remainder_m": budget.get("euclidean_remainder_m"),
        "evaluation_guard_m": budget.get("evaluation_guard_m"),
        "angle_guard_m": budget.get("angle_guard_m"),
        "roundoff_and_angle_guard_m":
            budget.get("roundoff_and_angle_guard_m"),
        "angle_bound_rad": budget.get("angle_bound_rad"),
        "total_subintervals": budget.get("total_subintervals"),
        "full_simpson_panels": budget.get("full_simpson_panels"),
        "local_simpson_panels": budget.get("local_simpson_panels"),
        "refinement_steps": budget.get("refinement_steps"),
        "confirmed": budget.get("confirmed"),
        "floating_point_assumption":
            budget.get("floating_point_assumption"),
    }


def audit_record(report):
    if report is None:
        return None
    return {
        "passed": bool(getattr(report, "passed", False)),
        "reason": getattr(report, "reason", ""),
        "detail": str(getattr(report, "detail", "")),
        "first_failure": getattr(report, "first_failure", None),
        "integration_budget": audit_integration_record(report),
    }


def attempt_record(attempt):
    return {
        "index": int(attempt.attempt_index),
        "source": attempt.initial_source,
        "label": attempt.initial_label,
        "solver": attempt.solver,
        "solver_status": attempt.solver_status,
        "status": attempt.status,
        "iterations": int(attempt.iterations),
        "objective": attempt.objective,
        "constraint_residual": attempt.constraint_residual,
        "endpoint_error_m": attempt.endpoint_error_m,
        "heading_error_rad": attempt.heading_error_rad,
        "wall_time_s": attempt.wall_time_s,
        "audit_passed": (
            None if attempt.candidate_audit is None
            else bool(attempt.candidate_audit.passed)),
        "rejection_reason": attempt.rejection_reason,
        "audit_first_failure": (
            None if attempt.candidate_audit is None
            else attempt.candidate_audit.first_failure),
        "audit_integration_budget": (
            None if attempt.candidate_audit is None
            else audit_integration_record(attempt.candidate_audit)),
        "detail": str(attempt.detail),
    }


def path_record(path):
    if path is None:
        return None
    knots = path.profile.knots
    return {
        "length_m": float(path.length_m),
        "knot_count": len(knots),
        "knots": [
            {"s": float(knot.s), "kappa": float(knot.value),
             "eta": float(knot.derivative)}
            for knot in knots],
        "max_abs_kappa": max(abs(float(knot.value)) for knot in knots),
        "max_abs_eta": max(abs(float(knot.derivative)) for knot in knots),
        "position_error_bound_m": float(path.position_error_bound_m),
        "angle_error_bound_rad": float(path.angle_error_bound_rad),
    }


def result_record(result, request):
    routes = getattr(result, "routes", ()) or ()
    path = getattr(result, "path", None)
    record = {
        "status": getattr(result, "status", ""),
        "reason": getattr(result, "reason", ""),
        "first_failure_layer": getattr(result, "first_failure_layer", ""),
        "wall_time_s": float(getattr(result, "wall_time_s", 0.0)),
        "executable": bool(getattr(result, "executable", False)),
        "deadline_exhausted": bool(
            getattr(result, "deadline_exhausted", False)),
        "attempts": [
            attempt_record(attempt)
            for attempt in getattr(result, "attempts", ()) or ()],
        "routes": [
            {"index": index, "source": route.source,
             "points": len(route.points), "length_m": float(route.length_m)}
            for index, route in enumerate(routes)],
        "path": path_record(path),
        "geometry_audit": audit_record(
            getattr(result, "geometry_audit", None)),
        "dynamic_audit": audit_record(
            getattr(result, "dynamic_audit", None)),
        "collision_audit": audit_record(
            getattr(result, "collision_audit", None)),
    }
    if path is not None:
        radio = path.endpoint()
        record["endpoint_error_m"] = math.hypot(
            radio.x - request.goal_qf.x, radio.y - request.goal_qf.y)
        wrapped = (radio.psi - request.goal_qf.psi + math.pi) % (
            2.0 * math.pi) - math.pi
        record["heading_error_rad"] = abs(wrapped)
        plan = getattr(result, "plan", None)
        if plan is not None:
            record["plan"] = {
                "duration_s": float(plan.duration_s),
                "request_sha256": getattr(plan, "request_sha256", ""),
                "planning_context_sha256": getattr(
                    result, "planning_context_sha256", ""),
                "model_sha256": getattr(result, "model_sha256", ""),
                "is_executable_path": bool(getattr(
                    plan, "is_executable_path", False)),
            }
    return record


def main():
    spec = load_spec()
    output = {
        "schema": "virtual_a_corridor.c0b.results.v1",
        "base_commit": spec["base_commit"],
        "run_utc": "20261117T052817Z",
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "numpy": np.__version__,
            "scipy": getattr(__import__("scipy"), "__version__", ""),
            "casadi": getattr(__import__("casadi"), "__version__", ""),
        },
        "cases": {},
    }
    for name in ("A", "B", "C"):
        map_request, identity = build_case(spec, name)
        print("CASE", name, json.dumps(identity, sort_keys=True), flush=True)
        result = solve(map_request)
        record = result_record(result, map_request.request)
        record["identity"] = identity
        output["cases"][name] = record
        print("RESULT", name, result.status, result.reason,
              result.first_failure_layer, "wall=%.3f" % result.wall_time_s,
              flush=True)
        print(json.dumps(record, indent=1), flush=True)
    destination = Path(__file__).resolve().parent / "results.json"
    destination.write_text(
        json.dumps(output, indent=1, sort_keys=True) + "\n",
        encoding="utf-8")
    print("WROTE", destination, flush=True)


if __name__ == "__main__":
    main()