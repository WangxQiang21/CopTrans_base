#!/usr/bin/env python3
"""C0-B targeted reproduction of the frozen C U-turn candidates.

The C0-A run did not store the solver profile (S, kappa knots, eta knots), so
C0-B performs exactly one targeted reproduction.  It rebuilds the C request and
grid from ``tests/fixtures/virtual_a_corridor/cases.json`` with the ROS float32
resolution, runs the unchanged offline adapter once, and captures every
candidate that reaches ``audit_geometry``.  For each captured candidate it also
replays the audit integration-plan refinement loop with instrumentation so the
same-candidate budget can be decomposed into Simpson remainder, guard, angle
term and panel counts.

The ``decomposition`` block intentionally replays the pre-C0-B (G1) budget:
``bounds.position_guard`` with ``POSITION_ROUNDOFF_GUARD_M`` per Simpson panel.
It is the frozen "before" decomposition for C0-B, not the current audit; the
post-fix same-candidate budget is recomputed by ``budget_analysis.py`` with the
implemented ``integration_bounds.evaluation_guard``.

Read-only with respect to the tracked source; writes only ``candidates.json``
and stdout next to this script.
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
from fwis_v3_optimizer.formation_continuous_a import audit as audit_module
from fwis_v3_optimizer.formation_continuous_a import bounds
from fwis_v3_optimizer.formation_continuous_a import (
    integration_bounds,
)
from fwis_v3_optimizer.formation_continuous_a import optimizer as optimizer_module
from fwis_v3_optimizer.formation_continuous_a.collision import Grid, load_map
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    GeometryAuditConfig,
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

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
CASES = ROOT / "tests" / "fixtures" / "virtual_a_corridor" / "cases.json"


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


def profile_record(path):
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


def audit_failure_record(report):
    return {
        "passed": bool(report.passed),
        "reason": report.reason,
        "detail": str(report.detail),
        "first_failure": getattr(report, "first_failure", None),
        "integration_available":
            getattr(report, "integration", None) is not None,
    }


def _initial_records(pieces, config):
    """The exact initial state constructed by audit._integration_plan."""
    base = []
    for piece in pieces:
        panels = audit_module._even_at_least(
            piece.h / config.audit_panel_width_m)
        jets = bounds.segment_jet_bounds(
            piece.c0, piece.c1, piece.c2, piece.c3, piece.h, 0.0, 1.0)
        k_upper = max(abs(jets["k"][0]), abs(jets["k"][1]))
        eta_upper = max(abs(jets["eta"][0]), abs(jets["eta"][1]))
        u_upper = max(abs(jets["u"][0]), abs(jets["u"][1]))
        g_upper = max(abs(jets["g"][0]), abs(jets["g"][1]))
        coarse_m4 = bounds.m4_upper(
            k_upper, eta_upper, u_upper, g_upper)
        record = {
            "segment_index": len(base),
            "s0": piece.s0, "s1": piece.s1, "h": piece.h,
            "c0": piece.c0, "c1": piece.c1, "c2": piece.c2,
            "c3": piece.c3,
            "subintervals": panels,
            "k_upper": k_upper, "eta_upper": eta_upper,
            "u_upper": u_upper, "g_upper": g_upper,
            "coarse_m4": coarse_m4,
        }
        audit_module._refresh_panel_bounds(record)
        base.append(record)
    return base


def _plan_step(base, pieces, start_psi):
    psi_span = abs(start_psi) + sum(
        record["h"] * record["k_upper"] for record in base)
    angle_bound = bounds.angle_budget(len(pieces), psi_span)
    remainder = integration_bounds.position_budget([
        (record["h"], record["subintervals"], record["panel_m4s"])
        for record in base])
    guard = bounds.position_guard(
        pieces[-1].s1,
        remainder["full_panels"] + remainder["local_panels"],
        angle_bound)
    total_budget = bounds._next_up(remainder["euclidean_m"] + guard)
    segments = []
    for index, record in enumerate(base):
        segment_remainder = remainder["segment_budgets"][index]
        segments.append({
            "segment_index": index,
            "s0": record["s0"],
            "s1": record["s1"],
            "h": record["h"],
            "subintervals": record["subintervals"],
            "panel_width_m": 2.0 * record["h"] / record["subintervals"],
            "full_panel_count": segment_remainder["full_panel_count"],
            "full_remainder_m": segment_remainder["full_remainder_m"],
            "local_remainder_m": segment_remainder["local_remainder_m"],
            "segment_remainder_m": segment_remainder["segment_remainder_m"],
            "maximum_panel_m4_upper_m":
                segment_remainder["maximum_panel_m4_upper_m"],
            "coarse_triangle_m4_upper_m": record["coarse_m4"],
            "k_upper": record["k_upper"],
            "eta_upper": record["eta_upper"],
            "u_upper": record["u_upper"],
            "g_upper": record["g_upper"],
        })
    return {
        "segments": segments,
        "per_coordinate_remainder_m": remainder["per_coordinate_m"],
        "euclidean_remainder_m": remainder["euclidean_m"],
        "total_subintervals": remainder["total_subintervals"],
        "full_simpson_panels": remainder["full_panels"],
        "local_simpson_panels": remainder["local_panels"],
        "angle_bound_rad": angle_bound,
        "psi_span_abs": psi_span,
        "roundoff_term_m": bounds.POSITION_ROUNDOFF_GUARD_M * (
            remainder["full_panels"] + remainder["local_panels"] + 8),
        "angle_guard_term_m": pieces[-1].s1 * angle_bound,
        "roundoff_and_angle_guard_m": guard,
        "total_budget_m": total_budget,
    }


def decompose(path, config):
    """Replay the pre-C0-B audit plan loop and record every iteration.

    ``_plan_step`` uses the G1 ``bounds.position_guard`` exactly as the audit
    did when it rejected these candidates in C0-A; this is the frozen "before"
    trace, not the current audit's budget.
    """
    pieces, _stations, _values, _derivatives = audit_module._independent_pieces(
        path.profile.knots)
    base = _initial_records(pieces, config)
    threshold = config.audit_position_budget_m
    trace = []
    refinement_steps = 0
    best = None
    while True:
        step = _plan_step(base, pieces, path.start_qf.psi)
        step["refinement_steps"] = refinement_steps
        trace.append(step)
        if best is None or step["total_budget_m"] < best["total_budget_m"]:
            best = step
        if step["total_budget_m"] <= threshold:
            break
        for record in base:
            if record["subintervals"] > config.audit_max_subintervals_per_segment:
                step["termination"] = "PANEL_LIMIT"
                break
        if "termination" in step:
            break
        total_subintervals = sum(record["subintervals"] for record in base)
        if total_subintervals > config.audit_max_total_subintervals:
            step["termination"] = "TOTAL_LIMIT"
            break
        if refinement_steps >= audit_module._AUDIT_REFINEMENT_LIMIT:
            step["termination"] = "REFINEMENT_LIMIT"
            break
        worst = max(
            range(len(base)),
            key=lambda index: step["segments"][index]["segment_remainder_m"])
        increment = max(8, 2 * ((base[worst]["subintervals"] // 8) // 2))
        candidate = base[worst]["subintervals"] + increment
        if candidate > config.audit_max_subintervals_per_segment:
            step["termination"] = "PANEL_LIMIT"
            break
        base[worst]["subintervals"] = candidate
        audit_module._refresh_panel_bounds(base[worst])
        refinement_steps += 1
    return {
        "threshold_m": threshold,
        "refinement_limit": audit_module._AUDIT_REFINEMENT_LIMIT,
        "panel_width_config_m": config.audit_panel_width_m,
        "initial": trace[0],
        "minimum": best,
        "final": trace[-1],
        "trace_length": len(trace),
        "trace": trace,
    }


def main():
    spec = json.loads(CASES.read_text(encoding="utf-8"))
    map_request, identity = build_case(spec, "C")
    print("IDENTITY", json.dumps(identity, sort_keys=True), flush=True)
    captured = []
    real_audit = optimizer_module.audit_geometry

    def capturing_audit(request, path, config=None, **kwargs):
        report = real_audit(request, path, config, **kwargs)
        captured.append({
            "path": path,
            "report": report,
            "config": config,
            "audit_config": kwargs.get("geometry_audit_config"),
        })
        print("AUDIT capture index={} passed={} reason={} detail={}".format(
            len(captured) - 1, report.passed, report.reason,
            str(report.detail)[:200]), flush=True)
        return report

    optimizer_module.audit_geometry = capturing_audit
    try:
        result = solve(map_request)
    finally:
        optimizer_module.audit_geometry = real_audit
    print("RESULT", result.status, result.reason,
          result.first_failure_layer, "wall=%.3f" % result.wall_time_s,
          flush=True)
    attempts = []
    for attempt in getattr(result, "attempts", ()) or ():
        attempts.append({
            "index": int(attempt.attempt_index),
            "status": attempt.status,
            "solver": attempt.solver,
            "solver_status": attempt.solver_status,
            "rejection_reason": attempt.rejection_reason,
            "audit_first_failure": (
                None if attempt.candidate_audit is None
                else attempt.candidate_audit.first_failure),
            "iterations": int(attempt.iterations),
            "objective": attempt.objective,
            "wall_time_s": attempt.wall_time_s,
            "initial_label": attempt.initial_label,
            "initial_source": attempt.initial_source,
        })
    candidates = []
    for index, item in enumerate(captured):
        path = item["path"]
        config = item["config"] or GeometryAuditConfig()
        candidates.append({
            "capture_index": index,
            "profile": profile_record(path),
            "audit": audit_failure_record(item["report"]),
            "decomposition": decompose(path, config),
        })
    output = {
        "schema": "virtual_a_corridor.c0b.candidates.v1",
        "run_utc": "20261117T052817Z",
        "base_commit": spec["base_commit"],
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "numpy": np.__version__,
            "scipy": getattr(__import__("scipy"), "__version__", ""),
            "casadi": getattr(__import__("casadi"), "__version__", ""),
        },
        "identity": identity,
        "result": {
            "status": result.status,
            "reason": result.reason,
            "first_failure_layer": result.first_failure_layer,
            "wall_time_s": result.wall_time_s,
            "executable": bool(getattr(result, "executable", False)),
            "attempts": attempts,
        },
        "candidate_count": len(candidates),
        "candidates": candidates,
    }
    destination = HERE / "candidates.json"
    destination.write_text(
        json.dumps(output, indent=1, sort_keys=True) + "\n",
        encoding="utf-8")
    print("WROTE", destination, flush=True)
    # Compact summary for the log.
    for candidate in candidates:
        decomposition = candidate["decomposition"]
        final = decomposition["final"]
        print(json.dumps({
            "capture_index": candidate["capture_index"],
            "rejection": candidate["audit"]["first_failure"],
            "initial_total": decomposition["initial"]["total_budget_m"],
            "minimum_total": decomposition["minimum"]["total_budget_m"],
            "minimum_steps": decomposition["minimum"]["refinement_steps"],
            "final_total": final["total_budget_m"],
            "final_steps": final["refinement_steps"],
            "final_termination": final.get("termination", "CONFIRMED"),
            "final_panels": final["total_subintervals"],
            "final_euclidean": final["euclidean_remainder_m"],
            "final_guard": final["roundoff_and_angle_guard_m"],
            "final_angle": final["angle_bound_rad"],
        }, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()