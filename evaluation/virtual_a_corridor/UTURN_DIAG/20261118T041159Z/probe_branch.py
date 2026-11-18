#!/usr/bin/env python3
"""Probe: endpoint heading branch versus corridor route homotopy.

The two live plans 600003 (success) and 600004 (budget exhausted) have nearly
the same physical U-turn.  This script rebuilds each plan's corridor chain and
the production route-guide initial value, then measures, *before any solve*:

  * the route guide polyline's unwrapped cumulative heading change,
  * the NLP endpoint target ``_wrap(goal.psi - start.psi)``,
  * the production initial guess's integrated local heading at the endpoint,
  * the initial guess's endpoint equalities, corridor halfspace residuals and
    per-vehicle curvature violations.

``--mode solve-branch`` then re-solves with a diagnostic-only monkeypatch that
shifts ``heading_change_rad`` by whole multiples of 2*pi to the branch nearest
the route guide's turning (physically identical endpoint orientation).  The
audits still wrap the heading error, so a solution must pass unchanged.
``--mode solve-default`` is the untouched control.

This is a diagnostic probe: it edits no production file and is never imported
by the planner.
"""
import argparse
import importlib.util
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a import optimizer
from fwis_v3_optimizer.formation_continuous_a.collision import load_map
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    PlannerConfig,
    RouteGuideConfig,
    _wrap,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (
    allocate_corridors,
    audit_corridor_chain,
    build_corridor_chain,
    make_corridor_guide,
)
from fwis_v3_optimizer.formation_continuous_a.pipeline import (
    G4_ROUTE_CLEARANCE_WEIGHT,
    make_route_guides,
)
from fwis_v3_optimizer.formation_continuous_a.route_processing import (
    simplify_route,
)
from fwis_v3_optimizer.formation_continuous_a.ros_adapter import (
    VirtualABackendSettings,
    VirtualAInitialContext,
    request_from_snapshot,
    solve,
)
from fwis_v3_optimizer.formation_request import validate_formation_request

HERE = os.path.dirname(os.path.abspath(__file__))


def _load_repro():
    spec = importlib.util.spec_from_file_location(
        "repro_u_turn", os.path.join(HERE, "repro_u_turn.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REPRO = _load_repro()


def jsonable(value):
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, (bool, int, float, str)) or value is None:
        return value
    return repr(value)


def polyline_turn(points):
    """Unwrapped cumulative heading change of a polyline."""
    total = 0.0
    previous = None
    for index in range(1, len(points)):
        heading = math.atan2(
            float(points[index][1]) - float(points[index - 1][1]),
            float(points[index][0]) - float(points[index - 1][0]))
        if previous is not None:
            total += _wrap(heading - previous)
        previous = heading
    return total


def build_case(args, plan_id):
    spec = REPRO.PLANS[plan_id]
    grid, metadata = load_map(args.map_yaml)
    message = REPRO.build_message(plan_id, spec["start"], spec["goal"])
    validation = validate_formation_request(message)
    if not validation.valid:
        raise SystemExit("plan {} request invalid: {} {}".format(
            plan_id, validation.reason, validation.detail))
    settings = VirtualABackendSettings(
        planner_config=PlannerConfig(
            guidance_mode="corridor",
            intervals=args.intervals,
            max_initials=args.max_initials,
            max_seconds=args.max_seconds,
            max_iterations=args.max_iterations,
            solver=args.solver),
        goal_policy="free",
        frame_id="map")
    context = VirtualAInitialContext(
        source="rviz_reset",
        context_id="probe-branch-{}-1".format(plan_id),
        qf=FormationPose(*spec["start"]))
    map_request = request_from_snapshot(
        validation.snapshot, grid, settings, context)
    request = map_request.request
    config = map_request.planner_config
    route_config = RouteGuideConfig(
        clearance_weight=G4_ROUTE_CLEARANCE_WEIGHT, max_seconds=8.0)
    routes = make_route_guides(
        grid, request.start_qf, request.goal_qf, route_config,
        blocked_points=())
    if not routes:
        raise SystemExit("plan {} produced no routes".format(plan_id))
    route = routes[0]
    simplified = simplify_route(grid, route)
    chain = build_corridor_chain(
        grid, simplified,
        box_padding_m=float(config.corridor_box_padding_m))
    chain_audit = audit_corridor_chain(grid, chain)
    guide = make_corridor_guide(
        chain,
        (float(request.start_qf.x), float(request.start_qf.y)),
        (float(request.goal_qf.x), float(request.goal_qf.y)))
    allocation = allocate_corridors(
        chain, guide, intervals=int(config.intervals))
    packet = optimizer._search_packet(request, config)
    return {
        "plan_id": plan_id,
        "grid": grid,
        "metadata": metadata,
        "request": request,
        "config": config,
        "map_request": map_request,
        "routes": routes,
        "route": route,
        "simplified": simplified,
        "chain": chain,
        "chain_audit": chain_audit,
        "guide": guide,
        "allocation": allocation,
        "packet": packet,
    }


def guide_branch_turn(case):
    turn = polyline_turn(case["guide"].points)
    packet = case["packet"]
    target = packet["heading_change_rad"]
    k = int(round((turn - target) / (2.0 * math.pi)))
    return turn, target, k, target + 2.0 * math.pi * k


def evaluate_initial(case):
    """Evaluate the production route-guide initial against the hard constraints."""
    request = case["request"]
    config = case["config"]
    packet = case["packet"]
    guide = case["guide"]
    n = int(packet["intervals"])
    guess = optimizer._route_guess(request, config, packet, guide, 0)
    S = float(guess.S)
    kappas = np.asarray(guess.kappas, dtype=float)
    etas = np.asarray(guess.etas, dtype=float)
    h = S / n
    corridor = optimizer._build_corridor_constraint(
        case["chain"], case["allocation"], request, config)
    nodes, weights = optimizer._gauss(int(config.collocation_points))
    x = 0.0
    y = 0.0
    psi = 0.0
    corridor_rows = []
    curvature_rows = []
    cmax = packet["cmax"] * (1.0 - float(config.curvature_margin))
    offsets = tuple((float(o.x), float(o.y))
                    for o in request.model.offsets)
    for j in range(n):
        c0, c1, c2, c3 = optimizer._coeffs(
            h, kappas[j], kappas[j + 1], etas[j], etas[j + 1])
        for q, rows, limits in corridor.groups[j]:
            shifts, _headings = optimizer._piece_position_samples(
                h, (c0, c1, c2, c3), psi, [float(q)], np.sin, np.cos,
                quadrature_count=corridor.quadrature_points)
            dx, dy = shifts[0]
            sample_x, sample_y = x + dx, y + dy
            for row, limit in zip(rows, limits):
                residual = row[0] * sample_x + row[1] * sample_y - limit
                corridor_rows.append({
                    "residual_m": float(residual),
                    "piece": int(j),
                    "q": float(q),
                    "sigma": float((j + q) / n),
                })
        for q in list(nodes) + [0.0, 1.0]:
            kappa = c0 + q * (c1 + q * (c2 + q * c3))
            eta = (c1 + q * (2.0 * c2 + 3.0 * q * c3)) / h
            for a, b in offsets:
                A = 1.0 - b * kappa
                B = a * kappa
                D = A * A + B * B
                c = (kappa * D + a * eta) / (D ** 1.5)
                curvature_rows.append({
                    "violation": float(abs(c) - cmax),
                    "piece": int(j),
                    "q": float(q),
                    "vehicle_curvature": float(c),
                })
        for node, weight in zip(nodes, weights):
            angle = psi + h * node * (
                c0 + node * (c1 / 2.0 + node * (c2 / 3.0 + node * c3 / 4.0)))
            x += h * weight * math.cos(angle)
            y += h * weight * math.sin(angle)
        psi += h * (c0 + c1 / 2.0 + c2 / 3.0 + c3 / 4.0)
    worst_corridor = max(corridor_rows, key=lambda row: row["residual_m"])
    worst_curvature = max(curvature_rows, key=lambda row: row["violation"])
    return {
        "initial_label": guess.label,
        "initial_source": guess.source,
        "S_m": S,
        "S_upper_m": float(packet["S_upper"]),
        "S_lower_m": float(packet["S_lower"]),
        "route_length_m": float(guess.metadata.get("route_length_m", 0.0)),
        "initial_kappa_scale": float(guess.metadata.get("length_multiplier", 0.0)),
        "psi_end_local_rad": float(psi),
        "endpoint_target_rad": float(packet["heading_change_rad"]),
        "psi_end_minus_target_rad": float(psi - packet["heading_change_rad"]),
        "endpoint_x_error_m": float(x - packet["dx"]),
        "endpoint_y_error_m": float(y - packet["dy"]),
        "corridor_sample_count": len(corridor_rows),
        "corridor_max_residual_m": float(worst_corridor["residual_m"]),
        "corridor_worst": worst_corridor,
        "corridor_positive_fraction": float(
            sum(1 for row in corridor_rows if row["residual_m"] > 0.0)
            / max(1, len(corridor_rows))),
        "curvature_max_violation": float(worst_curvature["violation"]),
        "curvature_worst": worst_curvature,
        "kappa_limit_rad_per_m": float(packet["kappa_limit"]),
        "vehicle_cmax_rad_per_m": float(cmax),
        "node_positions": [None],
    }


def summarize_result(case, result, wall):
    attempts = []
    for attempt in getattr(result, "attempts", ()):
        payload = attempt.as_dict()
        payload.pop("candidate_audit", None)
        attempts.append(payload)
    return {
        "plan_id": case["plan_id"],
        "status": getattr(result, "status", ""),
        "reason": getattr(result, "reason", ""),
        "detail": getattr(result, "detail", ""),
        "first_failure_layer": getattr(result, "first_failure_layer", ""),
        "wall_time_s": float(wall),
        "attempts": jsonable(attempts),
        "plan": None if getattr(result, "plan", None) is None else {
            "duration_s": getattr(result.plan, "duration_s", None),
        },
    }


def shifted_packet_wrapper(case, relaxed_upper, branch_override=None):
    original = optimizer._search_packet
    turn, target, k, shifted = guide_branch_turn(case)
    if branch_override is not None:
        k = int(branch_override)
        shifted = target + 2.0 * math.pi * k

    def patched(request, config):
        packet = original(request, config)
        packet["heading_change_rad"] = shifted
        if relaxed_upper is not None and packet["S_upper"] < relaxed_upper:
            packet["S_upper"] = float(relaxed_upper)
        if packet["S_lower"] >= packet["S_upper"]:
            packet["S_upper"] = max(
                packet["S_upper"], packet["S_lower"] + 1.0)
        return packet

    return patched, {
        "guide_turn_rad": float(turn),
        "original_target_rad": float(target),
        "branch_multiple": int(k),
        "shifted_target_rad": float(shifted),
        "relaxed_s_upper_m": None if relaxed_upper is None else float(
            relaxed_upper),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--plan", type=int, required=True)
    parser.add_argument(
        "--mode", choices=("evaluate", "solve-default", "solve-branch"),
        required=True)
    parser.add_argument("--intervals", type=int, default=8)
    parser.add_argument("--max-initials", type=int, default=6)
    parser.add_argument("--max-seconds", type=float, default=30.0)
    parser.add_argument("--max-iterations", type=int, default=120)
    parser.add_argument("--solver", default="ipopt")
    parser.add_argument("--relax-s-upper", type=float, default=None)
    parser.add_argument(
        "--branch-multiple", type=int, default=None,
        help="override the diagnostic branch shift (multiples of 2*pi)")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    case = build_case(args, args.plan)
    turn, target, k, shifted = guide_branch_turn(case)
    document = {
        "plan_id": args.plan,
        "mode": args.mode,
        "guide_length_m": float(case["guide"].length_m),
        "route_length_m": float(case["route"].length_m),
        "simplified_length_m": float(case["simplified"].length_m),
        "seed_lengths_m": list(case["guide"].quality.get(
            "seed_lengths_m", ())),
        "guide_turn_rad": float(turn),
        "guide_turn_deg": float(math.degrees(turn)),
        "endpoint_target_rad": float(target),
        "endpoint_target_deg": float(math.degrees(target)),
        "branch_multiple": int(k),
        "branch_shifted_target_rad": float(shifted),
        "branch_shifted_target_deg": float(math.degrees(shifted)),
        "packet": {
            "distance_m": float(case["packet"]["distance"]),
            "dx_m": float(case["packet"]["dx"]),
            "dy_m": float(case["packet"]["dy"]),
            "S_lower_m": float(case["packet"]["S_lower"]),
            "S_upper_m": float(case["packet"]["S_upper"]),
            "heading_change_rad": float(
                case["packet"]["heading_change_rad"]),
        },
        "chain_audit_passed": bool(case["chain_audit"]["passed"]),
        "portal_radius_m": list(case["guide"].quality.get(
            "portal_radius_m", ())),
        "segment_lengths_m": list(case["guide"].quality.get(
            "segment_lengths_m", ())),
        "allocation": [list(entry) for entry in case["allocation"]],
        "start": list(REPRO.PLANS[args.plan]["start"]),
        "goal": list(REPRO.PLANS[args.plan]["goal"]),
    }
    if args.mode == "evaluate":
        document["initial"] = evaluate_initial(case)
    else:
        if args.mode == "solve-branch":
            patched, branch_info = shifted_packet_wrapper(
                case, args.relax_s_upper,
                branch_override=args.branch_multiple)
            optimizer._search_packet = patched
            document["applied_branch"] = branch_info
        elif args.relax_s_upper is not None:
            original = optimizer._search_packet

            def relaxed(request, config):
                packet = original(request, config)
                if packet["S_upper"] < args.relax_s_upper:
                    packet["S_upper"] = float(args.relax_s_upper)
                return packet

            optimizer._search_packet = relaxed
            document["applied_relaxed_s_upper_m"] = float(args.relax_s_upper)
        started = time.monotonic()
        result = solve(case["map_request"])
        wall = time.monotonic() - started
        document["result"] = summarize_result(case, result, wall)
    Path(args.output).write_text(
        json.dumps(jsonable(document), indent=2, sort_keys=True) + "\n")
    print(json.dumps(jsonable(document), indent=2, sort_keys=True))
    print("PROBE plan={} mode={} status={}".format(
        args.plan, args.mode,
        document.get("result", {}).get("status", "evaluated")))


if __name__ == "__main__":
    main()