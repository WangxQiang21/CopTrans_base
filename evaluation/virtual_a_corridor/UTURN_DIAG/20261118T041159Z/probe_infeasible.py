#!/usr/bin/env python3
"""Probe: repeat attempt 0 of the failing plan and decompose its residual.

Runs the *same* IPOPT call the production pipeline makes for the route-guide
initial (same corridor halfspaces, same guide weights, same clearance proxy),
captures the returned decision vector ``z`` and reports which constraint
classes are violated at the solver's best-effort point:

  * u-continuity rows,
  * endpoint equalities (x, y, psi),
  * per-vehicle signed-curvature rows,
  * corridor halfspace rows, labelled by allocation entry, sigma, region and
    portal distance.

Diagnostic only; no production file is edited.
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
    ObstacleContext,
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
    _cached_clearance_field,
    make_route_guides,
)
from fwis_v3_optimizer.formation_continuous_a.route_processing import (
    simplify_route,
)
from fwis_v3_optimizer.formation_continuous_a.ros_adapter import (
    VirtualABackendSettings,
    VirtualAInitialContext,
    request_from_snapshot,
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


def polyline_turn(points):
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


def build(args):
    spec = REPRO.PLANS[args.plan]
    grid, metadata = load_map(args.map_yaml)
    message = REPRO.build_message(args.plan, spec["start"], spec["goal"])
    validation = validate_formation_request(message)
    if not validation.valid:
        raise SystemExit("invalid request")
    settings = VirtualABackendSettings(
        planner_config=PlannerConfig(
            guidance_mode="corridor",
            intervals=args.intervals,
            max_initials=args.max_initials,
            max_seconds=args.max_seconds,
            max_iterations=args.max_iterations,
            solver="ipopt"),
        goal_policy="free", frame_id="map")
    context = VirtualAInitialContext(
        source="rviz_reset",
        context_id="probe-infeasible-{}-1".format(args.plan),
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
    route = routes[0]
    simplified = simplify_route(grid, route)
    chain = build_corridor_chain(
        grid, simplified, box_padding_m=float(config.corridor_box_padding_m))
    audit_corridor_chain(grid, chain)
    guide = make_corridor_guide(
        chain,
        (float(request.start_qf.x), float(request.start_qf.y)),
        (float(request.goal_qf.x), float(request.goal_qf.y)))
    allocation = allocate_corridors(
        chain, guide, intervals=int(config.intervals))
    packet = optimizer._search_packet(request, config)
    turn = polyline_turn(guide.points)
    k = int(round((turn - packet["heading_change_rad"]) / (2.0 * math.pi)))
    return {
        "grid": grid,
        "request": request,
        "config": config,
        "packet": packet,
        "chain": chain,
        "guide": guide,
        "allocation": allocation,
        "grid_sha256": map_request.grid_sha256,
        "guide_turn": turn,
        "branch_multiple": k,
        "branch_target": packet["heading_change_rad"] + 2.0 * math.pi * k,
        "start": spec["start"],
        "goal": spec["goal"],
    }


def integrate(z, case, collocation):
    """Positions/headings/piece-starts of the C2 curve at decision vector z."""
    packet = case["packet"]
    n = int(packet["intervals"])
    S = float(z[0])
    kappas = np.asarray(z[1:1 + n + 1], dtype=float)
    etas = np.asarray(z[1 + n + 1:], dtype=float)
    h = S / n
    nodes, weights = optimizer._gauss(collocation)
    starts = []
    x = y = psi = 0.0
    for j in range(n):
        coeffs = optimizer._coeffs(
            h, kappas[j], kappas[j + 1], etas[j], etas[j + 1])
        starts.append((x, y, psi, coeffs))
        for node, weight in zip(nodes, weights):
            angle = psi + h * node * (
                coeffs[0] + node * (
                    coeffs[1] / 2.0 + node * (
                        coeffs[2] / 3.0 + node * coeffs[3] / 4.0)))
            x += h * weight * math.cos(angle)
            y += h * weight * math.sin(angle)
        psi += h * (coeffs[0] + coeffs[1] / 2.0
                    + coeffs[2] / 3.0 + coeffs[3] / 4.0)
    return {
        "S": S, "kappas": kappas, "etas": etas, "h": h,
        "starts": starts, "x": x, "y": y, "psi": psi,
    }


def sample_position(state, piece, q, quadrature):
    x0, y0, psi0, coeffs = state["starts"][piece]
    shifts, _ = optimizer._piece_position_samples(
        state["h"], coeffs, psi0, [float(q)], math.sin, math.cos,
        quadrature_count=quadrature)
    dx, dy = shifts[0]
    return x0 + dx, y0 + dy


def corridor_labels(case):
    """All corridor samples in allocation-entry order with sigma labels."""
    allocation = case["allocation"]
    request = case["request"]
    config = case["config"]
    n = int(config.intervals)
    quadrature = int(config.corridor_quadrature_points)
    nodes, _ = optimizer._gauss(quadrature)
    start = request.start_qf
    cosine, sine = math.cos(start.psi), math.sin(start.psi)
    from fwis_v3_optimizer.formation_continuous_a.corridor import (
        corridor_effective_margin,
    )
    margin = corridor_effective_margin(case["chain"], config)
    labels = []
    for entry_index, (left, right, corridor_index) in enumerate(allocation):
        sigmas = [left]
        sigmas.extend(left + (right - left) * float(node) for node in nodes)
        sigmas.append(right)
        region = case["chain"].regions[corridor_index]
        local_rows = tuple(
            (float(row[0] * cosine + row[1] * sine),
             float(-row[0] * sine + row[1] * cosine))
            for row in region.A)
        local_limits = tuple(
            float(limit) - margin
            - (float(row[0]) * start.x + float(row[1]) * start.y)
            for row, limit in zip(region.A, region.b))
        for sigma in sigmas:
            sigma = min(1.0, max(0.0, float(sigma)))
            scaled = sigma * n
            piece = int(math.floor(scaled))
            q = scaled - piece
            if piece >= n:
                piece, q = n - 1, 1.0
            labels.append({
                "entry": int(entry_index),
                "region": int(corridor_index),
                "sigma": float(sigma),
                "piece": int(piece),
                "q": float(q),
                "rows": local_rows,
                "limits": local_limits,
            })
    return labels


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--plan", type=int, required=True)
    parser.add_argument("--intervals", type=int, default=8)
    parser.add_argument("--max-initials", type=int, default=6)
    parser.add_argument("--max-seconds", type=float, default=30.0)
    parser.add_argument("--max-iterations", type=int, default=120)
    parser.add_argument("--branch-shift", action="store_true")
    parser.add_argument("--relax-s-upper", type=float, default=None)
    parser.add_argument("--wall-limit", type=float, default=None)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    case = build(args)
    request = case["request"]
    config = case["config"]
    packet = dict(case["packet"])
    if args.branch_shift:
        packet["heading_change_rad"] = case["branch_target"]
    if args.relax_s_upper is not None:
        packet["S_upper"] = max(packet["S_upper"], args.relax_s_upper)
    from dataclasses import replace as _dc_replace

    config = _dc_replace(
        config, max_seconds=float(args.max_seconds),
        max_iterations=int(args.max_iterations))
    cc = optimizer._build_corridor_constraint(
        case["chain"], case["allocation"], request, config)
    guess = optimizer._route_guess(request, config, packet, case["guide"], 0)
    guidance = optimizer._guess_route_guidance(
        guess, request, (case["guide"],), int(packet["intervals"]),
        collocation=int(config.collocation_points), obstacle_context=None)
    node_count = int(packet["intervals"]) + 1
    collocation_count = int(packet["intervals"]) * int(
        config.collocation_points)
    guide_weight = float(config.corridor_guide_weight)
    route_weight = guide_weight * node_count / (
        node_count + collocation_count)
    route_collocation_weight = guide_weight * collocation_count / (
        node_count + collocation_count)
    clearance_field = _cached_clearance_field(case["grid"])
    obstacle_context = ObstacleContext(
        grid_sha256=case["grid_sha256"],
        source="planning_clearance_field",
        clearance_field=clearance_field)
    clearance_proxy = optimizer._clearance_proxy(
        guess, request, packet, config, obstacle_context)
    started = time.monotonic()
    result = optimizer._solve_with_ipopt(
        request, config, packet, guess,
        wall_limit_s=args.wall_limit,
        route_points=guidance[0], route_weight=route_weight,
        route_kappas=guidance[1], route_curvature_weight=0.0,
        route_collocation_points=guidance[2],
        route_collocation_weight=route_collocation_weight,
        clearance_proxy=clearance_proxy,
        corridor_constraint=cc)
    wall = time.monotonic() - started
    z = result["z"]
    state = integrate(z, case, int(config.collocation_points))
    values = optimizer._numeric_constraint_vector(
        z, request, config, packet, int(config.collocation_points),
        corridor_constraint=cc)
    eq_end, signed_end, corridor_end = optimizer._constraint_value_offsets(
        request, config, packet, cc)
    n = int(packet["intervals"])
    continuity = np.asarray(values[:n - 1])
    endpoint = np.asarray(values[n - 1:eq_end])
    signed = np.asarray(values[eq_end:signed_end])
    corridor_values = np.asarray(values[signed_end:corridor_end])
    cmax = packet["cmax"] * (1.0 - float(config.curvature_margin))
    labels = corridor_labels(case)
    per_sample = []
    cursor = 0
    for label in labels:
        x, y = sample_position(
            state, label["piece"], label["q"], cc.quadrature_points)
        for row, limit in zip(label["rows"], label["limits"]):
            residual = row[0] * x + row[1] * y - limit
            per_sample.append({
                "residual_m": float(residual),
                "entry": label["entry"],
                "region": label["region"],
                "sigma": label["sigma"],
                "piece": label["piece"],
                "q": label["q"],
            })
            cursor += 1
    per_sample.sort(key=lambda item: -item["residual_m"])
    document = {
        "plan_id": args.plan,
        "branch_shift": bool(args.branch_shift),
        "branch_multiple": case["branch_multiple"],
        "packet_heading_change_rad": float(packet["heading_change_rad"]),
        "guide_turn_rad": float(case["guide_turn"]),
        "S_m": float(state["S"]),
        "solver_status": result["solver_status"],
        "iterations": result["iterations"],
        "objective": result["objective"],
        "wall_time_s": float(wall),
        "endpoint": {
            "x_error_m": float(state["x"] - packet["dx"]),
            "y_error_m": float(state["y"] - packet["dy"]),
            "psi_error_rad": float(state["psi"] - packet["heading_change_rad"]),
            "distance_error_m": float(math.hypot(
                state["x"] - packet["dx"], state["y"] - packet["dy"])),
        },
        "continuity_max_abs": float(np.max(np.abs(continuity)))
        if continuity.size else 0.0,
        "curvature_max_abs": float(np.max(np.abs(signed)))
        if signed.size else 0.0,
        "curvature_bound": float(cmax),
        "curvature_violation": float(
            np.max(np.abs(signed)) - cmax) if signed.size else 0.0,
        "corridor_max_residual_m": float(np.max(corridor_values))
        if corridor_values.size else 0.0,
        "corridor_worst_samples": per_sample[:12],
        "corridor_positive_fraction": float(
            np.mean(np.asarray(corridor_values) > 0.0))
        if corridor_values.size else 0.0,
        "allocation": [list(entry) for entry in case["allocation"]],
        "start": case["start"],
        "goal": case["goal"],
    }
    Path(args.output).write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n")
    print(json.dumps(document, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()