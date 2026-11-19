#!/usr/bin/env python3
"""Step 4 offline probe: finite guide-related S domain.

No ROS process starts.  The 600003/600004 requests and the runtime-scaled map
are the Step 0 reproduction inputs; this probe adds the Step 4 S-domain fields
that the production solver records in its attempts and search rounds.  It also
solves two synthetic straight-line constructions (50 m and 100 m) with the
same frozen solver policy to show that the historical 60 m cap is gone.

It is analysis/evidence code, not planner source.  Every expected bound is
recomputed here from the request/config; nothing is trusted from the result.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
UTURN = ROOT / "evaluation/virtual_a_corridor/UTURN_DIAG/20261118T041159Z"
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
sys.path.insert(0, str(UTURN))

from fwis_v3_optimizer.formation import FormationPose  # noqa: E402
from fwis_v3_optimizer.formation_continuous_a.collision import (  # noqa: E402
    load_map)
from fwis_v3_optimizer.formation_continuous_a.contracts import (  # noqa: E402
    PlannerConfig,
    request_fingerprint)
from fwis_v3_optimizer.formation_continuous_a.optimizer import (  # noqa: E402
    GUIDE_S_UPPER_FACTOR,
    make_initial_guesses,
    solve_open_path_guided,
)
from fwis_v3_optimizer.formation_continuous_a.ros_adapter import (  # noqa: E402
    VirtualABackendSettings,
    VirtualAInitialContext,
    request_from_snapshot,
    solve,
)
from fwis_v3_optimizer.formation_request import (  # noqa: E402
    validate_formation_request)
from repro_u_turn import PLANS, build_message, jsonable  # noqa: E402

REAL_PLAN_START = PLANS[600003]["start"]


def polyline_length(points):
    return float(sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])))


def expected_domain(request, config, heading_rad, guide_length_m=None):
    kappa_ref_max = min(
        float(config.kappa_limit),
        1.0 / float(request.limits.r_min_ackermann))
    lower = max(
        int(config.intervals) * 0.05, request.planning_distance_m,
        abs(heading_rad) / kappa_ref_max)
    upper = max(lower + 2.0, 12.0, 3.0 * request.planning_distance_m + 10.0)
    if guide_length_m is not None:
        upper = max(upper, GUIDE_S_UPPER_FACTOR * guide_length_m)
    return {
        "kappa_ref_max": kappa_ref_max,
        "distance_m": request.planning_distance_m,
        "guide_length_m": guide_length_m,
        "S_lower_m": lower,
        "S_upper_m": upper,
        "old_cap_m": 60.0,
        "exceeds_old_cap": upper > 60.0,
    }


def attempt_summary(attempt):
    return {
        "attempt_index": getattr(attempt, "attempt_index", None),
        "initial_source": getattr(attempt, "initial_source", None),
        "initial_label": getattr(attempt, "initial_label", None),
        "solver_status": getattr(attempt, "solver_status", None),
        "status": getattr(attempt, "status", None),
        "iterations": getattr(attempt, "iterations", None),
        "rejection_reason": getattr(attempt, "rejection_reason", None),
        "detail": getattr(attempt, "detail", None),
        "distance_m": getattr(attempt, "distance_m", None),
        "guide_length_m": getattr(attempt, "guide_length_m", None),
        "S_lower_m": getattr(attempt, "s_lower_m", None),
        "S_upper_m": getattr(attempt, "s_upper_m", None),
        "S_solution_m": getattr(attempt, "s_solution_m", None),
        "upper_active": getattr(attempt, "s_upper_active", None),
    }


def round_summary(record):
    return {
        "round_index": record.get("round_index"),
        "route_index": record.get("route_index"),
        "initial_mode": record.get("initial_mode"),
        "geometry_status": record.get("geometry_status"),
        "geometry_reason": record.get("geometry_reason"),
        "initial_count": record.get("initial_count"),
        "winding_k": record.get("winding_k"),
        "distance_m": record.get("distance_m"),
        "guide_length_m": record.get("guide_length_m"),
        "S_lower_m": record.get("S_lower_m"),
        "S_upper_m": record.get("S_upper_m"),
        "S_solution_m": record.get("S_solution_m"),
        "upper_active": record.get("upper_active"),
    }


def run_real(args, plan_id):
    spec = PLANS[plan_id]
    grid, metadata = load_map(args.map_yaml)
    message = build_message(plan_id, spec["start"], spec["goal"])
    validation = validate_formation_request(message)
    if not validation.valid:
        raise SystemExit("plan {} request invalid: {} {} {}".format(
            plan_id, validation.reason, validation.detail,
            validation.field))
    settings = VirtualABackendSettings(
        planner_config=PlannerConfig(
            guidance_mode=args.guidance_mode,
            intervals=args.intervals,
            max_initials=args.max_initials,
            max_seconds=args.max_seconds,
            max_iterations=args.max_iterations,
            solver=args.solver),
        goal_policy="free",
        frame_id="map")
    context = VirtualAInitialContext(
        source="rviz_reset",
        context_id="offline-s-domain-{}-1".format(plan_id),
        qf=FormationPose(*spec["start"]))
    map_request = request_from_snapshot(
        validation.snapshot, grid, settings, context)
    started = time.monotonic()
    result = solve(map_request)
    wall = time.monotonic() - started
    search = dict(getattr(result, "search", {}) or {})
    corridor_rounds = tuple(
        dict(search.get("corridor") or {}).get("rounds", ()) or ())
    guide = corridor_rounds[-1].get("guide") if corridor_rounds else None
    guide_length = polyline_length(guide.get("points") or ()) \
        if isinstance(guide, dict) else None
    request = map_request.request
    heading = getattr(result, "goal_heading_branch_rad", None)
    if heading is None:
        selected = dict(search.get("winding") or {}).get("selected") or {}
        heading = selected.get("unwrapped_heading_rad")
    if heading is None:
        delta = request.goal_qf.psi - request.start_qf.psi
        heading = math.atan2(math.sin(delta), math.cos(delta))
    expected = expected_domain(
        request, settings.planner_config, heading, guide_length)
    attempts = [attempt_summary(item)
                for item in (getattr(result, "attempts", ()) or ())]
    rounds = [round_summary(record)
              for record in (search.get("rounds", ()) or ())]
    path = getattr(result, "path", None)
    domain_checks = {
        "attempt_lower_matches": all(
            item["S_lower_m"] is None
            or abs(item["S_lower_m"] - expected["S_lower_m"]) <= 1e-9
            for item in attempts),
        "attempt_upper_matches": all(
            item["S_upper_m"] is None
            or abs(item["S_upper_m"] - expected["S_upper_m"]) <= 1e-9
            for item in attempts),
        "round_lower_matches": all(
            item["S_lower_m"] is None
            or abs(item["S_lower_m"] - expected["S_lower_m"]) <= 1e-9
            for item in rounds),
        "round_upper_matches": all(
            item["S_upper_m"] is None
            or abs(item["S_upper_m"] - expected["S_upper_m"]) <= 1e-9
            for item in rounds),
        "path_length_equals_solution": bool(
            path is not None and any(
                item["S_solution_m"] is not None
                and abs(item["S_solution_m"]
                        - float(getattr(path, "length_m", -1.0))) <= 1e-9
                for item in attempts)),
    }
    return {
        "kind": "real",
        "plan_id": plan_id,
        "status": getattr(result, "status", ""),
        "reason": getattr(result, "reason", ""),
        "detail": getattr(result, "detail", ""),
        "first_failure_layer": getattr(result, "first_failure_layer", ""),
        "wall_time_s": float(getattr(result, "wall_time_s", wall)),
        "request_sha256": getattr(result, "request_sha256", ""),
        "unwrapped_heading_rad": heading,
        "guide_length_m": guide_length,
        "expected": expected,
        "domain_checks": domain_checks,
        "rounds": rounds,
        "attempts": attempts,
        "plan": None if getattr(result, "plan", None) is None else {
            "kind": getattr(result.plan, "kind", ""),
            "duration_s": getattr(result.plan, "duration_s", None),
            "length_m": getattr(path, "length_m", None),
        },
        "reproduction": {
            "map_yaml": str(args.map_yaml),
            "map_metadata": jsonable(metadata),
            "grid_sha256": map_request.grid_sha256,
            "request_sha256": map_request.request_sha256,
            "planner_config": jsonable(
                map_request.planner_config.as_dict()),
            "start": spec["start"],
            "goal": spec["goal"],
            "max_seconds": args.max_seconds,
        },
    }


def run_synthetic(args, length_m, plan_id):
    """A straight +x construction from the 600003 start, solver only.

    The map is not consulted here: this probe verifies the finite S domain and
    initial-value range, not collision feasibility.  The same frozen initial
    count and soft wall apply; no guide is supplied, so the no-guide upper
    bound applies.
    """
    start = REAL_PLAN_START
    goal = (start[0] + float(length_m), start[1], 0.0)
    grid, metadata = load_map(args.map_yaml)
    message = build_message(plan_id, start, goal)
    validation = validate_formation_request(message)
    if not validation.valid:
        raise SystemExit("synthetic {} invalid: {} {} {}".format(
            length_m, validation.reason, validation.detail,
            validation.field))
    config = PlannerConfig(
        guidance_mode="legacy_route",
        intervals=args.intervals,
        max_initials=args.max_initials,
        max_seconds=args.max_seconds,
        max_iterations=args.max_iterations,
        solver=args.solver)
    settings = VirtualABackendSettings(
        planner_config=config, goal_policy="free", frame_id="map")
    context = VirtualAInitialContext(
        source="rviz_reset",
        context_id="offline-s-domain-synthetic-{}-1".format(plan_id),
        qf=FormationPose(*start))
    map_request = request_from_snapshot(
        validation.snapshot, grid, settings, context)
    request = map_request.request
    expected = expected_domain(request, config, 0.0, None)
    started = time.monotonic()
    result = solve_open_path_guided(request, config)
    wall = time.monotonic() - started
    attempts = [attempt_summary(item)
                for item in (getattr(result, "attempts", ()) or ())]
    initial_s = [float(guess.S) for guess in make_initial_guesses(
        request, config)]
    path = getattr(result, "path", None)
    return {
        "kind": "synthetic_straight",
        "plan_id": plan_id,
        "length_m": float(length_m),
        "status": getattr(result, "status", ""),
        "reason": getattr(result, "reason", ""),
        "detail": getattr(result, "detail", ""),
        "wall_time_s": wall,
        "request_sha256": request_fingerprint(request),
        "expected": expected,
        "initial_S_values_m": initial_s,
        "initial_above_old_cap": max(initial_s) > 60.0,
        "attempts": attempts,
        "plan": None if path is None else {
            "length_m": getattr(path, "length_m", None)},
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--guidance-mode", default="corridor")
    parser.add_argument("--intervals", type=int, default=8)
    parser.add_argument("--max-initials", type=int, default=6)
    parser.add_argument("--max-seconds", type=float, default=30.0)
    parser.add_argument("--max-iterations", type=int, default=120)
    parser.add_argument("--solver", default="ipopt")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    documents = [run_real(args, plan_id) for plan_id in (600003, 600004)]
    documents.append(run_synthetic(args, 50.0, 900001))
    documents.append(run_synthetic(args, 100.0, 900002))
    Path(args.output).write_text(
        json.dumps(documents, indent=2, sort_keys=True) + "\n")
    for document in documents:
        label = document.get("plan_id")
        print("plan={} kind={} status={} reason={} wall={:.3f}s".format(
            label, document["kind"], document["status"],
            document["reason"], document["wall_time_s"]))
        expected = document["expected"]
        print("  distance={:.6f} guide={} S_lower={:.6f} S_upper={:.6f} "
              "old_cap={} exceeds={}".format(
                  expected["distance_m"], expected["guide_length_m"],
                  expected["S_lower_m"], expected["S_upper_m"],
                  expected["old_cap_m"], expected["exceeds_old_cap"]))
        for attempt in document["attempts"]:
            print("  attempt={} k={} status={} S={} upper_active={} "
                  "lower={} upper={}".format(
                      attempt["attempt_index"], attempt["initial_label"],
                      attempt["status"], attempt["S_solution_m"],
                      attempt["upper_active"], attempt["S_lower_m"],
                      attempt["S_upper_m"]))
        if document["kind"] == "real":
            print("  domain_checks={}".format(document["domain_checks"]))
            for record in document["rounds"]:
                print("  round={} status={} k={} S_solution={} "
                      "upper_active={}".format(
                          record["round_index"],
                          record["geometry_status"],
                          record["winding_k"], record["S_solution_m"],
                          record["upper_active"]))
        else:
            print("  initial_S={} max_above_old_cap={}".format(
                ["{:.3f}".format(value)
                 for value in document["initial_S_values_m"]],
                document["initial_above_old_cap"]))


if __name__ == "__main__":
    main()