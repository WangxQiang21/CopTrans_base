#!/usr/bin/env python3
"""Offline reproduction of the map05 plan 600003 (success) / 600004 (budget).

The live session
``/tmp/expv3-test-ros.ASVoGG`` ran the real planner on a runtime-scaled copy
of ``maps/map05-p12-parking-row-env2`` (formation scale 2.625: 489x326,
resolution 0.064871625 m).  This script rebuilds the *same* map with the same
production tool and feeds the two recorded plan requests through the same
production admission and pipeline functions:

    load_map -> validate_formation_request -> request_from_snapshot -> solve

It does not start ROS and does not trust the live summary: every number in
the emitted JSON is recomputed here.  The only reconstructed inputs are the
two request poses printed by the live planner log.
"""
import argparse
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace

from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a.collision import load_map
from fwis_v3_optimizer.formation_continuous_a.contracts import PlannerConfig
from fwis_v3_optimizer.formation_continuous_a.ros_adapter import (
    VirtualABackendSettings,
    VirtualAInitialContext,
    request_from_snapshot,
    solve,
)
from fwis_v3_optimizer.formation_request import validate_formation_request

FORMATION_ID = "cargo_wedge"
OFFSETS = ((1, 1.0, 0.0), (2, 0.0, 1.0), (3, 0.0, -1.0))
BODY_LENGTH_M = 0.738
BODY_WIDTH_M = 0.500

# Poses printed by the live planner before admission (3 decimals only).
PLANS = {
    600003: {"start": (10.841, 12.071, 0.000),
             "goal": (10.153, 2.076, -3.126),
             "live": {"wall_s": 17.828, "status": "published",
                      "T_end_s": 63.656, "samples": 3184}},
    600004: {"start": (-3.625, 11.771, -0.022),
             "goal": (-2.490, 2.309, 3.102),
             "live": {"wall_s": 40.768, "status": "BUDGET_EXHAUSTED",
                      "reason": "SOFT_BUDGET_EXHAUSTED",
                      "layer": "GEOMETRY"}},
}


def centers(start, goal):
    """FormationModel.centers for the frozen cargo wedge."""
    result = {}
    for vehicle_id, ox, oy in OFFSETS:
        for name, pose in (("start", start), ("goal", goal)):
            c, s = math.cos(pose[2]), math.sin(pose[2])
            result[(vehicle_id, name)] = (pose[0] + c * ox - s * oy,
                                          pose[1] + s * ox + c * oy)
    return result


def build_message(plan_id, start, goal):
    """A minimal object with the FormationPlanRequest field surface."""
    rows = centers(start, goal)
    vehicles = []
    for vehicle_id, ox, oy in OFFSETS:
        sx, sy = rows[(vehicle_id, "start")]
        gx, gy = rows[(vehicle_id, "goal")]
        vehicles.append(SimpleNamespace(
            vehicle_id=vehicle_id,
            offset_in_formation=SimpleNamespace(x=ox, y=oy, z=0.0),
            start_center=SimpleNamespace(x=sx, y=sy, theta=start[2]),
            goal_center=SimpleNamespace(x=gx, y=gy, theta=goal[2]),
            goal_heading_free=True,
            body_length_m=BODY_LENGTH_M,
            body_width_m=BODY_WIDTH_M,
        ))
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id="map",
            stamp=SimpleNamespace(secs=1000, nsecs=0)),
        plan_id=plan_id,
        formation_id=FORMATION_ID,
        formation_start=SimpleNamespace(x=start[0], y=start[1],
                                        theta=start[2]),
        formation_goal=SimpleNamespace(x=goal[0], y=goal[1], theta=goal[2]),
        allow_reverse=True,
        vehicles=vehicles,
    )


def jsonable(value, depth=0):
    """Convert contract objects into JSON without candidate geometry."""
    if depth > 6:
        return "<depth-limit>"
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, dict):
        return {str(k): jsonable(v, depth + 1) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v, depth + 1) for v in value]
    if hasattr(value, "as_dict"):
        try:
            return jsonable(value.as_dict(), depth + 1)
        except Exception as exc:
            return "<as_dict failed: {!r}>".format(exc)
    return repr(value)


def route_summary(route):
    try:
        points = list(route.points)
    except Exception:
        return {"repr": repr(route)}
    return {
        "source": getattr(route, "source", ""),
        "point_count": len(points),
        "length_m": getattr(route, "length_m", None),
        "start": points[0],
        "goal": points[-1],
        "quality": jsonable(getattr(route, "quality", None)),
    }


def summarize(plan_id, result, wall):
    attempts = []
    for attempt in getattr(result, "attempts", ()):
        try:
            payload = attempt.as_dict()
        except Exception as exc:
            payload = {"repr": repr(attempt),
                       "as_dict_error": "{}: {}".format(
                           type(exc).__name__, exc)}
        audit = payload.pop("candidate_audit", None)
        if isinstance(audit, dict):
            payload["candidate_audit"] = {
                "passed": audit.get("passed"),
                "reason": audit.get("reason"),
                "first_failure": audit.get("first_failure"),
                "check_count": audit.get("check_count"),
            }
        attempts.append(payload)
    corridors = dict(getattr(result, "search", {}).get("corridor") or {})
    return {
        "plan_id": plan_id,
        "status": getattr(result, "status", ""),
        "reason": getattr(result, "reason", ""),
        "detail": getattr(result, "detail", ""),
        "first_failure_layer": getattr(result, "first_failure_layer", ""),
        "wall_time_s": float(getattr(result, "wall_time_s", wall)),
        "timings": dict(getattr(result, "timings", {})),
        "search_budget": jsonable(getattr(result, "search", {}).get("budget")),
        "routes": [route_summary(route)
                   for route in getattr(result, "routes", ())],
        "corridor": {
            "guidance_mode": corridors.get("guidance_mode"),
            "enforcement": corridors.get("enforcement"),
            "rounds": jsonable(corridors.get("rounds", ())),
        },
        "attempts": attempts,
        "metrics": jsonable(getattr(result, "metrics", {})),
        "plan": (None if getattr(result, "plan", None) is None else {
            "kind": getattr(result.plan, "kind", ""),
            "duration_s": getattr(result.plan, "duration_s", None),
            "length_m": getattr(getattr(result, "path", None),
                                "length_m", None),
        }),
    }


def run_one(args, plan_id):
    spec = PLANS[plan_id]
    grid, metadata = load_map(args.map_yaml)
    message = build_message(plan_id, spec["start"], spec["goal"])
    validation = validate_formation_request(message)
    if not validation.valid:
        raise SystemExit(
            "plan {} request invalid: {} {} {}".format(
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
        context_id="offline-repro-{}-1".format(plan_id),
        qf=FormationPose(*spec["start"]))
    map_request = request_from_snapshot(
        validation.snapshot, grid, settings, context)
    from fwis_v3_optimizer.formation_continuous_a import optimizer
    packet = optimizer._search_packet(map_request.request,
                                      map_request.planner_config)
    started = time.monotonic()
    result = solve(map_request)
    wall = time.monotonic() - started
    document = summarize(plan_id, result, wall)
    document.update({
        "reproduction": {
            "map_yaml": str(args.map_yaml),
            "map_metadata": jsonable(metadata),
            "grid_summary": map_request.grid_summary,
            "grid_sha256": map_request.grid_sha256,
            "request_sha256": map_request.request_sha256,
            "context_sha256": map_request.context_sha256
            if hasattr(map_request, "context_sha256")
            else None,
            "config": jsonable(map_request.planner_config.as_dict()),
            "s_search": {
                "distance_m": packet["distance"],
                "S_lower_m": packet["S_lower"],
                "S_upper_m": packet["S_upper"],
                "formula": "upper = min(60, max(12, 3*distance + 10))",
            },
            "start": spec["start"],
            "goal": spec["goal"],
            "requested_max_seconds": args.max_seconds,
        },
        "live": spec["live"],
    })
    return document


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--plan", type=int, action="append", required=True)
    parser.add_argument("--guidance-mode", default="corridor")
    parser.add_argument("--intervals", type=int, default=8)
    parser.add_argument("--max-initials", type=int, default=6)
    parser.add_argument("--max-seconds", type=float, default=30.0)
    parser.add_argument("--max-iterations", type=int, default=120)
    parser.add_argument("--solver", default="ipopt")
    parser.add_argument(
        "--relax-s-upper", type=float, default=None,
        help="diagnostic only: raise optimizer._search_packet's S upper bound "
             "to at least this many metres (never used on a normal run)")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.relax_s_upper is not None:
        from fwis_v3_optimizer.formation_continuous_a import optimizer

        original = optimizer._search_packet

        def relaxed(request, config):
            packet = original(request, config)
            if packet["S_upper"] < args.relax_s_upper:
                packet["S_upper"] = float(args.relax_s_upper)
            return packet

        optimizer._search_packet = relaxed
    documents = [run_one(args, plan_id) for plan_id in args.plan]
    Path(args.output).write_text(
        json.dumps(documents, indent=2, sort_keys=True) + "\n")
    for document in documents:
        print("plan={} status={} reason={} layer={} wall={:.3f}s "
              "attempts={} routes={}".format(
                  document["plan_id"], document["status"],
                  document["reason"], document["first_failure_layer"],
                  document["wall_time_s"], len(document["attempts"]),
                  len(document["routes"])))
        for attempt in document["attempts"]:
            print("  attempt={} label={} solver={} solver_status={} "
                  "status={} iter={} wall={:.3f}s detail={}".format(
                      attempt.get("attempt_index"),
                      attempt.get("initial_label"),
                      attempt.get("solver"),
                      attempt.get("solver_status"),
                      attempt.get("status"),
                      attempt.get("iterations"),
                      attempt.get("wall_time_s") or 0.0,
                      str(attempt.get("detail", ""))[:160]))


if __name__ == "__main__":
    main()