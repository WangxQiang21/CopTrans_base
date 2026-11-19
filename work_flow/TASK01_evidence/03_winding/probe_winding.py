#!/usr/bin/env python3
"""Step 3 offline probe: 600003/600004 finite winding evidence.

No ROS process starts.  The two frozen requests and the runtime-scaled map are
the Step 0 reproduction inputs; this probe only adds the Step 3 branch fields
(``search["winding"]``, per-round branch metadata and attempt metadata) to the
same production ``solve()`` path.  It is analysis/evidence code, not planner
source.
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
    PlannerConfig)
from fwis_v3_optimizer.formation_continuous_a.ros_adapter import (  # noqa: E402
    VirtualABackendSettings,
    VirtualAInitialContext,
    request_from_snapshot,
    solve,
)
from fwis_v3_optimizer.formation_request import (  # noqa: E402
    validate_formation_request)
from repro_u_turn import PLANS, build_message, jsonable  # noqa: E402


def polyline_length(points):
    return float(sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])))


def guide_summary(guide):
    if not isinstance(guide, dict):
        return None
    points = guide.get("points") or ()
    return {
        "source": guide.get("source"),
        "point_count": len(points),
        "length_m": polyline_length(points),
    }


def run_one(args, plan_id):
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
        context_id="offline-repro-{}-1".format(plan_id),
        qf=FormationPose(*spec["start"]))
    map_request = request_from_snapshot(
        validation.snapshot, grid, settings, context)
    started = time.monotonic()
    result = solve(map_request)
    wall = time.monotonic() - started
    search = dict(getattr(result, "search", {}) or {})
    corridor_rounds = tuple(
        dict(search.get("corridor") or {}).get("rounds", ()) or ())
    rounds = []
    for record in search.get("rounds", ()) or ():
        rounds.append({
            "round_index": record.get("round_index"),
            "route_index": record.get("route_index"),
            "initial_mode": record.get("initial_mode"),
            "geometry_status": record.get("geometry_status"),
            "geometry_reason": record.get("geometry_reason"),
            "initial_count": record.get("initial_count"),
            "new_initial_count": record.get("new_initial_count"),
            "winding_k": record.get("winding_k"),
            "winding_candidate_rank": record.get("winding_candidate_rank"),
            "heading_base_rad": record.get("heading_base_rad"),
            "unwrapped_heading_rad": record.get("unwrapped_heading_rad"),
            "guide_turn_rad": record.get("guide_turn_rad"),
            "guide_length_m": record.get("guide_length_m"),
        })
    attempts = []
    for attempt in getattr(result, "attempts", ()) or ():
        attempts.append({
            "attempt_index": getattr(attempt, "attempt_index", None),
            "initial_source": getattr(attempt, "initial_source", None),
            "initial_label": getattr(attempt, "initial_label", None),
            "solver_status": getattr(attempt, "solver_status", None),
            "status": getattr(attempt, "status", None),
            "iterations": getattr(attempt, "iterations", None),
            "rejection_reason": getattr(attempt, "rejection_reason", None),
            "heading_base_rad": getattr(attempt, "heading_base_rad", None),
            "winding_k": getattr(attempt, "winding_k", None),
            "guide_turn_rad": getattr(attempt, "guide_turn_rad", None),
            "endpoint_error_m": getattr(attempt, "endpoint_error_m", None),
            "heading_error_rad": getattr(attempt, "heading_error_rad", None),
        })
    return {
        "plan_id": plan_id,
        "status": getattr(result, "status", ""),
        "reason": getattr(result, "reason", ""),
        "detail": getattr(result, "detail", ""),
        "first_failure_layer": getattr(result, "first_failure_layer", ""),
        "wall_time_s": float(getattr(result, "wall_time_s", wall)),
        "request_sha256": getattr(result, "request_sha256", ""),
        "routes": [{
            "source": route.source,
            "point_count": len(route.points),
            "length_m": route.length_m,
        } for route in (getattr(result, "routes", ()) or ())],
        "corridor_guide": guide_summary(
            corridor_rounds[-1].get("guide") if corridor_rounds else None),
        "winding": jsonable(search.get("winding")),
        "recoveries": jsonable(search.get("recoveries")),
        "rounds": rounds,
        "attempts": attempts,
        "plan": None if getattr(result, "plan", None) is None else {
            "kind": getattr(result.plan, "kind", ""),
            "duration_s": getattr(result.plan, "duration_s", None),
            "length_m": getattr(getattr(result, "path", None),
                                "length_m", None),
        },
        "reproduction": {
            "map_yaml": str(args.map_yaml),
            "map_metadata": jsonable(metadata),
            "grid_sha256": map_request.grid_sha256,
            "request_sha256": map_request.request_sha256,
            "context_sha256": getattr(map_request, "context_sha256", None),
            "config": jsonable(map_request.planner_config.as_dict()),
            "start": spec["start"],
            "goal": spec["goal"],
            "max_seconds": args.max_seconds,
        },
    }


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
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    documents = [run_one(args, plan_id) for plan_id in args.plan]
    Path(args.output).write_text(
        json.dumps(documents, indent=2, sort_keys=True) + "\n")
    for document in documents:
        print("plan={} status={} reason={} layer={} wall={:.3f}s".format(
            document["plan_id"], document["status"], document["reason"],
            document["first_failure_layer"], document["wall_time_s"]))
        winding = document["winding"] or {}
        selected = winding.get("selected") or {}
        print("  guide={} length={} candidates={} selected_rank={} k={} "
              "base={} turn={}".format(
                  (document["corridor_guide"] or {}).get("source"),
                  document["corridor_guide"] and
                  document["corridor_guide"].get("length_m"),
                  [(item["candidate_rank"], item["winding_k"])
                   for item in winding.get("candidates", ())],
                  winding.get("selected_rank"),
                  selected.get("winding_k"),
                  selected.get("base_heading_rad"),
                  selected.get("guide_turn_rad")))
        for record in document["rounds"]:
            print("  round={} mode={} status={} reason={} k={} rank={} "
                  "initials={}".format(
                      record["round_index"], record["initial_mode"],
                      record["geometry_status"], record["geometry_reason"],
                      record["winding_k"],
                      record["winding_candidate_rank"],
                      record["initial_count"]))
        for recovery in document["recoveries"]:
            print("  recovery={} action={} reason={} k {} -> {}".format(
                recovery.get("recovery_index"), recovery.get("action"),
                recovery.get("reason"), recovery.get("from_winding_k"),
                recovery.get("to_winding_k")))


if __name__ == "__main__":
    main()