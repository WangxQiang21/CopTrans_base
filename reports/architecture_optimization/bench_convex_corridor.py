#!/usr/bin/env python3
"""Paired offline comparison of the optional convex formation corridor."""

import argparse
from dataclasses import asdict, replace
from hashlib import sha256
import json
import math
from pathlib import Path
import os
import runpy
from time import perf_counter, process_time

import numpy as np
import yaml

from fwis_backends.multi_4wis import formation_collision, uniform_search
from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_backends.multi_4wis.formation_collision import CargoFootprint, VehicleShape
from fwis_backends.multi_4wis.formation_retiming import FormationTimingLimits
from fwis_backends.multi_4wis.quality import FootprintDistance
from fwis_backends.multi_4wis.uniform_pipeline import UniformPlanningRequest, plan_uniform


def additional_requests():
    shape = (150, 160)
    resolution = 0.1
    ids = (1, 2, 3)
    team = FormationModel([VehicleOffset(1, 1.0, 0.0), VehicleOffset(2, 0.0, 1.0),
                           VehicleOffset(3, 0.0, -1.0)])
    shapes = dict.fromkeys(ids, VehicleShape(0.738, 0.5))
    limits = FormationTimingLimits(v_max=0.16, a_max=0.22, yaw_rate_max=0.4,
                                   yaw_accel_max=0.45, r_min_ackermann=.90, mode_switch_duration=.70)
    search = uniform_search.GraphSearchConfig(
        max_seconds=5.0, max_expansions=40, direct_goal_translation=True)
    successor = uniform_search.SuccessorConfig(timing_limits=limits)
    options = dict(search_config=search, successor_config=successor,
                   cargo_footprint=CargoFootprint(1.95, 2.70, .5, 0.0))

    portal = np.zeros(shape, dtype=bool)
    portal[60:65, :] = True
    portal[60:65, 60:98] = False
    start = FormationPose(3.0, 3.0, 0.0)
    goal = FormationPose(12.0, 10.0, 0.0)
    yield "portal_detour", UniformPlanningRequest(
        Grid(portal, resolution, 0.0, 0.0), team, shapes, start, dict.fromkeys(ids, 0.0), goal,
        **options)

    staggered = portal.copy()
    staggered[95:100, :] = True
    staggered[95:100, 108:145] = False
    yield "staggered_portals", UniformPlanningRequest(
        Grid(staggered, resolution, 0.0, 0.0), team, shapes, start, dict.fromkeys(ids, 0.0),
        FormationPose(6.0, 12.5, 0.0), **options)

    closed = np.zeros(shape, dtype=bool)
    closed[60:65, :] = True
    yield "closed_wall", UniformPlanningRequest(
        Grid(closed, resolution, 0.0, 0.0), team, shapes, start, dict.fromkeys(ids, 0.0), goal,
        **options)


def observed_search(request, convex):
    planner = replace(request, search_config=replace(
        request.search_config, convex_corridor=convex, native_frontier=True))
    original = uniform_search.convex_corridor
    probes = []

    def observed(*args, **kwargs):
        result = original(*args, **kwargs)
        probes.append((result.status, result.expansions, len(result.points)))
        return result

    uniform_search.convex_corridor = observed
    wall_start, cpu_start = perf_counter(), process_time()
    try:
        outcome = plan_uniform(planner)
    finally:
        wall_s, cpu_s = perf_counter() - wall_start, process_time() - cpu_start
        uniform_search.convex_corridor = original

    search, plan, audit = outcome.search, outcome.plan, outcome.evaluation
    poses = outcome.projection.timed_trajectory.formation_poses if outcome.projection else ()
    stages = outcome.stages or {}
    guide = None if search is None else getattr(search, "corridor_guide", None)
    record = {
        "wall_s": wall_s,
        "cpu_s": cpu_s,
        "first_certified_s": stages.get("first_certified_s"),
        "final_audit_s": stages.get("final_audit_s"),
        "search_wall_s": search.wall_time_s if search else None,
        "terminal_audits": len(search.terminal_audits) if search else None,
        "pipeline_fallback_reason": stages.get("convex_corridor_fallback"),
        "status": outcome.status,
        "reason": outcome.reason,
        "executable": outcome.executable,
        "used_connector": search.direct_goal_translation if search else None,
        "frontier_backend": search.frontier_backend if search else None,
        "graph_expansions": search.expansions if search else None,
        "corridors_probed": [
            {"status": status, "expansions": visited, "waypoint_count": points}
            for status, visited, points in probes
        ],
        "constructed_guide": ({name: guide.get(name) for name in (
            "status", "expansions", "waypoints", "length_m", "uses_shortcuts")} if guide else None),
        "plan_sha256": plan.model_sha256() if plan is not None else None,
        "duration_s": plan.duration_s if plan else None,
        "cargo_distance_m": sum(math.hypot(b.x - a.x, b.y - a.y)
                                for a, b in zip(poses, poses[1:])) if poses else None,
        "mode_switch_count": sum(a.mode != b.mode for a, b in zip(plan.edges, plan.edges[1:]))
        if plan else None,
        "primitive_sequence": [edge.primitive for edge in plan.edges] if plan else [],
        "mode_sequence": [edge.mode for edge in plan.edges] if plan else [],
        "independent_audit": audit.independent_audit if audit else None,
        "collision_certificate": audit.collision_certificate if audit else None,
        "audit_reason": audit.reason if audit else None,
        "audit_check_count": audit.checks if audit else None,
        "audit_cargo_footprint": audit.metrics.get("cargo_footprint") if audit else None,
    }
    if outcome.executable:
        assert audit is not None and audit.feasible and audit.collision_certificate
        assert audit.metrics.get("cargo_footprint") == asdict(request.cargo_footprint)
        assert all(mode in ("A", "O", "S") for mode in record["mode_sequence"])
    return outcome, record


def sampled_clearance(request, plan):
    """Physical cargo-to-occupied-cell clearance on a fixed sample grid."""
    cargo = request.cargo_footprint
    distance = FootprintDistance(request.grid, cargo.length_m, cargo.width_m)
    times = {float(x) for x in np.linspace(0.0, plan.duration_s, 241)}
    for phase in plan.phases:
        times.update((phase.t0_s, phase.t0_s + phase.duration_s))
    times = sorted(t for t in times if 0.0 <= t <= plan.duration_s)
    distances = (
        (distance.distance(cargo.pose_at(plan.state_at(t).qf)), t)
        for t in times
    )
    clearance, witness = min(distances)
    return {"minimum_m": clearance, "time_s": witness, "samples": len(times)}


def summary(rows):
    result = {}
    for key in ("wall_s", "cpu_s", "first_certified_s", "search_wall_s"):
        values = [row[key] for row in rows if row[key] is not None]
        if values:
            result[key] = {"count": len(values), "p50": float(np.median(values)), "max": max(values)}
            if len(values) >= 30:
                result[key]["p95"] = float(np.percentile(values, 95))
            if len(values) >= 100:
                result[key]["p99"] = float(np.percentile(values, 99))
    return result


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--output", type=Path, required=True)
    cli.add_argument("--pairs", type=int, default=1)
    cli.add_argument("--warmups", type=int, default=0)
    cli.add_argument("--case", action="append", help="run only named cases")
    args = cli.parse_args()
    if args.pairs < 1 or args.warmups < 0:
        cli.error("need positive pair count and nonnegative warm-ups")
    assert formation_collision._formation_collision_native is not None

    cases = []
    make_request = runpy.run_path(
        "ros2_ws/src/fwis_multi_planner_cpp/benchmark/compare_scene_matrix.py")["make_request"]
    offsets = yaml.safe_load(Path("ros2_ws/src/fwis_bringup/config/experiments.yaml").read_text())
    root = Path("ros2_ws/generated/scenes")
    for name in ("factory_indoor_01", "port_01", "factory_outdoor_01",
                 "factory_indoor_01_wide", "factory_indoor_01_critical", "factory_indoor_01_too_narrow"):
        cases.append((name, make_request(root / name, "full_route", offsets)))
    cases.extend(additional_requests())

    out = {
        "variants": {"legacy": {"convex_corridor": False}, "corridor": {"convex_corridor": True}},
        "common": {"native_frontier": True, "continuous_a_refinement": False,
                   "cargo_clearance": "sampled physical OBB on fixed 241-time grid and phase boundaries"},
        "pairs_per_case": args.pairs,
        "warmups_per_case": args.warmups,
        "cpu_affinity": sorted(os.sched_getaffinity(0)),
        "source": uniform_search.__file__,
        "native_sat": formation_collision._formation_collision_native.__file__,
        "cases": [],
    }
    selected = [(name, request) for name, request in cases if not args.case or name in args.case]
    if args.case and set(name for name, _ in selected) != set(args.case):
        cli.error("unknown case")
    for name, request in selected:
        samples = {"legacy": [], "corridor": []}
        final = {}
        for trial in range(args.warmups + args.pairs):
            for enabled in ((False, True) if trial % 2 == 0 else (True, False)):
                key = "corridor" if enabled else "legacy"
                outcome, timing = observed_search(request, enabled)
                if trial >= args.warmups:
                    samples[key].append(timing)
                    final[key] = outcome
        clearances = {}
        for key, outcome in final.items():
            if outcome.executable:
                clearances[key] = sampled_clearance(request, outcome.plan)
        record = {
            "scenario": name,
            "grid_occupancy_sha256": sha256(np.ascontiguousarray(request.grid.occ).tobytes()).hexdigest(),
            "resolution_m": request.grid.res,
            "start_xyz": list(asdict(request.start_qf).values()),
            "goal_xyz": list(asdict(request.goal_qf).values()),
            "cargo": asdict(request.cargo_footprint),
            "timing_limits": asdict(request.successor_config.timing_limits),
            "search_budget_s": request.search_config.max_seconds,
            "search_budget_expansions": request.search_config.max_expansions,
            "sampled_cargo_clearance_m": clearances,
            "same_final_plan": samples["legacy"][-1]["plan_sha256"] == samples["corridor"][-1]["plan_sha256"],
            "samples": samples,
            "summary": {key: summary(values) for key, values in samples.items()},
        }
        out["cases"].append(record)
        args.output.write_text(json.dumps(out, indent=2, allow_nan=False) + "\n")
        print(name, *(f"{key}={rows[-1]['reason']}:{rows[-1]['wall_s']:.3f}"
                      f":{rows[-1]['used_connector']}"
                      for key, rows in samples.items()), flush=True)


if __name__ == "__main__":
    main()
