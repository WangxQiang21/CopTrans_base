#!/usr/bin/env python3
"""Offline independent geometry, dynamics and collision audits of this plan.

The V2 task changed only display export/rendering.  This script does not re-run
the solver.  It rebuilds the exact authoritative OccupancyGrid from the
recorded synthetic-scene parameters, checks that the rebuilt grid summary and
digest equal the job's admitted grid, rebuilds the same plan with the
production envelope verifier, and then calls the three production audit
functions on that saved plan.

A pass proves the saved plan still passes independent geometry, dynamics and
continuous OBB/map collision audits bound to the recorded request, planning
context, model and grid.  It is not a new solver result and not a claim about
any other scene.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation_continuous_a.audit import (
    audit_dynamics,
    audit_geometry,
)
from fwis_v3_optimizer.formation_continuous_a.collision import audit_collision
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    COLLISION_SCOPE,
    DYNAMICS_SCOPE,
    GEOMETRY_SCOPE,
    DynamicAuditConfig,
    collision_config_from_dict,
    geometry_audit_config_from_dict,
    grid_signature,
    grid_summary,
    request_fingerprint,
)
from fwis_v3_optimizer.formation_playback import (
    verify_virtual_a_playback_plan,
    virtual_a_request_from_raw_request,
)

RUN = Path(__file__).resolve().parent
RESULT = RUN / "run_result.json"
MESSAGES = RUN / "display_messages.jsonl"


def last_record(entries, topic):
    found = None
    for entry in entries:
        if entry.get("topic") == topic:
            found = entry["document"]
    if found is None:
        raise RuntimeError("recorded topic missing: {}".format(topic))
    return found


def rebuilt_grid(job, scene):
    radius = float(scene["radius_m"])
    half_width = float(scene["half_width_m"])
    margin = float(scene["margin_m"])
    resolution = float(np.float32(scene["resolution_m"]))
    origin = float(scene["origin_xy"][0])
    size = 2.0 * (radius + half_width + margin)
    count = int(math.ceil(size / float(scene["resolution_m"])))
    coordinates = (np.arange(count) + 0.5) * float(scene["resolution_m"]) + origin
    grid_x, grid_y = np.meshgrid(coordinates, coordinates)
    radial = np.hypot(grid_x, grid_y)
    free = ((radial >= radius - half_width) &
            (radial <= radius + half_width) &
            (grid_x > -0.2))
    data = np.full((count, count), 100, dtype=np.int8)
    data[free] = 0
    return Grid(data != 0, resolution, origin, origin)


def report_summary(report):
    metrics = report.metrics if isinstance(report.metrics, dict) else {}
    statistics = metrics.get("checker_statistics")
    return {
        "scope": report.scope,
        "passed": report.passed,
        "reason": report.reason,
        "detail": report.detail,
        "first_failure": report.first_failure,
        "check_count": report.check_count,
        "request_sha256": report.request_sha256,
        "path_sha256": report.path_sha256,
        "model_sha256": (report.model_sha256 or
                         report.timing.get("model_sha256") or
                         metrics.get("model_sha256")),
        "grid_sha256": report.grid_sha256,
        "planning_context_sha256": report.planning_context_sha256,
        "timing": report.timing,
        "metrics": {
            "checker_statistics": statistics,
            "margin": metrics.get("margin"),
            "collision_checked": metrics.get("collision_checked"),
            "map_used": metrics.get("map_used"),
            "sweep_segment_count": metrics.get("sweep_segment_count"),
            "grid_sha256": metrics.get("grid_sha256"),
            "planning_context_sha256": metrics.get(
                "planning_context_sha256"),
            "model_sha256": metrics.get("model_sha256"),
        },
    }


def main():
    result = json.loads(RESULT.read_text())
    entries = [json.loads(line) for line in
               MESSAGES.read_text().splitlines() if line.strip()]
    job = result["job_state"]
    checks = []

    def add(name, detail, passed):
        checks.append({"name": name, "detail": detail,
                       "passed": bool(passed)})

    grid = rebuilt_grid(job, result["scene"])
    expected_grid = job["planning_context"]["grid"]
    add("rebuilt grid summary equals admitted grid",
        {"rebuilt": grid_summary(grid), "admitted": expected_grid},
        grid_summary(grid) == expected_grid)
    add("rebuilt grid digest equals admitted grid_sha256",
        {"rebuilt": grid_signature(grid),
         "admitted": job["grid_sha256"]},
        grid_signature(grid) == job["grid_sha256"])

    envelope = last_record(entries, "/formation/playback_plan")
    initial = job["initial_context"]
    request = virtual_a_request_from_raw_request(
        result["request"], initial_source=initial["source"],
        initial_context_id=initial["context_id"])
    fingerprint = request_fingerprint(request)
    add("request fingerprint matches admission",
        {"request": fingerprint, "job": job.get("request_sha256")},
        fingerprint == job.get("request_sha256"))
    geometry_config = geometry_audit_config_from_dict(
        job["planning_context"]["timing_config"]["geometry_audit_config"])
    trajectory, plan, digest = verify_virtual_a_playback_plan(
        plan_id=int(envelope["plan_id"]),
        formation_id=envelope["formation_id"],
        backend=envelope["backend"],
        semantic_version=envelope["semantic_version"],
        model_version=envelope["model_version"],
        model_sha256=envelope["model_sha256"],
        model_json=envelope["model_json"],
        certification=envelope["certification"],
        trajectory_fields=envelope["trajectory_fields"],
        expected_request=request,
        expected_context=job["planning_context"],
        expected_grid_sha256=job["grid_sha256"],
        geometry_audit_config=geometry_config)
    add("production envelope verifier",
        {"digest": digest, "samples": len(trajectory.times_from_start),
         "duration_s": plan.duration_s},
        digest == envelope["model_sha256"] and
        len(trajectory.times_from_start) == 1982)

    geometry = audit_geometry(
        request, plan.path, geometry_config,
        planning_context=job["planning_context"])
    geometry_summary = report_summary(geometry)
    add("geometry audit passed", geometry_summary,
        geometry.passed and geometry.scope == GEOMETRY_SCOPE)
    add("geometry audit produced checks",
        {"check_count": geometry.check_count,
         "checker_statistics": geometry.metrics.get(
             "checker_statistics") if isinstance(
                 geometry.metrics, dict) else None},
        geometry.check_count > 0)

    dynamics_config = DynamicAuditConfig(
        geometry_audit_config=geometry_config)
    dynamics = audit_dynamics(
        request, plan, dynamics_config,
        expected_planning_context=job["planning_context"])
    dynamics_summary = report_summary(dynamics)
    add("dynamics audit passed", dynamics_summary,
        dynamics.passed and dynamics.scope == DYNAMICS_SCOPE)
    add("dynamics audit produced checks",
        {"check_count": dynamics.check_count,
         "checker_statistics": dynamics.metrics.get(
             "checker_statistics") if isinstance(
                 dynamics.metrics, dict) else None},
        dynamics.check_count > 0)

    collision_config = collision_config_from_dict(
        job["planning_context"]["collision_config"])
    collision = audit_collision(request, plan, grid, collision_config)
    collision_summary = report_summary(collision)
    add("collision audit passed", collision_summary,
        collision.passed and collision.scope == COLLISION_SCOPE)
    add("collision audit bound to this plan and grid",
        {"model": collision_summary["model_sha256"],
         "plan_context": collision.planning_context_sha256,
         "grid": collision.grid_sha256,
         "job_model": envelope["model_sha256"],
         "job_context": job["planning_context_sha256"],
         "job_grid": job["grid_sha256"]},
        collision_summary["model_sha256"] == envelope["model_sha256"] and
        collision.planning_context_sha256 == job["planning_context_sha256"] and
        collision.grid_sha256 == job["grid_sha256"])
    add("collision sweep and static poses were checked",
        {"collision_checked": collision.metrics.get("collision_checked"),
         "map_used": collision.metrics.get("map_used"),
         "sweep_segments": collision.metrics.get("sweep_segment_count"),
         "check_count": collision.check_count},
        collision.metrics.get("collision_checked") is True and
        collision.metrics.get("map_used") is True and
        int(collision.metrics.get("sweep_segment_count") or 0) > 0 and
        collision.check_count > 0)

    document = {
        "schema": "virtual_a_corridor.v2.audit_verification.v1",
        "run": str(RUN),
        "verified": all(item["passed"] for item in checks),
        "checks": checks,
        "audits": {
            "geometry": geometry_summary,
            "dynamics": dynamics_summary,
            "collision": collision_summary,
        },
    }
    (RUN / "audit_verification.json").write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n")
    failed = [item for item in checks if not item["passed"]]
    print("checks={} passed={} failed={}".format(
        len(checks), len(checks) - len(failed), len(failed)))
    for key, summary in document["audits"].items():
        print("{} passed={} scope={} checks={}".format(
            key, summary["passed"], summary["scope"],
            summary["check_count"]))
    for item in failed:
        print("FAIL {}".format(item["name"]))
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())