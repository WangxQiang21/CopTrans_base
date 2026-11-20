#!/usr/bin/env python3
"""Reproduce frozen TASK02 request/grid/planning-context identity.

The TASK02 captures retained the raw ROS ``FormationPlanRequest`` fields in
``capture_summary.json`` (the ``request`` member), the canonical planner
context, and the canonical request SHA-256.  This script rebuilds the G4
objects from those retained fields and checks every digest against the run's
own admission record.  It never launches ROS and never modifies frozen inputs.

Examples (from the repository root):

    source /opt/ros/noetic/setup.bash
    source catkin_ws/devel/setup.bash
    export PYTHONPATH="$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"

    python3 work_flow/TASK03_evidence/20261119T133018Z/baseline/reproduce_run_identity.py

    python3 work_flow/TASK03_evidence/20261119T133018Z/baseline/reproduce_run_identity.py \\
      --capture work_flow/TASK02_evidence/20261119T054527Z/rework/live/map05_default/capture_summary.json \\
      --map-yaml work_flow/TASK03_evidence/20261119T133018Z/baseline/frozen_inputs/MAP05_straight_map.yaml \\
      --map-pgm work_flow/TASK03_evidence/20261119T133018Z/baseline/frozen_inputs/MAP05_straight_map.pgm \\
      --prefix MAP05_straight

The persisted ``<prefix>_request_reconstructed.json`` is the canonical
``request_payload`` (the exact material hashed by ``request_fingerprint``).
``<prefix>_planning_context.json`` is the captured canonical planning context.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import shutil
import sys
import tempfile

import numpy as np


HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_REPO = HERE.parents[3]
TASK02_LIVE = (
    DEFAULT_REPO
    / "work_flow/TASK02_evidence/20261119T054527Z/rework/live"
)
DEFAULT_CAPTURE = TASK02_LIVE / "s1_default/capture_summary.json"
FROZEN = HERE / "frozen_inputs"


def _canonical_sha(value):
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _reconstruct(repo, capture_path, map_yaml, map_pgm):
    sys.path.insert(0, str(repo / "catkin_ws/src/fwis_v3_optimizer/src"))
    from fwis_v3_optimizer.core import Grid
    from fwis_v3_optimizer.formation import (
        FormationModel,
        FormationPose,
        VehicleOffset,
    )
    from fwis_v3_optimizer.formation_collision import VehicleShape
    from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
    import fwis_v3_optimizer.formation_continuous_a.contracts as contracts
    from fwis_v3_optimizer.formation_continuous_a.collision import load_map

    capture = json.loads(capture_path.read_text())
    raw = capture["request"]
    admitted = capture["admitted_job"]
    admission = admitted["admission"]
    observed = {
        "plan_id": admission["plan_id"],
        "request_sha256": admission["request_sha256"],
        "grid_sha256": admission["grid_sha256"],
        "planning_context_sha256": admission["planning_context_sha256"],
    }
    if admitted["planning_context"]["request_sha256"] != observed["request_sha256"]:
        raise RuntimeError("capture request digest inconsistency")

    model = FormationModel([
        VehicleOffset(v["vehicle_id"], v["offset_x"], v["offset_y"])
        for v in raw["vehicles"]
    ])
    request = contracts.VirtualARequest(
        plan_id=raw["plan_id"],
        formation_id=raw["formation_id"],
        model=model,
        vehicle_shapes={
            v["vehicle_id"]: VehicleShape(v["body_length_m"], v["body_width_m"])
            for v in raw["vehicles"]
        },
        start_qf=FormationPose(
            raw["formation_start"]["x"],
            raw["formation_start"]["y"],
            raw["formation_start"]["theta"],
        ),
        start_headings={
            v["vehicle_id"]: v["start_theta"] for v in raw["vehicles"]
        },
        goal_qf=FormationPose(
            raw["formation_goal"]["x"],
            raw["formation_goal"]["y"],
            raw["formation_goal"]["theta"],
        ),
        initial=contracts.InitialAssumption(
            stationary=True,
            mode="A",
            wheels_straight=True,
            source=admission["initial_source"],
            context_id=admission["initial_context_id"],
        ),
        goal_policy=contracts.GoalPolicy.FREE,
        limits=FormationTimingLimits(),
    )

    temp = tempfile.mkdtemp(prefix="task03-run-identity.")
    try:
        shutil.copy(map_yaml, temp + "/map.yaml")
        shutil.copy(map_pgm, temp + "/map.pgm")
        grid_from_pgm, _ = load_map(temp + "/map.yaml")
    finally:
        shutil.rmtree(temp)
    # ROS OccupancyGrid carries resolution as float32.  Grid.__init__ stores
    # the value as a Python float but keeps the exact float32 bit pattern.
    runtime_resolution = admitted["planning_context"]["grid"]["resolution_m"]
    grid = Grid(
        grid_from_pgm.occ,
        np.float32(runtime_resolution),
        grid_from_pgm.ox,
        grid_from_pgm.oy,
    )

    planner_config = contracts.research_planner_config(
        guidance_mode="corridor", corridor_enforcement="hard"
    )
    context = contracts.planning_context(
        request,
        grid,
        planner_config,
        contracts.TimingConfig(),
        contracts.CollisionConfig(),
    )
    actual = {
        "request_sha256": contracts.request_fingerprint(request),
        "grid_sha256": contracts.grid_signature(grid),
        "planning_context_sha256": contracts.planning_context_sha256(context),
        "planner_config": planner_config.as_dict(),
        "timing_config": contracts.TimingConfig().as_dict(),
        "collision_config": contracts.CollisionConfig().as_dict(),
    }
    checked_context = admitted["planning_context"]
    checks = {
        "request": actual["request_sha256"] == observed["request_sha256"],
        "grid": actual["grid_sha256"] == observed["grid_sha256"],
        "planning_context":
            actual["planning_context_sha256"] == observed["planning_context_sha256"],
        "planner_config":
            actual["planner_config"] == checked_context["planner_config"],
        "timing_config":
            actual["timing_config"] == checked_context["timing_config"],
        "collision_config":
            actual["collision_config"] == checked_context["collision_config"],
        "captured_context_digest":
            contracts.planning_context_sha256(
                checked_context
            ) == observed["planning_context_sha256"],
    }
    if not all(checks.values()):
        raise RuntimeError("run identity mismatch: {}".format(checks))

    payload = contracts.request_payload(request)
    result = {
        "schema": "task03.run_identity_reconstruction.v1",
        "source_capture_summary": str(capture_path),
        "source_capture_sha256": hashlib.sha256(
            capture_path.read_bytes()
        ).hexdigest(),
        "frozen_map_yaml_sha256": hashlib.sha256(map_yaml.read_bytes()).hexdigest(),
        "frozen_map_pgm_sha256": hashlib.sha256(map_pgm.read_bytes()).hexdigest(),
        "observed": observed,
        "reconstructed": {
            "request_sha256": actual["request_sha256"],
            "grid_sha256": actual["grid_sha256"],
            "planning_context_sha256": actual["planning_context_sha256"],
            "grid_resolution_float32": float(np.float32(runtime_resolution)),
            "planner_config_sha256": _canonical_sha(actual["planner_config"]),
            "timing_config_sha256": _canonical_sha(actual["timing_config"]),
            "collision_config_sha256": _canonical_sha(actual["collision_config"]),
        },
        "checks": checks,
        "all_checks_pass": all(checks.values()),
    }
    return payload, context, result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo", type=pathlib.Path, default=DEFAULT_REPO,
        help="repository/worktree root",
    )
    parser.add_argument(
        "--capture", type=pathlib.Path, default=DEFAULT_CAPTURE,
        help="TASK02 capture_summary.json",
    )
    parser.add_argument(
        "--map-yaml", type=pathlib.Path,
        default=FROZEN / "S1_map.yaml", help="frozen map YAML",
    )
    parser.add_argument(
        "--map-pgm", type=pathlib.Path,
        default=FROZEN / "S1_map.pgm", help="frozen map PGM",
    )
    parser.add_argument(
        "--prefix", default="S1", help="output filename prefix",
    )
    parser.add_argument(
        "--output-dir", type=pathlib.Path, default=HERE,
        help="directory for generated identity files",
    )
    args = parser.parse_args()
    payload, context, result = _reconstruct(
        args.repo.resolve(), args.capture.resolve(),
        args.map_yaml.resolve(), args.map_pgm.resolve(),
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / (args.prefix + "_request_reconstructed.json")).write_text(
        json.dumps(payload, sort_keys=True, indent=2) + "\n"
    )
    (args.output_dir / (args.prefix + "_planning_context.json")).write_text(
        json.dumps(context, sort_keys=True, indent=2) + "\n"
    )
    (args.output_dir / (args.prefix + "_identity.json")).write_text(
        json.dumps(result, sort_keys=True, indent=2) + "\n"
    )
    print(json.dumps(result, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())