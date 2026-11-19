#!/usr/bin/env python3
"""Reproduce one frozen-scene virtual-A plan with an explicit guidance mode.

This script is diagnostic only.  It reads the frozen S1 scene bundle under
``evaluation/`` and calls the real ROS-free pipeline; it never writes into
that bundle and never imports a ROS node.

usage:
  PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src \
      python3 repro_s1_corridor.py --guidance corridor --output /tmp/out.json
  ... --code-root /tmp/task01_head_tree --guidance corridor ...
"""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
import time

WORKTREE = Path("/home/wq/whut_thesis/expv3_multi_virtual_a")
SCENE_DIR = (WORKTREE / "evaluation" / "virtual_formation_a" /
             "G4" / "20261116T081458Z" / "scenes" / "inputs" / "S1")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-dir", default=str(SCENE_DIR))
    parser.add_argument("--guidance", choices=("corridor", "legacy_route"),
                        default="corridor")
    parser.add_argument("--enforcement", choices=("hard", "soft"),
                        default="hard")
    parser.add_argument("--code-root", default="")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.code_root:
        root = Path(args.code_root)
        for entry in (root / "tests",
                      root / "catkin_ws" / "src" / "fwis_v3_optimizer" /
                      "src"):
            sys.path.insert(0, str(entry))

    from fwis_v3_optimizer.formation_continuous_a.collision import load_map
    from fwis_v3_optimizer.formation_continuous_a.contracts import (
        collision_config_from_dict,
        planner_config_from_dict,
        request_from_payload,
        timing_config_from_dict,
    )
    from fwis_v3_optimizer.formation_continuous_a.pipeline import (
        plan_virtual_a,
    )

    scene_dir = Path(args.scene_dir)
    grid, metadata = load_map(scene_dir / "map.yaml")
    payload = json.loads((scene_dir / "request.json").read_text(
        encoding="utf-8"))
    request = request_from_payload(payload["request"])
    planner = replace(
        planner_config_from_dict(payload["planner_config"]),
        guidance_mode=args.guidance,
        corridor_enforcement=args.enforcement)
    timing = timing_config_from_dict(payload["timing_config"])
    collision = collision_config_from_dict(payload["collision_config"])
    from fwis_v3_optimizer.formation_continuous_a.contracts import (
        VirtualAMapRequest,
    )
    map_request = VirtualAMapRequest(
        request, grid, planner, timing, collision)
    started = time.monotonic()
    result = plan_virtual_a(map_request)
    report = result.as_dict()
    rounds = (result.search.get("corridor") or {}).get("rounds") or []
    compact = {
        "schema": "task01_corridor_repro.v1",
        "code_root": args.code_root or str(WORKTREE),
        "scene_dir": str(scene_dir),
        "grid_sha256": metadata.get("grid_sha256"),
        "guidance_mode": args.guidance,
        "corridor_enforcement": args.enforcement,
        "status": result.status,
        "reason": result.reason,
        "detail": result.detail,
        "first_failure_layer": result.first_failure_layer,
        "executable": result.executable,
        "path_length_m": (result.path.length_m if result.path else None),
        "wall_time_s": time.monotonic() - started,
        "round_count": len(result.search.get("rounds") or []),
        "corridor_rounds": [
            {key: item.get(key) for key in (
                "round_index", "route_index", "effective_margin_m",
                "containment")}
            for item in rounds],
        "recoveries": result.search.get("recoveries"),
        "geometry_audit_passed": (None if result.geometry_audit is None
                                  else result.geometry_audit.passed),
        "dynamic_audit_passed": (None if result.dynamic_audit is None
                                 else result.dynamic_audit.passed),
        "collision_audit_passed": (None if result.collision_audit is None
                                   else result.collision_audit.passed),
        "full_report_sha256": __import__("hashlib").sha256(
            json.dumps(report, sort_keys=True, default=str).encode()
        ).hexdigest(),
    }
    Path(args.output).write_text(
        json.dumps(compact, indent=1, sort_keys=True, default=str) + "\n",
        encoding="utf-8")
    print(json.dumps({key: compact[key] for key in (
        "status", "reason", "path_length_m", "wall_time_s")},
        sort_keys=True))


if __name__ == "__main__":
    main()