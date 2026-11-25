#!/usr/bin/env python3
"""Blind native solve on each published Gazebo qualification world and certify its output."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess

import numpy as np

from benchmark import invoke
from cases import ROOT, digest, limits, packed
from closed_loop import FAMILIES, fixture_config, fixture_preflight, fixture_scene, footprint_margin
from qualification_scenes import SPECIAL_SCENES
from qualify import sha
from ranger_mini_gz.scene_generator import installed_paths, read_map, write_map
from ranger_mini_gz.scene_world import write_world


def request_from_scene(scene, task, cfg, grid):
    meta, cells = grid
    edges = np.diff(np.r_[0, (cells != 0).ravel(), 0].astype(int))
    starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
    multi = task == "formation"
    margin = footprint_margin(cfg, task)
    offsets = scene.offsets if multi else [(0., 0.)]
    length, width = [d + 2 * margin for d in scene.vehicle_size[:2]]
    start, goal = cfg["start"], cfg["goal"]
    start_headings = [pose[2] for pose in scene.spawn_poses(task).values()] if multi else [start[2]] * 3
    return {
        "grid": dict(width=cells.shape[1], height=cells.shape[0], resolution=float(np.float32(meta["resolution"])),
                     origin=meta["origin"][:2], occupied_runs=np.c_[starts, ends - starts].tolist()),
        "limits": limits(task, cfg["speed_profile"]), "allow_reverse": True,
        "task_duration_cap": cfg["planner_overrides"]["task_duration_cap_s"],
        "vehicles": [[x, y, length, width] for x, y in offsets],
        "cargo": [*scene.cargo.offset, *(d + 2 * margin for d in scene.cargo.size[:2])] if multi else None,
        "start": dict(reference=start, heading=start_headings, constrain_reference_yaw=True, constrain_heading=[True]*3),
        "goal": dict(reference=goal, heading=[goal[2]]*3, constrain_reference_yaw=True, constrain_heading=[not multi]*3)
    }


def check_all(executable, output, families):
    out = Path(output)
    out.mkdir(parents=True, exist_ok=False)
    locations = (ROOT / "ros2_ws/src/ranger_mini_gz", *installed_paths()[1:])
    linked = subprocess.check_output(["ldd", executable], text=True)
    library = Path(next(line.split("=>", 1)[1].split(" (", 1)[0].strip() for line in linked.splitlines()
                        if "libfwis_hybrid_joint.so =>" in line))
    summary = dict(executable=str(executable), binary_sha256=sha(executable), library_sha256=sha(library),
                   core_source_commit=subprocess.check_output(["git", "log", "-1", "--format=%H", "--",
                                                              "ros2_ws/src/fwis_hybrid_joint"], cwd=ROOT, text=True).strip(),
                   generated_utc=datetime.now(timezone.utc).isoformat(),
                   fixture_script_sha256=sha(Path(__file__).with_name("closed_loop.py")),
                   policy_sha256=sha(ROOT/"ros2_ws/src/fwis_hybrid_joint/config/policy.json"), cases=[])
    grids = {}
    for family in families:
        directory = out / family
        directory.mkdir()
        for task in ("single", "formation"):
            for profile in ("standard", "baseline"):
                cfg = fixture_config(family, task, profile)
                scene = fixture_scene(cfg, task, paths=locations)
                proof = fixture_preflight(family, task, cfg, scene)
                scene_key = scene.name + "_" + scene.variant
                map_dir = out / "maps" / scene_key
                if scene_key not in grids:
                    map_dir.mkdir(parents=True)
                    write_map(scene, map_dir, "map", scene.resolution)
                    grids[scene_key] = read_map(map_dir/"map.yaml")
                grid = grids[scene_key]
                world = directory / f"{task}.sdf"
                if not world.exists():
                    write_world(scene, world, task)
                req = request_from_scene(scene, task, cfg, grid)
                stem = directory / f"{task}_{profile}"
                budget = cfg["planner_overrides"]["solve_budget_s"]
                payload = dict(operation="solve", request=req, budget_s=budget)
                stem.with_suffix(".request.json").write_bytes(packed(payload))
                solved = invoke(executable, payload, budget + 7)
                stem.with_suffix(".result.json").write_bytes(packed(solved))
                if solved.get("status") == "process_failed":
                    raise RuntimeError(f"Native solver did not execute: {solved.get('reason')}")
                audit = {}
                if solved.get("plan"):
                    audit = invoke(executable, dict(operation="certify", request=req, plan=solved["plan"], budget_s=8), 15)
                modes = solved.get("metrics", {}).get("mode_sequence", "")
                if family == "necessary_aux":
                    mode_ok = any(m in modes for m in "OS")
                elif family in ("cross_dock", "close_turn"):
                    mode_ok = bool(modes)
                else:
                    mode_ok = modes == "A"
                gears = [phase["gear"][:len(req["vehicles"])] for phase in solved.get("plan", []) if phase["kind"] == "motion"]
                if family == "reverse":
                    mode_ok = mode_ok and bool(gears) and all(g == -1 for phase in gears for g in phase)
                row = dict(task=task, profile=profile, family=family, bounds=proof, backend=cfg["planner_backend"],
                           start=cfg["start"], goal=cfg["goal"], gears=gears, limits_sha256=digest(req["limits"]),
                           request_sha256=digest(req), map_pgm_sha256=sha(map_dir/"map.pgm"),
                           scenario_yaml_sha256=sha(locations[0]/"scenarios"/(scene.name+".yaml")),
                           world_sha256=sha(world),
                           request_duration_cap_s=req["task_duration_cap"], duration_s=solved.get("duration_s"),
                           solve_wall_s=solved.get("wall_s"), status=solved.get("status"), modes=modes,
                           audit=audit, passed=(solved.get("status")=="CERTIFIED_POLICY_COMPLIANT" and mode_ok and
                                               solved.get("actual_backend")==cfg["planner_backend"] and audit.get("executable") is True))
                summary["cases"].append(row)
                print(family, task, profile, row["passed"], modes, row["duration_s"], flush=True)
    (out/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    return all(r["passed"] for r in summary["cases"])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--executable", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True, help="New local evidence folder")
    p.add_argument("--families", default="special",
                   help="ordinary, special, all, or a comma-separated list of closed-loop family names")
    args = p.parse_args()
    groups = {"ordinary": [family for family in FAMILIES if family not in SPECIAL_SCENES],
              "special": list(SPECIAL_SCENES), "all": list(FAMILIES)}
    families = groups.get(args.families, args.families.split(","))
    if len(set(families)) != len(families) or any(f not in FAMILIES for f in families):
        p.error("--families must select unique known closed-loop families")
    return 0 if check_all(args.executable.resolve(), args.output, families) else 1


if __name__ == "__main__":
    raise SystemExit(main())
