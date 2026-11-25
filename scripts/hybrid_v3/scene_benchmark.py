#!/usr/bin/env python3
"""Blind native solve on each published Gazebo qualification world and certify its output."""
import argparse
import json
from pathlib import Path
import subprocess

import numpy as np
import yaml

from benchmark import invoke
from cases import ROOT, digest, limits, packed
from qualification_scenes import SPECIAL_SCENES, setup
from qualify import sha
from ranger_mini_gz.scene_generator import generate_scene, installed_paths, load_scene, read_map


def request_from_scene(scene, task, cfg, grid):
    meta, cells = grid
    edges = np.diff(np.r_[0, (cells != 0).ravel(), 0].astype(int))
    starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
    multi = task == "formation"
    margin = .05 if multi else .04
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


def check_all(executable, output):
    out = Path(output)
    out.mkdir(parents=True, exist_ok=False)
    locations = (ROOT / "ros2_ws/src/ranger_mini_gz", *installed_paths()[1:])
    linked = subprocess.check_output(["ldd", executable], text=True)
    library = Path(next(line.split("=>", 1)[1].split(" (", 1)[0].strip() for line in linked.splitlines()
                        if "libfwis_hybrid_joint.so =>" in line))
    summary = dict(executable=str(executable), binary_sha256=sha(executable), library_sha256=sha(library), cases=[])
    for family, name in SPECIAL_SCENES.items():
        directory = out / name
        scene = load_scene(name, paths=locations)
        generate_scene(scene, directory, "formation", locations[0]/"config/ranger.gui.config", audit=False)
        grid = read_map(directory/"map.yaml")
        for task in ("single", "formation"):
            for profile in ("standard", "baseline"):
                args = [str(ROOT/"scripts/test.sh"), "--dry-run", "--task", "multi" if task == "formation" else "single",
                        "--scenario", name, "--experiment", "full_route", "--planner-backend", "hybrid_joint",
                        "--speed-profile", profile]
                cfg = yaml.safe_load(subprocess.check_output(args, text=True))
                proof = setup(family, task, cfg, scene=scene)
                req = request_from_scene(scene, task, cfg, grid)
                stem = directory / f"{task}_{profile}"
                budget = cfg["planner_overrides"]["solve_budget_s"]
                payload = dict(operation="solve", request=req, budget_s=budget)
                stem.with_suffix(".request.json").write_bytes(packed(payload))
                solved = invoke(executable, payload, budget + 7)
                stem.with_suffix(".result.json").write_bytes(packed(solved))
                audit = {}
                if solved.get("plan"):
                    audit = invoke(executable, dict(operation="certify", request=req, plan=solved["plan"], budget_s=8), 15)
                modes = solved.get("metrics", {}).get("mode_sequence", "")
                mode_ok = modes == "A" if family == "multibend" else any(m in modes for m in "OS")
                row = dict(task=task, profile=profile, family=family, bounds=proof, backend=cfg["planner_backend"],
                           request_sha256=digest(req), map_pgm_sha256=sha(directory/"map.pgm"),
                           scenario_yaml_sha256=sha(locations[0]/"scenarios"/(name+".yaml")),
                           world_sha256=sha(directory/("world.sdf" if task=="formation" else "world_single.sdf")),
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
    args = p.parse_args()
    return 0 if check_all(args.executable.resolve(), args.output) else 1


if __name__ == "__main__":
    raise SystemExit(main())
