#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import sys
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE / "src"))
from lattice_planner import plan
from spatial_model import Pose, RasterWorld, Vehicle


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=str(HERE.parents[2] / "scenarios/S09_underground_garage.yaml"))
    ap.add_argument("--map", default=str(HERE.parents[2] / "maps/S09_underground_garage/map.yaml"))
    ap.add_argument("--output", default=None)
    ap.add_argument("--max-expansions", type=int, default=300_000)
    args = ap.parse_args()

    scene_path = Path(args.scene)
    scene = yaml.safe_load(scene_path.read_text())
    out = Path(args.output) if args.output else HERE / "results" / "raw" / f"s09_pair_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    out.mkdir(parents=True, exist_ok=True)

    vehicle = Vehicle()
    world = RasterWorld(args.map, vehicle)
    start = Pose(float(scene["start"]["x"]), float(scene["start"]["y"]), float(scene["start"]["theta"]))
    goal = Pose(float(scene["goal"]["x"]), float(scene["goal"]["y"]), float(scene["goal"]["theta"]))
    cases = [("ackermann_only", {"A"}),
             ("multi_mode", {"A", "P", "L", "S"})]
    summaries = []
    full_results = {}
    for name, modes in cases:
        print(f"[run] {name}, modes={sorted(modes)}", flush=True)
        run_world = RasterWorld(args.map, vehicle)
        result = plan(run_world, vehicle, start, goal, modes,
                      max_expansions=args.max_expansions)
        full_results[name] = result
        summary = {k: v for k, v in result.items() if k not in ("path", "modes", "actions", "best_pose")}
        summary["case"] = name
        summary["enabled_modes"] = ",".join(sorted(modes))
        if result.get("status") == "success":
            summary["mode_sequence"] = "".join(result["modes"])
            summary["mode_counts"] = {m: result["modes"].count(m) for m in sorted(set(result["modes"]))}
            summary["path_length_m"] = sum(math.hypot(b.x-a.x, b.y-a.y) for a,b in zip(result["path"], result["path"][1:]))
        elif "best_pose" in result:
            summary["best_pose"] = result["best_pose"].__dict__
        summaries.append(summary)
        print(f"[result] {name}: status={result['status']} expanded={result.get('expanded')} runtime={result.get('runtime_s',0):.2f}s", flush=True)

    (out / "results.json").write_text(json.dumps(full_results, default=lambda o: o.__dict__, indent=2))
    (out / "scenario_snapshot.yaml").write_text(scene_path.read_text())
    (out / "vehicle_snapshot.yaml").write_text((HERE / "specs" / "vehicle_params_ranger.yaml").read_text())
    map_meta = yaml.safe_load(Path(args.map).read_text())
    files = {"scene":scene_path,"map_yaml":Path(args.map),
             "map_image":Path(args.map).parent/map_meta["image"],
             "vehicle_spec":HERE/"specs"/"vehicle_params_ranger.yaml",
             "planner":HERE/"src"/"lattice_planner.py",
             "spatial_model":HERE/"src"/"spatial_model.py"}
    manifest = {"run_id":out.name,"date_local":datetime.now().astimezone().isoformat(),
                "python":platform.python_version(),"heuristic_weight":1.0,
                "xy_bin_m":0.20,"yaw_bin_deg":15,"primitive_translation_m":0.25,
                "terminal_xy_tolerance_m":0.23,"terminal_yaw_tolerance_deg":12,
                "mode_switch_cost_m":0.18,"sha256":{k:hashlib.sha256(p.read_bytes()).hexdigest() for k,p in files.items()}}
    (out/"manifest.json").write_text(json.dumps(manifest,indent=2))
    with (out / "summary.csv").open("w", newline="") as f:
        keys = sorted(set().union(*(s.keys() for s in summaries)))
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(summaries)

    fig, ax = plt.subplots(figsize=(13, 9))
    ax.imshow(world.occupied[::-1, :], cmap="gray_r", origin="lower",
              extent=[world.ox, world.ox + world.width*world.res,
                      world.oy, world.oy + world.height*world.res],
              interpolation="nearest", alpha=0.9)
    colors = {"ackermann_only": "tab:blue", "multi_mode": "tab:orange"}
    for name, result in full_results.items():
        if result.get("status") != "success":
            continue
        p = result["path"]
        ax.plot([q.x for q in p], [q.y for q in p], color=colors[name], lw=2.2, label=name)
    ax.scatter([start.x], [start.y], c="green", marker="o", s=70, label="start")
    ax.scatter([goal.x], [goal.y], c="red", marker="x", s=90, label="goal")
    ax.set(xlabel="x [m]", ylabel="y [m]", title="Ranger Mini ideal geometric planning — S09 default task")
    ax.set_aspect("equal")
    ax.grid(alpha=0.15)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out / "paths.png", dpi=170)
    print(f"[saved] {out}")


if __name__ == "__main__":
    main()
