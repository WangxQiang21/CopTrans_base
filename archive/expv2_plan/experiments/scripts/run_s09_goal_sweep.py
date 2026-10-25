#!/usr/bin/env python3
"""Run the ideal geometric planner over all documented S09 goal presets."""
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

PRESETS = [
    ("BOT_LEFT", .740, .500, -1.5708, "A"),
    ("BOT_MID", 5.860, .500, -1.5708, "A"),
    ("BOT_RIGHT", 10.980, .500, -1.5708, "A"),
    ("MID_A", 7.250, 4.000, 1.5708, "A"),
    ("MID_B", 2.700, 4.900, -1.5708, "A"),
    ("TOP_LEFT", .814, 9.214, -.7854, "P"),
    ("TOP_MID", 6.014, 9.214, -.7854, "P"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=str(HERE.parents[2] / "scenarios/S09_underground_garage.yaml"))
    ap.add_argument("--map", default=str(HERE.parents[2] / "maps/S09_underground_garage/map.yaml"))
    ap.add_argument("--output", default=None)
    ap.add_argument("--max-expansions", type=int, default=120_000)
    ap.add_argument("--max-seconds", type=float, default=35.0)
    ap.add_argument("--heuristic-weight", type=float, default=1.0)
    ap.add_argument("--presets", nargs="*", default=None)
    ap.add_argument("--all-pairs", action="store_true", help="run both Ackermann-only and multi-mode for each preset")
    args = ap.parse_args()
    scene_path = Path(args.scene)
    scene = yaml.safe_load(scene_path.read_text())
    vehicle = Vehicle()
    world = RasterWorld(args.map, vehicle)
    start = Pose(scene["start"]["x"], scene["start"]["y"], scene["start"]["theta"])
    run_id = datetime.now().strftime("s09_presets_%Y%m%d_%H%M%S")
    out = Path(args.output) if args.output else HERE / "results" / "raw" / run_id
    out.mkdir(parents=True, exist_ok=True)

    jobs = []
    if args.presets is None:
        selected = PRESETS
    else:
        preset_map = {p[0]:p for p in PRESETS}
        selected = [preset_map[name] for name in args.presets if name in preset_map]
    for name, x, y, yaw, expected in selected:
        goal = Pose(x, y, yaw)
        jobs.append((name, "multi_mode", goal, {"A", "P", "L", "S"}, expected))
        if args.all_pairs and name in ("BOT_MID", "BOT_RIGHT", "MID_B", "TOP_MID"):
            jobs.append((name, "ackermann_only", goal, {"A"}, expected))

    summaries, serial = [], {}
    for preset, method, goal, modes, expected in jobs:
        case = f"{preset}_{method}"
        print(f"[run] {case}", flush=True)
        # Fresh raster/cache per run so runtime does not benefit from a prior
        # planner's memoized collision queries.
        run_world = RasterWorld(args.map, vehicle)
        result = plan(run_world, vehicle, start, goal, modes,
                      max_expansions=args.max_expansions,
                      max_seconds=args.max_seconds,
                      heuristic_weight=args.heuristic_weight)
        serial[case] = result
        (out / f"{case}.json").write_text(json.dumps(result, default=lambda o:o.__dict__, indent=2))
        summary = {k:v for k,v in result.items() if k not in ("path", "modes", "actions", "best_pose")}
        summary.update(case=case, preset=preset, method=method,
                       expected_mode=expected, enabled_modes="".join(sorted(modes)),
                       heuristic_weight=args.heuristic_weight)
        if result.get("status") == "success":
            summary["mode_sequence"] = "".join(result["modes"])
            summary["mode_counts"] = json.dumps({m: result["modes"].count(m) for m in sorted(set(result["modes"]))})
            summary["path_length_m"] = sum(math.hypot(b.x-a.x,b.y-a.y) for a,b in zip(result["path"],result["path"][1:]))
            summary["used_expected_mode"] = expected in result["modes"]
        elif "best_pose" in result:
            summary["best_pose"] = result["best_pose"].__dict__
        summaries.append(summary)
        with (out / "summary.csv").open("w", newline="") as f:
            fields = sorted(set().union(*(r.keys() for r in summaries)))
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader(); writer.writerows(summaries)
        print(f"[result] {case}: {result['status']}, expanded={result.get('expanded')}, t={result.get('runtime_s',0):.2f}s", flush=True)

    (out / "results.json").write_text(json.dumps(serial, default=lambda o:o.__dict__, indent=2))
    (out / "scene_snapshot.yaml").write_text(scene_path.read_text())
    (out / "vehicle_snapshot.yaml").write_text((HERE / "specs" / "vehicle_params_ranger.yaml").read_text())
    with (out / "summary.csv").open("w", newline="") as f:
        fields = sorted(set().union(*(r.keys() for r in summaries)))
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader(); writer.writerows(summaries)
    map_meta = yaml.safe_load(Path(args.map).read_text())
    files = {"scene":scene_path,
             "map_yaml":Path(args.map),
             "map_image":Path(args.map).parent/map_meta["image"],
             "vehicle_spec":HERE/"specs"/"vehicle_params_ranger.yaml",
             "planner":HERE/"src"/"lattice_planner.py",
             "spatial_model":HERE/"src"/"spatial_model.py"}
    manifest = {"run_id":out.name,"date_local":datetime.now().astimezone().isoformat(),
                "python":platform.python_version(),"heuristic_weight":args.heuristic_weight,
                "xy_bin_m":0.20,"yaw_bin_deg":15,"primitive_translation_m":0.25,
                "terminal_xy_tolerance_m":0.23,"terminal_yaw_tolerance_deg":12,
                "mode_switch_cost_m":0.18,"max_seconds_per_case":args.max_seconds,
                "sha256":{k:hashlib.sha256(p.read_bytes()).hexdigest() for k,p in files.items()}}
    (out/"manifest.json").write_text(json.dumps(manifest,indent=2))

    fig, ax = plt.subplots(figsize=(13, 9))
    ax.imshow(world.occupied[::-1,:], cmap="gray_r", origin="lower",
              extent=[world.ox,world.ox+world.width*world.res,world.oy,world.oy+world.height*world.res],
              interpolation="nearest", alpha=.9)
    palette = plt.get_cmap("tab10")
    for idx, (case, result) in enumerate(serial.items()):
        if result.get("status") != "success": continue
        path = result["path"]
        ax.plot([p.x for p in path],[p.y for p in path],lw=1.5,alpha=.85,
                color=palette(idx%10),label=case)
    ax.scatter([start.x],[start.y],c="green",marker="o",s=75,label="start")
    ax.set(xlabel="x [m]",ylabel="y [m]",title="S09 preset sweep — ideal perfect-tracking geometric planner")
    ax.set_aspect("equal"); ax.grid(alpha=.15); ax.legend(fontsize=7,loc="upper right")
    fig.tight_layout(); fig.savefig(out/"preset_paths.png",dpi=170)
    print(f"[saved] {out}")


if __name__ == "__main__": main()
