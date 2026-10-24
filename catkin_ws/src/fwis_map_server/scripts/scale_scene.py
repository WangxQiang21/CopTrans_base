#!/usr/bin/env python3
"""Scale a scenario yaml by a constant factor k.

Use case: take a scenario authored for a standard passenger car (e.g. the
ISO 16787 parallel-parking layout in scenarios/_iso_parallel_passenger.yaml,
sized for a 4.50 x 1.80 m sedan) and produce a geometrically equivalent
scenario at Ranger Mini scale (0.738 x 0.500 m, k = 0.738/4.50 ~= 0.164).

What gets multiplied by k:
  - size_m
  - resolution         (so each cell still occupies the same fraction of
                        the layout; a 30 m x 16 m map at 0.05 m/cell becomes
                        a 4.92 m x 2.62 m map at ~0.0082 m/cell)
  - origin (x, y); origin theta is left untouched
  - border_thickness
  - every wall:
      rect:    x and y ranges
      circle:  center x/y and radius
      polygon: every vertex
  - start.x, start.y; start.theta untouched
  - goal.x,  goal.y;  goal.theta  untouched
  - every narrow_segments entry: x_range, y_range (if present), and w

Usage:
  python3 scale_scene.py \
      --in  exp/scenarios/_iso_parallel_passenger.yaml \
      --k   0.164 \
      --name S10_iso_parallel_scaled \
      --out exp/scenarios/S10_iso_parallel_scaled.yaml
"""
from __future__ import annotations

import argparse
import os
from typing import Any

import yaml


def _scale_pair(seq, k):
  return [float(v) * k for v in seq]


def _scale_wall(w: dict, k: float) -> dict:
  t = w["type"]
  out = {"type": t}
  if t == "rect":
    out["x"] = _scale_pair(w["x"], k)
    out["y"] = _scale_pair(w["y"], k)
  elif t == "circle":
    out["center"] = _scale_pair(w["center"], k)
    out["radius"] = float(w["radius"]) * k
  elif t == "polygon":
    out["points"] = [[float(p[0]) * k, float(p[1]) * k] for p in w["points"]]
  else:
    raise ValueError(f"unknown wall type: {t}")
  for extra in ("label", "note"):
    if extra in w:
      out[extra] = w[extra]
  return out


def _scale_pose(p: dict, k: float) -> dict:
  out = dict(p)
  out["x"] = float(p["x"]) * k
  out["y"] = float(p["y"]) * k
  return out


def _scale_narrow(n: dict, k: float) -> dict:
  out = dict(n)
  if "x_range" in n:
    out["x_range"] = _scale_pair(n["x_range"], k)
  if "y_range" in n:
    out["y_range"] = _scale_pair(n["y_range"], k)
  if "w" in n:
    out["w"] = float(n["w"]) * k
  return out


def scale_scene(scene: dict, k: float, name: str | None = None) -> dict:
  out: dict[str, Any] = {}
  out["name"] = name or (scene.get("name", "scene") + "_scaled")
  out["size_m"] = _scale_pair(scene["size_m"], k)
  out["resolution"] = float(scene["resolution"]) * k
  o = scene.get("origin", [0.0, 0.0, 0.0])
  out["origin"] = [float(o[0]) * k, float(o[1]) * k, float(o[2])]
  out["border"] = bool(scene.get("border", False))
  if "border_thickness" in scene:
    out["border_thickness"] = float(scene["border_thickness"]) * k
  out["walls"] = [_scale_wall(w, k) for w in scene.get("walls", [])]
  out["start"] = _scale_pose(scene["start"], k)
  out["goal"] = _scale_pose(scene["goal"], k)
  if "narrow_segments" in scene:
    out["narrow_segments"] = [_scale_narrow(n, k) for n in scene["narrow_segments"]]
  out["_scaled_from"] = {
    "source": scene.get("name", "?"),
    "k": float(k),
  }
  return out


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--in", dest="src", required=True, help="source yaml (passenger-car units)")
  ap.add_argument("--k", type=float, required=True, help="scale factor (e.g. 0.164 = 0.738/4.50)")
  ap.add_argument("--name", default=None, help="name field in the output yaml")
  ap.add_argument("--out", required=True, help="destination yaml path")
  args = ap.parse_args()

  with open(args.src, "r") as f:
    src = yaml.safe_load(f)
  scaled = scale_scene(src, args.k, args.name)

  os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
  with open(args.out, "w") as f:
    yaml.safe_dump(scaled, f, sort_keys=False)
  print(f"[scale_scene] k={args.k}  {args.src}  ->  {args.out}")
  print(f"              size_m={scaled['size_m']}  resolution={scaled['resolution']:.4f}")
  print(f"              start={scaled['start']}  goal={scaled['goal']}")


if __name__ == "__main__":
  main()
