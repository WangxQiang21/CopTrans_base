#!/usr/bin/env python3
"""Synthesize an OccupancyGrid from a scene yaml.

Outputs into <maps_dir>/<scene_name>/{map.pgm, map.yaml, scene_meta.yaml}.
The .pgm + .yaml pair follows ROS map_server conventions and can be loaded by
fwis_map_server.scripts.map_publisher (or rviz directly via Map display).

Usage:
  rosrun fwis_map_server gen_map.py \
    --scene $(rospack find fwis_v3_demo)/../../../maps/map16-s09-underground-garage/scene_source.yaml \
    --out   $(rospack find fwis_v3_demo)/../../../maps
"""
from __future__ import annotations

import argparse
import os
import shutil

import numpy as np
import yaml
from PIL import Image


def draw_rect(grid: np.ndarray, res: float, origin: tuple,
              x_range: tuple, y_range: tuple) -> None:
  """Mark cells inside a world-frame rect as occupied (value 0 in pgm convention)."""
  ox, oy, _ = origin
  H, W = grid.shape
  i0 = max(0, int(np.floor((y_range[0] - oy) / res)))
  i1 = min(H, int(np.ceil ((y_range[1] - oy) / res)))
  j0 = max(0, int(np.floor((x_range[0] - ox) / res)))
  j1 = min(W, int(np.ceil ((x_range[1] - ox) / res)))
  if i1 > i0 and j1 > j0:
    grid[i0:i1, j0:j1] = 0


def draw_circle(grid: np.ndarray, res: float, origin: tuple,
                center: tuple, radius: float) -> None:
  """Filled disk obstacle."""
  ox, oy, _ = origin
  H, W = grid.shape
  cx_px = (center[0] - ox) / res
  cy_px = (center[1] - oy) / res
  r_px  = radius / res
  i0 = max(0, int(np.floor(cy_px - r_px)))
  i1 = min(H, int(np.ceil (cy_px + r_px)))
  j0 = max(0, int(np.floor(cx_px - r_px)))
  j1 = min(W, int(np.ceil (cx_px + r_px)))
  if i1 <= i0 or j1 <= j0:
    return
  ys, xs = np.ogrid[i0:i1, j0:j1]
  mask = (xs - cx_px) ** 2 + (ys - cy_px) ** 2 <= r_px ** 2
  grid[i0:i1, j0:j1][mask] = 0


def draw_polygon(grid: np.ndarray, res: float, origin: tuple,
                 points: list) -> None:
  """Convex/concave polygon, points = [[x0,y0], [x1,y1], ...] in world frame."""
  ox, oy, _ = origin
  H, W = grid.shape
  pts_px = np.array([[(p[0] - ox) / res, (p[1] - oy) / res] for p in points])
  if len(pts_px) < 3:
    return
  # Bounding box
  i0 = max(0, int(np.floor(pts_px[:, 1].min())))
  i1 = min(H, int(np.ceil (pts_px[:, 1].max()) + 1))
  j0 = max(0, int(np.floor(pts_px[:, 0].min())))
  j1 = min(W, int(np.ceil (pts_px[:, 0].max()) + 1))
  if i1 <= i0 or j1 <= j0:
    return
  # Standard ray-casting point-in-polygon
  ys, xs = np.mgrid[i0:i1, j0:j1]
  xs_f = xs.astype(float) + 0.5
  ys_f = ys.astype(float) + 0.5
  inside = np.zeros_like(xs, dtype=bool)
  n = len(pts_px)
  j = n - 1
  for i in range(n):
    yi, yj = pts_px[i, 1], pts_px[j, 1]
    xi, xj = pts_px[i, 0], pts_px[j, 0]
    cond = ((yi > ys_f) != (yj > ys_f))
    denom = (yj - yi)
    safe_denom = np.where(np.abs(denom) < 1e-12, 1e-12, denom)
    x_intersect = (xj - xi) * (ys_f - yi) / safe_denom + xi
    inside ^= cond & (xs_f < x_intersect)
    j = i
  grid[i0:i1, j0:j1][inside] = 0


def gen(scene_yaml: str, out_dir: str) -> str:
  with open(scene_yaml, "r") as f:
    scene = yaml.safe_load(f)

  size_m = scene["size_m"]
  res    = float(scene["resolution"])
  origin = scene.get("origin", [0.0, 0.0, 0.0])

  W = int(round(size_m[0] / res))
  H = int(round(size_m[1] / res))
  # 254 = free (white), 0 = occupied (black). map_server thresholds: occupied <= 65 by default.
  grid = np.full((H, W), 254, dtype=np.uint8)

  if scene.get("border", False):
    bt = float(scene.get("border_thickness", 0.10))
    draw_rect(grid, res, origin, (0.0, size_m[0]), (0.0, bt))
    draw_rect(grid, res, origin, (0.0, size_m[0]), (size_m[1] - bt, size_m[1]))
    draw_rect(grid, res, origin, (0.0, bt),       (0.0, size_m[1]))
    draw_rect(grid, res, origin, (size_m[0] - bt, size_m[0]), (0.0, size_m[1]))

  for w in scene.get("walls", []):
    t = w["type"]
    if t == "rect":
      draw_rect(grid, res, origin, w["x"], w["y"])
    elif t == "circle":
      draw_circle(grid, res, origin, w["center"], float(w["radius"]))
    elif t == "polygon":
      draw_polygon(grid, res, origin, w["points"])
    else:
      raise ValueError(f"Unknown wall type: {t}")

  # ROS map convention: image y axis points down. We built it so that grid[i,j]
  # corresponds to world (x = j*res + ox, y = i*res + oy). map_server expects
  # the image with origin at the BOTTOM-LEFT and y growing UPWARD, so the
  # final image must be vertically flipped before saving.
  img_array = np.flipud(grid)

  scene_name = scene.get("map_id", scene.get(
      "name", os.path.splitext(os.path.basename(scene_yaml))[0]))
  scene_dir  = os.path.join(out_dir, scene_name)
  os.makedirs(scene_dir, exist_ok=True)

  pgm_path = os.path.join(scene_dir, "map.pgm")
  Image.fromarray(img_array, mode="L").save(pgm_path, format="PPM")

  map_yaml = {
    "image":           "map.pgm",
    "resolution":      res,
    "origin":          [float(origin[0]), float(origin[1]), float(origin[2])],
    "negate":          0,
    "occupied_thresh": 0.65,
    "free_thresh":     0.196,
  }
  with open(os.path.join(scene_dir, "map.yaml"), "w") as f:
    yaml.safe_dump(map_yaml, f, sort_keys=False)

  # Copy scene meta verbatim for downstream use
  shutil.copyfile(scene_yaml, os.path.join(scene_dir, "scene_meta.yaml"))

  print(f"[gen_map] wrote {pgm_path}  ({W}x{H} cells, {res*100:.1f} cm/px)")
  print(f"[gen_map] start={scene['start']}  goal={scene['goal']}")
  return scene_dir


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--scene", required=True, help="path to scenario yaml")
  ap.add_argument("--out",   required=True, help="output maps directory")
  args = ap.parse_args()
  gen(args.scene, args.out)


if __name__ == "__main__":
  main()
