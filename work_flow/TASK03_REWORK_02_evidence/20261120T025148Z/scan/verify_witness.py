#!/usr/bin/env python3
"""Independent recomputation of one assessment MAP_OBB_COLLISION witness.

Reconstructs the frozen s30 grid with the exact build_grid parameters, takes
the first collision station recorded by the production assessment, computes
the three vehicle OBBs directly from the synthetic formation model offsets
(no production collision code), and reports the exact overlap depth of each
vehicle rectangle with the occupied wall cells.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path('/home/wq/whut_thesis/expv3_multi_virtual_a')
sys.path.insert(0, str(ROOT / 'catkin_ws/src/fwis_v3_optimizer/src'))

from fwis_v3_optimizer.core import Grid  # noqa: E402

WALL = dict(wall_x=9.0, wall_y0=-10.0, wall_y1=0.5, wall_half=1.5)
OFFSETS = {1: (1.0, 0.0), 2: (0.0, 1.0), 3: (0.0, -1.0)}
HALF = (0.738 / 2.0, 0.500 / 2.0)


def build_grid(res=0.1):
    a, b, deg = 9.0, 11.0, 0.0
    theta = math.radians(deg)
    gx, gy = a + b * math.cos(theta), b * math.sin(theta)
    xmax = max(gx, WALL['wall_x']) + 2.5
    ymax = max(gy, WALL['wall_y1']) + 2.5
    ymin = min(0.0, WALL['wall_y0']) - 2.5
    cols = int((xmax + 2.5) / res) + 4
    rows = int((ymax - ymin + 2.5) / res) + 4
    grid = Grid(np.zeros((rows, cols), dtype=bool), res, -2.5, ymin - 2.5,
                length=0.738, width=0.500)
    ix0 = int((WALL['wall_x'] - WALL['wall_half'] - grid.ox) / res)
    ix1 = int((WALL['wall_x'] + WALL['wall_half'] - grid.ox) / res)
    iy0 = int((WALL['wall_y0'] - grid.oy) / res)
    iy1 = int((WALL['wall_y1'] - grid.oy) / res)
    grid.occ[iy0:iy1 + 1, ix0:ix1 + 1] = True
    return grid


def obb_corners(cx, cy, theta):
    c, s = math.cos(theta), math.sin(theta)
    hx, hy = HALF
    return [
        (cx + c * sx - s * sy, cy + s * sx + c * sy)
        for sx, sy in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy))]


def polygon_axes(points):
    axes = []
    for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]):
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length > 0.0:
            axes.append((-dy / length, dx / length))
    return axes


def sat_overlap(first, second):
    """Return the minimum translation depth (0 when separated)."""
    depth = 0.0
    for axis in polygon_axes(first) + polygon_axes(second):
        p1 = [axis[0] * x + axis[1] * y for x, y in first]
        p2 = [axis[0] * x + axis[1] * y for x, y in second]
        overlap = min(max(p1), max(p2)) - max(min(p1), min(p2))
        if overlap <= 0.0:
            return 0.0
        depth = max(depth, overlap)
    return depth


def main():
    grid = build_grid()
    stations = json.loads(
        (Path(__file__).parent / 's30.assessments.json').read_text())[0][
            'station_coverage']
    hit = next(st for st in stations
               if st.get('map_status') != 'CLEAR_AT_SAMPLES')
    pose = hit['guide_pose']
    fx, fy, psi = pose['x'], pose['y'], pose['heading_rad']
    c, s = math.cos(psi), math.sin(psi)
    res = float(grid.res)
    occupied = []
    iy, ix = np.nonzero(grid.occ)
    for row, col in zip(iy.tolist(), ix.tolist()):
        x0 = grid.ox + col * res
        y0 = grid.oy + row * res
        occupied.append([(x0, y0), (x0 + res, y0), (x0 + res, y0 + res),
                         (x0, y0 + res)])
    report = {
        'scene': 's30',
        'station_sigma': hit['sigma'],
        'station_s_m': hit['s_m'],
        'formation_pose': {'x': fx, 'y': fy, 'psi': psi},
        'model_offsets': OFFSETS,
        'half_extents_m': HALF,
        'vehicles': {},
    }
    for vid, (ox, oy) in OFFSETS.items():
        cx = fx + c * ox - s * oy
        cy = fy + s * ox + c * oy
        corners = obb_corners(cx, cy, psi)
        depths = [sat_overlap(corners, cell) for cell in occupied]
        worst = max(depths) if depths else 0.0
        report['vehicles'][str(vid)] = {
            'center': [cx, cy],
            'corners': corners,
            'max_penetration_m': worst,
            'collides_bare_obb': worst > 0.0,
        }
    report['any_bare_collision'] = any(
        row['collides_bare_obb'] for row in report['vehicles'].values())
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == '__main__':
    raise SystemExit(main())