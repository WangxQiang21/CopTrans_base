#!/usr/bin/env python3
"""Generate the two FR05 verification maps (30 m x 30 m, 0.05 m/cell).

empty_map:    entirely free.  The wedge exercises forward, lateral O and the
              S->A plan with a non-zero pivot and zero-angle vehicles.
blocked_map:  a full-height 0.05 m wall at x=12.5.  (11,10) is reachable from
              (10,10); (13.5,10) is not, so the failure run is an admitted
              planning failure rather than an invalid request.

Usage: python3 make_maps.py <FR05 directory>
"""
import sys
from pathlib import Path

from PIL import Image

SIZE = 600
RESOLUTION = 0.05
ORIGIN = (-15.0, -15.0)
FREE, OCCUPIED = 254, 0


def write_map(directory, occupied_cells=()):
    directory.mkdir(parents=True, exist_ok=True)
    image = Image.new("L", (SIZE, SIZE), FREE)
    pixels = image.load()
    for col, row in occupied_cells:
        pixels[col, row] = OCCUPIED
    image.save(directory / "map.pgm")
    (directory / "map.yaml").write_text(
        "image: map.pgm\n"
        "resolution: {res}\n"
        "origin:\n- {ox}\n- {oy}\n- 0.0\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n".format(
            res=RESOLUTION, ox=ORIGIN[0], oy=ORIGIN[1]),
        encoding="utf-8")


def cell(x):
    return int(round((x - ORIGIN[0]) / RESOLUTION))


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else
                Path(__file__).resolve().parents[1])
    write_map(root / "empty_map")
    wall = [(cell(12.5), row) for row in range(SIZE)]
    write_map(root / "blocked_map", wall)
    print("wrote", root / "empty_map", root / "blocked_map")


if __name__ == "__main__":
    main()
