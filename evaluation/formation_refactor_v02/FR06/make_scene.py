#!/usr/bin/env python3
"""Freeze the FR06 narrow-corner maps and their metadata.

Run from the worktree root or this directory:

    OPENBLAS_NUM_THREADS=1 python3 \
        evaluation/formation_refactor_v02/FR06/make_scene.py
"""
from datetime import datetime, timezone
import json
import platform
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))

import fr06_scene as scene  # noqa: E402

WIDTHS = {
    "wide": scene.WIDE_WIDTH_M,
    "tight": scene.TIGHT_WIDTH_M,
    "exploratory": scene.EXPLORATORY_WIDTH_M,
}


def main():
    maps = []
    for label, width in WIDTHS.items():
        directory = HERE / "maps" / "corridor_w{:.1f}".format(width)
        entry = scene.write_map_dir(width, directory)
        entry["label"] = label
        maps.append(entry)
    payload = {
        "schema": scene.SCHEMA,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "map_size_m": scene.MAP_SIZE_M,
        "resolution_m": scene.RESOLUTION_M,
        "cells": [scene.CELLS, scene.CELLS],
        "origin": list(scene.ORIGIN),
        "centreline": {
            "approach": [list(scene.APPROACH_START),
                         list(scene.ARC_START)],
            "arc": {"centre": list(scene.ARC_CENTRE),
                    "radius_m": scene.ARC_RADIUS_M,
                    "from_rad": -1.5707963267948966, "to_rad": 0.0},
            "exit": [list(scene.ARC_END), list(scene.EXIT_END)],
        },
        "start_qf": list(scene.START_QF),
        "goal_qf": list(scene.GOAL_QF),
        "start_headings": {"1": 0.0, "2": 0.0, "3": 0.0},
        "reference_is_not_a_search_seed": True,
        "maps": maps,
        "source_sha256": {
            "fr06_scene.py": scene.sha256_file(HERE / "fr06_scene.py"),
            "make_scene.py": scene.sha256_file(Path(__file__).resolve()),
        },
        "host": {
            "platform": platform.platform(),
            "machine": platform.machine(),
            "processor": platform.processor(),
            "python": platform.python_version(),
        },
    }
    (HERE / "scene_frozen.json").write_text(
        json.dumps(payload, sort_keys=True, indent=2, allow_nan=False),
        encoding="utf-8")
    for entry in maps:
        widths = entry["free_widths"]
        print("{label:11s} {width:.1f} m  free={free:5d} occ={occ:5d} "
              "sections={h:.2f}/{v:.2f}/{a:.2f} m".format(
                  label=entry["label"], width=entry["width_m"],
                  free=widths["free_cells"], occ=widths["occupied_cells"],
                  h=widths["horizontal_mid_section_free_m"],
                  v=widths["vertical_mid_section_free_m"],
                  a=widths["arc_radial_free_m"]))
    print("wrote", HERE / "scene_frozen.json")


if __name__ == "__main__":
    main()