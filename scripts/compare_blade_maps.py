"""Compare V1 reference maps with exported blade scenes. Coordinates use PGM top-left."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

BASELINE_SCENES = ("blade_right_angle_01", "blade_roundabout_01")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compare(original, candidate, stem):
    files = {}
    for ext in ("pgm", "yaml"):
        name = f"{stem}.{ext}"
        files[ext] = {"v1": digest(original / name), "v2": digest(candidate / name)}
    metadata = [yaml.safe_load((p / f"{stem}.yaml").read_text()) for p in (original, candidate)]
    images = []
    for directory, meta in zip((original, candidate), metadata):
        with Image.open(directory / meta["image"]) as image:
            images.append(np.array(image))
    a, b = images
    result = dict(sha256=files, v1_metadata=metadata[0], v2_metadata=metadata[1],
                  v1_size_px=list(a.shape[::-1]), v2_size_px=list(b.shape[::-1]))
    if a.shape != b.shape:
        return dict(result, verdict="尺寸不同", differing_cells=None)
    changed = a != b
    n = int(np.count_nonzero(changed))
    result["differing_cells"] = n
    if n:
        # Inclusive runs locate every difference without dumping entire images.
        runs = []
        for row in np.flatnonzero(changed.any(axis=1)):
            cols = np.flatnonzero(np.diff(np.r_[False, changed[row], False]))
            runs += [[int(row), int(start), int(end - 1)] for start, end in cols.reshape(-1, 2)]
        result["pgm_row_col_start_col_end"] = runs
    if n or metadata[0] != metadata[1]:
        result["verdict"] = "占用像素或地图元数据不同"
    elif all(pair["v1"] == pair["v2"] for pair in files.values()):
        result["verdict"] = "字节一致"
    else:
        result["verdict"] = "字节不同、地图等价"
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path, help="directory containing SHA256SUMS.txt")
    parser.add_argument("candidate", type=Path, help="generated scenes root")
    parser.add_argument("--scene", choices=BASELINE_SCENES, action="append",
                        help="compare only this scene (repeatable)")
    args = parser.parse_args()
    results = {scene: {stem: compare(args.baseline / scene, args.candidate / scene, stem)
                       for stem in ("map", "map_0025")}
               for scene in (args.scene or BASELINE_SCENES)}
    print(json.dumps(results, ensure_ascii=False, indent=2))
    return int(any(r["verdict"] not in ("字节一致", "字节不同、地图等价")
                   for scene in results.values() for r in scene.values()))


if __name__ == "__main__":
    raise SystemExit(main())
