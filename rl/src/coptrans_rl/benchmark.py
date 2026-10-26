"""Read the frozen engineering tasks with the ROS map server's semantics."""
from dataclasses import asdict
import hashlib
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

from .kernel_adapter import PROJECT_ROOT, Grid, Pose, stable_hash

DEFAULT_SCALES = {1: .75, 3: .7, 4: .5, 5: .7, 7: .8, 8: 1.2, 15: .8}
BASELINES = {"legacy": "a492235", "v1": "a415043", "v2": "c90e981"}


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_map_task(directory, contract, scale=1., **kwargs):
    directory = Path(directory).resolve()
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("invalid_map_scale")
    config = yaml.safe_load((directory/"map.yaml").read_text())
    origin = config["origin"]
    if int(config.get("negate", 0)) != 0 or abs(float(origin[2])) > 1e-12:
        raise ValueError("unsupported_negate_or_rotated_map_origin")
    image_path = directory/config["image"]
    with Image.open(image_path) as image:
        pixels = np.flipud(np.asarray(image.convert("L"))).copy()
    resolution = float(config["resolution"])
    free_threshold = float(config.get("free_thresh", .196))
    occupied_threshold = float(config.get("occupied_thresh", .65))
    if (not math.isfinite(resolution) or resolution <= 0 or
            not 0 <= free_threshold < occupied_threshold <= 1):
        raise ValueError("invalid_map_resolution_or_thresholds")
    h, w = pixels.shape
    cx, cy = float(origin[0])+w*resolution/2, float(origin[1])+h*resolution/2
    # Match map_publisher.load_map: unknown is occupied in the planner.
    occupied = pixels <= 255.-255.*occupied_threshold
    free = pixels >= 255.-255.*free_threshold
    occupied = occupied | (~occupied & ~free)
    grid = Grid(occupied, resolution*scale,
                cx-w*resolution*scale/2, cy-h*resolution*scale/2,
                contract.length, contract.width, contract.margin)
    metadata = yaml.safe_load((directory/"scene_meta.yaml").read_text())

    def pose(key):
        p = metadata[key]
        return Pose(cx+scale*(float(p["x"])-cx),
                    cy+scale*(float(p["y"])-cy), float(p["theta"]))

    start, goal = pose("start"), pose("goal")
    problem = contract.problem(grid, start, goal, task_id=directory.name, **kwargs)
    inputs = {"map_yaml_sha256": sha256_file(directory/"map.yaml"),
              "image_sha256": sha256_file(image_path),
              "scene_meta_sha256": sha256_file(directory/"scene_meta.yaml")}
    effective = {"occupied_sha256": hashlib.sha256(occupied.tobytes()).hexdigest(),
                 "shape": [h, w], "resolution": grid.res,
                 "origin": [grid.ox, grid.oy], "start": asdict(start),
                 "goal": asdict(goal)}
    try:
        display_path = str(directory.relative_to(PROJECT_ROOT))
    except ValueError:
        display_path = str(directory)
    entry = {"task_id": directory.name, "directory": display_path,
             "scale": scale, "source": inputs, "effective": effective,
             "effective_sha256": stable_hash(effective),
             "contract_sha256": contract.identity,
             "allow_reverse": problem.allow_reverse, "allow_shunting": problem.allow_shunting,
             "initial_mode": problem.initial_mode, "xy_tol": problem.xy_tol,
             "yaw_tol": problem.yaw_tol,
             "start_footprint_valid": grid.pose_clear(start),
             "goal_footprint_valid": grid.pose_clear(goal)}
    return problem, entry


def engineering_tasks(contract):
    for line in (PROJECT_ROOT/"maps/map_index.tsv").read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        directory, legacy_name, category = line.split("\t")
        number = int(directory[3:5])
        problem, entry = load_map_task(PROJECT_ROOT/"maps"/directory, contract,
                                       DEFAULT_SCALES.get(number, 1.))
        entry.update(legacy_name=legacy_name, category=category)
        yield problem, entry
