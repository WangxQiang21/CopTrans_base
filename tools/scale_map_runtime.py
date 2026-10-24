#!/usr/bin/env python3
"""Make a temporary, uniformly scaled copy of a ROS occupancy-map directory.

The raster stays unchanged. Scaling the map resolution and moving its origin
around the map center changes the map's metric size without resampling cells.
Scene metadata coordinates are transformed by the same world-coordinate map.
"""
from __future__ import annotations

import math
import shutil
import sys
from pathlib import Path

from PIL import Image
import yaml


def transform_metadata(value, key, center_x, center_y, scale):
    if isinstance(value, dict):
        return {
            child_key: transform_metadata(
                child_value, child_key, center_x, center_y, scale)
            for child_key, child_value in value.items()
        }
    if key in ("x", "y"):
        center = center_x if key == "x" else center_y
        if isinstance(value, (list, tuple)):
            return [center + scale * (float(item) - center)
                    for item in value]
        if isinstance(value, (int, float)):
            return center + scale * (float(value) - center)
    if key == "origin" and isinstance(value, (list, tuple)) and len(value) >= 2:
        result = list(value)
        result[0] = center_x + scale * (float(value[0]) - center_x)
        result[1] = center_y + scale * (float(value[1]) - center_y)
        return result
    if key in ("size_m", "size", "resolution", "border_thickness"):
        if isinstance(value, (list, tuple)):
            return [float(item) * scale for item in value]
        if isinstance(value, (int, float)):
            return float(value) * scale
    if isinstance(value, list):
        return [transform_metadata(item, "", center_x, center_y, scale)
                for item in value]
    return value


def scale_map(source: Path, destination: Path, scale: float) -> None:
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError("map scale must be a finite number greater than zero")
    source = source.resolve()
    destination = destination.resolve()
    if not (source / "map.yaml").is_file():
        raise FileNotFoundError(f"map.yaml not found under {source}")
    if destination.exists():
        raise FileExistsError(destination)
    shutil.copytree(source, destination)

    yaml_path = destination / "map.yaml"
    with yaml_path.open("r", encoding="utf-8") as stream:
        map_data = yaml.safe_load(stream)
    origin = list(map_data["origin"])
    image_path = destination / map_data["image"]
    with Image.open(image_path) as image:
        width, height = image.size
    resolution = float(map_data["resolution"])
    center_x = float(origin[0]) + width * resolution / 2.0
    center_y = float(origin[1]) + height * resolution / 2.0
    scaled_resolution = resolution * scale
    origin[0] = center_x - width * scaled_resolution / 2.0
    origin[1] = center_y - height * scaled_resolution / 2.0
    map_data["resolution"] = scaled_resolution
    map_data["origin"] = origin
    map_data["runtime_scale"] = scale
    with yaml_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(map_data, stream, sort_keys=False, allow_unicode=True)

    metadata_path = destination / "scene_meta.yaml"
    if metadata_path.is_file():
        with metadata_path.open("r", encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)
        metadata = transform_metadata(
            metadata, "", center_x, center_y, scale)
        with metadata_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(metadata, stream, sort_keys=False,
                           allow_unicode=True)


def main(argv: list[str]) -> int:
    if len(argv) != 4:
        print("usage: scale_map_runtime.py MAP_DIR DEST_DIR SCALE", file=sys.stderr)
        return 2
    try:
        scale_map(Path(argv[1]), Path(argv[2]), float(argv[3]))
    except Exception as exc:
        print(f"map scaling failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
