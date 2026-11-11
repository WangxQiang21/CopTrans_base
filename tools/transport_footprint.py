#!/usr/bin/env python3
"""Vehicle-only footprint metrics for the authoritative wedge contract.

Offsets, ids and body dimensions are read from
``catkin_ws/src/fwis_v3_demo/config/transport_formation.yaml`` -- this tool
keeps no private copy of the formation.  Swept area depends on a path and on
the body-heading policy, so only static envelopes and clearly labelled scale
factor candidates are reported here.  Cargo dimensions have not been
specified and are not included.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))

from fwis_v3_optimizer.transport_contract import (  # noqa: E402
    contract_from_mapping)

DEFAULT_CONFIG = (ROOT / "catkin_ws/src/fwis_v3_demo/config/"
                  "transport_formation.yaml")


def load_contract_file(path=DEFAULT_CONFIG):
    text = Path(path).read_text(encoding="utf-8")
    values = yaml.safe_load(text)
    return (contract_from_mapping(values),
            hashlib.sha256(text.encode("utf-8")).hexdigest())


def _cross(origin, first, second):
    return ((first[0] - origin[0]) * (second[1] - origin[1])
            - (first[1] - origin[1]) * (second[0] - origin[0]))


def _convex_hull_area(points):
    ordered = sorted(set(points))
    if len(ordered) < 3:
        return 0.0
    lower = []
    for point in ordered:
        while len(lower) >= 2 and _cross(
                lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(ordered):
        while len(upper) >= 2 and _cross(
                upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    hull = lower[:-1] + upper[:-1]
    area = 0.0
    for index, point in enumerate(hull):
        following = hull[(index + 1) % len(hull)]
        area += point[0] * following[1] - following[0] * point[1]
    return abs(area) / 2.0


def footprint_metrics(config_path=DEFAULT_CONFIG):
    contract, digest = load_contract_file(config_path)
    length = contract.body_length_m
    width = contract.body_width_m
    offsets = dict(zip(contract.vehicle_ids, contract.offsets))
    ids = list(contract.vehicle_ids)
    single = length * width
    circumradius = math.hypot(length / 2.0, width / 2.0)

    pair_distances = {}
    for index, first in enumerate(ids):
        for second in ids[index + 1:]:
            dx = offsets[first][0] - offsets[second][0]
            dy = offsets[first][1] - offsets[second][1]
            pair_distances["{}-{}".format(first, second)] = math.hypot(dx, dy)
    minimum_pair = min(pair_distances.values())
    pairwise_separated = minimum_pair > 2.0 * circumradius

    corners = []
    for x, y in offsets.values():
        corners.extend((
            (x - length / 2.0, y - width / 2.0),
            (x + length / 2.0, y - width / 2.0),
            (x + length / 2.0, y + width / 2.0),
            (x - length / 2.0, y + width / 2.0),
        ))
    xs = [corner[0] for corner in corners]
    ys = [corner[1] for corner in corners]
    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)

    max_center_radius = max(
        math.hypot(x, y) for x, y in offsets.values())
    outer_radius = max_center_radius + circumradius

    return {
        "config_path": str(Path(config_path).resolve()),
        "config_sha256": digest,
        "formation_id": contract.formation_id,
        "vehicle_ids": ids,
        "offsets_m": [list(offsets[vehicle_id]) for vehicle_id in ids],
        "body_length_m": length,
        "body_width_m": width,
        "pair_distances_m": pair_distances,
        "minimum_pair_distance_m": minimum_pair,
        "bodies_pairwise_separated": pairwise_separated,
        "single_body_area_m2": single,
        "three_bodies_union_area_m2": (3.0 * single
                                       if pairwise_separated else None),
        "body_circumradius_m": circumradius,
        "common_heading_rad": 0.0,
        "common_heading_aabb_m": [span_x, span_y],
        "common_heading_aabb_area_m2": span_x * span_y,
        "common_heading_hull_area_m2": _convex_hull_area(corners),
        "max_center_radius_m": max_center_radius,
        "heading_free_outer_radius_m": outer_radius,
        "heading_free_envelope_m2": math.pi * outer_radius ** 2,
        "required_axis_scale_factor": max(span_x / length, span_y / width),
        "conservative_disk_scale_factor": max(
            2.0 * outer_radius / length, 2.0 * outer_radius / width),
        "cargo_geometry_included": False,
        "safety_margin_included": False,
        "notes": [
            "AABB and hull are for the common cargo-heading pose used by "
            "the contract's initial_heading_policy=cargo_heading.",
            "heading_free_envelope_m2 is a conservative circle that contains "
            "any body heading; it is not a measured swept area.",
            "The legacy run_demo factor 3.75 is a pending old-strategy value "
            "and is not asserted sufficient for this formation.",
        ],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--out")
    args = parser.parse_args()
    text = json.dumps(footprint_metrics(args.config), indent=2,
                      sort_keys=True)
    if args.out:
        Path(args.out).write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())