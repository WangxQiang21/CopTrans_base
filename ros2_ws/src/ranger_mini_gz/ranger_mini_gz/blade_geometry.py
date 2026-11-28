"""Rigid blade and semantic road geometry for the two-Ranger sites."""

from dataclasses import dataclass
import json
import math

import numpy as np
import shapely as sg
from shapely.affinity import affine_transform, rotate
from shapely.geometry import shape

from ranger_mini_gz.scene_geometry import interpolate, positive, vector


def arc(center, radius, start, sweep, spacing=.05):
    angle = np.linspace(start, start + sweep, math.ceil(radius * abs(sweep) / spacing) + 1)
    return np.column_stack((center[0] + radius * np.cos(angle), center[1] + radius * np.sin(angle),
                            angle + math.copysign(math.pi / 2, sweep)))


def primitives(rows):
    parts = []
    for row in rows:
        if row["type"] == "line":
            a, b = np.array([row["start"], row["end"]], dtype=float)
            yaw = math.atan2(*(b - a)[::-1])
            part = np.array([(*a, yaw), (*b, yaw)])
        else:
            part = arc(row["center"], row["radius"], math.radians(row["start_angle_deg"]),
                       math.radians(row["sweep_deg"]))
        if parts and (np.linalg.norm(parts[-1][-1, :2] - part[0, :2]) > 1e-7 or
                      abs(math.remainder(parts[-1][-1, 2] - part[0, 2], math.tau)) > 1e-7):
            raise ValueError("discontinuous mountain centreline")
        parts.append(part)
    return parts


@dataclass
class BladeSite:
    config: dict
    road: object
    centreline: object = None
    island: object = None

    def occupied(self, size, resolution):
        # A free cell must fit in the road even at its most distant corner.
        inside = self.road.buffer(-resolution / math.sqrt(2), quad_segs=16)
        sg.prepare(inside)
        nx, ny = (round(s / resolution) for s in size)
        x = (np.arange(nx) + .5) * resolution
        y = (np.arange(ny) + .5) * resolution
        return ~sg.contains_xy(inside, x[None, :], y[:, None])


def make_site(raw, directory):
    road = raw["road"]
    additions = [*(sg.box(*r) for r in road.get("extra_rectangles", [])),
                 *(sg.Polygon(points) for points in road.get("extra_polygons", []))]
    if "centerline_primitives" in road:
        parts = primitives(road["centerline_primitives"])
        center = sg.LineString(np.vstack(parts)[:, :2])
        pavement = center.buffer(road["width_m"] / 2, cap_style="flat", quad_segs=64)
        return BladeSite(raw, sg.union_all([pavement, *additions]) if additions else pavement, center)
    if "rectangles" in road:
        return BladeSite(raw, sg.union_all([*(sg.box(*r) for r in road["rectangles"]), *additions]))
    if "geometry_file" not in road:
        center = sg.Point(road["center"])
        island = center.buffer(road["island_radius"], quad_segs=512)
        circle = center.buffer(road["outer_radius"], quad_segs=512)
        flare = sg.Polygon(road["west_flare_vertices"])
        flares = [rotate(flare, angle, origin=road["center"]) for angle in (0, 90, 180, 270)]
        pavement = sg.union_all([circle, *flares, *(sg.box(*r) for r in road["approach_rectangles"]), *additions])
        return BladeSite(raw, pavement.difference(island), island=island)
    # The original roundabout feature preserves the approaches and their flares.
    features = json.loads((directory / road["geometry_file"]).read_text())["features"]
    feature, = (f for f in features if f["properties"].get("role") == "drivable_area")
    pavement = shape(feature["geometry"])
    if feature["properties"]["name"] != raw["name"] or not pavement.is_valid:
        raise ValueError("invalid V1 road feature")
    island = sg.Polygon(pavement.interiors[0]) if "island_radius" in road else None
    return BladeSite(raw, pavement, island=island)


def load_pair(raw, sim, common, resolution):
    from ranger_mini_gz.scene_generator import Cargo, Scene, map_resolution, read_yaml
    payload = read_yaml(sim / "config/blade_payload.yaml")
    dimensions = read_yaml(common / "config/ranger_mini_2.yaml")["vehicle"]
    body = tuple(float(dimensions[k]) for k in ("length", "width", "height"))
    stations = np.array(payload["stations_from_root"], dtype=float)
    spacing = payload["support_distance"]
    span = payload["length"]
    if (not np.all(np.diff(stations[:, 0]) > 0) or stations[0, 0] != 0 or stations[-1, 0] != span or
            not math.isclose(max(stations[:, 1] - stations[:, 2]), payload["max_chord"]) or
            not math.isclose(payload["root_overhang"] + spacing + payload["tip_overhang"], span)):
        raise ValueError("inconsistent blade stations or overhangs")
    stations[:, 0] -= payload["root_overhang"] + spacing / 2
    outline = np.vstack((stations[:, :2], stations[::-1, (0, 2)]))
    offsets = tuple(vector(row, 2, "mount") for row in payload["offsets"])
    if offsets != ((-spacing / 2, 0.), (spacing / 2, 0.)):
        raise ValueError("blade origin must be the midpoint between the two supports")
    lo, hi = outline.min(axis=0), outline.max(axis=0)
    cargo = Cargo(payload["name"], (*map(float, hi - lo), payload["thickness"]),
                  tuple(map(float, (lo + hi) / 2)), payload["bottom"], payload["bottom"] - body[2],
                  payload["mount_diameter"], 1., tuple(map(tuple, outline.tolist())))
    size = positive(vector(raw["size"], 2, "size"), "size")
    poses = ("start", "goal", "key_pose", "second_bend") if "second_bend" in raw else ("start", "goal", "key_pose")
    experiment = {key: vector(raw[key], 3, key) for key in poses}
    experiment["ordered_gates"] = raw["road"].get("ordered_midpoint_gates", [])
    scene = Scene(raw["name"], "default", size, map_resolution(size, resolution or raw["resolution"]),
                  map_resolution(size, raw["comparison_resolution"]), (.19, .20, .20),
                  tuple(raw["camera"]["eye"]), tuple(raw["camera"]["target"]), (), {"blade_pair": experiment},
                  body, tuple(payload["vehicle_ids"]), offsets, "blade_pair", cargo, None,
                  initial_body_yaw_offsets=(0., 0.), blade_site=make_site(raw, sim / "scenarios"))
    for key in poses:
        if not scene.blade_site.road.covers(world_footprint(local_footprint(scene), experiment[key])):
            raise ValueError(f"{scene.name}: complete rigid footprint is outside the road at {key}")
    return scene


def local_footprint(scene):
    # Circumscribe the physical disk, including the chord error of its polygon.
    radius = math.hypot(*scene.vehicle_size[:2]) / (2 * math.cos(math.pi / 128))
    return sg.union_all([sg.Polygon(scene.cargo.outline),
                         *(sg.Point(p).buffer(radius, quad_segs=32) for p in scene.offsets)])


def world_footprint(body, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return affine_transform(body, (c, -s, s, c, x, y))


def street_witness(scene):
    experiment = scene.experiments["blade_pair"]
    waypoints = [experiment[p] if isinstance(p, str) else vector(p, 3, "reference")
                 for p in scene.blade_site.config["geometric_reference"]]
    road = scene.blade_site.road
    body = local_footprint(scene)
    step, turn = (.02, .003) if scene.blade_site.config.get("tight_bends") else (.05, .01)
    samples = [world_footprint(body, q) for q in interpolate(waypoints, max_translation=step, max_yaw=turn)]
    if not np.all(sg.covers(road, samples)):
        raise ValueError("geometric reference crosses the road boundary")
    gap = float(np.min(sg.distance(road.boundary, samples)))
    radius = max(np.linalg.norm(scene.cargo.outline, axis=1))
    # Nearest sample is at most half a step away in translation and rotation.
    lower = gap - (step + turn*radius) / 2 - math.sqrt(2) * scene.resolution
    if lower <= 0:
        raise ValueError("geometric reference fails the clearance bound")
    return dict(kind="rigid_geometry_witness", samples=len(samples), invalid_samples=0,
                sampled_min_clearance_m=gap, conservative_lower_bound_m=float(lower))


def mountain_witness(scene):
    """Evaluate the V2 design motion against the same road used by the SDF and raster."""
    rows = scene.blade_site.config["road"]["centerline_primitives"]
    parts = primitives(rows)
    turn = rows[4]
    hairpin = arc(turn["center"], turn["radius"] - 1, -math.pi / 2, math.pi)
    exp = scene.experiments["blade_pair"]

    def line(a, b):
        return np.linspace(a, b, max(2, math.ceil(math.dist(a[:2], b[:2]) / .05) + 1))

    route = np.vstack([line(exp["start"], parts[1][0]), parts[1], parts[2],
                       line(parts[2][-1], (90, 30, 0)), line((90, 30, 0), (97, 31, 0)),
                       line((97, 31, 0), hairpin[0]), hairpin, line(hairpin[-1], (97, 47, math.pi)),
                       line((97, 47, math.pi), (84, 48, math.pi)), line((84, 48, math.pi), exp["goal"])])
    road = scene.blade_site.road
    sg.prepare(road)
    body = local_footprint(scene)
    polygons = [world_footprint(body, q) for q in route]
    invalid = int(np.count_nonzero(~sg.covers(road, polygons)))
    boundary = np.asarray(road.exterior.coords)
    index = sg.STRtree(sg.linestrings(np.stack((boundary[:-1], boundary[1:]), axis=1)))
    _, gaps = index.query_nearest(polygons, return_distance=True, all_matches=False)
    translation = np.linalg.norm(np.diff(route[:, :2], axis=0), axis=1).max()
    rotation = np.abs(np.diff(route[:, 2])).max()
    radial_extent = max(np.linalg.norm(scene.cargo.outline, axis=1))
    motion = .5 * (translation + radial_extent * rotation)
    minimum = float(min(gaps))
    lower = minimum - motion - math.sqrt(2) * scene.resolution - .001
    if invalid or lower < .25:
        raise ValueError("the constructed mountain witness fails the nominal clearance")
    return dict(kind="rigid_geometry_witness", samples=len(route), invalid_samples=invalid,
                sampled_min_clearance_m=minimum, inter_sample_motion_bound_m=float(motion),
                conservative_lower_bound_m=float(lower), centerline_length_m=scene.blade_site.centreline.length)
