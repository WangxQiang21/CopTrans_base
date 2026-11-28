"""Generate a Gazebo world and ROS static map from one scene description."""

import argparse
from dataclasses import dataclass, replace
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np
import yaml

from ranger_mini_gz.scene_geometry import (
    Obstacle, clearance, interpolate, local_pose, positive, rasterize, rectangle, transform, vector,
)
from ranger_mini_gz.scene_world import STYLE_COLORS, write_gui_config, write_world


SCENARIOS = ("factory_indoor_01", "port_01", "factory_outdoor_01", "v3_multibend", "v3_auxiliary", "paper_yard")
BLADE_SCENES = ("blade_mountain_01", "blade_right_angle_01", "blade_roundabout_01")
BLADE_HARD_SCENES = ("blade_mountain_hard", "blade_right_angle_hard", "blade_roundabout_hard")
ALL_BLADE_SCENES = (*BLADE_SCENES, *BLADE_HARD_SCENES)
SCENE_VARIANTS = ("auto", "default", "west_single_lane", "wide", "critical", "too_narrow", "cts_assembly", "paper_cts")
TASKS = ("single", "formation")
WALL_THICKNESS = .15


@dataclass(frozen=True)
class Cargo:
    name: str
    size: tuple
    offset: tuple
    bottom: float
    mount_height: float
    mount_diameter: float
    opacity: float
    outline: tuple = ()


@dataclass(frozen=True)
class Scene:
    name: str
    variant: str
    size: tuple
    resolution: float
    comparison_resolution: float | None
    ground_color: tuple
    camera_eye: tuple
    camera_target: tuple
    obstacles: tuple
    experiments: dict
    vehicle_size: tuple
    vehicle_ids: tuple
    offsets: tuple
    formation_id: str
    cargo: Cargo
    pinch_width: float | None
    initial_body_yaw_offsets: tuple = (0., 0., 0.)
    assembly_starts: tuple = ()
    blade_site: object = None

    def spawn_poses(self, task):
        start = self.experiments[task]["start"]
        if task == "single":
            return {1: start}
        return {
            i: (*local_pose(start, offset)[:2], start[2] + yaw_offset)
            for i, offset, yaw_offset in zip(
                self.vehicle_ids, self.offsets, self.initial_body_yaw_offsets, strict=True)
        }


def read_yaml(path):
    return yaml.safe_load(path.read_text())


def installed_paths():
    from ament_index_python.packages import get_package_share_directory
    return tuple(Path(get_package_share_directory(package)) for package in
                 ("ranger_mini_gz", "fwis_common", "fwis_backends"))


def map_resolution(size, value):
    resolution, = positive((float(value),), "resolution")
    if any(abs(side / resolution - round(side / resolution)) > 1e-8 for side in size):
        raise ValueError("map resolution must divide both scene dimensions")
    return resolution


def boundary_rows(size, height):
    w, h = size
    t = WALL_THICKNESS
    for name, x, y, dx, dy in (
            ("south", w / 2, t / 2, w, t), ("north", w / 2, h - t / 2, w, t),
            ("west", t / 2, h / 2, t, h - 2 * t), ("east", w - t / 2, h / 2, t, h - 2 * t)):
        yield {"id": f"boundary_{name}", "type": "wall", "pose": [x, y, height / 2, 0],
               "size": [dx, dy, height], "style": "boundary"}


def load_scene(name, variant=None, resolution=None, paths=None):
    if name not in (*SCENARIOS, *ALL_BLADE_SCENES):
        raise ValueError(f"unknown scene {name!r}")
    sim, common, backends = paths or installed_paths()
    raw = read_yaml(sim / "scenarios" / f"{name}.yaml")
    if raw["name"] != name:
        raise ValueError("scene name differs from its filename")
    if name in ALL_BLADE_SCENES:
        from ranger_mini_gz.blade_geometry import load_pair
        if variant not in (None, "auto", "default"):
            raise ValueError(f"variant {variant!r} is not available for {name}")
        return load_pair(raw, sim, common, resolution)
    size = positive(vector(raw["size"], 2, "scene size"), "scene size")
    primary = map_resolution(size, raw["resolution"] if resolution is None else resolution)
    comparison = raw.get("comparison_resolution")
    if comparison is not None:
        comparison = map_resolution(size, comparison)
    walls, = positive((float(raw["boundary_height"]),), "boundary height")

    if variant in (None, "auto"):
        variant = raw.get("default_variant", "default")
    obstacle_rows = raw["obstacles"]
    pinch = raw.get("pinch")
    variant_rows = raw.get("obstacle_variants", {})
    assembly_starts = ()
    if variant in variant_rows:
        extra = variant_rows[variant]
        if isinstance(extra, dict):
            assembly_starts = tuple(vector(pose, 3, "assembly start") for pose in extra["assembly_starts"])
            extra = [*extra["obstacles"], *extra.get("additional_obstacles", ())]
        obstacle_rows = [*obstacle_rows, *extra]
    elif variant != "default":
        cases = raw.get("width_cases", {})
        if variant not in cases or pinch is None:
            raise ValueError(f"variant {variant!r} is not available for {name}")
        cabinet = next(row for row in obstacle_rows if row["id"] == pinch["obstacle"])
        cabinet["pose"][1] = cases[variant]["pose_y"]
        cabinet["size"][1] = cases[variant]["depth"]
    obstacles = tuple(map(Obstacle.load, (*boundary_rows(size, walls), *obstacle_rows)))
    obstacles += tuple(Obstacle.load(row, decoration=True) for row in raw.get("decorations", ()))
    names = {obstacle.name for obstacle in obstacles}
    if len(names) != len(obstacles):
        raise ValueError("scene entity IDs must be unique")
    for obstacle in obstacles:
        if obstacle.style not in STYLE_COLORS:
            raise ValueError(f"{obstacle.name}: unknown visual style {obstacle.style}")
        if obstacle.style == "fence" and obstacle.kind not in ("box", "wall"):
            raise ValueError(f"{obstacle.name}: fence visuals require a rectangular obstacle")
        lo, hi = obstacle.bounds
        if obstacle.map_obstacle and (np.any(lo < -1e-8) or np.any(hi > np.asarray(size) + 1e-8)):
            raise ValueError(f"{obstacle.name}: collision footprint crosses the map boundary")

    vehicle = read_yaml(common / "config" / "ranger_mini_2.yaml")["vehicle"]
    vehicle_size = positive(tuple(float(vehicle[key]) for key in ("length", "width", "height")),
                            "Ranger Mini size")
    contract = read_yaml(backends / "config" / "multi_4wis" / "transport_formation.yaml")
    offsets = tuple(vector(v, 2, "formation offset") for v in contract["offsets"])
    ids = tuple(contract["vehicle_ids"])
    if ids != (1, 2, 3) or len(offsets) != len(ids) or contract["initial_heading_policy"] != "cargo_heading":
        raise ValueError("scene launch expects three Rangers with cargo-heading initialization")
    if assembly_starts and len(assembly_starts) != len(ids):
        raise ValueError("assembly starts must contain one pose per vehicle in vehicle ID order")
    if not np.allclose(vehicle_size[:2], [contract["body_length_m"], contract["body_width_m"]]):
        raise ValueError("formation and vehicle footprint configurations disagree")

    load = read_yaml(sim / "config" / "cargo_geometry.yaml")["cargo"]
    cargo_size = positive(vector(load["size"], 3, "cargo.size"), "cargo.size")
    cargo_offset = vector(load["center_offset"], 2, "cargo.center_offset")
    mount_height, mount_diameter = positive(
        (float(load["mount_height"]), float(load["mount_diameter"])), "cargo mount dimensions")
    opacity = float(load["opacity"])
    if not 0 < opacity <= 1:
        raise ValueError("cargo opacity must be in (0, 1]")
    if np.any(np.abs(np.asarray(offsets) - cargo_offset) + mount_diameter / 2 >
              np.asarray(cargo_size[:2]) / 2):
        raise ValueError("cargo does not overlap the full diameter of each centre mount")
    cargo = Cargo(load["id"], cargo_size, cargo_offset, vehicle_size[2] + mount_height,
                  mount_height, mount_diameter, opacity)
    if names.intersection({"ground", "route_paint", "start_paint", "goal_paint", cargo.name,
                           *(f"robot{i}" for i in ids)}):
        raise ValueError("an obstacle uses a reserved Gazebo model name")

    experiments = {}
    for task in TASKS:
        entry = raw["experiments"][task]
        experiments[task] = {
            key: vector(entry[key], 3, f"{task}.{key}") for key in ("start", "goal")
        }
        experiments[task]["waypoints"] = tuple(
            vector(waypoint, 3, f"{task}.waypoint") for waypoint in entry.get("waypoints", ()))
    by_name = {obstacle.name: obstacle for obstacle in obstacles}
    width = None if pinch is None else float(
        by_name[pinch["obstacle"]].bounds[0][1] - by_name[pinch["opposite"]].bounds[1][1])

    scene = Scene(name, variant, size, primary, comparison,
                  vector(raw["ground_color"], 3, "ground_color"),
                  vector(raw["camera"]["eye"], 3, "camera.eye"),
                  vector(raw["camera"]["target"], 3, "camera.target"),
                  obstacles, experiments, vehicle_size, ids, offsets,
                  contract["formation_id"], cargo, width,
                  vector(raw.get("initial_body_yaw_offsets", [0.] * len(ids)), len(ids),
                         "initial_body_yaw_offsets"), assembly_starts=assembly_starts)
    for task in TASKS:
        for key in ("start", "goal"):
            reference = experiments[task][key]
            vehicles = scene.spawn_poses(task).values() if task == "formation" and key == "start" else None
            for part, polygon in model_footprints(scene, task, reference, vehicles):
                distance, witness = clearance(polygon, scene.obstacles, scene.size)
                if distance <= 0:
                    raise ValueError(f"{name} {task} {key}: {part} collides with {witness}")
    for index, pose in enumerate(assembly_starts, 1):
        gap = clearance_summary(scene, "single", [(pose, None)])["robot1"]
        if gap["distance_m"] <= 0:
            raise ValueError(f"{name} assembly S{index} collides with {gap['obstacle']}")
    return scene


def with_trial_road(scene, road_path):
    """Use the certified road polygon for a blade experiment."""
    if road_path is None:
        return scene
    import shapely as sg

    road = sg.from_geojson(road_path.read_text())
    if (scene.blade_site is None or road.geom_type != "Polygon" or road.is_empty
            or not road.is_valid or not sg.box(0., 0., *scene.size).covers(road)):
        raise ValueError("invalid archived blade road")
    return replace(scene, blade_site=replace(scene.blade_site, road=road))


def model_footprints(scene, task, reference, vehicle_poses=None):
    ranger = rectangle(*scene.vehicle_size[:2])
    if task == "single":
        yield "robot1", transform(ranger, reference)
        return
    if vehicle_poses is None:
        vehicle_poses = [local_pose(reference, offset) for offset in scene.offsets]
    for index, body in zip(scene.vehicle_ids, vehicle_poses, strict=True):
        yield f"robot{index}", transform(ranger, body)
    if scene.cargo.outline:
        yield "cargo", transform(scene.cargo.outline, reference)
    else:
        yield "cargo", transform(rectangle(*scene.cargo.size[:2]), local_pose(reference, scene.cargo.offset))


def clearance_summary(scene, task, references):
    """Return sampled minimum clearance from each body to environmental geometry."""
    results = {}
    for reference, vehicles in references:
        for name, polygon in model_footprints(scene, task, reference, vehicles):
            if scene.blade_site is None:
                distance, obstacle = clearance(polygon, scene.obstacles, scene.size)
            else:
                from shapely import Polygon
                body = Polygon(polygon)
                road = scene.blade_site.road
                distance = road.boundary.distance(body) if road.covers(body) else 0.
                obstacle = "road_boundary"
            if name not in results or distance < results[name]["distance_m"]:
                results[name] = {"distance_m": distance, "obstacle": obstacle,
                                 "reference": [round(value, 5) for value in reference]}
    return results


def scene_audit(scene):
    report = {}
    if scene.blade_site is not None:
        from ranger_mini_gz.blade_geometry import local_footprint, mountain_witness, street_witness, world_footprint
        road = scene.blade_site.road
        envelope = local_footprint(scene)
        for key, pose in scene.experiments["blade_pair"].items():
            if key == "ordered_gates":
                continue
            footprint = world_footprint(envelope, pose)
            report[key] = dict(bodies=clearance_summary(scene, "blade_pair", [(pose, None)]),
                               arbitrary_chassis_yaw_gap_m=road.boundary.distance(footprint),
                               inside_road=bool(road.covers(footprint)))
        if scene.blade_site.centreline is not None and not scene.blade_site.config.get("tight_bends"):
            report["mountain_witness"] = mountain_witness(scene)
        if "geometric_reference" in scene.blade_site.config:
            report["street_witness"] = street_witness(scene)
        return report
    for task in TASKS:
        report[task] = {
            key: clearance_summary(scene, task, [(scene.experiments[task][key], None)])
            for key in ("start", "goal")
        }
    experiment = scene.experiments["formation"]
    route = (experiment["start"], *experiment["waypoints"], experiment["goal"])
    report["reference_route"] = {
        task: clearance_summary(scene, task, ((pose, None) for pose in interpolate(route)))
        for task in TASKS
    }
    report["reference_route"]["sampling"] = {
        "max_translation_m": .05, "max_rotation_rad": .01,
        "max_motion_from_nearest_sample_m": round(.025 + .005 * max(
            math.hypot(*scene.cargo.offset) + math.hypot(*scene.cargo.size[:2]) / 2,
            max(math.hypot(*offset) for offset in scene.offsets) +
            math.hypot(*scene.vehicle_size[:2]) / 2), 6),
    }
    return report


def write_map(scene, directory, stem, resolution):
    grid = (scene.blade_site.occupied(scene.size, resolution) if scene.blade_site is not None
            else rasterize(scene.obstacles, scene.size, resolution))
    height, width = grid.shape
    image = np.where(np.flipud(grid), 0, 254).astype(np.uint8)
    pgm = f"P5\n{width} {height}\n255\n".encode() + image.tobytes()
    (directory / f"{stem}.pgm").write_bytes(pgm)
    (directory / f"{stem}.yaml").write_text(yaml.safe_dump({
        "image": f"{stem}.pgm", "mode": "trinary", "resolution": resolution,
        "origin": [0.0, 0.0, 0.0], "negate": 0, "occupied_thresh": .65, "free_thresh": .196,
    }, sort_keys=False))
    return {"resolution": resolution, "width": width, "height": height,
            "occupied_cells": int(grid.sum()), "raster_tolerance_m": math.sqrt(2) * resolution}


def read_map(yaml_path):
    """Read the binary PGM produced by write_map in ROS bottom-row-first order."""
    metadata = read_yaml(yaml_path)
    magic, dimensions, maximum, pixels = (yaml_path.parent / metadata["image"]).read_bytes().split(b"\n", 3)
    width, height = (int(value) for value in dimensions.split())
    if magic != b"P5" or maximum != b"255" or len(pixels) != width * height:
        raise ValueError(f"map {yaml_path} is not a generated 8-bit PGM")
    raw = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width)
    return metadata, np.where(np.flipud(raw) == 0, 100, 0).astype(np.int8)


def generate_scene(scene, directory, task, gui_template, *, audit=True, spawn_poses=None, placement="start",
                   blade_plant=None, blade_body_yaws=None, blade_real_time_factor=1.):
    if task not in scene.experiments:
        raise ValueError(f"unsupported task {task!r}")
    if task != "blade_pair" and placement != "start":
        raise ValueError("static placement is only available for blade scenes")
    if placement not in scene.experiments[task]:
        raise ValueError(f"placement {placement!r} is not available for {scene.name}")
    if blade_plant is not None:
        if task != "blade_pair" or not Path(blade_plant).is_file() or not math.isfinite(blade_real_time_factor) or blade_real_time_factor <= 0.:
            raise ValueError("blade simulation requires a blade scene, plant configuration and positive real-time factor")
    directory.mkdir(parents=True, exist_ok=True)
    maps = {"map": write_map(scene, directory, "map", scene.resolution)}
    if scene.comparison_resolution is not None and scene.comparison_resolution != scene.resolution:
        maps["map_0025"] = write_map(scene, directory, "map_0025", scene.comparison_resolution)
    payload_pose = scene.experiments[task][placement]
    world_checks = write_world(scene, directory / "world.sdf", task, placement=payload_pose,
                               blade_plant=blade_plant, blade_body_yaws=blade_body_yaws,
                               blade_real_time_factor=blade_real_time_factor)
    if task == "formation":
        write_world(scene, directory / "world_single.sdf", "single")
    if task != "blade_pair":
        write_gui_config(scene, gui_template, directory / "gui.config")
    spawns = scene.spawn_poses(task) if spawn_poses is None else spawn_poses
    metadata = {"scenario": scene.name, "variant": scene.variant, "task": task, "frame": "map",
                "size_m": list(scene.size), "map_origin": [0., 0., 0.], "maps": maps,
                "pinch_width_m": None if scene.pinch_width is None else round(scene.pinch_width, 6),
                "spawn_poses": {f"robot{i}": list(pose) for i, pose in spawns.items()},
                "experiments": scene.experiments, "formation_offsets_m": scene.offsets,
                "cargo": {"id": scene.cargo.name, "size_m": scene.cargo.size, "offset_m": scene.cargo.offset,
                          "bottom_m": scene.cargo.bottom, "opacity": scene.cargo.opacity,
                          "mount_height_m": scene.cargo.mount_height,
                          "mount_diameter_m": scene.cargo.mount_diameter},
                "environment_obstacles": [obstacle.name for obstacle in scene.obstacles if obstacle.map_obstacle]}
    if scene.assembly_starts:
        metadata["assembly_starts"] = {f"robot{i}": list(pose) for i, pose in zip(
            scene.vehicle_ids, scene.assembly_starts, strict=True)}
    if task == "blade_pair":
        from ranger_mini_gz.blade_world import camera_views
        cameras = camera_views(scene, placement)
        for view, camera in cameras.items():
            filename = "gui.config" if view == "overview" else f"gui_{view}.config"
            write_gui_config(scene, gui_template, directory / filename, **camera)
        headings = [payload_pose[2]]*2 if blade_body_yaws is None else blade_body_yaws
        metadata.update(placement=placement, payload_pose=list(payload_pose), static_placement=blade_plant is None,
                        cameras=cameras,
                        environment_geometry=world_checks,
                        spawn_poses={f"robot{i}": [*local_pose(payload_pose, o)[:2], float(yaw)]
                                     for i, o, yaw in zip(scene.vehicle_ids, scene.offsets, headings, strict=True)})
        metadata["environment_obstacles"] = [*world_checks]
        if scene.blade_site.island is not None:
            metadata["environment_obstacles"].append("central_island")
        metadata["cargo"]["outline_xy_m"] = list(map(list, scene.cargo.outline))
    if audit:
        metadata["clearance"] = scene_audit(scene)
    (directory / "scene_meta.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))
    files = [*maps.keys(), "world.sdf", "gui.config", "scene_meta.yaml"]
    files = [item for name in files for item in ((name + ".pgm", name + ".yaml") if name in maps else (name,))]
    if task == "formation":
        files.append("world_single.sdf")
    if task == "blade_pair":
        files += ["gui_top.config", "gui_detail.config", "gui_supports.config",
                  *(p.relative_to(directory).as_posix() for p in sorted((directory / "assets").iterdir()) if p.is_file())]
    manifest = {name: hashlib.sha256((directory / name).read_bytes()).hexdigest() for name in files}
    (directory / "manifest.yaml").write_text(yaml.safe_dump({"schema": 1, "sha256": manifest}, sort_keys=False))
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=(*SCENARIOS, *ALL_BLADE_SCENES))
    parser.add_argument("output", type=Path)
    parser.add_argument("--variant", choices=SCENE_VARIANTS, default="auto")
    parser.add_argument("--resolution", type=float)
    parser.add_argument("--task", choices=(*TASKS, "blade_pair"))
    parser.add_argument("--placement", choices=("start", "goal", "key_pose", "second_bend"), default="start")
    parser.add_argument("--view", choices=("overview", "top", "detail", "supports"), default="overview")
    parser.add_argument("--scene-package-dir", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--gazebo", dest="gazebo", action="store_true")
    parser.add_argument("--generate-only", dest="gazebo", action="store_false")
    parser.set_defaults(gazebo=False)
    args = parser.parse_args()
    paths = installed_paths()
    if args.scene_package_dir is not None:
        paths = (args.scene_package_dir, *paths[1:])
    scene = load_scene(args.scenario, args.variant, args.resolution, paths)
    task = args.task or ("blade_pair" if scene.blade_site is not None else "formation")
    if task not in scene.experiments:
        parser.error(f"task {task!r} is not available for {args.scenario}")
    if args.placement not in scene.experiments[task]:
        parser.error(f"placement {args.placement!r} is not available for {args.scenario}")
    if args.view != "overview" and task != "blade_pair":
        parser.error("the selected view requires a blade scene")
    metadata = generate_scene(scene, args.output, task, paths[0] / "config" / "ranger.gui.config",
                              placement=args.placement)
    print(json.dumps({"scenario": metadata["scenario"], "variant": metadata["variant"],
                      "output": str(args.output.resolve()), "maps": metadata["maps"]}, ensure_ascii=False), flush=True)
    if args.gazebo:
        from ament_index_python.packages import get_package_share_directory
        directory = args.output.resolve()
        share = Path(get_package_share_directory("ranger_mini_description")).parent
        config = "gui.config" if args.view == "overview" else f"gui_{args.view}.config"
        resources = os.pathsep.join(filter(None, (str(directory), str(share), os.environ.get("GZ_SIM_RESOURCE_PATH"))))
        env = dict(os.environ, GZ_SIM_RESOURCE_PATH=resources)
        command = ["gz", "sim", "-r", "--gui-config", str(directory / config)]
        if task == "blade_pair":
            env.setdefault("GZ_PARTITION", f"coptrans_blade_{os.getpid()}")
            print("正在打开 Gazebo。首次加载车辆网格可能需要数秒。", flush=True)
        os.execvpe("gz", [*command, str(directory / "world.sdf")], env)


if __name__ == "__main__":
    main()
