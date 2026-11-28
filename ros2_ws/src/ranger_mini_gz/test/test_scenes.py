from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from ranger_mini_gz.scene_generator import (
    SCENARIOS, clearance_summary, generate_scene, installed_paths, load_scene, read_map, scene_audit,
)
from ranger_mini_gz.scene_geometry import Obstacle, interpolate, rasterize


def paths():
    return (Path(__file__).resolve().parents[1], *installed_paths()[1:])


def parse_collision(model):
    entity = model.find("link/collision")
    x, y, z, _roll, _pitch, yaw = map(float, model.findtext("pose").split())
    row = {"id": model.attrib["name"], "pose": [x, y, z, yaw]}
    geom = entity.find("geometry")[0]
    if geom.tag == "box":
        row["size"] = list(map(float, geom.findtext("size").split()))
    elif geom.tag == "cylinder":
        row.update(type="cylinder", radius=float(geom.findtext("radius")),
                   height=float(geom.findtext("length")))
    else:
        assert geom.tag == "polyline"
        row.update(type="polygon", height=float(geom.findtext("height")),
                   vertices=[list(map(float, p.text.split())) for p in geom.findall("point")])
        assert float(entity.findtext("pose").split()[2]) == pytest.approx(-row["height"] / 2)
    return Obstacle.load(row)


@pytest.mark.parametrize("name", SCENARIOS)
def test_sdf_collision_raster_and_visual_only_objects(name, tmp_path):
    locations = paths()
    scene = load_scene(name, paths=locations)
    manifest = generate_scene(scene, tmp_path, "formation", locations[0] / "config/ranger.gui.config",
                              audit=False)
    world = ET.parse(tmp_path / "world.sdf")
    entities = {entity.attrib["name"]: entity for entity in world.findall("world/model")}
    assert len(entities) == len(world.findall("world/model"))
    obstacles = []
    for obstacle in scene.obstacles:
        entity = entities[obstacle.name]
        assert (entity.find("link/collision") is not None) is obstacle.collision
        if obstacle.map_obstacle:
            footprint = parse_collision(entity)
            assert np.allclose(footprint.pose, obstacle.pose, atol=1e-10)
            obstacles.append(footprint)
    for stem, grid_metadata in manifest["maps"].items():
        map_yaml, actual = read_map(tmp_path / f"{stem}.yaml")
        reconstructed = rasterize(obstacles, scene.size, map_yaml["resolution"])
        assert np.array_equal(actual, np.where(reconstructed, 100, 0))
        assert actual.shape == (grid_metadata["height"], grid_metadata["width"])
        assert set(np.unique(actual)) == {0, 100}
        assert (actual[0, :] == 100).all() and (actual[:, -1] == 100).all()
    payload = entities[scene.cargo.name]
    assert tuple(map(float, payload.findtext("link/collision/geometry/box/size").split())) == scene.cargo.size
    assert len(payload.findall("link/visual/geometry/cylinder")) == 3
    assert scene.cargo.name not in manifest["environment_obstacles"]
    assert not ET.parse(tmp_path / "world_single.sdf").findall(
        f"world/model[@name='{scene.cargo.name}']")


def test_grid_origin_and_pgm_row_order(tmp_path):
    locations = paths()
    scene = load_scene("factory_indoor_01", paths=locations)
    generate_scene(scene, tmp_path, "single", locations[0] / "config/ranger.gui.config", audit=False)
    metadata, grid = read_map(tmp_path / "map.yaml")
    row, column = int(8.85 / .05), int(8.55 / .05)  # CNC 01, in the north row.
    assert metadata["origin"] == [0, 0, 0]
    assert grid[row, column] == 100 and grid[grid.shape[0] - 1 - row, column] == 0
    pixels = (tmp_path / "map.pgm").read_bytes().split(b"\n", 3)[-1]
    assert pixels[(grid.shape[0] - 1 - row) * grid.shape[1] + column] == 0
    assert pixels[row * grid.shape[1] + column] == 254


def test_default_factory_west_limit_is_single_car_only():
    locations = paths()
    scene = load_scene("factory_indoor_01", paths=locations)
    assert scene.variant == "west_single_lane"
    obstacles = {obstacle.name: obstacle for obstacle in scene.obstacles}
    barrier = obstacles["west_single_lane_barrier"]
    opening = barrier.bounds[0][0] - obstacles["boundary_west"].bounds[1][0]
    assert opening == pytest.approx(1.10)
    assert barrier.bounds[1][0] > obstacles["storage_west"].bounds[0][0]
    assert opening < min(scene.cargo.size[:2])
    straight = ((.70, 14.75, np.pi / 2), (.70, 17.25, np.pi / 2))
    route = clearance_summary(scene, "single", ((pose, None) for pose in interpolate(straight)))
    assert route["robot1"]["distance_m"] == pytest.approx(.30)

    legacy = load_scene(scene.name, "default", paths=locations)
    assert barrier.name not in {obstacle.name for obstacle in legacy.obstacles}


@pytest.mark.parametrize("variant,width,minimum", [
    ("wide", 3.60, .25), ("default", 3.20, .25), ("critical", 3.00, .05),
    ("too_narrow", 2.60, 0.00),
])
def test_pinch_width_and_sampled_wedge_clearance(variant, width, minimum):
    scene = load_scene("factory_indoor_01", variant, paths=paths())
    report = scene_audit(scene)["reference_route"]
    assert scene.pinch_width == pytest.approx(width)
    assert report["formation"]["cargo"]["distance_m"] == pytest.approx(minimum)
    assert report["single"]["robot1"]["distance_m"] > .7
    if variant != "too_narrow":
        assert all(report["formation"][f"robot{i}"]["distance_m"] > 0 for i in (1, 2, 3))
    else:
        assert report["formation"]["cargo"]["obstacle"] == "cabinet_pinchoff"


def test_dual_blade_sweep_and_mounts_use_continuous_geometry():
    pytest.importorskip("shapely", minversion="2.1")
    from ranger_mini_gz.blade_certify import BladeCertifier

    scene = load_scene("blade_right_angle_01", paths=paths())
    experiment = scene.experiments["blade_pair"]
    waypoints = [experiment[p] if isinstance(p, str) else p
                 for p in scene.blade_site.config["geometric_reference"]]
    trajectory = np.array(tuple(interpolate(waypoints)))
    headings = np.repeat(trajectory[:, 2:], 2, axis=1)
    checker = BladeCertifier(scene)
    reference = checker.certify(trajectory, headings)
    assert reference["passed"]
    assert reference["swept_grid_clearance_lower_bound_m"] > .35

    endpoints = [0, len(trajectory) - 1]
    assert checker.check_poses(trajectory[endpoints], headings[endpoints]).all()
    shortcut = checker.certify(trajectory[endpoints], headings[endpoints])
    assert shortcut["reason"] == "swept_clearance_not_certified"

    centers = checker.mount_centers(trajectory)
    centers[len(centers) // 2, 1, 1] += .025
    measured = checker.certify(trajectory, headings, vehicle_centers=centers, mount_tolerance_m=.005)
    assert measured["reason"] == "mount_alignment"
    assert measured["max_mount_error_m"] == pytest.approx(.025)

    occupied = scene.blade_site.occupied(scene.size, scene.resolution)
    x, y, _ = trajectory[0]
    occupied[int(y / scene.resolution), int(x / scene.resolution)] = True
    raster_check = BladeCertifier(scene, occupied).certify(trajectory, headings)
    assert raster_check["reason"] == "grid_sample_clearance"
    assert raster_check["first_failure"]["body"] == "blade"


def test_tighter_blade_scenes_keep_endpoints_and_use_the_shared_rviz_layout(tmp_path):
    pytest.importorskip("shapely", minversion="2.1")
    from ament_index_python.packages import get_package_share_directory
    from ranger_mini_gz.rviz_layout import rviz_config
    from ranger_mini_gz.scene_generator import BLADE_HARD_SCENES, BLADE_SCENES, read_yaml

    for original, tightened in zip(BLADE_SCENES, BLADE_HARD_SCENES):
        baseline, narrow = (load_scene(name, paths=paths()) for name in (original, tightened))
        assert narrow.cargo.outline == baseline.cargo.outline
        assert narrow.offsets == baseline.offsets
        for pose in ("start", "goal"):
            assert narrow.experiments["blade_pair"][pose] == baseline.experiments["blade_pair"][pose]

    description = Path(get_package_share_directory("ranger_mini_description"))
    filename = rviz_config(description, tmp_path, narrow, "blade_pair", tracking_dashboard=True, experiment_id="test")
    layout = read_yaml(filename)
    assert any(panel["Class"] == "fwis_rviz_plugins/PlanningPanel" for panel in layout["Panels"])
    displays = layout["Visualization Manager"]["Displays"]
    models = [d["TF Prefix"] for d in displays if d["Class"] == "rviz_default_plugins/RobotModel"]
    assert models == ["robot1", "robot2"]
    dashboard, = (d for d in displays if d["Class"] == "fwis_rviz_plugins/TrackingHUD")
    assert dashboard["Vehicle Count"] == 2
    assert dashboard["Experiment ID"] == "test"
