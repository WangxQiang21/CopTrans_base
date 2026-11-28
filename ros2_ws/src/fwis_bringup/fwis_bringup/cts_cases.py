"""Geometry and case inputs shared by the CTS offline and simulation entrances."""

from math import radians

from fwis_bringup.experiment_log import read_config
from ranger_mini_gz.scene_geometry import local_pose


def load_cases(directory):
    history = read_config(directory / "cts_long_routes.yaml")
    map_fields = ("scenario", "scene_variant", "map_sha256")
    layout = {key: history[key] for key in map_fields}
    cases = {name: dict(case, **layout, suite="historical") for name, case in history["cases"].items()}
    for name, scene in read_config(directory / "cts_paper_routes.yaml").items():
        layout = {key: scene[key] for key in map_fields}
        for station, pose in scene["starts"].items():
            end = scene["end"]
            cases[f"{name}_{station}_E"] = dict(start=pose, goal=end, **layout, suite="paper")
            cases[f"{name}_E_{station}"] = dict(start=end, goal=pose, **layout, suite="paper")
    return cases


def case_poses(case):
    """Convert the user's geometric cargo centre poses to radians."""
    poses = {}
    for key in ("start", "goal"):
        x, y, yaw = case[key]
        poses[key] = (x, y, radians(yaw))
    return poses


def cargo_reference(center, offset):
    return local_pose(center, (-offset[0], -offset[1]))


def dock_assignment(raw):
    if raw == "auto":
        return None
    if sorted(raw.split(",")) != ["1", "2", "3"]:
        raise ValueError("dock assignment must be auto or a permutation such as 1,3,2")
    return [int(value) for value in raw.split(",")]
