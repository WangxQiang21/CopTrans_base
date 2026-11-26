"""Geometry and case inputs shared by the CTS offline and simulation entrances."""

from math import radians

from ranger_mini_gz.scene_geometry import local_pose


def case_poses(case):
    """Convert the user's geometric cargo centre poses to radians."""
    return {key: (x, y, radians(yaw)) for key, (x, y, yaw) in case.items()}


def cargo_reference(center, offset):
    return local_pose(center, (-offset[0], -offset[1]))


def dock_assignment(raw):
    if raw == "auto":
        return None
    if sorted(raw.split(",")) != ["1", "2", "3"]:
        raise ValueError("dock assignment must be auto or a permutation such as 1,3,2")
    return [int(value) for value in raw.split(",")]
