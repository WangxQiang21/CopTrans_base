"""Planar geometry shared by scenario collision checks and map rasterization."""

from dataclasses import dataclass
from functools import cached_property
import math
import re

import numpy as np


def vector(values, size, field):
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise ValueError(f"{field} needs {size} coordinates")
    result = tuple(float(v) for v in values)
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f"{field} contains a non-finite value")
    return result


def positive(values, field):
    if not all(math.isfinite(v) and v > 0 for v in values):
        raise ValueError(f"{field} must be finite and positive")
    return values


def rectangle(length, width):
    x, y = length / 2, width / 2
    return np.array([[-x, -y], [x, -y], [x, y], [-x, y]])


def transform(points, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + (x, y)


def local_pose(reference, offset):
    return *map(float, transform(offset, reference)), reference[2]


def normals(vertices):
    edges = np.roll(vertices, -1, axis=0) - vertices
    return np.column_stack((-edges[:, 1], edges[:, 0]))


def convex_polygon(vertices):
    # Require each other vertex to lie strictly on the same side of every edge.
    # This also rejects self-intersections, duplicate vertices and zero-area faces.
    edges = normals(vertices)
    signed = (vertices[None, :, :] - vertices[:, None, :]) * edges[:, None, :]
    signed = signed.sum(axis=2)
    inside = np.ones(signed.shape, dtype=bool)
    i = np.arange(len(vertices))
    inside[i, i] = inside[i, (i + 1) % len(vertices)] = False
    if np.all(signed[inside] > 1e-10):
        return vertices
    if np.all(signed[inside] < -1e-10):
        return vertices[::-1]
    raise ValueError("polygon must have distinct vertices in convex boundary order")


@dataclass(frozen=True)
class Obstacle:
    name: str
    kind: str
    pose: tuple
    size: tuple
    height: float
    radius: float
    vertices: tuple
    collision: bool
    map_obstacle: bool
    style: str
    color: tuple | None

    @classmethod
    def load(cls, row, decoration=False):
        name = row["id"]
        if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
            raise ValueError(f"invalid obstacle id {name!r}")
        kind = row.get("type", "box")
        pose = vector(row["pose"], 4, f"{name}.pose")
        size, radius, vertices = (), 0.0, ()
        if kind in ("box", "wall"):
            size = positive(vector(row["size"], 3, f"{name}.size"), f"{name}.size")
            height = size[2]
        elif kind == "cylinder":
            radius, height = positive(
                (float(row["radius"]), float(row["height"])), f"{name} radius and height")
        elif kind == "polygon":
            height, = positive((float(row["height"]),), f"{name}.height")
            vertices = tuple(vector(point, 2, f"{name}.vertices") for point in row["vertices"])
            if len(vertices) < 3:
                raise ValueError(f"{name} needs at least three polygon vertices")
            vertices = tuple(map(tuple, convex_polygon(np.array(vertices))))
        else:
            raise ValueError(f"{name}: unsupported geometry {kind!r}")
        collision = row.get("collision", not decoration)
        map_obstacle = row.get("map_obstacle", collision)
        if type(collision) is not bool or type(map_obstacle) is not bool or map_obstacle and not collision:
            raise ValueError(f"{name}: map obstacles must have physical collision")
        color = vector(row["color"], 3, f"{name}.color") if "color" in row else None
        if color is not None and not all(0 <= channel <= 1 for channel in color):
            raise ValueError(f"{name}: color components must be between zero and one")
        return cls(name, kind, pose, size, height, radius, vertices, collision, map_obstacle,
                   row.get("style", "plain"), color)

    @cached_property
    def footprint(self):
        if self.kind == "cylinder":
            return None
        local = self.vertices if self.kind == "polygon" else rectangle(*self.size[:2])
        return transform(local, (*self.pose[:2], self.pose[3]))

    @cached_property
    def bounds(self):
        if self.footprint is None:
            center = np.asarray(self.pose[:2])
            return center - self.radius, center + self.radius
        return self.footprint.min(axis=0), self.footprint.max(axis=0)

    @cached_property
    def axes(self):
        return normals(self.footprint)


def rasterize(obstacles, size, resolution):
    """Mark cells whose closed square intersects an obstacle's XY footprint."""
    width, height = (round(side / resolution) for side in size)
    grid = np.zeros((height, width), dtype=bool)
    half = resolution / 2
    for obstacle in obstacles:
        if not obstacle.map_obstacle:
            continue
        lo, hi = obstacle.bounds
        start = np.maximum(0, np.floor(lo / resolution - 1e-9).astype(int))
        stop = np.minimum((width, height), np.floor(hi / resolution + 1e-9).astype(int) + 1)
        ix, iy = start
        ex, ey = stop
        if ex <= ix or ey <= iy:
            continue
        x = (np.arange(ix, ex)[None, :] + .5) * resolution
        y = (np.arange(iy, ey)[:, None] + .5) * resolution
        if obstacle.footprint is None:
            dx = np.maximum(0.0, np.abs(x - obstacle.pose[0]) - half)
            dy = np.maximum(0.0, np.abs(y - obstacle.pose[1]) - half)
            hit = dx**2 + dy**2 <= obstacle.radius**2 + 1e-12
        else:
            hit = np.ones((ey - iy, ex - ix), dtype=bool)
            for axis in (*obstacle.axes, (1, 0), (0, 1)):
                px, py = axis
                bounds = obstacle.footprint @ axis
                projection = px * x + py * y
                support = half * (abs(px) + abs(py)) + 1e-10
                hit &= (projection >= bounds.min() - support) & (projection <= bounds.max() + support)
        grid[iy:ey, ix:ex] |= hit
    return grid


def point_segments_distance(points, polygon):
    edges = np.roll(polygon, -1, axis=0) - polygon
    relative = points[:, None, :] - polygon
    fractions = np.clip((relative * edges).sum(axis=2) / (edges**2).sum(axis=1), 0., 1.)
    return float(np.sqrt(((relative - fractions[..., None] * edges)**2).sum(axis=2).min()))


def xy_distance(body, body_axes, obstacle):
    if obstacle.footprint is None:
        center = np.asarray(obstacle.pose[:2])
        if np.all(((center - body) * body_axes).sum(axis=1) >= -1e-10):
            return 0.0
        return max(0.0, point_segments_distance(center[None, :], body) - obstacle.radius)
    return convex_distance(body, obstacle.footprint, body_axes, obstacle.axes)


def convex_distance(body, other, body_axes, other_axes):
    """Minimum distance between two convex polygon footprints."""
    axes = np.concatenate((body_axes, other_axes), axis=0)
    a, b = body @ axes.T, other @ axes.T
    if np.all((a.max(axis=0) >= b.min(axis=0) - 1e-10) &
              (b.max(axis=0) >= a.min(axis=0) - 1e-10)):
        return 0.0
    return min(point_segments_distance(body, other), point_segments_distance(other, body))


def clearance(body, obstacles, size):
    """Minimum XY distance to environmental collision geometry or the world boundary."""
    lo, hi = body.min(axis=0), body.max(axis=0)
    nearest = float(min(*lo, *(size - hi)))
    if nearest <= 0:
        return 0.0, "world_boundary"
    witness = "world_boundary"
    body_axes = normals(body)
    for obstacle in obstacles:
        if not obstacle.map_obstacle:
            continue
        lower, upper = obstacle.bounds
        gap = np.maximum(0, np.maximum(lower - hi, lo - upper))
        if gap @ gap > nearest**2:
            continue
        distance = xy_distance(body, body_axes, obstacle)
        if distance <= nearest:
            nearest, witness = distance, obstacle.name
        if nearest <= 0:
            break
    return nearest, witness


def interpolate(route, max_translation=.05, max_yaw=.01):
    yield route[0]
    for a, b in zip(route, route[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        d_yaw = math.remainder(b[2] - a[2], math.tau)
        steps = max(1, math.ceil(math.hypot(dx, dy) / max_translation),
                    math.ceil(abs(d_yaw) / max_yaw))
        for index in range(1, steps + 1):
            fraction = index / steps
            yield a[0] + fraction * dx, a[1] + fraction * dy, a[2] + fraction * d_yaw
