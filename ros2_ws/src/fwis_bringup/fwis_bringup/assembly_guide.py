"""Clearance-guided rest-to-rest candidates for difficult docking approaches."""

from heapq import heappop, heappush
from itertools import islice, product
from math import atan2, cos, hypot, pi, remainder, sin, sqrt
from time import monotonic

import numpy as np
from scipy.ndimage import distance_transform_edt


class DockingGuide:
    """Use planar occupancy to propose straight segments and stopped turns."""

    def __init__(self, grid, shape):
        metadata, cells = grid
        self.resolution = float(np.float32(metadata["resolution"]))
        self.origin = np.asarray(metadata["origin"][:2])
        self.cells = np.asarray(cells) != 0
        self.bounds = self.origin + self.resolution * np.asarray(self.cells.shape[::-1])
        self.half_length, self.half_width = shape[0] / 2, shape[1] / 2
        self.radius = hypot(self.half_length, self.half_width)
        self.clearance = distance_transform_edt(np.pad(~self.cells, 1))[1:-1, 1:-1] * self.resolution
        self.stride = max(1, round(.10 / self.resolution))

    def _nearby(self, at, reach):
        lower = np.asarray(at) - reach
        upper = np.asarray(at) + reach
        if np.any(lower <= self.origin) or np.any(upper >= self.bounds):
            return None
        begin = np.floor((lower - self.origin) / self.resolution).astype(int)
        end = np.floor((upper - self.origin) / self.resolution).astype(int) + 1
        y, x = np.nonzero(self.cells[begin[1]:end[1], begin[0]:end[0]])
        return self.origin + (np.c_[x, y] + begin + .5) * self.resolution - at

    def spin_clear(self, at):
        points = self._nearby(at, self.radius)
        return points is not None and np.all(
            np.sum(np.maximum(0., np.abs(points) - .5 * self.resolution)**2, axis=1) > self.radius**2)

    def segment_clear(self, start, end):
        a, b = np.asarray(start), np.asarray(end)
        dx, dy = b - a
        distance = hypot(dx, dy)
        if distance < 1e-8:
            return True
        c, s = dx / distance, dy / distance
        extent_x, extent_y = abs(c), abs(s)
        length, width = self.half_length + .5 * distance, self.half_width
        bounds = np.array([extent_x * length + extent_y * width, extent_y * length + extent_x * width])
        points = self._nearby((a + b) / 2, bounds)
        if points is None:
            return False
        x, y = points.T
        half_cell = .5 * self.resolution
        projection = half_cell * (extent_x + extent_y)
        overlaps = ((np.abs(x) <= bounds[0] + half_cell) &
                    (np.abs(y) <= bounds[1] + half_cell) &
                    (np.abs(c * x + s * y) <= length + projection) &
                    (np.abs(-s * x + c * y) <= width + projection))
        return not np.any(overlaps)

    def anchors(self, pose):
        x, y, yaw = pose
        if self.spin_clear((x, y)):
            yield (x, y)
        # Near a boundary the body can be able to leave/enter straight, even
        # when rotating it around the endpoint would touch an obstacle.
        for distance in (.40, .8, 1.2, 1.8, 2.5, 3.2):
            for direction in (1, -1):
                at = (x + direction * distance * cos(yaw), y + direction * distance * sin(yaw))
                if self.spin_clear(at) and self.segment_clear((x, y), at):
                    yield at

    def _centre(self, index):
        return tuple(self.origin + (self.stride * np.asarray(index) + .5) * self.resolution)

    def _nearest(self, at, accessible):
        x, y = np.round(((at - self.origin) / self.resolution - .5) / self.stride).astype(int)
        candidates = []
        for iy in range(max(0, y - 3), min(accessible.shape[0], y + 4)):
            for ix in range(max(0, x - 3), min(accessible.shape[1], x + 4)):
                if accessible[iy, ix]:
                    centre = self._centre((ix, iy))
                    if self.spin_clear(centre) and self.segment_clear(at, centre):
                        candidates.append((hypot(*np.subtract(centre, at)), (ix, iy)))
        return min(candidates)[1] if candidates else None

    def _grid_route(self, start, goal, radius, deadline):
        clearance = self.clearance[::self.stride, ::self.stride]
        accessible = clearance >= radius
        source, target = self._nearest(start, accessible), self._nearest(goal, accessible)
        if source is None or target is None:
            return None
        shape = accessible.shape
        offsets = ((1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1))
        frontier = [(0., 0., source)]
        costs, parent = {source: 0.}, {}
        while frontier:
            if monotonic() >= deadline:
                return None
            _, cost, node = heappop(frontier)
            if cost != costs[node]:
                continue
            if node == target:
                points = [goal, self._centre(node)]
                while node in parent:
                    node = parent[node]
                    points.append(self._centre(node))
                return [start, *reversed(points)]
            x, y = node
            for dx, dy in offsets:
                nx, ny = neighbor = x + dx, y + dy
                if not (0 <= ny < shape[0] and 0 <= nx < shape[1] and accessible[ny, nx]):
                    continue
                if dx and dy and not (accessible[y, nx] and accessible[ny, x]):
                    continue
                distance = sqrt(2.) if dx and dy else 1.
                new_cost = cost + distance * (1. + .035 / clearance[ny, nx]**2)
                if new_cost < costs.get(neighbor, float("inf")):
                    costs[neighbor], parent[neighbor] = new_cost, node
                    heappush(frontier, (new_cost + hypot(nx - target[0], ny - target[1]), new_cost, neighbor))
        return None

    def _shortcut(self, route):
        result = [route[0]]
        eligible = [i for i, at in enumerate(route) if i and self.spin_clear(at)]
        last = 0
        while last < len(route) - 1:
            for following in reversed(eligible):
                if following <= last:
                    return None
                at = route[following]
                if self.segment_clear(result[-1], at):
                    if hypot(*np.subtract(result[-1], at)) > 1e-7:
                        result.append(at)
                    last = following
                    break
            else:
                return None
        return result

    def routes(self, start, goal, deadline):
        """Yield geometry candidates from widest to tightest clearance."""
        starts = tuple(islice(self.anchors(start), 4))
        goals = tuple(islice(self.anchors(goal), 4))
        for a, b in product(starts, goals):
            if self.segment_clear(a, b):
                yield a, b, [a, b]
        for radius in (self.radius + self.resolution / 2, self.radius * .88, self.radius * .73):
            for a, b in product(starts, goals):
                if monotonic() >= deadline:
                    return
                route = self._grid_route(a, b, radius, deadline)
                if route is None:
                    continue
                path = self._shortcut(route)
                if path:
                    yield a, b, path


def _phase(mode, kind, start, end, duration, *, gear=1, ramp=0.):
    pieces = [np.linspace(a, b, 6).tolist() for a, b in zip(start[:2], end[:2])]
    pieces.append([start[2]] * 6)
    return dict(kind=kind, mode=mode, duration=duration, pieces=[pieces],
                heading0=[start[2]] * 3, heading1=[end[2]] * 3, gear=[gear] * 3, cruise_ramp_fraction=ramp)


def _duration(magnitude, speed, acceleration, jerk, dwell, ramp=0.):
    if ramp:
        peak = 1 / (1 - ramp)
        velocity, accel, impulse = peak, 1.5 * peak / ramp, 6 * peak / ramp**2
    else:
        velocity, accel, impulse = 1.875, 10 / sqrt(3.), 60.
    return 1.025 * max(dwell, velocity * magnitude / speed, sqrt(accel * magnitude / acceleration),
                       (impulse * magnitude / jerk)**(1 / 3))


def docking_plan(start, goal, departure, arrival, path, limits):
    """Construct a candidate in the public native analytic plan format."""
    plan = []
    pose = tuple(start)
    previous = ("A", 1)
    min_dwell = limits["min_motion_duration"]
    spin_steer = atan2(limits["wheelbase"], limits["track"])
    prepare_time = limits["prepare_time"] + .01

    def prepare(mode, gear):
        nonlocal previous
        if previous == (mode, gear):
            return
        delta = spin_steer if previous[0] != mode else 0.
        travel = max(1.875 * delta / limits["steer_rate"],
                     sqrt(10 / sqrt(3.) * delta / limits["steer_acceleration"]))
        plan.append(_phase(mode, "prepare", pose, pose, prepare_time + travel, gear=gear))
        previous = mode, gear

    def spin(target):
        nonlocal pose
        turn = remainder(target - pose[2], 2 * pi)
        if abs(turn) < 1e-7:
            return
        target_pose = (*pose[:2], pose[2] + turn)
        prepare("S", 1)
        speed = min(limits["spin_speed"], 2 * limits["wheel_speed"] / hypot(limits["wheelbase"], limits["track"]))
        duration = _duration(abs(turn), speed, limits["spin_acceleration"], limits["spin_jerk"], min_dwell)
        plan.append(_phase("S", "motion", pose, target_pose, duration))
        pose = target_pose

    def straight(at, yaw=None):
        nonlocal pose, previous
        dx, dy = at[0] - pose[0], at[1] - pose[1]
        distance = hypot(dx, dy)
        if distance < 1e-7:
            return
        direction = atan2(dy, dx)
        if yaw is None:
            yaw = direction if abs(remainder(direction - pose[2], 2*pi)) < pi/2 else direction + pi
        spin(yaw)
        gear = 1 if cos(direction - pose[2]) > 0 else -1
        prepare("A", gear)
        target = (*at, pose[2])
        speed = min(limits["speed"], limits["wheel_speed"])
        if plan and plan[-1]["mode"] == "A" and plan[-1]["kind"] == "motion" and plan[-1]["gear"][0] == gear:
            # Contiguous straight legs on the same heading use one clock.
            old = plan.pop()
            pose = (old["pieces"][0][0][0], old["pieces"][0][1][0], pose[2])
            distance = hypot(target[0] - pose[0], target[1] - pose[1])
        bounds = (distance, speed, limits["acceleration"], limits["jerk"], min_dwell)
        duration, ramp = min((_duration(*bounds, ramp), ramp) for ramp in (0., .125))
        plan.append(_phase("A", "motion", pose, target, duration, gear=gear, ramp=ramp))
        pose = target
        previous = "A", gear

    straight(departure, start[2])
    for at in path[1:]:
        straight(at)
    if hypot(goal[0] - arrival[0], goal[1] - arrival[1]) > 1e-7:
        straight(goal[:2], goal[2])
    else:
        spin(goal[2])
    return plan
