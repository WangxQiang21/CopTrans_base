"""Convex-region graph over one conservative fixed-heading formation raster.

Positive-width free faces connect adjacent rectangles. Candidate segments
can shortcut the portal polyline inside the union of the selected rectangles.
Exact A/O/S successors and independent audits still apply.
"""
from dataclasses import dataclass
import heapq
import math
import time

import numpy as np


@dataclass(frozen=True)
class CellRectangle:
    """Half-open bounds in occupancy-grid column/row indices."""
    left: int
    right: int
    bottom: int
    top: int

    @property
    def center(self):
        return ((self.left + self.right) / 2, (self.bottom + self.top) / 2)

    def contains(self, column, row):
        return self.left <= column < self.right and self.bottom <= row < self.top


@dataclass(frozen=True)
class ConvexCorridor:
    status: str
    expansions: int = 0
    points: tuple = ()
    rectangles: tuple = ()


def rectangle_graph(blocked, stopped):
    """Partition free cells into rectangles and return positive-width portals."""
    regions, neighbours, previous = [], [], []
    for row, occupied in enumerate(blocked):
        if stopped():
            return None, None
        edges = np.flatnonzero(np.diff(np.r_[False, ~occupied, False]))
        prior = {(left, right): index for left, right, index in previous}
        current = []
        for left, right in zip(edges[::2], edges[1::2]):
            left, right = int(left), int(right)
            index = prior.get((left, right))
            if index is None:
                index = len(regions)
                regions.append(CellRectangle(left, right, row, row + 1))
                neighbours.append([])
            else:
                region = regions[index]
                regions[index] = CellRectangle(left, right, region.bottom, row + 1)
            current.append((left, right, index))
        i = j = 0
        while i < len(previous) and j < len(current):
            lo, hi, first = previous[i]
            left, right, second = current[j]
            begin, end = max(lo, left), min(hi, right)
            if first != second and begin < end:
                neighbours[first].append((second, begin, end, row))
                neighbours[second].append((first, begin, end, row))
            if hi <= right:
                i += 1
            if right <= hi:
                j += 1
        previous = current
    return tuple(regions), neighbours


def _segment_interval(start, goal, rectangle):
    """Clip a segment to a closed rectangle in grid coordinates."""
    lower, upper = 0.0, 1.0
    bounds = ((rectangle.left, rectangle.right), (rectangle.bottom, rectangle.top))
    for first, second, (low, high) in zip(start, goal, bounds):
        step = second - first
        if abs(step) < 1e-12:
            if not low <= first <= high:
                return None
            continue
        enter, leave = sorted(((low - first) / step, (high - first) / step))
        lower, upper = max(lower, enter), min(upper, leave)
        if lower > upper:
            return None
    return lower, upper


def shortcut_corridor(grid, corridor, allowed, stopped):
    """Find a shorter direction-feasible polyline within the convex chain."""
    points = corridor.points
    coordinates = [((x - grid.ox) / grid.res, (y - grid.oy) / grid.res) for x, y in points]
    result = [points[0]] if points else []
    cursor = 0
    while cursor + 1 < len(points):
        candidate = cursor + 1
        for endpoint in range(len(points) - 1, candidate, -1):
            if stopped():
                return points
            if not allowed(points[cursor], points[endpoint]):
                continue
            coverage = 0.0
            intervals = sorted(value for rectangle in corridor.rectangles
                               if (value := _segment_interval(coordinates[cursor], coordinates[endpoint],
                                                              rectangle)) is not None)
            for lower, upper in intervals:
                if lower > coverage + 1e-12:
                    break
                coverage = max(coverage, upper)
            if coverage >= 1.0 - 1e-12:
                candidate = endpoint
                break
        result.append(points[candidate])
        cursor = candidate
    return tuple(result)


def convex_corridor(grid, blocked, start, goal, allowed_steps, *, deadline,
                    max_expansions, cancelled=None):
    """Find a connected rectangle chain and orthogonal waypoints in metres."""
    def stopped():
        return time.perf_counter() >= deadline or bool(cancelled and cancelled())

    regions, adjacency = rectangle_graph(blocked, stopped)
    if regions is None:
        return ConvexCorridor("CORRIDOR_BUDGET")

    def locate(point):
        column = math.floor((point[0] - grid.ox) / grid.res)
        row = math.floor((point[1] - grid.oy) / grid.res)
        return next((idx for idx, rect in enumerate(regions) if rect.contains(column, row)), None)

    source, target = locate(start), locate(goal)
    if source is None or target is None:
        return ConvexCorridor("CORRIDOR_ENDPOINT_BLOCKED")
    open_list = [(0.0, 0.0, source)]
    costs, parent = {source: 0.0}, {}
    expansions = 0
    while open_list:
        if stopped():
            return ConvexCorridor("CORRIDOR_BUDGET", expansions)
        _, travelled, index = heapq.heappop(open_list)
        if travelled != costs[index]:
            continue
        if index == target:
            break
        if expansions >= max_expansions:
            return ConvexCorridor("CORRIDOR_BUDGET", expansions)
        expansions += 1
        for neighbour, left, right, row in adjacency[index]:
            direction = 1 if regions[index].top == row else -1
            if (direction, 0) not in allowed_steps:
                continue
            distance = travelled + math.dist(regions[index].center, regions[neighbour].center)
            if distance < costs.get(neighbour, math.inf):
                costs[neighbour] = distance
                parent[neighbour] = index, left, right, row, direction
                remaining = math.dist(regions[neighbour].center, regions[target].center)
                heapq.heappush(open_list, (distance + remaining, distance, neighbour))
    else:
        return ConvexCorridor("CORRIDOR_DISCONNECTED", expansions)

    portals, chain = [], [target]
    while chain[-1] != source:
        index, left, right, row, direction = parent[chain[-1]]
        portals.append((left, right, row, direction))
        chain.append(index)

    points = [start]

    def direction(first, second):
        dx, dy = second[0] - first[0], second[1] - first[1]
        return (int(math.copysign(1, dy)) if abs(dy) > 1e-12 else 0,
                int(math.copysign(1, dx)) if abs(dx) > 1e-12 else 0)

    def append(point):
        step = direction(points[-1], point)
        if step == (0, 0):
            return True
        if step not in allowed_steps:
            return False
        if len(points) > 1 and step == direction(points[-2], points[-1]):
            points[-1] = point
        else:
            points.append(point)
        return True

    for left, right, row, vertical in reversed(portals):
        x = min(max(points[-1][0], grid.ox + (left + .5) * grid.res),
                grid.ox + (right - .5) * grid.res)
        y_enter = grid.oy + (row - .5 * vertical) * grid.res
        y_leave = grid.oy + (row + .5 * vertical) * grid.res
        if not (append((x, points[-1][1])) and append((x, y_enter)) and append((x, y_leave))):
            return ConvexCorridor("CORRIDOR_DIRECTION_CONFLICT", expansions)
    if not (append((goal[0], points[-1][1])) and append(goal)):
        return ConvexCorridor("CORRIDOR_DIRECTION_CONFLICT", expansions)
    return ConvexCorridor("FOUND", expansions, tuple(points),
                          tuple(regions[index] for index in reversed(chain)))
