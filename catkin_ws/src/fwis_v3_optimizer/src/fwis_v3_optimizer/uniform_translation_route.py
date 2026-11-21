"""Full-map routing in the conservative raster for a fixed formation pose."""

from dataclasses import dataclass
import heapq
import math
import time

from .core import Grid
from .formation_continuous_a.route_processing import segment_is_free


@dataclass(frozen=True)
class FixedHeadingRoute:
    status: str
    expansions: int = 0
    points: tuple = ()


def _axis_coordinates(origin, resolution, size, stride, first, last):
    # Include the requested endpoints exactly, avoiding short artificial
    # legs from a request pose to its nearby grid centre.
    coords = {first, last}
    for index in range(stride // 2, size, stride):
        value = origin + (index + 0.5) * resolution
        if min(abs(value - first), abs(value - last)) >= 0.5 * resolution:
            coords.add(value)
    return sorted(coords)


def fixed_heading_route(grid, blocked, start, goal, allowed_steps, *,
                        grid_step_m, turn_cost_m, deadline, max_expansions,
                        frame_angle_rad=0.0, cancelled=None):
    """Route across the map, preferring long straight runs between turns.

    This searches a rectilinear lattice of F positions. The direction in the
    key tracks corner cost and each jump checks all intervening raster cells.
    The result proposes waypoints for the original primitive factories.
    """
    right = grid.ox + grid.width * grid.res
    top = grid.oy + grid.height * grid.res
    for x, y in (start, goal):
        if not (grid.ox <= x < right and grid.oy <= y < top):
            return FixedHeadingRoute("ENDPOINT_OUTSIDE_GRID")
        if blocked[math.floor((y - grid.oy) / grid.res),
                   math.floor((x - grid.ox) / grid.res)]:
            return FixedHeadingRoute("ENDPOINT_BLOCKED_IN_RASTER")
    stride = max(1, int(grid_step_m / grid.res))
    if frame_angle_rad == 0.0:
        local_start, local_goal = start, goal
        xs = _axis_coordinates(grid.ox, grid.res, grid.width, stride, start[0], goal[0])
        ys = _axis_coordinates(grid.oy, grid.res, grid.height, stride, start[1], goal[1])
        columns = [math.floor((x - grid.ox) / grid.res) for x in xs]
        rows = [math.floor((y - grid.oy) / grid.res) for y in ys]

        def world_point(row, col):
            return xs[col], ys[row]

        def neighbour_clear(row, col, nr, nc):
            if nc != col:
                lo, hi = sorted((columns[col], columns[nc]))
                return not blocked[rows[row], lo:hi + 1].any()
            lo, hi = sorted((rows[row], rows[nr]))
            return not blocked[lo:hi + 1, columns[col]].any()
    else:
        cosine, sine = math.cos(frame_angle_rad), math.sin(frame_angle_rad)

        def local(x, y):
            dx, dy = x - start[0], y - start[1]
            return cosine * dx + sine * dy, -sine * dx + cosine * dy

        u, v = zip(*(local(x, y) for x in (grid.ox, right) for y in (grid.oy, top)))
        local_start = (0.0, 0.0)
        local_goal = tuple(0.0 if abs(value) < 1e-12 else value for value in local(*goal))
        xs = _axis_coordinates(min(u), grid.res, math.ceil((max(u) - min(u)) / grid.res),
                               stride, local_start[0], local_goal[0])
        ys = _axis_coordinates(min(v), grid.res, math.ceil((max(v) - min(v)) / grid.res),
                               stride, local_start[1], local_goal[1])
        raster = Grid(blocked, grid.res, grid.ox, grid.oy)
        edge_clear = {}

        def world_point(row, col):
            return (start[0] + cosine * xs[col] - sine * ys[row],
                    start[1] + sine * xs[col] + cosine * ys[row])

        def neighbour_clear(row, col, nr, nc):
            # A rotated segment is checked against the original raster cells
            # it touches. Rotating the raster itself could lose thin walls.
            edge = tuple(sorted(((row, col), (nr, nc))))
            if edge not in edge_clear:
                edge_clear[edge] = segment_is_free(raster, world_point(row, col), world_point(nr, nc))
            return edge_clear[edge]

    start_cell = (ys.index(local_start[1]), xs.index(local_start[0]), None)
    goal_cell = (ys.index(local_goal[1]), xs.index(local_goal[0]))

    best = {start_cell: 0.0}
    parent = {start_cell: None}
    opened = [(0.0, 0, 0.0, start_cell)]
    expansions = serial = 0
    while opened:
        if cancelled is not None and cancelled():
            return FixedHeadingRoute("REQUEST_CANCELLED", expansions)
        if time.perf_counter() >= deadline:
            return FixedHeadingRoute("TIME_BUDGET", expansions)
        _, _, distance, cell = heapq.heappop(opened)
        if distance != best[cell]:
            continue
        row, col, incoming = cell
        if (row, col) == goal_cell:
            points = [goal]
            previous_direction = incoming
            while cell != start_cell:
                row, col, direction = cell
                if direction != previous_direction:
                    points.append(world_point(row, col))
                previous_direction = direction
                cell = parent[cell]
            points.append(start)
            return FixedHeadingRoute("FOUND", expansions, tuple(reversed(points)))
        if expansions >= max_expansions:
            return FixedHeadingRoute("EXPANSION_BUDGET", expansions)
        expansions += 1
        for direction in allowed_steps:
            dy, dx = direction
            nr, nc = row + dy, col + dx
            if not (0 <= nr < len(ys) and 0 <= nc < len(xs)):
                continue
            if not neighbour_clear(row, col, nr, nc):
                continue
            step = abs(xs[nc] - xs[col]) if dx else abs(ys[nr] - ys[row])
            cost = distance + step
            if incoming is not None and incoming != direction:
                cost += turn_cost_m
            neighbour = (nr, nc, direction)
            if cost >= best.get(neighbour, math.inf):
                continue
            parent[neighbour] = cell
            best[neighbour] = cost
            remaining = abs(local_goal[0] - xs[nc]) + abs(local_goal[1] - ys[nr])
            serial += 1
            heapq.heappush(opened, (cost + remaining, serial, cost, neighbour))
    return FixedHeadingRoute("NO_ROUTE_IN_RASTER", expansions)
