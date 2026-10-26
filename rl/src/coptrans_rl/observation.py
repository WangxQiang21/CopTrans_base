"""Metric local crops, padded global map, and explicit path-history features."""
from collections import deque
import math

import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates

from .kernel_adapter import wrap


def goal_distance_field(grid, goal):
    """Four-connected point-robot guidance only; never a collision certificate."""
    distances = np.full(grid.occ.shape, np.inf, dtype=np.float64)
    ix, iy = int((goal.x-grid.ox)//grid.res), int((goal.y-grid.oy)//grid.res)
    if not (0 <= ix < grid.width and 0 <= iy < grid.height) or grid.occ[iy, ix]:
        return distances
    distances[iy, ix] = 0.
    queue = deque([(iy, ix)])
    while queue:
        y, x = queue.popleft()
        value = distances[y, x]+grid.res
        for ny, nx in ((y-1, x), (y+1, x), (y, x-1), (y, x+1)):
            if (0 <= ny < grid.height and 0 <= nx < grid.width and
                    not grid.occ[ny, nx] and not np.isfinite(distances[ny, nx])):
                distances[ny, nx] = value
                queue.append((ny, nx))
    return distances


class ObservationBuilder:
    state_dim = 60
    state_schema = "relative_goal_vehicle_lengths_v2"

    def __init__(self, problem, image_size=64):
        self.problem, self.size = problem, image_size
        self.grid = problem.grid
        self.length = problem.length
        self.distance = goal_distance_field(self.grid, problem.goal)
        self.distance_cap = 100.*self.length
        self.distance_image = np.minimum(
            self.distance, self.distance_cap)/self.distance_cap
        padded = np.pad(self.grid.occ, 1, constant_values=True)
        self.clearance = distance_transform_edt(~padded)[1:-1, 1:-1]*self.grid.res
        self.visits = np.zeros_like(self.distance, dtype=np.float32)
        self.record(problem.start)

    def record(self, pose):
        x = int(math.floor((pose.x-self.grid.ox)/self.grid.res))
        y = int(math.floor((pose.y-self.grid.oy)/self.grid.res))
        if 0 <= y < self.grid.height and 0 <= x < self.grid.width:
            self.visits[y, x] += 1

    def potential(self, pose):
        x = int(math.floor((pose.x-self.grid.ox)/self.grid.res))
        y = int(math.floor((pose.y-self.grid.oy)/self.grid.res))
        if 0 <= y < self.grid.height and 0 <= x < self.grid.width:
            return -float(self.distance_image[y, x])
        return -1.

    @staticmethod
    def _read(array, coords, fill, order=0):
        return map_coordinates(np.asarray(array, dtype=float), coords,
                               order=order, mode="constant", cval=fill,
                               prefilter=False).astype(np.float32)

    def encode(self, pose, prefix, steps, max_steps, proxy_elapsed, action_mask):
        grid, size, length = self.grid, self.size, self.length
        ys, xs = np.meshgrid(
            np.linspace(-4*length, 4*length, size),
            np.linspace(-4*length, 4*length, size), indexing="ij")
        c, s = math.cos(pose.th), math.sin(pose.th)
        local_coords = [(pose.y+s*xs+c*ys-grid.oy)/grid.res-.5,
                        (pose.x+c*xs-s*ys-grid.ox)/grid.res-.5]
        local = np.stack([
            self._read(grid.occ, local_coords, 1.),
            np.minimum(self._read(self.clearance, local_coords, 0., 1)/(4*length), 1.),
            self._read(self.distance_image, local_coords, 1., 1),
            np.minimum(self._read(self.visits, local_coords, 0.)/8., 1.)])
        extent = max(grid.height, grid.width)
        axis = np.linspace(0., extent-1., size)
        gy, gx = np.meshgrid(axis-(extent-grid.height)/2,
                             axis-(extent-grid.width)/2, indexing="ij")
        goal_x = (self.problem.goal.x-grid.ox)/grid.res-.5
        goal_y = (self.problem.goal.y-grid.oy)/grid.res-.5
        pose_x, pose_y = (pose.x-grid.ox)/grid.res-.5, (pose.y-grid.oy)/grid.res-.5
        radius = max(1., extent/size)
        global_map = np.stack([
            self._read(grid.occ, [gy, gx], 1.),
            self._read(self.distance_image, [gy, gx], 1., 1),
            (((gx-goal_x)**2+(gy-goal_y)**2) <= radius**2).astype(np.float32),
            (((gx-pose_x)**2+(gy-pose_y)**2) <= radius**2).astype(np.float32),
            np.minimum(self._read(self.visits, [gy, gx], 0.)/8., 1.)])
        previous, reversals, spin = 0, 0, 0.
        for a in prefix:
            if a.mode == 2:
                spin += a.amount
            else:
                direction = 1 if a.amount >= 0 else -1
                reversals += int(previous != 0 and previous != direction)
                previous, spin = direction, 0.
        dx, dy = self.problem.goal.x-pose.x, self.problem.goal.y-pose.y
        mode = prefix[-1].mode if prefix else self.problem.initial_mode
        last_control = prefix[-1].control if prefix else 0.
        state = [
            (c*dx+s*dy)/length, (-s*dx+c*dy)/length,
            math.sin(self.problem.goal.th-pose.th), math.cos(self.problem.goal.th-pose.th),
            *[float(mode == m) for m in range(3)],
            previous, reversals, wrap(spin)/math.pi,
            max(0., 1.-steps/max_steps), proxy_elapsed/300.,
            (pose.x-grid.ox)/(extent*grid.res),
            (pose.y-grid.oy)/(extent*grid.res), math.sin(pose.th), math.cos(pose.th),
            length, self.problem.width/length, self.problem.limits.r_min_ack/length,
            last_control]
        history = []
        for a in prefix[-8:]:
            amount = a.amount/(math.pi if a.mode == 2 else length)
            control = a.control*(self.problem.limits.r_min_ack if a.mode == 0 else
                                 2/math.pi if a.mode == 1 else 0.)
            history.extend([float(a.mode == m) for m in range(3)]+[amount, control])
        state.extend([0.]*(40-len(history))+history)
        result = {"local_map": local, "global_map": global_map,
                  "state": np.asarray(state, dtype=np.float32),
                  "valid_action_mask": np.asarray(action_mask, dtype=np.float32)}
        if result["state"].shape != (self.state_dim,):
            raise ValueError("state_schema_mismatch")
        if not all(np.all(np.isfinite(v)) for v in result.values()):
            raise ValueError("nonfinite_observation")
        return result
