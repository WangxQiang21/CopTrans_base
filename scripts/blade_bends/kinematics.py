"""Signed two-pin Ackermann and optional lateral or stationary motion for offline studies."""

from dataclasses import dataclass
import math
from time import perf_counter

import casadi as ca
import numpy as np
from scipy.integrate import solve_ivp
from scipy.ndimage import distance_transform_edt, map_coordinates

from ranger_mini_gz.blade_planner import Limits


ACKERMANN, OBLIQUE, SPIN = range(3)


def _edge(spacing, separation):
    """Both pin velocities have the same projection along the rigid blade."""
    state = ca.SX.sym("state", 5)
    command = ca.SX.sym("command", 6)
    next_curvature = ca.SX.sym("next_curvature", 2)
    mode = ca.SX.sym("lateral_spin", 2)

    def derivative(q, fraction):
        yaw, first, second = q[2], q[3], q[4]
        root_speed = command[0] * ca.cos(second - yaw)
        tip_speed = command[0] * ca.cos(first - yaw)
        k = (1 - fraction) * command[1:3] + fraction * next_curvature
        return ca.vertcat(
            (root_speed * ca.cos(first) + tip_speed * ca.cos(second)) / 2,
            (root_speed * ca.sin(first) + tip_speed * ca.sin(second)) / 2,
            command[0] * ca.sin(second - first) / separation,
            root_speed * k[0], tip_speed * k[1])

    # Signed shooting amplitudes can be several metres even on a short XY
    # chord. Resolve the integration of both chassis headings before validating
    # against the separate DOP853 rollout to a 10 micrometre splice tolerance.
    count = max(24, 4 * math.ceil(spacing / .16))
    h = 1 / count
    q = state
    samples = []
    for i in range(count):
        t = i * h
        k1 = derivative(q, t)
        k2 = derivative(q + h * k1 / 2, t + h / 2)
        k3 = derivative(q + h * k2 / 2, t + h / 2)
        k4 = derivative(q + h * k3, t + h)
        q += h * (k1 + 2 * k2 + 2 * k3 + k4) / 6
        if (i + 1) % (count // 4) == 0:
            f = (i + 1) / count
            c, s = ca.cos(state[2]), ca.sin(state[2])
            lateral = ca.vertcat(c * command[0] - s * command[3],
                                  s * command[0] + c * command[3], 0, 0, 0)
            spin = ca.vertcat(0, 0, 0, command[4], command[5])
            samples.append((1 - mode[0] - mode[1]) * q
                           + mode[0] * (state + f * lateral)
                           + mode[1] * (state + f * spin))
    return ca.Function("dual_pin_motion", [state, command, next_curvature, mode], samples)


class DistanceGrid:
    def __init__(self, scene):
        self.resolution = scene.resolution
        self.occupied = scene.blade_site.occupied(scene.size, scene.resolution)
        field = (distance_transform_edt(~self.occupied) - distance_transform_edt(self.occupied)) * self.resolution
        dy, dx = np.gradient(field, self.resolution)
        self.fields = (field, dx, dy)

    def at(self, points):
        coords = np.moveaxis(points[..., ::-1] / self.resolution - .5, -1, 0)
        return tuple(map_coordinates(img, coords, order=1, mode="nearest") for img in self.fields)


def outline_groups(outline, length=1.35):
    """Keep a contact on each part of the blade, including its two end faces."""
    groups = []
    for start, end in zip(outline, np.roll(outline, -1, axis=0)):
        vertices = np.linspace(start, end, max(1, math.ceil(np.linalg.norm(end - start) / length)) + 1)
        for a, b in zip(vertices[:-1], vertices[1:]):
            groups.append(np.linspace(a, b, 21))
    return np.array(groups)


def rotate(angles, points):
    c, s = np.cos(angles), np.sin(angles)
    return np.stack((c * points[..., 0] - s * points[..., 1],
                     s * points[..., 0] + c * points[..., 1]), axis=-1)


@dataclass
class Candidate:
    state: np.ndarray
    control: np.ndarray
    final_curvature: np.ndarray
    mode: np.ndarray
    iterations: list
    wall_seconds: float


class SignedBendPlanner:
    """Shared multiple shooting problem. Mode masks and gear warm starts are explicit."""

    def __init__(self, scene, reference, boundary_curvature, spacing, grid):
        self.reference = reference
        self.curvature = boundary_curvature
        self.grid = grid
        self.margin = scene.blade_site.config["planning_clearance_m"]
        self.offsets = np.array(scene.offsets)
        self.separation = math.dist(*self.offsets)
        self.groups = outline_groups(np.array(scene.cargo.outline))
        l, w = np.array(scene.vehicle_size[:2]) / 2
        self.corners = np.array([[-l, -w], [-l, w], [l, -w], [l, w]])
        self.count = n = len(reference) - 1
        self.edge = _edge(spacing, self.separation).map(n)
        self.advance_bound = max(3.2, 3.2 * spacing)
        opt = self.opt = ca.Opti()
        q = self.q = opt.variable(5, n + 1)
        u = self.u = opt.variable(6, n)
        slack = self.slack = opt.variable()
        modes = self.modes = opt.parameter(2, n)
        self.gear_lower = opt.parameter(1, n)
        self.gear_upper = opt.parameter(1, n)
        self.trust = opt.parameter(5, n + 1)
        ref = reference.T
        opt.subject_to(q[:, 0] == ref[:, 0])
        opt.subject_to(q[:, -1] == ref[:, -1])
        opt.subject_to(u[1:3, 0] == boundary_curvature[0])
        opt.subject_to(opt.bounded(0, slack, 3))
        opt.subject_to(u[0, :] <= self.gear_upper)
        opt.subject_to(u[0, :] >= self.gear_lower)
        for i, limit in enumerate((self.advance_bound, 1 / Limits.min_radius, 1 / Limits.min_radius,
                                   2.5, math.pi, math.pi)):
            opt.subject_to(opt.bounded(-limit, u[i, :], limit))
        opt.subject_to(opt.bounded(-3., q[:2, :] - self.trust[:2, :], 3.))
        k_next = ca.horzcat(u[1:3, 1:], boundary_curvature[-1])
        quarters = self.edge(q[:, :-1], u, k_next, modes)
        opt.subject_to(q[:, 1:] == quarters[-1])
        probes = ca.horzcat(q, *quarters[:-1])
        features = len(self.groups) + 8
        self.planes = [opt.parameter(features, probes.shape[1]) for _ in range(5)]
        nx, ny, rhs, px, py = self.planes
        positions = []
        for j in range(features):
            center = probes[:2, :]
            yaw = probes[2, :]
            if j >= len(self.groups):
                body = (j - len(self.groups)) // 4
                c, s = ca.cos(yaw), ca.sin(yaw)
                mount = self.offsets[body]
                center += ca.vertcat(mount[0] * c - mount[1] * s, mount[0] * s + mount[1] * c)
                yaw = probes[3 + body, :]
            c, s = ca.cos(yaw), ca.sin(yaw)
            positions.append(center + ca.vertcat(c * px[j, :] - s * py[j, :], s * px[j, :] + c * py[j, :]))
        x = ca.vertcat(*[p[0, :] for p in positions])
        y = ca.vertcat(*[p[1, :] for p in positions])
        opt.subject_to(ca.vec(nx * x + ny * y + slack - rhs) >= 0)
        cost = 8000 * slack + 200 * slack**2 + .32 * ca.sumsqr(q[:2, :] - ref[:2, :])
        cost += .18 * ca.sumsqr(q[2:, :] - ref[2:, :]) + .5 * ca.sum2(ca.sqrt(u[0, :]**2 + .005))
        cost += .09 * (ca.sumsqr(u[1:3, :]) + ca.sumsqr(ca.diff(u[1:3, :], 1, 1)))
        cost += .6 * ca.sum2(ca.sqrt(u[3, :]**2 + .005)) + .9 * ca.sumsqr(u[4:, :]) + .025 * ca.sumsqr(u)
        opt.minimize(cost)
        opt.solver("ipopt", {"print_time": False}, {"print_level": 0, "sb": "yes", "max_iter": 200,
                   "tol": 1e-8, "acceptable_tol": 2e-7, "acceptable_iter": 3, "mu_strategy": "adaptive",
                   "max_cpu_time": 45.})

    def _cuts(self, poses, reserve):
        # Re-evaluate the nearest point in every short strip at every pass.
        # Dense continuous polygon collision checking is independent of this filter.
        world = rotate(poses[:, None, None, 2], self.groups) + poses[:, None, None, :2]
        distance = self.grid.at(world)[0]
        which = distance.argmin(axis=2)
        blade = self.groups[np.arange(len(self.groups))[None, :], which]
        points = [rotate(poses[:, None, 2], blade) + poses[:, None, :2]]
        for i, mount in enumerate(self.offsets):
            origin = rotate(poses[:, 2], mount) + poses[:, :2]
            points.append(origin[:, None, :] + rotate(poses[:, None, i + 3], self.corners))
        world = np.concatenate(points, axis=1)
        corners = np.broadcast_to(self.corners, (len(poses), 4, 2))
        local = np.concatenate((blade, corners, corners), axis=1)
        gap, gx, gy = self.grid.at(world)
        active = gap < 2.8
        gx, gy = np.where(active, gx, 0.), np.where(active, gy, 0.)
        rhs = np.where(active, gx * world[..., 0] + gy * world[..., 1] - gap +
                       self.margin + reserve + self.grid.resolution / math.sqrt(2), -1.)
        for par, val in zip(self.planes, (gx, gy, rhs, local[..., 0], local[..., 1])):
            self.opt.set_value(par, val.T)
        return float(gap.min())

    def _samples(self, q, u, mask):
        k_next = np.vstack((u[1:, 1:3], self.curvature[-1]))
        p = self.edge(q[:-1].T, u.T, k_next.T, mask)
        return np.vstack([q, *(np.array(v).T for v in p[:-1])]), np.max(np.abs(np.array(p[-1]).T - q[1:]))

    def solve(self, modes, forced_reverse=(), seed=None, reserve=.06, repeats=4, initial_pose=None, forward_trial=False):
        begin = perf_counter()
        opt, n = self.opt, self.count
        initial = self.reference if initial_pose is None else initial_pose
        q = initial.copy() if seed is None else seed.state.copy()
        u = np.zeros((n, 6)) if seed is None else seed.control.copy()
        if seed is None:
            axial = np.cos(q[:-1, 3:5] - q[:-1, 2:3])
            lengths = np.linalg.norm(np.diff(q[:, :2], axis=0), axis=1)
            u[:, 0] = lengths / np.maximum(.4, axial.prod(axis=1))
            u[:, 1:3] = np.clip(np.diff(q[:, 3:], axis=0) / np.maximum(.2, u[:, :1] * axial[:, ::-1]),
                                  -1 / Limits.min_radius, 1 / Limits.min_radius)
            o = modes == OBLIQUE
            delta = rotate(-q[:-1,2], np.diff(q[:,:2],axis=0))
            u[o,0],u[o,3] = delta[o,0],delta[o,1]
            u[modes==SPIN,4:6] = np.diff(q[:,3:5],axis=0)[modes==SPIN]
        mask = np.array([modes == OBLIQUE, modes == SPIN], dtype=float)
        upper = np.full(n, self.advance_bound)
        if forced_reverse:
            steps = list(forced_reverse)
            upper[steps] = -.20
            u[steps, 0] = -.55
        opt.set_value(self.gear_upper, upper[None, :])
        lower = np.where((modes == ACKERMANN) & forward_trial, 0., -self.advance_bound)
        opt.set_value(self.gear_lower, lower[None, :])
        opt.set_value(self.modes, mask)
        history = []
        for _ in range(repeats):
            probes, _ = self._samples(q, u, mask)
            gap = self._cuts(probes, reserve)
            opt.set_value(self.trust, q.T)
            opt.set_initial(self.q, q.T)
            opt.set_initial(self.u, u.T)
            opt.set_initial(self.slack, max(.005, self.margin - gap))
            try:
                result = opt.solve()
            except RuntimeError:
                result = opt.debug
            status = opt.stats()["return_status"]
            slack = float(result.value(self.slack))
            q, u = result.value(self.q).T, result.value(self.u).T
            if not np.isfinite(q).all() or not np.isfinite(u).all():
                raise ValueError("the nonlinear solver returned nonfinite output")
            probes, defect = self._samples(q, u, mask)
            gap = self._cuts(probes, reserve)
            history.append(dict(solver_status=status, slack_m=slack,
                                sample_distance_m=gap, shooting_defect=float(defect)))
            if slack < 1e-5 and defect < 1e-6 and gap >= self.margin + reserve:
                break
        return Candidate(q, u, self.curvature[-1], modes, history, perf_counter() - begin)


def integrate(candidate, separation, step=.010):
    """Independent high-order rollout, with chord error bounds for the sweep certificate."""
    states = [candidate.state[0]]
    error, modes, gears, edges = [], [], [], []
    residual = 0.
    next_k = np.vstack((candidate.control[1:, 1:3], candidate.final_curvature))
    for i, (cmd, k_next, mode) in enumerate(zip(candidate.control, next_k, candidate.mode)):
        start = states[-1]
        advance = cmd[0]
        distance = (abs(advance) if mode == ACKERMANN else math.hypot(advance, cmd[3])
                    if mode == OBLIQUE else max(abs(cmd[4:6])))
        n = max(2, math.ceil(distance / step))
        t = np.linspace(0., 1., n + 1)
        if mode == ACKERMANN:
            def ode(tau, q):
                psi, h1, h2 = q[2:]
                v1 = advance * math.cos(h2 - psi)
                v2 = advance * math.cos(h1 - psi)
                k = cmd[1:3] + tau * (k_next - cmd[1:3])
                return [(v1 * math.cos(h1) + v2 * math.cos(h2)) / 2,
                        (v1 * math.sin(h1) + v2 * math.sin(h2)) / 2,
                        advance * math.sin(h2 - h1) / separation, v1 * k[0], v2 * k[1]]
            solution = solve_ivp(ode, (0, 1), start, t_eval=t, method="DOP853", atol=2e-13, rtol=2e-12)
            if not solution.success:
                raise ValueError(solution.message)
            trajectory = solution.y.T
            curvature = max(np.max(abs(cmd[1:3])), np.max(abs(k_next)))
            # Bounds are w.r.t. normalized edge station; the final term
            # accounts for curvature changing along a shot.
            ds = abs(advance)
            d2 = ds**2 * np.array([2 * curvature + 1 / separation, 2 * curvature / separation,
                                   curvature * (curvature + 1 / separation), curvature * (curvature + 1 / separation)])
            d2[2:] += ds * abs(k_next - cmd[1:3])
            pad = d2 / (8 * n**2) + 1e-8
            axial = np.cos(trajectory[1:, [4, 3]] - trajectory[1:, 2:3])
            motion = advance * axial
            gear = np.where(abs(motion)>1e-6, np.sign(motion),0).astype(np.int8)
        else:
            delta = np.zeros(5)
            if mode == SPIN:
                delta[3:] = cmd[4:6]
            else:
                delta[:2] = rotate(start[2], np.array([advance, cmd[3]]))
            trajectory = start + t[:, None] * delta
            pad = np.zeros(4)
            gear = np.zeros((n, 2), dtype=np.int8)
        states.extend(trajectory[1:])
        error.extend([pad] * n)
        modes.extend([mode] * n)
        gears.extend(gear)
        edges.extend([i] * n)
        residual = max(residual, float(np.max(abs(trajectory[-1] - candidate.state[i + 1]))))
    return (np.asarray(states), np.asarray(error), np.array(modes), np.array(gears),
            np.array(edges), residual)
