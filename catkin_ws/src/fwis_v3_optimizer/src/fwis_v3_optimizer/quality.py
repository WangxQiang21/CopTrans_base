"""Independent trajectory-quality contract, separate from search heuristics."""
from dataclasses import asdict, dataclass
import math

import numpy as np
from scipy.ndimage import binary_erosion
from scipy.spatial import cKDTree

from .core import (Action, Grid, Pose, advance, collapse_exact_runs, replay, wrap,
                   footprint_corners)
from .continuous_geometry import ContinuousPlan, sample_block
from .retiming import wheel_configuration


def shunting_violation(motions):
    """One gear reversal requires a real intervening Spin; repeated reversals fail.

    motions yields (mode, signed translation direction, signed Spin angle).
    Initial/full reverse motion remains eligible.
    """
    previous = None
    spin = 0.
    reversals = 0
    for index,(mode,direction,angle) in enumerate(motions):
        if mode == 2:
            spin += angle
            continue
        if previous is not None and direction != previous:
            reversals += 1
            if reversals > 1:
                return {"reason":"repeated_translation_reversals", "index":index,
                        "reversals":reversals}
            if abs(wrap(spin)) <= 1e-4:
                return {"reason":"translation_reversal_without_spin", "index":index,
                        "reversals":reversals}
        previous,spin = direction,0.
    return None


def action_shunting_violation(actions):
    return shunting_violation((a.mode,1 if a.amount >= 0 else -1,
                              a.amount if a.mode == 2 else 0.) for a in actions)


def _quintic_time_values(tau):
    s = tau**3*(10.-15.*tau+6.*tau*tau)
    ds = 30.*tau*tau*(1.-tau)*(1.-tau)
    dds = 60.*tau*(1.-tau)*(1.-2.*tau)
    ddds = 60.-360.*tau+360.*tau*tau
    return s, ds, dds, ddds


def _independent_motion_peaks(block, stations, speed, accel, jerk, limits):
    """Differentiate each wheel velocity analytically, independently of retiming."""
    c, dc, ddc = block.control_many(stations)
    angles, ratios, steering_first, steering_second = [], [], [], []
    for x in (limits.wheelbase/2., -limits.wheelbase/2.):
        for y in (limits.track/2., -limits.track/2.):
            if block.mode == 0:
                vx, vy = 1.-y*c, x*c
                dx, dy, ddx, ddy = -y*dc, x*dc, -y*ddc, x*ddc
                norm = vx*vx+vy*vy
                first = (vx*dy-vy*dx)/norm
                second = ((vx*ddy-vy*ddx)/norm-
                          first*2.*(vx*dx+vy*dy)/norm)
                angle, ratio = np.arctan2(vy, vx), np.sqrt(norm)
            else:
                angle, ratio = c, np.ones_like(c)
                first, second = dc, ddc
            angles.append(angle)
            ratios.append(ratio*speed)
            steering_first.append(first*speed)
            steering_second.append(second*speed*speed+first*accel)
    zero = np.zeros_like(speed)
    measurements = {
        "center_speed": speed, "path_acceleration": accel, "path_jerk": jerk,
        "wheel_speed": ratios, "wheel_angle": angles,
        "steer_rate": steering_first, "steer_acceleration": steering_second,
        "angular_speed": c*speed if block.mode == 0 else zero,
        "angular_acceleration": c*accel+dc*speed**2 if block.mode == 0 else zero,
        "angular_jerk": c*jerk+3.*dc*speed*accel+ddc*speed**3 if block.mode == 0 else zero,
        "lateral_acceleration": c*speed**2 if block.mode == 0 else zero,
        "oblique_normal_acceleration": dc*speed**2 if block.mode == 1 else zero}
    return {name: float(np.max(np.abs(values))) for name, values in measurements.items()}


def _continuous_dynamics_peaks(block, duration, limits):
    """Recompute dynamic extrema from geometry and time, not the peak report."""
    if block.mode == 2:
        amount = abs(block.signed_angle)
        speed = 1.875*amount/duration
        angles, factors = wheel_configuration(Action(2, block.signed_angle, 0.), limits)
        return {"center_speed": 0., "path_acceleration": 0., "path_jerk": 0.,
                "wheel_speed": float(np.max(factors))*speed,
                "lateral_acceleration": 0., "angular_speed": speed,
                "angular_acceleration": 10/math.sqrt(3)*amount/duration**2,
                "angular_jerk": 60*amount/duration**3,
                "steer_rate": 0., "steer_acceleration": 0.,
                "oblique_normal_acceleration": 0.,
                "wheel_angle": float(np.max(np.abs(angles)))}
    tau = np.linspace(0., 1., 401)
    q, v, a, j = _quintic_time_values(tau)
    return _independent_motion_peaks(block, block.length*q,
        block.length*v/duration, block.length*a/duration**2,
        block.length*j/duration**3, limits)


def _piecewise_continuous_dynamics_peaks(block, phase, limits, table=None):
    """Reconstruct polynomial and directly audit the continuous steering curve."""
    duration = float(phase["duration"])
    distance = float(phase["s1"]-phase["s0"])
    c1t = float(phase["start_speed"])*duration
    c2t = .5*float(phase.get("start_acceleration", 0.))*duration**2
    a1t = float(phase.get("end_acceleration", 0.))*duration**2
    tail = np.linalg.solve(
        np.asarray([[1., 1., 1.], [3., 4., 5.], [6., 12., 20.]]),
        np.asarray([distance-c1t-c2t, phase["end_speed"]*duration-c1t-2*c2t, a1t-2*c2t]))
    coeff = np.asarray([0., c1t, c2t, tail[0], tail[1], tail[2]])
    if not np.allclose(coeff, phase.get("coefficients", []), rtol=0., atol=1e-9):
        raise ValueError("continuous_time_profile_coefficients_mismatch")
    spatial = np.concatenate([np.linspace(a.s, b.s, 9)
        for a, b in zip(block.knots, block.knots[1:])])
    boundaries = np.asarray([k.s for k in block.knots])
    spatial = np.unique(np.r_[spatial, np.nextafter(boundaries, -np.inf),
                             np.nextafter(boundaries, np.inf)])
    spatial = spatial[(spatial > phase["s0"])&(spatial < phase["s1"])]
    lower, upper = np.zeros(len(spatial)), np.ones(len(spatial))
    for _ in range(42):
        guess = .5*(lower+upper)
        goes_right = np.polynomial.polynomial.polyval(guess, coeff) < spatial-phase["s0"]
        lower = np.where(goes_right, guess, lower)
        upper = np.where(goes_right, upper, guess)
    tau = np.r_[np.linspace(0., 1., 161), .5*(lower+upper)]
    q = np.polynomial.polynomial.polyval(tau, coeff)
    v = np.polynomial.polynomial.polyval(tau, np.arange(1, 6)*coeff[1:])/duration
    a = np.polynomial.polynomial.polyval(tau, np.arange(2, 6)*np.arange(1, 5)*coeff[2:])/duration**2
    j = np.polynomial.polynomial.polyval(tau, np.arange(3, 6)*np.arange(2, 5)*np.arange(1, 4)*coeff[3:])/duration**3
    if np.min(q) < -1e-8 or np.max(q) > distance+1e-8 or np.min(v) < -1e-8:
        raise ValueError("continuous_nonmonotone_time_profile")
    stations = np.minimum(phase["s0"]+np.clip(q, 0., distance),
                          np.nextafter(phase["s1"], phase["s0"]))
    stations[161:] = spatial
    return _independent_motion_peaks(block, stations,
                                      v, a, j, limits)


@dataclass(frozen=True)
class QualityProfile:
    name: str = "quality_cost_v2_prototype_uncalibrated"
    clearance: float = 4.
    shape: float = 1.
    adjustment: float = .5
    time: float = 1.
    length: float = .25
    reverse: float = 2.
    direction_change: float = 1.5
    reverse_early: float = 0.
    reverse_start: float = 0.
    reverse_yaw_early: float = 0.
    # Fixed score units. Runtime target_v may be overridden without changing
    # the normalization scale used to compare trajectories.
    reference_speed_mps: float = 2.
    reference_spin_omega_radps: float = .8
    preferred_clearance_factor: float = .20
    metric_spacing: float = .01
    sweep_spacing: float = .005

    def __post_init__(self):
        for key, value in asdict(self).items():
            if key == "name":
                continue
            if not math.isfinite(value) or value < 0:
                raise ValueError("invalid_quality_profile:" + key)
        if min(self.reference_speed_mps, self.reference_spin_omega_radps,
               self.preferred_clearance_factor, self.metric_spacing,
               self.sweep_spacing) <= 0:
            raise ValueError("invalid_quality_resolution")


def _reverse_preferences(segments, start_window_m=.738):
    total = sum(row[1] for row in segments)
    traveled = early = early_yaw = 0.
    first_gear = None
    first_reverse = 1.
    forward_prefix = 0.
    found_reverse = False
    for i, (mode, length, direction, yaw, controls) in enumerate(segments):
        if mode == 2:
            following = next((row for row in segments[i+1:] if row[0] != 2), None)
            if following is not None and following[2] < 0:
                early_yaw += abs(yaw)*(1.-traveled/max(total, 1e-12))**2
            continue
        if first_gear is None:
            first_gear = direction
        if direction < 0:
            if not found_reverse:
                first_reverse = traveled/max(total, 1e-12)
                forward_prefix = traveled
                found_reverse = True
            a, b = traveled/max(total, 1e-12), (traveled+length)/max(total, 1e-12)
            early += total*((1.-a)**3-(1.-b)**3)/3.
            if mode == 0:
                station, curvature = controls
                weight = (1.-(traveled+station)/max(total, 1e-12))**2
                early_yaw += float(np.trapz(np.abs(curvature)*weight, station))
        traveled += length
    return {"reverse_early_length_m": early,
            "reverse_start": (math.exp(-forward_prefix/max(start_window_m, 1e-9))
                              if found_reverse else 0.),
            "forward_prefix_before_reverse_m": forward_prefix,
            "reverse_yaw_early_rad": early_yaw,
            "first_reverse_progress": first_reverse}


def reverse_preference_actions(actions, start_window_m=.738):
    return _reverse_preferences([(a.mode, abs(a.amount) if a.mode != 2 else 0.,
        1 if a.amount >= 0 else -1, a.amount if a.mode == 2 else 0.,
        (np.asarray([0., abs(a.amount)]), np.asarray([a.control, a.control])))
        for a in actions if abs(a.amount) > 1e-10], start_window_m)


def reverse_preference_plan(plan, start_window_m=.738):
    segments = []
    for block in plan.blocks:
        stations = np.linspace(0., block.length, max(2, int(math.ceil(block.length/.025))+1))
        controls = (stations, block.control_many(stations)[0]) if block.mode == 0 else None
        segments.append((block.mode, block.length, block.direction, block.signed_angle, controls))
    return _reverse_preferences(segments, start_window_m)


class FootprintDistance:
    """Exact OBB-to-grid-cell Euclidean distance, including map boundaries."""
    def __init__(self, grid, length=.738, width=.500, obstacle_index=None):
        self.grid = grid
        self.hl, self.hw = length / 2, width / 2
        self.corner = math.hypot(self.hl, self.hw)
        if (obstacle_index is not None and grid.occ is obstacle_index.grid.occ and
                (grid.res, grid.ox, grid.oy) == (obstacle_index.grid.res,
                    obstacle_index.grid.ox, obstacle_index.grid.oy)):
            self.centers, self.tree = obstacle_index.centers, obstacle_index.tree
            return
        # An exterior rectangle's nearest obstacle point is on the union's
        # boundary. Interior cells cannot improve that distance. Retain an
        # 8-neighbor boundary superset; center-inside collisions are handled
        # explicitly below, including a rectangle wholly inside a solid wall.
        occupied=np.asarray(grid.occ,dtype=bool)
        boundary=occupied & ~binary_erosion(occupied,structure=np.ones((3,3),bool),
                                           border_value=0)
        iy, ix = np.nonzero(boundary)
        self.centers = np.column_stack((grid.ox + (ix + .5) * grid.res,
                                        grid.oy + (iy + .5) * grid.res))
        self.tree = cKDTree(self.centers) if len(ix) else None

    def _cell_distances(self, pose, centers):
        if not len(centers):
            return np.empty(0)
        c, s = math.cos(pose.th), math.sin(pose.th)
        h = self.grid.res / 2
        offsets = np.array([[-self.hl, -self.hw], [-self.hl, self.hw],
                            [self.hl, self.hw], [self.hl, -self.hw]])
        vertices = offsets @ np.array([[c, s], [-s, c]]) + [pose.x, pose.y]
        # Body vertices to occupied-cell edges.
        delta = np.maximum(np.abs(vertices[None, :, :] - centers[:, None, :]) - h, 0)
        a = np.min(np.linalg.norm(delta, axis=2), axis=1)
        # Cell vertices to body edges, expressed in the body's coordinates.
        cell = centers[:, None, :] + np.array([[-h, -h], [-h, h], [h, h], [h, -h]])
        d = cell - [pose.x, pose.y]
        local = np.stack((d[:, :, 0] * c + d[:, :, 1] * s,
                          -d[:, :, 0] * s + d[:, :, 1] * c), axis=2)
        b = np.min(np.linalg.norm(np.maximum(np.abs(local) - [self.hl, self.hw], 0),
                                  axis=2), axis=1)
        # Edge-crossing rectangles can overlap without any interior vertex.
        center_d = centers - [pose.x, pose.y]
        pad = h * (abs(c) + abs(s))
        overlap = ((np.abs(center_d[:, 0] * c + center_d[:, 1] * s) <= self.hl + pad) &
                   (np.abs(-center_d[:, 0] * s + center_d[:, 1] * c) <= self.hw + pad) &
                   (np.abs(center_d[:, 0]) <= h + self.hl * abs(c) + self.hw * abs(s)) &
                   (np.abs(center_d[:, 1]) <= h + self.hl * abs(s) + self.hw * abs(c)))
        return np.where(overlap, 0., np.minimum(a, b))

    def distance(self, pose):
        c, s = abs(math.cos(pose.th)), abs(math.sin(pose.th))
        rx, ry = self.hl * c + self.hw * s, self.hl * s + self.hw * c
        g = self.grid
        boundary = min(pose.x - rx - g.ox, pose.y - ry - g.oy,
                       g.ox + g.width * g.res - pose.x - rx,
                       g.oy + g.height * g.res - pose.y - ry)
        if boundary <= 0 or self.tree is None:
            return max(0., boundary)
        ix=int(math.floor((pose.x-g.ox)/g.res))
        iy=int(math.floor((pose.y-g.oy)/g.res))
        if g.occ[iy,ix]:
            return 0.
        _, index = self.tree.query([pose.x, pose.y])
        best = min(boundary, float(self._cell_distances(pose, self.centers[index:index+1])[0]))
        # Cells outside this ball cannot improve the current distance bound.
        ids = self.tree.query_ball_point([pose.x, pose.y],
                                        self.corner + g.res / math.sqrt(2) + best + 1e-10)
        if ids:
            best = min(best, float(np.min(self._cell_distances(pose, self.centers[ids]))))
        return best

    @staticmethod
    def _pose_array(poses):
        if isinstance(poses, np.ndarray):
            return np.asarray(poses, dtype=float).reshape((-1, 3))
        return np.asarray([(p.x, p.y, p.th) for p in poses], dtype=float).reshape((-1, 3))

    def _overlap_pairs(self, poses, centers):
        c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
        dx, dy = centers[:, 0]-poses[:, 0], centers[:, 1]-poses[:, 1]
        h = self.grid.res/2.
        pad = h*(abs(c)+abs(s))
        return ((abs(dx*c+dy*s) <= self.hl+pad) &
                (abs(-dx*s+dy*c) <= self.hw+pad) &
                (abs(dx) <= h+self.hl*abs(c)+self.hw*abs(s)) &
                (abs(dy) <= h+self.hl*abs(s)+self.hw*abs(c)))

    def _distance_pairs(self, poses, centers):
        """Exact rectangle/cell distances for aligned arrays of pose/cell pairs."""
        c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
        h = self.grid.res/2.
        ox = np.asarray([-self.hl, -self.hl, self.hl, self.hl])
        oy = np.asarray([-self.hw, self.hw, self.hw, -self.hw])
        vx = poses[:, 0, None]+c[:, None]*ox-s[:, None]*oy
        vy = poses[:, 1, None]+s[:, None]*ox+c[:, None]*oy
        dx = np.maximum(abs(vx-centers[:, 0, None])-h, 0.)
        dy = np.maximum(abs(vy-centers[:, 1, None])-h, 0.)
        body_to_cell = np.min(np.hypot(dx, dy), axis=1)
        dx = centers[:, 0, None]-poses[:, 0, None]+np.asarray([-h, -h, h, h])
        dy = centers[:, 1, None]-poses[:, 1, None]+np.asarray([-h, h, h, -h])
        lx = np.maximum(abs(dx*c[:, None]+dy*s[:, None])-self.hl, 0.)
        ly = np.maximum(abs(-dx*s[:, None]+dy*c[:, None])-self.hw, 0.)
        cell_to_body = np.min(np.hypot(lx, ly), axis=1)
        return np.where(self._overlap_pairs(poses, centers), 0.,
                        np.minimum(body_to_cell, cell_to_body))

    def _boundary_distances(self, poses):
        c, s = abs(np.cos(poses[:, 2])), abs(np.sin(poses[:, 2]))
        rx, ry = self.hl*c+self.hw*s, self.hl*s+self.hw*c
        g = self.grid
        return np.minimum.reduce((poses[:, 0]-rx-g.ox, poses[:, 1]-ry-g.oy,
            g.ox+g.width*g.res-poses[:, 0]-rx,
            g.oy+g.height*g.res-poses[:, 1]-ry))

    def distances(self, poses, chunk_size=128):
        """Batch the same exact clearance queries as distance(), without coarsening."""
        poses = self._pose_array(poses)
        result = np.maximum(0., self._boundary_distances(poses))
        if self.tree is None or not len(poses):
            return result
        g = self.grid
        active = np.flatnonzero(result > 0.)
        ix = ((poses[active, 0]-g.ox)/g.res).astype(int)
        iy = ((poses[active, 1]-g.oy)/g.res).astype(int)
        inside = g.occ[iy, ix]
        result[active[inside]] = 0.
        active = active[~inside]
        for offset in range(0, len(active), chunk_size):
            indices = active[offset:offset+chunk_size]
            query = poses[indices]
            _, nearest = self.tree.query(query[:, :2])
            best = np.minimum(result[indices], self._distance_pairs(query, self.centers[nearest]))
            radius = self.corner+g.res/math.sqrt(2.)+best+1e-10
            neighbors = self.tree.query_ball_point(query[:, :2], radius)
            counts = np.asarray([len(ids) for ids in neighbors], dtype=int)
            if np.sum(counts):
                owners = np.repeat(np.arange(len(query)), counts)
                cells = np.concatenate(neighbors).astype(int)
                values = self._distance_pairs(query[owners], self.centers[cells])
                np.minimum.at(best, owners, values)
            result[indices] = best
        return result

    def poses_clear(self, poses, chunk_size=256):
        """Exact batched SAT, including center-inside and map-boundary cases."""
        poses = self._pose_array(poses)
        clear = self._boundary_distances(poses) >= 0.
        if self.tree is None or not len(poses):
            return clear
        g = self.grid
        active = np.flatnonzero(clear)
        ix = np.clip(((poses[active, 0]-g.ox)/g.res).astype(int), 0, g.width-1)
        iy = np.clip(((poses[active, 1]-g.oy)/g.res).astype(int), 0, g.height-1)
        inside = g.occ[iy, ix]
        clear[active[inside]] = False
        active = active[~inside]
        # Intersecting rectangles must have centers within the sum of their
        # circumradii. Boundary cells suffice for an exterior body; the
        # center-inside case above also covers bodies wholly inside walls.
        radius = self.corner+g.res/math.sqrt(2.)+1e-10
        for offset in range(0, len(active), chunk_size):
            indices = active[offset:offset+chunk_size]
            query = poses[indices]
            neighbors = self.tree.query_ball_point(query[:, :2], radius)
            counts = np.asarray([len(ids) for ids in neighbors], dtype=int)
            if not np.sum(counts):
                continue
            owners = np.repeat(np.arange(len(query)), counts)
            cells = np.concatenate(neighbors).astype(int)
            blocked = self._overlap_pairs(query[owners], self.centers[cells])
            clear[indices[np.unique(owners[blocked])]] = False
        return clear


class TrajectoryEvaluator:
    def __init__(self, grid, start, goal, limits, length=.738, width=.500,
                 xy_tol=.16, yaw_tol=.14, profile=None, allow_reverse=False,
                 allow_shunting=True):
        self.grid, self.start, self.goal, self.limits = grid, start, goal, limits
        self.length, self.width = length, width
        self.xy_tol, self.yaw_tol = xy_tol, yaw_tol
        self.profile = profile or QualityProfile()
        self.allow_reverse = allow_reverse
        self.allow_shunting = bool(allow_shunting)
        self.distance = FootprintDistance(grid, length, width)
        self.safe_distance = FootprintDistance(grid, 2 * grid.half_l, 2 * grid.half_w,
                                               obstacle_index=self.distance)
        margin = 2 * grid.half_l - length
        # Cover the motion between sweep stations, not just sampled poses.
        self.sweep_grid = Grid(grid.occ, grid.res, grid.ox, grid.oy,
                               length, width, margin + self.profile.sweep_spacing)
        self.sweep_distance = FootprintDistance(self.sweep_grid,
            2*self.sweep_grid.half_l, 2*self.sweep_grid.half_w, obstacle_index=self.distance)
        # An evaluator belongs to one fixed occupancy map and one planning
        # request. Frozen block/start/spacing keys cannot reuse another shape.
        self._continuous_sweep_cache = set()
        self.sweep_stats = {"checked_blocks": 0, "reused_blocks": 0, "checked_poses": 0}
        self._block_sample_cache = {}
        self._block_sample_rows = 0
        self.sample_stats = {"computed_blocks": 0, "reused_blocks": 0}
        d = math.hypot(goal.x - start.x, goal.y - start.y)
        self.L0 = max(d, length)
        self.T0 = (max(d / self.profile.reference_speed_mps,
                       abs(wrap(goal.th-start.th)) /
                       self.profile.reference_spin_omega_radps, 1.)
                   if limits is not None else None)
        self.cache = {}

    def _sample_continuous_block(self, block, start, spacing):
        """Reuse exact geometry samples within this immutable-map request."""
        key = (block, start, spacing)
        if key in self._block_sample_cache:
            self.sample_stats["reused_blocks"] += 1
            return self._block_sample_cache[key]
        rows = sample_block(block, start, spacing)
        self.sample_stats["computed_blocks"] += 1
        if len(rows) <= 200000:
            if self._block_sample_rows+len(rows) > 200000:
                self._block_sample_cache.clear()
                self._block_sample_rows = 0
            self._block_sample_cache[key] = rows
            self._block_sample_rows += len(rows)
        return rows

    @staticmethod
    def signature(actions):
        return tuple((a.mode, round(a.amount, 9), round(a.control, 9))
                     for a in collapse_exact_runs(actions))

    def evaluate(self, actions, timed=None, allow_reverse=None):
        actions = collapse_exact_runs(list(actions))
        reverse_allowed = self.allow_reverse if allow_reverse is None else allow_reverse
        record = {"spec_version": "trajectory_quality_spec_v1.2",
                  "cost_profile": asdict(self.profile), "status": "INVALID",
                  "score": None, "reason": None, "geometry_status": "UNCHECKED",
                  "dynamic_status": "UNCHECKED",
                  "reference_scales": {"L0": self.L0, "T0": self.T0,
                                       "speed_mps": self.profile.reference_speed_mps,
                                       "spin_omega_radps": self.profile.reference_spin_omega_radps}}
        try:
            if not self.allow_shunting:
                violation = action_shunting_violation(actions)
                if violation is not None:
                    record["shunting_violation"] = violation
                    raise ValueError("shunting_forbidden:"+violation["reason"])
            for a in actions:
                if a.mode not in (0, 1, 2) or not all(math.isfinite(x) for x in (a.amount, a.control)):
                    raise ValueError("invalid_action")
                if not reverse_allowed and a.mode != 2 and a.amount < -1e-10:
                    raise ValueError("reverse_motion")
                if a.mode == 1 and abs(a.control) > math.pi / 2 + 1e-9:
                    raise ValueError("oblique_reverse_disguise")
                rmin = self.limits.r_min_ack if self.limits is not None else .810
                if a.mode == 0 and abs(a.control) > 1 / rmin + 1e-9:
                    raise ValueError("curvature_limit")
            forward = sum(a.amount for a in actions if a.mode != 2 and a.amount > 1e-10)
            backward = sum(-a.amount for a in actions if a.mode != 2 and a.amount < -1e-10)
            record["direction_policy"] = {"reverse_allowed": bool(reverse_allowed),
                                          "all_reverse_is_eligible": True}
            poses = replay(self.start, actions)
            end = poses[-1]
            xy_error = math.hypot(end.x-self.goal.x, end.y-self.goal.y)
            yaw_error = abs(wrap(end.th-self.goal.th))
            record["terminal"] = {"xy_error": xy_error, "yaw_error": yaw_error}
            if xy_error > self.xy_tol + 1e-8 or yaw_error > self.yaw_tol + 1e-8:
                raise ValueError("terminal")
            if not self.sweep_grid.trajectory_clear(poses, actions, self.profile.sweep_spacing):
                raise ValueError("covered_swept_collision")
            signature = self.signature(actions)
            if signature not in self.cache:
                self.cache[signature] = self._geometry_metrics(actions, poses)
            metrics = dict(self.cache[signature])
            metrics.update(reverse_preference_actions(actions, max(self.length, .25*self.L0)))
            record.update(status="GEOMETRY_VALID", geometry_status="PASS", metrics=metrics,
                          dynamic_status="UNCHECKED")
            if timed is None or self.limits is None:
                return record
            if timed.limits != self.limits:
                raise ValueError("timing_profile_mismatch")
            self._check_timing(timed, actions)
            terms = {"clearance": metrics["J_clear"], "shape": metrics["J_shape"],
                     "adjustment": metrics["J_adjust"], "time": timed.duration/self.T0,
                     "length": metrics["length_m"]/self.L0,
                     "reverse": metrics["reverse_length_m"]/self.L0,
                     "direction_change": metrics["direction_switches"],
                     "reverse_early": metrics["reverse_early_length_m"]/self.L0,
                     "reverse_start": metrics["reverse_start"],
                     "reverse_yaw_early": metrics["reverse_yaw_early_rad"]/math.pi}
            weights = {"clearance": self.profile.clearance, "shape": self.profile.shape,
                       "adjustment": self.profile.adjustment, "time": self.profile.time,
                       "length": self.profile.length, "reverse": self.profile.reverse,
                       "direction_change": self.profile.direction_change,
                       "reverse_early": self.profile.reverse_early,
                       "reverse_start": self.profile.reverse_start,
                       "reverse_yaw_early": self.profile.reverse_yaw_early}
            contributions = {key: terms[key]*weights[key] for key in terms}
            record.update(status="FEASIBLE", dynamic_status="KINEMATIC_PROXY_PASS",
                          terms=terms, contributions=contributions,
                          score=sum(contributions.values()), timing=timed.report())
        except (ValueError, ArithmeticError) as exc:
            if record["geometry_status"] == "PASS":
                record["dynamic_status"] = "FAIL"
            else:
                record["geometry_status"] = "FAIL"
            record.update(status="INVALID", score=None, reason=str(exc))
        return record

    def _continuous_block_collision(self, rows, block_index, global_s, block=None):
        """Same swept gate as final audit, with the first failed interval."""
        key = (block, rows[0][1], self.profile.sweep_spacing) if block is not None else None
        if key is not None and key in self._continuous_sweep_cache:
            self.sweep_stats["reused_blocks"] += 1
            return None
        poses = np.asarray([(r[1].x,r[1].y,r[1].th) for r in rows])
        delta = np.diff(poses, axis=0)
        delta[:, 2] = np.arctan2(np.sin(delta[:, 2]), np.cos(delta[:, 2]))
        counts = np.maximum(1, np.ceil((np.hypot(delta[:, 0],delta[:, 1])+
            self.sweep_grid.corner*abs(delta[:, 2]))/self.profile.sweep_spacing)).astype(int)
        edges = np.repeat(np.arange(len(counts)), counts)
        starts = np.repeat(np.cumsum(counts)-counts, counts)
        fractions = (np.arange(len(edges))-starts+1.)/counts[edges]
        query = poses[edges]+fractions[:, None]*delta[edges]
        query[:, 2] = np.arctan2(np.sin(query[:, 2]), np.cos(query[:, 2]))
        clear = self.sweep_distance.poses_clear(query)
        self.sweep_stats["checked_blocks"] += 1
        self.sweep_stats["checked_poses"] += len(query)
        failed = np.flatnonzero(~clear)
        if len(failed):
                edge = int(edges[failed[0]])
                a,b = rows[edge],rows[edge+1]
                point = b[1]
                return {"block_index": block_index, "mode": b[3], "direction": b[4],
                        "local_s_interval_m": [a[0],b[0]],
                        "global_s_interval_m": [global_s+a[0],global_s+b[0]],
                        "interval_end_pose": [point.x,point.y,point.th],
                        "interval_start_pose": [a[1].x,a[1].y,a[1].th],
                        "control": b[2], "frame_id": "map",
                        "sweep_spacing_m": self.profile.sweep_spacing,
                        "physical_body_corners_world_m": footprint_corners(
                            point,self.length,self.width),
                        "margin_body_corners_world_m": footprint_corners(
                            point,2*self.grid.half_l,2*self.grid.half_w),
                        "sweep_inflated_corners_world_m": footprint_corners(
                            point,2*self.sweep_grid.half_l,
                            2*self.sweep_grid.half_w),
                        # Preserve the original diagnostic key for existing
                        # report readers while naming the exact sweep layer.
                        "inflated_footprint_corners_world_m": footprint_corners(
                            point,2*self.sweep_grid.half_l,
                            2*self.sweep_grid.half_w)}
        if key is not None:
            if len(self._continuous_sweep_cache) >= 512:
                self._continuous_sweep_cache.clear()
            self._continuous_sweep_cache.add(key)
        return None

    def continuous_collision(self, plan):
        """Cheap strict sweep screening, with no retiming or clearance metrics."""
        pose, global_s = plan.start, 0.
        if not self.sweep_grid.pose_clear(pose):
            return {"reason": "start_collision", "global_s_interval_m": [0.,0.],
                    "interval_end_pose": [pose.x,pose.y,pose.th]}
        for bi,block in enumerate(plan.blocks):
            rows = self._sample_continuous_block(block,pose,self.profile.sweep_spacing)
            collision = self._continuous_block_collision(rows,bi,global_s,block)
            if collision is not None:
                return collision
            pose = rows[-1][1]
            global_s += block.length
        return None

    def evaluate_continuous_geometry(self, plan, timed=None, xy_tol=.02,
                                     yaw_tol=math.radians(1),
                                     allow_reverse=None, include_metrics=True,
                                     defer_score=False):
        """Independently check continuous geometry, exact goal, and swept safety.

        Timing is intentionally a separate contract. A geometry-only pass has
        no complete cost score and cannot be published as a feasible timed
        trajectory.
        """
        reverse_allowed = self.allow_reverse if allow_reverse is None else bool(allow_reverse)
        if timed is not None and not include_metrics and not defer_score:
            raise ValueError("continuous_timed_audit_requires_metrics")
        record = {"spec_version": "continuous_motion_v2_geometry_check_v1",
                  "status": "INVALID", "geometry_status": "UNCHECKED",
                  "score": None, "reason": None,
                  "goal_tolerance": {"xy_m": xy_tol, "yaw_rad": yaw_tol},
                  "reference_scales": {"L0": self.L0, "T0": self.T0,
                                       "speed_mps": self.profile.reference_speed_mps,
                                       "spin_omega_radps": self.profile.reference_spin_omega_radps}}
        try:
            if not isinstance(plan, ContinuousPlan):
                raise ValueError("continuous_plan_required")
            if (math.hypot(plan.start.x-self.start.x, plan.start.y-self.start.y) > 1e-8 or
                    abs(wrap(plan.start.th-self.start.th)) > 1e-8):
                raise ValueError("continuous_start_mismatch")
            if (math.hypot(plan.goal.x-self.goal.x, plan.goal.y-self.goal.y) > 1e-8 or
                    abs(wrap(plan.goal.th-self.goal.th)) > 1e-8):
                raise ValueError("continuous_goal_mismatch")
            if not all(math.isfinite(v) and v > 0 for v in (xy_tol, yaw_tol)):
                raise ValueError("continuous_invalid_goal_tolerance")
            if not plan.blocks:
                raise ValueError("continuous_empty_plan")
            if not self.allow_shunting:
                violation = shunting_violation((b.mode,b.direction,b.signed_angle)
                                              for b in plan.blocks)
                if violation is not None:
                    record["shunting_violation"] = violation
                    raise ValueError("shunting_forbidden:"+violation["reason"])
            for left, right in zip(plan.blocks, plan.blocks[1:]):
                if left.mode == right.mode and left.mode != 2 and left.direction != right.direction:
                    gap = left.control(left.length)[0]-right.control(0.)[0]
                    if left.mode == 1:
                        gap = .5*math.atan2(math.sin(2*gap), math.cos(2*gap))
                    if abs(gap) > 1e-7:
                        raise ValueError("continuous_same_mode_steering_jump")
            if not self.distance.distance(self.start) > 0:
                raise ValueError("start_collision")
            reverse_length = length = curvature_energy = curvature_tv = 0.
            oblique_tv = spin_angle = 0.
            body_min = safe_min = float("inf")
            low_clearance = risk_integral = peak_risk = 0.
            control_knots = mode_switches = direction_switches = 0
            metric_sample_count = 0
            pose = plan.start
            previous_mode = None
            previous_gear = None
            kmax = 1./(self.limits.r_min_ack if self.limits else .810)
            pref = self.profile.preferred_clearance_factor*self.width
            body_r = math.hypot(self.length, self.width)/2
            global_s = 0.
            for block_index,block in enumerate(plan.blocks):
                if block.mode not in (0, 1, 2):
                    raise ValueError("continuous_unknown_mode")
                if block.mode != 2 and block.direction < 0 and not reverse_allowed:
                    raise ValueError("reverse_motion")
                if previous_mode is not None and block.mode != previous_mode:
                    mode_switches += 1
                if block.mode != 2 and previous_gear is not None and block.direction != previous_gear:
                    direction_switches += 1
                if block.mode != 2:
                    previous_gear = block.direction
                previous_mode = block.mode
                # Keep geometric integration and swept collision checks fine,
                # while sampling the expensive exact OBB clearance query at its
                # own metric resolution. These used to share sweep_spacing,
                # doing two KD-tree distance queries at every 2.5 mm station.
                rows = self._sample_continuous_block(block, pose, self.profile.sweep_spacing)
                metric_rows = (self._sample_continuous_block(block, pose,
                                            self.profile.metric_spacing)
                               if include_metrics else ())
                metric_sample_count += len(metric_rows)
                control_knots += len(block.knots)
                if block.mode == 0:
                    controls = [row[2] for row in rows]
                    if any(abs(c) > kmax+1e-8 for c in controls):
                        raise ValueError("continuous_curvature_limit")
                    if include_metrics:
                        curvature_energy += sum(.5*(a[2]**2+b[2]**2)*(b[0]-a[0])
                                                for a, b in zip(rows, rows[1:]))/(kmax*kmax)
                        curvature_tv += sum(abs(b[2]-a[2]) for a, b in zip(rows, rows[1:]))
                        length += block.length
                    if block.direction < 0:
                        if include_metrics:
                            reverse_length += block.length
                elif block.mode == 1:
                    controls = [row[2] for row in rows]
                    if any(abs(c) > math.pi/2+1e-8 for c in controls):
                        raise ValueError("continuous_oblique_angle_limit")
                    if include_metrics:
                        oblique_tv += sum(abs(b[2]-a[2]) for a, b in zip(rows, rows[1:]))
                        length += block.length
                    if block.direction < 0:
                        if include_metrics:
                            reverse_length += block.length
                else:
                    if include_metrics:
                        spin_angle += abs(block.signed_angle)
                # Reject unsafe geometry before querying two exact clearance
                # distances at every metric station. The swept gate and its
                # resolution are identical for accepted candidates.
                collision = self._continuous_block_collision(rows,block_index,global_s,block)
                if collision is not None:
                    record["first_collision"] = collision
                    raise ValueError("continuous_swept_collision")
                if include_metrics:
                    block_risks = []
                    block_metric_positions = []
                    metric_poses = [row[1] for row in metric_rows]
                    distances = self.distance.distances(metric_poses)
                    safe_distances = self.safe_distance.distances(metric_poses)
                    for (local_s, sample_pose, control, mode, direction), distance, safe in zip(
                            metric_rows, distances, safe_distances):
                        body_min, safe_min = min(body_min, distance), min(safe_min, safe)
                        risk = max(0., (pref-distance)/pref)**2
                        block_risks.append(risk)
                        block_metric_positions.append(local_s)
                        peak_risk = max(peak_risk, risk)
                        low_clearance += float(distance < pref)
                    measure = (block.length if block.mode != 2 else
                               body_r*abs(block.signed_angle))
                    if len(block_risks) > 1:
                        positions = (np.linspace(0., measure, len(block_risks))
                                     if block.mode == 2 else
                                     np.asarray(block_metric_positions, dtype=float))
                        risk_integral += float(np.trapz(
                            block_risks, x=positions))
                pose = rows[-1][1]
                global_s += block.length
            xy_error = math.hypot(pose.x-self.goal.x, pose.y-self.goal.y)
            yaw_error = abs(wrap(pose.th-self.goal.th))
            record["terminal"] = {"x": pose.x, "y": pose.y, "yaw": pose.th,
                                  "xy_error_m": xy_error, "yaw_error_rad": yaw_error}
            if xy_error > xy_tol+1e-9 or yaw_error > yaw_tol+1e-9:
                raise ValueError("strict_terminal_failed")
            if include_metrics:
                record["metrics"] = {
                    "length_m": length, "reverse_length_m": reverse_length,
                    "spin_angle_rad": spin_angle,
                    "J_clear": risk_integral/max(self.L0, 1e-12)+.5*peak_risk,
                    "min_body_clearance_m": body_min,
                    "min_safe_clearance_m": safe_min,
                    "low_clearance_sample_fraction": low_clearance/max(1, metric_sample_count),
                    "clearance_metric_spacing_m": self.profile.metric_spacing,
                    "A_curvature_energy_normalized": curvature_energy/max(self.L0, 1e-12),
                    "A_curvature_total_variation": curvature_tv,
                    "O_angle_total_variation_rad": oblique_tv,
                    "oblique_adjustments": sum(
                        1 for block in plan.blocks if block.mode == 1 and
                        sum(abs(b.value-a.value) for a, b in
                            zip(block.knots, block.knots[1:])) > .05),
                    "mode_switches": mode_switches,
                    "direction_switches": direction_switches,
                    "control_knot_count": control_knots,
                    "sweep_spacing_m": self.profile.sweep_spacing}
                record["metrics"].update(reverse_preference_plan(
                    plan, max(self.length, .25*self.L0)))
            else:
                record["metrics"] = None
            record.update(status="GEOMETRY_VALID", geometry_status="PASS",
                          direction_policy={"reverse_allowed": reverse_allowed,
                                            "all_reverse_is_eligible": True},
                          dynamic_status="NOT_CHECKED",
                          timing_status="NOT_CHECKED",
                          statement=("geometry_only_no_complete_score" if include_metrics
                                     else "geometry_only_strict_sweep_without_metrics"))
        except (ValueError, ArithmeticError) as exc:
            record.update(status="INVALID", score=None,
                          geometry_status="FAIL", reason=str(exc))
        if record["status"] == "GEOMETRY_VALID" and timed is not None:
            try:
                timing = self._audit_continuous_timing(plan, timed, xy_tol, yaw_tol)
                if not include_metrics:
                    # Full geometry and independent timing certification have
                    # both completed. Only the soft quality score is deferred.
                    record.update(status="FEASIBLE", dynamic_status="PASS",
                        timing_status="PASS", timing=timing,
                        score_status="DEFERRED",
                        statement="strict_geometry_and_timing_pass_score_deferred")
                    return record
                metrics = record["metrics"]
                kmax = 1./(self.limits.r_min_ack if self.limits else .810)
                terms = {
                    "clearance": metrics["J_clear"],
                    "shape": (metrics["A_curvature_energy_normalized"]+
                              .1*metrics["A_curvature_total_variation"]/
                              max(kmax*self.L0, 1e-12)),
                    "adjustment": (metrics["A_curvature_total_variation"]/
                                   max(kmax, 1e-12)+
                                   metrics["O_angle_total_variation_rad"]/(math.pi/4)),
                    "time": timed.duration/max(self.T0, 1e-12),
                    "length": metrics["length_m"]/max(self.L0, 1e-12),
                    "reverse": metrics["reverse_length_m"]/max(self.L0, 1e-12),
                    "direction_change": metrics["direction_switches"],
                     "reverse_early": metrics["reverse_early_length_m"]/self.L0,
                     "reverse_start": metrics["reverse_start"],
                     "reverse_yaw_early": metrics["reverse_yaw_early_rad"]/math.pi}
                weights = {"clearance": self.profile.clearance,
                           "shape": self.profile.shape,
                           "adjustment": self.profile.adjustment,
                           "time": self.profile.time,
                           "length": self.profile.length,
                           "reverse": self.profile.reverse,
                           "direction_change": self.profile.direction_change,
                       "reverse_early": self.profile.reverse_early,
                       "reverse_start": self.profile.reverse_start,
                       "reverse_yaw_early": self.profile.reverse_yaw_early}
                contributions = {key: terms[key]*weights[key] for key in terms}
                record.update(status="FEASIBLE", dynamic_status="PASS",
                              timing_status="PASS", score=sum(contributions.values()),
                              cost_profile=self.profile.name,
                              terms=terms, contributions=contributions,
                              timing=timing)
            except (ValueError, ArithmeticError) as exc:
                record.update(status="INVALID", score=None,
                              dynamic_status="FAIL", timing_status="FAIL",
                              reason=str(exc))
        return record

    def _audit_continuous_timing(self, plan, timed, xy_tol, yaw_tol):
        from .continuous_retiming import ContinuousTimedPlan
        if not isinstance(timed, ContinuousTimedPlan):
            raise ValueError("continuous_timing_contract_required")
        if timed.interpolation_version != "continuous_motion_v2_piecewise_quintic_time_v3":
            raise ValueError("continuous_timing_version")
        if timed.limits != self.limits:
            raise ValueError("continuous_timing_profile_mismatch")
        if not timed.samples or abs(timed.samples[0][0]) > 1e-9:
            raise ValueError("continuous_timing_start")
        if abs(timed.samples[-1][0]-timed.duration) > 1e-8:
            raise ValueError("continuous_timing_end")
        if any(b[0] <= a[0] for a, b in zip(timed.samples, timed.samples[1:])):
            raise ValueError("continuous_nonincreasing_time")
        if timed.samples[0][2] != 0. or abs(timed.samples[-1][2]) > 1e-9:
            raise ValueError("continuous_endpoint_speed")
        if timed.same_mode_hold_seconds != 0. or timed.gear_change_hold_seconds != 0.:
            raise ValueError("continuous_forbidden_dwell")

        starts, pose = [], plan.start
        for block in plan.blocks:
            starts.append(pose)
            pose = self._sample_continuous_block(block, pose, .005)[-1][1]
        phase_index, switches = 0, 0
        translation_groups = {}
        previous_mode = None
        for index, block in enumerate(plan.blocks):
            if previous_mode is not None and block.mode != previous_mode:
                if phase_index >= len(timed.phases):
                    raise ValueError("continuous_missing_mode_switch_phase")
                phase = timed.phases[phase_index]
                if (phase.get("kind") != "mode_switch" or
                        phase.get("from_mode") != previous_mode or
                        phase.get("to_mode") != block.mode):
                    raise ValueError("continuous_phase_sequence")
                left=plan.blocks[index-1]
                old=Action(left.mode,left.signed_angle if left.mode==2 else left.direction,
                           0. if left.mode==2 else left.control(left.length)[0])
                new=Action(block.mode,block.signed_angle if block.mode==2 else block.direction,
                           0. if block.mode==2 else block.control(0.)[0])
                old_angles,_=wheel_configuration(old,self.limits)
                new_angles,_=wheel_configuration(new,self.limits)
                delta=float(np.max(np.abs(new_angles-old_angles)))
                rate,acc=self.limits.steer_rate_max,self.limits.steer_accel_max
                minimum=(2*math.sqrt(delta/acc) if delta<=rate*rate/acc
                         else delta/rate+rate/acc)
                if self.limits.translation_speed_policy == "mode_switch_only":
                    if (abs(phase["duration"]-self.limits.mode_switch_duration)>1e-8 or
                            abs(phase.get("steering_s",-1.)-phase["duration"])>1e-8 or
                            phase.get("setup_overlaps_steering") is not True or
                            phase.get("steering_profile") != "quintic_fixed_duration_kinematic" or
                            abs(phase.get("setup_s",-1.)-
                                min(self.limits.mode_setup_time,phase["duration"]))>1e-8):
                        raise ValueError("continuous_mode_switch_fixed_duration")
                elif (phase.get("steering_s",-1.)<minimum-1e-8 or
                      phase.get("setup_s",-1.)<self.limits.mode_setup_time-1e-8 or
                      abs(phase["duration"]-phase["steering_s"]-phase["setup_s"])>1e-8):
                    raise ValueError("continuous_mode_switch_steering_rate_accel")
                phase_index += 1
                switches += 1
            if block.mode == 2:
                if phase_index >= len(timed.phases):
                    raise ValueError("continuous_missing_spin_phase")
                phase = timed.phases[phase_index]
                if (phase.get("kind") != "spin" or
                        phase.get("block_index") != index or
                        phase.get("mode") != block.mode):
                    raise ValueError("continuous_phase_sequence")
                phase_index += 1
            else:
                local_s = 0.
                group = []
                while local_s < block.length-1e-8:
                    if phase_index >= len(timed.phases):
                        raise ValueError("continuous_missing_translation_phase")
                    phase = timed.phases[phase_index]
                    if (phase.get("kind") != "translation" or
                            phase.get("block_index") != index or
                            phase.get("mode") != block.mode or
                            phase.get("direction") != block.direction):
                        raise ValueError("continuous_phase_sequence")
                    if phase.get("profile") == "rest_to_rest":
                        if (group or abs(phase.get("s0", -1.)) > 1e-8 or
                                abs(phase.get("s1", -1.)-block.length) > 1e-8):
                            raise ValueError("continuous_rest_profile_domain")
                        local_s = block.length
                    elif phase.get("profile") == "piecewise_quintic":
                        if (abs(phase.get("s0", -1.)-local_s) > 1e-7 or
                                phase.get("s1", local_s) <= local_s+1e-9 or
                                phase.get("s1", block.length) > block.length+1e-8):
                            raise ValueError("continuous_piece_profile_domain")
                        local_s = float(phase["s1"])
                        group.append(phase)
                    else:
                        raise ValueError("continuous_unknown_translation_profile")
                    phase_index += 1
                if group:
                    for left, right in zip(group, group[1:]):
                        junction_speed = float(left["end_speed"])
                        if (junction_speed <= 1e-6 or
                                abs(junction_speed-right["start_speed"]) > 1e-8):
                            raise ValueError("continuous_same_mode_internal_stop")
                        if abs(left.get("end_acceleration", 0.)-
                               right.get("start_acceleration", 0.)) > 1e-8:
                            raise ValueError("continuous_internal_acceleration_jump")
                    translation_groups[index] = group
            previous_mode = block.mode
        if phase_index != len(timed.phases):
            raise ValueError("continuous_unmatched_phase")
        if timed.mode_switch_count != switches:
            raise ValueError("continuous_mode_switch_count")
        cursor = 0.
        dynamic = []
        bounds = {"center_speed": self.limits.v_max,
                  "path_acceleration": self.limits.a_max,
                  "path_jerk": self.limits.jerk_max,
                  "wheel_speed": self.limits.wheel_speed_max,
                  "lateral_acceleration": self.limits.lateral_a_max,
                  "angular_speed": self.limits.omega_max,
                  "angular_acceleration": self.limits.alpha_max,
                  "angular_jerk": self.limits.angular_jerk_max,
                  "steer_rate": self.limits.steer_rate_max,
                  "steer_acceleration": self.limits.steer_accel_max,
                  "oblique_normal_acceleration": self.limits.lateral_a_max,
                  "wheel_angle": self.limits.steer_angle_max}

        for phase in timed.phases:
            duration = phase.get("duration", 0.)
            if (not math.isfinite(duration) or duration <= 0 or
                    abs(phase.get("t0", -1.)-cursor) > 1e-8):
                raise ValueError("continuous_phase_time_gap")
            cursor += duration
            if phase["kind"] in ("translation", "spin"):
                block = plan.blocks[phase["block_index"]]
                if block.mode != phase["mode"]:
                    raise ValueError("continuous_phase_mode_mismatch")
                if (phase["kind"] == "translation" and
                        phase.get("profile") == "piecewise_quintic"):
                    measured = _piecewise_continuous_dynamics_peaks(
                        block, phase, self.limits)
                else:
                    measured = _continuous_dynamics_peaks(
                        block, duration, self.limits)
                phase_bounds = dict(bounds)
                if block.mode != 2:
                    phase_bounds.update(angular_speed=self.limits.translation_omega_max,
                        angular_acceleration=self.limits.translation_alpha_max,
                        angular_jerk=self.limits.translation_angular_jerk_max)
                    if self.limits.translation_speed_policy == "mode_switch_only":
                        # The interactive policy certifies center motion and
                        # geometry. Report actuator/body demand independently
                        # without applying the restricted profile's speed caps.
                        phase_bounds = {name: bound for name, bound in phase_bounds.items()
                            if name in ("center_speed", "path_acceleration",
                                        "path_jerk", "wheel_angle")}
                for name, bound in phase_bounds.items():
                    if bound is not None and measured[name] > bound+2e-4:
                        raise ValueError("continuous_dynamic_limit:"+name)
                if not phase.get("dynamic_peaks"):
                    raise ValueError("continuous_dynamic_report_missing")
                dynamic.append(measured)
        if abs(cursor-timed.duration) > 1e-8:
            raise ValueError("continuous_phase_duration")

        geometry_rows = {}
        for i, block in enumerate(plan.blocks):
            if block.mode != 2:
                geometry_rows[i] = self._sample_continuous_block(block, starts[i], .005)
        lookup_stations = {i: np.asarray([r[0] for r in rows])
                           for i, rows in geometry_rows.items()}
        phase_cursor = 0
        coefficient_cache = {}
        for t, actual_pose, speed, mode in timed.samples:
            if not math.isfinite(speed) or abs(speed) > self.limits.v_max+1e-8:
                raise ValueError("continuous_velocity_limit")
            while (phase_cursor < len(timed.phases)-1 and
                   t > timed.phases[phase_cursor]["t0"]+timed.phases[phase_cursor]["duration"]+1e-9):
                phase_cursor += 1
            phase = timed.phases[phase_cursor] if timed.phases else None
            if phase is None:
                raise ValueError("continuous_sample_outside_phase")
            tau = min(1., max(0., (t-phase["t0"])/phase["duration"]))
            if phase["kind"] == "mode_switch":
                expected_pose = Pose(*phase["pose"])
                expected_speed, expected_mode = 0., phase["to_mode"]
            else:
                block = plan.blocks[phase["block_index"]]
                expected_mode = block.mode
                if block.mode == 2:
                    progress, _, _, _ = _quintic_time_values(tau)
                    block_start = starts[phase["block_index"]]
                    expected_pose = Pose(block_start.x, block_start.y,
                                         block_start.th+block.signed_angle*progress)
                    expected_speed = 0.
                elif phase.get("profile") == "piecewise_quintic":
                    distance = phase["s1"]-phase["s0"]
                    c1t = phase["start_speed"]*phase["duration"]
                    c2t = .5*phase.get("start_acceleration", 0.)*phase["duration"]**2
                    a1t = phase.get("end_acceleration", 0.)*phase["duration"]**2
                    if phase_cursor not in coefficient_cache:
                        tail = np.linalg.solve(
                            np.asarray([[1., 1., 1.], [3., 4., 5.], [6., 12., 20.]]),
                            np.asarray([distance-c1t-c2t, phase["end_speed"]*phase["duration"]-c1t-2*c2t, a1t-2*c2t]))
                        coefficient_cache[phase_cursor] = np.asarray(
                            [0., c1t, c2t, tail[0], tail[1], tail[2]])
                    coeff = coefficient_cache[phase_cursor]
                    q = float(np.polynomial.polynomial.polyval(tau, coeff))
                    dsdt = float(np.polynomial.polynomial.polyval(
                        tau, np.arange(1, 6)*coeff[1:])/phase["duration"])
                    local_s = phase["s0"]+q
                    rows = geometry_rows[phase["block_index"]]
                    idx = min(len(rows)-2, max(0, int(np.searchsorted(
                        lookup_stations[phase["block_index"]], local_s, side="right")-1)))
                    left, right = rows[idx], rows[idx+1]
                    f = min(1., max(0., (local_s-left[0])/
                                    max(right[0]-left[0], 1e-12)))
                    expected_pose = Pose(left[1].x+(right[1].x-left[1].x)*f,
                                         left[1].y+(right[1].y-left[1].y)*f,
                                         left[1].th+(right[1].th-left[1].th)*f)
                    expected_speed = block.direction*dsdt
                else:
                    progress, ds, _, _ = _quintic_time_values(tau)
                    rows = geometry_rows[phase["block_index"]]
                    local_s = block.length*progress
                    idx = min(len(rows)-2, max(0, int(np.searchsorted(
                        lookup_stations[phase["block_index"]], local_s, side="right")-1)))
                    left, right = rows[idx], rows[idx+1]
                    f = (local_s-left[0])/max(right[0]-left[0], 1e-12)
                    expected_pose = Pose(left[1].x+(right[1].x-left[1].x)*f,
                                         left[1].y+(right[1].y-left[1].y)*f,
                                         left[1].th+(right[1].th-left[1].th)*f)
                    expected_speed = block.direction*block.length*ds/phase["duration"]
            if (mode != expected_mode or abs(speed-expected_speed) > 2e-5 or
                    math.hypot(actual_pose.x-expected_pose.x,
                               actual_pose.y-expected_pose.y) > 1e-4 or
                    abs(actual_pose.th-expected_pose.th) > 1e-4):
                raise ValueError("continuous_sample_interpolation_contract")
        end = timed.samples[-1][1]
        if (math.hypot(end.x-plan.goal.x, end.y-plan.goal.y) > xy_tol+1e-9 or
                abs(wrap(end.th-plan.goal.th)) > yaw_tol+1e-9):
            raise ValueError("continuous_timed_terminal")
        return {"profile": asdict(self.limits),
                "translation_speed_policy": self.limits.translation_speed_policy,
                "translation_dynamic_contract": (
                    "center_speed_acceleration_jerk_and_wheel_angle; "
                    "curvature_wheel_steering_demand_diagnostic_only"
                    if self.limits.translation_speed_policy == "mode_switch_only"
                    else "restricted_full_dynamics"),
                "hardware_certified": False,
                "duration_s": timed.duration,
                "phases_checked": len(timed.phases),
                "samples_checked": len(timed.samples),
                "mode_switch_hold_seconds": timed.mode_switch_hold_seconds,
                "same_mode_hold_seconds": 0., "gear_change_hold_seconds": 0.,
                "mode_switch_events": [dict(p) for p in timed.phases
                                       if p["kind"]=="mode_switch"],
                "independent_dynamic_peaks": dynamic}

    def _geometry_metrics(self, actions, poses):
        pref = self.profile.preferred_clearance_factor*self.width
        body_r = math.hypot(self.length, self.width)/2
        integral = peak_risk = length = low_length = curvature_energy = 0.
        body_min = safe_min = float("inf")
        kmax = 1/(self.limits.r_min_ack if self.limits else .810)
        o_events = a_reversals = 0
        o_tv = a_tv = curvature_jumps = 0.
        old = None
        confirmed_phi = None
        a_sign = 0
        for p, a in zip(poses, actions):
            measure = abs(a.amount)*(body_r if a.mode == 2 else 1.)
            n = max(1, math.ceil(measure/self.profile.metric_spacing))
            risks, low = [], []
            for i in range(n+1):
                q = advance(p, Action(a.mode, a.amount*i/n, a.control))
                d, dsafe = self.distance.distance(q), self.safe_distance.distance(q)
                body_min, safe_min = min(body_min, d), min(safe_min, dsafe)
                risks.append(max(0., (pref-d)/pref)**2)
                low.append(float(d < pref))
            integral += float(np.trapz(risks, dx=measure/n))
            peak_risk = max(peak_risk, max(risks))
            if a.mode != 2:
                length += abs(a.amount)
                low_length += float(np.trapz(low, dx=abs(a.amount)/n))
            if a.mode == 0:
                curvature_energy += abs(a.amount)*(a.control/kmax)**2
                if old is not None and old.mode == 0:
                    a_tv += abs(a.control-old.control)
                    curvature_jumps += abs(a.control-old.control)
                sign = 1 if a.control >= .10*kmax else -1 if a.control <= -.10*kmax else 0
                if sign:
                    if a_sign and sign != a_sign:
                        a_reversals += 1
                    a_sign = sign
                confirmed_phi = None
            elif a.mode == 1:
                if old is not None and old.mode == 1:
                    o_tv += abs(wrap(a.control-old.control))
                    if abs(wrap(a.control-confirmed_phi)) > 1e-4:
                        o_events += 1
                        confirmed_phi = a.control
                else:
                    confirmed_phi = a.control
                a_sign = 0
            else:
                a_sign, confirmed_phi = 0, None
            old = a
        if not actions:
            body_min = self.distance.distance(self.start)
            safe_min = self.safe_distance.distance(self.start)
            peak_risk = max(0., (pref-body_min)/pref)**2
        gears = [1 if a.amount > 0 else -1 for a in actions
                 if a.mode != 2 and abs(a.amount) > 1e-10]
        return {"J_clear": integral/self.L0+.5*peak_risk,
                "forward_length_m": sum(a.amount for a in actions if a.mode != 2 and a.amount > 0),
                "reverse_length_m": sum(-a.amount for a in actions if a.mode != 2 and a.amount < 0),
                "direction_switches": sum(a != b for a, b in zip(gears, gears[1:])),
                "J_shape": curvature_energy/self.L0,
                "J_adjust": o_events+o_tv/(math.pi/4)+.25*a_reversals,
                "length_m": length, "min_body_clearance_m": body_min,
                "min_safe_clearance_m": safe_min, "low_clearance_length_m": low_length,
                "low_clearance_fraction": low_length/max(length, 1e-12),
                "oblique_adjustments": o_events, "oblique_tv_rad": o_tv,
                "ackermann_reversals": a_reversals, "ackermann_tv": a_tv,
                "curvature_boundary_jump_total": curvature_jumps,
                "curvature_rate_interiors": 0.,
                "shape_representation": "constant_control_with_stationary_reconfiguration",
                "mode_switches": sum(a.mode != b.mode for a, b in zip(actions, actions[1:])),
                "spin_angle_rad": sum(abs(a.amount) for a in actions if a.mode == 2)}

    def _check_timing(self, timed, actions):
        limits = self.limits
        moving = [phase for phase in timed.phases if phase["kind"] != "reconfiguration"]
        declared = [Action(phase["mode"], phase["amount"], phase["control"]) for phase in moving]
        if self.signature(declared) != self.signature(actions):
            raise ValueError("timing_action_mismatch")
        if not timed.samples or abs(timed.samples[0][0]) > 1e-9:
            raise ValueError("timing_start")
        if abs(timed.samples[-1][0]-timed.duration) > 1e-8:
            raise ValueError("timing_duration_mismatch")
        if any(b[0] <= a[0] for a, b in zip(timed.samples, timed.samples[1:])):
            raise ValueError("non_increasing_time")
        if any(not math.isfinite(v) or abs(v) > limits.v_max+1e-8
               for _, _, v, _ in timed.samples):
            raise ValueError("velocity_limit")
        if timed.samples[0][2] != 0 or abs(timed.samples[-1][2]) > 1e-8:
            raise ValueError("endpoint_velocity")
        from .retiming import quintic, wheel_configuration
        phase_index = 0
        for t, p, v, mode in timed.samples:
            while (phase_index+1 < len(timed.phases) and
                   t > timed.phases[phase_index]["t0"]+timed.phases[phase_index]["duration"]+1e-9):
                phase_index += 1
            if not timed.phases:
                expected, expected_v = self.start, 0.
            else:
                phase = timed.phases[phase_index]
                tau = max(0., min(1., (t-phase["t0"])/phase["duration"]))
                if phase["kind"] == "reconfiguration":
                    expected, expected_v = Pose(*phase["pose"]), 0.
                else:
                    s, ds, _ = quintic(tau)
                    expected = advance(Pose(*phase["start"]),
                                       Action(phase["mode"], phase["amount"]*s, phase["control"]))
                    expected_v = 0. if phase["mode"] == 2 else phase["amount"]*ds/phase["duration"]
            if (math.hypot(expected.x-p.x, expected.y-p.y) > 1e-7 or
                    abs(wrap(expected.th-p.th)) > 1e-7 or abs(expected_v-v) > 1e-7):
                raise ValueError("sample_interpolation_contract")
            if timed.phases and 1e-8 < tau < 1-1e-8 and mode != phase["mode"]:
                raise ValueError("sample_mode_contract")
        expected_time = 0.
        for phase in timed.phases:
            duration = phase["duration"]
            if duration <= 0 or not math.isfinite(duration):
                raise ValueError("invalid_phase_duration")
            if abs(phase["t0"]-expected_time) > 1e-8:
                raise ValueError("phase_time_gap")
            expected_time += duration
            if phase["kind"] == "reconfiguration":
                delta = max(abs(a-b) for a, b in zip(phase["wheel_angles_start"], phase["wheel_angles_end"]))
                st = phase["steering_s"]
                if delta > 1e-8 and (st <= 0 or 1.875*delta/st > limits.steer_rate_max+1e-8 or
                        (10/math.sqrt(3))*delta/st**2 > limits.steer_accel_max+1e-8):
                    raise ValueError("steering_rate_limit")
                continue
            amount = abs(phase["amount"])
            expected_angles, expected_factors = wheel_configuration(
                Action(phase["mode"], phase["amount"], phase["control"]), limits)
            if (not np.allclose(expected_angles, phase["wheel_angles"], atol=1e-9) or
                    not np.allclose(expected_factors, phase["wheel_rate_factors"], atol=1e-9)):
                raise ValueError("wheel_configuration_contract")
            if max(abs(expected_angles)) > limits.steer_angle_max+1e-8:
                raise ValueError("wheel_angle_limit")
            speed = 1.875*amount/duration
            acceleration = (10/math.sqrt(3))*amount/duration**2
            jerk = 60*amount/duration**3
            spinning = phase["kind"] == "spin"
            if (speed > (limits.omega_max if spinning else limits.v_max)+1e-8 or
                    acceleration > (limits.alpha_max if spinning else limits.a_max)+1e-8 or
                    jerk > (limits.angular_jerk_max if spinning else limits.jerk_max)+1e-8):
                raise ValueError("acceleration_or_rate_limit")
            if max(phase["wheel_rate_factors"])*speed > limits.wheel_speed_max+1e-8:
                raise ValueError("wheel_speed_limit")
            if phase["mode"] == 0 and speed**2*abs(phase["control"]) > limits.lateral_a_max+1e-8:
                raise ValueError("lateral_acceleration_limit")
            if phase["mode"] == 0:
                curvature = abs(phase["control"])
                if (speed*curvature > limits.omega_max+1e-8 or
                        acceleration*curvature > limits.alpha_max+1e-8 or
                        jerk*curvature > limits.angular_jerk_max+1e-8):
                    raise ValueError("ackermann_angular_limit")
        if timed.phases and abs(expected_time-timed.duration) > 1e-8:
            raise ValueError("phase_duration_mismatch")
        if not all(self.sweep_grid.interpolated_clear(a[1], b[1], self.profile.sweep_spacing)
                   for a, b in zip(timed.samples, timed.samples[1:])):
            raise ValueError("display_interpolated_swept_collision")
