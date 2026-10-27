"""Lazy local mode/geometry proposals; guide connectivity is never a certificate."""
from dataclasses import dataclass
import heapq
import math
import time

import numpy as np
from scipy.ndimage import distance_transform_edt

from .action_mask import action_clear_vectorized
from .kernel_adapter import Action, Grid, Pose, advance, wrap, action_shunting_violation
from .local_connections import LocalConnections
from .reward import proxy_seconds
from fwis_v3_optimizer.core import fit_action_to_pose, collapse_exact_runs


@dataclass(frozen=True)
class Candidate:
    actions: tuple
    end: Pose
    proxy_s: float
    source: str


class Guide:
    def __init__(self, problem):
        grid = problem.grid
        self.grid = grid
        factor = max(1, math.ceil(.16/grid.res))
        self.res = grid.res*factor
        h, w = grid.occ.shape
        padded = np.pad(grid.occ, ((0, (-h) % factor), (0, (-w) % factor)),
                        constant_values=True)
        self.occ = padded.reshape(padded.shape[0]//factor, factor,
                                  padded.shape[1]//factor, factor).any(axis=(1, 3))
        self.distance = np.full(self.occ.shape, np.inf)
        clearance = distance_transform_edt(~self.occ)*self.res
        y, x = self.cell(problem.goal)
        if not self.inside(y, x) or self.occ[y, x]:
            return  # Fall back to Euclidean guidance, never reject the task.
        self.distance[y, x] = 0.
        queue = [(0., y, x)]
        while queue:
            d, y, x = heapq.heappop(queue)
            if d > self.distance[y, x]+1e-9:
                continue
            for dy, dx in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                ny, nx = y+dy, x+dx
                if not self.inside(ny,nx) or self.occ[ny,nx]:
                    continue
                nd = d+self.res*math.hypot(dx,dy)*(1.+.15/(.05+clearance[ny,nx]))
                if nd < self.distance[ny, nx]-1e-9:
                    self.distance[ny, nx] = nd
                    heapq.heappush(queue, (nd, ny, nx))

    def cell(self, pose):
        return (int(math.floor((pose.y-self.grid.oy)/self.res)),
                int(math.floor((pose.x-self.grid.ox)/self.res)))

    def inside(self, y, x):
        return 0 <= y < self.occ.shape[0] and 0 <= x < self.occ.shape[1]

    def distance_to_goal(self, pose, goal):
        y, x = self.cell(pose)
        if self.inside(y, x) and np.isfinite(self.distance[y, x]):
            return float(self.distance[y, x])
        return math.hypot(pose.x-goal.x, pose.y-goal.y)

    def anchors(self, pose, length):
        y, x = self.cell(pose)
        output, traveled, last = [], 0., 0.
        for _ in range(64):
            options = [(self.distance[ny,nx], ny, nx) for ny,nx in
                       ((y+dy,x+dx) for dy in (-1,0,1) for dx in (-1,0,1) if dy or dx)
                       if self.inside(ny,nx)]
            if not options or not self.inside(y,x):
                break
            d, ny, nx = min(options)
            if not math.isfinite(d) or d >= self.distance[y,x]-1e-9:
                break
            traveled += self.res*math.hypot(ny-y,nx-x)
            y,x = ny,nx
            if traveled-last >= .65*length:
                ahead = [(self.distance[ny,nx],ny,nx) for ny,nx in
                         ((y+dy,x+dx) for dy in (-1,0,1) for dx in (-1,0,1) if dy or dx)
                         if self.inside(ny,nx)]
                _,ay,ax = min(ahead)
                output.append((self.grid.ox+(x+.5)*self.res,
                               self.grid.oy+(y+.5)*self.res,math.atan2(ay-y,ax-x)))
                last = traveled
            if len(output) >= 4:
                break
        return output


class RuleCandidates:
    schema = "lazy_rule_macro_candidates_v2"

    def __init__(self, problem):
        self.problem = problem
        self.local = LocalConnections()
        self.guide = Guide(problem)
        # Match the final continuous auditor's sampling envelope.  The map and
        # vehicle contract are unchanged; proposals that only fit before the
        # auditor's extra half-spacing allowance are rejected here, not after
        # spending an endpoint fit on an already blocked prefix.
        grid = problem.grid
        spacing = min(.005, problem.quality_profile.sweep_spacing)
        margin = 2*grid.half_l-problem.length
        self.sweep_grid = Grid(grid.occ, grid.res, grid.ox, grid.oy,
                               problem.length, problem.width, margin+spacing)
        self.clear_cache = {}
        self.stats = {"proposed": 0, "history_rejected": 0, "collision_rejected": 0,
                      "generated": 0, "wall_s": 0., "sweep_s": 0.,
                      "sweep_checks": 0, "sweep_cache_hits": 0,
                      "sweep_envelope_spacing_m": spacing}

    def _clear(self, pose, action):
        # Exact keys only: quantizing a swept-volume cache can hide collisions.
        key = (pose.x, pose.y, pose.th, action.mode, action.amount, action.control)
        if key in self.clear_cache:
            self.stats["sweep_cache_hits"] += 1
            return self.clear_cache[key]
        begun = time.perf_counter()
        clear = action_clear_vectorized(self.sweep_grid, pose, action)
        self.stats["sweep_s"] += time.perf_counter()-begun
        self.stats["sweep_checks"] += 1
        if len(self.clear_cache) >= 32768:
            self.clear_cache.clear()
        self.clear_cache[key] = clear
        return clear

    def generate(self, pose, prefix=(), deadline=float("inf")):
        begun = time.perf_counter()
        p, grid = self.problem, self.problem.grid
        previous = prefix[-1].mode if prefix else p.initial_mode
        proposed, seen = [], set()

        def propose(actions, source):
            actions = tuple(collapse_exact_runs([a for a in actions if abs(a.amount)>1e-8]))
            if not actions:
                return
            if any(a.mode == 0 and abs(a.control)>1/p.limits.r_min_ack+1e-8 for a in actions):
                return
            key = tuple((a.mode, round(a.amount,7), round(a.control,7)) for a in actions)
            if key in seen:
                return
            seen.add(key)
            self.stats["proposed"] += 1
            if ((not p.allow_reverse and any(a.mode!=2 and a.amount<0 for a in actions)) or
                    (not p.allow_shunting and action_shunting_violation((*prefix,*actions)) is not None)):
                self.stats["history_rejected"] += 1
                return
            end, proxy, mode = pose, 0., previous
            for a in actions:
                proxy += proxy_seconds(a, mode, p.limits)
                mode, end = a.mode, advance(end,a)
            if not self.sweep_grid.pose_clear(end):
                return
            proposed.append(Candidate(actions,end,proxy,source))

        # Exact goal connection and independent guide/radial anchors. A single
        # center route never fixes all the available geometric branches.
        anchors = [(p.goal.x,p.goal.y,"goal",p.goal.th)]
        anchors += [(x,y,"guide",yaw) for x,y,yaw in self.guide.anchors(pose,p.length)]
        for radius in (.4,1.2,2.5):
            for offset in (0.,math.pi/4,-math.pi/4,math.pi/2,-math.pi/2,math.pi):
                yaw = pose.th+offset
                anchors.append((pose.x+radius*p.length*math.cos(yaw),
                                pose.y+radius*p.length*math.sin(yaw),"radial",yaw))
        for x,y,source,target_yaw in anchors:
            if time.perf_counter() >= deadline:
                break
            d = math.hypot(x-pose.x,y-pose.y)
            if d < 1e-7:
                if source == "goal":
                    propose([Action(2,wrap(p.goal.th-pose.th),0.)],"goal_spin")
                continue
            tangent = math.atan2(y-pose.y,x-pose.x)
            headings = [p.goal.th] if source == "goal" else list(dict.fromkeys([pose.th,tangent,target_yaw]))
            for yaw in headings:
                target = Pose(x,y,yaw)
                if not grid.pose_clear(target):
                    continue
                direct = fit_action_to_pose(0,pose,target)
                if direct:
                    propose([direct],source+"_A_direct")
                # Native local curves have an explicit horizon/length bound.
                if d <= 4*p.length:
                    for actions in self.local.ackermann(pose,target,p.limits.r_min_ack):
                        if sum(abs(a.amount) for a in actions) <= max(3*d,2*p.length):
                            propose(actions,source+"_A_curve")
                move = fit_action_to_pose(1,pose,Pose(x,y,pose.th))
                if move:
                    propose([move,Action(2,wrap(yaw-pose.th),0.)],source+"_O_spin")
                # Turning first gives a distinct swept volume and switch site.
                turn = Action(2,wrap(tangent-pose.th),0.)
                propose([turn,Action(0,d,0.),Action(2,wrap(yaw-tangent),0.)],source+"_spin_A")

        # Keep curved motion at several radii even when no anchor fits one arc.
        for angle in (wrap(p.goal.th-pose.th),math.pi/2,-math.pi/2,math.pi/4,-math.pi/4):
            propose([Action(2,angle,0.)],"local_spin")
        for amount in (.25*p.length,.5*p.length,p.length):
            for gear in (1,-1):
                for curvature in (-1.,-.5,0.,.5,1.):
                    propose([Action(0,gear*amount,curvature/p.limits.r_min_ack)],"A_arc")
        accepted = []
        proposed.sort(key=lambda c: c.proxy_s+self.guide.distance_to_goal(c.end,p.goal)/p.limits.target_v)
        for candidate in proposed:
            if time.perf_counter() >= deadline:
                break
            current = pose
            for a in candidate.actions:
                if not self._clear(current,a):
                    self.stats["collision_rejected"] += 1
                    break
                current = advance(current,a)
            else:
                accepted.append(candidate)
        self.stats["generated"] += len(accepted)
        self.stats["wall_s"] += time.perf_counter()-begun
        return accepted
