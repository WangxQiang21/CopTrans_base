"""Pure geometry/optimization kernel for the vehicle-center A/O/S planner."""
from dataclasses import dataclass, asdict
import os
import heapq
import math
import shutil
import tempfile
import time
import numpy as np
from scipy.ndimage import distance_transform_edt

O_CRUISE_SIGMA=math.radians(25.0)


def _symbolic_sinc(z, ca):
    """Stable sin(z)/z for exact symbolic constant-curvature motion.

    A global sixth-degree Taylor polynomial visibly shifts long arcs away
    from advance(). Use it only near zero, where the omitted term is below
    machine precision, and protect the unused division branch at z == 0.
    """
    near_zero=ca.fabs(z)<.01
    safe_denominator=ca.if_else(near_zero,1.0,z)
    series=1-z*z/6+z**4/120-z**6/5040
    return ca.if_else(near_zero,series,ca.sin(z)/safe_denominator)


def oblique_cruise_factor(phi):
    """Prefer Ackermann for motion nearly aligned with body heading."""
    return math.exp(-.5*(phi/O_CRUISE_SIGMA)**2)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    th: float


@dataclass(frozen=True)
class Action:
    mode: int  # 0 Ackermann, 1 oblique/lateral, 2 spin
    amount: float  # center arc length, center translation, or yaw increment
    control: float  # center curvature, oblique angle, or zero


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def has_reverse_motion(actions):
    """Return whether a translation primitive moves backward in body gear."""
    return any(action.mode != 2 and action.amount < -1e-10
               for action in actions)


def advance(p, a):
    if a.mode == 0:
        yaw = p.th + a.amount * a.control
        if abs(a.control) < 1e-10:
            return Pose(p.x + a.amount * math.cos(p.th),
                        p.y + a.amount * math.sin(p.th), wrap(yaw))
        return Pose(p.x + (math.sin(yaw) - math.sin(p.th)) / a.control,
                    p.y + (math.cos(p.th) - math.cos(yaw)) / a.control,
                    wrap(yaw))
    if a.mode == 1:
        return Pose(p.x + a.amount * math.cos(p.th + a.control),
                    p.y + a.amount * math.sin(p.th + a.control), p.th)
    if a.mode == 2:
        return Pose(p.x, p.y, wrap(p.th + a.amount))
    raise ValueError("invalid mode")


def replay(start, actions):
    poses = [start]
    for a in actions:
        poses.append(advance(poses[-1], a))
    return poses


def expand(start, actions, max_translation=.20, max_rotation=math.radians(10)):
    result = []
    for a in actions:
        limit = max_rotation if a.mode == 2 else max_translation
        count = max(1, math.ceil(abs(a.amount) / limit))
        result += [Action(a.mode, a.amount / count, a.control)] * count
    return replay(start, result), result


def collapse_exact_runs(actions):
    """Merge adjacent identical primitives without changing their geometry.

    A* emits fixed-length samples. Keeping every sample as an NLP knot makes
    the continuous solve scale with path length rather than control events.
    Only same-mode, same-control, same-sign runs are merged, so the composed
    primitive is exactly equivalent and never erases a reversal or steering
    change.
    """
    collapsed=[]
    for action in actions:
        if (collapsed and collapsed[-1].mode==action.mode and
                collapsed[-1].amount*action.amount>0 and
                abs(collapsed[-1].control-action.control)<1e-10):
            prior=collapsed[-1]
            collapsed[-1]=Action(prior.mode,prior.amount+action.amount,
                                 prior.control)
        else:
            collapsed.append(action)
    return collapsed


def bounded_research(start, goal, seed_actions, grid, xy_tol=.16, yaw_tol=.14,
                     max_iter=5000, padding=.8, initial_mode=0,
                     anytime=False, weights=None, allow_oblique=True,
                     max_mode_switches=1, mode_schedule=None):
    """Bounded A/O/S lattice fallback inside the seed's local spatial envelope.

    This is deliberately a fallback, not the primary global planner.  Search
    states retain exact poses; quantization is used only for duplicate keys.
    Every edge and terminal connector is swept-SAT checked.
    """
    seed_poses = replay(start, seed_actions)
    xs = [p.x for p in seed_poses] + [goal.x]
    ys = [p.y for p in seed_poses] + [goal.y]
    bounds = (min(xs)-padding, max(xs)+padding,
              min(ys)-padding, max(ys)+padding)
    if not grid.pose_clear(start) or not grid.pose_clear(goal):
        return None

    w={"spin":1.0,"reverse":1.0,"switch":2.0,"direction":20.0,
       "steer_tv":.40,"curvature":.01,"oblique_tv":1.50,
       "oblique_event_score":1.50,"oblique_use":.55,"oblique_cruise":.45}
    if weights: w.update(weights)
    if mode_schedule is not None:
        mode_schedule=tuple(int(mode) for mode in mode_schedule)
        if (not mode_schedule or any(mode not in (0,1,2)
                                     for mode in mode_schedule)):
            raise ValueError("invalid_bounded_research_mode_schedule")
        initial_mode=mode_schedule[0]
    # Gear zero means that the path has not translated yet. Gear is retained
    # in the search key so direction changes remain explicit route events.
    max_mode_switches=max(0,int(max_mode_switches))
    nodes = [(start, initial_mode, 0, 0, 0, 0.0, -1, None)]
    xy_res, yaw_res = .10, math.radians(10)
    def key(node):
        p, mode, gear, control, mode_switches = node[:5]
        return (round(p.x/xy_res), round(p.y/xy_res),
                round(wrap(p.th)/yaw_res), mode, gear, control,
                mode_switches)
    def heuristic(p):
        return max(max(0.0, math.hypot(p.x-goal.x, p.y-goal.y)-xy_tol),
                   .45*max(0.0, abs(wrap(p.th-goal.th))-yaw_tol))
    openq = [(heuristic(start), 0.0, 0)]
    best = {key(nodes[0]): 0.0}
    kmax = 1/.810
    goal_id, suffix = None, None
    best_actions,best_score=None,float("inf")
    iterations = 0

    def unwind(idx):
        path=[]
        while nodes[idx][6]>=0:
            path.append(nodes[idx][7]);idx=nodes[idx][6]
        return list(reversed(path))

    def connector(p, prior_mode, mode_switches, prior_gear):
        """Try terminal A/O fits in either travel direction."""
        proposals=[]
        spin_options=(False,) if mode_schedule is not None else (False,True)
        for spin_first in spin_options:
            q=p; mode=prior_mode; switches=mode_switches; prefix=[]
            if (mode_schedule is not None and
                    switches!=len(mode_schedule)-1):
                continue
            turn=wrap(goal.th-q.th)
            if spin_first:
                if abs(turn)<=1e-5:
                    continue
                if mode!=2 and switches>=max_mode_switches:
                    continue
                spin=Action(2,turn,0.0)
                if not grid.action_clear(q,spin,.035):
                    continue
                prefix.append(spin);q=advance(q,spin)
                if mode!=2:
                    switches+=1
                mode=2
                # Preserve the legacy terminal-pivot allowance: after a
                # collision-checked Spin, the final translation may select
                # its mode independently of the preceding route schedule.
                switches=0
            elif abs(turn)>yaw_tol:
                continue

            if mode_schedule is not None:
                translation_modes=(mode_schedule[-1],)
            else:
                translation_modes=( (0,1) if allow_oblique else (0,) )
            for translation_mode in translation_modes:
                if (translation_mode!=mode and
                        switches>=max_mode_switches):
                    continue
                action=fit_action_to_pose(translation_mode,q,goal,
                                          tolerance=max(xy_tol,yaw_tol))
                if action is None or abs(action.amount)<1e-7:
                    continue
                tail=prefix+[action]
                poses=replay(p,tail)
                if (math.hypot(poses[-1].x-goal.x,poses[-1].y-goal.y)>xy_tol or
                        abs(wrap(poses[-1].th-goal.th))>yaw_tol):
                    continue
                if grid.trajectory_clear(poses,tail):
                    proposals.append((sequence_cost(tail,w)["score"],tail))
        if not proposals:
            return None
        return min(proposals,key=lambda item:item[0])[1]

    while openq and iterations < max_iter:
        _, g, idx = heapq.heappop(openq)
        cur=nodes[idx]
        if g>cur[5]+1e-9: continue
        iterations+=1
        tail=connector(cur[0],cur[1],cur[4],cur[2])
        if tail is not None:
            candidate=unwind(idx)+tail
            candidate_score=sequence_cost(candidate,w)["score"]
            if candidate and candidate[0].mode!=initial_mode:
                candidate_score+=w["switch"]
            if candidate_score<best_score:
                best_score=candidate_score
                best_actions=candidate
                goal_id,suffix=idx,tail
            if not anytime:
                break
        p,mode,gear,control,mode_switches=cur[:5]
        candidates=[]
        for sign in (1,-1):
            for ci,k in enumerate((-kmax,0.0,kmax)):
                candidates.append((Action(0,sign*.20,k),sign,ci))
            if allow_oblique:
                for ci,phi in enumerate((-math.pi/2,-math.pi/4,0,
                                         math.pi/4,math.pi/2)):
                    candidates.append((Action(1,sign*.20,phi),sign,ci))
            candidates.append((Action(2,sign*math.radians(10),0),sign,0))
        for a,newgear,ci in candidates:
            changed=(a.mode!=mode)
            if mode_schedule is not None:
                if a.mode==mode_schedule[mode_switches]:
                    next_mode_switches=mode_switches
                elif (mode_switches+1<len(mode_schedule) and
                      a.mode==mode_schedule[mode_switches+1]):
                    next_mode_switches=mode_switches+1
                else:
                    continue
            else:
                if changed and mode_switches>=max_mode_switches: continue
                next_mode_switches=mode_switches+int(changed)
            q=advance(p,a)
            if not (bounds[0]<=q.x<=bounds[1] and bounds[2]<=q.y<=bounds[3]): continue
            if not grid.action_clear(p,a,.035): continue
            if a.mode==2:
                ng=g+w["spin"]*abs(a.amount)
            else:
                distance=abs(a.amount)
                ng=g+distance*(1+(w["reverse"] if a.amount<0 else 0.0))
                if gear and newgear!=gear:
                    ng+=w["direction"]
                if a.mode==0:
                    ng+=w["curvature"]*a.control*a.control*distance
                    if mode==0:
                        prior_k=(-kmax,0.0,kmax)[max(0,min(2,control))]
                        ng+=w["steer_tv"]*abs(a.control-prior_k)
                else:
                    ng+=distance*(w["oblique_use"]*abs(a.control)+
                                  w["oblique_cruise"]*
                                  oblique_cruise_factor(a.control))
                    if mode==1:
                        prior_phi=(-math.pi/2,-math.pi/4,0,
                                   math.pi/4,math.pi/2)[max(0,min(4,control))]
                        delta=abs(wrap(a.control-prior_phi))
                        ng+=(w["oblique_tv"]*delta+
                             w["oblique_event_score"]*delta/(delta+.03))
            if changed: ng+=w["switch"]
            next_gear=gear if a.mode==2 else newgear
            nxt=(q,a.mode,next_gear,ci,next_mode_switches,ng,idx,a)
            nk=key(nxt)
            if best.get(nk,float("inf"))<=ng+1e-9: continue
            best[nk]=ng
            nid=len(nodes);nodes.append(nxt)
            heapq.heappush(openq,(ng+heuristic(q),ng,nid))
    if goal_id is None or best_actions is None:
        return None
    actions=best_actions
    if not actions:
        return None
    poses=replay(start,actions)
    if not grid.trajectory_clear(poses,actions): return None
    return actions,iterations


def terminal_pose_graph_candidates(start, goal, seed_actions, grid,
                                   incumbent_score=float("inf"),
                                   tail_length=4.0,
                                   approach_offsets=(.6,.85,1.1,1.4,1.7),
                                   max_candidates=3, weights=None,
                                   profile=None):
    """Create cost-ranked terminal pose-graph alternatives around a safe seed.

    Seed anchors in the last ``tail_length`` meters connect to sampled poses
    on the goal axis through any exact A/O edge. At each sampled pose, direct
    A/O terminal edges and Spin-then-A/O edges compete. No mode sequence is
    selected in advance; all candidates use the common score and full sweep.
    """
    profile=profile if profile is not None else {}
    profile["terminal_pose_fits"]=0
    profile["terminal_pose_better_than_incumbent"]=0
    profile["terminal_pose_swept_safe"]=0
    profile["terminal_pose_best_cost"]=float("inf")
    profile["terminal_pose_approach_samples"]=0
    profile["terminal_pose_first_edge_rejects"]=0
    profile["terminal_pose_spin_rejects"]=0
    profile["terminal_pose_final_fit_rejects"]=0
    profile["terminal_pose_final_sweep_rejects"]=0
    events=collapse_exact_runs(seed_actions)
    if not events:
        return []
    seed_poses,sampled=expand(start,events,max_translation=.25,
                              max_rotation=math.radians(7.5))
    if not grid.trajectory_clear(seed_poses,sampled):
        return []
    metric=[0.0]
    spin_scale=(weights or {}).get("spin",1.0)
    for action in sampled:
        metric.append(metric[-1]+(
            abs(action.amount)*spin_scale if action.mode==2
            else abs(action.amount)))
    total=metric[-1]
    # A pure terminal Spin repeats the same center position at many sampled
    # knots. Do not treat those post-arrival knots as approach anchors: they
    # would preserve the old terminal Spin and then add another maneuver.
    last_translation_anchor=max(
        (i+1 for i,action in enumerate(sampled) if action.mode!=2),default=0)
    options=[];seen=set()
    for index,anchor in enumerate(seed_poses[:-1]):
        if index>last_translation_anchor:
            continue
        distance_from_end=total-metric[index]
        if distance_from_end>tail_length or distance_from_end<.10:
            continue
        prefix=sampled[:index]
        for offset in approach_offsets:
            ax=goal.x+offset*math.cos(goal.th)
            ay=goal.y+offset*math.sin(goal.th)
            for approach_yaw in {anchor.th,goal.th}:
                profile["terminal_pose_approach_samples"]+=1
                approach=Pose(ax,ay,approach_yaw)
                first_edges=[]
                if math.hypot(anchor.x-approach.x,
                              anchor.y-approach.y)<.01:
                    first_edges=[[]]
                else:
                    for mode in (0,1):
                        action=fit_action_to_pose(mode,anchor,approach)
                        if action is not None and grid.action_clear(
                                anchor,action,.035):
                            first_edges.append([action])
                if not first_edges:
                    profile["terminal_pose_first_edge_rejects"]+=1
                for first in first_edges:
                    at_approach=(advance(anchor,first[0]) if first
                                 else anchor)
                    turn=wrap(goal.th-at_approach.th)
                    spin_edges=[[]]
                    if abs(turn)>1e-3:
                        spin=Action(2,turn,0.0)
                        spin_edges=([[spin]] if grid.action_clear(
                            at_approach,spin,.035) else [])
                        if not spin_edges:
                            profile["terminal_pose_spin_rejects"]+=1
                    at_goal_yaw=Pose(at_approach.x,at_approach.y,goal.th)
                    for spin_edge in spin_edges:
                        for final_mode in (0,1):
                            final=fit_action_to_pose(
                                final_mode,at_goal_yaw,goal,
                                tolerance=max(.16,.14))
                            if final is None:
                                profile["terminal_pose_final_fit_rejects"]+=1
                                continue
                            if abs(final.amount)<1e-7:
                                continue
                            if not grid.action_clear(at_goal_yaw,final,.035):
                                profile["terminal_pose_final_sweep_rejects"]+=1
                                continue
                            candidate=collapse_exact_runs(
                                prefix+first+spin_edge+[final])
                            key=tuple((a.mode,round(a.amount,3),
                                       round(a.control,3)) for a in candidate)
                            if key in seen:
                                continue
                            seen.add(key)
                            score=sequence_cost(candidate,weights)["score"]
                            profile["terminal_pose_fits"]+=1
                            profile["terminal_pose_best_cost"]=min(
                                profile["terminal_pose_best_cost"],score)
                            if score>=incumbent_score-1e-8:
                                continue
                            profile["terminal_pose_better_than_incumbent"]+=1
                            dense_poses,dense=expand(
                                start,candidate,max_translation=.20,
                                max_rotation=math.radians(5))
                            if not grid.trajectory_clear(dense_poses,dense):
                                profile["terminal_pose_final_sweep_rejects"]+=1
                                continue
                            profile["terminal_pose_swept_safe"]+=1
                            poses=replay(start,candidate)
                            if (math.hypot(poses[-1].x-goal.x,
                                           poses[-1].y-goal.y)>.16+1e-5 or
                                    abs(wrap(poses[-1].th-goal.th))>.14+1e-5):
                                continue
                            options.append((score,candidate,poses,
                                            {"anchor":index,
                                             "approach_offset":offset,
                                             "approach_yaw":approach_yaw}))
    options.sort(key=lambda item:item[0])
    return options[:max_candidates]


class Grid:
    """ROS OccupancyGrid and exact OBB-vs-occupied-cell SAT validation."""
    def __init__(self, occupied, resolution, origin_x, origin_y,
                 length=.738, width=.500, margin=.04):
        self.occ = np.asarray(occupied, dtype=bool)
        self.height, self.width = self.occ.shape
        self.res = float(resolution)
        self.ox, self.oy = float(origin_x), float(origin_y)
        self.half_l = (length + margin) / 2
        self.half_w = (width + margin) / 2
        self.corner = math.hypot(self.half_l, self.half_w)
        self._clearance_lower_bound = None

    def center_clearance_lower_bound(self,x,y):
        """Conservative center-to-obstacle clearance including map edges."""
        ix=int(math.floor((float(x)-self.ox)/self.res))
        iy=int(math.floor((float(y)-self.oy)/self.res))
        if ix<0 or iy<0 or ix>=self.width or iy>=self.height:
            return 0.0
        if self._clearance_lower_bound is None:
            padded=np.pad(self.occ,1,mode="constant",constant_values=True)
            distance=distance_transform_edt(~padded)[1:-1,1:-1]*self.res
            self._clearance_lower_bound=np.maximum(
                0.0,distance-math.sqrt(2.0)*self.res)
        return float(self._clearance_lower_bound[iy,ix])

    def pose_clear(self, p):
        c, s = math.cos(p.th), math.sin(p.th)
        rx = self.half_l * abs(c) + self.half_w * abs(s)
        ry = self.half_l * abs(s) + self.half_w * abs(c)
        if (p.x-rx < self.ox or p.y-ry < self.oy or
            p.x+rx > self.ox+self.width*self.res or
            p.y+ry > self.oy+self.height*self.res):
            return False
        x0 = max(0, math.floor((p.x-rx-self.ox)/self.res))
        x1 = min(self.width-1, math.floor((p.x+rx-self.ox)/self.res))
        y0 = max(0, math.floor((p.y-ry-self.oy)/self.res))
        y1 = min(self.height-1, math.floor((p.y+ry-self.oy)/self.res))
        # Enumerate only occupied cells in the footprint's AABB, then apply
        # the same SAT separation tests in NumPy. This removes the Python
        # per-cell loop from repeated optimizer and anchor swept checks.
        local_occ=self.occ[y0:y1+1,x0:x1+1]
        oy_idx,ox_idx=np.nonzero(local_occ)
        if len(ox_idx)==0:
            return True
        dx=self.ox+(ox_idx+x0+.5)*self.res-p.x
        dy=self.oy+(oy_idx+y0+.5)*self.res-p.y
        half=self.res/2
        corner_pad=half*(abs(c)+abs(s))
        overlap=((np.abs(dx*c+dy*s)<=self.half_l+corner_pad) &
                 (np.abs(-dx*s+dy*c)<=self.half_w+corner_pad) &
                 (np.abs(dx)<=half+rx) & (np.abs(dy)<=half+ry))
        return not bool(np.any(overlap))

    def action_clear(self, p, a, max_corner_step=.01):
        rotation = abs(a.amount) if a.mode == 2 else (
            abs(a.amount*a.control) if a.mode == 0 else 0)
        translation = 0 if a.mode == 2 else abs(a.amount)
        n = max(1, math.ceil((translation+self.corner*rotation)/max_corner_step))
        for i in range(1, n+1):
            if not self.pose_clear(advance(p, Action(a.mode, a.amount*i/n, a.control))):
                return False
        return True

    def trajectory_clear(self, poses, actions, max_corner_step=.01):
        if not poses or len(poses) != len(actions)+1 or not self.pose_clear(poses[0]):
            return False
        return all(self.action_clear(p, a, max_corner_step)
                   for p, a in zip(poses, actions))

    def interpolated_clear(self, a, b, max_corner_step=.01):
        # Used only after exact action checks, for time-resampled output.
        dyaw = wrap(b.th-a.th)
        n = max(1, math.ceil((math.hypot(b.x-a.x,b.y-a.y)+self.corner*abs(dyaw))/max_corner_step))
        for i in range(1,n+1):
            t=i/n
            if not self.pose_clear(Pose(a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),wrap(a.th+t*dyaw))):
                return False
        return True


def features(grid):
    """Return four discs that conservatively cover the rectangular body.

    Each disc covers one quarter-length by full-width strip.  This keeps the
    corridor model conservative while halving its nonlinear feature
    constraints compared with splitting both length and width.
    """
    length, width = 2*grid.half_l, 2*grid.half_w
    dx = length/4
    r = math.hypot(dx/2,width/2)
    return [(-length/2+(i+.5)*dx, 0.0, r) for i in range(4)]


def _inscribed_heading_box(bounds, x, y, heading, aspect, resolution):
    """Return an ego-heading rectangle contained in a certified world AABB.

    The free-space search below certifies an axis-aligned box for each covered
    body feature center.  This converts that box to a conservative oriented
    trust region; all four returned corners remain inside the certified AABB.
    ``aspect`` keeps the local corridor longer along the reference heading.
    """
    xmin,xmax,ymin,ymax=bounds
    mx=max(0.0,min(x-xmin,xmax-x))
    my=max(0.0,min(y-ymin,ymax-y))
    c,s=abs(math.cos(heading)),abs(math.sin(heading))
    aspect=max(1.0,float(aspect))
    denom_x=c+s/aspect
    denom_y=s+c/aspect
    half_long=min(mx/max(denom_x,1e-12),my/max(denom_y,1e-12))
    # Preserve the seed as a feasible NLP initial point, including the rare
    # case where it lies exactly on a cell/AABB boundary.
    half_long=max(0.0,half_long-1e-9*resolution)
    half_lat=half_long/aspect
    return (x,y,half_long,half_lat,heading)


def corridor_box_corners(box):
    """World-frame corners of (cx, cy, half-long, half-lat, heading)."""
    x,y,half_long,half_lat,heading=box
    c,s=math.cos(heading),math.sin(heading)
    return [(x+u*c-v*s,y+u*s+v*c)
            for u,v in ((-half_long,-half_lat),(half_long,-half_lat),
                        (half_long,half_lat),(-half_long,half_lat))]


@dataclass(frozen=True)
class FeatureCorridor:
    """A feature-center region, never a physical vehicle envelope."""
    plan_id: object
    map_hash: object
    station: int
    feature_index: int
    reference_pose: Pose
    feature_offset: tuple
    feature_radius: float
    box: tuple
    certification: str
    used_in_optimization: bool
    frame_id: str = "map"
    schema: str = "feature_center_corridor_v1"

    def to_dict(self):
        record = asdict(self)
        record["corners_world_m"] = corridor_box_corners(self.box)
        record["size_m"] = [2*self.box[2], 2*self.box[3]]
        record["region_center_world_m"] = list(self.box[:2])
        record["half_extents_m"] = list(self.box[2:4])
        record["reference_yaw_rad"] = self.reference_pose.th
        return record


def footprint_corners(pose, length, width, margin=0.0):
    """Vehicle-center footprint; margin adds to total size (half per side)."""
    return corridor_box_corners((pose.x, pose.y, (length+margin)/2,
                                 (width+margin)/2, pose.th))


def select_corridor_display_stations(
        poses, boxes, max_spacing=.30, max_yaw_step=math.radians(8),
        narrow_half_width=.08, narrow_spacing=.12, extent_change=.06):
    """Choose corridor display stations from aligned trajectory poses/regions.

    This controls RViz marker density only. It does not alter corridor stations,
    optimizer variables, or the independent swept-footprint audit. Regions
    whose minimum lateral half extent is small are sampled more densely; yaw
    accumulation and changes in the represented region size also force a
    display sample.
    """
    if len(poses) != len(boxes):
        raise ValueError("display_pose_corridor_station_mismatch")
    if not poses:
        return []
    if max_spacing <= 0 or max_yaw_step <= 0 or narrow_spacing <= 0:
        raise ValueError("display_sampling_limits_must_be_positive")

    def region_extents(row):
        if not row:
            return None
        return (min(float(box[2]) for box in row),
                min(float(box[3]) for box in row))

    selected=[0]
    last=0
    distance=0.0
    yaw_change=0.0
    extents=[region_extents(row) for row in boxes]
    for i in range(1,len(poses)):
        prior,current=poses[i-1],poses[i]
        segment_distance=math.hypot(current.x-prior.x,current.y-prior.y)
        segment_yaw=abs(wrap(current.th-prior.th))
        # Select the preceding available station before the next source sample
        # would exceed the angular display limit. This avoids systematic
        # overshoot caused by the source's finite spatial sampling.
        if (yaw_change>1e-12 and yaw_change+segment_yaw>max_yaw_step and
                i-1>last):
            selected.append(i-1)
            last=i-1
            distance=segment_distance
            yaw_change=segment_yaw
        else:
            distance+=segment_distance
            yaw_change+=segment_yaw
        local_widths=[value[1] for value in (extents[last],extents[i])
                      if value is not None]
        local_spacing=(narrow_spacing if local_widths and
                       min(local_widths)<narrow_half_width else max_spacing)
        size_changed=(extents[last] is not None and extents[i] is not None and
                      max(abs(extents[i][j]-extents[last][j])
                          for j in (0,1))>=extent_change)
        if (distance>=local_spacing or yaw_change>=max_yaw_step or size_changed):
            selected.append(i)
            last=i
            distance=0.0
            yaw_change=0.0
    if len(poses)>1 and selected[-1]!=len(poses)-1:
        selected.append(len(poses)-1)
    return selected


def _angular_sector_from_vectors(vectors, reference_angle):
    """Return a conservative angular sector for a convex set of vectors.

    If all generating vectors lie in a cone narrower than pi, every convex
    combination lies in that cone too.  Otherwise the origin may be inside
    the set, or the direction is too ambiguous to use as a necessary bound.
    Angles are returned as offsets from ``reference_angle``.
    """
    deviations=[]
    for dx,dy in vectors:
        if math.hypot(dx,dy)<1e-7:
            return None
        deviations.append(wrap(math.atan2(dy,dx)-reference_angle))
    if not deviations:
        return None
    low=min(deviations);high=max(deviations)
    if (low>1e-7 or high<-1e-7 or
            high-low>=math.pi-1e-6):
        return None
    return low,high


def _corridor_center_aabb(feature_boxes):
    """Conservatively bound the body center from its two outer feature discs.

    The outer feature offsets are symmetric about the vehicle center, so the
    center is the average of their centers at every feasible yaw.  Averaging
    their corridor AABBs gives a superset of all feasible center positions.
    """
    first=corridor_box_corners(feature_boxes[0])
    last=corridor_box_corners(feature_boxes[-1])
    return (0.5*(min(p[0] for p in first)+min(p[0] for p in last)),
            0.5*(max(p[0] for p in first)+max(p[0] for p in last)),
            0.5*(min(p[1] for p in first)+min(p[1] for p in last)),
            0.5*(max(p[1] for p in first)+max(p[1] for p in last)))


def _aabb_difference_sector(start_box, end_box, reference_angle):
    """Bound directions of every displacement between two center AABBs."""
    dx_low=end_box[0]-start_box[1]
    dx_high=end_box[1]-start_box[0]
    dy_low=end_box[2]-start_box[3]
    dy_high=end_box[3]-start_box[2]
    if dx_low<=0.0<=dx_high and dy_low<=0.0<=dy_high:
        return None
    corners=[(dx,dy) for dx in (dx_low,dx_high)
             for dy in (dy_low,dy_high)]
    return _angular_sector_from_vectors(corners,reference_angle)


def build_corridors(grid, poses, max_extent=.60, covered_discs=True,
                    clearance_slack=0.0, *, records=None, plan_id=None,
                    map_hash=None, used_in_optimization=False):
    """Build per-feature corridors aligned with each reference pose heading.

    Free-space AABBs are first grown in map coordinates. Each is then
    conservatively inscribed by a heading-aligned rectangle, so the optimizer
    and RViz consume the same oriented local corridor. If a full-cover disc
    cannot be certified at one feature station, only that station falls back
    to a point corridor; other feature stations retain their disc clearance.
    ``clearance_slack`` is an experimental relaxation of the raster guard,
    bounded so it cannot remove the disc-radius threshold. Exact swept SAT
    remains mandatory for every returned candidate.
    """
    pad = np.pad(~grid.occ, 1, constant_values=False)
    clearance = distance_transform_edt(pad)[1:-1,1:-1]*grid.res
    feat = features(grid)
    radius = feat[0][2]
    clearance_slack=min(max(0.0,float(clearance_slack)),
                        math.sqrt(2)*grid.res)
    disc_allowed=clearance>=radius+math.sqrt(2)*grid.res-clearance_slack
    point_allowed=~grid.occ
    boxes = []
    all_disc_covered=bool(covered_discs)
    max_cells = max(1, int(max_extent/grid.res))
    aspect=grid.half_l/max(grid.half_w,1e-9)
    for station,p in enumerate(poses):
        c,s=math.cos(p.th),math.sin(p.th)
        row=[]
        for feature_index,(ox,oy,feature_radius) in enumerate(feat):
            x=p.x+ox*c-oy*s; y=p.y+ox*s+oy*c
            ix=math.floor((x-grid.ox)/grid.res);iy=math.floor((y-grid.oy)/grid.res)
            in_grid=0<=ix<grid.width and 0<=iy<grid.height
            use_disc=bool(covered_discs and in_grid and disc_allowed[iy,ix])
            if use_disc:
                allowed=disc_allowed
            else:
                if covered_discs:
                    all_disc_covered=False
                allowed=point_allowed
            if not (in_grid and allowed[iy,ix]):
                raise ValueError("coarse feature point lacks clearance")
            left=right=ix;bottom=top=iy
            changed=True
            while changed:
                changed=False
                if left>0 and ix-left<max_cells and np.all(allowed[bottom:top+1,left-1]):
                    left-=1;changed=True
                if right+1<grid.width and right-ix<max_cells and np.all(allowed[bottom:top+1,right+1]):
                    right+=1;changed=True
                if bottom>0 and iy-bottom<max_cells and np.all(allowed[bottom-1,left:right+1]):
                    bottom-=1;changed=True
                if top+1<grid.height and top-iy<max_cells and np.all(allowed[top+1,left:right+1]):
                    top+=1;changed=True
            bounds=(grid.ox+left*grid.res,grid.ox+(right+1)*grid.res,
                    grid.oy+bottom*grid.res,grid.oy+(top+1)*grid.res)
            box = _inscribed_heading_box(bounds,x,y,p.th,aspect,grid.res)
            row.append(box)
            if records is not None:
                records.append(FeatureCorridor(
                    plan_id, map_hash, station, feature_index, p, (ox,oy),
                    feature_radius, box,
                    ("disc_clearance_raster_guard" if use_disc and not clearance_slack
                     else "disc_clearance_relaxed_raster_guard" if use_disc
                     else "point_free_cell_only"), bool(used_in_optimization)))
        boxes.append(row)
    return feat,boxes,all_disc_covered


def _collapse_for_joint(start, actions, max_translation=2.0,
                        max_spin=math.pi,
                        ackermann_merge_tolerance=1e-5,
                        oblique_merge_tolerance=math.radians(.25)):
    """Use control/mode events, not uniform distance samples, as MINLP knots."""
    if not actions:
        return [], [start]
    # Cap exceptionally long individual primitives; ordinary same-control
    # samples collapse to a single event knot, keeping the integer horizon
    # small instead of making every fixed-distance sample a mode decision.
    split=[]
    for action in actions:
        limit=max_spin if action.mode==2 else max_translation
        count=max(1,math.ceil(abs(action.amount)/limit))
        split.extend(Action(action.mode,action.amount/count,action.control)
                     for _ in range(count))
    groups=[]; current=[]; length=0.0; mode=None; sign=None
    for action in split:
        action_sign=1 if action.amount>=0 else -1
        limit=max_spin if action.mode==2 else max_translation
        steering_change=False
        if current and action.mode==mode and action.mode in (0,1):
            if action.mode==0:
                delta=abs(action.control-current[-1].control)
                tolerance=max(1e-5,float(ackermann_merge_tolerance))
            else:
                delta=abs(wrap(action.control-current[-1].control))
                tolerance=max(math.radians(.25),
                              float(oblique_merge_tolerance))
            steering_change=delta>tolerance
        boundary=(current and (action.mode!=mode or action_sign!=sign or
                               steering_change or
                               length+abs(action.amount)>limit+1e-9))
        if boundary:
            groups.append(current);current=[];length=0.0
        if not current:
            mode,sign=action.mode,action_sign
        current.append(action);length+=abs(action.amount)
        if length>=limit-1e-9:
            groups.append(current);current=[];length=0.0;mode=None;sign=None
    if current: groups.append(current)

    collapsed=[]; refs=[start]; p=start
    for group in groups:
        mode=group[0].mode
        end=replay(p,group)[-1]
        if mode==0:
            amount=sum(a.amount for a in group)
            dtheta=sum(a.amount*a.control for a in group)
            control=dtheta/amount if abs(amount)>1e-9 else group[0].control
            candidate=Action(0,amount,control)
        elif mode==1:
            dx,dy=end.x-p.x,end.y-p.y
            distance=math.hypot(dx,dy)
            phi=wrap(math.atan2(dy,dx)-p.th) if distance>1e-9 else group[0].control
            amount=distance
            if phi>math.pi/2: phi-=math.pi;amount=-distance
            if phi<-math.pi/2: phi+=math.pi;amount=-distance
            candidate=Action(1,amount,phi)
        else:
            candidate=Action(2,sum(a.amount for a in group),0.0)
        collapsed.append(candidate);p=end;refs.append(end)
    return collapsed,refs


def _joint_seed_mode_gear_schedule(actions):
    """Return a seed's integer mode/gear assignment and first gear."""
    moving=[action for action in actions if action.mode!=2]
    first_gear=1 if (moving and moving[0].amount>=0) else 0
    gear=first_gear
    schedule=[]
    for action in actions:
        if action.mode!=2:
            gear=1 if action.amount>=0 else 0
        schedule.append((action.mode,gear))
    return first_gear,tuple(schedule)


def _collision_planes_on_sweep(grid, start, action, seed_start, seed_action,
                               guard=.005, max_corner_step=.01,
                               max_planes=4, min_spacing_m=.20,
                               existing_planes=()):
    """Return spaced seed-preserving SAT planes along a colliding sweep.

    Batching cuts from distinct points on the same bad segment can repair a
    whole collision interval in one fixed-schedule NLP. Each cut uses a fixed
    SAT axis and the larger vehicle support at candidate and seed headings.
    These are local feasibility cuts; exact replay and swept checks remain
    authoritative.
    """
    rotation=abs(action.amount) if action.mode==2 else (
        abs(action.amount*action.control) if action.mode==0 else 0.0)
    translation=0.0 if action.mode==2 else abs(action.amount)
    path_measure=translation+grid.corner*rotation
    count=max(1,int(math.ceil(path_measure/
                              max(.0005,float(max_corner_step)))))
    min_spacing=max(.05,float(min_spacing_m))
    max_planes=max(1,int(max_planes))
    def plane_key(plane):
        fraction,cell_x,cell_y,nx,ny,_=plane
        return (round(fraction,4),
                round((cell_x-grid.ox)/grid.res,3),
                round((cell_y-grid.oy)/grid.res,3),
                round(nx,3),round(ny,3))
    existing_keys={plane_key(plane) for plane in existing_planes}
    candidates=[]
    for i in range(1,count+1):
        fraction=i/float(count)
        sample_measure=fraction*path_measure
        candidate=advance(start,Action(action.mode,
                                       action.amount*fraction,
                                       action.control))
        if grid.pose_clear(candidate):
            continue
        seed_pose=advance(seed_start,Action(seed_action.mode,
                                            seed_action.amount*fraction,
                                            seed_action.control))
        c,s=math.cos(candidate.th),math.sin(candidate.th)
        rx=grid.half_l*abs(c)+grid.half_w*abs(s)
        ry=grid.half_l*abs(s)+grid.half_w*abs(c)
        x0=max(0,math.floor((candidate.x-rx-grid.ox)/grid.res))
        x1=min(grid.width-1,math.floor((candidate.x+rx-grid.ox)/grid.res))
        y0=max(0,math.floor((candidate.y-ry-grid.oy)/grid.res))
        y1=min(grid.height-1,math.floor((candidate.y+ry-grid.oy)/grid.res))
        local=grid.occ[y0:y1+1,x0:x1+1]
        iy,ix=np.nonzero(local)
        if not len(ix):
            continue
        dx=grid.ox+(ix+x0+.5)*grid.res-candidate.x
        dy=grid.oy+(iy+y0+.5)*grid.res-candidate.y
        cell_half=grid.res/2
        overlap=((np.abs(dx*c+dy*s)<=grid.half_l+cell_half*(abs(c)+abs(s))) &
                 (np.abs(-dx*s+dy*c)<=grid.half_w+cell_half*(abs(s)+abs(c))) &
                 (np.abs(dx)<=cell_half+rx) & (np.abs(dy)<=cell_half+ry))
        hits=np.flatnonzero(overlap)
        if not len(hits):
            continue
        axes=((1.0,0.0),(0.0,1.0),(c,s),(-s,c))
        possible=[]
        for hit in hits:
            cell_x=grid.ox+(ix[hit]+x0+.5)*grid.res
            cell_y=grid.oy+(iy[hit]+y0+.5)*grid.res
            qx=candidate.x-cell_x; qy=candidate.y-cell_y
            sx=seed_pose.x-cell_x; sy=seed_pose.y-cell_y
            for ax,ay in axes:
                seed_projection=sx*ax+sy*ay
                if abs(seed_projection)<1e-10:
                    continue
                sign=1.0 if seed_projection>0 else -1.0
                nx,ny=sign*ax,sign*ay
                cand_projection=nx*qx+ny*qy
                seed_projection=nx*sx+ny*sy
                cand_support=(grid.half_l*abs(c*nx+s*ny)+
                              grid.half_w*abs(-s*nx+c*ny))
                sc,ss=math.cos(seed_pose.th),math.sin(seed_pose.th)
                seed_support=(grid.half_l*abs(sc*nx+ss*ny)+
                              grid.half_w*abs(-ss*nx+sc*ny))
                obstacle_support=cell_half*(abs(nx)+abs(ny))
                required=max(cand_support,seed_support)+obstacle_support+guard
                # A seed-preserving cut cannot demand more clearance than the
                # safe seed actually has on this axis.  Near-grazing seed
                # paths used to lose every useful cut because the fixed guard
                # exceeded their available separation by a few millimeters.
                required=min(required,seed_projection-1e-6)
                if cand_projection+1e-6<required:
                    possible.append((required-cand_projection,cell_x,cell_y,
                                     nx,ny,required))
        if not possible:
            continue
        violation,cell_x,cell_y,nx,ny,required=min(
            possible,key=lambda item:item[0])
        plane=(float(fraction),float(cell_x),float(cell_y),float(nx),
               float(ny),float(required))
        if plane_key(plane) in existing_keys:
            continue
        candidates.append((float(violation),sample_measure,plane))
    # A sweep may contain many colliding samples. Keep the most penetrating
    # samples first, while spacing new planes along the primitive and omitting
    # cuts already active in this fixed-schedule repair. This avoids spending
    # an NLP pass on an early, mild contact when a later overlap is dominant.
    candidates.sort(key=lambda item:(-item[0],item[1]))
    planes=[]
    selected_measures=[]
    for _,sample_measure,plane in candidates:
        if any(abs(sample_measure-prior)<min_spacing
               for prior in selected_measures):
            continue
        planes.append(plane)
        selected_measures.append(sample_measure)
        if len(planes)>=max_planes:
            break
    return planes


def sequence_cost(actions, weights=None):
    """Common geometry score used to compare joint and fixed-sequence plans."""
    w={"spin":1.00,"reverse":1.00,"switch":2.00,"direction":20.0,
       "steer_tv":.40,"curvature":.01,"oblique_tv":1.50,
       "oblique_event_score":1.50,"oblique_use":.55,"oblique_cruise":.45,
       "low_curvature_switch":.35,"switch_turn_threshold":.35}
    if weights: w.update(weights)
    parts={name:0.0 for name in ("distance","spin","mode_switch","reverse",
                                 "direction","steer_tv","curvature",
                                 "oblique_tv","oblique_event","oblique_use",
                                 "oblique_cruise","low_curvature_switch")}
    switches=0;low_curvature_switches=0
    directions=0;spin_angle=0.0;reverse_length=0.0
    last_mode=None;last_gear=None;last_k=None;last_phi=None
    pose=Pose(0.0,0.0,0.0);last_motion_tangent=None
    for a in actions:
        if last_mode is not None and a.mode!=last_mode:
            parts["mode_switch"]+=w["switch"];switches+=1
            if (last_mode in (0,1) and a.mode in (0,1) and
                    last_motion_tangent is not None):
                incoming=(pose.th+(a.control if a.mode==1 else 0.0)+
                          (math.pi if a.amount<0 else 0.0))
                if abs(wrap(incoming-last_motion_tangent)) < \
                        w["switch_turn_threshold"]:
                    parts["low_curvature_switch"]+=w["low_curvature_switch"]
                    low_curvature_switches+=1
        if a.mode==2:
            angle=abs(a.amount);spin_angle+=angle
            parts["spin"]+=w["spin"]*angle
            last_k=None;last_phi=None
            last_motion_tangent=None
            pose=advance(pose,a)
        else:
            distance=abs(a.amount);parts["distance"]+=distance
            if a.amount<0:
                reverse_length+=distance
                parts["reverse"]+=w["reverse"]*distance
            gear=1 if a.amount>=0 else -1
            if last_gear is not None and gear!=last_gear:
                parts["direction"]+=w["direction"];directions+=1
            last_gear=gear
            if a.mode==0:
                if last_k is not None:
                    parts["steer_tv"]+=w["steer_tv"]*abs(a.control-last_k)
                parts["curvature"]+=w["curvature"]*a.control*a.control*distance
                last_k=a.control;last_phi=None
            else:
                if last_phi is not None:
                    delta=abs(wrap(a.control-last_phi))
                    parts["oblique_tv"]+=w["oblique_tv"]*delta
                    parts["oblique_event"]+=w["oblique_event_score"]*delta/(delta+.03)
                parts["oblique_use"]+=distance*w["oblique_use"]*abs(a.control)
                parts["oblique_cruise"]+=distance*w["oblique_cruise"]*oblique_cruise_factor(a.control)
                last_phi=a.control;last_k=None
            reached=advance(pose,a)
            last_motion_tangent=(
                reached.th if a.mode==0 else pose.th+a.control)
            if a.amount<0:
                last_motion_tangent+=math.pi
            last_motion_tangent=wrap(last_motion_tangent)
            pose=reached
        last_mode=a.mode
    score=sum(parts.values())
    return {"score":score,"cost_parts":parts,"switches":switches,
            "low_curvature_switches":low_curvature_switches,
            "direction_switches":directions,"spin_angle":spin_angle,
            "reverse_length":reverse_length}


def fit_action_to_pose(mode, start, target, tolerance=.003):
    """Fit one exact A/O/S primitive to a fixed pair of center poses."""
    dx,dy=target.x-start.x,target.y-start.y
    distance=math.hypot(dx,dy)
    dyaw=wrap(target.th-start.th)
    if mode==2:
        if distance>tolerance:
            return None
        action=Action(2,dyaw,0.0)
    elif mode==1:
        if abs(dyaw)>1e-4 or distance<1e-8:
            return None
        phi=wrap(math.atan2(dy,dx)-start.th)
        amount=distance
        if phi>math.pi/2:
            phi-=math.pi;amount=-distance
        if phi<-math.pi/2:
            phi+=math.pi;amount=-distance
        action=Action(1,amount,phi)
    elif mode==0:
        if distance<1e-8:
            return None
        if abs(dyaw)<1e-5:
            amount=dx*math.cos(start.th)+dy*math.sin(start.th)
            lateral=-dx*math.sin(start.th)+dy*math.cos(start.th)
            if abs(lateral)>tolerance or abs(amount)<1e-8:
                return None
            action=Action(0,amount,0.0)
        else:
            mid=start.th+dyaw/2
            along=dx*math.cos(mid)+dy*math.sin(mid)
            lateral=-dx*math.sin(mid)+dy*math.cos(mid)
            if abs(along)<1e-8 or abs(lateral)>tolerance:
                return None
            curvature=2*math.sin(dyaw/2)/along
            if abs(curvature)>1/.810+1e-8:
                return None
            amount=dyaw/curvature
            action=Action(0,amount,curvature)
    else:
        raise ValueError("invalid_mode")
    reached=advance(start,action)
    if (math.hypot(reached.x-target.x,reached.y-target.y)>tolerance or
            abs(wrap(reached.th-target.th))>tolerance):
        return None
    return action


def mode_schedule_neighbor_candidates(start, actions, grid, max_candidates=4,
                                      max_window=6, weights=None,
                                      sweep_step=.005,
                                      require_proxy_improvement=True,
                                      prefer_switch_reduction=False):
    """Build exact, swept-safe A/O substitutions over short same-mode runs.

    An O run has constant body heading, so its endpoints can be matched by a
    pair of equal-length Ackermann arcs with opposite curvature. Conversely,
    an A window with equal endpoint headings can be represented by one O
    translation. These are mode-neighborhood candidates, not MINLP solutions;
    the caller may continuously refine them under a fixed schedule.
    """
    if not actions or max_candidates<=0:
        return []
    actions=collapse_exact_runs(actions)
    poses=replay(start,actions)
    seed_score=sequence_cost(actions,weights)["score"]
    seed_switches=sum(a.mode!=b.mode for a,b in zip(actions,actions[1:]))
    candidates=[]
    seen=set()
    max_window=max(1,int(max_window))
    kmax=1/.810

    def add_candidate(candidate, info):
        if not candidate:
            return
        key=tuple((a.mode,round(a.amount,5),round(a.control,5))
                  for a in candidate)
        if key in seen:
            return
        seen.add(key)
        score=sequence_cost(candidate,weights)["score"]
        switches=sum(a.mode!=b.mode for a,b in zip(candidate,candidate[1:]))
        if prefer_switch_reduction and switches>=seed_switches:
            return
        if require_proxy_improvement and score>=seed_score-1e-7:
            return
        candidate_poses=replay(start,candidate)
        if not grid.trajectory_clear(candidate_poses,candidate,sweep_step):
            return
        candidates.append((score,candidate,candidate_poses,
                           dict(info, mode_switch_count=switches,
                                proxy_score=score)))

    for first in range(len(actions)):
        old_mode=actions[first].mode
        if old_mode not in (0,1):
            continue
        limit=min(len(actions),first+max_window)
        for last in range(first+1,limit+1):
            if any(action.mode!=old_mode
                   for action in actions[first:last]):
                break
            p,q=poses[first],poses[last]
            if old_mode==0:
                # A-to-O is exact whenever the window restores its initial
                # heading and the net displacement is forward-reachable.
                alternate=fit_action_to_pose(1,p,q,tolerance=.002)
                if alternate is not None and alternate.amount>1e-5:
                    candidate=(actions[:first]+[alternate]+actions[last:])
                    add_candidate(candidate,{"from_mode":0,"to_mode":1,
                                             "first":first,"last":last})
            else:
                # A straight O segment is the zero-curvature limit. Otherwise
                # derive a two-arc S curve from endpoint displacement and yaw.
                direct=fit_action_to_pose(0,p,q,tolerance=.002)
                alternatives=[]
                if direct is not None and direct.amount>1e-5:
                    alternatives.append([direct])
                dx,dy=q.x-p.x,q.y-p.y
                distance=math.hypot(dx,dy)
                if distance>1e-6 and abs(wrap(q.th-p.th))<1e-4:
                    bearing=math.atan2(dy,dx)
                    delta=wrap(2*wrap(bearing-p.th))
                    if abs(delta)>1e-5:
                        curvature=4*math.sin(delta/2)/distance
                        length=delta/curvature if abs(curvature)>1e-10 else 0.0
                        if (length>1e-5 and abs(curvature)<=kmax+1e-8):
                            arcs=[Action(0,length,curvature),
                                  Action(0,length,-curvature)]
                            endpoint=replay(p,arcs)[-1]
                            if (math.hypot(endpoint.x-q.x,endpoint.y-q.y)
                                    <=.002 and
                                    abs(wrap(endpoint.th-q.th))<=.002):
                                alternatives.append(arcs)
                for alternate_actions in alternatives:
                    candidate=(actions[:first]+alternate_actions+
                               actions[last:])
                    add_candidate(candidate,{"from_mode":1,"to_mode":0,
                                             "first":first,"last":last})
    if prefer_switch_reduction:
        candidates.sort(key=lambda item:(item[3]["mode_switch_count"],item[0]))
    else:
        candidates.sort(key=lambda item:item[0])
    return candidates[:int(max_candidates)]


def upstream_sos_replan_candidates(start, goal, seed_actions, grid,
                                   hotspot_idx, incumbent_score=float("inf"),
                                   max_iter=180, padding=.8, xy_tol=.16,
                                   yaw_tol=.14, max_candidates=3,
                                   upstream_distance=4.0, score_slack=.75,
                                   switch_bonus=.8, weights=None,
                                   min_upstream_clearance=0.0,
                                   max_upstream_windows=3):
    """Move terminal reorientation into free space, then replan to the goal.

    Candidate macros replace the seed between two existing event anchors with
    Spin-Oblique-Spin.  The downstream suffix is searched again from the new
    pose, so this is a seed for an actual local replan, not merely a relabeling
    of the old mode sequence.  Returned paths have passed a full swept-OBB
    check. A candidate may use a configured small score allowance only when
    its downstream maneuver count or steering variation actually decreases.
    """
    events=collapse_exact_runs(seed_actions)
    if len(events)<3:
        return [],{"sos_attempts":0,"sos_searches":0,"sos_safe":0}
    poses=replay(start,events)
    hotspot_idx=max(1,min(int(hotspot_idx),len(events)-1))
    path_s=[0.0]
    for action in events:
        path_s.append(path_s[-1]+(abs(action.amount) if action.mode!=2 else 0.0))

    # Q sits just before the high-maneuver section; P is selected upstream at
    # several distances so a spin can happen in a larger free patch.
    q_indices=[]
    for idx in (hotspot_idx+1,hotspot_idx+2):
        if (0<idx<len(events) and
                path_s[idx]-path_s[hotspot_idx]<=4.0 and
                idx not in q_indices):
            q_indices.append(idx)
    attempts=searches=suffix_successes=score_improved=0
    complexity_improved=safe_count=0
    clear_window_count=0
    clearance_rejected=0
    upstream_clearances=[]
    complexity_rejected=score_rejected=sweep_rejected=endpoint_rejected=0
    best_maneuver_reduction=0
    best_steering_reduction=0.0
    best_score=float("inf")
    found=[]

    def local_metrics(actions, initial_mode=None, horizon=4.0):
        window=[];distance=0.0
        for action in actions:
            if window and distance>=horizon:
                break
            window.append(action)
            if action.mode!=2:
                distance+=abs(action.amount)
        mode_changes=sum(a.mode!=b.mode
                         for a,b in zip(window,window[1:]))
        if (window and initial_mode is not None and
                window[0].mode!=initial_mode):
            mode_changes+=1
        direction_changes=sum(a.amount*b.amount<0
                              for a,b in zip(window,window[1:])
                              if a.mode!=2 and b.mode!=2)
        steering_tv=sum(abs(b.control-a.control)
                        for a,b in zip(window,window[1:])
                        if a.mode==b.mode and a.mode in (0,1))
        return mode_changes+direction_changes,steering_tv
    for qidx in q_indices:
        qbase=poses[qidx]
        p_cuts=[]
        for wanted in (.6,1.3,2.1,3.0,3.8):
            distance=min(wanted,upstream_distance)
            # Anchor P before the hotspot itself, even when Q is selected
            # after the difficult section so the macro can replace it.
            target_s=path_s[hotspot_idx]-distance
            if target_s<path_s[0] or distance<.45:
                continue
            for event_idx,action in enumerate(events[:qidx]):
                if action.mode==2:
                    continue
                begin,end=path_s[event_idx],path_s[event_idx+1]
                if begin-1e-8<=target_s<=end+1e-8 and end-begin>1e-8:
                    fraction=max(0.0,min(1.0,(target_s-begin)/(end-begin)))
                    if fraction<=1e-8:
                        prefix=events[:event_idx]
                        p=poses[event_idx]
                    elif fraction>=1-1e-8:
                        prefix=events[:event_idx+1]
                        p=poses[event_idx+1]
                    else:
                        partial=Action(action.mode,action.amount*fraction,
                                       action.control)
                        prefix=events[:event_idx]+[partial]
                        p=advance(poses[event_idx],partial)
                    p_clearance=grid.center_clearance_lower_bound(p.x,p.y)
                    if p_clearance+1e-9<min_upstream_clearance:
                        clearance_rejected+=1
                        break
                    cut=(round(target_s,5),prefix,p,event_idx,p_clearance)
                    if all(cut[0]!=old[0] for old in p_cuts):
                        p_cuts.append(cut)
                    break
        p_cuts.sort(key=lambda cut:(-cut[4],
                                    path_s[hotspot_idx]-cut[0]))
        p_cuts=p_cuts[:max(1,int(max_upstream_windows))]
        upstream_clearances.extend(cut[4] for cut in p_cuts)
        if not p_cuts:
            continue

        def outgoing_heading(idx):
            for offset,action in enumerate(events[idx:]):
                if action.mode==2 or abs(action.amount)<1e-8:
                    continue
                return wrap(poses[idx+offset].th +
                            (action.control if action.mode==1 else 0.0) +
                            (math.pi if action.amount<0 else 0.0))
            return goal.th

        # Use the exact event state for its body heading and a route tangent
        # as alternatives.  Duplicates are removed on the circle.
        q_yaws=[]
        for yaw in (qbase.th,outgoing_heading(qidx),goal.th):
            if all(abs(wrap(yaw-old))>math.radians(12) for old in q_yaws):
                q_yaws.append(yaw)
        macros=[]
        for p_s,prefix,p,event_idx,p_clearance in p_cuts:
            dx,dy=qbase.x-p.x,qbase.y-p.y
            if math.hypot(dx,dy)<.4:
                continue
            line_yaw=math.atan2(dy,dx)
            seed_action=events[event_idx]
            seed_heading=wrap(p.th+
                (seed_action.control if seed_action.mode==1 else 0.0)+
                (math.pi if seed_action.amount<0 else 0.0))
            p_yaws=[]
            for yaw in (line_yaw,seed_heading,qbase.th):
                if all(abs(wrap(yaw-old))>math.radians(12) for old in p_yaws):
                    p_yaws.append(yaw)
            for first_yaw in p_yaws:
                spin1=Action(2,wrap(first_yaw-p.th),0.0)
                if abs(spin1.amount)<math.radians(8):
                    continue
                if not grid.action_clear(p,spin1,.035):
                    continue
                after_spin=advance(p,spin1)
                oblique=fit_action_to_pose(
                    1,after_spin,Pose(qbase.x,qbase.y,first_yaw),
                    tolerance=1e-5)
                if (oblique is None or oblique.amount<0.0 or
                        not grid.action_clear(after_spin,oblique,.035)):
                    continue
                q_after=advance(after_spin,oblique)
                for qyaw in q_yaws:
                    spin2=Action(2,wrap(qyaw-first_yaw),0.0)
                    if abs(spin2.amount)<math.radians(8):
                        continue
                    if not grid.action_clear(q_after,spin2,.035):
                        continue
                    macros.append((p_s,prefix,event_idx,p_clearance,
                                   first_yaw,qyaw,spin1,oblique,spin2))

        if not macros:
            continue
        # A clear reorientation window means at least one upstream anchor
        # admits collision-free S-O-S primitives. Count windows before the
        # more expensive downstream bounded suffix search begins.
        clear_window_count+=1
        suffix_seed=events[qidx:]
        suffix_cache={}
        for (p_s,prefix,event_idx,p_clearance,first_yaw,qyaw,
             spin1,oblique,spin2) in macros:
            attempts+=1
            key=round(wrap(qyaw),4)
            if key not in suffix_cache:
                if abs(wrap(qyaw-qbase.th))<1e-6:
                    # Returning to the original event pose makes the original
                    # suffix an exact, already swept-safe seed. Other headings
                    # need an actual bounded search from the new pose.
                    suffix_cache[key]=(suffix_seed,0)
                else:
                    local_start=Pose(qbase.x,qbase.y,qyaw)
                    suffix_cache[key]=bounded_research(
                        local_start,goal,suffix_seed,grid,xy_tol,yaw_tol,
                        max_iter=max_iter,padding=padding,initial_mode=2,
                        anytime=True,weights=weights,
                        allow_oblique=any(a.mode==1 and abs(a.amount)>1e-8
                                          for a in suffix_seed))
                    searches+=1
            suffix=suffix_cache[key]
            if suffix is None:
                continue
            suffix_successes+=1
            # Compare the route segment that this candidate actually replaces:
            # the original motion from P to Q plus its downstream suffix,
            # against the S-O-S macro plus the replanned suffix. Comparing
            # only after Q misses steering and mode changes removed between
            # P and Q, which can reject the very upstream replan being tested.
            event_begin=path_s[event_idx]
            event_end=path_s[event_idx+1]
            remaining_fraction=(event_end-p_s)/max(1e-9,event_end-event_begin)
            remaining_fraction=max(0.0,min(1.0,remaining_fraction))
            replaced_segment=[]
            if remaining_fraction>1e-8:
                event=events[event_idx]
                replaced_segment.append(Action(
                    event.mode,event.amount*remaining_fraction,event.control))
            replaced_segment.extend(events[event_idx+1:qidx])
            replaced_segment.extend(suffix_seed)
            prior_mode=prefix[-1].mode if prefix else None
            before_maneuvers,before_steering=local_metrics(
                replaced_segment,prior_mode)
            candidate_segment=[spin1,oblique,spin2]+suffix[0]
            after_maneuvers,after_steering=local_metrics(
                candidate_segment,prior_mode)
            maneuver_reduction=before_maneuvers-after_maneuvers
            steering_reduction=before_steering-after_steering
            proposal=collapse_exact_runs(
                prefix+[spin1,oblique,spin2]+suffix[0])
            if (sequence_cost(proposal,weights)["direction_switches"]>0 or
                    has_reverse_motion(proposal)):
                continue
            score=sequence_cost(proposal,weights)["score"]
            best_maneuver_reduction=max(
                best_maneuver_reduction,maneuver_reduction)
            best_steering_reduction=max(
                best_steering_reduction,steering_reduction)
            has_simpler_suffix=(maneuver_reduction>0 or
                                steering_reduction>=.5)
            # Keep a candidate if it improves the common objective even when
            # the equal-length suffix has the same event count. If it spends
            # score slack, require a measurable downstream simplification.
            if not has_simpler_suffix and score>=incumbent_score-1e-7:
                complexity_rejected+=1
                continue
            best_score=min(best_score,score)
            if score<incumbent_score-1e-7:
                score_improved+=1
            if score>incumbent_score+score_slack+1e-7:
                score_rejected+=1
                continue
            complexity_improved+=1
            score_delta=max(0.0,score-incumbent_score)
            selection_bonus=min(score_delta+1e-3,
                switch_bonus*max(0,maneuver_reduction)+
                .2*max(0.0,steering_reduction))
            dense_poses,dense_actions=expand(
                start,proposal,max_translation=.25,
                max_rotation=math.radians(7.5))
            if not grid.trajectory_clear(dense_poses,dense_actions):
                sweep_rejected+=1
                continue
            end=dense_poses[-1]
            if (math.hypot(end.x-goal.x,end.y-goal.y)>xy_tol+1e-5 or
                    abs(wrap(end.th-goal.th))>yaw_tol+1e-5):
                endpoint_rejected+=1
                continue
            safe_count+=1
            found.append((score,proposal,dense_poses,{
                "solver":"upstream_sos_local_replan",
                "common_score":score,
                "sos_prefix_anchor_s":p_s,
                "sos_suffix_anchor":qidx,
                "sos_upstream_distance":path_s[qidx]-p_s,
                "sos_upstream_clearance":p_clearance,
                "sos_first_heading":first_yaw,
                "sos_suffix_heading":qyaw,
                "sos_replan_nodes":suffix[1],
                "sos_hotspot_maneuvers_before":before_maneuvers,
                "sos_hotspot_maneuvers_after":after_maneuvers,
                "sos_hotspot_steering_tv_before":before_steering,
                "sos_hotspot_steering_tv_after":after_steering,
                "sos_selection_bonus":selection_bonus}))
            if len(found)>=max_candidates:
                break
        if len(found)>=max_candidates:
            break
    found.sort(key=lambda item:item[0])
    return found[:max_candidates],{
        "sos_attempts":attempts,"sos_searches":searches,
        "sos_suffix_successes":suffix_successes,
        "sos_score_improved":score_improved,
        "sos_complexity_improved":complexity_improved,
        "sos_best_maneuver_reduction":best_maneuver_reduction,
        "sos_best_steering_reduction":best_steering_reduction,
        "sos_complexity_rejected":complexity_rejected,
        "sos_score_rejected":score_rejected,
        "sos_sweep_rejected":sweep_rejected,
        "sos_endpoint_rejected":endpoint_rejected,
        "sos_clear_window_count":clear_window_count,
        "sos_clearance_rejected_windows":clearance_rejected,
        "sos_min_upstream_clearance":min(upstream_clearances)
            if upstream_clearances else 0.0,
        "sos_max_upstream_clearance":max(upstream_clearances)
            if upstream_clearances else 0.0,
        "sos_safe":safe_count,"sos_best_score":best_score}


def upstream_suffix_replan_candidates(start, goal, seed_actions, grid,
                                      hotspot_idx,
                                      incumbent_score=float("inf"),
                                      max_iter=300, padding=.8, xy_tol=.16,
                                      yaw_tol=.14, upstream_distance=4.0,
                                      score_slack=.75,
                                      min_upstream_clearance=.0,
                                      max_searches=3,
                                      weights=None):
    """Replan from a high-clearance point before a maneuver hotspot.

    A high-clearance upstream anchor is connected to a nearby post-hotspot
    event pose. The local segment compares pure A, A-O-A, and early Spin+A;
    the original route after that event remains an exact suffix. Candidates
    still require shared score, endpoint checks, forward motion, and swept SAT.
    """
    events=collapse_exact_runs(seed_actions)
    if len(events)<2:
        return [],{"upstream_replan_windows":0,
                   "upstream_replan_searches":0,
                   "upstream_replan_search_failures":0,
                   "upstream_replan_safe":0}
    poses=replay(start,events)
    hotspot_idx=max(1,min(int(hotspot_idx),len(events)-1))
    path_s=[0.0]
    for action in events:
        path_s.append(path_s[-1]+(
            abs(action.amount) if action.mode!=2 else 0.0))
    cuts=[];clearance_rejected=0
    for wanted in (.6,1.3,2.1,3.0,3.8):
        distance=min(float(wanted),float(upstream_distance))
        target_s=path_s[hotspot_idx]-distance
        # If the hotspot lies within the upstream horizon, retain the route
        # start as a legitimate replan anchor instead of discarding every
        # farther sample. This is useful when the start is the nearest wide
        # patch before an early turn; swept checks still decide Spin safety.
        if target_s<path_s[0]:
            target_s=path_s[0]
            distance=path_s[hotspot_idx]-target_s
        if distance<.45:
            continue
        for event_idx,action in enumerate(events[:hotspot_idx]):
            if action.mode==2:
                continue
            begin,end=path_s[event_idx],path_s[event_idx+1]
            if not (begin-1e-8<=target_s<=end+1e-8 and end-begin>1e-8):
                continue
            fraction=max(0.0,min(1.0,(target_s-begin)/(end-begin)))
            if fraction<=1e-8:
                prefix=events[:event_idx]
                p=poses[event_idx]
                remainder=events[event_idx:]
                remainder_start=event_idx
            elif fraction>=1-1e-8:
                prefix=events[:event_idx+1]
                p=poses[event_idx+1]
                remainder=events[event_idx+1:]
                remainder_start=event_idx+1
            else:
                partial=Action(action.mode,action.amount*fraction,
                               action.control)
                prefix=events[:event_idx]+[partial]
                p=advance(poses[event_idx],partial)
                remaining=Action(action.mode,action.amount*(1-fraction),
                                 action.control)
                remainder=([remaining] if abs(remaining.amount)>1e-8 else [])
                remainder.extend(events[event_idx+1:])
                remainder_start=(event_idx if remainder and
                    abs(remainder[0].amount-remaining.amount)<1e-8
                    else event_idx+1)
            clearance=grid.center_clearance_lower_bound(p.x,p.y)
            if clearance+1e-9<min_upstream_clearance:
                clearance_rejected+=1
            else:
                key=round(target_s,3)
                if all(abs(key-old[0])>0.05 for old in cuts):
                    cuts.append((key,distance,prefix,p,event_idx,remainder,
                                 clearance,remainder_start))
            break
    cuts.sort(key=lambda c:(-c[6],c[1]))
    if not cuts:
        return [],{"upstream_replan_windows":0,
                   "upstream_replan_clearance_rejected":clearance_rejected,
                   "upstream_replan_searches":0,
                   "upstream_replan_search_failures":0,
                   "upstream_replan_safe":0,
                   "upstream_replan_best_clearance":0.0}
    # Try the best-clearance anchor first; a second point is available only
    # when the configured search budget leaves room for it.
    cuts=cuts[:2]
    q_indices=[]
    for idx in (hotspot_idx+1,hotspot_idx+2):
        if (idx<=len(events) and
                path_s[idx]-path_s[hotspot_idx]<=4.0 and
                idx not in q_indices):
            q_indices.append(idx)
    if not q_indices:
        q_indices=[len(events)]
    candidates=[];seen=set();attempts=searches=search_failures=0
    pattern_attempts={"pure_a":0,"a_o_a":0,"spin_a":0}
    pattern_hits={"pure_a":0,"a_o_a":0,"spin_a":0}
    pattern_rejects=score_rejects=sweep_rejects=endpoint_rejects=0
    best_score=float("inf");best_maneuver_reduction=0
    best_steering_reduction=0.0

    def local_metrics(actions,initial_mode=None,horizon=4.0):
        window=[];distance=0.0
        for action in actions:
            if window and distance>=horizon:
                break
            window.append(action)
            if action.mode!=2:
                distance+=abs(action.amount)
        mode_events=sum(a.mode!=b.mode for a,b in zip(window,window[1:]))
        if window and initial_mode is not None and window[0].mode!=initial_mode:
            mode_events+=1
        gear_events=sum(a.amount*b.amount<0 for a,b in zip(window,window[1:])
                        if a.mode!=2 and b.mode!=2)
        steering_tv=sum(abs(b.control-a.control)
                        for a,b in zip(window,window[1:])
                        if a.mode==b.mode and a.mode in (0,1))
        return mode_events+gear_events,steering_tv

    def has_aoa_pattern(actions):
        word=[]
        for action in collapse_exact_runs(actions):
            if action.mode!=2 and (not word or word[-1]!=action.mode):
                word.append(action.mode)
        return word==[0,1,0] and not any(a.mode==2 for a in actions)

    for _,distance,prefix,p,event_idx,remainder,p_clearance,remainder_start in cuts:
      prior_mode=prefix[-1].mode if prefix else 0
      for qidx in q_indices:
        if attempts>=max_searches:
            break
        if qidx<=remainder_start:
            continue
        local_count=qidx-remainder_start
        local_seed=remainder[:local_count]
        if not local_seed:
            continue
        local_goal=poses[qidx]
        tail=events[qidx:]
        spin_heading=None
        for offset,action in enumerate(events[hotspot_idx:qidx]):
            if action.mode==2 or abs(action.amount)<1e-8:
                continue
            spin_heading=wrap(
                poses[hotspot_idx+offset].th+
                (action.control if action.mode==1 else 0.0)+
                (math.pi if action.amount<0 else 0.0))
            break
        if spin_heading is None:
            spin_heading=wrap(math.atan2(local_goal.y-p.y,
                                         local_goal.x-p.x))
        specs=[("pure_a",None,False),
               ("a_o_a",None,True),
               ("spin_a",spin_heading,False)]
        for pattern,desired_heading,allow_oblique in specs:
            if attempts>=max_searches:
                break
            attempts+=1;pattern_attempts[pattern]+=1
            pre=[];local_start=p;initial_mode=0
            if desired_heading is not None:
                spin=Action(2,wrap(desired_heading-p.th),0.0)
                if abs(spin.amount)<math.radians(8) or not grid.action_clear(
                        p,spin,.035):
                    pattern_rejects+=1
                    continue
                pre=[spin];local_start=advance(p,spin)
            searches+=1
            suffix=bounded_research(
                local_start,local_goal,local_seed,grid,.002,.002,
                max_iter=max_iter,padding=padding,initial_mode=initial_mode,
                anytime=True,weights=weights,allow_oblique=allow_oblique,
                mode_schedule=((0,1,0) if pattern=="a_o_a" else (0,)))
            if suffix is None:
                search_failures+=1
                continue
            suffix_actions=suffix[0]
            if has_reverse_motion(suffix_actions):
                pattern_rejects+=1
                continue
            if pattern in ("pure_a","spin_a"):
                if not suffix_actions or any(a.mode!=0 for a in suffix_actions):
                    pattern_rejects+=1
                    continue
            elif not has_aoa_pattern(suffix_actions):
                pattern_rejects+=1
                continue
            pattern_hits[pattern]+=1
            proposal=collapse_exact_runs(prefix+pre+suffix_actions+tail)
            if (sequence_cost(proposal,weights)["direction_switches"]>0 or
                    has_reverse_motion(proposal)):
                pattern_rejects+=1
                continue
            key=tuple((a.mode,round(a.amount,3),round(a.control,3))
                      for a in proposal)
            if key in seen:
                continue
            seen.add(key)
            before_maneuvers,before_steering=local_metrics(local_seed,prior_mode)
            after_segment=pre+suffix_actions
            after_maneuvers,after_steering=local_metrics(
                after_segment,prior_mode)
            maneuver_reduction=before_maneuvers-after_maneuvers
            steering_reduction=before_steering-after_steering
            best_maneuver_reduction=max(best_maneuver_reduction,
                                        maneuver_reduction)
            best_steering_reduction=max(best_steering_reduction,
                                        steering_reduction)
            score=sequence_cost(proposal,weights)["score"]
            best_score=min(best_score,score)
            simpler=(maneuver_reduction>0 or steering_reduction>=.5)
            if score>=incumbent_score-1e-7 and not simpler:
                score_rejects+=1
                continue
            if score>incumbent_score+score_slack+1e-7:
                score_rejects+=1
                continue
            dense_poses,dense_actions=expand(
                start,proposal,max_translation=.25,
                max_rotation=math.radians(7.5))
            if not grid.trajectory_clear(dense_poses,dense_actions):
                sweep_rejects+=1
                continue
            end=dense_poses[-1]
            if (math.hypot(end.x-goal.x,end.y-goal.y)>xy_tol+1e-5 or
                    abs(wrap(end.th-goal.th))>yaw_tol+1e-5):
                endpoint_rejects+=1
                continue
            candidates.append((score,proposal,dense_poses,{
                "solver":"upstream_suffix_replan",
                "common_score":score,
                "upstream_replan_pattern":pattern,
                "upstream_replan_anchor_s":path_s[hotspot_idx]-distance,
                "upstream_replan_distance":distance,
                "upstream_replan_clearance":p_clearance,
                "upstream_replan_hotspot":hotspot_idx,
                "upstream_replan_suffix_start":qidx,
                "upstream_replan_goal_horizon":path_s[-1]-path_s[qidx],
                "upstream_replan_nodes":suffix[1],
                "upstream_replan_maneuvers_before":before_maneuvers,
                "upstream_replan_maneuvers_after":after_maneuvers,
                "upstream_replan_steering_tv_before":before_steering,
                "upstream_replan_steering_tv_after":after_steering}))
    candidates.sort(key=lambda item:item[0])
    return candidates,{"upstream_replan_windows":len(cuts),
        "upstream_replan_clearance_rejected":clearance_rejected,
        "upstream_replan_searches":searches,
        "upstream_replan_search_failures":search_failures,
        "upstream_replan_attempts":attempts,
        "upstream_replan_safe":len(candidates),
        "upstream_replan_pure_a_attempts":pattern_attempts["pure_a"],
        "upstream_replan_a_o_a_attempts":pattern_attempts["a_o_a"],
        "upstream_replan_spin_a_attempts":pattern_attempts["spin_a"],
        "upstream_replan_pure_a_hits":pattern_hits["pure_a"],
        "upstream_replan_a_o_a_hits":pattern_hits["a_o_a"],
        "upstream_replan_spin_a_hits":pattern_hits["spin_a"],
        "upstream_replan_pattern_rejects":pattern_rejects,
        "upstream_replan_score_rejects":score_rejects,
        "upstream_replan_sweep_rejects":sweep_rejects,
        "upstream_replan_endpoint_rejects":endpoint_rejects,
        "upstream_replan_best_clearance":max(c[6] for c in cuts),
        "upstream_replan_best_score":best_score,
        "upstream_replan_best_maneuver_reduction":best_maneuver_reduction,
        "upstream_replan_best_steering_reduction":best_steering_reduction}


def equivalent_mode_sequences(start, seed_actions, max_candidates=8,
                              beam_width=48, fit_tolerance=.003,
                              weights=None):
    """Generate low-cost mode schedules that exactly preserve seed knot poses.

    This is a discrete master over *equivalent local motions*, not a mode
    state machine: all three vehicle models are tested against each seed edge,
    infeasible fits are discarded, and a bounded beam retains cost-diverse
    schedules for the continuous corridor optimizer to refine.
    """
    seed,refs=_collapse_for_joint(start,seed_actions)
    if not seed:
        return []
    beam=[[]]
    for i,(p,q) in enumerate(zip(refs,refs[1:])):
        options=[]
        for mode in (0,1,2):
            action=fit_action_to_pose(mode,p,q,fit_tolerance)
            if (action is not None and
                    (action.mode==2 or action.amount>=-1e-10)):
                options.append(action)
        original=seed[i]
        if not any(a.mode==original.mode and
                   abs(a.amount-original.amount)<1e-6 and
                   abs(a.control-original.control)<1e-6 for a in options):
            options.append(original)
        expanded=[prefix+[action] for prefix in beam for action in options]
        expanded.sort(key=lambda seq: sequence_cost(seq,weights)["score"])
        # Keep distinct mode/control schedules so nearby alternatives do not
        # consume the entire beam with duplicate prefixes.
        unique=[]
        keys=set()
        for seq in expanded:
            key=tuple((a.mode,1 if a.amount>=0 else -1,
                       round(a.control,3)) for a in seq)
            if key in keys:
                continue
            keys.add(key);unique.append(seq)
            if len(unique)>=beam_width:
                break
        beam=unique
    result=[];seen=set()
    for seq in sorted(beam,key=lambda a:sequence_cost(a,weights)["score"]):
        key=tuple((a.mode,round(a.amount,3),round(a.control,3)) for a in seq)
        if key in seen:
            continue
        seen.add(key)
        result.append((seq,sequence_cost(seq,weights)["score"]))
        if len(result)>=max_candidates:
            break
    # The source schedule is always retained even if near-duplicate rounding
    # caused it to fall behind the beam cap.
    seed_key=tuple((a.mode,round(a.amount,3),round(a.control,3)) for a in seed)
    if not any(tuple((a.mode,round(a.amount,3),round(a.control,3))
                     for a in seq)==seed_key for seq,_ in result):
        if len(result)>=max_candidates:
            result.pop()
        result.append((seed,sequence_cost(seed,weights)["score"]))
        result.sort(key=lambda item:item[1])
    return result


def anchor_mode_sequences(start, seed_actions, grid, max_candidates=2,
                          max_span=3, beam_width=12, max_edge_length=1.6,
                          weights=None, profile=None):
    """Search local mode schedules and switch locations on a swept-safe anchor path.

    Unlike ``equivalent_mode_sequences``, an edge may skip intermediate seed
    anchors. The selected A/O/Spin primitive must exactly connect the two
    retained center poses and pass full-footprint swept SAT. Thus the mode
    boundary can move and the inter-anchor geometry can change without running
    a high-dimensional MINLP over every search sample.
    """
    profile=profile if profile is not None else {}
    total_begin=time.perf_counter()
    events=collapse_exact_runs(seed_actions)
    if not events or has_reverse_motion(events):
        return []
    stage_begin=time.perf_counter()
    refs,primitives=expand(start,events,max_translation=.50,
                           max_rotation=math.radians(15))
    profile["expand_s"]=time.perf_counter()-stage_begin
    stage_begin=time.perf_counter()
    if not grid.trajectory_clear(refs,primitives):
        return []
    profile["seed_sweep_s"]=time.perf_counter()-stage_begin
    # Beam elements are action prefixes; geometric anchors are path vertices.
    beams=[[] for _ in refs]
    beams[0]=[[]]
    edge_seconds=score_seconds=0.0
    for i,p in enumerate(refs[:-1]):
        if not beams[i]:
            continue
        jmax=min(len(refs)-1,i+max(1,int(max_span)))
        proposals={j:[] for j in range(i+1,jmax+1)}
        # Edge feasibility depends only on its endpoint pair and mode; do the
        # comparatively expensive swept SAT once, then reuse it for all beam
        # prefixes reaching this anchor.
        edge_options={}
        edge_begin=time.perf_counter()
        for j in range(i+1,jmax+1):
            q=refs[j];options=[]
            if math.hypot(q.x-p.x,q.y-p.y)<=max_edge_length+1e-9:
                for mode in (0,1,2):
                    action=fit_action_to_pose(mode,p,q,tolerance=.005)
                    if action is None:
                        continue
                    if action.mode!=2 and action.amount<0.0:
                        continue
                    if mode==2 and abs(action.amount)>math.pi+1e-9:
                        continue
                    if mode!=2 and abs(action.amount)>max_edge_length+1e-9:
                        continue
                    if grid.action_clear(p,action):
                        options.append(action)
            edge_options[j]=options
        edge_seconds+=time.perf_counter()-edge_begin
        score_begin=time.perf_counter()
        for prefix in beams[i]:
            for j,options in edge_options.items():
                for action in options:
                    seq=prefix+[action]
                    proposals[j].append((sequence_cost(seq,weights)["score"],seq))
        for j,candidates in proposals.items():
            candidates.extend((sequence_cost(seq,weights)["score"],seq)
                              for seq in beams[j])
            candidates.sort(key=lambda item:item[0])
            unique=[];seen=set();per_signature={}
            for score,seq in candidates:
                key=tuple((a.mode,1 if a.amount>=0 else -1,
                           round(a.amount,3),round(a.control,3)) for a in seq)
                if key in seen:
                    continue
                seen.add(key)
                last=seq[-1]
                runs=[]
                for action in seq:
                    if not runs or runs[-1][0]!=action.mode:
                        runs.append([action.mode,abs(action.amount)])
                    else:
                        runs[-1][1]+=abs(action.amount)
                # Preserve different switch placements as well as different
                # mode words; bin run lengths coarsely to retain alternatives
                # without letting tiny anchor shifts fill the beam.
                run_signature=tuple((mode,round(length/(.30 if mode==2 else .60)))
                                    for mode,length in runs)
                signature=(run_signature,last.mode,
                           1 if last.amount>=0 else -1,round(last.control/.15))
                if per_signature.get(signature,0)>=2:
                    continue
                per_signature[signature]=per_signature.get(signature,0)+1
                unique.append(seq)
                if len(unique)>=beam_width:
                    break
            beams[j]=unique
        score_seconds+=time.perf_counter()-score_begin
    profile["edge_fit_sweep_s"]=edge_seconds
    profile["beam_score_s"]=score_seconds
    result_begin=time.perf_counter()
    result=[];seen=set()
    for seq in sorted(beams[-1],key=lambda actions:sequence_cost(actions,weights)["score"]):
        key=tuple((a.mode,1 if a.amount>=0 else -1,
                   round(a.amount,3),round(a.control,3)) for a in seq)
        if key in seen:
            continue
        seen.add(key)
        result.append((seq,sequence_cost(seq,weights)["score"]))
        if len(result)>=max_candidates:
            break
    profile["result_s"]=time.perf_counter()-result_begin
    profile["total_s"]=time.perf_counter()-total_begin
    return result


def iterated_mode_geometry_refine(start, goal, seed_actions, grid,
                                  incumbent_actions=None, xy_tol=.16,
                                  yaw_tol=.14, rounds=2, max_candidates=3,
                                  max_refinements_per_round=1,
                                  proposal_slack=.25,
                                  min_round_improvement=.0025, max_iter=100,
                                  weights=None, profile=None):
    """Alternate local mode/anchor search with continuous trajectory refinement.

    Each round rebuilds the bounded A/O/Spin anchor graph around the latest
    incumbent geometry. The graph may move mode boundaries and replace the
    geometry between retained anchors; the fixed-mode NLP then adjusts the
    resulting continuous controls. Only exact-endpoint, swept-SAT-safe and
    strictly non-worsening candidates become the next round's incumbent.

    This is a bounded local discrete-continuous coordination method, not a
    global mixed-integer optimum. The caller's incumbent is never discarded.
    """
    profile=profile if profile is not None else {}
    profile["joint_rounds_requested"]=max(0,int(rounds))
    profile["joint_rounds_accepted"]=0
    profile["joint_graph_candidates"]=0
    profile["joint_nlp_attempts"]=0
    profile["joint_nlp_accepts"]=0
    profile["joint_iteration_trace"]=[]
    profile["joint_early_stop_low_gain"]=0.0
    begin=time.perf_counter()
    incumbent=collapse_exact_runs(
        incumbent_actions if incumbent_actions is not None else seed_actions)
    anchor_seed=collapse_exact_runs(seed_actions)
    if has_reverse_motion(incumbent) or has_reverse_motion(anchor_seed):
        raise ValueError("iterated_joint_reverse_motion_forbidden")
    poses=replay(start,incumbent)
    if (not incumbent or not grid.trajectory_clear(poses,incumbent) or
            math.hypot(poses[-1].x-goal.x,poses[-1].y-goal.y)>xy_tol+1e-5 or
            abs(wrap(poses[-1].th-goal.th))>yaw_tol+1e-5):
        raise ValueError("iterated_joint_incumbent_invalid")
    incumbent_score=sequence_cost(incumbent,weights)["score"]
    incumbent_boxes=[]
    incumbent_corridor=None
    visited=set()

    def signature(actions):
        return tuple((a.mode,1 if a.amount>=0 else -1,
                      round(a.amount,4),round(a.control,4)) for a in actions)

    visited.add(signature(incumbent))
    for round_index in range(max(0,int(rounds))):
        round_begin=time.perf_counter()
        graph_profile={}
        alternatives=anchor_mode_sequences(
            start,anchor_seed,grid,max_candidates=max_candidates,
            max_span=3,beam_width=24,weights=weights,profile=graph_profile)
        profile["joint_graph_candidates"]+=len(alternatives)
        round_best_score=incumbent_score
        round_best_actions=incumbent
        round_best_poses=poses
        round_best_boxes=incumbent_boxes
        round_best_corridor=incumbent_corridor
        attempted=0
        accepted_solver="none"
        for schedule,graph_score in alternatives:
            schedule=collapse_exact_runs(schedule)
            key=signature(schedule)
            if key in visited:
                continue
            visited.add(key)
            if graph_score>incumbent_score+proposal_slack:
                continue
            proposal_poses=replay(start,schedule)
            if (math.hypot(proposal_poses[-1].x-goal.x,
                           proposal_poses[-1].y-goal.y)>xy_tol+1e-5 or
                    abs(wrap(proposal_poses[-1].th-goal.th))>yaw_tol+1e-5 or
                    not grid.trajectory_clear(proposal_poses,schedule)):
                continue
            # The graph primitive itself is an exact, independently swept
            # candidate. Keep it as a safe option while asking the continuous
            # subproblem whether its geometry can improve further.
            if graph_score<round_best_score-1e-7:
                round_best_score=graph_score
                round_best_actions=schedule
                round_best_poses=proposal_poses
                round_best_boxes=[]
                round_best_corridor=None
                accepted_solver="anchor_graph"
            if attempted>=max_refinements_per_round:
                continue
            attempted+=1
            profile["joint_nlp_attempts"]+=1
            try:
                refined_poses,refined_actions,boxes,meta=refine(
                    start,goal,schedule,grid,xy_tol,yaw_tol,
                    max_iter=max_iter,weights=weights)
                refined_score=sequence_cost(refined_actions,weights)["score"]
                if (refined_score<round_best_score-1e-7 and
                        grid.trajectory_clear(refined_poses,refined_actions)):
                    round_best_score=refined_score
                    round_best_actions=collapse_exact_runs(refined_actions)
                    round_best_poses=replay(start,round_best_actions)
                    round_best_boxes=boxes
                    round_best_corridor=meta.get("corridor")
                    accepted_solver="anchor_graph+ipopt"
                    profile["joint_nlp_accepts"]+=1
            except Exception as exc:
                profile.setdefault("joint_nlp_errors",[]).append(
                    str(exc).splitlines()[-1])
            # A single refined proposal per round bounds latency. Alternatives
            # still remain available to the caller's ordinary candidate set.
            if attempted>=max_refinements_per_round:
                break
        improved=round_best_score<incumbent_score-1e-7
        relative_gain=((incumbent_score-round_best_score)/
                       max(abs(incumbent_score),1e-9)) if improved else 0.0
        profile["joint_iteration_trace"].append({
            "round":round_index+1,"seed_score":incumbent_score,
            "graph_candidates":len(alternatives),"nlp_attempts":attempted,
            "result_score":round_best_score,"accepted":improved,
            "relative_gain":relative_gain,
            "source":accepted_solver,"graph_time":graph_profile.get("total_s",0.0),
            "round_time":time.perf_counter()-round_begin})
        if not improved:
            break
        incumbent_score=round_best_score
        incumbent=round_best_actions
        poses=round_best_poses
        incumbent_boxes=round_best_boxes
        incumbent_corridor=round_best_corridor
        anchor_seed=incumbent
        profile["joint_rounds_accepted"]+=1
        # Keep a small accepted improvement, but avoid paying for another full
        # anchor-graph/NLP round once progress is already below the configured
        # relative threshold. This is a continuation criterion, not an
        # acceptance threshold; every candidate still needs strict score
        # descent, endpoint agreement, and swept-SAT validation above.
        if (round_index+1<max(0,int(rounds)) and
                relative_gain<max(0.0,float(min_round_improvement))):
            profile["joint_early_stop_low_gain"]=1.0
            break
    profile["joint_total_s"]=time.perf_counter()-begin
    if profile["joint_rounds_accepted"] and not incumbent_boxes:
        _,incumbent_boxes,covered=build_corridors(grid,poses)
        incumbent_corridor="disc" if covered else "mixed_disc_point"
    if incumbent_corridor is None:
        _,incumbent_boxes,covered=build_corridors(grid,poses)
        incumbent_corridor="disc" if covered else "mixed_disc_point"
    meta={"solver":"iterated_mode_geometry",
          "corridor":incumbent_corridor,
          "score":incumbent_score,
          "rounds_accepted":profile["joint_rounds_accepted"],
          "trace":profile["joint_iteration_trace"],
          "profile":profile}
    return poses,incumbent,incumbent_boxes,meta


def joint_refine(start, goal, seed_actions, grid, xy_tol=.16, yaw_tol=.14,
                 max_translation=2.0, max_spin=math.pi,
                 max_knots=32, time_limit=5.0,
                 terminal_xy_tol=.015, terminal_yaw_tol=.02,
                 terminal_xy_weight=100.0, terminal_yaw_weight=10.0,
                 weights=None, algorithm="B-BB",
                 variable_selection="most-fractional",
                 node_comparison="depth-first",
                 subproblem_time_limit=4.0, bonmin_iteration_limit=0,
                 bonmin_subproblem_iteration_limit=600,
                 bonmin_unsolved_nlp_retries=0,
                 collision_retries=0,
                 collision_repair_iterations=0,
                 collision_repair_time_limit=2.0,
                 warm_start_time_limit=6.0,
                 warm_start_early_accept_improvement=0.0,
                 use_seed_cutoff=True,
                 collision_spacing=.75,
                 adaptive_collision_refinement=False,
                 corridor_clearance_slack=0.0,
                 use_feasibility_pump=True,
                 use_minlp_pump=True,
                 root_resolves=0,
                 dive_fractional=False,
                 random_seed=0,
                 use_feasible_seed_warm_start=False,
                 warm_start_collision_retries=2,
                 use_rins_heuristic=False,
                 use_cbc_seed_incumbent=False,
                 ackermann_merge_tolerance=1e-5,
                 oblique_merge_tolerance=math.radians(.25),
                 _forbidden_schedules=(),_collision_planes=(),
                 _fixed_schedule=None,_initial_actions=None,
                 geometry_yaw_bounds=False,
                 geometry_spin_pruning=False,
                 geometry_oblique_bounds=False):
    """Corridor-constrained MINLP that jointly selects A/O/S and geometry.

    The seed is only a trust-region/corridor initializer. Binary mode
    variables are solved with continuous center poses and controls by Bonmin;
    one route-level travel direction is shared by all translation segments.
    Every integer solution is replayed with the exact A/O/S equations and
    rejected unless independent swept OBB checks and terminal tolerances pass.
    """
    import casadi as ca
    call_begin=time.perf_counter()
    bonmin_seed_incumbent_status="not_requested"
    if not seed_actions:
        raise ValueError("joint_empty_seed")
    if str(algorithm)=="B-Hyb":
        # Bonmin exposes nlp_failure_behavior only for B-BB. This planner's
        # mixed-mode trajectory NLP is nonconvex, and B-Hyb can let an
        # unsolved Ipopt node escape CasADi as an uncaught C++ exception,
        # terminating the optimizer process. Fail closed before building the
        # model so the caller can preserve its already-safe local plan.
        raise ValueError("joint_unsupported_algorithm_for_nonconvex_nlp:B-Hyb")
    if str(node_comparison) not in (
            "best-bound","depth-first","breadth-first","dynamic",
            "best-guess"):
        raise ValueError("joint_invalid_node_comparison:%s"%node_comparison)
    bonmin_unsolved_nlp_retries=min(
        3,max(0,int(bonmin_unsolved_nlp_retries)))
    joint_xy_tol=min(float(xy_tol),max(1e-4,float(terminal_xy_tol)))
    joint_yaw_tol=min(float(yaw_tol),max(1e-4,float(terminal_yaw_tol)))
    terminal_solver_slack=5e-4
    seed_full,actions_full=expand(start,seed_actions)
    if not grid.trajectory_clear(seed_full,actions_full):
        raise ValueError("joint_seed_sat_collision")
    seed_end=replay(start,seed_actions)[-1]
    if (math.hypot(seed_end.x-goal.x,seed_end.y-goal.y)>
            joint_xy_tol+terminal_solver_slack or
            abs(wrap(seed_end.th-goal.th))>
            joint_yaw_tol+terminal_solver_slack):
        # Local optimizers accept the planner's wider endpoint tolerance.
        # Refit the terminal primitive before building the MINLP corridor so
        # Bonmin and its fixed-schedule warm start receive a tight endpoint.
        replace_prefix=seed_actions[:-1]
        replace_end=replay(start,replace_prefix)[-1]
        append_prefix=list(seed_actions)
        append_end=seed_end
        terminal_modes=[seed_actions[-1].mode]+[
            mode for mode in (0,1,2) if mode!=seed_actions[-1].mode]
        projected=[]
        for terminal_mode in terminal_modes:
            for prefix,prefix_end in ((replace_prefix,replace_end),
                                      (append_prefix,append_end)):
                terminal=fit_action_to_pose(
                    terminal_mode,prefix_end,goal,tolerance=.002)
                if (terminal is None or
                        (terminal_mode==2 and
                         abs(terminal.amount)>max_spin+1e-9)):
                    continue
                candidate=prefix+[terminal]
                candidate_poses=replay(start,candidate)
                end=candidate_poses[-1]
                if (math.hypot(end.x-goal.x,end.y-goal.y)>
                        joint_xy_tol+terminal_solver_slack or
                        abs(wrap(end.th-goal.th))>
                        joint_yaw_tol+terminal_solver_slack or
                        not grid.trajectory_clear(candidate_poses,candidate,
                                                  .0025)):
                    continue
                projected.append((sequence_cost(candidate,weights)["score"],
                                  candidate))
        if projected:
            seed_actions=min(projected,key=lambda item:item[0])[1]
            seed_full,actions_full=expand(start,seed_actions)
    actions,refs=_collapse_for_joint(
        start,seed_actions,max_translation,max_spin,
        ackermann_merge_tolerance,oblique_merge_tolerance)
    n=len(actions)
    if n==0 or n>max_knots:
        raise ValueError("joint_horizon_out_of_range:%d"%n)
    initial_actions=(actions if _initial_actions is None
                     else list(_initial_actions))
    if len(initial_actions)!=n:
        raise ValueError("joint_initial_action_horizon_mismatch")
    fixed_schedule=(None if _fixed_schedule is None else
                    tuple((int(mode),int(gear))
                          for mode,gear in _fixed_schedule))
    if fixed_schedule is not None and len(fixed_schedule)!=n:
        raise ValueError("joint_fixed_schedule_horizon_mismatch")
    if fixed_schedule is not None and any(gear not in (0,1) for _,gear in fixed_schedule):
        raise ValueError("joint_fixed_schedule_invalid_gear")
    joint_sweep_step=.0025
    single_knot_modes=None
    if n==1:
        fits=[]
        for candidate_mode in (0,1,2):
            fitted=fit_action_to_pose(candidate_mode,start,goal,
                                      tolerance=max(joint_xy_tol,
                                                    joint_yaw_tol))
            if fitted is not None:
                fits.append(fitted)
        if fixed_schedule is not None:
            fixed_mode,fixed_gear=fixed_schedule[0]
            fits=[action for action in fits
                  if action.mode==fixed_mode and
                  (action.mode==2 or
                   int(action.amount>=0)==fixed_gear)]
        if not fits:
            raise ValueError("joint_single_knot_goal_unreachable")
        single_knot_modes={action.mode for action in fits}
        # Give Bonmin a feasible integer/continuous root point. The original
        # seed can deliberately use a mode that cannot reach the requested
        # goal (the global mode decision is expected to replace it).
        actions=[min(fits,key=lambda action:
                     sequence_cost([action],weights)["score"])]
        refs=[start,advance(start,actions[0])]
        candidate_poses=replay(start,actions)
        candidate_end=candidate_poses[-1]
        if (grid.trajectory_clear(candidate_poses,actions,joint_sweep_step) and
                math.hypot(candidate_end.x-goal.x,
                           candidate_end.y-goal.y)<=joint_xy_tol+terminal_solver_slack and
                abs(wrap(candidate_end.th-goal.th))<=
                    joint_yaw_tol+terminal_solver_slack):
            score=sequence_cost(actions,weights)
            _,boxes,_=build_corridors(grid,candidate_poses)
            return candidate_poses,actions,boxes,{
                "solver":"joint_single_knot_exact_enumeration",
                "score":score["score"],"seed_score":sequence_cost(
                    seed_actions,weights)["score"],
                "mode_switches":score["switches"],
                "direction_switches":score["direction_switches"],
                "spin_angle":score["spin_angle"],
                "reverse_length":score["reverse_length"],
                "terminal_repaired":False,"selected_schedule":[
                    [actions[0].mode, 1 if actions[0].amount>=0 else 0]]}
    if _initial_actions is None:
        initial_actions=actions
    if len(initial_actions)!=n:
        raise ValueError("joint_initial_action_horizon_mismatch")
    # The knot-collapse helper also returns the pre-collapse group endpoints.
    # Those points can diverge from the actual collapsed constant-control
    # actions when a merge tolerance is nonzero. Center corridors on the same
    # trajectory used by the model's X and control initial values so the root
    # NLP does not start outside its own corridor constraints.
    refs=replay(start,initial_actions)
    _,seed_schedule=_joint_seed_mode_gear_schedule(actions)
    seed_poses=refs
    seed_end=seed_poses[-1]
    validated_seed_candidate=(
        n>1 and
        grid.trajectory_clear(seed_poses,initial_actions,joint_sweep_step) and
        math.hypot(seed_end.x-goal.x,seed_end.y-goal.y)<=
            joint_xy_tol+terminal_solver_slack and
        abs(wrap(seed_end.th-goal.th))<=joint_yaw_tol+terminal_solver_slack)
    warm_start_meta=None
    warm_start_result=None
    warm_start_error=None
    warm_start_wall_seconds=0.0
    # Geometry and terminal checks do not establish feasibility for every
    # discretized dynamics constraint. When requested, project the collapsed
    # seed's fixed schedule first, then let Bonmin search from that NLP point.
    if (use_feasible_seed_warm_start and fixed_schedule is None and
            _initial_actions is None and n>1):
        warm_start_begin=time.perf_counter()
        try:
            warm_result=joint_refine(
                start,goal,seed_actions,grid,xy_tol=xy_tol,yaw_tol=yaw_tol,
                max_translation=max_translation,max_spin=max_spin,
                max_knots=max_knots,
                ackermann_merge_tolerance=ackermann_merge_tolerance,
                oblique_merge_tolerance=oblique_merge_tolerance,
                time_limit=max(.1,min(6.0,float(warm_start_time_limit))),
                terminal_xy_tol=terminal_xy_tol,
                terminal_yaw_tol=terminal_yaw_tol,
                terminal_xy_weight=terminal_xy_weight,
                terminal_yaw_weight=terminal_yaw_weight,weights=weights,
                algorithm=algorithm,variable_selection=variable_selection,
                node_comparison=node_comparison,
                subproblem_time_limit=subproblem_time_limit,
                bonmin_iteration_limit=bonmin_iteration_limit,
                bonmin_subproblem_iteration_limit=(
                    bonmin_subproblem_iteration_limit),
                bonmin_unsolved_nlp_retries=bonmin_unsolved_nlp_retries,
                use_seed_cutoff=use_seed_cutoff,
                collision_spacing=collision_spacing,
                adaptive_collision_refinement=adaptive_collision_refinement,
                corridor_clearance_slack=corridor_clearance_slack,
                geometry_yaw_bounds=geometry_yaw_bounds,
                geometry_spin_pruning=geometry_spin_pruning,
                geometry_oblique_bounds=geometry_oblique_bounds,
                use_feasibility_pump=use_feasibility_pump,
                use_minlp_pump=use_minlp_pump,root_resolves=root_resolves,
                dive_fractional=dive_fractional,random_seed=random_seed,
                use_feasible_seed_warm_start=False,
                collision_retries=min(
                    max(0,int(warm_start_collision_retries)),
                    max(0,int(collision_repair_iterations))),
                collision_repair_iterations=0,
                collision_repair_time_limit=collision_repair_time_limit,
                warm_start_time_limit=warm_start_time_limit,
                warm_start_early_accept_improvement=(
                    warm_start_early_accept_improvement),
                use_rins_heuristic=use_rins_heuristic,
                _fixed_schedule=seed_schedule,_initial_actions=actions)
            if len(warm_result[1])!=n:
                raise ValueError("joint_warm_start_horizon_mismatch")
            warm_start_result=warm_result
            initial_actions=list(warm_result[1])
            warm_start_meta=warm_result[3]
        except Exception as exc:
            traceback=[]
            tb=exc.__traceback__
            while tb is not None:
                frame=tb.tb_frame
                traceback.append("%s:%d:%s"%(
                    os.path.basename(frame.f_code.co_filename),tb.tb_lineno,
                    frame.f_code.co_name))
                tb=tb.tb_next
            warm_start_error="%s: %s [%s]"%(
                type(exc).__name__,str(exc).splitlines()[-1],
                ">".join(traceback[-6:]))
        finally:
            warm_start_wall_seconds=time.perf_counter()-warm_start_begin
    if warm_start_result is not None and fixed_schedule is None:
        required_improvement=max(
            0.0,min(1.0,float(warm_start_early_accept_improvement)))
        seed_score=sequence_cost(seed_actions,weights)["score"]
        warm_score=sequence_cost(warm_start_result[1],weights)["score"]
        relative_improvement=(seed_score-warm_score)/max(1e-9,abs(seed_score))
        if (required_improvement>0.0 and
                relative_improvement>=required_improvement):
            warm_poses,warm_actions,warm_boxes,warm_meta=warm_start_result
            warm_meta["solver"]="ipopt_fixed_schedule_early_accept"
            warm_meta["fallback_reason"]=(
                "warm_start_improvement_threshold")
            warm_meta["seed_score"]=float(seed_score)
            warm_meta["warm_start_score"]=float(warm_score)
            warm_meta["warm_start_relative_improvement"]=float(
                relative_improvement)
            warm_meta["warm_start_early_accept_improvement"]=float(
                required_improvement)
            warm_meta["warm_start_wall_seconds"]=warm_start_wall_seconds
            warm_meta["bonmin_solve_seconds"]=0.0
            warm_meta["bonmin_status"]="SKIPPED_warm_start_gain_threshold"
            return warm_poses,warm_actions,warm_boxes,warm_meta
    # Constrain the full body envelope at regular interior stations. The seed
    # defines only the corridor locations; the MINLP state at each station is
    # computed from its own mode, gear, and continuous controls.
    collision_spacing=max(.05,float(collision_spacing))
    collision_substeps=max(2,int(math.ceil(
        max(float(max_translation),grid.corner*float(max_spin))/
        collision_spacing)))
    corridor_clearance_slack=min(
        max(0.0,float(corridor_clearance_slack)),math.sqrt(2)*grid.res)
    # If any seed station falls back from a full-cover disc corridor to a
    # point corridor, add one local refinement level on that segment. This
    # adds another full-body constraint between the weak station and its
    # neighbors without increasing density on already-clear spans.
    if adaptive_collision_refinement:
        feat=features(grid)
        padded_free=np.pad(~grid.occ,1,constant_values=False)
        clearance=distance_transform_edt(padded_free)[1:-1,1:-1]*grid.res
        disc_threshold=(feat[0][2]+math.sqrt(2)*grid.res-
                        corridor_clearance_slack)
    segment_substeps=[]
    segment_fractions=[]
    for k in range(n):
        steps=collision_substeps
        if adaptive_collision_refinement:
            needs_refinement=False
            for j in range(collision_substeps+1):
                fraction=j/float(collision_substeps)
                p=advance(refs[k],Action(actions[k].mode,
                                         fraction*actions[k].amount,
                                         actions[k].control))
                c,s=math.cos(p.th),math.sin(p.th)
                for ox,oy,_ in feat:
                    x=p.x+ox*c-oy*s; y=p.y+ox*s+oy*c
                    ix=math.floor((x-grid.ox)/grid.res)
                    iy=math.floor((y-grid.oy)/grid.res)
                    if (ix<0 or iy<0 or ix>=grid.width or iy>=grid.height or
                            clearance[iy,ix]<disc_threshold):
                        needs_refinement=True
                        break
                if needs_refinement:
                    break
            if needs_refinement:
                steps*=2
        segment_substeps.append(steps)
        segment_fractions.append([j/float(steps)
                                  for j in range(1,steps)])

    corridor_refs=[refs[0]]
    knot_box_indices=[0]
    interior_box_indices=[]
    for k in range(n):
        segment_indices=[]
        for fraction in segment_fractions[k]:
            segment_indices.append(len(corridor_refs))
            corridor_refs.append(advance(
                refs[k],Action(actions[k].mode,
                               fraction*actions[k].amount,actions[k].control)))
        interior_box_indices.append(segment_indices)
        corridor_refs.append(refs[k+1])
        knot_box_indices.append(len(corridor_refs)-1)
    feat,all_boxes,covered_discs=build_corridors(
        grid,corridor_refs,clearance_slack=corridor_clearance_slack)
    boxes=[all_boxes[index] for index in knot_box_indices]
    interior_boxes=[[all_boxes[index] for index in indices]
                    for indices in interior_box_indices]
    yaw_ref=[refs[0].th]
    for p in refs[1:]: yaw_ref.append(yaw_ref[-1]+wrap(p.th-yaw_ref[-1]))

    # Derive necessary mode/control bounds from the same certified local
    # corridors used by the MINLP collision constraints. The two outer body
    # feature centers are separated along the vehicle axis, so their
    # corridor difference bounds feasible body yaw. Their average bounds the
    # body center; disjoint center boxes rule out Spin, while the direction of
    # center-box displacement bounds an O primitive's travel tangent.
    yaw_bounds=[]
    yaw_bound_widths=[]
    for k,feature_boxes in enumerate(boxes):
        low=yaw_ref[k]-math.pi;high=yaw_ref[k]+math.pi
        first=corridor_box_corners(feature_boxes[0])
        last=corridor_box_corners(feature_boxes[-1])
        feature_difference=[(x1-x0,y1-y0)
                            for x1,y1 in last for x0,y0 in first]
        sector=_angular_sector_from_vectors(feature_difference,yaw_ref[k])
        if sector is not None:
            candidate_low=yaw_ref[k]+sector[0]
            candidate_high=yaw_ref[k]+sector[1]
            if candidate_high-candidate_low<high-low:
                low,high=candidate_low,candidate_high
        yaw_bounds.append((low,high))
        yaw_bound_widths.append(high-low)

    center_aabbs=[_corridor_center_aabb(feature_boxes)
                  for feature_boxes in boxes]
    spin_possible=[]
    oblique_tangent_bounds=[]
    oblique_control_bounds=[]
    for k in range(n):
        start_box=center_aabbs[k];end_box=center_aabbs[k+1]
        dx_low=end_box[0]-start_box[1]
        dx_high=end_box[1]-start_box[0]
        dy_low=end_box[2]-start_box[3]
        dy_high=end_box[3]-start_box[2]
        spin_possible.append(
            dx_low<=1e-9 and dx_high>=-1e-9 and
            dy_low<=1e-9 and dy_high>=-1e-9)
        displacement=(refs[k+1].x-refs[k].x,
                      refs[k+1].y-refs[k].y)
        displacement_length=math.hypot(*displacement)
        tangent_bounds=None
        control_bounds=None
        if displacement_length>1e-6:
            reference_tangent=math.atan2(displacement[1],displacement[0])
            reference_tangent=(yaw_ref[k]+
                wrap(reference_tangent-yaw_ref[k]))
            sector=_aabb_difference_sector(
                start_box,end_box,reference_tangent)
            if sector is not None:
                raw_low=reference_tangent+sector[0]
                raw_high=reference_tangent+sector[1]
                possible_low=yaw_bounds[k][0]-math.pi/2
                possible_high=yaw_bounds[k][1]+math.pi/2
                branches=[]
                for turn in range(-2,3):
                    branch_low=raw_low+2*math.pi*turn
                    branch_high=raw_high+2*math.pi*turn
                    if (branch_high>=possible_low-1e-9 and
                            branch_low<=possible_high+1e-9):
                        branches.append((branch_low,branch_high))
                if not branches:
                    tangent_bounds=False
                elif len(branches)==1:
                    tangent_bounds=branches[0]
                    control_low=max(-math.pi/2,
                        tangent_bounds[0]-yaw_bounds[k][1])
                    control_high=min(math.pi/2,
                        tangent_bounds[1]-yaw_bounds[k][0])
                    if control_low<=control_high+1e-9:
                        control_bounds=(control_low,control_high)
                    else:
                        tangent_bounds=False
        oblique_tangent_bounds.append(tangent_bounds)
        oblique_control_bounds.append(control_bounds)
    geometry_bound_stats={
        "enabled":bool(geometry_yaw_bounds or geometry_spin_pruning or
                        geometry_oblique_bounds),
        "yaw_tightened_knots":sum(width<2*math.pi-1e-6
                                   for width in yaw_bound_widths),
        "yaw_interval_widths_rad":[round(width,6)
                                    for width in yaw_bound_widths],
        "spin_eliminated_knots":sum(not possible
                                     for possible in spin_possible),
        "oblique_tangent_bounded_knots":sum(
            bound is not None and bound is not False
            for bound in oblique_tangent_bounds),
        "oblique_eliminated_knots":sum(
            bound is False for bound in oblique_tangent_bounds),
        "oblique_tangent_widths_rad":[
            round(bound[1]-bound[0],6)
            if bound is not None and bound is not False else None
            for bound in oblique_tangent_bounds]}
    if not geometry_yaw_bounds:
        yaw_bounds=[(reference-math.pi,reference+math.pi)
                    for reference in yaw_ref]
        yaw_bound_widths=[2*math.pi for _ in yaw_ref]
    if not geometry_spin_pruning:
        spin_possible=[True for _ in range(n)]
    if not geometry_oblique_bounds:
        oblique_tangent_bounds=[None for _ in range(n)]
        oblique_control_bounds=[None for _ in range(n)]
    geometry_bound_stats.update({
        "enabled":bool(geometry_yaw_bounds or geometry_spin_pruning or
                        geometry_oblique_bounds),
        "yaw_tightened_knots":sum(width<2*math.pi-1e-6
                                   for width in yaw_bound_widths),
        "yaw_interval_widths_rad":[round(width,6)
                                    for width in yaw_bound_widths],
        "spin_eliminated_knots":sum(not possible
                                     for possible in spin_possible),
        "oblique_tangent_bounded_knots":sum(
            bound is not None and bound is not False
            for bound in oblique_tangent_bounds),
        "oblique_eliminated_knots":sum(
            bound is False for bound in oblique_tangent_bounds),
        "oblique_tangent_widths_rad":[
            round(bound[1]-bound[0],6)
            if bound is not None and bound is not False else None
            for bound in oblique_tangent_bounds],
        "yaw_bounds_enabled":bool(geometry_yaw_bounds),
        "spin_pruning_enabled":bool(geometry_spin_pruning),
        "oblique_bounds_enabled":bool(geometry_oblique_bounds)})

    w={"spin":1.00,"reverse":1.00,"switch":2.00,"direction":20.0,
       "steer_tv":.40,"curvature":.01,"oblique_tv":1.50,"oblique_event":1.50,
       "oblique_use":.55,"oblique_cruise":.45,"track":.015,
       "low_curvature_switch":.35,"switch_turn_threshold":.35}
    if weights: w.update(weights)
    opt=ca.Opti()
    X=opt.variable(3,n+1)
    # A one-hot A/O/S vector has only two independent bits.  Encode A and O
    # as the integer variables and derive Spin as their complement.  This
    # cuts one integer variable per knot while preserving the exact mode set.
    mode_bits=opt.variable(2,n)
    mode=ca.vertcat(mode_bits[0,:],mode_bits[1,:],
                    1-mode_bits[0,:]-mode_bits[1,:])
    # Every translation knot chooses forward or reverse gear jointly with its
    # mode and geometry. A/O distances are nonnegative magnitudes; their gear
    # bit supplies the signed direction in the exact motion equations.
    # Disaggregate A and O translation lengths. The previous formulation
    # selected whole pose equations with mode*pose terms, which made even the
    # root NLP multiply the state by fractional mode variables.  These channel
    # lengths are zero outside their active mode and let every state equation
    # add only a mode-specific displacement increment.
    ds_ack=opt.variable(1,n);ds_obl=opt.variable(1,n)
    gear_bit=opt.variable(1,n)
    direction_change=opt.variable(1,n)
    dtheta=opt.variable(1,n)
    kappa=opt.variable(1,n);phi=opt.variable(1,n)
    spin_abs=opt.variable(1,n);kappa_abs=opt.variable(1,n);phi_abs=opt.variable(1,n)
    kappa_tv=opt.variable(1,n);phi_tv=opt.variable(1,n)
    mode_change=opt.variable(1,n)
    if fixed_schedule is None:
        opt.set_domain(mode_bits,"integer")
        opt.set_domain(gear_bit,"integer")
        opt.set_domain(direction_change,"integer")
    opt.subject_to(opt.bounded(0,mode_bits,1))
    opt.subject_to(opt.bounded(0,gear_bit,1))
    opt.subject_to(mode_bits[0,:]+mode_bits[1,:]<=1)
    opt.subject_to(opt.bounded(0,mode_change,1))
    opt.subject_to(opt.bounded(0,direction_change,1))
    if fixed_schedule is not None:
        for k,(fixed_mode,fixed_gear) in enumerate(fixed_schedule):
            if fixed_mode not in (0,1,2) or fixed_gear not in (0,1):
                raise ValueError("joint_invalid_fixed_schedule")
            opt.subject_to(mode[fixed_mode,k]==1)
            opt.subject_to(gear_bit[0,k]==fixed_gear)
    if n==1:
        # With one primitive the exact endpoint geometry fully determines
        # which vehicle modes can reach the goal. Remove impossible branches
        # before Bonmin starts; this is an exact reachability cut, not a mode
        # preference or scene rule.
        for candidate_mode in (0,1,2):
            if candidate_mode not in single_knot_modes:
                opt.subject_to(mode[candidate_mode,0]==0)
    for k in range(n):
        if not spin_possible[k]:
            opt.subject_to(mode[2,k]==0)
        if oblique_tangent_bounds[k] is False:
            opt.subject_to(mode[1,k]==0)
        elif oblique_tangent_bounds[k] is not None:
            tangent_low,tangent_high=oblique_tangent_bounds[k]
            control_low,control_high=oblique_control_bounds[k]
            opt.subject_to(phi[0,k]>=control_low*mode[1,k])
            opt.subject_to(phi[0,k]<=control_high*mode[1,k])
            # The disjunction relaxes when O is inactive. In the O branch,
            # the exact motion direction must lie in the corridor-derived
            # angular sector between the two center regions.
            tangent_big_m=4*math.pi
            opt.subject_to(X[2,k]+phi[0,k]>=
                tangent_low-tangent_big_m*(1-mode[1,k]))
            opt.subject_to(X[2,k]+phi[0,k]<=
                tangent_high+tangent_big_m*(1-mode[1,k]))
    opt.subject_to(ds_ack>=.005*mode[0,:])
    opt.subject_to(ds_ack<=max_translation*mode[0,:])
    opt.subject_to(ds_obl>=.005*mode[1,:])
    opt.subject_to(ds_obl<=max_translation*mode[1,:])
    # Necessary route-level reachability cut: the sum of forward translation
    # arc lengths must cover at least the shortest displacement from the start
    # to the allowed terminal disk. Spin contributes no translation. This is
    # implied by the exact motion equations, but makes the implication visible
    # to Bonmin's node relaxations so an all-spin word can be discarded early.
    translation_distance_lb=max(
        0.0,math.hypot(goal.x-start.x,goal.y-start.y)-
        joint_xy_tol-terminal_solver_slack)
    # The controls are row vectors (1 x n), so sum2 reduces over knots.
    opt.subject_to(ca.sum2(ds_ack+ds_obl)>=translation_distance_lb)
    opt.subject_to(opt.bounded(-max_spin*mode[2,:],dtheta,max_spin*mode[2,:]))
    kmax=1/.810
    opt.subject_to(opt.bounded(-kmax*mode[0,:],kappa,kmax*mode[0,:]))
    opt.subject_to(opt.bounded(-math.pi/2*mode[1,:],phi,math.pi/2*mode[1,:]))
    opt.subject_to(spin_abs>=dtheta);opt.subject_to(spin_abs>=-dtheta)
    opt.subject_to(kappa_abs>=kappa);opt.subject_to(kappa_abs>=-kappa)
    opt.subject_to(phi_abs>=phi);opt.subject_to(phi_abs>=-phi)
    # Absolute-value epigraphs also need mode-scaled upper bounds. Without
    # these bounds, an inactive-mode epigraph can grow arbitrarily while its
    # disaggregated distance is numerically a few ulps below zero. The
    # bilinear distance*absolute-control objective can then exploit the tiny
    # negative distance and report a large, spurious negative cost.
    opt.subject_to(spin_abs<=max_spin*mode[2,:])
    opt.subject_to(kappa_abs<=(1/.810)*mode[0,:])
    opt.subject_to(phi_abs<=math.pi/2*mode[1,:])
    opt.subject_to(kappa_tv>=0);opt.subject_to(phi_tv>=0)

    # The source labels initialize the exact mode/gear disjunct and relaxed NLP.
    for forbidden in _forbidden_schedules:
        if len(forbidden)!=n:
            raise ValueError("joint_invalid_forbidden_schedule")
        matches=0
        for k,(mode_id,gear) in enumerate(forbidden):
            mode_match=mode[int(mode_id),k]
            gear_match=(gear_bit[0,k] if int(gear)==1 else
                        1-gear_bit[0,k])
            matches+=mode_match*gear_match
        # Exclude only the exact mode/gear word after a swept rejection.
        opt.subject_to(matches<=n-1)
    objective=0
    for k in range(n):
        # The first primitive has no predecessor and therefore incurs no
        # mode-switch cost.
        if k==0:
            opt.subject_to(mode_change[0,k]==0)
            opt.subject_to(direction_change[0,k]==0)
        else:
            previous=mode[:,k-1]
            # With two independent one-hot bits, the maximum absolute bit
            # difference is an exact 0/1 switch indicator for integer modes.
            # This epigraph also gives the bilinear objective below a strong
            # linear lower bound in the relaxed root NLP.
            for bit in range(2):
                opt.subject_to(mode_change[0,k]>=
                               mode_bits[bit,k]-mode_bits[bit,k-1])
                opt.subject_to(mode_change[0,k]>=
                               mode_bits[bit,k-1]-mode_bits[bit,k])
            translation_cur=mode[0,k]+mode[1,k]
            opt.subject_to(direction_change[0,k]>=
                gear_bit[0,k]-gear_bit[0,k-1]-(1-translation_cur))
            opt.subject_to(direction_change[0,k]>=
                gear_bit[0,k-1]-gear_bit[0,k]-(1-translation_cur))
            opt.subject_to(direction_change[0,k]<=translation_cur)
            # Spin carries the last selected translation gear so a reversal
            # immediately after a pivot is still counted once.
            opt.subject_to(gear_bit[0,k]-gear_bit[0,k-1]<=1-mode[2,k])
            opt.subject_to(gear_bit[0,k-1]-gear_bit[0,k]<=1-mode[2,k])
            same_ack=mode[0,k-1]+mode[0,k]
            same_oblique=mode[1,k-1]+mode[1,k]
            tv_relax=2.0
            opt.subject_to(kappa_tv[0,k]>=kappa[0,k]-kappa[0,k-1]-tv_relax*(2-same_ack))
            opt.subject_to(kappa_tv[0,k]>=kappa[0,k-1]-kappa[0,k]-tv_relax*(2-same_ack))
            opt.subject_to(phi_tv[0,k]>=phi[0,k]-phi[0,k-1]-math.pi*(2-same_oblique))
            opt.subject_to(phi_tv[0,k]>=phi[0,k-1]-phi[0,k]-math.pi*(2-same_oblique))
        th=X[2,k];xx=X[0,k];yy=X[1,k]
        ds_a=ds_ack[0,k];ds_o=ds_obl[0,k]
        signed_a=(2*gear_bit[0,k]-1)*ds_a
        signed_o=(2*gear_bit[0,k]-1)*ds_o
        z=signed_a*kappa[0,k]/2
        sinc=_symbolic_sinc(z,ca)
        dx_ack=signed_a*sinc*ca.cos(th+z)
        dy_ack=signed_a*sinc*ca.sin(th+z)
        dx_obl=signed_o*ca.cos(th+phi[0,k])
        dy_obl=signed_o*ca.sin(th+phi[0,k])
        opt.subject_to(X[0,k+1]==xx+dx_ack+dx_obl)
        opt.subject_to(X[1,k+1]==yy+dy_ack+dy_obl)
        opt.subject_to(X[2,k+1]==th+signed_a*kappa[0,k]+dtheta[0,k])

        objective+=ds_ack[0,k]+ds_obl[0,k]
        objective+=w["spin"]*spin_abs[0,k]
        objective+=w["reverse"]*(1-gear_bit[0,k])*(ds_a+ds_o)
        if k>0:
            objective+=w["direction"]*direction_change[0,k]
        objective+=w["curvature"]*ds_ack[0,k]*kappa_abs[0,k]**2
        if k>0:
            linear_switch=mode_change[0,k]
            agreement_switch=1-ca.dot(mode[:,k-1],mode[:,k])
            # Both terms equal the exact switch indicator for integer one-hot
            # modes. Their average keeps the stronger linear relaxation while
            # retaining the useful mode-agreement shape in the smooth NLP.
            objective+=w["switch"]*.5*(linear_switch+agreement_switch)
        objective+=w["oblique_use"]*ds_obl[0,k]*phi_abs[0,k]
        objective+=w["oblique_cruise"]*ds_obl[0,k]*ca.exp(
            -.5*(phi[0,k]/O_CRUISE_SIGMA)**2)
        objective+=w["steer_tv"]*kappa_tv[0,k]
        objective+=w["oblique_tv"]*phi_tv[0,k]
        if k>0:
            phi_delta=ca.sqrt((phi[0,k]-phi[0,k-1])**2+1e-10)
            event=phi_delta/(phi_delta+.03)
            objective+=w["oblique_event"]*event*mode[1,k]*mode[1,k-1]
            # Match the shared route score's penalty for an A<->O switch
            # whose travel tangent barely changes. The exact score uses a
            # threshold indicator; a narrow sigmoid keeps the MINLP objective
            # differentiable while approximating that same transition cost.
            previous_tangent=(mode[0,k-1]*X[2,k]+
                              mode[1,k-1]*(X[2,k-1]+phi[0,k-1]))
            current_tangent=(mode[0,k]*X[2,k]+
                             mode[1,k]*(X[2,k]+phi[0,k]))
            tangent_delta=ca.atan2(
                ca.sin(current_tangent-previous_tangent),
                ca.cos(current_tangent-previous_tangent))
            tangent_delta_abs=ca.sqrt(tangent_delta**2+1e-10)
            turn_threshold=max(1e-3,float(w["switch_turn_threshold"]))
            transition_center=max(
                0.0,turn_threshold-min(.02,.1*turn_threshold))
            low_curvature_indicator=1/(1+ca.exp(
                60*(tangent_delta_abs-transition_center)))
            ao_switch=(mode[0,k-1]*mode[1,k]+
                       mode[1,k-1]*mode[0,k])
            objective+=(float(w["low_curvature_switch"])*
                        ao_switch*low_curvature_indicator)

    opt.subject_to(X[:,0]==ca.vertcat(start.x,start.y,start.th))
    goal_yaw=yaw_ref[-1]+wrap(goal.th-yaw_ref[-1])
    opt.subject_to((X[0,n]-goal.x)**2+(X[1,n]-goal.y)**2<=
                   (joint_xy_tol+terminal_solver_slack)**2)
    opt.subject_to(opt.bounded(goal_yaw-joint_yaw_tol-terminal_solver_slack,
                               X[2,n],
                               goal_yaw+joint_yaw_tol+terminal_solver_slack))
    # The tolerances define feasibility, not an invitation to stop at their
    # boundary.  Penalize terminal residuals so short-cutting is only chosen
    # when the actual route-cost benefit justifies it.
    objective+=float(terminal_xy_weight)*((X[0,n]-goal.x)**2+
                                          (X[1,n]-goal.y)**2)
    objective+=float(terminal_yaw_weight)*(X[2,n]-goal_yaw)**2
    for k,p in enumerate(refs):
        # These bounds are necessary consequences of the outer feature-disc
        # corridors. They retain the seed's unwrapped yaw branch while
        # removing orientations that cannot place both body ends in their
        # certified local free-space boxes.
        opt.subject_to(opt.bounded(yaw_bounds[k][0],X[2,k],
                                   yaw_bounds[k][1]))
        c,s=ca.cos(X[2,k]),ca.sin(X[2,k])
        for (ox0,oy0,_),(cx,cy,half_long,half_lat,heading) in zip(feat,boxes[k]):
            fx=X[0,k]+ox0*c-oy0*s;fy=X[1,k]+ox0*s+oy0*c
            ch,sh=math.cos(heading),math.sin(heading)
            along=(fx-cx)*ch+(fy-cy)*sh
            lateral=-(fx-cx)*sh+(fy-cy)*ch
            opt.subject_to(opt.bounded(-half_long,along,half_long))
            opt.subject_to(opt.bounded(-half_lat,lateral,half_lat))
        objective+=w["track"]*((X[0,k]-p.x)**2+(X[1,k]-p.y)**2)
    # Constrain exact interior states as well as every knot. Sparse knot-only
    # corridors let long primitives cut through obstacles between safe ends;
    # the final exact sweep remains mandatory for every accepted candidate.
    for k in range(n):
        ds_a=ds_ack[0,k];ds_o=ds_obl[0,k]
        signed_a=(2*gear_bit[0,k]-1)*ds_a
        signed_o=(2*gear_bit[0,k]-1)*ds_o
        xx,yy,th=X[0,k],X[1,k],X[2,k]
        for j,fraction in enumerate(segment_fractions[k]):
            z=signed_a*kappa[0,k]*fraction/2
            sinc=_symbolic_sinc(z,ca)
            dx_ack=signed_a*fraction*sinc*ca.cos(th+z)
            dy_ack=signed_a*fraction*sinc*ca.sin(th+z)
            dx_obl=signed_o*fraction*ca.cos(th+phi[0,k])
            dy_obl=signed_o*fraction*ca.sin(th+phi[0,k])
            xf=xx+dx_ack+dx_obl
            yf=yy+dy_ack+dy_obl
            thf=(th+fraction*(signed_a*kappa[0,k]+dtheta[0,k]))
            ref_fraction=corridor_refs[interior_box_indices[k][j]]
            yaw_fraction=yaw_ref[k]+wrap(ref_fraction.th-refs[k].th)
            opt.subject_to(opt.bounded(yaw_fraction-math.pi,thf,
                                       yaw_fraction+math.pi))
            c,s=ca.cos(thf),ca.sin(thf)
            for (ox0,oy0,_),(cx,cy,half_long,half_lat,heading) in zip(
                    feat,interior_boxes[k][j]):
                fx=xf+ox0*c-oy0*s;fy=yf+ox0*s+oy0*c
                ch,sh=math.cos(heading),math.sin(heading)
                along=(fx-cx)*ch+(fy-cy)*sh
                lateral=-(fx-cx)*sh+(fy-cy)*ch
                opt.subject_to(opt.bounded(-half_long,along,half_long))
                opt.subject_to(opt.bounded(-half_lat,lateral,half_lat))
    # Collision-specific SAT cuts use a fixed separating axis selected from
    # an exact colliding sweep sample. They supplement the seed corridor and
    # never replace the exact swept check on returned trajectories.
    for (k,fraction,cell_x,cell_y,nx,ny,required) in _collision_planes:
        ds_a=ds_ack[0,k];ds_o=ds_obl[0,k]
        signed_a=(2*gear_bit[0,k]-1)*ds_a
        signed_o=(2*gear_bit[0,k]-1)*ds_o
        xx,yy,th=X[0,k],X[1,k],X[2,k]
        z=signed_a*kappa[0,k]*fraction/2
        sinc=_symbolic_sinc(z,ca)
        dx_ack=signed_a*fraction*sinc*ca.cos(th+z)
        dy_ack=signed_a*fraction*sinc*ca.sin(th+z)
        dx_obl=signed_o*fraction*ca.cos(th+phi[0,k])
        dy_obl=signed_o*fraction*ca.sin(th+phi[0,k])
        xf=xx+dx_ack+dx_obl
        yf=yy+dy_ack+dy_obl
        opt.subject_to(nx*(xf-cell_x)+ny*(yf-cell_y)>=required)
    objective+=w["track"]*.25*ca.sumsqr(X[2,:]-ca.DM(yaw_ref).T)
    opt.minimize(objective)
    x_seed=ca.DM([[p.x for p in refs],[p.y for p in refs],yaw_ref])
    mode_seed=ca.DM([
        [1.0 if a.mode==0 else 0.0 for a in initial_actions],
        [1.0 if a.mode==1 else 0.0 for a in initial_actions]])
    ds_ack_seed=ca.DM([[abs(a.amount) if a.mode==0 else 0
                        for a in initial_actions]])
    ds_obl_seed=ca.DM([[abs(a.amount) if a.mode==1 else 0
                        for a in initial_actions]])
    _,initial_schedule=_joint_seed_mode_gear_schedule(initial_actions)
    gear_seed=ca.DM([[float(gear) for _,gear in initial_schedule]])
    turn_seed=ca.DM([[a.amount if a.mode==2 else 0 for a in initial_actions]])
    kappa_seed=ca.DM([[a.control if a.mode==0 else 0 for a in initial_actions]])
    phi_seed=ca.DM([[a.control if a.mode==1 else 0 for a in initial_actions]])
    initial_poses=replay(start,initial_actions)
    initial_yaw=[initial_poses[0].th]
    for p in initial_poses[1:]:
        initial_yaw.append(initial_yaw[-1]+wrap(p.th-initial_yaw[-1]))
    x_initial=ca.DM([[p.x for p in initial_poses],
                     [p.y for p in initial_poses],initial_yaw])
    opt.set_initial(X,x_initial)
    opt.set_initial(mode_bits,mode_seed)
    mode_change_guess=np.zeros((1,n))
    direction_change_guess=np.zeros((1,n))
    for k in range(1,n):
        mode_change_guess[0,k]=float(
            initial_actions[k].mode!=initial_actions[k-1].mode)
        direction_change_guess[0,k]=float(
            initial_actions[k].mode!=2 and initial_actions[k-1].mode!=2 and
            ((initial_actions[k].amount>0)!=(initial_actions[k-1].amount>0)))
    opt.set_initial(mode_change,ca.DM(mode_change_guess))
    opt.set_initial(gear_bit,gear_seed)
    opt.set_initial(direction_change,ca.DM(direction_change_guess))
    kappa_tv_guess=np.zeros((1,n));phi_tv_guess=np.zeros((1,n))
    for k,action in enumerate(initial_actions):
        if k>0 and initial_actions[k-1].mode==action.mode==0:
            kappa_tv_guess[0,k]=abs(action.control-initial_actions[k-1].control)
        if k>0 and initial_actions[k-1].mode==action.mode==1:
            phi_tv_guess[0,k]=abs(wrap(action.control-initial_actions[k-1].control))
    opt.set_initial(kappa_tv,ca.DM(kappa_tv_guess))
    opt.set_initial(phi_tv,ca.DM(phi_tv_guess))
    opt.set_initial(ds_ack,ds_ack_seed)
    opt.set_initial(ds_obl,ds_obl_seed)
    opt.set_initial(dtheta,turn_seed)
    opt.set_initial(kappa,kappa_seed)
    opt.set_initial(phi,phi_seed)
    opt.set_initial(spin_abs,ca.DM([[abs(a.amount) if a.mode==2 else 0
                                     for a in initial_actions]]))
    opt.set_initial(kappa_abs,ca.DM([[abs(a.control) if a.mode==0 else 0
                                      for a in initial_actions]]))
    opt.set_initial(phi_abs,ca.DM([[abs(a.control) if a.mode==1 else 0
                                    for a in initial_actions]]))
    # CasADi's Bonmin plugin accepts BONMIN parameters through this dictionary.
    # Its bundled NLP subsolver has limited option forwarding; this is one
    # reason the MINLP remains an opt-in experiment for now.
    setup_elapsed=time.perf_counter()-call_begin
    solver_time_limit=max(.05,float(time_limit)-setup_elapsed)
    subproblem_wall_limit=min(max(.05,float(subproblem_time_limit)),
                              solver_time_limit)
    minlp_pump_time_limit=min(2.0,solver_time_limit)
    bonmin_budget_meta={
        "requested_time_limit":float(time_limit),
        "model_setup_seconds":float(setup_elapsed),
        "effective_solver_time_limit":float(solver_time_limit),
        "subproblem_wall_time_limit":float(subproblem_wall_limit),
        "minlp_pump_time_limit":float(minlp_pump_time_limit)}
    bonmin_opts={"print_level":0,"bb_log_level":0,"nlp_log_level":0,
                 "algorithm":str(algorithm),
                 "random_generator_seed":int(random_seed),
                 "time_limit":solver_time_limit,
                 "node_comparison":str(node_comparison),
                 "variable_selection":str(variable_selection),
                 "heuristic_feasibility_pump":
                     "yes" if use_feasibility_pump else "no",
                 "heuristic_RINS":
                     "yes" if use_rins_heuristic else "no",
                 "pump_for_minlp":"yes" if use_minlp_pump else "no"}
    if int(bonmin_iteration_limit)>0:
        bonmin_opts["iteration_limit"]=int(bonmin_iteration_limit)
    if bonmin_unsolved_nlp_retries>0:
        bonmin_opts["num_retry_unsolved_random_point"]=(
            bonmin_unsolved_nlp_retries)
    if algorithm=="B-BB":
        bonmin_opts.update({
            "warm_start":"optimum",
            # Nonconvex NLP relaxations may fail to prove local optimality.
            # Fathom those nodes so the tree can still search for a feasible
            # integer point; repeated root solves can exceed the wall budget.
            "nlp_failure_behavior":"fathom"})
    if int(root_resolves)>0:
        bonmin_opts["num_resolve_at_root"]=int(root_resolves)
    if dive_fractional:
        bonmin_opts["heuristic_dive_fractional"]="yes"
    opts={"print_time":False,"bonmin":bonmin_opts}
    initial_assignment=[item for item in opt.initial()
                        if item.is_op(ca.OP_EQ)]
    g0=np.asarray(opt.debug.value(opt.g,initial_assignment),
                  dtype=float).reshape(-1)
    gl=np.asarray(opt.debug.value(opt.lbg),dtype=float).reshape(-1)
    gu=np.asarray(opt.debug.value(opt.ubg),dtype=float).reshape(-1)
    initial_violation=np.maximum(np.maximum(gl-g0,0.0),g0-gu)
    # Corridor bounds are assembled from rounded map-cell geometry. This is a
    # diagnostic of the supplied continuous start point, not a feasibility
    # gate for the MINLP: Bonmin must be allowed to repair a coarse point that
    # misses the stricter terminal box or a corridor face. Every returned
    # trajectory still has to satisfy the original constraints and exact SAT.
    initial_feasibility_tolerance=1e-4
    bad_initial=np.flatnonzero(
        initial_violation>initial_feasibility_tolerance)
    initial_point_feasible=(len(bad_initial)==0)
    worst_initial=sorted(
        bad_initial,key=lambda i:initial_violation[i],reverse=True)[:4]
    initial_violation_records=[
        "%s value=%.6g bounds=[%.6g,%.6g]" % (
            opt.debug.g_describe(int(i)).splitlines()[-1].strip(),
            float(g0[i]),float(gl[i]),float(gu[i]))
        for i in worst_initial]
    seed_objective=float(opt.debug.value(objective,initial_assignment))
    # If the supplied seed already meets the strict terminal box and exact
    # swept check, and its actual MINLP initial assignment satisfies every
    # model constraint, keep that known integer-feasible path as the safe
    # fallback. Re-projecting the same schedule through IPOPT can fail on a
    # nonconvex corridor even though the supplied path itself is feasible.
    if validated_seed_candidate and initial_point_feasible:
        _,seed_boxes,seed_corridor_covered=build_corridors(grid,seed_poses)
        seed_score=sequence_cost(seed_actions,weights)["score"]
        warm_start_meta={"solver":"validated_integer_seed",
                         "score":float(seed_score),
                         "knots":int(n),
                         "corridor":("disc" if seed_corridor_covered
                                     else "mixed_disc_point"),
                         "validated_seed_fallback":True}
        warm_start_result=(seed_poses,seed_actions,seed_boxes,
                           warm_start_meta)
    cutoff_delta=max(1e-6,abs(seed_objective)*1e-8)
    feasible_warm_start_used=warm_start_meta is not None
    # A Bonmin cutoff is an incumbent bound. The coarse seed is an incumbent
    # only when its full MINLP constraints are already satisfied; an infeasible
    # start must not place an artificial upper bound on the feasibility search.
    cutoff_eligible=(feasible_warm_start_used or initial_point_feasible)
    objective_cutoff=(seed_objective+cutoff_delta
                      if feasible_warm_start_used
                      else seed_objective-cutoff_delta)
    if use_seed_cutoff and cutoff_eligible:
        bonmin_opts["cutoff"]=objective_cutoff
    solve_begin=time.perf_counter()
    # BONMIN checks its global time limit between NLP node solves, so one
    # Ipopt call can otherwise run far beyond the requested budget. BONMIN
    # reads Ipopt limits from bonmin.opt; give each call an isolated working
    # directory to bound subproblem time without leaving shared config behind.
    original_cwd=os.getcwd()
    solver_dir=None
    try:
        if fixed_schedule is not None:
            # Repair the geometry of a MINLP-selected schedule with a local
            # continuous NLP. The incoming point may violate a collision cut;
            # exact replay and swept SAT decide whether the repair is usable.
            opt.solver("ipopt",{"print_time":False},
                       {"print_level":0,"max_iter":600,
                        "max_wall_time":solver_time_limit})
            sol=opt.solve_limited()
        else:
            solver_dir=tempfile.mkdtemp(prefix="fwis_bonmin_")
            options_path=os.path.join(solver_dir,"bonmin.opt")
            option_lines=["print_level 0",
                          "max_iter %d"%max(
                              1,int(bonmin_subproblem_iteration_limit)),
                          "max_wall_time %.6g"%subproblem_wall_limit,
                          "bonmin.algorithm %s"%str(algorithm),
                          "bonmin.random_generator_seed %d"%int(random_seed),
                          "bonmin.time_limit %.6g"%solver_time_limit,
                          "bonmin.node_comparison %s"%str(node_comparison),
                          "bonmin.variable_selection %s"%str(variable_selection),
                          "bonmin.heuristic_feasibility_pump %s" %
                              ("yes" if use_feasibility_pump else "no"),
                          "bonmin.heuristic_RINS %s" %
                              ("yes" if use_rins_heuristic else "no"),
                          "bonmin.pump_for_minlp %s" %
                              ("yes" if use_minlp_pump else "no"),
                          "pump_for_minlp.time_limit %.6g"%minlp_pump_time_limit,
                          "pump_for_minlp.solution_limit 1"]
            if int(bonmin_iteration_limit)>0:
                option_lines.append("bonmin.iteration_limit %d" %
                                    int(bonmin_iteration_limit))
            if bonmin_unsolved_nlp_retries>0:
                option_lines.append(
                    "bonmin.num_retry_unsolved_random_point %d" %
                    bonmin_unsolved_nlp_retries)
            if use_seed_cutoff and cutoff_eligible:
                option_lines.append("bonmin.cutoff %.12g"%objective_cutoff)
            if int(root_resolves)>0:
                option_lines.append("bonmin.num_resolve_at_root %d"%int(root_resolves))
            if dive_fractional:
                option_lines.append("bonmin.heuristic_dive_fractional yes")
            if algorithm=="B-BB":
                option_lines.extend(["bonmin.nlp_failure_behavior fathom",
                                     "bonmin.warm_start optimum"])
            with open(options_path,"w",encoding="utf-8") as stream:
                stream.write("\n".join(option_lines)+"\n")
            os.chdir(solver_dir)
            solver_plugin="bonmin"
            if use_cbc_seed_incumbent and fixed_schedule is None:
                if not (validated_seed_candidate and initial_point_feasible):
                    bonmin_seed_incumbent_status="skipped_seed_not_feasible"
                else:
                    try:
                        plugin_available=bool(ca.has_nlpsol("bonminseed"))
                    except Exception:
                        plugin_available=False
                    if not plugin_available:
                        bonmin_seed_incumbent_status="plugin_unavailable"
                        raise RuntimeError(
                            "joint_bonminseed_plugin_unavailable:"
                            "build_plugin_and_set_CASADIPATH")
                    opts["seed_incumbent"]=True
                    solver_plugin="bonminseed"
                    bonmin_seed_incumbent_status="plugin_selected"
            if use_cbc_seed_incumbent:
                print("[v3 optimizer] CBC seed incumbent plugin=%s status=%s" %
                      (solver_plugin,bonmin_seed_incumbent_status),flush=True)
            opt.solver(solver_plugin,opts,{})
            try:
                # A limited solve can return a non-incumbent NLP iterate.
                # Every point still goes through exact safety checks below.
                sol=opt.solve_limited()
            finally:
                os.chdir(original_cwd)
    except Exception as exc:
        bonmin_debug_mode=None
        bonmin_debug_integrality_gap=None
        bonmin_debug_terminal_error=None
        bonmin_debug_mode_values=None
        bonmin_debug_poses=None
        try:
            debug_modes=np.asarray(opt.debug.value(mode),dtype=float).reshape((3,n))
            if np.all(np.isfinite(debug_modes)):
                bonmin_debug_mode_values=debug_modes.tolist()
                bonmin_debug_mode="".join(
                    str(int(np.argmax(debug_modes[:,k]))) for k in range(n))
                bonmin_debug_integrality_gap=float(
                    np.max(np.abs(debug_modes-np.rint(debug_modes))))
        except Exception:
            pass
        try:
            debug_states=np.asarray(opt.debug.value(X),dtype=float).reshape((3,n+1))
            if np.all(np.isfinite(debug_states)):
                bonmin_debug_poses=debug_states.T.tolist()
                bonmin_debug_terminal_error=float(math.hypot(
                    debug_states[0,-1]-goal.x,
                    debug_states[1,-1]-goal.y))
        except Exception:
            pass
        if warm_start_result is not None:
            warm_poses,warm_actions,warm_boxes,warm_meta=warm_start_result
            warm_meta["solver"]="ipopt_feasible_seed_fallback"
            warm_meta["fallback_reason"]="bonmin_no_integer_incumbent"
            warm_meta["bonmin_rins_heuristic_enabled"]=(
                bool(use_rins_heuristic))
            warm_meta["bonmin_status"]=(
                str(exc).split("return_status is '")[-1].split("'")[0]
                if "return_status is '" in str(exc) else
                str(exc).splitlines()[-1])
            warm_meta["bonmin_error"]=str(exc).splitlines()[-1]
            warm_meta["bonmin_seed_incumbent_plugin"]=(
                bonmin_seed_incumbent_status)
            warm_meta["bonmin_solve_seconds"]=time.perf_counter()-solve_begin
            warm_meta["bonmin_warm_start_wall_seconds"]=(
                warm_start_wall_seconds)
            warm_meta["bonmin_warm_start_error"]=warm_start_error
            warm_meta["bonmin_root_initial_point_feasible"]=(
                initial_point_feasible)
            warm_meta["bonmin_root_initial_violation_count"]=int(len(bad_initial))
            warm_meta["bonmin_root_initial_violation_max"]=(
                float(initial_violation.max()) if len(initial_violation) else 0.0)
            warm_meta["bonmin_root_seed_cutoff_enabled"]=(
                bool(use_seed_cutoff and cutoff_eligible))
            warm_meta["bonmin_validated_seed_fallback"]=(
                bool(warm_meta.get("validated_seed_fallback",False)))
            warm_meta["bonmin_debug_last_mode_word"]=bonmin_debug_mode
            warm_meta["bonmin_debug_mode_integrality_gap"]=(
                bonmin_debug_integrality_gap)
            warm_meta["bonmin_debug_terminal_position_error"]=(
                bonmin_debug_terminal_error)
            warm_meta["bonmin_debug_mode_values"]=bonmin_debug_mode_values
            warm_meta["bonmin_debug_states"]=bonmin_debug_poses
            return warm_poses,warm_actions,warm_boxes,warm_meta
        diagnostics=""
        if len(bad_initial):
            worst=sorted(bad_initial,key=lambda i:initial_violation[i],reverse=True)[:5]
            diagnostics=" initial_infeasible=%d max=%.4g constraints=%s" % (
                len(bad_initial),float(initial_violation.max()),
                [opt.debug.g_describe(int(i)) for i in worst])
        raise RuntimeError("joint_minlp_failed:%s%s"%(str(exc),diagnostics)) from exc
    finally:
        if os.getcwd()!=original_cwd:
            os.chdir(original_cwd)
        if solver_dir is not None:
            shutil.rmtree(solver_dir,ignore_errors=True)
    solver_stats=sol.stats()
    solver_status=str(solver_stats.get("return_status","unknown"))
    solve_seconds=time.perf_counter()-solve_begin
    solved_constraint_violation_count=None
    solved_constraint_violation_max=None
    solved_constraint_violation_records=[]
    try:
        solved_g=np.asarray(sol.value(opt.g),dtype=float).reshape(-1)
        solved_gl=np.asarray(sol.value(opt.lbg),dtype=float).reshape(-1)
        solved_gu=np.asarray(sol.value(opt.ubg),dtype=float).reshape(-1)
        solved_violation=np.maximum(
            np.maximum(solved_gl-solved_g,0.0),solved_g-solved_gu)
        solved_constraint_violation_max=(
            float(solved_violation.max()) if len(solved_violation) else 0.0)
        bad_solved=np.flatnonzero(solved_violation>1e-5)
        solved_constraint_violation_count=int(len(bad_solved))
        worst_solved=sorted(
            bad_solved,key=lambda i:solved_violation[i],reverse=True)[:4]
        solved_constraint_violation_records=[
            "%s value=%.6g bounds=[%.6g,%.6g]" % (
                opt.debug.g_describe(int(i)).splitlines()[-1].strip(),
                float(solved_g[i]),float(solved_gl[i]),float(solved_gu[i]))
            for i in worst_solved]
    except Exception:
        # Some plugin return objects do not expose every auxiliary value.
        # Keep the physical replay and exact sweep as the acceptance checks.
        pass
    solved_objective=float(sol.value(objective))
    chosen=np.asarray(sol.value(mode),dtype=float).reshape((3,n))
    solved_gear=np.asarray(sol.value(gear_bit),dtype=float).reshape(-1)
    direction_change_sol=np.asarray(sol.value(direction_change),
                                    dtype=float).reshape(-1)
    if np.max(np.abs(chosen-np.rint(chosen)))>1e-5:
        raise ValueError("joint_noninteger_incumbent:%s"%solver_status)
    selected_schedule=tuple(
        (int(np.argmax(chosen[:,k])),int(round(solved_gear[k])))
        for k in range(n))
    selected_mode_word="".join(str(mode_id)
                                for mode_id,_ in selected_schedule)
    selected_gear_word="".join("+" if gear==1 else "-"
                                for _,gear in selected_schedule)
    ds_ack_sol=np.asarray(sol.value(ds_ack)).reshape(-1)
    ds_obl_sol=np.asarray(sol.value(ds_obl)).reshape(-1)
    turns=np.asarray(sol.value(dtheta)).reshape(-1)
    ks=np.asarray(sol.value(kappa)).reshape(-1);phis=np.asarray(sol.value(phi)).reshape(-1)
    x_sol=np.asarray(sol.value(X),dtype=float).reshape((3,n+1))
    spin_abs_sol=np.asarray(sol.value(spin_abs),dtype=float).reshape(-1)
    kappa_abs_sol=np.asarray(sol.value(kappa_abs),dtype=float).reshape(-1)
    phi_abs_sol=np.asarray(sol.value(phi_abs),dtype=float).reshape(-1)
    kappa_tv_sol=np.asarray(sol.value(kappa_tv),dtype=float).reshape(-1)
    phi_tv_sol=np.asarray(sol.value(phi_tv),dtype=float).reshape(-1)
    mode_change_sol=np.asarray(sol.value(mode_change),dtype=float).reshape(-1)
    # Independently reconstruct the scalar objective from the returned
    # variables. Bonmin/CasADi objective values should equal this sum; keeping
    # the terms separate makes sign or variable-order errors diagnosable.
    model_objective_components={name:0.0 for name in (
        "distance","spin","reverse","direction","curvature","switch","oblique_use",
        "oblique_cruise","steer_tv","oblique_tv","oblique_event",
        "low_curvature_switch","terminal_xy","terminal_yaw",
        "track_position","track_yaw")}
    for k in range(n):
        model_objective_components["distance"]+=(
            float(ds_ack_sol[k])+float(ds_obl_sol[k]))
        model_objective_components["spin"]+=(
            float(w["spin"])*float(spin_abs_sol[k]))
        if selected_schedule[k][1]==0:
            model_objective_components["reverse"]+=(
                float(w["reverse"])*(float(ds_ack_sol[k])+float(ds_obl_sol[k])))
        model_objective_components["direction"]+=(
            float(w["direction"])*float(direction_change_sol[k]))
        model_objective_components["curvature"]+=(
            float(w["curvature"])*float(ds_ack_sol[k])*
            float(kappa_abs_sol[k])**2)
        model_objective_components["oblique_use"]+=(
            float(w["oblique_use"])*float(ds_obl_sol[k])*
            float(phi_abs_sol[k]))
        model_objective_components["oblique_cruise"]+=(
            float(w["oblique_cruise"])*float(ds_obl_sol[k])*
            math.exp(-.5*(float(phis[k])/O_CRUISE_SIGMA)**2))
        model_objective_components["steer_tv"]+=(
            float(w["steer_tv"])*float(kappa_tv_sol[k]))
        model_objective_components["oblique_tv"]+=(
            float(w["oblique_tv"])*float(phi_tv_sol[k]))
        if k>0:
            agreement_switch=1.0-float(np.dot(chosen[:,k-1],chosen[:,k]))
            model_objective_components["switch"]+=(
                float(w["switch"])*.5*(float(mode_change_sol[k])+agreement_switch))
            phi_delta=math.sqrt((float(phis[k])-float(phis[k-1]))**2+1e-10)
            event=phi_delta/(phi_delta+.03)
            model_objective_components["oblique_event"]+=(
                float(w["oblique_event"])*event*
                float(chosen[1,k])*float(chosen[1,k-1]))
            previous_tangent=(float(chosen[0,k-1])*float(x_sol[2,k])+ 
                              float(chosen[1,k-1])*(float(x_sol[2,k-1])+float(phis[k-1])))
            current_tangent=(float(chosen[0,k])*float(x_sol[2,k])+ 
                             float(chosen[1,k])*(float(x_sol[2,k])+float(phis[k])))
            tangent_delta=math.atan2(
                math.sin(current_tangent-previous_tangent),
                math.cos(current_tangent-previous_tangent))
            tangent_delta_abs=math.sqrt(tangent_delta*tangent_delta+1e-10)
            turn_threshold=max(1e-3,float(w["switch_turn_threshold"]))
            transition_center=max(
                0.0,turn_threshold-min(.02,.1*turn_threshold))
            low_curvature_indicator=1.0/(1.0+math.exp(
                60*(tangent_delta_abs-transition_center)))
            ao_switch=(float(chosen[0,k-1])*float(chosen[1,k])+ 
                       float(chosen[1,k-1])*float(chosen[0,k]))
            model_objective_components["low_curvature_switch"]+=(
                float(w["low_curvature_switch"])*
                ao_switch*low_curvature_indicator)
    model_objective_components["terminal_xy"]=(
        float(terminal_xy_weight)*((float(x_sol[0,n])-goal.x)**2+
                                   (float(x_sol[1,n])-goal.y)**2))
    model_objective_components["terminal_yaw"]=(
        float(terminal_yaw_weight)*(float(x_sol[2,n])-goal_yaw)**2)
    for k,p in enumerate(refs):
        model_objective_components["track_position"]+=(
            float(w["track"])*((float(x_sol[0,k])-p.x)**2+
                                (float(x_sol[1,k])-p.y)**2))
    model_objective_components["track_yaw"]=(
        float(w["track"])*.25*float(np.sum((x_sol[2,:]-np.asarray(yaw_ref))**2)))
    model_objective_recomputed=float(sum(model_objective_components.values()))
    model_objective_delta=float(solved_objective-model_objective_recomputed)
    if (not math.isfinite(solved_objective) or solved_objective<0.0 or
            abs(model_objective_delta)>1e-5):
        print("[v3 optimizer] model objective audit reported=%.9g "
              "recomputed=%.9g delta=%.9g terms=%s" % (
                  solved_objective,model_objective_recomputed,
                  model_objective_delta,model_objective_components),flush=True)
        print("[v3 optimizer] model objective oblique audit weight=%.9g "
              "raw_products=%s modes=%s ds_obl=%s phi=%s phi_abs=%s" % (
                  float(w["oblique_use"]),
                  [float(ds_obl_sol[k])*float(phi_abs_sol[k])
                   for k in range(n)],
                  np.argmax(chosen,axis=0).astype(int).tolist(),
                  np.round(ds_obl_sol,8).tolist(),
                  np.round(phis,8).tolist(),
                  np.round(phi_abs_sol,8).tolist()),flush=True)
    refined=[]
    for k in range(n):
        m=int(np.argmax(chosen[:,k]))
        gear=1 if selected_schedule[k][1]==1 else -1
        if m==0: refined.append(Action(0,gear*float(ds_ack_sol[k]),float(ks[k])))
        elif m==1: refined.append(Action(1,gear*float(ds_obl_sol[k]),float(phis[k])))
        else: refined.append(Action(2,float(turns[k]),0.0))
    poses=replay(start,refined)
    terminal_repaired=False
    terminal_position_error=math.hypot(poses[-1].x-goal.x,
                                       poses[-1].y-goal.y)
    terminal_yaw_error=abs(wrap(poses[-1].th-goal.th))
    if (terminal_position_error>joint_xy_tol+terminal_solver_slack+1e-5 or
            terminal_yaw_error>joint_yaw_tol+terminal_solver_slack+1e-5):
        # A limited Bonmin solve may expose an integer incumbent whose
        # continuous values are slightly infeasible. Project only its final
        # primitive exactly onto the goal, then apply the normal bounds and
        # swept-footprint checks. No collision or endpoint slack is added.
        prefix=refined[:-1]
        prefix_end=replay(start,prefix)[-1]
        repair_modes=[refined[-1].mode]+[
            candidate_mode for candidate_mode in (0,1,2)
            if candidate_mode!=refined[-1].mode]
        repaired_candidates=[]
        for candidate_mode in repair_modes:
            terminal=fit_action_to_pose(candidate_mode,prefix_end,goal,
                                        tolerance=.005)
            if terminal is None:
                continue
            limit=max_spin if candidate_mode==2 else max_translation
            if (abs(terminal.amount)<.005-1e-9 or
                    abs(terminal.amount)>limit+1e-9):
                continue
            candidate=prefix+[terminal]
            candidate_poses=replay(start,candidate)
            if not grid.trajectory_clear(candidate_poses,candidate,
                                        joint_sweep_step):
                continue
            repaired_candidates.append((
                sequence_cost(candidate,weights)["score"],candidate,
                candidate_poses))
        if not repaired_candidates:
            mode_word="".join(str(action.mode) for action in refined)
            amounts=",".join("%.3f"%action.amount for action in refined)
            schedule_repair_error=None
            # A Bonmin limited solve can return a useful integral mode word
            # even when its continuous iterate misses the terminal box. Try
            # that exact word in a bounded fixed-schedule NLP before discarding
            # it. The candidate is still accepted only after the common
            # terminal and swept-footprint checks in the fixed-schedule path.
            if (fixed_schedule is None and
                    all(mode_id==2 for mode_id,_ in selected_schedule)):
                # A spin-only word cannot change position. Bonmin's limited
                # non-incumbent iterates can expose this integer default even
                # though it cannot satisfy the hard terminal position box.
                schedule_repair_error=(
                    "bonmin_schedule_has_no_translation")
            elif (fixed_schedule is None and
                    tuple(selected_schedule)!=tuple(seed_schedule)):
                repair_begin=time.perf_counter()
                try:
                    schedule_repair=joint_refine(
                        start,goal,seed_actions,grid,xy_tol=xy_tol,
                        yaw_tol=yaw_tol,max_translation=max_translation,
                        max_spin=max_spin,max_knots=max_knots,
                        ackermann_merge_tolerance=ackermann_merge_tolerance,
                        oblique_merge_tolerance=oblique_merge_tolerance,
                        time_limit=max(.1,float(collision_repair_time_limit)),
                        terminal_xy_tol=terminal_xy_tol,
                        terminal_yaw_tol=terminal_yaw_tol,
                        terminal_xy_weight=terminal_xy_weight,
                        terminal_yaw_weight=terminal_yaw_weight,
                        weights=weights,algorithm=algorithm,
                        variable_selection=variable_selection,
                        node_comparison=node_comparison,
                        subproblem_time_limit=max(
                            float(subproblem_time_limit),
                            float(collision_repair_time_limit)),
                        bonmin_iteration_limit=bonmin_iteration_limit,
                        bonmin_subproblem_iteration_limit=(
                            bonmin_subproblem_iteration_limit),
                        bonmin_unsolved_nlp_retries=(
                            bonmin_unsolved_nlp_retries),
                        use_seed_cutoff=use_seed_cutoff,
                        collision_spacing=collision_spacing,
                        adaptive_collision_refinement=(
                            adaptive_collision_refinement),
                        corridor_clearance_slack=corridor_clearance_slack,
                        geometry_yaw_bounds=geometry_yaw_bounds,
                        geometry_spin_pruning=geometry_spin_pruning,
                        geometry_oblique_bounds=geometry_oblique_bounds,
                        use_feasibility_pump=use_feasibility_pump,
                        use_minlp_pump=use_minlp_pump,
                        root_resolves=root_resolves,
                        dive_fractional=dive_fractional,
                        random_seed=random_seed,
                        collision_retries=min(
                            1,max(0,int(collision_repair_iterations))),
                        collision_repair_iterations=0,
                        collision_repair_time_limit=(
                            collision_repair_time_limit),
                        warm_start_time_limit=warm_start_time_limit,
                        warm_start_early_accept_improvement=(
                            warm_start_early_accept_improvement),
                        use_rins_heuristic=use_rins_heuristic,
                        use_feasible_seed_warm_start=False,
                        _fixed_schedule=selected_schedule,
                        _initial_actions=refined)
                    repair_meta=schedule_repair[3]
                    repair_meta["fallback_reason"]=(
                        "bonmin_integer_schedule_terminal_repair")
                    repair_meta["bonmin_rins_heuristic_enabled"]=(
                        bool(use_rins_heuristic))
                    repair_meta["bonmin_returned_modes"]=(
                        selected_mode_word)
                    repair_meta["bonmin_returned_gears"]=(
                        selected_gear_word)
                    repair_meta["bonmin_returned_schedule"]=(
                        [list(pair) for pair in selected_schedule])
                    repair_meta["bonmin_returned_position_error"]=(
                        terminal_position_error)
                    repair_meta["bonmin_status"]=solver_status
                    repair_meta["bonmin_solve_seconds"]=solve_seconds
                    repair_meta["bonmin_schedule_repair_wall_seconds"]=(
                        time.perf_counter()-repair_begin)
                    repair_meta["bonmin_feasible_seed_warm_start"]=(
                        warm_start_result is not None)
                    repair_meta["bonmin_warm_start_error"]=warm_start_error
                    repair_meta["bonmin_warm_start_wall_seconds"]=(
                        warm_start_wall_seconds)
                    repair_meta["bonmin_root_initial_point_feasible"]=(
                        initial_point_feasible)
                    repair_meta["bonmin_root_initial_violation_count"]=(
                        int(len(bad_initial)))
                    repair_meta["bonmin_root_initial_violation_max"]=(
                        float(initial_violation.max())
                        if len(bad_initial) else 0.0)
                    repair_meta["bonmin_root_initial_violation_records"]=(
                        initial_violation_records)
                    repair_meta["bonmin_root_seed_cutoff_enabled"]=(
                        bool(use_seed_cutoff and cutoff_eligible))
                    repair_meta.update(bonmin_budget_meta)
                    repair_score=sequence_cost(
                        schedule_repair[1],weights)["score"]
                    warm_score=(sequence_cost(
                        warm_start_result[1],weights)["score"]
                        if warm_start_result is not None else float("inf"))
                    if repair_score < warm_score-1e-9:
                        return schedule_repair
                except Exception as repair_exc:
                    schedule_repair_error=(
                        str(repair_exc).splitlines()[-1])
            # The fixed-schedule warm start has already passed the same
            # terminal and swept checks as any returned candidate. Retain it
            # when Bonmin has no terminal incumbent instead of solving the
            # seed schedule again from the poorer coarse point.
            if warm_start_result is not None:
                warm_poses,warm_actions,warm_boxes,warm_meta=warm_start_result
                warm_meta["solver"]="ipopt_feasible_seed_fallback"
                warm_meta["fallback_reason"]="bonmin_no_terminal_incumbent"
                warm_meta["bonmin_rins_heuristic_enabled"]=(
                    bool(use_rins_heuristic))
                warm_meta["bonmin_status"]=solver_status
                warm_meta["bonmin_returned_modes"]=selected_mode_word
                warm_meta["bonmin_returned_gears"]=selected_gear_word
                warm_meta["bonmin_returned_schedule"]=([
                    list(pair) for pair in selected_schedule]
                    if fixed_schedule is None else None)
                warm_meta["bonmin_returned_position_error"]=(
                    terminal_position_error)
                warm_meta["bonmin_schedule_repair_error"]=(
                    schedule_repair_error)
                warm_meta["bonmin_feasible_seed_warm_start"]=True
                warm_meta["bonmin_warm_start_error"]=warm_start_error
                warm_meta["bonmin_solve_seconds"]=solve_seconds
                warm_meta["bonmin_warm_start_wall_seconds"]=(
                    warm_start_wall_seconds)
                warm_meta["bonmin_root_initial_point_feasible"]=(
                    initial_point_feasible)
                warm_meta["bonmin_root_initial_violation_count"]=(
                    int(len(bad_initial)))
                warm_meta["bonmin_root_initial_violation_max"]=(
                    float(initial_violation.max())
                    if len(initial_violation) else 0.0)
                warm_meta["bonmin_root_initial_violation_records"]=(
                    initial_violation_records)
                warm_meta["bonmin_root_seed_cutoff_enabled"]=(
                    bool(use_seed_cutoff and cutoff_eligible))
                warm_meta["bonmin_translation_distance_lb"]=(
                    float(translation_distance_lb))
                warm_meta.update(bonmin_budget_meta)
                return warm_poses,warm_actions,warm_boxes,warm_meta
            # A limited Bonmin solve can return its default all-spin point
            # without an integer incumbent.  That point cannot be projected
            # onto a distant goal, so give the already swept-safe seed
            # schedule one continuous, corridor-constrained optimization
            # attempt.  This is a clearly labelled fallback: it does not
            # claim that Bonmin selected the schedule.
            if (fixed_schedule is None and
                    int(collision_repair_iterations)>0):
                # Keep the no-incumbent fallback from consuming the entire
                # MINLP-candidate repair budget.  The selected Bonmin schedule
                # still gets the full configured repair count below; this cap
                # only bounds the weaker seed-schedule fallback and its tail.
                seed_fallback_retries=min(
                    4,max(0,int(collision_repair_iterations)))
                try:
                    fallback=joint_refine(
                        start,goal,seed_actions,grid,xy_tol=xy_tol,yaw_tol=yaw_tol,
                        max_translation=max_translation,max_spin=max_spin,
                        max_knots=max_knots,
                        ackermann_merge_tolerance=ackermann_merge_tolerance,
                        oblique_merge_tolerance=oblique_merge_tolerance,
                        time_limit=max(.1,float(collision_repair_time_limit)),
                        terminal_xy_tol=terminal_xy_tol,
                        terminal_yaw_tol=terminal_yaw_tol,
                        terminal_xy_weight=terminal_xy_weight,
                        terminal_yaw_weight=terminal_yaw_weight,weights=weights,
                        algorithm=algorithm,variable_selection=variable_selection,
                        node_comparison=node_comparison,
                        subproblem_time_limit=subproblem_time_limit,
                        bonmin_iteration_limit=bonmin_iteration_limit,
                        bonmin_subproblem_iteration_limit=(
                            bonmin_subproblem_iteration_limit),
                        bonmin_unsolved_nlp_retries=(
                            bonmin_unsolved_nlp_retries),
                        use_seed_cutoff=use_seed_cutoff,
                        collision_spacing=collision_spacing,
                        adaptive_collision_refinement=adaptive_collision_refinement,
                        corridor_clearance_slack=corridor_clearance_slack,
                        geometry_yaw_bounds=geometry_yaw_bounds,
                        geometry_spin_pruning=geometry_spin_pruning,
                        geometry_oblique_bounds=geometry_oblique_bounds,
                        use_feasibility_pump=use_feasibility_pump,
                        use_minlp_pump=use_minlp_pump,
                        root_resolves=root_resolves,
                        dive_fractional=dive_fractional,
                        random_seed=random_seed,
                        collision_retries=seed_fallback_retries,
                        collision_repair_iterations=0,
                        collision_repair_time_limit=collision_repair_time_limit,
                        warm_start_time_limit=warm_start_time_limit,
                        warm_start_early_accept_improvement=(
                            warm_start_early_accept_improvement),
                        use_rins_heuristic=use_rins_heuristic,
                        _fixed_schedule=seed_schedule,
                        _initial_actions=actions)
                    fallback[3]["solver"]="ipopt_seed_schedule_fallback"
                    fallback[3]["fallback_reason"]=(
                        "bonmin_no_terminal_incumbent")
                    fallback[3]["seed_fallback_collision_retries"]=(
                        seed_fallback_retries)
                    fallback[3]["bonmin_status"]=solver_status
                    fallback[3]["bonmin_returned_modes"]=selected_mode_word
                    fallback[3]["bonmin_returned_gears"]=selected_gear_word
                    fallback[3]["bonmin_returned_schedule"]=([
                        list(pair) for pair in selected_schedule]
                        if fixed_schedule is None else None)
                    fallback[3]["bonmin_returned_position_error"]=(
                        terminal_position_error)
                    fallback[3]["bonmin_feasible_seed_warm_start"]=(
                        warm_start_meta is not None)
                    fallback[3]["bonmin_warm_start_error"]=warm_start_error
                    fallback[3]["bonmin_solve_seconds"]=solve_seconds
                    fallback[3]["bonmin_warm_start_wall_seconds"]=(
                        warm_start_wall_seconds)
                    fallback[3]["bonmin_root_initial_point_feasible"]=(
                        initial_point_feasible)
                    fallback[3]["bonmin_root_initial_violation_count"]=(
                        int(len(bad_initial)))
                    fallback[3]["bonmin_root_initial_violation_max"]=(
                        float(initial_violation.max())
                        if len(initial_violation) else 0.0)
                    fallback[3]["bonmin_root_initial_violation_records"]=(
                        initial_violation_records)
                    fallback[3]["bonmin_root_seed_cutoff_enabled"]=(
                        bool(use_seed_cutoff and cutoff_eligible))
                    fallback[3]["bonmin_translation_distance_lb"]=(
                        float(translation_distance_lb))
                    fallback[3].update(bonmin_budget_meta)
                    return fallback
                except Exception as fallback_exc:
                    fallback_failure=str(fallback_exc).splitlines()[-1]
            else:
                fallback_failure=None
            raise ValueError(
                "joint_terminal_projection_failed:status=%s,solve_s=%.3f,"
                "pos=%.5f,yaw=%.5f,modes=%s,amounts=%s,end=(%.3f,%.3f,%.3f),"
                "seed_fallback=%s" % (
                    solver_status,solve_seconds,
                    terminal_position_error,terminal_yaw_error,
                    mode_word,amounts,poses[-1].x,poses[-1].y,poses[-1].th,
                    fallback_failure))
        _,refined,poses=min(repaired_candidates,key=lambda item:item[0])
        terminal_repaired=True
    if (math.hypot(poses[-1].x-goal.x,poses[-1].y-goal.y)>
            joint_xy_tol+terminal_solver_slack+1e-5):
        raise ValueError("joint_terminal_position_violation")
    if (abs(wrap(poses[-1].th-goal.th))>
            joint_yaw_tol+terminal_solver_slack+1e-5):
        raise ValueError("joint_terminal_yaw_violation")
    if not grid.trajectory_clear(poses,refined,joint_sweep_step):
        bad_segments=[k for k,(p,a) in enumerate(zip(poses,refined))
                      if not grid.action_clear(p,a,joint_sweep_step)]
        mode_word="".join(str(action.mode) for action in refined)
        amounts=",".join("%.3f"%action.amount for action in refined)
        controls=",".join("%.6f"%action.control for action in refined)
        schedule=selected_schedule
        collision_planes=list(_collision_planes)
        for k in bad_segments:
            planes=_collision_planes_on_sweep(
                grid,poses[k],refined[k],refs[k],actions[k],
                # Cut discovery is heuristic; the independent candidate
                # acceptance sweep below remains at 2.5 mm.
                max_corner_step=.01,max_planes=4,
                min_spacing_m=.20,
                existing_planes=[plane[1:] for plane in _collision_planes
                                 if int(plane[0])==k])
            for plane in planes:
                fraction,cell_x,cell_y,nx,ny,required=plane
                collision_planes.append((k,round(fraction,6),cell_x,cell_y,
                                         nx,ny,required))
        collision_planes=tuple(sorted(set(collision_planes)))
        blocked_schedules=tuple(_forbidden_schedules)
        # Prefer a collision-specific geometry cut when one is available.
        # Excluding the entire integer schedule would also remove safe
        # continuous realizations of the same mode/gear pattern.
        if (fixed_schedule is None and
                len(collision_planes)==len(_collision_planes) and
                schedule!=seed_schedule and
                schedule not in blocked_schedules):
            blocked_schedules+=(schedule,)
        new_collision_information=(
            len(blocked_schedules)>len(_forbidden_schedules) or
            len(collision_planes)>len(_collision_planes))
        if (int(collision_retries)>0 and new_collision_information and
                (fixed_schedule is not None or
                 int(collision_repair_iterations)<=0) and
                float(time_limit)>=.10):
            if fixed_schedule is not None:
                return joint_refine(
                    start,goal,seed_actions,grid,xy_tol=xy_tol,yaw_tol=yaw_tol,
                    max_translation=max_translation,max_spin=max_spin,
                    max_knots=max_knots,
                    ackermann_merge_tolerance=ackermann_merge_tolerance,
                    oblique_merge_tolerance=oblique_merge_tolerance,
                    time_limit=time_limit,
                    terminal_xy_tol=terminal_xy_tol,
                    terminal_yaw_tol=terminal_yaw_tol,
                    terminal_xy_weight=terminal_xy_weight,
                    terminal_yaw_weight=terminal_yaw_weight,weights=weights,
                    algorithm=algorithm,variable_selection=variable_selection,
                    node_comparison=node_comparison,
                    subproblem_time_limit=subproblem_time_limit,
                    bonmin_iteration_limit=bonmin_iteration_limit,
                    bonmin_subproblem_iteration_limit=(
                        bonmin_subproblem_iteration_limit),
                    bonmin_unsolved_nlp_retries=(
                        bonmin_unsolved_nlp_retries),
                    use_seed_cutoff=use_seed_cutoff,
                    collision_spacing=collision_spacing,
                    adaptive_collision_refinement=adaptive_collision_refinement,
                    corridor_clearance_slack=corridor_clearance_slack,
                    geometry_yaw_bounds=geometry_yaw_bounds,
                    geometry_spin_pruning=geometry_spin_pruning,
                    geometry_oblique_bounds=geometry_oblique_bounds,
                    use_feasibility_pump=use_feasibility_pump,
                    use_minlp_pump=use_minlp_pump,
                    root_resolves=root_resolves,
                    dive_fractional=dive_fractional,
                    random_seed=random_seed,
                    collision_retries=int(collision_retries)-1,
                    collision_repair_iterations=0,
                    collision_repair_time_limit=collision_repair_time_limit,
                    warm_start_time_limit=warm_start_time_limit,
                    warm_start_early_accept_improvement=(
                        warm_start_early_accept_improvement),
                    use_rins_heuristic=use_rins_heuristic,
                    _collision_planes=collision_planes,
                    _fixed_schedule=fixed_schedule,
                    _initial_actions=refined)
            return joint_refine(
                start,goal,seed_actions,grid,xy_tol=xy_tol,yaw_tol=yaw_tol,
                max_translation=max_translation,max_spin=max_spin,
                max_knots=max_knots,
                ackermann_merge_tolerance=ackermann_merge_tolerance,
                oblique_merge_tolerance=oblique_merge_tolerance,
                time_limit=time_limit,
                terminal_xy_tol=terminal_xy_tol,
                terminal_yaw_tol=terminal_yaw_tol,
                terminal_xy_weight=terminal_xy_weight,
                terminal_yaw_weight=terminal_yaw_weight,weights=weights,
                algorithm=algorithm,variable_selection=variable_selection,
                node_comparison=node_comparison,
                subproblem_time_limit=subproblem_time_limit,
                bonmin_iteration_limit=bonmin_iteration_limit,
                bonmin_subproblem_iteration_limit=(
                    bonmin_subproblem_iteration_limit),
                bonmin_unsolved_nlp_retries=(
                    bonmin_unsolved_nlp_retries),
                use_seed_cutoff=use_seed_cutoff,
                collision_spacing=collision_spacing,
                adaptive_collision_refinement=adaptive_collision_refinement,
                corridor_clearance_slack=corridor_clearance_slack,
                geometry_yaw_bounds=geometry_yaw_bounds,
                geometry_spin_pruning=geometry_spin_pruning,
                geometry_oblique_bounds=geometry_oblique_bounds,
                use_feasibility_pump=use_feasibility_pump,
                use_minlp_pump=use_minlp_pump,
                root_resolves=root_resolves,
                dive_fractional=dive_fractional,
                random_seed=random_seed,
                collision_retries=int(collision_retries)-1,
                collision_repair_iterations=collision_repair_iterations,
                collision_repair_time_limit=collision_repair_time_limit,
                warm_start_time_limit=warm_start_time_limit,
                warm_start_early_accept_improvement=(
                    warm_start_early_accept_improvement),
                _forbidden_schedules=blocked_schedules,
                _collision_planes=collision_planes,
                use_rins_heuristic=use_rins_heuristic)
        repair_failure=None
        repair_wall_seconds=0.0
        if (fixed_schedule is None and
                int(collision_repair_iterations)>0 and
                len(collision_planes)>len(_collision_planes)):
            repair_begin=time.perf_counter()
            try:
                repair_result=joint_refine(
                    start,goal,seed_actions,grid,xy_tol=xy_tol,yaw_tol=yaw_tol,
                    max_translation=max_translation,max_spin=max_spin,
                    max_knots=max_knots,
                    ackermann_merge_tolerance=ackermann_merge_tolerance,
                    oblique_merge_tolerance=oblique_merge_tolerance,
                    time_limit=max(.1,float(collision_repair_time_limit)),
                    terminal_xy_tol=terminal_xy_tol,
                    terminal_yaw_tol=terminal_yaw_tol,
                    terminal_xy_weight=terminal_xy_weight,
                    terminal_yaw_weight=terminal_yaw_weight,weights=weights,
                    algorithm=algorithm,variable_selection=variable_selection,
                    node_comparison=node_comparison,
                    # Let the fixed-schedule repair use its declared wall
                    # budget even when Bonmin's node NLPs have a shorter cap.
                    subproblem_time_limit=max(
                        float(subproblem_time_limit),
                        float(collision_repair_time_limit)),
                    bonmin_iteration_limit=bonmin_iteration_limit,
                    bonmin_subproblem_iteration_limit=(
                        bonmin_subproblem_iteration_limit),
                    bonmin_unsolved_nlp_retries=(
                        bonmin_unsolved_nlp_retries),
                    use_seed_cutoff=use_seed_cutoff,
                    collision_spacing=collision_spacing,
                    adaptive_collision_refinement=adaptive_collision_refinement,
                    corridor_clearance_slack=corridor_clearance_slack,
                    geometry_yaw_bounds=geometry_yaw_bounds,
                    geometry_spin_pruning=geometry_spin_pruning,
                    geometry_oblique_bounds=geometry_oblique_bounds,
                    use_feasibility_pump=use_feasibility_pump,
                    use_minlp_pump=use_minlp_pump,
                    root_resolves=root_resolves,
                    dive_fractional=dive_fractional,
                    random_seed=random_seed,
                    collision_retries=int(collision_repair_iterations),
            collision_repair_iterations=0,
            collision_repair_time_limit=collision_repair_time_limit,
            warm_start_time_limit=warm_start_time_limit,
            warm_start_early_accept_improvement=(
                warm_start_early_accept_improvement),
            use_rins_heuristic=use_rins_heuristic,
            _collision_planes=collision_planes,
                    _fixed_schedule=schedule,
                    _initial_actions=refined)
                repair_wall_seconds=time.perf_counter()-repair_begin
                repair_meta=repair_result[3]
                repair_meta["bonmin_selected_schedule"]=[list(pair)
                                                          for pair in schedule]
                repair_meta["bonmin_status"]=solver_status
                repair_meta["bonmin_solve_seconds"]=solve_seconds
                repair_meta["bonmin_feasible_seed_warm_start"]=(
                    warm_start_meta is not None)
                repair_meta["bonmin_warm_start_error"]=warm_start_error
                repair_meta["bonmin_warm_start_wall_seconds"]=(
                    warm_start_wall_seconds)
                repair_meta["bonmin_root_initial_point_feasible"]=(
                    initial_point_feasible)
                repair_meta["bonmin_root_initial_violation_count"]=(
                    int(len(bad_initial)))
                repair_meta["bonmin_root_initial_violation_max"]=(
                    float(initial_violation.max())
                    if len(initial_violation) else 0.0)
                repair_meta["bonmin_root_initial_violation_records"]=(
                    initial_violation_records)
                repair_meta["bonmin_root_seed_cutoff_enabled"]=(
                    bool(use_seed_cutoff and cutoff_eligible))
                repair_meta["bonmin_seed_incumbent_plugin"]=(
                    bonmin_seed_incumbent_status)
                repair_meta["collision_repair_wall_seconds"]=(
                    repair_wall_seconds)
                repair_meta.update(bonmin_budget_meta)
                return repair_result
            except Exception as exc:
                repair_wall_seconds=time.perf_counter()-repair_begin
                repair_failure=str(exc).splitlines()[-1]
        raise ValueError(
            "joint_solution_swept_sat_collision:segments=%s,modes=%s,"
            "amounts=%s,controls=%s,planes=%d,new_planes=%d,repair=%s,"
            "bonmin_solve_s=%.3f,warm_start_s=%.3f,repair_wall_s=%.3f,"
            "feasible_seed_warm_start=%s,warm_error=%s"%
            (bad_segments,mode_word,amounts,controls,len(_collision_planes),
             max(0,len(collision_planes)-len(_collision_planes)),
             repair_failure,solve_seconds,warm_start_wall_seconds,
             repair_wall_seconds,warm_start_meta is not None,
             warm_start_error))
    score=sequence_cost(refined,weights)
    meta={"corridor":"disc" if covered_discs else "mixed_disc_point",
          "solver":("ipopt_fixed_schedule_repair" if fixed_schedule is not None
                    else "bonmin_minlp"),
          "knots":n,"objective":solved_objective,
          "model_objective_recomputed":model_objective_recomputed,
          "model_objective_delta":model_objective_delta,
          "model_objective_components":model_objective_components,
          "model_constraint_violation_count":(
              solved_constraint_violation_count),
          "model_constraint_violation_max":(
              solved_constraint_violation_max),
          "model_constraint_violation_records":(
              solved_constraint_violation_records),
          "fixed_schedule":(list(fixed_schedule)
                            if fixed_schedule is not None else None),
          "bonmin_rins_heuristic_enabled":bool(use_rins_heuristic),
          "bonmin_seed_incumbent_plugin":bonmin_seed_incumbent_status,
          "collision_substeps":collision_substeps,
          "collision_substeps_per_segment":segment_substeps,
          "swept_validation_step_m":joint_sweep_step,
          "adaptive_collision_refinement":bool(adaptive_collision_refinement),
          "corridor_clearance_slack":max(0.0,float(corridor_clearance_slack)),
          "adaptively_refined_segments":[k for k,steps in
              enumerate(segment_substeps) if steps>collision_substeps],
          "collision_retries_used":len(_forbidden_schedules),
          "collision_planes_used":len(_collision_planes),
          "solver_status":solver_status,"solve_seconds":solve_seconds,
          "requested_time_limit":float(time_limit),
          "model_setup_seconds":float(setup_elapsed),
          "effective_solver_time_limit":float(solver_time_limit),
          "subproblem_wall_time_limit":float(subproblem_wall_limit),
          "minlp_pump_time_limit":float(minlp_pump_time_limit),
          "bonmin_selected_schedule":([
              list(pair) for pair in selected_schedule]
              if fixed_schedule is None else None),
          "bonmin_returned_modes":(selected_mode_word
                                    if fixed_schedule is None else None),
          "bonmin_returned_gears":(selected_gear_word
                                    if fixed_schedule is None else None),
          "bonmin_subproblem_iteration_limit":max(
              1,int(bonmin_subproblem_iteration_limit)),
          "bonmin_unsolved_nlp_retries":bonmin_unsolved_nlp_retries,
          "translation_distance_lower_bound":float(
              translation_distance_lb),
          "geometry_derived_mode_bounds":geometry_bound_stats,
          "seed_objective":seed_objective,
          "initial_point_feasible":initial_point_feasible,
          "initial_violation_count":int(len(bad_initial)),
          "initial_violation_max":(float(initial_violation.max())
                                   if len(initial_violation) else 0.0),
          "initial_violation_records":initial_violation_records,
          "seed_cutoff_eligible":cutoff_eligible,
          "feasible_seed_warm_start":feasible_warm_start_used,
          "warm_start_wall_seconds":warm_start_wall_seconds,
          "warm_start_solver_seconds":(
              float(warm_start_meta.get("solve_seconds",0.0))
              if warm_start_meta is not None else 0.0),
          "warm_start_collision_retries":max(
              0,int(warm_start_collision_retries)),
          "warm_start_solver":(warm_start_meta.get("solver")
                                if warm_start_meta is not None else None),
          "warm_start_score":(warm_start_meta.get("score")
                               if warm_start_meta is not None else None),
          "warm_start_error":warm_start_error,
          "seed_cutoff_enabled":bool(use_seed_cutoff and cutoff_eligible and
                                      fixed_schedule is None),
          "feasibility_pump_enabled":bool(use_feasibility_pump and
                                            fixed_schedule is None),
          "minlp_pump_enabled":bool(use_minlp_pump and
                                     fixed_schedule is None),
          "root_resolves":int(root_resolves),
          "dive_fractional_enabled":bool(dive_fractional),
          "random_seed":int(random_seed),
          "terminal_repaired":terminal_repaired,
          "score":score["score"],"mode_switches":score["switches"],
          "direction_switches":score["direction_switches"],
          "spin_angle":score["spin_angle"],"reverse_length":score["reverse_length"],
          "seed_score":sequence_cost(actions,weights)["score"]}
    return poses,refined,boxes,meta


def refine(start, goal, seed_actions, grid, xy_tol=.16, yaw_tol=.14,
           max_iter=100, weights=None, allow_initial_collision=False,
           acceptance_score_limit=None, max_wall_time_s=None):
    """Fixed-sequence NLP minimizes the shared weighted motion score."""
    import casadi as ca
    profile={}
    total_begin=time.perf_counter()
    w={"spin":1.00,"reverse":1.00,"switch":2.00,"direction":20.0,
       "steer_tv":.40,"curvature":.01,"oblique_tv":1.50,"oblique_event":1.50,
       "oblique_use":.55,"oblique_cruise":.45,
       "terminal_xy":8.0,"terminal_yaw":2.0}
    if weights: w.update(weights)
    event_actions=collapse_exact_runs(seed_actions)
    # NLP knots follow control/mode events; intermediate samples are retained
    # for corridor construction and swept-SAT validation, not as variables.
    # Keep the continuous NLP compact: the exact primitive sweep is still
    # checked at a much finer resolution after replay, so these are only
    # optimization knots, not the collision-checking resolution.
    stage_begin=time.perf_counter()
    seed, actions = expand(start, event_actions, max_translation=1.0,
                           max_rotation=math.radians(30))
    profile["expand_s"]=time.perf_counter()-stage_begin
    stage_begin=time.perf_counter()
    if not grid.trajectory_clear(seed, actions) and not allow_initial_collision:
        if seed and not grid.pose_clear(seed[0]):
            raise ValueError("coarse_sat_collision_at_start")
        for index,(pose,action) in enumerate(zip(seed,actions)):
            if not grid.action_clear(pose,action):
                raise ValueError("coarse_sat_collision_edge_%d_mode_%d_amount_%.5f_control_%.5f_at_%.3f_%.3f_%.3f" %
                                 (index,action.mode,action.amount,action.control,
                                 pose.x,pose.y,pose.th))
        raise ValueError("coarse_sat_collision_unknown")
    profile["seed_sweep_s"]=time.perf_counter()-stage_begin
    stage_begin=time.perf_counter()
    feat, boxes,covered_discs = build_corridors(grid, seed)
    profile["corridor_s"]=time.perf_counter()-stage_begin
    n=len(actions)
    if n == 0:
        raise ValueError("empty_seed")
    build_begin=time.perf_counter()
    opt=ca.Opti()
    X=opt.variable(3,n+1); D=opt.variable(n); C=opt.variable(n)
    x,y,th=X[0,:],X[1,:],X[2,:]
    # IPOPT's exact abs() has a kink at zero; most seed controls include zero
    # steering/oblique angle, so use a near-identical smooth approximation.
    smooth_abs=lambda z: ca.sqrt(z*z+1e-10)
    opt.subject_to(X[:,0]==ca.vertcat(start.x,start.y,start.th))
    objective=0;last_mode=None;last_gear=None
    for k,a in enumerate(actions):
        nominal_min=math.radians(0.5) if a.mode==2 else .01
        minimum=min(nominal_min,max(1e-6,.5*abs(a.amount)))
        maximum=max(2*abs(a.amount),minimum*1.05)
        lo,hi=(minimum,maximum) if a.amount>0 else (-maximum,-minimum)
        opt.subject_to(opt.bounded(lo,D[k],hi))
        if last_mode is not None and a.mode!=last_mode:
            objective+=w["switch"]
        if a.mode==0:
            opt.subject_to(opt.bounded(-1/.810,C[k],1/.810))
            z=D[k]*C[k]/2
            sinc=_symbolic_sinc(z,ca)
            opt.subject_to(x[k+1]==x[k]+D[k]*sinc*ca.cos(th[k]+z))
            opt.subject_to(y[k+1]==y[k]+D[k]*sinc*ca.sin(th[k]+z))
            opt.subject_to(th[k+1]==th[k]+D[k]*C[k])
            objective+=smooth_abs(D[k])
            if a.amount<0: objective+=w["reverse"]*smooth_abs(D[k])
            objective+=w["curvature"]*C[k]*C[k]*smooth_abs(D[k])
            if last_mode==0:
                objective+=w["steer_tv"]*smooth_abs(C[k]-C[k-1])
        elif a.mode==1:
            opt.subject_to(opt.bounded(-math.pi/2,C[k],math.pi/2))
            opt.subject_to(x[k+1]==x[k]+D[k]*ca.cos(th[k]+C[k]))
            opt.subject_to(y[k+1]==y[k]+D[k]*ca.sin(th[k]+C[k]))
            opt.subject_to(th[k+1]==th[k])
            objective+=smooth_abs(D[k])
            if a.amount<0: objective+=w["reverse"]*smooth_abs(D[k])
            objective+=w["oblique_use"]*smooth_abs(D[k])*smooth_abs(C[k])
            objective+=w["oblique_cruise"]*smooth_abs(D[k])*ca.exp(
                -.5*(C[k]/O_CRUISE_SIGMA)**2)
            if last_mode==1:
                delta=smooth_abs(C[k]-C[k-1])
                objective+=w["oblique_tv"]*delta
                objective+=w["oblique_event"]*delta/(delta+.03)
        else:
            opt.subject_to(C[k]==0)
            opt.subject_to(x[k+1]==x[k]);opt.subject_to(y[k+1]==y[k])
            opt.subject_to(th[k+1]==th[k]+D[k])
            objective+=w["spin"]*smooth_abs(D[k])
        if a.mode!=2:
            gear=1 if a.amount>0 else -1
            if last_gear is not None and gear!=last_gear:
                objective+=w["direction"]
            last_gear=gear
        last_mode=a.mode
    for k,row in enumerate(boxes):
        c,s=ca.cos(th[k]),ca.sin(th[k])
        for (ox,oy,_),(cx,cy,half_long,half_lat,heading) in zip(feat,row):
            fx=x[k]+ox*c-oy*s;fy=y[k]+ox*s+oy*c
            ch,sh=math.cos(heading),math.sin(heading)
            along=(fx-cx)*ch+(fy-cy)*sh
            lateral=-(fx-cx)*sh+(fy-cy)*ch
            opt.subject_to(opt.bounded(-half_long,along,half_long))
            opt.subject_to(opt.bounded(-half_lat,lateral,half_lat))
    unwrapped=[seed[0].th]
    for p in seed[1:]: unwrapped.append(unwrapped[-1]+wrap(p.th-unwrapped[-1]))
    target_yaw=unwrapped[-1]+wrap(goal.th-unwrapped[-1])
    opt.subject_to((x[-1]-goal.x)**2+(y[-1]-goal.y)**2<=xy_tol**2)
    opt.subject_to(opt.bounded(target_yaw-yaw_tol,th[-1],target_yaw+yaw_tol))
    # The hard tolerance defines feasibility. These smooth endpoint terms keep
    # a cost-improving solution away from that boundary, leaving margin for
    # discretization and downstream tracking.
    objective+=w["terminal_xy"]*((x[-1]-goal.x)**2+(y[-1]-goal.y)**2)
    objective+=w["terminal_yaw"]*(th[-1]-target_yaw)**2
    objective+=.04*ca.sumsqr(x-ca.DM([p.x for p in seed]).T)
    objective+=.04*ca.sumsqr(y-ca.DM([p.y for p in seed]).T)
    for k in range(1,n):
        if actions[k].mode==actions[k-1].mode and actions[k].mode!=2:
            # Small proximal regularizer; weighted TV carries the shared score.
            objective+=.01*(C[k]-C[k-1])**2+.01*(D[k]-D[k-1])**2
    opt.set_initial(X,np.vstack(([p.x for p in seed],[p.y for p in seed],unwrapped)))
    opt.set_initial(D,[a.amount for a in actions]);opt.set_initial(C,[a.control for a in actions])
    opt.minimize(objective)
    solver_options={"print_level":0,"max_iter":max_iter,
                    "tol":1e-6,"acceptable_tol":1e-5,"sb":"yes"}
    if max_wall_time_s is not None:
        max_wall_time_s=float(max_wall_time_s)
        if not math.isfinite(max_wall_time_s) or max_wall_time_s<=0:
            raise ValueError("invalid_refine_wall_time")
        solver_options["max_wall_time"]=max_wall_time_s
    opt.solver("ipopt",{"print_time":False},solver_options)
    profile["nlp_build_s"]=time.perf_counter()-build_begin
    # Return the best bounded-iteration iterate when IPOPT reaches its cap.
    # It is never trusted blindly: the replay below must still satisfy terminal
    # tolerances, common-cost non-regression, and exact swept SAT.
    solve_begin=time.perf_counter()
    sol=opt.solve_limited()
    profile["nlp_solve_s"]=time.perf_counter()-solve_begin
    solver_stats=sol.stats()
    profile["nlp_iterations"]=int(solver_stats.get("iter_count",-1))
    profile["nlp_return_status"]=str(solver_stats.get("return_status","unknown"))
    check_begin=time.perf_counter()
    ds=np.asarray(sol.value(D)).reshape(-1);cs=np.asarray(sol.value(C)).reshape(-1)
    proposed=[Action(a.mode,float(ds[k]),float(cs[k])) for k,a in enumerate(actions)]
    # A point-feature fallback corridor is guidance, not a collision proof.
    # Backtrack in primitive-control space; accept only a non-worsening common
    # score that satisfies the hard terminal and swept-SAT checks.
    seed_score=sequence_cost(actions,w)["score"]
    acceptance_limit=(seed_score if acceptance_score_limit is None else
                      float(acceptance_score_limit))
    for alpha in (1.0,.75,.5,.375,.25,.1875,.125,.0625,.03125,.015625,.0078125):
        refined=[Action(a.mode,
                        a.amount+alpha*(b.amount-a.amount),
                        a.control+alpha*(b.control-a.control))
                 for a,b in zip(actions,proposed)]
        poses=replay(start,refined)
        if math.hypot(poses[-1].x-goal.x,poses[-1].y-goal.y)>xy_tol+1e-5:
            continue
        if abs(wrap(poses[-1].th-goal.th))>yaw_tol+1e-5:
            continue
        if grid.trajectory_clear(poses,refined):
            result_score=sequence_cost(refined,w)["score"]
            if result_score>acceptance_limit+1e-8:
                continue
            meta={"corridor":"disc" if covered_discs else "mixed_disc_point",
                  "line_search_alpha":alpha,
                  "seed_cost":seed_score,
                  "acceptance_score_limit":acceptance_limit,
                  "result_cost":result_score,
                  "nlp_objective":float(sol.value(objective))}
            profile["line_search_sweep_s"]=time.perf_counter()-check_begin
            profile["total_s"]=time.perf_counter()-total_begin
            meta["profile"]=profile
            return poses,refined,boxes,meta
    raise ValueError("optimized_sat_collision_or_goal_violation")


def time_sample(start, actions, v_demo=.5, omega_demo=.5, dt=.1):
    """Smoothstep per contiguous mode+gear group; center speed is zero in Spin."""
    groups=[]
    for a in actions:
        sign=1 if a.amount>0 else -1
        if not groups or groups[-1][0].mode!=a.mode or (groups[-1][0].amount>0)!=(sign>0):
            groups.append([])
        groups[-1].append(a)
    first_mode=groups[0][0].mode if groups else 0
    result=[(0.0,start,0.0,first_mode)]
    elapsed=0.0;p=start
    for group in groups:
        total=sum(abs(a.amount) for a in group)
        rate=omega_demo if group[0].mode==2 else v_demo
        duration=max(dt,1.5*total/rate)
        count=max(2,math.ceil(duration/dt))
        taus=[j/count for j in range(1,count+1)]
        # Preserve every primitive boundary in the timed path. Otherwise two
        # neighboring samples can straddle an instantaneous steering/oblique
        # control change; linear interpolation then cuts across the corner,
        # creating a collision that is not on either checked primitive.
        cumulative=0.0
        for action in group[:-1]:
            cumulative+=abs(action.amount)
            target=cumulative/total
            lo,hi=0.0,1.0
            for _ in range(48):
                mid=(lo+hi)/2
                if 3*mid*mid-2*mid*mid*mid<target:lo=mid
                else:hi=mid
            taus.append((lo+hi)/2)
        taus.sort()
        unique_taus=[]
        for tau in taus:
            if not unique_taus or tau-unique_taus[-1]>1e-10:
                unique_taus.append(tau)
        for tau in unique_taus:
            tau=min(1.0,max(0.0,tau))
            length=total*(3*tau*tau-2*tau*tau*tau)
            remaining=length;q=p
            for a in group:
                part=min(remaining,abs(a.amount))
                q=advance(q,Action(a.mode,math.copysign(part,a.amount),a.control))
                remaining-=part
                if remaining<=1e-10:break
            # Numeric theta is deliberately unwrapped for downstream control.
            # ROS quaternions remain periodic when rendered by pose_stamped().
            q=Pose(q.x,q.y,result[-1][1].th+wrap(q.th-result[-1][1].th))
            gear_sign=1.0 if group[0].amount>0 else -1.0
            v=(0.0 if group[0].mode==2 else
               gear_sign*total*(6*tau-6*tau*tau)/duration)
            result.append((elapsed+duration*tau,q,v,group[0].mode))
        elapsed+=duration
        p=result[-1][1]
    return result
