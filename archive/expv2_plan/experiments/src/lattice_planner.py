"""Small multi-mode lattice A* for reproducible spatial-planning experiments."""
from __future__ import annotations

import heapq
import itertools
import math
import time

from spatial_model import Pose, RasterWorld, Vehicle, propagate, wrap


DS = 0.25
YAW_BIN = math.radians(15)
XY_BIN = 0.20


def actions(vehicle: Vehicle, enabled_modes: set[str]):
    out = []
    if "A" in enabled_modes:
        kmax = 1.0 / vehicle.min_turn_radius
        for ds in (DS, -DS):
            for k in (-kmax, 0.0, kmax):
                out.append(("A", ds, k))
    if "P" in enabled_modes:
        for ds in (DS, -DS):
            for phi in (-math.pi / 4, 0.0, math.pi / 4):
                out.append(("P", ds, phi))
    if "L" in enabled_modes:
        for ds in (DS, -DS):
            for phi in (-math.pi / 2, math.pi / 2):
                out.append(("L", ds, phi))
    if "S" in enabled_modes:
        for dyaw in (-YAW_BIN, YAW_BIN):
            out.append(("S", dyaw, 0.0))
    return out


def state_key(p: Pose, last_mode: str):
    ix = round(p.x / XY_BIN)
    iy = round(p.y / XY_BIN)
    it = round(wrap(p.yaw) / YAW_BIN)
    return ix, iy, it, last_mode


def action_swept_clear(world: RasterWorld, start: Pose, mode: str,
                       amount: float, control: float, vehicle: Vehicle) -> bool:
    if mode == "S":
        n = max(1, math.ceil(abs(amount) / math.radians(1)))
    else:
        n = max(1, math.ceil(abs(amount) / 0.01))
    for i in range(1, n + 1):
        nxt = propagate(start, mode, amount * i / n, control, vehicle)
        if not world.pose_clear(nxt):
            return False
    return True


def direct_action(mode: str, start: Pose, end: Pose, vehicle: Vehicle):
    """Find one exact same-mode primitive connecting two poses, if one exists."""
    dx, dy = end.x-start.x, end.y-start.y
    dtheta = wrap(end.yaw-start.yaw)
    tol = 0.012
    if mode == "S":
        if math.hypot(dx,dy) <= tol and abs(dtheta) > 1e-5:
            return ("S",dtheta,0.0)
        return None
    if mode in ("P","L"):
        if abs(dtheta) > 1e-4:
            return None
        distance=math.hypot(dx,dy)
        if distance < 1e-5:
            return None
        direction=math.atan2(dy,dx)
        candidates=[]
        signs=(1.0,-1.0)
        for sign in signs:
            phi=wrap(direction-(0.0 if sign>0 else math.pi)-start.yaw)
            if mode=="P" and abs(phi)<=math.pi/2+1e-6:
                candidates.append(("P",sign*distance,max(-math.pi/2,min(math.pi/2,phi))))
            if mode=="L":
                for lat in (-math.pi/2,math.pi/2):
                    ds=(dx*math.cos(start.yaw+lat)+dy*math.sin(start.yaw+lat))
                    if abs(wrap(phi-lat))<1e-5:
                        candidates.append(("L",ds,lat))
        for candidate in candidates:
            q=propagate(start,*candidate,vehicle)
            if math.hypot(q.x-end.x,q.y-end.y)<=tol and abs(wrap(q.yaw-end.yaw))<=1e-4:
                return candidate
        return None
    if mode == "A":
        if abs(dtheta)<1e-5:
            ds=dx*math.cos(start.yaw)+dy*math.sin(start.yaw)
            q=propagate(start,"A",ds,0.0,vehicle)
            if math.hypot(q.x-end.x,q.y-end.y)<=tol:
                return ("A",ds,0.0)
            return None
        fx=math.sin(end.yaw)-math.sin(start.yaw)
        fy=math.cos(start.yaw)-math.cos(end.yaw)
        denom=fx*fx+fy*fy
        if denom<1e-10:
            return None
        curvature=(fx*dx+fy*dy)/denom
        if abs(curvature)<1e-8 or abs(curvature)>1.0/vehicle.min_turn_radius+1e-6:
            return None
        ds=dtheta/curvature
        q=propagate(start,"A",ds,curvature,vehicle)
        if math.hypot(q.x-end.x,q.y-end.y)<=tol and abs(wrap(q.yaw-end.yaw))<=1e-4:
            return ("A",ds,curvature)
    return None


def smooth_same_mode_runs(world: RasterWorld, vehicle: Vehicle, start: Pose,
                          raw_actions: list):
    """Shortcut within each mode run while preserving exact segment endpoints."""
    if not raw_actions:
        return []
    raw_poses=[start]
    for mode,amount,control in raw_actions:
        raw_poses.append(propagate(raw_poses[-1],mode,amount,control,vehicle))
    output=[]
    i=0
    while i<len(raw_actions):
        mode=raw_actions[i][0]
        run_end=i
        while run_end+1<len(raw_actions) and raw_actions[run_end+1][0]==mode:
            run_end+=1
        cursor=i
        while cursor<=run_end:
            selected=None
            selected_end=cursor
            for j in range(run_end,cursor,-1):
                candidate=direct_action(mode,raw_poses[cursor],raw_poses[j+1],vehicle)
                if candidate is not None and action_swept_clear(world,raw_poses[cursor],*candidate,vehicle):
                    selected=candidate; selected_end=j; break
            if selected is None:
                selected=raw_actions[cursor]
            output.append(selected)
            cursor=selected_end+1
        i=run_end+1
    # Merge adjacent equal controls (exactly the same continuous primitive).
    merged=[]
    for action in output:
        if merged and action[0]==merged[-1][0] and abs(action[2]-merged[-1][2])<1e-9:
            prev=merged[-1]
            merged[-1]=(prev[0],prev[1]+action[1],prev[2])
        else:
            merged.append(action)
    return merged


def plan(world: RasterWorld, vehicle: Vehicle, start: Pose, goal: Pose,
         enabled_modes: set[str], xy_tol=0.23, yaw_tol=math.radians(12),
         switch_cost=0.18, max_expansions=300_000, max_seconds=60.0,
         heuristic_weight=1.0):
    if not world.pose_clear(start):
        return {"status": "invalid_start", "expanded": 0}
    if not world.pose_clear(goal):
        return {"status": "invalid_goal", "expanded": 0,
                "runtime_s": 0.0, "goal_pose": goal}
    root_key = state_key(start, "-")
    # Immutable labels prevent parent-chain corruption when a quantized state
    # is reached again with a lower cost after descendants were expanded.
    nodes = [(root_key, start, 0.0, None, None)]
    best_node = {root_key: 0}
    g_cost = {root_key: 0.0}
    serial = itertools.count()
    start_pos_lb = max(0.0, math.hypot(start.x-goal.x, start.y-goal.y)-xy_tol)
    start_yaw_lb = max(0.0, abs(wrap(start.yaw-goal.yaw))-yaw_tol)
    start_h = max(start_pos_lb, 0.45 * start_yaw_lb)
    queue = [(heuristic_weight * start_h,
              abs(wrap(start.yaw - goal.yaw)), 0.0, next(serial), root_key, 0)]
    primitives = actions(vehicle, enabled_modes)
    expanded = 0
    t0 = time.perf_counter()
    best_node_id = 0
    best_h = math.inf

    while queue and expanded < max_expansions:
        elapsed = time.perf_counter() - t0
        if elapsed >= max_seconds:
            return {"status": "timeout", "expanded": expanded,
                    "runtime_s": elapsed, "best_pose": nodes[best_node_id][1],
                    "best_heuristic": best_h}
        _, _, cost, _, key, node_id = heapq.heappop(queue)
        if node_id != best_node.get(key) or cost > g_cost.get(key, math.inf) + 1e-10:
            continue
        pose = nodes[node_id][1]
        h = math.hypot(pose.x - goal.x, pose.y - goal.y)
        if h + 0.15 * abs(wrap(pose.yaw - goal.yaw)) < best_h:
            best_h, best_node_id = h + 0.15 * abs(wrap(pose.yaw - goal.yaw)), node_id
        if h <= xy_tol and abs(wrap(pose.yaw - goal.yaw)) <= yaw_tol:
            end_key = key
            break
        expanded += 1

        for mode, amount, control in primitives:
            nxt = propagate(pose, mode, amount, control, vehicle)
            if not action_swept_clear(world, pose, mode, amount, control, vehicle):
                continue
            new_key = state_key(nxt, mode)
            # Translation length plus a radius-scaled rotation length.
            step_cost = abs(amount) if mode != "S" else 0.45 * abs(amount)
            if key[3] not in ("-", mode):
                step_cost += switch_cost
            new_cost = cost + step_cost
            if new_cost + 1e-9 >= g_cost.get(new_key, math.inf):
                continue
            g_cost[new_key] = new_cost
            child_id = len(nodes)
            nodes.append((new_key, nxt, new_cost, node_id, (mode, amount, control)))
            best_node[new_key] = child_id
            distance_lb = max(0.0, math.hypot(nxt.x-goal.x, nxt.y-goal.y)-xy_tol)
            yaw_lb = 0.45 * max(0.0, abs(wrap(nxt.yaw-goal.yaw))-yaw_tol)
            heuristic = max(distance_lb, yaw_lb)
            heading_tie_break = abs(wrap(nxt.yaw - goal.yaw))
            heapq.heappush(queue, (new_cost + heuristic_weight * heuristic, heading_tie_break,
                                   new_cost, next(serial), new_key, child_id))
    else:
        return {"status": "search_exhausted", "expanded": expanded,
                "runtime_s": time.perf_counter() - t0,
                "best_pose": nodes[best_node_id][1], "best_heuristic": best_h}

    node_chain = []
    node_id = best_node[end_key]
    while node_id is not None:
        node_chain.append(node_id)
        node_id = nodes[node_id][3]
    node_chain.reverse()
    raw_actions = [nodes[i][4] for i in node_chain[1:]]
    raw_action_count = len(raw_actions)
    chosen_actions = smooth_same_mode_runs(world, vehicle, start, raw_actions)
    path = [start]
    for mode, amount, control in chosen_actions:
        path.append(propagate(path[-1], mode, amount, control, vehicle))
    modes = [a[0] for a in chosen_actions]
    # Independent dense replay of the selected primitives. The planner's
    # expansion check is <=5 cm/3 degrees; final acceptance rechecks at
    # <=1 cm/1 degree using the exact mode propagator.
    replay_pose = start
    validator_samples = 1
    validator_ok = world.pose_clear(start)
    for mode, amount, control in chosen_actions:
        n = max(1, math.ceil(abs(amount) / 0.01),
                math.ceil(abs(amount) / math.radians(1)) if mode == "S" else 1)
        for _ in range(n):
            replay_pose = propagate(replay_pose, mode, amount/n, control, vehicle)
            validator_samples += 1
            if not world.pose_clear(replay_pose):
                validator_ok = False
                break
        if not validator_ok:
            break
    if not validator_ok:
        return {"status": "validator_reject", "expanded": expanded,
                "runtime_s": time.perf_counter()-t0,
                "validator_samples": validator_samples}
    final_xy_error = math.hypot(path[-1].x-goal.x,path[-1].y-goal.y)
    final_yaw_error = abs(wrap(path[-1].yaw-goal.yaw))
    if final_xy_error > xy_tol + 1e-8 or final_yaw_error > yaw_tol + 1e-8:
        return {"status":"goal_tolerance_reject", "expanded":expanded,
                "runtime_s":time.perf_counter()-t0, "validator_samples":validator_samples,
                "replayed_goal_error_xy":final_xy_error,"replayed_goal_error_yaw":final_yaw_error}
    result = {
        "status": "success", "path": path, "modes": modes,
        "actions": chosen_actions, "validator_status": "pass",
        "validator_samples": validator_samples,
        "expanded": expanded, "runtime_s": time.perf_counter() - t0,
        "cost": sum(abs(a[1]) if a[0] != "S" else 0.45*abs(a[1]) for a in chosen_actions)
                + switch_cost * sum(a[0] != b[0] for a,b in zip(chosen_actions,chosen_actions[1:])),
        "raw_action_count": raw_action_count, "action_count": len(chosen_actions),
        "action_reduction_pct": 100.0*(raw_action_count-len(chosen_actions))/max(1,raw_action_count),
        "path_length_m": sum(abs(a[1]) for a in chosen_actions if a[0] != "S"),
        "goal_error_xy": final_xy_error,
        "goal_error_yaw": final_yaw_error,
        "mode_switches": sum(a != b for a, b in zip(modes, modes[1:])),
    }
    return result
