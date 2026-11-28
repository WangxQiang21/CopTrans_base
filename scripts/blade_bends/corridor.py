"""Connect a geometric corridor with exact rigid motions and optional chassis realignments."""
import math
from time import perf_counter

import numpy as np

from ranger_mini_gz.blade_bend_search import difference, primitive
from ranger_mini_gz.blade_planner import Limits
from scenes import clearance_seed


def corridor_rollout(scene, reference, checker, smoothing, guide_map):
    begin = perf_counter()
    guide = clearance_seed(scene, reference, smoothing=smoothing, dense=True,
                           grid_region=checker._spaces[1].polygon if guide_map == "grid" else None)
    margin = scene.blade_site.config["planning_clearance_m"]
    limits = Limits(clearance=margin)
    n = len(guide)
    # A long certified shortcut replaces all intervening guide poses. Backtrack
    # when one shortcut leaves the remaining corridor disconnected.
    blocked = set()
    checked = 0

    def search(i):
        nonlocal checked
        if i == n - 1:
            return []
        if i in blocked:
            return None
        for j in range(n - 1, i, -1):
            a, b = guide[i, :3], guide[j, :3]
            for edge in primitive(a, b, checker.offsets, limits, False):
                checked += 1
                rough, yaws = edge.samples(np.linspace(0., 1., 9))
                if not checker.check_poses(rough, yaws, margin_m=margin+.0002).all():
                    continue
                travel = np.linalg.norm(edge.tangent) + checker.cargo_radius * abs(edge.delta)
                cargo, yaws = edge.samples(np.linspace(0., 1., max(3, math.ceil(travel/.042)) + 1))
                if not checker.check_poses(cargo, yaws, margin_m=margin+.0002).all():
                    continue
                following = search(j)
                if following is not None:
                    return [edge, *following]
        blocked.add(i)
        return None

    chain = search(0)
    if chain is None:
        return None, dict(certified=False, reason="corridor_graph_exhausted", edges_checked=checked,
                          solve_wall_s=perf_counter()-begin)
    # Between arcs both supports stay put. Choose forward or reverse for each
    # chassis by its smallest alignment angle. Reverse is available on every edge.
    local = [reference[0].copy()]
    errors, modes, gears, events = [], [], [], []

    def align(target):
        start = local[-1]
        delta = difference(start[3:], target)
        largest = np.max(abs(delta))
        if largest <= 1e-8:
            return
        count = max(2, math.ceil(largest/.012))
        samples = np.tile(start, (count, 1))
        samples[:, 3:] += np.linspace(0., 1., count+1)[1:, None]*delta
        local.extend(samples)
        modes.extend([2]*count)
        gears.extend([[0, 0]]*count)
        errors.extend([[0.]*4]*count)
        events.append(dict(mode="Spin", xy=start[:2].tolist(), end_xy=start[:2].tolist(),
                           chassis_yaw_change_deg=np.rad2deg(delta).tolist(), cargo_yaw_change_rad=0.,
                           body_frame_displacement_m=[[0.,0.],[0.,0.]]))

    for edge in chain:
        current = local[-1][3:]
        reverse = abs(difference(current, edge.heading+math.pi)) < abs(difference(current, edge.heading))
        align(edge.heading+math.pi*reverse)
        count = max(2, math.ceil((np.linalg.norm(edge.tangent)+checker.cargo_radius*abs(edge.delta))/.025))
        u = np.linspace(0., 1., count+1)[1:]
        cargo, headings = edge.samples(u)
        local.extend(np.column_stack((cargo, headings + local[-1][3:]-edge.heading)))
        modes.extend([0]*count)
        gears.extend([np.where(reverse, -1, 1)]*count)
        errors.extend([[abs(edge.delta)*np.linalg.norm(edge.tangent)/(8*count**2),0.,0.,0.]]*count)
    align(reference[-1, 3:])
    local = np.asarray(local)
    # Recover the continuous angle branch of the original transport trajectory.
    report = dict(edges_checked=checked, accepted_arcs=len(chain), solve_wall_s=perf_counter()-begin,
                  max_arc_curvature_per_m=max(float(max(abs(e.delta)/e.chassis_speed)) for e in chain))
    return (local, np.asarray(errors), np.array(modes), np.asarray(gears), events), report
