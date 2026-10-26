"""Exact primitive-mask reference implementation, with shared gear rules."""
import time
import math
import numpy as np

from .kernel_adapter import Action, advance, action_shunting_violation


def action_clear_vectorized(grid, pose, action, max_corner_step=.005):
    """The reference's same sample locations and OBB/cell SAT, in batches.

    This changes neither the physical raster nor the sweep spacing. Occupied
    cells are gathered once for the union of the sampled footprints. Near
    floating-point contact, use the scalar kernel to resolve exact parity.
    """
    if not grid.pose_clear(advance(pose, action)):
        return False
    rotation = abs(action.amount) if action.mode == 2 else (
        abs(action.amount*action.control) if action.mode == 0 else 0.)
    translation = 0. if action.mode == 2 else abs(action.amount)
    count = max(1, math.ceil((translation+grid.corner*rotation)/max_corner_step))
    amounts = action.amount*np.arange(1, count+1, dtype=float)/count
    if action.mode == 0 and abs(action.control) >= 1e-10:
        heading = pose.th+amounts*action.control
        x = pose.x+(np.sin(heading)-math.sin(pose.th))/action.control
        y = pose.y+(math.cos(pose.th)-np.cos(heading))/action.control
    elif action.mode == 2:
        heading = pose.th+amounts
        x, y = np.full(count, pose.x), np.full(count, pose.y)
    else:
        heading = np.full(count, pose.th)
        direction = pose.th+(action.control if action.mode == 1 else 0.)
        x, y = pose.x+amounts*math.cos(direction), pose.y+amounts*math.sin(direction)
    c, s = np.cos(heading), np.sin(heading)
    rx = grid.half_l*np.abs(c)+grid.half_w*np.abs(s)
    ry = grid.half_l*np.abs(s)+grid.half_w*np.abs(c)
    bounds = np.stack([x-rx-grid.ox, y-ry-grid.oy,
                       grid.ox+grid.width*grid.res-x-rx,
                       grid.oy+grid.height*grid.res-y-ry])
    ambiguous_bounds = np.flatnonzero(np.min(np.abs(bounds), axis=0) < 1e-10)
    if np.any(bounds < -1e-10):
        return False
    for index in ambiguous_bounds:
        if not grid.pose_clear(advance(pose, Action(action.mode, float(amounts[index]), action.control))):
            return False
    # A circumscribed body disk fits whenever the conservative center
    # clearance exceeds its radius. Only uncertain samples need SAT.
    grid.center_clearance_lower_bound(pose.x, pose.y)
    col = np.clip(np.floor((x-grid.ox)/grid.res).astype(int), 0, grid.width-1)
    row = np.clip(np.floor((y-grid.oy)/grid.res).astype(int), 0, grid.height-1)
    uncertain = grid._clearance_lower_bound[row, col] <= grid.corner
    if not np.any(uncertain):
        return True
    original_indices = np.flatnonzero(uncertain)
    x, y, c, s, rx, ry = [array[uncertain] for array in (x, y, c, s, rx, ry)]
    count = len(x)
    x0 = max(0, math.floor((float(np.min(x-rx))-grid.ox)/grid.res)-1)
    x1 = min(grid.width-1, math.floor((float(np.max(x+rx))-grid.ox)/grid.res)+1)
    y0 = max(0, math.floor((float(np.min(y-ry))-grid.oy)/grid.res)-1)
    y1 = min(grid.height-1, math.floor((float(np.max(y+ry))-grid.oy)/grid.res)+1)
    iy, ix = np.nonzero(grid.occ[y0:y1+1, x0:x1+1])
    if len(ix) == 0:
        return True
    cx = grid.ox+(ix+x0+.5)*grid.res
    cy = grid.oy+(iy+y0+.5)*grid.res
    # Bound transient memory on fine/dense maps.
    chunk = max(1, 131072//len(cx))
    half = grid.res/2
    for begin in range(0, count, chunk):
        end = min(count, begin+chunk)
        cc, ss = c[begin:end, None], s[begin:end, None]
        dx, dy = cx[None, :]-x[begin:end, None], cy[None, :]-y[begin:end, None]
        padding = half*(np.abs(cc)+np.abs(ss))
        separations = [np.abs(dx*cc+dy*ss)-grid.half_l-padding,
                       np.abs(-dx*ss+dy*cc)-grid.half_w-padding,
                       np.abs(dx)-half-rx[begin:end, None],
                       np.abs(dy)-half-ry[begin:end, None]]
        separation = np.maximum.reduce(separations)
        if np.any(separation < -1e-10):
            return False
        ambiguous = np.flatnonzero(np.any(np.abs(separation) <= 1e-10, axis=1))
        for index in original_indices[ambiguous+begin]:
            if not grid.pose_clear(advance(pose, Action(action.mode, float(amounts[index]), action.control))):
                return False
    return True


def valid_actions(problem, pose, prefix, library, max_corner_step=.005, reference=False):
    if not 0 < max_corner_step <= .005:
        raise ValueError("mask_sweep_must_not_exceed_5mm")
    begun = time.perf_counter()
    mask = np.zeros(len(library), dtype=bool)
    counts = {"direction": 0, "history": 0, "collision": 0}
    if not problem.grid.pose_clear(pose):
        return mask, dict(counts, invalid_current_pose=True,
                          wall_s=time.perf_counter()-begun)
    for i, a in enumerate(library.actions):
        if a.mode != 2 and a.amount < 0 and not problem.allow_reverse:
            counts["direction"] += 1
            continue
        if (not problem.allow_shunting and
                action_shunting_violation((*prefix, a)) is not None):
            counts["history"] += 1
            continue
        clear = (problem.grid.action_clear(pose, a, max_corner_step) if reference else
                 action_clear_vectorized(problem.grid, pose, a, max_corner_step))
        if not clear:
            counts["collision"] += 1
            continue
        mask[i] = True
    return mask, dict(counts, wall_s=time.perf_counter()-begun)
