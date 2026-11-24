"""Mode/path search with whole-route NLP feedback and independent ranking.

The C++ seed is one incumbent. Structural substitutions are searched without
an old sequence_cost acceptance gate. NLP states are free over the map rather
than fixed to seed anchors; nonlinear distance-field constraints are proxies
and every result is independently retimed and swept-checked.
"""
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from itertools import islice
import math
import time

import numpy as np
from scipy.ndimage import distance_transform_edt

from .core import (Action, Pose, _symbolic_sinc, advance, collapse_exact_runs,
                   anchor_mode_sequences, expand, features, fit_action_to_pose, has_reverse_motion,
                   mode_schedule_neighbor_candidates, replay, wrap)
from .quality import (TrajectoryEvaluator, FootprintDistance,
                      reverse_preference_actions, action_shunting_violation)
from .retiming import retime, PrimitiveTimingCache
from .continuous_geometry import (fit_continuous_endpoint, fit_continuous_suffix,
                                   fit_mode_boundaries, plan_from_actions,
                                   smooth_control_candidate)
from .continuous_optimizer import optimize_continuous_geometry
from .legacy_geometry_provider import LegacyGeometryCandidateProvider
from .continuous_retiming import retime_continuous
from .geometry_incumbent import GeometryIncumbentStore


class ModeMasterFailure(ValueError):
    """Search failure with the complete diagnostic report attached."""
    def __init__(self, report):
        self.report = report
        super().__init__("mode_master_no_feasible_candidate")


def mode_anchor_budget_trigger(config, actions):
    """Explain why a route may need the bounded anchor-search reserve.

    A coarse planner seed can have few mode changes while still containing
    shunting that the restricted-profile repair expands into a high-switch
    feasible incumbent. Reserve the same bounded time for that repair path.
    """
    switches = sum(a.mode != b.mode for a, b in zip(actions, actions[1:]))
    if not config.continuous_motion_v2:
        return None
    if switches >= config.mode_anchor_min_switches:
        return "high_switch_seed"
    if (not config.allow_shunting and
            action_shunting_violation(actions) is not None):
        return "shunting_seed_repair"
    return None


def mode_anchor_switch_count(config, actions):
    """Return the switch count when an incumbent is eligible for anchoring."""
    switches = sum(a.mode != b.mode for a, b in zip(actions, actions[1:]))
    return switches if switches >= config.mode_anchor_min_switches else None


def forward_connections(p, q, radius=.810):
    """Forward-only CSC Dubins candidates; exact endpoints are rechecked.

    All four CSC families are offered, including longer forward connections
    that a shortest Reeds-Shepp connector with reverse motion would hide.
    CCC families and global optimality are not claimed by this connector.
    """
    distance = math.hypot(q.x-p.x, q.y-p.y)/radius
    bearing = math.atan2(q.y-p.y, q.x-p.x)
    a, b = (p.th-bearing) % (2*math.pi), (q.th-bearing) % (2*math.pi)
    sa, sb, ca, cb, cab = math.sin(a), math.sin(b), math.cos(a), math.cos(b), math.cos(a-b)
    mod = lambda value: value % (2*math.pi)
    options = []
    square = 2+distance**2-2*cab+2*distance*(sa-sb)
    if square >= -1e-9:
        angle = math.atan2(cb-ca, distance+sa-sb)
        options.append(("LSL", (mod(angle-a), math.sqrt(max(0, square)), mod(b-angle))))
    square = 2+distance**2-2*cab+2*distance*(sb-sa)
    if square >= -1e-9:
        angle = math.atan2(ca-cb, distance-sa+sb)
        options.append(("RSR", (mod(a-angle), math.sqrt(max(0, square)), mod(angle-b))))
    square = -2+distance**2+2*cab+2*distance*(sa+sb)
    if square >= -1e-9:
        straight = math.sqrt(max(0, square))
        angle = math.atan2(-ca-cb, distance+sa+sb)-math.atan2(-2, straight)
        options.append(("LSR", (mod(angle-a), straight, mod(angle-b))))
    square = -2+distance**2+2*cab-2*distance*(sa+sb)
    if square >= -1e-9:
        straight = math.sqrt(max(0, square))
        angle = math.atan2(ca+cb, distance-sa-sb)-math.atan2(2, straight)
        options.append(("RSL", (mod(a-angle), straight, mod(b-angle))))
    result = []
    for word, amounts in options:
        actions = [Action(0, amount*radius, (1 if symbol == "L" else -1 if symbol == "R" else 0)/radius)
                   for symbol, amount in zip(word, amounts) if amount > 1e-9]
        end = replay(p, actions)[-1]
        if math.hypot(end.x-q.x, end.y-q.y) < 1e-7 and abs(wrap(end.th-q.th)) < 1e-7:
            result.append(actions)
    return sorted(result, key=lambda actions: sum(a.amount for a in actions))


def forward_corner_connections(p, q, radius=.810):
    """Exact straight/arc/straight alternatives for an ordinary corridor bend.

    CSC alone can cut diagonally across a wall when both boundary poses are
    on the incoming/outgoing straights. Solve the two tangent lengths for an
    SCS fillet instead. These connections carry no collision certificate.
    """
    turn = wrap(q.th-p.th)
    if abs(turn) < .02 or abs(turn) >= math.pi-.02:
        return []
    c0, s0, c1, s1 = math.cos(p.th), math.sin(p.th), math.cos(q.th), math.sin(q.th)
    determinant = c0*s1-s0*c1
    if abs(determinant) < 1e-8:
        return []
    result = []
    for factor in (1., 1.25, 1.5):
        r = radius*factor
        arc = Action(0, abs(turn)*r, math.copysign(1./r, turn))
        arc_end = advance(p, arc)
        dx, dy = q.x-arc_end.x, q.y-arc_end.y
        before = (dx*s1-dy*c1)/determinant
        after = (c0*dy-s0*dx)/determinant
        if min(before, after) < -1e-9:
            continue
        curve = ([Action(0, before, 0.)] if before > 1e-9 else [])+[arc]
        if after > 1e-9:
            curve.append(Action(0, after, 0.))
        end = replay(p, curve)[-1]
        if math.hypot(end.x-q.x, end.y-q.y) < 1e-7 and abs(wrap(end.th-q.th)) < 1e-7:
            result.append(curve)
    return result


def reverse_connections(p, q, radius=.810):
    """Reverse-gear CSC connectors between exact body poses.

    A reverse Ackermann arc is a forward Dubins arc after rotating both body
    headings by pi. Mapping the virtual controls back also flips curvature;
    every result is replayed in the original signed kinematics before return.
    This is a candidate generator, not a Reeds-Shepp completeness claim.
    """
    virtual_start = Pose(p.x, p.y, wrap(p.th + math.pi))
    virtual_goal = Pose(q.x, q.y, wrap(q.th + math.pi))
    result = []
    for curve in forward_connections(virtual_start, virtual_goal, radius):
        actions = [Action(0, -action.amount, -action.control)
                   for action in curve]
        end = replay(p, actions)[-1]
        if (math.hypot(end.x-q.x, end.y-q.y) < 1e-7 and
                abs(wrap(end.th-q.th)) < 1e-7):
            result.append(actions)
    return sorted(result, key=lambda actions: sum(abs(a.amount) for a in actions))


def late_reverse_proposals(start, actions, radius=.810):
    """Follow a reverse seed's geometry forward, then reorient at later anchors.

    A pi body-heading shift and a sign flip of A amount/curvature preserve the
    world path and symmetric footprint. Start/anchor Spins and alternative
    forward connectors must still pass the independent swept checker.
    """
    actions = collapse_exact_runs(actions)
    first = next((i for i, a in enumerate(actions) if a.mode != 2), None)
    if first is None or actions[first].amount >= 0:
        return
    poses = replay(start, actions)
    leading = sum(a.amount for a in actions[:first])
    begin_turn = wrap(leading+math.pi)
    for anchor in range(len(actions)-1, first+1, -1):
        prefix = actions[first:anchor]
        if any(a.mode != 2 and a.amount > 0 for a in prefix):
            continue
        transformed = ([Action(2, begin_turn, 0.)] if abs(begin_turn)>1e-9 else [])
        transformed += [Action(a.mode, -a.amount if a.mode != 2 else a.amount,
                                -a.control if a.mode == 0 else a.control) for a in prefix]
        target = Pose(poses[anchor].x, poses[anchor].y, wrap(poses[anchor].th+math.pi))
        banks = [transformed]+forward_connections(start, target, radius)[:2]
        for index, forward in enumerate(banks):
            end = replay(start, forward)[-1]
            turn = wrap(poses[anchor].th-end.th)
            candidate = collapse_exact_runs(forward+[Action(2, turn, 0.)]+actions[anchor:])
            yield candidate, {"operation": "late_reverse_reorientation",
                              "anchor_action_index": anchor,
                              "forward_prefix_connector": index,
                              "anchor_pose": [target.x, target.y, target.th]}


def structural_proposals(start, actions, max_window=8, max_length_ratio=1.8, radius=.810):
    """Replace, merge or split control/mode intervals, including unsafe starts.

    Exact shortcuts and forward A alternatives compete with prototypes whose
    boundary yaw is initially inconsistent. The latter receive a whole-route
    NLP opportunity, rather than being rejected by a fixed seed anchor yaw.
    """
    actions = collapse_exact_runs(actions)
    poses = replay(start, actions)
    seen = set()
    # Always offer a bounded route-level family. Local replacement windows
    # alone can miss a stable O/A connector when the seed has more controls
    # than max_window, even though the whole endpoint pair has a simple fit.
    route_start, route_goal = poses[0], poses[-1]
    route_length = sum(abs(a.amount) for a in actions if a.mode != 2)
    route_alternatives = []
    for mode in (0, 1, 2):
        fit = fit_action_to_pose(mode, route_start, route_goal, tolerance=.002)
        if fit is not None:
            route_alternatives.append(([fit], "route_level_mode_fit"))
    if route_length >= .2:
        for curve in forward_connections(route_start, route_goal, radius):
            if sum(a.amount for a in curve) <= max_length_ratio*route_length+.3:
                route_alternatives.append((curve, "route_level_forward_A_connection"))
        for curve in reverse_connections(route_start, route_goal, radius):
            if sum(abs(a.amount) for a in curve) <= max_length_ratio*route_length+.3:
                route_alternatives.append((curve, "route_level_reverse_A_connection"))
    for replacement, operation in route_alternatives:
        if sum(abs(a.amount) for a in replacement if a.mode != 2) > max_length_ratio*route_length+.3:
            continue
        proposed = collapse_exact_runs(replacement)
        key = TrajectoryEvaluator.signature(proposed)
        if key == TrajectoryEvaluator.signature(actions) or key in seen:
            continue
        seen.add(key)
        yield proposed, {"operation": operation, "first": 0,
                         "last": len(actions)}
    for first in range(len(actions)):
        for last in range(first+1, min(len(actions), first+max_window)+1):
            p, q = poses[first], poses[last]
            original = actions[first:last]
            old_distance = sum(abs(a.amount) for a in original if a.mode != 2)
            alternatives = []
            for mode in (0, 1, 2):
                fit = fit_action_to_pose(mode, p, q, tolerance=.002)
                if fit is not None:
                    alternatives.append(([fit], "exact_mode_merge"))
            # Reorientation need not occur at the final user pose. These
            # exact-boundary edits can move it to a swept-safe end of a local
            # window while keeping a constant O angle during translation.
            turn=wrap(q.th-p.th)
            if abs(turn)>.01:
                before=fit_action_to_pose(1,p,Pose(q.x,q.y,p.th),.002)
                if before is not None:
                    alternatives.append(([before,Action(2,turn,0.)],
                                         "stable_O_then_spin"))
                after=fit_action_to_pose(1,Pose(p.x,p.y,q.th),q,.002)
                if after is not None:
                    alternatives.append(([Action(2,turn,0.),after],
                                         "spin_then_stable_O"))
            if old_distance >= .2:
                d = math.hypot(q.x-p.x, q.y-p.y)
                if d > .05 and any(a.mode != 0 for a in original):
                    # Prototype A does not inherit the O/Spin boundary yaw or
                    # center displacement as a fixed connector constraint.
                    # The whole-route NLP supplies its feasible geometry.
                    amount = max(d, abs(wrap(q.th-p.th))*radius)
                    curvature = wrap(q.th-p.th)/amount
                    alternatives.append(([Action(0, amount, curvature)], "free_yaw_A_prototype"))
                for curve in forward_connections(p, q, radius):
                    if sum(a.amount for a in curve) <= max_length_ratio*old_distance+.3:
                        alternatives.append((curve, "forward_A_connection"))
                for curve in reverse_connections(p, q, radius):
                    if sum(abs(a.amount) for a in curve) <= max_length_ratio*old_distance+.3:
                        alternatives.append((curve, "reverse_A_connection"))
                # A pair of opposite arcs can replace constant-yaw O drift
                # with gentler-than-maximum curvature, preserving endpoints.
                d = math.hypot(q.x-p.x, q.y-p.y)
                if d > .05 and abs(wrap(q.th-p.th)) < 1e-4:
                    turn = 2*wrap(math.atan2(q.y-p.y, q.x-p.x)-p.th)
                    curvature = 4*math.sin(turn/2)/d
                    if abs(curvature) > 1e-8 and abs(curvature) <= 1/radius:
                        amount = turn/curvature
                        if amount > 0:
                            pair = [Action(0, amount, curvature), Action(0, amount, -curvature)]
                            end = replay(p, pair)[-1]
                            if math.hypot(end.x-q.x, end.y-q.y) < 1e-6:
                                alternatives.append((pair, "gentle_A_pair"))
                # Free-boundary prototype: prior/following A/Spin geometry may
                # adjust yaw in the NLP. No prescribed A-O or S-O-S recipe.
                phi = wrap(math.atan2(q.y-p.y, q.x-p.x)-p.th)
                if d > .05 and abs(phi) <= math.pi/2 and abs(wrap(q.th-p.th)) > .01:
                    alternatives.append(([Action(1, d, phi)], "free_yaw_O_prototype"))
            for replacement, operation in alternatives:
                # Nearly collinear opposite-arc fits can have tiny curvature
                # and kilometre-scale amounts. All replacement families share
                # the same local length bound before timing or swept checks.
                if (sum(abs(a.amount) for a in replacement if a.mode != 2) >
                        max_length_ratio*old_distance+.3):
                    continue
                proposed = collapse_exact_runs(actions[:first]+replacement+actions[last:])
                key = TrajectoryEvaluator.signature(proposed)
                if key == TrajectoryEvaluator.signature(actions) or key in seen:
                    continue
                seen.add(key)
                yield proposed, {"operation": operation, "first": first, "last": last}


def movable_corner_connections(p, q, anchor, path_clear, radius=.810,
                               deadline=None, max_candidates=2):
    """Move an interior corner pose, preserving only the window endpoints."""
    heading = p.th+.5*wrap(q.th-p.th)
    c0, s0, c1, s1 = math.cos(p.th), math.sin(p.th), math.cos(q.th), math.sin(q.th)
    produced = 0
    for u in (0, 1, -1, 2, -2, 3, -3):
        for v in (0, 1, -1, 2, -2, 3, -3):
            if deadline is not None and time.perf_counter() >= deadline:
                return
            step = radius/16.
            mid = Pose(anchor.x+step*(u*c0+v*c1),
                       anchor.y+step*(u*s0+v*s1), heading)
            if not path_clear([mid], []):
                continue
            def clear_curves(first, last):
                for curve in forward_connections(first, last, radius):
                    if deadline is not None and time.perf_counter() >= deadline:
                        return
                    if sum(a.amount for a in curve) > 1.8*math.hypot(q.x-p.x, q.y-p.y)+.3:
                        continue
                    if path_clear(replay(first, curve), curve):
                        yield curve
            prefix = next(clear_curves(p, mid), None)
            if prefix is None:
                continue
            suffix = next(clear_curves(mid, q), None)
            if suffix is None:
                continue
            yield collapse_exact_runs(prefix+suffix)
            produced += 1
            if produced >= max_candidates:
                return


def repaired_terminal_arc(start, goal, actions, radius=.810):
    """Re-anchor an A/Spin tail to the real goal for structural proposals.

    A certified continuous incumbent can have a repaired suffix while its
    retained primitive schedule still ends at the coarse lattice goal. Fit
    the final arc to goal position and retain a legal terminal Spin for yaw.
    This returns an uncertified proposal; callers still sweep/audit it.
    """
    actions = collapse_exact_runs(actions)
    last = next((i for i in range(len(actions)-1, -1, -1)
                 if actions[i].mode != 2), None)
    if (last is None or last == len(actions)-1 or
            actions[last].mode != 0 or actions[last].amount <= 0):
        return None
    p = replay(start, actions[:last])[-1]
    distance = math.hypot(goal.x-p.x, goal.y-p.y)
    if distance < 1e-8:
        return None
    bearing = wrap(math.atan2(goal.y-p.y, goal.x-p.x)-p.th)
    curvature = 2*math.sin(bearing)/distance
    amount = 2*bearing/curvature if abs(curvature) > 1e-9 else distance
    if amount <= 0 or abs(curvature) > 1./radius+1e-9:
        return None
    arc = Action(0, amount, curvature)
    end = advance(p, arc)
    turn = wrap(goal.th-end.th)
    proposed = actions[:last]+[arc]+([Action(2, turn, 0.)] if abs(turn) > 1e-9 else [])
    end = replay(start, proposed)[-1]
    if math.hypot(end.x-goal.x, end.y-goal.y) > 1e-7 or abs(wrap(end.th-goal.th)) > 1e-7:
        return None
    return proposed


def combined_spin_replacements(start, actions, path_clear, radius=.810,
                               max_window=8, max_paths=4, deadline=None,
                               diversify=False):
    """Compose disjoint, exact A connectors around every Spin in one pass.

    Windows are explicitly centered on Spins, including late route corners.
    A small interval-DAG beam combines swept-screened replacements without
    changing their boundary poses. These are proposals only: continuous
    conversion, endpoint fitting, retiming and strict auditing remain required.
    """
    # Long collapsed straights otherwise supply anchors only metres away
    # from a corner. Add exact interior anchors so the connector can begin
    # turning on the approach and rejoin soon after the bend.
    refined = []
    for action in collapse_exact_runs(actions):
        pieces = max(1, math.ceil(abs(action.amount)/.6)) if action.mode == 0 else 1
        refined.extend(Action(action.mode, action.amount/pieces, action.control)
                       for _ in range(pieces))
    actions = refined
    poses = replay(start, actions)
    count = len(actions)
    edges = [[] for _ in range(count)]
    windows = set()
    for spin, action in enumerate(actions):
        if action.mode != 2 or abs(action.amount) <= 1e-9:
            continue
        for first in range(max(0, spin-max_window+1), spin):
            for last in range(spin+2, min(count, first+max_window)+1):
                windows.add((first, last))
    for first, last in sorted(windows):
        if deadline is not None and time.perf_counter() >= deadline:
            break
        original = actions[first:last]
        # Preserve gear policy; an actual Spin needed for a reversal is not
        # replaced by a forward-only connector.
        if (original[0].mode != 0 or original[-1].mode != 0 or
                any(a.mode != 2 and a.amount < 0 for a in original)):
            continue
        old_length = sum(abs(a.amount) for a in original if a.mode != 2)
        spin_angle = sum(abs(a.amount) for a in original if a.mode == 2)
        connections = forward_connections(poses[first], poses[last], radius)
        connections.extend(forward_corner_connections(poses[first], poses[last], radius))
        for curve in sorted(connections, key=lambda seq: sum(a.amount for a in seq)):
            length = sum(a.amount for a in curve)
            if length > 1.8*old_length+.3:
                continue
            if deadline is not None and time.perf_counter() >= deadline:
                break
            curve_poses = replay(poses[first], curve)
            if not path_clear(curve_poses, curve):
                continue
            # Ranking proxy only. Full-route actual execution time decides
            # candidate admission below; this proxy grants no safety pass.
            gain = old_length-length+radius*spin_angle
            removed = sum(a.mode == 2 for a in original)
            edges[first].append((last, curve, gain, removed))
            break  # shortest clear CSC for this exact boundary pair
    # A seed can hug an inner wall on both approach straights. Fixed CSC/SCS
    # tangents then collide despite a feasible A bend with a small lane shift.
    # For uncovered Spins, search bounded movable interior poses as well.
    for spin, action in enumerate(actions):
        if action.mode != 2:
            continue
        if any(first <= spin < last for first, row in enumerate(edges)
               for last, _, _, _ in row):
            continue
        local = [(first, last) for first, last in windows
                 if first < spin < last-1 and
                 actions[first].mode == actions[last-1].mode == 0 and
                 sum(a.mode == 2 for a in actions[first:last]) == 1 and
                 all(a.mode == 2 or (a.mode == 0 and a.amount > 0)
                     for a in actions[first:last])]
        local.sort(key=lambda span: (
            abs(actions[span[0]].control)+abs(actions[span[1]-1].control),
            -min(spin-span[0], span[1]-spin-1), -(span[1]-span[0]), span))
        successes = 0
        for first, last in local[:4]:
            if deadline is not None and time.perf_counter() >= deadline:
                break
            original = actions[first:last]
            old_length = sum(a.amount for a in original if a.mode != 2)
            for curve in movable_corner_connections(poses[first], poses[last],
                    poses[spin], path_clear, radius, deadline):
                length = sum(a.amount for a in curve)
                if length <= 1.8*old_length+.3:
                    edges[first].append((last, curve,
                        old_length-length+radius*abs(action.amount), 1))
                    successes += 1
            if successes >= 2:
                break
    paths = [[] for _ in range(count+1)]
    paths[0] = [(0, 0., ())]
    for first in range(count):
        paths[first] = sorted(paths[first], key=lambda item: (-item[0], -item[1]))[:max_paths]
        for removed, gain, edits in paths[first]:
            paths[first+1].append((removed, gain, edits))
            for last, curve, improvement, eliminated in edges[first]:
                paths[last].append((removed+eliminated, gain+improvement,
                                    edits+((first, last, curve),)))
    bank = paths[count]
    if diversify:
        bank = list(bank)+[(removed, gain, ((first, last, curve),))
            for first, row in enumerate(edges) for last, curve, gain, removed in row]
        by_coverage = {}
        for removed, gain, edits in bank:
            covered = tuple(i for i, action in enumerate(actions) if action.mode == 2 and
                            any(first <= i < last for first, last, _ in edits))
            if covered and (covered not in by_coverage or gain > by_coverage[covered][1]):
                by_coverage[covered] = (removed, gain, edits)
        bank = list(by_coverage.values())
    seen = set()
    # This family supplies simultaneous Spin elimination, even when a length
    # proxy undervalues saved braking/acceleration. Actual time remains the
    # solver's final objective; this beam ordering is candidate coverage only.
    for removed, gain, edits in sorted(bank, key=lambda item: (-item[0], -item[1])):
        if not edits:
            continue
        proposed, cursor = [], 0
        for first, last, curve in edits:
            proposed.extend(actions[cursor:first])
            proposed.extend(curve)
            cursor = last
        proposed = collapse_exact_runs(proposed+actions[cursor:])
        signature = TrajectoryEvaluator.signature(proposed)
        if signature in seen or action_shunting_violation(proposed) is not None:
            continue
        seen.add(signature)
        yield proposed, {"operation": "combined_spin_A_connections",
                         "windows": [[first, last] for first, last, _ in edits],
                         "remaining_spin_blocks": sum(a.mode == 2 for a in proposed),
                         "composition_proxy_gain": gain}
        if len(seen) >= max_paths:
            break


def forward_detour_proposals(start, goal, radius=.810):
    """Exact CSCSC connectors with a short movable approach to the turn.

    A single CSC fixes its tangent straight from the start pose. A short
    approach arc and straight can move that tangent around an obstacle
    without inserting a stationary Spin. All candidates still need sweep
    and executable-trajectory certification by the caller.
    """
    for curvature in (0., 1./radius, -1./radius, .5/radius, -.5/radius):
        for fraction in ((0.,) if curvature == 0. else (.15, .3, .5)):
            first = ([Action(0, radius*fraction, curvature)] if fraction else [])
            after_arc = replay(start, first)[-1]
            for straight in (.5, 1., 1.5, 1.75, 2.):
                prefix = first+[Action(0, radius*straight, 0.)]
                pose = advance(after_arc, prefix[-1])
                for suffix in forward_connections(pose, goal, radius):
                    yield collapse_exact_runs(prefix+suffix), {
                        "operation": "forward_A_moved_tangent",
                        "approach_arc_m": radius*fraction,
                        "approach_curvature": curvature,
                        "approach_straight_m": radius*straight}


def upstream_heading_proposals(start, goal, actions, allow_shunting=True, grid=None):
    """Alternative stable-control suffixes at existing segment stations.

    Search free-space reorientation upstream of a reversal/curvature-heavy
    suffix. All A/O suffixes and both translation directions compete; this
    is an initializer family, not a prescribed mode sequence or map rule.
    Whole-path refinement may subsequently move every intermediate pose.
    """
    poses = replay(start, actions)
    edge_cache = {}
    def admitted(proposed):
        if not allow_shunting and action_shunting_violation(proposed) is not None:
            return False
        if grid is not None:
            for pose,action in zip(replay(start,proposed),proposed):
                key=(pose,action)
                if key not in edge_cache:
                    edge_cache[key]=grid.action_clear(pose,action,.04)
                if not edge_cache[key]:
                    return False
        return True
    for index in reversed(range(len(actions))):
        action = actions[index]
        if action.mode == 2:
            continue
        for fraction in (1., .5, 0.):
            prefix = list(actions[:index])
            if fraction > 0:
                prefix.append(Action(action.mode, fraction*action.amount, action.control))
            p = advance(poses[index], Action(action.mode, fraction*action.amount, action.control))
            distance = math.hypot(goal.x-p.x, goal.y-p.y)
            if distance < .05:
                continue
            bearing = math.atan2(goal.y-p.y, goal.x-p.x)
            for heading in (goal.th, p.th, bearing, wrap(bearing+math.pi)):
                aligned = Pose(p.x, p.y, heading)
                align_action = Action(2, wrap(heading-p.th), 0)
                alignment = ([align_action] if abs(align_action.amount)>1e-10
                             else [])
                if grid is not None and alignment and not grid.action_clear(p,align_action,.01):
                    continue
                for mode in (0, 1):
                    fit = fit_action_to_pose(mode, aligned, Pose(goal.x, goal.y, heading), .002)
                    if fit is None:
                        continue
                    suffix = [Action(2, wrap(heading-p.th), 0), fit,
                              Action(2, wrap(goal.th-heading), 0)]
                    proposed = collapse_exact_runs(prefix+[a for a in suffix if abs(a.amount)>1e-10])
                    if not admitted(proposed):
                        continue
                    yield proposed, {"operation": "upstream_heading_replan",
                                     "segment": index, "fraction": fraction,
                                     "heading": heading, "suffix_mode": mode}
                # Replan the suffix from this upstream station with complete
                # exact-endpoint A connections in both gears. A single fitted
                # primitive below is retained as a cheap alternate; the CSC
                # family can move the turn away from a constrained terminal.
                for direction, connectors in (
                        ("forward", forward_connections(aligned, goal)),
                        ("reverse", reverse_connections(aligned, goal))):
                    for connector in connectors:
                        proposed = collapse_exact_runs(prefix+alignment+connector)
                        if not admitted(proposed):
                            continue
                        yield proposed, {"operation": "upstream_heading_replan",
                                         "segment": index, "fraction": fraction,
                                         "heading": heading,
                                         "suffix_mode": "A_"+direction}


def shunt_free_seed_proposals(start, actions, grid, time_limit=.8, beam_width=6):
    """Replace short gear excursions by swept O/Spin bridges at free stations.

    Exact coarse boundary poses are retained. This only builds initializers;
    C1 conversion, true endpoint, timing and the final 5 mm audit still apply.
    """
    deadline=time.perf_counter()+time_limit
    beam=[(collapse_exact_runs(actions),[])]
    completed=set()
    edge_cache={}
    def clear(p,action):
        key=(p,action)
        if key not in edge_cache:
            edge_cache[key]=grid.action_clear(p,action,.025)
        return edge_cache[key]
    def proxy(item):
        seq=item[0]
        return (sum(abs(a.amount)/(0.8 if a.mode==2 else 2.) for a in seq)+
                .15*sum(a.mode!=b.mode for a,b in zip(seq,seq[1:])))
    for _ in range(4):
        next_beam=[]
        for seq,history in beam:
            if time.perf_counter() >= deadline:
                break
            if action_shunting_violation(seq) is None:
                signature=TrajectoryEvaluator.signature(seq)
                if signature in completed:
                    continue
                completed.add(signature)
                yield seq,{"operation":"shunt_free_seed_repair","bridges":history}
                continue
            groups=[]
            for i,a in enumerate(seq):
                if a.mode==2:
                    continue
                direction=1 if a.amount>0 else -1
                if not groups or groups[-1][2]!=direction:
                    groups.append([i,i,direction,abs(a.amount)])
                else:
                    groups[-1][1]=i;groups[-1][3]+=abs(a.amount)
            excursion=next((j for j in range(1,len(groups)-1)
                if groups[j-1][2]==groups[j+1][2] and
                   groups[j][3]<=2*grid.half_l),None)
            if excursion is None:
                continue
            left,right=groups[excursion-1][1],groups[excursion+1][0]
            direction=groups[excursion-1][2]
            poses=replay(start,seq)
            for extent in (.4,.8,1.2):
                before,after=seq[left],seq[right]
                tail=min(extent,abs(before.amount))
                head=min(extent,abs(after.amount))
                retained=abs(before.amount)-tail
                prefix=seq[:left]+([Action(before.mode,direction*retained,before.control)]
                                  if retained>1e-9 else [])
                p=replay(start,prefix)[-1]
                q=advance(poses[right],Action(after.mode,direction*head,after.control))
                remaining=abs(after.amount)-head
                suffix=([Action(after.mode,direction*remaining,after.control)]
                        if remaining>1e-9 else [])+seq[right+1:]
                dx,dy=q.x-p.x,q.y-p.y
                distance=math.hypot(dx,dy)
                if distance<.03:
                    continue
                turn=wrap(q.th-p.th)
                centers=[(p.x,p.y),(q.x,q.y)]
                for offset in (0.,.2,-.2,.4,-.4,.6,-.6):
                    centers.append(((p.x+q.x)/2-offset*dy/distance,
                                    (p.y+q.y)/2+offset*dx/distance))
                for x,y in centers:
                    if time.perf_counter() >= deadline:
                        break
                    mid=Pose(x,y,p.th)
                    aligned=Pose(x,y,q.th)
                    bridge=[]
                    for a,b in ((p,mid),(aligned,q)):
                        if math.hypot(a.x-b.x,a.y-b.y)<1e-8:
                            action=None
                        else:
                            action=fit_action_to_pose(1,a,b,.002)
                            if action is None or (1 if action.amount>0 else -1)!=direction:
                                bridge=None;break
                        if a is p:
                            if action is not None:bridge.append(action)
                            if abs(turn)>1e-8:bridge.append(Action(2,turn,0.))
                        elif action is not None:
                            bridge.append(action)
                    if not bridge:
                        continue
                    cursor=p
                    valid=True
                    for action in bridge:
                        if not clear(cursor,action):
                            valid=False;break
                        cursor=advance(cursor,action)
                    if not valid:
                        continue
                    candidate=collapse_exact_runs(prefix+bridge+suffix)
                    next_beam.append((candidate,history+[{
                        "excursion_length_m":groups[excursion][3],
                        "window_m":extent,"spin_pose":[x,y,p.th],
                        "signed_spin_rad":turn}]))
        if not next_beam:
            break
        unique={}
        for item in sorted(next_beam,key=proxy):
            unique.setdefault(TrajectoryEvaluator.signature(item[0]),item)
        beam=list(unique.values())[:beam_width]
    for seq,history in beam:
        if (action_shunting_violation(seq) is None and
                TrajectoryEvaluator.signature(seq) not in completed):
            yield seq,{"operation":"shunt_free_seed_repair","bridges":history}


def optimize_geometry(start, goal, seed, grid, evaluator, time_limit=1., max_knots=40,
                      allow_reverse=False):
    """Fixed one proposal's modes, jointly optimize all intermediate poses.

    Distance-field/disc costs guide the NLP. This proxy does not certify
    safety, and a failed local solve is not a proof of schedule infeasibility.
    """
    import casadi as ca
    begin = time.perf_counter()
    actions = []
    for a in collapse_exact_runs(seed):
        count = (max(2, math.ceil(abs(a.amount)/.9)) if a.mode == 0 else
                 max(1, math.ceil(abs(a.amount)/.9)) if a.mode == 1 else
                 max(1, math.ceil(abs(a.amount)/math.radians(40))))
        actions.extend(Action(a.mode, a.amount/count, a.control) for _ in range(count))
    if not actions or len(actions) > max_knots:
        raise ValueError("geometry_horizon")
    refs = replay(start, actions)
    yaw = [start.th]
    for a in actions:
        yaw.append(yaw[-1]+(a.amount if a.mode == 2 else a.amount*a.control if a.mode == 0 else 0))
    # Same original O control segment shares one phi variable despite spatial
    # discretization; A receives separate curvature nodes.
    o_groups, previous, group = [], None, -1
    for a in actions:
        if a.mode == 1 and (previous is None or previous.mode != 1 or abs(wrap(a.control-previous.control)) > 1e-8):
            group += 1
        o_groups.append(group if a.mode == 1 else -1)
        previous = a
    opt = ca.Opti()
    n = len(actions)
    X, D, C = opt.variable(3, n+1), opt.variable(n), opt.variable(n)
    opt.subject_to(X[:, 0] == ca.vertcat(start.x, start.y, start.th))
    target_yaw = yaw[-1]+wrap(goal.th-yaw[-1])
    opt.subject_to(X[:, -1] == ca.vertcat(goal.x, goal.y, target_yaw))
    opt.subject_to(opt.bounded(grid.ox, X[0, :], grid.ox+grid.width*grid.res))
    opt.subject_to(opt.bounded(grid.oy, X[1, :], grid.oy+grid.height*grid.res))
    # Unwrapped yaw remains free; no per-seed-knot orientation corridor.
    opt.subject_to(opt.bounded(min(yaw+[target_yaw])-math.pi, X[2, :], max(yaw+[target_yaw])+math.pi))
    field = distance_transform_edt(np.pad(~grid.occ, 1, constant_values=False))[1:-1, 1:-1]*grid.res
    field = np.maximum(0, field-grid.res/math.sqrt(2))
    x_axis = grid.ox+(np.arange(grid.width)+.5)*grid.res
    y_axis = grid.oy+(np.arange(grid.height)+.5)*grid.res
    distance = ca.interpolant("distance", "linear", [x_axis.tolist(), y_axis.tolist()],
                              field.T.ravel(order="F").tolist())
    pref = evaluator.profile.preferred_clearance_factor*evaluator.width
    feat = features(grid)
    turning_radius = evaluator.limits.r_min_ack
    objective = 0
    group_variable = {}
    for k, a in enumerate(actions):
        if time.perf_counter()-begin > time_limit-.03:
            raise ValueError("geometry_build_budget")
        th = X[2, k]
        sign = -1 if allow_reverse and a.mode != 2 and a.amount < 0 else 1
        magnitude = sign*D[k]
        if a.mode == 0:
            opt.subject_to(opt.bounded(.002, magnitude, max(1.8, 1.8*abs(a.amount))))
            opt.subject_to(opt.bounded(-1/turning_radius, C[k], 1/turning_radius))
            z = D[k]*C[k]/2
            dx, dy = D[k]*_symbolic_sinc(z, ca)*ca.cos(th+z), D[k]*_symbolic_sinc(z, ca)*ca.sin(th+z)
            dth = D[k]*C[k]
            objective += evaluator.profile.shape*magnitude*(C[k]*turning_radius)**2/evaluator.L0
            if k and actions[k-1].mode == 0:
                objective += turning_radius**2*(C[k]-C[k-1])**2/(.05+.5*(ca.fabs(D[k])+ca.fabs(D[k-1])))/evaluator.L0
        elif a.mode == 1:
            opt.subject_to(opt.bounded(.002, magnitude, max(1.8, 1.8*abs(a.amount))))
            opt.subject_to(opt.bounded(-math.pi/2, C[k], math.pi/2))
            if o_groups[k] in group_variable:
                opt.subject_to(C[k] == C[group_variable[o_groups[k]]])
            else:
                group_variable[o_groups[k]] = k
            dx, dy, dth = D[k]*ca.cos(th+C[k]), D[k]*ca.sin(th+C[k]), 0
            if k and actions[k-1].mode == 1:
                delta = ca.sqrt((C[k]-C[k-1])**2+1e-10)
                objective += evaluator.profile.adjustment*(delta/(math.pi/4)+delta/(delta+.03))
        else:
            opt.subject_to(opt.bounded(-math.pi, D[k], math.pi))
            opt.subject_to(C[k] == 0)
            dx, dy, dth = 0, 0, D[k]
        opt.subject_to(X[:, k+1] == X[:, k]+ca.vertcat(dx, dy, dth))
        amount = ca.sqrt(D[k]**2+1e-10) if a.mode == 2 else magnitude
        rate = evaluator.limits.omega_max if a.mode == 2 else evaluator.limits.v_max
        objective += evaluator.profile.time*amount/(rate*evaluator.T0)
        if a.mode != 2:
            objective += evaluator.profile.length*amount/evaluator.L0
            if sign < 0:
                objective += evaluator.profile.reverse*amount/evaluator.L0
        # Exact primitive interior positions: three stations, followed by
        # dense independent SAT after solving. Feature discs are conservative
        # proxies; local slack is NOT a relaxation of final safety acceptance.
        for fraction in (0., .5, 1.):
            if a.mode == 0:
                zf = fraction*D[k]*C[k]/2
                xf = X[0, k]+fraction*D[k]*_symbolic_sinc(zf, ca)*ca.cos(th+zf)
                yf = X[1, k]+fraction*D[k]*_symbolic_sinc(zf, ca)*ca.sin(th+zf)
            else:
                xf, yf = X[0, k]+fraction*dx, X[1, k]+fraction*dy
            tf = th+fraction*dth
            for ox, oy, feature_radius in feat:
                fx, fy = xf+ox*ca.cos(tf)-oy*ca.sin(tf), yf+ox*ca.sin(tf)+oy*ca.cos(tf)
                opt.subject_to(opt.bounded(float(x_axis[0]), fx, float(x_axis[-1])))
                opt.subject_to(opt.bounded(float(y_axis[0]), fy, float(y_axis[-1])))
                clearance = distance(ca.vertcat(fx, fy))-feature_radius
                opt.subject_to(clearance >= -.035)
                deficit = ca.fmax(0, (pref-clearance)/pref)
                objective += evaluator.profile.clearance*amount*deficit**2/(12*evaluator.L0)
    opt.minimize(objective)
    opt.set_initial(X, np.array([[p.x for p in refs], [p.y for p in refs], yaw]))
    opt.set_initial(D, [a.amount if allow_reverse or a.mode == 2 else abs(a.amount) for a in actions])
    opt.set_initial(C, [a.control for a in actions])
    remaining = time_limit-(time.perf_counter()-begin)
    if remaining < .05:
        raise ValueError("geometry_setup_budget")
    opt.solver("ipopt", {"print_time": False}, {"print_level": 0, "max_iter": 120,
                                               "max_wall_time": remaining, "tol": 1e-6})
    try:
        sol = opt.solve_limited()
        amounts, controls = np.array(sol.value(D)).ravel(), np.array(sol.value(C)).ravel()
    except RuntimeError:
        # No debug-iterate promotion: preserve the original safe candidate.
        raise ValueError("geometry_no_solution")
    refined = collapse_exact_runs([Action(a.mode, float(d), float(c))
                                   for a, d, c in zip(actions, amounts, controls)])
    return refined, {"solver": "whole_path_ipopt", "objective_is_proxy": True,
                     "solver_status": opt.stats().get("return_status", "unknown"),
                     "geometry_seconds": time.perf_counter()-begin,
                     "continuous_nodes": n, "fixed_boundary_poses": 2}


@dataclass
class ModeMasterConfig:
    budget_s: float = 8.
    nlp_time_s: float = 2.
    max_nlp_calls: int = 4
    max_candidates: int = 20
    max_window: int = 8
    sampling_dt: float = .05
    validation_reserve_s: float = .5
    candidate_evaluation_reserve_s: object = None
    max_duration_regression: float = .10
    allow_reverse: object = None
    allow_shunting: bool = False
    allow_reverse_fallback: object = None  # deprecated constructor alias
    continuous_motion_v2: bool = False
    continuous_fit_max_iterations: int = 40
    continuous_smoothing_bandwidth_m: float = .4
    continuous_control_spacing_m: float = .4
    mode_anchor_min_switches: int = 12
    mode_anchor_extra_budget_s: float = 15.0
    mode_anchor_parent_candidates: int = 2
    mode_anchor_max_candidates: int = 12
    legacy_geometry_fallback_s: float = 1.5
    legacy_geometry_max_candidates: int = 4
    # Zero retains the exhaustive anytime frontier within the wall budget.
    max_structural_expansions: int = 0
    endpoint_fit_feasible_stop: bool = False
    max_stagnant_expansions: int = 0
    anchor_refinement_early_accept_relative_gain: float = 0.
    strict_auxiliary_first: bool = False
    selection_objective: str = "common_score"
    min_time_gain_to_expand_s: float = 0.

    def __post_init__(self):
        if self.candidate_evaluation_reserve_s is None:
            self.candidate_evaluation_reserve_s = min(
                3.0, max(.05, .15*self.budget_s),
                .5*max(.1, self.budget_s-self.validation_reserve_s))
        if self.allow_reverse_fallback is not None:
            if not isinstance(self.allow_reverse_fallback, bool):
                raise ValueError("legacy_reverse_policy_requires_boolean")
            if (self.allow_reverse is not None and
                    self.allow_reverse != self.allow_reverse_fallback):
                raise ValueError("conflicting_reverse_policy_aliases")
            if self.allow_reverse is None:
                self.allow_reverse = self.allow_reverse_fallback
        if self.allow_reverse is None:
            self.allow_reverse = True
        if not isinstance(self.allow_reverse, bool):
            raise ValueError("direction_policy_requires_boolean")
        if not isinstance(self.allow_shunting,bool):
            raise ValueError("shunting_policy_requires_boolean")
        if not isinstance(self.continuous_motion_v2, bool):
            raise ValueError("continuous_motion_v2_requires_boolean")
        if not isinstance(self.endpoint_fit_feasible_stop, bool):
            raise ValueError("endpoint_fit_feasible_stop_requires_boolean")
        if not isinstance(self.strict_auxiliary_first, bool):
            raise ValueError("strict_auxiliary_first_requires_boolean")
        if self.selection_objective not in ("common_score", "minimum_time"):
            raise ValueError("unknown_selection_objective")
        if (not math.isfinite(self.min_time_gain_to_expand_s) or
                self.min_time_gain_to_expand_s < 0.):
            raise ValueError("invalid_time_expansion_gain")
        if (not all(math.isfinite(x) and x > 0 for x in
                    (self.budget_s, self.nlp_time_s, self.sampling_dt)) or
                not 0 <= self.validation_reserve_s < self.budget_s or
                not 0 < self.candidate_evaluation_reserve_s <
                    self.budget_s-self.validation_reserve_s or
                not math.isfinite(self.max_duration_regression) or
                self.max_duration_regression < 0 or
                self.max_nlp_calls < 0 or self.max_candidates < 1 or self.max_window < 1 or
                self.mode_anchor_min_switches < 1 or
                not math.isfinite(self.mode_anchor_extra_budget_s) or
                self.mode_anchor_extra_budget_s < 0 or
                self.mode_anchor_parent_candidates < 1 or
                self.mode_anchor_max_candidates < 1 or
                not math.isfinite(self.legacy_geometry_fallback_s) or
                self.legacy_geometry_fallback_s <= 0 or
                self.legacy_geometry_max_candidates < 1 or
                self.max_structural_expansions < 0 or
                self.max_stagnant_expansions < 0 or
                not math.isfinite(self.anchor_refinement_early_accept_relative_gain) or
                not 0 <= self.anchor_refinement_early_accept_relative_gain < 1 or
                self.continuous_fit_max_iterations < 1 or
                not math.isfinite(self.continuous_smoothing_bandwidth_m) or
                self.continuous_smoothing_bandwidth_m <= 0 or
                not math.isfinite(self.continuous_control_spacing_m) or
                self.continuous_control_spacing_m <= 0):
            raise ValueError("invalid_mode_master_budget")


def duration_guard_candidates(candidates, max_regression):
    """Keep score competition within a bounded slowdown of the fastest feasible route."""
    if not candidates:
        return [], {"applied": True, "reason": "empty_candidate_set"}
    fastest = min(float(item["timed"].duration) for item in candidates)
    cap = fastest*(1.+float(max_regression))
    eligible = [item for item in candidates
                if float(item["timed"].duration) <= cap+1e-9]
    eligible_objects = {id(item) for item in eligible}
    excluded = [item["id"] for item in candidates
                if id(item) not in eligible_objects]
    return eligible, {"applied": True, "fastest_duration_s": fastest,
                      "max_regression_fraction": float(max_regression),
                      "duration_cap_s": cap,
                      "excluded_candidate_ids": excluded,
                      "eligible_candidate_ids": [item["id"] for item in eligible]}


class ModePathMaster:
    def __init__(self, evaluator, config=None):
        self.evaluator = evaluator
        self.config = config or ModeMasterConfig()

    def solve(self, seed, auxiliary_seeds=None):
        begin = time.perf_counter()
        seed_mode_switches = sum(a.mode != b.mode for a,b in zip(seed,seed[1:]))
        adaptive_budget_trigger = mode_anchor_budget_trigger(self.config, seed)
        adaptive_budget_extension_s = (self.config.mode_anchor_extra_budget_s
                                       if adaptive_budget_trigger else 0.0)
        effective_budget_s = self.config.budget_s+adaptive_budget_extension_s
        deadline = begin+effective_budget_s
        work_deadline = deadline-self.config.validation_reserve_s
        # A full continuous candidate audit can take seconds on a long route.
        # Stop opening new audits early enough to finish the active one and
        # leave the configured final-validation reserve intact.
        search_deadline = (work_deadline-
                           self.config.candidate_evaluation_reserve_s)
        nlp_call_limit = self.config.max_nlp_calls
        candidates, seen, errors = [], set(), []
        minimum_time = self.config.selection_objective == "minimum_time"
        def primary_value(item):
            return (float(item["timed"].duration)/self.evaluator.T0
                    if minimum_time else item["audit"]["score"])
        geometry_store = GeometryIncumbentStore(self.evaluator)
        first_feasible = None
        nlp_calls = 0
        nlp_modes_seen = set()
        continuous_nlp_seeds_seen = set()
        continuous_nlp_reused = 0
        continuous_optimizer_report = None
        legacy_geometry_report = None
        mode_anchor_reports = []
        anchor_refinement_accepted = False
        coarse_sweep_cache = {}
        coarse_sweep_counts = {"checked_edges": 0, "reused_edges": 0}
        duration_estimate = PrimitiveTimingCache(self.evaluator.limits,
                                                allow_reverse=self.config.allow_reverse)

        def coarse_action_clear(pose, action, spacing):
            # Same stations and occupied-cell SAT as Grid.action_clear,
            # queried in small batches against the shared obstacle index.
            # Chunking preserves early rejection on blocked approaches.
            rotation = abs(action.amount) if action.mode == 2 else (
                abs(action.amount*action.control) if action.mode == 0 else 0.)
            translation = 0. if action.mode == 2 else abs(action.amount)
            count = max(1, math.ceil((translation+
                self.evaluator.grid.corner*rotation)/spacing))
            for first in range(1, count+1, 32):
                poses = [advance(pose, Action(action.mode,
                    action.amount*i/count, action.control))
                    for i in range(first, min(count+1, first+32))]
                if not np.all(self.evaluator.safe_distance.poses_clear(poses)):
                    return False
            return True

        def coarse_path_clear(poses, actions, spacing=.04):
            if not poses or not self.evaluator.grid.pose_clear(poses[0]):
                return False
            for pose, action in zip(poses, actions):
                key = (pose.x, pose.y, pose.th, action.mode, action.amount,
                       action.control, spacing)
                if key not in coarse_sweep_cache:
                    coarse_sweep_cache[key] = coarse_action_clear(pose, action, spacing)
                    coarse_sweep_counts["checked_edges"] += 1
                else:
                    coarse_sweep_counts["reused_edges"] += 1
                if not coarse_sweep_cache[key]:
                    return False
            return True
        corner_distance = FootprintDistance(self.evaluator.grid,
            2*self.evaluator.grid.half_l+.02,
            2*self.evaluator.grid.half_w+.02,
            obstacle_index=self.evaluator.safe_distance)
        corner_edge_cache = {}
        def corner_path_clear(poses, actions):
            # Prefer connector placements with room for C1 conversion and
            # the strict sweep's between-station inflation. This is only a
            # candidate-generator guard; original certified paths stay eligible.
            if not poses or not bool(corner_distance.poses_clear([poses[0]])[0]):
                return False
            for pose, action in zip(poses, actions):
                key = (pose, action)
                if key not in corner_edge_cache:
                    rotation = abs(action.amount) if action.mode == 2 else (
                        abs(action.amount*action.control) if action.mode == 0 else 0.)
                    travel = 0. if action.mode == 2 else abs(action.amount)
                    count = max(1, math.ceil((travel+corner_distance.corner*rotation)/.025))
                    clear = True
                    for first in range(1, count+1, 32):
                        stations = [advance(pose, Action(action.mode,
                            action.amount*i/count, action.control))
                            for i in range(first, min(count+1, first+32))]
                        if not np.all(corner_distance.poses_clear(stations)):
                            clear = False
                            break
                    corner_edge_cache[key] = clear
                if not corner_edge_cache[key]:
                    return False
            return True
        stage_seconds = {name: 0.0 for name in (
            "continuous_plan_build", "endpoint_fit", "geometry_screen", "retime", "independent_audit",
            "proposal_generation", "proposal_ranking", "nlp_solve", "final_scoring")}
        stage_counts = {name: 0 for name in (
            "continuous_plan_build", "endpoint_fit", "geometry_screen", "retime", "independent_audit",
            "proposal_generation", "proposal_ranking", "nlp_solve", "final_scoring")}
        funnel = {name: 0 for name in (
            "proposals_generated", "duplicate", "empty", "reverse_disallowed",
            "estimate_failed", "coarse_sweep_rejected", "terminal_rejected",
            "shortlisted", "candidate_evaluation_attempted", "candidate_feasible",
            "nlp_attempted", "budget_stopped", "anchor_mode_proposals")}

        def evaluate(actions, source, parent=None, detail=None,
                     reverse_allowed=None, reserve_budget=True,
                     continuous_seed=None, geometry_variant=None,
                     continuous_endpoint_tolerance=None, enable_suffix_repair=True):
            nonlocal first_feasible
            if (reserve_budget and time.perf_counter()+
                    self.config.candidate_evaluation_reserve_s > work_deadline):
                errors.append({"source": source,
                               "reason": "candidate_evaluation_budget_reserve"})
                funnel["budget_stopped"] += 1
                return None
            if reverse_allowed is None:
                reverse_allowed = self.config.allow_reverse
            key = (bool(reverse_allowed), self.evaluator.signature(actions), geometry_variant)
            if key in seen:
                funnel["duplicate"] += 1
                return None
            seen.add(key)
            funnel["candidate_evaluation_attempted"] += 1
            if not self.config.allow_shunting:
                violation = action_shunting_violation(actions)
                if violation is not None:
                    errors.append({"source":source,"parent":parent,
                                   "reason":"shunting_forbidden:"+violation["reason"],
                                   "shunting_violation":violation})
                    return None
            candidate_stages = {"continuous_plan_build": 0.0, "endpoint_fit": 0.0,
                                "geometry_screen": 0.0, "retime": 0.0, "independent_audit": 0.0}
            @contextmanager
            def measured(stage):
                stage_begin = time.perf_counter()
                stage_counts[stage] += 1
                try:
                    yield
                finally:
                    candidate_stages[stage] += time.perf_counter()-stage_begin

            rejection_start = len(errors)
            try:
                continuous_plan = None
                if self.config.continuous_motion_v2:
                    with measured("continuous_plan_build"):
                        continuous_plan = continuous_seed or plan_from_actions(
                            self.evaluator.start, self.evaluator.goal, actions)
                    endpoint_tolerance = continuous_endpoint_tolerance or (
                        .015, math.radians(.75))
                    unprojected_plan = continuous_plan
                    provider_audit = (detail or {}).get("geometry_provider", {})
                    certified_provider_seed = bool(
                        (detail or {}).get("preserve_certified_continuous_seed") and
                        continuous_seed is not None and
                        provider_audit.get("geometry_audit", {}).get(
                            "geometry_status") == "PASS")
                    if certified_provider_seed:
                        # The provider has already completed the same strict
                        # 5 mm geometry and terminal audit against this
                        # evaluator. Keep that immutable suffix repair intact;
                        # the normal post-retiming audit below still repeats
                        # the full independent geometry and dynamic contract.
                        fit_report = {
                            "success": True,
                            "status": "provider_geometry_already_certified",
                            "endpoint_projection": "preserved",
                            "provider_map_hash": provider_audit.get("map_hash"),
                        }
                    else:
                        with measured("endpoint_fit"):
                            if (detail or {}).get("preserve_mode_boundaries"):
                                continuous_plan,fit_report=fit_mode_boundaries(
                                    continuous_plan,actions,
                                    r_min_ack=self.evaluator.limits.r_min_ack,
                                    max_iterations=self.config.continuous_fit_max_iterations,
                                    stop_on_feasible=(bool(candidates) and
                                        self.config.endpoint_fit_feasible_stop),
                                    deadline=work_deadline-min(1.,self.config.candidate_evaluation_reserve_s))
                            else:
                                continuous_plan, fit_report = fit_continuous_endpoint(
                                continuous_plan, self.evaluator.limits.r_min_ack,
                                max_iterations=self.config.continuous_fit_max_iterations,
                                terminal_xy_tol=endpoint_tolerance[0],
                                terminal_yaw_tol=endpoint_tolerance[1],
                                stop_on_feasible=(bool(candidates) and
                                    self.config.endpoint_fit_feasible_stop),
                                deadline=work_deadline-min(1., self.config.candidate_evaluation_reserve_s))
                    collision = None
                    if fit_report["success"] and not certified_provider_seed:
                        with measured("geometry_screen"):
                            collision = self.evaluator.continuous_collision(continuous_plan)
                    if collision is not None or not fit_report["success"]:
                        original_fit_report = fit_report
                        errors.append({"source": source,
                            "reason": ("continuous_endpoint_projection_collision" if collision
                                       else "continuous_endpoint_fit_failed"),
                            "first_collision": collision, "fit": original_fit_report})
                        if not enable_suffix_repair:
                            return None
                        # Endpoint projection must not discard a safe prefix.
                        # Restrict a retry to the tail of the unprojected C1
                        # curve, with full strict sweep before any retiming.
                        with measured("geometry_screen"):
                            original_collision = self.evaluator.continuous_collision(unprojected_plan)
                        repair_attempts = []
                        if original_collision is None:
                            for tail_length in (2.,4.):
                                fit_deadline = work_deadline-min(
                                    1.,self.config.candidate_evaluation_reserve_s)
                                if time.perf_counter() >= fit_deadline:
                                    break
                                try:
                                    with measured("endpoint_fit"):
                                        repaired, repair_fit = fit_continuous_suffix(
                                            unprojected_plan, tail_length_m=tail_length,
                                            r_min_ack=self.evaluator.limits.r_min_ack,
                                            max_iterations=self.config.continuous_fit_max_iterations,
                                            terminal_xy_tol=1e-7,terminal_yaw_tol=1e-7,
                                            deadline=fit_deadline)
                                    with measured("geometry_screen"):
                                        repair_collision = (self.evaluator.continuous_collision(repaired)
                                                            if repair_fit["success"] else None)
                                    repair_attempts.append(dict(repair_fit,
                                        first_collision=repair_collision))
                                    if repair_fit["success"] and repair_collision is None:
                                        continuous_plan, fit_report, collision = repaired, repair_fit, None
                                        fit_report["original_projection"] = original_fit_report
                                        fit_report["repair_attempts"] = repair_attempts
                                        break
                                except (ValueError,RuntimeError) as exc:
                                    repair_attempts.append({"tail_length_m":tail_length,
                                                            "reason":str(exc)})
                        if collision is not None or not fit_report["success"]:
                            errors.append({"source": source,
                                "reason": "continuous_suffix_repair_failed",
                                "first_collision": collision,
                                "unprojected_collision": original_collision,
                                "repair_attempts": repair_attempts})
                            return None
                    with measured("retime"):
                        timed = retime_continuous(
                            continuous_plan, self.evaluator.limits,
                            self.config.sampling_dt)
                    with measured("independent_audit"):
                        audit = self.evaluator.evaluate_continuous_geometry(
                            continuous_plan, timed, allow_reverse=reverse_allowed,
                            include_metrics=not minimum_time, defer_score=minimum_time)
                else:
                    with measured("retime"):
                        timed = retime(self.evaluator.start, actions, self.evaluator.limits,
                                       self.config.sampling_dt, allow_reverse=reverse_allowed)
                    with measured("independent_audit"):
                        audit = self.evaluator.evaluate(actions, timed, allow_reverse=reverse_allowed)
                        for level in range(1, 5):
                            if audit.get("reason") != "display_interpolated_swept_collision":
                                break
                            timed = retime(self.evaluator.start, actions, self.evaluator.limits,
                                           self.config.sampling_dt/(2**level), allow_reverse=reverse_allowed)
                            audit = self.evaluator.evaluate(actions, timed, allow_reverse=reverse_allowed)
                if audit["status"] != "FEASIBLE":
                    if audit.get("geometry_status") == "PASS":
                        geometry_store.preserve(actions, source, parent,
                            plan=continuous_plan, audit=audit,
                            reverse_allowed=reverse_allowed,
                            found_at_s=time.perf_counter()-begin,
                            execution_error=audit.get("reason"))
                    errors.append({"source": source, "reason": audit.get("reason"), "audit": audit})
                    return None
                if first_feasible is None:
                    first_feasible = time.perf_counter()-begin
                record = {"id": len(candidates), "source": source, "parent": parent,
                          "actions": collapse_exact_runs(actions), "timed": timed,
                          "audit": audit, "detail": detail or {},
                          "found_at_s": time.perf_counter()-begin,
                          "stage_timings_s": candidate_stages}
                if hasattr(timed, "report"):
                    record["timing_diagnostics"] = timed.report()
                else:
                    switch_events = [phase for phase in timed.phases
                                     if phase.get("mode_switch_event")]
                    record["timing_diagnostics"] = {
                        "profile": asdict(self.evaluator.limits),
                        "duration_s": float(timed.duration),
                        "translation_s": float(timed.moving_seconds),
                        "spin_s": float(timed.spin_seconds),
                        "mode_switch_hold_seconds": float(sum(
                            phase["duration"] for phase in switch_events)),
                        "mode_switch_count": len(switch_events),
                        "phases": timed.phases}
                if continuous_plan is not None:
                    record["continuous_plan"] = continuous_plan
                    record["fit_report"] = fit_report
                candidates.append(record)
                funnel["candidate_feasible"] += 1
                return record
            except (ValueError, RuntimeError) as exc:
                errors.append({"source": source, "reason": str(exc)})
                # Retiming failure must not erase a spatially valid route.
                # Audit only on this failure path; ordinary successful
                # candidates already carry a full independent certificate.
                if candidate_stages["retime"] > 0.:
                    try:
                        with measured("geometry_screen"):
                            geometry_store.preserve(actions, source, parent,
                                plan=continuous_plan, reverse_allowed=reverse_allowed,
                                found_at_s=time.perf_counter()-begin,
                                execution_error=str(exc))
                    except (ValueError, RuntimeError) as retention_error:
                        errors.append({"source": source,
                            "reason": "geometry_retention_failed:"+str(retention_error)})
                return None
            finally:
                for stage, duration in candidate_stages.items():
                    stage_seconds[stage] += duration
                for rejection in errors[rejection_start:]:
                    rejection.update(parent=parent, detail=detail or {},
                                     mode_direction_word=[[a.mode, 1 if a.amount >= 0 else -1]
                                                          for a in actions],
                                     stage_timings_s=dict(candidate_stages),
                                     remaining_budget_s=max(0., deadline-time.perf_counter()))

        def evaluate_schedule(actions, source, parent=None, detail=None,
                              reserve_budget=True, reverse_allowed=None):
            # Every structural schedule needs a continuous control initial
            # guess. Regularizing only the original C++ seed left later
            # winners with centimetre-scale curvature jumps and crawl speeds.
            varying = False
            if self.config.continuous_motion_v2:
                plan = plan_from_actions(self.evaluator.start,
                                         self.evaluator.goal, actions)
                # Establish a safe incumbent before spending time on smooth
                # initializers. If the raw continuous conversion is already
                # certifiable, later geometry optimization starts from that
                # immutable snapshot; failed smoothing guesses cannot consume
                # the refinement and final-audit reserve.
                raw_result = evaluate(
                    actions, source, parent, detail,
                    reverse_allowed=reverse_allowed,
                    reserve_budget=reserve_budget, continuous_seed=plan,
                    geometry_variant=("raw_schedule",),
                    enable_suffix_repair=False)
                if (raw_result is None and source == "combined_spin_A_connections" and
                        detail and detail.get("corner_refinement_pass") and
                        self.evaluator.limits.translation_speed_policy == "mode_switch_only"):
                    # Wide control blending can cut a tight but swept-clear
                    # constant-curvature connector into the inner wall. Offer
                    # a narrower C1 transition, preserving continuous controls
                    # and all independent geometric/time certification gates.
                    corner_plan = plan_from_actions(self.evaluator.start,
                        self.evaluator.goal, actions, transition_length=.04)
                    raw_result = evaluate(actions, source, parent,
                        dict(detail or {}, control_transition_length_m=.04),
                        reverse_allowed=reverse_allowed, reserve_budget=reserve_budget,
                        continuous_seed=corner_plan,
                        geometry_variant=("narrow_corner_schedule", .04),
                        enable_suffix_repair=False)
                if (raw_result is not None and minimum_time and
                        source == "combined_spin_A_connections" and
                        self.evaluator.limits.translation_speed_policy == "mode_switch_only"):
                    # Spend this family's bounded audit quota on distinct
                    # corner placements. Its certified raw initializer already
                    # has continuous controls and executable timing; smoothing
                    # variants remain recovery options when raw conversion fails.
                    return raw_result
                varying = any(any(abs(b.value-a.value) > 1e-8
                                  for a, b in zip(block.knots, block.knots[1:]))
                              for block in plan.blocks if block.mode != 2)
                if varying:
                    bandwidth = self.config.continuous_smoothing_bandwidth_m
                    spacing = self.config.continuous_control_spacing_m
                    def evaluate_scale(scale, spacing_scale=None, enable_suffix_repair=False):
                        local_bandwidth = bandwidth*scale
                        local_spacing = spacing*(scale if spacing_scale is None else spacing_scale)
                        regularized = smooth_control_candidate(
                            plan, bandwidth=local_bandwidth,
                            station_spacing=local_spacing)
                        return evaluate(actions, source, parent,
                            dict(detail or {}, control_initialization="regularized",
                                 bandwidth_m=local_bandwidth,
                                 control_spacing_m=local_spacing),
                            reverse_allowed=reverse_allowed,
                            reserve_budget=reserve_budget, continuous_seed=regularized,
                            geometry_variant=("regularized_schedule",
                                              local_bandwidth, local_spacing,
                                              enable_suffix_repair),
                            enable_suffix_repair=enable_suffix_repair,
                            continuous_endpoint_tolerance=(1e-7, 1e-7))
                    result = evaluate_scale(1.)
                    if result is not None:
                        return result
                    # Preserve the previous 30 cm / 20 cm initializer as the
                    # second resolution before accepting sharp seed controls.
                    # Both scales are subject to the same geometry audit.
                    result = evaluate_scale(.75, .5)
                    if result is not None:
                        return result
                # Keep a safe raw geometry as an incumbent, but let the
                # established regularized initializers compete before
                # declaring it the only viable schedule. The raw candidate
                # remains in `candidates`, so a failed or unsafe smoother
                # cannot delete it.
                if raw_result is not None:
                    return raw_result
            # A wider transition can be unsafe in a genuinely tight passage.
            # Retain the narrower geometry only after the same full audit.
            # v2 already audited this exact raw conversion above. Repeating
            # its projection under another variant key cannot rescue it;
            # empty-pool suffix recovery below remains a separate attempt.
            result = (None if self.config.continuous_motion_v2 else
                evaluate(actions, source, parent, detail,
                         reverse_allowed=reverse_allowed,
                         reserve_budget=reserve_budget, enable_suffix_repair=False))
            if result is None and self.config.continuous_motion_v2 and not candidates:
                # Keep established initializer resolutions in competition
                # before adding a suffix repair. A repaired first resolution
                # must not hide an already feasible smoother second one.
                # This is empty-pool recovery; don't spend full repair/audit
                # slots on every failed neighbor once an incumbent exists.
                if varying:
                    result = evaluate_scale(1., enable_suffix_repair=True)
                    if result is None:
                        result = evaluate_scale(.75,.5,enable_suffix_repair=True)
                else:
                    result = evaluate(actions,source,parent,detail,
                        reverse_allowed=reverse_allowed,reserve_budget=reserve_budget,
                        geometry_variant=("suffix_repair",),enable_suffix_repair=True)
            return result

        def refine_continuous_geometry(geometry_seed):
            """Spend the available local-solver budget on a certified incumbent."""
            nonlocal continuous_optimizer_report, nlp_calls, continuous_nlp_reused
            if (not self.config.continuous_motion_v2 or geometry_seed is None or
                    nlp_calls >= nlp_call_limit):
                return None
            if (minimum_time and self.evaluator.limits.translation_speed_policy ==
                    "mode_switch_only"):
                # This NLP freezes lengths, directions, modes and Spin angles.
                # With curvature-independent cruise it cannot shorten time;
                # use the budget to explore different route structures.
                continuous_optimizer_report = {"attempted": False,
                    "status": "fixed_schedule_time_invariant",
                    "selection_objective": "minimum_time"}
                return None
            # Do not immediately optimize an already optimized child again,
            # or rebuild the same local NLP for floating-point copies of a
            # previously tried fixed geometry. This affects search work only;
            # it never reuses a safety certificate for a different path.
            plan = geometry_seed["continuous_plan"]
            key = tuple((b.mode,b.direction,round(b.length,8),round(b.signed_angle,8),
                tuple((round(k.s,8),round(k.value,8),round(k.derivative,8))
                      for k in b.knots)) for b in plan.blocks)
            if (geometry_seed["source"] == "continuous_geometry_nlp" or
                    key in continuous_nlp_seeds_seen):
                continuous_nlp_reused += 1
                return None
            nlp_budget = (work_deadline-time.perf_counter()-
                          self.config.candidate_evaluation_reserve_s)
            if nlp_budget <= .15:
                return None
            nlp_begin = time.perf_counter()
            continuous_nlp_seeds_seen.add(key)
            try:
                # CasADi/IPOPT wall-time limits omit model construction. Give
                # certified anchor candidates priority while the shared
                # budget still has room; the incumbent remains eligible if
                # the local solve times out or produces no audited candidate.
                nlp_time_limit = min(self.config.nlp_time_s,
                                     max(.1, .27*nlp_budget))
                optimized_plan, continuous_optimizer_report = (
                    optimize_continuous_geometry(
                        geometry_seed["continuous_plan"], self.evaluator,
                        time_limit=nlp_time_limit, sample_spacing=.65))
                stage_seconds["nlp_solve"] += time.perf_counter()-nlp_begin
                stage_counts["nlp_solve"] += 1
                if continuous_optimizer_report.get("attempted"):
                    nlp_calls += 1
                    funnel["nlp_attempted"] += 1
                if continuous_optimizer_report.get("status") in (
                        "solver_feasible_candidate", "budget_feasible_iterate",
                        "converged_feasible", "iteration_limit_feasible"):
                    return evaluate(
                        geometry_seed["actions"], "continuous_geometry_nlp",
                        geometry_seed["id"], continuous_optimizer_report,
                        reserve_budget=False, continuous_seed=optimized_plan,
                        geometry_variant=("direct_continuous_nlp",))
            except (ValueError, RuntimeError) as exc:
                stage_seconds["nlp_solve"] += time.perf_counter()-nlp_begin
                stage_counts["nlp_solve"] += 1
                errors.append({"source": "continuous_geometry_nlp",
                               "reason": str(exc).splitlines()[-1]})
            return None

        seed = collapse_exact_runs(seed)
        initial = evaluate_schedule(seed, "coarse_incumbent", reserve_budget=False)
        strict_auxiliary_first_used = False
        if (initial is None and self.config.continuous_motion_v2 and
                self.config.strict_auxiliary_first and not self.config.allow_shunting and
                action_shunting_violation(seed) is not None):
            # A policy-invalid provisional seed must not spend seconds in
            # repair before a legal planner alternative gets its first audit.
            for proposal, proposal_source in (auxiliary_seeds or []):
                if time.perf_counter() >= search_deadline:
                    break
                if action_shunting_violation(proposal) is not None:
                    continue
                # This is a quick raw-geometry audit, not a new primary
                # route. Keep the established auxiliary refinement order:
                # promoting it to `initial` would run anchor search before
                # the structural connectors that shorten this route.
                auxiliary_incumbent = evaluate(proposal,
                    str(proposal_source or "planner_auxiliary_candidate"),
                    detail={"seed_role":"legal_auxiliary_before_provisional_repair"},
                    reserve_budget=False, geometry_variant=("raw_schedule",),
                    enable_suffix_repair=False)
                if auxiliary_incumbent is not None:
                    strict_auxiliary_first_used = True
                    break
        if (initial is None and not candidates and not self.config.allow_shunting and
                action_shunting_violation(seed) is not None):
            for proposal,details in shunt_free_seed_proposals(
                    self.evaluator.start,seed,self.evaluator.grid,
                    time_limit=min(.8,max(.1,search_deadline-time.perf_counter()))):
                funnel["proposals_generated"] += 1
                initial=evaluate_schedule(proposal,"shunt_free_seed_repair",
                    detail=dict(details,preserve_mode_boundaries=True),reserve_budget=False)
                if initial is not None or time.perf_counter()>=search_deadline:
                    break
        initial_mode_switches = (mode_anchor_switch_count(
            self.config, initial["actions"]) if initial is not None else None)
        # Search a moved-anchor mode schedule after a certified incumbent
        # exists. The first bounded A/O merge exposes useful parent routes;
        # anchor_mode_sequences then changes several mode boundaries together
        # and keeps only swept-safe, forward-only initializers. Each candidate
        # still goes through the same continuous fit, retiming and strict audit.
        if (self.config.continuous_motion_v2 and initial is not None and
                initial_mode_switches is not None and
                time.perf_counter() < search_deadline):
            mode_neighbor_begin = time.perf_counter()
            parent_options = mode_schedule_neighbor_candidates(
                self.evaluator.start, initial["actions"], self.evaluator.grid,
                max_candidates=self.config.mode_anchor_parent_candidates,
                max_window=self.config.max_window, sweep_step=.04,
                require_proxy_improvement=False,
                prefer_switch_reduction=True)
            stage_seconds["proposal_generation"] += time.perf_counter()-mode_neighbor_begin
            stage_counts["proposal_generation"] += 1
            if not parent_options:
                parent_options = [(float("inf"), initial["actions"], [],
                                   {"operation": "incumbent_anchor_fallback"})]
            anchor_options = []
            base_switches = sum(a.mode != b.mode for a,b in
                                zip(initial["actions"], initial["actions"][1:]))
            for parent_rank, (_, parent_actions, _, parent_detail) in enumerate(
                    parent_options[:self.config.mode_anchor_parent_candidates]):
                if time.perf_counter() >= search_deadline:
                    break
                search_profile = {}
                anchor_begin = time.perf_counter()
                try:
                    generated = anchor_mode_sequences(
                        self.evaluator.start, parent_actions, self.evaluator.grid,
                        max_candidates=self.config.mode_anchor_max_candidates,
                        max_span=3, beam_width=12, max_edge_length=1.6,
                        profile=search_profile)
                except (ValueError, RuntimeError) as exc:
                    mode_anchor_reports.append({
                        "parent_rank": parent_rank,
                        "parent_detail": parent_detail,
                        "status": "generation_failed",
                        "reason": str(exc).splitlines()[-1],
                        "search_profile": search_profile})
                    continue
                stage_seconds["proposal_generation"] += time.perf_counter()-anchor_begin
                stage_counts["proposal_generation"] += 1
                mode_anchor_reports.append({
                    "parent_rank": parent_rank,
                    "parent_mode_switches": sum(
                        a.mode != b.mode for a,b in zip(parent_actions,parent_actions[1:])),
                    "parent_detail": parent_detail,
                    "generated_count": len(generated),
                    "search_profile": search_profile})
                for actions, proxy_score in generated:
                    switches = sum(a.mode != b.mode for a,b in zip(actions,actions[1:]))
                    if switches >= base_switches:
                        continue
                    if (has_reverse_motion(actions) or
                            action_shunting_violation(actions) is not None):
                        continue
                    signature = self.evaluator.signature(actions)
                    anchor_options.append((switches, float(proxy_score), signature,
                                           actions, parent_rank, parent_detail))
            anchor_options.sort(key=lambda item:(item[0],item[1]))
            selected_anchor_options = []
            seen_anchor_words = set()
            for item in anchor_options:
                _, _, _, actions, parent_rank, parent_detail = item
                word = tuple(a.mode for i,a in enumerate(actions)
                             if i == 0 or a.mode != actions[i-1].mode)
                signature = item[2]
                if signature in {prior[2] for prior in selected_anchor_options}:
                    continue
                if word in seen_anchor_words:
                    continue
                selected_anchor_options.append(item)
                seen_anchor_words.add(word)
                if len(selected_anchor_options) >= 2:
                    break
            funnel["anchor_mode_proposals"] += len(anchor_options)
            for switches, proxy_score, _, actions, parent_rank, parent_detail in selected_anchor_options:
                if time.perf_counter() >= search_deadline:
                    funnel["budget_stopped"] += 1
                    break
                anchor_candidate = evaluate_schedule(
                    actions, "anchor_mode_schedule",
                    initial["id"],
                    {"operation": "anchor_mode_sequences",
                     "preserve_mode_boundaries": True,
                     "mode_switch_count": switches,
                     "proxy_score": proxy_score,
                     "anchor_parent_rank": parent_rank,
                     "anchor_parent_detail": parent_detail},
                    reserve_budget=True)
                if anchor_candidate is not None:
                    before_refinement = min(primary_value(c) for c in candidates)
                    refined = refine_continuous_geometry(anchor_candidate)
                    required_gain = self.config.anchor_refinement_early_accept_relative_gain
                    if (required_gain > 0. and refined is not None and
                            primary_value(refined) <= before_refinement*(1.-required_gain)):
                        anchor_refinement_accepted = True
                        break
                    if nlp_calls >= nlp_call_limit:
                        break
        auxiliary_seed_reports = []

        def evaluate_auxiliary_seed_pool():
            """Spend only leftover search time on extra planner seeds.

            Once a strict incumbent exists, the shared budget first explores
            structural neighbors and bounded geometry refinement. Planner
            auxiliary seeds remain eligible afterwards; when no incumbent
            exists they are evaluated immediately because they may establish
            the only feasible route.
            """
            for proposal, proposal_source in (auxiliary_seeds or []):
                proposal = collapse_exact_runs(proposal)
                source = str(proposal_source or "planner_auxiliary_candidate")
                if time.perf_counter() >= search_deadline:
                    auxiliary_seed_reports.append({"source": source,
                                                   "status": "budget_skipped"})
                    continue
                if (has_reverse_motion(proposal) and
                        not self.config.allow_reverse):
                    errors.append({"source": source,
                                   "reason": "reverse_forbidden_auxiliary_seed"})
                    auxiliary_seed_reports.append({"source": source,
                                                   "status": "direction_rejected"})
                    continue
                violation = (None if self.config.allow_shunting else
                             action_shunting_violation(proposal))
                if violation is None:
                    candidate = evaluate_schedule(
                        proposal, source, detail={"seed_role": "auxiliary"},
                        reserve_budget=True)
                    auxiliary_seed_reports.append({"source": source,
                        "status": "feasible" if candidate is not None else "rejected"})
                    continue
                errors.append({"source": source,
                               "reason": "untrusted_seed_policy_violation",
                               "shunting_violation": violation})
                proposal_limit = min(.8, max(.1, search_deadline-
                                              time.perf_counter()))
                repaired_count = 0
                candidate = None
                for repaired, details in shunt_free_seed_proposals(
                        self.evaluator.start, proposal, self.evaluator.grid,
                        time_limit=proposal_limit):
                    repaired_count += 1
                    if time.perf_counter() >= search_deadline:
                        break
                    candidate = evaluate_schedule(
                        repaired, "auxiliary_shunt_free_seed_repair",
                        detail=dict(details, parent_source=source,
                                    preserve_mode_boundaries=True),
                        reserve_budget=True)
                    if candidate is not None:
                        break
                auxiliary_seed_reports.append({"source": source,
                    "status": "repaired_feasible" if candidate is not None else
                              "repair_rejected",
                    "repair_proposals": repaired_count})

        if initial is None:
            evaluate_auxiliary_seed_pool()
        if (not candidates and time.perf_counter() < search_deadline):
            # Recover a strict geometry path from the legacy fixed-corridor,
            # anchor, and exact-local-repair sources before spending the
            # remaining search budget.  The provider does not produce a timed
            # fallback: each result re-enters the same restricted-profile
            # retimer and independent execution audit as every other source.
            provider_budget = min(
                self.config.legacy_geometry_fallback_s,
                search_deadline-time.perf_counter())
            if provider_budget > .05:
                provider = LegacyGeometryCandidateProvider(
                    self.evaluator,
                    max_candidates=self.config.legacy_geometry_max_candidates)
                provider_inputs = [(seed, "planner_primary_seed")]
                provider_inputs.extend((actions, str(source))
                                       for actions, source in (auxiliary_seeds or []))
                geometry_candidates, legacy_geometry_report = provider.generate(
                    provider_inputs, budget_s=provider_budget)
                for candidate in geometry_candidates:
                    if (time.perf_counter() +
                            self.config.candidate_evaluation_reserve_s > work_deadline):
                        legacy_geometry_report["execution_evaluation_stopped"] = (
                            "candidate_evaluation_budget_reserve")
                        break
                    evaluated = evaluate(
                        list(candidate.actions), candidate.source,
                        parent=candidate.parent_source,
                        detail={"geometry_provider": candidate.summary(),
                                "seed_role": "legacy_geometry_fallback",
                                "preserve_certified_continuous_seed":
                                    candidate.continuous_plan is not None},
                        reserve_budget=True,
                        continuous_seed=candidate.continuous_plan,
                        geometry_variant=("legacy_geometry_provider",
                                          candidate.topology_id))
                    legacy_geometry_report.setdefault(
                        "execution_evaluations", []).append({
                            "source": candidate.source,
                            "topology_id": candidate.topology_id,
                            "status": ("FEASIBLE" if evaluated is not None else
                                       "REJECTED"),
                            "reason": None if evaluated is not None else
                            (errors[-1].get("reason") if errors else
                             "candidate_not_admitted")})
                    if evaluated is not None:
                        break
        if (self.config.continuous_motion_v2 and candidates and
                nlp_calls < nlp_call_limit):
            # A single coarse seed is still an eligible continuous incumbent.
            # Requiring a prior "regularized" initializer here silently
            # skipped the only geometry refinement on legacy-feasible routes,
            # including the long low-speed case. The optimizer enforces its
            # own variable and time budgets and the result must pass the same
            # endpoint, swept-geometry, retiming, and independent audits.
            geometry_seed = min(candidates, key=primary_value)
            refine_continuous_geometry(geometry_seed)
        if (self.config.continuous_motion_v2 and has_reverse_motion(seed) and
                not anchor_refinement_accepted):
            admitted = 0
            for proposal, details in late_reverse_proposals(
                    self.evaluator.start, seed, self.evaluator.limits.r_min_ack):
                funnel["proposals_generated"] += 1
                if (time.perf_counter() >= search_deadline or admitted >= 2 or
                        len(candidates) >= self.config.max_candidates):
                    break
                if not coarse_path_clear(
                        replay(self.evaluator.start, proposal), proposal):
                    funnel["coarse_sweep_rejected"] += 1
                    continue
                child = evaluate_schedule(proposal, "late_reverse_reorientation",
                                          initial["id"] if initial else None, details)
                if child is not None:
                    admitted += 1
        pending = [(primary_value(initial) if initial else float("inf"),
                    seed, "whole_path_seed", None, {})]
        if initial is None and not self.config.continuous_motion_v2:
            # Independent forward and reverse words enter the same feasible
            # candidate pool when the coarse seed cannot be accepted.
            p, q = self.evaluator.start, self.evaluator.goal
            starts = (forward_connections(p, q, self.evaluator.limits.r_min_ack) +
                      reverse_connections(p, q, self.evaluator.limits.r_min_ack))
            distance = math.hypot(q.x-p.x, q.y-p.y)
            if distance > 1e-8:
                bearing = math.atan2(q.y-p.y, q.x-p.x)
                starts.append([a for a in [Action(2, wrap(bearing-p.th), 0),
                               Action(1, distance, 0), Action(2, wrap(q.th-bearing), 0)]
                               if abs(a.amount) > 1e-10])
            for alternative in starts:
                if time.perf_counter() >= search_deadline:
                    break
                candidate = evaluate_schedule(alternative, "independent_direction_seed")
                priority = candidate["audit"]["score"] if candidate else float("inf")
                pending.append((priority, alternative, "whole_path_direction_seed", None, {}))
        expanded = set()
        stagnant_expansions = 0
        search_stop_reason = ("improving_anchor_refinement" if anchor_refinement_accepted
                              else "wall_budget_or_candidate_limit")
        while (not anchor_refinement_accepted and time.perf_counter() < search_deadline and
               len(candidates) < self.config.max_candidates):
            if (self.config.continuous_motion_v2 and candidates and
                    self.config.max_stagnant_expansions and
                    stagnant_expansions >= self.config.max_stagnant_expansions):
                search_stop_reason = "stagnant_expansion_limit"
                break
            if (self.config.continuous_motion_v2 and candidates and
                    self.config.max_structural_expansions and
                    len(expanded) >= self.config.max_structural_expansions):
                search_stop_reason = "structural_expansion_limit"
                break
            score_before_expansion = min((primary_value(item) for item in candidates),
                                         default=float("inf"))
            bases = sorted(candidates, key=primary_value)
            # Structural proposals depend on actions, not on the continuous
            # initialization/NLP variant. Expand each exact action sequence
            # once even when several separately audited variants share it.
            base = next((item for item in bases
                         if tuple(item["actions"]) not in expanded), None)
            if (base is None and not candidates and self.config.continuous_motion_v2
                    and None not in expanded):
                # Constant-control coarse geometry can be swept safe even when
                # its first continuous conversion is unsafe. Search its local
                # structural alternatives before abandoning that route for
                # unrelated start-to-goal connectors.
                base = {"id": None, "actions": seed}
            if base is not None:
                expanded.add(tuple(base["actions"]) if base["id"] is not None else None)
                structural_limit = min(160, max(60, self.config.max_candidates*6))
                upstream_limit = min(24, max(12, self.config.max_candidates//2))
                proposal_begin = time.perf_counter()
                proposal_actions = base["actions"]
                terminal_reanchored = False
                endpoint = replay(self.evaluator.start, proposal_actions)[-1]
                if minimum_time and (math.hypot(endpoint.x-self.evaluator.goal.x,
                        endpoint.y-self.evaluator.goal.y) > self.evaluator.xy_tol or
                        abs(wrap(endpoint.th-self.evaluator.goal.th)) > self.evaluator.yaw_tol):
                    repaired_actions = repaired_terminal_arc(self.evaluator.start,
                        self.evaluator.goal, proposal_actions, self.evaluator.limits.r_min_ack)
                    if repaired_actions is not None and coarse_path_clear(
                            replay(self.evaluator.start, repaired_actions), repaired_actions):
                        proposal_actions = repaired_actions
                        terminal_reanchored = True
                        funnel["structural_terminal_reanchors"] = funnel.get(
                            "structural_terminal_reanchors", 0)+1
                proposals = list(islice(structural_proposals(
                    self.evaluator.start, proposal_actions,
                    self.config.max_window,
                    radius=self.evaluator.limits.r_min_ack), structural_limit*8))
                if (minimum_time and any(a.mode == 2 for a in base["actions"])):
                    proposals.extend(combined_spin_replacements(
                        self.evaluator.start, proposal_actions, coarse_path_clear,
                        radius=self.evaluator.limits.r_min_ack,
                        max_window=self.config.max_window,
                        deadline=search_deadline,
                        diversify=terminal_reanchored))
                    # Include a distinct turn-placement family before the
                    # first non-improving neighborhood can stop the search.
                    route_length = sum(abs(a.amount) for a in proposal_actions
                                       if a.mode != 2)
                    proposals.extend((seq, detail) for seq, detail in
                        forward_detour_proposals(self.evaluator.start,
                            self.evaluator.goal, self.evaluator.limits.r_min_ack)
                        if sum(a.amount for a in seq) <= 1.8*route_length+.3)
                # Each window offers several connector families. Applying
                # the execution shortlist limit before cheap ranking used to
                # exhaust it on early windows, hiding safe middle/suffix
                # replacements. Scan a bounded larger bank; ranking still
                # observes search_deadline and full candidate audits keep
                # their existing quotas/reserves.
                # Offer bounded, exact A/O schedule substitutions from the
                # current certified route. A fast sweep is only a prefilter;
                # every admitted candidate still passes the full 5 mm audit.
                neighbor_limit = min(4, max(1, self.config.max_candidates//5))
                for score, actions, _, detail in mode_schedule_neighbor_candidates(
                        self.evaluator.start, proposal_actions,
                        self.evaluator.grid, max_candidates=neighbor_limit,
                        max_window=self.config.max_window,
                        sweep_step=.04,
                        require_proxy_improvement=False,
                        prefer_switch_reduction=True):
                    proposals.append((actions, dict(detail,
                        operation="mode_schedule_neighbor",
                        proposal_score=score)))
                proposals.extend(islice(upstream_heading_proposals(
                    self.evaluator.start, self.evaluator.goal,
                    proposal_actions,allow_shunting=self.config.allow_shunting,
                    grid=self.evaluator.grid), upstream_limit))
                stage_seconds["proposal_generation"] += time.perf_counter()-proposal_begin
                stage_counts["proposal_generation"] += 1
                funnel["proposals_generated"] += len(proposals)
                # Use bounded time estimates to rank this structural bank.
                # Final selection still uses exact swept safety and the full
                # independent score. Positive and reverse routes get separate
                # shortlist slots.
                ranked = []
                ranking_begin = time.perf_counter()
                for proposal_index, (actions, details) in enumerate(proposals):
                    if time.perf_counter() >= search_deadline:
                        funnel["budget_stopped"] += len(proposals)-proposal_index
                        break
                    signature = self.evaluator.signature(actions)
                    if any(key[:2] == (allowed, signature) for key in seen
                           for allowed in (False, True)):
                        funnel["duplicate"] += 1
                        continue
                    if not actions:
                        funnel["empty"] += 1
                        continue
                    if (not self.config.allow_reverse and
                            has_reverse_motion(actions)):
                        funnel["reverse_disallowed"] += 1
                        continue
                    try:
                        poses = replay(self.evaluator.start, actions)
                        end = poses[-1]
                        exact_terminal = (
                            math.hypot(end.x-self.evaluator.goal.x,
                                       end.y-self.evaluator.goal.y) <=
                                self.evaluator.xy_tol and
                            abs(wrap(end.th-self.evaluator.goal.th)) <=
                                self.evaluator.yaw_tol)
                        if not exact_terminal:
                            funnel["terminal_rejected"] += 1
                            if self.config.continuous_motion_v2:
                                continue
                        if (self.config.continuous_motion_v2 and not self.config.allow_shunting and
                                action_shunting_violation(actions) is not None):
                            continue
                        coarse_clear = coarse_path_clear(poses, actions)
                        if not coarse_clear:
                            funnel["coarse_sweep_rejected"] += 1
                            if self.config.continuous_motion_v2:
                                continue
                        # v2 never sends rejected prototypes to the constant-
                        # action NLP below. Do not time/rank that dead queue.
                        if (minimum_time and
                                details.get("operation") == "combined_spin_A_connections" and
                                self.evaluator.limits.translation_speed_policy == "mode_switch_only"):
                            # The primitive proxy restarts at every A control
                            # change. These composed bends must be ranked with
                            # the continuous retimer, which flows through A
                            # control changes and stops only at mode boundaries.
                            duration = retime_continuous(plan_from_actions(
                                self.evaluator.start, self.evaluator.goal, actions),
                                self.evaluator.limits, dt=.2, spatial_dt=.1).duration
                        else:
                            duration = duration_estimate(actions)
                        pref = reverse_preference_actions(actions,
                            max(self.evaluator.length, .25*self.evaluator.L0))
                        if not minimum_time:
                            duration += self.evaluator.T0*(
                                self.evaluator.profile.reverse_early*pref["reverse_early_length_m"]/self.evaluator.L0+
                                self.evaluator.profile.reverse_start*pref["reverse_start"]+
                                self.evaluator.profile.reverse_yaw_early*pref["reverse_yaw_early_rad"]/math.pi)
                        ranked.append((duration, coarse_clear and exact_terminal,
                                       actions, details))
                    except ValueError:
                        funnel["estimate_failed"] += 1
                        continue
                stage_seconds["proposal_ranking"] += time.perf_counter()-ranking_begin
                stage_counts["proposal_ranking"] += 1
                safe = sorted((item for item in ranked if item[1]),
                              key=lambda item:item[0])
                prototypes = sorted((item for item in ranked if not item[1]),
                    key=lambda item:(not item[3]["operation"].startswith("free_yaw"),
                                     item[0]))
                safe_forward=[item for item in safe
                              if not has_reverse_motion(item[2])]
                safe_reverse=[item for item in safe
                              if has_reverse_motion(item[2])]
                # Give one exact A/O switch-reduction neighbor an audit slot
                # before the generic duration-ranked bank. It still competes
                # under the same common score and duration guard; this only
                # prevents a useful WP5 structure from being starved by the
                # first cheap exact merge in a short search budget.
                mode_neighbors=sorted(
                    (item for item in safe_forward
                     if item[3].get("operation")=="mode_schedule_neighbor"),
                    key=lambda item:item[0])
                other_forward=[item for item in safe_forward
                               if item[3].get("operation")!="mode_schedule_neighbor"]
                shortlist=(mode_neighbors[:1]+
                           sorted(other_forward[:6]+safe_reverse[:6],
                                  key=lambda item:item[0])+prototypes[:2])
                if minimum_time:
                    shortlist = sorted(safe, key=lambda item:item[0])[:8]
                funnel["shortlisted"] += len(shortlist)
                proposal_audits = 0
                certified_compositions = set()
                for shortlist_index, (duration, coarse_clear, actions, details) in enumerate(shortlist):
                    if time.perf_counter() >= search_deadline or len(candidates) >= self.config.max_candidates:
                        funnel["budget_stopped"] += len(shortlist)-shortlist_index
                        break
                    composition_word = None
                    if details.get("operation") == "combined_spin_A_connections":
                        word = []
                        for action in actions:
                            pair = (action.mode, 0 if action.mode == 2 else
                                    (1 if action.amount >= 0 else -1))
                            if not word or pair != word[-1]:
                                word.append(pair)
                        composition_word = tuple(word)
                        # Preserve audit slots for distinct alternatives after
                        # one variant of this composed mode word is certified.
                        if composition_word in certified_compositions:
                            funnel["composition_variants_skipped"] = funnel.get(
                                "composition_variants_skipped", 0)+1
                            continue
                    if not coarse_clear:
                        # A non-exact prototype only goes to NLP if one call
                        # remains; directly auditing its known-wrong endpoint
                        # wastes the continuous safety budget.
                        if nlp_calls < nlp_call_limit:
                            pending.append((duration/self.evaluator.T0+2.,
                                actions,"whole_path_proposal",base["id"],details))
                        continue
                    child = evaluate_schedule(actions, details["operation"],
                                              base["id"], details)
                    if child is not None and composition_word is not None:
                        certified_compositions.add(composition_word)
                    proposal_audits += 1
                    # Refinement can still change all intermediate poses.
                    priority = primary_value(child) if child else duration/self.evaluator.T0+2.
                    pending.append((priority, actions, "whole_path_proposal", base["id"], details))
                    # Audit only the top exact proposal before giving its
                    # distinct topology a chance in the NLP. More proposals
                    # are revisited through the next incumbent if budget allows.
                    audit_limit = (3 if minimum_time and
                        any(a.mode == 2 for a in base["actions"]) else 1)
                    if (proposal_audits >= audit_limit and candidates) or proposal_audits >= 3:
                        funnel["budget_stopped"] += len(shortlist)-shortlist_index-1
                        break
            nlp_budget = (work_deadline-time.perf_counter()-
                          self.config.candidate_evaluation_reserve_s)
            if (not self.config.continuous_motion_v2 and pending and
                    nlp_calls < nlp_call_limit and nlp_budget > .15):
                pending.sort(key=lambda item: item[0])
                def mode_word(item):
                    seq = item[1]
                    return tuple(a.mode for i,a in enumerate(seq) if i == 0 or a.mode != seq[i-1].mode)
                index = next((i for i,item in enumerate(pending)
                              if mode_word(item) not in nlp_modes_seen), 0)
                selected = pending.pop(index)
                nlp_modes_seen.add(mode_word(selected))
                _, actions, source, parent, details = selected
                nlp_calls += 1
                funnel["nlp_attempted"] += 1
                nlp_begin = time.perf_counter()
                try:
                    refined, meta = optimize_geometry(self.evaluator.start, self.evaluator.goal,
                                                       actions, self.evaluator.grid, self.evaluator,
                                                       min(self.config.nlp_time_s, nlp_budget),
                                                       allow_reverse=self.config.allow_reverse)
                    stage_seconds["nlp_solve"] += time.perf_counter()-nlp_begin
                    stage_counts["nlp_solve"] += 1
                    # The NLP call is admitted only after reserving time for
                    # this full candidate evaluation and the final incumbent
                    # audit. Do not reserve a second candidate slot here.
                    evaluate(refined, source, parent, dict(details, **meta),
                             reserve_budget=False)
                except (ValueError, RuntimeError) as exc:
                    stage_seconds["nlp_solve"] += time.perf_counter()-nlp_begin
                    stage_counts["nlp_solve"] += 1
                    errors.append({"source": source, "reason": str(exc).splitlines()[-1]})
            elif base is None:
                search_stop_reason = "distinct_action_frontier_exhausted"
                break
            if self.config.continuous_motion_v2 and candidates:
                best_after = min(primary_value(item) for item in candidates)
                improvement_tolerance = max(1e-9,abs(best_after)*1e-7)
                if minimum_time:
                    improvement_tolerance = max(improvement_tolerance,
                        self.config.min_time_gain_to_expand_s/self.evaluator.T0)
                if best_after < score_before_expansion-improvement_tolerance:
                    stagnant_expansions = 0
                else:
                    stagnant_expansions += 1
        corner_refinement = []
        if (minimum_time and self.config.continuous_motion_v2 and candidates and
                self.evaluator.limits.translation_speed_policy == "mode_switch_only"):
            # The generic frontier is deliberately small. Give remaining
            # interior bends a separate bounded family after that frontier,
            # without treating a legal terminal Spin as a defect or stopping
            # a certified incumbent from remaining eligible.
            def interior_spins(actions):
                last_move = max((i for i, a in enumerate(actions) if a.mode != 2), default=-1)
                return sum(a.mode == 2 for a in actions[:last_move])
            for refinement_index in range(4):
                if time.perf_counter() >= search_deadline or len(candidates) >= self.config.max_candidates:
                    break
                base = min(candidates, key=primary_value)
                before_spins = interior_spins(base["actions"])
                if not before_spins:
                    break
                refinement_begin = time.perf_counter()
                proposal_actions = base["actions"]
                endpoint = replay(self.evaluator.start, proposal_actions)[-1]
                if math.hypot(endpoint.x-self.evaluator.goal.x,
                              endpoint.y-self.evaluator.goal.y) > self.evaluator.xy_tol:
                    repaired = repaired_terminal_arc(self.evaluator.start,
                        self.evaluator.goal, proposal_actions, self.evaluator.limits.r_min_ack)
                    if repaired is not None and coarse_path_clear(
                            replay(self.evaluator.start, repaired), repaired):
                        proposal_actions = repaired
                proposals = list(combined_spin_replacements(self.evaluator.start,
                    proposal_actions, corner_path_clear,
                    radius=self.evaluator.limits.r_min_ack,
                    max_window=max(12, self.config.max_window),
                    deadline=search_deadline, diversify=True))
                stage_seconds["proposal_generation"] += time.perf_counter()-refinement_begin
                stage_counts["proposal_generation"] += 1
                funnel["proposals_generated"] += len(proposals)
                ranked = []
                ranking_begin = time.perf_counter()
                for actions, details in proposals:
                    if time.perf_counter() >= search_deadline:
                        break
                    signature = self.evaluator.signature(actions)
                    if any(key[:2] == (allowed, signature) for key in seen
                           for allowed in (False, True)):
                        continue
                    if not coarse_path_clear(replay(self.evaluator.start, actions), actions):
                        continue
                    try:
                        duration = retime_continuous(plan_from_actions(
                            self.evaluator.start, self.evaluator.goal, actions),
                            self.evaluator.limits, dt=.2, spatial_dt=.1).duration
                        ranked.append((duration, actions, details))
                    except ValueError:
                        continue
                stage_seconds["proposal_ranking"] += time.perf_counter()-ranking_begin
                stage_counts["proposal_ranking"] += 1
                improved = False
                attempted = 0
                for duration, actions, details in sorted(ranked, key=lambda item: item[0]):
                    if time.perf_counter() >= search_deadline or len(candidates) >= self.config.max_candidates:
                        break
                    child = evaluate_schedule(actions, "combined_spin_A_connections",
                        base["id"], dict(details, corner_refinement_pass=refinement_index+1))
                    attempted += 1
                    if (child is not None and primary_value(child) < primary_value(base) and
                            interior_spins(child["actions"]) < before_spins):
                        improved = True
                        break
                    if attempted >= 2:
                        break
                after = min(candidates, key=primary_value)
                corner_refinement.append({"parent": base["id"],
                    "before_interior_spins": before_spins,
                    "after_interior_spins": interior_spins(after["actions"]),
                    "before_duration_s": base["timed"].duration,
                    "after_duration_s": after["timed"].duration,
                    "audits_attempted": attempted, "improved": improved})
                if not improved:
                    break
        if self.config.continuous_motion_v2 and not candidates:
            starts = forward_connections(self.evaluator.start, self.evaluator.goal,
                                          self.evaluator.limits.r_min_ack)
            if self.config.allow_reverse:
                starts += reverse_connections(self.evaluator.start, self.evaluator.goal,
                                              self.evaluator.limits.r_min_ack)
            for alternative in starts:
                if time.perf_counter() >= search_deadline:
                    break
                evaluate_schedule(alternative, "independent_direction_seed")
        forward_candidate_count = sum(not has_reverse_motion(item["actions"])
                                      for item in candidates)
        reverse_fallback_activated = not candidates and self.config.allow_reverse
        if reverse_fallback_activated:
            # Last-resort structural recovery for an empty feasible pool.
            # Reverse motion already participates in the normal candidate
            # search; this branch has no forward-only acceptance rule.
            recovered = evaluate(seed, "empty_pool_seed_recovery", reverse_allowed=True)
            # Cheap swept screening prevents impossible in-place rotations
            # near the dead end from spending final-evaluation/NLP budgets.
            if recovered:
                ranked = []
                bank_seen = set()
                for proposal, detail in upstream_heading_proposals(
                        self.evaluator.start, self.evaluator.goal, seed):
                    if time.perf_counter() >= search_deadline:
                        break
                    signature = self.evaluator.signature(proposal)
                    if signature in bank_seen:
                        continue
                    bank_seen.add(signature)
                    if not self.evaluator.grid.trajectory_clear(
                            replay(self.evaluator.start, proposal), proposal, .02):
                        continue
                    try:
                        duration = retime(self.evaluator.start, proposal, self.evaluator.limits,
                                          .5, allow_reverse=True, build_samples=False).duration
                        reverse = sum(-a.amount for a in proposal if a.mode != 2 and a.amount < 0)
                        ranked.append((duration/self.evaluator.T0+
                            self.evaluator.profile.reverse*reverse/self.evaluator.L0, proposal, detail))
                    except ValueError:
                        continue
                for _, proposal, detail in sorted(ranked, key=lambda item: item[0])[:6]:
                    if time.perf_counter() >= search_deadline or len(candidates) >= self.config.max_candidates:
                        break
                    evaluate(proposal, "empty_pool_upstream", recovered["id"], detail, reverse_allowed=True)
                recovered = min(candidates, key=primary_value)
            if (recovered and not self.config.continuous_motion_v2 and
                    nlp_calls < self.config.max_nlp_calls and
                    search_deadline-time.perf_counter() > .15):
                try:
                    nlp_calls += 1
                    funnel["nlp_attempted"] += 1
                    nlp_begin = time.perf_counter()
                    refined, meta = optimize_geometry(self.evaluator.start, self.evaluator.goal,
                        recovered["actions"], self.evaluator.grid, self.evaluator,
                        min(self.config.nlp_time_s, search_deadline-time.perf_counter()), allow_reverse=True)
                    stage_seconds["nlp_solve"] += time.perf_counter()-nlp_begin
                    stage_counts["nlp_solve"] += 1
                    evaluate(refined, "empty_pool_nlp", recovered["id"], meta,
                             reverse_allowed=True, reserve_budget=False)
                except (ValueError, RuntimeError) as exc:
                    stage_seconds["nlp_solve"] += time.perf_counter()-nlp_begin
                    stage_counts["nlp_solve"] += 1
                    errors.append({"source": "empty_pool_nlp", "reason": str(exc).splitlines()[-1]})
            if recovered:
                for proposal, detail in structural_proposals(self.evaluator.start, seed,
                        self.config.max_window, radius=self.evaluator.limits.r_min_ack):
                    if time.perf_counter() >= search_deadline or len(candidates) >= self.config.max_candidates:
                        break
                    evaluate(proposal, "empty_pool_structure", recovered["id"], detail, reverse_allowed=True)
        if initial is not None and not anchor_refinement_accepted:
            evaluate_auxiliary_seed_pool()
        if not candidates:
            geometry_report = geometry_store.report()
            has_geometry = geometry_report["geometry_incumbent"] is not None
            raise ModeMasterFailure({
                "backend": "mode_path_v2" if self.config.continuous_motion_v2 else "mode_path_v1",
                "status": "GEOMETRY_ONLY" if has_geometry else "REJECTED",
                "reason": ("geometry_feasible_execution_unavailable" if has_geometry else
                           "no_feasible_candidate_in_search_budget"),
                "geometry_status": "PASS" if has_geometry else "NOT_CERTIFIED",
                "execution_status": "NOT_CERTIFIED",
                **geometry_report,
                "infeasibility_proof": False, "rejections": errors,
                "stage_timings_s": stage_seconds, "stage_call_counts": stage_counts,
                "proposal_funnel": funnel, "coarse_sweep_cache": coarse_sweep_counts,
                "continuous_optimizer": continuous_optimizer_report,
                "total_s": time.perf_counter()-begin, "first_feasible_s": first_feasible,
                "budget_s": effective_budget_s,
                "configured_budget_s": self.config.budget_s,
                "adaptive_budget_extension_s": adaptive_budget_extension_s,
                "adaptive_budget_trigger": adaptive_budget_trigger,
                "budget_scope": "optimizer_backend_only",
                "budget_exceeded": time.perf_counter() > deadline,
                "nlp_calls": nlp_calls, "candidates": [], "selected_id": None,
                "legacy_geometry_provider": legacy_geometry_report})
        duration_eligible, duration_guard = duration_guard_candidates(
            candidates, self.config.max_duration_regression)
        if minimum_time:
            fastest = min(float(item["timed"].duration) for item in candidates)
            time_tolerance = max(1e-7, 1e-8*fastest)
            duration_eligible = [item for item in candidates
                if float(item["timed"].duration) <= fastest+time_tolerance]
            duration_guard.update(selection_objective="minimum_time",
                duration_cap_s=fastest+time_tolerance,
                max_regression_fraction=0.,
                eligible_candidate_ids=[item["id"] for item in duration_eligible],
                excluded_candidate_ids=[item["id"] for item in candidates
                    if float(item["timed"].duration) > fastest+time_tolerance])
            for item in duration_eligible:
                if item["audit"]["score"] is not None:
                    continue
                scoring_begin = time.perf_counter()
                audit = self.evaluator.evaluate_continuous_geometry(
                    item["continuous_plan"], item["timed"],
                    allow_reverse=self.config.allow_reverse)
                stage_seconds["final_scoring"] += time.perf_counter()-scoring_begin
                stage_counts["final_scoring"] += 1
                if audit["status"] != "FEASIBLE":
                    raise ValueError("final_quality_audit_failed:"+str(audit.get("reason")))
                item["audit"] = audit
        best_score = min(item["audit"]["score"] for item in duration_eligible)
        tie_tolerance = max(1e-9, 1e-7*abs(best_score))
        equivalent = [item for item in duration_eligible
                      if item["audit"]["score"] <= best_score+tie_tolerance]
        def oblique_length(item):
            if item.get("continuous_plan") is not None:
                return sum(b.length for b in item["continuous_plan"].blocks if b.mode == 1)
            return sum(abs(a.amount) for a in item["actions"] if a.mode == 1)
        # Degenerate straight A/O motion is mechanically identical. Numerical
        # roundoff must not change its mode label; meaningful O gains compete
        # normally through the common score above.
        winner = min(equivalent, key=lambda item: (
                     item["audit"]["metrics"]["oblique_adjustments"],
                     item["audit"]["metrics"]["mode_switches"], oblique_length(item),
                     item["audit"]["score"]))
        reverse_candidate_count = sum(has_reverse_motion(item["actions"])
                                      for item in candidates)
        rejection_reason_counts = {}
        for error in errors:
            reason = str(error.get("reason", "unknown"))
            rejection_reason_counts[reason] = rejection_reason_counts.get(reason, 0)+1
        report = {"backend": ("mode_path_v2" if self.config.continuous_motion_v2 else
                              "mode_path_v1"), "first_feasible_s": first_feasible,
                  "selection_objective": self.config.selection_objective,
                  "selection_scope": "fully_certified_candidates_in_this_request",
                  "quality_scores_deferred": sum(item["audit"].get("score_status") ==
                      "DEFERRED" for item in candidates),
                  "min_time_gain_to_expand_s": self.config.min_time_gain_to_expand_s,
                  "direction_policy": "bidirectional_joint_candidate_competition",
                  "allow_reverse": self.config.allow_reverse,
                  "allow_reverse_fallback": self.config.allow_reverse,
                  "forward_candidate_count": forward_candidate_count,
                  "reverse_candidate_count": reverse_candidate_count,
                  "empty_pool_recovery_activated": reverse_fallback_activated,
                  "selected_has_reverse": has_reverse_motion(winner["actions"]),
                  # Legacy report keys retained for old probe readers.
                  "reverse_fallback_activated": reverse_fallback_activated,
                  "reverse_fallback_used": has_reverse_motion(winner["actions"]),
                  "forward_failure_is_infeasibility_proof": False,
                  "total_s": time.perf_counter()-begin, "nlp_calls": nlp_calls,
                  "stage_timings_s": stage_seconds,
                  "stage_call_counts": stage_counts,
                  "proposal_funnel": funnel,
                  "mode_anchor_reports": mode_anchor_reports,
                  "coarse_sweep_cache": coarse_sweep_counts,
                  "continuous_sweep_cache": dict(self.evaluator.sweep_stats),
                  "continuous_sample_cache": dict(self.evaluator.sample_stats),
                  "structural_action_parents_expanded": len(expanded),
                  "corner_refinement": corner_refinement,
                  "structural_expansion_limit": self.config.max_structural_expansions,
                  "search_stop_reason": search_stop_reason,
                  "stagnant_structural_expansions": stagnant_expansions,
                  "primitive_timing_cache": dict(duration_estimate.stats),
                  "duplicate_continuous_nlp_skipped": continuous_nlp_reused,
                  "anchor_refinement_early_accepted": anchor_refinement_accepted,
                  "strict_auxiliary_first_used": strict_auxiliary_first_used,
                  "rejection_reason_counts": rejection_reason_counts,
                  "continuous_optimizer": continuous_optimizer_report,
                  "legacy_geometry_provider": legacy_geometry_report,
                  "duration_guard": duration_guard,
                  "candidates": [{k: v for k, v in item.items() if k not in ("timed", "actions")}
                                 for item in candidates], "rejections": errors,
                  "selected_id": winner["id"], "selected_source": winner["source"],
                  "score_tie_tolerance": tie_tolerance,
                  "selected": winner["audit"],
                  "selected_timing": winner.get("timing_diagnostics"),
                  "budget_s": effective_budget_s,
                  "configured_budget_s": self.config.budget_s,
                  "adaptive_budget_extension_s": adaptive_budget_extension_s,
                  "adaptive_budget_trigger": adaptive_budget_trigger,
                  "mode_anchor_base_switches": initial_mode_switches,
                  "candidate_evaluation_reserve_s": self.config.candidate_evaluation_reserve_s,
                  "budget_scope": "optimizer_backend_only",
                  "budget_exceeded": time.perf_counter() > deadline,
                  "initial_is_single_cpp_seed": not bool(auxiliary_seeds),
                  "auxiliary_seed_reports": auxiliary_seed_reports,
                  "motion_representation": ("continuous_motion_v2" if
                      self.config.continuous_motion_v2 else "exact_primitive_quintic_v1"),
                  "region_graph_implemented": False,
                  "continuous_curvature_implemented": self.config.continuous_motion_v2}
        report["candidates"] = [
            {k: v for k, v in item.items()
             if k not in ("timed", "actions", "continuous_plan")}
            for item in candidates]
        report.update(geometry_store.report())
        return winner, report
