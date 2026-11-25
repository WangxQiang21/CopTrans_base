"""Propose formation routes that cruise in A and avoid fragile oblique motion.

Proposals use the normal edge factories and cheap static previews. The caller
must independently certify the complete swept path and common timing before use.
"""

from dataclasses import dataclass
from heapq import nsmallest
import math

from .formation import FormationPose
from .uniform_motion import (
    MODE_O, MODE_S, PRIMITIVE_A_ARC, UniformMotionError,
    a_arc_edge, a_arc_required_headings, a_straight_edge, o_translation_edge, s_spin_edge,
)
from .uniform_search import _o_branch, _wrap
from .uniform_timing import timing_motion_limits


_ANGULAR_TOL = 1e-9
_O_MARGIN_RAD = math.radians(12.)
_ALIGNED_O_SPREAD_RAD = math.radians(25.)
_SHORT_OBLIQUE_ALLOWANCE_M = 3.
_ENDPOINT_SPIN_ALLOWANCE_M = 2.


def route_quality(plan):
    """Score a certified plan in equivalent seconds and report the trade-offs."""
    switches = 0
    mid_route_spins = 0
    oblique_distance = 0.
    near_longitudinal_distance = 0.
    oblique_spread = 0.
    spread_cost = 0.
    block_spread = 0.
    block_distance = 0.
    block_near_longitudinal = 0.
    cruise_oblique = 0.
    cruise_near_longitudinal = 0.

    def finish_oblique_block():
        nonlocal spread_cost, block_spread, block_distance, block_near_longitudinal
        nonlocal cruise_oblique, cruise_near_longitudinal
        # Preserve short lateral positioning when an added stationary S/A
        # manoeuvre would cost more time than it saves.
        excess = max(0., block_distance - _SHORT_OBLIQUE_ALLOWANCE_M)
        cruise_oblique += excess
        cruise_near_longitudinal += block_near_longitudinal * min(
            1., excess / _SHORT_OBLIQUE_ALLOWANCE_M)
        spread_cost += min(1., block_spread / _ALIGNED_O_SPREAD_RAD)
        block_spread = block_distance = block_near_longitudinal = 0.

    previous = None
    for phase in plan.phases:
        if phase.kind != "motion":
            continue
        switches += previous is not None and previous != phase.mode
        previous = phase.mode
        if phase.mode == MODE_S:
            pose = phase.edge.start_qf
            if all(math.hypot(pose.x - endpoint.x, pose.y - endpoint.y) > _ENDPOINT_SPIN_ALLOWANCE_M
                   for endpoint in (plan.start_qf, plan.goal_qf)):
                mid_route_spins += 1
        if phase.mode != MODE_O:
            finish_oblique_block()
            continue
        edge = phase.edge
        distance = edge.control.parameter_length
        oblique_distance += distance
        course = math.atan2(edge.end_qf.y - edge.start_qf.y, edge.end_qf.x - edge.start_qf.x)
        headings = [yaw for _, yaw in edge.start_headings]
        spread = max(abs(_wrap(a - b)) for a in headings for b in headings)
        lateral_angle = min(abs(math.remainder(course - yaw, math.pi)) for yaw in headings)
        near_longitudinal = distance * max(0., 1. - lateral_angle / _O_MARGIN_RAD)
        near_longitudinal_distance += near_longitudinal
        block_distance += distance
        block_near_longitudinal += near_longitudinal
        block_spread = max(block_spread, spread)
        oblique_spread = max(oblique_spread, spread)
    finish_oblique_block()
    return {
        "duration_s": plan.duration_s,
        "mode_switches": switches,
        "mid_route_spins": mid_route_spins,
        "oblique_distance_m": oblique_distance,
        "near_longitudinal_oblique_m": near_longitudinal_distance,
        "oblique_heading_spread_deg": math.degrees(oblique_spread),
        "score_s": (plan.duration_s + 2.5 * switches + 6.0 * mid_route_spins
                    + .35 * cruise_oblique + .75 * cruise_near_longitudinal + 8.0 * spread_cost),
    }


@dataclass(frozen=True)
class TranslationLeg:
    target: FormationPose
    course: float
    distance: float
    original: tuple


def _translation_legs(edges):
    """Join same-direction neighbouring O primitives before proposing cruise legs."""
    legs = []
    for edge in edges:
        if edge.mode != MODE_O:
            legs.append(edge)
            continue
        target = edge.end_qf
        course = math.atan2(target.y - edge.start_qf.y, target.x - edge.start_qf.x)
        length = edge.control.parameter_length
        if (legs and isinstance(legs[-1], TranslationLeg)
                and abs(_wrap(course - legs[-1].course)) <= _ANGULAR_TOL):
            previous = legs.pop()
            length += previous.distance
            legs.append(TranslationLeg(target, course, length, previous.original + (edge,)))
        else:
            legs.append(TranslationLeg(target, course, length, (edge,)))
    return legs


class RouteProposals:
    def __init__(self, request, baseline, checker, expired):
        self.request = request
        self.baseline = baseline
        self.checker = checker
        self.expired = expired
        self.model = request.model
        self.limits = timing_motion_limits(request.successor_config.timing_limits)
        self.reverse = request.successor_config.allow_reverse
        self.step = request.successor_config.translation_step_m
        self.legs = _translation_legs(baseline.edges) if baseline is not None else ()

    def _align(self, qf, headings, target):
        deltas = {vehicle: _wrap(target[vehicle] - theta) for vehicle, theta in headings.items()}
        if max(map(abs, deltas.values())) <= _ANGULAR_TOL:
            return (), headings
        spin = s_spin_edge(self.model, qf, headings, deltas, self.limits)
        return (spin,), dict(spin.end_headings)

    def _straight(self, qf, headings, target, reverse):
        vector = target.x - qf.x, target.y - qf.y
        course = math.atan2(vector[1], vector[0]) + (math.pi if reverse else 0.)
        prep, aligned = self._align(qf, headings, dict.fromkeys(headings, course))
        edge = a_straight_edge(self.model, qf, aligned, vector,
                               dict.fromkeys(headings, -1 if reverse else 1),
                               self.limits, self.reverse)
        return prep + (edge,)

    def _straight_choices(self, qf, headings, target):
        choices = [self._straight(qf, headings, target, False)]
        if self.reverse:
            choices.append(self._straight(qf, headings, target, True))
        return sorted(choices, key=lambda edges: sum(
            edge.control.parameter_length for edge in edges if edge.mode == MODE_S))

    def _arc(self, qf, headings, pivot, turn):
        tangents = a_arc_required_headings(
            self.model, qf, turn, pivot, dict.fromkeys(headings, 1), self.limits)
        directions = {i: -1 if self.reverse and abs(_wrap(tangents[i] - theta)) > math.pi / 2.
                      else 1 for i, theta in headings.items()}
        target = {i: tangents[i] + (math.pi if directions[i] < 0 else 0.)
                  for i in headings}
        prep, aligned = self._align(qf, headings, target)
        edge = a_arc_edge(self.model, qf, aligned, turn, pivot, directions,
                          self.limits, self.reverse)
        return prep + (edge,)

    def _oblique(self, qf, headings, leg):
        vector = leg.target.x - qf.x, leg.target.y - qf.y
        course = math.atan2(vector[1], vector[0])
        branches = {i: _o_branch(theta, course, self.reverse) for i, theta in headings.items()}
        if any(branch is None for branch in branches.values()):
            return ()
        return (o_translation_edge(
            self.model, qf, headings, vector, {i: branch[0] for i, branch in branches.items()},
            {i: branch[1] for i, branch in branches.items()}, self.limits, self.reverse),)

    def _preview(self, edges):
        """Reject clear failures early. This coarse test is not a certificate."""
        for edge in edges:
            length = edge.control.parameter_length
            spacing = math.radians(12.) if edge.mode == MODE_S or edge.primitive == PRIMITIVE_A_ARC else .45
            count = max(2, math.ceil(length / spacing))
            for index in range(count + 1):
                if self.expired():
                    return False
                state = edge.state_at(length * (index / count))
                if not self.checker.static_check(state.qf, dict(state.headings)).success:
                    return False
        return True

    def _common_oblique(self, qf, headings, legs):
        angles = [qf.psi, *headings.values(),
                  *(index * math.pi / 8. for index in range(16))]

        def score(yaw):
            turn = max(abs(_wrap(yaw - theta)) for theta in headings.values())
            lateral = min(abs(math.remainder(leg.course - yaw, math.pi)) for leg in legs)
            return turn + 4. * max(0., _O_MARGIN_RAD - lateral)

        for yaw in sorted(angles, key=score)[:6]:
            edges, aligned = self._align(qf, headings, dict.fromkeys(headings, yaw))
            current = qf
            for leg in legs:
                motion = self._oblique(current, aligned, leg)
                if not motion:
                    break
                edges += motion
                current = leg.target
            else:
                if self._preview(edges):
                    return edges
        return ()

    def _rewrite(self, take_ackermann, common_yaw):
        qf, headings = self.baseline.start_qf, dict(self.baseline.start_headings)
        result = ()
        index = 0
        while index < len(self.legs) and not self.expired():
            leg = self.legs[index]
            if not isinstance(leg, TranslationLeg):
                if leg.mode == MODE_S:
                    edges, _ = self._align(qf, headings, dict(leg.end_headings))
                else:
                    prep, _ = self._align(qf, headings, dict(leg.start_headings))
                    edges = prep + (leg,)
                index += 1
                qf, terminal = leg.end_qf, leg.end_headings
            else:
                edges = ()
                if take_ackermann(leg):
                    edges = next((option for option in self._straight_choices(qf, headings, leg.target)
                                  if self._preview(option)), ())
                if edges:
                    index += 1
                else:
                    end = index + 1
                    while (end < len(self.legs) and isinstance(self.legs[end], TranslationLeg)
                           and not take_ackermann(self.legs[end])):
                        end += 1
                    if common_yaw:
                        edges = self._common_oblique(qf, headings, self.legs[index:end])
                    if edges:
                        index = end
                    else:
                        original = dict(leg.original[0].start_headings)
                        prep, aligned = self._align(qf, headings, original)
                        edges = prep + (self._oblique(qf, aligned, leg) or leg.original)
                        index += 1
                qf, terminal = edges[-1].end_qf, edges[-1].end_headings
            result += edges
            headings = dict(terminal)
        if index != len(self.legs):
            return ()
        if self.request.goal_headings is not None and not self.request.successor_config.goal_heading_free:
            prep, _ = self._align(qf, headings, dict(self.request.goal_headings))
            result += prep
        return result

    @staticmethod
    def _arc_displacement(start_yaw, goal_yaw, pivot):
        px, py = pivot
        dc = math.cos(start_yaw) - math.cos(goal_yaw)
        ds = math.sin(start_yaw) - math.sin(goal_yaw)
        return dc * px - ds * py, ds * px + dc * py

    def arc_straight_arc(self):
        start = self.request.start_qf
        goal = self.request.goal_qf if self.baseline is None else self.baseline.goal_qf
        direct = math.atan2(goal.y - start.y, goal.x - start.x)
        span = math.hypot(goal.x - start.x, goal.y - start.y)
        if span < 5. * self.step:
            return
        axis = round(direct / (math.pi / 2.)) * (math.pi / 2.)
        headings = dict(self.request.start_headings)
        extent = max(math.hypot(offset.x, offset.y) for offset in self.model.offsets) + 2. * self.step
        spacing = extent / 12.
        pivots = [(x * spacing, y * spacing) for x in range(-12, 13) for y in range(-12, 13)]
        for bearing in dict.fromkeys((axis, direct, direct + math.pi / 12., direct - math.pi / 12.)):
            first_turn, last_turn = _wrap(bearing - start.psi), _wrap(goal.psi - bearing)
            if min(abs(first_turn), abs(last_turn)) <= math.radians(2.):
                continue
            starts = []
            ends = []
            for pivot in pivots:
                if self.expired():
                    return
                try:
                    edges = self._arc(start, headings, pivot, first_turn)
                    if self._preview(edges):
                        starts.append((edges, math.hypot(*pivot) * abs(first_turn)))
                except UniformMotionError:
                    pass
                dx, dy = self._arc_displacement(bearing, goal.psi, pivot)
                entry = FormationPose(goal.x - dx, goal.y - dy, bearing)
                try:
                    arc = self._arc(entry, headings, pivot, last_turn)[-1]
                    if self._preview((arc,)):
                        ends.append((pivot, entry, math.hypot(*pivot) * abs(last_turn)))
                except UniformMotionError:
                    pass
            lateral_x, lateral_y = -math.sin(bearing), math.cos(bearing)

            def pairs():
                for prefix, first_length in starts:
                    exit_pose = prefix[-1].end_qf
                    for pivot, entry, last_length in ends:
                        dx, dy = entry.x - exit_pose.x, entry.y - exit_pose.y
                        distance = math.hypot(dx, dy)
                        if distance >= .5 * span and first_length + distance + last_length <= 1.6 * span:
                            weight = (first_length + distance + last_length
                                      + 5. * abs(lateral_x * dx + lateral_y * dy))
                            yield weight, prefix, pivot, entry
                    if self.expired():
                        return

            yielded = 0
            for _, prefix, pivot, entry in nsmallest(640, pairs(), key=lambda pair: pair[0]):
                if self.expired():
                    return
                previous = prefix[-1]
                origin = previous.end_qf
                dx, dy = entry.x - origin.x, entry.y - origin.y
                cruise_headings = dict.fromkeys(headings, math.atan2(dy, dx))
                if any(not self.checker.static_check(FormationPose(
                        origin.x + f * dx, origin.y + f * dy, bearing), cruise_headings).success
                       for f in (.50, .25, .75)):
                    continue
                for cruise in self._straight_choices(previous.end_qf, dict(previous.end_headings), entry):
                    if not self._preview(cruise):
                        continue
                    after = cruise[-1]
                    try:
                        suffix = self._arc(after.end_qf, dict(after.end_headings), pivot, last_turn)
                    except UniformMotionError:
                        continue
                    if not self._preview(suffix):
                        continue
                    if (self.request.goal_headings is not None
                            and not self.request.successor_config.goal_heading_free):
                        after = suffix[-1]
                        prep, _ = self._align(after.end_qf, dict(after.end_headings),
                                              dict(self.request.goal_headings))
                        suffix += prep
                    yield prefix + cruise + suffix
                    yielded += 1
                    break
                if yielded >= 3:
                    return

    def candidates(self):
        moving = [leg for leg in self.legs if isinstance(leg, TranslationLeg)]
        if not moving:
            return
        longest = sorted(moving, key=lambda leg: leg.distance, reverse=True)
        for name, predicate, common_yaw in (
            ("ackermann_long_legs_aligned_oblique", lambda leg: leg.distance >= 2. * self.step, True),
            ("ackermann_long_legs", lambda leg: leg.distance >= 2. * self.step, False),
            ("ackermann_longest_leg", lambda leg: leg is longest[0], True),
            ("common_oblique_heading", lambda leg: False, True),
        ):
            if self.expired():
                return
            try:
                edges = self._rewrite(predicate, common_yaw)
            except UniformMotionError:
                continue
            if edges:
                yield name, edges
