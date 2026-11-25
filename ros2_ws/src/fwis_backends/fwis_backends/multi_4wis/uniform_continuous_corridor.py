"""Propose continuous Ackermann paths through orthogonal travel corridors.

The preview tests the cargo and three steering body footprints at sampled
stations. Callers must still independently audit the entire swept path and
its common time law before dispatch.
"""

from functools import lru_cache
from itertools import groupby, islice, product
import math
import time

import numpy as np

from .continuous_geometry import ControlKnot
from .formation import FormationPose
from .formation_continuous_a.geometry import C2CurvatureProfile


# Each end ramp integrates a quadratic B-spline. The resulting cubic pieces
# meet a flat curvature plateau, and meet straight travel with k, k' and k''
# all zero. Long ramps limit curvature change at the vehicle mounts.
_RAMP_FRACTION = .41
_GAUSS_X, _GAUSS_W = np.polynomial.legendre.leggauss(6)


def _rotation(x, y, angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return cosine * x - sine * y, sine * x + cosine * y


def _bend_knots(angle, length, ramp_in=_RAMP_FRACTION, ramp_out=_RAMP_FRACTION):
    entry_cell, exit_cell = ramp_in * length / 3., ramp_out * length / 3.
    peak = angle / ((1. - (ramp_in + ramp_out) / 2.) * length)
    ramps = ((0., 0.), (1. / 6., .5), (5. / 6., .5), (1., 0.))
    entering = tuple(ControlKnot(index * entry_cell, peak * kappa, peak * eta / entry_cell)
                     for index, (kappa, eta) in enumerate(ramps))
    leaving = tuple(ControlKnot(length - index * exit_cell, peak * kappa, -peak * eta / exit_cell)
                    for index, (kappa, eta) in reversed(tuple(enumerate(ramps))))
    return entering + leaving


@lru_cache(maxsize=192)
def _unit_bend(angle, *ramps):
    knots = _bend_knots(angle, 1., *ramps)
    profile = C2CurvatureProfile.from_knots(knots)
    # Include each polynomial boundary, then integrate the local heading
    # on short intervals for the geometric preview and endpoint solve.
    stations = sorted(set(np.linspace(0., 1., 51)) | {knot.s for knot in knots})
    x = y = 0.
    result = [(x, y, 0., 0., 0.)]
    for left, right in zip(stations, stations[1:]):
        half_step = (right - left) / 2.
        nodes = (left + right) / 2. + half_step * _GAUSS_X
        yaws = np.asarray([profile.psi_at(float(node)) for node in nodes])
        x += half_step * float(np.dot(_GAUSS_W, np.cos(yaws)))
        y += half_step * float(np.dot(_GAUSS_W, np.sin(yaws)))
        jet = profile.jet_at(float(right))
        result.append((x, y, profile.psi_at(float(right)), jet.kappa, jet.eta))
    return tuple(result)


def _bend_displacement(heading, angle, length=1., *ramps):
    x, y, *_ = _unit_bend(angle, *ramps)[-1]
    return _rotation(length * x, length * y, heading)


def _mount_tangent(mount, curvature, slope, curvature_limit):
    normal = 1. - mount.y * curvature
    if normal <= 0.:
        return None
    side = mount.x * curvature
    factor = math.hypot(normal, side)
    body_curvature = (curvature + mount.x * slope / factor**2) / factor
    return math.atan2(side, normal) if abs(body_curvature) < curvature_limit else None


def _compact_turn_length(angle, ramps, model, radius):
    # This is only a sampled starting length for the template search.
    # The continuous certificate independently checks every mount.
    stations = _unit_bend(angle, *ramps)
    limit = 1. / radius - .001
    low = 0.
    high = 6. * (radius + max(math.hypot(mount.x, mount.y) for mount in model.offsets))
    for _ in range(24):
        length = (low + high) / 2.
        if all(_mount_tangent(mount, curvature / length, slope / length**2, limit) is not None
               for *_, curvature, slope in stations for mount in model.offsets):
            high = length
        else:
            low = length
    return high


def _preview(request, checker, pose, commands, expired):
    curvature_limit = 1. / request.successor_config.timing_limits.r_min_ackermann
    for turn, length, *ramps in commands:
        if length <= 0. or expired():
            return None
        if turn:
            samples = ((x * length, y * length, angle, kappa / length, eta / length**2)
                       for x, y, angle, kappa, eta in _unit_bend(turn, *ramps))
        else:
            samples = ((float(x), 0., 0., 0., 0.)
                       for x in np.linspace(0., length, 1 + math.ceil(length / .35)))
        for x, y, angle, kappa, eta in samples:
            if expired():
                return None
            dx, dy = _rotation(x, y, pose.psi)
            reference = FormationPose(pose.x + dx, pose.y + dy, pose.psi + angle)
            headings = {}
            for mount in request.model.offsets:
                tangent = _mount_tangent(mount, kappa, eta, curvature_limit)
                if tangent is None:
                    return None
                headings[mount.vehicle_id] = reference.psi + tangent
            if not checker.static_check(reference, headings).success:
                return None
        pose = reference
    return pose


def _lead_in(request, checker, axis, alignment_span, step, expired):
    start = request.start_qf
    yaw = math.remainder(start.psi - axis, math.tau)
    if abs(yaw) < 1e-8:
        yield (), FormationPose(start.x, start.y, axis)
        return

    # Scan the perpendicular approach lanes in the direction the cars
    # already face. The two bends smoothly move onto the selected axis.
    lateral_start = _rotation(start.x, start.y, -axis)[1]
    lateral_origin = round(lateral_start / step) * step
    sign = math.copysign(1., yaw)
    lanes = (lateral_origin + sign * index * step - lateral_start
             for index in range(1 + math.ceil(alignment_span / step)))
    forward_lengths = sorted(
        (index * step for index in range(1, 1 + math.ceil(alignment_span / step))),
        key=lambda length: abs(length - alignment_span * .8))
    headings = (yaw + sign * spread for spread in (.22, .30, .14, .38, .07, .50))
    displacements = [(mid, _bend_displacement(yaw, mid - yaw),
                      _bend_displacement(mid, -mid)) for mid in headings]
    _, taper_offset = _bend_displacement(yaw, -yaw)
    for lateral in lanes:
        if expired():
            return
        # A car already pointing close to the lane can ease into it with
        # one turn. Overshooting its yaw first can miss the nearest clear lane.
        if abs(taper_offset) > 1e-8:
            length = lateral / taper_offset
            if step <= length <= alignment_span:
                commands = ((-yaw, length),)
                endpoint = _preview(request, checker, start, commands, expired)
                if endpoint is not None:
                    yield commands, endpoint
        for forward in forward_lengths:
            for mid, first, second in displacements:
                if expired():
                    return
                det = first[0] * second[1] - first[1] * second[0]
                if abs(det) <= 1e-9:
                    continue
                first_length = (forward * second[1] - lateral * second[0]) / det
                second_length = (lateral * first[0] - forward * first[1]) / det
                if min(first_length, second_length) < step:
                    continue
                if max(abs(mid - yaw) / first_length, abs(mid) / second_length) > .30:
                    continue
                commands = ((mid - yaw, first_length), (-mid, second_length))
                endpoint = _preview(request, checker, start, commands, expired)
                if endpoint is not None:
                    yield commands, endpoint
                    break


def _crossing_lanes(request, checker, lead, axis, normal_heading, goal_x, goal_y,
                    first_bend, last_bend, step, expired):
    first_dx, first_dy = _bend_displacement(0., *first_bend)
    last_dx, last_dy = _bend_displacement(normal_heading, *last_bend)
    target_yaw = math.remainder(request.goal_qf.psi - axis, math.tau)
    goal_cos, goal_sin = math.cos(target_yaw), math.sin(target_yaw)
    grid = checker.grid
    corners = product((grid.ox, grid.ox + grid.occ.shape[1] * grid.res),
                      (grid.oy, grid.oy + grid.occ.shape[0] * grid.res))
    max_forward = max(_rotation(x - lead.x, y - lead.y, -axis)[0] for x, y in corners)
    departure = goal_x - last_dx
    if goal_cos > 0.:
        first_forward = first_dx + step
        max_forward = min(max_forward, departure - step)
    else:
        first_forward = max(first_dx, departure) + step
    first_index = math.ceil(first_forward / step)
    last_index = math.floor(max_forward / step)
    if first_index > last_index:
        return
    required_straight = first_index * step - first_dx
    if (required_straight > 2. * step and
            _preview(request, checker, lead, ((0., required_straight),), expired) is None):
        return
    parallel_routes = []
    for index in range(first_index, 1 + last_index):
        if expired():
            return
        crossing_x = index * step
        departure_length = (goal_x - crossing_x - last_dx) / goal_cos
        departure_y = goal_y - last_dy - departure_length * goal_sin
        crossing_length = (departure_y - first_dy) / math.sin(normal_heading)
        if crossing_length <= step or departure_length <= step:
            continue
        # Screening this long straight first rejects unusable crossing lanes
        # before building either of their adjacent swept turns.
        dx, dy = _rotation(crossing_x, first_dy, axis)
        crossing_start = FormationPose(lead.x + dx, lead.y + dy, axis + normal_heading)
        if _preview(request, checker, crossing_start, ((0., crossing_length),), expired) is None:
            continue
        route = ((0., crossing_x - first_dx), first_bend, (0., crossing_length), last_bend,
                 (0., departure_length))
        if goal_cos < 0.:
            yield route
        else:
            parallel_routes.append((index, route))

    # Start with the middle of a clear lane band. At its edges a sampled
    # straight can pass while the adjacent bend clips the shoulder.
    bands = groupby(enumerate(parallel_routes), key=lambda row: row[1][0] - row[0])
    for _, band in bands:
        routes = [route for _, (_, route) in band]
        centre = (len(routes) - 1) / 2.
        yield from (routes[i] for i in sorted(range(len(routes)), key=lambda i: abs(i - centre)))


def _profile(commands):
    distance = 0.
    knots = [ControlKnot(0., 0., 0.)]
    for angle, length, *ramps in commands:
        if angle:
            knots.extend(ControlKnot(distance + knot.s, knot.value, knot.derivative)
                         for knot in _bend_knots(angle, length, *ramps)[1:])
        else:
            knots.append(ControlKnot(distance + length, 0., 0.))
        distance += length
    return C2CurvatureProfile.from_knots(knots)


def _terminal_curves(request, checker, axis, normal, last_turn, leads, limit, expired):
    """Solve the entry and crossing lengths for a compact terminal bend."""
    radius = request.successor_config.timing_limits.r_min_ackermann
    goal = request.goal_qf
    step = max(.25, checker.grid.res)
    backoffs = (0.,)
    if request.successor_config.allow_reverse:
        backoffs += tuple(index * step for index in range(1, 1 + math.floor(limit / step)))
    bend_pairs = []
    for ramps in ((.36, .20), (.39, .25), (.30, .30)):
        first_length = _compact_turn_length(normal, ramps, request.model, radius)
        last_length = _compact_turn_length(last_turn, ramps, request.model, radius)
        for entry_scale, exit_scale in product((1.26, 1.12, 1.03), (1.03, 1.07, 1.13)):
            first = normal, first_length * entry_scale, *ramps
            last = last_turn, last_length * exit_scale, *ramps
            bend_pairs.append((first, _bend_displacement(0., *first),
                               last, _bend_displacement(normal, *last)))

    # Try an exactly anchored bend before moving the turnaround beyond the
    # goal. A straight reverse docking leg uses the same body headings as
    # this forward preview. Its timed motion is audited separately.
    for backoff in backoffs:
        if expired():
            return
        target = goal
        if backoff:
            target = _preview(request, checker, goal, ((0., backoff),), expired)
            if target is None:
                continue
        for approach, lead in leads:
            goal_x, goal_y = _rotation(target.x - lead.x, target.y - lead.y, -axis)
            for first, (first_x, first_y), last, (last_x, last_y) in bend_pairs:
                if expired():
                    return
                entry = goal_x - first_x - last_x
                crossing = (goal_y - first_y - last_y) / math.sin(normal)
                if min(entry, crossing) <= 1e-8:
                    continue
                dx, dy = _rotation(entry + first_x, first_y, axis)
                crossing_start = FormationPose(lead.x + dx, lead.y + dy, axis + normal)
                if _preview(request, checker, crossing_start, ((0., crossing),), expired) is None:
                    continue
                route = ((0., entry), first, (0., crossing), last)
                endpoint = _preview(request, checker, lead, route, expired)
                if endpoint is not None and math.hypot(endpoint.x - target.x,
                                                       endpoint.y - target.y) <= 1e-8:
                    yield _profile(approach + route), target


def corridor_candidates(request, checker, cutoff, *, terminal_docking=True):
    """Yield C2 approach profiles and their exact endpoints, forward first."""
    lead_yaw = request.start_qf.psi
    if any(abs(math.remainder(yaw - lead_yaw, math.tau)) > 1e-8
           for yaw in request.start_headings.values()):
        return
    axis = round(lead_yaw / (math.pi / 2.)) * (math.pi / 2.)
    goal_yaw = math.remainder(request.goal_qf.psi - axis, math.tau)
    if abs(math.cos(goal_yaw)) <= .9:
        return
    offset = request.goal_qf.x - request.start_qf.x, request.goal_qf.y - request.start_qf.y
    _, transverse = _rotation(*offset, -axis)
    mount_radius = max(math.hypot(mount.x, mount.y) for mount in request.model.offsets)
    minimum_radius = request.successor_config.timing_limits.r_min_ackermann + mount_radius
    if abs(transverse) < 5. * minimum_radius:
        return

    def expired():
        return time.perf_counter() >= cutoff or (request.cancelled is not None and request.cancelled())

    normal = math.copysign(math.pi / 2., transverse)
    last_turn = math.remainder(goal_yaw - normal, math.tau)
    step = max(.10, 4 * checker.grid.res)
    radii = tuple(minimum_radius * scale for scale in (1.88, 2.05, 2.20, 2.42, 2.66, 1.78))
    first_scale = _bend_displacement(0., normal)[0]
    last_scale = abs(_bend_displacement(normal, last_turn)[0])
    leads = []

    def connect(approach, lead, first_bend, last_bend, lane_step):
        goal_x, goal_y = _rotation(request.goal_qf.x - lead.x, request.goal_qf.y - lead.y, -axis)
        for route in _crossing_lanes(request, checker, lead, axis, normal, goal_x, goal_y,
                                     first_bend, last_bend, lane_step, expired):
            endpoint = _preview(request, checker, lead, route, expired)
            if endpoint is not None and math.hypot(
                    endpoint.x - request.goal_qf.x, endpoint.y - request.goal_qf.y) <= 1e-8:
                yield _profile(approach + route)

    def forward_curves():
        found = False
        for approach, lead in _lead_in(request, checker, axis, 2.65 * minimum_radius, step, expired):
            leads.append((approach, lead))
            for first_radius, second_radius in product(radii, repeat=2):
                if expired():
                    return
                first_bend = normal, first_radius / first_scale
                last_bend = last_turn, second_radius / last_scale
                for profile in connect(approach, lead, first_bend, last_bend, step):
                    found = True
                    yield profile
        if found or expired() or not leads:
            return
        # Where the symmetric exit bend is too wide, shrink its footprint
        # with a shorter asymmetric curvature ramp.
        ramps = (.39, .25)
        radius = request.successor_config.timing_limits.r_min_ackermann
        length = _compact_turn_length(last_turn, ramps, request.model, radius)
        lane_step = max(.05, checker.grid.res)
        for approach, lead in leads:
            for first_radius, scale in product(radii, (1., 1.02, 1.05, 1.10)):
                if expired():
                    return
                first_bend = normal, first_radius / first_scale
                last_bend = last_turn, length * scale, *ramps
                yield from connect(approach, lead, first_bend, last_bend, lane_step)

    for profile in islice(forward_curves(), 3):
        yield profile, request.goal_qf
    if terminal_docking and leads and not expired():
        yield from islice(_terminal_curves(request, checker, axis, normal, last_turn,
                                           leads, minimum_radius, expired), 4)


def corridor_curves(request, checker, cutoff):
    """Yield forward C2 candidates that terminate at the requested pose."""
    for profile, _ in corridor_candidates(request, checker, cutoff, terminal_docking=False):
        yield profile
