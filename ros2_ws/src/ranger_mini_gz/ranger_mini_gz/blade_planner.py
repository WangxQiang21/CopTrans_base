"""Plan Ackermann cruises and local lateral repairs for a rigid blade."""

from dataclasses import dataclass, replace
import math
from time import perf_counter

import numpy as np
from scipy.interpolate import make_interp_spline
import shapely as sg

from ranger_mini_gz.blade_geometry import arc, local_footprint, primitives, world_footprint


@dataclass(frozen=True)
class Limits:
    clearance: float = .23
    cargo_speed: float = .80
    body_speed: float = .90
    high_steer_speed: float = .65
    high_steer_angle: float = math.radians(20)
    wheel_speed: float = 1.15
    acceleration: float = .36
    jerk: float = 1.30
    yaw_rate: float = .40
    steer_angle: float = math.radians(40)
    steer_rate: float = .80
    steer_accel: float = 1.50
    min_radius: float = .90
    mode_switch_radius: float = .47644
    mode_dwell: float = .90
    wheelbase: float = .494
    track: float = .364


def bezier(control):
    a, b, c, d = np.asarray(control)
    u = np.linspace(0., 1., 401)[:, None]
    v = 1-u
    points = v**3*a + 3*v*v*u*b + 3*v*u*u*c + u**3*d
    tangent = v*v*(b-a) + 2*v*u*(c-b) + u*u*(d-c)
    return np.column_stack((points, np.arctan2(tangent[:, 1], tangent[:, 0])))


def seed(scene, offset):
    experiment = scene.experiments["blade_pair"]
    start, goal = experiment["start"], experiment["goal"]
    site = scene.blade_site
    if "geometric_reference" in site.config:
        return np.asarray([experiment[p] if isinstance(p, str) else p
                           for p in site.config["geometric_reference"]], dtype=float)
    if site.centreline is not None:
        rows = site.config["road"]["centerline_primitives"]
        parts = primitives(rows)
        turn = rows[4]
        cx, cy = turn["center"]
        radius = turn["radius"] - 1 + offset
        hairpin = arc((cx, cy), radius, -math.pi/2, math.pi)
        return np.vstack((start, parts[1], parts[2], (cx-23, cy-turn["radius"], 0),
                          (cx-13, cy-radius, 0), hairpin, (cx-13, cy+radius, math.pi),
                          (cx-26, goal[1], math.pi), goal))
    # Join the west and north approaches to the southeast sweep. Both ordered
    # gates are checked after smoothing because a curve can cut a corner.
    cx, cy = site.config["road"]["center"]
    road = site.config["road"]
    radius = road["island_radius"] + (road["annulus_width"]-.5)/2 + offset
    a = radius / math.sqrt(2)
    b = (6 + 4*offset) / math.sqrt(2)
    entry = bezier([(start[0]+1, cy), (start[0]+7, cy),
                    (cx-a-b, cy-a+b), (cx-a, cy-a)])
    middle = arc((cx, cy), radius, -3*math.pi/4, math.pi)
    exit_curve = bezier([(cx+a, cy+a), (cx+a-b, cy+a+b), (cx, cy+18), (cx, goal[1]-1)])
    return np.vstack((start, entry, middle, exit_curve, goal))


def fit(points, spacing):
    station = np.r_[0., np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1).cumsum()]
    distinct = np.r_[True, np.diff(station) > 1e-8]
    knots = np.linspace(0, station[-1], max(8, math.ceil(station[-1]/spacing)+1))
    values = np.column_stack([np.interp(knots, station[distinct], column[distinct]) for column in points.T])
    # Both ends are straight so the nominal chassis yaws agree with the scene
    # headings and steering begins and ends at zero.
    boundary = [[(1, [math.cos(yaw), math.sin(yaw), 0.]), (2, [0., 0., 0.])]
                for yaw in (values[0, 2], values[-1, 2])]
    return make_interp_spline(knots, values, k=5, bc_type=boundary)


def kinematics(spline, station, offsets):
    cargo, first, second = (spline(station, nu=order) for order in range(3))
    yaw = cargo[:, 2:3]
    c, s = np.cos(yaw), np.sin(yaw)
    ox, oy = np.asarray(offsets).T
    lever = np.stack((c*ox-s*oy, s*ox+c*oy), axis=2)
    normal = np.stack((-lever[:, :, 1], lever[:, :, 0]), axis=2)
    theta_s = first[:, None, 2:3]
    center = cargo[:, None, :2] + lever
    velocity = first[:, None, :2] + theta_s*normal
    acceleration = second[:, None, :2] + second[:, None, 2:3]*normal - theta_s**2*lever
    norm = np.linalg.norm(velocity, axis=2)
    heading = np.unwrap(np.arctan2(velocity[:, :, 1], velocity[:, :, 0]), axis=0)
    heading += math.tau*np.rint((cargo[0, 2]-heading[0])/math.tau)
    cross = velocity[:, :, 0]*acceleration[:, :, 1] - velocity[:, :, 1]*acceleration[:, :, 0]
    yaw_rate = np.divide(cross, norm**2, out=np.full_like(norm, np.inf), where=norm>1e-9)
    return cargo, np.dstack((center, heading)), norm, yaw_rate


def steering_and_wheels(speed, yaw_rate, limits):
    longitudinal = speed[:, :, None] + yaw_rate[:, :, None] * np.array([-1., 1.]) * limits.track/2
    transverse = yaw_rate[:, :, None] * limits.wheelbase/2
    # Both axles have the same magnitude and opposite steering direction.
    steer = np.arctan2(transverse, longitudinal)
    wheels = np.hypot(longitudinal, transverse)
    return steer, wheels


def derivative_bound(spline, order):
    derivative = spline.derivative(order)
    coeff = derivative.c[:len(derivative.t)-derivative.k-1]
    return np.max(np.linalg.norm(coeff[:, :2], axis=1)), np.max(np.abs(coeff[:, 2]))


def chord_error(spline, station, body_speed_s, offsets):
    xy2, yaw2 = derivative_bound(spline, 2)
    _, yaw1 = derivative_bound(spline, 1)
    xy3, yaw3 = derivative_bound(spline, 3)
    arms = np.linalg.norm(offsets, axis=1)
    a2 = xy2 + arms*(yaw2 + yaw1**2)
    a3 = xy3 + arms*(yaw3 + 3*yaw1*yaw2 + yaw1**3)
    delta = np.diff(station)
    # A derivative lies in the convex hull of its B-spline coefficients.
    # The minimum contact tangent norm can drop by at most ||p''|| ds/2.
    lower = np.minimum(body_speed_s[:-1], body_speed_s[1:]) - delta[:, None]*a2/2
    if np.any(lower <= 0):
        raise ValueError("a mount tangent can vanish between samples")
    heading2 = a3/lower + 2*(a2/lower)**2
    return delta[:, None]**2 / 8 * np.column_stack((
        np.full(len(delta), xy2), np.full(len(delta), yaw2), heading2))


def routes(scene, limits, reference=None, check_gates=True, rejected=None):
    road = scene.blade_site.road
    sg.prepare(road)
    outline = local_footprint(scene)
    rings = [np.asarray(r.coords) for r in (road.exterior, *road.interiors)]
    edges = sg.STRtree(sg.linestrings(np.concatenate([np.stack((r[:-1], r[1:]), axis=1) for r in rings])))
    gates = scene.experiments["blade_pair"]["ordered_gates"] if check_gates else []
    positions = ((0.,) if reference is not None or
                 (scene.blade_site.centreline is None and scene.blade_site.island is None) else (0., -.25, .25))
    for offset in positions:
        initial = seed(scene, offset) if reference is None else reference
        spacing_values = (2.1, 3.2, 5., 6.) if reference is None else (1.2, 1.7, 2.1, 2.7, 3.2, 4.2, 5., 6.)
        for spacing in spacing_values:
            spline = fit(initial, spacing)
            station = np.linspace(0., spline.t[-1], math.ceil(spline.t[-1]/.20)+1)
            cargo, _, speed, yaw = kinematics(spline, station, scene.offsets)
            if np.any(speed < .25):
                if rejected is not None:
                    rejected.append(dict(spacing_m=spacing, seed_offset_m=offset, reason="mount_tangent_near_zero"))
                continue
            curvature = np.max(np.abs(yaw)/speed)
            if curvature > 1/limits.min_radius:
                if rejected is not None:
                    rejected.append(dict(spacing_m=spacing, seed_offset_m=offset, reason="ackermann_radius",
                                         min_body_radius_m=float(1/curvature)))
                continue
            last = -1
            for gate in gates:
                ids = np.flatnonzero((cargo[:, 0]>=gate["x"][0]) & (cargo[:, 0]<=gate["x"][1]) &
                                     (cargo[:, 1]>=gate["y"][0]) & (cargo[:, 1]<=gate["y"][1]) &
                                     (np.arange(len(cargo))>last))
                if not len(ids):
                    if rejected is not None:
                        rejected.append(dict(spacing_m=spacing, seed_offset_m=offset, reason="ordered_gates"))
                    break
                last = ids[0]
            else:
                bodies = [world_footprint(outline, q) for q in cargo]
                covers = sg.covers(road, bodies)
                if np.all(covers):
                    _, distance = edges.query_nearest(bodies, return_distance=True, all_matches=False)
                    margin = float(distance.min())
                    if margin >= limits.clearance:
                        yield spline, float(offset), float(spacing), margin
                    elif rejected is not None:
                        rejected.append(dict(spacing_m=spacing, seed_offset_m=offset, reason="road_clearance",
                                             minimum_m=margin))
                elif rejected is not None:
                    first = int(np.flatnonzero(~covers)[0])
                    rejected.append(dict(spacing_m=spacing, seed_offset_m=offset, reason="footprint_outside_road",
                                         cargo_pose=cargo[first].tolist()))


def clock_profile(length, speed, period):
    # Sextic integral of the quintic speed ramp, and its time-reverse at the exit.
    ramp = min(3.6, length/speed)
    duration = length/speed + ramp
    count = math.ceil(duration/period)
    time = np.linspace(0., duration, count+1)
    reverse = time > duration/2
    reflected = np.minimum(time, duration-time)
    u = np.minimum(reflected/ramp, 1.)
    station = speed*(ramp*u**4*(2.5 + u*(-3+u)) + np.maximum(reflected-ramp, 0.))
    ds = speed*u**3*(10+u*(-15+6*u))
    return time, np.where(reverse, length-station, station), ds


def dynamic_metrics(t, speed, yaw, steer, wheels, cargo_speed):
    acc = np.gradient(speed, t, axis=0, edge_order=2)
    steering_rate = np.gradient(steer, t, axis=0, edge_order=2)
    return dict(body_speed=float(np.max(speed)), yaw_rate=float(np.max(np.abs(yaw))),
                cargo_speed=float(np.max(cargo_speed)), wheel_speed=float(np.max(wheels)),
                acceleration=float(np.max(np.abs(acc))),
                jerk=float(np.max(np.abs(np.gradient(acc, t, axis=0, edge_order=2)))),
                steer_angle=float(np.max(np.abs(steer))), steer_rate=float(np.max(np.abs(steering_rate))),
                steer_accel=float(np.max(np.abs(np.gradient(steering_rate, t, axis=0, edge_order=2)))))


def time_scale(spline, offsets, limits, period):
    length = spline.t[-1]
    coarse = np.linspace(0., length, math.ceil(length/.035)+1)
    _, _, speeds, yaw_rates = kinematics(spline, coarse, offsets)
    cargo_norm = np.linalg.norm(spline(coarse, nu=1)[:, :2], axis=1)
    steer, wheels = steering_and_wheels(speeds, yaw_rates, limits)
    high_steer = np.max(np.abs(steer), axis=2) >= limits.high_steer_angle
    caps = [limits.cargo_speed/cargo_norm.max(), limits.body_speed/speeds.max(),
            limits.yaw_rate/max(1e-9, np.abs(yaw_rates).max()), limits.wheel_speed/wheels.max()]
    if np.any(high_steer):
        caps.append(limits.high_steer_speed/speeds[high_steer].max())
    cruise = .95*min(caps)
    for _ in range(7):
        time, station, ds = clock_profile(length, cruise, period)
        cargo, cars, speeds, yaw_rates = kinematics(spline, station, offsets)
        steer, _ = steering_and_wheels(speeds, yaw_rates, limits)
        v, w = speeds*ds[:, None], yaw_rates*ds[:, None]
        _, wheels = steering_and_wheels(v, w, limits)
        cargo_speed = np.linalg.norm(spline(station, nu=1)[:, :2], axis=1)*ds
        metrics = dynamic_metrics(time, v, w, steer, wheels, cargo_speed)
        curvature = np.abs(yaw_rates)/speeds
        if curvature.max() > 1/limits.min_radius or metrics["steer_angle"] > limits.steer_angle:
            raise ValueError("Ackermann steering limit")
        if all(value <= getattr(limits, key) for key, value in metrics.items()):
            metrics["min_body_turn_radius_m"] = float(1/max(curvature.max(), 1e-9))
            metrics["ackermann_switch_boundary_m"] = limits.mode_switch_radius
            metrics["cargo_path_length_m"] = float(np.linalg.norm(np.diff(cargo[:, :2], axis=0), axis=1).sum())
            metrics["chassis_path_length_m"] = np.linalg.norm(np.diff(cars[:, :, :2], axis=0), axis=2).sum(axis=0).tolist()
            return dict(time=time, cargo=cargo, vehicles=cars, feedforward=np.dstack((v, w)),
                        interpolation_error=chord_error(spline, station, speeds, offsets)), metrics
        cruise *= .86
    raise ValueError("unable to time-scale the route within steering and acceleration limits")


def ackermann_options(scene, limits, period, reference=None, *, check_gates=True, rejected=None):
    candidates = []
    for spline, offset, spacing, gap in routes(scene, limits, reference, check_gates, rejected):
        try:
            trajectory, dynamics = time_scale(spline, scene.offsets, limits, period)
        except ValueError as error:
            if rejected is not None:
                rejected.append(dict(spacing_m=spacing, seed_offset_m=offset, reason=str(error)))
            continue
        duration = float(trajectory["time"][-1])
        # Keep reserve for closed-loop tracking and avoid sharp steering when
        # durations are similar. All accepted routes use one Ackermann run.
        score = duration + 6/max(.1, gap) + 8/dynamics["min_body_turn_radius_m"]
        candidates.append((score, trajectory, dynamics, offset, spacing, gap))
    candidates.sort(key=lambda row: row[0])
    return candidates


def oblique_segment(route, offsets, limits, period):
    start, end = route
    distance = np.linalg.norm(end[:2]-start[:2])
    if distance <= 0. or abs(math.remainder(end[2]-start[2], math.tau)) > 1e-8:
        raise ValueError("the lateral transfer requires nonzero travel and fixed cargo yaw")
    direction = (end[:2]-start[:2])/distance
    c, s = math.cos(start[2]), math.sin(start[2])
    local = np.array((c*direction[0]+s*direction[1], -s*direction[0]+c*direction[1]))
    if abs(local[1]) < .2:
        raise ValueError("the lateral transfer has no significant lateral motion")
    time, station, speed = clock_profile(distance, min(.32, limits.high_steer_speed), period)
    cargo = np.column_stack((start[:2]+station[:, None]*direction, np.full(len(time), start[2])))
    rotation = np.array(((c, -s), (s, c)))
    centers = cargo[:, None, :2]+np.asarray(offsets) @ rotation.T
    cars = np.dstack((centers, np.repeat(cargo[:, 2:3], 2, axis=1)))
    velocity = np.repeat((speed[:, None]*local)[:, None, :], 2, axis=1)
    feedforward = np.dstack((velocity, np.zeros((len(time), 2))))
    return dict(time=time, cargo=cargo, vehicles=cars, feedforward=feedforward,
                interpolation_error=np.zeros((len(time)-1, 4)))


def join_segments(parts, period, dwell):
    data = {key: [] for key in ("time", "cargo", "vehicles", "feedforward", "motion_mode", "interpolation_error")}
    elapsed = 0.
    events = []
    for i, (mode, part) in enumerate(parts):
        if i:
            previous = parts[i-1][1]
            if not np.allclose(previous["cargo"][-1], part["cargo"][0], atol=1e-8, rtol=0.) or not np.allclose(
                    previous["vehicles"][-1], part["vehicles"][0], atol=1e-7, rtol=0.):
                raise ValueError("stationary mode transition has inconsistent mount poses or chassis headings")
            if parts[i-1][0] != mode:
                count = math.ceil(dwell/period)+1
                events.append(dict(mode=mode, time_s=elapsed+period, stationary_setup_s=(count-1)*period))
                data["time"].append(elapsed + np.arange(1, count+1)*period)
                data["motion_mode"].append(np.full(count, mode))
                data["cargo"].append(np.repeat(part["cargo"][:1], count, axis=0))
                data["vehicles"].append(np.repeat(part["vehicles"][:1], count, axis=0))
                data["feedforward"].append(np.zeros((count, 2, 3)))
                data["interpolation_error"].append(np.zeros((count, 4)))
                elapsed += count*period
        else:
            events.append(dict(mode=mode, time_s=0.))
        first = int(i != 0)
        v = part["feedforward"]
        if v.shape[-1] == 2:
            v = np.dstack((v[:, :, 0], np.zeros(v.shape[:2]), v[:, :, 1]))
        for k, values in (("time", part["time"]+elapsed), ("cargo", part["cargo"]),
                          ("vehicles", part["vehicles"]), ("feedforward", v),
                          ("motion_mode", np.full(len(v), mode))):
            data[k].append(values[first:])
        data["interpolation_error"].append(part["interpolation_error"])
        elapsed += part["time"][-1]
    return {key: np.concatenate(values) for key, values in data.items()}, events


def certify_candidate(certifier, data, limit):
    mount_error = float(np.max(np.linalg.norm(
        data["vehicles"][:, :, :2] - certifier.mount_centers(data["cargo"]), axis=2)))
    if mount_error > 1e-9:
        raise ValueError("stored vehicle pose differs from its rigid mount")
    report = certifier.certify(data["cargo"], data["vehicles"][:, :, 2], times=data["time"],
                              clearance_margin_m=limit, interpolation_error=data["interpolation_error"])
    report["max_stored_mount_error_m"] = mount_error
    return report


def repair_anchors(scene, reference):
    # Sample ordinary geometric anchors and keep the narrowest locations
    # separated along the route. Distance along the path excludes terminal
    # corrections and allows both cruises to develop a tangent.
    station = np.r_[0., np.linalg.norm(np.diff(reference[:, :2], axis=0), axis=1).cumsum()]
    probes = np.arange(8., station[-1]-8., 1.)
    cargo = np.column_stack([np.interp(probes, station, values) for values in reference.T])
    outline = local_footprint(scene)
    road = scene.blade_site.road
    footprints = [world_footprint(outline, pose) for pose in cargo]
    safe = sg.covers(road, footprints)
    boundary_gap = sg.distance(road.boundary, footprints)
    order = sorted(np.flatnonzero(safe), key=lambda i: boundary_gap[i])
    selected = []
    for i in order:
        if all(abs(probes[i]-v)>4. for v in selected):
            selected.append(probes[i])
            yield reference[station < probes[i]], cargo[i], reference[station > probes[i]]
            if len(selected) == 4:
                return


def local_repairs(scene, certifier, reference, limits, period, allow_oblique):
    anchors = list(repair_anchors(scene, reference))
    distances = (0., .25, -.25, .45, -.45, .7, -.7) if allow_oblique else (0.,)
    for lateral in distances:
        for before, pivot, after in anchors:
            direction = np.array((-math.sin(pivot[2]), math.cos(pivot[2]), 0.))
            destination = pivot+lateral*direction
            if not certifier.check_pose(destination, margin_m=limits.clearance):
                continue
            # A stopped Ackermann split is tried before any lateral repairs.
            incoming = ackermann_options(scene, limits, period, np.vstack((before, pivot)), check_gates=False)
            if not incoming:
                continue
            outgoing = ackermann_options(scene, limits, period, np.vstack((destination, after)), check_gates=False)
            if not outgoing:
                continue
            parts = [(0, incoming[0][1])]
            if lateral:
                parts.append((1, oblique_segment(np.array((pivot, destination)), scene.offsets, limits, period)))
            parts.append((0, outgoing[0][1]))
            data, events = join_segments(parts, period, limits.mode_dwell)
            report = certify_candidate(certifier, data, limits.clearance)
            if report["passed"]:
                return data, events, report, float(lateral), pivot.tolist(), [incoming[0][2], outgoing[0][2]]
    return None


def plan(scene, occupied, period=.025, *, ackermann_only=False):
    from ranger_mini_gz.blade_certify import BladeCertifier

    begin = perf_counter()
    config = scene.blade_site.config
    hard_margin = scene.blade_site.config.get("planning_clearance_m", Limits.clearance)
    limits = replace(Limits(), clearance=hard_margin)
    certifier = BladeCertifier(scene, occupied)
    # Attempt to smooth the whole route without a stop or a mode change.
    reference = seed(scene, 0.) if config.get("tight_bends") else None
    rejections = []
    candidates = ackermann_options(scene, limits, period, reference, rejected=rejections)
    tried = []
    for _, data, dynamics, offset, spacing, gap in candidates:
        cert_start = perf_counter()
        report = certify_candidate(certifier, data, limits.clearance)
        tried.append(dict(seed_offset_m=offset, spline_spacing_m=spacing, screen_clearance_m=gap,
                          certificate=report, certification_wall_s=perf_counter()-cert_start))
        if report["passed"]:
            return data, dict(status="passed", planner="dual_rigid_blade_ackermann", mode_sequence=["Ackermann"],
                              mode_switches=0, duration_s=float(data["time"][-1]), dynamics=dynamics,
                              limits=vars(limits), feasible_candidates=len(candidates), attempts=tried,
                              wall_s=perf_counter()-begin)
    ackermann = dict(status="failed", reason="no whole-route Ackermann candidate passed the geometry and steering checks",
                     feasible_candidates=len(candidates), attempts=tried, rejected_candidates=rejections,
                     wall_s=perf_counter()-begin)
    if reference is None:
        return None, ackermann
    repair = local_repairs(scene, certifier, reference, limits, period, not ackermann_only)
    if repair is None:
        if not ackermann_only:
            from ranger_mini_gz.blade_bend_search import chain_plan, shortest_chain
            chain = shortest_chain(reference, certifier, limits, allow_oblique=True)
            if chain:
                data, events = chain_plan(chain, scene, period, limits)
                report = certify_candidate(certifier, data, limits.clearance)
                if report["passed"]:
                    names = ("Ackermann", "Oblique", "Spin")
                    return data, dict(status="passed", planner="dual_rigid_blade_bend_search",
                                      duration_s=float(data["time"][-1]),
                                      mode_switches=len(events)-1, mode_sequence=[names[e["mode"]] for e in events],
                                      mode_events=events, primitives=len(chain),
                                      ackermann_no_stop_result=ackermann, limits=vars(limits),
                                      attempts=[dict(certificate=report)], wall_s=perf_counter()-begin)
        return None, dict(ackermann, reason="no tested continuous or locally split route passed",
                          wall_s=perf_counter()-begin)
    data, events, report, adjustment, pose, dynamics = repair
    mode_names = ("Ackermann", "Oblique")
    return data, dict(status="passed", planner="dual_rigid_blade_local_repair",
                      mode_sequence=[mode_names[e["mode"]] for e in events], mode_switches=len(events)-1,
                      duration_s=float(data["time"][-1]), mode_events=events, cruise_dynamics=dynamics,
                      lateral_adjustment_m=adjustment, repaired_pose=pose, ackermann_no_stop_result=ackermann,
                      limits=vars(limits), attempts=[dict(certificate=report)],
                      wall_s=perf_counter()-begin)
