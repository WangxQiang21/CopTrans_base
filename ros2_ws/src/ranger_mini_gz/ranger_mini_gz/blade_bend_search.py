"""Search geometry-certified rigid turns with stationary chassis alignment."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class Primitive:
    kind: int
    start: np.ndarray
    end: np.ndarray
    tangent: np.ndarray
    delta: float
    chassis_speed: np.ndarray
    heading: np.ndarray

    def samples(self, u):
        turn = self.delta*u
        c, s = np.cos(turn/2), np.sin(turn/2)
        x, y = self.tangent
        ch = np.column_stack((c*x-s*y, s*x+c*y))*((u*np.sinc(turn/(2*math.pi)))[:, None])
        cargo = np.column_stack((self.start[:2]+ch, self.start[2]+turn))
        return cargo, self.heading[None, :]+turn[:, None]


def primitive(a, b, offsets, limits, oblique):
    turn = b[2]-a[2]
    xy = b[:2]-a[:2]
    c, s = math.cos(turn/2), math.sin(turn/2)
    tangent = np.array((c*xy[0]+s*xy[1], -s*xy[0]+c*xy[1]))/np.sinc(turn/(2*math.pi))
    c, s = math.cos(a[2]), math.sin(a[2])
    x, y = offsets.T
    velocity = tangent + turn*np.column_stack((-s*x-c*y, c*x-s*y))
    speeds = np.linalg.norm(velocity, axis=1)
    if oblique and abs(turn) < 1e-8:
        ux, uy = tangent
        if abs(c*uy-s*ux) > .25*np.linalg.norm(tangent):
            yield Primitive(1, a, b, tangent, 0., speeds, np.full(2, a[2]))
    if np.min(speeds) < .03 or abs(turn)*limits.min_radius > np.min(speeds):
        return
    yield Primitive(0, a, b, tangent, turn, speeds, np.arctan2(velocity[:, 1], velocity[:, 0]))


def difference(from_heading, to_heading):
    return (to_heading-from_heading+math.pi)%math.tau-math.pi


def tested(edge, certifier, margin):
    length = np.linalg.norm(edge.tangent)+certifier.cargo_radius*abs(edge.delta)
    chassis = edge.chassis_speed.max()+certifier.chassis_radius*abs(edge.delta)
    travel = max(length, chassis)
    count = max(3, math.ceil(travel/.075))
    u = np.linspace(0., 1., count+1)
    cargo, yaws = edge.samples(u)
    return bool(np.all(certifier.check_poses(cargo, yaws, margin_m=margin+travel/(2*count))))


def estimated_duration(edge, limits):
    radius_cap = limits.yaw_rate / max(abs(edge.delta), 1e-10)
    speed_cap = (.32 if edge.kind == 1 else .65)/edge.chassis_speed.max()
    return 2.8 + 1/min(radius_cap, speed_cap)


def alignment_cost(previous, following, limits):
    angle = max(abs(difference(previous, following)))
    return 0. if angle < 1e-7 else 2*limits.mode_dwell+2.8+angle/.30


def shortest_chain(points, certifier, limits, allow_oblique):
    offsets = certifier.offsets
    layers = [[] for _ in points]
    # The label includes the previous edge because each chassis has an
    # independent heading and heading determines the next alignment cost.
    layers[0].append((0., None, None, np.full(2, points[0, 2])))
    for j in range(1, len(points)):
        for i in range(j-1, -1, -1):
            if not layers[i]:
                continue
            chord = math.dist(points[j, :2], points[i, :2])
            if chord > 28. or abs(points[j, 2]-points[i, 2]) > .92:
                continue
            for edge in primitive(points[i], points[j], offsets, limits, allow_oblique):
                if not tested(edge, certifier, limits.clearance+.006):
                    continue
                travel = estimated_duration(edge, limits)
                choose = min(layers[i], key=lambda label: label[0]+alignment_cost(label[3], edge.heading, limits))
                head = edge.heading+edge.delta
                cost = choose[0]+alignment_cost(choose[3], edge.heading, limits)+travel
                if edge.kind or (choose[1] and choose[1].kind):
                    cost += limits.mode_dwell
                layers[j].append((cost, edge, choose, head))
        # A small set of different incoming headings is useful without making
        # the search pay repeatedly for the same straight approach.
        distinct = []
        for label in sorted(layers[j], key=lambda v: v[0]):
            if all(max(abs(difference(label[3], old[3]))) > .035 for old in distinct):
                distinct.append(label)
                if len(distinct) == 7:
                    break
        layers[j] = distinct
    if not layers[-1]:
        return []
    goal_yaw = np.full(2, points[-1, 2])
    label = min(layers[-1], key=lambda v: v[0]+alignment_cost(v[3], goal_yaw, limits))
    chain = []
    while label[1] is not None:
        chain.append(label[1])
        label = label[2]
    return chain[::-1]


def align_segment(cargo, heading, delta, offsets, period, limits):
    from ranger_mini_gz.blade_planner import clock_profile

    maximum = float(np.max(np.abs(delta)))
    # Keep spin acceleration below the translation-yaw response limit.
    speed = min(.31, math.sqrt(.19*maximum), limits.yaw_rate)
    time, station, omega = clock_profile(maximum, speed, period)
    body_heading = heading + (station[:, None]/maximum)*delta
    payload = np.repeat(cargo[None, :], len(time), axis=0)
    c, s = math.cos(cargo[2]), math.sin(cargo[2])
    centers = np.asarray(offsets) @ np.array(((c, s), (-s, c))) + cargo[:2]
    vehicle = np.dstack((np.repeat(centers[None, :, :], len(time), axis=0), body_heading))
    ff = np.zeros((len(time),2,3))
    ff[:,:,2] = (omega[:,None]/maximum)*delta
    return dict(time=time, cargo=payload, vehicles=vehicle, feedforward=ff,
                interpolation_error=np.zeros((len(time)-1,4)))


def move_segment(edge, offsets, headings, period, limits):
    from ranger_mini_gz.blade_planner import clock_profile

    length = max(float(edge.chassis_speed.max()), float(np.linalg.norm(edge.tangent)))
    max_yaw_ratio = abs(edge.delta)/length
    speed = min(.32 if edge.kind == 1 else .66, math.sqrt(.15*length),
                limits.yaw_rate/max(max_yaw_ratio, 1e-10))
    time, station, ds = clock_profile(length, speed, period)
    u = station/length
    cargo, yaws = edge.samples(u)
    yaws += headings-yaws[0]
    first = np.sin(cargo[:, 2:3])
    second = np.cos(cargo[:, 2:3])
    ox, oy = np.asarray(offsets).T
    centers = np.stack((cargo[:, :1]+second*ox-first*oy, cargo[:, 1:2]+first*ox+second*oy), axis=2)
    vehicles = np.dstack((centers,yaws))
    ff = np.zeros((len(time), 2, 3))
    if edge.kind == 0:
        ff[:,:,0] = ds[:,None]*(edge.chassis_speed/length)
        ff[:,:,2] = (edge.delta/length)*ds[:,None]
    else:
        x, y = edge.tangent/length
        c, s = np.cos(headings), np.sin(headings)
        ff[:,:,0] = ds[:,None]*(c*x+s*y)
        ff[:,:,1] = ds[:,None]*(-s*x+c*y)
    remainder = np.zeros((len(time)-1,4))
    remainder[:,0] = abs(edge.delta)*np.linalg.norm(edge.tangent)*np.diff(u)**2/8
    return dict(time=time, cargo=cargo, vehicles=vehicles, feedforward=ff, interpolation_error=remainder)


def chain_plan(chain, scene, period, limits):
    from ranger_mini_gz.blade_planner import join_segments

    parts = []
    yaw = np.full(2, chain[0].start[2])
    for edge in chain:
        delta = difference(yaw, edge.heading)
        if max(abs(delta)) > 1e-7:
            parts.append((2,align_segment(edge.start, yaw, delta, scene.offsets, period, limits)))
            yaw = yaw+delta
        parts.append((edge.kind,move_segment(edge, scene.offsets, yaw, period, limits)))
        yaw = yaw+edge.delta
    goal = chain[-1].end
    delta = difference(yaw, np.full(2, goal[2]))
    if max(abs(delta)) > 1e-7:
        parts.append((2,align_segment(goal, yaw, delta, scene.offsets, period, limits)))
    if parts[0][0] != 0:
        first = parts[0][1]
        stop = dict(time=np.array([0.,period]), cargo=np.repeat(first["cargo"][:1],2,axis=0),
                    vehicles=np.repeat(first["vehicles"][:1],2,axis=0), feedforward=np.zeros((2,2,3)),
                    interpolation_error=np.zeros((1,4)))
        parts.insert(0,(0,stop))
    return join_segments(parts, period, limits.mode_dwell)
