"""Local road edits on the two existing hard maps. All coordinates are in metres."""
from dataclasses import replace
import math
from pathlib import Path

import numpy as np
import shapely as sg
from scipy.optimize import brentq

from ranger_mini_gz.blade_geometry import arc, primitives
from ranger_mini_gz.scene_generator import load_scene


def narrow_mountain(site, bend, width, return_radius=None, return_tangent=2.):
    """Trim the pavement along normals of the original centreline."""
    road_config = site.config["road"]
    original_width = road_config["width_m"]
    if not 1.3 < width <= original_width:
        raise ValueError(f"mountain width must be in (1.3, {original_width}] m")
    parts = primitives(road_config["centerline_primitives"])
    if bend == "return":
        # Include the original curved widening at this S bend, with each
        # cut ending on an unmodified straight portion of the roadway.
        path = np.vstack(([77., 46., math.pi], parts[6], parts[7][1:], [48., 54., math.pi]))
        if return_radius is not None:
            if not 5.5 <= return_radius <= 8.:
                raise ValueError("return radius must be between 5.5 m and 8.0 m")
            # Two tangent circular curves with a short straight between the
            # opposing turns. Match the original approaches on both sides.
            if not 1. <= return_tangent <= 8.:
                raise ValueError("return tangent must be between 1 and 8 m")
            straight = return_tangent
            angle = brentq(lambda a: 2*return_radius*(1-math.cos(a)) + straight*math.sin(a) - 8.,
                           0., math.pi/2)
            mid_x = (parts[6][0, 0] + parts[7][-1, 0])/2
            half_x = return_radius*math.sin(angle) + straight*math.cos(angle)/2
            west, east = mid_x-half_x, mid_x+half_x
            first = arc((east, 46.+return_radius), return_radius, -math.pi/2, -angle)
            second = arc((west, 54.-return_radius), return_radius, math.pi/2-angle, angle)
            path = np.vstack(([77., 46., math.pi], first, second, [48., 54., math.pi]))
        endpoints = ((85., 46.), (40., 53.4))
        taper = 6.
        plot_bounds = [[48., 79.], [41., 60.]]
    elif bend == "hairpin":
        path = np.vstack(([97., 30., 0.], parts[4], [97., 46., math.pi]))
        endpoints = ((89., 30.8), (88., 45.))
        taper = 9.
        plot_bounds = [[94., 119.], [24., 52.]]
    else:
        raise ValueError(bend)

    stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
    sample = np.unique(np.r_[stations, np.linspace(0., stations[-1], math.ceil(stations[-1] / .065) + 1)])
    xy = np.column_stack([np.interp(sample, stations, c) for c in path[:, :2].T])
    heading = np.interp(sample, stations, np.unwrap(path[:, 2]))
    normal = np.column_stack((-np.sin(heading), np.cos(heading)))
    # Match the width and its derivatives where the new edges meet the
    # original pavement on the two straight approaches.
    t = np.clip(np.minimum(sample, stations[-1] - sample) / taper, 0., 1.)
    transition = t**3 * (10 - 15*t + 6*t*t)
    half = (original_width + (width-original_width) * transition) / 2
    if width == original_width and return_radius is None:
        return site.road, endpoints, plot_bounds, taper
    edges = normal * half[:, None]
    road_section = sg.Polygon(np.vstack((xy + edges, (xy-edges)[::-1])))
    if return_radius is not None:
        window = sg.box(48., 40., 77., 61.)
        return site.road.difference(window).union(road_section), endpoints, plot_bounds, taper
    # The window follows the same curve, avoids the other lanes and extends
    # past the existing local curve widening on each side of the road.
    window_edges = normal * 5.4
    window = sg.Polygon(np.vstack((xy+window_edges, (xy-window_edges)[::-1])))
    if not (road_section.is_valid and window.is_valid):
        raise ValueError("the selected taper is wider than the local bend allows")
    trimmed = site.road.difference(window).union(site.road.intersection(window).intersection(road_section))
    return trimmed, endpoints, plot_bounds, taper


def study_scene(source, kind, width, tip_extension=0., exit_curb_y=None, bend="return",
                return_radius=None, return_tangent=2.):
    name = "blade_right_angle_hard" if kind == "street" else "blade_mountain_hard"
    base = load_scene(name, paths=tuple(source / p for p in ("ranger_mini_gz", "fwis_common", "fwis_backends")))
    if kind == "street":
        # Inset the inside corner of the first elbow with a flat face and tapered ends.
        depth = 6.15 - width
        if not 0 <= depth < 5:
            raise ValueError("street width must be in (1.15, 6.15]")
        x, y = 40.925, 25.075
        corner = sg.Polygon([(x - 3.5, y + .1), (x - 3.5, y), (x - 1.8, y - depth),
                             (x + depth, y - depth), (x + depth, y + 1.8),
                             (x, y + 3.5), (x - .1, y + 3.5)])
        if exit_curb_y is not None:
            if not y + 1.5 < exit_curb_y < 33.:
                raise ValueError("exit curb must be within the first bend")
            corner = sg.Polygon([(x - .12, y), (x, y), (x + depth, exit_curb_y),
                                 (x + max(0., depth - .55), exit_curb_y + .85),
                                 (x, exit_curb_y + 3.), (x - .12, exit_curb_y + 3.)])
        road = base.blade_site.road.difference(corner) if depth else base.blade_site.road
        ends = ((24., 22.), (43.6, 36.))
        settings = dict(base=name, first_elbow_clear_width_m=width, inside_setback_m=depth,
                        second_elbow_width_m=6.15, exit_curb_y=exit_curb_y)
    elif kind == "mountain":
        road, ends, bounds, taper = narrow_mountain(base.blade_site, bend, width, return_radius, return_tangent)
        settings = dict(base=name, mountain_bend=bend, width_m=width, original_width_m=5.8,
                        taper_length_m=taper, centerline_min_radius_m=return_radius or 8.,
                        plot_bounds=bounds, geometry="original_centerline_with_smooth_width_tapers")
        if return_radius is not None:
            settings.update(geometry="tangent_reverse_curves", intermediate_tangent_length_m=return_tangent)
    else:
        raise ValueError(kind)
    if road.geom_type != "Polygon" or not road.is_valid:
        raise ValueError("the modified road is disconnected or invalid")
    edited = blade_span(replace(base, blade_site=replace(base.blade_site, road=road)), tip_extension)
    settings.update(tip_extension_m=tip_extension, blade_length_m=float(np.ptp(np.array(edited.cargo.outline)[:, 0])),
                    support_spacing_m=math.dist(*edited.offsets), map_resolution_m=base.resolution,
                    required_clearance_m=base.blade_site.config["planning_clearance_m"])
    return base, edited, ends, settings


def blade_span(scene, tip_extension):
    if not tip_extension:
        return scene
    outline = np.array(scene.cargo.outline)
    pin_x = max(x for x, _ in scene.offsets)
    tip = outline[:, 0].max()
    if tip_extension <= pin_x - tip:
        raise ValueError("the tip must extend beyond the outer support")
    outline[:, 0] += tip_extension * np.maximum(0., outline[:, 0] - pin_x) / (tip - pin_x)
    low, high = outline.min(axis=0), outline.max(axis=0)
    cargo = replace(scene.cargo, outline=tuple(map(tuple, outline)),
                    size=(*map(float, high - low), scene.cargo.size[2]),
                    offset=tuple((low + high) / 2))
    return replace(scene, cargo=cargo)


def elbow_slice(scene, angle, margin):
    # Convex Minkowski sums on a complete triangulation give forbidden XY
    # translations for this blade yaw. Use the blade alone to relax both cars.
    workspace = sg.box(-10, -10, 75, 75)
    available = scene.blade_site.road.buffer(-margin, quad_segs=48)
    solid = workspace.difference(available)
    a = math.radians(angle)
    rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    reflected = sg.Polygon(-np.array(scene.cargo.outline) @ rot.T)
    triangle_sets = [sg.get_coordinates(sg.constrained_delaunay_triangles(obj)).reshape(-1, 4, 2)[:, :3]
                     for obj in (solid, reflected)]
    sums = (triangle_sets[0][:, None, :, None, :] + triangle_sets[1][None, :, None, :, :]).reshape(-1, 9, 2)
    forbidden = sg.union_all(sg.convex_hull(sg.multipoints(sums)))
    return sg.box(0., 0., scene.size[0], 36.).difference(forbidden)


def cruise_reference(scene, archive):
    from ranger_mini_gz.blade_planner import plan

    if archive:
        with np.load(Path(archive), allow_pickle=False) as f:
            data = {name: f[name] for name in f.files}
    else:
        data, report = plan(scene, scene.blade_site.occupied(scene.size, scene.resolution))
        if data is None:
            raise ValueError(report)
    experiment = scene.experiments["blade_pair"]
    if any(not np.allclose(p, data["cargo"][i], atol=1e-7, rtol=0)
           for p, i in ((experiment["start"], 0), (experiment["goal"], -1))):
        raise ValueError("warm start has different route endpoints")
    return data


def reference_section(data, ends, spacing):
    cargo = data["cargo"]
    first, last = (int(np.argmin(np.linalg.norm(cargo[:, :2] - point, axis=1))) for point in ends)
    if first >= last:
        raise ValueError("reversed local section endpoints")
    raw = np.column_stack([cargo[first:last + 1], data["vehicles"][first:last + 1, :, 2]])
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(raw[:, :2], axis=0), axis=1))]
    knots = np.linspace(0., distance[-1], math.ceil(distance[-1] / spacing) + 1)
    reference = np.column_stack([np.interp(knots, distance, col) for col in raw.T])
    twist = data["feedforward"][[first, last]]
    curvature = twist[:, :, -1] / twist[:, :, 0]
    return reference, first, last, curvature


def clearance_seed(scene, reference, *, smoothing=.55, dense=False, grid_region=None):
    """A geometric warm start through the return S, independent of the mode set."""
    from scipy.ndimage import gaussian_filter1d
    margin = scene.blade_site.config["planning_clearance_m"] + (.06 if grid_region is None else .008)
    road = scene.blade_site.road if grid_region is None else scene.blade_site.road.intersection(grid_region)
    safe = road.buffer(-margin).intersection(sg.box(29., 38., 96., 62.))
    columns = np.arange(30., 95., .025)
    verticals = np.stack((np.column_stack((columns, np.full_like(columns, 38.))),
                          np.column_stack((columns, np.full_like(columns, 62.)))), axis=1)
    sections = sg.bounds(sg.intersection(safe, sg.linestrings(verticals)))
    valid = np.isfinite(sections[:, 1])
    columns, sections = columns[valid], sections[valid]
    outline = np.array(scene.cargo.outline)
    boundary = []
    for a, b in zip(outline, np.roll(outline, -1, axis=0)):
        boundary.extend(np.linspace(a, b, max(1, math.ceil(np.linalg.norm(a - b) / .1)), endpoint=False))
    # Include all possible chassis yaws when constructing a geometric seed.
    t = np.arange(16) * math.tau / 16
    disk = np.column_stack((np.cos(t), np.sin(t))) * math.hypot(*scene.vehicle_size[:2]) / (2 * math.cos(math.pi / 16))
    body = np.vstack((boundary, *(disk + off for off in scene.offsets)))
    x = np.linspace(reference[0, 0], reference[-1, 0], int((reference[0, 0]-reference[-1, 0]) / .15) + 1)
    angles = np.deg2rad(np.arange(90., 220., .4))
    lower, upper = [], []
    for yaw in angles:
        c, s = math.cos(yaw), math.sin(yaw)
        query = x[:, None] + c * body[:, 0] - s * body[:, 1]
        dy = s * body[:, 0] + c * body[:, 1]
        lower.append((np.interp(query, columns, sections[:, 1]) - dy).max(axis=1))
        upper.append((np.interp(query, columns, sections[:, 3]) - dy).min(axis=1))
    lower, upper = np.array(lower).T, np.array(upper).T
    y, width = (lower + upper) / 2, upper - lower
    stage = 100 * np.minimum(width - .01, 0.)**2 + .3 / (.25 + np.maximum(width, 0.)) + .025*(angles-math.pi)**2
    cost = stage[0] + 20 * (angles-reference[0, 2])**2 + 4 * (y[0]-reference[0, 1])**2
    previous = []
    for i in range(1, len(x)):
        choices = np.full((11, len(angles)), np.inf)
        for jump in range(-5, 6):
            ids = np.arange(max(0, jump), min(len(angles), len(angles)+jump))
            choices[jump+5, ids] = cost[ids-jump] + .028 * jump**2 + 10 * (y[i, ids]-y[i-1, ids-jump])**2
        best = choices.argmin(axis=0)
        previous.append(best - 5)
        cost = stage[i] + choices[best, np.arange(len(angles))]
    last = np.argmin(cost + 20 * (angles-reference[-1, 2])**2 + 4 * (y[-1]-reference[-1, 1])**2)
    selected = [last]
    for jumps in reversed(previous):
        last -= jumps[last]
        selected.append(last)
    selected.reverse()
    middle = np.column_stack((x, y[np.arange(len(x)), selected], angles[selected]))
    if smoothing:
        middle[:, 1:] = gaussian_filter1d(middle[:, 1:], smoothing, axis=0)
    direction = np.column_stack((np.cos(middle[:, 2]), np.sin(middle[:, 2])))
    centers = middle[:, None, :2] + direction[:, None, :] * np.array(scene.offsets)[None, :, :1]
    tangent = np.gradient(centers, axis=0)
    yaws = np.unwrap(np.arctan2(tangent[:, :, 1], tangent[:, :, 0]), axis=0)
    yaws += math.tau * np.rint((reference[0, 3:] - yaws[0]) / math.tau)
    profile = np.column_stack((middle, yaws))
    if dense:
        reference = np.column_stack([np.interp(-x, -reference[:, 0], col) for col in reference.T])
    guess = np.column_stack([np.interp(-reference[:, 0], -x, col) for col in profile.T])
    blend = np.clip(np.minimum(abs(reference[:, 0]-x[0]), abs(reference[:, 0]-x[-1])) / 4., 0., 1.)
    return reference + blend[:, None] * (guess - reference)
