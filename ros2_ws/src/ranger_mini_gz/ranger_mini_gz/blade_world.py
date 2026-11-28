"""Gazebo environment meshes and static mounted pair for the three blade sites."""

from functools import lru_cache
import math
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import shapely as sg
from shapely.affinity import rotate
from shapely.ops import substring

from ranger_mini_gz.blade_mesh import Mesh, blade_mesh, materials, polygons
from ranger_mini_gz.model_launch import SPAWN_HEIGHT
from ranger_mini_gz.scene_world import item, numbers, pose, shape, static_link


def camera_views(scene, placement):
    cx, cy, yaw = scene.experiments["blade_pair"][placement]
    w, h = scene.size
    # At the straight ends, look down the road at both supports. The bends
    # have individual sightlines that clear the inner cliff or street corner.
    supports = {
        "eye": [cx-10.5*math.cos(yaw)+2.6*math.sin(yaw), cy-10.5*math.sin(yaw)-2.6*math.cos(yaw), 1.8],
        "target": [cx, cy, .5], "hfov": 1.34}
    if placement in ("key_pose", "second_bend"):
        key = "key_supports" if placement == "key_pose" else "second_bend_supports"
        supports = scene.blade_site.config["camera"][key]
    return {
        "overview": {"eye": list(scene.camera_eye), "target": list(scene.camera_target), "hfov": 1.32},
        "top": {"eye": [w/2, h/2, max(w*.85, h*1.47)], "target": [w/2, h/2, 0.], "hfov": 1.32},
        "detail": {"eye": [cx+2., cy-8., 24.], "target": [cx, cy, .3], "hfov": 1.18},
        "supports": supports,
    }


def mesh_entity(link, name, uri, collision):
    kind = "collision" if collision else "visual"
    entity = item(link, kind, name=f"{name}_{kind}")
    item(item(item(entity, "geometry"), "mesh"), "uri", uri)
    return entity


def tiles(bounds, frontage, depth):
    x0, y0, x1, y1 = bounds
    xx = np.linspace(x0, x1, max(1, round((x1 - x0) / frontage)) + 1)
    yy = np.linspace(y0, y1, max(1, round((y1 - y0) / depth)) + 1)
    for x, end_x in zip(xx, xx[1:]):
        for y, end_y in zip(yy, yy[1:]):
            yield sg.box(x, y, end_x, end_y)


def mountain(site, bounds):
    settings = site.config["terrain"]
    setback = settings["min_rock_xy_setback_from_road_m"] + .004
    land = bounds.difference(site.road.buffer(setback, quad_segs=32))
    cut_lo, cut_hi = settings["near_road_cut_height_m"]
    ridge_lo, ridge_hi = settings["ridge_height_m"]
    peak_lo, peak_hi = settings["background_peak_height_m"]

    @lru_cache(maxsize=None)
    def height(x, y):
        gap = max(0., site.road.distance(sg.Point(x, y)) - setback)
        rock = .5 + .5 * math.sin(.37*x + .19*y) * math.cos(.29*y - .08*x)
        cut = cut_lo + (cut_hi - cut_lo)*rock
        ridge = ridge_lo + (ridge_hi - ridge_lo)*(.45 + .38*math.sin(.085*x + .16*y) + .16*rock)
        peak = peak_lo + (peak_hi - peak_lo)*(.44 + .39*math.cos(.12*x - .17*y) + .16*rock)
        shoulder = min(gap / 5.2, 1.)
        outer = min(max(gap - 5.2, 0.) / 13., 1.)
        return cut + shoulder*(ridge-cut) + outer*(peak-ridge)

    terrain = Mesh()
    for tile in tiles(bounds.bounds, 1.4, 1.4):
        for p in polygons(land.intersection(tile)):
            x, y = p.centroid.coords[0]
            layer = .6*math.sin(.12*x + .35*y) + .4*math.sin(.36*x - .1*y)
            color = "rock" + str(min(3, int((1 + layer)*2)))
            terrain.surface(p, height, color)
    # Each exposed edge of the triangulated top needs one wall, even where a
    # curved road boundary was split across terrain tiles.
    vertices = np.array(list(terrain.vertices))
    faces = np.concatenate(list(terrain.faces.values())) - 1
    edges = faces[:, ((0, 1), (1, 2), (2, 0))].reshape(-1, 2)
    _, first, counts = np.unique(np.sort(edges, axis=1), axis=0, return_index=True, return_counts=True)
    rim = vertices[edges[first[counts == 1]]]
    if abs(np.linalg.norm(rim[:, 1, :2] - rim[:, 0, :2], axis=1).sum() - land.length) > .001:
        raise ValueError("rock mesh has an open seam inside the terrain")
    for a, b in rim:
        left, right = a[:2], b[:2]
        a_mid, b_mid = (*left, a[2]*.58), (*right, b[2]*.58)
        terrain.face([(*left, -.12), (*right, -.12), b_mid, a_mid], "rock_cut")
        terrain.face([a_mid, b_mid, b, a], "rock_seam")
    rail = Mesh()
    rail_height = settings["outer_guardrail_height_m"]
    side = site.centreline.offset_curve(-(site.config["road"]["width_m"]/2 +
                                          settings["min_guardrail_xy_setback_from_road_m"] + .065), quad_segs=64)
    for begin, end in ((26., 63.), (107., 142.), (161., 191.)):
        line = substring(side, begin, end)
        if "extra_polygons" in site.config["road"]:
            # Bends with paved overhang relief can cross the nominal verge.
            # Trim those rail sections outside the actual collision region.
            line = line.difference(site.road.buffer(settings["min_guardrail_xy_setback_from_road_m"] + .052))
        for segment in sg.get_parts(line):
            rail.prism(segment.buffer(.035, cap_style="flat"), rail_height-.14, rail_height, "steel")
            for station in np.arange(.25, segment.length, 2.4):
                p = segment.interpolate(station)
                rail.prism(sg.box(p.x-.03, p.y-.03, p.x+.03, p.y+.03), 0., rail_height-.07, "steel")
    return terrain, rail


def facade(mesh, a, b, height, material, windows):
    def panel(left, right, bottom, top, color):
        start = a + (b-a)*left
        end = a + (b-a)*right
        mesh.face([(*start, bottom), (*end, bottom), (*end, top), (*start, top)], color)

    width = np.linalg.norm(b-a)
    if not windows or width < 2.1:
        panel(0., 1., 0., height, material)
        return
    bays = max(1, round(width / 2.6))
    floors = max(1, int(height / 2.7))
    for floor in range(floors):
        base, ceiling = height*floor/floors, height*(floor+1)/floors
        for j in range(bays):
            left, right = j/bays, (j+1)/bays
            door = floor == 0 and j == bays//2
            lower, upper = (base if door else base+.8), min(ceiling-.24, base+2.25)
            inner_l, inner_r = left+.21/bays, right-.21/bays
            panel(left, inner_l, base, ceiling, material)
            panel(inner_r, right, base, ceiling, material)
            if lower > base:
                panel(inner_l, inner_r, base, lower, material)
            panel(inner_l, inner_r, lower, upper, "door" if door else "glass")
            panel(inner_l, inner_r, upper, ceiling, material)


def house(mesh, poly, road, index, height):
    p = sg.orient_polygons(poly)
    xmin, ymin, xmax, ymax = p.bounds
    ridge_x = (xmax-xmin) <= (ymax-ymin)
    low, high = (xmin, xmax) if ridge_x else (ymin, ymax)
    middle = (low+high)/2
    eave = height - min(.95, .14*(high-low))
    wall = ("plaster", "stone", "cream", "brick", "plaster")[index % 5]
    roof = ("slate", "tile", "zinc", "brown_tile", "slate", "zinc")[index % 6]

    def top(x, y):
        c = x if ridge_x else y
        return height - (height-eave)*abs(c-middle)/max(1e-9, (high-low)/2)

    for ring in (p.exterior, *p.interiors):
        coords = np.array(ring.coords)
        for a, b in zip(coords, coords[1:]):
            street = road.distance(sg.Point((a+b)/2)) < 2.
            facade(mesh, a, b, eave, wall, street)
            ci = 0 if ridge_x else 1
            cuts = [a, b]
            if (a[ci]-middle)*(b[ci]-middle) < 0:
                cuts.insert(1, a+(b-a)*((middle-a[ci])/(b[ci]-a[ci])))
            for start, end in zip(cuts, cuts[1:]):
                mesh.face([(*start, eave), (*end, eave), (*end, top(*end)), (*start, top(*start))], wall)
    first = sg.box(xmin-1, ymin-1, middle if ridge_x else xmax+1, ymax+1 if ridge_x else middle)
    for part in (p.intersection(first), p.difference(first)):
        for face in polygons(part):
            mesh.surface(face, top, roof)


def buildings(site, bounds):
    specs = site.config["buildings"]
    protected = site.road if site.island is None else site.road.union(site.island)
    # Leave the flares, overhangs and roof projections clear of the road as well.
    available = bounds.difference(protected.buffer(specs["min_footprint_setback_m"]+.002, quad_segs=64))
    width, depth, gap = (specs[key] for key in ("width_m", "depth_m", "gap_m"))
    heights = specs["height_options_m"]
    center = tuple(site.config["road"].get("center", (0, 0)))
    result = Mesh()
    footprints = []
    for angle in specs.get("rotations_deg", (0,)):
        for row in specs["rows"]:
            a, b = np.asarray(row).reshape(2, 2)
            length = np.linalg.norm(b-a)
            count = int((length + gap) / (width + gap))
            if not count:
                continue
            along = (b-a) / length
            across = depth * np.array((-along[1], along[0]))
            margin = (length - count*width - (count-1)*gap) / 2
            for index in range(count):
                p = a + (margin + index*(width+gap))*along
                q = p + width*along
                lot = rotate(sg.Polygon([p, q, q+across, p+across]), angle, origin=center)
                for footprint in polygons(lot.intersection(available)):
                    if footprint.area < .65*width*depth:
                        continue
                    i = len(footprints)
                    house(result, footprint, site.road, i, heights[i % len(heights)])
                    footprints.append(footprint)
    return result, footprints


def add_pair(world, scene, directory, placement, plant=None, body_yaws=None):
    import xacro
    from ament_index_python.packages import get_package_share_directory
    description = Path(get_package_share_directory("ranger_mini_description"))
    transport = item(world, "model", name="blade_transport")
    item(transport, "static", "true")
    item(transport, "pose", pose(placement[0], placement[1], yaw=placement[2]))
    payload = item(transport, "link", name=scene.cargo.name)
    item(payload, "pose", pose(z=scene.cargo.bottom))
    blade = blade_mesh(scene.cargo.outline, scene.cargo.size[2])
    blade.write(directory, "blade")
    for collision in (False, True):
        mesh_entity(payload, "rigid_blade", "assets/blade.obj", collision)
    if plant is not None:
        plugin = item(world, "plugin", filename="gz-sim-python-system-loader-system",
                      name="gz::sim::systems::PythonSystemLoader")
        item(plugin, "module_name", "ranger_mini_gz.blade_system")
        item(plugin, "settings", str(Path(plant).resolve()))
        item(plugin, "mount_offsets", (*scene.offsets[0], *scene.offsets[1]))
    headings = (placement[2], placement[2]) if body_yaws is None else body_yaws
    for index, offset, color, heading in zip(scene.vehicle_ids, scene.offsets, ("white", "orange"),
                                              headings, strict=True):
        mount = pose(*offset, z=-scene.cargo.mount_height/2)
        dims = (scene.cargo.mount_diameter/2, scene.cargo.mount_height)
        shape(payload, f"mount{index}", "cylinder", dims, mount, (.54, .56, .58))
        shape(payload, f"mount{index}_collision", "cylinder", dims, mount)
        urdf = xacro.process_file(str(description / "urdf/ranger_mini.urdf.xacro"), mappings={
            "shell_color": color, "mesh_uri": "model://ranger_mini_description/meshes/painted"})
        with tempfile.NamedTemporaryFile(suffix=".urdf", mode="w") as temp:
            temp.write(urdf.toxml())
            temp.flush()
            robot = ET.fromstring(subprocess.check_output(["gz", "sdf", "-p", temp.name])).find("model")
        robot.set("name", f"robot{index}")
        for plugin in robot.findall("plugin"):
            robot.remove(plugin)
        if plant is None:
            item(robot, "pose", pose(*offset, z=SPAWN_HEIGHT))
            transport.append(robot)
        else:
            c, s = math.cos(placement[2]), math.sin(placement[2])
            dx, dy = offset
            item(robot, "pose", pose(placement[0] + c*dx - s*dy, placement[1] + s*dx + c*dy,
                                     z=SPAWN_HEIGHT, yaw=heading))
            world.append(robot)


def write_blade_environment(world, scene, output, placement, plant=None, body_yaws=None):
    site = scene.blade_site
    assets = output / "assets"
    assets.mkdir(exist_ok=True)
    materials(assets)
    bounds = sg.box(0, 0, *scene.size)
    checks = {}

    def install(name, mesh, *, solid=True, minimum_setback=None):
        mesh.write(assets, name)
        link = static_link(world, name, pose())
        uri = f"assets/{name}.obj"
        mesh_entity(link, "surface", uri, False)
        if solid:
            mesh_entity(link, "surface", uri, True)
        if minimum_setback is not None:
            xy = np.array(list(mesh.vertices))[:, :2]
            indices = np.concatenate(list(mesh.faces.values())) - 1
            # Vertical faces project to lines. Their edges also participate in the check.
            triangles = sg.polygons(xy[indices])
            tree = sg.STRtree(triangles)
            _, distances = tree.query_nearest(site.road, return_distance=True, all_matches=False)
            gap = float(distances.min())
            if gap < minimum_setback - 1e-6:
                raise ValueError(f"{name} XY setback {gap:.6f} m < {minimum_setback} m")
            checks[name] = dict(minimum_xy_setback_m=gap, required_m=minimum_setback,
                                exported_triangles=len(indices), visual_equals_collision=solid)

    pavement = Mesh()
    pavement.prism(site.road, -.12, 0., "asphalt")
    install("pavement", pavement)
    paint = Mesh()
    edge = site.road.buffer(-.12).difference(site.road.buffer(-.23))
    for piece in polygons(edge):
        paint.surface(piece, lambda x, y: .006, "white_line")
    if site.centreline is not None:
        for s in np.arange(2., site.centreline.length-2., 8.):
            for p in polygons(substring(site.centreline, s, s+2.4).buffer(.058, cap_style="flat")):
                paint.surface(p, lambda x, y: .007, "yellow_line")
    install("road_markings", paint, solid=False)
    verge = Mesh()
    if site.centreline is not None:
        terrain, rail = mountain(site, bounds)
        required = site.config["terrain"]
        install("rock_massif", terrain, minimum_setback=required["min_rock_xy_setback_from_road_m"])
        install("outer_guardrail", rail, minimum_setback=required["min_guardrail_xy_setback_from_road_m"])
        verge.prism(site.road.buffer(.42).difference(site.road), -.12, .045, "kerb")
    else:
        curb = bounds.difference(site.road)
        if site.island is not None:
            curb = curb.difference(site.island)
            island = Mesh()
            island.prism(site.island, -.12, .11, "kerb")
            island.prism(site.island.buffer(-.20), .11, .15, "grass")
            cx, cy = site.config["road"]["center"]
            for j in range(13):
                a = j*2.4
                r = 1.5 + j*.32
                shrub = sg.Point(cx+r*math.cos(a), cy+r*math.sin(a)).buffer(.45+.08*(j%3), quad_segs=5)
                crown = shrub.centroid
                ring = np.array(sg.orient_polygons(shrub).exterior.coords)
                inner = np.array(crown.coords[0]) + .69*(ring - crown.coords[0])
                for k, (a, b) in enumerate(zip(ring, ring[1:])):
                    z = .43 + .07*(j%3)
                    island.face([(*a, .15), (*b, .15), (*inner[k+1], z), (*inner[k], z)], "shrub")
                    island.face([(*inner[k], z), (*inner[k+1], z),
                                 (crown.x, crown.y, z+.16)], "shrub")
            install("central_island", island)
        near_road = curb.intersection(site.road.buffer(.28))
        verge.prism(near_road, -.12, .07, "paving")
        verge.prism(curb.difference(near_road), -.12, .0, "grass")
        houses, footprints = buildings(site, bounds)
        install("street_blocks", houses, minimum_setback=site.config["buildings"]["min_footprint_setback_m"])
        checks["street_blocks"].update(houses=len(footprints),
                                       footprint_area_m2=sg.union_all(footprints).area)
    install("roadside", verge)
    add_pair(world, scene, assets, placement, plant, body_yaws)
    return checks
