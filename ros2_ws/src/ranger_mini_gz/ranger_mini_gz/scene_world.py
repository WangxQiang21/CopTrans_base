"""Write an SDFormat world with local, procedural industrial visuals."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


STYLE_COLORS = {
    "plain": (.55, .59, .60), "boundary": (.53, .55, .55),
    "cnc": (.74, .79, .80), "production": (.53, .61, .65),
    "cabinet": (.61, .65, .67), "column": (.57, .59, .56),
    "fence": (.93, .66, .19), "pallet": (.56, .39, .23),
    "rack": (.29, .43, .52), "container": (.39, .52, .61),
    "trailer": (.60, .37, .24), "steel": (.29, .33, .35),
    "bollard": (.95, .69, .15),
}
DARK = (.13, .19, .23)
METAL = (.42, .49, .52)
SAFETY = (.95, .67, .14)


def item(parent, tag, value=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if value is not None:
        child.text = value if isinstance(value, str) else numbers(value)
    return child


def numbers(values):
    return " ".join(repr(float(v)) for v in values)


def pose(x=0, y=0, z=0, yaw=0):
    return x, y, z, 0, 0, yaw


def static_link(world, name, placement):
    model = item(world, "model", name=name)
    item(model, "static", "true")
    item(model, "pose", placement)
    return item(model, "link", name="main")


def shape(link, name, kind, dimensions, placement=None, color=None, opacity=1):
    tag = "collision" if color is None else "visual"
    entity = item(link, tag, name=name)
    if placement is not None:
        item(entity, "pose", placement)
    geometry = item(item(entity, "geometry"), kind)
    if kind == "box":
        item(geometry, "size", dimensions)
    elif kind == "cylinder":
        item(geometry, "radius", dimensions[:1])
        item(geometry, "length", dimensions[1:])
    else:
        height, vertices = dimensions
        item(geometry, "height", (height,))
        for vertex in vertices:
            item(geometry, "point", vertex)
    if color is not None:
        material = item(entity, "material")
        item(material, "ambient", (*color, 1))
        item(material, "diffuse", (*color, 1))
        item(material, "specular", (.11, .13, .14, 1))
        if opacity < 1:
            item(entity, "transparency", (1 - opacity,))
            item(entity, "cast_shadows", "false")


def obstacle_model(world, obstacle):
    x, y, z, yaw = obstacle.pose
    link = static_link(world, obstacle.name, pose(x, y, z, yaw))
    color = obstacle.color or STYLE_COLORS[obstacle.style]
    if obstacle.kind == "polygon":
        placement = pose(z=-obstacle.height / 2)
        kind, dimensions = "polyline", (obstacle.height, obstacle.vertices)
    elif obstacle.kind == "cylinder":
        placement = None
        kind, dimensions = "cylinder", (obstacle.radius, obstacle.height)
    else:
        placement = None
        kind, dimensions = "box", obstacle.size
    if obstacle.collision:
        shape(link, "footprint", kind, dimensions, placement)
    if obstacle.style == "fence":
        fence_visuals(link, obstacle.size, color)
    else:
        shape(link, "body", kind, dimensions, placement, color)
        if obstacle.kind == "cylinder" and obstacle.style == "bollard":
            shape(link, "warning_band", "cylinder",
                  (obstacle.radius + .002, .14), pose(z=.12), DARK)
        if kind == "box":
            box_details(link, obstacle.style, obstacle.size, color)


def fence_visuals(link, dimensions, color):
    length, depth, height = dimensions
    rail = min(.045, depth)
    for i in range(3):
        z = -height / 2 + .20 + i * (height - .24) / 2
        shape(link, f"rail_{i}", "box", (length, depth, rail), pose(z=z), color)
    count = max(2, math.ceil(length / .23) + 1)
    for i in range(count):
        x = -length / 2 + rail / 2 + i * (length - rail) / (count - 1)
        width = rail if i in (0, count - 1) else rail / 3
        shape(link, f"upright_{i}", "box", (width, depth * .60, height), pose(x), color)


def box_details(link, style, dimensions, color):
    length, depth, height = dimensions

    def face(name, x, z, sx, sz, shade, side=-1, relief=.005):
        shape(link, name, "box", (sx, .012, sz), pose(x, side * (depth / 2 + relief), z), shade)

    if style in ("cnc", "cabinet", "production"):
        face("kickplate", 0, -.37 * height, length * .99, .21 * height, DARK)
        if style == "cnc":
            face("machine_door", -.04 * length, .06 * height, .59 * length, .60 * height, DARK)
            face("window", -.04 * length, .09 * height, .46 * length, .39 * height,
                 (.24, .38, .44), relief=.014)
            face("control", .36 * length, .22 * height, .15 * length, .20 * height, METAL)
            face("screen", .36 * length, .23 * height, .11 * length, .10 * height,
                 (.28, .53, .49), relief=.014)
        else:
            panels = max(2, math.ceil(length / 1.15))
            for i in range(1, panels):
                x = length * (i / panels - .5)
                face(f"seam_{i}", x, .10 * height, .018, .64 * height, DARK)
            for i in range(4):
                face(f"vent_{i}", .23 * length, (.18 + .037 * i) * height,
                     .22 * length, .013, METAL)
        face("warning", .37 * length, -.13 * height, .14 * length, .07 * height, SAFETY)
    elif style == "container":
        count = max(2, math.ceil(length / .48))
        ribs = tuple(c * .69 for c in color)
        for side in (-1, 1):
            for i in range(count):
                face(f"rib_{side}_{i}", length * ((i + .5) / count - .5),
                     0, .04, height * .87, ribs, side)
        for side in (-1, 1):
            for i in (-1, 1):
                shape(link, f"door_rod_{side}_{i}", "box", (.012, .045, height * .81),
                      pose(side * (length / 2 + .006), i * depth / 4, 0), METAL)
    elif style in ("trailer", "steel", "rack", "pallet"):
        rail = (.69, .55, .36) if style in ("trailer", "pallet") else METAL
        bands = 3 if style == "steel" else 2
        for side in (-1, 1):
            for i in range(bands):
                face(f"band_{side}_{i}", 0, height * ((i + 1) / (bands + 1) - .5),
                     .98 * length, .046, rail, side)
        if style == "trailer":
            shape(link, "deck", "box", (.98 * length, .96 * depth, .020),
                  pose(z=height / 2 + .010), rail)
            for i in range(4):
                shape(link, f"tire_{i}", "cylinder", (.23, .11),
                      (length * (.18 + .10 * (i // 2)), (-1)**i * (depth / 2 + .014),
                       -.26 * height, math.pi / 2, 0, 0), DARK)
    elif style == "column" and height >= 1.25:
        for i in range(3):
            face(f"protective_band_{i}", 0, (-.40 + i * .05) * height,
                 .98 * length, .041, SAFETY if i % 2 == 0 else DARK)


def floor_markings(world, scene):
    lane = static_link(world, "route_paint", pose())
    experiment = scene.experiments["formation"]
    points = (experiment["start"], *experiment["waypoints"], experiment["goal"])
    line_index = 0
    for a, b in zip(points, points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < .05:
            continue
        yaw = math.atan2(dy, dx)
        for offset in (-1.42, 1.42):
            x = (a[0] + b[0]) / 2 - offset * math.sin(yaw)
            y = (a[1] + b[1]) / 2 + offset * math.cos(yaw)
            shape(lane, f"route_{line_index}", "box", (length - .025, .038, .002),
                  pose(x, y, .005, yaw), SAFETY)
            line_index += 1
    for name, point, color in (("start", experiment["start"], (.21, .63, .57)),
                               ("goal", experiment["goal"], (.32, .65, .76))):
        x, y, yaw = point
        zone = static_link(world, f"{name}_paint", pose(x, y, .008, yaw))
        for i in (-1, 1):
            shape(zone, f"side_{i}", "box", (3.1, .035, .002), pose(.50, i * 1.54), color)
            shape(zone, f"end_{i}", "box", (.035, 3.08, .002), pose(.50 + i * 1.55), color)


def cargo_model(world, scene):
    cargo = scene.cargo
    reference = scene.experiments["formation"]["start"]
    x, y, yaw = reference
    link = static_link(world, cargo.name, pose(x, y, cargo.bottom, yaw))
    length, width, height = cargo.size
    dx, dy = cargo.offset
    center = pose(dx, dy, height / 2)
    shape(link, "payload_obb", "box", cargo.size, center)
    shape(link, "machine_cover", "box", cargo.size, center, (.36, .56, .61), cargo.opacity)
    # Keep the skid rails open visually so the three centre-mounted supports remain visible.
    for side in (-1, 1):
        shape(link, f"skid_rail_{side}", "box", (length, .065, .055),
              pose(dx, dy + side * (width - .065) / 2, .028), METAL)
        shape(link, f"top_rail_{side}", "box", (length, .045, .045),
              pose(dx, dy + side * (width - .050) / 2, height - .023), METAL)
        for end in (-1, 1):
            shape(link, f"corner_{side}_{end}", "box", (.070, .070, height),
                  pose(dx + end * (length - .070) / 2, dy + side * (width - .070) / 2,
                       height / 2), METAL)
    shape(link, "service_hatch", "box", (.82, .57, .025),
          pose(dx - .12, dy, height - .013), (.47, .54, .56))
    for index, (rx, ry) in zip(scene.vehicle_ids, scene.offsets):
        shape(link, f"centre_mount_{index}", "cylinder",
              (cargo.mount_diameter / 2, cargo.mount_height),
              pose(rx, ry, -cargo.mount_height / 2), (.72, .75, .77))


def write_world(scene, path, task, *, placement=None, blade_plant=None, blade_body_yaws=None,
                blade_real_time_factor=1.):
    sdf = ET.Element("sdf", version="1.9")
    world = item(sdf, "world", name=scene.name)
    physics = item(world, "physics", name="default", type="ignored")
    item(physics, "max_step_size", "0.002")
    item(physics, "real_time_factor", str(blade_real_time_factor) if blade_plant else "1")
    for filename, system in (("physics", "Physics"), ("user-commands", "UserCommands"),
                             ("scene-broadcaster", "SceneBroadcaster")):
        item(world, "plugin", filename=f"gz-sim-{filename}-system", name=f"gz::sim::systems::{system}")
    environment = item(world, "scene")
    item(environment, "ambient", (.47, .50, .53, 1))
    item(environment, "background", (.63, .70, .75, 1))
    item(environment, "grid", "false")
    light = item(world, "light", name="sun", type="directional")
    item(light, "pose", pose(z=12))
    item(light, "cast_shadows", "true")
    item(light, "diffuse", (.84, .83, .78, 1))
    item(light, "specular", (.35, .35, .35, 1))
    item(light, "direction", (-.24, .12, -.96) if task == "blade_pair" else (-.5, .29, -.82))
    if task == "blade_pair":
        from ranger_mini_gz.blade_world import write_blade_environment
        item(light, "visualize", "false")
        checks = write_blade_environment(world, scene, path.parent, placement, blade_plant, blade_body_yaws)
        ET.indent(sdf, space="  ")
        ET.ElementTree(sdf).write(path, encoding="utf-8", xml_declaration=True)
        return checks
    w, h = scene.size
    ground = static_link(world, "ground", pose(w / 2, h / 2, -.075))
    shape(ground, "surface_collision", "box", (w, h, .15))
    shape(ground, "surface_visual", "box", (w, h, .15), color=scene.ground_color)
    for obstacle in scene.obstacles:
        obstacle_model(world, obstacle)
    floor_markings(world, scene)
    if task == "formation":
        cargo_model(world, scene)
    ET.indent(sdf, space="  ")
    ET.ElementTree(sdf).write(path, encoding="utf-8", xml_declaration=True)


def write_gui_config(scene, template, path, *, eye=None, target=None, hfov=None):
    eye = scene.camera_eye if eye is None else eye
    target = scene.camera_target if target is None else target
    camera_delta = np.array(target) - eye
    yaw = math.atan2(camera_delta[1], camera_delta[0]) if any(camera_delta[:2]) else math.pi / 2
    pitch = math.atan2(-camera_delta[2], math.hypot(*camera_delta[:2]))
    content = Path(template).read_text().split("?>", 1)[-1]
    root = ET.fromstring("<gui>" + content + "</gui>")
    root.find("window/width").text = "1260"
    root.find("window/height").text = "820"
    root.find("plugin[@filename='MinimalScene']/camera_pose").text = numbers(
        (*eye, 0, pitch, yaw))
    if hfov is not None:
        # MinimalScene accepts degrees; the SDF camera sensors accept radians.
        item(root.find("plugin[@filename='MinimalScene']"), "horizontal_fov", (math.degrees(hfov),))
    for plugin in root.findall("plugin[@filename='EntityTree']"):
        root.remove(plugin)
    ET.indent(root, space="  ")
    path.write_text('<?xml version="1.0"?>\n' +
                    "".join(ET.tostring(element, encoding="unicode") for element in root))
