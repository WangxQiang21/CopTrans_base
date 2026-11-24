"""Convert formation display geometry into ROS MarkerArray messages."""
from collections import defaultdict
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Quaternion, Vector3
from scipy.spatial.transform import Rotation
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import xacro


def _origin(element):
    origin = element.find("origin")
    if origin is None:
        return (0.0, 0.0, 0.0), Rotation.identity()
    xyz = tuple(map(float, origin.get("xyz", "0 0 0").split()))
    rpy = tuple(map(float, origin.get("rpy", "0 0 0").split()))
    return xyz, Rotation.from_euler("xyz", rpy)


def _urdf_meshes(robot):
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    children = defaultdict(list)
    for joint in robot.findall("joint[@type='fixed']"):
        children[joint.find("parent").attrib["link"]].append(joint)

    def visit(name, position, orientation):
        for visual in links[name].findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is not None:
                xyz, rotation = _origin(visual)
                scale = tuple(map(float, mesh.get("scale", "1 1 1").split()))
                yield mesh.attrib["filename"], position + orientation.apply(xyz), orientation * rotation, scale
        for joint in children[name]:
            xyz, rotation = _origin(joint)
            yield from visit(joint.find("child").attrib["link"],
                             position + orientation.apply(xyz), orientation * rotation)

    return tuple(visit("base_link", (0.0, 0.0, 0.0), Rotation.identity()))


def load_ranger_meshes():
    """Read the painted Ranger visuals and fixed wheel transforms once at startup."""
    folder = Path(get_package_share_directory("ranger_mini_description"))
    path = str(folder / "urdf" / "ranger_mini.urdf.xacro")
    models = {}
    for vehicle, color in enumerate(("white", "orange", "blue"), 1):
        robot = xacro.process_file(path, mappings={
            "shell_color": color, "mesh_uri": "package://ranger_mini_description/meshes/painted",
        })
        models[f"vehicle_{vehicle}"] = _urdf_meshes(ET.fromstring(robot.toxml()))
    return models


def _vehicle_markers(spec, stamp, meshes):
    # Align the tire mesh bottom with the map plane, as in the Gazebo spawn pose.
    base_height = 0.322
    position = (spec.position[0], spec.position[1], base_height)
    orientation = Rotation.from_euler("z", spec.yaw)
    for index, (uri, xyz, rotation, scale) in enumerate(meshes):
        marker = Marker()
        marker.header.frame_id = spec.frame_id
        marker.header.stamp = stamp
        marker.ns = spec.namespace
        marker.id = -1 - index  # Preserve the IDs of heading, path and text markers.
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = uri
        marker.mesh_use_embedded_materials = True
        marker.pose.position = point_message(position + orientation.apply(xyz))
        x, y, z, w = map(float, (orientation * rotation).as_quat())
        marker.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
        marker.scale = Vector3(x=scale[0], y=scale[1], z=scale[2])
        yield marker


def point_message(values):
    x, y, z = map(float, values)
    return Point(x=x, y=y, z=z)


def color_message(values):
    red, green, blue, alpha = map(float, values)
    return ColorRGBA(r=red, g=green, b=blue, a=alpha)


def marker_array(specs, stamp, vehicle_meshes=None):
    array = MarkerArray()
    for spec in specs:
        meshes = vehicle_meshes.get(spec.namespace.rsplit("/", 1)[-1]) if vehicle_meshes else None
        if meshes and spec.action == "ADD" and spec.marker_type == "CUBE":
            array.markers.extend(_vehicle_markers(spec, stamp, meshes))
            continue
        marker = Marker()
        marker.header.frame_id = spec.frame_id
        marker.header.stamp = stamp
        # RViz limits a namespaced DELETEALL to that exact namespace.
        marker.ns = "" if spec.action == "DELETEALL" else spec.namespace
        marker.id = int(spec.marker_id)
        marker.type = getattr(Marker, spec.marker_type)
        marker.action = getattr(Marker, spec.action)
        marker.pose.position = point_message(spec.position)
        marker.pose.orientation = Quaternion(
            z=math.sin(spec.yaw / 2.0), w=math.cos(spec.yaw / 2.0))
        marker.scale = Vector3(x=float(spec.scale[0]), y=float(spec.scale[1]), z=float(spec.scale[2]))
        marker.color = color_message(spec.color)
        marker.points = [point_message(point) for point in spec.points]
        marker.colors = [color_message(color) for color in spec.colors]
        marker.text = spec.text
        if meshes and marker.type == Marker.TEXT_VIEW_FACING:
            marker.text = spec.text.partition(" | ")[0]
            marker.scale.z *= 0.8
            marker.color = color_message((0.92, 0.95, 0.98, 1.0))
        array.markers.append(marker)
    return array
