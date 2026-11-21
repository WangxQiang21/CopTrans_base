"""Convert formation display geometry into ROS MarkerArray messages."""
import math

from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def marker_array(specs, stamp):
    array = MarkerArray()
    for spec in specs:
        marker = Marker()
        marker.header.frame_id = spec.frame_id
        marker.header.stamp = stamp
        marker.ns = spec.namespace
        marker.id = int(spec.marker_id)
        marker.type = getattr(Marker, spec.marker_type)
        marker.action = getattr(Marker, spec.action)
        marker.pose.position = Point(*spec.position)
        marker.pose.orientation = Quaternion(
            z=math.sin(spec.yaw / 2.0), w=math.cos(spec.yaw / 2.0))
        marker.scale = Vector3(*spec.scale)
        marker.color = ColorRGBA(*spec.color)
        marker.points = [Point(*point) for point in spec.points]
        marker.colors = [ColorRGBA(*color) for color in spec.colors]
        marker.text = spec.text
        array.markers.append(marker)
    return array
