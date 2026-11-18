#!/usr/bin/env python3
"""Record every /formation/corridors MarkerArray seen by an independent node."""
import argparse
import json

import rospy
from visualization_msgs.msg import Marker, MarkerArray

ACTION_NAMES = {
    Marker.ADD: "ADD",
    Marker.DELETE: "DELETE",
    Marker.DELETEALL: "DELETEALL",
}
TYPE_NAMES = {
    getattr(Marker, name): name
    for name in ("TRIANGLE_LIST", "LINE_STRIP", "ARROW", "TEXT_VIEW_FACING")
}


def summarize(message):
    markers = []
    for marker in message.markers:
        z_values = [point.z for point in marker.points]
        markers.append({
            "namespace": marker.ns,
            "id": marker.id,
            "type": TYPE_NAMES.get(marker.type, str(marker.type)),
            "action": ACTION_NAMES.get(marker.action, str(marker.action)),
            "points": len(marker.points),
            "z_min": min(z_values) if z_values else None,
            "z_max": max(z_values) if z_values else None,
            "color": [marker.color.r, marker.color.g, marker.color.b,
                      marker.color.a],
            "text": marker.text,
            "frame_id": marker.header.frame_id,
        })
    return markers


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=180.0)
    args = parser.parse_args()
    rospy.init_node("v1_corridor_recorder", anonymous=True)
    output = open(args.output, "w", encoding="utf-8")

    def callback(message):
        record = {
            "wall_time": rospy.get_time(),
            "marker_count": len(message.markers),
            "markers": summarize(message),
        }
        output.write(json.dumps(record, sort_keys=True) + "\n")
        output.flush()

    rospy.Subscriber("/formation/corridors", MarkerArray, callback,
                     queue_size=10)
    rospy.sleep(args.duration)
    output.close()


if __name__ == "__main__":
    main()