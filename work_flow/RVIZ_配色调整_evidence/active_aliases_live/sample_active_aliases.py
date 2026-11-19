#!/usr/bin/env python3
"""Record live /formation/path/*_active values and route metadata.

This is evidence-only: it subscribes to the already-running playback node and
never publishes planning, reset, or trajectory data.
"""
import hashlib
import json
import sys
from pathlib import Path as FilePath

import rospy
from nav_msgs.msg import Path as PathMessage
from std_msgs.msg import String as StringMessage
import json as _json

TOPICS = (
    "/formation/path/astar_active",
    "/formation/path/astar_raw_0",
    "/formation/path/astar_raw_1",
    "/formation/path/astar_raw_2",
    "/formation/path/portal_guide_active",
    "/formation/path/portal_guide_0",
    "/formation/path/portal_guide_1",
    "/formation/path/portal_guide_2",
)


def path_record(message):
    points = [[float(pose.pose.position.x), float(pose.pose.position.y)]
              for pose in message.poses]
    digest = hashlib.sha256(
        json.dumps(points, separators=(",", ":")).encode("utf-8")).hexdigest()
    return {
        "frame_id": message.header.frame_id,
        "stamp": [int(message.header.stamp.secs),
                  int(message.header.stamp.nsecs)],
        "point_count": len(points),
        "first": points[0] if points else None,
        "last": points[-1] if points else None,
        "points_sha256": digest,
    }


def main():
    output = FilePath(sys.argv[1]).resolve()
    rospy.init_node("rviz_active_alias_evidence", anonymous=True)
    records = {}
    for topic in TOPICS:
        try:
            message = rospy.wait_for_message(topic, PathMessage, timeout=5.0)
        except Exception as exc:  # noqa: BLE001 - evidence must record failure
            records[topic] = {"error": "{}: {}".format(
                type(exc).__name__, exc)}
        else:
            records[topic] = path_record(message)
    try:
        debug = rospy.wait_for_message(
            "/formation/planning_debug", StringMessage, timeout=5.0)
        document = _json.loads(debug.data)
        records["planning_debug"] = {
            "schema": document.get("schema"),
            "plan_id": document.get("plan_id"),
            "route_index": document.get("route_index"),
            "route_count": document.get("route_count"),
            "corridor_route_id": (document.get("corridor") or {}).get(
                "route_id"),
            "state": document.get("state"),
        }
    except Exception as exc:  # noqa: BLE001
        records["planning_debug"] = {"error": "{}: {}".format(
            type(exc).__name__, exc)}
    output.write_text(json.dumps(records, indent=2, sort_keys=True) + "\n")
    print("wrote", output)
    for topic, record in records.items():
        if isinstance(record, dict):
            print(topic, record.get("point_count", record.get("error")))


if __name__ == "__main__":
    main()
