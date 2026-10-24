#!/usr/bin/env python3
"""Publish scene_meta's vehicle-center goal after the ROS demo is ready."""
import math
from pathlib import Path

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid


def main():
    rospy.init_node("fwis_v3_auto_goal")
    map_dir = Path(rospy.get_param("~map_dir"))
    scene_meta = map_dir / "scene_meta.yaml"
    with scene_meta.open("r", encoding="utf-8") as stream:
        goal = yaml.safe_load(stream)["goal"]

    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    rospy.wait_for_message("/map", OccupancyGrid)
    rospy.wait_for_message("/start_pose", PoseStamped)
    rate = rospy.Rate(20)
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rate.sleep()
    if rospy.is_shutdown():
        return
    # Give the planner a short callback cycle to consume its latched map/start.
    rospy.sleep(0.5)
    theta = float(goal["theta"])
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = float(goal["x"])
    msg.pose.position.y = float(goal["y"])
    msg.pose.orientation.z = math.sin(theta / 2)
    msg.pose.orientation.w = math.cos(theta / 2)
    pub.publish(msg)
    rospy.loginfo("[v3 auto goal] sent center goal (%.3f, %.3f, %.3f)",
                  msg.pose.position.x, msg.pose.position.y, theta)
    rospy.sleep(0.5)


if __name__ == "__main__":
    main()
