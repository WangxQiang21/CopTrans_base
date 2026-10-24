#!/usr/bin/env python3
"""Publish a pre-generated map.pgm/map.yaml as nav_msgs/OccupancyGrid (latched).

Standalone replacement for ros-noetic-map-server (which is not installed on
this machine) that also republishes scene_meta.yaml's start pose on
/start_pose so RViz and the planner see the initial vehicle pose.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import rospy
import tf2_ros
import yaml
from PIL import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid


def yaw_to_quat(yaw: float):
  half = yaw * 0.5
  return (0.0, 0.0, math.sin(half), math.cos(half))


def load_map(map_dir: str):
  with open(os.path.join(map_dir, "map.yaml"), "r") as f:
    meta = yaml.safe_load(f)
  pgm_path = os.path.join(map_dir, meta["image"])
  img = np.asarray(Image.open(pgm_path).convert("L"))
  # ROS map_server flips the image vertically when loading.
  img = np.flipud(img)
  res = float(meta["resolution"])
  ox, oy = float(meta["origin"][0]), float(meta["origin"][1])

  occupied_thresh = float(meta.get("occupied_thresh", 0.65)) * 255
  free_thresh     = float(meta.get("free_thresh", 0.196)) * 255

  occ = np.full(img.shape, -1, dtype=np.int8)
  occ[img <= 255 - occupied_thresh] = 100  # dark = occupied
  occ[img >= 255 - free_thresh] = 0        # bright = free

  msg = OccupancyGrid()
  msg.header.frame_id = "map"
  msg.info.resolution = res
  msg.info.width  = img.shape[1]
  msg.info.height = img.shape[0]
  msg.info.origin.position.x = ox
  msg.info.origin.position.y = oy
  msg.info.origin.orientation.w = 1.0
  msg.data = occ.flatten().tolist()
  return msg


def load_scene_meta(map_dir: str):
  path = os.path.join(map_dir, "scene_meta.yaml")
  if not os.path.exists(path):
    return None
  with open(path, "r") as f:
    return yaml.safe_load(f)


def main() -> None:
  rospy.init_node("fwis_map_publisher")
  map_dir = rospy.get_param("~map_dir", "")
  if not map_dir:
    rospy.logfatal("~map_dir is required (directory containing map.pgm + map.yaml)")
    sys.exit(1)

  grid = load_map(map_dir)
  meta = load_scene_meta(map_dir)

  pub_map   = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
  pub_start = rospy.Publisher("/start_pose", PoseStamped, queue_size=1, latch=True)

  grid.header.stamp = rospy.Time.now()
  pub_map.publish(grid)
  rospy.loginfo("[fwis_map_publisher] /map published: %dx%d, res=%.3f m",
                grid.info.width, grid.info.height, grid.info.resolution)

  br = tf2_ros.StaticTransformBroadcaster()
  if meta is not None and "start" in meta:
    s = meta["start"]
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = rospy.Time.now()
    ps.pose.position.x = float(s["x"])
    ps.pose.position.y = float(s["y"])
    qx, qy, qz, qw = yaw_to_quat(float(s.get("theta", 0.0)))
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    pub_start.publish(ps)

    # Static TF map -> base_link at the start pose so RViz can show the robot.
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "base_link"
    t.transform.translation.x = ps.pose.position.x
    t.transform.translation.y = ps.pose.position.y
    t.transform.rotation = ps.pose.orientation
    br.sendTransform(t)
    rospy.loginfo("[fwis_map_publisher] start pose latched at (%.2f, %.2f, %.2f rad)",
                  ps.pose.position.x, ps.pose.position.y, float(s.get("theta", 0.0)))

  rospy.spin()


if __name__ == "__main__":
  main()
