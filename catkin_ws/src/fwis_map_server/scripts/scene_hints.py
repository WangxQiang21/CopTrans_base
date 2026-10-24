#!/usr/bin/env python3
"""Publish RViz hint markers showing which goals exercise which modes.

These are static MarkerArray annotations laid on top of the map: a labeled
pin (numbered) for every "preset goal" plus a short caption telling the
user what mode the planner ought to pick to reach that goal.

A companion service ``goal_<N>`` is also offered: rostopic pub on
``/preset_goal`` with std_msgs/Int32 N to fire that goal at the planner.
"""
from __future__ import annotations

import os

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA, Int32
from visualization_msgs.msg import Marker, MarkerArray


# (x, y, theta, label, expected mode)
# Every scene here MUST have its source YAML in the corresponding
# maps/mapNN-*/scene_source.yaml directory so gen_map.py can build it and
# map_publisher.py can serve the generated occupancy map.
#
# S09 is the only scenario. All previous S01..S08/S10 have been removed
# per the thesis-scope decision: "one map done well" > "many demo maps".
#
# S09_underground_garage has 4 target types intentionally covering all four
# vehicle motion modes:
#   BOT_LEFT / BOT_MID / BOT_RIGHT (bottom row perpendicular slots)
#     - reachable by ACKERMANN back-in (long entry, curve into slot)
#     - Lateral shortcut possible from lane if lane width is tight
#   MID_A / MID_B (middle back-to-back rows) — Ackermann-friendly
#   TOP_LEFT (west-wall angled slot) — Diagonal-45 native fit
#   TOP_MID  (middle angled slot)   — Diagonal-45 native fit
#
# The mode label ("expected mode") is the *ideal* mode; the planner is free
# to choose others if geometry forces it. Test cases cover pure-mode goals
# and mixed-mode goals separately.
PRESETS = {
  "S09_underground_garage": [
    # --- Bottom row (perpendicular / horizontal) ---
    (0.740, 0.500, 1.5708, "BOT_LEFT (west end)\nREVERSE ACKERMANN", "A"),
    (5.860, 0.500, 1.5708, "BOT_MID (default)\nREVERSE ACKERMANN", "A"),
    (10.980, 0.500, 1.5708, "BOT_RIGHT (east end)\nREVERSE ACKERMANN", "A"),
    # --- Middle rows (back-to-back) ---
    (6.580, 4.000, 1.5708,  "MID_A / R2A (open north)\nREVERSE ACKERMANN", "A"),
    (2.700, 4.900, -1.5708, "MID_B / R2B (open south)\nREVERSE ACKERMANN", "A"),
    # --- Top row (45° angled slots) — Diagonal is the native fit ---
    (0.814, 9.214, -0.7854, "TOP_LEFT (west 45°)\nDIAGONAL BACK-IN", "P"),
    (6.014, 9.214, -0.7854, "TOP_MID  (middle 45°)\nDIAGONAL BACK-IN", "P"),
  ],
}

MODE_COLOR = {
  "A": ColorRGBA(0.20, 0.85, 0.30, 1.0),  # green  - Ackermann
  "L": ColorRGBA(1.00, 0.45, 0.05, 1.0),  # orange - Lateral
  "P": ColorRGBA(0.20, 0.55, 1.00, 1.0),  # blue   - Diagonal
  "S": ColorRGBA(0.85, 0.30, 0.95, 1.0),  # purple - Spin
  "M": ColorRGBA(0.95, 0.95, 0.30, 1.0),  # yellow - mixed
}


class SceneHints:
  def __init__(self):
    self.scene = rospy.get_param("~scene", "S09_underground_garage")
    self.presets = PRESETS.get(self.scene, [])
    self.pub = rospy.Publisher("/scene_hints", MarkerArray,
                               queue_size=1, latch=True)
    self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped,
                                    queue_size=1)
    self.sub = rospy.Subscriber("/preset_goal", Int32, self.on_preset)
    rospy.Timer(rospy.Duration(2.0), self.publish_once, oneshot=True)

  def publish_once(self, _evt):
    arr = MarkerArray()
    delete = Marker()
    delete.header.frame_id = "map"
    delete.header.stamp = rospy.Time.now()
    delete.ns = "scene_hints"
    delete.action = Marker.DELETEALL
    arr.markers.append(delete)

    for i, (x, y, _, label, mode_key) in enumerate(self.presets):
      color = MODE_COLOR.get(mode_key, ColorRGBA(1, 1, 1, 1))

      # Sphere pin
      m = Marker()
      m.header.frame_id = "map"
      m.header.stamp = rospy.Time.now()
      m.ns = "scene_hints"
      m.id = i * 2
      m.type = Marker.SPHERE
      m.action = Marker.ADD
      m.pose.position.x = x
      m.pose.position.y = y
      m.pose.position.z = 0.10
      m.pose.orientation.w = 1.0
      m.scale.x = m.scale.y = m.scale.z = 0.30
      m.color = color
      arr.markers.append(m)

      # Floating text label
      t = Marker()
      t.header.frame_id = "map"
      t.header.stamp = rospy.Time.now()
      t.ns = "scene_hints"
      t.id = i * 2 + 1
      t.type = Marker.TEXT_VIEW_FACING
      t.action = Marker.ADD
      t.pose.position.x = x
      t.pose.position.y = y
      t.pose.position.z = 0.85
      t.pose.orientation.w = 1.0
      t.scale.z = 0.30
      t.color = color
      t.text = f"#{i+1}\n{label}"
      arr.markers.append(t)
    self.pub.publish(arr)
    rospy.loginfo("[scene_hints] %d preset goals published for scene=%s",
                  len(self.presets), self.scene)

  def on_preset(self, msg: Int32):
    idx = int(msg.data) - 1
    if not (0 <= idx < len(self.presets)):
      rospy.logwarn("[scene_hints] no preset #%d", msg.data)
      return
    x, y, theta, label, _ = self.presets[idx]
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = rospy.Time.now()
    ps.pose.position.x = x
    ps.pose.position.y = y
    import math
    ps.pose.orientation.z = math.sin(theta * 0.5)
    ps.pose.orientation.w = math.cos(theta * 0.5)
    self.pub_goal.publish(ps)
    rospy.loginfo("[scene_hints] firing preset #%d (%.2f, %.2f) -> %s",
                  msg.data, x, y, label.replace("\n", " "))


def main():
  rospy.init_node("fwis_scene_hints")
  SceneHints()
  rospy.spin()


if __name__ == "__main__":
  main()
