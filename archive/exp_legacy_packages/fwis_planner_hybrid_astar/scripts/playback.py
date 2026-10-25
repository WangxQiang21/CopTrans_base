#!/usr/bin/env python3
"""Vehicle playback for fwis Hybrid A* output.

Subscribes ``/plan/trajectory`` (fwis_msgs/Trajectory) and animates the
vehicle along it.  Speed is mode-dependent and the vehicle stops for
``dwell`` seconds whenever the next pose changes mode (so it's obvious
in RViz which mode is active).

The vehicle Marker uses ``ns=robot, id=0`` and is the only publisher on
that namespace - the planner node no longer publishes a competing cube,
so no flicker.

Visualization style is inspired by Panda1224/official_planner.
"""
from __future__ import annotations

import math
import threading
from typing import List, Tuple

import rospy
from geometry_msgs.msg import (Point, Pose2D, PoseStamped,
                               PoseWithCovarianceStamped)
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from fwis_msgs.msg import Trajectory


def yaw_from_quat(q):
  s = 2.0 * (q.w * q.z + q.x * q.y)
  c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  return math.atan2(s, c)


MODE_NAME = {0: "Ackermann", 1: "Lateral", 2: "Diagonal", 3: "Spin"}
MODE_COLOR = {  # RGBA (0..1)
  0: (0.20, 0.85, 0.30, 0.92),  # green   = Ackermann
  1: (1.00, 0.45, 0.05, 0.92),  # orange  = Lateral
  2: (0.20, 0.55, 1.00, 0.92),  # blue    = Diagonal
  3: (0.85, 0.30, 0.95, 0.92),  # purple  = Spin
}


def yaw_to_quat(yaw: float):
  return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class Playback:
  def __init__(self):
    # ----- params (yaml-loaded under the node's private namespace) -----
    self.v_ack = float(rospy.get_param("~playback/v_ackermann", 0.30))
    self.v_lat = float(rospy.get_param("~playback/v_lateral",   0.18))
    self.v_dia = float(rospy.get_param("~playback/v_diagonal",  0.22))
    self.w_spin = float(rospy.get_param("~playback/w_spin",     0.6))  # rad/s
    rospy.loginfo("[playback] speeds: v_ack=%.3f v_lat=%.3f v_dia=%.3f w_spin=%.3f",
                  self.v_ack, self.v_lat, self.v_dia, self.w_spin)
    self.dwell = float(rospy.get_param("~playback/mode_dwell",  0.5))
    self.fps   = float(rospy.get_param("~playback/fps",         30.0))
    self.loop  = bool(rospy.get_param("~playback/loop",         False))

    self.length = float(rospy.get_param("~vehicle/length", 0.738))
    self.width  = float(rospy.get_param("~vehicle/width",  0.500))
    self.height = float(rospy.get_param("~vehicle/height", 0.338))
    self.mesh_resource = rospy.get_param("~vehicle/mesh", "")

    # ----- state -----
    self.lock = threading.Lock()
    self.poses: List[Tuple[float, float, float, int]] = []  # (x,y,yaw,mode)
    self.idx = 0
    self.frac = 0.0           # progress in [0,1) within the [idx, idx+1] segment
    self.dwell_t_left = 0.0   # seconds remaining in current dwell
    # Static pose used when no trajectory has been received yet.
    self.start_xy_yaw = None  # (x, y, yaw) or None

    self.sub  = rospy.Subscriber("/plan/trajectory_optimized", Trajectory,
                                 self.on_traj, queue_size=1)
    self.sub_start = rospy.Subscriber("/start_pose", PoseStamped,
                                      self.on_start, queue_size=1)
    self.sub_init  = rospy.Subscriber("/initialpose",
                                      PoseWithCovarianceStamped,
                                      self.on_init, queue_size=1)
    self.pub_robot = rospy.Publisher("/robot/marker", Marker,
                                      queue_size=1, latch=True)
    self.pub_hud   = rospy.Publisher("/robot/hud", MarkerArray,
                                      queue_size=1, latch=True)
    self.pub_corridors = rospy.Publisher("/robot/corridor_markers", MarkerArray,
                                          queue_size=1, latch=True)
    # 启动时清除 RViz 残留走廊
    del_arr = MarkerArray()
    dm = Marker()
    dm.action = Marker.DELETEALL
    del_arr.markers.append(dm)
    self.pub_corridors.publish(del_arr)

    self._corridor_markers: List[Marker] = []  # cached from planner
    self._corridor_centers: List[Tuple[float,float]] = []
    self._activated_corridors = set()  # 已经"点亮"的走廊 ID，不消失
    rospy.Subscriber("/plan/corridor_markers", MarkerArray,
                     self.on_corridors, queue_size=1)

  # ----- callbacks -----
  def on_traj(self, msg: Trajectory):
    pts = []
    n = min(len(msg.pose), len(msg.mode))
    for i in range(n):
      p: Pose2D = msg.pose[i]
      pts.append((p.x, p.y, p.theta, int(msg.mode[i])))
    with self.lock:
      # Empty plan = planner failed; keep showing the last known start pose
      # instead of disappearing. Also reset playback state.
      if pts:
        self.poses = pts
        self.idx = 0
        self.frac = 0.0
        self.dwell_t_left = 0.0
        self._activated_corridors.clear()
      else:
        self.poses = []
    rospy.loginfo("[playback] received trajectory: %d poses", len(pts))

  def on_corridors(self, msg: MarkerArray):
    # 新走廊到达 → 先清除 RViz 中上一轮的旧走廊
    del_arr = MarkerArray()
    dm = Marker()
    dm.action = Marker.DELETEALL
    del_arr.markers.append(dm)
    self.pub_corridors.publish(del_arr)
    markers = []
    centers = []
    for m in msg.markers:
      if m.action != Marker.ADD or len(m.points) < 4:
        continue
      cx = sum(p.x for p in m.points[:4]) / 4.0
      cy = sum(p.y for p in m.points[:4]) / 4.0
      markers.append(m)
      centers.append((cx, cy))
    self._corridor_markers = markers
    self._corridor_centers = centers
    self._activated_corridors.clear()

  def on_start(self, msg: PoseStamped):
    self.start_xy_yaw = (msg.pose.position.x,
                         msg.pose.position.y,
                         yaw_from_quat(msg.pose.orientation))

  def on_init(self, msg: PoseWithCovarianceStamped):
    self.start_xy_yaw = (msg.pose.pose.position.x,
                         msg.pose.pose.position.y,
                         yaw_from_quat(msg.pose.pose.orientation))
    # User just reset the start - clear stale trajectory so the vehicle stays
    # at the new pose until a fresh plan arrives.
    with self.lock:
      self.poses = []
      self.idx = 0
      self.frac = 0.0
      self.dwell_t_left = 0.0

  # ----- main loop -----
  def run(self):
    dt = 1.0 / max(self.fps, 1.0)
    rate = rospy.Rate(self.fps)
    while not rospy.is_shutdown():
      self.tick(dt)
      rate.sleep()

  def tick(self, dt: float):
    with self.lock:
      if not self.poses:
        # Idle: display vehicle at the latest start pose so the user can see
        # where they are even before a successful plan exists.
        if self.start_xy_yaw is not None:
          x, y, yaw = self.start_xy_yaw
          self._publish(x, y, yaw, mode=0, v=0.0, dwell=False)
        return

      # End of trajectory: park (or restart if loop)
      if self.idx >= len(self.poses) - 1 and self.frac >= 1.0:
        if self.loop:
          self.idx = 0
          self.frac = 0.0
        else:
          x, y, yaw, m = self.poses[-1]
          self._publish(x, y, yaw, m, v=0.0, dwell=False)
          return

      # Hold position while dwelling on a mode boundary
      if self.dwell_t_left > 0.0:
        x, y, yaw, m = self.poses[self.idx]
        self.dwell_t_left -= dt
        # mode shown is the *current* (post-switch) one
        self._publish(x, y, yaw, m, v=0.0, dwell=True)
        return

      # Travel along the trajectory for `dt` seconds. We may cross several
      # primitive boundaries within the SAME mode, but always stop to dwell
      # at a mode change so the user can see the switch.
      #
      # Each segment (idx -> idx+1) is either:
      #   - a PIVOT: both endpoints are Spin with the same (x,y), only yaw
      #     changes -> advance by w_spin (rad/s)
      #   - a TRANSLATION: position changes -> advance by the linear speed
      #     of the mode at idx (v_ack / v_lat / v_dia)
      # The old code used cur_m (set once at tick start) to pick the segment
      # type, which broke when a Spin segment was immediately followed by a
      # Diagonal segment with the same yaw: seg_len was computed as |dyaw|=0
      # and the playback snapped to the Diagonal endpoint in one frame.
      time_left = dt
      while time_left > 0 and self.idx + 1 < len(self.poses):
        x0, y0, yaw0, m0 = self.poses[self.idx]
        x1, y1, yaw1, m1 = self.poses[self.idx + 1]

        # If we've finished the current segment and the next is a different
        # mode, fire a dwell and stop.
        if self.frac >= 1.0 and m1 != m0:
          self.dwell_t_left = self.dwell
          self.idx += 1
          self.frac = 0.0
          x, y, yaw, m = self.poses[self.idx]
          self._publish(x, y, yaw, m, v=0.0, dwell=True)
          return

        # Classify the segment.
        is_pivot = (m0 == 3 and m1 == 3
                    and abs(x1 - x0) < 1e-6 and abs(y1 - y0) < 1e-6)
        if is_pivot:
          seg_len = abs(math.atan2(math.sin(yaw1 - yaw0),
                                   math.cos(yaw1 - yaw0)))
          speed = self.w_spin  # rad/s
        else:
          seg_len = math.hypot(x1 - x0, y1 - y0)
          speed = self._v_for_mode(m0)  # m/s
        if seg_len < 1e-6:
          self.idx += 1
          self.frac = 0.0
          continue

        # Distance/angular distance left in this segment.
        ds_left = (1.0 - self.frac) * seg_len
        adv = speed * time_left  # how far we can go in this tick
        if adv >= ds_left - 1e-9:
          # Finish this segment, carry leftover time into the next iteration.
          time_left -= ds_left / max(speed, 1e-9)
          self.idx += 1
          self.frac = 0.0
          # If the new idx is a different mode, dwell on the next tick.
          if self.idx < len(self.poses) and self.poses[self.idx][3] != m0:
            x, y, yaw, _ = self.poses[self.idx]
            self.dwell_t_left = self.dwell
            self._publish(x, y, yaw, self.poses[self.idx][3],
                          v=0.0, dwell=True)
            return
        else:
          self.frac += adv / seg_len
          time_left = 0.0

      # Render current (idx, frac).
      if self.idx < len(self.poses):
        cur_m = self.poses[self.idx][3]
        speed = self.w_spin if (cur_m == 3 and self.idx + 1 < len(self.poses)
                                and self.poses[self.idx + 1][3] == 3
                                and abs(self.poses[self.idx + 1][0] - self.poses[self.idx][0]) < 1e-6
                                and abs(self.poses[self.idx + 1][1] - self.poses[self.idx][1]) < 1e-6
                               ) else self._v_for_mode(cur_m)
        self._render_current(cur_m, speed)

  def _render_current(self, mode: int, vmax: float):
    if not self.poses:
      return
    if self.idx >= len(self.poses) - 1:
      x, y, yaw, m = self.poses[-1]
      self._publish(x, y, yaw, m, v=0.0, dwell=False)
      return
    x0, y0, yaw0, m0 = self.poses[self.idx]
    x1, y1, yaw1, _  = self.poses[self.idx + 1]
    f = self.frac
    x = x0 + (x1 - x0) * f
    y = y0 + (y1 - y0) * f
    dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
    yaw = yaw0 + dyaw * f
    self._publish(x, y, yaw, m0, v=vmax, dwell=False)

  def _v_for_mode(self, m: int) -> float:
    return {0: self.v_ack, 1: self.v_lat, 2: self.v_dia}.get(m, self.v_ack)

  # ----- visualization -----
  def _publish(self, x, y, yaw, mode, v, dwell):
    self._publish_vehicle(x, y, yaw, mode, dwell)
    self._publish_hud(x, y, yaw, mode, v, dwell)
    self._publish_corridors(x, y)

  def _publish_corridors(self, x, y):
    """逐帧累加走廊: 车经过的走廊不消失, 新到的逐步点亮"""
    if not self._corridor_markers:
      return
    for i, (cx, cy) in enumerate(self._corridor_centers):
      if math.hypot(cx - x, cy - y) < 3.0:
        self._activated_corridors.add(i)
    arr = MarkerArray()
    for i in sorted(self._activated_corridors):
      if i >= len(self._corridor_markers):
        continue  # 新 trajectory 走廊变少, 跳过旧 index
      m = self._corridor_markers[i]
      m.header.stamp = rospy.Time.now()
      m.ns = "corridor_playback"
      m.id = i
      arr.markers.append(m)
    self.pub_corridors.publish(arr)

  def _publish_vehicle(self, x, y, yaw, mode, dwell):
    qx, qy, qz, qw = yaw_to_quat(yaw)
    if self.mesh_resource:
      m = Marker()
      m.header.frame_id = "map"
      m.header.stamp = rospy.Time.now()
      m.ns = "robot"
      m.id = 0
      m.type = Marker.MESH_RESOURCE
      m.action = Marker.ADD
      m.mesh_resource = self.mesh_resource
      m.mesh_use_embedded_materials = False
      m.scale.x = m.scale.y = m.scale.z = 1.0
      m.pose.position.x = x
      m.pose.position.y = y
      m.pose.position.z = 0.0
      m.pose.orientation.x, m.pose.orientation.y = qx, qy
      m.pose.orientation.z, m.pose.orientation.w = qz, qw
      r, g, b, a = MODE_COLOR.get(mode, (0.9, 0.9, 0.9, 0.9))
      if dwell:
        a = 0.55  # fade slightly while dwelling
      m.color = ColorRGBA(r, g, b, a)
    else:
      m = Marker()
      m.header.frame_id = "map"
      m.header.stamp = rospy.Time.now()
      m.ns = "robot"
      m.id = 0
      m.type = Marker.CUBE
      m.action = Marker.ADD
      m.pose.position.x = x
      m.pose.position.y = y
      m.pose.position.z = self.height * 0.5
      m.pose.orientation.x, m.pose.orientation.y = qx, qy
      m.pose.orientation.z, m.pose.orientation.w = qz, qw
      m.scale.x = self.length
      m.scale.y = self.width
      m.scale.z = self.height
      r, g, b, a = MODE_COLOR.get(mode, (0.9, 0.9, 0.9, 0.9))
      if dwell:
        a = 0.55
      m.color = ColorRGBA(r, g, b, a)
    self.pub_robot.publish(m)

  def _publish_hud(self, x, y, yaw, mode, v, dwell):
    arr = MarkerArray()
    # Big text floating above the vehicle
    t = Marker()
    t.header.frame_id = "map"
    t.header.stamp = rospy.Time.now()
    t.ns = "hud"
    t.id = 0
    t.type = Marker.TEXT_VIEW_FACING
    t.action = Marker.ADD
    t.pose.position.x = x
    t.pose.position.y = y
    t.pose.position.z = self.height + 0.7
    t.pose.orientation.w = 1.0
    t.scale.z = 0.45
    r, g, b, _ = MODE_COLOR.get(mode, (1.0, 1.0, 1.0, 1.0))
    t.color = ColorRGBA(r, g, b, 1.0)
    if mode == 3:
      label = f"{MODE_NAME[3]}  ω={v:.2f} rad/s"
    else:
      label = f"{MODE_NAME.get(mode, '?')}  v={v:.2f} m/s"
    if dwell:
      label += f"   [SWITCH {max(self.dwell_t_left, 0):.1f}s]"
    t.text = label
    arr.markers.append(t)

    # An arrow pointing along yaw, color matches mode
    a = Marker()
    a.header.frame_id = "map"
    a.header.stamp = rospy.Time.now()
    a.ns = "hud"
    a.id = 1
    a.type = Marker.ARROW
    a.action = Marker.ADD
    a.scale.x = 0.7
    a.scale.y = 0.12
    a.scale.z = 0.12
    a.color = ColorRGBA(r, g, b, 0.95)
    a.pose.position.x = x
    a.pose.position.y = y
    a.pose.position.z = self.height + 0.05
    qx, qy, qz, qw = yaw_to_quat(yaw)
    a.pose.orientation.x, a.pose.orientation.y = qx, qy
    a.pose.orientation.z, a.pose.orientation.w = qz, qw
    arr.markers.append(a)

    self.pub_hud.publish(arr)


def main():
  rospy.init_node("fwis_playback")
  pb = Playback()
  pb.run()


if __name__ == "__main__":
  main()
