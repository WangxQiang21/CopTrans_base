#!/usr/bin/env python3
"""Publish painted parking-slot outlines as RViz markers.

The OccupancyGrid only carries free/occupied cells, so painted slot lines
must live on a visualization layer instead of in the map. This node reads
``parking_lines:`` from the scene yaml (one entry per axis-aligned rect)
and publishes a latched MarkerArray on /parking_lines.

Scene yaml schema (each entry):
  parking_lines:
    - {rect: [x0, y0, x1, y1], label: "L0 SLOT", color: white}

color is one of: white | gray | orange | red | green | blue | yellow.
label is optional; if present it is rendered as a TEXT_VIEW_FACING above
the rectangle.
"""
from __future__ import annotations

import os

import rospy
import yaml
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


COLORS = {
  "white":  ColorRGBA(1.00, 1.00, 1.00, 1.0),
  "gray":   ColorRGBA(0.55, 0.55, 0.55, 1.0),
  "orange": ColorRGBA(1.00, 0.55, 0.10, 1.0),
  "red":    ColorRGBA(0.95, 0.20, 0.20, 1.0),
  "green":  ColorRGBA(0.25, 0.85, 0.30, 1.0),
  "blue":   ColorRGBA(0.20, 0.55, 1.00, 1.0),
  "yellow": ColorRGBA(0.95, 0.95, 0.30, 1.0),
}


def _resolve_scene_yaml(scene_yaml: str, scene: str, scenarios_dir: str) -> str:
  """Resolve the map-local scene source, with a legacy named-file fallback."""
  if scene_yaml and os.path.isfile(scene_yaml):
    return scene_yaml
  candidate = os.path.join(scenarios_dir, f"{scene}.yaml")
  if os.path.isfile(candidate):
    return candidate
  candidate = os.path.join(scenarios_dir, "scene_source.yaml")
  if os.path.isfile(candidate):
    return candidate
  return ""


def _polygon_marker(idx, points, color: ColorRGBA, line_w: float,
                    dashed: bool = False,
                    dash_len: float = 0.10, gap_len: float = 0.06) -> Marker:
  """Polygon outline (for angled/rotated slot markings).

  Uses the same dashed style as _rect_marker but with arbitrary corner
  points, so angled slots get angled outlines instead of axis-aligned
  bounding boxes.
  """
  m = Marker()
  m.header.frame_id = "map"
  m.header.stamp = rospy.Time.now()
  m.ns = "parking_lines"
  m.id = idx
  m.action = Marker.ADD
  m.pose.orientation.w = 1.0
  m.scale.x = line_w
  m.color = color

  pts = [(float(p[0]), float(p[1])) for p in points]
  pts.append(pts[0])  # close the loop

  if not dashed:
    m.type = Marker.LINE_STRIP
    for x, y in pts:
      p = Point(); p.x = x; p.y = y; p.z = 0.01
      m.points.append(p)
    return m

  m.type = Marker.LINE_LIST
  period = dash_len + gap_len
  for i in range(len(pts) - 1):
    sx, sy = pts[i]
    ex, ey = pts[i + 1]
    seg_len = ((ex - sx) ** 2 + (ey - sy) ** 2) ** 0.5
    if seg_len < 1e-6:
      continue
    ux, uy = (ex - sx) / seg_len, (ey - sy) / seg_len
    n = max(1, int(round(seg_len / period)))
    actual_period = seg_len / n
    dl = actual_period * (dash_len / period)
    for k in range(n):
      t0 = k * actual_period
      t1 = t0 + dl
      pa = Point(); pa.x = sx + ux * t0; pa.y = sy + uy * t0; pa.z = 0.01
      pb = Point(); pb.x = sx + ux * t1; pb.y = sy + uy * t1; pb.z = 0.01
      m.points.append(pa); m.points.append(pb)
  return m


def _rect_marker(idx: int, rect, color: ColorRGBA, line_w: float,
                 dashed: bool = False,
                 dash_len: float = 0.10,
                 gap_len: float = 0.06) -> Marker:
  """Axis-aligned rectangle outline.

  Solid rects (roads, curbs, booth) use LINE_STRIP for one continuous loop.
  Dashed rects (parking slot outlines) use LINE_LIST with segments emitted
  at (dash_len, gap_len) spacing along each of the 4 sides.
  """
  x0, y0, x1, y1 = float(rect[0]), float(rect[1]), float(rect[2]), float(rect[3])
  m = Marker()
  m.header.frame_id = "map"
  m.header.stamp = rospy.Time.now()
  m.ns = "parking_lines"
  m.id = idx
  m.action = Marker.ADD
  m.pose.orientation.w = 1.0
  m.scale.x = line_w
  m.color = color

  corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]

  if not dashed:
    m.type = Marker.LINE_STRIP
    for x, y in corners:
      p = Point(); p.x = x; p.y = y; p.z = 0.01
      m.points.append(p)
    return m

  m.type = Marker.LINE_LIST
  period = dash_len + gap_len
  for i in range(4):
    sx, sy = corners[i]
    ex, ey = corners[i + 1]
    seg_len = ((ex - sx) ** 2 + (ey - sy) ** 2) ** 0.5
    if seg_len < 1e-6:
      continue
    ux, uy = (ex - sx) / seg_len, (ey - sy) / seg_len
    # Fit an integer number of dashes centred on this side so both ends
    # start with a dash (looks cleaner at slot corners).
    n = max(1, int(round(seg_len / period)))
    actual_period = seg_len / n
    dl = actual_period * (dash_len / period)
    for k in range(n):
      t0 = k * actual_period
      t1 = t0 + dl
      x_a = sx + ux * t0; y_a = sy + uy * t0
      x_b = sx + ux * t1; y_b = sy + uy * t1
      pa = Point(); pa.x = x_a; pa.y = y_a; pa.z = 0.01
      pb = Point(); pb.x = x_b; pb.y = y_b; pb.z = 0.01
      m.points.append(pa); m.points.append(pb)
  return m


def _label_marker(idx: int, rect, label: str, color: ColorRGBA,
                  text_h: float) -> Marker:
  x0, y0, x1, y1 = rect
  m = Marker()
  m.header.frame_id = "map"
  m.header.stamp = rospy.Time.now()
  m.ns = "parking_lines"
  m.id = idx
  m.type = Marker.TEXT_VIEW_FACING
  m.action = Marker.ADD
  m.pose.position.x = 0.5 * (x0 + x1)
  m.pose.position.y = 0.5 * (y0 + y1)
  m.pose.position.z = max(y1 - y0, x1 - x0) * 0.4 + 0.05
  m.pose.orientation.w = 1.0
  m.scale.z = text_h
  m.color = color
  m.text = label
  return m


class ParkingLines:
  def __init__(self):
    scene = rospy.get_param("~scene", "")
    scene_yaml = rospy.get_param("~scene_yaml", "")
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../"))
    scenarios_dir = rospy.get_param(
        "~scenarios_dir",
        os.path.join(project_root, "maps", "map16-s09-underground-garage"))
    self.line_width = float(rospy.get_param("~line_width", 0.02))
    self.text_height = float(rospy.get_param("~text_height", 0.10))

    path = _resolve_scene_yaml(scene_yaml, scene, scenarios_dir)
    self.entries = []
    if not path:
      rospy.logwarn("[parking_lines] no scene yaml found "
                    "(scene=%s yaml=%s) -> publishing empty array",
                    scene, scene_yaml)
    else:
      with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
      self.entries = data.get("parking_lines", []) or []
      rospy.loginfo("[parking_lines] loaded %d entries from %s",
                    len(self.entries), path)

    self.pub = rospy.Publisher("/parking_lines", MarkerArray,
                               queue_size=1, latch=True)
    rospy.Timer(rospy.Duration(2.0), self._publish_once, oneshot=True)

  def _publish_once(self, _evt):
    arr = MarkerArray()
    delete = Marker()
    delete.header.frame_id = "map"
    delete.header.stamp = rospy.Time.now()
    delete.ns = "parking_lines"
    delete.action = Marker.DELETEALL
    arr.markers.append(delete)

    next_id = 0
    for entry in self.entries:
      color_name = str(entry.get("color", "white")).lower()
      color = COLORS.get(color_name, COLORS["white"])
      slot_colors = {"white", "red", "orange"}
      default_dashed = color_name in slot_colors
      dashed = bool(entry.get("dashed", default_dashed))

      # Support polygon outlines (for angled/rotated slots) in addition
      # to axis-aligned rects.
      poly = entry.get("polygon")
      rect = entry.get("rect")

      if poly and len(poly) >= 3:
        arr.markers.append(_polygon_marker(next_id, poly, color,
                                            self.line_width, dashed=dashed))
        next_id += 1
      elif rect and len(rect) == 4:
        arr.markers.append(_rect_marker(next_id, rect, color,
                                        self.line_width, dashed=dashed))
        next_id += 1
      else:
        rospy.logwarn("[parking_lines] skipping malformed entry: %s", entry)
        continue

      label = entry.get("label")
      if label:
        # For label position, use centroid of rect or polygon
        if rect:
          x0, y0, x1, y1 = rect
          cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        else:
          xs = [p[0] for p in poly]; ys = [p[1] for p in poly]
          cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        arr.markers.append(_label_marker(next_id, [cx - 0.2, cy - 0.2,
                                                    cx + 0.2, cy + 0.2],
                                         str(label), color, self.text_height))
        next_id += 1
    self.pub.publish(arr)


def main():
  rospy.init_node("fwis_parking_lines")
  ParkingLines()
  rospy.spin()


if __name__ == "__main__":
  main()
