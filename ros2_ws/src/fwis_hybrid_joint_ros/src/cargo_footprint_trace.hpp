#pragma once

#include "fwis_hybrid_joint/model.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <array>
#include <cmath>
#include <vector>

namespace hybrid_ros
{
inline constexpr char cargo_trace_topic[] = "/formation/cargo_footprint_trace";

inline visualization_msgs::msg::MarkerArray clear_cargo_trace()
{
  visualization_msgs::msg::MarkerArray message;
  auto & marker = message.markers.emplace_back();
  marker.header.frame_id = "map";
  marker.ns = "cargo_footprint_trace";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  return message;
}

inline visualization_msgs::msg::MarkerArray cargo_trace_markers(
  const std::vector<geometry_msgs::msg::Pose2D> & reference,
  const fwis_hybrid_joint::Footprint & footprint)
{
  auto message = clear_cargo_trace();
  if (reference.empty()) {
    return message;
  }
  auto & marker = message.markers.front();
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = .024;
  marker.color.r = .12f;
  marker.color.g = .56f;
  marker.color.b = .51f;
  marker.color.a = .74f;

  auto cargo_pose = [&](const geometry_msgs::msg::Pose2D & pose) {
      const auto center = fwis_planning_core::formation_center(
        {pose.x, pose.y, pose.theta}, {footprint[0], footprint[1]});
      auto result = pose;
      result.x = center.x;
      result.y = center.y;
      return result;
    };
  auto distance = [](const auto & a, const auto & b) {
      return std::hypot(b.x - a.x, b.y - a.y);
    };
  constexpr double pi = 3.14159265358979323846;
  auto heading_change = [](const auto & a, const auto & b) {
      return std::abs(std::remainder(b.theta - a.theta, 2 * pi));
    };

  std::vector<geometry_msgs::msg::Pose2D> footprints{cargo_pose(reference.front())};
  auto previous = footprints.front();
  double travelled = 0.0;
  for (std::size_t i = 1; i + 1 < reference.size(); ++i) {
    const auto center = cargo_pose(reference[i]);
    travelled += distance(previous, center);
    previous = center;
    const double yaw_change = heading_change(footprints.back(), center);
    // Along a straight aisle use distance alone. Near turns add outlines
    // for meaningful changes in cargo posture, including near a fixed pivot.
    if (travelled >= 2.5 || (travelled >= .8 && yaw_change >= pi / 9.) ||
      yaw_change >= pi / 5.)
    {
      footprints.push_back(center);
      travelled = 0.0;
    }
  }
  const auto goal = cargo_pose(reference.back());
  const double goal_distance = distance(footprints.back(), goal);
  const double goal_turn = heading_change(footprints.back(), goal);
  if (footprints.size() > 1 && goal_distance < .6 && goal_turn < pi / 18.) {
    footprints.back() = goal;
  } else if (goal_distance > 1e-6 || goal_turn > 1e-6) {
    footprints.push_back(goal);
  }

  const double half_length = footprint[2] / 2.;
  const double half_width = footprint[3] / 2.;
  marker.points.reserve(10 * footprints.size());
  for (const auto & pose : footprints) {
    const fwis_planning_core::Transform2d transform({pose.x, pose.y, pose.theta});
    auto point = [&](double x, double y) {
        const auto mapped = transform.apply({x, y});
        geometry_msgs::msg::Point corner;
        corner.x = mapped.x;
        corner.y = mapped.y;
        corner.z = .045;
        return corner;
      };
    const std::array corners{
      point(half_length, half_width), point(-half_length, half_width),
      point(-half_length, -half_width), point(half_length, -half_width)};
    for (std::size_t i = 0; i < corners.size(); ++i) {
      marker.points.push_back(corners[i]);
      marker.points.push_back(corners[(i + 1) % corners.size()]);
    }
    // A short nose tick distinguishes opposite headings of the same box.
    marker.points.push_back(point(half_length, 0.));
    marker.points.push_back(point(.78 * half_length, 0.));
  }
  return message;
}
}  // namespace hybrid_ros
