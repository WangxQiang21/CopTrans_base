#pragma once
#include "fwis_hybrid_joint/model.hpp"
#include <fwis_v3_msgs/msg/formation_plan_request.hpp>
#include <fwis_v3_msgs/msg/formation_playback_plan.hpp>
#include <fwis_v3_msgs/msg/timed_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <json/json.h>

namespace hybrid_ros
{
namespace hj = fwis_hybrid_joint;
using Envelope = fwis_v3_msgs::msg::FormationPlaybackPlan;
using Formation = fwis_v3_msgs::msg::FormationPlanRequest;
using Timed = fwis_v3_msgs::msg::TimedTrajectory;

struct Context
{
  hj::Request request;
  uint32_t id{};
  std::string formation_id;
  std::vector<uint32_t> ids;
};
struct Decoded { Context context; hj::Plan plan; };
struct Settings
{
  explicit Settings(rclcpp::Node & node);
  bool multi;
  std::string profile;
  double margin, length, width, lead, budget;
  hj::Request base;
};
std::shared_ptr<const hj::GridSnapshot> grid_from(const nav_msgs::msg::OccupancyGrid &);
hj::Pose2d pose_from(const geometry_msgs::msg::Pose &);
Context formation_context(const Formation &, const Settings &, std::shared_ptr<const hj::GridSnapshot>);
Context single_context(hj::Pose2d, hj::Pose2d, uint32_t, const Settings &,
  std::shared_ptr<const hj::GridSnapshot>);
std::string build_digest();
std::string digest(const std::string &);
Json::Value parse(const std::string &);
std::string json(const Json::Value &);
Json::Value contract(const Context &);
Envelope encode(const Context &, const hj::Plan &, int64_t epoch);
Envelope encode(const Context &, const hj::Plan &, int64_t epoch, int64_t publication_steady_ns, int64_t publication_ros_ns);
Decoded decode(const Envelope &, std::shared_ptr<const hj::GridSnapshot>, const Context * expected = nullptr);
void display_samples(Envelope &, const Context &, const hj::Plan &);
geometry_msgs::msg::Pose2D pose2(hj::Pose2d);
inline int64_t ns(const builtin_interfaces::msg::Time & s)
{
  return int64_t(s.sec) * 1000000000 + s.nanosec;
}
inline int64_t ns(const builtin_interfaces::msg::Duration & s)
{
  return int64_t(s.sec) * 1000000000 + s.nanosec;
}
inline rclcpp::QoS durable()
{
  return rclcpp::QoS(1).transient_local();
}
constexpr const char * schema = "HybridTimedPlan/v2";
constexpr const char * envelope_topic = "/hybrid_joint/plan";
constexpr const char * permit_topic = "/hybrid_joint/execution_permit";
}
