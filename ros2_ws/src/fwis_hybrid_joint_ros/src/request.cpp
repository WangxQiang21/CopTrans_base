#include "fwis_hybrid_joint_ros/transport.hpp"
#include <cmath>
#include <limits>
namespace hybrid_ros
{
namespace
{
hj::Pose2d pose(const geometry_msgs::msg::Pose2D & p)
{
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.theta)) {
    throw std::invalid_argument("nonfinite pose");
  }
  return {p.x, p.y, p.theta};
}
}
Settings::Settings(rclcpp::Node & n)
{
  const auto task = n.declare_parameter("task", std::string("single"));
  if (task != "single" && task != "formation") {
    throw std::invalid_argument("unknown task");
  }
  multi = task == "formation";
  profile = n.declare_parameter("speed_profile", std::string("resolved"));
  auto number = [&](const char * key, double value) {return n.declare_parameter(key, value);};
  auto motion = [&](const char * single, const char * formation, double value) {
      return number(
        (std::string(
          multi ? "timing_limits." : "quality.limits.") + (multi ? formation : single)).c_str(), value);
    };
  margin = number(multi ? "collision_margin" : "vehicle.safety_margin", multi ? .05 : .04);
  length = number("vehicle.length", .738);
  width = number("vehicle.width", .50);
  lead = number("lead_s", 1.25);
  budget = number("solve_budget_s", multi ? 12. : 8.);
  base.task_duration_cap = number("task_duration_cap_s", 300.);
  auto & l = base.limits;
  l.radius = number(multi ? "timing_limits.r_min_ackermann" : "vehicle.r_min_ack", .90);
  l.speed = motion("v_max", "v_max", .16);
  l.acceleration = motion("a_max", "a_max", .22);
  l.jerk = motion("jerk_max", "jerk_max", multi ? 2. : .8);
  l.lateral_acceleration = motion("lateral_a_max", "lateral_accel_max", multi ? .6 : 2.);
  l.spin_speed = motion("omega_max", "yaw_rate_max", .40);
  l.spin_acceleration = motion("alpha_max", "yaw_accel_max", .45);
  l.spin_jerk = motion("angular_jerk_max", "yaw_jerk_max", multi ? 3. : 1.);
  l.yaw_rate = multi ? l.spin_speed : motion("translation_omega_max", "", .40);
  l.yaw_acceleration = multi ? l.spin_acceleration : motion("translation_alpha_max", "", .45);
  l.yaw_jerk = multi ? l.spin_jerk : motion("translation_angular_jerk_max", "", 1.);
  l.wheelbase = number("vehicle.wheelbase", .494);
  l.track = number("vehicle.track", .364);
  l.wheel_speed = motion("wheel_speed_max", "wheel_speed_max", multi ? 1. : 3.);
  l.steer_angle = motion("steer_angle_max", "steer_angle_max", M_PI_2);
  l.steer_rate = motion("steer_rate_max", "steer_rate_max", .8);
  l.steer_acceleration = motion("steer_accel_max", "steer_accel_max", 1.5);
  l.cargo_center_speed = motion("cargo_center_speed", "cargo_center_speed", 0.);
  l.high_steer_angle = motion("high_steer_angle", "high_steer_angle", .3490658503988659);
  l.high_steer_speed = motion("high_steer_speed", "high_steer_speed", 0.);
  l.min_motion_duration = motion("min_mode_dwell_s", "min_mode_dwell_s", .30);
  l.prepare_time = number(multi ? "timing_limits.mode_switch_duration" : "vehicle.mode_switch_duration", .70);
  if (multi) {
    base.align_terminal_chassis = n.declare_parameter("align_terminal_chassis", false);
    base.cargo = hj::Footprint{number("cargo_footprint.offset_x_m", .5), number(
        "cargo_footprint.offset_y_m",
        0.),
      number("cargo_footprint.length_m", 1.95) + 2 * margin,
      number("cargo_footprint.width_m", 2.70) + 2 * margin};
  }
  if (!(lead > 0 && budget > 0 && margin >= 0 && length > 0 && width > 0 && base.task_duration_cap > 0)) {
    throw std::invalid_argument("invalid request configuration");
  }
}
hj::Pose2d pose_from(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  if (std::abs(q.x) > 1e-7 || std::abs(q.y) > 1e-7 ||
    !std::isfinite(q.x + q.y + q.z + q.w + p.position.x + p.position.y) ||
    std::abs(std::hypot(q.z, q.w) - 1) > 1e-5)
  {
    throw std::invalid_argument("invalid planar pose");
  }
  return {p.position.x, p.position.y, std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)};
}
std::shared_ptr<const hj::GridSnapshot> grid_from(const nav_msgs::msg::OccupancyGrid & m)
{
  auto p = pose_from(m.info.origin);
  if (m.header.frame_id != "map" || std::abs(p.yaw) > 1e-7 || m.info.width == 0 || m.info.height == 0 ||
    uint64_t(m.info.width) * m.info.height != m.data.size() || m.data.size() > 50000000 ||
    !(m.info.resolution > 0))
  {
    throw std::invalid_argument("invalid occupancy grid");
  }
  Json::Value meta(Json::arrayValue);
  for (double v:{double(m.info.width), double(m.info.height), double(m.info.resolution), p.x, p.y}) {
    meta.append(v);
  }
  auto key = digest(json(meta) + ":" + std::string(m.data.begin(), m.data.end()));
  return std::make_shared<hj::GridSnapshot>(
    m.info.width, m.info.height, m.info.resolution,
    fwis_planning_core::Point2d{p.x, p.y}, std::vector<uint8_t>(m.data.begin(), m.data.end()), 0, key);
}
Context single_context(
  hj::Pose2d start, hj::Pose2d goal, uint32_t id, const Settings & s,
  std::shared_ptr<const hj::GridSnapshot> g)
{
  Context c{s.base, id, "single", {1}};
  c.request.grid = std::move(g);
  c.request.vehicles = {{0, 0, s.length + 2 * s.margin, s.width + 2 * s.margin}};
  c.request.start.reference = start;
  c.request.start.heading[0] = start.yaw;
  c.request.goal.reference = goal;
  c.request.goal.heading[0] = goal.yaw;
  return c;
}
Context formation_context(const Formation & m, const Settings & s, std::shared_ptr<const hj::GridSnapshot> g)
{
  if (m.header.frame_id != "map" || !m.plan_id || m.formation_id.empty() || m.vehicles.size() != 3 ||
    m.continuation_plan_id || !m.continuation_model_sha256.empty())
  {
    throw std::invalid_argument("invalid or unsupported formation request");
  }
  Context c{s.base, m.plan_id, m.formation_id, {}};
  c.request.grid = std::move(g);
  c.request.allow_reverse = m.allow_reverse;
  c.request.start.reference = pose(m.formation_start);
  c.request.goal.reference = pose(m.formation_goal);
  for (size_t i = 0; i < 3; ++i) {
    const auto & v = m.vehicles[i];
    const auto & o = v.offset_in_formation;
    if (v.vehicle_id != i + 1 || !std::isfinite(o.x + o.y) || o.z != 0. ||
      !(v.body_length_m > 0 && v.body_width_m > 0))
    {
      throw std::invalid_argument("invalid vehicle identity or geometry");
    }
    auto start = pose(v.start_center), goal = pose(v.goal_center);
    auto check = [&](auto p, auto ref) {
        auto center = fwis_planning_core::formation_center(ref, {o.x, o.y});
        if (std::hypot(center.x - p.x, center.y - p.y) > .002) {
          throw std::invalid_argument("mount endpoint differs from request");
        }
      };
    check(start, c.request.start.reference);
    check(goal, c.request.goal.reference);
    c.ids.push_back(v.vehicle_id);
    c.request.vehicles.push_back({o.x, o.y, v.body_length_m + 2 * s.margin, v.body_width_m + 2 * s.margin});
    c.request.start.heading[i] = start.yaw;
    c.request.goal.heading[i] = goal.yaw;
    c.request.goal.constrain_heading[i] = !v.goal_heading_free;
  }
  return c;
}
geometry_msgs::msg::Pose2D pose2(hj::Pose2d p)
{
  geometry_msgs::msg::Pose2D result;
  result.x = p.x;
  result.y = p.y;
  result.theta = p.yaw;
  return result;
}
} // namespace hybrid_ros
