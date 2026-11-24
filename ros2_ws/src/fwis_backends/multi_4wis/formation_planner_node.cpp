// S4 ROS entry point.  It publishes a FormationPlanSeed only as
// GEOMETRY_ONLY_UNCERTIFIED: no timed trajectory, no FEASIBLE state, no
// per-vehicle A/O/S assignment, and no executable claim.
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include <fwis_v3_msgs/msg/formation_plan_request.hpp>
#include <fwis_v3_msgs/msg/formation_plan_seed.hpp>
#include <fwis_v3_msgs/msg/formation_primitive.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fwis_ros/parameters.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "fwis_v3_planner/formation_model.hpp"
#include "fwis_v3_planner/formation_search.hpp"

namespace {

namespace formation = fwis::formation;

std::string json_escape(const std::string& text) {
  std::ostringstream out;
  for (unsigned char ch : text) {
    switch (ch) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (ch < 0x20) {
          const char hex[] = "0123456789abcdef";
          out << "\\u00" << hex[(ch >> 4) & 0xf] << hex[ch & 0xf];
        } else {
          out << static_cast<char>(ch);
        }
    }
  }
  return out.str();
}

formation::FormationRequest request_from_message(
    const fwis_v3_msgs::msg::FormationPlanRequest& message) {
  formation::FormationRequest request;
  request.frame_id = message.header.frame_id;
  request.stamp_secs = message.header.stamp.sec;
  request.stamp_nsecs = message.header.stamp.nanosec;
  request.plan_id = message.plan_id;
  request.formation_id = message.formation_id;
  request.start_qf = {message.formation_start.x, message.formation_start.y,
                      message.formation_start.theta};
  request.goal_qf = {message.formation_goal.x, message.formation_goal.y,
                     message.formation_goal.theta};
  request.allow_reverse = message.allow_reverse;
  request.vehicles.reserve(message.vehicles.size());
  for (const auto& source : message.vehicles) {
    formation::FormationVehicle vehicle;
    vehicle.vehicle_id = source.vehicle_id;
    vehicle.offset_x = source.offset_in_formation.x;
    vehicle.offset_y = source.offset_in_formation.y;
    vehicle.offset_z = source.offset_in_formation.z;
    vehicle.start_x = source.start_center.x;
    vehicle.start_y = source.start_center.y;
    vehicle.start_theta = source.start_center.theta;
    vehicle.goal_x = source.goal_center.x;
    vehicle.goal_y = source.goal_center.y;
    vehicle.goal_theta = source.goal_center.theta;
    vehicle.goal_heading_free = source.goal_heading_free;
    vehicle.body_length_m = source.body_length_m;
    vehicle.body_width_m = source.body_width_m;
    request.vehicles.push_back(vehicle);
  }
  return request;
}

fwis_v3_msgs::msg::FormationPrimitive primitive_to_message(
    const formation::FormationPrimitive& primitive) {
  fwis_v3_msgs::msg::FormationPrimitive message;
  message.geometry_type = primitive.geometry_type;
  message.dx = primitive.dx;
  message.dy = primitive.dy;
  message.dpsi = primitive.dpsi;
  message.pivot_x = primitive.pivot_x;
  message.pivot_y = primitive.pivot_y;
  return message;
}

geometry_msgs::msg::Pose2D to_pose2d(const formation::FormationPose& pose) {
  geometry_msgs::msg::Pose2D message;
  message.x = pose.x;
  message.y = pose.y;
  message.theta = pose.psi;
  return message;
}

class FormationPlannerNode : public rclcpp::Node {
 public:
  FormationPlannerNode() : Node("formation_planner",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    load_config();
    sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", fwis_ros::map_qos(*this),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) { on_map(message); });
    sub_request_ = create_subscription<fwis_v3_msgs::msg::FormationPlanRequest>(
        "/formation/plan_request", rclcpp::QoS(1),
        [this](fwis_v3_msgs::msg::FormationPlanRequest::ConstSharedPtr message) { on_request(message); });
    pub_seed_ = create_publisher<fwis_v3_msgs::msg::FormationPlanSeed>("/formation/plan_seed", rclcpp::QoS(1).transient_local());
    pub_status_ = create_publisher<std_msgs::msg::String>("/formation/status", rclcpp::QoS(1).transient_local());
    RCLCPP_INFO(get_logger(), "[formation planner] S4 geometry-only front end ready");
  }

 private:
  void load_config() {
    fwis_ros::parameter(*this, "formation.search.xy_res", config_.xy_res,
                      config_.xy_res);
    fwis_ros::parameter(*this, "formation.search.yaw_res", config_.yaw_res,
                      config_.yaw_res);
    fwis_ros::parameter(*this, "formation.search.translation_step_m",
                      config_.translation_step_m,
                      config_.translation_step_m);
    fwis_ros::parameter(*this, "formation.search.rotation_step_rad",
                      config_.rotation_step_rad,
                      config_.rotation_step_rad);
    fwis_ros::parameter(*this, "formation.search.goal_xy_tol_m",
                      config_.goal_xy_tol_m, config_.goal_xy_tol_m);
    fwis_ros::parameter(*this, "formation.search.goal_yaw_tol_rad",
                      config_.goal_yaw_tol_rad,
                      config_.goal_yaw_tol_rad);
    fwis_ros::parameter(*this, "formation.search.max_corner_step_m",
                      config_.max_corner_step_m,
                      config_.max_corner_step_m);
    fwis_ros::parameter(*this, "formation.search.max_expansions",
                      config_.max_expansions, config_.max_expansions);
    fwis_ros::parameter(*this, "formation.search.max_candidates",
                      config_.max_candidates, config_.max_candidates);
    fwis_ros::parameter(*this, "formation.search.max_wall_time_s",
                      config_.max_wall_time_s, config_.max_wall_time_s);
    fwis_ros::parameter(*this, "formation.cost.translation", config_.w_translation,
                      config_.w_translation);
    fwis_ros::parameter(*this, "formation.cost.rotation", config_.w_rotation,
                      config_.w_rotation);
    fwis_ros::parameter(*this, "formation.cost.clearance", config_.w_clearance,
                      config_.w_clearance);
    fwis_ros::parameter(*this, "formation.cost.switch", config_.w_switch,
                      config_.w_switch);
    fwis_ros::parameter(*this, "formation.cost.risk_horizon_m",
                      config_.risk_horizon_m, config_.risk_horizon_m);
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& message) {
    map_frame_id_ = message->header.frame_id;
    const double origin_yaw = tf2::getYaw(message->info.origin.orientation);
    if (!std::isfinite(origin_yaw) || std::abs(origin_yaw) > 1e-7) {
      has_map_ = false;
      map_error_ = "rotated or non-finite map origins are not supported";
      RCLCPP_ERROR(get_logger(), "[formation planner] %s", map_error_.c_str());
      return;
    }
    if (!std::isfinite(message->info.resolution) ||
        message->info.resolution <= 0.0 || message->info.width <= 0 ||
        message->info.height <= 0 || message->data.empty() ||
        !std::isfinite(message->info.origin.position.x) ||
        !std::isfinite(message->info.origin.position.y)) {
      has_map_ = false;
      map_error_ = "map has invalid dimensions, origin, or data";
      RCLCPP_ERROR(get_logger(), "[formation planner] %s", map_error_.c_str());
      return;
    }
    std::vector<int8_t> data(message->data.begin(), message->data.end());
    map_ = formation::FormationMap(
        message->info.width, message->info.height, message->info.resolution,
        message->info.origin.position.x, message->info.origin.position.y,
        std::move(data));
    has_map_ = map_.valid();
    map_error_ = has_map_ ? "" : "map data does not match its dimensions";
    if (!has_map_) {
      RCLCPP_ERROR(get_logger(), "[formation planner] %s", map_error_.c_str());
    } else if (pending_request_) {
      const auto request = pending_request_;
      pending_request_.reset();
      on_request(request);
    }
  }

  void on_request(const fwis_v3_msgs::msg::FormationPlanRequest::ConstSharedPtr& message) {
    ++requests_seen_;
    const formation::FormationRequest request = request_from_message(*message);
    const formation::FormationValidation validation =
        formation::validate_formation_request(request);
    if (!validation.valid) {
      publish_status(validation, request, nullptr, "REJECTED", false,
                     "NONE");
      return;
    }
    if (!has_map_) {
      pending_request_ = message;
      publish_status(validation, request, nullptr, "MAP_UNAVAILABLE", false,
                     "NONE");
      return;
    }
    if (map_frame_id_ != "map") {
      publish_status(validation, request, nullptr, "MAP_FRAME_MISMATCH",
                     false, "NONE");
      return;
    }

    const formation::FormationSearchResult result =
        formation::search_formation_geometry(validation.request,
                                             validation.model, map_, config_);
    if (!result.success) {
      publish_status(validation, request, &result, "GEOMETRY_SEARCH_FAILED",
                     true, "NONE");
      return;
    }

    publish_seed(validation.request, result);
    publish_status(validation, request, &result,
                   "GEOMETRY_ONLY_UNCERTIFIED", true, "GEOMETRY_ONLY");
  }

  void publish_seed(const formation::FormationRequest& request,
                    const formation::FormationSearchResult& result) {
    fwis_v3_msgs::msg::FormationPlanSeed seed;
    seed.header.stamp = get_clock()->now();
    seed.header.frame_id = "map";
    seed.plan_id = request.plan_id;
    seed.formation_id = request.formation_id;
    seed.formation_start = to_pose2d(request.start_qf);
    seed.formation_goal = to_pose2d(request.goal_qf);
    seed.primitives.reserve(result.primitives.size());
    for (const auto& primitive : result.primitives) {
      seed.primitives.push_back(primitive_to_message(primitive));
    }
    pub_seed_->publish(seed);
    RCLCPP_INFO(get_logger(), "[formation planner] published GEOMETRY_ONLY seed plan_id=%u "
             "formation_id=%s primitives=%zu",
             request.plan_id, request.formation_id.c_str(),
             result.primitives.size());
  }

  void publish_status(const formation::FormationValidation& validation,
                      const formation::FormationRequest& raw_request,
                      const formation::FormationSearchResult* result,
                      const std::string& node_state, bool search_started,
                      const std::string& certification) {
    const bool success = result != nullptr && result->success;
    const std::string reason =
        validation.valid
            ? (success ? ""
                       : (result != nullptr
                              ? result->reason
                              : (node_state == "MAP_UNAVAILABLE"
                                     ? "MAP_UNAVAILABLE"
                                     : (node_state == "MAP_FRAME_MISMATCH"
                                            ? "MAP_FRAME_MISMATCH"
                                            : "GEOMETRY_SEARCH_FAILED"))))
            : validation.reason;
    const std::string detail =
        validation.valid
            ? (result != nullptr ? result->detail
                                 : (node_state == "MAP_UNAVAILABLE" ? map_error_
                                                                   : ""))
            : validation.detail;
    const std::string field = validation.field;

    std::ostringstream json;
    json << "{"
         << "\"schema\":\"formation_frontend_status/v1\","
         << "\"valid\":" << (validation.valid ? "true" : "false") << ","
         << "\"reason\":\"" << json_escape(reason) << "\","
         << "\"field\":\"" << json_escape(field) << "\","
         << "\"vehicle_id\":";
    if (validation.has_vehicle_id) {
      json << validation.vehicle_id;
    } else {
      json << "null";
    }
    json << ",\"detail\":\"" << json_escape(detail) << "\","
         << "\"plan_id\":" << raw_request.plan_id << ","
         << "\"formation_id\":\"" << json_escape(raw_request.formation_id)
         << "\","
         << "\"node_state\":\"" << json_escape(node_state) << "\","
         << "\"certification\":\"" << json_escape(certification) << "\","
         << "\"search_started\":" << (search_started ? "true" : "false")
         << ","
         << "\"trajectory_published\":false,"
         << "\"executable\":false,"
         << "\"requests_seen\":" << requests_seen_ << ","
         << "\"expanded_nodes\":"
         << (result != nullptr ? result->expanded_nodes : 0) << ","
         << "\"generated_nodes\":"
         << (result != nullptr ? result->generated_nodes : 0) << ","
         << "\"goal_candidates\":"
         << (result != nullptr ? result->goal_candidates : 0) << ","
         << "\"primitive_count\":"
         << (success ? static_cast<int>(result->primitives.size()) : 0) << ","
         << "\"geometry_only\":"
         << ((result == nullptr || result->geometry_only) ? "true" : "false")
         << "}";
    std_msgs::msg::String status;
    status.data = json.str();
    pub_status_->publish(status);
    if (!validation.valid) {
      RCLCPP_WARN(get_logger(), "[formation planner] rejected request: %s %s",
               validation.field.c_str(), validation.detail.c_str());
    } else if (!success) {
      RCLCPP_WARN(get_logger(), "[formation planner] geometry search failed: %s %s",
               reason.c_str(), detail.c_str());
    }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<fwis_v3_msgs::msg::FormationPlanRequest>::SharedPtr sub_request_;
  rclcpp::Publisher<fwis_v3_msgs::msg::FormationPlanSeed>::SharedPtr pub_seed_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  fwis_v3_msgs::msg::FormationPlanRequest::ConstSharedPtr pending_request_;

  formation::FormationSearchConfig config_;
  formation::FormationMap map_;
  bool has_map_ = false;
  std::string map_frame_id_;
  std::string map_error_;
  int requests_seen_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FormationPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
