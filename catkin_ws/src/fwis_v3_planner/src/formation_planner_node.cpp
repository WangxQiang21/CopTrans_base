// S4 ROS entry point.  It publishes a FormationPlanSeed only as
// GEOMETRY_ONLY_UNCERTIFIED: no timed trajectory, no FEASIBLE state, no
// per-vehicle A/O/S assignment, and no executable claim.
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include <fwis_v3_msgs/FormationPlanRequest.h>
#include <fwis_v3_msgs/FormationPlanSeed.h>
#include <fwis_v3_msgs/FormationPrimitive.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>

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
    const fwis_v3_msgs::FormationPlanRequest& message) {
  formation::FormationRequest request;
  request.frame_id = message.header.frame_id;
  request.stamp_secs = message.header.stamp.sec;
  request.stamp_nsecs = message.header.stamp.nsec;
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

fwis_v3_msgs::FormationPrimitive primitive_to_message(
    const formation::FormationPrimitive& primitive) {
  fwis_v3_msgs::FormationPrimitive message;
  message.geometry_type = primitive.geometry_type;
  message.dx = primitive.dx;
  message.dy = primitive.dy;
  message.dpsi = primitive.dpsi;
  message.pivot_x = primitive.pivot_x;
  message.pivot_y = primitive.pivot_y;
  return message;
}

geometry_msgs::Pose2D to_pose2d(const formation::FormationPose& pose) {
  geometry_msgs::Pose2D message;
  message.x = pose.x;
  message.y = pose.y;
  message.theta = pose.psi;
  return message;
}

class FormationPlannerNode {
 public:
  FormationPlannerNode() : private_nh_("~") {
    load_config();
    sub_map_ = nh_.subscribe("/map", 1, &FormationPlannerNode::on_map, this);
    sub_request_ = nh_.subscribe("/formation/plan_request", 1,
                                 &FormationPlannerNode::on_request, this);
    pub_seed_ = nh_.advertise<fwis_v3_msgs::FormationPlanSeed>(
        "/formation/plan_seed", 1, true);
    pub_status_ = nh_.advertise<std_msgs::String>("/formation/status", 1, true);
    ROS_INFO("[formation planner] S4 geometry-only front end ready");
  }

 private:
  void load_config() {
    private_nh_.param("formation/search/xy_res", config_.xy_res,
                      config_.xy_res);
    private_nh_.param("formation/search/yaw_res", config_.yaw_res,
                      config_.yaw_res);
    private_nh_.param("formation/search/translation_step_m",
                      config_.translation_step_m,
                      config_.translation_step_m);
    private_nh_.param("formation/search/rotation_step_rad",
                      config_.rotation_step_rad,
                      config_.rotation_step_rad);
    private_nh_.param("formation/search/goal_xy_tol_m",
                      config_.goal_xy_tol_m, config_.goal_xy_tol_m);
    private_nh_.param("formation/search/goal_yaw_tol_rad",
                      config_.goal_yaw_tol_rad,
                      config_.goal_yaw_tol_rad);
    private_nh_.param("formation/search/max_corner_step_m",
                      config_.max_corner_step_m,
                      config_.max_corner_step_m);
    private_nh_.param("formation/search/max_expansions",
                      config_.max_expansions, config_.max_expansions);
    private_nh_.param("formation/search/max_candidates",
                      config_.max_candidates, config_.max_candidates);
    private_nh_.param("formation/search/max_wall_time_s",
                      config_.max_wall_time_s, config_.max_wall_time_s);
    private_nh_.param("formation/cost/translation", config_.w_translation,
                      config_.w_translation);
    private_nh_.param("formation/cost/rotation", config_.w_rotation,
                      config_.w_rotation);
    private_nh_.param("formation/cost/clearance", config_.w_clearance,
                      config_.w_clearance);
    private_nh_.param("formation/cost/switch", config_.w_switch,
                      config_.w_switch);
    private_nh_.param("formation/cost/risk_horizon_m",
                      config_.risk_horizon_m, config_.risk_horizon_m);
  }

  void on_map(const nav_msgs::OccupancyGrid::ConstPtr& message) {
    map_frame_id_ = message->header.frame_id;
    const double origin_yaw = tf::getYaw(message->info.origin.orientation);
    if (!std::isfinite(origin_yaw) || std::abs(origin_yaw) > 1e-7) {
      has_map_ = false;
      map_error_ = "rotated or non-finite map origins are not supported";
      ROS_ERROR("[formation planner] %s", map_error_.c_str());
      return;
    }
    if (!std::isfinite(message->info.resolution) ||
        message->info.resolution <= 0.0 || message->info.width <= 0 ||
        message->info.height <= 0 || message->data.empty() ||
        !std::isfinite(message->info.origin.position.x) ||
        !std::isfinite(message->info.origin.position.y)) {
      has_map_ = false;
      map_error_ = "map has invalid dimensions, origin, or data";
      ROS_ERROR("[formation planner] %s", map_error_.c_str());
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
      ROS_ERROR("[formation planner] %s", map_error_.c_str());
    } else if (pending_request_) {
      const auto request = pending_request_;
      pending_request_.reset();
      on_request(request);
    }
  }

  void on_request(const fwis_v3_msgs::FormationPlanRequest::ConstPtr& message) {
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
    fwis_v3_msgs::FormationPlanSeed seed;
    seed.header.stamp = ros::Time::now();
    seed.header.frame_id = "map";
    seed.plan_id = request.plan_id;
    seed.formation_id = request.formation_id;
    seed.formation_start = to_pose2d(request.start_qf);
    seed.formation_goal = to_pose2d(request.goal_qf);
    seed.primitives.reserve(result.primitives.size());
    for (const auto& primitive : result.primitives) {
      seed.primitives.push_back(primitive_to_message(primitive));
    }
    pub_seed_.publish(seed);
    ROS_INFO("[formation planner] published GEOMETRY_ONLY seed plan_id=%u "
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
    std_msgs::String status;
    status.data = json.str();
    pub_status_.publish(status);
    if (!validation.valid) {
      ROS_WARN("[formation planner] rejected request: %s %s",
               validation.field.c_str(), validation.detail.c_str());
    } else if (!success) {
      ROS_WARN("[formation planner] geometry search failed: %s %s",
               reason.c_str(), detail.c_str());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber sub_map_;
  ros::Subscriber sub_request_;
  ros::Publisher pub_seed_;
  ros::Publisher pub_status_;
  fwis_v3_msgs::FormationPlanRequest::ConstPtr pending_request_;

  formation::FormationSearchConfig config_;
  formation::FormationMap map_;
  bool has_map_ = false;
  std::string map_frame_id_;
  std::string map_error_;
  int requests_seen_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "formation_planner");
  FormationPlannerNode node;
  ros::spin();
  return 0;
}
