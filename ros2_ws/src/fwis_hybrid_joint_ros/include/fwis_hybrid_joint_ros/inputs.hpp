#pragma once
#include "fwis_hybrid_joint_ros/transport.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>

namespace hybrid_ros
{
class Inputs : public rclcpp::Node
{
protected:
  explicit Inputs(const char * name)
  : Node(name), settings(*this)
  {
    const bool planner = std::string(name)=="hybrid_joint_planner";
    readiness = create_publisher<std_msgs::msg::String>(
      planner ? "/hybrid_joint/planner_readiness" : "/hybrid_joint/readiness", durable());
    readiness_timer = create_wall_timer(std::chrono::milliseconds(20), [this] {update_readiness();});
    map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", durable(), [this](const nav_msgs::msg::OccupancyGrid & m) {
        try {
          auto next = grid_from(m);
          if (!grid || grid->hash() != next->hash()) {
            invalidate();
            grid = std::move(next);
            announced_map.clear();
            update_readiness();
          }
        } catch (const std::exception & e) {invalidate();grid.reset();failure(0, e.what());}
      });
    if (settings.multi) {
      multi_sub = create_subscription<Formation>(
        "/formation/plan_request", 10, [this](const Formation & m) {
          try {
            if (!request_ready()) {throw std::invalid_argument("native_clock_or_map_not_ready");}
            auto c = formation_context(m, settings, grid);
            if (last_id && m.plan_id <= last_id) {throw std::invalid_argument("stale or reused plan ID");}
            last_id = m.plan_id;
            invalidate();
            requested(std::move(c), m.vehicles);
          } catch (const std::exception & e) {invalidate();failure(m.plan_id, e.what());}
        });
    } else {
      auto set_start = [this](const std_msgs::msg::Header & h, const geometry_msgs::msg::Pose & p) {
          try {
            if (h.frame_id != "map") {
              throw std::invalid_argument("invalid start frame");
            }
            start = pose_from(p);
            invalidate();
          } catch (const std::exception & e) {start.reset();invalidate();failure(0, e.what());}
        };
      start_sub =
        create_subscription<geometry_msgs::msg::PoseStamped>(
        "/start_pose", 10,
        [set_start](const geometry_msgs::msg::PoseStamped & m) {set_start(m.header, m.pose);});
      initial_sub =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        [set_start](const geometry_msgs::msg::PoseWithCovarianceStamped & m) {
          set_start(m.header, m.pose.pose);
        });
      goal_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 10, [this](const geometry_msgs::msg::PoseStamped & m) {
          ++last_id;
          invalidate();
          try {
            if (!request_ready()) {throw std::invalid_argument("native_clock_or_map_not_ready");}
            if (!start || !last_id || m.header.frame_id != "map") {
              throw std::invalid_argument("missing map/start or bad goal");
            }
            requested(single_context(*start, pose_from(m.pose), last_id, settings, grid), {});
          } catch (const std::exception & e) {failure(last_id, e.what());}
        });
    }
  }
  bool request_ready() const {
    return grid && clock_advanced && now().nanoseconds()>0 &&
      hj::Clock::now()-last_advance<=std::chrono::milliseconds(200);
  }
  using Specs = std::vector<fwis_v3_msgs::msg::FormationVehicleSpec>;
  virtual void requested(Context, Specs) = 0;
  virtual void invalidate() = 0;
  virtual void map_ready() {}
  virtual void failure(uint32_t, const std::string &) = 0;
  const Settings settings;
  std::shared_ptr<const hj::GridSnapshot> grid;
  const char * model_topic()const {return settings.multi ? "/formation/playback_plan" : envelope_topic;}
  static std_msgs::msg::String message(std::string content)
  {
    std_msgs::msg::String msg;
    msg.data = std::move(content);
    return msg;
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr readiness;
  rclcpp::TimerBase::SharedPtr readiness_timer;
  int64_t last_clock{};
  bool clock_advanced{}, sent_ready{};
  std::string announced_map;
  hj::Clock::time_point last_advance{}, last_report{};
  void update_readiness() {
    const auto stamp=now().nanoseconds();
    const auto wall=hj::Clock::now();
    if(stamp<=0 || stamp<last_clock) clock_advanced=false;
    else if(last_clock>0 && stamp>last_clock) {
      clock_advanced=true;
      last_advance=wall;
    }
    last_clock=stamp;
    const bool ready=request_ready();
    const bool new_map=ready && announced_map!=grid->hash();
    if(ready!=sent_ready || new_map || wall-last_report>=std::chrono::milliseconds(50)) {
      Json::Value row;
      row["ready"]=ready;
      row["clock_ns"]=Json::Int64(stamp);
      row["map_sha256"]=grid ? grid->hash() : "";
      row["solver_build"]=build_digest();
      readiness->publish(message(json(row)));
      last_report=wall;
      sent_ready=ready;
    }
    if(new_map) {
      announced_map=grid->hash();
      map_ready();
    }
  }
  uint32_t last_id{};
  std::optional<hj::Pose2d> start;
  rclcpp::Subscription<Formation>::SharedPtr multi_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub, goal_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub;
};
}
