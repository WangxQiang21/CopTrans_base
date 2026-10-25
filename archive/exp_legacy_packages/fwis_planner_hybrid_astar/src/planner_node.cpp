#include <chrono>
#include <memory>
#include <mutex>
#include <vector>

#include <fwis_msgs/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "fwis_common/grid_map.hpp"
#include "fwis_common/kinematics.hpp"
#include "fwis_common/modes.hpp"
#include "fwis_common/safe_corridor.hpp"
#include "fwis_common/vehicle.hpp"
#include "fwis_planner_hybrid_astar/planner.hpp"

namespace fwis {

class PlannerNode {
 public:
  PlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    // All params live under the node's private namespace, organized under
    // `search/`, `cost/`, `heuristic/`, `vehicle/`. The yaml is loaded by
    // the launch file via <rosparam command="load" />.
    pnh_.param("search/xy_res",        cfg_.xy_res,        0.20);
    pnh_.param("search/yaw_res",       cfg_.yaw_res,       0.1745);
    pnh_.param("search/step_len",      cfg_.step_len,      0.20);
    pnh_.param("search/n_steer",       cfg_.n_steer,       5);
    pnh_.param("search/n_phi",         cfg_.n_phi,         7);
    pnh_.param("search/max_iter",      cfg_.max_iter,      200000);
    pnh_.param("search/goal_xy_tol",      cfg_.goal_xy_tol,      0.30);
    pnh_.param("search/goal_yaw_tol",     cfg_.goal_yaw_tol,     0.5);
    pnh_.param("search/parallel_goal_yaw_tol", cfg_.parallel_goal_yaw_tol, 0.35);
    pnh_.param("search/rs_shot_radius",   cfg_.rs_shot_radius,   8.0);
    pnh_.param("search/rs_shot_check_every", cfg_.rs_shot_check_every, 5);
    pnh_.param("search/allow_reverse",    cfg_.allow_reverse,    true);
    pnh_.param("search/smooth_step",      smooth_step_,          0.05);
    pnh_.param("topology/clearance", cfg_.topology_clearance, 0.22);
    pnh_.param("topology/clearance_weight", cfg_.topology_clearance_weight, 0.35);
    pnh_.param("topology/sample_step", cfg_.topology_sample_step, 0.10);
    pnh_.param("topology/candidate_count", cfg_.topology_candidate_count, 3);
    pnh_.param("topology/turn_weight", cfg_.topology_turn_weight, 0.25);
    pnh_.param("topology/use_macro_frontend", cfg_.use_macro_frontend, true);

    pnh_.param("corridor/enabled",           show_corridors_,        true);
    pnh_.param("corridor/fast_step",         corridor_cfg_.fast_step, 0.2);
    pnh_.param("corridor/fine_step",         corridor_cfg_.fine_step, 0.05);
    pnh_.param("corridor/max_distance",      corridor_cfg_.max_distance, 3.0);
    pnh_.param("corridor/axis_aligned",      corridor_cfg_.axis_aligned, true);
    pnh_.param("corridor/resample_distance", corridor_cfg_.resample_distance, 0.1);
    pnh_.param("corridor/visualization_stride", corridor_viz_stride_, 1);

    // Mode kill-switches (ablation experiments). Default: all on.
    pnh_.param("search/enable_lateral",  cfg_.enable_lateral,  true);
    pnh_.param("search/enable_diagonal", cfg_.enable_diagonal, true);
    pnh_.param("search/enable_spin",     cfg_.enable_spin,     true);

    // Time-equivalent cost (方案C)
    pnh_.param("cost/w_time",            cfg_.w_time,            2.0);
    pnh_.param("cost/v_cruise",          cfg_.v_cruise,          1.0);
    pnh_.param("cost/k_rev",             cfg_.k_rev,             0.8);
    pnh_.param("cost/omega_spin",        cfg_.omega_spin,        0.5);
    pnh_.param("cost/d_spin",            cfg_.d_spin,            2.0);
    pnh_.param("cost/f_spin_min",        cfg_.f_spin_min,        0.5);
    pnh_.param("cost/f_spin_max",        cfg_.f_spin_max,        2.5);
    pnh_.param("cost/k_exit_spin",       cfg_.k_exit_spin,       0.3);
    pnh_.param("cost/yaw_snap_threshold", cfg_.yaw_snap_threshold, 0.05);
    pnh_.param("cost/w_dir_change",        cfg_.w_dir_change,        1.0);
    pnh_.param("cost/w_steer",             cfg_.w_steer,             0.8);
    pnh_.param("cost/w_phi",               cfg_.w_phi,               1.5);
    pnh_.param("cost/w_steer_change_ack",  cfg_.w_steer_change_ack,  1.5);
    pnh_.param("cost/w_phi_change_dia",    cfg_.w_phi_change_dia,    1.5);
    pnh_.param("cost/w_curv_change",       cfg_.w_curv_change,       1.5);
    pnh_.param("cost/spin_yaw_step",       cfg_.spin_yaw_step,       0.0873);
    pnh_.param("cost/v_ref",               cfg_.v_ref,               1.0);
    pnh_.param("cost/t_switch_servo",      cfg_.t_switch_servo,      0.5);
    pnh_.param("cost/a_max_decel",         cfg_.a_max_decel,         2.0);
    pnh_.param("cost/d_slow",              cfg_.d_slow,              2.0);
    pnh_.param("cost/d_slow_obs",          cfg_.d_slow_obs,          0.8);
    pnh_.param("cost/v_floor_frac",        cfg_.v_floor_frac,        0.4);
    pnh_.param("cost/w_enter_lateral",     cfg_.w_enter_lateral,     3.0);
    pnh_.param("cost/w_enter_diagonal",    cfg_.w_enter_diagonal,    0.5);
    pnh_.param("cost/w_enter_spin",        cfg_.w_enter_spin,        1.0);
    pnh_.param("cost/w_enter_ackermann",   cfg_.w_enter_ackermann,   0.0);

    pnh_.param("heuristic/inflation",  cfg_.h_inflation,    1.3);

    pnh_.param("vehicle/length",    vp_.length,     0.738);
    pnh_.param("vehicle/width",     vp_.width,      0.500);
    pnh_.param("vehicle/wheelbase", vp_.wheelbase,  0.494);
    pnh_.param("vehicle/track",     vp_.track,      0.364);
    pnh_.param("vehicle/r_min_ack", vp_.r_min_ack,  0.81);
    pnh_.param("vehicle/phi_max",   vp_.phi_max,    1.571);

    double c_switch_cruise = cfg_.v_ref * cfg_.t_switch_servo
                             + cfg_.v_ref * cfg_.v_ref / cfg_.a_max_decel;
    double spin90_cost = cfg_.w_time * (M_PI / 2.0 / cfg_.omega_spin);
    ROS_INFO("[fwis_planner] time-cost: w_time=%.1f  v_cruise=%.1f  k_rev=%.1f"
             "  ω_spin=%.2f  spin90°=%.2f  d_spin=%.1f  f_min=%.2f",
             cfg_.w_time, cfg_.v_cruise, cfg_.k_rev,
             cfg_.omega_spin, spin90_cost, cfg_.d_spin, cfg_.f_spin_min);
    ROS_INFO("[fwis_planner] C_switch(cruise v_ref=%.1f)=%.2f  t_servo=%.2f"
             "  a_max=%.2f  d_slow=%.2f  dir_chg=%.2f  k_exit_spin=%.2f",
             cfg_.v_ref, c_switch_cruise,
             cfg_.t_switch_servo, cfg_.a_max_decel, cfg_.d_slow,
             cfg_.w_dir_change, cfg_.k_exit_spin);
    ROS_INFO("[fwis_planner] cost: steer=%.2f  phi=%.2f  Δsteer=%.2f  Δphi=%.2f"
             "  Δcurv=%.2f  h_infl=%.2f",
             cfg_.w_steer, cfg_.w_phi, cfg_.w_steer_change_ack,
             cfg_.w_phi_change_dia, cfg_.w_curv_change, cfg_.h_inflation);
    ROS_INFO("[fwis_planner] modes: Ack=on Lat=%s Dia=%s Spin=%s",
             cfg_.enable_lateral  ? "on" : "OFF",
             cfg_.enable_diagonal ? "on" : "OFF",
             cfg_.enable_spin     ? "on" : "OFF");

    sub_map_   = nh_.subscribe("/map", 1, &PlannerNode::on_map, this);
    sub_init_  = nh_.subscribe("/initialpose", 1, &PlannerNode::on_init, this);
    sub_start_ = nh_.subscribe("/start_pose", 1, &PlannerNode::on_start, this);
    sub_goal_  = nh_.subscribe("/move_base_simple/goal", 1, &PlannerNode::on_goal, this);

    pub_path_  = nh_.advertise<nav_msgs::Path>("/plan/path", 1, true);
    pub_traj_  = nh_.advertise<fwis_msgs::Trajectory>("/plan/trajectory", 1, true);
    pub_modes_ = nh_.advertise<visualization_msgs::MarkerArray>("/plan/mode_strip", 1, true);
    pub_mark_  = nh_.advertise<visualization_msgs::MarkerArray>("/plan/markers", 1, true);
    pub_corridors_ = nh_.advertise<visualization_msgs::MarkerArray>("/plan/corridor_markers", 1, true);
    ROS_INFO("[fwis_planner] ready. Set goal in RViz to plan.");
  }

 private:
  static double yaw_of(const geometry_msgs::Quaternion& q) {
    tf2::Quaternion qq(q.x, q.y, q.z, q.w);
    double r, p, y;
    tf2::Matrix3x3(qq).getRPY(r, p, y);
    return y;
  }

  void on_map(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mu_);
    map_.reset(*msg);
    ROS_INFO("[fwis_planner] map: %dx%d, res=%.3f m", msg->info.width,
             msg->info.height, msg->info.resolution);
  }

  void on_init(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    geometry_msgs::PoseStamped ps;
    ps.header = msg->header;
    ps.pose = msg->pose.pose;
    set_start(ps, "RViz initialpose");
  }

  void on_start(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    set_start(*msg, "scene_meta");
  }

  void set_start(const geometry_msgs::PoseStamped& msg, const char* src) {
    std::lock_guard<std::mutex> lk(mu_);
    start_.x = msg.pose.position.x;
    start_.y = msg.pose.position.y;
    start_.theta = yaw_of(msg.pose.orientation);
    start_.mode = Mode::Ackermann;
    has_start_ = true;
    ROS_INFO("[fwis_planner] start (%s) set: (%.2f, %.2f, %.2f)",
             src, start_.x, start_.y, start_.theta);
    if (map_.valid() &&
        map_.obb_collides(start_.x, start_.y, start_.theta,
                          vp_.length + 0.02, vp_.width + 0.02)) {
      ROS_WARN("[fwis_planner] start (%.2f, %.2f) lies inside an obstacle - "
               "drag '2D Pose Estimate' to a free cell.", start_.x, start_.y);
    }
    publish_static_markers();
  }

  void on_goal(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    PlanState g;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (!has_start_ || !map_.valid()) {
        ROS_WARN("[fwis_planner] need both /map and a start pose before planning.");
        return;
      }
      g.x = msg->pose.position.x;
      g.y = msg->pose.position.y;
      g.theta = yaw_of(msg->pose.orientation);
      g.mode = Mode::Ackermann;
      goal_ = g;
      has_goal_ = true;
    }
    plan_now(g);
  }

  void plan_now(const PlanState& goal) {
    PlanState s;
    GridMap mp;
    bool ok = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      s = start_;
      mp = map_;
      ok = has_start_;
    }
    if (!ok) return;

    // Sanity-check endpoints against the map BEFORE running the search,
    // so the user gets an actionable message instead of a silent "iter=1 fail".
    auto in_obstacle = [&](const PlanState& p) {
      return mp.obb_collides(p.x, p.y, p.theta,
                             vp_.length + 0.02,
                             vp_.width  + 0.02);
    };
    if (in_obstacle(s)) {
      ROS_ERROR("[fwis_planner] START (%.2f, %.2f, %.2f) is inside an "
                "obstacle. Move it with '2D Pose Estimate' onto a free cell.",
                s.x, s.y, s.theta);
      publish_path({});
      publish_static_markers();
      return;
    }
    if (in_obstacle(goal)) {
      ROS_ERROR("[fwis_planner] GOAL (%.2f, %.2f, %.2f) is inside an "
                "obstacle. Pick another goal in free space.",
                goal.x, goal.y, goal.theta);
      publish_path({});
      publish_static_markers();
      return;
    }

    auto t0 = std::chrono::steady_clock::now();
    PlanResult res;
    if (cfg_.use_macro_frontend) {
      // Varying the generic clearance preference exposes different corridor
      // homotopies where the map permits them.  Candidates are compared with
      // one common metric before the continuous OCP sees a seed.
      const std::array<double, 5> clearance_scales = {0.45, 0.70, 1.0, 1.45, 2.1};
      const int count = std::max(1, std::min(
          cfg_.topology_candidate_count,
          static_cast<int>(clearance_scales.size())));
      double best_score = std::numeric_limits<double>::infinity();
      int chosen = -1;
      for (int ci = 0; ci < count; ++ci) {
        PlannerConfig candidate_cfg = cfg_;
        candidate_cfg.topology_clearance_weight *= clearance_scales[ci];
        MacroTopologyPlanner planner(candidate_cfg, vp_);
        PlanResult candidate = planner.plan(mp, s, goal);
        if (!candidate.success || candidate.path.size() < 2) continue;
        double length = 0.0, clearance_risk = 0.0, turn = 0.0;
        for (size_t i = 1; i < candidate.path.size(); ++i) {
          const auto& a = candidate.path[i - 1];
          const auto& b = candidate.path[i];
          const double ds = std::hypot(b.x - a.x, b.y - a.y);
          length += ds;
          clearance_risk += ds / std::max(mp.distance(b.x, b.y), 0.05);
          turn += std::abs(wrap_pi(b.theta - a.theta));
        }
        const double score = length + cfg_.topology_clearance_weight * clearance_risk
                           + cfg_.topology_turn_weight * turn;
        if (score < best_score) {
          best_score = score;
          chosen = ci;
          res = std::move(candidate);
        }
      }
      ROS_INFO("[fwis_planner] macro candidates=%d chosen=%d score=%.2f",
               count, chosen, best_score);
    } else {
      HybridAStarPlanner planner(cfg_, vp_);
      res = planner.plan(mp, s, goal);
    }

    auto dt = std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - t0).count();

    if (!res.success) {
      ROS_WARN("[fwis_planner] FAILED  iter=%d  dt=%.1f ms", res.iterations, dt);
      publish_path({});
      publish_static_markers();
      return;
    }
    // Macro-front-end edges are intentionally sparse: the safe-corridor OCP
    // consumes this topology seed and produces the executable dense path.
    // Keep a dense visualization/publication seed for the legacy fallback.
    auto smooth = cfg_.use_macro_frontend ? res.path
                                          : densify_path_per_mode(res.path, smooth_step_);

    // Diagnostic: mode distribution and suspicious Spin segments
    int nA=0, nL=0, nP=0, nS=0;
    for (auto& p : res.path) {
      switch (p.mode) {
        case Mode::Ackermann: ++nA; break;
        case Mode::Lateral:   ++nL; break;
        case Mode::Diagonal:  ++nP; break;
        case Mode::Spin:      ++nS; break;
      }
    }
    double spin_frac = res.path.empty() ? 0.0 : 100.0 * nS / res.path.size();
    std::string spin_warn;
    if (nS > 0 && !res.path.empty()) {
      // Check if any Spin happens far from goal
      auto& g = goal_;
      for (size_t i = 0; i < res.path.size(); ++i) {
        if (res.path[i].mode == Mode::Spin) {
          double dg = std::hypot(res.path[i].x - g.x, res.path[i].y - g.y);
          if (dg > cfg_.d_spin + 0.5) {
            spin_warn = "  SPIN@" + std::to_string(i) + "(d_goal="
                      + std::to_string(dg).substr(0,4) + "m)";
            break;
          }
        }
      }
    }
    ROS_INFO("[fwis_planner] OK  raw=%zu  smooth=%zu  iter=%d  cost=%.2f"
             "  dt=%.1f ms  [A%dp,L%dp,P%dp,S%dp(%.0f%%)]%s",
             res.path.size(), smooth.size(), res.iterations, res.cost,
             dt, nA, nL, nP, nS, spin_frac, spin_warn.c_str());
    publish_path(smooth);
    publish_static_markers();
    publish_corridors(smooth, mp);
  }

  void publish_path(const std::vector<PlanState>& path) {
    // 1. nav_msgs/Path (for any generic path display)
    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    for (auto& p : path) {
      geometry_msgs::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      tf2::Quaternion q;
      q.setRPY(0, 0, p.theta);
      ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y();
      ps.pose.orientation.z = q.z();
      ps.pose.orientation.w = q.w();
      msg.poses.push_back(ps);
    }
    pub_path_.publish(msg);

    // 2. fwis_msgs/Trajectory (carries mode labels for the playback node)
    fwis_msgs::Trajectory tj;
    tj.header.frame_id = "map";
    tj.header.stamp = ros::Time::now();
    for (auto& p : path) {
      geometry_msgs::Pose2D ps;
      ps.x = p.x;
      ps.y = p.y;
      ps.theta = p.theta;
      tj.pose.push_back(ps);
      tj.v.push_back(0.0);
      tj.omega.push_back(0.0);
      tj.mode.push_back(static_cast<uint8_t>(p.mode));
      tj.phi.push_back(p.phi);
      tj.d_switch.push_back(0.0);
    }
    pub_traj_.publish(tj);

    // 3. Mode-colored line strip (single MarkerArray refresh; lifetime=0 = forever
    //    until next overwrite of same id)
    visualization_msgs::MarkerArray mark;
    {
      // delete previous strip first (clean redraw)
      visualization_msgs::Marker del;
      del.header.frame_id = "map";
      del.header.stamp = ros::Time::now();
      del.ns = "mode_strip";
      del.action = visualization_msgs::Marker::DELETEALL;
      mark.markers.push_back(del);
    }
    int id = 0;
    for (size_t i = 1; i < path.size(); ++i) {
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time::now();
      m.ns = "mode_strip";
      m.id = id++;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = 0.06;
      m.pose.orientation.w = 1.0;
      m.color = mode_color(path[i].mode);
      m.color.a = 0.95;
      geometry_msgs::Point p0;
      p0.x = path[i - 1].x;
      p0.y = path[i - 1].y;
      p0.z = 0.05;
      geometry_msgs::Point p1;
      p1.x = path[i].x;
      p1.y = path[i].y;
      p1.z = 0.05;
      m.points.push_back(p0);
      m.points.push_back(p1);
      mark.markers.push_back(m);
    }
    pub_modes_.publish(mark);
  }

  static std_msgs::ColorRGBA mode_color(Mode m) {
    std_msgs::ColorRGBA c;
    c.a = 1.0;
    switch (m) {
      case Mode::Ackermann: c.r = 0.20; c.g = 0.85; c.b = 0.30; break;
      case Mode::Lateral:   c.r = 1.00; c.g = 0.45; c.b = 0.05; break;
      case Mode::Diagonal:  c.r = 0.20; c.g = 0.55; c.b = 1.00; break;
      case Mode::Spin:      c.r = 0.85; c.g = 0.30; c.b = 0.95; break;
    }
    return c;
  }

  void publish_static_markers() {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = ros::Time::now();
    del.ns = "endpoints";
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);

    if (has_start_) {
      visualization_msgs::Marker s;
      s.header.frame_id = "map";
      s.header.stamp = ros::Time::now();
      s.ns = "endpoints";
      s.id = 0;
      s.type = visualization_msgs::Marker::ARROW;
      s.action = visualization_msgs::Marker::ADD;
      s.scale.x = 0.6;
      s.scale.y = 0.12;
      s.scale.z = 0.12;
      s.color.r = 0.10; s.color.g = 0.95; s.color.b = 0.10; s.color.a = 1.0;
      s.pose.position.x = start_.x;
      s.pose.position.y = start_.y;
      s.pose.position.z = 0.20;
      tf2::Quaternion q; q.setRPY(0, 0, start_.theta);
      s.pose.orientation.x = q.x(); s.pose.orientation.y = q.y();
      s.pose.orientation.z = q.z(); s.pose.orientation.w = q.w();
      arr.markers.push_back(s);
    }
    if (has_goal_) {
      visualization_msgs::Marker g;
      g.header.frame_id = "map";
      g.header.stamp = ros::Time::now();
      g.ns = "endpoints";
      g.id = 1;
      g.type = visualization_msgs::Marker::ARROW;
      g.action = visualization_msgs::Marker::ADD;
      g.scale.x = 0.6;
      g.scale.y = 0.12;
      g.scale.z = 0.12;
      g.color.r = 0.95; g.color.g = 0.10; g.color.b = 0.10; g.color.a = 1.0;
      g.pose.position.x = goal_.x;
      g.pose.position.y = goal_.y;
      g.pose.position.z = 0.20;
      tf2::Quaternion q; q.setRPY(0, 0, goal_.theta);
      g.pose.orientation.x = q.x(); g.pose.orientation.y = q.y();
      g.pose.orientation.z = q.z(); g.pose.orientation.w = q.w();
      arr.markers.push_back(g);
    }
    pub_mark_.publish(arr);
  }

  void publish_corridors(const std::vector<PlanState>& path, const GridMap& map) {
    if (!show_corridors_ || path.size() < 2) {
      // clear previous corridors
      visualization_msgs::Marker del;
      del.header.frame_id = "map";
      del.header.stamp = ros::Time::now();
      del.ns = "corridors";
      del.action = visualization_msgs::Marker::DELETEALL;
      visualization_msgs::MarkerArray arr;
      arr.markers.push_back(del);
      pub_corridors_.publish(arr);
      return;
    }

    std::vector<Pose2D> poses;
    for (const auto& p : path) {
      poses.push_back({p.x, p.y, p.theta});
    }

    auto corridors = fwis::build_corridors(poses, map, vp_, corridor_cfg_);
    auto& front_corridors = corridors.first;
    auto& rear_corridors  = corridors.second;

    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.header.frame_id = "map";
    del.header.stamp = ros::Time::now();
    del.ns = "corridors";
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);

    int id = 0;
    for (size_t i = 0; i < front_corridors.size(); i += corridor_viz_stride_) {
      const auto& c = front_corridors[i];
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time::now();
      m.ns = "corridors";
      m.id = id++;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = 0.04;
      m.color.r = 0.10; m.color.g = 0.85; m.color.b = 0.10; m.color.a = 0.60;
      geometry_msgs::Point p;
      // 4 corners of the corridor rectangle, in world coords
      double c_bwd = c.cx - c.backward * c.tan_x;
      double c_fwd = c.cx + c.forward * c.tan_x;
      double c_left = c.cy - c.left * c.norm_y;
      double c_right = c.cy + c.right * c.norm_y;
      if (std::abs(c.tan_x) >= std::abs(c.tan_y)) {
        c_bwd = c.cx - c.backward * c.tan_x;
        c_fwd = c.cx + c.forward * c.tan_x;
        c_left = c.cy - c.left * c.norm_y;
        c_right = c.cy + c.right * c.norm_y;
      }
      // use axis-aligned rectangle directly
      double fx0 = c.cx - c.backward * c.tan_x - c.left * c.norm_x;
      double fy0 = c.cy - c.backward * c.tan_y - c.left * c.norm_y;
      double fx1 = c.cx + c.forward  * c.tan_x - c.left * c.norm_x;
      double fy1 = c.cy + c.forward  * c.tan_y - c.left * c.norm_y;
      double fx2 = c.cx + c.forward  * c.tan_x + c.right * c.norm_x;
      double fy2 = c.cy + c.forward  * c.tan_y + c.right * c.norm_y;
      double fx3 = c.cx - c.backward * c.tan_x + c.right * c.norm_x;
      double fy3 = c.cy - c.backward * c.tan_y + c.right * c.norm_y;
      p.x = fx0; p.y = fy0; p.z = 0.04; m.points.push_back(p);
      p.x = fx1; p.y = fy1; p.z = 0.04; m.points.push_back(p);
      p.x = fx2; p.y = fy2; p.z = 0.04; m.points.push_back(p);
      p.x = fx3; p.y = fy3; p.z = 0.04; m.points.push_back(p);
      p.x = fx0; p.y = fy0; p.z = 0.04; m.points.push_back(p);  // close loop
      arr.markers.push_back(m);
    }
    // rear corridors in blue
    for (size_t i = 0; i < rear_corridors.size(); i += corridor_viz_stride_) {
      const auto& c = rear_corridors[i];
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time::now();
      m.ns = "corridors";
      m.id = id++;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = 0.04;
      m.color.r = 0.10; m.color.g = 0.30; m.color.b = 0.90; m.color.a = 0.50;
      geometry_msgs::Point p;
      double fx0 = c.cx - c.backward * c.tan_x - c.left * c.norm_x;
      double fy0 = c.cy - c.backward * c.tan_y - c.left * c.norm_y;
      double fx1 = c.cx + c.forward  * c.tan_x - c.left * c.norm_x;
      double fy1 = c.cy + c.forward  * c.tan_y - c.left * c.norm_y;
      double fx2 = c.cx + c.forward  * c.tan_x + c.right * c.norm_x;
      double fy2 = c.cy + c.forward  * c.tan_y + c.right * c.norm_y;
      double fx3 = c.cx - c.backward * c.tan_x + c.right * c.norm_x;
      double fy3 = c.cy - c.backward * c.tan_y + c.right * c.norm_y;
      p.x = fx0; p.y = fy0; p.z = 0.04; m.points.push_back(p);
      p.x = fx1; p.y = fy1; p.z = 0.04; m.points.push_back(p);
      p.x = fx2; p.y = fy2; p.z = 0.04; m.points.push_back(p);
      p.x = fx3; p.y = fy3; p.z = 0.04; m.points.push_back(p);
      p.x = fx0; p.y = fy0; p.z = 0.04; m.points.push_back(p);
      arr.markers.push_back(m);
    }
    pub_corridors_.publish(arr);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_map_, sub_init_, sub_start_, sub_goal_;
  ros::Publisher  pub_path_, pub_traj_, pub_modes_, pub_mark_, pub_corridors_;

  std::mutex      mu_;
  GridMap         map_;
  PlanState       start_;
  PlanState       goal_;
  bool            has_start_ = false;
  bool            has_goal_  = false;
  PlannerConfig   cfg_;
  VehicleParams   vp_;
  double          smooth_step_ = 0.05;
  CorridorConfig  corridor_cfg_;
  bool            show_corridors_ = true;
  int             corridor_viz_stride_ = 1;
};

}  // namespace fwis

int main(int argc, char** argv) {
  ros::init(argc, argv, "fwis_planner_hybrid_astar");
  ros::NodeHandle nh, pnh("~");
  fwis::PlannerNode node(nh, pnh);
  ros::spin();
  return 0;
}
