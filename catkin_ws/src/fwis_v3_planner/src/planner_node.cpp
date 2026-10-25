#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <fwis_common/grid_map.hpp>
#include <fwis_common/reeds_shepp.hpp>
#include <fwis_v3_msgs/ModePrimitive.h>
#include <fwis_v3_msgs/PlanSeed.h>
#include <fwis_v3_msgs/PlanSeedCandidates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr uint8_t A = 0, O = 1, S = 2;
constexpr double O_CRUISE_SIGMA = 25.0*PI/180.0;
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double oblique_cruise_factor(double phi) {
  const double z=phi/O_CRUISE_SIGMA;
  return std::exp(-0.5*z*z);
}
struct Pose { double x=0, y=0, th=0; };
struct Action { uint8_t mode=A; double amount=0, control=0; };
int direction_switch_count(const std::vector<Action>& actions) {
  int previous=0, switches=0;
  for(const auto& action:actions) {
    if(action.mode==S || std::abs(action.amount)<1e-10) continue;
    const int gear=action.amount>0?1:-1;
    if(previous!=0 && previous!=gear) ++switches;
    previous=gear;
  }
  return switches;
}
std::string shunting_violation(const std::vector<Action>& actions) {
  int previous=0,reversals=0;
  double spin=0.0;
  for(const auto& action:actions) {
    if(action.mode==S) { spin+=action.amount; continue; }
    if(std::abs(action.amount)<1e-10) continue;
    const int gear=action.amount>0?1:-1;
    if(previous!=0 && previous!=gear) {
      ++reversals;
      if(reversals>1) return "repeated_translation_reversals";
      if(std::abs(wrap(spin))<=1e-4)
        return "translation_reversal_without_spin";
    }
    previous=gear;
    spin=0.0;
  }
  return "";
}
bool has_reverse_motion(const std::vector<Action>& actions) {
  return std::any_of(actions.begin(),actions.end(),[](const Action& action) {
    return action.mode!=S && action.amount < -1e-10;
  });
}
struct CostBreakdown {
  double distance=0,spin=0,mode_switch=0,reverse=0,direction=0;
  double reverse_early=0,reverse_start=0,reverse_yaw_early=0;
  double low_curvature_switch=0;
  double steer_tv=0,curvature=0,oblique_tv=0,oblique_event=0;
  double oblique_use=0,oblique_cruise=0;
  double total() const {
    return distance+spin+mode_switch+low_curvature_switch+reverse+direction+steer_tv+
           curvature+oblique_tv+oblique_event+oblique_use+oblique_cruise+
           reverse_early+reverse_start+reverse_yaw_early;
  }
};
Pose advance(const Pose& p, const Action& a) {
  Pose q=p;
  if (a.mode==A) {
    double yaw=p.th+a.amount*a.control;
    if (std::abs(a.control)<1e-10) {
      q.x+=a.amount*std::cos(p.th); q.y+=a.amount*std::sin(p.th);
    } else {
      q.x+=(std::sin(yaw)-std::sin(p.th))/a.control;
      q.y+=(std::cos(p.th)-std::cos(yaw))/a.control;
    }
    q.th=wrap(yaw);
  } else if (a.mode==O) {
    q.x+=a.amount*std::cos(p.th+a.control);
    q.y+=a.amount*std::sin(p.th+a.control);
  } else q.th=wrap(p.th+a.amount);
  return q;
}
double yaw_of(const geometry_msgs::Quaternion& q) {
  return tf::getYaw(q);
}
geometry_msgs::PoseStamped stamp_pose(const Pose& p, const ros::Time& t) {
  geometry_msgs::PoseStamped out;
  out.header.frame_id="map"; out.header.stamp=t;
  out.pose.position.x=p.x; out.pose.position.y=p.y;
  out.pose.orientation=tf::createQuaternionMsgFromYaw(p.th);
  return out;
}
struct Node { Pose p; uint8_t mode=A; int gear=1, control=0, parent=-1;
              bool switched=false,has_translation=false,has_control=false;
              double g=0; Action action; bool has_forward=false;
              int shunt_reversals=0; bool spun_since_translation=false;
              double motion_cost_lb=0.; };

// Obstacle-free relaxation in the project's A/O/Spin model. Translation must
// cover the position deficit; yaw needs Ackermann travel or paid Spin. O
// cannot change yaw, and Spin cannot translate, so those states sometimes
// require one additional mode event. The initial choice of mode is free.
double mode_motion_lower_bound(const Pose& p,uint8_t mode,bool initial,
                              const Pose& goal,double xy_tol,double yaw_tol,
                              double radius,double spin_weight,double switch_weight) {
  const double distance=std::max(0.,std::hypot(p.x-goal.x,p.y-goal.y)-xy_tol);
  const double angle=std::max(0.,std::abs(wrap(p.th-goal.th))-yaw_tol);
  const bool needs_switch=(mode==O && angle>1e-10) ||
                          (mode==S && distance>1e-10);
  return std::max(distance,std::min(radius,spin_weight)*angle)+
         ((!initial && needs_switch)?switch_weight:0.);
}
struct CellRun { int x0=0,x1=-1;
  bool operator==(const CellRun& other) const {return x0==other.x0 && x1==other.x1;}
};
struct CellSlab { int y0=0,y1=-1; std::vector<CellRun> runs; };
std::vector<CellSlab> merge_occupied_rows(const std::vector<std::vector<CellRun>>& rows) {
  std::vector<CellSlab> slabs;
  for(size_t y=0;y<rows.size();++y) {
    if(!slabs.empty() && slabs.back().runs==rows[y]) slabs.back().y1=int(y);
    else slabs.push_back({int(y),int(y),rows[y]});
  }
  return slabs;
}
bool occupied_slabs_clear(const std::vector<CellSlab>& slabs,const Pose& p,
                          double hl,double hw,double res,double ox,double oy,
                          int ix0,int ix1,int iy0,int iy1) {
  const double c=std::cos(p.th),s=std::sin(p.th);
  const double rx=hl*std::abs(c)+hw*std::abs(s);
  const double ry=hl*std::abs(s)+hw*std::abs(c);
  auto slab=std::lower_bound(slabs.begin(),slabs.end(),iy0,
      [](const CellSlab& row,int y) {return row.y1<y;});
  for(;slab!=slabs.end() && slab->y0<=iy1;++slab) {
    const double hh=(slab->y1-slab->y0+1)*res/2;
    const double cy=oy+(slab->y0+slab->y1+1)*res/2;
    auto run=std::lower_bound(slab->runs.begin(),slab->runs.end(),ix0,
        [](const CellRun& r,int x) {return r.x1<x;});
    for(;run!=slab->runs.end() && run->x0<=ix1;++run) {
      const double rh=(run->x1-run->x0+1)*res/2;
      const double cx=ox+(run->x0+run->x1+1)*res/2;
      const double dx=cx-p.x,dy=cy-p.y;
      if(std::abs(dx*c+dy*s)>hl+rh*std::abs(c)+hh*std::abs(s)) continue;
      if(std::abs(-dx*s+dy*c)>hw+rh*std::abs(s)+hh*std::abs(c)) continue;
      if(std::abs(dx)>rh+rx || std::abs(dy)>hh+ry) continue;
      return false;
    }
  }
  return true;
}
struct Key {
  int x,y,th,mode,gear,control,switched,has_translation,has_control,has_forward;
  int shunt_reversals,spun_since_translation;
  bool operator==(const Key& b) const {
    return x==b.x&&y==b.y&&th==b.th&&mode==b.mode&&gear==b.gear&&
           control==b.control&&switched==b.switched&&
           has_translation==b.has_translation&&has_control==b.has_control&&
           has_forward==b.has_forward&&shunt_reversals==b.shunt_reversals&&
           spun_since_translation==b.spun_since_translation;
  }
};
struct KeyHash {
  size_t operator()(const Key& k) const {
    size_t h=0;
    for (int v : {k.x,k.y,k.th,k.mode,k.gear,k.control,k.switched,
                   k.has_translation,k.has_control,k.has_forward,
                   k.shunt_reversals,k.spun_since_translation})
      h^=std::hash<int>{}(v)+0x9e3779b9+(h<<6)+(h>>2);
    return h;
  }
};
struct QueueItem {
  double f;
  int id;
  bool operator<(const QueueItem& b) const {
    if (f != b.f) return f > b.f;
    return id > b.id;
  }
};

class PlannerNode {
 public:
  PlannerNode(): pnh_("~") {
    pnh_.param("search/step_len",step_,0.20);
    pnh_.param("search/xy_res",xy_res_,0.10);
    pnh_.param("search/yaw_res",yaw_res_,PI/18);
    pnh_.param("search/max_iter",max_iter_,180000);
    std::string backend;
    pnh_.param<std::string>("planning/backend",backend,"legacy");
    std::string direction_policy;
    const bool has_direction_policy=
        pnh_.getParam("planning/direction_policy",direction_policy);
    bool legacy_reverse=false;
    const bool has_legacy_reverse=
        pnh_.getParam("planning/allow_reverse_fallback",legacy_reverse);
    if(pnh_.hasParam("planning/direction_policy")&&!has_direction_policy) {
      ROS_FATAL("planning/direction_policy must be a string");
      throw std::invalid_argument("direction_policy_type_error");
    }
    if(pnh_.hasParam("planning/allow_reverse_fallback")&&!has_legacy_reverse) {
      ROS_FATAL("deprecated planning/allow_reverse_fallback must be boolean");
      throw std::invalid_argument("legacy_direction_policy_type_error");
    }
    if(has_direction_policy && direction_policy!="bidirectional" &&
       direction_policy!="forward_only") {
      ROS_FATAL_STREAM("unknown planning/direction_policy: "<<direction_policy);
      throw std::invalid_argument("unknown_direction_policy");
    }
    if(has_direction_policy && has_legacy_reverse &&
       ((direction_policy=="bidirectional")!=legacy_reverse)) {
      ROS_FATAL("conflicting direction_policy and deprecated allow_reverse_fallback");
      throw std::invalid_argument("conflicting_direction_policy");
    }
    if(!has_direction_policy) {
      if(has_legacy_reverse) {
        direction_policy=legacy_reverse?"bidirectional":"forward_only";
        ROS_WARN("planning/allow_reverse_fallback is deprecated; use planning/direction_policy=%s",
                 direction_policy.c_str());
      } else {
        direction_policy=(backend=="mode_path_v1"||backend=="mode_path_v2")?
                          "bidirectional":"forward_only";
      }
    } else if(has_legacy_reverse) {
      ROS_WARN("planning/allow_reverse_fallback is deprecated; use planning/direction_policy=%s",
               direction_policy.c_str());
    }
    // Apply the configured direction policy to every backend. In particular,
    // the multimodal Hybrid A* backend must be able to compare reverse-capable
    // candidates when its bidirectional policy is selected.
    allow_reverse_=direction_policy=="bidirectional";
    bool allow_shunting=false;
    const bool has_shunting_policy=
        pnh_.getParam("planning/allow_shunting",allow_shunting);
    if(pnh_.hasParam("planning/allow_shunting")&&!has_shunting_policy) {
      ROS_FATAL("planning/allow_shunting must be boolean");
      throw std::invalid_argument("allow_shunting_policy_type_error");
    }
    allow_shunting_=allow_shunting;
    pnh_.param("search/reverse_fallback_max_iter",reverse_fallback_max_iter_,30000);
    pnh_.param("search/reverse_fallback_budget_s",reverse_fallback_budget_s_,3.0);
    pnh_.param("search/max_iter_with_incumbent",incumbent_iter_,5000);
    pnh_.param("search/guided_iter_without_incumbent",
               guided_iter_without_incumbent_,max_iter_/2);
    pnh_.param("search/ackermann_probe_iter",ackermann_probe_iter_,30000);
    pnh_.param("search/reverse_competition_max_iter",reverse_competition_max_iter_,90000);
    pnh_.param("search/route_graph_max_expansions",route_graph_max_expansions_,8000);
    pnh_.param("search/route_graph_dense_spacing",route_graph_dense_spacing_,0.30);
    pnh_.param("search/route_graph_sparse_spacing",route_graph_sparse_spacing_,0.70);
    pnh_.param("search/route_graph_continuous_adaptive",
               route_graph_continuous_adaptive_,false);
    pnh_.param("search/route_graph_turn_window",route_graph_turn_window_,0.45);
    pnh_.param("search/route_graph_turn_threshold",route_graph_turn_threshold_,0.35);
    pnh_.param("search/route_graph_max_edge_length",route_graph_max_edge_length_,3.0);
    pnh_.param("search/enable_route_heuristic",enable_route_heuristic_,true);
    pnh_.param("search/terminal_radius",terminal_radius_,1.5);
    pnh_.param("search/mode_radius",mode_radius_,5.0);
    pnh_.param("search/goal_xy_tol",xy_tol_,0.16);
    pnh_.param("search/goal_yaw_tol",yaw_tol_,0.14);
    pnh_.param("vehicle/length",length_,0.738);
    pnh_.param("vehicle/width",width_,0.500);
    pnh_.param("vehicle/r_min_ack",radius_,0.810);
    pnh_.param("collision/margin",margin_,0.04);
    pnh_.param("collision/sweep_guard",sweep_guard_,0.005);
    pnh_.param("collision/route_sweep_step",route_sweep_step_,0.01);
    pnh_.param("collision/a_graph_sweep_step",a_graph_sweep_step_,0.02);
    double nominal_speed=0.5,nominal_omega=0.5;
    pnh_.param("time/v_demo",nominal_speed,0.5);
    pnh_.param("time/omega_demo",nominal_omega,0.5);
    const double spin_distance_equivalent=(nominal_omega>1e-6)?
        nominal_speed/nominal_omega:1.0;
    pnh_.param("cost/w_spin",w_spin_,spin_distance_equivalent);
    pnh_.param("cost/w_switch",w_switch_,2.00);
    pnh_.param("cost/w_low_curvature_switch",low_curvature_switch_penalty_,0.35);
    pnh_.param("cost/switch_turn_threshold",switch_turn_threshold_,0.35);
    pnh_.param("cost/w_reverse",w_reverse_,1.00);
    if(backend=="mode_path_v2") {
      pnh_.param("quality/cost/reverse_early",w_reverse_early_,4.0);
      pnh_.param("quality/cost/reverse_start",w_reverse_start_,3.0);
      pnh_.param("quality/cost/reverse_yaw_early",w_reverse_yaw_early_,2.0);
    }
    pnh_.param("cost/w_direction",w_direction_,20.0);
    if(backend=="mode_path_v2")
      pnh_.param("quality/cost/direction_change",w_direction_,1.5);
    pnh_.param("cost/w_steer_tv",w_steer_tv_,0.40);
    pnh_.param("cost/w_curvature",w_curvature_,0.01);
    pnh_.param("cost/w_oblique_tv",w_oblique_tv_,1.50);
    pnh_.param("cost/w_oblique_event_score",w_oblique_event_score_,1.50);
    pnh_.param("cost/w_oblique_use",w_oblique_use_,0.55);
    pnh_.param("cost/w_oblique_cruise",w_oblique_cruise_,0.45);
    sub_map_=nh_.subscribe("/map",1,&PlannerNode::on_map,this);
    sub_start_=nh_.subscribe("/start_pose",1,&PlannerNode::on_start,this);
    sub_init_=nh_.subscribe("/initialpose",1,&PlannerNode::on_init,this);
    sub_goal_=nh_.subscribe("/move_base_simple/goal",1,&PlannerNode::on_goal,this);
    pub_path_=nh_.advertise<nav_msgs::Path>("/plan/path_coarse",1,true);
    pub_seed_=nh_.advertise<fwis_v3_msgs::PlanSeed>("/plan/seed",1,true);
    pub_seed_candidates_=nh_.advertise<fwis_v3_msgs::PlanSeedCandidates>(
        "/plan/seed_candidates",1,true);
    pub_markers_=nh_.advertise<visualization_msgs::MarkerArray>("/plan/markers",1,true);
    pub_status_=nh_.advertise<std_msgs::String>("/plan/search_status",1,true);
    ROS_INFO("[v3 planner] ready; coordinates are vehicle center");
  }
 private:
  Key key(const Node& n,double xy_resolution,
          bool proposal_search=false) const {
    // The auxiliary search intentionally matches the historical route-search
    // state merge so it can recover useful initializers cheaply. Its terminal
    // path is checked with the exact policy and remains untrusted if illegal.
    return {static_cast<int>(std::lround(n.p.x/xy_resolution)),static_cast<int>(std::lround(n.p.y/xy_resolution)),
            static_cast<int>(std::lround(wrap(n.p.th)/yaw_res_)),n.mode,n.gear,n.control,
            int(n.switched),int(n.has_translation),int(n.has_control),int(n.has_forward),
            proposal_search?0:n.shunt_reversals,
            // Once the single permitted reversal has happened, Spin history
            // cannot affect any future legal transition. Before it happens,
            // retain only whether a Spin occurred; terminal candidates still
            // pass the exact accumulated-angle policy check.
            proposal_search?0:int(n.shunt_reversals==0 &&
                                  n.spun_since_translation)};
  }
  double heuristic(const Pose& p,bool guided,uint8_t mode=A,bool initial=true) const {
    double d=std::max(0.0,std::hypot(p.x-goal_.x,p.y-goal_.y)-xy_tol_);
    if(guided&&!goal_field_.empty()) {
      const double grid_d=map_.sample_distance_field(goal_field_,p.x,p.y);
      if(std::isfinite(grid_d)) {
        // Raw-grid distance guides search around obstacle rows. The scale
        // compensates for 8-neighbour raster quantization; SAT remains the
        // actual footprint feasibility test and budget_hit reports anytime use.
        d=std::max(d,.85*std::max(0.0,grid_d-2*map_.resolution()));
      }
    }
    return std::max(d,mode_motion_lower_bound(p,mode,initial,goal_,
        xy_tol_,yaw_tol_,radius_,w_spin_,w_switch_));
  }
  double clearance_lower_bound(double x,double y) const {
    int ix,iy;
    if(!map_.world_to_idx(x,y,ix,iy) || safe_clearance_.empty()) return 0.0;
    // Chebyshev cell distance lower-bounds center-to-center Euclidean
    // distance. Subtract both half-cell diagonals to bound distance to the
    // occupied cell square for an arbitrary query point in its own cell.
    const double res=map_.resolution();
    const auto cells=safe_clearance_[iy*map_.width()+ix];
    return std::max(0.0,static_cast<double>(cells)*res-std::sqrt(2.0)*res);
  }
  bool pose_clear(const Pose& p,double extra_guard=0.0) const {
    // Exact OBB-vs-occupied-cell SAT. The legacy GridMap point-sampled OBB
    // can miss a cell at a narrow gap, so it is not an admissible gate here.
    const double res=map_.resolution(),
                 guard=sweep_guard_+extra_guard,
                 hl=(length_+margin_+2*guard)/2,
                 hw=(width_+margin_+2*guard)/2;
    const double c=std::cos(p.th),s=std::sin(p.th);
    const double rx=hl*std::abs(c)+hw*std::abs(s);
    const double ry=hl*std::abs(s)+hw*std::abs(c);
    const double ox=map_.origin_x(),oy=map_.origin_y();
    if(p.x-rx<ox || p.y-ry<oy || p.x+rx>ox+map_.width()*res ||
       p.y+ry>oy+map_.height()*res) return false;
    const double corner=std::hypot(hl,hw);
    // Skip exact SAT only when a precomputed conservative lower bound proves
    // that even the circumscribed vehicle disk is clear.
    if(clearance_lower_bound(p.x,p.y)>corner) return true;
    // Include cells whose squares touch the OBB's AABB, not only cells whose
    // centers fall inside it. SAT below rejects the extra broad-phase cells.
    const int ix0=std::max(0,int(std::floor((p.x-rx-ox)/res))-1);
    const int ix1=std::min(map_.width()-1,int(std::floor((p.x+rx-ox)/res))+1);
    const int iy0=std::max(0,int(std::floor((p.y-ry-oy)/res))-1);
    const int iy1=std::min(map_.height()-1,int(std::floor((p.y+ry-oy)/res))+1);
    // Equal consecutive raster rows form exact obstacle rectangles. A thick
    // straight wall now needs one SAT rather than one SAT per occupied row.
    return occupied_slabs_clear(occupied_slabs_,p,hl,hw,res,ox,oy,ix0,ix1,iy0,iy1);
  }
  bool action_clear(const Pose& p,const Action& a,double max_step=0.035) const {
    double rotation=(a.mode==S)?std::abs(a.amount):
                    (a.mode==A?std::abs(a.amount*a.control):0.0);
    double travel=(a.mode==S)?0.0:std::abs(a.amount);
    const double sweep_extent=sweep_guard_+max_step/2;
    double r=0.5*std::hypot(length_+margin_+2*sweep_extent,
                           width_+margin_+2*sweep_extent);
    int n=std::max(1,static_cast<int>(std::ceil((travel+r*rotation)/max_step)));
    for (int i=1;i<=n;++i) {
      Action partial=a; partial.amount=a.amount*i/n;
      // Bound the unsampled motion between neighboring configurations by
      // inflating the footprint with half the maximum corner displacement.
      if (!pose_clear(advance(p,partial),max_step/2)) return false;
    }
    return true;
  }
  bool append_connector(Pose& p,uint8_t& mode,bool& just_switched,
                        const Action& a,std::vector<Action>& suffix) const {
    if (a.mode!=mode && just_switched) return false;
    if (!action_clear(p,a)) return false;
    p=advance(p,a);mode=a.mode;just_switched=false;suffix.push_back(a);
    return true;
  }
  bool append_oblique_to(Pose& p,uint8_t& mode,bool& just_switched,
                         const Pose& target,std::vector<Action>& suffix,
                         bool allow_reverse=false) const {
    double dx=target.x-p.x,dy=target.y-p.y,distance=std::hypot(dx,dy);
    if (distance<=1e-8) return true;
    double phi=wrap(std::atan2(dy,dx)-p.th),amount=distance;
    if (phi>PI/2) {phi-=PI;amount=-distance;}
    if (phi<-PI/2) {phi+=PI;amount=-distance;}
    if (amount<0.0 && !allow_reverse) return false;
    return append_connector(p,mode,just_switched,Action{O,amount,phi},suffix);
  }
  // Generate swept-checked terminal connections. O translation can be
  // disabled for the Ackermann-first pass; Spin remains available for yaw.
  void terminal_candidates(const Node& n,std::vector<std::vector<Action>>& out,
                           double max_distance,bool rich=true,
                           bool allow_oblique=true,bool allow_reverse=false) const {
    if (std::hypot(n.p.x-goal_.x,n.p.y-goal_.y)>max_distance) return;
    auto finish=[&](std::vector<Action> prefix,bool oblique_first) {
      Pose p=n.p;uint8_t mode=n.mode;bool switched=n.switched;
      std::vector<Action> path;
      for(const auto& a:prefix)
        if(!append_connector(p,mode,switched,a,path)) return;
      if(oblique_first) {
        if(!allow_oblique) return;
        if(!append_oblique_to(p,mode,switched,goal_,path,allow_reverse)) return;
        double turn=wrap(goal_.th-p.th);
        if(std::abs(turn)>yaw_tol_ &&
           !append_connector(p,mode,switched,Action{S,turn,0},path)) return;
      } else {
        double turn=wrap(goal_.th-p.th);
        if(std::abs(turn)>yaw_tol_ &&
           !append_connector(p,mode,switched,Action{S,turn,0},path)) return;
        if(allow_oblique) {
          if(!append_oblique_to(p,mode,switched,goal_,path,allow_reverse)) return;
        } else if(std::hypot(p.x-goal_.x,p.y-goal_.y)>xy_tol_) return;
      }
      if((allow_reverse || !has_reverse_motion(path)) &&
         std::hypot(p.x-goal_.x,p.y-goal_.y)<=xy_tol_ &&
         std::abs(wrap(p.th-goal_.th))<=yaw_tol_) out.push_back(std::move(path));
    };
    finish({},false);
    if(!rich) return;
    finish({},true);

  }
  bool terminal_rs(const Node& n,std::vector<Action>& suffix,
                   double max_distance=1.8,double max_length=5.0,
                   bool allow_reverse=false) const {
    if (std::hypot(n.p.x-goal_.x,n.p.y-goal_.y)>max_distance) return false;
    if (n.mode!=A && n.switched) return false;
    auto curve=fwis::reeds_shepp(n.p.x,n.p.y,n.p.th,
                                goal_.x,goal_.y,goal_.th,radius_);
    if (!std::isfinite(curve.length) || curve.segs.empty() || curve.length>max_length) return false;
    Pose p=n.p;
    for (const auto& seg:curve.segs) {
      if (seg.length<1e-8) continue;
      double curvature=seg.type=='L'?1/radius_:(seg.type=='R'?-1/radius_:0);
      Action a{A,seg.direction*seg.length,curvature};
      if(a.amount<0.0 && !allow_reverse) return false;
      if (!action_clear(p,a)) return false;
      suffix.push_back(a);p=advance(p,a);
    }
    return !suffix.empty() && std::hypot(p.x-goal_.x,p.y-goal_.y)<=xy_tol_ &&
           std::abs(wrap(p.th-goal_.th))<=yaw_tol_;
  }
  CostBreakdown reverse_preference_parts(Pose pose,
       const std::vector<Action>& actions,int previous_gear=0) const {
    CostBreakdown out;
    if(w_reverse_early_+w_reverse_start_+w_reverse_yaw_early_<=0) return out;
    const double reference=std::max(length_,std::hypot(goal_.x-start_.x,goal_.y-start_.y));
    const double start_window=std::max(length_,.25*reference);
    bool seen_reverse=previous_gear<0;
    for(size_t i=0;i<actions.size();++i) {
      const auto& a=actions[i];
      const double done=std::hypot(pose.x-start_.x,pose.y-start_.y);
      const double remaining=std::hypot(pose.x-goal_.x,pose.y-goal_.y);
      const double late=remaining/std::max(done+remaining,1e-9);
      const double weight=late*late;
      if(a.mode!=S && std::abs(a.amount)>1e-10) {
        if(a.amount<0) {
          out.reverse_early+=w_reverse_early_*std::abs(a.amount)*weight;
          if(!seen_reverse)
            out.reverse_start+=w_reverse_start_*reference*std::exp(-done/start_window);
          seen_reverse=true;
          if(a.mode==A) out.reverse_yaw_early+=
              w_reverse_yaw_early_*reference*std::abs(a.amount*a.control)*weight/PI;
        }
      } else if(a.mode==S) {
        auto next=std::find_if(actions.begin()+i+1,actions.end(),[](const Action& b){
          return b.mode!=S && std::abs(b.amount)>1e-10;
        });
        if(next!=actions.end() && next->amount<0)
          out.reverse_yaw_early+=w_reverse_yaw_early_*reference*
                                 std::abs(a.amount)*weight/PI;
      }
      pose=advance(pose,a);
    }
    return out;
  }
  CostBreakdown action_sequence_cost_parts(const std::vector<Action>& actions) const {
    CostBreakdown parts;int prior=-1,last_gear=0;
    const auto preferences=reverse_preference_parts(start_,actions);
    parts.reverse_early=preferences.reverse_early;
    parts.reverse_start=preferences.reverse_start;
    parts.reverse_yaw_early=preferences.reverse_yaw_early;
    double last_k=0.0,last_phi=0.0;
    bool has_gear=false,has_k=false,has_phi=false;
    Pose pose{0,0,0};double previous_tangent=0.0;bool has_previous_tangent=false;
    for(const auto& a:actions) {
      if(prior>=0&&a.mode!=prior) {
        parts.mode_switch+=w_switch_;
        if(prior!=S && a.mode!=S && has_previous_tangent) {
          const double incoming=a.mode==A?
              pose.th+(a.amount<0?PI:0.0):
              pose.th+a.control+(a.amount<0?PI:0.0);
          if(std::abs(wrap(incoming-previous_tangent))<switch_turn_threshold_)
            parts.low_curvature_switch+=low_curvature_switch_penalty_;
        }
      }
      if(a.mode==S) {
        parts.spin+=w_spin_*std::abs(a.amount);has_k=false;has_phi=false;
        has_previous_tangent=false;
      } else {
        const double distance=std::abs(a.amount);
        const int gear=a.amount>=0?1:-1;
        parts.distance+=distance;
        if(gear<0) parts.reverse+=w_reverse_*distance;
        if(has_gear&&gear!=last_gear) parts.direction+=w_direction_;
        last_gear=gear;has_gear=true;
        if(a.mode==A) {
          if(has_k) parts.steer_tv+=w_steer_tv_*std::abs(a.control-last_k);
          parts.curvature+=w_curvature_*a.control*a.control*distance;
          last_k=a.control;has_k=true;has_phi=false;
          const Pose end=advance(pose,a);
          previous_tangent=end.th+(a.amount<0?PI:0.0);
          has_previous_tangent=true;
        } else {
          if(has_phi) {
            const double delta=std::abs(wrap(a.control-last_phi));
            parts.oblique_tv+=w_oblique_tv_*delta;
            parts.oblique_event+=w_oblique_event_score_*delta/(delta+.03);
          }
          // Sustained oblique motion has a distance-weighted cost, so a safe
          // Spin that aligns the body can compete on long diagonal segments.
          parts.oblique_use+=distance*w_oblique_use_*std::abs(a.control);
          parts.oblique_cruise+=distance*w_oblique_cruise_*
                                oblique_cruise_factor(a.control);
          last_phi=a.control;has_phi=true;has_k=false;
          previous_tangent=pose.th+a.control+(a.amount<0?PI:0.0);
          has_previous_tangent=true;
        }
      }
      pose=advance(pose,a);
      prior=a.mode;
    }
    return parts;
  }
  double action_sequence_cost(const std::vector<Action>& actions) const {
    return action_sequence_cost_parts(actions).total();
  }
  // Sparse mixed-mode graph on route waypoints.  Unlike the O/S DP below,
  // this graph has explicit A Reeds-Shepp edges as well as O translations and
  // in-place Spin edges.  Its state retains mode, gear, and the last control
  // bin so transition costs are Markov up to control quantization.
  bool sparse_mixed_route(const std::vector<Pose>& wp,
                          std::vector<Action>& result,double& result_cost,
                          double incumbent_cost,int expansion_limit,
                          int& expanded,bool& budget_hit,int& max_waypoint,
                          bool allow_oblique=true) const {
    expanded=0;budget_hit=false;max_waypoint=-1;
    if(wp.size()<2) return false;
    std::vector<double> yaw{start_.th};
    auto add_yaw=[&](double a) {
      for(double old:yaw) if(std::abs(wrap(old-a))<1e-5) return;
      yaw.push_back(a);
    };
    for(int k=0;k<8;++k) add_yaw(-PI+k*PI/4.0);
    // A terminal heading already within the accepted yaw tolerance is
    // represented by its nearest compass bin; retaining a near-duplicate
    // exact-goal bin only multiplies expensive swept-checked A connectors.
    bool goal_yaw_covered=false;
    for(double a:yaw)
      if(std::abs(wrap(a-goal_.th))<=yaw_tol_) goal_yaw_covered=true;
    if(!goal_yaw_covered) add_yaw(goal_.th);
    const int H=static_cast<int>(yaw.size()),N=static_cast<int>(wp.size());
    constexpr int OControls=17,NoControl=20,ControlCount=21;
    constexpr int ModeCount=4,NoneMode=3,Gears=3,ShuntStates=3;
    // 0: no Spin since last translation; 1: Spin occurred; 2: the single
    // permitted reversal has been used. Full paths receive the exact policy
    // check after reconstruction.
    auto sid=[&](int i,int h,int mode,int control,int gear,int shunt) {
      return ((((((i*H+h)*ModeCount+mode)*ControlCount+control)*Gears+gear)*
                ShuntStates)+shunt);
    };
    const int total=N*H*ModeCount*ControlCount*Gears*ShuntStates;
    std::vector<double> dist(total,std::numeric_limits<double>::infinity());
    std::vector<int> parent(total,-1);
    std::vector<std::vector<Action>> parent_edge(total);
    const int start_h=0;
    const int root=sid(0,start_h,NoneMode,NoControl,1,0);
    dist[root]=0;
    struct MixItem {
      double f,g;
      int id;
      bool operator<(const MixItem& b) const {
        if (f != b.f) return f > b.f;
        if (g != b.g) return g > b.g;
        return id > b.id;
      }
    };
    auto heuristic=[&](int i) {return std::hypot(wp[i].x-goal_.x,wp[i].y-goal_.y);};
    std::vector<double> local_turn(N,0.0);
    for(int i=1;i+1<N;++i) {
      const double before=std::atan2(wp[i].y-wp[i-1].y,wp[i].x-wp[i-1].x);
      const double after=std::atan2(wp[i+1].y-wp[i].y,wp[i+1].x-wp[i].x);
      local_turn[i]=std::abs(wrap(after-before));
    }
    std::priority_queue<MixItem> open;open.push({heuristic(0),0.0,root});
    std::vector<int8_t> spin_cache(N*H*H,-1);
    auto spin_clear=[&](int i,int h1,int h2) {
      const int index=(i*H+h1)*H+h2;
      int8_t& cached=spin_cache[index];
      if(cached<0) {
        const double dth=wrap(yaw[h2]-yaw[h1]);
        cached=std::abs(dth)<=1e-8 ||
            action_clear(Pose{wp[i].x,wp[i].y,yaw[h1]},Action{S,dth,0},route_sweep_step_);
      }
      return cached!=0;
    };
    struct AEdge { int j,h;std::vector<Action> actions; };
    std::vector<uint8_t> a_built(N*H,0);
    std::vector<std::vector<AEdge>> a_cache(N*H);
    auto a_edges=[&](int i,int h)->const std::vector<AEdge>& {
      const int key=i*H+h;
      if(a_built[key]) return a_cache[key];
      a_built[key]=1;
      for(int j=i+1;j<std::min(N,i+4);++j) for(int hj=0;hj<H;++hj) {
        auto curve=fwis::reeds_shepp(wp[i].x,wp[i].y,yaw[h],
                                     wp[j].x,wp[j].y,yaw[hj],radius_);
        if(!std::isfinite(curve.length) || curve.segs.empty() || curve.length>3.2) continue;
        Pose p{wp[i].x,wp[i].y,yaw[h]};std::vector<Action> actions;
        bool clear=true;
        for(const auto& seg:curve.segs) {
          if(seg.length<1e-7) continue;
          const double k=seg.type=='L'?1/radius_:(seg.type=='R'?-1/radius_:0.0);
          Action a{A,seg.direction*seg.length,k};
          if(!action_clear(p,a,a_graph_sweep_step_)) {clear=false;break;}
          p=advance(p,a);actions.push_back(a);
        }
        if(!clear || actions.empty() ||
           std::hypot(p.x-wp[j].x,p.y-wp[j].y)>1e-4 ||
           std::abs(wrap(p.th-yaw[hj]))>1e-4) continue;
        a_cache[key].push_back({j,hj,std::move(actions)});
      }
      return a_cache[key];
    };
    auto append_cost=[&](int mode,int control,int gear,const std::vector<Action>& edge,
                         const Pose& origin) {
      double score=reverse_preference_parts(origin,edge,gear-1).total();
      int prior_mode=mode,prior_gear=gear-1;
      bool has_k=mode==A&&control>=17&&control<=19;
      bool has_phi=mode==O&&control<OControls;
      double last_k=has_k?(control==17?-1/radius_:(control==19?1/radius_:0.0)):0.0;
      double last_phi=has_phi?-PI/2+control*(PI/16.0):0.0;
      for(const auto& a:edge) {
        if(prior_mode!=NoneMode&&prior_mode!=a.mode) score+=w_switch_;
        if(a.mode==S) {
          score+=w_spin_*std::abs(a.amount);has_k=false;has_phi=false;
        } else {
          const double d=std::abs(a.amount);const int g=a.amount<0?-1:1;
          score+=d*(1+(g<0?w_reverse_:0));
          if(prior_gear!=0&&prior_gear!=g) score+=w_direction_;
          prior_gear=g;
          if(a.mode==A) {
            if(prior_mode==A&&has_k) score+=w_steer_tv_*std::abs(a.control-last_k);
            score+=w_curvature_*a.control*a.control*d;
            last_k=a.control;has_k=true;has_phi=false;
          } else {
            if(prior_mode==O&&has_phi) {
              const double delta=std::abs(wrap(a.control-last_phi));
              score+=w_oblique_tv_*delta;
              score+=w_oblique_event_score_*delta/(delta+.03);
            }
            score+=w_oblique_use_*d*std::abs(a.control);
            score+=w_oblique_cruise_*d*oblique_cruise_factor(a.control);
            last_phi=a.control;has_phi=true;has_k=false;
          }
        }
        prior_mode=a.mode;
      }
      return score;
    };
    auto next_control=[&](uint8_t mode,const std::vector<Action>& edge) {
      if(mode==S) return NoControl;
      const Action& a=edge.back();
      if(mode==A) return a.control< -1e-7?17:(a.control>1e-7?19:18);
      int b=static_cast<int>(std::lround((a.control+PI/2)/(PI/16.0)));
      return std::max(0,std::min(OControls-1,b));
    };
    double best_goal=incumbent_cost;
    int best_state=-1;std::vector<Action> terminal;
    while(!open.empty() && expanded<expansion_limit) {
      const auto q=open.top();open.pop();
      if(q.g>dist[q.id]+1e-9) continue;
      if(q.f>=best_goal-1e-9) break;
      ++expanded;
      int z=q.id;const int shunt_state=z%ShuntStates;z/=ShuntStates;
      const int gi=z%Gears;z/=Gears;
      const int ci=z%ControlCount;z/=ControlCount;
      const int mode=z%ModeCount;z/=ModeCount;
      const int hi=z%H;const int wi=z/H;
      max_waypoint=std::max(max_waypoint,wi);
      auto relax=[&](int j,int hj,int nm,int nc,int ng,
                     const std::vector<Action>& edge,double extra=0.0) {
        // Route edges may include gear changes. Charge each change in the
        // shared cost and retain the last translation gear in the graph state.
        int edge_gear=gi-1; // {-1, no translation yet, +1}
        int next_shunt=shunt_state;
        for(const auto& action:edge) {
          if(action.mode==S) {
            if(!allow_shunting_ && next_shunt==0) next_shunt=1;
            continue;
          }
          if(std::abs(action.amount)<1e-10) continue;
          const int action_gear=action.amount>0?1:-1;
          if(!allow_shunting_ && edge_gear!=0 && action_gear!=edge_gear) {
            if(next_shunt!=1) return;
            next_shunt=2;
          } else if(!allow_shunting_ && next_shunt==1) {
            // A same-gear translation consumes the Spin before any later
            // reversal can be considered.
            next_shunt=0;
          }
          edge_gear=action_gear;
        }
        if(edge_gear!=0 && ng!=1 && edge_gear!=ng-1) return;
        if(allow_shunting_) next_shunt=0;
        double switch_extra=0.0;
        if(mode!=NoneMode && nm!=NoneMode && mode!=nm) {
          const double local_demand=std::max(local_turn[wi],
              local_turn[std::min(wi+1,N-1)]);
          // Keep mode-switch preference independent from anchor spacing:
          // route_graph_turn_threshold_ only decides where to densify.
          if(local_demand<switch_turn_threshold_)
            switch_extra=low_curvature_switch_penalty_;
        }
        const double candidate=q.g+append_cost(mode,ci,gi,edge,
            Pose{wp[wi].x,wp[wi].y,yaw[hi]})+extra+switch_extra;
        const int to=sid(j,hj,nm,nc,ng,next_shunt);
        if(candidate+1e-9>=dist[to]) return;
        dist[to]=candidate;parent[to]=q.id;parent_edge[to]=edge;
        open.push({candidate+heuristic(j),candidate,to});
      };
      if(wi==N-1) {
        const double turn=wrap(goal_.th-yaw[hi]);
        std::vector<Action> end;
        if(std::abs(turn)>yaw_tol_) {
          if(!action_clear(wp[wi],Action{S,turn,0},route_sweep_step_)) continue;
          end.push_back({S,turn,0});
        }
        const double candidate=q.g+append_cost(mode,ci,gi,end,
            Pose{wp[wi].x,wp[wi].y,yaw[hi]});
        if(candidate<best_goal) {best_goal=candidate;best_state=q.id;terminal=end;}
        continue;
      }
      for(int hj=0;hj<H;++hj) if(hj!=hi && spin_clear(wi,hi,hj)) {
        const double dth=wrap(yaw[hj]-yaw[hi]);
        std::vector<Action> edge{{S,dth,0}};
        relax(wi,hj,S,NoControl,gi,edge);
      }
      // O edges keep body heading fixed.  Spin, when useful, is an explicit
      // graph edge at the source, not a mode-sequence template.
      for(int j=wi+1;allow_oblique&&j<std::min(N,wi+7);++j) {
        const double vx=wp[j].x-wp[wi].x,vy=wp[j].y-wp[wi].y;
        const double d=std::hypot(vx,vy);
        if(d<1e-7 || d>route_graph_max_edge_length_) continue;
        double phi=wrap(std::atan2(vy,vx)-yaw[hi]),amount=d;
        if(phi>PI/2) {phi-=PI;amount=-d;}
        if(phi<-PI/2) {phi+=PI;amount=-d;}
        Action move{O,amount,phi};
        if(!action_clear(Pose{wp[wi].x,wp[wi].y,yaw[hi]},move,route_sweep_step_)) continue;
        std::vector<Action> edge{move};const int g=amount<0?-1:1;
        relax(j,hi,O,next_control(O,edge),g+1,edge);
      }
      // Lazy-cache short Reeds-Shepp connectors. Exact per-primitive sweep
      // validation rejects colliding arcs; only validated A edges enter graph.
      for(const auto& aedge:a_edges(wi,hi)) {
        int end_gear=gi;
        for(const auto& a:aedge.actions) if(std::abs(a.amount)>1e-8)
          end_gear=(a.amount<0?-1:1)+1;
        relax(aedge.j,aedge.h,A,next_control(A,aedge.actions),end_gear,aedge.actions);
      }
    }
    budget_hit=!open.empty() && expanded>=expansion_limit;
    if(best_state<0) return false;
    std::vector<std::vector<Action>> rev;
    for(int s=best_state;s!=root;s=parent[s]) {
      if(s<0||parent[s]<0) return false;
      rev.push_back(parent_edge[s]);
    }
    std::vector<Action> candidate;
    for(auto it=rev.rbegin();it!=rev.rend();++it)
      candidate.insert(candidate.end(),it->begin(),it->end());
    candidate.insert(candidate.end(),terminal.begin(),terminal.end());
    if(!allow_shunting_ && !shunting_violation(candidate).empty()) return false;
    Pose p{start_.x,start_.y,start_.th};
    for(const auto& a:candidate) p=advance(p,a);
    if(std::hypot(p.x-goal_.x,p.y-goal_.y)>xy_tol_||
       std::abs(wrap(p.th-goal_.th))>yaw_tol_) return false;
    result_cost=action_sequence_cost(candidate);result=std::move(candidate);
    return true;
  }
  // Fast, heading-free topological proposal.  It plans only the center path
  // on the 2-D occupancy grid, string-pulls it using exact O-mode swept OBB
  // checks, then adds a terminal Spin only when the requested yaw requires it.
  // This is a feasible incumbent proposal, not a prescribed maneuver: the
  // A/O/S lattice may replace it whenever it finds a lower-cost solution.
  bool oblique_route(std::vector<Action>& actions,double& cost,
                     std::string& reason) const {
    if(!pose_clear(start_) || !pose_clear(goal_)) {reason="endpoint_obb";return false;}
    const int w=map_.width(),h=map_.height(),n=w*h;
    const double res=map_.resolution(),ox=map_.origin_x(),oy=map_.origin_y();
    int sx,sy,gx,gy;
    if(!map_.world_to_idx(start_.x,start_.y,sx,sy) ||
       !map_.world_to_idx(goal_.x,goal_.y,gx,gy)) {reason="endpoint_outside_grid";return false;}
    const int src=sy*w+sx,dst=gy*w+gx;
    struct RouteItem { double f,g;int idx; };
    struct RouteCmp { bool operator()(const RouteItem& a,const RouteItem& b) const {
      if (a.f != b.f) return a.f > b.f;
      if (a.g != b.g) return a.g > b.g;
      return a.idx > b.idx;
    }};
    constexpr int dx[8]={1,-1,0,0,1,1,-1,-1};
    constexpr int dy[8]={0,0,1,-1,1,-1,1,-1};
    const double circumscribed=0.5*std::hypot(length_+margin_,width_+margin_);
    // Every A* variant queries the same grid-cell centers. Cache their
    // unchanged clearance/risk values once instead of converting each
    // neighbor to world coordinates and back for every expansion.
    std::vector<double> cell_clearance(n),cell_narrow(n);
    for(int i=0;i<n;++i) {
      cell_clearance[i]=std::max(0.0,static_cast<double>(safe_clearance_[i])*res-
                                    std::sqrt(2.0)*res);
      cell_narrow[i]=std::max(0.0,circumscribed+0.15-cell_clearance[i])/
                    (circumscribed+0.15);
    }
    auto cell_free=[&](int x,int y,double clearance_floor) {
      if(x<0||y<0||x>=w||y>=h) return false;
      const int idx=y*w+x;
      if(idx==src||idx==dst) return true;
      // Do not inflate by a circumscribed disk here: that disconnects valid
      // narrow passages for the oriented rectangular body. Clearance biases
      // the 2-D topology search; exact swept OBB-SAT validates every proposal.
      return safe_clearance_[idx]!=0 &&
             (clearance_floor<=0.0 || cell_clearance[idx]>=clearance_floor);
    };
    auto find_cells=[&](double clearance_weight,double clearance_floor) {
      std::vector<double> gscore(n,std::numeric_limits<double>::infinity());
      std::vector<int> parent(n,-1);
      std::vector<uint8_t> closed(n,0);
      std::priority_queue<RouteItem,std::vector<RouteItem>,RouteCmp> open;
      auto heuristic=[&](int idx) {
        return res*std::hypot(double(idx%w-gx),double(idx/w-gy));
      };
      gscore[src]=0;open.push({heuristic(src),0,src});
      while(!open.empty()) {
        const auto cur=open.top();open.pop();
        if(closed[cur.idx] || cur.g>gscore[cur.idx]+1e-9) continue;
        if(cur.idx==dst) break;
        closed[cur.idx]=1;
        const int x=cur.idx%w,y=cur.idx/w;
        for(int k=0;k<8;++k) {
          const int nx=x+dx[k],ny=y+dy[k];
          if(!cell_free(nx,ny,clearance_floor)) continue;
          if(k>=4 && (!cell_free(x+dx[k],y,clearance_floor)||
                      !cell_free(x,y+dy[k],clearance_floor))) continue;
          const int ni=ny*w+nx;
          if(closed[ni]) continue;
          const double step=res*((k<4)?1.0:std::sqrt(2.0));
          const double ng=cur.g+step*(1.0+clearance_weight*cell_narrow[ni]);
          if(ng+1e-9>=gscore[ni]) continue;
          gscore[ni]=ng;parent[ni]=cur.idx;
          open.push({ng+heuristic(ni),ng,ni});
        }
      }
      std::vector<int> cells;
      if(src!=dst && parent[dst]<0) return cells;
      int id=dst;cells.push_back(id);
      while(id!=src) {
        id=parent[id];if(id<0) {cells.clear();return cells;}
        cells.push_back(id);
      }
      std::reverse(cells.begin(),cells.end());
      return cells;
    };
    std::vector<Action> best_proposal;
    double best_cost=std::numeric_limits<double>::infinity();
    std::vector<Action> best_ackermann_proposal;
    double best_ackermann_cost=std::numeric_limits<double>::infinity();
    int ackermann_calls=0,ackermann_successes=0,ackermann_expansions=0;
    int xy_routes=0,duplicate_routes=0;
    int heuristic_heading_attempts=0,start_spin_blocked=0;
    int start_spin_ok=0,route_ok_count=0,route_edge_blocked=0,goal_spin_ok=0;
    int terminal_spin_blocked=0,terminal_pose_rejected=0;
    int dp_terminal_states=0,dp_terminal_spin_blocked=0,dp_pose_rejected=0;
    int dp_max_reachable_waypoint=-1,dp_max_waypoint_count=0;
    int mixed_calls=0,mixed_successes=0;
    int duplicate_waypoint_variants=0;
    bool saw_blocked_edge=false,saw_blocked_terminal_spin=false;
    bool any_xy_route=false;
    int mixed_expansions=0;bool mixed_budget_hit=false;
    double mixed_search_seconds=0.0;
    double cell_search_seconds=0.0,route_heuristic_seconds=0.0,route_dp_seconds=0.0;
    std::vector<std::string> route_details;
    std::vector<std::vector<int>> seen_cell_routes;
    // Clearance floors are optional topology proposals. They steer the center
    // away from point-passable gaps when the usual route candidates cannot
    // yield a swept-safe vehicle path; they never replace exact OBB-SAT.
    const double half_width=(width_+margin_+2*sweep_guard_)/2;
    const std::array<std::pair<double,double>,6> route_configs{{
        {0.0,0.0},{.5,0.0},{1.5,0.0},{3.0,0.0},
        {.5,half_width},{.5,half_width+.065}}};
    for(const auto& config:route_configs) {
      const double clearance_weight=config.first,clearance_floor=config.second;
      if(clearance_floor>0.0 && !best_proposal.empty()) break;
      const auto cell_t0=std::chrono::steady_clock::now();
      const auto cells=find_cells(clearance_weight,clearance_floor);
      cell_search_seconds+=std::chrono::duration<double>(
          std::chrono::steady_clock::now()-cell_t0).count();
      if(cells.empty()) continue;
      if(std::find(seen_cell_routes.begin(),seen_cell_routes.end(),cells)!=seen_cell_routes.end()) {
        ++duplicate_routes;
        continue;
      }
      seen_cell_routes.push_back(cells);
      ++xy_routes;
      any_xy_route=true;
      std::vector<Pose> raw;raw.push_back(start_);
      for(size_t i=1;i<cells.size();++i) {
        const int cell=cells[i];
        Pose p{ox+((cell%w)+.5)*res,oy+((cell/w)+.5)*res,start_.th};
        if(std::hypot(p.x-raw.back().x,p.y-raw.back().y)>.2*res) raw.push_back(p);
      }
      if(std::hypot(goal_.x-raw.back().x,goal_.y-raw.back().y)>.2*res)
        raw.push_back(Pose{goal_.x,goal_.y,start_.th});
      std::vector<double> headings{start_.th,goal_.th};
      for(int k=0;k<8;++k) headings.push_back(k*PI/4.0);
      if(raw.size()>1)
        headings.push_back(std::atan2(raw[1].y-raw[0].y,raw[1].x-raw[0].x));
      std::vector<double> unique_headings;
      for(double heading:headings) {
        bool duplicate=false;
        for(double prior:unique_headings)
          if(std::abs(wrap(heading-prior))<1e-5) {duplicate=true;break;}
        if(!duplicate) unique_headings.push_back(heading);
      }
      const auto route_heuristic_t0=std::chrono::steady_clock::now();
      if(enable_route_heuristic_) for(double heading:unique_headings) {
      ++heuristic_heading_attempts;
      Pose p=start_;std::vector<Action> proposal;
      const double start_turn=wrap(heading-p.th);
      if(std::abs(start_turn)>1e-5) {
        Action spin{S,start_turn,0};if(!action_clear(p,spin,route_sweep_step_)) {
          ++start_spin_blocked;
          continue;
        }
        proposal.push_back(spin);p=advance(p,spin);
      }
      ++start_spin_ok;
      bool route_ok=true;size_t i=0;
      while(i+1<raw.size()) {
        size_t chosen=i;Action chosen_action{O,0,0};
        for(size_t j=raw.size()-1;j>i;--j) {
          const double vx=raw[j].x-p.x,vy=raw[j].y-p.y;
          const double d=std::hypot(vx,vy);if(d<1e-8) {chosen=j;break;}
          double phi=wrap(std::atan2(vy,vx)-p.th),amount=d;
          if(phi>PI/2) {phi-=PI;amount=-d;}
          if(phi<-PI/2) {phi+=PI;amount=-d;}
          if(amount<0.0) continue;
          Action a{O,amount,phi};
          if(action_clear(p,a,route_sweep_step_)) {chosen=j;chosen_action=a;break;}
        }
        if(chosen==i) {route_ok=false;saw_blocked_edge=true;++route_edge_blocked;break;}
        if(std::abs(chosen_action.amount)>1e-8) {
          proposal.push_back(chosen_action);p=advance(p,chosen_action);
        }
        i=chosen;
      }
      if(!route_ok) continue;
      ++route_ok_count;
      const double goal_turn=wrap(goal_.th-p.th);
      if(std::abs(goal_turn)>yaw_tol_) {
        Action spin{S,goal_turn,0};if(!action_clear(p,spin,route_sweep_step_)) {
          saw_blocked_terminal_spin=true;
          ++terminal_spin_blocked;
          continue;
        }
        proposal.push_back(spin);p=advance(p,spin);
      }
      ++goal_spin_ok;
      if(std::hypot(p.x-goal_.x,p.y-goal_.y)>xy_tol_ ||
         std::abs(wrap(p.th-goal_.th))>yaw_tol_) {
        ++terminal_pose_rejected;
        continue;
      }
      if(direction_switch_count(proposal)>0 || has_reverse_motion(proposal)) continue;
      const double candidate_cost=action_sequence_cost(proposal);
      if(candidate_cost<best_cost) {
        best_cost=candidate_cost;best_proposal=std::move(proposal);
      }
      }
      if(enable_route_heuristic_)
        route_heuristic_seconds+=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-route_heuristic_t0).count();

      // Route-local pose/mode dynamic program.  The former proposal kept one
      // body heading for the entire route, which prevents a cheap Spin at an
      // intermediate open patch from avoiding a long oblique/reverse segment.
      // Build sparse waypoints from the 2-D topology and let each edge choose
      // an O translation heading, with an optional swept-checked Spin at its
      // source.  This is a general graph optimization, not a maneuver template.
      const auto route_dp_t0=std::chrono::steady_clock::now();
      std::vector<double> route_arc(raw.size(),0.0);
      for(size_t ri=1;ri<raw.size();++ri)
        route_arc[ri]=route_arc[ri-1]+
            std::hypot(raw[ri].x-raw[ri-1].x,raw[ri].y-raw[ri-1].y);
      auto point_at_arc=[&](double s) {
        s=std::max(0.0,std::min(route_arc.back(),s));
        auto it=std::lower_bound(route_arc.begin(),route_arc.end(),s);
        if(it==route_arc.begin()) return raw.front();
        if(it==route_arc.end()) return raw.back();
        const size_t hi=static_cast<size_t>(it-route_arc.begin()),lo=hi-1;
        const double span=route_arc[hi]-route_arc[lo];
        const double t=span>1e-9?(s-route_arc[lo])/span:0.0;
        return Pose{raw[lo].x+t*(raw[hi].x-raw[lo].x),
                    raw[lo].y+t*(raw[hi].y-raw[lo].y),0.0};
      };
      auto signed_turn_demand=[&](double s) {
        const double left=std::max(0.0,s-route_graph_turn_window_);
        const double right=std::min(route_arc.back(),s+route_graph_turn_window_);
        if(right-left<0.15) return 0.0;
        const Pose a=point_at_arc(left),b=point_at_arc(s),c=point_at_arc(right);
        if(std::hypot(b.x-a.x,b.y-a.y)<0.05 ||
           std::hypot(c.x-b.x,c.y-b.y)<0.05) return 0.0;
        const double h1=std::atan2(b.y-a.y,b.x-a.x);
        const double h2=std::atan2(c.y-b.y,c.x-b.x);
        return wrap(h2-h1);
      };
      auto turn_demand=[&](double s) {
        return std::abs(signed_turn_demand(s));
      };
      double since_waypoint=0.0,since_base_waypoint=0.0,max_turn_demand=0.0;
      int dense_waypoints=0,intermediate_waypoints=0,sparse_waypoints=0;
      int forced_waypoints=0,forced_corner=0,forced_curvature_sign=0;
      int forced_curvature_extrema=0,forced_curvature_rate_extrema=0;
      int forced_clearance_minima=0,base_waypoints=0;
      std::vector<std::pair<double,Pose>> adaptive_samples{{0.0,raw.front()}};
      std::vector<std::pair<double,Pose>> base_samples{{0.0,raw.front()}};

      // Continuous spacing uses route shape and conservative free-space
      // clearance. Curvature extrema and meaningful raw turns remain explicit
      // anchors so a mode boundary cannot jump across the geometric event.
      // The disabled branch retains the previous binary sampler exactly for
      // paired baseline probes.
      const unsigned kCorner=1u,kCurvatureSign=2u,kCurvatureExtremum=4u;
      const unsigned kCurvatureRateExtremum=8u,kClearanceMinimum=16u;
      std::vector<double> signed_curvature(raw.size(),0.0);
      std::vector<double> curvature_rate(raw.size(),0.0);
      std::vector<double> route_clearance(raw.size(),0.0);
      std::vector<unsigned> forced_reason(raw.size(),0u);
      // A quarter-turn over the lookahead window is full curvature demand.
      // The binary threshold remains useful for legacy sampling, but using its
      // lower value here would treat every 45-degree raster transition as a
      // request for the densest spacing.
      const double curvature_reference=std::max(route_graph_turn_threshold_,PI/2.0);
      const double curvature_scale=curvature_reference/
          std::max(0.15,2.0*route_graph_turn_window_);
      const double curvature_rate_scale=curvature_scale/
          std::max(0.15,route_graph_turn_window_);
      const double clearance_target=
          std::hypot((length_+margin_)/2.0,(width_+margin_)/2.0)+0.35;
      if(route_graph_continuous_adaptive_) {
        for(size_t ri=0;ri<raw.size();++ri) {
          route_clearance[ri]=clearance_lower_bound(raw[ri].x,raw[ri].y);
          if(ri>0 && ri+1<raw.size()) {
            const double span=std::max(0.15,
                std::min(route_arc.back(),route_arc[ri]+route_graph_turn_window_)-
                std::max(0.0,route_arc[ri]-route_graph_turn_window_));
            signed_curvature[ri]=signed_turn_demand(route_arc[ri])/span;
            const double h0=std::atan2(raw[ri].y-raw[ri-1].y,
                                       raw[ri].x-raw[ri-1].x);
            const double h1=std::atan2(raw[ri+1].y-raw[ri].y,
                                       raw[ri+1].x-raw[ri].x);
            if(std::abs(wrap(h1-h0))>=route_graph_turn_threshold_)
              forced_reason[ri]|=kCorner;
          }
        }
        for(size_t ri=1;ri+1<raw.size();++ri) {
          const double left_s=std::max(0.0,
              route_arc[ri]-0.5*route_graph_turn_window_);
          const double right_s=std::min(route_arc.back(),
              route_arc[ri]+0.5*route_graph_turn_window_);
          auto left_it=std::lower_bound(route_arc.begin(),route_arc.end(),left_s);
          auto right_it=std::lower_bound(route_arc.begin(),route_arc.end(),right_s);
          const size_t left_i=std::min(raw.size()-1,
              static_cast<size_t>(left_it-route_arc.begin()));
          const size_t right_i=std::min(raw.size()-1,
              static_cast<size_t>(right_it-route_arc.begin()));
          const double ds=std::max(1e-4,route_arc[right_i]-route_arc[left_i]);
          curvature_rate[ri]=(signed_curvature[right_i]-
                              signed_curvature[left_i])/ds;
        }
        for(size_t ri=1;ri+1<raw.size();++ri) {
          const double before=signed_curvature[ri]-signed_curvature[ri-1];
          const double after=signed_curvature[ri+1]-signed_curvature[ri];
          const double prominence=std::abs(before)+std::abs(after);
          const bool curvature_extremum=before*after<=0.0 &&
              std::abs(signed_curvature[ri])>=0.15*curvature_scale &&
              prominence>=0.20*curvature_scale;
          if(curvature_extremum) forced_reason[ri]|=kCurvatureExtremum;
          const double rate_radius=std::max(route_graph_turn_window_,
              0.75*std::max(route_graph_dense_spacing_,
                            route_graph_sparse_spacing_));
          const double left_s=std::max(0.0,route_arc[ri]-rate_radius);
          const double right_s=std::min(route_arc.back(),route_arc[ri]+rate_radius);
          auto left_it=std::lower_bound(route_arc.begin(),route_arc.end(),left_s);
          auto right_it=std::lower_bound(route_arc.begin(),route_arc.end(),right_s);
          const size_t left_i=static_cast<size_t>(left_it-route_arc.begin());
          const size_t right_i=std::min(raw.size()-1,
              static_cast<size_t>(right_it-route_arc.begin()));
          const double rate_magnitude=std::abs(curvature_rate[ri]);
          double local_rate_min=rate_magnitude;
          bool rate_is_local_peak=true;
          for(size_t rj=left_i;rj<=right_i;++rj) {
            const double neighbor=std::abs(curvature_rate[rj]);
            local_rate_min=std::min(local_rate_min,neighbor);
            if(neighbor>rate_magnitude+1e-9 ||
               (std::abs(neighbor-rate_magnitude)<=1e-9 && rj<ri))
              rate_is_local_peak=false;
          }
          const bool curvature_rate_peak=rate_is_local_peak &&
              rate_magnitude>=0.35*curvature_rate_scale &&
              rate_magnitude-local_rate_min>=0.25*curvature_rate_scale &&
              std::abs(signed_curvature[ri])>=0.15*curvature_scale;
          if(curvature_rate_peak) forced_reason[ri]|=kCurvatureRateExtremum;
          if(signed_curvature[ri-1]*signed_curvature[ri+1]<0.0 &&
             std::max(std::abs(signed_curvature[ri-1]),
                      std::abs(signed_curvature[ri+1]))>=0.20*curvature_scale)
            forced_reason[ri]|=kCurvatureSign;
          const double prev=route_clearance[ri-1],here=route_clearance[ri],
                       next=route_clearance[ri+1];
          const bool clearance_min=(here<prev-0.02 && here<=next+0.02) ||
                                   (here<=prev+0.02 && here<next-0.02);
          if(clearance_min) forced_reason[ri]|=kClearanceMinimum;
        }
      }
      for(size_t ri=1;ri<raw.size();++ri) {
        const double ds=std::hypot(raw[ri].x-raw[ri-1].x,
                                   raw[ri].y-raw[ri-1].y);
        since_waypoint+=ds;
        since_base_waypoint+=ds;
        const double demand=turn_demand(route_arc[ri]);
        max_turn_demand=std::max(max_turn_demand,demand);
        double normalized_demand=0.0;
        double spacing=route_graph_sparse_spacing_;
        if(route_graph_continuous_adaptive_) {
          const double curvature_pressure=std::max(0.0,std::min(1.0,
              demand/std::max(1e-6,curvature_reference)));
          const double clearance_pressure=std::max(0.0,std::min(1.0,
              (clearance_target-route_clearance[ri])/
                  std::max(1e-6,clearance_target)));
          normalized_demand=std::max(curvature_pressure,clearance_pressure);
          const double dense=std::min(route_graph_dense_spacing_,
                                      route_graph_sparse_spacing_);
          const double sparse=std::max(route_graph_dense_spacing_,
                                       route_graph_sparse_spacing_);
          spacing=std::max(dense,std::min(sparse,
              sparse-normalized_demand*(sparse-dense)));
        } else {
          normalized_demand=demand>=route_graph_turn_threshold_?1.0:0.0;
          spacing=normalized_demand>0.5?route_graph_dense_spacing_:
                                         route_graph_sparse_spacing_;
        }
        const unsigned reason=route_graph_continuous_adaptive_?
                              forced_reason[ri]:0u;
        const bool forced=reason!=0u;
        if((since_waypoint>=spacing || forced) && ri+1<raw.size()) {
          adaptive_samples.push_back({route_arc[ri],raw[ri]});
          since_waypoint=0.0;
          if(normalized_demand>=0.75) ++dense_waypoints;
          else if(normalized_demand<=0.25) ++sparse_waypoints;
          else ++intermediate_waypoints;
          if(forced) {
            ++forced_waypoints;
            if(reason&kCorner) ++forced_corner;
            if(reason&kCurvatureSign) ++forced_curvature_sign;
            if(reason&kCurvatureExtremum) ++forced_curvature_extrema;
            if(reason&kCurvatureRateExtremum) ++forced_curvature_rate_extrema;
            if(reason&kClearanceMinimum) ++forced_clearance_minima;
          }
        }
        // Keep the previously validated raster-vertex schedule as an
        // independent fallback graph while the turn-adaptive graph explores
        // a different spacing schedule on the same 2-D route.
        if(since_base_waypoint>=route_graph_sparse_spacing_ &&
           ri+1<raw.size()) {
          base_samples.push_back({route_arc[ri],raw[ri]});
          since_base_waypoint=0.0;
          ++base_waypoints;
        }
      }
      auto make_waypoints=[&](std::vector<std::pair<double,Pose>>& samples) {
        samples.push_back({route_arc.back(),raw.back()});
        std::sort(samples.begin(),samples.end(),
            [](const auto& a,const auto& b) { return a.first<b.first; });
        std::vector<Pose> result;result.reserve(samples.size());
        for(const auto& sample:samples) {
          if(result.empty() || std::hypot(sample.second.x-result.back().x,
                                          sample.second.y-result.back().y)>1e-5)
            result.push_back(sample.second);
        }
        return result;
      };
      const std::vector<Pose> wp_adaptive=make_waypoints(adaptive_samples);
      const std::vector<Pose> wp_base=make_waypoints(base_samples);
      const std::array<std::pair<const std::vector<Pose>*,const char*>,2>
          waypoint_variants{{{&wp_base,"base"},{&wp_adaptive,"adaptive"}}};
      auto same_waypoints=[](const std::vector<Pose>& a,
                             const std::vector<Pose>& b) {
        if(a.size()!=b.size()) return false;
        for(size_t i=0;i<a.size();++i)
          if(std::hypot(a[i].x-b[i].x,a[i].y-b[i].y)>1e-7) return false;
        return true;
      };
      for(const auto& waypoint_variant:waypoint_variants) {
      if(waypoint_variant.second[0]=='a' &&
         same_waypoints(wp_base,wp_adaptive)) {
        ++duplicate_waypoint_variants;
        continue;
      }
      const std::vector<Pose>& wp=*waypoint_variant.first;
      std::vector<double> yaw_bins; yaw_bins.reserve(9);
      for(int k=0;k<8;++k) yaw_bins.push_back(-PI+k*PI/4.0);
      yaw_bins.push_back(start_.th);
      const int H=static_cast<int>(yaw_bins.size()), Wp=static_cast<int>(wp.size());
      // Preserve incoming steering and gear in the DP key.  phi is quantized
      // only for the state key; candidate actions retain their exact angle.
      // This restores the Markov property of the discrete cost approximation
      // (the previous pose-only DP could not price O-angle TV correctly).
      constexpr int PhiBins=17,NoPhi=17,Gears=3;
      const double phi_bin_width=PI/16.0;
      const int state_count=Wp*H*(PhiBins+1)*Gears;
      int route_candidate_edges=0,route_feasible_edges=0;
      auto state_id=[&](int wi,int hi,int pi,int gi) {
        return (((wi*H+hi)*(PhiBins+1)+pi)*Gears+gi);
      };
      std::vector<double> dp(state_count,std::numeric_limits<double>::infinity());
      std::vector<int> parent_state(state_count,-1);
      std::vector<Action> parent_spin(state_count),parent_move(state_count);
      int initial_h=static_cast<int>(std::min_element(yaw_bins.begin(),yaw_bins.end(),[&](double a,double b){
        return std::abs(wrap(a-start_.th))<std::abs(wrap(b-start_.th));
      })-yaw_bins.begin());
      const int initial_state=state_id(0,initial_h,NoPhi,1); // gear=0 (none)
      dp[initial_state]=0.0;
      for(int wi=0;wi<Wp-1;++wi) {
        const Pose base{wp[wi].x,wp[wi].y,0.0};
        std::vector<uint8_t> spin_ok(H*H,0);
        for(int hi=0;hi<H;++hi) for(int hj=0;hj<H;++hj) {
          const double turn=wrap(yaw_bins[hj]-yaw_bins[hi]);
          spin_ok[hi*H+hj]=std::abs(turn)<=1e-5 ||
              action_clear(Pose{base.x,base.y,yaw_bins[hi]},Action{S,turn,0},route_sweep_step_);
        }
        std::vector<std::vector<Action>> moves(std::min(7,Wp-wi),std::vector<Action>(H));
        std::vector<std::vector<uint8_t>> move_ok(moves.size(),std::vector<uint8_t>(H,0));
        for(int wj=wi+1;wj<std::min(Wp,wi+7);++wj) {
          const int offset=wj-wi;
          for(int hj=0;hj<H;++hj) {
            const double vx=wp[wj].x-base.x,vy=wp[wj].y-base.y;
            const double distance=std::hypot(vx,vy);
            if(distance<1e-7 || distance>route_graph_max_edge_length_) continue;
            const double direction=std::atan2(vy,vx);
            double phi=wrap(direction-yaw_bins[hj]),amount=distance;
            if(phi>PI/2) {phi-=PI;amount=-distance;}
            if(phi<-PI/2) {phi+=PI;amount=-distance;}
            if(amount<0.0) continue;
            Action move{O,amount,phi};
            const Pose from{base.x,base.y,yaw_bins[hj]};
            const Pose moved=advance(from,move);
            if(action_clear(from,move,route_sweep_step_) &&
               std::hypot(moved.x-wp[wj].x,moved.y-wp[wj].y)<=1e-5) {
              moves[offset][hj]=move;move_ok[offset][hj]=1;
            }
          }
        }
        for(int hi=0;hi<H;++hi) for(int pi=0;pi<=NoPhi;++pi)
          for(int gi=0;gi<Gears;++gi) {
            const int from=state_id(wi,hi,pi,gi);
            if(!std::isfinite(dp[from])) continue;
            const int prior_gear=gi-1; // {-1, no-gear=0, +1}
            for(int wj=wi+1;wj<std::min(Wp,wi+7);++wj) {
              const int offset=wj-wi;
              for(int hj=0;hj<H;++hj) {
                ++route_candidate_edges;
                const double turn=wrap(yaw_bins[hj]-yaw_bins[hi]);
                if(!spin_ok[hi*H+hj] || !move_ok[offset][hj]) continue;
                ++route_feasible_edges;
                const Action& move=moves[offset][hj];
                const double distance=std::abs(move.amount);
                const int gear=move.amount<0?-1:1;
                if(prior_gear!=0 && prior_gear!=gear) continue;
                const bool spun=std::abs(turn)>1e-5;
                double edge=distance*(1+(gear<0?w_reverse_:0)+
                                      w_oblique_use_*std::abs(move.control)+
                                      w_oblique_cruise_*
                                        oblique_cruise_factor(move.control));
                if(prior_gear!=0 && prior_gear!=gear) edge+=w_direction_;
                // A Spin clears the O-control history in the sequence cost;
                // without Spin, charge steering-angle variation.
                if(!spun && pi!=NoPhi) {
                  const double prior_phi=-PI/2+std::min(pi,PhiBins-1)*phi_bin_width;
                  const double delta=std::abs(wrap(move.control-prior_phi));
                  edge+=w_oblique_tv_*delta+
                        w_oblique_event_score_*delta/(delta+.03);
                }
                if(spun) edge+=w_spin_*std::abs(turn)+
                    (wi==0?w_switch_:2*w_switch_);
                int pbin=static_cast<int>(std::lround((move.control+PI/2)/phi_bin_width));
                pbin=std::max(0,std::min(PhiBins-1,pbin));
                const int gear_index=gear+1;
                const int to=state_id(wj,hj,pbin,gear_index);
                if(dp[from]+edge+1e-9>=dp[to]) continue;
                dp[to]=dp[from]+edge;parent_state[to]=from;
                parent_spin[to]=Action{S,turn,0};parent_move[to]=move;
              }
            }
          }
      }
      int route_dp_max_reachable_waypoint=-1;
      for(int wi=0;wi<Wp;++wi) {
        bool reachable=false;
        for(int hi=0;hi<H&&!reachable;++hi)
          for(int pi=0;pi<=NoPhi&&!reachable;++pi)
            for(int gi=0;gi<Gears;++gi)
              if(std::isfinite(dp[state_id(wi,hi,pi,gi)])) {
                reachable=true;
                break;
              }
        if(reachable) route_dp_max_reachable_waypoint=wi;
      }
      if(route_dp_max_reachable_waypoint>dp_max_reachable_waypoint) {
        dp_max_reachable_waypoint=route_dp_max_reachable_waypoint;
        dp_max_waypoint_count=Wp;
      }
      double dp_cost=std::numeric_limits<double>::infinity();
      int final_state=-1;Action final_spin{S,0,0};
      for(int hi=0;hi<H;++hi) for(int pi=0;pi<PhiBins;++pi)
        for(int gi=0;gi<Gears;++gi) {
        const int state=state_id(Wp-1,hi,pi,gi);
        if(!std::isfinite(dp[state])) continue;
        ++dp_terminal_states;
        const double turn=wrap(goal_.th-yaw_bins[hi]);
        Action spin{S,turn,0};
        if(std::abs(turn)>yaw_tol_ && !action_clear(wp.back(),spin,route_sweep_step_)) {
          ++dp_terminal_spin_blocked;
          continue;
        }
        const double total=dp[state]+(std::abs(turn)>yaw_tol_?
            w_spin_*std::abs(turn)+w_switch_:0.0);
        if(total<dp_cost) {dp_cost=total;final_state=state;final_spin=spin;}
      }
      if(final_state>=0) {
        std::vector<std::pair<Action,Action>> reversed_edges;
        for(int state=final_state;state!=initial_state && parent_state[state]>=0;
            state=parent_state[state])
          reversed_edges.push_back({parent_spin[state],parent_move[state]});
        std::vector<Action> dynamic_proposal;
        for(auto it=reversed_edges.rbegin();it!=reversed_edges.rend();++it) {
          if(std::abs(it->first.amount)>1e-5) dynamic_proposal.push_back(it->first);
          dynamic_proposal.push_back(it->second);
        }
        if(std::abs(final_spin.amount)>yaw_tol_) dynamic_proposal.push_back(final_spin);
        Pose check=start_;
        for(const auto& a:dynamic_proposal) check=advance(check,a);
        if(direction_switch_count(dynamic_proposal)==0 &&
           !has_reverse_motion(dynamic_proposal) &&
           std::hypot(check.x-goal_.x,check.y-goal_.y)<=xy_tol_ &&
           std::abs(wrap(check.th-goal_.th))<=yaw_tol_) {
          const double exact_cost=action_sequence_cost(dynamic_proposal);
          if(exact_cost<best_cost) {best_cost=exact_cost;best_proposal=std::move(dynamic_proposal);}
        } else ++dp_pose_rejected;
      }
      std::vector<Action> mixed_proposal;double mixed_cost;
      // The sparse mixed graph is costlier than a 2-D route. Cover the
      // shortest and middle-clearance routes; the strongest-clearance route
      // still contributes its cheap O/S proposal without repeating A-edge SAT.
      const bool mixed_topology=(clearance_weight==0.0 ||
                                 clearance_weight==1.5 || clearance_floor>0.0);
      const int remaining_graph_budget=std::max(0,route_graph_max_expansions_-mixed_expansions);
      int route_mixed_max_waypoint=-1,route_mixed_expansions=0;
      if(mixed_topology && remaining_graph_budget>0) {
        int expanded=0;bool cap=false;
        ++mixed_calls;
        const auto mixed_t0=std::chrono::steady_clock::now();
        const bool mixed_ok=sparse_mixed_route(wp,mixed_proposal,mixed_cost,best_cost,
                                               remaining_graph_budget,expanded,cap,
                                               route_mixed_max_waypoint);
        mixed_search_seconds+=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-mixed_t0).count();
        if(mixed_ok) {
          ++mixed_successes;
          if(mixed_cost<best_cost) {
            best_cost=mixed_cost;best_proposal=std::move(mixed_proposal);
          }
        }
        int ack_expanded=0,ack_max_waypoint=-1;bool ack_cap=false;
        const int ack_budget=std::max(0,remaining_graph_budget-expanded);
        if(ack_budget>0) {
          std::vector<Action> ack_proposal;double ack_cost=best_ackermann_cost;
          ++ackermann_calls;
          const bool ack_ok=sparse_mixed_route(
              wp,ack_proposal,ack_cost,best_ackermann_cost,ack_budget,
              ack_expanded,ack_cap,ack_max_waypoint,false);
          if(ack_ok) {
            ++ackermann_successes;
            if(ack_cost<best_ackermann_cost) {
              best_ackermann_cost=ack_cost;
              best_ackermann_proposal=std::move(ack_proposal);
            }
          }
          ackermann_expansions+=ack_expanded;
        }
        mixed_expansions+=expanded+ack_expanded;
        mixed_budget_hit=mixed_budget_hit||cap||ack_cap;
        route_mixed_expansions=expanded;
      }
      route_details.push_back("floor:"+std::to_string(clearance_floor)+
                              ",weight:"+std::to_string(clearance_weight)+
                              ",sampling:"+waypoint_variant.second+
                              ",path_m:"+std::to_string(route_arc.back())+
                              ",wps:"+std::to_string(Wp)+
                              ",dense_wps:"+std::to_string(dense_waypoints)+
                              ",intermediate_wps:"+
                                  std::to_string(intermediate_waypoints)+
                              ",sparse_wps:"+std::to_string(sparse_waypoints)+
                              ",forced_wps:"+std::to_string(forced_waypoints)+
                              ",forced_corner:"+std::to_string(forced_corner)+
                              ",forced_curv_sign:"+
                                  std::to_string(forced_curvature_sign)+
                              ",forced_curv_extrema:"+
                                  std::to_string(forced_curvature_extrema)+
                              ",forced_curv_rate_extrema:"+
                                  std::to_string(forced_curvature_rate_extrema)+
                              ",forced_clearance_min:"+
                                  std::to_string(forced_clearance_minima)+
                              ",base_wps:"+std::to_string(base_waypoints)+
                              ",dp_states:"+std::to_string(state_count)+
                              ",candidate_edges:"+
                                  std::to_string(route_candidate_edges)+
                              ",feasible_edges:"+
                                  std::to_string(route_feasible_edges)+
                              ",max_turn:"+std::to_string(max_turn_demand)+
                              ",dp:"+std::to_string(route_dp_max_reachable_waypoint)+
                              ",mixed:"+std::to_string(route_mixed_max_waypoint)+
                              ",expanded:"+std::to_string(route_mixed_expansions)+
                              ",dp_x:"+std::to_string(wp[std::max(0,route_dp_max_reachable_waypoint)].x)+
                              ",dp_y:"+std::to_string(wp[std::max(0,route_dp_max_reachable_waypoint)].y));
      }
      route_dp_seconds+=std::chrono::duration<double>(
          std::chrono::steady_clock::now()-route_dp_t0).count();
    }
    auto diagnostics=[&]() {
      std::string details;
      for(const auto& entry:route_details) {
        if(!details.empty()) details+="|";
        details+=entry;
      }
      return "xy_routes="+std::to_string(xy_routes)+
             " duplicate_routes="+std::to_string(duplicate_routes)+
             " duplicate_waypoint_variants="+
                 std::to_string(duplicate_waypoint_variants)+
             " heuristic_headings="+std::to_string(heuristic_heading_attempts)+
             " start_spin_blocked="+std::to_string(start_spin_blocked)+
             " start_spin_ok="+std::to_string(start_spin_ok)+
             " heuristic_route_ok="+std::to_string(route_ok_count)+
             " route_edge_blocked="+std::to_string(route_edge_blocked)+
             " terminal_spin_blocked="+std::to_string(terminal_spin_blocked)+
             " terminal_pose_rejected="+std::to_string(terminal_pose_rejected)+
             " dp_terminal_states="+std::to_string(dp_terminal_states)+
             " dp_max_reachable_waypoint="+std::to_string(dp_max_reachable_waypoint)+
             " dp_waypoint_count="+std::to_string(dp_max_waypoint_count)+
             " dp_terminal_spin_blocked="+std::to_string(dp_terminal_spin_blocked)+
             " dp_pose_rejected="+std::to_string(dp_pose_rejected)+
             " mixed_calls="+std::to_string(mixed_calls)+
             " mixed_successes="+std::to_string(mixed_successes)+
             " ackermann_calls="+std::to_string(ackermann_calls)+
             " ackermann_successes="+std::to_string(ackermann_successes)+
             " ackermann_expansions="+std::to_string(ackermann_expansions)+
             " mixed_expansions="+std::to_string(mixed_expansions)+
             " mixed_budget_hit="+(mixed_budget_hit?"1":"0")+
             " cell_astar_seconds="+std::to_string(cell_search_seconds)+
             " route_heuristic_seconds="+std::to_string(route_heuristic_seconds)+
             " route_dp_seconds="+std::to_string(route_dp_seconds)+
             " route_graph_wall_seconds="+
                 std::to_string(route_dp_seconds)+
             " mixed_search_seconds="+std::to_string(mixed_search_seconds)+
             " route_details="+details;
    };
    if(!any_xy_route) {
      reason="xy_astar_disconnected "+diagnostics();
      return false;
    }
    if(best_proposal.empty() && best_ackermann_proposal.empty()) {
      if(enable_route_heuristic_&&start_spin_ok==0) reason="initial_spin_swept_collision";
      else if(route_ok_count==0) reason=saw_blocked_edge?"oblique_route_swept_collision":"no_route_candidate";
      else if(goal_spin_ok==0) reason=saw_blocked_terminal_spin?"terminal_spin_swept_collision":"no_terminal_candidate";
      else reason="terminal_tolerance";
      reason+=" "+diagnostics();
      return false;
    }
    // Compare swept-safe A/Spin and mixed candidates using the same score.
    if(!best_ackermann_proposal.empty() &&
       (best_proposal.empty() || best_ackermann_cost<best_cost)) {
      best_proposal=std::move(best_ackermann_proposal);
      best_cost=best_ackermann_cost;
    }
    reason="route_candidate "+diagnostics();
    actions=std::move(best_proposal);cost=best_cost;
    return true;
  }
  bool search(std::vector<Action>& solution,int& iterations,bool guided,
              double& solution_cost,const std::vector<Action>* incumbent=nullptr,
              double incumbent_cost=std::numeric_limits<double>::infinity(),
              int node_budget=-1,double lattice_scale=1.0,
              bool allow_oblique=true,bool allow_reverse=false,
              std::vector<Action>* untrusted_policy_proposal=nullptr) {
    if (!pose_clear(start_) || !pose_clear(goal_)) return false;
    const bool incumbent_allowed=incumbent &&
        (allow_reverse || !has_reverse_motion(*incumbent)) &&
        (allow_shunting_ || shunting_violation(*incumbent).empty());
    const int natural_budget=(incumbent_allowed && std::isfinite(incumbent_cost))?
                             std::min(max_iter_,incumbent_iter_):max_iter_;
    const int search_budget=node_budget>=0?std::min(natural_budget,node_budget):natural_budget;
    if(search_budget<=0) return false;
    const bool proposal_search=untrusted_policy_proposal!=nullptr;
    const double lattice_step=step_*lattice_scale;
    const double lattice_xy_res=xy_res_*lattice_scale;
    const auto search_begin=std::chrono::steady_clock::now();
    std::vector<Node> nodes; nodes.reserve(static_cast<size_t>(search_budget)*2);
    nodes.push_back({start_,A,1,0,-1,false,false,false,0,{A,0,0}});
    std::priority_queue<QueueItem> open;
    open.push({heuristic(start_,guided),0});
    std::unordered_map<Key,double,KeyHash> best;
    // Rehashing this large state table recomputes every stored key's hash.
    // Reserve once using the same capacity estimate as the node pool.
    best.reserve(static_cast<size_t>(search_budget)*2);
    best[key(nodes.front(),lattice_xy_res,proposal_search)]=0;
    const double kmax=1.0/radius_;
    double best_terminal_cost=incumbent_allowed?incumbent_cost:
        std::numeric_limits<double>::infinity();
    double best_untrusted_cost=std::numeric_limits<double>::infinity();
    std::vector<Action> best_actions=incumbent_allowed?
        *incumbent:std::vector<Action>{};
    while (!open.empty() && iterations<search_budget) {
      if(allow_reverse && std::chrono::duration<double>(
          std::chrono::steady_clock::now()-search_begin).count()>=reverse_fallback_budget_s_) break;
      auto qi=open.top(); open.pop();
      Node cur=nodes[qi.id];
      auto it=best.find(key(cur,lattice_xy_res,proposal_search));
      if (it==best.end() || cur.g>it->second+1e-8) continue;
      // Use only the obstacle-free bound for pruning, never the raster guide.
      // motion_cost_lb omits all optional penalties, avoiding dependence on
      // discrepancies between edge scoring and final whole-sequence scoring.
      if(cur.motion_cost_lb+heuristic(cur.p,false,cur.mode,cur.parent<0)>=
         best_terminal_cost-1e-8) continue;
      ++iterations;
      auto consider_terminal=[&](const std::vector<Action>& suffix) {
        std::vector<Action> candidate;
        int parent=qi.id;
        while(nodes[parent].parent>=0) {
          if(std::abs(nodes[parent].action.amount)>1e-12)
            candidate.push_back(nodes[parent].action);
          parent=nodes[parent].parent;
        }
        std::reverse(candidate.begin(),candidate.end());
        candidate.insert(candidate.end(),suffix.begin(),suffix.end());
        if(!allow_reverse && has_reverse_motion(candidate)) return;
        const double terminal_cost=action_sequence_cost(candidate);
        const std::string violation=shunting_violation(candidate);
        if(!allow_shunting_ && !proposal_search && !violation.empty()) return;
        if(proposal_search && !violation.empty() &&
           terminal_cost<best_untrusted_cost) {
          best_untrusted_cost=terminal_cost;
          *untrusted_policy_proposal=candidate;
        }
        if (terminal_cost<best_terminal_cost) {
          if(!violation.empty()) return;
          best_terminal_cost=terminal_cost;
          best_actions=std::move(candidate);
        }
      };
      std::vector<std::vector<Action>> connectors;
      const double goal_distance=std::hypot(cur.p.x-goal_.x,cur.p.y-goal_.y);
      if(guided) {
        if(goal_distance<=terminal_radius_+1.25 && iterations%4==0)
          terminal_candidates(cur,connectors,terminal_radius_+1.25,true,
                              allow_oblique,allow_reverse);
      } else if(goal_distance<=.55) {
        terminal_candidates(cur,connectors,.55,false,allow_oblique,allow_reverse);
      }
      for(const auto& connector:connectors) consider_terminal(connector);
      std::vector<Action> suffix;
      if (iterations%12==0 && terminal_rs(cur,suffix,1.8,5.0,allow_reverse) &&
          (allow_reverse || !has_reverse_motion(suffix))) {
        consider_terminal(suffix);
      }
      auto push=[&](Node next,double edge_cost) {
        next.parent=qi.id;
        next.g=cur.g+edge_cost;
        next.motion_cost_lb=cur.motion_cost_lb+
            (next.action.mode==S?w_spin_*std::abs(next.action.amount):
                                 std::abs(next.action.amount))+
            (next.mode!=cur.mode && cur.parent>=0?w_switch_:0.);
        if(next.motion_cost_lb+heuristic(next.p,false,next.mode,false)>=
           best_terminal_cost-1e-8) return;
        if(proposal_search && next.shunt_reversals>3)
          next.shunt_reversals=3;
        Key k=key(next,lattice_xy_res,proposal_search);
        auto old=best.find(k);
        if (old!=best.end() && old->second<=next.g+1e-8) return;
        // A dominated successor cannot enter the queue even if its sweep is
        // clear. Check that cheap condition first; surviving motion edges
        // still pass exactly the same whole-body sweep before best is updated.
        if(std::abs(next.action.amount)>0.0 &&
           !action_clear(cur.p,next.action)) return;
        if(old!=best.end()) old->second=next.g;
        else best.emplace(k,next.g);
        int id=static_cast<int>(nodes.size()); nodes.push_back(next);
        // Preserve the established exploration order. The stronger mode-aware
        // relaxation is used for safe incumbent pruning, not as a new queue
        // bias that might change a budget-limited search's preferred route.
        open.push({next.g+heuristic(next.p,guided),id});
      };
      if (cur.mode==A) {
        for (int gear : {1,-1}) {
          if(!allow_reverse && gear<0) continue;
          if(!allow_reverse && cur.has_translation && gear!=cur.gear) continue;
          const bool reversal=cur.has_translation&&gear!=cur.gear;
          if(!allow_shunting_ && !proposal_search && reversal &&
             (cur.shunt_reversals>=1 ||
              !cur.spun_since_translation))
            continue;
          for (int ci=-2;ci<=2;++ci) {
          Action a{A,gear*lattice_step,ci*kmax/2};
          Node next=cur; next.p=advance(cur.p,a); next.gear=gear;
          next.has_forward=cur.has_forward || gear>0;
          if(reversal) ++next.shunt_reversals;
          next.spun_since_translation=false;
          next.control=ci; next.switched=false;next.has_translation=true;
          next.has_control=true;next.action=a;
          double cost=lattice_step*(1+(gear<0?w_reverse_:0))+
                      (cur.has_translation&&gear!=cur.gear?w_direction_:0.0)+
                      (cur.mode==A&&cur.has_control?
                        w_steer_tv_*std::abs((ci-cur.control)*kmax/2):0.0)+
                      w_curvature_*(ci*kmax/2)*(ci*kmax/2)*lattice_step;
          if(gear<0) cost+=reverse_preference_parts(
              cur.p,{a},cur.has_translation?cur.gear:0).total();
          push(next,cost);
          }
        }
      } else if (cur.mode==O) {
        for (int gear : {1,-1}) {
          if(!allow_reverse && gear<0) continue;
          if(!allow_reverse && cur.has_translation && gear!=cur.gear) continue;
          const bool reversal=cur.has_translation&&gear!=cur.gear;
          if(!allow_shunting_ && !proposal_search && reversal &&
             (cur.shunt_reversals>=1 ||
              !cur.spun_since_translation))
            continue;
          for (int ci=-3;ci<=3;++ci) {
          Action a{O,gear*lattice_step,ci*PI/6};
          Node next=cur; next.p=advance(cur.p,a); next.gear=gear;
          next.has_forward=cur.has_forward || gear>0;
          if(reversal) ++next.shunt_reversals;
          next.spun_since_translation=false;
          next.control=ci; next.switched=false;next.has_translation=true;
          next.has_control=true;next.action=a;
          double cost=lattice_step*(1+(gear<0?w_reverse_:0))+
                      (cur.has_translation&&gear!=cur.gear?w_direction_:0.0)+
                      (cur.mode==O&&cur.has_control?
                        w_oblique_tv_*std::abs((ci-cur.control)*PI/6)+
                        w_oblique_event_score_*
                          (std::abs(ci-cur.control)*PI/6)/
                          (std::abs(ci-cur.control)*PI/6+.03):0.0)+
                      (cur.mode==O?w_oblique_use_*lattice_step*std::abs(a.control):0.0);
          cost+=cur.mode==O?w_oblique_cruise_*lattice_step*
                             oblique_cruise_factor(a.control):0.0;
          if(gear<0) cost+=reverse_preference_parts(
              cur.p,{a},cur.has_translation?cur.gear:0).total();
          push(next,cost);
          }
        }
      } else {
        for (int sign : {-1,1}) {
          Action a{S,sign*PI/18,0};
          Node next=cur; next.p=advance(cur.p,a);
          // Direction-constrained searches have no reversal transition, so
          // retaining Spin history there only fragments equivalent states.
          next.spun_since_translation=allow_reverse;
          next.has_control=false;next.switched=false;next.action=a;
          push(next,w_spin_*std::abs(a.amount));
        }
      }
      if (!cur.switched) for (uint8_t m : {A,O,S}) {
        if(!allow_oblique && m==O) continue;
        if (m==cur.mode) continue;
        if(guided && cur.mode==A && m!=A && goal_distance>mode_radius_) continue;
        Node next=cur; next.mode=m;next.control=0;next.has_control=false;
        next.switched=true; next.action={m,0,0};
        push(next,cur.parent<0?0.0:w_switch_);
      }
    }
    if (std::isfinite(best_terminal_cost)) {
      solution=std::move(best_actions);
      solution_cost=best_terminal_cost;
      return true;
    }
    return false;
  }
  void publish_status(const std::string& s) {
    std_msgs::String m; m.data=s; pub_status_.publish(m); ROS_INFO_STREAM("[v3 planner] "<<s);
  }
  void publish_endpoints() {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker erase;
    erase.header.frame_id="map";erase.header.stamp=ros::Time::now();
    erase.action=visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(erase);
    auto add=[&](const Pose& p,int id,float r,float g,float b) {
      visualization_msgs::Marker m;
      m.header.frame_id="map";m.header.stamp=ros::Time::now();
      m.ns="v3_endpoints";m.id=id;m.type=visualization_msgs::Marker::ARROW;
      m.action=visualization_msgs::Marker::ADD;
      m.pose=stamp_pose(p,m.header.stamp).pose;
      m.pose.position.z=.15;
      m.scale.x=.50;m.scale.y=.12;m.scale.z=.12;
      m.color.r=r;m.color.g=g;m.color.b=b;m.color.a=1;
      markers.markers.push_back(m);
    };
    if(has_start_) add(start_,0,.15f,1.f,.15f);
    if(has_goal_) add(goal_,1,1.f,.15f,.15f);
    pub_markers_.publish(markers);
  }
  void on_map(const nav_msgs::OccupancyGrid::ConstPtr& m) {
    if (std::abs(tf::getYaw(m->info.origin.orientation))>1e-7) {
      ROS_ERROR("[v3 planner] rotated map origins are not supported yet"); return;
    }
    map_.reset(*m); has_map_=map_.valid();
    safe_clearance_.clear();
    occupied_runs_by_row_.clear();
    occupied_slabs_.clear();
    if(has_map_) {
      const int w=map_.width(),h=map_.height(),n=w*h;
      const uint16_t inf=std::numeric_limits<uint16_t>::max();
      safe_clearance_.assign(n,inf);
      occupied_runs_by_row_.resize(h);
      for(int y=0;y<h;++y) for(int x=0;x<w;++x) {
        const int i=y*w+x;
        if(map_.is_occupied(map_.origin_x()+(x+.5)*map_.resolution(),
                            map_.origin_y()+(y+.5)*map_.resolution())) {
          safe_clearance_[i]=0;
          auto& runs=occupied_runs_by_row_[y];
          if(runs.empty() || runs.back().x1!=x-1) runs.push_back({x,x});
          else runs.back().x1=x;
        }
      }
      occupied_slabs_=merge_occupied_rows(occupied_runs_by_row_);
      // Two-pass 8-neighbour (Chebyshev) distance transform: faster and
      // tighter than scaling GridMap's 4-neighbour Manhattan distance.
      for(int y=0;y<h;++y) for(int x=0;x<w;++x) {
        const int i=y*w+x;
        if(safe_clearance_[i]==0) continue;
        uint16_t d=safe_clearance_[i];
        auto relax=[&](int nx,int ny) {
          if(nx>=0&&ny>=0&&nx<w&&ny<h) {
            const uint16_t v=safe_clearance_[ny*w+nx];
            if(v<inf-1) d=std::min<uint16_t>(d,static_cast<uint16_t>(v+1));
          }
        };
        relax(x-1,y);relax(x,y-1);relax(x-1,y-1);relax(x+1,y-1);
        safe_clearance_[i]=d;
      }
      for(int y=h-1;y>=0;--y) for(int x=w-1;x>=0;--x) {
        const int i=y*w+x;
        if(safe_clearance_[i]==0) continue;
        uint16_t d=safe_clearance_[i];
        auto relax=[&](int nx,int ny) {
          if(nx>=0&&ny>=0&&nx<w&&ny<h) {
            const uint16_t v=safe_clearance_[ny*w+nx];
            if(v<inf-1) d=std::min<uint16_t>(d,static_cast<uint16_t>(v+1));
          }
        };
        relax(x+1,y);relax(x,y+1);relax(x+1,y+1);relax(x-1,y+1);
        safe_clearance_[i]=d;
      }
    }
  }
  void on_start(const geometry_msgs::PoseStamped::ConstPtr& m) {
    if (m->header.frame_id!="map") {ROS_WARN("[v3 planner] start must be in map frame");return;}
    start_={m->pose.position.x,m->pose.position.y,yaw_of(m->pose.orientation)}; has_start_=true;
    publish_endpoints();
  }
  void on_init(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& m) {
    if (m->header.frame_id!="map") {ROS_WARN("[v3 planner] initialpose must be in map frame");return;}
    start_={m->pose.pose.position.x,m->pose.pose.position.y,yaw_of(m->pose.pose.orientation)};has_start_=true;
    publish_endpoints();
  }
  void on_goal(const geometry_msgs::PoseStamped::ConstPtr& m) {
    ++plan_id_;
    nav_msgs::Path blank; blank.header.frame_id="map"; blank.header.stamp=ros::Time::now(); pub_path_.publish(blank);
    fwis_v3_msgs::PlanSeed empty; empty.header=blank.header;empty.plan_id=plan_id_;pub_seed_.publish(empty);
    fwis_v3_msgs::PlanSeedCandidates empty_candidates;
    empty_candidates.header=blank.header;empty_candidates.plan_id=plan_id_;
    pub_seed_candidates_.publish(empty_candidates);
    if (m->header.frame_id!="map") {has_goal_=false;publish_endpoints();publish_status("invalid_goal_frame");return;}
    goal_={m->pose.position.x,m->pose.position.y,yaw_of(m->pose.orientation)};
    has_goal_=true;publish_endpoints();
    if (!has_map_||!has_start_) {publish_status("missing_map_or_start");return;}
    goal_field_=map_.goal_distance_field(goal_.x,goal_.y);
    int iterations=0; std::vector<Action> actions;
    auto t0=std::chrono::steady_clock::now();
    int guided_iterations=0,fallback_iterations=0,ackermann_probe_iterations=0;
    double guided_cost=std::numeric_limits<double>::infinity();
    double fallback_cost=std::numeric_limits<double>::infinity();
    std::vector<Action> guided_actions,fallback_actions;
    auto route2d_t0=std::chrono::steady_clock::now();
    std::vector<Action> route2d_actions;double route2d_cost=std::numeric_limits<double>::infinity();
    std::string route2d_reason="";
    const bool route2d_ok=oblique_route(route2d_actions,route2d_cost,route2d_reason);
    const bool route2d_has_oblique=std::any_of(
        route2d_actions.begin(),route2d_actions.end(),
        [](const Action& action) {return action.mode==O &&
                                         std::abs(action.amount)>1e-8;});
    const double route2d_seconds=std::chrono::duration<double>(
      std::chrono::steady_clock::now()-route2d_t0).count();
    std::vector<Action> ackermann_probe_actions;
    double ackermann_probe_cost=std::numeric_limits<double>::infinity();
    bool ackermann_probe_ok=false;
    double ackermann_probe_seconds=0.0;
    // A mixed incumbent can hide a lower-cost A/Spin alternative when the
    // sparse waypoint graph missed its heading sequence. Search both mode
    // families and compare feasible candidates with the shared score.
    if((!route2d_ok || route2d_has_oblique) && ackermann_probe_iter_>0) {
      auto probe_t0=std::chrono::steady_clock::now();
      const int probe_budget=std::min(max_iter_,ackermann_probe_iter_);
      ackermann_probe_ok=search(
          ackermann_probe_actions,ackermann_probe_iterations,true,
          ackermann_probe_cost,nullptr,
          std::numeric_limits<double>::infinity(),probe_budget,1.0,false);
      ackermann_probe_seconds=std::chrono::duration<double>(
          std::chrono::steady_clock::now()-probe_t0).count();
    }
    const int guided_budget=route2d_ok?max_iter_:
        std::min(max_iter_,std::max(0,
            guided_iter_without_incumbent_-ackermann_probe_iterations));
    const std::vector<Action>* guided_incumbent=route2d_ok?&route2d_actions:nullptr;
    double guided_incumbent_cost=route2d_cost;
    // The A/Spin probe is another candidate, not a hard mode-family gate.
    // O stays available so its lower-curvature local segments can compete.
    const bool guided_allow_oblique=true;
    // First preserve the forward-only feasible envelope. A route-graph
    // incumbent that already satisfies the shared Spin/reversal contract is
    // also allowed to compete, including its legal reverse segment.
    const bool route2d_reverse_allowed=route2d_ok && allow_reverse_ &&
        has_reverse_motion(route2d_actions) &&
        (allow_shunting_ || shunting_violation(route2d_actions).empty());
    const bool guided_allow_reverse=route2d_reverse_allowed;
    if(ackermann_probe_ok &&
       (!guided_incumbent || ackermann_probe_cost<guided_incumbent_cost)) {
      guided_incumbent=&ackermann_probe_actions;
      guided_incumbent_cost=ackermann_probe_cost;
    }
    auto guided_t0=std::chrono::steady_clock::now();
    int guided_refine_iterations=0;
    bool guided_ok=search(guided_actions,guided_refine_iterations,true,guided_cost,
                          guided_incumbent,guided_incumbent_cost,
                          guided_budget,1.0,guided_allow_oblique,
                          guided_allow_reverse);
    guided_iterations=ackermann_probe_iterations+guided_refine_iterations;
    const double guided_seconds=std::chrono::duration<double>(
      std::chrono::steady_clock::now()-guided_t0).count();
    // Guided search is an anytime planner: once its incumbent is feasible and
    // sweep-checked, do not spend a second full budget on unguided search.
    // The fallback is for recovering feasibility, not duplicating optimization.
    bool fallback_tried=!guided_ok;
    bool fallback_ok=false;
    double fallback_seconds=0.0;
    const int remaining_budget=std::max(0,max_iter_-guided_iterations);
    if(fallback_tried && remaining_budget>0) {
      auto fallback_t0=std::chrono::steady_clock::now();
      fallback_ok=search(fallback_actions,fallback_iterations,false,fallback_cost,
                         nullptr,std::numeric_limits<double>::infinity(),
                         remaining_budget,route2d_ok?1.0:0.5,true);
      fallback_seconds=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-fallback_t0).count();
    }
    bool use_fallback=fallback_ok&&(!guided_ok||fallback_cost<guided_cost);
    bool ok=guided_ok||fallback_ok;
    bool reverse_tried=false,reverse_ok=false,reverse_selected=false;
    bool reverse_search_competed=guided_allow_reverse;
    int reverse_iterations=0;
    double reverse_seconds=0.0,reverse_cost=std::numeric_limits<double>::infinity();
    std::vector<Action> reverse_actions;
    std::vector<Action> auxiliary_policy_proposal;
    const int reverse_competition_budget=std::max(
        reverse_fallback_max_iter_,reverse_competition_max_iter_);
    // Compare a bounded bidirectional candidate even when the forward search
    // found a route. A legal forward incumbent remains available throughout;
    // the reverse candidate replaces it only when the shared action cost is
    // lower. This restores direction competition without making the legacy
    // forward feasibility envelope depend on reverse-state search cost.
    if(allow_reverse_ && ok && !guided_allow_reverse) {
      reverse_tried=true;reverse_search_competed=true;
      const auto reverse_begin=std::chrono::steady_clock::now();
      reverse_ok=search(reverse_actions,reverse_iterations,true,reverse_cost,
                        nullptr,std::numeric_limits<double>::infinity(),
                        reverse_competition_budget,1.0,true,true,
                        allow_shunting_?nullptr:&auxiliary_policy_proposal);
      reverse_seconds=std::chrono::duration<double>(
          std::chrono::steady_clock::now()-reverse_begin).count();
      const double selected_cost=use_fallback?fallback_cost:guided_cost;
      reverse_selected=reverse_ok && reverse_cost+1e-9<selected_cost;
    } else if(!ok && allow_reverse_) {
      reverse_tried=true;
      reverse_search_competed=true;
      const auto reverse_begin=std::chrono::steady_clock::now();
      reverse_ok=search(reverse_actions,reverse_iterations,true,reverse_cost,
                        nullptr,std::numeric_limits<double>::infinity(),
                        reverse_competition_budget,1.0,true,true,
                        allow_shunting_?nullptr:&auxiliary_policy_proposal);
      reverse_seconds=std::chrono::duration<double>(
          std::chrono::steady_clock::now()-reverse_begin).count();
      ok=reverse_ok;
      reverse_selected=reverse_ok;
    }
    auto mode_profile=[](const std::vector<Action>& seq) {
      std::array<int,3> n{{0,0,0}};int switches=0,prior=-1;
      for(const auto& a:seq) {++n[a.mode];if(prior>=0&&prior!=a.mode)++switches;prior=a.mode;}
      return "A"+std::to_string(n[A])+"/O"+std::to_string(n[O])+"/S"+
             std::to_string(n[S])+"/sw"+std::to_string(switches);
    };
    const std::string guided_profile=guided_ok?mode_profile(guided_actions):"none";
    const std::string fallback_profile=fallback_ok?mode_profile(fallback_actions):"none";
    if(use_fallback) actions=std::move(fallback_actions);
    else if(guided_ok) actions=std::move(guided_actions);
    if(reverse_selected) actions=std::move(reverse_actions);
    iterations=guided_iterations+fallback_iterations+reverse_iterations;
    double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
    if (!ok) {
      const std::string reason=iterations>=max_iter_ ? "search_budget_exhausted" :
                                                       "search_no_path_in_discretized_lattice";
      publish_status(reason+" id="+std::to_string(plan_id_)+
                     " direction_policy="+(reverse_search_competed?
                       "bidirectional_shared_cost":"forward_only")+
                     " allow_shunting="+(allow_shunting_?"1":"0")+
                     " reverse_search_competed="+
                       (reverse_search_competed?"1":"0")+
                     " reverse_fallback_tried="+(reverse_tried?"1":"0")+
                     " reverse_fallback_seconds="+std::to_string(reverse_seconds)+
                     " iterations="+std::to_string(iterations)+
                     " guided_iterations="+std::to_string(guided_iterations)+
                     " ackermann_probe_iterations="+
                         std::to_string(ackermann_probe_iterations)+
                     " fallback_iterations="+std::to_string(fallback_iterations)+
                     " route2d="+(route2d_ok?"feasible":"unavailable")+
                     " route2d_reason="+route2d_reason+
                     " mode_policy=shared_cost_all_modes"+
                     " route2d_seconds="+std::to_string(route2d_seconds)+
                     " ackermann_probe="+(ackermann_probe_ok?"feasible":"unavailable")+
                     " ackermann_probe_seconds="+
                         std::to_string(ackermann_probe_seconds)+
                     " guided_seconds="+std::to_string(guided_seconds)+
                     " fallback_seconds="+std::to_string(fallback_seconds)+
                     " fallback_lattice_scale="+std::to_string(route2d_ok?1.0:0.5)+
                     " seconds="+std::to_string(seconds)+
                     " start=("+std::to_string(start_.x)+","+std::to_string(start_.y)+","+
                     std::to_string(start_.th)+") goal=("+std::to_string(goal_.x)+","+
                     std::to_string(goal_.y)+","+std::to_string(goal_.th)+")");
      return;
    }
    if(!allow_shunting_) {
      const std::string violation=shunting_violation(actions);
      if(!violation.empty()) {
        publish_status("action_policy_rejected id="+std::to_string(plan_id_)+
                       " policy=no_shunting reason="+violation);
        return;
      }
    }
    fwis_v3_msgs::PlanSeed seed;seed.header=blank.header;seed.plan_id=plan_id_;
    seed.start_center.x=start_.x;seed.start_center.y=start_.y;seed.start_center.theta=start_.th;
    seed.goal_center.x=goal_.x;seed.goal_center.y=goal_.y;seed.goal_center.theta=goal_.th;
    nav_msgs::Path path;path.header=seed.header;
    Pose p=start_;path.poses.push_back(stamp_pose(p,path.header.stamp));
    for (const auto& a:actions) {
      fwis_v3_msgs::ModePrimitive msg;msg.mode=a.mode;msg.amount=a.amount;msg.control=a.control;
      seed.primitives.push_back(msg);
      int n=std::max(1,static_cast<int>(std::ceil(std::abs(a.amount)/(a.mode==S?PI/90:0.04))));
      for(int j=1;j<=n;++j) {Action part=a;part.amount=a.amount*j/n;
        path.poses.push_back(stamp_pose(advance(p,part),path.header.stamp));}
      p=advance(p,a);
    }
    std::array<int,3> mode_actions{{0,0,0}};
    std::array<double,3> mode_amount{{0.0,0.0,0.0}};
    int mode_switches=0,previous_mode=-1;
    int steer_changes=0;
    double curvature_tv=0.0,oblique_tv=0.0,reverse_distance=0.0;
    double previous_curvature=0.0,previous_oblique=0.0;
    bool previous_ackermann=false,previous_oblique_move=false;
    for(const auto& a:actions) {
      ++mode_actions[a.mode];mode_amount[a.mode]+=std::abs(a.amount);
      if(previous_mode>=0&&previous_mode!=a.mode)++mode_switches;
      if(a.mode==A) {
        if(a.amount<0) reverse_distance+=std::abs(a.amount);
        if(previous_ackermann) {
          const double delta=std::abs(a.control-previous_curvature);
          curvature_tv+=delta;
          if(delta>0.05)++steer_changes;
        }
        previous_curvature=a.control;previous_ackermann=true;
        previous_oblique_move=false;
      } else if(a.mode==O) {
        if(previous_oblique_move)oblique_tv+=std::abs(wrap(a.control-previous_oblique));
        previous_oblique=a.control;previous_oblique_move=true;
        previous_ackermann=false;
      } else {
        previous_ackermann=false;previous_oblique_move=false;
      }
      previous_mode=a.mode;
    }
    const CostBreakdown cost_parts=action_sequence_cost_parts(actions);
    fwis_v3_msgs::PlanSeedCandidates seed_candidates;
    seed_candidates.header=seed.header;seed_candidates.plan_id=seed.plan_id;
    if(!allow_shunting_ && !auxiliary_policy_proposal.empty() &&
       !shunting_violation(auxiliary_policy_proposal).empty()) {
      seed_candidates.proposal_source="untrusted_shunting_search";
      for(const auto& action:auxiliary_policy_proposal) {
        fwis_v3_msgs::ModePrimitive primitive;
        primitive.mode=action.mode;primitive.amount=action.amount;
        primitive.control=action.control;
        seed_candidates.proposal_primitives.push_back(primitive);
      }
    }
    pub_path_.publish(path);pub_seed_candidates_.publish(seed_candidates);
    pub_seed_.publish(seed);
    publish_status("search_success id="+std::to_string(plan_id_)+
      " direction_policy="+(reverse_search_competed?
                       "bidirectional_shared_cost":"forward_only")+
      " allow_shunting="+(allow_shunting_?"1":"0")+
      " reverse_search_competed="+
        (reverse_search_competed?"1":"0")+
      " reverse_fallback_tried="+(reverse_tried?"1":"0")+
      " reverse_fallback_used="+(reverse_selected?"1":"0")+
      " selected_has_reverse="+(has_reverse_motion(actions)?"1":"0")+
      " reverse_fallback_iterations="+std::to_string(reverse_iterations)+
      " reverse_fallback_seconds="+std::to_string(reverse_seconds)+" actions="+
      std::to_string(actions.size())+" iterations="+std::to_string(iterations)+
      " seconds="+std::to_string(seconds)+
      " modes=A"+std::to_string(mode_actions[A])+"/"+std::to_string(mode_amount[A])+"m,O"+
      std::to_string(mode_actions[O])+"/"+std::to_string(mode_amount[O])+"m,S"+
      std::to_string(mode_actions[S])+"/"+std::to_string(mode_amount[S])+"rad,switches="+
      std::to_string(mode_switches)+" steer_changes="+std::to_string(steer_changes)+
      " curvature_tv="+std::to_string(curvature_tv)+" oblique_tv="+
      std::to_string(oblique_tv)+" reverse="+std::to_string(reverse_distance)+"m"+
      " budget_hit="+(iterations>=max_iter_?"1":"0")+
      " incumbent_budget_hit="+((guided_incumbent&&
          guided_refine_iterations>=incumbent_iter_)?"1":"0")+
      " selected="+(reverse_selected?"reverse_competitor":use_fallback?"baseline":"guided")+
      " auxiliary_shunting_proposal="+
        (seed_candidates.proposal_primitives.empty()?"0":"1")+
      " guided_cost="+std::to_string(guided_cost)+
      " cost_parts={distance:"+std::to_string(cost_parts.distance)+
      ",spin:"+std::to_string(cost_parts.spin)+
      ",mode_switch:"+std::to_string(cost_parts.mode_switch)+
      ",low_curvature_switch:"+std::to_string(cost_parts.low_curvature_switch)+
      ",reverse:"+std::to_string(cost_parts.reverse)+
      ",reverse_early:"+std::to_string(cost_parts.reverse_early)+
      ",reverse_start:"+std::to_string(cost_parts.reverse_start)+
      ",reverse_yaw_early:"+std::to_string(cost_parts.reverse_yaw_early)+
      ",direction:"+std::to_string(cost_parts.direction)+
      ",steer_tv:"+std::to_string(cost_parts.steer_tv)+
      ",curvature:"+std::to_string(cost_parts.curvature)+
      ",oblique_tv:"+std::to_string(cost_parts.oblique_tv)+
      ",oblique_event:"+std::to_string(cost_parts.oblique_event)+
      ",oblique_use:"+std::to_string(cost_parts.oblique_use)+
      ",oblique_cruise:"+std::to_string(cost_parts.oblique_cruise)+
      ",sum:"+std::to_string(cost_parts.total())+"}"+
      " baseline_cost="+std::to_string(fallback_cost)+
      " route2d="+(route2d_ok?"feasible":"unavailable")+
      " route2d_cost="+std::to_string(route2d_cost)+
      " route2d_seconds="+std::to_string(route2d_seconds)+
      " ackermann_probe="+(ackermann_probe_ok?"feasible":"unavailable")+
      " ackermann_probe_cost="+std::to_string(ackermann_probe_cost)+
      " ackermann_probe_iterations="+
          std::to_string(ackermann_probe_iterations)+
      " ackermann_probe_seconds="+std::to_string(ackermann_probe_seconds)+
      " route2d_reason="+route2d_reason+
      " mode_policy=shared_cost_all_modes"+
      " fallback_lattice_scale="+std::to_string(route2d_ok?1.0:0.5)+
      " guided_modes="+guided_profile+" baseline_modes="+fallback_profile+
      " start=("+std::to_string(start_.x)+","+std::to_string(start_.y)+","+
      std::to_string(start_.th)+") goal=("+std::to_string(goal_.x)+","+
      std::to_string(goal_.y)+","+std::to_string(goal_.th)+")");
  }
  ros::NodeHandle nh_,pnh_;
  ros::Subscriber sub_map_,sub_start_,sub_init_,sub_goal_;
  ros::Publisher pub_path_,pub_seed_,pub_seed_candidates_,pub_status_,pub_markers_;
  fwis::GridMap map_;Pose start_,goal_;
  std::vector<uint16_t> safe_clearance_;
  std::vector<std::vector<CellRun>> occupied_runs_by_row_;
  std::vector<CellSlab> occupied_slabs_;
  std::vector<float> goal_field_;
  bool has_map_=false,has_start_=false,has_goal_=false,enable_route_heuristic_=true;
  bool allow_reverse_=false,allow_shunting_=false;
  int reverse_fallback_max_iter_=30000;
  double reverse_fallback_budget_s_=3.0;
  uint32_t plan_id_=0;
  double step_,xy_res_,yaw_res_,xy_tol_,yaw_tol_,length_,width_,radius_,margin_;
  double terminal_radius_,mode_radius_,sweep_guard_,route_sweep_step_,a_graph_sweep_step_;
  double route_graph_dense_spacing_,route_graph_sparse_spacing_,route_graph_turn_window_;
  bool route_graph_continuous_adaptive_=false;
  double route_graph_turn_threshold_,route_graph_max_edge_length_,low_curvature_switch_penalty_;
  double switch_turn_threshold_;
  double w_spin_,w_switch_,w_reverse_,w_direction_;
  double w_reverse_early_=0,w_reverse_start_=0,w_reverse_yaw_early_=0;
  double w_steer_tv_,w_curvature_,w_oblique_tv_,w_oblique_event_score_;
  double w_oblique_use_,w_oblique_cruise_;
  int max_iter_,incumbent_iter_,guided_iter_without_incumbent_,
      ackermann_probe_iter_,route_graph_max_expansions_,reverse_competition_max_iter_;
};
}  // namespace

int main(int argc,char** argv) {
  ros::init(argc,argv,"fwis_v3_planner");
  PlannerNode node; ros::spin(); return 0;
}
