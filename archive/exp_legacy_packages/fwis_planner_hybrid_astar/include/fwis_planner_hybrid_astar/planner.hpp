#pragma once
#include <cstdint>
#include <memory>
#include <vector>

#include "fwis_common/grid_map.hpp"
#include "fwis_common/kinematics.hpp"
#include "fwis_common/modes.hpp"
#include "fwis_common/vehicle.hpp"

namespace fwis {

struct PlannerConfig {
  // ---- Search resolution ----
  double xy_res        = 0.20;
  double yaw_res       = 0.1745;
  double step_len      = 0.20;

  // ---- Time-equivalent cost (方案C) ----
  // Base: w_time × t_step. Every mode compared in TIME.
  //   Ack/Lat/Dia: t_step = ds / v_cruise
  //   Spin:        t_step = |dtheta| / omega_spin
  //   Reverse:     t_step = ds / (v_cruise × k_rev)
  double w_time            = 2.0;
  double v_cruise          = 1.0;
  double k_rev             = 0.8;
  double omega_spin        = 0.35;  // rad/s, spin angular velocity

  // Spin distance modulation: f_spin = clamp(d_goal/d_spin, f_spin_min, f_spin_max)
  // Near goal → cheap (terminal heading fix). Far → expensive (deter cruise spin).
  double d_spin            = 2.0;
  double f_spin_min        = 0.5;   // 原来0.15, Spin太便宜导致大量巡航Spin
  double f_spin_max        = 2.5;

  // Exit-spin discount: C_switch multiplier when leaving Spin (already at low speed)
  double k_exit_spin       = 0.3;

  // Terminal yaw snap (post-search Spin if heading error exceeds threshold)
  double yaw_snap_threshold = 0.05;

  // Forward/reverse direction switch
  double w_dir_change      = 1.0;

  // Control effort
  double w_steer           = 0.8;
  double w_phi             = 1.5;

  // Control smoothness
  double w_steer_change_ack = 1.5;
  double w_phi_change_dia   = 1.5;
  double w_curv_change     = 1.5;

  // Spin yaw step per primitive
  double spin_yaw_step     = 0.0873;

  // Velocity-coupled mode-switch cost (Wang 2025 Eq. 11)
  double v_ref             = 1.0;
  double t_switch_servo    = 0.5;
  double a_max_decel       = 2.0;
  double d_slow            = 2.0;
  double d_slow_obs        = 0.8;
  double v_floor_frac      = 0.4;

  // Mode-entry sticky cost
  double w_enter_lateral   = 3.0;
  double w_enter_diagonal  = 0.5;
  double w_enter_spin      = 1.0;   // 原来0.3, 进入Spin成本太低
  double w_enter_ackermann = 0.0;

  // Heuristic
  double h_inflation       = 1.3;

  // Search controls
  int    max_iter        = 400000;
  double goal_xy_tol     = 0.15;
  double goal_yaw_tol    = 0.30;
  // Parallel/diagonal translation preserves body yaw.  It is therefore a
  // terminal-alignment mode, not a generic holonomic shortcut (Wang 2025).
  double parallel_goal_yaw_tol = 0.35;
  double rs_shot_radius  = 8.0;
  int    rs_shot_check_every = 5;

  int    n_steer         = 5;
  int    n_phi           = 7;
  bool   allow_reverse   = true;

  bool   enable_lateral  = true;
  bool   enable_diagonal = true;
  bool   enable_spin     = true;

  // ---- Topology / macro-action front end ----
  // The new front end searches a clearance-aware 2-D channel graph and
  // delegates curvature and mode allocation to the corridor OCP.  These are
  // map-scale quantities, deliberately independent of parking templates.
  double topology_clearance = 0.22;
  double topology_clearance_weight = 0.35;
  double topology_sample_step = 0.10;
  int    topology_candidate_count = 3;
  double topology_turn_weight = 0.25;
  bool   use_macro_frontend = true;
};

struct PlanState {
  double x, y, theta;
  Mode   mode;
  bool   reverse = false;
  double g = 0.0, h = 0.0;
  double phi = 0.0;
  double curv = 0.0;
  double dist_since_switch = 1e9;
  // A mode transition is a zero-displacement state.  It represents the
  // vehicle stopping and reconfiguring its steering, not motion in the new
  // mode.  The next expansion performs intra-modal motion.
  bool   is_mode_switch = false;
  int    parent = -1;
};

struct PlanResult {
  bool                  success = false;
  std::vector<PlanState> path;
  int                   iterations = 0;
  double                cost = 0.0;
};

class HybridAStarPlanner {
 public:
  HybridAStarPlanner(const PlannerConfig& cfg,
                     const VehicleParams& vp)
      : cfg_(cfg), vp_(vp) {
    delta_max_ = std::atan(vp_.wheelbase / vp_.r_min_ack);
  }

  PlanResult plan(const GridMap& map,
                  const PlanState& start,
                  const PlanState& goal) const;

 private:
  uint64_t hash_state(const PlanState& s) const;
  double   heuristic(const GridMap& map, const PlanState& s, const PlanState& g,
                     const std::vector<float>& holonomic_field) const;
  std::vector<PlanState> expand(const PlanState& s, const PlanState& goal) const;
  bool segment_collision_free(const GridMap& map,
                              const PlanState& s_from,
                              Mode m, double ds, double cmd) const;
  bool reached(const PlanState& s, const PlanState& g) const;
  bool try_rs_shot(const GridMap& map,
                   const PlanState& s,
                   const PlanState& goal,
                   std::vector<PlanState>& out,
                   double& cost_delta) const;
  bool try_translation_shot(const GridMap& map,
                            const PlanState& s,
                            const PlanState& goal,
                            std::vector<PlanState>& out,
                            double& cost_delta) const;
  bool try_analytic_shot(const GridMap& map,
                         const PlanState& s,
                         const PlanState& goal,
                         std::vector<PlanState>& out,
                         double& cost_delta) const;

  PlannerConfig cfg_;
  VehicleParams vp_;
  double        delta_max_;
};

// Coarse topology search for the new planner pipeline.  It returns a sparse,
// collision-checked channel centreline; no mode is chosen at this stage.  The
// downstream corridor OCP owns the continuous curvature and multi-mode twist
// allocation, preventing primitive-level mode flicker.
class MacroTopologyPlanner {
 public:
  MacroTopologyPlanner(const PlannerConfig& cfg, const VehicleParams& vp)
      : cfg_(cfg), vp_(vp) {}

  PlanResult plan(const GridMap& map, const PlanState& start,
                  const PlanState& goal) const;

 private:
  bool center_segment_free(const GridMap& map, double x0, double y0,
                           double x1, double y1) const;
  bool body_segment_free(const GridMap& map, double x0, double y0,
                         double x1, double y1) const;
  PlannerConfig cfg_;
  VehicleParams vp_;
};

std::vector<PlanState> densify_path_per_mode(const std::vector<PlanState>& path,
                                              double target_step);

}  // namespace fwis
