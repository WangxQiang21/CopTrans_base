#include "fwis_planner_hybrid_astar/planner.hpp"
#include "fwis_common/reeds_shepp.hpp"
#include "fwis_common/velocity_heuristic.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace fwis {

namespace {
struct PQItem { double f; int idx; };
struct PQCmp { bool operator()(const PQItem& a, const PQItem& b) const { return a.f > b.f; } };
}  // namespace

uint64_t HybridAStarPlanner::hash_state(const PlanState& s) const {
  int ix   = static_cast<int>(std::floor(s.x / cfg_.xy_res));
  int iy   = static_cast<int>(std::floor(s.y / cfg_.xy_res));
  int iyaw = static_cast<int>(std::floor(wrap_pi(s.theta) / cfg_.yaw_res)) & 0xFFF;
  // A state with a different gear or wheel command has a different set of
  // feasible/low-cost continuations.  Collapsing them into one spatial label
  // is what made the search favour short A-L-A-L fragments.
  int iphi = static_cast<int>(std::llround(
      (wrap_pi(s.phi) + M_PI) * 63.0 / (2.0 * M_PI)));
  iphi = std::max(0, std::min(63, iphi));
  uint64_t key = 0;
  key |= (static_cast<uint64_t>(ix) & 0xFFFFFFFFULL);
  key |= ((static_cast<uint64_t>(iy) & 0xFFFFFFFFULL) << 32);
  uint64_t aux = (static_cast<uint64_t>(iyaw) << 10)
               | (static_cast<uint64_t>(iphi) << 4)
               | (static_cast<uint64_t>(s.reverse) << 3)
               | static_cast<uint64_t>(s.mode);
  return key ^ (aux * 0x9E3779B97F4A7C15ULL);
}

// ---- Admissible time lower bound ----
//
// The former heuristic mixed weighted, mode-specific estimates with an
// early-stop test for an RS candidate.  Those estimates are not guaranteed
// lower bounds once steering/mode-entry penalties are included, so a large
// terminal arc could be accepted before a cheaper topological route was
// explored.  Keep the search bound sound first; a 2-D obstacle-distance map
// and validated per-mode RS lower bounds can be added on top later.
double HybridAStarPlanner::heuristic(
    const GridMap& map, const PlanState& s, const PlanState& g,
    const std::vector<float>& holonomic_field) const {
  double dx = g.x - s.x;
  double dy = g.y - s.y;
  double euc = std::hypot(dx, dy);
  const double h_euc = cfg_.w_time * euc / std::max(cfg_.v_cruise, 1e-6);
  // The 2-D field accounts for obstacle detours but ignores vehicle
  // orientation and footprint, so it remains an admissible lower bound.
  const double h_holo_m = holonomic_field.empty()
      ? std::numeric_limits<double>::infinity()
      : map.sample_distance_field(holonomic_field, s.x, s.y);
  const double h_xy = std::isfinite(h_holo_m)
      ? std::max(h_euc, cfg_.w_time * h_holo_m /
                           std::max(cfg_.v_cruise, 1e-6))
      : h_euc;

  // Lateral and diagonal primitives preserve theta.  Any remaining heading
  // error must therefore be supplied either by bounded-curvature Ackermann
  // motion (|dtheta| <= ds / r_min) or by Spin.  The cheaper of those two
  // physical limits is a lower bound and materially reduces orientation-only
  // branches without using a non-admissible weighted heuristic.
  const double yaw_err = std::abs(wrap_pi(g.theta - s.theta));
  const double yaw_time_per_rad = std::min(
      vp_.r_min_ack / std::max(cfg_.v_cruise, 1e-6),
      cfg_.enable_spin ? cfg_.f_spin_min / std::max(cfg_.omega_spin, 1e-6)
                       : std::numeric_limits<double>::infinity());
  const double h_yaw = cfg_.w_time * yaw_err * yaw_time_per_rad;
  return std::max(h_xy, h_yaw);
}

// ---- Time-equivalent expand (方案C) ----
std::vector<PlanState> HybridAStarPlanner::expand(const PlanState& s,
                                                  const PlanState& goal) const {
  std::vector<PlanState> out;
  out.reserve(static_cast<size_t>(cfg_.n_steer + cfg_.n_phi + 2) * 2);

  const double ds = cfg_.step_len;
  const std::array<double, 2> signs = { +1.0, -1.0 };

  const double c_switch_base = c_switch_at(
      s.x, s.y, goal.x, goal.y,
      cfg_.v_ref, cfg_.d_slow,
      cfg_.t_switch_servo, cfg_.a_max_decel,
      cfg_.v_floor_frac);

  const double d_goal = std::hypot(goal.x - s.x, goal.y - s.y);
  const bool parallel_entry_allowed =
      std::abs(wrap_pi(s.theta - goal.theta)) <= cfg_.parallel_goal_yaw_tol;
  const double f_spin = std::max(cfg_.f_spin_min,
                                 std::min(cfg_.f_spin_max, d_goal / cfg_.d_spin));

  auto switch_cost = [&](Mode src, Mode /*dst*/) {
    double base = c_switch_base;
    if (src == Mode::Spin) base *= cfg_.k_exit_spin;
    return base;
  };

  auto length_term = [&](bool reverse) {
    double t = cfg_.w_time * (ds / cfg_.v_cruise);
    if (reverse) t *= (1.0 / cfg_.k_rev);
    return t;
  };

  auto dir_change_term = [&](bool s_rev, bool n_rev) {
    return (s_rev != n_rev) ? cfg_.w_dir_change : 0.0;
  };

  // Wang et al. model the 4-D state as two distinct operations:
  // intra-modal motion, and an inter-modal transition at the *same pose*.
  // Do not allow a primitive to both switch mode and move; that artificial
  // edge was the source of the A-L-A-L/P flicker in this planner.

  // --- Ackermann intra-modal primitives ---
  if (s.mode == Mode::Ackermann)
  for (int i = 0; i < cfg_.n_steer; ++i) {
    double t = (cfg_.n_steer == 1) ? 0.0
              : -1.0 + 2.0 * static_cast<double>(i) / (cfg_.n_steer - 1);
    double delta = t * delta_max_;
    double kappa = std::tan(delta) / vp_.wheelbase;
    for (double sgn : signs) {
      if (sgn < 0 && !cfg_.allow_reverse) continue;
      double dseg = ds * sgn;
      Pose2D ns = step({s.x, s.y, s.theta}, Mode::Ackermann, dseg, delta, vp_);
      PlanState n;
      n.x = ns.x; n.y = ns.y; n.theta = wrap_pi(ns.theta);
      n.mode = Mode::Ackermann;
      n.reverse = (sgn < 0);
      n.phi = delta;
      n.curv = kappa;

      double cost = length_term(n.reverse);
      cost += cfg_.w_steer * std::abs(delta) * ds;
      cost += dir_change_term(s.reverse, n.reverse);
      cost += cfg_.w_steer_change_ack * std::abs(delta - s.phi);
      cost += cfg_.w_curv_change      * std::abs(kappa - s.curv);
      n.dist_since_switch = s.dist_since_switch + ds;
      n.g = s.g + cost;
      out.push_back(n);
    }
  }

  // --- Lateral ---
  if (cfg_.enable_lateral && s.mode == Mode::Lateral)
  for (double sgn : signs) {
    if (sgn < 0 && !cfg_.allow_reverse) continue;
    double dseg = ds * sgn;
    Pose2D ns = step({s.x, s.y, s.theta}, Mode::Lateral, dseg, 0.0, vp_);
    PlanState n;
    n.x = ns.x; n.y = ns.y; n.theta = wrap_pi(ns.theta);
    n.mode = Mode::Lateral;
    n.reverse = (sgn < 0);
    n.phi = 0.0;
    n.curv = 0.0;

    double cost = length_term(n.reverse);
    cost += dir_change_term(s.reverse, n.reverse);
    n.dist_since_switch = s.dist_since_switch + ds;
    n.g = s.g + cost;
    out.push_back(n);
  }

  // --- Spin ---
  if (cfg_.enable_spin && s.mode == Mode::Spin)
  for (double yaw_sgn : signs) {
    double dtheta = cfg_.spin_yaw_step * yaw_sgn;
    Pose2D ns = step({s.x, s.y, s.theta}, Mode::Spin, dtheta, 0.0, vp_);
    PlanState n;
    n.x = ns.x; n.y = ns.y; n.theta = wrap_pi(ns.theta);
    n.mode = Mode::Spin;
    n.reverse = false;
    n.phi = 0.0;
    n.curv = 0.0;

    double spin_time = std::abs(dtheta) / cfg_.omega_spin;
    double cost = cfg_.w_time * spin_time * f_spin;
    n.dist_since_switch = s.dist_since_switch
                          + std::abs(dtheta) * (cfg_.v_cruise / cfg_.omega_spin);
    n.g = s.g + cost;
    out.push_back(n);
  }

  // --- Diagonal ---
  if (cfg_.enable_diagonal && s.mode == Mode::Diagonal)
  for (int i = 0; i < cfg_.n_phi; ++i) {
    double t = (cfg_.n_phi == 1) ? 0.0
              : -1.0 + 2.0 * static_cast<double>(i) / (cfg_.n_phi - 1);
    double phi = t * vp_.phi_max;
    if (std::abs(phi) < 1e-6) continue;
    for (double sgn : signs) {
      if (sgn < 0 && !cfg_.allow_reverse) continue;
      double dseg = ds * sgn;
      Pose2D ns = step({s.x, s.y, s.theta}, Mode::Diagonal, dseg, phi, vp_);
      PlanState n;
      n.x = ns.x; n.y = ns.y; n.theta = wrap_pi(ns.theta);
      n.mode = Mode::Diagonal;
      n.reverse = (sgn < 0);
      n.phi = phi;
      n.curv = 0.0;

      double cost = length_term(n.reverse);
      cost += cfg_.w_phi * std::abs(phi) * ds;
      cost += dir_change_term(s.reverse, n.reverse);
      cost += cfg_.w_phi_change_dia * std::abs(phi - s.phi);
      n.dist_since_switch = s.dist_since_switch + ds;
      n.g = s.g + cost;
      out.push_back(n);
    }
  }

  // --- Inter-modal, zero-displacement switch primitives ---
  // Switching consumes the full-stop/reconfiguration/re-acceleration cost,
  // but does not smuggle a movement primitive into the new mode.
  // A reconfiguration must be followed by at least one physical primitive
  // before another switch.  Otherwise the search can use A->P->A at one
  // pose purely to reset the steering-change term.
  if (s.is_mode_switch) return out;
  auto add_switch = [&](Mode dst, bool enabled, double entry_cost) {
    if (!enabled || dst == s.mode) return;
    PlanState n = s;
    n.mode = dst;
    n.reverse = false;
    n.phi = 0.0;
    n.curv = 0.0;
    n.dist_since_switch = 0.0;
    n.is_mode_switch = true;
    n.g = s.g + switch_cost(s.mode, dst) + entry_cost;
    out.push_back(n);
  };
  add_switch(Mode::Ackermann, true, cfg_.w_enter_ackermann);
  add_switch(Mode::Lateral, cfg_.enable_lateral, cfg_.w_enter_lateral);
  // In parallel/diagonal motion theta is invariant.  Enter it only after
  // aligning the vehicle with the required terminal orientation; otherwise it
  // becomes an unrealistic all-purpose lateral shortcut.
  add_switch(Mode::Diagonal, cfg_.enable_diagonal && parallel_entry_allowed,
             cfg_.w_enter_diagonal);
  add_switch(Mode::Spin, cfg_.enable_spin, cfg_.w_enter_spin);

  return out;
}

bool HybridAStarPlanner::segment_collision_free(const GridMap& map,
                                                const PlanState& s_from,
                                                Mode m, double ds, double cmd) const {
  const double max_step = 0.05;
  const int N = std::max(3, static_cast<int>(std::ceil(std::abs(ds) / max_step)));
  for (int k = 1; k <= N; ++k) {
    double frac = static_cast<double>(k) / N;
    Pose2D p = step({s_from.x, s_from.y, s_from.theta}, m, ds * frac, cmd, vp_);
    if (map.obb_collides(p.x, p.y, p.theta,
                         vp_.length + 0.04,
                         vp_.width  + 0.04)) {
      return false;
    }
  }
  return true;
}

bool HybridAStarPlanner::reached(const PlanState& s, const PlanState& g) const {
  // A pure steering reconfiguration is not a drivable terminal state.
  if (s.is_mode_switch) return false;
  if (std::hypot(s.x - g.x, s.y - g.y) > cfg_.goal_xy_tol) return false;
  return std::abs(wrap_pi(s.theta - g.theta)) <= cfg_.goal_yaw_tol;
}

// ---- RS Ackermann terminal shot (时间等效) ----
bool HybridAStarPlanner::try_rs_shot(const GridMap& map,
                                     const PlanState& s,
                                     const PlanState& goal,
                                     std::vector<PlanState>& out,
                                     double& cost_delta) const {
  out.clear();
  cost_delta = 0.0;

  RSCurve curve = reeds_shepp(s.x, s.y, s.theta, goal.x, goal.y, goal.theta,
                               vp_.r_min_ack);
  if (!std::isfinite(curve.length)) return false;

  double ds = cfg_.step_len * 0.5;
  auto samples = sample_rs(s.x, s.y, s.theta, curve, vp_.r_min_ack, ds);
  bool clear = true;
  for (const auto& p : samples) {
    if (map.obb_collides(p[0], p[1], p[2],
                         vp_.length + 0.02, vp_.width + 0.02)) {
      clear = false; break;
    }
  }
  if (!clear) return false;

  std::vector<PlanState> cand;
  double accum = s.g;
  PlanState cur = s;
  size_t seg_idx = 0;
  double seg_progress = 0.0;
  for (size_t i = 1; i < samples.size(); ++i) {
    const auto& prev = samples[i-1];
    const auto& p = samples[i];
    double step = std::hypot(p[0]-prev[0], p[1]-prev[1]);
    if (step < 1e-9) continue;
    seg_progress += step;
    while (seg_idx < curve.segs.size() &&
           seg_progress > curve.segs[seg_idx].length + 1e-6) {
      seg_progress -= curve.segs[seg_idx].length;
      seg_idx++;
    }
    int dir = (seg_idx < curve.segs.size())
              ? curve.segs[seg_idx].direction : 1;
    char seg_type = (seg_idx < curve.segs.size())
                    ? curve.segs[seg_idx].type : 'S';
    double seg_kappa = 0.0;
    if (seg_type == 'L') seg_kappa = +1.0 / vp_.r_min_ack;
    else if (seg_type == 'R') seg_kappa = -1.0 / vp_.r_min_ack;
    PlanState n = cur;
    n.x = p[0]; n.y = p[1]; n.theta = p[2];
    n.mode = Mode::Ackermann;
    n.reverse = (dir < 0);
    n.curv = seg_kappa;
    n.phi = std::atan(seg_kappa * vp_.wheelbase);

    double cost_step = cfg_.w_time * (step / cfg_.v_cruise);
    if (n.reverse) cost_step *= (1.0 / cfg_.k_rev);
    if (n.reverse != cur.reverse) cost_step += cfg_.w_dir_change;
    cost_step += cfg_.w_steer * std::abs(n.phi) * step;
    cost_step += cfg_.w_curv_change * std::abs(seg_kappa - cur.curv);
    accum += cost_step;
    n.g = accum;
    cand.push_back(n);
    cur = n;
  }

  if (!cand.empty()) {
    cand.back().x = goal.x;
    cand.back().y = goal.y;
    cand.back().theta = goal.theta;
  }
  out = std::move(cand);
  cost_delta = accum - s.g;
  return true;
}

// ---- Terminal translation shot for yaw-preserving modes ----
// This is the missing counterpart of the Ackermann RS shot.  Once a search
// state is already aligned with the parking orientation, Lateral/Diagonal can
// finish on a collision-checked straight segment instead of forcing a large
// Ackermann arc merely because the target is nearby.
bool HybridAStarPlanner::try_translation_shot(
    const GridMap& map, const PlanState& s, const PlanState& goal,
    std::vector<PlanState>& out, double& cost_delta) const {
  out.clear();
  cost_delta = 0.0;
  if (s.is_mode_switch ||
      std::abs(wrap_pi(goal.theta - s.theta)) > cfg_.goal_yaw_tol) return false;

  const double dx = goal.x - s.x;
  const double dy = goal.y - s.y;
  const double distance = std::hypot(dx, dy);
  if (distance < 1e-6) return true;

  bool reverse = false;
  double command = 0.0;
  if (s.mode == Mode::Lateral) {
    // Project onto the body lateral axis.  The residual prevents this from
    // silently becoming an omnidirectional terminal connector.
    const double lateral = -std::sin(s.theta) * dx + std::cos(s.theta) * dy;
    const double longitudinal = std::cos(s.theta) * dx + std::sin(s.theta) * dy;
    if (std::abs(longitudinal) > cfg_.goal_xy_tol) return false;
    reverse = lateral < 0.0;
  } else if (s.mode == Mode::Diagonal) {
    const double bearing = std::atan2(dy, dx);
    const double phi_fwd = wrap_pi(bearing - s.theta);
    const double phi_rev = wrap_pi(bearing + M_PI - s.theta);
    const bool fwd_ok = std::abs(phi_fwd) <= vp_.phi_max + 1e-6;
    const bool rev_ok = cfg_.allow_reverse &&
                        std::abs(phi_rev) <= vp_.phi_max + 1e-6;
    if (!fwd_ok && !rev_ok) return false;
    // Select the lower local control cost; collision is validated below.
    const double fwd_cost = fwd_ok ? cfg_.w_phi * std::abs(phi_fwd)
                                      + cfg_.w_phi_change_dia * std::abs(phi_fwd - s.phi)
                                    : std::numeric_limits<double>::infinity();
    const double rev_cost = rev_ok ? (cfg_.w_phi * std::abs(phi_rev)
                                      + cfg_.w_phi_change_dia * std::abs(phi_rev - s.phi)
                                      + cfg_.w_dir_change)
                                    : std::numeric_limits<double>::infinity();
    reverse = rev_cost < fwd_cost;
    command = reverse ? phi_rev : phi_fwd;
  } else {
    return false;
  }

  const double signed_distance = reverse ? -distance : distance;
  if (!segment_collision_free(map, s, s.mode, signed_distance, command)) return false;

  const int n = std::max(1, static_cast<int>(std::ceil(distance / (0.5 * cfg_.step_len))));
  const double step_distance = distance / n;
  PlanState cur = s;
  double accum = s.g;
  for (int i = 1; i <= n; ++i) {
    PlanState next = cur;
    Pose2D p = step({cur.x, cur.y, cur.theta}, s.mode,
                    reverse ? -step_distance : step_distance, command, vp_);
    next.x = p.x; next.y = p.y; next.theta = s.theta;
    next.mode = s.mode;
    next.reverse = reverse;
    next.phi = command;
    next.curv = 0.0;
    const double travel_time = cfg_.w_time * step_distance / cfg_.v_cruise;
    double c = reverse ? travel_time / cfg_.k_rev : travel_time;
    if (i == 1 && reverse != s.reverse) c += cfg_.w_dir_change;
    if (s.mode == Mode::Diagonal) {
      c += cfg_.w_phi * std::abs(command) * step_distance;
      if (i == 1) c += cfg_.w_phi_change_dia * std::abs(command - s.phi);
    }
    accum += c;
    next.g = accum;
    out.push_back(next);
    cur = next;
  }
  out.back().x = goal.x;
  out.back().y = goal.y;
  out.back().theta = goal.theta;
  cost_delta = accum - s.g;
  return true;
}

// ---- Analytic multi-mode terminal shot (Spin → Diagonal, 时间等效) ----
bool HybridAStarPlanner::try_analytic_shot(const GridMap& map,
                                           const PlanState& s,
                                           const PlanState& goal,
                                           std::vector<PlanState>& out,
                                           double& cost_delta) const {
  out.clear();
  cost_delta = 0.0;

  double yaw_err = wrap_pi(goal.theta - s.theta);
  double dx = goal.x - s.x;
  double dy = goal.y - s.y;
  double dist = std::hypot(dx, dy);

  bool need_spin = std::abs(yaw_err) > 1e-4;
  bool need_diag = dist > 1e-4;
  if (need_spin && !cfg_.enable_spin) return false;
  if (need_diag && !cfg_.enable_diagonal) return false;

  // Segment 1: Spin (time-equivalent with f_spin modulation)
  double accum_g = s.g;
  const double spin_step = cfg_.spin_yaw_step;
  int n_spin = need_spin ? static_cast<int>(std::ceil(std::abs(yaw_err) / spin_step)) : 0;
  double spin_sgn = (yaw_err >= 0) ? +1 : -1;
  PlanState cur = s;

  for (int i = 1; i <= n_spin; ++i) {
    double th_new = (i == n_spin) ? goal.theta
                                  : wrap_pi(s.theta + spin_sgn * spin_step * i);
    if (map.obb_collides(s.x, s.y, th_new,
                         vp_.length + 0.02, vp_.width + 0.02)) {
      return false;
    }
    PlanState n = cur;
    n.theta = th_new;
    n.mode = Mode::Spin;
    n.reverse = false;
    n.phi = 0.0;
    n.curv = 0.0;
    double dtheta_step = wrap_pi(th_new - cur.theta);
    double dg = std::hypot(cur.x - goal.x, cur.y - goal.y);
    double f = std::max(cfg_.f_spin_min, std::min(cfg_.f_spin_max, dg / cfg_.d_spin));
    accum_g += cfg_.w_time * (std::abs(dtheta_step) / cfg_.omega_spin) * f;
    if (i == 1 && cur.mode != Mode::Spin) {
      accum_g += c_switch_at(cur.x, cur.y, goal.x, goal.y,
                             cfg_.v_ref, cfg_.d_slow,
                             cfg_.t_switch_servo, cfg_.a_max_decel,
                             cfg_.v_floor_frac)
               + cfg_.w_enter_spin;
    }
    n.g = accum_g;
    out.push_back(n);
    cur = n;
  }

  // Segment 2: Diagonal
  if (need_diag) {
    double motion_dir = std::atan2(dy, dx);
    double phi = wrap_pi(motion_dir - goal.theta);

    const int N = std::max(6, static_cast<int>(std::ceil(dist / (cfg_.step_len * 0.5))));
    if (map.obb_collides(s.x, s.y, goal.theta,
                         vp_.length + 0.02, vp_.width + 0.02)) {
      return false;
    }
    for (int k = 1; k <= N; ++k) {
      double frac = static_cast<double>(k) / N;
      double xk = s.x + dx * frac;
      double yk = s.y + dy * frac;
      if (map.obb_collides(xk, yk, goal.theta,
                           vp_.length + 0.02, vp_.width + 0.02)) {
        return false;
      }
    }

    double step_len = dist / N;
    for (int k = 1; k <= N; ++k) {
      double frac = static_cast<double>(k) / N;
      PlanState n = cur;
      n.x = s.x + dx * frac;
      n.y = s.y + dy * frac;
      n.theta = goal.theta;
      n.mode = Mode::Diagonal;
      n.reverse = false;
      n.phi = phi;
      n.curv = 0.0;

      double c_step = cfg_.w_time * (step_len / cfg_.v_cruise)
                    + cfg_.w_phi * std::abs(phi) * step_len;
      if (k == 1 && cur.mode != Mode::Diagonal) {
        double c_sw_dia = c_switch_at(cur.x, cur.y, goal.x, goal.y,
                                      cfg_.v_ref, cfg_.d_slow,
                                      cfg_.t_switch_servo, cfg_.a_max_decel,
                                      cfg_.v_floor_frac);
        if (cur.mode == Mode::Spin) c_sw_dia *= cfg_.k_exit_spin;
        c_step += c_sw_dia + cfg_.w_enter_diagonal;
      }
      accum_g += c_step;
      n.g = accum_g;
      out.push_back(n);
      cur = n;
    }
  }

  cost_delta = accum_g - s.g;
  return true;
}

PlanResult HybridAStarPlanner::plan(const GridMap& map,
                                    const PlanState& start,
                                    const PlanState& goal) const {
  PlanResult res;
  if (!map.valid()) return res;

  const std::vector<float> holonomic_field =
      map.goal_distance_field(goal.x, goal.y);

  std::vector<PlanState> nodes;
  nodes.reserve(8192);
  std::priority_queue<PQItem, std::vector<PQItem>, PQCmp> open;
  std::unordered_map<uint64_t, double> best_g;

  PlanState s0 = start;
  s0.g = 0.0;
  s0.h = heuristic(map, s0, goal, holonomic_field);
  s0.parent = -1;
  s0.dist_since_switch = 1e9;
  nodes.push_back(s0);
  best_g[hash_state(s0)] = 0.0;
  open.push({s0.g + s0.h, 0});

  int iter = 0;
  int goal_idx = -1;
  std::vector<PlanState> analytic_tail;

  const double rs_shot_radius = cfg_.rs_shot_radius;
  const int rs_shot_check_every = cfg_.rs_shot_check_every;
  double best_shot_total = std::numeric_limits<double>::infinity();
  int    best_shot_parent = -1;
  std::vector<PlanState> best_shot_tail;

  while (!open.empty() && iter < cfg_.max_iter) {
    auto top = open.top(); open.pop();
    int idx = top.idx;
    if (top.f - 1e-9 > nodes[idx].g + nodes[idx].h) continue;

    if (reached(nodes[idx], goal)) { goal_idx = idx; break; }

    double d_goal = std::hypot(nodes[idx].x - goal.x,
                               nodes[idx].y - goal.y);

    if (std::isfinite(best_shot_total) &&
        best_shot_total <= top.f + 1e-6) {
      goal_idx = best_shot_parent;
      analytic_tail = std::move(best_shot_tail);
      break;
    }

    if (nodes[idx].mode == Mode::Ackermann && !nodes[idx].is_mode_switch &&
        d_goal <= rs_shot_radius && iter % rs_shot_check_every == 0) {
      // An Ackermann RS shot is valid only while already in Ackermann mode.
      // Other modes must first take the explicit zero-displacement switch.
      {
        std::vector<PlanState> tail;
        double dc = 0.0;
        if (try_rs_shot(map, nodes[idx], goal, tail, dc)) {
          double total = nodes[idx].g + dc;
          if (total < best_shot_total) {
            best_shot_total = total;
            best_shot_parent = idx;
            best_shot_tail = std::move(tail);
          }
        }
      }
    }

    if ((nodes[idx].mode == Mode::Lateral ||
         nodes[idx].mode == Mode::Diagonal) &&
        !nodes[idx].is_mode_switch && d_goal <= rs_shot_radius &&
        iter % rs_shot_check_every == 0) {
      std::vector<PlanState> tail;
      double dc = 0.0;
      if (try_translation_shot(map, nodes[idx], goal, tail, dc)) {
        double total = nodes[idx].g + dc;
        if (total < best_shot_total) {
          best_shot_total = total;
          best_shot_parent = idx;
          best_shot_tail = std::move(tail);
        }
      }
    }

    auto succs = expand(nodes[idx], goal);
    for (auto& n : succs) {
      if (n.is_mode_switch) {
        if (map.obb_collides(n.x, n.y, n.theta,
                             vp_.length + 0.04, vp_.width + 0.04)) continue;
      } else {
        double ds_signed = (n.reverse ? -1.0 : 1.0) * cfg_.step_len;
        if (!segment_collision_free(map, nodes[idx], n.mode, ds_signed, n.phi)) continue;
      }

      uint64_t k = hash_state(n);
      auto it = best_g.find(k);
      if (it != best_g.end() && it->second <= n.g + 1e-9) continue;
      best_g[k] = n.g;

      n.h = heuristic(map, n, goal, holonomic_field);
      n.parent = idx;
      int new_idx = static_cast<int>(nodes.size());
      nodes.push_back(n);
      open.push({n.g + n.h, new_idx});
    }
    ++iter;
  }
  res.iterations = iter;

  if (goal_idx < 0 && std::isfinite(best_shot_total)) {
    goal_idx = best_shot_parent;
    analytic_tail = std::move(best_shot_tail);
  }
  if (goal_idx < 0) return res;

  std::vector<PlanState> rev;
  for (int i = goal_idx; i >= 0; i = nodes[i].parent) rev.push_back(nodes[i]);
  std::reverse(rev.begin(), rev.end());

  for (auto& p : analytic_tail) rev.push_back(p);

  res.success = true;
  res.path    = std::move(rev);
  res.cost    = res.path.empty() ? 0.0 : res.path.back().g;
  return res;
}

bool MacroTopologyPlanner::center_segment_free(const GridMap& map,
                                               double x0, double y0,
                                               double x1, double y1) const {
  const double length = std::hypot(x1 - x0, y1 - y0);
  const int n = std::max(1, static_cast<int>(std::ceil(
      length / std::max(map.resolution(), 0.02))));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    const double x = x0 + (x1 - x0) * t;
    const double y = y0 + (y1 - y0) * t;
    if (map.is_occupied(x, y)) return false;
  }
  return true;
}

bool MacroTopologyPlanner::body_segment_free(const GridMap& map,
                                             double x0, double y0,
                                             double x1, double y1) const {
  const double length = std::hypot(x1 - x0, y1 - y0);
  if (length < 1e-6) return !map.obb_collides(
      x0, y0, 0.0, vp_.length + 0.04, vp_.width + 0.04);
  const double theta = std::atan2(y1 - y0, x1 - x0);
  const int n = std::max(2, static_cast<int>(std::ceil(length / 0.05)));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    const double x = x0 + (x1 - x0) * t;
    const double y = y0 + (y1 - y0) * t;
    if (map.obb_collides_dense(x, y, theta,
                               vp_.length + 0.04, vp_.width + 0.04)) return false;
  }
  return true;
}

PlanResult MacroTopologyPlanner::plan(const GridMap& map,
                                      const PlanState& start,
                                      const PlanState& goal) const {
  PlanResult res;
  if (!map.valid()) return res;

  const int stride = std::max(1, static_cast<int>(std::lround(
      cfg_.topology_sample_step / map.resolution())));
  const int cw = (map.width() + stride - 1) / stride;
  const int ch = (map.height() + stride - 1) / stride;
  auto coarse_of = [&](double x, double y, int& cx, int& cy) {
    int ix, iy;
    if (!map.world_to_idx(x, y, ix, iy)) return false;
    cx = std::min(cw - 1, ix / stride);
    cy = std::min(ch - 1, iy / stride);
    return true;
  };
  auto world_of = [&](int cx, int cy) {
    const int ix = std::min(map.width() - 1, cx * stride + stride / 2);
    const int iy = std::min(map.height() - 1, cy * stride + stride / 2);
    return std::pair<double, double>{map.origin_x() + (ix + 0.5) * map.resolution(),
                                     map.origin_y() + (iy + 0.5) * map.resolution()};
  };

  int sx, sy, gx, gy;
  if (!coarse_of(start.x, start.y, sx, sy) ||
      !coarse_of(goal.x, goal.y, gx, gy)) return res;
  const int ncell = cw * ch;
  const int sidx = sy * cw + sx;
  const int gidx = gy * cw + gx;
  struct Item { double f; int idx; };
  struct Cmp { bool operator()(const Item& a, const Item& b) const {
    return a.f > b.f;
  }};
  std::priority_queue<Item, std::vector<Item>, Cmp> open;
  std::vector<double> gcost(ncell, std::numeric_limits<double>::infinity());
  std::vector<int> parent(ncell, -1);
  gcost[sidx] = 0.0;
  open.push({0.0, sidx});
  constexpr int kdx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  constexpr int kdy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  int expanded = 0;

  while (!open.empty()) {
    const Item cur = open.top(); open.pop();
    const int ci = cur.idx;
    const int cx = ci % cw, cy = ci / cw;
    auto [wx, wy] = world_of(cx, cy);
    const double h = std::hypot(wx - goal.x, wy - goal.y);
    if (cur.f > gcost[ci] + h + 1e-9) continue;
    if (ci == gidx) break;
    ++expanded;
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + kdx[k], ny = cy + kdy[k];
      if (nx < 0 || ny < 0 || nx >= cw || ny >= ch) continue;
      const int ni = ny * cw + nx;
      auto [nwx, nwy] = world_of(nx, ny);
      if (ni != gidx && ni != sidx &&
          map.distance(nwx, nwy) < cfg_.topology_clearance) continue;
      if (!center_segment_free(map, wx, wy, nwx, nwy)) continue;
      const double ds = std::hypot(nwx - wx, nwy - wy);
      const double clearance = std::max(map.distance(nwx, nwy), 0.05);
      const double ng = gcost[ci] + ds *
          (1.0 + cfg_.topology_clearance_weight / clearance);
      if (ng + 1e-9 >= gcost[ni]) continue;
      gcost[ni] = ng;
      parent[ni] = ci;
      open.push({ng + std::hypot(nwx - goal.x, nwy - goal.y), ni});
    }
  }
  res.iterations = expanded;
  if (!std::isfinite(gcost[gidx])) return res;

  std::vector<std::pair<double, double>> raw;
  for (int at = gidx; at >= 0; at = parent[at]) raw.push_back(world_of(at % cw, at / cw));
  std::reverse(raw.begin(), raw.end());
  raw.front() = {start.x, start.y};
  raw.back() = {goal.x, goal.y};

  // Greedy line-of-sight reduction converts the grid trace into a compact
  // topology polyline.  Unlike spline smoothing it never changes geometry:
  // every accepted macro edge is swept with the full vehicle footprint.
  std::vector<std::pair<double, double>> sparse;
  for (size_t i = 0; i + 1 < raw.size();) {
    size_t chosen = i + 1;
    for (size_t j = raw.size() - 1; j > i + 1; --j) {
      if (body_segment_free(map, raw[i].first, raw[i].second,
                            raw[j].first, raw[j].second)) {
        chosen = j;
        break;
      }
    }
    sparse.push_back(raw[i]);
    i = chosen;
  }
  sparse.push_back(raw.back());

  // Preserve the pose semantics used by the swept-body test above.  A macro
  // edge is travelled at its own tangent heading; heading changes happen as
  // explicit in-place rotations at a vertex.  Assigning the *next* edge's
  // heading to a vertex (the old implementation) created an unvalidated
  // diagonal interpolation that could cut through pillars.
  res.path.reserve(sparse.size() * 8);
  double length = 0.0;
  auto append = [&](double x, double y, double theta, Mode mode) {
    PlanState p{};
    p.x = x; p.y = y; p.theta = wrap_pi(theta); p.mode = mode; p.g = length;
    res.path.push_back(p);
  };
  auto append_spin = [&](double x, double y, double from, double to) {
    const double delta = wrap_pi(to - from);
    const int n = std::max(1, static_cast<int>(std::ceil(
        std::abs(delta) / std::max(cfg_.spin_yaw_step, 0.03))));
    for (int k = 1; k <= n; ++k) {
      const double theta = wrap_pi(from + delta * static_cast<double>(k) / n);
      if (map.obb_collides_dense(x, y, theta,
                                 vp_.length + 0.04, vp_.width + 0.04)) return false;
      append(x, y, theta, Mode::Spin);
    }
    return true;
  };

  append(start.x, start.y, start.theta, Mode::Ackermann);
  double current_theta = start.theta;
  for (size_t i = 0; i + 1 < sparse.size(); ++i) {
    const double edge_theta = std::atan2(sparse[i + 1].second - sparse[i].second,
                                         sparse[i + 1].first - sparse[i].first);
    if (!append_spin(sparse[i].first, sparse[i].second, current_theta, edge_theta)) {
      res.path.clear();
      return res;
    }
    length += std::hypot(sparse[i + 1].first - sparse[i].first,
                         sparse[i + 1].second - sparse[i].second);
    append(sparse[i + 1].first, sparse[i + 1].second, edge_theta, Mode::Ackermann);
    current_theta = edge_theta;
  }
  if (!append_spin(goal.x, goal.y, current_theta, goal.theta)) {
    res.path.clear();
    return res;
  }
  res.cost = length;
  res.success = res.path.size() >= 2;
  return res;
}

namespace {

inline std::pair<double,double>
catmull_rom(double t,
            double x0, double y0,
            double x1, double y1,
            double x2, double y2,
            double x3, double y3) {
  auto t_at = [](double tprev, double xa, double ya, double xb, double yb) {
    double d = std::pow(std::hypot(xb - xa, yb - ya), 0.5);
    return tprev + std::max(d, 1e-6);
  };
  double t0 = 0.0;
  double t1 = t_at(t0, x0, y0, x1, y1);
  double t2 = t_at(t1, x1, y1, x2, y2);
  double t3 = t_at(t2, x2, y2, x3, y3);
  double tt = t1 + t * (t2 - t1);

  auto lerp = [](double t, double tA, double tB,
                 double xA, double yA, double xB, double yB) {
    double w = (t - tA) / std::max(tB - tA, 1e-9);
    return std::make_pair(xA + (xB - xA) * w, yA + (yB - yA) * w);
  };
  auto a1 = lerp(tt, t0, t1, x0, y0, x1, y1);
  auto a2 = lerp(tt, t1, t2, x1, y1, x2, y2);
  auto a3 = lerp(tt, t2, t3, x2, y2, x3, y3);
  auto b1 = lerp(tt, t0, t2, a1.first, a1.second, a2.first, a2.second);
  auto b2 = lerp(tt, t1, t3, a2.first, a2.second, a3.first, a3.second);
  auto c  = lerp(tt, t1, t2, b1.first, b1.second, b2.first, b2.second);
  return c;
}

}  // namespace

std::vector<PlanState> densify_path_per_mode(const std::vector<PlanState>& path,
                                             double target_step) {
  std::vector<PlanState> out;
  if (path.size() < 2) return path;
  if (target_step <= 0.0) target_step = 0.05;

  // Catmull-Rom interpolation changes the search geometry and can overshoot
  // into an obstacle at mode boundaries.  The front end must publish the
  // collision-checked topology it actually found; continuous geometric
  // improvement belongs to the safe-corridor NLP back end.
  out.push_back(path.front());
  for (size_t i = 1; i < path.size(); ++i) {
    const PlanState& a = path[i - 1];
    const PlanState& b = path[i];
    const double ds = std::hypot(b.x - a.x, b.y - a.y);
    const double dtheta = wrap_pi(b.theta - a.theta);
    const int n_div = std::max(
        1, static_cast<int>(std::ceil(std::max(ds / target_step,
                                                std::abs(dtheta) / 0.05))));
    for (int k = 1; k <= n_div; ++k) {
      const double t = static_cast<double>(k) / n_div;
      PlanState p = (k == n_div) ? b : a;
      p.x = a.x + t * (b.x - a.x);
      p.y = a.y + t * (b.y - a.y);
      p.theta = wrap_pi(a.theta + t * dtheta);
      // A zero-displacement mode-switch edge is represented by its endpoint
      // only, preserving the stop state for the velocity optimiser.
      out.push_back(p);
    }
  }
  return out;
}

}  // namespace fwis
