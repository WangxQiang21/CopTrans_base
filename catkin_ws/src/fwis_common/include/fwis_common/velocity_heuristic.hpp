// Velocity heuristic for the Hybrid A* frontend.
//
// The frontend does NOT carry velocity in its state (would explode the search
// space). Instead we estimate a "plausible" local speed v_local(s) purely
// from geometric context and use it to price mode-switch cost via
//
//   C_switch(v_local) = v_local * t_switch_servo  +  v_local^2 / a_max_decel
//
// (K01 §4.2; Wang 2025 Eq. 11 with v_ref replaced by v_local).
//
// The heuristic must be:
//   * Non-negative and bounded by v_ref
//   * Smooth (so cost changes continuously along the search)
//   * Cheap to compute (O(1) per node)
//
// Current rule: linear decay in the range [0, d_slow] to goal, saturating at
// v_ref beyond d_slow.
//
// v_local  =  v_ref * min(1, dist_to_goal / d_slow)
//
// This gives:
//   dist=0    -> v=0     (at goal, no cost for switching)
//   dist=d/2  -> v=0.5*v_ref
//   dist=d    -> v_ref   (cruise)
//
// A future extension adds a dist-to-obstacle factor:
//   v_local *= min(1, dist_to_nearest_obs / d_slow_obs)
// but the frontend does not have cheap access to nearest-obstacle distance
// (SDF), so we defer this to the backend QP where kappa(s) and clearance are
// already computed.

#pragma once

#include <algorithm>
#include <cmath>

namespace fwis {

// Distance-based v_local. All quantities in SI (m, m/s).
//
// Design note: near goal we deliberately DO NOT let v_local → 0. If we did,
// C_switch → 0 and the frontend would happily flip modes every few cells
// (they cost almost nothing). A floor of v_ref/2 keeps switches meaningful
// even inside a parking slot.
inline double v_local_estimate(double x, double y,
                               double goal_x, double goal_y,
                               double v_ref, double d_slow,
                               double v_floor_frac = 0.5) {
  double d = std::hypot(goal_x - x, goal_y - y);
  double alpha = std::min(1.0, d / std::max(d_slow, 1e-6));
  double alpha_floored = std::max(alpha, v_floor_frac);
  return v_ref * alpha_floored;
}

// Velocity-coupled mode-switch cost (in meters equivalent, so it composes
// with w_length * path_length directly in the g-value).
//
//   C_switch(v) = v * t_switch_servo  +  v^2 / a_max_decel
//
// Interpretation:
//   v * t_switch_servo   -- fixed servo re-orient time at local speed
//   v^2 / a_max_decel    -- symmetric decel-then-accel distance
//                           = 2 * (v^2 / (2*a)) = v^2 / a
//
// Bounds:
//   v=0        -> C_switch = 0      (free switch at rest)
//   v=v_ref=1  -> C_switch = t_switch_servo + 1/a_max
//                          = 0.5 + 0.5 = 1.0 m at Wang defaults
inline double c_switch_v_local(double v_local,
                               double t_switch_servo,
                               double a_max_decel) {
  double t_term = v_local * t_switch_servo;
  double a_term = (v_local * v_local) / std::max(a_max_decel, 1e-6);
  return t_term + a_term;
}

// Convenience: combine the two into a single call that Ackermann/L/P/S
// expand branches use.
inline double c_switch_at(double x, double y,
                          double goal_x, double goal_y,
                          double v_ref, double d_slow,
                          double t_switch_servo, double a_max_decel,
                          double v_floor_frac = 0.5) {
  double v = v_local_estimate(x, y, goal_x, goal_y, v_ref, d_slow, v_floor_frac);
  return c_switch_v_local(v, t_switch_servo, a_max_decel);
}

}  // namespace fwis
