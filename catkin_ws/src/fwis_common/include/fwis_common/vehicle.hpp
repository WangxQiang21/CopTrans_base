#pragma once
namespace fwis {

// AGILE-X Ranger Mini 2.0 (default values, override via param if needed)
struct VehicleParams {
  double length      = 0.738;  // L_v
  double width       = 0.500;  // W_v
  double wheelbase   = 0.494;  // L_b
  double track       = 0.364;  // T_w
  double v_max       = 1.5;    // m/s (60% of factory)
  double r_min_ack   = 0.81;   // Ackermann minimum turning radius (m)
  double delta_max   = 0.0;    // computed lazily; see kinematics
  double phi_max     = 1.5707963267948966; // 90 deg (lateral) for diagonal
};

// Default footprint margin used for collision checking
struct FootprintParams {
  double inflation_m = 0.05;   // safety margin in meters
};

}  // namespace fwis
