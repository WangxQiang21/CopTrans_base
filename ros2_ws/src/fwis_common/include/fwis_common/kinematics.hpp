#pragma once
#include <cmath>
#include "fwis_common/modes.hpp"
#include "fwis_common/vehicle.hpp"

namespace fwis {

struct Pose2D { double x, y, theta; };

// Step kinematics for a single vehicle, given mode and step length ds (signed).
// Ackermann: steer angle delta in [-delta_max, delta_max], wheelbase L_b
//   dx = ds*cos(theta), dy = ds*sin(theta), dtheta = ds*tan(delta)/L_b
// Lateral: pure sideway motion, theta unchanged
//   dx = -ds*sin(theta), dy = ds*cos(theta)
// Diagonal: heading + offset angle phi, theta unchanged
//   dx = ds*cos(theta+phi), dy = ds*sin(theta+phi)
inline Pose2D step(const Pose2D& s,
                   Mode m,
                   double ds,
                   double delta_or_phi,
                   const VehicleParams& vp) {
  Pose2D ns = s;
  switch (m) {
    case Mode::Ackermann: {
      double c = std::cos(s.theta);
      double si = std::sin(s.theta);
      ns.x = s.x + ds * c;
      ns.y = s.y + ds * si;
      ns.theta = s.theta + ds * std::tan(delta_or_phi) / vp.wheelbase;
      break;
    }
    case Mode::Lateral: {
      double c = std::cos(s.theta);
      double si = std::sin(s.theta);
      ns.x = s.x - ds * si;
      ns.y = s.y + ds * c;
      ns.theta = s.theta;  // unchanged
      break;
    }
    case Mode::Diagonal: {
      double a = s.theta + delta_or_phi;
      ns.x = s.x + ds * std::cos(a);
      ns.y = s.y + ds * std::sin(a);
      ns.theta = s.theta;  // unchanged
      break;
    }
    case Mode::Spin: {
      // Zero-radius pivot. ds is interpreted as a yaw step (rad), not meters.
      // Position fixed; theta rotates by ds (signed = direction of pivot).
      ns.x = s.x;
      ns.y = s.y;
      ns.theta = s.theta + ds;
      break;
    }
  }
  return ns;
}

// Wrap angle to (-pi, pi]
inline double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

}  // namespace fwis
