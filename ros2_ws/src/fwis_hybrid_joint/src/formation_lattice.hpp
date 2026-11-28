#pragma once

#include "fwis_hybrid_joint/model.hpp"

namespace fwis_hybrid_joint {
// Coordinates are the formation reference, including for an Ackermann pivot.
// Consecutive poses either translate at fixed yaw or turn at fixed reference XY.
std::vector<Pose2d> formation_channel(const Request& request,
  const std::vector<std::array<double,4>>& obstacles, int headings, double spacing,
  const std::function<bool()>& stop);
std::vector<Pose2d> formation_channel_on_goal_grid(const Request&, const std::vector<std::array<double,4>>&,
  int headings,double spacing,const std::function<bool()>& stop);
}
