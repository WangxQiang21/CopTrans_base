// S4 formation front end: a geometry-only search in q_F = (x_F, y_F, psi_F).
// Success means "a non-colliding rigid-formation geometry candidate exists".
// It does not mean that any vehicle can execute the candidate with A/O/S
// kinematics, timing, actuators, or a certified swept trajectory.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "fwis_v3_planner/formation_collision.hpp"
#include "fwis_v3_planner/formation_model.hpp"

namespace fwis {
namespace formation {

struct FormationSearchConfig {
  double xy_res = 0.20;
  double yaw_res = 0.17453292519943295;  // 10 deg
  double translation_step_m = 0.50;
  double rotation_step_rad = 0.17453292519943295;  // 10 deg
  double goal_xy_tol_m = 0.15;
  double goal_yaw_tol_rad = 0.15;
  double max_corner_step_m = 0.05;
  int max_expansions = 50000;
  int max_candidates = 4;
  double max_wall_time_s = 8.0;

  double w_translation = 1.0;
  double w_rotation = 0.50;
  double w_clearance = 0.25;
  double w_switch = 0.20;
  double risk_horizon_m = 0.50;
};

struct FormationSearchResult {
  bool success = false;
  std::string reason;  // OK, START_FORMATION_COLLISION, ...
  std::string detail;
  std::vector<FormationPrimitive> primitives;
  FormationPose start_qf;
  FormationPose goal_qf;
  double cost = 0.0;
  int expanded_nodes = 0;
  int generated_nodes = 0;
  int goal_candidates = 0;
  bool geometry_only = true;
  bool budget_exhausted = false;
};

FormationSearchResult search_formation_geometry(
    const FormationRequest& request, const FormationModel& model,
    const FormationMap& map, const FormationSearchConfig& config);

}  // namespace formation
}  // namespace fwis