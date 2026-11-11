#include "fwis_v3_planner/formation_search.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fwis {
namespace formation {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct SearchKey {
  int64_t x = 0;
  int64_t y = 0;
  int64_t psi = 0;

  bool operator==(const SearchKey& other) const {
    return x == other.x && y == other.y && psi == other.psi;
  }
};

struct SearchKeyHash {
  size_t operator()(const SearchKey& key) const {
    size_t h = std::hash<int64_t>{}(key.x);
    h ^= std::hash<int64_t>{}(key.y) + 0x9e3779b97f4a7c15ULL + (h << 6) +
         (h >> 2);
    h ^= std::hash<int64_t>{}(key.psi) + 0x9e3779b97f4a7c15ULL + (h << 6) +
         (h >> 2);
    return h;
  }
};

struct SearchNode {
  FormationPose qf;
  int64_t psi_bin = 0;
  int mode_hint = -1;
  int gear_hint = 0;
  int parent = -1;
  double g = 0.0;
  FormationPrimitive action;
  bool has_action = false;
};

struct OpenItem {
  double f = 0.0;
  int node_index = -1;

  bool operator<(const OpenItem& other) const {
    if (f != other.f) return f > other.f;
    return node_index > other.node_index;
  }
};

bool finite_config(const FormationSearchConfig& config) {
  const double positive[] = {
      config.xy_res, config.yaw_res, config.translation_step_m,
      config.rotation_step_rad, config.max_corner_step_m,
      config.max_wall_time_s};
  for (double value : positive) {
    if (!std::isfinite(value) || value <= 0.0) return false;
  }
  const double nonnegative[] = {
      config.goal_xy_tol_m, config.goal_yaw_tol_rad, config.w_translation,
      config.w_rotation, config.w_clearance, config.w_switch,
      config.risk_horizon_m};
  for (double value : nonnegative) {
    if (!std::isfinite(value) || value < 0.0) return false;
  }
  return config.max_expansions > 0 && config.max_candidates > 0;
}

SearchKey key_of(const FormationPose& qf, const FormationSearchConfig& config) {
  return {static_cast<int64_t>(std::llround(qf.x / config.xy_res)),
          static_cast<int64_t>(std::llround(qf.y / config.xy_res)),
          static_cast<int64_t>(std::llround(qf.psi / config.yaw_res))};
}

double heuristic(const FormationPose& qf, const FormationPose& goal) {
  // Geometric guidance only.  This is not a proven admissible time bound for
  // formation rotation or per-vehicle mode changes.
  return std::hypot(goal.x - qf.x, goal.y - qf.y);
}

bool same_primitive(const FormationPrimitive& a,
                    const FormationPrimitive& b) {
  constexpr double eps = 1e-9;
  return a.geometry_type == b.geometry_type &&
         std::abs(a.dx - b.dx) <= eps && std::abs(a.dy - b.dy) <= eps &&
         std::abs(a.dpsi - b.dpsi) <= eps &&
         std::abs(a.pivot_x - b.pivot_x) <= eps &&
         std::abs(a.pivot_y - b.pivot_y) <= eps;
}

void append_unique(std::vector<FormationPrimitive>& primitives,
                   const FormationPrimitive& primitive) {
  for (const auto& existing : primitives) {
    if (same_primitive(existing, primitive)) return;
  }
  primitives.push_back(primitive);
}

std::vector<FormationPrimitive> generate_primitives(
    const FormationPose& qf, const FormationPose& goal,
    const FormationSearchConfig& config) {
  std::vector<FormationPrimitive> primitives;
  primitives.reserve(12);

  // Eight formation-frame translation directions.  The formation frame keeps
  // these directions meaningful when psi_F changes.
  const double step = config.translation_step_m;
  for (int k = 0; k < 8; ++k) {
    const double angle = k * kPi / 4.0;
    FormationPrimitive primitive;
    primitive.geometry_type = FormationPrimitive::TRANSLATION;
    primitive.dx = step * std::cos(angle);
    primitive.dy = step * std::sin(angle);
    append_unique(primitives, primitive);
  }

  // Direct translation toward the geometric goal, clipped to one step.  It
  // removes grid-direction residue only; it is not an endpoint certification.
  const double world_dx = goal.x - qf.x;
  const double world_dy = goal.y - qf.y;
  const double distance = std::hypot(world_dx, world_dy);
  if (distance > 1e-9) {
    const double c = std::cos(qf.psi);
    const double s = std::sin(qf.psi);
    const double local_dx = c * world_dx + s * world_dy;
    const double local_dy = -s * world_dx + c * world_dy;
    const double amount = std::min(step, distance);
    FormationPrimitive primitive;
    primitive.geometry_type = FormationPrimitive::TRANSLATION;
    primitive.dx = local_dx / distance * amount;
    primitive.dy = local_dy / distance * amount;
    append_unique(primitives, primitive);
  }

  // Shortest-direction rotation toward the goal psi plus both fixed-direction
  // rotation probes.  The pivot is explicit: first version rotates about the
  // current formation reference origin.
  const double goal_delta = wrap_angle(goal.psi - qf.psi);
  if (std::abs(goal_delta) > 1e-9) {
    FormationPrimitive primitive;
    primitive.geometry_type = FormationPrimitive::ROTATION;
    primitive.dpsi = std::copysign(
        std::min(config.rotation_step_rad, std::abs(goal_delta)), goal_delta);
    append_unique(primitives, primitive);
  }
  for (double direction : {1.0, -1.0}) {
    FormationPrimitive primitive;
    primitive.geometry_type = FormationPrimitive::ROTATION;
    primitive.dpsi = direction * config.rotation_step_rad;
    append_unique(primitives, primitive);
  }
  return primitives;
}

double clearance_risk(const FormationCollisionChecker& checker,
                      const FormationMap& map, const FormationPose& qf,
                      const FormationSearchConfig& config) {
  const std::vector<VehicleBody> bodies = checker.rigid_bodies(qf);
  double risk = 0.0;
  for (const auto& body : bodies) {
    const double distance = map.distance(body.x, body.y);
    if (!std::isfinite(distance)) {
      risk += config.risk_horizon_m;
    } else if (distance < config.risk_horizon_m) {
      risk += config.risk_horizon_m - distance;
    }
  }
  return risk;
}

std::vector<FormationPrimitive> reconstruct(
    const std::vector<SearchNode>& nodes, int index) {
  std::vector<FormationPrimitive> reversed;
  while (index >= 0 && nodes[static_cast<size_t>(index)].has_action) {
    reversed.push_back(nodes[static_cast<size_t>(index)].action);
    index = nodes[static_cast<size_t>(index)].parent;
  }
  std::reverse(reversed.begin(), reversed.end());
  return reversed;
}

bool full_sweep_clear(const FormationRequest& request,
                      const FormationCollisionChecker& checker,
                      const std::vector<FormationPrimitive>& primitives,
                      const FormationSearchConfig& config) {
  FormationPose qf = request.start_qf;
  for (const auto& primitive : primitives) {
    if (!checker.sweep_clear(qf, primitive, config.max_corner_step_m)) {
      return false;
    }
    FormationPose next;
    if (!integrate_primitive(qf, primitive, &next)) return false;
    qf = next;
  }
  return true;
}

// Finish both components of SE(2), rather than leaving a yaw residual for a
// translation-only terminal fit in the Python backend.
bool connect_exact_goal(const FormationRequest& request,
                        const FormationCollisionChecker& checker,
                        const FormationSearchConfig& config,
                        const FormationPose& from,
                        std::vector<FormationPrimitive>* candidate) {
  for (bool rotate_first : {false, true}) {
    FormationPose pose = from;
    auto completed = *candidate;
    bool clear = true;
    for (int i = 0; i < 2; ++i) {
      const bool rotate = (i == 0) == rotate_first;
      FormationPrimitive primitive;
      if (rotate) {
        primitive.geometry_type = FormationPrimitive::ROTATION;
        primitive.dpsi = wrap_angle(request.goal_qf.psi - pose.psi);
        if (std::abs(primitive.dpsi) < 1e-10) continue;
      } else {
        const double dx = request.goal_qf.x - pose.x;
        const double dy = request.goal_qf.y - pose.y;
        if (std::hypot(dx, dy) < 1e-10) continue;
        primitive.geometry_type = FormationPrimitive::TRANSLATION;
        primitive.dx = std::cos(pose.psi) * dx + std::sin(pose.psi) * dy;
        primitive.dy = -std::sin(pose.psi) * dx + std::cos(pose.psi) * dy;
      }
      if (!checker.sweep_clear(pose, primitive, config.max_corner_step_m)) {
        clear = false;
        break;
      }
      FormationPose next;
      if (!integrate_primitive(pose, primitive, &next)) { clear = false; break; }
      completed.push_back(primitive);
      pose = next;
    }
    if (clear && std::hypot(pose.x - request.goal_qf.x,
                            pose.y - request.goal_qf.y) < 1e-7 &&
        std::abs(wrap_angle(pose.psi - request.goal_qf.psi)) < 1e-7 &&
        full_sweep_clear(request, checker, completed, config)) {
      *candidate = std::move(completed);
      return true;
    }
  }
  return false;
}

}  // namespace

FormationSearchResult search_formation_geometry(
    const FormationRequest& request, const FormationModel& model,
    const FormationMap& map, const FormationSearchConfig& config) {
  FormationSearchResult result;
  result.start_qf = request.start_qf;
  result.goal_qf = request.goal_qf;
  result.geometry_only = true;

  if (!finite_config(config)) {
    result.reason = "INVALID_SEARCH_CONFIG";
    result.detail = "search configuration contains an invalid value";
    return result;
  }
  if (!map.valid()) {
    result.reason = "MAP_UNAVAILABLE";
    result.detail = "formation occupancy map is invalid";
    return result;
  }
  if (model.empty() ||
      model.vehicles.size() != request.vehicles.size()) {
    result.reason = "INVALID_FORMATION_MODEL";
    result.detail = "formation model is empty or inconsistent with the request";
    return result;
  }

  const FormationCollisionChecker checker(map, model, request);
  if (!checker.start_clear()) {
    result.reason = "START_FORMATION_COLLISION";
    result.detail = "start formation is not clear of the map or itself";
    return result;
  }
  if (!checker.goal_clear()) {
    result.reason = "GOAL_FORMATION_COLLISION";
    result.detail = "goal formation is not clear of the map or itself";
    return result;
  }

  std::vector<SearchNode> nodes;
  nodes.reserve(static_cast<size_t>(config.max_expansions) + 1);
  std::unordered_map<SearchKey, double, SearchKeyHash> best_g;
  std::priority_queue<OpenItem> open;

  const SearchKey start_key = key_of(request.start_qf, config);
  SearchNode root;
  root.qf = request.start_qf;
  root.psi_bin = start_key.psi;
  root.parent = -1;
  root.g = 0.0;
  nodes.push_back(root);
  best_g[start_key] = 0.0;
  open.push({heuristic(root.qf, request.goal_qf), 0});

  const auto started = std::chrono::steady_clock::now();
  int expanded = 0;
  int generated = 0;
  int goal_candidates = 0;
  bool budget_exhausted = false;

  while (!open.empty()) {
    if (expanded >= config.max_expansions) {
      budget_exhausted = true;
      break;
    }
    if ((expanded & 255) == 0) {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        started)
              .count();
      if (elapsed >= config.max_wall_time_s) {
        budget_exhausted = true;
        break;
      }
    }

    const OpenItem item = open.top();
    open.pop();
    if (item.node_index < 0 ||
        static_cast<size_t>(item.node_index) >= nodes.size()) {
      continue;
    }
    const SearchNode node = nodes[static_cast<size_t>(item.node_index)];
    const SearchKey node_key = key_of(node.qf, config);
    const auto best_it = best_g.find(node_key);
    if (best_it == best_g.end() || node.g > best_it->second + 1e-9) {
      continue;
    }
    ++expanded;

    const double xy_error =
        std::hypot(node.qf.x - request.goal_qf.x,
                   node.qf.y - request.goal_qf.y);
    const double yaw_error =
        std::abs(wrap_angle(request.goal_qf.psi - node.qf.psi));
    if (xy_error <= config.goal_xy_tol_m &&
        yaw_error <= config.goal_yaw_tol_rad) {
      ++goal_candidates;
      std::vector<FormationPrimitive> candidate =
          reconstruct(nodes, item.node_index);
      if (connect_exact_goal(request, checker, config, node.qf, &candidate)) {
        result.success = true;
        result.reason = "OK";
        result.detail = "GEOMETRY_ONLY_UNCERTIFIED";
        result.primitives = candidate;
        result.cost = node.g;
        result.expanded_nodes = expanded;
        result.generated_nodes = generated;
        result.goal_candidates = goal_candidates;
        return result;
      }
      if (goal_candidates >= config.max_candidates) {
        // The near-goal geometry was checked the configured number of times;
        // no executable or feasibility claim is made on these failures.
        budget_exhausted = true;
        break;
      }
    }

    const std::vector<FormationPrimitive> primitives =
        generate_primitives(node.qf, request.goal_qf, config);
    for (const auto& primitive : primitives) {
      ++generated;
      FormationPose next;
      if (!integrate_primitive(node.qf, primitive, &next)) continue;
      if (!std::isfinite(next.x) || !std::isfinite(next.y) ||
          !std::isfinite(next.psi)) {
        continue;
      }
      if (!checker.sweep_clear(node.qf, primitive,
                               config.max_corner_step_m)) {
        continue;
      }

      const double translation =
          primitive.geometry_type == FormationPrimitive::TRANSLATION
              ? std::hypot(primitive.dx, primitive.dy)
              : 0.0;
      const double rotation =
          primitive.geometry_type == FormationPrimitive::ROTATION
              ? std::abs(primitive.dpsi)
              : 0.0;
      const double switch_cost =
          (node.mode_hint >= 0 &&
           node.mode_hint != static_cast<int>(primitive.geometry_type))
              ? config.w_switch
              : 0.0;
      const double risk = clearance_risk(checker, map, next, config);
      const double next_g =
          node.g + config.w_translation * translation +
          config.w_rotation * rotation + config.w_clearance * risk +
          switch_cost;

      const SearchKey next_key = key_of(next, config);
      const auto prior = best_g.find(next_key);
      if (prior != best_g.end() && next_g >= prior->second - 1e-9) {
        continue;
      }
      best_g[next_key] = next_g;

      SearchNode child;
      child.qf = next;
      child.psi_bin = next_key.psi;
      child.mode_hint = static_cast<int>(primitive.geometry_type);
      child.gear_hint =
          primitive.geometry_type == FormationPrimitive::ROTATION
              ? (primitive.dpsi >= 0.0 ? 1 : -1)
              : 1;
      child.parent = item.node_index;
      child.g = next_g;
      child.action = primitive;
      child.has_action = true;
      nodes.push_back(child);
      open.push({next_g + heuristic(next, request.goal_qf),
                 static_cast<int>(nodes.size()) - 1});
    }
  }

  result.expanded_nodes = expanded;
  result.generated_nodes = generated;
  result.goal_candidates = goal_candidates;
  result.budget_exhausted = budget_exhausted;
  if (budget_exhausted) {
    result.reason = "FORMATION_BUDGET_EXHAUSTED";
    result.detail =
        "geometry search exhausted its expansion, wall-clock, or candidate budget";
  } else {
    result.reason = "FORMATION_NO_GEOMETRIC_PATH";
    result.detail = "no non-colliding q_F geometry candidate was found";
  }
  return result;
}

}  // namespace formation
}  // namespace fwis
